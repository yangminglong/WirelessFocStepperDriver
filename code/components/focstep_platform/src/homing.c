#include "homing.h"
#include "foc_motor.h"
#include "ipropi_sense.h"

#include <math.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "HOME";

/* 轮询周期。回零是慢过程, 100ms 足够 —— 注意这里的"位置不变"判据是按
 * 100ms 窗口算的, 改这个值要同步复核 CONFIG_FOCSTEP_HOME_CONFIRM_MS。 */
#define POLL_MS 100
/* 位置"不再前进"的判据: 一个轮询窗口内位移小于此值 (rad)。
 * 取 0.002 rad ≈ 0.03 圈, 远小于正常回零速度下的位移。 */
#define STILL_EPS_RAD 0.002f

static bool s_has_datum = false;
static float s_datum_closed = 0.0f;
static float s_datum_open = 0.0f;

const char *homing_result_str(homing_result_t r)
{
    switch (r) {
    case HOME_OK:
        return "成功";
    case HOME_ERR_NOT_READY:
        return "未就绪";
    case HOME_ERR_NO_CONTACT:
        return "未碰到限位(行程超限)";
    case HOME_ERR_UNREPEATABLE:
        return "两次结果不一致(基准不可信)";
    default:
        return "?";
    }
}

/* 单次逼近: 朝 end 推, 直到判定接触。
 *
 * ★ 两条判据, 任一满足且持续确认时间即判接触:
 *   ① **电流相对基线的抬升** `peak > baseline × RISE_PCT`
 *      —— 这是"StallGuard 等价物": 接触瞬间负载刚开始增大就能响应,
 *         不必等到位置完全停住。回零的早期检测靠它。
 *   ② **位置停滞** 位移 < EPS
 *      —— 保底。电流信号不可用 (钳位/标定错/未使能) 时仍能工作。
 *
 * ⚠️ 判据①必须用**相对**基线, 不能用绝对阈值。绝对阈值会踩这个坑:
 *    回零力矩下自由运行的电流本来就接近 ITRIP, 绝对阈值在接触前就已满足,
 *    判据形同虚设, 回零退化成"只看位置停没停"。 (初版就是这么写的)
 */
static homing_result_t approach(home_end_t end, float *out_angle)
{
    const float dir = (end == HOME_END_CLOSED) ? -1.0f : +1.0f;
    const float max_travel = (float)CONFIG_FOCSTEP_HOME_MAX_TRAVEL_MRAD / 1000.0f;
    const float rise_ratio = (float)CONFIG_FOCSTEP_HOME_RISE_PCT / 100.0f;
    const float cur_floor = (float)CONFIG_FOCSTEP_HOME_MIN_CURRENT_MA;

    float start = foc_motor_get_angle_rad();

    /* 目标故意设到行程之外 —— 机构会顶在机械限位上, 电机堵转。 */
    foc_motor_set_target_rad(start + dir * max_travel * 1.2f);
    foc_motor_set_mode(FOC_MODE_POSITION);

    TickType_t t0 = xTaskGetTickCount();
    TickType_t still_since = 0;
    TickType_t rise_since = 0;
    TickType_t baseline_until = t0 + pdMS_TO_TICKS(CONFIG_FOCSTEP_HOME_BASELINE_MS);
    float baseline = 0.0f;    /* 自由运行时的电流矢量幅值峰值 */
    bool baseline_done = false;
    bool rise_enabled = true; /* 基线不可用 (太小) 时关掉判据① */
    float last = start;

    ESP_LOGI(TAG, "逼近中: 前 %d ms 采自由运行电流基线...", CONFIG_FOCSTEP_HOME_BASELINE_MS);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));

        float now = foc_motor_get_angle_rad();
        float moved = fabsf(now - last);
        last = now;
        float peak = ipropi_envelope_take();

        /* ---- 阶段 1: 采基线 (取窗口内最大值, 偏保守, 少误触发) ---- */
        if (!baseline_done) {
            if (peak > baseline) {
                baseline = peak;
            }
            if (xTaskGetTickCount() >= baseline_until) {
                baseline_done = true;
                rise_enabled = (baseline >= cur_floor);
                ESP_LOGI(TAG, "基线 %.0f mA %s (判据①阈值 %.0f mA)",
                         baseline, rise_enabled ? "可用" : "**太小, 不可用 ⇒ 只靠位置判据**",
                         baseline * rise_ratio);
            }
            continue;
        }

        /* ---- 阶段 2: 两条判据并行 ---- */
        bool by_current = rise_enabled && (peak > baseline * rise_ratio);
        bool by_still = (moved < STILL_EPS_RAD);

        if (by_current) {
            if (rise_since == 0) {
                rise_since = xTaskGetTickCount();
            }
        } else {
            rise_since = 0;
        }
        if (by_still) {
            if (still_since == 0) {
                still_since = xTaskGetTickCount();
            }
        } else {
            still_since = 0;
        }

        TickType_t confirm = pdMS_TO_TICKS(CONFIG_FOCSTEP_HOME_CONFIRM_MS);
        bool hit_current = (rise_since && (xTaskGetTickCount() - rise_since) >= confirm);
        bool hit_still = (still_since && (xTaskGetTickCount() - still_since) >= confirm);

        if (hit_current || hit_still) {
            *out_angle = now;
            ESP_LOGI(TAG, "接触[%s]: 角=%.3f rad (自 %.3f 移动 %.3f), 电流 %.0f mA (基线 %.0f)",
                     hit_current ? "电流抬升" : "位置停滞",
                     (double)now, (double)start, (double)fabsf(now - start), peak, baseline);
            return HOME_OK;
        }

        /* 行程保护: 走了超过 max_travel 还没接触 ⇒ 机构无限位或已损坏 */
        if (fabsf(now - start) > max_travel) {
            ESP_LOGE(TAG, "走了 %.3f rad 仍未接触 ⇒ 判定无接触", (double)max_travel);
            return HOME_ERR_NO_CONTACT;
        }
        /* 总超时保护 (电机堵转发热) */
        if ((xTaskGetTickCount() - t0) > pdMS_TO_TICKS(CONFIG_FOCSTEP_HOME_TIMEOUT_MS)) {
            ESP_LOGE(TAG, "回零超时");
            return HOME_ERR_NO_CONTACT;
        }
    }
}

/* 单次带退避的完整逼近 */
static homing_result_t approach_with_backoff(home_end_t end, float *out_angle)
{
    homing_result_t r = approach(end, out_angle);
    if (r != HOME_OK) {
        return r;
    }
    /* 退避: 松开机械限位, 避免长期顶住 */
    const float backoff = (float)CONFIG_FOCSTEP_HOME_BACKOFF_MRAD / 1000.0f;
    float dir = (end == HOME_END_CLOSED) ? +1.0f : -1.0f;
    foc_motor_set_target_rad(*out_angle + dir * backoff);
    /* 等退避走完 —— 用固定延时比再判一次更简单可靠 */
    TickType_t t0 = xTaskGetTickCount();
    float want = fabsf(backoff);
    while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(CONFIG_FOCSTEP_HOME_TIMEOUT_MS)) {
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
        if (fabsf(foc_motor_get_angle_rad() - *out_angle) >= want * 0.9f) {
            break;
        }
    }
    return HOME_OK;
}

homing_result_t homing_run(home_end_t end, float *out_angle)
{
    if (!foc_motor_position_ready()) {
        return HOME_ERR_NOT_READY;
    }

    /* ★ 回零必须用**小力矩** —— 以运行力矩顶机械限位会损坏机构。
     *   用 P 控制 + 限制电压上限的方式, 而不是把速度调慢 (慢不代表力小)。 */
    float saved_limit = foc_motor_get_voltage_limit();
    float home_v = (float)CONFIG_FOCSTEP_HOME_TORQUE_MV / 1000.0f;
    foc_motor_set_voltage_limit(home_v);
    ipropi_stall_reset();
    ipropi_envelope_take();
    ESP_LOGW(TAG, "回零 %s 端: 力矩限 %.2fV (原 %.2fV), 最大行程 %.0f mrad",
             (end == HOME_END_CLOSED) ? "全关" : "全开", (double)home_v,
             (double)saved_limit, (double)CONFIG_FOCSTEP_HOME_MAX_TRAVEL_MRAD);

    float a1 = 0.0f, a2 = 0.0f;

    /* 第一次逼近 */
    homing_result_t r = approach_with_backoff(end, &a1);
    if (r != HOME_OK) {
        goto out;
    }
    /* 退回到行程内一点, 再做第二次 —— 重复性检查是必须的:
     * 偏一步的基准会让机构永远偏, 而这是没法在现场发现的那种错。 */
    vTaskDelay(pdMS_TO_TICKS(500));
    ipropi_stall_reset();
    ipropi_envelope_take();

    r = approach_with_backoff(end, &a2);
    if (r != HOME_OK) {
        goto out;
    }

    {
        float tol = (float)CONFIG_FOCSTEP_HOME_REPEAT_TOL_MRAD / 1000.0f;
        float diff = fabsf(a2 - a1);
        if (diff > tol) {
            ESP_LOGE(TAG, "重复性不合格: 两次 %.3f / %.3f rad, 差 %.4f > 容差 %.4f",
                     (double)a1, (double)a2, (double)diff, (double)tol);
            r = HOME_ERR_UNREPEATABLE;
            goto out;
        }
        ESP_LOGI(TAG, "重复性 OK: %.3f / %.3f rad (差 %.4f ≤ %.4f)",
                 (double)a1, (double)a2, (double)diff, (double)tol);
        *out_angle = a2;
        r = HOME_OK;
    }

out:
    /* 无论成败都要恢复电压上限 —— 否则电机会一直没力气 */
    foc_motor_set_voltage_limit(saved_limit);
    ESP_LOGI(TAG, "力矩上限已恢复 %.2fV", (double)saved_limit);
    return r;
}

homing_result_t homing_learn_range(void)
{
    float a_closed = 0.0f, a_open = 0.0f;

    ESP_LOGW(TAG, "=== 开始自学习限位 (会依次顶两端机械限位) ===");
    homing_result_t r = homing_run(HOME_END_CLOSED, &a_closed);
    if (r != HOME_OK) {
        ESP_LOGE(TAG, "全关端回零失败: %s", homing_result_str(r));
        return r;
    }
    r = homing_run(HOME_END_OPEN, &a_open);
    if (r != HOME_OK) {
        ESP_LOGE(TAG, "全开端回零失败: %s", homing_result_str(r));
        return r;
    }

    if (fabsf(a_open - a_closed) < 0.1f) {
        ESP_LOGE(TAG, "两端的角几乎相同 (%.3f / %.3f) ⇒ 行程没学到, 检查机构",
                 (double)a_closed, (double)a_open);
        return HOME_ERR_NO_CONTACT;
    }

    s_datum_closed = a_closed;
    s_datum_open = a_open;
    s_has_datum = true;

    /* 行程范围按端点排序存 —— move_to 的 0.0/1.0 映射依赖它 */
    float lo = (a_closed < a_open) ? a_closed : a_open;
    float hi = (a_closed < a_open) ? a_open : a_closed;
    foc_motor_set_travel_range(lo, hi);

    /* 标定后位置可信, 落盘 */
    foc_motor_set_target_rad(a_closed);
    foc_motor_save_position();

    ESP_LOGW(TAG, "=== 自学习完成: 全关 %.3f rad, 全开 %.3f rad, 行程 %.3f rad ===",
             (double)a_closed, (double)a_open, (double)(a_open - a_closed));
    return HOME_OK;
}

bool homing_has_datum(void)
{
    return s_has_datum;
}

void homing_print_status(void)
{
    float rmin = 0, rmax = 0;
    foc_motor_get_travel_range(&rmin, &rmax);
    printf("基准     : %s\n", s_has_datum ? "已建立" : "未建立");
    printf("全关 / 全开: %.3f / %.3f rad\n", (double)s_datum_closed, (double)s_datum_open);
    printf("行程限位 : [%.3f, %.3f] rad  (跨度 %.3f)\n",
           (double)rmin, (double)rmax, (double)(rmax - rmin));
    printf("当前位置 : %.3f rad   位置可信: %s\n",
           (double)foc_motor_get_angle_rad(),
           foc_motor_position_trusted() ? "是" : "**否 (需回零)**");
    printf("回零参数 : 力矩 %d mV, 最大行程 %d mrad, 退避 %d mrad,\n"
           "           确认 %d ms, 重复性容差 %d mrad, 总超时 %d ms\n",
           CONFIG_FOCSTEP_HOME_TORQUE_MV, CONFIG_FOCSTEP_HOME_MAX_TRAVEL_MRAD,
           CONFIG_FOCSTEP_HOME_BACKOFF_MRAD, CONFIG_FOCSTEP_HOME_CONFIRM_MS,
           CONFIG_FOCSTEP_HOME_REPEAT_TOL_MRAD, CONFIG_FOCSTEP_HOME_TIMEOUT_MS);
}
