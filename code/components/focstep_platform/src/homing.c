#include "homing.h"
#include "foc_motor.h"
#include "ipropi_sense.h"
#include "platform_events.h"
#include "power_state.h"

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

/* 行程最小跨度用 foc_motor 的口径 (FOC_MOTOR_MIN_SPAN_RAD) —— 两处必须一致,
 * 真正的校验在 foc_motor 里, 这里只做"标定前先自查"避免白跑一趟。 */
/* 手动标定 (mark) 允许的最大速度: 门还在动时标会把端点标错 */
#define MARK_MAX_VEL_RAD 0.5f

/* ── 标定策略 (安全前提 + 失败处理) ───────────────────────────
 * 默认值来自 Kconfig; 运行时可被 homing_set_policy() 改 (比如上位机授权后开自动标定)。 */
#ifdef CONFIG_FOCSTEP_HOME_AUTO_ENABLE
#define HOME_AUTO_DEFAULT true
#else
#define HOME_AUTO_DEFAULT false
#endif
/* 自动标定的零点方向: 默认负端 (常规安装)。反装机构打开 FOCSTEP_HOME_AUTO_INVERTED。
 * ⚠️ 标定前"哪一侧是零点"是**待求量**, 推不出来 ⇒ 只能配。 */
#ifdef CONFIG_FOCSTEP_HOME_AUTO_INVERTED
#define HOME_AUTO_ZERO_DIR HOME_DIR_POS
#else
#define HOME_AUTO_ZERO_DIR HOME_DIR_NEG
#endif

static homing_policy_t s_policy = {
    .retries = CONFIG_FOCSTEP_HOME_RETRY,
    .auto_calib = HOME_AUTO_DEFAULT,
    .zero_dir = HOME_AUTO_ZERO_DIR,
};

const char *homing_dir_str(home_dir_t dir)
{
    return (dir == HOME_DIR_POS) ? "正(角度增大)" : "负(角度减小)";
}

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
    case HOME_ERR_RANGE:
        return "行程非法(两端重合/跨度太窄/顺序不对)";
    case HOME_ERR_NOT_ALLOWED:
        return "自动标定未获允许";
    default:
        return "?";
    }
}

void homing_get_policy(homing_policy_t *out)
{
    if (out) {
        *out = s_policy;
    }
}

void homing_set_policy(const homing_policy_t *in)
{
    if (!in) {
        return;
    }
    ESP_LOGW(TAG, "标定策略更新: 重试 %u → %u 次, 自动标定 %s → %s",
             s_policy.retries, in->retries,
             s_policy.auto_calib ? "允许" : "禁止", in->auto_calib ? "允许" : "禁止");
    s_policy = *in;
}

/* 回零结果 → 事件 (应用决定点灯/重试/进故障态, 平台不做决策) */
static void post_homed(homing_result_t r, float angle)
{
    focstep_evt_home_t e = {
        .base = { .timestamp_ms = platform_now_ms() },
        .result = (int)r,
        .angle = angle,
    };
    platform_event_post(FOCSTEP_EVT_HOMED, &e, sizeof(e));
}

/*
 * 把"某一端点的接触角/标定角"写进行程 (另一端**物理位置保持不变**, 见 foc_motor.h)。
 *
 * ★ 跨度校验只有一处 (foc_motor 的 range_commit) —— 那里拒绝就整笔不写,
 *   旧的标定继续有效。这里只把错误码翻译成回零错误码。
 */
static homing_result_t apply_endpoint(bool as_zero, float angle)
{
    esp_err_t r = as_zero ? foc_motor_set_zero(angle) : foc_motor_set_end(angle);
    if (r == ESP_OK) {
        return HOME_OK;
    }
    if (r == ESP_ERR_INVALID_ARG) {
        ESP_LOGE(TAG, "%s = %.3f rad 被拒: 与另一端跨度不足 %.3f rad "
                      "⇒ 判定标错端了 (或方向不对), 本次不做任何修改",
                 as_zero ? "零点" : "满行程点", (double)angle,
                 (double)FOC_MOTOR_MIN_SPAN_RAD);
        return HOME_ERR_RANGE;
    }
    ESP_LOGE(TAG, "%s 写入失败: %s", as_zero ? "零点" : "满行程点", esp_err_to_name(r));
    return HOME_ERR_NOT_READY;
}

/* 单次逼近: 朝 dir 推, 直到判定接触。
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
static homing_result_t approach(home_dir_t dir, float *out_contact_rad)
{
    const float dirf = (float)dir; /* HOME_DIR_NEG = -1, HOME_DIR_POS = +1 */
    const float max_travel = (float)CONFIG_FOCSTEP_HOME_MAX_TRAVEL_MRAD / 1000.0f;
    const float rise_ratio = (float)CONFIG_FOCSTEP_HOME_RISE_PCT / 100.0f;
    const float cur_floor = (float)CONFIG_FOCSTEP_HOME_MIN_CURRENT_MA;

    float start = foc_motor_get_angle_rad();

    /* 目标故意设到行程之外 —— 机构会顶在机械限位上, 电机堵转。 */
    foc_motor_set_target_rad(start + dirf * max_travel * 1.2f);
    foc_motor_set_mode(FOC_MODE_POSITION);

    TickType_t t0 = xTaskGetTickCount();
    TickType_t still_since = 0;
    TickType_t rise_since = 0;
    TickType_t baseline_until = t0 + pdMS_TO_TICKS(CONFIG_FOCSTEP_HOME_BASELINE_MS);
    float baseline = 0.0f;    /* 自由运行时的电流矢量幅值峰值 */
    bool baseline_done = false;
    bool rise_enabled = true; /* 基线不可用 (太小) 时关掉判据① */
    float last = start;

    ESP_LOGI(TAG, "朝%s逼近中: 前 %d ms 采自由运行电流基线...",
             homing_dir_str(dir), CONFIG_FOCSTEP_HOME_BASELINE_MS);

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
            *out_contact_rad = now;
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
static homing_result_t approach_with_backoff(home_dir_t dir, float *out_contact_rad)
{
    homing_result_t r = approach(dir, out_contact_rad);
    if (r != HOME_OK) {
        return r;
    }
    /* 退避: 松开机械限位, 避免长期顶住 (方向与逼近相反) */
    const float backoff = (float)CONFIG_FOCSTEP_HOME_BACKOFF_MRAD / 1000.0f;
    foc_motor_set_target_rad(*out_contact_rad - (float)dir * backoff);
    /* 等退避走完 —— 用固定延时比再判一次更简单可靠 */
    TickType_t t0 = xTaskGetTickCount();
    float want = fabsf(backoff);
    while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(CONFIG_FOCSTEP_HOME_TIMEOUT_MS)) {
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
        if (fabsf(foc_motor_get_angle_rad() - *out_contact_rad) >= want * 0.9f) {
            break;
        }
    }
    return HOME_OK;
}

homing_result_t homing_probe(home_dir_t dir, float *out_contact_rad)
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
    ESP_LOGW(TAG, "回零(只测) 朝%s: 力矩限 %.2fV (原 %.2fV), 最大行程 %.0f mrad",
             homing_dir_str(dir), (double)home_v,
             (double)saved_limit, (double)CONFIG_FOCSTEP_HOME_MAX_TRAVEL_MRAD);

    float a1 = 0.0f, a2 = 0.0f;

    /* 第一次逼近 */
    homing_result_t r = approach_with_backoff(dir, &a1);
    if (r != HOME_OK) {
        goto out;
    }
    /* 退回到行程内一点, 再做第二次 —— 重复性检查是必须的:
     * 偏一步的基准会让机构永远偏, 而这是没法在现场发现的那种错。 */
    vTaskDelay(pdMS_TO_TICKS(500));
    ipropi_stall_reset();
    ipropi_envelope_take();

    r = approach_with_backoff(dir, &a2);
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
        *out_contact_rad = a2;
        r = HOME_OK;
    }

out:
    /* 无论成败都要恢复电压上限 —— 否则电机会一直没力气 */
    foc_motor_set_voltage_limit(saved_limit);
    ESP_LOGI(TAG, "力矩上限已恢复 %.2fV", (double)saved_limit);
    return r;
}

/* 一次尝试 (两端各探一次 + 两次落定)。**收成 static**: 重试是"标定"这个操作的
 * 契约 (策略 FOCSTEP_HOME_RETRY), 不是另一种操作 —— 所以不需要第二个公开名字。 */
static homing_result_t learn_range_once(home_dir_t zero_dir)
{
    /* 满行程点的方向是**派生量**: 两端都是顶出来的机械限位, 而推拉机构只有两个,
     * 所以它必然在零点的反向。只有"零点在正端"(反装) 与"零点在负端"(常规) 两种。 */
    const home_dir_t end_dir = (home_dir_t)(-(int)zero_dir);

    float a_zero = 0.0f, a_end = 0.0f;

    ESP_LOGW(TAG, "=== 两端自学习 (会依次顶两端机械限位) ===");
    homing_result_t r = homing_probe(zero_dir, &a_zero);
    if (r != HOME_OK) {
        ESP_LOGE(TAG, "零点端(朝%s)回零失败: %s", homing_dir_str(zero_dir), homing_result_str(r));
        post_homed(r, a_zero);
        return r;
    }
    r = homing_probe(end_dir, &a_end);
    if (r != HOME_OK) {
        ESP_LOGE(TAG, "满行程端(朝%s)回零失败: %s", homing_dir_str(end_dir), homing_result_str(r));
        post_homed(r, a_end);
        return r;
    }

    if (fabsf(a_end - a_zero) < FOC_MOTOR_MIN_SPAN_RAD) {
        ESP_LOGE(TAG, "两端的角几乎相同 (%.3f / %.3f) ⇒ 行程没学到, 检查机构与方向",
                 (double)a_zero, (double)a_end);
        post_homed(HOME_ERR_RANGE, a_zero);
        return HOME_ERR_RANGE;
    }

    /* ★ 先零点再满行程点 (顺序不能反: 没有零点就没有 100% 可言)。
     *   两次写入各自独立, 另一端**物理位置不动** —— 见 range_commit()。 */
    r = apply_endpoint(true, a_zero);
    if (r == HOME_OK) {
        r = apply_endpoint(false, a_end);
    }

    post_homed(r, a_zero);
    if (r == HOME_OK) {
        float span = 0.0f;
        foc_motor_get_range(nullptr, nullptr, &span);
        ESP_LOGW(TAG, "=== 两端自学习完成: 零点 %.3f rad (朝%s) / 满行程点 %.3f rad (朝%s) "
                      "⇒ span %.3f rad (%s) ===",
                 (double)a_zero, homing_dir_str(zero_dir),
                 (double)a_end, homing_dir_str(end_dir), (double)span,
                 (span > 0) ? "开度增大 = 角度增大" : "开度增大 = 角度减小 (反装)");
    }
    return r;
}

/* 落定: 只写端点, **不动电机** (角度是上一轮 homing_probe 测的) */
homing_result_t homing_apply_contact(float angle, bool as_zero)
{
    if (!foc_motor_position_ready()) {
        return HOME_ERR_NOT_READY;
    }
    ESP_LOGW(TAG, "落定已测接触点 %.3f rad 为%s (另一端保持不变)",
             (double)angle, as_zero ? "零点(0%)" : "满行程点(100%)");
    return apply_endpoint(as_zero, angle);
}

homing_result_t homing_learn_range(home_dir_t zero_dir)
{
    if (!foc_motor_position_ready()) {
        return HOME_ERR_NOT_READY;
    }

    /* ★ 重试属于**这个操作的契约**, 不属于任何调用方: 判据抖动值得再试一次,
     *   而轮数上限的道理是每轮都会让门顶两次机械限位, 多轮只增加磨损。 */
    homing_result_t r = HOME_ERR_NO_CONTACT;
    for (uint8_t attempt = 0; attempt <= s_policy.retries; attempt++) {
        r = learn_range_once(zero_dir);
        if (r == HOME_OK) {
            if (attempt > 0) {
                ESP_LOGW(TAG, "第 %u 次重试成功", (unsigned)attempt);
            }
            return HOME_OK;
        }
        ESP_LOGW(TAG, "标定第 %u 次失败: %s (策略重试上限 %u)",
                 (unsigned)(attempt + 1), homing_result_str(r), (unsigned)s_policy.retries);
        if (!foc_motor_position_ready()) {
            break; /* 编码器/电机已经不健康, 再试也没意义 */
        }
    }

    /* 失败处理的前半段 (与自动/手动无关的那半): 旧的标定**原样保留**
     * (range_commit 拒绝写入时整笔不改), 位置仍不可信 ⇒ 开度指令都会被拒 */
    ESP_LOGE(TAG, "标定失败 (共 %u 次): %s ⇒ 保持原有标定, 位置仍不可信, "
                  "请人工确认机构后用 learn/mark 重标",
             (unsigned)(s_policy.retries + 1), homing_result_str(r));
    return r;
}

homing_result_t homing_auto(void)
{
    /* ① 安全前提 */
    if (!s_policy.auto_calib) {
        ESP_LOGW(TAG, "自动标定未获允许 (策略 auto_calib=false) ⇒ 一个动作都不做。"
                      "允许它会让门自己跑到底, 必须在现场确认机构通畅后再开");
        return HOME_ERR_NOT_ALLOWED;
    }
    /* ② 方向: 无人值守 ⇒ 从策略取 (零点在正端还是负端推不出来, 只能配) */
    ESP_LOGW(TAG, "自动标定: 零点朝%s (策略 zero_dir; 反装机构改 "
                  "FOCSTEP_HOME_AUTO_INVERTED 或用 learn both pos)",
             homing_dir_str(s_policy.zero_dir));
    homing_result_t r = homing_learn_range(s_policy.zero_dir);

    /* ③ 失败后果: **只有这条路径**拉故障。手动路径失败时操作员就在旁边,
     *    而且 PS_FAULT 会失磁 + 锁存, 反而挡住他下一步的 home/mark。 */
    if (r != HOME_OK) {
        ESP_LOGE(TAG, "自动标定失败: %s ⇒ 报 PS_FAULT_CALIB 并失磁, 需人工介入后重标",
                 homing_result_str(r));
        power_state_on_fault(PS_FAULT_CALIB);
    }
    return r;
}

/* 手动标定的共用实现: 把**当前位置**定为端点 */
static esp_err_t mark_endpoint(bool as_zero)
{
    if (!foc_motor_position_ready()) {
        ESP_LOGE(TAG, "电机/编码器未就绪");
        return ESP_ERR_INVALID_STATE;
    }
    if (!as_zero && foc_motor_range_state() == RANGE_NONE) {
        ESP_LOGE(TAG, "还没有零点 ⇒ 请先标零点 (mark zero)");
        return ESP_ERR_INVALID_STATE;
    }
    /* 安全: 静止才能标 —— 门还在动时标会把端点标错 */
    foc_motor_status_t st;
    foc_motor_status(&st);
    if (fabsf(st.velocity) > MARK_MAX_VEL_RAD) {
        ESP_LOGW(TAG, "电机还在动 (%.2f rad/s > %.2f) ⇒ 拒绝标定, 等停稳",
                 (double)st.velocity, (double)MARK_MAX_VEL_RAD);
        return ESP_ERR_INVALID_STATE;
    }

    float angle = foc_motor_get_angle_rad();
    ESP_LOGW(TAG, "手动标定: 把当前位置 %.3f rad 定为%s",
             (double)angle, as_zero ? "零点(0%)" : "满行程点(100%)");

    switch (apply_endpoint(as_zero, angle)) {
    case HOME_OK:
        /* 手标 = 位置锚定: "我此刻就在这个基准点上" ⇒ 位置可信 (决定 #2),
         * foc_motor 侧已置位并落盘 */
        post_homed(HOME_OK, angle);
        return ESP_OK;
    case HOME_ERR_RANGE:
        post_homed(HOME_ERR_RANGE, angle);
        return ESP_ERR_INVALID_ARG;
    default:
        return ESP_FAIL;
    }
}

esp_err_t homing_mark_zero(void)
{
    return mark_endpoint(true);
}

esp_err_t homing_mark_end(void)
{
    return mark_endpoint(false);
}

range_state_t homing_range_state(void)
{
    return foc_motor_range_state();
}

bool homing_has_range(void)
{
    /* 判据是**持久化**的行程 (NVS 恢复来的), 而不是"本次开机跑过 learn 没有" */
    return foc_motor_has_range();
}

void homing_print_status(void)
{
    range_state_t st = foc_motor_range_state();
    float zero = 0.0f, end = 0.0f, span = 0.0f;
    foc_motor_get_range(&zero, &end, &span);

    printf("行程状态 : %s\n", foc_motor_range_state_str(st));
    if (st == RANGE_NONE) {
        printf("           **未标定 —— 开度指令会被拒绝; 先 learn 或 mark**\n");
    } else {
        printf("零点(0%%) : %.3f rad\n", (double)zero);
    }
    if (st == RANGE_BOTH) {
        printf("满行程(100%%): %.3f rad   span %.3f rad (带符号)\n",
               (double)end, (double)span);
        printf("方向解读 : 开度增大 = 朝%s, %s\n",
               (span > 0) ? "正(角度增大)" : "负(角度减小)",
               (span > 0) ? "常规安装 (零点在角度小的一侧)"
                          : "反装 (零点在角度大的一侧)");
    } else if (st == RANGE_ZERO_ONLY) {
        printf("           **只有零点 —— 再标满行程点才有 0%%/100%% 语义**\n");
    }
    printf("当前位置 : %.3f rad   位置可信: %s\n",
           (double)foc_motor_get_angle_rad(),
           foc_motor_position_trusted() ? "是" : "**否 (需标定/回零)**");
    printf("标定策略 : 失败重试 %u 次, 自动标定 %s (零点朝%s) —— Kconfig "
           "FOCSTEP_HOME_RETRY / AUTO_ENABLE / AUTO_INVERTED\n",
           (unsigned)s_policy.retries, s_policy.auto_calib ? "允许" : "禁止",
           homing_dir_str(s_policy.zero_dir));
    printf("回零参数 : 力矩 %d mV, 最大行程 %d mrad, 退避 %d mrad,\n"
           "           确认 %d ms, 重复性容差 %d mrad, 总超时 %d ms\n",
           CONFIG_FOCSTEP_HOME_TORQUE_MV, CONFIG_FOCSTEP_HOME_MAX_TRAVEL_MRAD,
           CONFIG_FOCSTEP_HOME_BACKOFF_MRAD, CONFIG_FOCSTEP_HOME_CONFIRM_MS,
           CONFIG_FOCSTEP_HOME_REPEAT_TOL_MRAD, CONFIG_FOCSTEP_HOME_TIMEOUT_MS);
}
