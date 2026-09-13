#include "app_door.h"

#include "board_pins.h"
#include "can_link.h"
#include "foc_motor.h"
#include "homing.h"
#include "ipropi_sense.h"
#include "led_ws2812.h"
#include "platform_events.h"
#include "power_state.h"
#include "wakeup.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "APP";

/* 助动: 判定"用户在拉"的速度阈值 (rad/s)。低于它认为是噪声/静止。 */
#define ASSIST_VEL_THRESHOLD 0.5f
/* 助动超时: 多久没有手拉动作就结束接管回深睡 (Kconfig 可调) */

static app_mode_t s_mode = APP_MODE_IDLE;
static TickType_t s_mode_since = 0;
static TickType_t s_last_cmd = 0;
static bool s_stall_acted = false;
static float s_assist_volts = 2.0f;

const char *app_door_mode_name(app_mode_t m)
{
    switch (m) {
    case APP_MODE_IDLE:    return "IDLE";
    case APP_MODE_ASSIST:  return "ASSIST";
    case APP_MODE_RUNNING: return "RUNNING";
    default:               return "?";
    }
}

app_mode_t app_door_mode(void)
{
    return s_mode;
}

/* ★ 平台只提供颜色 (LED_COLOR_*), **"哪个状态用什么颜色"是应用的决定** ——
 *  这就是 docs/doc.md §10.3 那一列的落点。重构前这个映射写在平台层里。 */
static void set_mode(app_mode_t m)
{
    if (m == s_mode) {
        return;
    }
    ESP_LOGI(TAG, "%s -> %s", app_door_mode_name(s_mode), app_door_mode_name(m));
    s_mode = m;
    s_mode_since = xTaskGetTickCount();

    switch (m) {
    case APP_MODE_IDLE:
        led_set_color(LED_COLOR_OFF);        /* 深睡=灭 */
        break;
    case APP_MODE_ASSIST:
        /* §10.3: 唤醒接管 = CAN 收发器睡眠 (Rs 高) */
        power_state_set_can_active(false);
        led_set_color(LED_COLOR_VIOLET);     /* 唤醒接管=紫 */
        foc_motor_set_voltage_limit(s_assist_volts); /* 助动用小力矩，防冲击 */
        break;
    case APP_MODE_RUNNING:
        /* §10.3: 运行 = CAN 工作 (Rs 低) */
        power_state_set_can_active(true);
        led_set_color(LED_COLOR_GREEN);      /* 运行=绿 */
        /* 恢复运行力矩 (助动时被调小过) */
        foc_motor_set_voltage_limit(foc_motor_default_voltage_limit());
        break;
    }
}

/* ------------------------------------------------------------------ */
/* 事件处理                                                            */

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base != FOCSTEP_EVENT) {
        return;
    }

    switch ((focstep_event_id_t)id) {
    case FOCSTEP_EVT_BUTTON_SINGLE:
        ESP_LOGI(TAG, "按键: 短按 → 停");
        app_door_command(APP_CMD_STOP);
        break;

    case FOCSTEP_EVT_BUTTON_DOUBLE: {
        /* 双击在开/关之间切换 */
        static bool want_open = true;
        want_open = !want_open;
        ESP_LOGI(TAG, "按键: 双击 → %s", want_open ? "开" : "关");
        app_door_command(want_open ? APP_CMD_OPEN : APP_CMD_CLOSE);
        break;
    }

    case FOCSTEP_EVT_BUTTON_LONG:
        ESP_LOGI(TAG, "按键: 长按 → 回深睡");
        set_mode(APP_MODE_IDLE);
        power_state_request(PS_SLEEP);
        break;

    case FOCSTEP_EVT_CAN_CMD: {
        const focstep_evt_can_cmd_t *e = (const focstep_evt_can_cmd_t *)data;
        switch ((can_cmd_t)e->cmd) {
        case CAN_CMD_OPEN:  app_door_command(APP_CMD_OPEN);  break;
        case CAN_CMD_CLOSE: app_door_command(APP_CMD_CLOSE); break;
        case CAN_CMD_STOP:  app_door_command(APP_CMD_STOP);  break;
        default: break;
        }
        break;
    }

    case FOCSTEP_EVT_STALL: {
        const focstep_evt_stall_t *e = (const focstep_evt_stall_t *)data;
        if (s_stall_acted) {
            break; /* 锁存期间不重复处置 */
        }
        s_stall_acted = true;
        ESP_LOGW(TAG, "堵门: 停运动并保持力矩 (峰值 %.0f mA)", e->peak_ma);
        /* 不直接断电 —— 保持力矩防门滑落 */
        foc_motor_set_mode(FOC_MODE_TORQUE);
        foc_motor_set_torque(0.0f);
        foc_motor_set_mode(FOC_MODE_IDLE);
        break;
    }

    case FOCSTEP_EVT_STALL_CLEARED:
        s_stall_acted = false;
        break;

    case FOCSTEP_EVT_NFAULT:
        /* 平台已自动拉低 nSLEEP; 应用负责点灯告警 */
        led_show_fault_code(PS_FAULT_NFAULT);
        break;

    case FOCSTEP_EVT_ENCODER_UNHEALTHY:
        led_show_fault_code(PS_FAULT_ENCODER);
        break;

    case FOCSTEP_EVT_VBUS_LOW:
        led_show_fault_code(PS_FAULT_VBUS_LOW);
        break;

    case FOCSTEP_EVT_CAN_LOST:
        /* §10.4 ④: 上报, 但不影响本地控制 (手拉助动照常工作) */
        ESP_LOGW(TAG, "CAN 掉线 (本地控制不受影响)");
        break;

    case FOCSTEP_EVT_STATE_CHANGED: {
        const focstep_evt_state_t *e = (const focstep_evt_state_t *)data;
        if ((power_state_t)e->to == PS_SLEEP) {
            set_mode(APP_MODE_IDLE);
        } else if ((power_state_t)e->to == PS_FAULT) {
            set_mode(APP_MODE_IDLE);
            led_show_fault_code(power_state_fault_code());
        }
        break;
    }

    case FOCSTEP_EVT_WOKE: {
        /* 被编码器 INT 唤醒 = 手拉 ⇒ 进接管助动 */
        const focstep_evt_wake_t *e = (const focstep_evt_wake_t *)data;
        if (e->src == WAKE_SRC_ENCODER_INT) {
            ESP_LOGI(TAG, "手拉唤醒 → 唤醒接管");
            power_state_request(PS_ACTIVE);
            set_mode(APP_MODE_ASSIST);
        }
        break;
    }

    default:
        break;
    }
}

/* ------------------------------------------------------------------ */

void app_door_command(app_cmd_t cmd)
{
    s_last_cmd = xTaskGetTickCount();

    switch (cmd) {
    case APP_CMD_WAKE:
        if (power_state_current() == PS_SLEEP) {
            power_state_request(PS_ACTIVE);
            set_mode(APP_MODE_ASSIST);
        }
        break;

    case APP_CMD_OPEN:
    case APP_CMD_CLOSE: {
        if (power_state_current() != PS_ACTIVE) {
            power_state_request(PS_ACTIVE);
        }
        if (!foc_motor_position_trusted()) {
            /* 位置不可信时拒绝执行 —— 否则会以错误基准冲到机械限位。
             * 正确做法是先回零 (learn / home 命令, 或应用自动触发)。 */
            ESP_LOGE(TAG, "位置不可信, 拒绝 %s 指令 —— 请先回零 (learn)",
                     (cmd == APP_CMD_OPEN) ? "开" : "关");
            led_show_fault_code(PS_FAULT_ENCODER);
            return;
        }
        set_mode(APP_MODE_RUNNING);
        /* 解锁堵转锁存 —— 新指令意味着重新开始判定 */
        ipropi_stall_reset();
        ipropi_envelope_take();
        s_stall_acted = false;
        foc_motor_move_to((cmd == APP_CMD_OPEN) ? 1.0f : 0.0f);
        break;
    }

    case APP_CMD_STOP:
        foc_motor_stop();
        if (power_state_current() == PS_ACTIVE) {
            /* 停在原地保持力矩, 不立刻掉电 */
            set_mode(APP_MODE_ASSIST);
        }
        break;

    default:
        break;
    }
}

/* ------------------------------------------------------------------ */

void app_door_tick(void)
{
    TickType_t now = xTaskGetTickCount();

    /* 堵转判定: 采样已在 FOC 循环里做成峰值包络 (1kHz), 这里只读包络 + 判定。
     * 触发后 ipropi 会发 FOCSTEP_EVT_STALL, 由本文件的事件处理器处置。 */
    if (power_state_current() == PS_ACTIVE) {
        ipropi_stall_update(ipropi_envelope_take());
    } else {
        ipropi_envelope_take(); /* 丢弃, 免得下次拿旧峰值误判 */
    }

    switch (s_mode) {
    case APP_MODE_ASSIST: {
        /* 手拉助动: 用编码器速度判断用户意图方向, 给同向力矩。
         * 不做位置环 —— 否则会跟用户"较劲"。 */
        foc_motor_status_t st;
        foc_motor_status(&st);
        float v = st.velocity;
        if (fabsf(v) > ASSIST_VEL_THRESHOLD) {
            foc_motor_set_torque((v > 0) ? s_assist_volts : -s_assist_volts);
        } else {
            /* 没在动 → 零力矩, 不跟用户较劲也不发热 */
            foc_motor_set_torque(0.0f);
        }
        /* 接管超时 → 回深睡 */
        if ((now - s_mode_since) > pdMS_TO_TICKS(CONFIG_FOCSTEP_WAKE_HOLD_MS)) {
            ESP_LOGI(TAG, "接管超时, 回深睡");
            set_mode(APP_MODE_IDLE);
            power_state_request(PS_SLEEP);
        }
        break;
    }

    case APP_MODE_RUNNING: {
        /* 到位判定: 位置误差进入容差且速度接近 0 */
        foc_motor_status_t st;
        foc_motor_status(&st);
        float err = fabsf(st.target_rad - st.multi_turn_rad);
        if (err < 0.05f && fabsf(st.velocity) < 0.5f) {
            ESP_LOGI(TAG, "到位 (误差 %.4f rad), 转入保持", (double)err);
            set_mode(APP_MODE_ASSIST);
        } else if ((now - s_last_cmd) > pdMS_TO_TICKS(CONFIG_FOCSTEP_IDLE_TO_SLEEP_MS)) {
            ESP_LOGI(TAG, "长时间无指令, 回深睡");
            set_mode(APP_MODE_IDLE);
            power_state_request(PS_SLEEP);
        }
        break;
    }

    case APP_MODE_IDLE:
    default:
        break;
    }
}

esp_err_t app_door_init(void)
{
    /* 助动力矩来自 Kconfig (字符串) */
    s_assist_volts = strtof(CONFIG_FOCSTEP_ASSIST_TORQUE, NULL);
    if (s_assist_volts <= 0.0f) {
        s_assist_volts = 0.5f;
    }

    esp_err_t ret = esp_event_handler_instance_register(
        FOCSTEP_EVENT, ESP_EVENT_ANY_ID, on_event, NULL, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "订阅平台事件失败: %s", esp_err_to_name(ret));
        return ret;
    }

    s_mode = APP_MODE_IDLE;
    s_mode_since = xTaskGetTickCount();
    s_last_cmd = s_mode_since;
    led_set_color(LED_COLOR_OFF);        /* 深睡=灭 */

    ESP_LOGI(TAG, "推拉门应用就绪: 助动力矩 %.2fV, 接管保持 %d ms, 空闲入睡 %d ms",
             (double)s_assist_volts, CONFIG_FOCSTEP_WAKE_HOLD_MS,
             CONFIG_FOCSTEP_IDLE_TO_SLEEP_MS);
    return ESP_OK;
}
