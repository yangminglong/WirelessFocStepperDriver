#include "power_state.h"
#include "board_pins.h"
#include "foc_motor.h"
#include "bus_voltage.h"
#include "platform_events.h"

#include <string.h>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "PWR";

/* KTH5701 命令字 —— 用字面量避免 C 文件 include C++ 头 */
#define ENC_CMD_CONTINUOUS   0x10
#define ENC_CMD_WAKEUP_SLEEP 0x20

static power_state_t s_state = PS_SLEEP;
static uint8_t s_fault_code = 0;
static TickType_t s_fault_since = 0;
static uint32_t s_transitions = 0;
static bool s_inited = false;

/* nFAULT ISR 里置位, tick 里消费 */
static volatile bool s_nfault_flag = false;

/* ------------------------------------------------------------------
 * nSLEEP: 本文件的唯一核心职责
 * ⚠️ 全工程**只有这里**可以写 GPIO18。理由见 power_state.h 顶部:
 *    SLEEP/FAULT 态若忘了拉低 nSLEEP, VREF 分压会持续耗 106µA@3.4V
 *    (折算 24V 输入侧 ≈17.6µA ≈ 0.42mW), 占深睡档基线 (25~65µA@24V) 的 27~70%。
 * ------------------------------------------------------------------ */
static void nsleep_set(bool enable_motor)
{
    gpio_set_level((gpio_num_t)PIN_DRV_nSLEEP, enable_motor ? 1 : 0);
    /* VREF 门控 P-MOS 的栅极就挂在这根线上, 硬件自动跟随, 固件无需额外动作 */
}

/* ------------------------------------------------------------------
 * nFAULT (§10.4 ③): 两片 DRV8874 的 nFAULT 线与, 分不清是哪一片。
 * ------------------------------------------------------------------ */
static void IRAM_ATTR nfault_isr_handler(void *arg)
{
    (void)arg;
    /* ISR 里直接断电 —— 唯一允许在 ISR 上下文碰 nSLEEP 的地方,
     * 且是"关断"方向, 即使与主循环竞争也只会更安全。 */
    gpio_set_level((gpio_num_t)PIN_DRV_nSLEEP, 0);
    s_nfault_flag = true;
}

static bool nfault_asserted(void)
{
    /* nFAULT 低有效 (10k 上拉) */
    return gpio_get_level((gpio_num_t)PIN_DRV_nFAULT) == 0;
}

/* ------------------------------------------------------------------ */

static void post_state_change(power_state_t from, power_state_t to)
{
    focstep_evt_state_t e = {};
    e.base.timestamp_ms = platform_now_ms();
    e.from = (int)from;
    e.to = (int)to;
    platform_event_post(FOCSTEP_EVT_STATE_CHANGED, &e, sizeof(e));
}

static void enter_state(power_state_t st)
{
    if (st == s_state) {
        return;
    }
    power_state_t from = s_state;
    ESP_LOGI(TAG, "%s -> %s", power_state_name(from), power_state_name(st));
    s_state = st;
    s_transitions++;

    switch (st) {
    case PS_SLEEP:
        /* 顺序: 停运动 → 电机断电 → 编码器转低功耗档 → (轻睡档) 暂停 FOC 任务
         * ⚠️ `nsleep_set(false)` **一根脚同时办三件事** (见 power_state.h 文件头):
         *    DRV 断电 + VREF 门控关断 + **CAN 的 Rs 转睡眠** —— 不需要额外调用。 */
        foc_motor_stop();
        nsleep_set(false);
        foc_motor_encoder_set_mode(ENC_CMD_WAKEUP_SLEEP);
#if CONFIG_FOCSTEP_SLEEP_MODE_LIGHT || CONFIG_FOCSTEP_PA_WAKE_ENABLE
        /* 需要"系统能真正 idle"的两档都要暂停 FOC 任务:
         *   · 轻睡档 (显式 esp_light_sleep_start): 否则 1kHz 任务让系统永远不闲;
         *   · 无线监听档 (PA + PM 自动轻睡): 同上, 而且它是长驻的那一档。
         * 顺带解决另一面: 醒来时 vTaskDelayUntil 会追打时间基 (暂停时已置 resync)。
         * ⚠️ 暂停必须是**协作式**的 —— 见 foc_motor_pause_loop 的注释 (I2C 锁)。 */
        foc_motor_pause_loop();
#endif
        break;

    case PS_ACTIVE:
#if CONFIG_FOCSTEP_SLEEP_MODE_LIGHT || CONFIG_FOCSTEP_PA_WAKE_ENABLE
        /* 先把被暂停的 FOC 任务放回来 (它恢复时会自动重置时间基) */
        foc_motor_resume_loop();
#endif
        /* 编码器先切连续档 (否则读不到角度), 再使能电机 */
        foc_motor_encoder_set_mode(ENC_CMD_CONTINUOUS);
        nsleep_set(true);
        /* ★ 唤醒后先切 brake 泄放反灌能量, 再读角度 (§10.3 硬性行为)。
         *   PH/EN 模式下 brake = 两相 EN 恒低 = 两个低边导通。 */
        foc_motor_brake();
        vTaskDelay(pdMS_TO_TICKS(20));
        foc_motor_enable(true);
        break;

    case PS_FAULT:
        /* 硬件故障: 电机必须断电 */
        foc_motor_stop();
        nsleep_set(false);
        s_fault_since = xTaskGetTickCount();
        break;
    }

    post_state_change(from, st);
}

esp_err_t power_state_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_DRV_nSLEEP,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "nSLEEP gpio config failed");

    /* 上电安全态: 电机断电。板上 GPIO18 本来就有 10k 下拉, 这里再显式拉低,
     * 保证 VREF 门控 P-MOS 关断, 同时让 CAN 的 Rs 保持睡眠。 */
    nsleep_set(false);

    /* nFAULT: 低有效, 10k 上拉。任一下降沿立即断电。 */
    gpio_config_t nf = {
        .pin_bit_mask = 1ULL << PIN_DRV_nFAULT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&nf), TAG, "nFAULT gpio config failed");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add((gpio_num_t)PIN_DRV_nFAULT,
                                             nfault_isr_handler, NULL),
                        TAG, "nFAULT isr add failed");

    s_inited = true;
    ESP_LOGI(TAG, "init done; nSLEEP=低 (电机断电, VREF 门控关断)");
    return ESP_OK;
}

void power_state_request(power_state_t want)
{
    if (!s_inited) {
        return;
    }
    if (want == PS_FAULT && s_state != PS_FAULT) {
        enter_state(PS_FAULT);
        return;
    }
    if (s_state == PS_FAULT) {
        /* 故障态必须显式清除才能离开 */
        ESP_LOGW(TAG, "处于 FAULT, 先 power_state_clear_fault() 才能离开");
        return;
    }
    enter_state(want);
}

void power_state_clear_fault(void)
{
    s_fault_code = 0;
    if (s_state == PS_FAULT) {
        enter_state(PS_SLEEP);
    }
}

void power_state_on_fault(uint8_t code)
{
    s_fault_code = (code == 0) ? PS_FAULT_NFAULT : code;
    enter_state(PS_FAULT);
    ESP_LOGE(TAG, "故障码 %u", s_fault_code);
}

void power_state_tick(void)
{
    if (!s_inited) {
        return;
    }

    /* ---- §10.4 ③: nFAULT (硬件保护, 平台职责) ---- */
    if (s_nfault_flag) {
        s_nfault_flag = false;
        if (s_state != PS_FAULT) {
            ESP_LOGE(TAG, "nFAULT 拉低 (两片线与, 无法区分是哪一片)");
            s_fault_code = PS_FAULT_NFAULT;
            enter_state(PS_FAULT);
            focstep_evt_base_t e = {.timestamp_ms = platform_now_ms()};
            platform_event_post(FOCSTEP_EVT_NFAULT, &e, sizeof(e));
            return;
        }
    }

    if (s_state == PS_FAULT) {
        /* nFAULT 释放且持续 3s → 自动恢复。
         * (应用若想保持故障态不恢复, 可订阅事件后自行处理) */
        if (!nfault_asserted()) {
            if ((xTaskGetTickCount() - s_fault_since) > pdMS_TO_TICKS(3000)) {
                ESP_LOGW(TAG, "nFAULT 已释放, 恢复");
                s_fault_code = 0;
                enter_state(PS_SLEEP);
            }
        } else {
            s_fault_since = xTaskGetTickCount();
        }
        return;
    }

    /* ---- §10.4 ①: 母线电压 (硬件保护, 平台职责) ---- */
    if (s_state == PS_ACTIVE) {
        int mv = 0;
        if (bus_voltage_read_mv(&mv) == ESP_OK && mv > 0 &&
            mv < CONFIG_FOCSTEP_VBUS_MIN_ENABLE_MV) {
            ESP_LOGW(TAG, "VBUS %d mV < %d mV: 禁使能电机",
                     mv, CONFIG_FOCSTEP_VBUS_MIN_ENABLE_MV);
            s_fault_code = PS_FAULT_VBUS_LOW;
            enter_state(PS_FAULT);
            focstep_evt_vbus_t e = {.base.timestamp_ms = platform_now_ms(), .vbus_mv = mv};
            platform_event_post(FOCSTEP_EVT_VBUS_LOW, &e, sizeof(e));
            return;
        }
    }
}

power_state_t power_state_current(void)
{
    return s_state;
}

const char *power_state_name(power_state_t s)
{
    switch (s) {
    case PS_SLEEP:
        return "SLEEP";
    case PS_ACTIVE:
        return "ACTIVE";
    case PS_FAULT:
        return "FAULT";
    default:
        return "?";
    }
}

uint8_t power_state_fault_code(void)
{
    return s_fault_code;
}

void power_state_snapshot(power_state_snapshot_t *out)
{
    if (!out) {
        return;
    }
    out->state = s_state;
    out->fault_code = s_fault_code;
    out->transitions = s_transitions;
}
