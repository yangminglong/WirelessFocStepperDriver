#include "can_link.h"
#include "board_pins.h"
#include "platform_events.h"

#include <string.h>

#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "CAN";

/* 自定义单帧协议: ID = base + cmd, 1 字节数据 = 0xA5 (防误触发) */
#define CAN_DLC 1
#define CAN_MAGIC 0xA5

static bool s_inited = false;
static bool s_active = false; /* TWAI 已 start (与收发器的醒睡无关, 见下) */
static TickType_t s_last_rx = 0;
static bool s_bus_off = false;
static bool s_lost_reported = false;
static uint32_t s_rx_count = 0;

/* ★ 这里**没有** can_set_active() —— Rs 接在 VREF 门控 NPN 的集电极上, **随 nSLEEP 硬件派生**:
 *       nSLEEP(GPIO18) 高 → NPN 饱和 → 集电极 ≈0.1V → Rs 低 = CAN 唤醒
 *       nSLEEP(GPIO18) 低 → NPN 截止 → 集电极 3.4V  → Rs 高 = CAN 睡眠
 *   ⇒ CAN 的醒睡由 power_state.c 抬高/拉低 GPIO18 间接决定, **软件无法独立控制**,
 *     上电默认 nSLEEP=低 ⇒ Rs 高 ⇒ CAN 睡眠 —— 由硬件保证, 不靠固件。
 *   见 docs/doc.md §5.3 与 board_pins.h。 */

esp_err_t can_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    /* ⚠️ 没有 Rs 的 GPIO 要配 (上电默认睡眠由硬件保证)。
     *
     * ⚠️⚠️ **TWAI 一旦 start 就占用 GPIO16/17, 会把 UART0 控制台顶掉** ——
     *    这两个脚是共用的 (4P 调试排针 / CAN 收发器), 同一时刻只能有一个外设驱动。
     *    ⇒ **本函数只能在"用 CAN 档"调用**: 该档的控制台走 USB-Serial-JTAG。
     *    app_main.c 用 CONFIG_FOCSTEP_CAN_ENABLE 把这条开关钉在构建期。 */

    twai_general_config_t gcfg = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)PIN_CAN_TXD, (gpio_num_t)PIN_CAN_RXD,
#if CONFIG_FOCSTEP_CAN_NO_ACK
        TWAI_MODE_NO_ACK /* 单板 bring-up: 无其他节点 ACK 时也能发出去 */
#else
        TWAI_MODE_NORMAL
#endif
    );
    twai_timing_config_t tcfg;
    switch (CONFIG_FOCSTEP_CAN_BITRATE) {
    case 125000:
        tcfg = (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS();
        break;
    case 250000:
        tcfg = (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();
        break;
    case 500000:
        tcfg = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
        break;
    case 800000:
        tcfg = (twai_timing_config_t)TWAI_TIMING_CONFIG_800KBITS();
        break;
    case 1000000:
        tcfg = (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();
        break;
    default:
        ESP_LOGW(TAG, "unsupported bitrate %d, falling back to 500k", CONFIG_FOCSTEP_CAN_BITRATE);
        tcfg = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
        break;
    }
    twai_filter_config_t fcfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_RETURN_ON_ERROR(twai_driver_install(&gcfg, &tcfg, &fcfg), TAG, "driver install failed");
    /* 先不进 started, 由 ensure_started() 在第一次 tick 再启 —— 深睡档不需要 CAN */
    s_last_rx = xTaskGetTickCount();
    s_inited = true;
    ESP_LOGI(TAG, "init done: bitrate=%d mode=%s", CONFIG_FOCSTEP_CAN_BITRATE,
#if CONFIG_FOCSTEP_CAN_NO_ACK
             "NO_ACK"
#else
             "NORMAL"
#endif
    );
    return ESP_OK;
}

static void ensure_started(void)
{
    if (!s_active) {
        if (twai_start() != ESP_OK) {
            ESP_LOGE(TAG, "twai_start failed");
        } else {
            s_active = true;
            s_last_rx = xTaskGetTickCount();
        }
    }
}

void can_tick(void)
{
    if (!s_inited || !s_active) {
        return;
    }

    twai_status_info_t st;
    if (twai_get_status_info(&st) == ESP_OK) {
        if (st.state == TWAI_STATE_BUS_OFF) {
            s_bus_off = true;
            /* 自动恢复: 总线关闭后必须显式 init 回 on */
            twai_initiate_recovery();
            ESP_LOGW(TAG, "bus-off, recovery initiated");
        } else {
            s_bus_off = false;
        }
    }

    twai_message_t msg;
    /* 零等待: 这是周期任务的一部分, 不能阻塞 FOC */
    while (twai_receive(&msg, 0) == ESP_OK) {
        s_last_rx = xTaskGetTickCount();
        s_rx_count++;

        if (msg.data_length_code != CAN_DLC || msg.data[0] != CAN_MAGIC) {
            continue;
        }
        uint32_t id = msg.identifier - CONFIG_FOCSTEP_CAN_NODE_ID;
        can_cmd_t c = CAN_CMD_NONE;
        switch (id) {
        case 1:
            c = CAN_CMD_OPEN;
            break;
        case 2:
            c = CAN_CMD_CLOSE;
            break;
        case 3:
            c = CAN_CMD_STOP;
            break;
        default:
            break;
        }
        if (c != CAN_CMD_NONE) {
            /* 只发事件, 不执行 —— 怎么响应是应用的事 */
            focstep_evt_can_cmd_t e = {.base.timestamp_ms = platform_now_ms(), .cmd = (int)c};
            platform_event_post(FOCSTEP_EVT_CAN_CMD, &e, sizeof(e));
        }
    }

    /* ---- §10.4 ④: 丢帧 (只报一次, 边沿触发) ---- */
    bool lost = (xTaskGetTickCount() - s_last_rx) > pdMS_TO_TICKS(CONFIG_FOCSTEP_CAN_TIMEOUT_MS);
    if (lost && !s_lost_reported) {
        s_lost_reported = true;
        ESP_LOGW(TAG, "CAN 丢帧超过 %d ms", CONFIG_FOCSTEP_CAN_TIMEOUT_MS);
        focstep_evt_base_t e = {.timestamp_ms = platform_now_ms()};
        platform_event_post(FOCSTEP_EVT_CAN_LOST, &e, sizeof(e));
    } else if (!lost && s_lost_reported) {
        s_lost_reported = false;
    }
}

bool can_link_lost(void)
{
    if (!s_inited || !s_active) {
        return false; /* 没启用 CAN 就谈不上"丢帧" */
    }
    return (xTaskGetTickCount() - s_last_rx) > pdMS_TO_TICKS(CONFIG_FOCSTEP_CAN_TIMEOUT_MS);
}

bool can_bus_off(void)
{
    return s_bus_off;
}

uint32_t can_rx_count(void)
{
    return s_rx_count;
}
