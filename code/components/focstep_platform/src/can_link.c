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
static bool s_active = false;
static TickType_t s_last_rx = 0;
static bool s_bus_off = false;
static bool s_lost_reported = false;
static uint32_t s_rx_count = 0;

esp_err_t can_set_active(bool active)
{
    /* Rs 低 = 正常工作, Rs 高 = 睡眠 (10k 上拉决定上电默认睡眠) */
    esp_err_t ret = gpio_set_level((gpio_num_t)PIN_CAN_RS, active ? 0 : 1);
    if (ret == ESP_OK) {
        s_active = active;
    }
    return ret;
}

esp_err_t can_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_CAN_RS,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "Rs gpio config failed");
    /* 上电安全态: CAN 睡眠 (§10.3) */
    ESP_RETURN_ON_ERROR(can_set_active(false), TAG, "Rs init failed");

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
    /* 先不进 started, 由 can_set_active(true) 时再启 —— 深睡档不需要 CAN */
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
        can_set_active(true);
        if (twai_start() != ESP_OK) {
            ESP_LOGE(TAG, "twai_start failed");
        } else {
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
