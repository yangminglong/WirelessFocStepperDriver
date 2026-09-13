#include "platform_button.h"
#include "board_pins.h"
#include "platform_events.h"

#include "esp_log.h"
#include "esp_check.h"

/* 官方组件: 消抖 + 短按/双击/长按事件识别 */
#include "iot_button.h"
#include "button_gpio.h"

static const char *TAG = "PLAT_BTN";

/* 长按阈值。要明显长于"双击"的窗口, 否则双击会被误判成长按。 */
#define LONG_PRESS_MS  3000
#define SHORT_PRESS_MS 50

/* ★ 本模块只发事件, 不 import power_state —— 决策在应用层 */
static void post_button(focstep_event_id_t id, int clicks, uint32_t hold_ms)
{
    focstep_evt_button_t e = {};
    e.base.timestamp_ms = platform_now_ms();
    e.click_count = clicks;
    e.hold_ms = hold_ms;
    platform_event_post(id, &e, sizeof(e));
}

static void on_single_click(void *arg, void *usr_data)
{
    (void)arg;
    (void)usr_data;
    ESP_LOGI(TAG, "短按");
    post_button(FOCSTEP_EVT_BUTTON_SINGLE, 1, 0);
}

static void on_double_click(void *arg, void *usr_data)
{
    (void)arg;
    (void)usr_data;
    ESP_LOGI(TAG, "双击");
    post_button(FOCSTEP_EVT_BUTTON_DOUBLE, 2, 0);
}

static void on_long_press(void *arg, void *usr_data)
{
    (void)arg;
    (void)usr_data;
    ESP_LOGI(TAG, "长按 %d ms", LONG_PRESS_MS);
    post_button(FOCSTEP_EVT_BUTTON_LONG, 1, LONG_PRESS_MS);
}

esp_err_t platform_button_init(void)
{
    button_config_t cfg = {
        .long_press_time = LONG_PRESS_MS,
        .short_press_time = SHORT_PRESS_MS,
    };
    button_gpio_config_t gpio_cfg = {
        .gpio_num = PIN_STATUS_LED, /* GPIO9 —— BOOT 按键 */
        .active_level = 0,          /* 按下接地 = 低有效 */
        .enable_power_save = false, /* 深睡时按键不参与唤醒, 无需低功耗轮询 */
        .disable_pull = false,      /* 板上已有 10k 上拉, 内部再上一个也无妨 */
    };

    button_handle_t btn = NULL;
    ESP_RETURN_ON_ERROR(iot_button_new_gpio_device(&cfg, &gpio_cfg, &btn),
                        TAG, "iot_button_new_gpio_device failed");

    ESP_RETURN_ON_ERROR(
        iot_button_register_cb(btn, BUTTON_SINGLE_CLICK, NULL, on_single_click, NULL),
        TAG, "register SINGLE_CLICK failed");
    ESP_RETURN_ON_ERROR(
        iot_button_register_cb(btn, BUTTON_DOUBLE_CLICK, NULL, on_double_click, NULL),
        TAG, "register DOUBLE_CLICK failed");
    ESP_RETURN_ON_ERROR(
        iot_button_register_cb(btn, BUTTON_LONG_PRESS_START, NULL, on_long_press, NULL),
        TAG, "register LONG_PRESS_START failed");

    ESP_LOGI(TAG, "init done (GPIO%d, 低有效) —— 只发事件, 决策在应用层", PIN_STATUS_LED);
    return ESP_OK;
}
