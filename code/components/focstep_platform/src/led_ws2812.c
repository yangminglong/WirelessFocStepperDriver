#include "led_ws2812.h"
#include "board_pins.h"

#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* 官方组件: 闪灯模式引擎 (blink_step_t 表 + 自带任务), 底层复用 led_strip。
 * 自己写闪灯时序没有意义 —— 这本来就是 led_indicator 的职责。 */
#include "led_indicator.h"
#include "led_indicator_strips.h"

static const char *TAG = "LED";

#define RMT_RESOLUTION_HZ (10 * 1000 * 1000) /* 10MHz → 0.1µs 分辨率 */

/* 颜色 (RGB 打包成 0xRRGGBB)。数值取得很小, 配合 Kconfig 亮度, 天然满足"禁全白"。 */
#define RGB_OFF    0x000000
#define RGB_VIOLET 0x180018
#define RGB_GREEN  0x001800
#define RGB_BLUE   0x000018
#define RGB_WHITE  0x101010 /* 注意: 平台侧再乘 Kconfig 亮度, 天然远离"全白" */
#define RGB_DIM    0x020202
#define RGB_RED    0x180000 /* 故障闪码 */

/*
 * 闪灯模式表。索引即 led_indicator_start() 的 blink_type。
 * 每个模式必须以 LED_BLINK_STOP 或 LED_BLINK_LOOP 结尾。
 */
static const blink_step_t s_st_off[] = {
    {LED_BLINK_RGB, RGB_OFF, 0},
    {LED_BLINK_HOLD, LED_STATE_OFF, 0},
    {LED_BLINK_STOP, 0, 0},
};

/* 常亮颜色 —— 应用自己选哪个颜色代表哪个状态 */
static const blink_step_t s_st_dim[] = {
    {LED_BLINK_RGB, RGB_DIM, 0},
    {LED_BLINK_HOLD, LED_STATE_ON, 0},
    {LED_BLINK_LOOP, 0, 0},
};
static const blink_step_t s_st_violet[] = {
    {LED_BLINK_RGB, RGB_VIOLET, 0},
    {LED_BLINK_HOLD, LED_STATE_ON, 0},
    {LED_BLINK_LOOP, 0, 0},
};
static const blink_step_t s_st_green[] = {
    {LED_BLINK_RGB, RGB_GREEN, 0},
    {LED_BLINK_HOLD, LED_STATE_ON, 0},
    {LED_BLINK_LOOP, 0, 0},
};
static const blink_step_t s_st_blue[] = {
    {LED_BLINK_RGB, RGB_BLUE, 0},
    {LED_BLINK_HOLD, LED_STATE_ON, 0},
    {LED_BLINK_LOOP, 0, 0},
};
static const blink_step_t s_st_white[] = {
    {LED_BLINK_RGB, RGB_WHITE, 0},
    {LED_BLINK_HOLD, LED_STATE_ON, 0},
    {LED_BLINK_LOOP, 0, 0},
};

/* 故障闪码: 亮 120ms / 灭 200ms 重复 N 次, 然后长灭 1200ms 再循环。
 * docs/doc.md §10.4 的故障码 1~7 各一条 (N = 故障码)。
 * 每条的重复次数不同, 无法用同一个宏参数化, 故显式写 7 条。 */
static const blink_step_t s_st_fault1[] = {
    {LED_BLINK_RGB, RGB_RED, 0}, {LED_BLINK_HOLD, LED_STATE_ON, 120},
    {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_OFF, 1200},
    {LED_BLINK_LOOP, 0, 0},
};
static const blink_step_t s_st_fault2[] = {
    {LED_BLINK_RGB, RGB_RED, 0},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_OFF, 1200},
    {LED_BLINK_LOOP, 0, 0},
};
static const blink_step_t s_st_fault3[] = {
    {LED_BLINK_RGB, RGB_RED, 0},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_OFF, 1200},
    {LED_BLINK_LOOP, 0, 0},
};
static const blink_step_t s_st_fault4[] = {
    {LED_BLINK_RGB, RGB_RED, 0},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_OFF, 1200},
    {LED_BLINK_LOOP, 0, 0},
};
static const blink_step_t s_st_fault5[] = {
    {LED_BLINK_RGB, RGB_RED, 0},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_OFF, 1200},
    {LED_BLINK_LOOP, 0, 0},
};
static const blink_step_t s_st_fault6[] = {
    {LED_BLINK_RGB, RGB_RED, 0},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_OFF, 1200},
    {LED_BLINK_LOOP, 0, 0},
};

static const blink_step_t s_st_fault7[] = {
    {LED_BLINK_RGB, RGB_RED, 0},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_ON, 120}, {LED_BLINK_HOLD, LED_STATE_OFF, 200},
    {LED_BLINK_HOLD, LED_STATE_OFF, 1200},
    {LED_BLINK_LOOP, 0, 0},
};

static blink_step_t const *s_blink_lists[] = {
    [LED_IDX_OFF]         = s_st_off,
    [LED_IDX_DIM]         = s_st_dim,
    [LED_IDX_VIOLET]      = s_st_violet,
    [LED_IDX_GREEN]       = s_st_green,
    [LED_IDX_BLUE]        = s_st_blue,
    [LED_IDX_WHITE]       = s_st_white,
    [LED_IDX_FAULT1]      = s_st_fault1,
    [LED_IDX_FAULT2]      = s_st_fault2,
    [LED_IDX_FAULT3]      = s_st_fault3,
    [LED_IDX_FAULT4]      = s_st_fault4,
    [LED_IDX_FAULT5]      = s_st_fault5,
    [LED_IDX_FAULT6]      = s_st_fault6,
    [LED_IDX_FAULT7]      = s_st_fault7,
};

static led_indicator_handle_t s_led = NULL;
static bool s_inited = false;

/* 灯带 3.4V **常供、无供电门控** ⇒ 本模块不提供 led_power() / led_power_is_on():
 *   开关灯只靠 led_set_color() 写颜色。理由见 led_ws2812.h 头注释。 */

esp_err_t led_set_color(led_color_t color)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    int idx = (int)color;
    if (idx < 0 || idx >= LED_IDX_MAX) {
        idx = LED_IDX_OFF;
    }

    /* ★ "灭" 是**发一帧全黑**: s_st_off 先写 RGB=0 再置 OFF,
     *   灯珠据此锁存全黑, 之后 IC 回到静态 ≦1µA。
     *   ⚠️ **这是深睡不亮灯的唯一保障 —— 已无硬件兜底**, 见 led_ws2812.h 纪律 1。 */
    return led_indicator_start(s_led, idx);
}

esp_err_t led_off_and_wait(void)
{
    esp_err_t ret = led_set_color(LED_COLOR_OFF);
    if (ret != ESP_OK) {
        return ret;
    }
    /* 让出调度 ≥5 个 led_indicator 定时器周期 (周期 1ms) ⇒ Tmr Svc 必然执行
     * s_st_off: 写 RGB=0, 并同步等到 led_strip_refresh() 的 RMT 发送完成
     * (3 灯珠 × 24bit @800kHz ≈ 90µs)。睡眠路径上一次性的 5ms 可忽略。 */
    vTaskDelay(pdMS_TO_TICKS(5));
    return ESP_OK;
}

void led_show_fault_code(uint8_t code)
{
    if (code < 1 || code > 7) {
        code = 1;
    }
    led_set_color((led_color_t)(LED_IDX_FAULT1 + (code - 1)));
}

esp_err_t led_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    /* ★ 不配置供电脚: 灯带 3.4V **常供、无门控**, DIN 交给下面 led_strip 的 RMT 通道。
     *   GPIO8 是 nFAULT, 由 power_state.c 管。 */

    led_indicator_config_t cfg = {
        .blink_lists = s_blink_lists,
        .blink_list_num = LED_IDX_MAX,
    };
    led_indicator_strips_config_t strips_cfg = {
        .led_strip_cfg = {
            .strip_gpio_num = PIN_WS2812_DIN,
            .max_leds = 1,
            .led_model = LED_MODEL_WS2812,
            .flags = {.invert_out = 0},
        },
        .led_strip_driver = LED_STRIP_RMT,
        .led_strip_rmt_cfg = {
            .resolution_hz = RMT_RESOLUTION_HZ,
            .flags = {.with_dma = 0},
        },
    };
    ESP_RETURN_ON_ERROR(led_indicator_new_strips_device(&cfg, &strips_cfg, &s_led),
                        TAG, "led_indicator_new_strips_device failed");

    /* 亮度在颜色值之上再缩放一次, 双重限幅防"全白拉垮 Buck" */
    led_indicator_set_brightness(s_led, CONFIG_FOCSTEP_WS2812_BRIGHTNESS);

    /* ★ 上电安全态 = 灭。灯带**常供、无门控**, 没有任何"上电即断电"的硬件保障 ⇒
     *   必须显式发一帧全黑。(V6 有"上电零闪", 上电瞬间本来也不会闪。) */
    ESP_RETURN_ON_ERROR(led_indicator_start(s_led, LED_IDX_OFF), TAG, "init off failed");

    s_inited = true;
    ESP_LOGI(TAG, "init done (led_indicator + led_strip, brightness=%d)",
             CONFIG_FOCSTEP_WS2812_BRIGHTNESS);
    return ESP_OK;
}
