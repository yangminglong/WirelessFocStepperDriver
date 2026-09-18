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

/* 灯带上实贴的灯珠数 (D4/D5/D6 三颗级联)。**必须与实际贴片数一致**:
 * 少配 ⇒ 后面的灯珠收不到数据、锁存上一次的颜色, 睡眠时白耗电;
 * 多配 ⇒ 多发几十 µs, 无害。 */
#define LEDS_COUNT 3

/* 颜色 (RGB 打包成 0xRRGGBB)。**数值本身就是亮度** —— 见下面 LED_ALL 的说明。 */
#define RGB_OFF    0x000000
#define RGB_VIOLET 0x180018
#define RGB_GREEN  0x001800
#define RGB_BLUE   0x000018
#define RGB_WHITE  0x101010 /* 微亮灰白, 不是"全白" */
#define RGB_DIM    0x020202
#define RGB_RED    0x180000 /* 故障闪码 */

/* ------------------------------------------------------------------
 * 表里所有颜色值都要包一层 LED_ALL():
 *   ① 带上 MAX_INDEX 索引位, 整条灯带 (D4/D5/D6) 才一起显示 —— 裸 0xRRGGBB 的
 *      索引位是 0, 只会写给第一颗。
 *   ② 亮灭一律用 RGB 步 + hold 表达, **不用 {LED_BLINK_HOLD, LED_STATE_*}, 因为**:
 *      - hold==0 的 HOLD/RGB 步不置闪灯引擎内部的 leave 标志, 配 LED_BLINK_LOOP
 *        会让回调在 while(!leave) 里空转并一直占着 mutex, 之后所有 led_* 调用
 *        都会永久阻塞;
 *      - HOLD 步会把亮度直接置成 MAX_BRIGHTNESS(255), 颜色值里那个小 V 被丢掉 ——
 *        RGB_WHITE 会真的变成全白, 违背纪律 2/3。
 *      RGB 步的 hold 非 0 时会正常让出, 且亮度就是颜色值算出来的 V。
 * ------------------------------------------------------------------ */
/* "写给整条灯带" 的索引位 = MAX_INDEX(127) << 25。
 * ⚠️ 这里不复用组件自带的 SET_IRGB: 它是在 int 上做左移, 而 127<<25 超出 int 范围,
 *    属于有符号左移溢出 (C 里的未定义行为), 只是编译器碰巧给出期望值。
 *    下面的断言把这个常量钉在 led_convert.h 的定义上, 防止两处口径漂移。 */
#define LED_IDX_ALL_BITS 0xFE000000u
_Static_assert(LED_IDX_ALL_BITS == ((uint32_t)MAX_INDEX << 25),
               "LED_IDX_ALL_BITS 与 led_convert.h 的 MAX_INDEX 不一致");

#define LED_ALL(c) (LED_IDX_ALL_BITS | (uint32_t)SET_RGB(GET_RED(c), GET_GREEN(c), GET_BLUE(c)))

/*
 * 闪灯模式表。索引即 led_indicator_start() 的 blink_type。
 * 每个模式必须以 LED_BLINK_STOP 或 LED_BLINK_LOOP 结尾。
 */
static const blink_step_t s_st_off[] = {
    {LED_BLINK_RGB, LED_ALL(RGB_OFF), 0},
    {LED_BLINK_STOP, 0, 0},
};

/* 常亮颜色 —— 应用自己选哪个颜色代表哪个状态。
 * 周期重新写一帧: 灯珠靠锁存维持末态, 定期重发可自愈丢帧。 */
static const blink_step_t s_st_dim[] = {
    {LED_BLINK_RGB, LED_ALL(RGB_DIM), 1000},
    {LED_BLINK_LOOP, 0, 0},
};
static const blink_step_t s_st_violet[] = {
    {LED_BLINK_RGB, LED_ALL(RGB_VIOLET), 1000},
    {LED_BLINK_LOOP, 0, 0},
};
static const blink_step_t s_st_green[] = {
    {LED_BLINK_RGB, LED_ALL(RGB_GREEN), 1000},
    {LED_BLINK_LOOP, 0, 0},
};
static const blink_step_t s_st_blue[] = {
    {LED_BLINK_RGB, LED_ALL(RGB_BLUE), 1000},
    {LED_BLINK_LOOP, 0, 0},
};
static const blink_step_t s_st_white[] = {
    {LED_BLINK_RGB, LED_ALL(RGB_WHITE), 1000},
    {LED_BLINK_LOOP, 0, 0},
};

/* 故障闪码: 亮 120ms / 灭 200ms 重复 N 次, 然后长灭 1200ms 再循环。
 * docs/doc.md §10.4 的故障码 1~7 各一条 (N = 故障码)。
 * 每条的重复次数不同, 无法用同一个宏参数化, 故显式写 7 条。 */
static const blink_step_t s_st_fault1[] = {
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120},
    {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_OFF), 1200},
    {LED_BLINK_LOOP, 0, 0},
};
static const blink_step_t s_st_fault2[] = {
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_OFF), 1200},
    {LED_BLINK_LOOP, 0, 0},
};
static const blink_step_t s_st_fault3[] = {
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_OFF), 1200},
    {LED_BLINK_LOOP, 0, 0},
};
static const blink_step_t s_st_fault4[] = {
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_OFF), 1200},
    {LED_BLINK_LOOP, 0, 0},
};
static const blink_step_t s_st_fault5[] = {
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_OFF), 1200},
    {LED_BLINK_LOOP, 0, 0},
};
static const blink_step_t s_st_fault6[] = {
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_OFF), 1200},
    {LED_BLINK_LOOP, 0, 0},
};
static const blink_step_t s_st_fault7[] = {
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_RED), 120}, {LED_BLINK_RGB, LED_ALL(RGB_OFF), 200},
    {LED_BLINK_RGB, LED_ALL(RGB_OFF), 1200},
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
static int s_active_idx = LED_IDX_OFF; /* 最近一次 start 的模式, 切换前要 stop 掉 */

/* 灯带 3.4V **常供、无供电门控** ⇒ 本模块不提供 led_power() / led_power_is_on():
 *   开关灯只靠 led_set_color() 写颜色。理由见 led_ws2812.h 头注释。 */

esp_err_t led_set_color(led_color_t color)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    int idx = (int)color;
    /* 枚举封闭, 只有强转能越界。这里必须挡住: led_indicator 内部直接索引
     * p_blink_steps[blink_type], 越界是写坏内存而不是显示错色。 */
    if (idx < 0 || idx >= LED_IDX_MAX) {
        idx = LED_IDX_OFF;
    }

    /* ★ 切换前必须 stop 掉上一个模式: 模式表以 LED_BLINK_LOOP 收尾时,
     *   p_blink_steps[idx] 永远不会回到 LED_BLINK_STOP, 而引擎的
     *   "找第一个未完成模式" 扫描会一直把它选回来 —— 不 stop 就切不干净,
     *   熄灯后旧颜色会在同一个回调里被重新锁存。 */
    led_indicator_stop(s_led, s_active_idx);
    s_active_idx = idx;

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
    /* 全黑帧必须**同步**送出去再返回, 不能靠"睡一会儿"等引擎的定时器:
     *   led_indicator_set_on_off() 取 mutex 时会与 Tmr Svc 里的闪灯回调串行,
     *   内部再走到 led_strip_refresh() → rmt_tx_wait_all_done(chan, -1),
     *   返回时帧已物理移出。若改用 vTaskDelay, 一旦此刻回调正持锁, 本帧会被
     *   延后 50ms 才发, 固定延时救不了。 */
    return led_indicator_set_on_off(s_led, false);
}

esp_err_t led_show_fault_code(uint8_t code)
{
    if (code < 1 || code > 7) {
        code = 1;
    }
    return led_set_color((led_color_t)(LED_IDX_FAULT1 + (code - 1)));
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
            .max_leds = LEDS_COUNT,
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

    /* ★ 上电安全态 = 灭。灯带**常供、无门控**, 没有任何"上电即断电"的硬件保障 ⇒
     *   必须显式发一帧全黑。(V6 有"上电零闪", 上电瞬间本来也不会闪。) */
    ESP_RETURN_ON_ERROR(led_indicator_start(s_led, LED_IDX_OFF), TAG, "init off failed");

    s_inited = true;
    ESP_LOGI(TAG, "init done (led_indicator + led_strip, %d leds)", LEDS_COUNT);
    return ESP_OK;
}
