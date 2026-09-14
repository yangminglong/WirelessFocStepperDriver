#pragma once

/*
 * WS2812 状态灯
 *
 * 硬件: GPIO8 = 供电 P-MOS 栅极 (上电=高即灯灭), GPIO21 = DIN (100Ω 串联)
 *
 * 实现: **官方 espressif/led_indicator** (闪灯模式引擎) + **espressif/led_strip**
 *       (RMT 驱动)。闪灯时序是 led_indicator 的本职, 不自造。
 *       本模块只补它管不到的两件事:
 *        ① **供电门控** (GPIO8) —— led_indicator 不知道这颗灯是可以断电的
 *        ② **断电前先拉低 DIN** —— 否则 WS2812 会经数据线寄生取电
 *
 * ⚠️ 两条硬纪律:
 *   1. **断电必须先拉低 DIN**。否则灯会微亮/乱闪, 且这部分电流不进 VDD 回路,
 *      待机电流对不上账。
 *   2. **禁全白** —— 单颗全白约 60mA, 会瞬间拉垮 Buck。颜色值本身取得很小,
 *      再叠加 Kconfig 亮度缩放, 双重限幅。
 *
 * GPIO8 是 strapping 脚: 上电锁存电平由 100k 上拉决定 (=1, 满足 SPI boot),
 * **严禁加下拉电阻**。本模块只在系统起来之后才操作它。
 *
 * 模式表见 led_ws2812.c; 状态映射见 docs/doc.md §10.3:
 *   深睡=灭 / 唤醒接管=紫 / 运行=绿 / 故障=红闪码
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 闪灯模式索引 —— 同时也是 led_indicator 的 blink_type。
 *
 * ★ 平台层只提供**颜色**与**故障闪码**, 不规定"什么颜色代表什么状态" ——
 *   那是应用语义。应用自己把状态映射到颜色 (见 app_door.c)。
 *   (重构前这里叫 LED_IDX_WAKE_ASSIST / LED_IDX_RUNNING, 把门机的四态
 *    写进了驱动层。)
 *
 * 故障闪码占 6 个索引 (码值 1~6, 与 power_state.h 的 PS_FAULT_* 对应)。
 */
typedef enum {
    LED_IDX_OFF = 0,   /* 灭 (并断电) */
    LED_IDX_DIM,       /* 极暗常亮 —— 比灭更省事, 用于"待机但要看得到" */
    LED_IDX_VIOLET,    /* 紫 */
    LED_IDX_GREEN,     /* 绿 */
    LED_IDX_BLUE,      /* 蓝 */
    LED_IDX_WHITE,     /* 白 */
    LED_IDX_FAULT1,    /* 红闪 1 次 */
    LED_IDX_FAULT2,
    LED_IDX_FAULT3,
    LED_IDX_FAULT4,
    LED_IDX_FAULT5,
    LED_IDX_FAULT6,
    LED_IDX_FAULT7,    /* 闪 7 次 = PS_FAULT_CALIB (行程标定缺失/失败) */
    LED_IDX_MAX,
} led_idx_t;

/* 语义别名 —— 只是让调用点更好读, 不含状态含义 */
typedef enum {
    LED_COLOR_OFF = LED_IDX_OFF,
    LED_COLOR_DIM = LED_IDX_DIM,
    LED_COLOR_VIOLET = LED_IDX_VIOLET,
    LED_COLOR_GREEN = LED_IDX_GREEN,
    LED_COLOR_BLUE = LED_IDX_BLUE,
    LED_COLOR_WHITE = LED_IDX_WHITE,
    LED_COLOR_FAULT = LED_IDX_FAULT1, /* 起始索引; 具体码用 led_show_fault_code() */
} led_color_t;

esp_err_t led_init(void);

/* 供电门控。关断前内部会先把 DIN 拉低 (纪律 1)。
 * 重新上电时会自动重放最近一次请求的模式。 */
esp_err_t led_power(bool on);

/* 设置灯色/模式 (需要供电时会自动开) */
esp_err_t led_set_color(led_color_t color);

/* 故障码闪灯 (docs/doc.md §10.4): 1=过流 2=过温 3=编码器失效 4=CAN掉线 5=nFAULT 6=母线过低
 * 闪灯时序由 led_indicator 自己的任务推进, **无需周期性调用**。 */
void led_show_fault_code(uint8_t code);

bool led_power_is_on(void);

#ifdef __cplusplus
}
#endif
