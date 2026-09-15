#pragma once

/*
 * WS2812 状态灯
 *
 * 硬件: **GPIO6 = DIN** (串联 300~470Ω, 可选件)。**3.4V 常供, 无供电门控**。
 *
 * 实现: **官方 espressif/led_indicator** (闪灯模式引擎) + **espressif/led_strip**
 *       (RMT 驱动)。闪灯时序是 led_indicator 的本职, 不自造。
 *
 * ★ 供电门控为什么被移除 (docs/doc.md §五.2):
 *   WS2812B-2020-V6 的**静态电流 ≦1µA**、供电 3.3~5.3V、**上电零闪** ——
 *   三颗合计 3µA@3.4V (折算 24V 输入侧 ≈0.4µA), 对深睡档 (25~65µA) 可忽略
 *   ⇒ 原 P-MOS 门控 (GPIO8) 存在的理由消失, 顺带把那个脚还给了 nFAULT。
 *   另: 灯带 VDD 恒有电 ⇒ "DIN 被驱动为高 + VDD=0" 这个**倒灌前提不存在**。
 *
 * ⚠️ 三条硬纪律:
 *   1. **进睡前必须发一帧全黑、然后停止发送**。睡眠不亮灯**已无硬件兜底**,
 *      全靠固件 (见下面 §五.2 的"睡眠不亮灯改由固件保证")。
 *      漏发的代价: 三颗全亮约 **60mA**。
 *   2. **禁全白** —— 单颗全白约 60mA, 会瞬间拉垮 Buck。颜色值本身取得很小,
 *      再叠加 Kconfig 亮度缩放, 双重限幅。
 *   3. **3.4V 轨余量只有 0.05~0.1V** (V6 的 VDD 下限 3.3V, 本板轨 3.35~3.4V),
 *      全白 108mA 的阶跃可能击穿下限 ⇒ 纪律 2 不只是省电问题。
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
    LED_IDX_OFF = 0,   /* 灭: 发一帧全黑, 灯珠锁存 (不再断电) */
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

/* 设置灯色/模式。**LED_COLOR_OFF = 发一帧全黑** (灯珠锁存全黑、IC 回到 ≦1µA),
 * 不再是断电 —— 门控已移除, 见上方纪律 1。 */
esp_err_t led_set_color(led_color_t color);

/* 故障码闪灯 (docs/doc.md §10.4): 1=过流 2=过温 3=编码器失效 4=CAN掉线 5=nFAULT
 * 6=母线过低 7=行程标定缺失/失败
 * 闪灯时序由 led_indicator 自己的任务推进, **无需周期性调用**。 */
void led_show_fault_code(uint8_t code);

#ifdef __cplusplus
}
#endif
