#pragma once

/*
 * 板载按键 —— **只发事件, 不做任何决策**
 *
 * 硬件: GPIO9 —— BOOT 按键直连到 GND, 10k 上拉, LED 也挂在这一脚上。
 *   ⚠️ doc.md §四: "满足SPI boot=1，BOOT按键直连GPIO9到GND，**严禁加消抖电容**"
 *      ⇒ 消抖只能做在**固件**里。这正是用官方 espressif/button 组件的原因。
 *   ⚠️ GPIO9 是 strapping 脚, 但只在复位释放瞬间锁存 —— 运行期当普通输入用没问题。
 *      副作用: 开机时按住会进下载模式, 这是该引脚拓扑的固有行为。
 *
 * ★ 重构要点: 本模块**不再调用 `power_state_*()`**。
 *   重构前它直接调, 于是"按键"(驱动层) 知道了"门的四态"(应用层) —— 层次倒置,
 *   换项目就得改驱动。现在只发 `FOCSTEP_EVT_BUTTON_*` 事件, 应用自己订阅决定。
 *
 * 事件:
 *   FOCSTEP_EVT_BUTTON_SINGLE / _DOUBLE / _LONG
 *   见 platform_events.h 的 focstep_evt_button_t
 */

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t platform_button_init(void);

#ifdef __cplusplus
}
#endif
