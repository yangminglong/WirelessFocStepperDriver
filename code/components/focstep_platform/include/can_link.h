#pragma once

/*
 * CAN 总线 (TWAI)
 *
 * 硬件: **GPIO16 = TXD, GPIO17 = RXD** —— 与 **4P 调试排针共用同一对网络**
 *   (docs/doc.md §5.3)。同一时刻只能有一个外设驱动它们 ⇒ **本模块只能在"用 CAN 档"
 *   启用**, 该档的控制台走 USB-Serial-JTAG (GPIO12/13)。app_main.c 用
 *   `CONFIG_FOCSTEP_CAN_ENABLE` 把这条开关钉在构建期。
 *   ⚠️ U6 的 RXD 与 GPIO17 之间串有 **1.5kΩ** —— 那是为了让 USB-UART 适配器在
 *      收发器在线时仍能拉低 GPIO17。**属硬件职责, 软件不用管**。
 *
 * ★ **Rs 没有 GPIO**: 它挂在反相 N-MOS (Q3=DMN3150L) 的漏极上, **随 nSLEEP 硬件派生** ——
 *     nSLEEP(GPIO18) 高 → Q3 导通 → Rs 低 = CAN 唤醒;  nSLEEP(GPIO18) 低 → Q3 截止 → Rs 高 = CAN 睡眠。
 *   ⇒ 上电默认 nSLEEP=低 ⇒ **CAN 默认睡眠**(硬件保证); 且**软件无法独立控制 CAN 醒睡**。
 *   ⇒ 本模块**不提供** can_set_active() 一类接口 —— CAN 醒睡完全由硬件派生, 软件无从插手。
 *
 * ⚠️ docs/doc.md 尚未定 CAN 波特率 —— 默认 500k 是占位项, 见 §9.1 #3。
 * ⚠️ 菊花链拓扑: 只在总线两端节点保留终端电阻, 中间节点拆除 (板载为分裂终端 2×60Ω)。
 *
 * §10.4 安全逻辑⑤: CAN 丢帧/总线关闭 → 上报故障码 4, **不影响本地控制**
 * (手拉助动仍然工作)。
 * ⚠️ **用 4P 调试口时 CAN 总线必须拔掉** —— 此时总线上无其他节点, TWAI 必然报错,
 *    那是**预期行为、不是故障**, 应关闭丢帧上报 (见 docs/doc.md §10.4 ⑤)。
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


/* 本地控制指令 (供上层状态机消费) */
typedef enum {
    CAN_CMD_NONE = 0,
    CAN_CMD_OPEN,
    CAN_CMD_CLOSE,
    CAN_CMD_STOP,
} can_cmd_t;

/* 安装 TWAI 驱动。⚠️ **只能在"用 CAN 档"调用** —— 它会占用 GPIO16/17 顶掉 UART0 控制台。 */
esp_err_t can_init(void);

/* 周期性调用 (建议 10~20ms): 收帧、超时检测。
 * 收到指令时**发 `FOCSTEP_EVT_CAN_CMD` 事件**, 不直接执行 —— 响应策略在应用层。
 * 超时则发 `FOCSTEP_EVT_CAN_LOST` (边沿触发, 只报一次)。 */
void can_tick(void);

/* §10.4 ④: 超过 timeout_ms 未收到任何帧 ⇒ true */
bool can_link_lost(void);

/* 是否检测到总线关闭 (bus-off) */
bool can_bus_off(void);

uint32_t can_rx_count(void);

#ifdef __cplusplus
}
#endif
