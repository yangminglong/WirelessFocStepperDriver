#pragma once

/*
 * CAN 总线 (TWAI)
 *
 * 硬件: GPIO18=TXD, GPIO20=RXD, GPIO19=Rs (10k 上拉, 上电默认 CAN 睡眠)
 *   Rs 高 = 只监听/睡眠, Rs 低 = 正常工作 —— 与 §10.3 状态表一致:
 *   深睡/唤醒接管 = Rs 高, 运行 = Rs 低。
 *
 * ⚠️ docs/doc.md **全文未提及 CAN 波特率** —— 默认 500k 是本次新增的占位项,
 *    定稿前需确认 (见 code/driver/main/Kconfig.projbuild)。
 * ⚠️ 菊花链拓扑: 只在总线两端节点保留 120Ω 终端电阻, 中间节点拆除。
 *
 * §10.4 安全逻辑④: CAN 丢帧/总线关闭 → 上报故障码 4, **不影响本地控制**
 * (手拉助动仍然工作)。
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

esp_err_t can_init(void);

/* Rs 收发器使能: true = 正常工作 (Rs 低), false = 睡眠 (Rs 高) */
esp_err_t can_set_active(bool active);

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
