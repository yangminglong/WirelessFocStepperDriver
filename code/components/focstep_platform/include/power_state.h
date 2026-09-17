#pragma once

/*
 * 电源状态机 (平台层, 三态) —— nSLEEP 纪律的唯一落点
 *
 * |状态  | nSLEEP | 编码器          |电机   |
 * |------|--------|-----------------|-------|
 * |SLEEP | 低     | Wake-up&Sleep档 |断电   |
 * |ACTIVE| 高     | 连续测量        |可用   |
 * |FAULT | 低     | 保持            |断电   |
 *
 * ★★ 为什么 `nSLEEP` 是本模块的**唯一职责核心** —— 这一根脚上挂着 **DRV 断电与
 *
 *  1. **VREF 由 MCP4725 (DAC) 动态驱动** (docs/doc.md §五.4)。DRV8874 的 VREF 是
 *     12-bit DAC 输出 (I2C 0x60), 待机进 **PD (PD1:PD0=01, 内部 100k 下拉)**: VREF=0,
 *     60nA typ / 2µA max (等效 ≈0)。**⇒ 睡眠/故障必须先 nSLEEP=0 再发 DAC PD;
 *     唤醒先 DAC 输出目标 VREF → 稳定 → 再抬 nSLEEP (§5.4 顺序不可颠倒)。**
 *     ⚠️ 若睡眠态漏掉 DAC PD, 正常模式 210µA 常挂 3.4V 轨, 待机预算当场破。
 *  2. **CAN 收发器的 Rs 挂在反相 N-MOS (Q3=DMN3150L) 的漏极上** (docs/doc.md §5.3) ⇒
 *     nSLEEP 高 = Q3 导通 = Rs 低 = **CAN 唤醒**;  nSLEEP 低 = Q3 截止 = Rs 高 = **CAN 睡眠**。
 *     **⇒ CAN 的醒睡由这里间接决定, 软件无法独立控制** —— 这正是本模块 (以及
 *       整个固件) **没有 can_set_active()** 的原因。上电默认 nSLEEP=低 ⇒ CAN 默认睡眠。
 *  3. **nFAULT 必须立即拉低 nSLEEP** (§10.4 ③)。
 *
 * ⚠️ 因此: **本文件是 GPIO18 的唯一写入方**。任何其他模块都不得直接操作 nSLEEP。
 *
 * ── 为什么不设钩子表 ──────────────────────────────────────────
 * 由应用注入的函数指针表 (`power_state_hooks_t` 一类) 在这里有两个问题:
 *   ① 单订阅、字段定形 ⇒ 换个应用形态就得改结构体;
 *   ② 平台内部模块之间也要绕道钩子, 平白多一层。
 *
 * ⇒ **平台内部直接互相调用** (`foc_motor_enable()` / `bus_voltage_read_mv()` /
 *   `gpio_get_level()`), 应用则订阅事件。
 *
 * ── 应用怎么接 ────────────────────────────────────────────────
 *   · 订阅 `FOCSTEP_EVT_STATE_CHANGED` 决定灯色与电机行为
 *   · 深睡策略(何时睡)、手拉助动、开关停语义 都在应用层
 *   · `§10.3` 的四态是**应用概念**: 唤醒接管/运行 是 ACTIVE 的两种子模式,
 *     差别在"电机做什么", 由应用控制
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PS_SLEEP = 0, /* nSLEEP 低, 电机断电, DAC PD */
    PS_ACTIVE,    /* nSLEEP 高, 电机可用, 编码器连续测量 */
    PS_FAULT,     /* 硬件故障: 电机断电, 等释放后恢复 */
} power_state_t;

/* 故障码 (§10.4) */
#define PS_FAULT_OVERCURRENT 1 /* 过流 */
#define PS_FAULT_OVERTEMP    2 /* 过温 */
#define PS_FAULT_ENCODER     3 /* 编码器失效 */
#define PS_FAULT_CAN_LOST    4 /* CAN 掉线 */
#define PS_FAULT_NFAULT      5 /* DRV8874 nFAULT 拉低 */
#define PS_FAULT_VBUS_LOW    6 /* 母线过低 */
#define PS_FAULT_CALIB       7 /* 行程标定缺失/失败 (零点或满行程点没标出来) */

esp_err_t power_state_init(void);

/* 周期性调用 (100ms 即可)。内部会做 §10.4 的硬件级安全检查。 */
void power_state_tick(void);

/* 请求进入某一状态。应用调用。
 * ⚠️ 从 FAULT 恢复必须先 power_state_clear_fault()。 */
void power_state_request(power_state_t want);

/* 清除故障锁存 (nFAULT 已释放才会真正恢复) */
void power_state_clear_fault(void);
void power_state_on_fault(uint8_t code);

/* ★ 没有 power_state_set_can_active() —— CAN 的醒睡由 nSLEEP 硬件派生,
 *   软件无法独立控制 (见文件头第 2 条, 以及 docs/doc.md §5.3)。 */

power_state_t power_state_current(void);
const char *power_state_name(power_state_t s);
uint8_t power_state_fault_code(void);

typedef struct {
    power_state_t state;
    uint8_t fault_code;
    uint32_t transitions;
} power_state_snapshot_t;

void power_state_snapshot(power_state_snapshot_t *out);

#ifdef __cplusplus
}
#endif
