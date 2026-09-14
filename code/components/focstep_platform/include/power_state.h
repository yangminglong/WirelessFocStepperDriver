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
 * ★★ 为什么 `nSLEEP` 是本模块的**唯一职责核心**:
 *
 *  1. **VREF 门控由 nSLEEP 硬件驱动** (docs/doc.md §五.4)。DRV8874 的 VREF 由
 *     10k+22k 从 3.4V 分压产生, 该分压原方案无门控、常态耗 **106µA ≈ 0.42mW@24V**,
 *     是整机待机预算 (0.25mW) 的 **1.7 倍**。改法是给分压加高边 P-MOS, 栅极由
 *     GPIO16 (nSLEEP) 经 NPN 驱动。**⇒ 固件侧零改动, 但 SLEEP/FAULT 态必须真正
 *     拉低 nSLEEP, 否则 P-MOS 不关断, 那 106µA 照烧, 待机预算当场破。**
 *  2. **nFAULT 必须立即拉低 nSLEEP** (§10.4 ③)。
 *
 * ⚠️ 因此: **本文件是 GPIO16 的唯一写入方**。任何其他模块都不得直接操作 nSLEEP。
 *
 * ── 与重构前的变化 ────────────────────────────────────────────
 * 原来这里有一张 `power_state_hooks_t` 函数指针表, 由应用注入。
 * 它有两个问题: ① 单订阅、字段定形 ⇒ 换个应用形态就得改结构体;
 * ② 平台内部模块之间也要绕道钩子, 平白多一层。
 *
 * 现在: **平台内部直接互相调用** (`foc_motor_enable()` / `bus_voltage_read_mv()` /
 * `can_set_active()` / `gpio_get_level()`), 应用则订阅事件。
 *
 * ── 应用怎么接 ────────────────────────────────────────────────
 *   · 订阅 `FOCSTEP_EVT_STATE_CHANGED` 决定灯色、CAN Rs、电机行为
 *   · 深睡策略(何时睡)、手拉助动、开关停语义 都在应用层
 *   · `§10.3` 的四态是**应用概念**: 唤醒接管/运行 是 ACTIVE 的两种子模式,
 *     差别在"电机做什么 + CAN Rs", 两者都由应用控制
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PS_SLEEP = 0, /* nSLEEP 低, 电机断电, VREF 门控关断 */
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

/* CAN 收发器使能 (Rs)。§10.3: 唤醒接管=睡眠, 运行=工作。
 * 是**应用**决定何时切, 平台只提供接口。 */
void power_state_set_can_active(bool active);

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
