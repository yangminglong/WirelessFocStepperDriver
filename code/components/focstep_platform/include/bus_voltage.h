#pragma once

/*
 * 母线电压采样 (高边门控)
 *
 * 电路 (docs/doc.md §五.1): VM → 高边 P-MOS(GPIO7 经 NPN) → 100k+8.2k 分压 → 1nF → GPIO2(ADC1_CH2)
 *   分压比 = 8.2/(100+8.2) = 0.0758  ⇒ 24V→1.82V  30V→2.27V  36V→2.73V
 *   源阻抗 = 100k∥8.2k = 7.6kΩ,  × 1nF ⇒ τ = 7.6µs
 *
 * ⚠️ 门控只允许做在**高边**。若开关串在 8.2k 下方, 关断时上臂电阻会把 ADC 节点
 *    抬到 VM 电位 (24V), 超引脚耐压、读数失真并有损伤风险。
 * ⚠️ 门控后采样存在盲区(= 采样周期)。**过压检测与门控可并存** —— 前提是把采样
 *    交给 LP 核提到 ~100Hz (盲区 10ms), 且**只在 LightSleep 档成立**; DeepSleep
 *    档唤醒要 100~500ms, 盲区就是 100~500ms。详见 docs/doc.md §5.1 / §五.8。
 * ⚠️ 配错极性会导致常态耗 5.3mW, 推翻深睡档待机基线 (25~65µA@24V)。
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


esp_err_t bus_voltage_init(void);

/* 拉门控 → 等建立 → 采样 → 释放门控。返回 mV。
 * settle_ms 由 Kconfig 给 (默认 2ms, 远大于 5τ≈38µs)。
 * ⚠️ **导通时间直接决定待机开销**: 2ms@100Hz = 48µA@24V, 5τ@100Hz = 0.84µA@24V。
 *    doc.md §六 的预算按后者算, 前者需改成 µs 级忙等。 */
esp_err_t bus_voltage_read_mv(int *mv);

/* 仅操作门控 (自检用: 关断后应能在外部分压节点量到 0V) */
esp_err_t bus_voltage_gate(bool on);

/* 安全逻辑①: VM < 阈值时禁止使能电机 */
bool bus_voltage_allow_motor_enable(int mv);

/* 缓存的上次读数 (避免频繁门控) */
int bus_voltage_last_mv(void);

#ifdef __cplusplus
}
#endif
