#pragma once

/*
 * 母线电压采样 (高边门控 + 恒流下沉栅极驱动)
 *
 * 电路 (docs/doc.md §五.1):
 *   VM → 高边 P-MOS Q2(BSS84, −60V) → 100k+8.2k 分压 → 1nF → 4.7k 串阻 → GPIO2(ADC1_CH2)
 *   栅极由**恒流下沉**驱动: GPIO7 → Q2(BC846B) + Re 33k ⇒ I = 2.6V/33k = 79µA
 *                           ⇒ Vgs = −79µA × 100k = **−7.9V, 与母线电压无关**
 *   分压比 = 8.2/(100+8.2) = **0.0758**
 *     6.0V→0.455V  8.4V→0.637V  25.2V→1.91V  30V→2.27V
 *   源阻抗 = (100k∥8.2k) + 4.7k 串阻 = **12.3kΩ**, × 1nF ⇒ τ = 12.3µs, 5τ ≈ 62µs
 *
 * ⚠️ **宽压 6.0~25.2V (2S~6S) 靠的就是恒流下沉**: 固定分压给出 Vgs ∝ VM,
 *    要同时满足"6V 时给够驱动"与"48V 时不超 ±20V"在数学上无解
 *    (需 k≥0.667 且 k≤0.417)。2S 档 Q3 工作在饱和区, 栅极落到 ≈2.7V
 *    ⇒ Vgs = −(VM−2.7), 母线越低反而**自动给足驱动**。
 *
 * ⚠️ 门控只允许做在**高边**。若开关串在 8.2k 下方, 关断时上臂电阻会把 ADC 节点
 *    抬到 VM 电位 (24V), 超引脚耐压、读数失真并有损伤风险。
 * ⚠️ 门控后采样存在盲区(= 采样周期)。**过压检测与门控可并存** —— 前提是把采样
 *    交给 LP 核提到 ~100Hz (盲区 10ms), 且**只在 LightSleep 档成立**; DeepSleep
 *    档唤醒要 100~500ms, 盲区就是 100~500ms。详见 docs/doc.md §5.1 / §五.8。
 * ⚠️ 配错极性会导致常态耗 5.3mW, 推翻深睡档待机基线 (25~65µA@24V)。
 * ⚠️ 瞬态过压时 `ADC 节点 = VM × 0.0758` 会超引脚额定 (VM ≥ 47.5V ⇒ 节点 ≥ 3.6V)。
 *    当前设计接受之 (100k 上臂把注入电流限在 ≤440µA, 且读数饱和 = 正确报过压);
 *    若要硬保, 分压比降到 0.064 (R4=120k/R5=8.2k)。见 docs/doc.md §五.1。
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


esp_err_t bus_voltage_init(void);

/* 拉门控 → µs 级忙等建立 → 采样 → 释放门控。返回 mV。
 * 等待时间由 Kconfig `FOCSTEP_VBUS_GATE_SETTLE_US` 给 (默认 70µs)。
 * ⚠️ **导通时间直接决定待机开销**: 导通期 312µA (分压 233 + 栅极 79),
 *    70µs@100Hz ⇒ **2.2µA@25.2V**; 若用 vTaskDelay 的 2ms ⇒ **62µA, 差 28 倍**。
 *    ⇒ 必须走 `esp_rom_delay_us`, 不能用 vTaskDelay (1kHz tick 最小 1ms)。 */
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
