#pragma once

/*
 * IPROPI 相电流采样 (S0)
 *
 * ── 定位 ──────────────────────────────────────────────────────
 * **不是 FOC 电流环**。原因见 docs/doc.md §十 与实验结论:
 *   - esp_simplefoc 的 StepperMotor / HybridStepperMotor 都把
 *     torque_controller 写死为 TorqueControlType::voltage
 *     (源码 src/StepperMotor.cpp:27, HybridStepperMotor.cpp:27)
 *   - LowsideCurrentSense 要求 MCPWM 的 driver params, 而步进驱动走 LEDC,
 *     强转后可能越界写 ⇒ 不可用
 *   - InlineCurrentSense 是三相 120° 语义, 两相步进是 90°
 * ⇒ 本模块只做 **堵转/堵门检测** (§10.4 要求), 并为将来的 S1~S4 留数据通路。
 *
 * ── 硬件事实 (DRV8874 数据手册) ───────────────────────────────
 *   IPROPI(µA) = (I_LS1 + I_LS2) × A_IPROPI          §7.3.3.1 Eq.1
 *   - **只有低边 drain→source 方向的电流计入, 反方向计 0**
 *     ⇒ IPROPI 恒 ≥ 0, **无符号**。符号须由驱动方向重建 (本模块不做)。
 *   - V_IPROPI 被内部**钳位到 V_VREF** (1.5A 档 = 2.35V)
 *     ⇒ ADC 可测上限 ≈ 2.35 / (3.48k × 450µA/A) ≈ **1.494A**, 再高读数不再上升。
 *       所以堵转只能判"到顶", 不能测幅值 —— 阈值必须 < ITRIP。
 *   - A_IPROPI: 手册正文 450、应用示例 455 µA/A **自相矛盾** ⇒ 靠 Kconfig 标定。
 *
 * ── 与驱动模式的耦合 (重要) ──────────────────────────────────
 *   本设计用 **PH/EN 模式 (PMODE=低)**, EN=0 时桥进入 Brake(低边慢衰减),
 *   此时 IPROPI **连续有效**, 无需对齐 PWM 采样窗口。
 *   (若用 PWM 模式 PMODE=高, IN1=IN2=0 是 Coast, 手册明确 "cannot be sensed"。)
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


/* 相索引: 0 = DRV8874#1 (GPIO4/ADC1_CH4), 1 = DRV8874#2 (GPIO5/ADC1_CH5) */
#define IPROPI_PHASE_COUNT 2

esp_err_t ipropi_init(void);
void ipropi_deinit(void);

/* 单相电流, 单位 mA。读失败返回负值。 */
float ipropi_read_ma(int phase);

/* 同时读两相 (FOC 循环用, 两次 oneshot 读约 40µs) */
esp_err_t ipropi_read_both_ma(float *ma1, float *ma2);

/* 原始电压, 单位 mV —— 自检命令用, 便于与万用表比对 */
esp_err_t ipropi_read_mv(int phase, int *mv);

/* ── 堵转/堵门判定 ────────────────────────────────────────────
 *
 * ★ 判据必须是**电流矢量幅值 √(ia²+ib²)**, 不能用单相瞬时值。
 *   理由: 稳态下 ia = I·cos(θe), ib = I·sin(θe), 于是 max(|ia|,|ib|) 在
 *   **0.707·I ~ I** 之间随电角度周期性摆动。若拿单相值去比阈值, 每次它掉到
 *   阈值以下就会把累计时间清零 ⇒ **持续时间条件永远凑不满 ⇒ 一次都触发不了**。
 *   幅值 √(ia²+ib²) 恒等于 I, 与电角度无关。
 *
 * ★ 采样也不能放在主循环 (100ms)。门机慢速时电频率仍有几十 Hz (电气周期 ~10ms),
 *   100ms 采一个瞬时值是严重欠采样。做法: **FOC 循环 (1kHz) 里更新峰值包络,
 *   主循环读并清零** —— 得到"过去这段时间的电流峰值", 天然与角度无关。
 *
 * 用法:
 *   FOC 循环里    ipropi_envelope_update(ma1, ma2);        // 1kHz
 *   主循环里      float peak = ipropi_envelope_take();     // ~100ms, 读并清零
 *                 ipropi_stall_update(peak);
 *
 * ⚠️ 因为 VIPROPI 被钳位到 V_VREF, 电流幅值上限 ≈1.494A, 再高读数不再上升。
 *    所以阈值的语义是"电流已达上限且持续" —— 不是精确的力矩测量,
 *    且 stall_current_ma **必须 < ITRIP**。
 */

/* 电流矢量幅值 (mA)。与电角度无关, 是堵转判定的正确输入。 */
float ipropi_magnitude_ma(float ma1, float ma2);

/* FOC 循环 (1kHz) 调用: 更新峰值包络 */
void ipropi_envelope_update(float ma1, float ma2);

/* 主循环调用: 读走"自上次调用以来的电流峰值 (mA)"并清零 */
float ipropi_envelope_take(void);

void ipropi_stall_config(float stall_current_ma, uint32_t stall_ms);

/* 输入应为幅值 (通常来自 ipropi_envelope_take)。
 * 返回 true = **当前处于堵转锁存态** —— 一旦触发就保持, 直到
 * ipropi_stall_reset() 被显式调用。这样做是为了避免"保持力矩期间电流仍在 ⇒
 * 反复触发"的抖动。 */
bool ipropi_stall_update(float peak_ma);

/* 是否处于堵转锁存态 */
bool ipropi_stall_active(void);

/* 解除锁存。应在**下达新运动指令**或电流确实降下来时调用,
 * 不要在触发处理里立刻调用。 */
void ipropi_stall_reset(void);

/* 供标定: 用当前读数反推 A_IPROPI (给定负载电流) */
float ipropi_estimate_a_ipropi(float known_load_a, float measured_mv, float r_ipropi_ohm);

#ifdef __cplusplus
}
#endif
