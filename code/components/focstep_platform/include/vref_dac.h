#pragma once

/*
 * MCP4725 动态 VREF (DAC) 驱动封装
 *
 * doc.md §5.4 定稿规格:
 *   - U8 (MCP4725, I2C 0x60), VOUT 经 R13(470Ω) 串到两片 DRV8874 VREF (两片各挂 1nF 对地)
 *   - VDD = 3.4V 轨经 L4 磁珠, 12-bit, VREF = ITRIP × (R_IPROPI × A_IPROPI)
 *   - ITRIP 档位: 0.3A→0.47V / 0.6A→0.94V / 1.0A→1.57V / 1.5A→2.35V / 1.8A→2.82V
 *   - 固件上限 1.85A (VREF 2.90V) —— **自设余量, 不是器件钳位**: DRV8874 的 VREF
 *     推荐范围 0~3.6V, 手册未规定内部钳位; 硬件边界是 DAC 满量程 (≈VDD 3.35V)
 *   - 待机: DAC PD (PD1:PD0=10, 内部 100k 下拉) → VREF=0, 60nA typ / 2µA max
 *   - **上电输出 = 出厂 EEPROM 值 = 满量程中点 (VOUT ≈ VDD/2), 不是 0V** ——
 *     上电期间靠 nSLEEP 的 10k 下拉让 DRV 保持睡眠, 再由 vref_dac_init() 写 PD 收口
 *   - 唤醒顺序: 先 DAC 输出目标 VREF → 稳定 → 再抬 nSLEEP (由 power_state 编排)
 *
 * ★ 档位只有一个真源: Kconfig 的 FOCSTEP_ITRIP_MA (唤醒默认档) 与
 *   FOCSTEP_R_IPROPI_OHM / FOCSTEP_A_IPROPI_UA_PER_A (V/A 换算)。
 *   本模块不另存一份常量 —— 标定 R_IPROPI/A_IPROPI 后 VREF↔ITRIP 自动跟着走。
 *
 * ★ **档位一变, IPROPI 的可测上限跟着变** (引脚被 DRV 钳在 V_VREF 上) ⇒ 本模块
 *   写完 DAC 会把 ADC 量程一并跟过去 (ipropi_set_ceiling_mv), 上层不必自己同步。
 *
 * 总线: 与 KTH5701 共用 i2cdev 管理的同一条 I2C 总线 (GPIO14/15),
 * 首个设备初始化时建 i2c_master bus, 本模块复用 (见 kth5701.cpp 说明)。
 * 速率取 Kconfig 的 FOCSTEP_DAC_I2C_HZ (默认 400k, 与编码器同档) ——
 * 不直接用 mcp4725_init_desc 就是因为它把速率硬编码成 1MHz, 而本总线只有 4.7k 上拉。
 *
 * ⚠️ GPIO18 (nSLEEP) 与 PD 命令的编排不在此模块 —— power_state.c 是唯一编排方
 * (§5.4 纪律)。**改档位不属于该纪律** (不碰 nSLEEP), 允许的调用方:
 *   · power_state.c 的唤醒路径 (vref_dac_set_default)
 *   · foc_motor_set_itrip() —— 上层应用改档的正式入口 (带运行态守卫)
 *   · 自检命令 (console do_vref)
 */

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 固件自设的档位上限 (doc.md §5.4: clamp ≤1.85A, 即 VREF 2.90V)。
 * ⚠️ 这是**自设余量而非器件钳位** —— 见文件头对 DRV8874 VREF 范围的说明。 */
#define VREF_DAC_MAX_ITRIP_A     1.85f

/* 档位下限: 再低 IPROPI 就整体落进 ADC 的差区, 且 DRV 比较器失调占比变大。 */
#define VREF_DAC_MIN_ITRIP_A     0.1f

/* 按"我要用的峰值相电流"给档时的钳位裕量: ITRIP = 峰值 × 本值 (doc.md §5.4)。
 * 留裕量的两个理由: ① 电流环本身有超调; ② IPROPI 的钳位点就是 ITRIP ⇒
 * 读数一旦顶到钳位就再也反映不了真实电流, 峰值必须留在钳位之下。 */
#define VREF_DAC_CLAMP_MARGIN    1.25f

/**
 * @brief 初始化 MCP4725 (i2cdev 描述符, 地址 0x60) 并进入 PD 初态。
 *
 * ⚠️ 必须在 foc_motor_init() 之后调用 (I2C 总线由编码器那边建起来),
 * 且在 power_state_init() 之前 (后者假定上电时 DAC 已在 PD)。
 * @return ESP_OK / 错误码
 */
esp_err_t vref_dac_init(void);

/**
 * @brief 按 ITRIP 目标电流输出 VREF (VREF = itrip × (R_IPROPI×A_IPROPI), clamp ≤1.85A)。
 *
 * 写完 DAC 后**自动把 ADC 量程跟到新 VREF** (IPROPI 的钳位点就是 VREF,
 * 量程收窄才有低电流档的分辨率) —— 调用方不必自己调 ipropi_set_ceiling_mv()。
 * @param itrip_a 目标斩波阈值 (A), 0 = 输出 0V
 */
esp_err_t vref_dac_set(float itrip_a);

/**
 * @brief 纯换算: "应用要用的峰值电流" → 实际生效档位 (含 1.25× 裕量与上下界)。不碰硬件。
 *
 * 与 vref_dac_voltage_for() 的区别: 那个是"档位 → 电压", 这个是"峰值需求 → 档位"。
 * @param peak_a 峰值相电流 (A); ≤0 返回 0
 * @return 会被写进 DAC 的 ITRIP (A); 超出可支持范围时返回 clamp 后的值 ——
 *         要严格拒绝请先用 vref_dac_peak_supported()
 */
float vref_dac_itrip_for_peak(float peak_a);

/** @brief peak_a 留出 VREF_DAC_CLAMP_MARGIN 后是否仍在 1.85A 上限内 */
bool vref_dac_peak_supported(float peak_a);

/**
 * @brief 按峰值电流设定档位, 并让 ADC 量程跟随。
 *
 * ⚠️ 与 vref_dac_set() 的区别: 本函数**拒绝**留不出裕量的入参 (ESP_ERR_INVALID_ARG,
 *    且不碰 DAC), 而不是静默 clamp —— 静默 clamp 会让调用方以为"档位覆盖了我报的
 *    电流", 实际桥给不出那个电流: IPROPI 恒读钳位值, 堵转与力矩判据一起失灵。
 */
esp_err_t vref_dac_set_for_peak(float peak_a);

/** @brief Kconfig 的唤醒默认档 (FOCSTEP_ITRIP_MA), 单位 A */
float vref_dac_default_itrip_a(void);

/** @brief 输出唤醒默认档 (Kconfig 的 FOCSTEP_ITRIP_MA), 唤醒接管用 */
esp_err_t vref_dac_set_default(void);

/**
 * @brief 进 PD (PD1:PD0=10, 内部 100k 下拉) → VREF=0, 60nA typ。
 * 睡眠/故障态由 power_state 编排: 先 nSLEEP=0 再调本函数。
 */
esp_err_t vref_dac_pd(void);

/** @brief 档位电流 → 目标电压 (含 clamp), 供自检/日志换算 */
float vref_dac_voltage_for(float itrip_a);

/**
 * @brief 当前生效的档位电流 (A); 从未写过或已进 PD 时为 0。
 *
 * 它同时就是 IPROPI 的**可测上限**: IPROPI 引脚电压 = I × (R_IPROPI×A_IPROPI),
 * 被 DRV 内部钳位在 V_VREF 上, 所以能测到的最大电流恰好等于 ITRIP。
 * 自检提示不要再自己换算一遍 (那样会与 Kconfig 的标定值脱节)。
 */
float vref_dac_itrip_a(void);

#ifdef __cplusplus
}
#endif
