#pragma once

/*
 * MCP4725 动态 VREF (DAC) 驱动封装
 *
 * doc.md §5.4 定稿规格:
 *   - U8 (MCP4725, I2C 0x60), VOUT 直连两片 DRV8874 VREF + 100k 对地兜底
 *   - VDD = 3.4V 轨, 12-bit, VREF = ITRIP × (R_IPROPI × A_IPROPI)
 *   - ITRIP 档位: 0.3A→0.47V / 0.6A→0.94V / 1.0A→1.57V / 1.5A→2.35V / 1.8A→2.82V
 *   - VREF ≥3.0V 后 DRV 内部钳位 (ITRIP 封顶 1.915A), 固件 clamp ≤1.85A
 *   - 待机: DAC PD (PD1:PD0=10, 内部 100k 下拉) → VREF=0, 60nA typ / 2µA max
 *   - 唤醒顺序: 先 DAC 输出目标 VREF → 稳定 → 再抬 nSLEEP (由 power_state 编排)
 *
 * ★ 档位只有一个真源: Kconfig 的 FOCSTEP_ITRIP_MA (唤醒默认档) 与
 *   FOCSTEP_R_IPROPI_OHM / FOCSTEP_A_IPROPI_UA_PER_A (V/A 换算)。
 *   本模块不另存一份常量 —— 标定 R_IPROPI/A_IPROPI 后 VREF↔ITRIP 自动跟着走。
 *
 * 总线: 与 KTH5701 共用 i2cdev 管理的同一条 I2C 总线 (GPIO14/15),
 * 首个设备初始化时建 i2c_master bus, 本模块复用 (见 kth5701.cpp 说明)。
 *
 * ⚠️ GPIO18 (nSLEEP) 的编排不在此模块 —— power_state.c 是唯一编排方
 * (§5.4 纪律), 自检命令 (console do_vref) 除外。
 */

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* DRV 内部钳位前的固件上限 (doc.md §5.4: clamp ≤1.85A) */
#define VREF_DAC_MAX_ITRIP_A     1.85f

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
 * @param itrip_a 目标斩波阈值 (A), 0 = 输出 0V
 */
esp_err_t vref_dac_set(float itrip_a);

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
