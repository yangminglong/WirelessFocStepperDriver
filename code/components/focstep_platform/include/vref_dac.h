#pragma once

/*
 * MCP4725 动态 VREF (DAC) 驱动封装
 *
 * doc.md §5.4 定稿规格:
 *   - U8 (MCP4725, I2C 0x60), VOUT 直连两片 DRV8874 VREF + 100k 对地兜底
 *   - VDD = 3.4V 轨, 12-bit, VREF = ITRIP × 1.566 V/A
 *   - ITRIP 档位: 0.3A→0.47V / 0.6A→0.94V / 1.0A→1.57V / 1.5A→2.35V / 1.8A→2.82V
 *   - VREF ≥3.0V 后 DRV 内部钳位 (ITRIP 封顶 1.915A), 固件 clamp ≤1.85A
 *   - 待机: DAC PD (PD1:PD0=01, 内部 100k 下拉) → VREF=0, 60nA typ / 2µA max
 *   - 唤醒顺序: 先 DAC 输出目标 VREF → 稳定 → 再抬 nSLEEP (由 power_state 编排)
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

/* 唤醒接管默认档 (doc.md §10.3: 默认 1.0A) */
#define VREF_DAC_DEFAULT_ITRIP_A 1.0f
/* DRV 内部钳位前的固件上限 (doc.md §5.4: clamp ≤1.85A) */
#define VREF_DAC_MAX_ITRIP_A     1.85f

/**
 * @brief 初始化 MCP4725 (i2cdev 描述符, 地址 0x60) 并进入 PD 初态。
 *
 * 上电默认 EEPROM=0 → VREF=0 (安全); 此处再显式进 PD 兜底。
 * @return ESP_OK / 错误码
 */
esp_err_t vref_dac_init(void);

/**
 * @brief 按 ITRIP 目标电流输出 VREF (VREF = itrip × 1.566, clamp ≤1.85A)。
 * @param itrip_a 目标斩波阈值 (A), 0 = 输出 0V
 */
esp_err_t vref_dac_set(float itrip_a);

/** @brief 输出默认档 (1.0A → 1.57V), 唤醒接管用 */
esp_err_t vref_dac_set_default(void);

/**
 * @brief 进 PD (PD1:PD0=01, 内部 100k 下拉) → VREF=0, 60nA typ。
 * 睡眠/故障态由 power_state 编排: 先 nSLEEP=0 再调本函数。
 */
esp_err_t vref_dac_pd(void);

/** @brief 档位电流 → 目标电压 (含 clamp), 供自检/日志换算 */
float vref_dac_voltage_for(float itrip_a);

#ifdef __cplusplus
}
#endif
