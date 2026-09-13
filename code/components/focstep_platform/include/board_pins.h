#pragma once

/*
 * FocStepper 驱动板 GPIO 分配 —— **唯一真源**
 *
 * 抄自 docs/doc.md §四「GPIO引脚分配表（ESP32-C6）」v0.2 重排版。
 * 任何引脚改动只改这里; 其余文件一律引用本头文件, 不得硬编码 GPIO 号。
 *
 * ⚠️ 占用 24 脚, **LP 域（GPIO0~7）已用满, 无空闲 GPIO**。
 *    新增功能只能复用既有信号（例: VREF 门控复用 nSLEEP, 见 docs/doc.md §五.4）。
 */

/* ---- LP 域 (GPIO0~7): 深睡期间仍供电, 支持 ext1 唤醒 ---- */
#define PIN_XTAL32K_P        0  /* 32.768kHz 晶振, 不可再作他用 */
#define PIN_XTAL32K_N        1  /* 32.768kHz 晶振, 不可再作他用 */
#define PIN_VBUS_ADC         2  /* 母线分压 (经高边门控), ADC1_CH2 */
#define PIN_ENCODER_INT      3  /* KTH5701 INT: 高有效、锁存、读数据清零 */
#define PIN_IPROPI_1         4  /* DRV8874#1 电流反馈, ADC1_CH4 */
#define PIN_IPROPI_2         5  /* DRV8874#2 电流反馈, ADC1_CH5 */
#define PIN_DRV_PMODE        6  /* 两片共用: 低=PH/EN、高=PWM、Hi-Z=独立半桥 */
#define PIN_VBUS_GATE        7  /* 母线分压高边 P-MOS 栅极 (经 NPN 电平转换) */

/* ---- HP 域 (GPIO8~23) ---- */
#define PIN_WS2812_PWR       8  /* WS2812 供电 P-MOS 栅极, 上电=高(灯灭) */
#define PIN_STATUS_LED       9  /* 低有效 + 10k 上拉; BOOT 按键直连此脚 */
#define PIN_DRV1_IN2        10  /* DRV8874#1 EN */
#define PIN_DRV1_IN1        11  /* DRV8874#1 PH */
/* GPIO12/13 = USB D-/D+ */
#define PIN_DRV2_IN2        14  /* DRV8874#2 EN */
#define PIN_DRV2_IN1        15  /* DRV8874#2 IN1 (strapping 脚) */
#define PIN_DRV_nSLEEP      16  /* 两片共用, 10k 下拉, 上电默认电机断电 */
#define PIN_DRV_nFAULT      17  /* 两片线与 (10k 上拉) —— 无法区分是哪一片 */
#define PIN_CAN_TXD         18
#define PIN_CAN_RS          19  /* 10k 上拉, 上电 CAN 默认睡眠 */
#define PIN_CAN_RXD         20
#define PIN_WS2812_DIN      21  /* 100Ω 串联, RMT 驱动 */
#define PIN_I2C_SDA         22
#define PIN_I2C_SCL         23

/*
 * ★ VREF 门控说明 (docs/doc.md §五.4)
 *
 * DRV8874 的 VREF 由 10k+22k 从 3.4V 轨分压产生, 决定两件事:
 *   ① 斩波阈值 ITRIP = V_VREF / (R_IPROPI × A_IPROPI)
 *   ② IPROPI 的内部钳位点 = **ADC 可测上限**
 * 该分压**原方案无门控**, 常态耗 106µA ≈ 0.42mW@24V —— 是整机待机预算
 * (0.25mW) 的 1.7 倍。改法是给分压加高边 P-MOS, 其栅极由 **nSLEEP (GPIO16)**
 * 经 NPN 驱动: GPIO16 高 → VREF 有效; GPIO16 低 → VREF=0。
 *
 * ⇒ **固件侧零改动**, 但必须遵守两条纪律 (见 power_state.c):
 *   1. power_state.c 是 GPIO16 的唯一写入方
 *   2. 深睡/故障态必须真正拉低 nSLEEP, 否则 106µA 照烧, 待机预算当场破
 */

/*
 * ⚠️ Strapping 脚纪律 (docs/doc.md §四, 上电锁存电平核查表)
 *   GPIO4(MTMS)=0  由 IPROPI-1 的 R_IPROPI 对地决定      ✅ 本设计不用 SDIO
 *   GPIO5(MTDI)=0  由 IPROPI-2 的 R_IPROPI 对地决定      ✅ 同上
 *   GPIO8=1        由 WS2812 P-MOS 栅极 100k 上拉决定    ⚠️ **严禁加下拉**
 *   GPIO9=1        10k 上拉 + LED                        ✅
 *   GPIO15         由 DRV8874#2 IN1 内部下拉决定          ⚠️ 默认不影响启动
 *   —— GPIO8=0 且 GPIO9=0 是非法组合, 会导致无法进入下载模式。
 *
 * ⚠️ JTAG: GPIO4~7 是 RISC-V JTAG 默认复用脚 (MTMS/MTDI/MTCK/MTDO),
 *    本设计用作 IPROPI-1/IPROPI-2/PMODE/分压门控 ⇒ **外部 JTAG 不可用**,
 *    调试只能走 USB-Serial-JTAG (GPIO12/13)。
 */

/*
 * ★★ PH/EN 模式下 EN 到底在哪根脚 —— **必须上板裁决**
 *
 * 数据手册的引脚命名是 **EN/IN1** 和 **PH/IN2** (见 Recommended Operating Conditions:
 * "Logic input voltage: EN/IN1, MODE, nSLEEP, PH/IN2")。
 * 即: **EN 功能在名为 IN1 的引脚上, PH 功能在名为 IN2 的引脚上。**
 *
 * 而 docs/doc.md §四 的引脚表写的是:
 *   GPIO10 = "DRV8874#1 IN2 **（EN）**"   ← 名为 IN2 却标成 EN
 *   GPIO11 = "DRV8874#1 IN1 **（PH）**"   ← 名为 IN1 却标成 PH
 * ⇒ **两处交叉, 与数据手册相反**。文档没说清是按引脚名还是按功能标的。
 *
 * 这不是小事: PH/EN 模式下只有 **EN=0 才是 Brake** (两个低边导通)。
 * 若把 PH 当成 EN 去拉低, 实际是"EN 仍为 1、PH=0" ⇒ **满压反转驱动**,
 * 而不是刹车。唤醒时"先切 brake 泄放反灌能量"那一步会变成猛冲。
 *
 * ⇒ 用 Kconfig `FOCSTEP_PHEN_SWAPPED` 在台面上裁决, 不必改固件重烧:
 *    默认(未置换) = 按**功能**标注, 即 GPIO10=EN、GPIO11=PH。
 *    打开开关    = 按**引脚名**标注, 即 GPIO11=EN、GPIO10=PH。
 *
 * 上板自检法 (见 cmd_console 的 selftest):
 *    1. 两相 EN 全拉低, 用手转轴 → 应有**明显阻尼** (brake), 且电机不主动转;
 *       若电机使劲朝一个方向转 ⇒ EN/PH 认反了, 打开本开关重试。
 *    2. 抬 nSLEEP 前务必确认第 1 步通过。
 */
#if CONFIG_FOCSTEP_PHEN_SWAPPED
#define PIN_DRV1_EN         PIN_DRV1_IN1
#define PIN_DRV1_PH         PIN_DRV1_IN2
#define PIN_DRV2_EN         PIN_DRV2_IN1
#define PIN_DRV2_PH         PIN_DRV2_IN2
#else
#define PIN_DRV1_EN         PIN_DRV1_IN2
#define PIN_DRV1_PH         PIN_DRV1_IN1
#define PIN_DRV2_EN         PIN_DRV2_IN2
#define PIN_DRV2_PH         PIN_DRV2_IN1
#endif
