#pragma once

/*
 * FocStepper 驱动板 GPIO 分配 —— **唯一真源**
 *
 * 抄自 docs/doc.md §四「GPIO引脚分配表（ESP32-C6）」。
 * 任何引脚改动只改这里; 其余文件一律引用本头文件, 不得硬编码 GPIO 号。
 *
 * ⚠️ 占用 22 脚 (= 模块实际引出的全部 GPIO), **LP 域与 HP 域均已用满, 0 余量**。
 *    新增功能只能复用既有信号 (例: CAN Rs 反相复用 nSLEEP, 见 docs/doc.md §5.3)。
 */

/* ---- LP 域 (GPIO0~7): 深睡期间仍供电, 支持 ext1 唤醒 ---- */
#define PIN_XTAL32K_P        0  /* 32.768kHz 晶振, 不可再作他用 */
#define PIN_XTAL32K_N        1  /* 32.768kHz 晶振, 不可再作他用 */
#define PIN_VBUS_ADC         2  /* 母线分压 (经高边门控), ADC1_CH2 */
#define PIN_ENCODER_INT      3  /* KTH5701 INT: 高有效、锁存、读数据清零 */
#define PIN_IPROPI_1         4  /* DRV8874#1 电流反馈, ADC1_CH4 */
#define PIN_IPROPI_2         5  /* DRV8874#2 电流反馈, ADC1_CH5 */
#define PIN_WS2812_DIN       6  /* RMT 驱动, 串联 300~470Ω。PMODE 已移到 GPIO23 */
#define PIN_VBUS_GATE        7  /* 母线分压高边 P-MOS 栅极 (经 NPN 电平转换) */

/* ---- HP 域 (GPIO8~23) ---- */
#define PIN_DRV_nFAULT       8  /* 两片线与 (10k 上拉) —— 无法区分是哪一片 */
#define PIN_STATUS_LED       9  /* 低有效 + 10k 上拉; BOOT 按键直连此脚 */
/* GPIO10/11 ⛔ 模块未引出 —— 见下面「22 脚硬约束」 */
/* GPIO12/13 = USB D-/D+ */
#define PIN_I2C_SDA         14  /* KTH5701 编码器 + MCP4725 DAC (0x60), 同总线 */
#define PIN_I2C_SCL         15  /* KTH5701/MCP4725。strapping 脚, 电平由 I2C 的 4.7k 上拉提供 */
#define PIN_DEBUG_TXD       16  /* = TXD0 (UART0 控制台), 4P 排针; 同时接 U6 的 TXD */
#define PIN_DEBUG_RXD       17  /* = RXD0 (UART0 控制台), 4P 排针; 同时接 U6 的 RXD */
#define PIN_DRV_nSLEEP      18  /* 两片共用, 10k 下拉, 上电默认电机断电 */
#define PIN_DRV1_IN1        19  /* DRV8874#1 IN1 = EN (PMODE=低 时, 低=Brake) */
#define PIN_DRV1_IN2        20  /* DRV8874#1 IN2 = PH */
#define PIN_DRV2_IN1        21  /* DRV8874#2 IN1 = EN */
#define PIN_DRV2_IN2        22  /* DRV8874#2 IN2 = PH */
#define PIN_DRV_PMODE       23  /* 两片共用: 低=PH/EN、高=PWM、Hi-Z=独立半桥 */

/*
 * ---- CAN = 与 UART0 调试口**共用** GPIO16/17 (docs/doc.md §5.3) ----
 *
 * 收发器 SN65HVD231 的 TXD/RXD 与 4P 调试排针挂在同一对网络上, 靠**使用纪律**互斥:
 *   用 4P 调试口 → CAN 总线必须拔掉;  用 CAN → 调试改走 USB-Serial-JTAG。
 * ⚠️ **"用 UART 时 CAN 不可用" 这句话救不了电气问题** —— 收发器的 RXD 是**推挽输出**,
 *    按 TI SLOS346 表 2: Rs=高(睡眠) 时 RXD **被驱动为高**(非高阻), Rs=低时镜像总线,
 *    **两个状态都在驱动** ⇒ 软件切不掉它。
 *    ⇒ 硬件上必须在 **U6 的 RXD 与 GPIO17 之间串一颗 1.5kΩ** (见 docs/doc.md §5.3;
 *      漏贴则 4P 调试口收不到数据, 且两个推挽输出对顶互灌十几~几十 mA)。
 */
#define PIN_CAN_TXD         PIN_DEBUG_TXD   /* GPIO16 */
#define PIN_CAN_RXD         PIN_DEBUG_RXD   /* GPIO17 */

/*
 * ⚠️ **没有 PIN_CAN_RS** —— Rs 接在反相 N-MOS (Q3=DMN3150L) 的漏极上, **随 nSLEEP 硬件派生**:
 *    nSLEEP 高 → Q3 导通 (栅极 100k) → 漏极 ≈0V → Rs 低 = CAN 唤醒
 *    nSLEEP 低 → Q3 截止 → 漏极经 100k 上拉 = 3.4V → Rs 高 = CAN 睡眠
 *    (复位/深睡 GPIO18 高阻时由 10k 下拉兜底 → Q3 关断 → 睡眠, 安全)
 * ⇒ **软件无法独立控制 CAN 醒睡**, 故本工程不提供 can_set_active() 一类接口 (docs/doc.md §5.3)。
 */

/*
 * ★ VREF 动态档位说明 (docs/doc.md §五.4)
 *
 * DRV8874 的 VREF 由 **MCP4725 (U8, I2C 0x60) 直接驱动**, 决定两件事:
 *   ① 斩波阈值 ITRIP = V_VREF / (R_IPROPI × A_IPROPI)
 *   ② IPROPI 的内部钳位点 = **ADC 可测上限** —— VREF 降档 → 低电流档分辨率提升
 * 12-bit DAC 挂在编码器同一条 I2C 总线 (GPIO14/15), 待机进 PD (VREF=0, 60nA)。
 *
 * ⇒ 固件侧: `vref_dac.c` 提供档位 API (vref_dac_set/pd), **power_state.c 是
 *   GPIO18 与 DAC PD 的唯一编排方** (§5.4 纪律):
 *   睡眠 nSLEEP=0 → DAC PD; 唤醒 DAC 输出目标 VREF → 稳定 → nSLEEP=1。
 *   深睡/故障态漏掉 DAC PD = 210µA 常挂 3.4V 轨, 待机预算当场破。
 */

/*
 * ⚠️ 22 脚硬约束 (docs/doc.md §四, 均为手册事实非推断)
 *   1. **模块没有 GPIO10/GPIO11** —— ESP32-C6-MINI-1 只引出 22 个 GPIO
 *      (GPIO0~9 + GPIO12~23), 官方 53 脚定义表中无此二名 (NC 脚为 4、7、21、32~35)。
 *      ⇒ **按 GPIO0~23 共 24 脚做预算会超编 2 个**, 这是原版把 DRV#1 的 EN/PH
 *        画到 GPIO10/11 上去的原因。
 *   2. **GPIO16/17 只以 TXD0/RXD0 引出**, 即 UART0 —— 它们同时是控制台通路,
 *      **与 CAN 分时共用** (见上)。
 *
 * ⚠️ Strapping 脚纪律 (docs/doc.md §四, 上电锁存电平核查表)
 *   GPIO4(MTMS)=0  由 IPROPI-1 的 R_IPROPI 对地决定      ✅ 本设计不用 SDIO
 *   GPIO5(MTDI)=0  由 IPROPI-2 的 R_IPROPI 对地决定      ✅ 同上
 *   GPIO8=1        由 **nFAULT 的 10k 上拉**决定          ⚠️ **严禁加下拉**
 *   GPIO9=1        由芯片内部 45k 上拉保证                ✅ 无需外部上拉
 *   GPIO15         由 **I2C SCL 的 4.7k 上拉**决定        ⚠️ **不能让它浮空**
 *   —— GPIO8=0 且 GPIO9=0 是非法组合, 会导致无法进入下载模式。
 *
 * ⚠️ JTAG: GPIO4~7 是 RISC-V JTAG 默认复用脚 (MTMS/MTDI/MTCK/MTDO),
 *    本设计分别用作 IPROPI-1、IPROPI-2、**WS2812 DIN**、分压门控 ⇒ **外部 JTAG 不可用**。
 *    ⚠️ **本文件内注意: 斜杠不要紧邻星号** —— 两者相邻会构成 C 的嵌套注释起始符,
 *       被 GCC 判为 -Wcomment 直接编译失败。
 *    控制台另有两条通路: 4P 排针 (GPIO16/17, 默认) 与 USB-Serial-JTAG (GPIO12/13, 用 CAN 时)。
 */

/*
 * ★ EN/PH 定线 (docs/doc.md §四 表下注), 原理图按此实现
 *
 * 表中 IN1/IN2 是 DRV8874 的**引脚名**, 括号内是该脚在 PMODE=低(PH/EN)模式下的**功能**。
 * 数据手册引脚名即 **EN/IN1** 和 **PH/IN2** (Recommended Operating Conditions:
 * "Logic input voltage: EN/IN1, MODE, nSLEEP, PH/IN2"), 两脚内部均有下拉。
 * ⇒ **EN 在 IN1 脚上, PH 在 IN2 脚上**。本板定线:
 *      GPIO19 / GPIO21 → IN1 = EN
 *      GPIO20 / GPIO22 → IN2 = PH
 *   与下面**默认分支**一致 ⇒ 正常打样不需要动 Kconfig。
 *
 * ⚠️ 这件事要命: PH/EN 模式下只有 **EN=0 才是 Brake** (两个低边导通)。
 * 若原理图把两根接反 (GPIO19 实际接到 IN2/PH), 则"先切 brake"实际是
 * "EN 仍为 1、PH=0" ⇒ **满压反转驱动**, 唤醒时不是刹车而是猛冲。
 *
 * ⇒ `FOCSTEP_PHEN_SWAPPED` 保留为**接反救回开关** (台面救板, 不必重画):
 *    默认(关) = 与原理图一致, EN 认在 IN1 = GPIO19 / GPIO21
 *    打开      = 接反救回,     EN 认在 IN2 = GPIO20 / GPIO22
 *
 * 打样验线 (platform_console 的 brake 命令):
 *    1. 两相 EN 全拉低, 用手转轴 → 应有**明显阻尼** (brake), 且电机不主动转;
 *       若电机使劲朝一个方向转 ⇒ 两根接反了, 打开本开关重试。
 *    2. 抬 nSLEEP 前务必确认第 1 步通过。
 */
#if CONFIG_FOCSTEP_PHEN_SWAPPED
#define PIN_DRV1_EN         PIN_DRV1_IN2
#define PIN_DRV1_PH         PIN_DRV1_IN1
#define PIN_DRV2_EN         PIN_DRV2_IN2
#define PIN_DRV2_PH         PIN_DRV2_IN1
#else
#define PIN_DRV1_EN         PIN_DRV1_IN1
#define PIN_DRV1_PH         PIN_DRV1_IN2
#define PIN_DRV2_EN         PIN_DRV2_IN1
#define PIN_DRV2_PH         PIN_DRV2_IN2
#endif
