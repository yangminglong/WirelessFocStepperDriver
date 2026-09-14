#pragma once

/*
 * 唤醒源管理
 *
 * 唤醒源 (docs/doc.md §六):
 *   - 编码器 KTH5701 INT → GPIO3 (LP 域) → **ext1**
 *   - CAN 收发器 RXD    → GPIO20 **(HP 域, 不支持 ext1)**
 *   - 干接点            → 与编码器 INT 共线
 *
 * ★ 先澄清一个常见误解: **ESP32-C6 深睡期间 BLE/Wi-Fi 控制器无法保持接收**
 *   (v3.3/v4.4/v5.1 三代源码验证)。所以"无线唤醒"在本板上只能唤醒到轻睡,
 *   深睡档不监听射频。本模块只管本地唤醒源。
 *
 * ── ext1 关键约束 ────────────────────────────────────────────
 * 只有 **LP 域 (GPIO0~7)** 的 RTCIO 支持 ext1。本板 GPIO0/1 被晶振占用,
 * 可用的是 GPIO2~7, 其中只有 GPIO3 (编码器 INT) 是输入唤醒源。
 *
 * ⚠️ KTH5701 的 INT 是**高有效且锁存**, **读一次数据即清零** ⇒ 必须配
 *    ESP_EXT1_WAKEUP_ANY_HIGH。若配成 ANY_LOW 则永远唤不醒;
 *    若读数据不清锁存则唤醒后 INT 一直高, 会反复唤醒。
 *    **上板自检第 10 项就是验这一条** —— 它决定 §10.3 深睡档能否成立。
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
    WAKE_SRC_NONE = 0,
    WAKE_SRC_ENCODER_INT, /* KTH5701 INT / 干接点 (共线) */
    WAKE_SRC_OTHER,
} wake_src_t;

esp_err_t wakeup_init(void);

/* 配置 ext1 唤醒源。调用后可以进深睡。 */
esp_err_t wakeup_enable_encoder_ext1(void);

/* 关闭 ext1 唤醒 (进入运行态时用, 避免 INT 抖动干扰) */
esp_err_t wakeup_disable_ext1(void);

/* 查询是什么唤醒的 (从 RTC 域读, 深睡唤醒后第一时间调用) */
wake_src_t wakeup_get_source(void);

/* 进深睡前调用: 打印将要生效的唤醒配置, 便于串口核对 */
void wakeup_log_config(void);

/* ── 轻睡档 (Kconfig FOCSTEP_SLEEP_MODE_LIGHT) ────────────────── */

/* 编码器 INT 线当前是否为高 (锁存未清)。
 * 轻睡循环用它判断"这次醒来是不是手拉" —— 因为 INT 是**锁存**的,
 * 只看电平就够, 不依赖唤醒掩码。 */
bool wakeup_encoder_int_asserted(void);

/* ★ 轻睡醒来后立即调用, 一个函数做完三件事:
 *     ① 关 ext1 (处理期间不再被它打断; 下次进睡前重新 arm)
 *     ② 读一次编码器数据**清 INT 锁存**并做"睡着期间是否被动过"的一致性检查
 *     ③ 发 FOCSTEP_EVT_WOKE 事件 (由应用决定接管还是继续睡)
 * ⚠️ 必须在**再次进睡之前**调用, 否则同一次中断会把 MCU 立刻重复唤醒。
 * 返回本次唤醒源。 */
wake_src_t wakeup_resume_from_light_sleep(void);

#ifdef __cplusplus
}
#endif
