#pragma once

/*
 * 无限位回零 (sensorless homing) —— docs/doc.md §10.4 ② "上电自学习限位"
 *
 * 不依赖任何限位开关: 以**回零力矩**朝机械限位推, 靠"位置不再前进 + 电流抬升"
 * 判定接触, 把该点设为基准, 并在两端之间自学习出行程范围。
 *
 * ── 判据为什么这么选 ──────────────────────────────────────────
 * **位置是主判据, 电流只是确认。**
 *   · 不能用"位置误差大" —— 目标故意设在行程外, 误差从一开始就是大的。
 *     正确的表述是 **"还在下指令, 但实际位置不再变化"**。
 *   · 不能只用电流 —— VIPROPI 被钳位在 ≈1.49A, **分不出"轻触"和"顶死"**,
 *     而门机恰恰在意这个区分; 且 AERR ±6% 让它作为绝对量本就不准。
 *   · 位置是**比值判据**: 免疫 A_IPROPI 容差、温度、轨压漂移。
 *
 * ── 必须回零的场合 ────────────────────────────────────────────
 * 平时不需要: KTH5701 是**单圈绝对**编码器, 多圈计数存 NVS。
 * 但深睡期间 C6 不在计数, 而**手拉门是本产品的核心用法** ⇒ 多圈计数会发散。
 * ⚠️ 而且这个歧义是**根本性**的: 只有单圈绝对传感器, 分不清"动了 0.3 圈"与
 *    "动了 1.3 圈" —— 单圈读数一样。所以策略只能是: **动过就标记不可信, 先回零**。
 *    见 foc_motor_restore_position()。
 *
 * ── 安全 ──────────────────────────────────────────────────────
 * 以运行力矩顶机械限位会损坏机构。回零全程使用 Kconfig 的 **FOCSTEP_HOME_TORQUE**
 * (明显小于运行力矩), 结束/失败后必须恢复原电压上限。
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HOME_END_CLOSED = 0, /* 全关端 (通常作为基准零点) */
    HOME_END_OPEN = 1,   /* 全开端 */
} home_end_t;

typedef enum {
    HOME_OK = 0,
    HOME_ERR_NOT_READY,    /* 电机/编码器未就绪 */
    HOME_ERR_NO_CONTACT,   /* 走了 HOME_MAX_TRAVEL 还没碰到限位 */
    HOME_ERR_UNREPEATABLE, /* 两次顶到的位置差超过容差 —— 基准不可信 */
} homing_result_t;

const char *homing_result_str(homing_result_t r);

/*
 * 朝某一端回零。
 * 成功时 *out_angle 为接触点的绝对多圈角 (rad), 且电机已退回 HOME_BACKOFF。
 * **会做两次approach 并比对重复性** —— 不重复就报 HOME_ERR_UNREPEATABLE,
 * 因为偏一步的基准会让机构永远偏。
 */
homing_result_t homing_run(home_end_t end, float *out_angle);

/*
 * 完整自学习: 回零到两端, 建立基准与行程范围, 存 NVS, 标记位置可信。
 * 这是上电/失位后应该调用的那一个。
 */
homing_result_t homing_learn_range(void);

/* 位置是否已建立基准 */
bool homing_has_datum(void);

/* 自检用: 打印当前基准与行程 */
void homing_print_status(void);

#ifdef __cplusplus
}
#endif
