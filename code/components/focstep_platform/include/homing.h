#pragma once

/*
 * 无限位回零与行程标定 (sensorless homing) —— docs/doc.md §10.4 ② "上电自学习限位"
 *
 * 不依赖任何限位开关: 以**回零力矩**朝机械限位推, 靠"位置不再前进 + 电流抬升"
 * 判定接触, 把该点作为端点写进行程 (见 foc_motor.h 的行程模型)。
 *
 * ── 判据为什么这么选 ──────────────────────────────────────────
 * **位置是主判据, 电流只是确认。**
 *   · 不能用"位置误差大" —— 目标故意设在行程外, 误差从一开始就是大的。
 *     正确的表述是 **"还在下指令, 但实际位置不再变化"**。
 *   · 不能只用电流 —— VIPROPI 被钳位在 ≈1.49A, **分不出"轻触"和"顶死"**,
 *     而门机恰恰在意这个区分; 且 AERR ±6% 让它作为绝对量本就不准。
 *   · 位置是**比值判据**: 免疫 A_IPROPI 容差、温度、轨压漂移。
 *
 * ── 方向与端点语义: 各自在"能知道的那一侧"给 ──────────────────
 *
 *   标定**前**: 能知道的是**方向** —— 两个机械限位只可能是"零点那一侧"与
 *              "满行程点那一侧", 而"哪一侧是零点"正是待求量 ⇒ 只能给方向。
 *   标定**后**: 能派生的是**端点身份与方向** —— span 的符号就定义了方向
 *              (span > 0 = 零点在角度小的一侧), 不必再问。
 *
 * 由此分工: `homing_probe(dir)` 探路测点 (动电机, 只测), `homing_apply_contact()`
 * 落定写点 (不动电机, 只写)。**两个量不互相推导, 所以没有任何启发式容差** ——
 * 身份的判定不靠"离哪个端点近", 也不靠魔数, 而是由调用方声明。
 *
 * 注意 `homing_learn_range()` 的两端: 两端都是**顶出来的机械限位**, 而一个推拉
 * 机构只有两个限位 ⇒ 满行程点的方向必然是零点的**反向** (只有 2 种组合, 不是 4 种:
 * "零点在正端"就是反装机构那一种)。所以它只收一个方向。
 * 前提是两端都有机械限位; 若某一侧没有限位 (门能滑出口袋), 那一侧改用 `mark`。
 *
 * ── 必须回零的场合 ────────────────────────────────────────────
 * 平时不需要: KTH5701 是**单圈绝对**编码器, 多圈计数存 NVS。
 * 但深睡期间 C6 不在计数, 而**手拉门是本产品的核心用法** ⇒ 多圈计数会发散。
 * ⚠️ 而且这个歧义是**根本性**的: 只有单圈绝对传感器, 分不清"动了 0.3 圈"与
 *    "动了 1.3 圈" —— 单圈读数一样。所以策略只能是: **动过就标记不可信, 先回零**。
 *    见 foc_motor_restore_position()。
 *
 * ── 安全 ──────────────────────────────────────────────────────
 * 以运行力矩顶机械限位会损坏机构。回零全程使用 Kconfig 的 **FOCSTEP_HOME_TORQUE_MV**
 * (明显小于运行力矩), 结束/失败后必须恢复原电压上限。
 * 自动 (无人值守) 标定会让门自己动起来 ⇒ 单独由 homing_policy_t.auto_calib 门控, 默认关。
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#include "foc_motor.h" /* range_state_t (行程三态) */

#ifdef __cplusplus
extern "C" {
#endif

/* 回零方向: 朝角度减小 (负) 还是朝角度增大 (正) 的方向推。
 * 枚举值本身就是符号 (±1), 实现里直接当乘数用。 */
typedef enum {
    HOME_DIR_NEG = -1,
    HOME_DIR_POS = +1,
} home_dir_t;

const char *homing_dir_str(home_dir_t dir);

typedef enum {
    HOME_OK = 0,
    HOME_ERR_NOT_READY,    /* 电机/编码器未就绪 */
    HOME_ERR_NO_CONTACT,   /* 走了 HOME_MAX_TRAVEL 还没碰到限位 */
    HOME_ERR_UNREPEATABLE, /* 两次顶到的位置差超过容差 —— 基准不可信 */
    HOME_ERR_RANGE,        /* 会让行程非法 (两端重合 / 跨度太窄 / 顺序不对) */
    HOME_ERR_NOT_ALLOWED,  /* 自动标定未获允许 (安全前提不满足) —— 一个动作都没做 */
} homing_result_t;

const char *homing_result_str(homing_result_t r);

/* ── 标定策略 (安全前提 + 失败处理) ───────────────────────────
 * 默认值来自 Kconfig, 运行时可改 (比如由上位机在"允许自动标定"后置位)。
 * 放在一处是为了让"什么时候允许门自己动"只有一个判据, 不会散落各处。 */
typedef struct {
    uint8_t retries;    /* 一次完整标定失败后的重试次数 (0 = 不重试)。
                         * 默认 CONFIG_FOCSTEP_HOME_RETRY = 1 (即最多做两轮)。 */
    bool auto_calib;    /* ★ 是否允许**无人值守**自动标定。
                         * 默认 CONFIG_FOCSTEP_HOME_AUTO_ENABLE = n (关):
                         * 自动标定会让门自己跑到底, 装歪/卡住/有人扶着时会出事实伤害,
                         * 所以默认必须关, 由人在现场显式打开。 */
    home_dir_t zero_dir; /* 自动标定的默认零点方向 —— 无人值守时没人能告诉它
                         * "哪一侧是零点", 而标定前这恰恰是**待求量** ⇒ 只能配。
                         * 默认 CONFIG_FOCSTEP_HOME_AUTO_INVERTED = n ⇒ HOME_DIR_NEG
                         * (常规安装, 等价 `learn both neg`)。 */
} homing_policy_t;

void homing_get_policy(homing_policy_t *out);
void homing_set_policy(const homing_policy_t *in);

/* 探路 + 测点: 朝 dir 顶机械限位, **只测, 不改行程**。
 * 成功时 *out_contact_rad 为接触点的绝对多圈角 (rad), 且电机已退回 HOME_BACKOFF。
 * **会做两次逼近并比对重复性** —— 不重复就报 HOME_ERR_UNREPEATABLE,
 * 因为偏一步的基准会让机构永远偏。
 * 怎么用: `home <方向>` 看接触点是否合理, 再用 homing_apply_contact() 落定。 */
homing_result_t homing_probe(home_dir_t dir, float *out_contact_rad);

/* 落定: 把**已测到的接触点**写成端点 (不动电机) —— 单端重标的第二步。
 *
 * 典型流程 (命令台): `home neg` 测点 → 看日志确认合理 → `learn zero` 落定。
 * 拆成两步的价值: 一次到底时, 门中途撞到异物被接触判据误判的假端点会**直接写进
 * 标定**; 拆开后操作员先看到"自起点只移动了 0.03 rad"这种离谱数字, 可以否掉。
 *
 * @param angle   homing_probe() 返回的接触角 (由**调用方**持有 —— 平台模块不带会话状态)
 * @param as_zero true = 写成零点 (0%), false = 写成满行程点 (100%)
 *
 * ⚠️ as_zero=false 时要求已有零点 (无零点就没有 100% 可言) ⇒ HOME_ERR_NOT_READY。
 * ⚠️ 另一端的物理位置**保持不变** (决定 #6): 只改被落定的这一端, span 按新端点重算。
 * ⚠️ 若本次落定把 span **变号**, 说明 0%/100% 与门开合的对应关系翻了过来
 *    (那正是 `dir invert` 的语义) ⇒ 会大声告警, 但**不拦** (标定是有意的人为动作)。 */
homing_result_t homing_apply_contact(float angle, bool as_zero);

/* 标定**两端**: 朝 zero_dir 顶到的一端定为**零点 (0% 开度)**, 另一端 (反向)
 * 定为**满行程点 (100% 开度)**, 落盘并标记位置可信。
 * 常规机构 = `neg`; 反装机构 (零点在角度大的一侧) = `pos`。
 *
 * ★ 契约里含**重试**与**失败处理** (不是可选包装): 失败按策略 s_policy.retries
 *   重跑 (每次都是完整的两端标定), 仍失败则保持原有标定不动、位置仍不可信,
 *   并打一条明确的 ERROR。这些是"标定"这件事本身的条款 ⇒ 就写在这个名字下,
 *   不再另立一个 `..._with_retry` 式的名字 (机制会变, 名字会过期)。
 * ⚠️ 无人值守路径的**额外**后果 (报 PS_FAULT_CALIB 并失磁) 不在这里, 见 homing_auto()。
 *
 * 前提: 两端都有机械限位。某一侧没有限位 (门能滑出口袋) ⇒ 报 NO_CONTACT,
 * 那一侧请用 `mark end` (手推到位再标)。 */
homing_result_t homing_learn_range(home_dir_t zero_dir);

/* 无人值守自动标定 —— 只做三件事 (其余全在 homing_learn_range 里):
 *   ① 安全前提: 策略 auto_calib 允许 + 就绪 (不满足 ⇒ HOME_ERR_NOT_ALLOWED, 一动不动)
 *   ② 方向: 取策略的 zero_dir (没人能告诉它哪一侧是零点, 只能配)
 *   ③ 失败后果: 报 PS_FAULT_CALIB 并失磁, 需要人工介入
 * ⚠️ ③ 只有这条路径有 —— 手动路径失败时操作员就在旁边, 拉故障反而会挡住他下一步的
 *    `home`/`mark` (那些需要 PS_ACTIVE)。这条不对称就是本函数存在的理由。 */
homing_result_t homing_auto(void);

/* 手动标定: **把当前位置定为该端点**, 不推限位。
 * 适合"手推到机械端点后直接标" —— 这是免拆卸重标定的快捷方式。
 * 要求电机静止 (速度超阈值拒绝); 成功后落盘 + **位置可信** (决定 #2:
 * 刚标的这一点就是实物基准点, 已知自己在哪)。
 * ⚠️ 标的是**当前读数**作为端点值, 不会把读数强制归零 ——
 *    行程是任意 rad 的两个端点, 一切映射都相对它, 归不归零不影响功能。
 *    `mark_end` 同样要求已有零点。 */
esp_err_t homing_mark_zero(void);
esp_err_t homing_mark_end(void);

/* 行程三态 (直接转发 foc_motor): NONE / ZERO_ONLY / BOTH。
 * "全开/全关"只在 BOTH 时才成立。 */
range_state_t homing_range_state(void);

/* 行程是否**完整** (= BOTH)。判据是持久化的 NVS, 不是本次开机跑过 learn 没有 */
bool homing_has_range(void);

/* 自检用: 打印当前行程、方向解读与回零参数 */
void homing_print_status(void);

#ifdef __cplusplus
}
#endif
