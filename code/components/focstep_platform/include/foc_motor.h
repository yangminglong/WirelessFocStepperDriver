#pragma once

/*
 * FOC 步进电机封装 (esp_simplefoc)
 *
 * ── 两个关键设计选择, 都有源码/手册依据 ──────────────────────
 *
 * ★ 1. 用 `StepperDriver2PWM` + **PH/EN 模式 (PMODE=低)** —— 即 doc.md v0.7 §10.1 定稿口径
 *      (v0.6 及以前文档写的 `StepperDriver4PWM` + PMODE=高**已作废**)。依据 DRV8874 数据手册真值表:
 *
 *        Table 3 PH/EN (PMODE=低):  EN=0 → **Brake (Low-Side Slow Decay)**
 *        Table 4 PWM  (PMODE=高):  IN1=IN2=0 → **Coast (Hi-Z)**
 *
 *      而 `StepperDriver4PWM::setPwm()` 的实现是"一个输入出 PWM、另一个恒为 0"
 *      (esp_hal_stepper.cpp), 即每周期都停在 (0,0) = **Coast**。手册 §7.3.3.1 明确:
 *      *"In coast mode, the current is freewheeling and **cannot be sensed**"*。
 *      ⇒ 4PWM 会让 IPROPI **只在导通期有效** (S0 堵转检测和将来的电流环都受损)。
 *
 *      PH/EN 的 Brake 是低边慢衰减, 手册: *"allows for the motor winding current to be
 *      sensed in both the drive and brake low-side slow-decay periods allowing for
 *      **continuous current monitoring**"*。⇒ IPROPI 连续, 无需对齐 PWM 采样窗口。
 *
 *      附带好处: 慢衰减的电流纹波更小 (对齐 §10.4 的"无异响"), 且 LEDC 占用 4→2 路。
 *      **引脚完全不变**: GPIO10/11/14/15 本来就是每片一 EN 一 PH。
 *      ⚠️ PMODE (GPIO6) 必须保持**低**。doc §五.4 定稿"上电默认低（PH/EN 安全态）", 与本设计一致。
 *
 * ★ 2. 力矩环用 `TorqueControlType::estimated_current` (需实测 phase_resistance)。
 *      事实边界 (已核对源码):
 *        - `StepperMotor.cpp:27` / `HybridStepperMotor.cpp:27` 构造里把 torque_controller
 *          默认成 voltage, 注释 "current and foc_current not supported yet" ——
 *          **但它只是默认值**: torque_controller 是 FOCMotor 的 public 成员,
 *          且有 updateTorqueControlType()。
 *        - `FOCMotor::loopFOC():609` **通用处理** estimated_current, 只要求
 *          `_isset(phase_resistance)` ⇒ **对步进电机可用**。
 *        - `foc_current` 需要 current_sense; 而 `LowsideCurrentSense` 要求 MCPWM 的
 *          driver params, 步进驱动给的是 LEDC 的 ⇒ init 必失败 (端口实现缺陷, 可能越界写)。
 *      ⇒ estimated_current 是**最接近电流环**的可用选项; 真电流环见 docs 的 S1~S4。
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 运动模式 —— **通用, 不含应用语义**。
 *
 * ★ 重构要点: 原来这里有个 `FOC_MODE_ASSIST`(手拉助动)。但"助动"是**门机概念**,
 *   不是电机概念 —— 换个项目(比如云台、机械臂)这个概念就不存在。
 *   现在只提供通用的 `FOC_MODE_TORQUE`, 应用自己决定何时给多大力矩。
 *   门的助动逻辑在 app_door.c 里。
 */
typedef enum {
    FOC_MODE_IDLE = 0,   /* 无力矩 (但桥仍使能 → Brake 阻尼) */
    FOC_MODE_TORQUE,     /* 直接给 q 轴力矩。应用可拿它做助动/张紧/保持 */
    FOC_MODE_VELOCITY,   /* 速度环 */
    FOC_MODE_POSITION,   /* 位置环: 走到目标 */
} foc_mode_t;

esp_err_t foc_motor_init(void);

/* 使能/失能电机。
 * ⚠️ 实现方**不碰 nSLEEP** —— 那是 power_state.c 的唯一职责 (它同时门控 VREF)。 */
esp_err_t foc_motor_enable(bool on);

/* ★ 唤醒后必须**先切 brake** 泄放反灌能量, 再读角度 (docs/doc.md §10.3 硬性行为)。
 *  在 PH/EN 模式下 brake = EN 恒低 = 两个低边导通。 */
esp_err_t foc_motor_brake(void);

/* 启动 1kHz FOC 任务 */
esp_err_t foc_motor_start_loop(void);
void foc_motor_stop_loop(void);

void foc_motor_set_mode(foc_mode_t mode);
foc_mode_t foc_motor_get_mode(void);

/* 位置环目标: 归一化 0.0 = 零点(全关), 1.0 = 满行程点(全开)。
 * ⚠️ 要求: 位置可信 **且** 行程完整(BOTH) —— 否则拒绝执行 (打日志, 不动)。
 * 输入会被**夹到 [0,1]**; 需要越过零点/超出满程请用 foc_motor_move_to_ext()。 */
void foc_motor_move_to(float normalized);

/* 同 move_to, 但**不夹紧**: 允许负开度(越过零点)或 >1(超出满行程点)。
 * 用途: 手工标定/探边/机构调试。门禁仍要求位置可信 + 行程完整;
 * 越界会打警告日志 (这是"故意允许"的动作)。 */
esp_err_t foc_motor_move_to_ext(float normalized);
void foc_motor_stop(void);

/* 绝对多圈角目标 (rad)。回零用 —— 可以指到行程之外, 逼机构顶到机械限位。 */
void foc_motor_set_target_rad(float rad);
float foc_motor_get_target_rad(void);
float foc_motor_get_angle_rad(void);
int32_t foc_motor_turns(void);

/* 临时改电压上限。**回零必须调小** (Kconfig 的 FOCSTEP_HOME_TORQUE),
 * 否则以运行力矩顶机械限位会损坏机构。 */
void foc_motor_set_voltage_limit(float volts);
float foc_motor_get_voltage_limit(void);

/* Kconfig 配的运行电压上限。应用临时改小力矩 (如助动) 后, 用这个恢复。 */
float foc_motor_default_voltage_limit(void);

/* 直接给 q 轴力矩 (单位 V, 受 voltage_limit 限幅)。配合 FOC_MODE_TORQUE 使用。
 * 正负号 = 方向。应用可拿它做助动/张紧/保持 —— 具体策略在应用层。 */
void foc_motor_set_torque(float volts);

/* 速度环目标 (rad/s)。配合 FOC_MODE_VELOCITY 使用。 */
void foc_motor_set_velocity(float rad_s);

/* 状态快照 (供自检命令) */
typedef struct {
    float angle_rad;       /* 单圈机械角 (含方向与零位) */
    float multi_turn_rad;  /* 多圈角 */
    float velocity;        /* rad/s */
    float target_rad;
    int32_t turns;
    uint32_t loop_us;      /* 最近一次 loopFOC 耗时 */
    bool enabled;
    bool sensor_ok;
} foc_motor_status_t;

void foc_motor_status(foc_motor_status_t *out);

/* 电角对齐 (initFOC)。⚠️ 需要电机能自由转动 —— 上板时先确保门是松开的。 */
esp_err_t foc_motor_align(void);

/* 相电阻实测 (characteriseMotor)。结果写回 Kconfig 的占位值需人工回填。 */
esp_err_t foc_motor_characterise(float volts);

/* ── 位置持久化与可信度 ────────────────────────────────────────
 *
 * 存的是 **(多圈计数, 当时的单圈角)** 一对, 不是只有圈数。
 *
 * 为什么必须成对存: 深睡期间 C6 不在计数, 而**手拉门是本产品的核心用法**
 * ⇒ 多圈计数必然可能发散。只存圈数就完全无从判断"睡着时动过没有"。
 * 存了单圈角, 醒来一比对就知道。
 *
 * ⚠️ 但**歧义是根本性的**: 只靠单圈绝对传感器, 无法区分"移动了 0.3 圈"与
 *    "移动了 1.3 圈" —— 单圈读数一样。所以策略只能是: **动过就标记不可信,
 *    下一条运动指令前先回零** (见 homing.h)。
 */
esp_err_t foc_motor_save_position(void);
esp_err_t foc_motor_restore_position(void);

/* ── 行程模型: 零点 + 满行程点 ─────────────────────────────────
 *
 * 行程由**两个语义端点**定义, 而**不是**按角度大小排的 min/max:
 *
 *     零点 (0% 开度) ──span──► 满行程点 (100% 开度)
 *
 *   · `zero_rad` / `end_rad` 都是绝对多圈角 (rad), 各自独立保存
 *   · `span = end_rad - zero_rad` **带符号** ⇒ "正方向"由 span 的符号表达,
 *     天然支持"零点的角度比满行程点更大"这种反装机构, 不需要排序
 *   · 开度映射: `target = zero_rad + normalized × span`
 *
 * 三态 (没有"有行程无零点"这种状态):
 *   NONE        未标定 ⇒ **拒绝一切开度指令** (不再退回任何占位值)
 *   ZERO_ONLY   只标了零点 ⇒ 位置可锚定, 但开度不可用
 *   BOTH        零点 + 满行程点 ⇒ 0%/100% 生效 (即"全关/全开")
 *
 * ⚠️ 行程是**编码器机械角坐标系**里的两个端点 ⇒ `ENCODER_DIRECTION` /
 *    `ENCODER_ZERO_OFFSET` 一改, 旧行程立刻不再对应机械端点。本模块把这两个
 *    值随行程一起存 NVS (指纹), 开机比对不上就**自动作废行程并标记位置不可信**
 *    (必须重标) —— 否则会出现"能跑但整段错位"这种现场发现不了的错。
 */
typedef enum {
    RANGE_NONE = 0,
    RANGE_ZERO_ONLY,
    RANGE_BOTH,
} range_state_t;

/* 行程最小跨度 (rad): 两端离得比这还近就说明标反了端或机构有问题 ⇒ 拒绝写入。
 * foc_motor 与 homing 共用这一个口径 (两处的校验必须一致)。 */
#define FOC_MOTOR_MIN_SPAN_RAD 0.1f

const char *foc_motor_range_state_str(range_state_t st);
range_state_t foc_motor_range_state(void);

/* 行程是否**完整**(= BOTH)。不完整时开度指令会被拒绝 */
bool foc_motor_has_range(void);

/* 标定"零点"(0% 开度) 与"满行程点"(100% 开度)。
 * ★ 若另一端已标定, **它的物理位置保持不变**, 只按新端点重算 span ——
 *   即重标零点不会把满行程点一起拖走 (反之亦然)。
 * 跨度校验: |span| 必须 ≥ 0.1 rad, 否则拒绝 (什么都不改)。
 * 约束: 先有零点才能设满行程点 (无零点 ⇒ ESP_ERR_INVALID_STATE)。 */
esp_err_t foc_motor_set_zero(float rad);
esp_err_t foc_motor_set_end(float rad);

/* 反向: 把零点与满行程点**对调** ⇒ "正方向"随之取反 (0%/100% 互换)。
 * ⚠️ 这是纯数据变换, 不碰编码器/Motor 的任何约定 —— 反转**编码器**约定仍走
 *    Kconfig 的 ENCODER_DIRECTION (那条路会让已标定行程按指纹校验作废)。
 * 要求 BOTH (只有一端时"反向"没有意义)。 */
esp_err_t foc_motor_invert_travel(void);

/* 读行程: zero/end 为绝对角, span = end - zero (可负)。任一指针可为 NULL。 */
void foc_motor_get_range(float *zero_rad, float *end_rad, float *span_rad);

/* 位置是否可信 (是否需要在执行运动指令前先回零) */
bool foc_motor_position_trusted(void);
void foc_motor_invalidate_position(const char *reason);

/* 电机/编码器是否已就绪到可以跑回零 (编码器 OK 且 FOC 已初始化) */
bool foc_motor_position_ready(void);

/* 编码器健康检查 (供 power_state 钩子) */
bool foc_motor_encoder_healthy(void);

/* 取编码器对象句柄 (void* 以免 C 侧 include C++ 头) */
void *foc_motor_encoder_handle(void);

/* 切换编码器功耗档。cmd: 0x10=连续测量, 0x20=Wake-up&Sleep (深睡档) */
esp_err_t foc_motor_encoder_set_mode(uint8_t cmd);

/* ── 轻睡档 (Kconfig FOCSTEP_SLEEP_MODE_LIGHT) 支持 ─────────────
 *
 * ★ 为什么需要这一组: DeepSleep 唤醒即复位 ⇒ 一切靠"重跑 init + 读 NVS"解决;
 *   而 LightSleep 唤醒**不复位、从睡眠点继续** ⇒ 必须显式处理三件事:
 *     ① 1kHz 的 FOC 任务要暂停, 否则系统永远到不了 idle, 且醒来会追打时间基;
 *     ② KTH5701 的 INT 是**锁存型**, 醒来必须读一次数据清掉, 否则立刻重复唤醒;
 *     ③ 睡着期间门若被拉动, 多圈计数会发散 ⇒ 醒来要重做"位置是否可信"的判定。
 */

/* 暂停/恢复 FOC 任务。进 PS_SLEEP 前暂停, 回 PS_ACTIVE 时恢复。
 * 恢复时会自动重置时间基 —— ⚠️ 直接调 esp_light_sleep_start() 时 IDF **不补偿**
 * FreeRTOS tick (只有自动轻睡才走 pm_step_tick), 不重置会让 vTaskDelayUntil 连续追打。 */
void foc_motor_pause_loop(void);
void foc_motor_resume_loop(void);

/* 读一次编码器数据清 INT 锁存 (KTH5701: 高有效、锁存、读数据清零)。
 * 返回 INT 是否已归低。⚠️ 轻睡醒来后**必须**调用, 否则下次入睡会被立即再次唤醒。 */
bool foc_motor_encoder_clear_int(void);

/* 睡前记下当前单圈角 (只存 RAM, **不写 NVS** —— 轻睡 RAM 保留);
 * 醒来调 check 比对, 超容差 ⇒ 位置标记不可信, 需重新回零。
 * 判据与 foc_motor_restore_position() 里的一致性检查相同 (共用一个 helper)。 */
void foc_motor_mark_sleep_angle(void);
esp_err_t foc_motor_check_sleep_angle(void);

#ifdef __cplusplus
}
#endif
