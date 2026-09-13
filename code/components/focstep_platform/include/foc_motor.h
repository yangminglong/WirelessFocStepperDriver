#pragma once

/*
 * FOC 步进电机封装 (esp_simplefoc)
 *
 * ── 两个关键设计选择, 都有源码/手册依据 ──────────────────────
 *
 * ★ 1. 用 `StepperDriver2PWM` + **PH/EN 模式 (PMODE=低)**, 不用文档 §10.1 写的
 *      `StepperDriver4PWM` + PMODE=高。依据 DRV8874 数据手册真值表:
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
 *      ⚠️ PMODE (GPIO6) 必须保持**低**。doc §五.4 本来就写了"上电默认低（PH/EN安全态）"。
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

/* 位置环目标: 归一化 0.0 = 全关, 1.0 = 全开 */
void foc_motor_move_to(float normalized);
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

/* 行程限位 (由回零自学习得到, 存 NVS) */
void foc_motor_set_travel_range(float min_rad, float max_rad);
void foc_motor_get_travel_range(float *min_rad, float *max_rad);

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

#ifdef __cplusplus
}
#endif
