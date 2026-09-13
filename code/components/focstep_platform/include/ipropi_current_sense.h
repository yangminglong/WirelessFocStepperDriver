#pragma once

/*
 * 用 DRV8874 IPROPI 实现 SimpleFOC 的 CurrentSense —— 支撑 foc_current
 *
 * ── 为什么这条路走得通 ────────────────────────────────────────
 * 之前判断"步进电机做不到 foc_current"是**错的**。错在把
 * `LowsideCurrentSense`（一个绑定 MCPWM 定时器 + 三电阻采样的**具体实现**）
 * 当成了拿到 CurrentSense 的唯一途径。实际上 `CurrentSense` 是**抽象接口**,
 * 只有两个纯虚 (init / getPhaseCurrents), 和 Sensor 一样可以自己写子类。
 *
 * 而且基类**已经内建了步进分支**, 不是为我们打的补丁:
 *   - `StepperDriver::type()` 返回 `DriverType::Stepper`
 *   - `CurrentSense::linkDriver()` 自动取 driver->type()
 *   - `CurrentSense::getABCurrents()`: *"if so there is no need to Clarke transform"*
 *     ⇒ 两相直接 `alpha=a, beta=b`, 不做 Clarke
 *   - `CurrentSense::getDQCurrents()` 的 Park 变换是通用的
 *   - `driverAlign()` 有专门的 `alignStepperDriver()` 分支
 *   ⇒ `getFOCCurrents()`（foc_current 唯一用到的接口）**开箱即用**。
 *
 * ── 本实现要解决的唯一硬问题: 符号 ───────────────────────────
 * DRV8874 手册 §7.3.3.1:
 *   `IPROPI(µA) = (I_LS1 + I_LS2) × A_IPROPI`
 * "ILSx in Equation 1 is only valid when the current flows from drain to source
 *  in the low-side MOSFET. If current flows from source to drain, the value of
 *  ILSx for that channel is **zero**."
 * ⇒ **IPROPI 恒 ≥ 0, 只给幅值, 没有符号。**
 *
 * 符号只能由**驱动方向**重建: 在 PH/EN 模式下, 相电流的方向就是最近一次
 * 指令的方向 (`setPwm` 里 Ua 的正负)。慢衰减 (EN=0, 两个低边导通) 期间电流
 * **继续沿原方向流动**, 所以"上一次指令的符号"仍然成立 —— 这正是选 PH/EN
 * 而不是 PWM 模式的又一个理由 (PWM 模式的 Coast 会切断电流通路)。
 *
 * ── 已知限制 (不掩饰) ────────────────────────────────────────
 *  1. **过零点附近有死区**: |I|→0 时符号无意义。因为 |I| 也→0, 误差有界,
 *     但它确实让电流环在零附近不干净。这是本方案最本质的缺陷。
 *  2. **符号有 1 个 FOC 周期的滞后** (1kHz 下 1ms): 读电流发生在 loopFOC 开头,
 *     而符号来自上一周期的 setPwm。对门机这种慢负载可忽略。
 *  3. **两相不是同时采样**: 两次 oneshot 读相隔约 40µs。
 *  4. **精度上限**: AERR 在 1–2A 档 ±6%, <0.4A 档是 ±30mA 固定偏置 ⇒
 *     作为电流环反馈传感器偏粗。
 *  5. **超 ITRIP 就测不到**: V_IPROPI 被钳位到 V_VREF ⇒ 可测上限 ≈1.49A,
 *     再高读数不再上升 (但 DRV8874 自己会斩波, 环也不会要求更高)。
 */

#include "common/base_classes/CurrentSense.h"
#include "drivers/StepperDriver2PWM.h"

/*
 * 记录最近一次指令方向的 StepperDriver2PWM 子类。
 * 符号重建的唯一来源 —— 没有它 CurrentSense 拿不到方向。
 */
class SignTrackingStepperDriver : public StepperDriver2PWM {
public:
    SignTrackingStepperDriver(int pwm1, int dir1, int pwm2, int dir2)
        : StepperDriver2PWM(pwm1, dir1, pwm2, dir2) {}

    void setPwm(float Ualpha, float Ubeta) override
    {
        /* 只记方向, 不记幅值 —— 幅值由 IPROPI 实测。
         * 慢衰减期间电流方向不变, 所以"上一次指令的方向"始终有效。 */
        _sign_a = (Ualpha >= 0.0f) ? 1 : -1;
        _sign_b = (Ubeta >= 0.0f) ? 1 : -1;
        StepperDriver2PWM::setPwm(Ualpha, Ubeta);
    }

    inline int sign_a() const { return _sign_a; }
    inline int sign_b() const { return _sign_b; }

private:
    volatile int _sign_a = 1;
    volatile int _sign_b = 1;
};

class IpropiCurrentSense : public CurrentSense {
public:
    explicit IpropiCurrentSense(SignTrackingStepperDriver *drv) : _drv(drv) {}

    /* 返回 0 = 失败, 1 = 成功 (与 SimpleFOC 约定一致) */
    int init() override;

    PhaseCurrent_s getPhaseCurrents() override;

    void disable() override; /* 失能时把读数清零, 避免环里用到陈旧值 */
    void enable() override { _enabled = true; }

    /* 诊断计数 */
    uint32_t read_fail_count() const { return _fail; }

private:
    SignTrackingStepperDriver *_drv;
    bool _enabled = true;
    uint32_t _fail = 0;
};
