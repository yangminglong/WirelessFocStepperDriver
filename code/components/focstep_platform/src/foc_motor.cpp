#include "foc_motor.h"
#include "board_pins.h"

#include <cmath>
#include <cstring>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* esp_simplefoc / arduino-foc */
#include "esp_simplefoc.h"
#include "StepperMotor.h"
#include "drivers/StepperDriver2PWM.h"
#include "common/foc_utils.h"

/* kth5701.h 是 C++ 头, 本文件是 C++ */
#include "kth5701.h"
#include "ipropi_current_sense.h"
#include "ipropi_sense.h"

static const char *TAG = "FOC";

/* Kconfig 里的浮点参数是字符串, 这里解析一次 */
static float cfg_float(const char *s, float fallback)
{
    if (s == nullptr || s[0] == '\0') {
        return fallback;
    }
    float v = strtof(s, nullptr);
    return (v == 0.0f && s[0] != '0') ? fallback : v;
}

static SignTrackingStepperDriver *s_driver = nullptr;
static StepperMotor *s_motor = nullptr;
static KTH5701 *s_encoder = nullptr;
static IpropiCurrentSense *s_cs = nullptr;

static TaskHandle_t s_loop_task = nullptr;
static volatile bool s_loop_run = false;
static volatile foc_mode_t s_mode = FOC_MODE_IDLE;

static volatile float s_torque_target = 0.0f;   /* FOC_MODE_TORQUE 用 */
static volatile float s_velocity_target = 0.0f; /* FOC_MODE_VELOCITY 用 */
static volatile float s_target_rad = 0.0f;
static volatile uint32_t s_loop_us = 0;

#define NVS_NS  "focstep"
#define NVS_KEY_TURNS      "turns"
#define NVS_KEY_ANGLE_MRAD "angle_mrad" /* 存盘时的单圈角, 毫弧度 */
#define NVS_KEY_RANGE_MIN  "rmin_mrad"
#define NVS_KEY_RANGE_MAX  "rmax_mrad"
#define NVS_KEY_TRUSTED    "trusted"

/* 行程限位与位置可信度 */
static float s_range_min = 0.0f;
static float s_range_max = 0.0f;
static bool s_position_trusted = false;

/* ------------------------------------------------------------------ */

esp_err_t foc_motor_init(void)
{
    /* ★★ PMODE 必须为**低** = PH/EN 模式。
     * 理由见 foc_motor.h 顶部: PH/EN 的 EN=0 是 Brake(低边慢衰减), IPROPI 连续可测;
     * PWM 模式的 (IN1=IN2=0) 是 Coast, 手册明确 "cannot be sensed"。
     * doc §五.4 本来就说"上电默认低（PH/EN安全态）"。 */
    gpio_config_t pmode = {
        .pin_bit_mask = 1ULL << PIN_DRV_PMODE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&pmode), TAG, "PMODE gpio config failed");
    ESP_RETURN_ON_ERROR(gpio_set_level((gpio_num_t)PIN_DRV_PMODE, 0),
                        TAG, "PMODE set low failed");

    /* 编码器 */
    s_encoder = new KTH5701(I2C_NUM_0,
                            (gpio_num_t)PIN_I2C_SCL,
                            (gpio_num_t)PIN_I2C_SDA,
                            CONFIG_FOCSTEP_ENCODER_I2C_ADDR,
                            CONFIG_FOCSTEP_ENCODER_I2C_HZ);
    ESP_RETURN_ON_ERROR(s_encoder->begin(), TAG, "encoder init failed");
    s_encoder->set_direction(CONFIG_FOCSTEP_ENCODER_DIRECTION);
    s_encoder->set_zero_offset(cfg_float(CONFIG_FOCSTEP_ENCODER_ZERO_OFFSET, 0.0f));

    /* 驱动: ⚠️ 用 2PWM 的 3 参数构造 —— (pwm, dir) 语义 = (EN, PH)。
     * 用 SignTrackingStepperDriver 子类是为了记录**最近一次指令的方向**,
     * 供 foc_current 重建 IPROPI 的符号 (IPROPI 无符号, 见 ipropi_current_sense.h)。 */
    s_driver = new SignTrackingStepperDriver(PIN_DRV1_EN, PIN_DRV1_PH,
                                             PIN_DRV2_EN, PIN_DRV2_PH);
    s_driver->voltage_power_supply = 24.0f; /* 母线标称; 运行中可由 vbus 更新 */
    s_driver->voltage_limit = cfg_float(CONFIG_FOCSTEP_VOLTAGE_LIMIT, 12.0f);
    ESP_RETURN_ON_FALSE(s_driver->init(), ESP_FAIL, TAG, "driver init failed");

    /* 电机: pp = 整步数/4。1.8° 步进 200 步/转 ⇒ pp=50 */
    s_motor = new StepperMotor(CONFIG_FOCSTEP_POLE_PAIRS,
                               cfg_float(CONFIG_FOCSTEP_PHASE_RESISTANCE, NOT_SET),
                               NOT_SET, NOT_SET, NOT_SET);
    s_motor->linkDriver(s_driver);
    s_motor->linkSensor(s_encoder);

    s_motor->voltage_limit = cfg_float(CONFIG_FOCSTEP_VOLTAGE_LIMIT, 12.0f);
    s_motor->velocity_limit = cfg_float(CONFIG_FOCSTEP_VELOCITY_LIMIT, 20.0f);

    /* ★ 力矩环选择。torque_controller 是 FOCMotor 的 public 成员,
     *   构造里那个 voltage 只是默认值, 不是限制。 */
#if CONFIG_FOCSTEP_TC_FOC_CURRENT
    /* 实测电流环。基类 CurrentSense 对 DriverType::Stepper 会跳过 Clarke,
     * getFOCCurrents() 开箱即用 —— 详见 ipropi_current_sense.h。 */
    s_cs = new IpropiCurrentSense(s_driver);
    ESP_RETURN_ON_FALSE(s_cs->init() != 0, ESP_FAIL, TAG, "current sense init failed");
    s_motor->linkCurrentSense(s_cs);
    s_motor->current_limit = (float)CONFIG_FOCSTEP_CURRENT_LIMIT * 0.001f;
    s_motor->PID_current_q.P = cfg_float(CONFIG_FOCSTEP_PID_CURR_P, 0.2f);
    s_motor->PID_current_q.I = cfg_float(CONFIG_FOCSTEP_PID_CURR_I, 20.0f);
    s_motor->PID_current_q.D = cfg_float(CONFIG_FOCSTEP_PID_CURR_D, 0.0f);
    s_motor->PID_current_d.P = cfg_float(CONFIG_FOCSTEP_PID_CURR_P, 0.2f);
    s_motor->PID_current_d.I = cfg_float(CONFIG_FOCSTEP_PID_CURR_I, 20.0f);
    s_motor->PID_current_d.D = cfg_float(CONFIG_FOCSTEP_PID_CURR_D, 0.0f);
    s_motor->LPF_current_q.Tf = cfg_float(CONFIG_FOCSTEP_PID_CURR_LPF, 0.005f);
    s_motor->LPF_current_d.Tf = cfg_float(CONFIG_FOCSTEP_PID_CURR_LPF, 0.005f);
    s_motor->torque_controller = TorqueControlType::foc_current;
    ESP_LOGW(TAG, "torque control: foc_current (实验性, 符号靠指令方向重建, 过零点有死区)");
    ESP_LOGW(TAG, "  电流上限 %d mA, PID P=%s I=%s —— ⚠️ 必须上板整定",
             CONFIG_FOCSTEP_CURRENT_LIMIT,
             CONFIG_FOCSTEP_PID_CURR_P, CONFIG_FOCSTEP_PID_CURR_I);
#elif CONFIG_FOCSTEP_TC_ESTIMATED
    if (s_motor->phase_resistance != NOT_SET) {
        s_motor->torque_controller = TorqueControlType::estimated_current;
        ESP_LOGI(TAG, "torque control: estimated_current (R=%.3fΩ)", s_motor->phase_resistance);
    } else {
        s_motor->torque_controller = TorqueControlType::voltage;
        ESP_LOGW(TAG, "torque control: voltage (相电阻未设置, 退回电压模式)");
    }
#else
    s_motor->torque_controller = TorqueControlType::voltage;
    ESP_LOGI(TAG, "torque control: voltage");
#endif

    /* ⚠️ motor.init() 内部**不调用 initFOC()** (结束时停在 motor_uncalibrated),
     *    对齐由 foc_motor_align() 显式触发 —— 上板时要先确保门是松开的。 */
    ESP_RETURN_ON_FALSE(s_motor->init(), ESP_FAIL, TAG, "motor init failed");


    ESP_LOGI(TAG, "init done: pp=%d PMODE=低(PH-EN) EN/PH %s",
             CONFIG_FOCSTEP_POLE_PAIRS,
#if CONFIG_FOCSTEP_PHEN_SWAPPED
             "已交换"
#else
             "未交换"
#endif
    );
    return ESP_OK;
}

esp_err_t foc_motor_enable(bool on)
{
    if (!s_motor) {
        return ESP_ERR_INVALID_STATE;
    }
    if (on) {
        s_motor->enable();
    } else {
        s_motor->disable();
    }
    return ESP_OK;
}

esp_err_t foc_motor_brake(void)
{
    if (!s_driver) {
        return ESP_ERR_INVALID_STATE;
    }
    /* PH/EN 模式下 setPwm(0,0) ⇒ 两相 EN=0 ⇒ 两个低边导通 = Brake。
     * 这正是 §10.3 要求的"唤醒后先切 brake 泄放反灌能量"。 */
    s_driver->setPwm(0.0f, 0.0f);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */

static void foc_loop_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(CONFIG_FOCSTEP_FOC_PERIOD_MS);
    uint32_t tick = 0;

    while (s_loop_run) {
        uint32_t t0 = (uint32_t)(esp_timer_get_time());

        s_motor->loopFOC();

        /* 堵转检测的采样必须在这里 (1kHz), 不能在主循环 (100ms) ——
         * 门机慢速时电气周期也只有 ~10ms, 100ms 采一个瞬时值是严重欠采样。
         * 这里只更新峰值包络, 判定与处置留给主循环 (不在此处做状态切换)。 */
        {
            float ma1 = 0.0f, ma2 = 0.0f;
            if (ipropi_read_both_ma(&ma1, &ma2) == ESP_OK) {
                ipropi_envelope_update(ma1, ma2);
            }
        }

        /* 位置/速度环降频 (没必要跟 1kHz) */
        if ((tick % CONFIG_FOCSTEP_MOTION_DOWNSAMPLE) == 0) {
            switch (s_mode) {
            case FOC_MODE_IDLE:
                /* Brake: 无力矩但保持阻尼。 */
                s_motor->target = s_motor->shaft_angle;
                s_motor->move();
                break;

            case FOC_MODE_TORQUE:
                /* 直接给力矩 —— 不走位置环, 否则会跟外力"较劲"。
                 * 应用层负责决定给多大力、什么方向 (例: 门的助动)。 */
                s_motor->move(s_torque_target);
                break;

            case FOC_MODE_VELOCITY:
                s_motor->move(s_velocity_target);
                break;

            case FOC_MODE_POSITION:
                s_motor->target = s_target_rad;
                s_motor->move();
                break;
            }
        }
        tick++;

        s_loop_us = (uint32_t)(esp_timer_get_time()) - t0;
        vTaskDelayUntil(&last, period);
    }

    s_loop_task = nullptr;
    vTaskDelete(nullptr);
}

esp_err_t foc_motor_start_loop(void)
{
    if (s_loop_task) {
        return ESP_OK;
    }
    s_loop_run = true;
    /* C6 只有一个应用核, 绑核无意义 —— 靠**优先级**抢占。
     * FOC 循环用最高优先级, 日志/命令台走低优先级。 */
    BaseType_t ok = xTaskCreate(foc_loop_task, "foc", 4096, nullptr,
                                configMAX_PRIORITIES - 1, &s_loop_task);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_FAIL, TAG, "foc task create failed");
    return ESP_OK;
}

void foc_motor_stop_loop(void)
{
    s_loop_run = false;
    /* 任务自己会 vTaskDelete */
}

/* ------------------------------------------------------------------ */

void foc_motor_set_mode(foc_mode_t mode)
{
    s_mode = mode;
}

foc_mode_t foc_motor_get_mode(void)
{
    return s_mode;
}

void foc_motor_move_to(float normalized)
{
    if (!s_motor) {
        return;
    }
    if (normalized < 0.0f) {
        normalized = 0.0f;
    }
    if (normalized > 1.0f) {
        normalized = 1.0f;
    }
    if (!s_position_trusted) {
        /* 位置不可信时拒绝执行位置指令 —— 否则会以错误的基准冲到机械限位。
         * 由 homing 流程负责先建立基准。 */
        ESP_LOGW(TAG, "位置不可信, 拒绝位置指令 (需先回零)");
        return;
    }
    /* 行程限位优先用回零自学习到的值 (存 NVS); 没有才退回 Kconfig 占位 */
    float min_a = (s_range_max > s_range_min) ? s_range_min
                                              : cfg_float(CONFIG_FOCSTEP_ENCODER_MIN_ANGLE, 0.0f);
    float max_a = (s_range_max > s_range_min) ? s_range_max
                                              : cfg_float(CONFIG_FOCSTEP_ENCODER_MAX_ANGLE, _2PI);
    s_target_rad = min_a + normalized * (max_a - min_a);
    s_mode = FOC_MODE_POSITION;
}

void foc_motor_stop(void)
{
    if (s_motor) {
        s_target_rad = s_motor->shaft_angle;
    }
    s_mode = FOC_MODE_IDLE;
    s_torque_target = 0.0f;
    s_velocity_target = 0.0f;
}

void foc_motor_set_torque(float volts)
{
    s_torque_target = volts;
    s_mode = FOC_MODE_TORQUE;
}

void foc_motor_set_velocity(float rad_s)
{
    s_velocity_target = rad_s;
    s_mode = FOC_MODE_VELOCITY;
}

void foc_motor_status(foc_motor_status_t *out)
{
    if (!out || !s_motor || !s_encoder) {
        return;
    }
    out->angle_rad = s_encoder->get_mech_angle();
    out->multi_turn_rad = s_encoder->get_multi_turn_angle();
    out->velocity = s_motor->shaft_velocity;
    out->target_rad = s_motor->target;
    out->turns = s_encoder->get_turns();
    out->loop_us = s_loop_us;
    out->enabled = s_motor->enabled;
    out->sensor_ok = (s_encoder->error_count() == 0);
}

esp_err_t foc_motor_align(void)
{
    if (!s_motor) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGW(TAG, "initFOC: 电机将转动以对齐电角 —— 确保门/负载是松开的");
    int rc = s_motor->initFOC();
    if (rc == 0) {
        ESP_LOGE(TAG, "initFOC failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "initFOC ok: sensor_direction=%d zero_elec=%.3f rad",
             (int)s_motor->sensor_direction, s_motor->zero_electric_angle);
    return ESP_OK;
}

esp_err_t foc_motor_characterise(float volts)
{
    if (!s_motor) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGW(TAG, "characteriseMotor(%.1fV): 电机会转动, 结果需人工回填 Kconfig", volts);
    int rc = s_motor->characteriseMotor(volts);
    if (rc == 0) {
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "phase_resistance=%.4f Ω, phase_inductance=%.6f H",
             s_motor->phase_resistance, s_motor->phase_inductance);
    return ESP_OK;
}

bool foc_motor_encoder_healthy(void)
{
    if (!s_encoder) {
        return false;
    }
    /* §10.4: 编码器失效 → 故障码 3。用累计 I2C 错误数做判据;
     * 阈值取 100 是为了容忍偶发总线噪声, 只在持续失败时判失效。 */
    return s_encoder->error_count() < 100;
}

void *foc_motor_encoder_handle(void)
{
    return static_cast<void *>(s_encoder);
}

esp_err_t foc_motor_encoder_set_mode(uint8_t cmd)
{
    if (!s_encoder) {
        return ESP_ERR_INVALID_STATE;
    }
    /* cmd 用 KTH5701_CMD_* 的字面量 (0x20=Wake-up&Sleep, 0x10=连续) */
    return s_encoder->set_mode((uint8_t)(cmd | KTH5701_AXIS_ALL));
}

/* ── 位置持久化 ────────────────────────────────────────────────
 * 存 **(圈数, 当时的单圈角)** 一对。
 *
 * 只存圈数不够: 深睡期间 C6 不在计数, 而手拉门是本产品的核心用法
 * ⇒ 多圈计数可能发散。存了单圈角, 醒来一比对就知道"睡着时动过没有"。
 */
esp_err_t foc_motor_save_position(void)
{
    if (!s_encoder) {
        return ESP_ERR_INVALID_STATE;
    }
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (ret != ESP_OK) {
        return ret;
    }
    int32_t t = s_encoder->get_turns();
    /* 存单圈角为 0.001 rad 单位的整数, 避免 NVS 浮点 blob 的字节序问题 */
    int32_t a = (int32_t)(s_encoder->get_mech_angle() * 1000.0f);

    ret = nvs_set_i32(h, NVS_KEY_TURNS, t);
    if (ret == ESP_OK) {
        ret = nvs_set_i32(h, NVS_KEY_ANGLE_MRAD, a);
    }
    /* 一并存行程限位与可信标志 */
    if (ret == ESP_OK) {
        ret = nvs_set_i32(h, NVS_KEY_RANGE_MIN, (int32_t)(s_range_min * 1000.0f));
    }
    if (ret == ESP_OK) {
        ret = nvs_set_i32(h, NVS_KEY_RANGE_MAX, (int32_t)(s_range_max * 1000.0f));
    }
    if (ret == ESP_OK) {
        ret = nvs_set_u8(h, NVS_KEY_TRUSTED, s_position_trusted ? 1 : 0);
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(h);
    }
    nvs_close(h);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "位置已存: turns=%" PRIi32 " angle=%.3f rad trusted=%d",
                 t, (double)s_encoder->get_mech_angle(), (int)s_position_trusted);
    }
    return ret;
}

esp_err_t foc_motor_restore_position(void)
{
    if (!s_encoder) {
        return ESP_ERR_INVALID_STATE;
    }
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "无历史位置 (首次上电) ⇒ 需要回零");
        s_position_trusted = false;
        return ESP_OK;
    }
    if (ret != ESP_OK) {
        return ret;
    }

    int32_t t = 0;
    int32_t a = 0;
    uint8_t trusted = 0;
    if (nvs_get_i32(h, NVS_KEY_TURNS, &t) != ESP_OK ||
        nvs_get_i32(h, NVS_KEY_ANGLE_MRAD, &a) != ESP_OK) {
        nvs_close(h);
        ESP_LOGW(TAG, "历史位置不完整 ⇒ 需要回零");
        s_position_trusted = false;
        return ESP_OK;
    }
    nvs_get_u8(h, NVS_KEY_TRUSTED, &trusted);
    int32_t rmin = 0, rmax = 0;
    bool has_range = (nvs_get_i32(h, NVS_KEY_RANGE_MIN, &rmin) == ESP_OK &&
                      nvs_get_i32(h, NVS_KEY_RANGE_MAX, &rmax) == ESP_OK);
    nvs_close(h);

    s_encoder->set_turns(t);

    /* ★ 一致性检查: 把"存的时候的绝对位置"与"现在的绝对位置"比。
     *   注意这里**不能**直接断言位置错了 —— 若睡着时移动不足半圈, 用
     *   恢复的圈数算出来的位置反而是对的。所以只在移动量超过阈值时判可疑。
     *   (根本歧义: 单圈绝对传感器分不清 0.3 圈与 1.3 圈, 见 foc_motor.h) */
    float saved_angle = (float)a / 1000.0f;
    float now_angle = s_encoder->get_mech_angle();
    float delta = fabsf(now_angle - saved_angle);
    if (delta > _2PI - delta) {
        delta = _2PI - delta; /* 取最小的角度差 */
    }

    if (has_range) {
        s_range_min = (float)rmin / 1000.0f;
        s_range_max = (float)rmax / 1000.0f;
    }

    if (!trusted) {
        s_position_trusted = false;
        ESP_LOGW(TAG, "历史位置标记为不可信 ⇒ 需要回零");
    } else if (delta > (float)CONFIG_FOCSTEP_POS_TRUST_TOL_MRAD / 1000.0f) {
        s_position_trusted = false;
        ESP_LOGW(TAG, "单圈角变化 %.3f rad (超容差 %d mrad) ⇒ 睡着时被动过, "
                      "多圈计数不可信, 需要回零",
                 (double)delta, CONFIG_FOCSTEP_POS_TRUST_TOL_MRAD);
    } else {
        s_position_trusted = true;
        ESP_LOGI(TAG, "位置已恢复: turns=%" PRIi32 " angle=%.3f rad (偏差 %.4f rad) ✓",
                 t, (double)now_angle, (double)delta);
    }
    return ESP_OK;
}

bool foc_motor_position_trusted(void)
{
    return s_position_trusted;
}

bool foc_motor_position_ready(void)
{
    /* 编码器在且没持续出错; 电机对象已建好。
     * 注意不要求 initFOC 已跑过 —— 回零本身不依赖电角对齐的精度,
     * 但为安全起见仍需电机能正常出力。 */
    return (s_motor != nullptr && s_encoder != nullptr && s_encoder->error_count() < 100);
}

void foc_motor_invalidate_position(const char *reason)
{
    if (s_position_trusted) {
        ESP_LOGW(TAG, "位置标记为不可信: %s", reason ? reason : "?");
    }
    s_position_trusted = false;
}

void foc_motor_set_target_rad(float rad)
{
    s_target_rad = rad;
    s_mode = FOC_MODE_POSITION;
}

float foc_motor_get_target_rad(void)
{
    return s_target_rad;
}

float foc_motor_get_angle_rad(void)
{
    return s_encoder ? s_encoder->get_multi_turn_angle() : 0.0f;
}

int32_t foc_motor_turns(void)
{
    return s_encoder ? s_encoder->get_turns() : 0;
}

void foc_motor_set_voltage_limit(float volts)
{
    if (s_motor) {
        s_motor->voltage_limit = volts;
    }
    if (s_driver && volts < s_driver->voltage_limit) {
        s_driver->voltage_limit = volts;
    }
}

float foc_motor_get_voltage_limit(void)
{
    return s_motor ? s_motor->voltage_limit : 0.0f;
}

float foc_motor_default_voltage_limit(void)
{
    return cfg_float(CONFIG_FOCSTEP_VOLTAGE_LIMIT, 12.0f);
}

void foc_motor_set_travel_range(float min_rad, float max_rad)
{
    s_range_min = min_rad;
    s_range_max = max_rad;
    ESP_LOGI(TAG, "行程限位: [%.3f, %.3f] rad", (double)min_rad, (double)max_rad);
}

void foc_motor_get_travel_range(float *min_rad, float *max_rad)
{
    if (min_rad) {
        *min_rad = s_range_min;
    }
    if (max_rad) {
        *max_rad = s_range_max;
    }
}
