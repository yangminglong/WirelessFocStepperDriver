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
#include "freertos/semphr.h" /* 协作式暂停用的两把信号量 (见 foc_motor_pause_loop) */

/* esp_simplefoc / arduino-foc */
#include "esp_simplefoc.h"
#include "StepperMotor.h"
#include "drivers/StepperDriver2PWM.h"
#include "common/foc_utils.h"

/* kth5701.h 是 C++ 头, 本文件是 C++ */
#include "kth5701.h"
#include "ipropi_current_sense.h"
#include "ipropi_sense.h"
#include "vref_dac.h" /* 动态 VREF: ITRIP 档 (foc_motor_set_itrip) */

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
static volatile bool s_loop_resync = false; /* resume 后重置时间基 (轻睡档) */
static volatile foc_mode_t s_mode = FOC_MODE_IDLE;

/* ── 协作式暂停 (轻睡档 / 无线监听档用) ────────────────────────
 
 *    在那里被挂起会把锁一直攥着, 别人再去访问 I2C (监听档醒来清 INT 就要读)
 *    就阻塞/超时 —— INT 清不掉 ⇒ 反复唤醒。
 * 改法: 立标志 → 循环在**安全点**(loopFOC 之前, 不持任何锁)自己交还控制权 →
 *    暂停方用信号量确认; 恢复时给另一把信号量放行。 */
static volatile bool s_loop_pause_req = false;
static volatile bool s_loop_paused = false;
static bool s_paused_by_suspend = false; /* 兜底路径 (循环没按时响应) */
static SemaphoreHandle_t s_loop_pause_sem = nullptr;  /* 循环 → 暂停方: 我已停 */
static SemaphoreHandle_t s_loop_resume_sem = nullptr; /* 暂停方 → 循环: 继续 */

static volatile float s_torque_target = 0.0f;   /* FOC_MODE_TORQUE 用 */
static volatile float s_velocity_target = 0.0f; /* FOC_MODE_VELOCITY 用 */
static volatile float s_target_rad = 0.0f;
static volatile uint32_t s_loop_us = 0;

#define NVS_NS  "focstep"
#define NVS_KEY_TURNS      "turns"
#define NVS_KEY_ANGLE_MRAD "angle_mrad" /* 存盘时的单圈角, 毫弧度 */
/* 行程: 两个**绝对角**端点, 各自独立存 —— 所以"重标一端"不会动到另一端 */
#define NVS_KEY_RANGE_ZERO "rzero_mrad"
#define NVS_KEY_RANGE_END  "rend_mrad"
#define NVS_KEY_RANGE_ST   "rstate"    /* range_state_t */
/* 兼容键: 早期固件按"角度大小"排的 min/max —— 只做开机清理, 不读写 */
#define NVS_KEY_RANGE_MIN_OLD "rmin_mrad"
#define NVS_KEY_RANGE_MAX_OLD "rmax_mrad"
/* 标定行程时的"指纹": 行程建立在编码器方向与零位之上, 这两个 Kconfig 一改,
 * 旧行程就不再对应机械端点 (见 restore 里的一致性检查) */
#define NVS_KEY_ENC_DIR    "renc_dir"  /* ENCODER_DIRECTION 的值 */
#define NVS_KEY_ENC_ZERO   "renc_zero" /* ENCODER_ZERO_OFFSET, mrad */
#define NVS_KEY_TRUSTED    "trusted"

/* 行程 (零点 + 满行程点, 见 foc_motor.h) 与位置可信度 */
static float s_zero_rad = 0.0f;      /* 0% 开度 */
static float s_end_rad = 0.0f;       /* 100% 开度 */
static range_state_t s_range_state = RANGE_NONE;
static bool s_position_trusted = false;

/* 轻睡档: 睡前的单圈角。**只存 RAM** —— 轻睡不丢内存, 没必要写 NVS (省一次 flash 写)。 */
static float s_sleep_angle = 0.0f;
static bool s_sleep_angle_valid = false;

/* 两角之间的最小差 (0~π)。restore(NVS) 与轻睡醒来(RAM) 共用同一判据。 */
static float angle_delta(float a, float b)
{
    float d = fabsf(a - b);
    return (d > _2PI - d) ? (_2PI - d) : d;
}

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
        /* ★ 协作式暂停点: 必须在 loopFOC() **之前** —— 此处不持任何锁。
         *  */
        if (s_loop_pause_req) {
            s_loop_paused = true;
            s_loop_resync = true; /* 放行后重置时间基 */
            if (s_loop_pause_sem) {
                xSemaphoreGive(s_loop_pause_sem);
            }
            if (s_loop_resume_sem) {
                xSemaphoreTake(s_loop_resume_sem, portMAX_DELAY);
            }
            s_loop_paused = false;
            last = xTaskGetTickCount();
            continue;
        }

        /* 轻睡醒来: 时间基必须重置 —— 直接调 esp_light_sleep_start() 时 IDF 不补偿
         * FreeRTOS tick, 不重置的话 vTaskDelayUntil 会连续追打 (见 foc_motor_pause_loop)。 */
        if (s_loop_resync) {
            last = xTaskGetTickCount();
            s_loop_resync = false;
        }

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
    /* 协作式暂停用的两把信号量 (见 foc_motor_pause_loop) */
    if (!s_loop_pause_sem) {
        s_loop_pause_sem = xSemaphoreCreateBinary();
        s_loop_resume_sem = xSemaphoreCreateBinary();
        ESP_RETURN_ON_FALSE(s_loop_pause_sem && s_loop_resume_sem, ESP_ERR_NO_MEM,
                            TAG, "pause sem create failed");
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
    /* 应用语义上的开度 = 夹在 [0,1] 之内 (0 = 全关 / 1 = 全开)。
     * 需要越过零点或超出满行程点 (探边/调试) 走 move_to_ext()。 */
    if (normalized < 0.0f) {
        normalized = 0.0f;
    }
    if (normalized > 1.0f) {
        normalized = 1.0f;
    }
    (void)foc_motor_move_to_ext(normalized);
}

esp_err_t foc_motor_move_to_ext(float normalized)
{
    if (!s_motor) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_position_trusted) {
        /* 位置不可信时拒绝执行位置指令 —— 否则会以错误的基准冲到机械限位。
         * 由 homing 流程负责先建立基准。 */
        ESP_LOGW(TAG, "位置不可信, 拒绝位置指令 (需先回零)");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_range_state != RANGE_BOTH) {
        /* ★ "全开/全关"只在**零点 + 满行程点都标定**后才生效 —— 缺一个就不做任何
         *   开度推断: 占位角与实际机构无关, 照它跑等于用错误基准冲机械限位。 */
        ESP_LOGW(TAG, "行程不完整 (%s) ⇒ 拒绝开度指令: 先标零点与满行程点",
                 foc_motor_range_state_str(s_range_state));
        return ESP_ERR_INVALID_STATE;
    }
    if (normalized < 0.0f || normalized > 1.0f) {
        ESP_LOGW(TAG, "★ 越界开度 %.3f ⇒ 越过零点/超出满行程点, 机构会顶到机械限位",
                 (double)normalized);
    }
    /* span 带符号 ⇒ 反装机构 ("零点"的角比"满行程点"更大) 不需要任何特殊处理 */
    s_target_rad = s_zero_rad + normalized * (s_end_rad - s_zero_rad);
    s_mode = FOC_MODE_POSITION;
    return ESP_OK;
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

/* ── 轻睡档支持 (Kconfig FOCSTEP_SLEEP_MODE_LIGHT) ────────────── */

void foc_motor_pause_loop(void)
{
    if (!s_loop_task || s_loop_paused) {
        return;
    }
    s_loop_pause_req = true;

    /* 等循环自己走到安全点并交还控制权 (最多 ~2 个 FOC 周期)。
     * 等不到说明它卡在 I2C 里了 —— 此时 vTaskSuspend 只是兜底,
     * 并且值得打一条警告 (清 INT 可能因此失败)。 */
    if (s_loop_pause_sem &&
        xSemaphoreTake(s_loop_pause_sem, pdMS_TO_TICKS(50)) == pdTRUE) {
        ESP_LOGI(TAG, "FOC 任务已暂停 (协作式, 不持锁)");
    } else {
        ESP_LOGW(TAG, "FOC 循环 50ms 未响应, 兜底挂起 (可能正卡在 I2C)");
        vTaskSuspend(s_loop_task);
        s_paused_by_suspend = true;
        s_loop_paused = true;
    }
}

void foc_motor_resume_loop(void)
{
    if (!s_loop_task || !s_loop_paused) {
        return;
    }
    if (s_paused_by_suspend) {
        s_paused_by_suspend = false;
        s_loop_pause_req = false;
        s_loop_paused = false;
        vTaskResume(s_loop_task);
    } else {
        s_loop_pause_req = false;
        if (s_loop_resume_sem) {
            xSemaphoreGive(s_loop_resume_sem); /* 循环自己继续, 并重置时间基 */
        }
    }
    ESP_LOGI(TAG, "FOC 任务已恢复 (时间基将重置)");
}

bool foc_motor_encoder_clear_int(void)
{
    if (!s_encoder) {
        return false;
    }
    /* 手册语义: **读一次数据即清零** —— 读测量帧就是"清锁存"的动作 */
    uint8_t buf[KTH5701_XY_FRAME_LEN];
    size_t len = sizeof(buf);
    esp_err_t ret = s_encoder->data_read(KTH5701_AXIS_XY, buf, &len);

    vTaskDelay(pdMS_TO_TICKS(2)); /* 给芯片时间把 INT 拉低 */
    bool low = (gpio_get_level((gpio_num_t)PIN_ENCODER_INT) == 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "读数据清 INT 失败: %s (INT=%d)", esp_err_to_name(ret), (int)!low);
    }
    return low;
}

void foc_motor_mark_sleep_angle(void)
{
    if (!s_encoder) {
        return;
    }
    s_sleep_angle = s_encoder->get_mech_angle();
    s_sleep_angle_valid = true;
    ESP_LOGI(TAG, "睡前单圈角 %.3f rad (轻睡醒来比对用, 不写 NVS)", (double)s_sleep_angle);
}

esp_err_t foc_motor_check_sleep_angle(void)
{
    if (!s_encoder) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_sleep_angle_valid) {
        ESP_LOGW(TAG, "没有睡前角度记录, 跳过轻睡一致性检查");
        return ESP_ERR_INVALID_STATE;
    }
    float delta = angle_delta(s_encoder->get_mech_angle(), s_sleep_angle);
    s_sleep_angle_valid = false;

    if (delta > (float)CONFIG_FOCSTEP_POS_TRUST_TOL_MRAD / 1000.0f) {
        s_position_trusted = false;
        ESP_LOGW(TAG, "轻睡期间单圈角变化 %.3f rad (超容差 %d mrad) ⇒ 多圈计数不可信, 需要回零",
                 (double)delta, CONFIG_FOCSTEP_POS_TRUST_TOL_MRAD);
    } else {
        ESP_LOGI(TAG, "轻睡期间单圈角偏差 %.4f rad ✓ 位置仍可信 (turns=%" PRIi32 ")",
                 (double)delta, s_encoder->get_turns());
    }
    return ESP_OK;
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
    /* 一并存行程 (两个绝对角 + 三态)、它的"指纹"(方向+零位)与可信标志 */
    if (ret == ESP_OK) {
        ret = nvs_set_i32(h, NVS_KEY_RANGE_ZERO, (int32_t)(s_zero_rad * 1000.0f));
    }
    if (ret == ESP_OK) {
        ret = nvs_set_i32(h, NVS_KEY_RANGE_END, (int32_t)(s_end_rad * 1000.0f));
    }
    if (ret == ESP_OK) {
        ret = nvs_set_u8(h, NVS_KEY_RANGE_ST, (uint8_t)s_range_state);
    }
    /* 顺手清掉两个兼容键 (按角度排序的 min/max): 现行口径是 (零点, 满行程点),
     * 留着只会让后来的人误读。不存在时返回 NOT_FOUND, 属正常 —— 别让它冲掉 ret。 */
    (void)nvs_erase_key(h, NVS_KEY_RANGE_MIN_OLD);
    (void)nvs_erase_key(h, NVS_KEY_RANGE_MAX_OLD);
    if (ret == ESP_OK) {
        ret = nvs_set_i8(h, NVS_KEY_ENC_DIR, (int8_t)CONFIG_FOCSTEP_ENCODER_DIRECTION);
    }
    if (ret == ESP_OK) {
        ret = nvs_set_i32(h, NVS_KEY_ENC_ZERO,
                          (int32_t)(cfg_float(CONFIG_FOCSTEP_ENCODER_ZERO_OFFSET, 0.0f) * 1000.0f));
    }
    if (ret == ESP_OK) {
        ret = nvs_set_u8(h, NVS_KEY_TRUSTED, s_position_trusted ? 1 : 0);
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(h);
    }
    nvs_close(h);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "位置已存: turns=%" PRIi32 " angle=%.3f rad trusted=%d 行程: 零点 %.3f / "
                      "满行程点 %.3f (%s)",
                 t, (double)s_encoder->get_mech_angle(), (int)s_position_trusted,
                 (double)s_zero_rad, (double)s_end_rad, foc_motor_range_state_str(s_range_state));
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
    uint8_t rstate = (uint8_t)RANGE_NONE;
    int32_t rzero = 0, rend = 0;
    nvs_get_u8(h, NVS_KEY_RANGE_ST, &rstate);
    /* 三态非 NONE 才算有行程; 两个端点必须都读得到, 否则当未标定处理 */
    bool has_range = (rstate != (uint8_t)RANGE_NONE &&
                      nvs_get_i32(h, NVS_KEY_RANGE_ZERO, &rzero) == ESP_OK &&
                      nvs_get_i32(h, NVS_KEY_RANGE_END, &rend) == ESP_OK);
    /* 行程的"指纹": 标定时用的编码器方向与零位 */
    int8_t saved_dir = 0;
    int32_t saved_zero = 0;
    bool has_meta = (nvs_get_i8(h, NVS_KEY_ENC_DIR, &saved_dir) == ESP_OK &&
                     nvs_get_i32(h, NVS_KEY_ENC_ZERO, &saved_zero) == ESP_OK);
    nvs_close(h);

    s_encoder->set_turns(t);

    /* ★ 一致性检查: 把"存的时候的绝对位置"与"现在的绝对位置"比。
     *   注意这里**不能**直接断言位置错了 —— 若睡着时移动不足半圈, 用
     *   恢复的圈数算出来的位置反而是对的。所以只在移动量超过阈值时判可疑。
     *   (根本歧义: 单圈绝对传感器分不清 0.3 圈与 1.3 圈, 见 foc_motor.h) */
    float saved_angle = (float)a / 1000.0f;
    float now_angle = s_encoder->get_mech_angle();
    float delta = angle_delta(now_angle, saved_angle);

    if (has_range) {
        /* ★★ 行程一致性检查 (比单圈角检查更硬): 行程是"编码器机械角坐标系"里的端点,
         *    而该坐标系由 ENCODER_DIRECTION + ENCODER_ZERO_OFFSET 定义。
         *    这两个值一改 (固件重编/换装磁铁/重装机构), 旧行程就不再对应机械端点。
         *    这里直接**作废**, 而不是静默沿用 —— 否则会"能跑但整段错位",
         *    现场极难发现。作废后必须重新标定 (learn / mark)。 */
        const int8_t cur_dir = (int8_t)CONFIG_FOCSTEP_ENCODER_DIRECTION;
        const int32_t cur_zero =
            (int32_t)(cfg_float(CONFIG_FOCSTEP_ENCODER_ZERO_OFFSET, 0.0f) * 1000.0f);
        if (!has_meta || saved_dir != cur_dir || saved_zero != cur_zero) {
            ESP_LOGE(TAG, "行程作废: 标定时 dir=%d zero=%d mrad, 现在 dir=%d zero=%d mrad "
                          "⇒ 编码器方向/零位变过, 旧的行程不再对应机械端点, 必须重标定",
                     (int)saved_dir, (int)saved_zero, (int)cur_dir, (int)cur_zero);
            s_range_state = RANGE_NONE;
            s_zero_rad = 0.0f;
            s_end_rad = 0.0f;
            s_position_trusted = false;
            return ESP_OK; /* 行程 + 位置都不可用, 等重新标定 */
        }
        s_zero_rad = (float)rzero / 1000.0f;
        s_end_rad = (float)rend / 1000.0f;
        s_range_state = (range_state_t)rstate;
        /* BOTH 但两端几乎重合 ⇒ NVS 内容自相矛盾 (旧固件残留 / 写入中断), 作废重标 */
        if (s_range_state == RANGE_BOTH &&
            fabsf(s_end_rad - s_zero_rad) < FOC_MOTOR_MIN_SPAN_RAD) {
            ESP_LOGE(TAG, "行程作废: 恢复出的零点/满行程点几乎重合 (%.3f / %.3f rad) "
                          "⇒ NVS 内容不一致, 必须重标定",
                     (double)s_zero_rad, (double)s_end_rad);
            s_range_state = RANGE_NONE;
            s_zero_rad = 0.0f;
            s_end_rad = 0.0f;
            s_position_trusted = false;
            return ESP_OK;
        }
        ESP_LOGI(TAG, "行程已恢复: 零点 %.3f / 满行程点 %.3f rad (span %.3f, %s, dir=%d zero=%d mrad)",
                 (double)s_zero_rad, (double)s_end_rad, (double)(s_end_rad - s_zero_rad),
                 foc_motor_range_state_str(s_range_state), (int)saved_dir, (int)saved_zero);
    } else if (rstate != (uint8_t)RANGE_NONE) {
        ESP_LOGW(TAG, "行程状态是 %u 但端点读不齐 ⇒ 按未标定处理, 需要重标", (unsigned)rstate);
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

/* ── 动态 VREF: 运行时改 ITRIP 档 ──────────────────────────────────
 * 功能只做两件 vref_dac 做不到的事, 其余 (裕量换算/上下界/I2C/ADC 量程跟随)
 * 全在平台层:
 *   ① 降档守卫 —— 只有本层知道电机是不是正在使能;
 *   ② 堵转阈值告警 —— 阈值是本层配下去的 (ipropi_stall_config)。
 */
static esp_err_t apply_itrip(float itrip_a, const char *why)
{
    if (s_motor->enabled && itrip_a < vref_dac_itrip_a()) {
        /* 降档会让 DRV 立刻按新阈值斩波 (力矩台阶); 更糟的是电流环若仍在要求更大的
         * 电流, IPROPI 会顶在钳位上 ⇒ 堵转与力矩判据一起失灵。升档没有这个问题。 */
        ESP_LOGE(TAG, "电机使能中不许降 ITRIP (%.2fA → %.2fA, %s): 先 foc_motor_enable(false)",
                 (double)vref_dac_itrip_a(), (double)itrip_a, why);
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = vref_dac_set(itrip_a); /* 内部连 ADC 量程一起跟过去 */
    if (ret != ESP_OK) {
        return ret;
    }
    float stall_a = (float)CONFIG_FOCSTEP_STALL_CURRENT_MA * 0.001f;
    if (itrip_a < stall_a) {
        ESP_LOGW(TAG, "ITRIP %.2fA < 堵转阈值 %.2fA: 该档下 IPROPI 先被钳位, 堵转判定不会触发",
                 (double)itrip_a, (double)stall_a);
    }
    return ESP_OK;
}

esp_err_t foc_motor_set_itrip(float peak_a)
{
    if (!s_motor) {
        return ESP_ERR_INVALID_STATE;
    }
    /* 留不出钳位裕量就**不设档** (平台层同样拒绝, 这里先拦一道给出更清楚的日志) */
    if (!vref_dac_peak_supported(peak_a)) {
        ESP_LOGE(TAG, "峰值 %.2fA 留不出 %.2f× 钳位裕量 (上限 %.2fA): 不写 DAC",
                 (double)peak_a, (double)VREF_DAC_CLAMP_MARGIN, (double)VREF_DAC_MAX_ITRIP_A);
        return ESP_ERR_INVALID_ARG;
    }
    return apply_itrip(vref_dac_itrip_for_peak(peak_a), "峰值档");
}

esp_err_t foc_motor_set_itrip_default(void)
{
    if (!s_motor) {
        return ESP_ERR_INVALID_STATE;
    }
    return apply_itrip(vref_dac_default_itrip_a(), "Kconfig 默认档");
}

float foc_motor_get_itrip(void)
{
    return vref_dac_itrip_a();
}

const char *foc_motor_range_state_str(range_state_t st)
{
    switch (st) {
    case RANGE_NONE:
        return "未标定";
    case RANGE_ZERO_ONLY:
        return "只有零点";
    case RANGE_BOTH:
        return "零点+满行程点";
    default:
        return "?";
    }
}

range_state_t foc_motor_range_state(void)
{
    return s_range_state;
}

bool foc_motor_has_range(void)
{
    /* "有行程" = 完整标定 (BOTH)。只有零点时开度无从定义, 不算有行程 */
    return (s_range_state == RANGE_BOTH);
}

/*
 * 行程写入的唯一入口 (set_zero / set_end 共用)。
 *
 * ★ 决定 #6: **只改被标定的那一端, 另一端的物理位置保持不动。**
 *   存的是两个绝对角 ⇒ 重标零点时满行程点的值自然"基于新零点重新计算"
 *   (span 是算出来的, 不是存下来的)。
 *
 * 校验: 跨度必须 ≥ FOC_MOTOR_MIN_SPAN_RAD, 否则判定标错了端 (或方向不对),
 *       拒绝写入 —— 什么都不改, 旧的标定继续有效。
 */
static esp_err_t range_commit(bool mark_zero, float rad)
{
    if (!mark_zero && s_range_state == RANGE_NONE) {
        /* 三态里没有"有行程无零点" —— 没有零点, 100% 开度就无从定义 */
        ESP_LOGE(TAG, "还没有零点 ⇒ 拒绝先标满行程点 (行程得有零点才是行程)");
        return ESP_ERR_INVALID_STATE;
    }

    float zero, end;
    range_state_t next;
    if (mark_zero) {
        zero = rad;
        /* 已有满行程点时它**不动**; 否则另一端还不存在 (跨度无从谈起) */
        end = (s_range_state == RANGE_BOTH) ? s_end_rad : rad;
        next = (s_range_state == RANGE_BOTH) ? RANGE_BOTH : RANGE_ZERO_ONLY;
    } else {
        zero = s_zero_rad;
        end = rad;
        next = RANGE_BOTH;
    }

    if (next == RANGE_BOTH && fabsf(end - zero) < FOC_MOTOR_MIN_SPAN_RAD) {
        ESP_LOGE(TAG, "标定被拒: 零点 %.3f / 满行程点 %.3f 跨度 %.3f < 最小 %.3f ⇒ "
                      "两端标反了或机构有问题 (本次不做任何修改)",
                 (double)zero, (double)end, (double)fabsf(end - zero),
                 (double)FOC_MOTOR_MIN_SPAN_RAD);
        return ESP_ERR_INVALID_ARG;
    }

    /* ★ 方向翻转告警: 原本已完整、这次落定却把 span 变号 ⇒ 0%/100% 与门开合的
     *   对应关系被翻了过来 —— 那正是 dir invert 的语义。标定是有意的人为动作,
     *   所以**只告警不拦** (误拦比告警更烦), 但必须让人看见。 */
    if (s_range_state == RANGE_BOTH && next == RANGE_BOTH) {
        float old_span = s_end_rad - s_zero_rad;
        float new_span = end - zero;
        if ((old_span > 0.0f) != (new_span > 0.0f)) {
            ESP_LOGW(TAG, "⚠️ 本次标定把行程方向翻转了: span %.3f → %.3f ⇒ "
                          "0%%/100%% 与门开合的对应关系随之互换。若非本意请复核方向; "
                          "有意翻转建议改用 dir invert (等价且更直白)",
                     (double)old_span, (double)new_span);
        }
    }

    s_zero_rad = zero;
    s_end_rad = end;
    s_range_state = next;

    /* ★ 标定即锚定: 被标的这一点是**实物基准点**, 所以此刻位置可信 (决定 #2)。
     *   必须显式置位 —— 旧代码只把行程存进 NVS, 可信标志仍是 false, 于是
     *   "标完还得复位一次让 restore 把可信标志读回来" (隐藏坑)。 */
    s_position_trusted = true;
    /* 停在原地保持, 并立刻落盘 (圈数 + 单圈角 + 行程 + 指纹 + 可信标志) */
    foc_motor_set_target_rad(foc_motor_get_angle_rad());
    foc_motor_save_position();

    ESP_LOGW(TAG, "已标定%s = %.3f rad ⇒ 零点 %.3f / 满行程点 %.3f (span %.3f, %s)",
             mark_zero ? "零点(0%)" : "满行程点(100%)", (double)rad,
             (double)s_zero_rad, (double)s_end_rad, (double)(s_end_rad - s_zero_rad),
             foc_motor_range_state_str(s_range_state));
    return ESP_OK;
}

esp_err_t foc_motor_set_zero(float rad)
{
    return range_commit(true, rad);
}

esp_err_t foc_motor_set_end(float rad)
{
    return range_commit(false, rad);
}

esp_err_t foc_motor_invert_travel(void)
{
    if (s_range_state != RANGE_BOTH) {
        ESP_LOGE(TAG, "行程不完整 (%s) ⇒ 不能反向 (反向 = 交换零点与满行程点)",
                 foc_motor_range_state_str(s_range_state));
        return ESP_ERR_INVALID_STATE;
    }
    float tmp = s_zero_rad;
    s_zero_rad = s_end_rad;
    s_end_rad = tmp;
    foc_motor_save_position();

    ESP_LOGW(TAG, "行程已反向: 新零点 %.3f (原满行程点) / 新满行程点 %.3f (原零点), span %.3f "
                  "⇒ 0%%/100%% 与正方向同时取反",
             (double)s_zero_rad, (double)s_end_rad, (double)(s_end_rad - s_zero_rad));
    return ESP_OK;
}

void foc_motor_get_range(float *zero_rad, float *end_rad, float *span_rad)
{
    if (zero_rad) {
        *zero_rad = s_zero_rad;
    }
    if (end_rad) {
        *end_rad = s_end_rad;
    }
    if (span_rad) {
        *span_rad = s_end_rad - s_zero_rad; /* 带符号 */
    }
}
