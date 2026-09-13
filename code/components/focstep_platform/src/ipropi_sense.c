#include "ipropi_sense.h"
#include "board_pins.h"
#include "platform_events.h"

#include <math.h>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "IPROPI";

/* Kconfig 提供标定与阈值 */
#define CFG_R_IPROPI_OHM   CONFIG_FOCSTEP_R_IPROPI_OHM
#define CFG_A_IPROPI       CONFIG_FOCSTEP_A_IPROPI_UA_PER_A

/* GPIO4 → ADC1_CH4, GPIO5 → ADC1_CH5 (doc.md §四) */
static const adc_channel_t s_chan[IPROPI_PHASE_COUNT] = {
    ADC_CHANNEL_4,
    ADC_CHANNEL_5,
};

static adc_oneshot_unit_handle_t s_adc = NULL;
static adc_cali_handle_t s_cali = NULL;
static bool s_inited = false;

/* 堵转判定状态 */
static float s_stall_ma = 1200.0f;
static uint32_t s_stall_ms = 300;
static TickType_t s_stall_since = 0;
static bool s_stall_active = false;

/* 峰值包络: FOC 循环写, 主循环读并清零 */
static volatile float s_env_peak = 0.0f;

static adc_atten_t kconfig_atten(void)
{
#if CONFIG_FOCSTEP_ADC_ATTEN_DB == 0
    return ADC_ATTEN_DB_0;
#elif CONFIG_FOCSTEP_ADC_ATTEN_DB == 12
    return ADC_ATTEN_DB_12;
#elif CONFIG_FOCSTEP_ADC_ATTEN_DB == 6
    return ADC_ATTEN_DB_6;
#else
    return ADC_ATTEN_DB_12;
#endif
}

esp_err_t ipropi_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &s_adc), TAG, "adc unit new failed");

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = kconfig_atten(),
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    for (int i = 0; i < IPROPI_PHASE_COUNT; i++) {
        ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, s_chan[i], &chan_cfg),
                            TAG, "adc chan %d config failed", (int)s_chan[i]);
    }

    /* 标定: 曲线拟合优先, 退而求其次用线性拟合; C6 两者之一必然可用 */
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = kconfig_atten(),
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali) == ESP_OK) {
        ESP_LOGI(TAG, "calibration: curve fitting");
    }
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (s_cali == NULL) {
        adc_cali_line_fitting_config_t cali_cfg = {
            .unit_id = ADC_UNIT_1,
            .atten = kconfig_atten(),
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        if (adc_cali_create_scheme_line_fitting(&cali_cfg, &s_cali) == ESP_OK) {
            ESP_LOGI(TAG, "calibration: line fitting");
        }
    }
#endif
    if (s_cali == NULL) {
        /* 没标定也能跑, 只是绝对精度差。堵转判定是相对量, 影响有限。 */
        ESP_LOGW(TAG, "no ADC calibration scheme available, readings will be uncalibrated");
    }

    s_inited = true;
    ESP_LOGI(TAG, "init done: R_IPROPI=%dΩ A_IPROPI=%dµA/A atten=%ddB",
             CFG_R_IPROPI_OHM, CFG_A_IPROPI, CONFIG_FOCSTEP_ADC_ATTEN_DB);
    return ESP_OK;
}

void ipropi_deinit(void)
{
    if (s_cali) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_delete_scheme_curve_fitting(s_cali);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        adc_cali_delete_scheme_line_fitting(s_cali);
#endif
        s_cali = NULL;
    }
    if (s_adc) {
        adc_oneshot_del_unit(s_adc);
        s_adc = NULL;
    }
    s_inited = false;
}

esp_err_t ipropi_read_mv(int phase, int *mv)
{
    if (!s_inited || phase < 0 || phase >= IPROPI_PHASE_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    int raw = 0;
    esp_err_t ret = adc_oneshot_read(s_adc, s_chan[phase], &raw);
    if (ret != ESP_OK) {
        return ret;
    }
    if (s_cali) {
        return adc_cali_raw_to_voltage(s_cali, raw, mv);
    }
    /* 无标定时的粗略换算 (12bit, 12dB 满量程约 3100mV) */
    *mv = raw * 3100 / 4095;
    return ESP_OK;
}

float ipropi_read_ma(int phase)
{
    int mv = 0;
    if (ipropi_read_mv(phase, &mv) != ESP_OK) {
        return -1.0f;
    }
    /* V = I × A_IPROPI × R  ⇒  I(mA) = mV / (R_IPROPI × A_IPROPI(µA/A)) × 1000
     *   量纲: mV / (Ω × µA/A) = mV / (mV/A × 1e-6 × 1e3) ... 直接展开:
     *   A_IPROPI 单位 µA/A ⇒ I[A] × A_IPROPI[µA/A] = I_IPROPI[µA]
     *   V[mV] = I_IPROPI[µA] × R[Ω] / 1000
     *   ⇒ I[mA] = mV × 1000 / (A_IPROPI × R) × 1000 / 1000
     * 化简: I[mA] = mV / (A_IPROPI × R_IPROPI) × 1e6
     */
    float denom = (float)CFG_A_IPROPI * (float)CFG_R_IPROPI_OHM; /* µA/A × Ω */
    if (denom <= 0.0f) {
        return -1.0f;
    }
    return (float)mv * 1e6f / denom;
}

esp_err_t ipropi_read_both_ma(float *ma1, float *ma2)
{
    int mv1 = 0, mv2 = 0;
    esp_err_t r1 = ipropi_read_mv(0, &mv1);
    esp_err_t r2 = ipropi_read_mv(1, &mv2);
    if (r1 != ESP_OK || r2 != ESP_OK) {
        return (r1 != ESP_OK) ? r1 : r2;
    }
    float denom = (float)CFG_A_IPROPI * (float)CFG_R_IPROPI_OHM;
    if (ma1) {
        *ma1 = (float)mv1 * 1e6f / denom;
    }
    if (ma2) {
        *ma2 = (float)mv2 * 1e6f / denom;
    }
    return ESP_OK;
}

float ipropi_magnitude_ma(float ma1, float ma2)
{
    /* 电流矢量幅值 = √(ia² + ib²)。两相正交, 所以它恒等于相电流幅值,
     * 与电角度无关 —— 这正是堵转判据该用的量。 */
    return sqrtf(ma1 * ma1 + ma2 * ma2);
}

void ipropi_envelope_update(float ma1, float ma2)
{
    /* 峰值保持。由 FOC 循环 (1kHz) 调用, 主循环读走并清零。
     * 单精度 float 的读写在本平台是原子的, 且只有 FOC 任务写、主循环读,
     * 故无需加锁 —— 最坏情况是读到一次略微陈旧的值。 */
    float mag = ipropi_magnitude_ma(ma1, ma2);
    if (mag > s_env_peak) {
        s_env_peak = mag;
    }
}

float ipropi_envelope_take(void)
{
    float v = s_env_peak;
    s_env_peak = 0.0f;
    return v;
}

void ipropi_stall_config(float stall_current_ma, uint32_t stall_ms)
{
    s_stall_ma = stall_current_ma;
    s_stall_ms = stall_ms;
}

bool ipropi_stall_update(float peak_ma)
{
    TickType_t now = xTaskGetTickCount();

    /* 已锁存则直接返回, 不再重复计数 —— 避免保持力矩期间反复触发 */
    if (s_stall_active) {
        return true;
    }

    if (peak_ma < s_stall_ma) {
        s_stall_since = 0;
        return false;
    }

    if (s_stall_since == 0) {
        s_stall_since = now;
        return false;
    }

    if ((now - s_stall_since) >= pdMS_TO_TICKS(s_stall_ms)) {
        s_stall_active = true;
        ESP_LOGW(TAG, "STALL: 电流峰值 %.0f mA 持续超过 %.0f mA 达 %" PRIu32 "ms",
                 peak_ma, s_stall_ma, s_stall_ms);
        /* 只发事件, 不停车 —— 怎么处置是应用的事 (门机: 停车保持力矩) */
        focstep_evt_stall_t e = {.base.timestamp_ms = platform_now_ms(), .peak_ma = peak_ma};
        platform_event_post(FOCSTEP_EVT_STALL, &e, sizeof(e));
    }
    return s_stall_active;
}

bool ipropi_stall_active(void)
{
    return s_stall_active;
}

void ipropi_stall_reset(void)
{
    bool was = s_stall_active;
    s_stall_since = 0;
    s_stall_active = false;
    if (was) {
        focstep_evt_base_t e = {.timestamp_ms = platform_now_ms()};
        platform_event_post(FOCSTEP_EVT_STALL_CLEARED, &e, sizeof(e));
    }
}

float ipropi_estimate_a_ipropi(float known_load_a, float measured_mv, float r_ipropi_ohm)
{
    /* 由 V = I × A × R 反解 A:
     *   I[A] × A[µA/A] × R[Ω] = V[mV] × 1000
     *   ⇒ A[µA/A] = V[mV] × 1000 / (I[A] × R[Ω]) */
    if (known_load_a <= 0.0f || r_ipropi_ohm <= 0.0f) {
        return 0.0f;
    }
    return measured_mv * 1000.0f / (known_load_a * r_ipropi_ohm);
}
