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
static bool s_inited = false;

/* ── ADC 衰减档表 (C6 手册 Table 5-6 的"标定有效量程") ────────────────
 * ★ 三个档的标定句柄在 init 时**全部建好并保留到 deinit**, 切档只换"用哪一个",
 *   不做运行期 create/delete —— 这样任何并发读者都不可能碰到已释放的句柄。
 *   最坏情况退化为"拿旧档的标定解释新档的原始码"(单个采样偏差), 由 s_switching 兜住。 */
typedef struct {
    adc_atten_t atten;
    int         range_mv;   /* 标定有效量程 (mV); 选档按 90% 留边, 见 set_ceiling_mv */
    const char *name;
} ipropi_atten_t;

static const ipropi_atten_t s_atten_tab[3] = {
    { ADC_ATTEN_DB_0,  1000, "0dB"  },
    { ADC_ATTEN_DB_6,  1900, "6dB"  },
    { ADC_ATTEN_DB_12, 3300, "12dB" },
};
static adc_cali_handle_t s_cali[3] = { NULL, NULL, NULL };
static uint8_t s_cali_kind[3] = { 0, 0, 0 }; /* 0=无标定 1=curve 2=line —— 删的时候要按建的方案删 */
static int s_atten_idx = 2;                  /* 上电取最宽档, init 里按 Kconfig 定 */
static volatile bool s_switching = false;    /* 切档窗口: 读接口让调用方跳过本次 */

/* 堵转判定状态 */
static float s_stall_ma = 1200.0f;
static uint32_t s_stall_ms = 300;
static TickType_t s_stall_since = 0;
static bool s_stall_active = false;

/* 峰值包络: FOC 循环写, 主循环读并清零 */
static volatile float s_env_peak = 0.0f;

/* ★ Kconfig 的衰减档只作**上电初值**: 一旦 ipropi_set_ceiling_mv() 被调用,
 *   量程就跟随 VREF 走 (Kconfig 允许 0~12 的任意整数, 非 0/6 一律按 12dB 处理)。 */
static int kconfig_atten_idx(void)
{
#if CONFIG_FOCSTEP_ADC_ATTEN_DB == 0
    return 0;   /* 0dB:  0~1000mV */
#elif CONFIG_FOCSTEP_ADC_ATTEN_DB == 6
    return 1;   /* 6dB:  0~1900mV */
#else
    return 2;   /* 12dB (默认): 0~3300mV */
#endif
}

/* 为某一档建标定: 曲线拟合优先, 退而求其次线性拟合。
 * ⚠️ 建不出的档**不会被自动选中** —— 自动选档宁可退到更宽的已标定档,
 *    也不静默用未标定读数 (未标定在 C6 上误差可达几十 mV)。 */
static void cali_create_one(int idx)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cf = {
        .unit_id = ADC_UNIT_1,
        .atten = s_atten_tab[idx].atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cf, &s_cali[idx]) == ESP_OK) {
        s_cali_kind[idx] = 1;
        ESP_LOGI(TAG, "calibration %s: curve fitting", s_atten_tab[idx].name);
        return;
    }
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t lf = {
        .unit_id = ADC_UNIT_1,
        .atten = s_atten_tab[idx].atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_line_fitting(&lf, &s_cali[idx]) == ESP_OK) {
        s_cali_kind[idx] = 2;
        ESP_LOGI(TAG, "calibration %s: line fitting", s_atten_tab[idx].name);
        return;
    }
#endif
    ESP_LOGW(TAG, "calibration %s unavailable: 该档不会被自动选中", s_atten_tab[idx].name);
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

    s_atten_idx = kconfig_atten_idx();

    /* 三个档的标定一次建齐 —— 切档只换句柄, 不做运行期 create/delete */
    for (int i = 0; i < 3; i++) {
        cali_create_one(i);
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = s_atten_tab[s_atten_idx].atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    for (int i = 0; i < IPROPI_PHASE_COUNT; i++) {
        ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, s_chan[i], &chan_cfg),
                            TAG, "adc chan %d config failed", (int)s_chan[i]);
    }

    s_inited = true;
    /* 无标定也能跑 (堵转判定是相对量), 但要在日志里说清楚 —— 绝对精度差几十 mV。 */
    ESP_LOGI(TAG, "init done: R_IPROPI=%dΩ A_IPROPI=%dµA/A atten=%s(%dmV)%s",
             CFG_R_IPROPI_OHM, CFG_A_IPROPI,
             s_atten_tab[s_atten_idx].name, s_atten_tab[s_atten_idx].range_mv,
             s_cali[s_atten_idx] ? "" : " [uncalibrated]");
    return ESP_OK;
}

void ipropi_deinit(void)
{
    for (int i = 0; i < 3; i++) {
        /* 按**建立时**的方案删: 两个方案都编译进固件时, 用错的那个删会走错误的
         * 释放路径 (曲线拟合不可用时才会建线性拟合, 所以不能只按编译宏二选一)。 */
        if (s_cali_kind[i] == 1) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
            adc_cali_delete_scheme_curve_fitting(s_cali[i]);
#endif
        } else if (s_cali_kind[i] == 2) {
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
            adc_cali_delete_scheme_line_fitting(s_cali[i]);
#endif
        }
        s_cali[i] = NULL;
        s_cali_kind[i] = 0;
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
    /* 切档窗口内先退出去: 此刻原始码与量程可能对不上, 用旧标定解释会得到一个
     * 离谱的电流值 —— 而它是堵转判定的输入。调用方跳过本次采样即可 (FOC 循环正是这么做的)。 */
    if (s_switching) {
        return ESP_ERR_INVALID_STATE;
    }
    int raw = 0;
    esp_err_t ret = adc_oneshot_read(s_adc, s_chan[phase], &raw);
    if (ret != ESP_OK) {
        return ret;
    }
    int idx = s_atten_idx;
    if (s_cali[idx]) {
        return adc_cali_raw_to_voltage(s_cali[idx], raw, mv);
    }
    /* 无标定时的粗略换算 (12bit), 按**当前档**的量程折算 */
    *mv = raw * s_atten_tab[idx].range_mv / 4095;
    return ESP_OK;
}

int ipropi_ceiling_mv(void)
{
    return s_inited ? s_atten_tab[s_atten_idx].range_mv : -1;
}

int ipropi_set_ceiling_mv(int ceiling_mv)
{
    if (!s_inited || ceiling_mv <= 0) {
        return -1;
    }
    /* 量程要盖住 ceiling **再留 10% 边**: 表里的量程是"误差合规"的上界, 贴着它用
     * 会把边缘误差吃进读数, 而 VREF 本身也有容差。 */
    int need = ceiling_mv + ceiling_mv / 10;
    int pick = -1;
    for (int i = 0; i < 3; i++) {          /* 从窄到宽, 第一个够用的就是最窄的 */
        if (s_cali[i] && need <= s_atten_tab[i].range_mv) {
            pick = i;
            break;
        }
    }
    if (pick < 0) {
        for (int i = 2; i >= 0; i--) {     /* 没有已标定档能覆盖 ⇒ 退到最宽的已标定档 */
            if (s_cali[i]) {
                pick = i;
                break;
            }
        }
        if (pick < 0) {
            return -1;                     /* 三个档都没标定, 不动 */
        }
        ESP_LOGW(TAG, "ceiling %d mV 超出已标定档量程, 退到 %s(%d mV): 读数会削顶",
                 ceiling_mv, s_atten_tab[pick].name, s_atten_tab[pick].range_mv);
    }
    if (pick == s_atten_idx) {
        return s_atten_tab[pick].range_mv;  /* 已在档上 (幂等) */
    }

    int prev = s_atten_idx;
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = s_atten_tab[pick].atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t ret = ESP_OK;
    s_switching = true;                     /* 让读者先退出去 */
    for (int i = 0; i < IPROPI_PHASE_COUNT; i++) {
        if (adc_oneshot_config_channel(s_adc, s_chan[i], &chan_cfg) != ESP_OK) {
            ret = ESP_FAIL;
            break;
        }
    }
    if (ret == ESP_OK) {
        s_atten_idx = pick;
        ESP_LOGI(TAG, "ADC 量程跟随 VREF: %s -> %s (覆盖 %d mV)",
                 s_atten_tab[prev].name, s_atten_tab[pick].name, ceiling_mv);
    } else {
        /* 回滚: 通道配置与 s_atten_idx 必须一致, 否则读数会按错的量程折算 */
        chan_cfg.atten = s_atten_tab[prev].atten;
        for (int i = 0; i < IPROPI_PHASE_COUNT; i++) {
            adc_oneshot_config_channel(s_adc, s_chan[i], &chan_cfg);
        }
        ESP_LOGE(TAG, "ADC 量程切换失败, 保持 %s", s_atten_tab[prev].name);
    }
    s_switching = false;

    return ret == ESP_OK ? s_atten_tab[pick].range_mv : -1;
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
    /* 与 ipropi_read_ma 同一条守卫: Kconfig 的两个系数都没有 range 下限,
     * 配成 0 就是除零 (IEEE 下会算出 inf 并被打印成电流值)。 */
    if (denom <= 0.0f) {
        return ESP_ERR_INVALID_STATE;
    }
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
