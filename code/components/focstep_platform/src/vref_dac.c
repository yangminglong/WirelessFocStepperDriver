#include "vref_dac.h"

#include "esp_log.h"
#include "esp_check.h"
#include "mcp4725.h"
#include "board_pins.h"
#include "ipropi_sense.h"

static const char *TAG = "VREF_DAC";

static i2c_dev_t s_dac;
static bool s_inited = false;
static float s_itrip_a = 0.0f; /* 当前生效档位; 0 = 未写或已 PD */

/* VDD 参考: 3.4V 轨 (TPP363070Q 输出, 实际 3.35~3.40V; 对 ITRIP 影响 <1%) */
#define VREF_DAC_VDD_V       3.4f

/* VREF = ITRIP × (R_IPROPI × A_IPROPI), 两个系数都来自 Kconfig ——
 * 与 ipropi_sense.c 用的是同一对符号, 标定后 VREF↔ITRIP 不会与实测电流脱节。 */
#define VREF_V_PER_A \
    ((float)CONFIG_FOCSTEP_R_IPROPI_OHM * (float)CONFIG_FOCSTEP_A_IPROPI_UA_PER_A * 1e-6f)

/* 唤醒默认档 —— 只是 Kconfig 真源的单位换算, 不是第二份档位常量 */
#define VREF_DAC_DEFAULT_ITRIP_A ((float)CONFIG_FOCSTEP_ITRIP_MA * 0.001f)

/* ⚠️ 这里的 3.0V 是**固件自设的余量上限**, 不是器件钳位 —— DRV8874 的 VREF
 *    推荐工作范围是 0~3.6V, 手册未规定内部钳位 (真正的硬件边界是 DAC 满量程
 *    ≈VDD 3.35V, 折算 ≈2.14A)。取 3.0V 是为了让 1.85A 档离满量程与推荐上限
 *    都留出余量。
 *    该余量随 R_IPROPI/A_IPROPI 标定值一起变化, 所以做成编译期断言而不是运行期
 *    分支 (后者永远不可达, 反而让人以为还有第二道防线)。
 *    整数形式: 1.85 × R × A × 1e-6 ≤ 3.0  ⟺  185 × R × A ≤ 3e8 */
_Static_assert((long)CONFIG_FOCSTEP_R_IPROPI_OHM *
                   (long)CONFIG_FOCSTEP_A_IPROPI_UA_PER_A * 185L <= 300000000L,
               "VREF_DAC_MAX_ITRIP_A 折算出的 VREF 超过自设的 3.0V 余量上限 "
               "(器件边界其实是 DAC 满量程 ≈VDD 3.35V): "
               "请检查 FOCSTEP_R_IPROPI_OHM / FOCSTEP_A_IPROPI_UA_PER_A 的标定值");

esp_err_t vref_dac_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }
    esp_err_t ret = i2cdev_init();
    ESP_RETURN_ON_ERROR(ret, TAG, "i2cdev_init failed");

    /* ★ 自己填描述符而不用 mcp4725_init_desc(): 那个函数把 clk_speed 硬编码成 1MHz
     *   (mcp4725.c 的 I2C_FREQ_HZ), 而本总线上拉只有 4.7k —— Fm+ (1MHz) 要求
     *   t_r ≤ 120ns, 4.7k×~25pF 已 ~260ns, 边沿进不了规格 ⇒ 写可能偶发 NACK;
     *   而唤醒路径上 DAC 写失败会**直接拒绝使能电机**。写法与 kth5701.cpp 一致。 */
    s_dac.port = I2C_NUM_0;
    s_dac.addr = MCP4725A0_I2C_ADDR0;
    s_dac.addr_bit_len = I2C_ADDR_BIT_LEN_7;
    s_dac.cfg.sda_io_num = (gpio_num_t)PIN_I2C_SDA;
    s_dac.cfg.scl_io_num = (gpio_num_t)PIN_I2C_SCL;
    s_dac.cfg.master.clk_speed = CONFIG_FOCSTEP_DAC_I2C_HZ;
    ret = i2c_dev_create_mutex(&s_dac);
    ESP_RETURN_ON_ERROR(ret, TAG, "i2c_dev_create_mutex failed");

    /* ★ 出厂 EEPROM = **满量程中点** (PD=00, D11=1, D10~D0=0) ⇒ 上电 VOUT ≈ VDD/2
     *   ≈ 1.7V, **不是 0V**。上电期间 DRV 被 nSLEEP 的 10k 下拉压在睡眠态, 这个
     *   电压不产生任何桥电流; 这里显式进 PD 把 VREF 收到 0 (内部 100k 对地),
     *   之后才允许抬 nSLEEP。 */
    ret = mcp4725_set_power_mode(&s_dac, false, MCP4725_PM_PD_100K);
    ESP_RETURN_ON_ERROR(ret, TAG, "mcp4725 PD init failed");

    s_itrip_a = 0.0f;
    s_inited = true;
    ESP_LOGI(TAG, "init done: MCP4725 @0x60 (%d Hz), 初始 PD (VREF=0, 60nA typ)",
             CONFIG_FOCSTEP_DAC_I2C_HZ);
    return ESP_OK;
}

float vref_dac_voltage_for(float itrip_a)
{
    if (itrip_a < 0.0f) {
        itrip_a = 0.0f;
    }
    if (itrip_a > VREF_DAC_MAX_ITRIP_A) {
        itrip_a = VREF_DAC_MAX_ITRIP_A;
    }
    return itrip_a * VREF_V_PER_A;
}

esp_err_t vref_dac_set(float itrip_a)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    float v = vref_dac_voltage_for(itrip_a); /* 含 clamp */
    /* 日志报 **clamp 后**的档位: 报原始入参会让 "ITRIP=5.00A → VREF=2.897V"
     * 这种自相矛盾的行进日志, 上板排查时比没有日志更坏。 */
    float applied = v / VREF_V_PER_A;
    /* eeprom=false: 只写 DAC 寄存器 (不掉电保存) —— 掉电后回到出厂 EEPROM 的中点值 */
    esp_err_t ret = mcp4725_set_voltage(&s_dac, VREF_DAC_VDD_V, v, false);
    ESP_RETURN_ON_ERROR(ret, TAG, "set VREF %.3fV failed (I2C)", v);
    s_itrip_a = applied;

    /* ★ 档位决定 IPROPI 的钳位点 ⇒ **ADC 量程必须跟着 VREF 走**: 降档后量程不收窄,
     *   读数会被钳住 (报出的电流比实际小); 升档后量程不跟着放宽, 读数会削顶。
     *   放在 DAC 写成功之后 —— 写失败时档位没变, 量程也不该变。 */
    if (v > 0.0f) {
        int ceil_mv = ipropi_set_ceiling_mv((int)(v * 1000.0f + 0.5f));
        if (ceil_mv < 0) {
            ESP_LOGW(TAG, "ADC 量程未跟随 (ipropi 未初始化, 或无已标定档)");
        }
        ESP_LOGI(TAG, "ITRIP=%.2fA → VREF=%.3fV (ADC 量程 %d mV)", applied, v, ceil_mv);
    } else {
        ESP_LOGI(TAG, "ITRIP=0 → VREF=0V (ADC 量程保持)");
    }
    return ESP_OK;
}

float vref_dac_default_itrip_a(void)
{
    return VREF_DAC_DEFAULT_ITRIP_A;
}

esp_err_t vref_dac_set_default(void)
{
    return vref_dac_set(VREF_DAC_DEFAULT_ITRIP_A);
}

/* ── "峰值需求 → 档位" (供上层按负载/力矩档切换) ─────────────────── */

float vref_dac_itrip_for_peak(float peak_a)
{
    if (peak_a <= 0.0f) {
        return 0.0f;
    }
    float v = peak_a * VREF_DAC_CLAMP_MARGIN;
    if (v < VREF_DAC_MIN_ITRIP_A) {
        v = VREF_DAC_MIN_ITRIP_A;
    }
    if (v > VREF_DAC_MAX_ITRIP_A) {
        v = VREF_DAC_MAX_ITRIP_A;
    }
    return v;
}

bool vref_dac_peak_supported(float peak_a)
{
    /* 留不出裕量就是**不支持**, 不做静默 clamp: 否则调用方会以为"档位覆盖了我报的
     * 电流", 而实际桥给不出 —— 现场表现为"指令收下了、门没力", 比直接报错难查得多。 */
    return peak_a > 0.0f && peak_a * VREF_DAC_CLAMP_MARGIN <= VREF_DAC_MAX_ITRIP_A;
}

esp_err_t vref_dac_set_for_peak(float peak_a)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!vref_dac_peak_supported(peak_a)) {
        ESP_LOGE(TAG, "峰值 %.2fA 留不出 %.2f× 钳位裕量 (上限 %.2fA): 不写 DAC",
                 peak_a, VREF_DAC_CLAMP_MARGIN, VREF_DAC_MAX_ITRIP_A);
        return ESP_ERR_INVALID_ARG;
    }
    return vref_dac_set(vref_dac_itrip_for_peak(peak_a));
}

esp_err_t vref_dac_pd(void)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = mcp4725_set_power_mode(&s_dac, false, MCP4725_PM_PD_100K);
    ESP_RETURN_ON_ERROR(ret, TAG, "DAC PD failed (I2C)");
    s_itrip_a = 0.0f;
    /* 不动 ADC 量程: PD 期 VREF=0, 量程无所谓; 唤醒路径会经 vref_dac_set() 重新同步两者 */
    ESP_LOGI(TAG, "PD: VREF=0 (60nA typ / 2µA max)");
    return ESP_OK;
}

float vref_dac_itrip_a(void)
{
    return s_itrip_a;
}
