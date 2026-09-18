#include "vref_dac.h"

#include "esp_log.h"
#include "esp_check.h"
#include "mcp4725.h"
#include "board_pins.h"

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

/* ⚠️ DRV8874 的 VREF 引脚内部钳位在 3.0V, 超过就失去调节能力 (ITRIP 封顶)。
 *    VREF_DAC_MAX_ITRIP_A 档必须仍在钳位之下 —— 这条约束随 R_IPROPI/A_IPROPI
 *    标定值一起变化, 所以做成编译期断言而不是运行期分支 (后者永远不可达,
 *    反而让人以为还有第二道防线)。
 *    整数形式: 1.85 × R × A × 1e-6 ≤ 3.0  ⟺  185 × R × A ≤ 3e8 */
_Static_assert((long)CONFIG_FOCSTEP_R_IPROPI_OHM *
                   (long)CONFIG_FOCSTEP_A_IPROPI_UA_PER_A * 185L <= 300000000L,
               "VREF_DAC_MAX_ITRIP_A 折算出的 VREF 超过 DRV8874 的 3.0V 内部钳位: "
               "请检查 FOCSTEP_R_IPROPI_OHM / FOCSTEP_A_IPROPI_UA_PER_A 的标定值");

esp_err_t vref_dac_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }
    esp_err_t ret = i2cdev_init();
    ESP_RETURN_ON_ERROR(ret, TAG, "i2cdev_init failed");

    ret = mcp4725_init_desc(&s_dac, MCP4725A0_I2C_ADDR0, I2C_NUM_0,
                            (gpio_num_t)PIN_I2C_SDA, (gpio_num_t)PIN_I2C_SCL);
    ESP_RETURN_ON_ERROR(ret, TAG, "mcp4725_init_desc failed");

    /* 上电默认 EEPROM=0 → VREF=0 (限流最小, 安全); 显式进 PD 兜底 */
    ret = mcp4725_set_power_mode(&s_dac, false, MCP4725_PM_PD_100K);
    ESP_RETURN_ON_ERROR(ret, TAG, "mcp4725 PD init failed");

    s_itrip_a = 0.0f;
    s_inited = true;
    ESP_LOGI(TAG, "init done: MCP4725 @0x60, 初始 PD (VREF=0, 60nA typ)");
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
    /* eeprom=false: 只写 DAC 寄存器 (掉电不保存, 上电回到 EEPROM=0 安全态) */
    esp_err_t ret = mcp4725_set_voltage(&s_dac, VREF_DAC_VDD_V, v, false);
    ESP_RETURN_ON_ERROR(ret, TAG, "set VREF %.3fV failed (I2C)", v);
    s_itrip_a = applied;
    ESP_LOGI(TAG, "ITRIP=%.2fA → VREF=%.3fV", applied, v);
    return ESP_OK;
}

esp_err_t vref_dac_set_default(void)
{
    return vref_dac_set(VREF_DAC_DEFAULT_ITRIP_A);
}

esp_err_t vref_dac_pd(void)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = mcp4725_set_power_mode(&s_dac, false, MCP4725_PM_PD_100K);
    ESP_RETURN_ON_ERROR(ret, TAG, "DAC PD failed (I2C)");
    s_itrip_a = 0.0f;
    ESP_LOGI(TAG, "PD: VREF=0 (60nA typ / 2µA max)");
    return ESP_OK;
}

float vref_dac_itrip_a(void)
{
    return s_itrip_a;
}
