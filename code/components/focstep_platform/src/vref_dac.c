#include "vref_dac.h"

#include "esp_log.h"
#include "esp_check.h"
#include "mcp4725.h"
#include "board_pins.h"

static const char *TAG = "VREF_DAC";

static i2c_dev_t s_dac;
static bool s_inited = false;

/* VDD 参考: 3.4V 轨 (TPP363070Q 输出, 实际 3.35~3.40V; 对 ITRIP 影响 <1%) */
#define VREF_DAC_VDD_V       3.4f
/* VREF = ITRIP × 1.566 V/A (R_IPROPI=3.48k, A_IPROPI=450µA/A, 见 doc.md §5.4) */
#define VREF_V_PER_A         1.566f
/* 1.85A 档对应 VREF = 1.85 × 1.566 ≈ 2.90V (DRV 内部 3.0V 钳位前留余量) */
#define VREF_MAX_V           2.90f

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
    float v = itrip_a * VREF_V_PER_A;
    if (v > VREF_MAX_V) {
        v = VREF_MAX_V;
    }
    return v;
}

esp_err_t vref_dac_set(float itrip_a)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    float v = vref_dac_voltage_for(itrip_a);
    /* eeprom=false: 只写 DAC 寄存器 (掉电不保存, 上电回到 EEPROM=0 安全态) */
    esp_err_t ret = mcp4725_set_voltage(&s_dac, VREF_DAC_VDD_V, v, false);
    ESP_RETURN_ON_ERROR(ret, TAG, "set VREF %.3fV failed (I2C)", v);
    ESP_LOGI(TAG, "ITRIP=%.2fA → VREF=%.3fV", itrip_a, v);
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
    ESP_LOGI(TAG, "PD: VREF=0 (60nA typ / 2µA max)");
    return ESP_OK;
}
