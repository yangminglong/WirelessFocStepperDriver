#include "bus_voltage.h"
#include "board_pins.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_rom_sys.h" /* esp_rom_delay_us: µs 级忙等, 见 bus_voltage_read_mv */
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "VBUS";

static adc_oneshot_unit_handle_t s_adc = NULL;
static adc_cali_handle_t s_cali = NULL;
static bool s_inited = false;
static int s_last_mv = 0;

esp_err_t bus_voltage_gate(bool on)
{
#if CONFIG_FOCSTEP_VBUS_GATE_ACTIVE_HIGH
    int level = on ? 1 : 0;
#else
    int level = on ? 0 : 1;
#endif
    return gpio_set_level((gpio_num_t)PIN_VBUS_GATE, level);
}

esp_err_t bus_voltage_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_VBUS_GATE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gate gpio config failed");

    /* 默认关断: 省电, 且保证上电时不往 ADC 节点灌电压 */
    ESP_RETURN_ON_ERROR(bus_voltage_gate(false), TAG, "gate off failed");

    adc_oneshot_unit_init_cfg_t unit_cfg = {.unit_id = ADC_UNIT_1};
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &s_adc), TAG, "adc unit new failed");

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc, ADC_CHANNEL_2, &chan_cfg),
                        TAG, "adc chan2 config failed");

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
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
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        if (adc_cali_create_scheme_line_fitting(&cali_cfg, &s_cali) == ESP_OK) {
            ESP_LOGI(TAG, "calibration: line fitting");
        }
    }
#endif

    s_inited = true;
    ESP_LOGI(TAG, "init done: divider=%d/%d, gate=%s, settle=%dus, min-enable=%dmV",
             CONFIG_FOCSTEP_VBUS_DIVIDER_NUM, CONFIG_FOCSTEP_VBUS_DIVIDER_DEN,
             CONFIG_FOCSTEP_VBUS_GATE_ACTIVE_HIGH ? "active-high" : "active-low",
             CONFIG_FOCSTEP_VBUS_GATE_SETTLE_US, CONFIG_FOCSTEP_VBUS_MIN_ENABLE_MV);
    return ESP_OK;
}

esp_err_t bus_voltage_read_mv(int *mv)
{
    if (!s_inited || mv == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = bus_voltage_gate(true);
    if (ret != ESP_OK) {
        return ret;
    }

    /* 源阻抗 = (100k∥8.2k) + 4.7k 串阻 = 12.3kΩ, ×1nF ⇒ τ=12.3µs, 5τ≈62µs;
     * 另 Q2 栅极建立 ≈5µs ⇒ 取 CONFIG_FOCSTEP_VBUS_GATE_SETTLE_US (默认 70)。
     *
     * ⚠️ **导通时间直接决定待机开销, 必须走 µs 级忙等**:
     *    导通期间 分压吸 VM/108.2k ≈ 233µA + 恒流下沉栅极支路 79µA = **312µA**。
     *    @100Hz + 70µs ⇒ 平均 **2.2µA@25.2V** (折算 24V ≈2.3µA, doc.md §五.1);
     *    若导通 2ms ⇒ **65µA —— 差 20 倍**, 单这一项就吃掉整个深睡档预算。
     *    ⇒ **不能用 vTaskDelay**: CONFIG_FREERTOS_HZ=1000 时最小就是 1ms, 下不去。 */
    esp_rom_delay_us(CONFIG_FOCSTEP_VBUS_GATE_SETTLE_US);

    int raw = 0;
    ret = adc_oneshot_read(s_adc, ADC_CHANNEL_2, &raw);

    /* 无论采样成功与否都要关断门控 —— 否则会常态耗电 */
    bus_voltage_gate(false);

    if (ret != ESP_OK) {
        return ret;
    }

    int node_mv = 0;
    if (s_cali) {
        ret = adc_cali_raw_to_voltage(s_cali, raw, &node_mv);
        if (ret != ESP_OK) {
            return ret;
        }
    } else {
        node_mv = raw * 3100 / 4095;
    }

    /* 反算母线: V_bus = V_node × (上臂+下臂)/下臂 */
    int vbus = node_mv * CONFIG_FOCSTEP_VBUS_DIVIDER_DEN / CONFIG_FOCSTEP_VBUS_DIVIDER_NUM;
    s_last_mv = vbus;
    *mv = vbus;
    return ESP_OK;
}

bool bus_voltage_allow_motor_enable(int mv)
{
    return mv >= CONFIG_FOCSTEP_VBUS_MIN_ENABLE_MV;
}

int bus_voltage_last_mv(void)
{
    return s_last_mv;
}
