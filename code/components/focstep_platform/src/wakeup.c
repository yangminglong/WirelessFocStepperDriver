#include "wakeup.h"
#include "board_pins.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"

static const char *TAG = "WAKE";

static bool s_ext1_enabled = false;

esp_err_t wakeup_init(void)
{
    esp_err_t ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        /* 已被别处装过不算错 */
        return ret;
    }
    return ESP_OK;
}

esp_err_t wakeup_enable_encoder_ext1(void)
{
    /* ⚠️ KTH5701 INT: 高有效、锁存、读数据清零
     *    ⇒ ANY_HIGH。配错极性则永远唤不醒。 */
    ESP_RETURN_ON_ERROR(
        esp_sleep_enable_ext1_wakeup_io(1ULL << PIN_ENCODER_INT, ESP_EXT1_WAKEUP_ANY_HIGH),
        TAG, "ext1 enable failed");

    /* LP 域引脚在深睡期间要保持输入功能; 上拉由 KTH5701 侧或板上决定,
     * 这里只确保不启用内部上下拉以免与外部冲突。 */
    ESP_RETURN_ON_ERROR(rtc_gpio_pullup_dis((gpio_num_t)PIN_ENCODER_INT),
                        TAG, "rtc pullup dis failed");
    ESP_RETURN_ON_ERROR(rtc_gpio_pulldown_dis((gpio_num_t)PIN_ENCODER_INT),
                        TAG, "rtc pulldown dis failed");

    s_ext1_enabled = true;
    ESP_LOGI(TAG, "ext1 armed: GPIO%d ANY_HIGH (KTH5701 INT, latched)", PIN_ENCODER_INT);
    return ESP_OK;
}

esp_err_t wakeup_disable_ext1(void)
{
    if (!s_ext1_enabled) {
        return ESP_OK;
    }
    esp_err_t ret = esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT1);
    s_ext1_enabled = false;
    return ret;
}

wake_src_t wakeup_get_source(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    switch (cause) {
    case ESP_SLEEP_WAKEUP_EXT1: {
        uint64_t mask = esp_sleep_get_ext1_wakeup_status();
        if (mask & (1ULL << PIN_ENCODER_INT)) {
            return WAKE_SRC_ENCODER_INT;
        }
        ESP_LOGW(TAG, "ext1 wake but mask=0x%llX does not include GPIO%d",
                 (unsigned long long)mask, PIN_ENCODER_INT);
        return WAKE_SRC_OTHER;
    }
    case ESP_SLEEP_WAKEUP_UNDEFINED:
        return WAKE_SRC_NONE; /* 冷启动 */
    default:
        ESP_LOGI(TAG, "wake cause=%d", (int)cause);
        return WAKE_SRC_OTHER;
    }
}

void wakeup_log_config(void)
{
    /* 只打印**唤醒源**配置。"多久没动作就睡"是应用策略, 由应用自己打 ——
     * 平台层不该引用应用级 Kconfig。 */
    ESP_LOGI(TAG, "wake config: ext1=%s (GPIO%d ANY_HIGH)",
             s_ext1_enabled ? "armed" : "off", PIN_ENCODER_INT);
}
