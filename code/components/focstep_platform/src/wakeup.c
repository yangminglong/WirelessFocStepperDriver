#include "wakeup.h"
#include "board_pins.h"
#include "foc_motor.h"       /* 轻睡: 清 INT 锁存 / 睡着期间位置一致性检查 */
#include "platform_events.h" /* 轻睡醒来上报 FOCSTEP_EVT_WOKE */

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

    /* 编码器 INT 线: 输入, **不用内部上下拉** (高有效/锁存; 电平由 KTH5701 侧决定)。
     * 以前只在读的时候调 gpio_get_level() 而从未配过方向 —— 轻睡循环要反复读它,
     * 这里显式配置一次。 */
    gpio_config_t int_io = {
        .pin_bit_mask = 1ULL << PIN_ENCODER_INT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE, /* 唤醒走 ext1, 不用 GPIO 中断 */
    };
    ESP_RETURN_ON_ERROR(gpio_config(&int_io), TAG, "encoder INT gpio config failed");
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

/* ── 轻睡档 (Kconfig FOCSTEP_SLEEP_MODE_LIGHT) ───────────────────── */

bool wakeup_encoder_int_asserted(void)
{
    return gpio_get_level((gpio_num_t)PIN_ENCODER_INT) != 0;
}

wake_src_t wakeup_resume_from_light_sleep(void)
{
    wake_src_t src = wakeup_get_source();

    /* ① 先关唤醒源: 处理期间不再被它打断; 下次进睡前由调用方重新 arm。 */
    wakeup_disable_ext1();

    /* ② 清 INT 锁存 —— KTH5701 是"高有效 + 锁存 + 读一次数据清零"。
     *    不清掉就再睡 ⇒ 同一次中断立刻把 MCU 再唤醒 (死循环)。 */
    if (wakeup_encoder_int_asserted()) {
        if (foc_motor_encoder_clear_int()) {
            ESP_LOGI(TAG, "INT 锁存已清 (读数据后归低)");
        } else {
            ESP_LOGW(TAG, "读了数据 INT 仍为高 —— 锁存语义可能不成立, 先跑自检 `int`; "
                          "此状态下别指望轻睡能睡着");
        }
    }

    /* ③ 睡着期间的一致性检查 + 上报。
     *    与深睡口径一致: 超容差就标记位置不可信, 下一条运动指令前必须回零。 */
    if (src == WAKE_SRC_ENCODER_INT) {
        foc_motor_check_sleep_angle();
        focstep_evt_wake_t e = {.base.timestamp_ms = platform_now_ms(), .src = (int)src};
        platform_event_post(FOCSTEP_EVT_WOKE, &e, sizeof(e));
    } else {
        ESP_LOGI(TAG, "轻睡醒来 (src=%d), 非编码器 INT", (int)src);
    }
    return src;
}
