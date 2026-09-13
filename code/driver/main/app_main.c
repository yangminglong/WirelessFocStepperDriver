/*
 * FocStepper 推拉门应用 —— 入口与任务编排
 *
 * 分层:
 *   components/focstep_platform/   平台层 (跨项目复用, 无应用语义)
 *   main/app_door.c                应用语义 (门: 助动/开关停/灯色/指令映射)
 *   main/app_console.c             应用命令
 *   本文件                          只做初始化和任务编排, 不含业务逻辑
 *
 * ⚠️ 本文件**不得直接操作 GPIO16 (nSLEEP)** —— 那是平台 power_state.c 的独占职责,
 *    因为它同时硬件门控 VREF 分压 (关不断就常态耗 106µA ≈ 0.42mW@24V)。
 */

#include <stdio.h>

#include "app_door.h"

#include "board_pins.h"
#include "bus_voltage.h"
#include "can_link.h"
#include "foc_motor.h"
#include "homing.h"
#include "ipropi_sense.h"
#include "led_ws2812.h"
#include "net_ota.h"
#include "platform_button.h"
#include "platform_console.h"
#include "platform_events.h"
#include "power_state.h"
#include "wakeup.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAIN";

esp_err_t app_console_init(void); /* app_console.c */

static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

void app_main(void)
{
    ESP_LOGI(TAG, "FocStepper 推拉门驱动 booting");

    ESP_ERROR_CHECK(init_nvs());

    /* ---- 1. 事件总线必须最先 ---- */
    ESP_ERROR_CHECK(platform_events_init());

    /* ---- 2. 平台外设, 顺序有讲究 ---- */
    /* 母线门控先关断: 保证上电到第一次采样之间不耗电, 也不往 ADC 节点灌电压 */
    ESP_ERROR_CHECK(bus_voltage_init());
    /* 灯: 上电安全态 = 灭 (GPIO8 高, 同时满足 strapping) */
    ESP_ERROR_CHECK(led_init());
    /* 按键: 只发事件 */
    ESP_ERROR_CHECK(platform_button_init());
    /* CAN: 上电默认睡眠 (Rs 高) */
    ESP_ERROR_CHECK(can_init());
    /* 唤醒源 */
    ESP_ERROR_CHECK(wakeup_init());

    /* ---- 3. 电机与采样 ---- */
    ESP_ERROR_CHECK(ipropi_init());
    ipropi_stall_config((float)CONFIG_FOCSTEP_STALL_CURRENT_MA, CONFIG_FOCSTEP_STALL_MS);
    ESP_ERROR_CHECK(foc_motor_init());
    /* 恢复位置。会自动做一致性检查: 若深睡期间被动过 (单圈角对不上),
     * 则标记位置不可信 ⇒ 后续位置指令会被拒绝, 直到回零 (见 homing.c)。 */
    ESP_ERROR_CHECK(foc_motor_restore_position());

    /* ---- 4. 电源状态机 (它一上来就把 nSLEEP 拉低) ---- */
    ESP_ERROR_CHECK(power_state_init());

    /* ---- 5. 命令台: 平台先, 应用后 ---- */
    ESP_ERROR_CHECK(platform_console_init());
#if CONFIG_FOCSTEP_CONSOLE_ENABLE
    app_console_init();
#endif

    /* ---- 6. FOC 任务 + 应用 ---- */
    ESP_ERROR_CHECK(foc_motor_start_loop());
    ESP_ERROR_CHECK(app_door_init());

    /* ---- 7. 平台态控制面 (默认关, 开着会打破深睡) ---- */
    esp_err_t net_ret = net_ota_start();
    if (net_ret == ESP_OK) {
        net_ota_print_info();
        ESP_LOGW(TAG, "平台态: 浏览器打开 http://focstep-xxxx.local/ 做 OTA");
    } else if (net_ret != ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGE(TAG, "控制面启动失败: %s (本地功能不受影响)", esp_err_to_name(net_ret));
    }

    /* ---- 8. 唤醒原因 → 发事件, 由应用决定怎么做 ---- */
    wake_src_t src = wakeup_get_source();
    if (src != WAKE_SRC_NONE) {
        focstep_evt_wake_t e = {.base.timestamp_ms = platform_now_ms(), .src = (int)src};
        platform_event_post(FOCSTEP_EVT_WOKE, &e, sizeof(e));
        if (src == WAKE_SRC_ENCODER_INT && !foc_motor_position_trusted()) {
            ESP_LOGW(TAG, "睡着期间被移动 ⇒ 位置需重新回零 (learn / home)");
        }
    } else {
        ESP_LOGI(TAG, "冷启动");
    }

    ESP_LOGI(TAG, "boot done. 平台自检: id/regs/circle/vbus/brake/vref/int");
    ESP_LOGI(TAG, "应用命令: open/close/stop/wake/mode/learn/home");
    ESP_LOGI(TAG, "⚠️ 首次上板请先 `id` 验芯片, 再 `brake` 裁决 EN/PH, 然后 `learn`");

    /* ================= 主循环 ================= */
    TickType_t last = xTaskGetTickCount();

    while (1) {
        power_state_tick();
        can_tick();
        app_door_tick();
        /* 注意: 灯不需要在这里 tick —— 闪灯时序由 led_indicator 自己的任务推进 */

        /* 深睡 */
        if (power_state_current() == PS_SLEEP && CONFIG_FOCSTEP_DEEP_SLEEP_ENABLE) {
            /* 深睡前存位置 —— **(圈数, 单圈角)** 成对存, 唤醒时才能判断
             * "睡着期间被动过没有" (§10.4 ②)。只存圈数是不够的。 */
            foc_motor_save_position();

#if CONFIG_FOCSTEP_WAKE_ON_ENCODER
            ESP_ERROR_CHECK(wakeup_enable_encoder_ext1());
#endif
            wakeup_log_config();
            ESP_LOGI(TAG, "进入深睡 (nSLEEP 已拉低 ⇒ VREF 门控关断)");
            ESP_LOGI(TAG, "⚠️ 若此时 VREF 分压没关断, 会持续耗 106µA = 0.42mW@24V");

            esp_deep_sleep_start();
            /* 不会返回 */
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
    }
}
