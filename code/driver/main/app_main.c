/*
 * FocStepper 推拉门应用 —— 入口与任务编排
 *
 * 分层:
 *   components/focstep_platform/   平台层 (跨项目复用, 无应用语义)
 *   main/app_door.c                应用语义 (门: 助动/开关停/灯色/指令映射)
 *   main/app_console.c             应用命令
 *   本文件                          只做初始化和任务编排, 不含业务逻辑
 *
 * ⚠️ 本文件**不得直接操作 GPIO18 (nSLEEP)** —— 那是平台 power_state.c 的独占职责,
 *    GPIO18 与 DAC PD 的编排顺序由 power_state.c 保证 (§5.4: 先断电再 PD / 先 DAC 后唤醒)
 *    与 **CAN 收发器的 Rs** (docs/doc.md §5.3) —— **一根脚管三件事**。
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
#include "pa_wake.h"
#include "platform_button.h"
#include "platform_console.h"
#include "platform_events.h"
#include "power_state.h"
#include "vref_dac.h"
#include "wakeup.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAIN";

esp_err_t app_console_init(void); /* app_console.c */

/* 是否正处于"无线监听档"(见下面的睡眠分流)。用于进出该档时的一次性动作 */
static bool s_listening = false;

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
    /* 灯: 上电安全态 = 灭 (显式发一帧全黑; 灯带常供电, 无供电门控) */
    ESP_ERROR_CHECK(led_init());
    /* 按键: 只发事件 */
    ESP_ERROR_CHECK(platform_button_init());
    /* CAN: 上电默认睡眠**由硬件保证** (Rs 随 nSLEEP=低 ⇒ 高 ⇒ 睡眠)。
     * ⚠️ **只在"用 CAN 档"才初始化** —— TWAI 一装上就占用 GPIO16/17, 会把 UART0
     *    控制台顶掉 (那两个脚与收发器是共用的, docs/doc.md §5.3)。该档的控制台
     *    走 USB-Serial-JTAG。开关见 Kconfig 的 FOCSTEP_CAN_ENABLE。 */
#if CONFIG_FOCSTEP_CAN_ENABLE
    ESP_ERROR_CHECK(can_init());
#endif
    /* 唤醒源 (本地: 编码器 INT / 干接点) */
    ESP_ERROR_CHECK(wakeup_init());

    /* 无线唤醒接收端 (BLE 周期广播): 这里只建 host 与控制器, **不起射频** ——
     * 何时监听是应用策略, 由 app_door_init() 按 FOCSTEP_PA_BOOT_LISTEN 开窗。 */
#if CONFIG_FOCSTEP_PA_WAKE_ENABLE
    if (pa_wake_init(CONFIG_FOCSTEP_RECEIVER_ID) != ESP_OK) {
        ESP_LOGW(TAG, "无线唤醒未就绪 (本地功能不受影响)");
    }
#endif

    /* ---- 3. 电机与采样 ---- */
    ESP_ERROR_CHECK(ipropi_init());
    ipropi_stall_config((float)CONFIG_FOCSTEP_STALL_CURRENT_MA, CONFIG_FOCSTEP_STALL_MS);
    ESP_ERROR_CHECK(foc_motor_init());
    /* 恢复位置。会自动做一致性检查: 若深睡期间被动过 (单圈角对不上),
     * 则标记位置不可信 ⇒ 后续位置指令会被拒绝, 直到回零 (见 homing.c)。 */
    ESP_ERROR_CHECK(foc_motor_restore_position());

    /* ---- 4. DAC (MCP4725 动态 VREF) ----
     * 位置有讲究: 必须在 foc_motor_init() 之后 (I2C 总线由编码器那边建起来),
     * 且在 power_state_init() 之前 —— 那个模块假定上电时 DAC 已在 PD, 且它的
     * GPIO18 编排全程依赖 vref_dac_* 可用。 */
    ESP_ERROR_CHECK(vref_dac_init());

    /* ---- 5. 电源状态机 (它一上来就把 nSLEEP 拉低) ---- */
    ESP_ERROR_CHECK(power_state_init());

    /* ---- 6. 命令台: 平台先, 应用后 ---- */
    ESP_ERROR_CHECK(platform_console_init());
#if CONFIG_FOCSTEP_CONSOLE_ENABLE
    app_console_init();
#endif

    /* ---- 7. FOC 任务 + 应用 ---- */
    ESP_ERROR_CHECK(foc_motor_start_loop());
    ESP_ERROR_CHECK(app_door_init());

    /* ---- 8. 平台态控制面 (默认关, 开着会打破深睡) ---- */
    esp_err_t net_ret = net_ota_start();
    if (net_ret == ESP_OK) {
        net_ota_print_info();
        ESP_LOGW(TAG, "平台态: 浏览器打开 http://focstep-xxxx.local/ 做 OTA");
    } else if (net_ret != ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGE(TAG, "控制面启动失败: %s (本地功能不受影响)", esp_err_to_name(net_ret));
    }

    /* ---- 9. 唤醒原因 → 发事件, 由应用决定怎么做 ---- */
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
    ESP_LOGI(TAG, "⚠️ 首次上板请先 `id` 验芯片, 再 `brake` 验 EN/PH 接法, 然后 `learn`");

    /* ================= 主循环 ================= */
    TickType_t last = xTaskGetTickCount();

    while (1) {
        power_state_tick();
#if CONFIG_FOCSTEP_CAN_ENABLE
        can_tick();
#endif
        app_door_tick();
        /* 注意: 灯不需要在这里 tick —— 闪灯时序由 led_indicator 自己的任务推进 */

        /* ---- 睡眠档 (PS_SLEEP) ---- */
        if (power_state_current() == PS_SLEEP && CONFIG_FOCSTEP_SLEEP_ENABLE) {

#if CONFIG_FOCSTEP_PA_WAKE_ENABLE
            /* 【监听档】无线唤醒开着时**不进显式睡眠** —— 交给 PM 自动轻睡,
             * 由 BLE 控制器按 PA 窗口排唤醒 (显式 esp_light_sleep_start() 只认
             * 我们配的冷唤醒源, 不会给 PA 窗口留时间)。见 code/README.md §13。 */
            if (pa_wake_mode() == PA_WAKE_PA) {
                if (!s_listening) {
                    s_listening = true;
                    /* 本地手拉仍要能唤醒: arm ext1。INT 是锁存型 ⇒ 醒来查电平即可。 */
#if CONFIG_FOCSTEP_WAKE_ON_ENCODER
                    ESP_ERROR_CHECK(wakeup_enable_encoder_ext1());
#endif
                    wakeup_log_config();
                    ESP_LOGW(TAG, "进入无线监听档: 节拍 %d ms, 由 PM 自动轻睡接管",
                             CONFIG_FOCSTEP_PA_TICK_MS);
                }
                /* 本地手拉: 与轻睡档共用同一条恢复路径
                 * (关 ext1 → 读一次数据清 INT 锁存 → 一致性检查 → 发 WOKE 事件) */
                if (wakeup_encoder_int_asserted()) {
                    s_listening = false;
                    wakeup_resume_from_light_sleep();
                }
                /* 无线指令由 NimBLE host 任务直接发事件; 这里只按慢节拍跑安全逻辑。
                 * ⚠️ 节拍必须慢: 每次唤醒都要付控制器固定的轻睡退出开销。 */
                vTaskDelay(pdMS_TO_TICKS(CONFIG_FOCSTEP_PA_TICK_MS));
                continue;
            }
            /* 从监听档退出 (或从未进入): 关掉监听期用的 ext1, 并重置时间基,
             * 免得 100ms 的 vTaskDelayUntil 追打。 */
            if (s_listening) {
                s_listening = false;
                wakeup_disable_ext1();
                last = xTaskGetTickCount();
            }
#endif /* CONFIG_FOCSTEP_PA_WAKE_ENABLE */

#if CONFIG_FOCSTEP_SLEEP_MODE_DEEP
            /* DeepSleep: 唤醒即复位 ⇒ 睡前把 **(圈数, 单圈角)** 成对写进 NVS,
             * 唤醒后由 foc_motor_restore_position() 判断"睡着期间被动过没有" (§10.4 ②)。
             * 只存圈数是不够的。 */
            foc_motor_save_position();

#if CONFIG_FOCSTEP_WAKE_ON_ENCODER
            ESP_ERROR_CHECK(wakeup_enable_encoder_ext1());
#endif
            wakeup_log_config();
            ESP_LOGI(TAG, "进入深睡 (nSLEEP 已拉低 ⇒ DRV 断电, DAC 已 PD)");
            ESP_LOGI(TAG, "⚠️ 若此时 DAC 未进 PD, 会持续耗 210µA (正常模式)");

            /* 灯带全黑必须**确认已锁存**再睡 —— led_set_color(OFF) 是异步的,
             * 见 led_off_and_wait() 注释 (§6.3 步骤 5)。 */
            esp_err_t lret = led_off_and_wait();
            if (lret != ESP_OK) {
                ESP_LOGW(TAG, "灯带全黑未确认 (%s): 灯珠可能锁存上一颜色, "
                              "睡眠期间约 60mA 且无硬件兜底", esp_err_to_name(lret));
            }

            esp_deep_sleep_start();
            /* 不会返回 */

#else /* CONFIG_FOCSTEP_SLEEP_MODE_LIGHT */
            /* LightSleep: 唤醒**不复位**, 从 esp_light_sleep_start() 之后继续执行。
             * ⇒ 位置只记在 RAM (不写 NVS); 醒来必须清 INT 锁存 + 重做可信度判定。
             * ⚠️ 直接调 esp_light_sleep_start() 时 IDF **不补偿 FreeRTOS tick**
             *    (只有自动轻睡才走 pm_step_tick) ⇒ 醒来必须重置时间基。 */
            foc_motor_mark_sleep_angle();

#if CONFIG_FOCSTEP_WAKE_ON_ENCODER
            ESP_ERROR_CHECK(wakeup_enable_encoder_ext1());
#endif
            wakeup_log_config();
            ESP_LOGI(TAG, "进入轻睡 (唤醒不复位; nSLEEP 已拉低 ⇒ DRV 断电, DAC 已 PD)");

            /* 灯带全黑必须**确认已锁存**再睡 —— led_set_color(OFF) 是异步的,
             * 见 led_off_and_wait() 注释 (§6.3 步骤 5)。 */
            esp_err_t lret = led_off_and_wait();
            if (lret != ESP_OK) {
                ESP_LOGW(TAG, "灯带全黑未确认 (%s): 灯珠可能锁存上一颜色, "
                              "轻睡期间约 60mA 且无硬件兜底", esp_err_to_name(lret));
            }

            /* 醒来先看 INT 电平再决定: KTH5701 的 INT 是**锁存**的 ⇒
             * "这次醒来是不是手拉"只看电平就够, 不依赖唤醒掩码
             * (掩码在 RTC 域, 且与共线的干接点分不开)。 */
            while (power_state_current() == PS_SLEEP) {
                esp_err_t sret = esp_light_sleep_start();
                bool int_high = wakeup_encoder_int_asserted();

                if (sret != ESP_OK && sret != ESP_ERR_SLEEP_REJECT) {
                    ESP_LOGW(TAG, "轻睡失败: %s ⇒ 退回主循环 (查唤醒源/PM 配置)",
                             esp_err_to_name(sret));
                    break;
                }
                if (!int_high) {
                    /* 不是编码器唤醒 (RTC 定时/干接点等), 或被拒绝但本脚没触发:
                     * 被拒绝 = "有唤醒源已处于有效态", 延一拍免得空转, 然后接着睡。 */
                    if (sret == ESP_ERR_SLEEP_REJECT) {
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                    continue;
                }
                /* 手拉: 关唤醒源 + 清 INT 锁存 + 一致性检查 + 发 WOKE 事件 */
                wakeup_resume_from_light_sleep();
                break;
            }

            /* ★ 时间基重置: 轻睡期间 tick 不前进, 不重置 vTaskDelayUntil 会连续追打 */
            last = xTaskGetTickCount();
            /* 让高优先级的事件任务先把 WOKE 处理完 (它会请求 PS_ACTIVE) */
            vTaskDelay(pdMS_TO_TICKS(1));
#endif
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
    }
}
