/*
 * 应用层命令台 —— 用 platform_console_register() 追加, 不改平台层文件
 *
 * 对比重构前: 命令全挤在 cmd_console.cpp 的同一个函数里, 加一条就得动驱动层。
 */

#include "app_door.h"

#include "foc_motor.h"
#include "homing.h"
#include "platform_console.h"
#include "power_state.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"

static const char *TAG = "APP_CMD";

static int do_open(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    app_door_command(APP_CMD_OPEN);
    printf("已下达: 开\n");
    return 0;
}

static int do_close(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    app_door_command(APP_CMD_CLOSE);
    printf("已下达: 关\n");
    return 0;
}

static int do_stop(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    app_door_command(APP_CMD_STOP);
    printf("已下达: 停\n");
    return 0;
}

static int do_wake(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    app_door_command(APP_CMD_WAKE);
    printf("已下达: 唤醒接管\n");
    return 0;
}

static int do_mode(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    printf("应用子模式 : %s\n", app_door_mode_name(app_door_mode()));
    printf("平台状态   : %s (fault=%u)\n",
           power_state_name(power_state_current()), power_state_fault_code());
    printf("§10.3 映射 : 深睡=SLEEP / 唤醒接管=ACTIVE+ASSIST / 运行=ACTIVE+RUNNING / 故障=FAULT\n");
    float rmin = 0, rmax = 0;
    foc_motor_get_travel_range(&rmin, &rmax);
    printf("位置可信   : %s   行程 [%.3f, %.3f] rad\n",
           foc_motor_position_trusted() ? "是 ✅" : "**否 ❌ (先跑 learn)**",
           (double)rmin, (double)rmax);
    printf("助动力矩   : %s V  (Kconfig FOCSTEP_ASSIST_TORQUE)\n",
           CONFIG_FOCSTEP_ASSIST_TORQUE);
    return 0;
}

esp_err_t app_console_init(void)
{
    if (!platform_console_ready()) {
        ESP_LOGE(TAG, "平台命令台未就绪 —— 要先调 platform_console_init()");
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = ESP_OK;
    ret |= platform_console_register("open",  "开门",              do_open);
    ret |= platform_console_register("close", "关门",              do_close);
    ret |= platform_console_register("stop",  "停止并保持",         do_stop);
    ret |= platform_console_register("wake",  "进入唤醒接管(助动)",  do_wake);
    ret |= platform_console_register("mode",  "打印应用/平台状态与映射", do_mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "部分应用命令注册失败");
    } else {
        ESP_LOGI(TAG, "应用命令已注册: open / close / stop / wake / mode");
    }
    return ret;
}
