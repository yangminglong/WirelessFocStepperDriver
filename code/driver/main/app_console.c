/*
 * 应用层命令台 —— 用 platform_console_register() 追加, 不改平台层文件
 *
 * 对比重构前: 命令全挤在 cmd_console.cpp 的同一个函数里, 加一条就得动驱动层。
 */

#include "app_door.h"

#include "foc_door_link.h" /* 无线指令集 (应用层协议) */
#include "foc_motor.h"
#include "homing.h"
#include "pa_wake.h"
#include "platform_console.h"
#include "power_state.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

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
    range_state_t rst = foc_motor_range_state();
    float zero = 0.0f, end = 0.0f, span = 0.0f;
    foc_motor_get_range(&zero, &end, &span);
    printf("行程状态   : %s\n", foc_motor_range_state_str(rst));
    if (rst != RANGE_NONE) {
        printf("零点(0%%)   : %.3f rad\n", (double)zero);
    }
    if (rst == RANGE_BOTH) {
        printf("满行程(100%%): %.3f rad  span %.3f rad ⇒ 开度增大 = 朝%s\n",
               (double)end, (double)span, (span > 0) ? "正" : "负");
    } else if (rst == RANGE_ZERO_ONLY) {
        printf("             **缺满行程点 ⇒ 开/关会被拒 (0%%/100%% 还不成立)**\n");
    }
    printf("位置可信   : %s   开/关可用: %s\n",
           foc_motor_position_trusted() ? "是 ✅" : "**否 ❌ (先 learn/mark)**",
           (foc_motor_position_trusted() && homing_has_range()) ? "是 ✅"
                                                               : "**否 ❌ (需位置可信 + 行程完整)**");
    printf("助动力矩   : %s V  (Kconfig FOCSTEP_ASSIST_TORQUE)\n",
           CONFIG_FOCSTEP_ASSIST_TORQUE);
    return 0;
}

/*
 * 无线指令本地注入 (不经射频): 只用来**单独验"命令 → 事件 → 应用映射"这一链** ——
 * 射频/同步的问题与映射的问题分开定位。
 * 注: "注入业务命令"是应用工具 —— 命令名属于应用协议, 所以不在平台命令台里。
 */
static int do_pa_send(int argc, char **argv)
{
    if (argc < 2) {
        printf("用法: pa_send <none|wake|open|close|stop|set_t> [arg]\n");
        printf("  set_t 的 arg = 期望唤醒周期 T (ms), 0 用默认值\n");
        return 1;
    }
    uint8_t cmd = 0xFF;
    if (!strcmp(argv[1], "none"))        cmd = FOC_DOOR_CMD_NONE;
    else if (!strcmp(argv[1], "wake"))   cmd = FOC_DOOR_CMD_WAKE;
    else if (!strcmp(argv[1], "open"))   cmd = FOC_DOOR_CMD_OPEN;
    else if (!strcmp(argv[1], "close"))  cmd = FOC_DOOR_CMD_CLOSE;
    else if (!strcmp(argv[1], "stop"))   cmd = FOC_DOOR_CMD_STOP;
    else if (!strcmp(argv[1], "set_t"))  cmd = FOC_DOOR_CMD_SET_T;
    else {
        printf("未知命令: %s\n", argv[1]);
        return 1;
    }
    uint32_t arg = (argc > 2) ? (uint32_t)strtoul(argv[2], nullptr, 10) : 0;

    esp_err_t r = pa_wake_inject_cmd(cmd, arg);
    printf("本地注入 %s(arg=%u) → %s\n", foc_door_cmd_name(cmd), (unsigned)arg,
           esp_err_to_name(r));
    if (cmd == FOC_DOOR_CMD_NONE) {
        printf("(NONE 是心跳: 应用的过滤钩子会挡下它, 属预期 —— 看 `wl` 的 filtered 计数)\n");
    }
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
    ret |= platform_console_register("pa_send", "本地注入无线命令 (验事件→应用映射)", do_pa_send);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "部分应用命令注册失败");
    } else {
        ESP_LOGI(TAG, "应用命令已注册: open / close / stop / wake / mode / pa_send");
    }
    return ret;
}
