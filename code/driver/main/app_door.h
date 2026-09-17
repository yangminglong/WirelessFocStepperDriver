#pragma once

/*
 * 推拉门应用 —— 本项目的全部应用语义都在这里
 *
 * 平台层 (components/focstep_platform) 只提供通用能力; 下面这些概念**只存在于本文件**:
 *   · "全开 / 全关"、"手拉助动"、"堵门保持"
 *   · docs/doc.md §10.3 的四态语义 (平台只有 SLEEP/ACTIVE/FAULT 三态)
 *   · 灯色映射、按键语义、CAN 指令语义
 *
 * 换一个项目 (云台/机械臂/闸机) 就换掉这个文件, 平台层不用动。
 *
 * ── §10.3 四态如何映射到平台三态 ──────────────────────────────
 * ⚠️ **"CAN Rs" 不是应用可控制的一列**: Rs 挂在 nSLEEP 的派生网络上
 *    (硬件派生, 见 docs/doc.md §5.3), 软件不控。下表那一列只说明**实际会是什么状态**。
 *
 * | §10.3 状态  | 平台 power_state | 应用子模式 | CAN 实际状态 (随 nSLEEP) |
 * |------------|------------------|-----------|------------------------|
 * | 深睡        | SLEEP            | —         | 睡眠 (nSLEEP 低)        |
 * | 唤醒接管    | ACTIVE           | ASSIST    | **工作** (nSLEEP 高)    |
 * | 运行        | ACTIVE           | RUNNING   | 工作                    |
 * | 故障        | FAULT            | —         | 睡眠 (nSLEEP 低)        |
 */

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    APP_CMD_NONE = 0,
    APP_CMD_OPEN,
    APP_CMD_CLOSE,
    APP_CMD_STOP,
    APP_CMD_WAKE, /* 唤起但不动作 */
} app_cmd_t;

/* 订阅平台事件 + 建立初始状态 */
esp_err_t app_door_init(void);

/* 应用级周期任务 (100ms): 助动方向跟随、深睡决策、闪灯 */
void app_door_tick(void);

/* 下达应用指令 (按键/CAN/无线 最终都汇到这里) */
void app_door_command(app_cmd_t cmd);

/* 应用子模式 */
typedef enum {
    APP_MODE_IDLE = 0, /* 非 ACTIVE, 或 ACTIVE 但无所事事 */
    APP_MODE_ASSIST,   /* 唤醒接管: 跟随手拉方向助动 */
    APP_MODE_RUNNING,  /* 运行: 走位置环 */
} app_mode_t;

app_mode_t app_door_mode(void);
const char *app_door_mode_name(app_mode_t m);

#ifdef __cplusplus
}
#endif
