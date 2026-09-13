#pragma once

/*
 * 平台命令台 + **扩展点**
 *
 * ── 为什么需要 ────────────────────────────────────────────────
 * 重构前 `cmd_console.cpp` 把**平台自检** (id/regs/xyz/ipropi/vref/…) 和
 * **应用命令** (home/learn/open/close/…) 注册在同一个函数里, 没有扩展点 ——
 * 新项目要加自己的命令**必须改驱动层文件**。
 *
 * 现在: 平台注册自己的那一套, 应用通过 `platform_console_register()` 追加。
 *
 * ── 应用层怎么用 ──────────────────────────────────────────────
 *   platform_console_init();                 // 建 REPL + 注册平台命令
 *   platform_console_register("open", "开门", my_open_cb);
 *   platform_console_register("close", "关门", my_close_cb);
 *
 * ⚠️ 必须先 init 再 register (REPL 未建立时注册会失败并打日志)。
 */

#include <stdbool.h>
#include "esp_err.h"
#include "esp_console.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 建立 REPL 并注册**平台级**自检命令 (受 FOCSTEP_PLATFORM_CONSOLE_ENABLE 控制)。
 * 命令任务跑在**低优先级** —— FOC 循环是最高优先级, 不能被日志拖慢。
 */
esp_err_t platform_console_init(void);

/*
 * 追加注册一条命令。应用层用。
 * 失败返回非 0 (通常是名字重复或 REPL 未建立)。
 */
esp_err_t platform_console_register(const char *name, const char *help,
                                    esp_console_cmd_func_t fn);

/* REPL 是否已就绪 (register 前可用它判断) */
bool platform_console_ready(void);

#ifdef __cplusplus
}
#endif
