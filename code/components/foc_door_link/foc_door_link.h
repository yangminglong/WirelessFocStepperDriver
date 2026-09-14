#pragma once

/*
 * 推拉门无线指令集 —— **应用层**协议 (两个工程共用: 驱动板 <-> 无线发送端)
 *
 * ⚠️ 为什么单独一个组件, 而不是塞进承载层:
 *    承载层 (components/foc_link_protocol) **只搬字节、不解释业务**;
 *    "开/关/停/唤醒接管/唤醒周期" 是**门这个产品**的概念。
 *    换产品 (云台/机械臂/...) 时只换本头文件的命令集, 帧格式与射频那边一行不动。
 *
 * 载荷字段对应: pkt.cmd = FOC_DOOR_CMD_*,  pkt.arg = 该命令的参数 (不需要则 0)。
 *
 * 命令语义:
 *   NONE  纯心跳: 不产生任何动作。**接收端不要把它当事件上报**(省 host 侧开销);
 *         发送端空闲时持续发它 —— 它同时是"链路还在"的凭据。
 *   WAKE  唤起驱动板进入"唤醒接管"(读角度、准备助动), 本身不动作。
 *   OPEN / CLOSE / STOP  开关停 —— 与本地按键、CAN 指令**同一个落点**。
 *   SET_T 调接收端唤醒周期: arg = 期望 T (ms); 0 = 用接收端默认值。
 *         接收端实际只能取到 per_adv_ival × (skip+1) 的整数倍 (见 pa_wake.h)。
 *
 * ⚠️ 加/改命令就改这里 (两端必须同时升级); 承载层的 `FOC_LINK_VERSION`
 *    只在**帧格式**变化时才动。
 */

#include <stdint.h>

#include "foc_link_protocol.h" /* 载荷结构体 (pkt.cmd / pkt.arg 就是本文件的命令与参数) */

#ifdef __cplusplus
extern "C" {
#endif

/* 应用命令集。数值是线格式的一部分 —— 改动等同改协议。 */
#define FOC_DOOR_CMD_NONE    0 /* 纯心跳, 不产生动作 */
#define FOC_DOOR_CMD_WAKE    1 /* 进入唤醒接管, 不动作 */
#define FOC_DOOR_CMD_OPEN    2
#define FOC_DOOR_CMD_CLOSE   3
#define FOC_DOOR_CMD_STOP    4
#define FOC_DOOR_CMD_SET_T   5 /* arg = 期望唤醒周期 T (ms), 0 = 接收端默认 */

/* 命令名 —— 供日志/自检使用 */
const char *foc_door_cmd_name(uint8_t cmd);

#ifdef __cplusplus
}
#endif
