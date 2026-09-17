#pragma once

/*
 * 平台层事件总线
 *
 * ── 为什么需要它 ──────────────────────────────────────────────
 * 底层模块若直接调用应用层的 `power_state_*()`, 按键模块(驱动) 与命令台(工具)
 * 就都得知道"门的四态"。这是**层次倒置** —— 换个项目就得改驱动层。
 *
 * ⇒ 所有底层模块**只发事件, 不做决策**; 应用订阅并决定怎么做。
 * 基于 IDF 内置的 `esp_event` (官方机制, 不自己造)。
 *
 * ── 应用层怎么用 ──────────────────────────────────────────────
 *   static void on_focstep(void *arg, esp_event_base_t base, int32_t id, void *data) {
 *       switch (id) {
 *       case FOCSTEP_EVT_STALL: {
 *           const focstep_evt_stall_t *e = data;
 *           ... 决定停车/报警/忽略 ...
 *           break;
 *       }
 *       }
 *   }
 *   platform_events_init();
 *   esp_event_handler_instance_register(FOCSTEP_EVENT, ESP_EVENT_ANY_ID, on_focstep, NULL, NULL);
 *
 * ⚠️ 事件在**调用者的任务上下文**里同步派发 (esp_event 的默认行为)。
 *    所以处理器要短 —— 长活儿丢给队列/任务, 别在里面 delay。
 *    特别地, `FOCSTEP_EVT_NFAULT` 来自 ISR 之外的 nFAULT 任务上下文。
 */

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "esp_event.h"
#include "esp_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

ESP_EVENT_DECLARE_BASE(FOCSTEP_EVENT);

typedef enum {
    /* 按键。data = focstep_evt_button_t */
    FOCSTEP_EVT_BUTTON_SINGLE = 0,
    FOCSTEP_EVT_BUTTON_DOUBLE,
    FOCSTEP_EVT_BUTTON_LONG,

    /* 堵转/堵门。data = focstep_evt_stall_t。**只报一次** (锁存),
     * 解除由 ipropi_stall_reset() 触发 FOCSTEP_EVT_STALL_CLEARED。 */
    FOCSTEP_EVT_STALL,
    FOCSTEP_EVT_STALL_CLEARED,

    /* DRV8874 nFAULT 拉低 (两片线与)。data = NULL。平台层已自动拉低 nSLEEP。 */
    FOCSTEP_EVT_NFAULT,

    /* CAN 收到一条已解析的指令。data = focstep_evt_can_cmd_t */
    FOCSTEP_EVT_CAN_CMD,
    /* CAN 丢帧/总线关闭 */
    FOCSTEP_EVT_CAN_LOST,

    /* 电源状态变化。data = focstep_evt_state_t */
    FOCSTEP_EVT_STATE_CHANGED,

    /* 编码器持续读失败 (可能接线松/磁铁掉了/芯片坏) */
    FOCSTEP_EVT_ENCODER_UNHEALTHY,

    /* 母线电压越限。data = focstep_evt_vbus_t */
    FOCSTEP_EVT_VBUS_LOW,
    FOCSTEP_EVT_VBUS_HIGH,

    /* 回零完成 (成功或失败)。data = focstep_evt_home_t */
    FOCSTEP_EVT_HOMED,

    /* 从深睡被唤醒。data = focstep_evt_wake_t */
    FOCSTEP_EVT_WOKE,

    /* 无线链路 (BLE 周期广播) 收到一条给本板的载荷。
     * data = focstep_evt_pa_cmd_t。平台层只做校验/寻址/去重与投递,
     * **不解释 cmd 的含义** —— 命令语义见应用层协议 components/foc_door_link。
     * 应用可用 pa_wake_set_payload_filter() 滤掉不需要的包 (典型: 自己的心跳,
     * 逐包投递要多付约 13% 的 C_RX)。平台侧模块: pa_wake.c */
    FOCSTEP_EVT_PA_CMD,
    /* 无线唤醒链路的同步状态变化 (同步成功 / 失步重扫)。
     * data = focstep_evt_pa_sync_t。用于应用层判断"现在能不能被远程唤醒"。 */
    FOCSTEP_EVT_PA_SYNC,
} focstep_event_id_t;

typedef struct {
    uint32_t timestamp_ms;
} focstep_evt_base_t;

typedef struct {
    focstep_evt_base_t base;
    int      click_count;
    uint32_t hold_ms;
} focstep_evt_button_t;

typedef struct {
    focstep_evt_base_t base;
    float peak_ma; /* 触发时的电流矢量幅值峰值 */
} focstep_evt_stall_t;

typedef struct {
    focstep_evt_base_t base;
    int cmd; /* can_cmd_t */
} focstep_evt_can_cmd_t;

typedef struct {
    focstep_evt_base_t base;
    int from; /* power_state_t */
    int to;
} focstep_evt_state_t;

typedef struct {
    focstep_evt_base_t base;
    int vbus_mv;
} focstep_evt_vbus_t;

typedef struct {
    focstep_evt_base_t base;
    int result;  /* homing_result_t */
    float angle; /* 接触点 */
} focstep_evt_home_t;

typedef struct {
    focstep_evt_base_t base;
    int src; /* wake_src_t */
} focstep_evt_wake_t;

typedef struct {
    focstep_evt_base_t base;
    int      cmd;  /* **应用层协议值** (components/foc_door_link: FOC_DOOR_CMD_*) */
    uint32_t arg;  /* 该命令的参数 (含义随 cmd; 不用的命令为 0) */
} focstep_evt_pa_cmd_t;

typedef struct {
    focstep_evt_base_t base;
    int      synced;   /* 1 = 已建立周期同步, 0 = 未同步 (重扫中/已关闭) */
    uint32_t t_ms;     /* 当前实际生效的唤醒周期 T = per_adv_ival×(skip+1) */
    uint32_t lost_cnt; /* 累计失步次数 (判链路质量) */
} focstep_evt_pa_sync_t;

/* 初始化事件循环与事件基。可重复调用。 */
esp_err_t platform_events_init(void);

/* 便捷发送 (应用层也可用来发自己的事件) */
void platform_event_post(focstep_event_id_t id, const void *data, size_t len);

/* 单调毫秒时钟。发事件前填到 evt.base.timestamp_ms 里。 */
static inline uint32_t platform_now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

#ifdef __cplusplus
}
#endif
