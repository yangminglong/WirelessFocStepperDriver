#pragma once

/*
 * 无线唤醒接收端 (BLE 周期广播 / PA)
 *
 * 定位: 平台层能力 —— "让驱动板能被远程唤醒"。
 *   本地唤醒源仍归 wakeup.c (ext1 / 编码器 INT); 本模块只管射频那一半,
 *   并且**只发事件、不做决策** (收到指令后干什么由应用层定)。
 *
 * ── 为什么是 PA (依据 tests/ble_sync 实测, C6) ──────────────────
 *   深睡下射频无法保持接收 (wakeup.h 已澄清这一条), 任何无线唤醒都得让芯片
 *   停在**轻睡**。PA 让接收端建立周期同步, 收包调度由 BLE 控制器自己完成:
 *
 *       I_RX = f + C_RX / T        C_RX = 0.160 mA·s (实测)
 *       T    = per_adv_ival × (skip + 1)
 *
 *   对比 legacy 扫描每次唤醒 ~5.1 mA·s, PA 便宜约 32 倍。
 *   ⇒ **T 是唯一的"功耗-时延"旋钮**, 可由对端用一条带参命令动态改
 *     (命令语义在应用层: components/foc_door_link; 平台只提供 set_T_ms 接口)。
 *
 * ⚠️ 分层: 本模块**不解释载荷内容** —— `cmd`/`arg` 原样投递给上层 (事件),
 *    命令名与语义由应用层协议定义。平台只做"要不要打扰上层"的过滤 (钩子由应用装)。
 *
 * ── 与睡眠的关系 (最容易踩的一处) ─────────────────────────────
 *   监听期必须让**控制器自己排唤醒** ⇒ 需要 PM 自动轻睡
 *   (esp_pm_configure(light_sleep_enable=true)), **不能**用显式
 *   esp_light_sleep_start() (那条路只认我们配的冷唤醒源, 且 BT 未停时的
 *   手动轻睡在 IDF 里是受限用法)。
 *   ⇒ Kconfig 已把 FOCSTEP_SLEEP_MODE_LIGHT 与 PA 做成互斥;
 *     监听期由 app_main 的"监听档"分支负责不进显式睡眠 (见 code/README.md §13)。
 *
 * ── 模式 (运行时可选; 上电默认值由应用决定) ──────────────────
 *   PA_WAKE_OFF: 停同步 + 关 PM 轻睡, 回 µA 档深睡
 *   PA_WAKE_PA : 建同步 + 持续监听 (亚 mA 级, 按 T 分档)
 *   预留: 将来加 ESP-NOW 模式 (tests/espnow_wakeup 已验证, 功耗高一个量级)
 *
 * ⚠️ 载荷未加密 (EAD 未做) ⇒ receiver_id/session/sequence 只能防误触发, 不能防伪造。
 * ⚠️ 应用层纪律: PS_SLEEP 且监听开着时, 主循环必须放慢到 FOCSTEP_PA_TICK_MS,
 *    否则每 100ms 一次唤醒会把自动轻睡搅碎, 监听功耗反而比 T=0.5s 的理论值高一个量级。
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PA_WAKE_OFF = 0, /* 不监听 (µA 档深睡, 只能本地唤醒) */
    PA_WAKE_PA,      /* BLE 周期广播监听 */
} pa_wake_mode_t;

typedef struct {
    pa_wake_mode_t mode;
    bool     host_ready;   /* NimBLE host 已同步 (地址就绪) */
    bool     synced;       /* 已建立周期同步 */
    bool     slow_scan;    /* 扫描已从"快扫"降到"慢扫"(对端不在场) */
    uint8_t  sid;          /* 周期广播 SID */
    uint8_t  skip;         /* 当前 skip */
    uint16_t per_adv_ival_ms; /* 发射端的周期广播间隔 (由同步事件读到) */
    uint32_t t_ms;         /* **实际**生效的 T = per_adv_ival_ms × (skip+1) */
    uint32_t t_want_ms;    /* 期望的 T (SET_T 或 Kconfig 给的) */
    int8_t   rssi;         /* 最近一次周期报文的 RSSI */
    uint32_t lost_cnt;     /* 累计失步次数 (判链路质量) */
    uint32_t pkt_ok;       /* 通过 CRC 且寻址到本板的包数 */
    uint32_t cmd_cnt;      /* 投递给上层的载荷数 (过滤后) */
    uint32_t filtered;     /* 被应用过滤钩子挡下的包数 (典型: 心跳) */
    /* 丢弃计数 —— 现场分得清"没收到"到底是"链路噪声/别人家的包/版本不符" */
    uint32_t drop_crc;     /* CRC 错 */
    uint32_t drop_dup;     /* 重复帧 (发送端重复发 N 次, 只认第一条) */
    uint32_t drop_not_me;  /* receiver_id 不是本板 */
    uint32_t drop_badver;  /* 版本不符 (对端固件没跟着升级) */
    uint32_t drop_noise;   /* 不是本协议 (别人的广播) */
} pa_wake_status_t;

/*
 * 初始化。只建 host 与控制器, **不启射频** (射频由 set_mode 决定)。
 * @param my_receiver_id 本板 ID (应用传 CONFIG_FOCSTEP_RECEIVER_ID);
 *                       0 在协议里表示广播, 不可作为本板 ID。
 * ⚠️ 需在 nvs_flash_init() 之后调用 (BLE 的 PHY 校准数据存 NVS)。
 */
esp_err_t pa_wake_init(uint32_t my_receiver_id);

/* 运行时开关。PA_WAKE_PA 会起扫描→建同步→开 PM 自动轻睡;
 * PA_WAKE_OFF 会停同步→关 PM 轻睡。可反复调用。 */
esp_err_t pa_wake_set_mode(pa_wake_mode_t mode);
pa_wake_mode_t pa_wake_mode(void);

/*
 * 设置期望的唤醒周期 T (ms)。
 *   · 实际值只能取 per_adv_ival × (skip+1), 故会四舍五入到最近可达值
 *     (发射端 240ms 时: T=500 → skip=1 → 实得 480ms; T=3000 → skip=12 → 实得 3120ms)
 *   · 若已同步: terminate → 等失步确认 → 用新 skip 重建 (中间有 0.5~1s 空窗)
 *   · 若未同步: 只记期望值, 同步成功后自动套用
 * @param t_ms 0 = 用 Kconfig 默认值
 * ⚠️ 别频繁调: 每次重建都有空窗, 发送端侧也做了最小间隔限制。
 */
esp_err_t pa_wake_set_T_ms(uint32_t t_ms);

/* 实际生效的 T (未同步时返回期望值) */
uint32_t pa_wake_T_ms(void);

void pa_wake_status(pa_wake_status_t *out);

/*
 * 本地注入一条无线指令 (自检用): 不经过射频, 直接走平台的投递路径。
 * 用来单独验"命令 → 事件 → 应用映射"这一链, 与真链路的问题分开定位。
 * ⚠️ cmd 的取值是**应用层协议**的事 (components/foc_door_link) —— 平台不看它。
 */
esp_err_t pa_wake_inject_cmd(uint8_t cmd, uint32_t arg);

/*
 * 载荷过滤钩子 (可选)。平台对**每个通过校验的包**调用它;
 * 返回 false 表示"不必打扰上层" (典型: 对端的心跳)。
 * 为什么需要它: 逐包都发事件会让 host 侧多耗约 13% 的 C_RX (实测), 而心跳是常态。
 * ⚠️ 回调在 NimBLE host 任务上下文执行, **必须短小、不得阻塞**; 传 NULL 恢复"全部投递"。
 */
typedef bool (*pa_wake_payload_filter_t)(uint8_t cmd, uint16_t arg);
void pa_wake_set_payload_filter(pa_wake_payload_filter_t cb);

#ifdef __cplusplus
}
#endif
