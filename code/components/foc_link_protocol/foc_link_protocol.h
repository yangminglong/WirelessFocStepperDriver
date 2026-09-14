#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * FocStepper 无线链路**承载层**协议 (帧格式 + 收发工具)
 *
 * ⚠️ 分层纪律: 本层**只搬字节, 不解释业务**。
 *    `cmd` / `arg` 对这里是不透明的两个字段; "开/关/停/唤醒周期" 这类
 *    **应用层命令集定义在 components/foc_door_link** (两个工程共用同一份)。
 *    底层不定义具体应用命令 —— 换产品时只换上层那一个头文件, 帧格式不动。
 *
 * 载体: BLE 周期广播 (Periodic Advertising) 的 periodic adv data。
 *   PA 数据最长 254B, 本协议只用 20B —— 余量留给将来扩展 (EAD 加密等)。
 *   两端共用一个头文件, 避免各写一份导致线格式漂移。
 *
 * 设计依据: ESP32_wakeup_by_BLE_adv/tests/ble_sync
 *   - 收发成本模型 I = f + C/T, C_RX = 0.160 mA·s (C6 实测)
 *   - 接收端常态是"没有事件", 故载荷必须能表达"无动作" (由上层约定取值)
 *
 * ── v2 (当前) 相对 v1 的变化 ──────────────────────────────────
 *   1. 新增 `arg` 字段 (16 位), 供**上层协议**携带一个参数 (v1 的 18B → 20B)。
 *   2. version 1 → 2。**两端必须同时升级**: 版本不匹配时 decode 直接判"非本协议",
 *      不会把 v1 包误解析成 v2 (反之亦然); 对端没升级会打出专门告警, 见 peek_version()。
 *
 * ⚠️ 本协议目前**未加密** (EAD 未做), receiver_id/session/sequence 只能防误触发,
 *    不能防伪造。量产前须补 EAD 或换 128-bit Service Data + 认证。
 */

/* 固定标识。改协议线格式时必须同时改 magic 或 version, 否则新旧固件互认。 */
#define FOC_LINK_MAGIC   0xF0C5
#define FOC_LINK_VERSION 2

/* CRC 覆盖范围: magic 起, 到 crc16 字段之前 */
#define FOC_LINK_CRC_COVER_LEN 18
#define FOC_LINK_PKT_LEN       20

/* role: 通道号 (承载层只做多路复用, 通道里跑什么由上层定)。
 *   1 = 指令通道 (常规业务)  2 = 复位通道 (保留: 便于接收端在故障态也能响应) */
#define FOC_LINK_ROLE_CMD 1
#define FOC_LINK_ROLE_RST 2

/*
 * ⚠️ 必须 packed: 这是 20B 线格式的内存视图, encode/decode 按固定偏移逐字节读写。
 *    不加 packed 时编译器会插入填充, 用 sizeof 做长度校验的一侧会把合法包误判为格式错误。
 *    (tests/components/adv_protocol 上实测踩过同类问题, 见其注释)
 */
typedef struct __attribute__((packed)) {
    uint16_t magic;       /* FOC_LINK_MAGIC */
    uint8_t  version;     /* FOC_LINK_VERSION */
    uint8_t  role;        /* FOC_LINK_ROLE_* (通道) */
    uint32_t receiver_id; /* 0 = 广播给所有板; 否则按板寻址 */
    uint8_t  cmd;         /* **上层协议定义** (见 components/foc_door_link); 本层不解释 */
    uint8_t  session;     /* 发送端每次启动递增; 接收端丢弃旧 session */
    uint16_t sequence;    /* 同一 session 内严格递增, 用于去重 */
    uint32_t nonce;       /* 每次发包变化, 为 EAD 预留 */
    uint16_t arg;         /* **上层协议定义**的参数 (含义随 cmd); 不用的命令填 0 */
    uint16_t crc16;       /* CRC-16/CCITT-FALSE, 覆盖前 18B */
} foc_link_pkt_t;

_Static_assert(sizeof(foc_link_pkt_t) == FOC_LINK_PKT_LEN,
               "foc_link_pkt_t must match FOC_LINK_PKT_LEN exactly");

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, 无反射, 无输出异或 */
uint16_t foc_link_crc16(const uint8_t *data, size_t len);

/* 编码为 20B 线格式到 out (须 >= FOC_LINK_PKT_LEN)。pkt->crc16 被忽略, 由本函数计算。 */
void foc_link_encode(uint8_t *out, const foc_link_pkt_t *pkt);

/*
 * 解码 20B 线格式。
 * 返回 true 表示"magic/version/长度都合法" —— **不代表 CRC 正确**。
 * 调用者必须再比较 *crc_ok, 这样"CRC 错"与"不是本协议的包"能被区分开
 * (前者说明链路有问题, 后者只是收到了别人的广播)。
 */
bool foc_link_decode(const uint8_t *in, size_t len,
                     foc_link_pkt_t *pkt, bool *crc_ok);

/* 接收端去重/防重放。调用者持有状态, 便于放进深睡唤醒后的 RAM 或 NVS。 */
typedef struct {
    uint8_t  session;
    uint16_t sequence;
    bool     valid;
} foc_link_rx_state_t;

typedef enum {
    FOC_LINK_ACCEPT = 0,     /* 新指令, 应执行 */
    FOC_LINK_DUP,            /* 重复包 (同 session 且 sequence 未前进), 丢弃 */
    FOC_LINK_OLD_SESSION,    /* 旧 session, 丢弃 */
    FOC_LINK_NOT_FOR_ME,     /* receiver_id 不匹配 */
} foc_link_rx_result_t;

/*
 * 判定一个已通过 CRC 校验的包是否应被接受。
 * 接受时内部状态前进; 被拒时状态不变。
 * 语义: sequence 必须**严格大于**上次; session 变化时重置 sequence 基线,
 *       但**只接受更新的 session** (按 uint8 环形比较, 允许回绕)。
 */
foc_link_rx_result_t foc_link_rx_filter(foc_link_rx_state_t *st,
                                        const foc_link_pkt_t *pkt,
                                        uint32_t my_receiver_id);

/* 供日志/自检使用 —— ⚠️ 命令名的映射属于**应用层**, 见 components/foc_door_link */

/*
 * 只看 magic + version (前 3 字节), 不校验长度与 CRC。
 * 返回: 0 = 不是本协议 (或长度不足); 否则是包里的版本号。
 * 用途: decode() 失败时区分两种原因 ——
 *   "版本不符"(对端固件没跟着升级, 必须同时重烧) 与 "别人的广播"(正常, 静默丢)。
 *   少了这个区分, 版本不匹配在现场表现为"什么都没收到", 极难定位。
 */
uint8_t foc_link_peek_version(const uint8_t *in, size_t len);
