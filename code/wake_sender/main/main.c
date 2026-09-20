/*
 * FocStepper 无线唤醒发送端 (BLE 周期广播 / PA)
 *
 * 定位: **临时发送端**。给驱动板发"唤醒/开/关/停"指令。
 * 载荷格式与驱动板共用 code/components/foc_link_protocol。
 *
 * ── 为什么是 PA ──────────────────────────────────────────────
 * 接收端常态是深睡, 只有 PA 能在 <1s 时延预算下进亚 mA:
 *   I_RX = floor + C_RX / T,  C_RX = 0.160 mA·s  (C6 实测)
 * 对比 legacy 扫描 ~5.1 mA·s (便宜约 32 倍), ESP-NOW ~4.42 mA·s。
 *
 * ── 发射端的约束 ─────────────────────────────────────────────
 * **发射端没有 skip 自由度**(它是发送方), 每个周期事件都必须发。
 * 所以 I_TX = floor + C_TX / itvl, C_TX = 0.134 mA·s ——
 * 双方都电池供电时, **约束在 TX, 不在 RX**。
 * 降发射功率无效: 实测 -9dBm 只让 C_TX 降 2%。
 *
 * ⚠️ 本发送端**不保证送达**(PA 是单向链路, 无 ACK)。靠 Kconfig 的
 *    SENDER_REPEAT 重复发送换可靠性 —— 比 PAwR 便宜得多。
 *    ⚠️ 重复帧必须**共用同一个 sequence**(见 s_seq 的注释), 否则接收端会把
 *       N 帧都当新指令 ⇒ 同一条 OPEN 执行 N 次。
 *
 * ── T 策略 (v2 新增) ──────────────────────────────────────────
 *   载荷 v2 加了 `arg` 字段与 FOC_DOOR_CMD_SET_T: 发送端可以动态改接收端的
 *   唤醒周期 T (接收端 T = per_adv_ival×(skip+1), 由它自己换算成 skip)。
 *   策略: 被运动唤醒 → T=SENDER_T_ACTIVE_MS(快响应); 倒计时无动作 → T_IDLE(省电)。
 *   ⚠️ 改 T **没有回执** —— 只能到接收端本地用 `wl status` 看实得值。
 *
 * ⚠️ 载荷**未加密**(EAD 未做)。receiver_id/session/sequence 只能防误触发,
 *    不能防伪造。量产前必须补。
 */

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_pm.h"
#include "esp_bt.h"
#include "esp_console.h"
#include "esp_timer.h"
#include "driver/gpio.h" /* 运动检测输入 (陀螺仪 INT, 见 SENDER_MOTION_GPIO) */
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_gap.h"
#include "host/ble_hs_adv.h"
#include "services/gap/ble_svc_gap.h"

#include "foc_door_link.h" /* 应用层指令集 (内含承载层 foc_link_protocol.h) */

static const char *TAG = "PA_TX";

#define ADV_INSTANCE 1
#define SID 2

static uint8_t s_own_addr_type;
static bool s_started = false;
static uint8_t s_session = 1;
static uint8_t s_pending_cmd = FOC_DOOR_CMD_NONE;
static uint16_t s_pending_arg = 0;
static uint16_t s_pending_seq = 0; /* 这条指令的序号 (N 个重复帧共用) */
static int s_repeat_left = 0;

/* ⚠️ 序号语义: **一条逻辑指令一个序号**, 重复发的 N 帧共用同一个。
 * 若每发一帧就 ++, 接收端的去重逻辑 (session+sequence 严格递增) 会把 N 帧都判成
 * "新指令" ⇒ 同一条 OPEN 被执行 N 次、唤醒流程走 N 遍。
 * ⇒ 接收端第 2..N 帧判 DUP 丢弃, "重复换可靠性"才成立。 */
static uint16_t s_seq = 0;

/* ── 运动驱动的 T 策略 (陀螺仪可插拔) ──────────────────────────
 * 被运动唤醒 → T 调小 (快响应); 倒计时内无新动作 → T 调回 (省电)。
 * 陀螺仪硬件未上时用命令台 `motion` 模拟; 接上后把它的 INT 接到
 * SENDER_MOTION_GPIO 即可 (轮询下降沿, 20Hz 足够判"有人动了")。
 * ⚠️ 低有效: LIS3DH 的 INT1 必须配成低有效 + 锁存 (doc_wake_sender.md §4.2)。 */
static bool s_motion_active = false;
static bool s_t_auto = true;
static TickType_t s_motion_since = 0;

void ble_store_config_init(void);

/* ------------------------------------------------------------------ */

static void build_payload(uint8_t *out, uint8_t cmd, uint16_t seq, uint16_t arg)
{
    foc_link_pkt_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic = FOC_LINK_MAGIC;
    pkt.version = FOC_LINK_VERSION;
    pkt.role = FOC_LINK_ROLE_CMD;
    pkt.receiver_id = CONFIG_SENDER_RECEIVER_ID;
    pkt.cmd = cmd;
    pkt.session = s_session;
    pkt.sequence = seq;
    pkt.arg = arg; /* SET_T 时 = 目标 T (ms); 其余指令填 0 */
    /* nonce 每次变化 —— 为将来的 EAD/防重放预留, 现在只用来区分包 */
    pkt.nonce = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);

    foc_link_encode(out, &pkt);
}

/* 更新周期广播数据。**可以在广播进行中调用** —— 这是 PA 发指令的方式:
 * 广播本身不停, 只换载荷。 */
static esp_err_t update_periodic_data(uint8_t cmd, uint16_t seq, uint16_t arg)
{
    uint8_t payload[FOC_LINK_PKT_LEN];
    build_payload(payload, cmd, seq, arg);

    struct os_mbuf *data = os_msys_get_pkthdr(sizeof(payload), 0);
    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_NO_MEM, TAG, "os_msys_get_pkthdr failed");

    int rc = os_mbuf_append(data, payload, sizeof(payload));
    if (rc != 0) {
        os_mbuf_free_chain(data);
        return ESP_FAIL;
    }

#if MYNEWT_VAL(BLE_PERIODIC_ADV_ENH)
    rc = ble_gap_periodic_adv_set_data(ADV_INSTANCE, data, NULL);
#else
    rc = ble_gap_periodic_adv_set_data(ADV_INSTANCE, data);
#endif
    if (rc != 0) {
        ESP_LOGE(TAG, "set_data failed rc=%d", rc);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void start_periodic_adv(void)
{
    struct ble_gap_ext_adv_params params;
    struct ble_gap_periodic_adv_params pparams;
    struct ble_hs_adv_fields adv_fields;
    struct os_mbuf *data;
    ble_addr_t addr;
#if MYNEWT_VAL(BLE_PERIODIC_ADV_ENH)
    struct ble_gap_periodic_adv_enable_params eparams;
    memset(&eparams, 0, sizeof(eparams));
#endif
    int rc;

    rc = ble_hs_id_gen_rnd(1, &addr);
    assert(rc == 0);

    memset(&params, 0, sizeof(params));
    params.own_addr_type = BLE_OWN_ADDR_RANDOM;
    params.primary_phy = BLE_HCI_LE_PHY_1M;
    params.secondary_phy = BLE_HCI_LE_PHY_2M;
    params.sid = SID;

    rc = ble_gap_ext_adv_configure(ADV_INSTANCE, &params, NULL, NULL, NULL);
    assert(rc == 0);

    rc = ble_gap_ext_adv_set_addr(ADV_INSTANCE, &addr);
    assert(rc == 0);

    /* 主广播只放名字 —— 接收端靠周期广播拿载荷 */
    memset(&adv_fields, 0, sizeof(adv_fields));
    adv_fields.name = (const uint8_t *)"FocStep";
    adv_fields.name_len = strlen((char *)adv_fields.name);

    data = os_msys_get_pkthdr(BLE_HCI_MAX_ADV_DATA_LEN, 0);
    assert(data);
    rc = ble_hs_adv_set_fields_mbuf(&adv_fields, data);
    assert(rc == 0);
    rc = ble_gap_ext_adv_set_data(ADV_INSTANCE, data);
    assert(rc == 0);

    /* 周期广播参数。
     * ⚠️ itvl_min 必须 == itvl_max —— 见 Kconfig 说明。 */
    memset(&pparams, 0, sizeof(pparams));
    pparams.include_tx_power = 0;
    pparams.itvl_min = BLE_GAP_PERIODIC_ITVL_MS(CONFIG_SENDER_PA_ITVL_MS);
    pparams.itvl_max = BLE_GAP_PERIODIC_ITVL_MS(CONFIG_SENDER_PA_ITVL_MS);

    rc = ble_gap_periodic_adv_configure(ADV_INSTANCE, &pparams);
    assert(rc == 0);

    /* 起始载荷 = 心跳 (CMD_NONE) */
    s_seq++;
    ESP_ERROR_CHECK(update_periodic_data(FOC_DOOR_CMD_NONE, s_seq, 0));

#if MYNEWT_VAL(BLE_PERIODIC_ADV_ENH)
    rc = ble_gap_periodic_adv_start(ADV_INSTANCE, &eparams);
#else
    rc = ble_gap_periodic_adv_start(ADV_INSTANCE);
#endif
    assert(rc == 0);

    rc = ble_gap_ext_adv_start(ADV_INSTANCE, 0, 0);
    assert(rc == 0);

    s_started = true;
    ESP_LOGI(TAG, "PA started: itvl=%dms sid=%u recv_id=0x%X",
             CONFIG_SENDER_PA_ITVL_MS, SID, CONFIG_SENDER_RECEIVER_ID);
}

static void on_reset(int reason)
{
    ESP_LOGE(TAG, "resetting state; reason=%d", reason);
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);
    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "error determining address type; rc=%d", rc);
        return;
    }
    start_periodic_adv();
}

static void host_task(void *param)
{
    ESP_LOGI(TAG, "BLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ------------------------------------------------------------------ */
/* 指令派发: 一条指令重复发 N 次 (PA 无 ACK, 靠重复换可靠性)            */

static void request_command(uint8_t cmd, uint16_t arg)
{
    const char *name = foc_door_cmd_name(cmd);
    if (!s_started) {
        printf("PA 还没起来, 稍后再试\n");
        return;
    }
    s_pending_cmd = cmd;
    s_pending_arg = arg;
    s_repeat_left = CONFIG_SENDER_REPEAT;
    /* ★ 一条逻辑指令一个序号: N 个重复帧共用, 接收端只认第一帧 */
    s_pending_seq = ++s_seq;
    printf("已排队 %s (arg=%u) 序号=%u, 重复 %d 次 × %d ms\n",
           name, arg, s_pending_seq, CONFIG_SENDER_REPEAT, CONFIG_SENDER_PA_ITVL_MS);
}

/* 每 50ms 跑一次: 推进重复计数, 发完就回到心跳 */
static void dispatch_tick(void)
{
    static int64_t next_us = 0;
    if (s_repeat_left <= 0) {
        return;
    }
    int64_t now = esp_timer_get_time();
    if (now < next_us) {
        return;
    }
    /* 按 PA 间隔推, 保证每个周期事件都带上这条指令 (载荷不同 => 每个周期都发新的) */
    next_us = now + (int64_t)CONFIG_SENDER_PA_ITVL_MS * 1000;

    if (update_periodic_data(s_pending_cmd, s_pending_seq, s_pending_arg) == ESP_OK) {
        s_repeat_left--;
        if (s_repeat_left == 0) {
            s_pending_cmd = FOC_DOOR_CMD_NONE;
            s_pending_arg = 0;
            s_seq++;
            update_periodic_data(FOC_DOOR_CMD_NONE, s_seq, 0);
            ESP_LOGI(TAG, "指令发送完毕, 回到心跳");
        }
    }
}

/* ------------------------------------------------------------------ */
/* 运动驱动的 T 策略                                                    */

/* 自动策略: 运动 → T_ACTIVE; 静止 hold 时间 → T_IDLE。
 * 手动 `t <ms>` 会关掉自动 (`t auto` 恢复)。 */
static bool policy_apply_t(uint32_t t_ms, const char *why)
{
    /* T 走协议里的 16 位参数域 ⇒ 上限就是 65535 ms。
     * ⚠️ 不能靠 (uint16_t) 截断: 70000 会静默变成 4464, 接收端照单全收,
     *    唤醒周期直接差 15 倍, 而两端日志都看不出异常。 */
    if (t_ms == 0) {
        ESP_LOGW(TAG, "T=0 无效 (接收端会回落到 Kconfig 默认值), 已忽略");
        return false;
    }
    if (t_ms > 0xFFFFu) {
        ESP_LOGW(TAG, "T=%u ms 超出协议字段 (最大 65535), 已忽略", (unsigned)t_ms);
        return false;
    }
    ESP_LOGI(TAG, "T → %u ms (%s)", (unsigned)t_ms, why);
    request_command(FOC_DOOR_CMD_SET_T, (uint16_t)t_ms);
    return true;
}

static void policy_on_motion(void)
{
    s_motion_since = xTaskGetTickCount();
    if (!s_t_auto || s_motion_active) {
        return;
    }
    s_motion_active = true;
    ESP_LOGI(TAG, "检测到运动 (陀螺仪/模拟)");
    policy_apply_t(CONFIG_SENDER_T_ACTIVE_MS, "运动");
}

static void policy_tick(void)
{
    /* 运动检测输入: **低有效** —— 按键/干接点/运动三件事共用一次 ext1 的
     * "任意低"口径 (doc_wake_sender.md §4.2/§4.8), 所以这里数的是**下降沿**。
     * 未接 (GPIO<0) 时只靠命令台 `motion` 模拟。 */
#if CONFIG_SENDER_MOTION_GPIO >= 0
    /* 初值 1: 上电前就有动作 (INT 已是低) 时, 第一个采样点也算一次。 */
    static int last_level = 1;
    int lvl = gpio_get_level((gpio_num_t)CONFIG_SENDER_MOTION_GPIO);
    if (!lvl && last_level) {
        policy_on_motion();
    }
    last_level = lvl;
    /* ⚠️ INT1 是**锁存**低有效: 一段低电平只算一次动作 —— 正确, 因为
     * "有人动了"本来就是事件而非持续状态。清锁存 (读 INT1_SRC) 由 LIS3DH
     * 驱动负责, 见 §4.2 纪律二; 不清则第二次动作看不到新的下降沿。 */
#endif

    if (s_t_auto && s_motion_active &&
        (xTaskGetTickCount() - s_motion_since) > pdMS_TO_TICKS(CONFIG_SENDER_MOTION_HOLD_MS)) {
        s_motion_active = false;
        policy_apply_t(CONFIG_SENDER_T_IDLE_MS, "运动倒计时结束, 转省电");
    }
}

/* ------------------------------------------------------------------ */
/* 控制台                                                              */

static int cmd_open(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    request_command(FOC_DOOR_CMD_OPEN, 0);
    return 0;
}
static int cmd_close(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    request_command(FOC_DOOR_CMD_CLOSE, 0);
    return 0;
}
static int cmd_stop(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    request_command(FOC_DOOR_CMD_STOP, 0);
    return 0;
}
static int cmd_wake(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    request_command(FOC_DOOR_CMD_WAKE, 0);
    return 0;
}
static int cmd_heartbeat(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    s_repeat_left = 0;
    s_pending_cmd = FOC_DOOR_CMD_NONE;
    s_pending_arg = 0;
    s_seq++;
    update_periodic_data(FOC_DOOR_CMD_NONE, s_seq, 0);
    printf("回到心跳 (CMD_NONE), 序号 %u\n", s_seq);
    return 0;
}
static int cmd_payload(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    uint8_t buf[FOC_LINK_PKT_LEN];
    build_payload(buf, s_pending_cmd, s_pending_seq ? s_pending_seq : s_seq, s_pending_arg);
    printf("载荷 %d 字节: ", FOC_LINK_PKT_LEN);
    for (int i = 0; i < FOC_LINK_PKT_LEN; i++) {
        printf("%02X ", buf[i]);
    }
    printf("\n(little-endian; magic=0x%04X ver=%d recv=0x%X session=%u seq=%u arg=%u)\n",
           FOC_LINK_MAGIC, FOC_LINK_VERSION, CONFIG_SENDER_RECEIVER_ID,
           s_session, s_pending_seq, s_pending_arg);
    return 0;
}

/* ---- T 策略命令 (陀螺仪未上时的手动/模拟入口) ---- */
static int cmd_t(int argc, char **argv)
{
    if (argc > 1 && !strcmp(argv[1], "auto")) {
        s_t_auto = true;
        s_motion_active = false;
        printf("T 策略 → 自动: 运动 %d ms / 静止 %d ms (倒计时 %d ms)\n",
               CONFIG_SENDER_T_ACTIVE_MS, CONFIG_SENDER_T_IDLE_MS,
               CONFIG_SENDER_MOTION_HOLD_MS);
        return 0;
    }
    if (argc > 1) {
        uint32_t ms = (uint32_t)strtoul(argv[1], NULL, 10);
        /* 先确认值被接纳再切"手动": 否则一个打错的 T 会白白关掉自动策略 */
        if (!policy_apply_t(ms, "手动")) {
            printf("未生效: T 的合法范围是 1~65535 ms\n");
            return 1;
        }
        s_t_auto = false;
        printf("T 策略 → 手动 (t auto 恢复自动)\n");
        return 0;
    }
    printf("T 策略: %s | 运动时 %d ms, 静止时 %d ms, 倒计时 %d ms\n",
           s_t_auto ? "自动" : "手动",
           CONFIG_SENDER_T_ACTIVE_MS, CONFIG_SENDER_T_IDLE_MS,
           CONFIG_SENDER_MOTION_HOLD_MS);
    printf("注意: 改 T 只在**接收端**生效 (PA 是单向链路, 没有回执); "
           "接收端用 `wl status` 看实得值\n");
    return 0;
}

static int cmd_motion(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    s_t_auto = true;
    policy_on_motion();
    printf("已模拟一次运动唤醒 (T → %d ms)\n", CONFIG_SENDER_T_ACTIVE_MS);
    return 0;
}
static int cmd_session(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    s_session++;
    s_seq = 0; /* 新 session ⇒ 接收端无条件接受并重置序号基线 */
    s_repeat_left = 0;
    s_seq++;
    update_periodic_data(FOC_DOOR_CMD_NONE, s_seq, 0);
    printf("session → %u, sequence 归零\n", s_session);
    return 0;
}

static void console_start(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.prompt = "pa_tx>";
    repl_cfg.task_priority = 2;

    esp_console_dev_uart_config_t dev_cfg = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&dev_cfg, &repl_cfg, &repl));

    const esp_console_cmd_t cmds[] = {
        {.command = "open",      .help = "发 OPEN 指令",        .func = cmd_open},
        {.command = "close",     .help = "发 CLOSE 指令",       .func = cmd_close},
        {.command = "stop",      .help = "发 STOP 指令",        .func = cmd_stop},
        {.command = "wake",      .help = "发 WAKE 指令",        .func = cmd_wake},
        {.command = "heartbeat", .help = "回到 CMD_NONE 心跳",  .func = cmd_heartbeat},
        {.command = "payload",   .help = "打印当前载荷十六进制", .func = cmd_payload},
        {.command = "session",   .help = "递增 session 并归零 sequence", .func = cmd_session},
        {.command = "t",         .help = "t [ms|auto] 设/查唤醒周期 T", .func = cmd_t},
        {.command = "motion",    .help = "模拟一次陀螺仪运动唤醒",     .func = cmd_motion},
    };
    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

/* ------------------------------------------------------------------ */

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

#if CONFIG_PM_ENABLE
    /* 广播事件由控制器自行调度, host 不参与 ⇒ 事件之间可以轻睡。
     * 发射端没有 skip 自由度, 所以它的功耗 ~= 地板 + C/itvl。 */
    esp_pm_config_t pm_config = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true
#endif
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
#endif

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", ret);
        return;
    }

#if CONFIG_SENDER_TX_POWER_N9
    /* ⚠️ 实测只省 2% —— 这不是省电手段, 是缩小覆盖做区域隔离用的。 */
    esp_err_t perr = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N9);
    ESP_LOGI(TAG, "ADV TX power = -9dBm, rc=%d", (int)perr);
#endif

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

#if CONFIG_BT_NIMBLE_GAP_SERVICE
    /* GAP service 只在 BT_NIMBLE_GATT_SERVER 打开时存在, 后者 depends on
     * BT_NIMBLE_ROLE_PERIPHERAL。纯广播端会关掉 peripheral ⇒ 必须守卫,
     * 否则链接期 undefined reference。 */
    int rc = ble_svc_gap_device_name_set("focstep_pa_tx");
    assert(rc == 0);
#endif

    ble_store_config_init();
    nimble_port_freertos_init(host_task);

#if CONFIG_SENDER_MOTION_GPIO >= 0
    /* 运动检测输入 (陀螺仪 INT): 只做输入轮询, 20Hz 足够判"有人动了"。
     * 陀螺仪换型/换实现时只改这里, 策略代码 (policy_*) 不动。
     * 低有效 ⇒ 上拉: 传感器没贴/没接时读高, 不会误报"有人动了"。 */
    {
        gpio_config_t mg = {
            .pin_bit_mask = 1ULL << CONFIG_SENDER_MOTION_GPIO,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&mg));
        ESP_LOGI(TAG, "运动检测输入 = GPIO%d", CONFIG_SENDER_MOTION_GPIO);
    }
#endif

    console_start();
    ESP_LOGI(TAG, "就绪。指令: open / close / stop / wake / heartbeat / payload");
    ESP_LOGI(TAG, "⚠️ 发送端无 ACK, 靠 Kconfig 的 SENDER_REPEAT=%d 重复发送保证送达",
             CONFIG_SENDER_REPEAT);

    while (1) {
        dispatch_tick();
        policy_tick();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
