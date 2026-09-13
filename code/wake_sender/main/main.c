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

#include "foc_link_protocol.h"

static const char *TAG = "PA_TX";

#define ADV_INSTANCE 1
#define SID 2

static uint8_t s_own_addr_type;
static bool s_started = false;
static uint16_t s_sequence = 0;
static uint8_t s_session = 1;
static uint8_t s_pending_cmd = FOC_LINK_CMD_NONE;
static int s_repeat_left = 0;

void ble_store_config_init(void);

/* ------------------------------------------------------------------ */

static void build_payload(uint8_t *out, uint8_t cmd)
{
    foc_link_pkt_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.magic = FOC_LINK_MAGIC;
    pkt.version = FOC_LINK_VERSION;
    pkt.role = FOC_LINK_ROLE_CMD;
    pkt.receiver_id = CONFIG_SENDER_RECEIVER_ID;
    pkt.cmd = cmd;
    pkt.session = s_session;
    pkt.sequence = ++s_sequence;
    /* nonce 每次变化 —— 为将来的 EAD/防重放预留, 现在只用来区分包 */
    pkt.nonce = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);

    foc_link_encode(out, &pkt);
}

/* 更新周期广播数据。**可以在广播进行中调用** —— 这是 PA 发指令的方式:
 * 广播本身不停, 只换载荷。 */
static esp_err_t update_periodic_data(uint8_t cmd)
{
    uint8_t payload[FOC_LINK_PKT_LEN];
    build_payload(payload, cmd);

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
    ESP_ERROR_CHECK(update_periodic_data(FOC_LINK_CMD_NONE));

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

static void request_command(uint8_t cmd)
{
    const char *name = foc_link_cmd_name(cmd);
    if (!s_started) {
        printf("PA 还没起来, 稍后再试\n");
        return;
    }
    s_pending_cmd = cmd;
    s_repeat_left = CONFIG_SENDER_REPEAT;
    printf("已排队指令 %s, 将重复 %d 次 (每次间隔 %dms)\n",
           name, CONFIG_SENDER_REPEAT, CONFIG_SENDER_PA_ITVL_MS);
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
    /* 按 PA 间隔推, 保证每个周期事件带上不同载荷 */
    next_us = now + (int64_t)CONFIG_SENDER_PA_ITVL_MS * 1000;

    if (update_periodic_data(s_pending_cmd) == ESP_OK) {
        s_repeat_left--;
        if (s_repeat_left == 0) {
            s_pending_cmd = FOC_LINK_CMD_NONE;
            ESP_LOGI(TAG, "指令发送完毕, 回到心跳");
        }
    }
}

/* ------------------------------------------------------------------ */
/* 控制台                                                              */

static int cmd_open(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    request_command(FOC_LINK_CMD_OPEN);
    return 0;
}
static int cmd_close(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    request_command(FOC_LINK_CMD_CLOSE);
    return 0;
}
static int cmd_stop(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    request_command(FOC_LINK_CMD_STOP);
    return 0;
}
static int cmd_wake(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    request_command(FOC_LINK_CMD_WAKE);
    return 0;
}
static int cmd_heartbeat(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    s_repeat_left = 0;
    s_pending_cmd = FOC_LINK_CMD_NONE;
    update_periodic_data(FOC_LINK_CMD_NONE);
    printf("回到心跳 (CMD_NONE)\n");
    return 0;
}
static int cmd_payload(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    uint8_t buf[FOC_LINK_PKT_LEN];
    build_payload(buf, FOC_LINK_CMD_NONE);
    printf("载荷 %d 字节: ", FOC_LINK_PKT_LEN);
    for (int i = 0; i < FOC_LINK_PKT_LEN; i++) {
        printf("%02X ", buf[i]);
    }
    printf("\n(little-endian; magic=0x%04X ver=%d recv=0x%X session=%u seq=%u)\n",
           FOC_LINK_MAGIC, FOC_LINK_VERSION, CONFIG_SENDER_RECEIVER_ID,
           s_session, s_sequence);
    return 0;
}
static int cmd_session(int argc, char **argv)
{
    s_session++;
    s_sequence = 0;
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

    console_start();
    ESP_LOGI(TAG, "就绪。指令: open / close / stop / wake / heartbeat / payload");
    ESP_LOGI(TAG, "⚠️ 发送端无 ACK, 靠 Kconfig 的 SENDER_REPEAT=%d 重复发送保证送达",
             CONFIG_SENDER_REPEAT);

    while (1) {
        dispatch_tick();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
