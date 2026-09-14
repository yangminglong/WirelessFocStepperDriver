/*
 * 无线唤醒接收端 (BLE 周期广播 / PA) —— 实现
 *
 * 接口与纪律见 include/pa_wake.h。本文件只有三件事:
 *   ① 扫描 → 找到 SID 匹配的周期广播 → 建立周期同步 → 停扫描 (省电的关键一步)
 *   ② 解析周期报文里的 foc_link 载荷: 去重/寻址/CRC 之后, 指令发事件, SET_T 自己消化
 *   ③ 模式开关: 起停射频 + 起停 PM 自动轻睡 (监听期必须让控制器自己排唤醒)
 *
 * 实测依据 (tests/ble_sync): C_RX=0.160 mA·s, 失步会自动重扫, 同步后必须 disc_cancel。
 * ⚠️ 改这里之前先读 code/README.md §13 (睡眠架构与三条纪律)。
 */

#include "pa_wake.h"

#include <string.h>

#include "esp_log.h"

static const char *TAG = "PA";

#if CONFIG_FOCSTEP_PA_WAKE_ENABLE

#include "platform_events.h"
#include "foc_link_protocol.h"

#include "esp_check.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/util/util.h"

/* 与发送端约定的周期广播 SID (发送端侧 SID 宏见 wake_sender/main/main.c) */
#define PA_SID 2

/* sync_timeout 必须 > T。直接取满 0x4000 × 10ms = 163.84s, 一劳永逸。
 * (skip 改大时 T 会到十几秒, 取小了会"同步超时丢失") */
#define PA_SYNC_TIMEOUT 0x4000

/* skip 上限 (ble_gap.c 校验 skip ≤ 0x1F3) */
#define PA_SKIP_MAX 0x1F3

/* 同步事件里的 per_adv_ival 单位是 1.25ms (实测: 240ms → 192) */
#define PA_IVAL_TO_MS(v) ((uint16_t)(((uint32_t)(v) * 125u) / 100u))

/* 期望 T 的下限: 比发射端的 per_adv_ival 小时没有意义 */
#define PA_T_MIN_MS 100u

/* 慢扫相位 (快扫到期后转这里)。⚠️ 绝不能沿用"itvl=window=0"(=控制器默认 16/16
 * = 100% 占空比): 发射端不在场时那就是**常开射频, 实测 84mA**, 把省电全部吃掉。
 * 慢扫 30/10000 ≈ 0.3% ≈ 亚 mA 级; 代价是发射端后出现时发现延迟变长。 */
#define PA_SCAN_SLOW_WINDOW_MS   30u
#define PA_SCAN_SLOW_INTERVAL_MS 10000u

static pa_wake_mode_t s_mode = PA_WAKE_OFF;
static uint32_t s_my_id = 1;
static bool     s_inited = false;

static bool     s_host_synced = false; /* ble_hs 就绪 (地址可推断) */
static bool     s_scanning = false;
static bool     s_synced = false;
static uint16_t s_sync_handle = 0;
static uint8_t  s_skip = 0;
static uint8_t  s_want_skip = 0;
static bool     s_recreate_pending = false; /* terminate 后按 s_want_skip 重建 */
static bool     s_warned_legacy = false;    /* legacy 报告只警告一次 */

static ble_addr_t s_adv_addr;
static uint16_t   s_per_adv_ival_ms = 0; /* 由同步事件读到 */
static uint32_t   s_t_want_ms = CONFIG_FOCSTEP_PA_WAKE_T_MS;

static int8_t   s_rssi = 0;
static uint32_t s_lost_cnt = 0;
static uint32_t s_pkt_ok = 0;
static uint32_t s_cmd_cnt = 0;
static uint32_t s_filtered = 0; /* 被应用过滤钩子挡下的包 (典型: 心跳) */
static pa_wake_payload_filter_t s_payload_filter = nullptr;
/* 丢弃计数 (现场排障用: 分得清"没收到"是链路噪声、别人家的包、还是版本不符) */
static uint32_t s_drop_crc = 0, s_drop_dup = 0, s_drop_not_me = 0;
static uint32_t s_drop_badver = 0, s_drop_noise = 0;

/* 扫描相位: 快扫(找得到对端就快) → 快扫超时后转慢扫(对端不在场时别烧电) */
static bool s_slow_scan = false;
static esp_timer_handle_t s_scan_phase_tmr = nullptr;

/* session/sequence 去重状态。放 RAM: 深睡复位后重来是可接受的
 * (最坏是执行一次重复指令, 而指令本身是幂等的开关停) */
static foc_link_rx_state_t s_rx;

/* 扫描与周期同步用同一个 GAP 回调: 周期报文 (PERIODIC_REPORT/SYNC/LOST)
 * 是**挂在 sync_create 传进去的回调**上的, 不是全局回调 —— 建同步时漏传就收不到报文。
 * 见 tests/ble_sync/ble_periodic_sync/main/main.c (实测跑通的写法)。 */
static int gap_event(struct ble_gap_event *event, void *arg);

/* ------------------------------------------------------------------ */
/* 工具                                                                */

static uint32_t skip_to_T_ms(uint8_t skip)
{
    return (uint32_t)s_per_adv_ival_ms * (uint32_t)(skip + 1u);
}

/* 期望 T → 最接近的可达 skip。未同步 (不知道 per_adv_ival) 时返回 0:
 * 先按"最快"建, 同步成功后拿到 per_adv_ival 再按需重建。 */
static uint8_t T_ms_to_skip(uint32_t t_ms)
{
    if (s_per_adv_ival_ms == 0) {
        return 0;
    }
    uint32_t n = (t_ms + s_per_adv_ival_ms / 2u) / s_per_adv_ival_ms; /* 四舍五入 */
    if (n == 0) {
        n = 1;
    }
    if (n - 1u > PA_SKIP_MAX) {
        n = PA_SKIP_MAX + 1u;
    }
    return (uint8_t)(n - 1u);
}

static void post_sync_event(void)
{
    focstep_evt_pa_sync_t e = {
        .base.timestamp_ms = platform_now_ms(),
        .synced = s_synced ? 1 : 0,
        .t_ms = s_synced ? skip_to_T_ms(s_skip) : s_t_want_ms,
        .lost_cnt = s_lost_cnt,
    };
    platform_event_post(FOCSTEP_EVT_PA_SYNC, &e, sizeof(e));
}

/* 自动轻睡开关。监听期必须开, 否则控制器排不了唤醒 (它靠 PM 锁申请唤醒窗口) */
static void pm_apply(pa_wake_mode_t mode)
{
#if CONFIG_PM_ENABLE
    esp_pm_config_t cfg = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = (mode == PA_WAKE_PA),
#else
        .light_sleep_enable = false,
#endif
    };
    esp_err_t r = esp_pm_configure(&cfg);
    ESP_LOGI(TAG, "PM 自动轻睡: %s (rc=%s)",
             (mode == PA_WAKE_PA) ? "开" : "关", esp_err_to_name(r));
    if (mode == PA_WAKE_PA && r != ESP_OK) {
        ESP_LOGE(TAG, "轻睡没打开 ⇒ 监听功耗会高一个量级 (查 CONFIG_PM_ENABLE/tickless)");
    }
#else
    (void)mode;
    ESP_LOGW(TAG, "CONFIG_PM_ENABLE=n ⇒ 监听期不会进轻睡, 功耗会高一个量级");
#endif
}

/* ------------------------------------------------------------------ */
/* 扫描 / 同步                                                          */

static void scan_start(void);

static void sync_create(uint8_t skip)
{
    if (!s_host_synced || s_synced) {
        return;
    }
    /* ⚠️ 结构体必须清零: 官方示例漏了 memset, reports_disabled 等位域是栈上垃圾值,
     *    会"同步成功但一个 report 都不投递"(tests/ble_sync/docs/02 §5 实测踩过)。 */
    struct ble_gap_periodic_sync_params params;
    memset(&params, 0, sizeof(params));
    params.skip = skip;
    params.sync_timeout = PA_SYNC_TIMEOUT;

    s_skip = skip;
    int rc = ble_gap_periodic_adv_sync_create(&s_adv_addr, PA_SID, &params, gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "建立周期同步失败 rc=%d (同一时刻只允许一个 pending 同步)", rc);
        s_scanning = false;
        scan_start();
        return;
    }
    ESP_LOGI(TAG, "正在建立周期同步: skip=%u sync_timeout=%u(×10ms)", skip, PA_SYNC_TIMEOUT);
}

/* 快扫到期 → 转慢扫。回调跑在 esp_timer 任务里; NimBLE 主机 API 自带锁, 可重入。 */
static void scan_phase_cb(void *arg)
{
    (void)arg;
    if (s_mode != PA_WAKE_PA || s_synced) {
        return;
    }
    ESP_LOGW(TAG, "快扫 %d ms 没找到发射端, 转慢扫 (省电; 发现延迟变长)",
             CONFIG_FOCSTEP_PA_SCAN_FAST_MS);
    s_slow_scan = true;
    scan_start(); /* 内部会先取消当前扫描 */
}

static void scan_start(void)
{
    if (!s_host_synced || s_synced || s_mode != PA_WAKE_PA) {
        return;
    }
    /* 已在扫: 先停 (切相位/重扫都走这条路) */
    if (s_scanning) {
        ble_gap_disc_cancel();
        s_scanning = false;
    }
    uint8_t own_addr_type = 0;
    if (ble_hs_id_infer_auto(0, &own_addr_type) != 0) {
        ESP_LOGE(TAG, "推断地址类型失败");
        return;
    }
    uint32_t win_ms = s_slow_scan ? PA_SCAN_SLOW_WINDOW_MS : CONFIG_FOCSTEP_PA_SCAN_WINDOW_MS;
    uint32_t itvl_ms = s_slow_scan ? PA_SCAN_SLOW_INTERVAL_MS : CONFIG_FOCSTEP_PA_SCAN_INTERVAL_MS;
    if (win_ms > itvl_ms) {
        win_ms = itvl_ms; /* window 不能大于 interval */
    }

    /* ⚠️ 扫描是**限占空比**的: 这一步只为了"发现一次 SID", 建立同步后立刻
     *    disc_cancel()。忘了 cancel 或把 window 开到 == interval (100%),
     *    射频就常开 (~84mA), PA 的省电效果会被完全淹没。 */
    struct ble_gap_disc_params disc = {0};
    disc.filter_duplicates = 0;
    disc.passive = 1; /* 被动: 只收, 不发 scan_req */
    disc.itvl = (uint16_t)(itvl_ms * 8 / 5);   /* 单位 0.625ms (= ms×8/5, 全程整数) */
    disc.window = (uint16_t)(win_ms * 8 / 5);

    int rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc, gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "启动扫描失败 rc=%d", rc);
        return;
    }
    s_scanning = true;
    ESP_LOGI(TAG, "扫描中 (%s: window=%ums itvl=%ums, 找 SID=%d 的周期广播)",
             s_slow_scan ? "慢扫" : "快扫", (unsigned)win_ms, (unsigned)itvl_ms, PA_SID);

    /* 快扫相位挂一个一次性定时器; 慢扫不需要 */
    if (!s_slow_scan && s_scan_phase_tmr) {
        esp_timer_stop(s_scan_phase_tmr);
        esp_timer_start_once(s_scan_phase_tmr, (uint64_t)CONFIG_FOCSTEP_PA_SCAN_FAST_MS * 1000ULL);
    }
}

/* ------------------------------------------------------------------ */
/* 载荷处理                                                            */

/* 载荷投递。**射频收到与本地注入 (`pa_send`) 共用这一条路径** ——
 * 自检命令才能反映真实行为。@param rssi -128 = 本地注入 (不可能是真射频值)
 *
 * ⚠️ 平台**不解释** cmd/arg —— 命令语义属于应用层协议 (components/foc_door_link)。
 *    这里只问一句"要不要打扰上层"(应用装的过滤钩子, 典型是滤掉它自己的心跳,
 *    省 host 侧开销), 然后把 (cmd, arg) **原样**发事件。 */
static void dispatch_cmd(uint8_t cmd, uint32_t arg, int8_t rssi)
{
    if (s_payload_filter && !s_payload_filter(cmd, (uint16_t)arg)) {
        s_filtered++;
        return;
    }
    s_cmd_cnt++;
    ESP_LOGI(TAG, "投递载荷 cmd=%u arg=%u (rssi=%d%s)",
             cmd, (unsigned)arg, rssi, (rssi == -128) ? ", 本地注入" : "");
    focstep_evt_pa_cmd_t e = {
        .base.timestamp_ms = platform_now_ms(),
        .cmd = cmd,
        .arg = arg,
    };
    platform_event_post(FOCSTEP_EVT_PA_CMD, &e, sizeof(e));
}

static void handle_report(const uint8_t *data, uint16_t len, int8_t rssi)
{
    s_rssi = rssi;

    foc_link_pkt_t pkt;
    bool crc_ok = false;
    if (!foc_link_decode(data, len, &pkt, &crc_ok)) {
        /* 区分"版本不符"与"别人的广播": 前者必须让现场一眼看见, 否则表现为
         * "什么都没收到" —— 两端固件必须同时重烧。 */
        uint8_t v = foc_link_peek_version(data, len);
        if (v != 0) {
            if (s_drop_badver++ < 3) {
                ESP_LOGW(TAG, "收到 v%u 的包, 本端是 v%d —— 对端固件没跟着升级? "
                              "两端必须同时重烧 (线格式已变)", v, FOC_LINK_VERSION);
            }
        } else {
            s_drop_noise++;
        }
        return;
    }
    if (!crc_ok) {
        s_drop_crc++;
        if (s_drop_crc <= 3) {
            ESP_LOGW(TAG, "CRC 错 (链路噪声?)");
        }
        return;
    }
    if (pkt.role != FOC_LINK_ROLE_CMD) {
        return; /* RST 通道暂未使用 */
    }

    foc_link_rx_result_t r = foc_link_rx_filter(&s_rx, &pkt, s_my_id);
    if (r == FOC_LINK_NOT_FOR_ME) {
        s_drop_not_me++;
        return; /* 寻址给别的板 */
    }
    if (r != FOC_LINK_ACCEPT) {
        s_drop_dup++; /* 重复帧: 发送端每条指令重复发 N 次, 这里必须幂等 */
        return;
    }
    s_pkt_ok++;
    dispatch_cmd(pkt.cmd, pkt.arg, rssi);
}

/* ------------------------------------------------------------------ */
/* NimBLE 回调                                                         */

static void on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE host reset, reason=%d", reason);
    s_host_synced = false;
    s_synced = false;
    s_scanning = false;
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ensure_addr 失败 rc=%d", rc);
        return;
    }
    s_host_synced = true;
    ESP_LOGI(TAG, "BLE host 就绪");
    if (s_mode == PA_WAKE_PA) {
        scan_start();
    }
}

static void host_task(void *param)
{
    (void)param;
    nimble_port_run(); /* 只在 nimble_port_stop() 时返回 */
    nimble_port_freertos_deinit();
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_EXT_DISC: {
        struct ble_gap_ext_disc_desc *disc = (struct ble_gap_ext_disc_desc *)&event->ext_disc;
        if (disc->sid != PA_SID || s_synced || s_recreate_pending) {
            return 0;
        }
        memcpy(&s_adv_addr, &disc->addr, sizeof(s_adv_addr));
        s_scanning = false;
        ESP_LOGI(TAG, "发现 SID=%d 的发射端, 建立同步", PA_SID);
        sync_create(T_ms_to_skip(s_t_want_ms));
        return 0;
    }

    case BLE_GAP_EVENT_DISC:
        /* 收到 legacy 报告说明扫描没上报扩展事件。正常不会走到这里
         * (我们扫的是 SID 过滤的周期广播)。若反复出现而不是 EXT_DISC,
         * 说明扫描方式/控制器能力与预期不符 —— 见 README §13。 */
        if (!s_warned_legacy) {
            s_warned_legacy = true;
            ESP_LOGW(TAG, "收到 legacy 广播报告 (预期 EXT_DISC), 请核对扫描方式");
        }
        return 0;

    case BLE_GAP_EVENT_PERIODIC_SYNC: {
        if (event->periodic_sync.status != 0) {
            ESP_LOGW(TAG, "同步失败 status=%d, 重扫", event->periodic_sync.status);
            s_synced = false;
            scan_start();
            return 0;
        }
        s_synced = true;
        s_sync_handle = event->periodic_sync.sync_handle;
        s_per_adv_ival_ms = PA_IVAL_TO_MS(event->periodic_sync.per_adv_ival);
        ESP_LOGI(TAG, "已同步: per_adv_ival=%ums adv_clk_accuracy=%uppm handle=%u",
                 s_per_adv_ival_ms, event->periodic_sync.adv_clk_accuracy, s_sync_handle);

        /* 同步建立后立刻停扫描 —— 否则 100% 占空比的扫描会把省电全部吃掉 */
        if (s_scanning) {
            ble_gap_disc_cancel();
            s_scanning = false;
        }

        /* 现在知道 per_adv_ival 了, 把期望 T 落到准确 skip 上 */
        {
            uint8_t want = T_ms_to_skip(s_t_want_ms);
            if (want != s_skip) {
                ESP_LOGI(TAG, "按期望 T=%ums 调整 skip: %u → %u", s_t_want_ms, s_skip, want);
                s_want_skip = want;
                s_recreate_pending = true;
                ble_gap_periodic_adv_sync_terminate(s_sync_handle);
                return 0;
            }
        }
        ESP_LOGI(TAG, "实际唤醒周期 T = %u ms (skip=%u)", skip_to_T_ms(s_skip), s_skip);
        post_sync_event();
        return 0;
    }

    case BLE_GAP_EVENT_PERIODIC_SYNC_LOST:
        s_synced = false;
        s_sync_handle = 0;
        s_lost_cnt++;
        if (s_recreate_pending) {
            /* 我们自己 terminate 的 (改 skip): 不等重扫, 直接按新参数重建 */
            s_recreate_pending = false;
            ESP_LOGI(TAG, "按 skip=%u 重建同步", s_want_skip);
            sync_create(s_want_skip);
        } else {
            ESP_LOGW(TAG, "失步 (累计 %u 次), 重新扫描", s_lost_cnt);
            scan_start();
        }
        post_sync_event();
        return 0;

    case BLE_GAP_EVENT_PERIODIC_REPORT:
        handle_report(event->periodic_report.data,
                      event->periodic_report.data_length,
                      event->periodic_report.rssi);
        return 0;

    default:
        return 0;
    }
}

/* ------------------------------------------------------------------ */
/* 对外 API                                                            */

esp_err_t pa_wake_init(uint32_t my_receiver_id)
{
    if (s_inited) {
        return ESP_OK;
    }
    /* 0 在协议里表示广播, 不可作为本板 ID */
    s_my_id = (my_receiver_id == 0) ? 1 : my_receiver_id;

    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init: %s (查 CONFIG_BT_ENABLED/FOCSTEP_PA_WAKE_ENABLE)",
                 esp_err_to_name(ret));
        return ret;
    }

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_store_config_init();

    /* 快扫→慢扫的相位切换定时器 (见 scan_start) */
    esp_timer_create_args_t tmr = {
        .callback = scan_phase_cb,
        .name = "pa_scan",
    };
    esp_err_t terr = esp_timer_create(&tmr, &s_scan_phase_tmr);
    if (terr != ESP_OK) {
        ESP_LOGW(TAG, "相位定时器创建失败: %s (快扫不会自动转慢扫)", esp_err_to_name(terr));
        s_scan_phase_tmr = nullptr;
    }

    nimble_port_freertos_init(host_task);
    s_inited = true;
    ESP_LOGI(TAG, "PA 接收端就绪: 本板 id=0x%X, SID=%d, 期望 T=%ums",
             (unsigned)s_my_id, PA_SID, s_t_want_ms);
    ESP_LOGI(TAG, "模式默认 OFF —— 何时监听由应用决定 (wake pa / 上电窗口)");
    return ESP_OK;
}

esp_err_t pa_wake_set_mode(pa_wake_mode_t mode)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (mode == s_mode) {
        return ESP_OK;
    }
    s_mode = mode;

    if (mode == PA_WAKE_PA) {
        pm_apply(mode);
        scan_start();
    } else {
        if (s_scanning) {
            ble_gap_disc_cancel();
            s_scanning = false;
        }
        if (s_synced && s_sync_handle) {
            s_recreate_pending = false;
            ble_gap_periodic_adv_sync_terminate(s_sync_handle); /* 失步事件里清标志 */
        }
        pm_apply(mode);
    }
    ESP_LOGW(TAG, "无线唤醒模式 → %s", (mode == PA_WAKE_PA) ? "PA 监听" : "关闭");
    post_sync_event();
    return ESP_OK;
}

pa_wake_mode_t pa_wake_mode(void)
{
    return s_mode;
}

esp_err_t pa_wake_set_T_ms(uint32_t t_ms)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    if (t_ms == 0) {
        t_ms = CONFIG_FOCSTEP_PA_WAKE_T_MS;
    }
    if (t_ms < PA_T_MIN_MS) {
        t_ms = PA_T_MIN_MS;
    }
    s_t_want_ms = t_ms;

    if (!s_synced) {
        ESP_LOGI(TAG, "未同步: 记下期望 T=%ums, 同步后套用", t_ms);
        return ESP_OK;
    }

    uint8_t want = T_ms_to_skip(t_ms);
    if (want == s_skip) {
        ESP_LOGI(TAG, "T 已满足: skip=%u ⇒ %ums (期望 %ums)",
                 s_skip, skip_to_T_ms(s_skip), t_ms);
        return ESP_OK;
    }

    /* 重建: 先 terminate, 在失步事件里 create
     * (ble_gap.c: 同一时刻不允许有第二个 pending 同步) */
    s_want_skip = want;
    s_recreate_pending = true;
    esp_err_t r = ble_gap_periodic_adv_sync_terminate(s_sync_handle);
    if (r != 0) {
        s_recreate_pending = false;
        ESP_LOGE(TAG, "terminate 失败 rc=%d, T 维持原值", r);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "调整 T: %ums → skip=%u (目标 %ums)", skip_to_T_ms(s_skip), want, t_ms);
    return ESP_OK;
}

uint32_t pa_wake_T_ms(void)
{
    return s_synced ? skip_to_T_ms(s_skip) : s_t_want_ms;
}

esp_err_t pa_wake_inject_cmd(uint8_t cmd, uint32_t arg)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    /* rssi = -128 标记本地注入 (真射频不可能给出这个值) */
    dispatch_cmd(cmd, arg, -128);
    return ESP_OK;
}

void pa_wake_set_payload_filter(pa_wake_payload_filter_t cb)
{
    s_payload_filter = cb;
}

void pa_wake_status(pa_wake_status_t *out)
{
    if (!out) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->mode = s_mode;
    out->host_ready = s_host_synced;
    out->synced = s_synced;
    out->slow_scan = s_slow_scan;
    out->sid = PA_SID;
    out->skip = s_skip;
    out->per_adv_ival_ms = s_per_adv_ival_ms;
    out->t_ms = pa_wake_T_ms();
    out->t_want_ms = s_t_want_ms;
    out->rssi = s_rssi;
    out->lost_cnt = s_lost_cnt;
    out->pkt_ok = s_pkt_ok;
    out->cmd_cnt = s_cmd_cnt;
    out->filtered = s_filtered;
    out->drop_crc = s_drop_crc;
    out->drop_dup = s_drop_dup;
    out->drop_not_me = s_drop_not_me;
    out->drop_badver = s_drop_badver;
    out->drop_noise = s_drop_noise;
}

#else /* !CONFIG_FOCSTEP_PA_WAKE_ENABLE ---------------------------------- */
/*
 * 未编入这张能力时的空实现: 让调用方 (app_main / 命令台) 不必到处写 #if。
 * 语义: "这个功能不存在", 用 ESP_ERR_NOT_SUPPORTED 表达 —— 与 net_ota 一致。
 */

esp_err_t pa_wake_init(uint32_t my_receiver_id)
{
    (void)my_receiver_id;
    ESP_LOGW(TAG, "无线唤醒接收未编入 (FOCSTEP_PA_WAKE_ENABLE=n)");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t pa_wake_set_mode(pa_wake_mode_t mode)
{
    (void)mode;
    return ESP_ERR_NOT_SUPPORTED;
}

pa_wake_mode_t pa_wake_mode(void)
{
    return PA_WAKE_OFF;
}

esp_err_t pa_wake_set_T_ms(uint32_t t_ms)
{
    (void)t_ms;
    return ESP_ERR_NOT_SUPPORTED;
}

uint32_t pa_wake_T_ms(void)
{
    return 0;
}

void pa_wake_status(pa_wake_status_t *out)
{
    if (out) {
        memset(out, 0, sizeof(*out));
    }
}

esp_err_t pa_wake_inject_cmd(uint8_t cmd, uint32_t arg)
{
    (void)cmd;
    (void)arg;
    return ESP_ERR_NOT_SUPPORTED;
}

void pa_wake_set_payload_filter(pa_wake_payload_filter_t cb)
{
    (void)cb;
}

#endif /* CONFIG_FOCSTEP_PA_WAKE_ENABLE */
