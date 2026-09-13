#include "net_ota.h"
#include "board_pins.h"

#include <string.h>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "mdns.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

static const char *TAG = "OTA";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define WIFI_MAX_RETRY     5

/* 上传页 (EMBED_TXTFILES 打进固件) */
extern const uint8_t ota_page_html_start[] asm("_binary_ota_page_html_start");
extern const uint8_t ota_page_html_end[] asm("_binary_ota_page_html_end");

static httpd_handle_t s_server = NULL;
static bool s_running = false;
static EventGroupHandle_t s_wifi_eg = NULL;
static int s_retry = 0;

/* ------------------------------------------------------------------ */
/* Wi-Fi                                                               */

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry < WIFI_MAX_RETRY) {
            s_retry++;
            ESP_LOGW(TAG, "Wi-Fi 断开, 重连 %d/%d", s_retry, WIFI_MAX_RETRY);
            esp_wifi_connect();
        } else {
            xEventGroupSetBits(s_wifi_eg, WIFI_FAIL_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "获取 IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        s_retry = 0;
        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t wifi_connect(void)
{
    s_wifi_eg = xEventGroupCreate();
    ESP_RETURN_ON_FALSE(s_wifi_eg != NULL, ESP_ERR_NO_MEM, TAG, "event group");

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init");
    /* ⚠️ default event loop 必须显式建。它若不存在, 后面 httpd_start 会在
     *    tcpip_send_msg_wait_sem 处断言并反复重启。 */
    esp_err_t r = esp_event_loop_create_default();
    if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) {
        return r;
    }
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_cfg), TAG, "esp_wifi_init");

    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
                            WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL),
                        TAG, "reg wifi evt");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
                            IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL),
                        TAG, "reg ip evt");

    wifi_config_t wc = {0};
    strlcpy((char *)wc.sta.ssid, CONFIG_FOCSTEP_WIFI_SSID, sizeof(wc.sta.ssid));
    strlcpy((char *)wc.sta.password, CONFIG_FOCSTEP_WIFI_PASSWORD, sizeof(wc.sta.password));
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wc), TAG, "set config");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_eg,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, pdMS_TO_TICKS(20000));
    if (bits & WIFI_CONNECTED_BIT) {
        return ESP_OK;
    }
    ESP_LOGE(TAG, "Wi-Fi 连接失败");
    return ESP_FAIL;
}

/* ------------------------------------------------------------------ */
/* mDNS                                                                */

static void mdns_start(void)
{
    esp_err_t ret = mdns_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "mdns_init 失败: %s", esp_err_to_name(ret));
        return;
    }
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char host[32];
    snprintf(host, sizeof(host), "focstep-%02x%02x%02x", mac[3], mac[4], mac[5]);
    mdns_hostname_set(host);
    mdns_instance_name_set("FocStepper 驱动板");
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    ESP_LOGI(TAG, "mDNS: http://%s.local/", host);
}

/* ------------------------------------------------------------------ */
/* HTTP: 上传页 + OTA 接收                                             */

static esp_err_t get_index(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, (const char *)ota_page_html_start,
                           ota_page_html_end - ota_page_html_start);
}

static esp_err_t post_ota(httpd_req_t *req)
{
    ESP_LOGI(TAG, "OTA 开始, 长度 %d 字节", req->content_len);

    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (part == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no OTA partition");
        return ESP_FAIL;
    }

    esp_ota_handle_t ota = 0;
    esp_err_t ret = esp_ota_begin(part, OTA_WITH_SEQUENTIAL_WRITES, &ota);
    if (ret != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota_begin failed");
        return ESP_FAIL;
    }

    /* 静态缓冲: 避免每包 malloc。4KB 与 flash 扇区对齐, 写入效率好。 */
    static char buf[4096];
    int remaining = req->content_len;
    bool header_checked = false;

    while (remaining > 0) {
        int want = (remaining > (int)sizeof(buf)) ? (int)sizeof(buf) : remaining;
        int got = httpd_req_recv(req, buf, want);
        if (got <= 0) {
            if (got == HTTPD_SOCK_ERR_TIMEOUT) {
                continue; /* 超时不是错误, 继续收 */
            }
            ESP_LOGE(TAG, "recv 失败: %d", got);
            esp_ota_abort(ota);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv failed");
            return ESP_FAIL;
        }

        /* 校验镜像头 —— 传错文件(比如传了别的板子的 bin)在这里就拦住,
         * 而不是等重启后变砖。 */
        if (!header_checked && got >= (int)sizeof(esp_image_header_t) + (int)sizeof(esp_app_desc_t)) {
            esp_app_desc_t *desc = (esp_app_desc_t *)(buf + sizeof(esp_image_header_t));
            if (desc->magic_word != ESP_APP_DESC_MAGIC_WORD) {
                ESP_LOGE(TAG, "不是合法的 app 镜像");
                esp_ota_abort(ota);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "not a valid app image");
                return ESP_FAIL;
            }
            ESP_LOGI(TAG, "待写入镜像: %s v%s (idf %s)", desc->project_name,
                     desc->version, desc->idf_ver);
            header_checked = true;
        }

        ret = esp_ota_write(ota, buf, got);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ota_write 失败: %s", esp_err_to_name(ret));
            esp_ota_abort(ota);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "write failed");
            return ESP_FAIL;
        }
        remaining -= got;
    }

    ret = esp_ota_end(ota);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ota_end 失败: %s", esp_err_to_name(ret));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "image validation failed");
        return ESP_FAIL;
    }
    ret = esp_ota_set_boot_partition(part);
    if (ret != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "set boot partition failed");
        return ESP_FAIL;
    }

    ESP_LOGW(TAG, "OTA 成功, 即将重启");
    httpd_resp_sendstr(req, "OK");
    /* 给 HTTP 响应一点时间发出去再重启 */
    vTaskDelay(pdMS_TO_TICKS(800));
    esp_restart();
    return ESP_OK;
}

static esp_err_t http_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    /* 默认 8 个 handler 槽位, 加满会 abort 并重启循环 —— 本项目踩过类似坑。
     * 这里用不到那么多, 但显式给足余量。 */
    cfg.max_uri_handlers = 16;
    cfg.stack_size = 8192;
    cfg.lru_purge_enable = true;

    ESP_RETURN_ON_ERROR(httpd_start(&s_server, &cfg), TAG, "httpd_start failed");

    httpd_uri_t uri_index = {
        .uri = "/", .method = HTTP_GET, .handler = get_index, .user_ctx = NULL,
    };
    httpd_uri_t uri_ota = {
        .uri = "/ota", .method = HTTP_POST, .handler = post_ota, .user_ctx = NULL,
    };
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_server, &uri_index), TAG, "reg /");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_server, &uri_ota), TAG, "reg /ota");
    return ESP_OK;
}

/* ------------------------------------------------------------------ */

esp_err_t net_ota_start(void)
{
#if !CONFIG_FOCSTEP_WIFI_ENABLE
    ESP_LOGI(TAG, "Wi-Fi 控制面未启用 (FOCSTEP_WIFI_ENABLE=n) —— 这是默认且正确的配置");
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (CONFIG_FOCSTEP_WIFI_SSID[0] == '\0') {
        ESP_LOGW(TAG, "未配置 SSID, 跳过。请在 sdkconfig.defaults.local 里填");
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (s_running) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(wifi_connect(), TAG, "wifi connect failed");
    mdns_start();
    ESP_RETURN_ON_ERROR(http_start(), TAG, "http start failed");

    s_running = true;
    ESP_LOGW(TAG, "⚠️ 控制面已起: Wi-Fi + HTTP + mDNS。这会打破深睡, 用完请 net_ota_stop()");
    return ESP_OK;
#endif
}

void net_ota_stop(void)
{
    if (s_server) {
        httpd_stop(s_server);
        s_server = NULL;
    }
    mdns_free();
    esp_wifi_stop();
    esp_wifi_deinit();
    s_running = false;
    ESP_LOGI(TAG, "控制面已关闭");
}

bool net_ota_is_running(void)
{
    return s_running;
}

void net_ota_print_info(void)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    esp_app_desc_t desc;
    printf("running : %s @ 0x%08" PRIx32 "\n", run ? run->label : "?",
           run ? run->address : 0);
    if (esp_ota_get_partition_description(run, &desc) == ESP_OK) {
        printf("  version=%s  project=%s  idf=%s  built=%s %s\n",
               desc.version, desc.project_name, desc.idf_ver,
               desc.date, desc.time);
    }
    printf("next OTA: %s @ 0x%08" PRIx32 "\n", next ? next->label : "?",
           next ? next->address : 0);
    printf("state   : %s\n", esp_ota_get_state_partition(run, NULL) == ESP_OK ? "ok" : "?");
}
