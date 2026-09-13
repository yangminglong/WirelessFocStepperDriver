#include "platform_events.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "PLAT_EVT";

ESP_EVENT_DEFINE_BASE(FOCSTEP_EVENT);

static bool s_inited = false;

esp_err_t platform_events_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }
    /* default event loop 由 net_ota 也可能创建, 这里容忍已存在 */
    esp_err_t ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default: %s", esp_err_to_name(ret));
        return ret;
    }
    s_inited = true;
    ESP_LOGI(TAG, "事件总线就绪 (base=FOCSTEP_EVENT)");
    return ESP_OK;
}

void platform_event_post(focstep_event_id_t id, const void *data, size_t len)
{
    if (!s_inited) {
        return;
    }
    /* esp_event_post 会拷贝数据, 所以调用者不必保证生命周期。
     * 但 esp_event 对 >4 字节的 data 会 malloc —— 事件很少, 可接受。 */
    esp_err_t ret = esp_event_post(FOCSTEP_EVENT, (int32_t)id, data, len, 0);
    if (ret != ESP_OK) {
        /* 事件循环队列满时不阻塞调用者 —— 丢事件比卡住电机控制安全 */
        ESP_LOGW(TAG, "post id=%d 失败: %s", (int)id, esp_err_to_name(ret));
    }
}


