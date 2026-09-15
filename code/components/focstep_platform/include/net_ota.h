#pragma once

/*
 * Wi-Fi 控制面 + OTA + mDNS (平台态)
 *
 * ⚠️ **默认关闭** (Kconfig FOCSTEP_WIFI_ENABLE=n)。
 *    理由: 开 Wi-Fi 会打破深睡。本板的深睡档待机基线 (25~65µA@24V) 建立在
 *    "深睡 + 本地感知"之上, 射频常开不在预算内。
 *    它只在**平台态调试 / 装门前升级**时启用。
 *
 * 组件选择 (全部官方):
 *   - espressif/mdns          局域网发现 (focstep-xxxx.local)
 *   - esp_http_server         OTA 上传页
 *   - esp_ota_ops             分区写入与切换
 *   - esp_wifi / esp_netif    连接
 *
 * ⚠️ 驱动板装在门里之后再拆成本极高 ⇒ **上板前务必先把 OTA 打通**。
 */

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 返回:
 *  ESP_OK      平台态已就绪 (Wi-Fi 已连 + HTTP/mDNS 已起)
 *  ESP_ERR_NOT_SUPPORTED  未启用 (Kconfig 关闭或未配 SSID)
 *  其它        失败原因 (不致命, 调用者应继续跑本地功能)
 *
 * ⚠️ 顺序要求 (踩过): httpd_start() 必须在 esp_netif_init() **之后**,
 *    否则 tcpip_send_msg_wait_sem 会断言崩溃并重启循环。
 */
esp_err_t net_ota_start(void);

/* 关闭控制面 (回到睡眠态前调用) */
void net_ota_stop(void);

bool net_ota_is_running(void);

/* 当前运行分区与下一个 OTA 分区的版本串, 供自检命令打印 */
void net_ota_print_info(void);

#ifdef __cplusplus
}
#endif
