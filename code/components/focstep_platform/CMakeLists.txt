# FocStepper 平台层 —— 跨项目复用的底层驱动
#
# 本组件**不含任何应用语义**: 没有"门"、"开/关"、"助动"这类概念。
# 换项目时整个目录拷走即可, 应用层通过事件与 API 接进来。
#
# 目录约定:
#   include/  公共头 (应用层只需 include 这些)
#   src/      实现

idf_component_register(
    SRCS
        # 传感器
        "src/kth5701.cpp"
        # 电流
        "src/ipropi_sense.c"
        "src/ipropi_current_sense.cpp"
        # 电压
        "src/bus_voltage.c"
        # 电机
        "src/foc_motor.cpp"
        "src/homing.c"
        # 电源
        "src/power_state.c"
        "src/wakeup.c"
        # 无线唤醒 (BLE 周期广播接收端; 未编入时为空实现)
        "src/pa_wake.c"
        # 人机
        "src/led_ws2812.c"
        "src/platform_button.c"
        # 通信
        "src/can_link.c"
        "src/net_ota.c"
        # 框架
        "src/platform_events.c"
        "src/platform_console.cpp"
    INCLUDE_DIRS
        "include"
    EMBED_TXTFILES
        "src/ota_page.html"
    REQUIRES
        driver
        esp_adc
        esp_timer
        nvs_flash
        esp_wifi
        esp_netif
        esp_event
        esp_http_server
        app_update
        mdns
        esp_pm              # pa_wake: 监听期开 PM 自动轻睡
        bt                  # pa_wake: NimBLE 主机与控制器 (FOCSTEP_PA_WAKE_ENABLE 时才引用)
        foc_link_protocol   # pa_wake: PA 载荷解析 (与发送端共用同一份线格式)
)

# 托管组件 (espressif/esp_simplefoc 等) 由应用层在 main/idf_component.yml 声明,
# 并通过 idf_component.yml 的 requirements 传递到本组件。
# 见本目录下的 idf_component.yml。
