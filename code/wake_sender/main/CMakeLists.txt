idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        foc_door_link       # 应用层指令集 (内含承载层 foc_link_protocol)
        nvs_flash
        console
        esp_pm   # esp_pm_configure (轻睡)
        bt       # esp_ble_tx_power_set (控制器层发射功率)
)
