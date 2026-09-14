# 应用层 (推拉门) —— 换项目就换这个目录
#
# 平台层在 components/focstep_platform, 通过 REQUIRES 引入。
idf_component_register(
    SRCS
        "app_main.c"
        "app_door.c"
        "app_console.c"
    INCLUDE_DIRS "."
    REQUIRES
        focstep_platform
        foc_link_protocol   # 承载层: 帧格式 (平台层内部已用)
        foc_door_link       # 应用层: 无线指令集 (app_door / app_console 用)
        esp_timer
        nvs_flash
)
