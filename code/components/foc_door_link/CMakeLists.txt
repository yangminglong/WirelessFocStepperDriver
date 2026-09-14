# 应用层指令集 (门) —— 与承载层 foc_link_protocol **分开**:
#   承载层只搬字节; "开/关/停" 这类业务命令属于产品。
#   换产品时只换本组件, 帧格式与射频那边不动。
idf_component_register(
    SRCS "foc_door_link.c"
    INCLUDE_DIRS "."
    REQUIRES foc_link_protocol
)
