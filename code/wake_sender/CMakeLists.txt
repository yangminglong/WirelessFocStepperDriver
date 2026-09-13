cmake_minimum_required(VERSION 3.16)

# 只引入共享的 PA 载荷协议 (与驱动板同一份, 避免两端线格式漂移)。
# ⚠️ 不要写成 ../components —— 那会把 focstep_platform (驱动板的平台层) 也编进来。
#    发送端没有电机/编码器, 而且平台层引用的是驱动板项目的 Kconfig。
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components/foc_link_protocol")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(focstep_wake_sender)
