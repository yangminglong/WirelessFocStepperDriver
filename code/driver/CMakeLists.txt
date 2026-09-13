cmake_minimum_required(VERSION 3.16)

# 共享的 PA 载荷协议 (两端共用, 见 code/components/foc_link_protocol)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(focstep_driver)
