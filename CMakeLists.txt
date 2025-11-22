cmake_minimum_required(VERSION 3.16)

# Khai báo thêm thư mục chứa component nằm ngoài folder "components" mặc định
# "communicate_init" là tên folder chứa spi_init
set(EXTRA_COMPONENT_DIRS communicate_init)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Energy_Meter)
