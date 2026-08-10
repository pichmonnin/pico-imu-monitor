cmake_minimum_required(VERSION 3.22)
set(PICO_BOARD pico2)
include(pico_sdk_import.cmake)
project(pico_imu_monitor C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(app 
    main.cpp 
)
target_link_libraries(app pico_stdlib hardware_i2c)
pico_enable_stdio_usb(app 1)
pico_enable_stdio_uart(app 1)

pico_add_extra_outputs(app)

