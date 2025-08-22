cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
# User is free to modify the file as much as necessary
#

# ---- Compiler settings ----
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# ---- Build type ----
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# ---- Project name / target ----
set(CMAKE_PROJECT_NAME crane)
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# 需要编译 C++ 源码（你有大量 .cpp）
enable_language(C CXX ASM)

# 生成 compile_commands.json 以供 clangd 等使用
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# ---- Create executable target ----
add_executable(${CMAKE_PROJECT_NAME})

# ---- Add STM32CubeMX generated sources (produces target 'stm32cubemx') ----
add_subdirectory(cmake/stm32cubemx)

# ---- Link directories (if you have any extra libs) ----
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths if needed
)

# ---- Add user sources to executable ----
# 注意：这里“只列文件”，不要把别的 CMake 命令写进来
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # applications
    applications/buzzer_task.cpp
    applications/can_task.cpp
    applications/led_task.cpp
    applications/motor_task.cpp
    applications/plot_task.cpp
    applications/velocity_smoother.cpp

    # sp_middleware
    sp_middleware/io/adc/adc.cpp
    sp_middleware/io/bmi088/bmi088.cpp
    sp_middleware/io/buzzer/buzzer.cpp
    sp_middleware/io/can/can.cpp
    sp_middleware/io/dbus/dbus.cpp
    sp_middleware/io/led/led.cpp
    sp_middleware/io/plotter/plotter.cpp

    sp_middleware/motor/dm_motor/dm_motor.cpp
    sp_middleware/motor/rm_motor/rm_motor.cpp
    sp_middleware/motor/super_cap/super_cap.cpp

    sp_middleware/referee/pm02/pm02.cpp

    sp_middleware/tools/crc/crc.cpp
    sp_middleware/tools/low_pass_filter/low_pass_filter.cpp
    sp_middleware/tools/pid/pid.cpp
    sp_middleware/tools/mahony/mahony.cpp
    sp_middleware/tools/math_tools/math_tools.cpp
    sp_middleware/tools/swerve/swerve.cpp
    applications/host_serial.cpp


    # 飞特舵机驱动（C 文件）
    applications/sts3215.c
    applications/sts_port_hal.c
)

# ---- Include paths ----
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    applications          # 放 sts3215.h / sts_port.h（如果头文件在这里）
    sp_middleware
    Core/Inc              # 需要 HAL/FreeRTOS 等 Cube 头文件
)

# ---- Preprocessor macros (if any) ----
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# ---- Remove wrong 'ob' default lib when using cpp (STM toolchain quirk) ----
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# ---- Link libraries ----
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Add user defined libraries
)

# 可选：打开更多警告（建议后期再加）
# target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
#     $<$<COMPILE_LANGUAGE:C,CXX>:-Wall -Wextra -Wno-unused-parameter>
# )
