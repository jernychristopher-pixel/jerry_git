cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME SlamTest)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
#
# Lib 按功能分目录, 每个功能内部再分 inc/ (头文件) 与 src/ (源文件):
#   app/inc          编译开关与应用级配置
#   control/{inc,src} 控制算法(位置式 PID / 运动学 / 航向保持 / 位置环)
#   comm/{inc,src}    通信(二进制协议 / VOFA 文本)
#   driver/{inc,src}  外设驱动(电机 PWM / ADC / 软件 I2C / OLED / MPU6050)
#   imu/{inc,src}     IMU 姿态解算(DMP 与 InvenSense 驱动)
add_executable(${CMAKE_PROJECT_NAME}
        Lib/comm/src/vofa.c
        Lib/control/src/pid.c
        Lib/driver/src/motor.c
        Lib/driver/src/oled.c
        Lib/control/src/kinematics.c
        Lib/comm/src/protocol.c
        Lib/comm/src/m44_protocol.c
        Lib/driver/src/i2c_sw.c
        Lib/driver/src/mpu6050.c
        Lib/imu/src/inv_mpu.c
        Lib/imu/src/inv_mpu_dmp_motion_driver.c
        Lib/imu/src/imu.c
        Lib/control/src/yaw_hold.c
        Lib/driver/src/adc.c
        Lib/control/src/pos_loop.c)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
# 每个功能的 inc/ 都加进搜索路径, 源码里仍然可以写 #include "pid.h" 这种短名字,
# 不需要带目录前缀。新增模块时记得在这里补一行。
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Lib/app/inc
    Lib/control/inc
    Lib/comm/inc
    Lib/driver/inc
    Lib/imu/inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# M4-4 协议模式: 默认关闭, 保持原 SlamTest 37 字节协议。
# 现场编译 M4-4 时:
#   cmake -S . -B build/M44 -DAPP_USE_M44_PROTOCOL=ON ...
#   cmake --build build/M44
option(APP_USE_M44_PROTOCOL "Use M4-4 AA55+CRC16 protocol" OFF)
if(APP_USE_M44_PROTOCOL)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        APP_USE_M44_PROTOCOL=1
    )
endif()

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# 每次构建自动生成:
#   build/<BuildType>/SlamTest.bin  -> 无线烧录工具使用
#   build/<BuildType>/SlamTest.hex  -> ST-Link 备份/离线烧录用
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.bin for wireless flashing"
    VERBATIM)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.hex"
    VERBATIM)

