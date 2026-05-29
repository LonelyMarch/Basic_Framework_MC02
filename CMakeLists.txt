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
set(CMAKE_PROJECT_NAME Basic_Framework_MC02)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# CLion STM32CubeCLT profiles may pass compilers directly without passing
# CMAKE_TOOLCHAIN_FILE. Load the project toolchain before project() so CMake
# treats this as a bare-metal cross compile instead of a Windows host build.
if(NOT CMAKE_TOOLCHAIN_FILE)
    include(${CMAKE_CURRENT_LIST_DIR}/cmake/gcc-arm-none-eabi.cmake)
endif()

# Core project settings
project(${CMAKE_PROJECT_NAME} C ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Drivers/CMSIS/DSP/Lib/GCC
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_printf.c
    Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT.c
    Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT_ASM_ARMv7M.s

    bsp/dwt/bsp_dwt.c
    bsp/bsp_init.c
    bsp/bsp_frame_queue.c
    bsp/adc/bsp_adc.c
    bsp/pwm/bsp_pwm.c
    bsp/gpio/bsp_gpio.c
    bsp/spi/bsp_spi.c
    bsp/iic/bsp_iic.c
    bsp/can/bsp_can.c
    bsp/usart/bsp_usart.c
    bsp/usb/bsp_usb.c
    bsp/bsp_service.c
    bsp/log/bsp_log.c
    bsp/flash/bsp_flash.c
    bsp/flash/bsp_flash_async.c
    bsp/qspi_flash/bsp_qspi_flash.c

    modules/algorithm/controller.c
    modules/algorithm/kalman_filter.c
    modules/algorithm/QuaternionEKF.c
    modules/algorithm/chassis_kinematics.c
    modules/algorithm/algo_filter.c
    modules/algorithm/crc8.c
    modules/algorithm/crc16.c
    modules/algorithm/user_lib.c
    modules/BMI088/bmi088.c
    modules/imu/ins_task.c
    modules/master_machine/master_process.c
    modules/master_machine/seasky_protocol.c
    modules/motor/DJImotor/dji_motor.c
    modules/motor/HTmotor/HT04.c
    modules/motor/LKmotor/LK9025.c
    modules/motor/DMmotor/dmmotor.c
    modules/motor/step_motor/step_motor.c
    modules/motor/servo_motor/servo_motor.c
    modules/motor/motor_task.c
    modules/oled/oled.c
    modules/referee/crc_ref.c
    modules/referee/rm_referee.c
    modules/referee/referee_UI.c
    modules/referee/referee_task.c
    modules/remote/remote_control.c
    modules/super_cap/super_cap.c
    modules/can_comm/can_comm.c
    modules/message_center/message_center.c
    modules/daemon/daemon.c
    modules/alarm/buzzer.c

    application/gimbal/gimbal.c
    application/chassis/chassis.c
    application/shoot/shoot.c
    application/cmd/robot_cmd.c
    application/robot.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    application/chassis
    application/shoot
    application/gimbal
    application/cmd
    application

    bsp/dwt
    bsp/can
    bsp/usart
    bsp/usb
    bsp/gpio
    bsp/spi
    bsp/iic
    bsp/log
    bsp/flash
    bsp/qspi_flash
    bsp/adc
    bsp/pwm
    bsp

    modules/algorithm
    modules/BMI088
    modules/imu
    modules/master_machine
    modules/motor/DJImotor
    modules/motor/LKmotor
    modules/motor/HTmotor
    modules/motor/step_motor
    modules/motor/servo_motor
    modules/motor/DMmotor
    modules/motor
    modules/oled
    modules/referee
    modules/remote
    modules/super_cap
    modules/can_comm
    modules/message_center
    modules/daemon
    modules/alarm
    modules

    Middlewares/ST/ARM/DSP/Include/dsp
    Middlewares/Third_Party/SEGGER/RTT
    Middlewares/Third_Party/SEGGER/Config
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM7
    $<$<CONFIG:Debug>:DISABLE_LOG_SYSTEM=0>
    $<$<CONFIG:Release>:DISABLE_LOG_SYSTEM=1>
    $<$<CONFIG:RelWithDebInfo>:DISABLE_LOG_SYSTEM=1>
    $<$<CONFIG:MinSizeRel>:DISABLE_LOG_SYSTEM=1>
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    c
    m
    nosys
    arm_cortexM7lfdp_math
)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary -S $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating size report, HEX and BIN"
)
