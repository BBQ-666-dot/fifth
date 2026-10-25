cmake_minimum_required(VERSION 3.22)

project(standard_robot C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(FIRMWARE_NAME "standard_robot")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

set(CPU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    "Src/*.c"
    "bsp/boards/*.c"
    "components/devices/*.c"
    "components/algorithm/*.c"
    "components/support/*.c"
    "components/controller/*.c"
    "application/assist/*.c"
    "application/chassis/*.c"
    "application/communication/*.c"
    "application/custom_controller/*.c"
    "application/gimbal/*.c"
    "application/IMU/*.c"
    "application/mechanical_arm/*.c"
    "application/music/*.c"
    "application/other/*.c"
    "application/referee/*.c"
    "application/remote_control/*.c"
    "application/robot_cmd/*.c"
    "application/shoot/*.c"
)
list(REMOVE_ITEM APP_SOURCES
    "${CMAKE_SOURCE_DIR}/application/assist/uart_receive.c"
)

file(GLOB HAL_SOURCES CONFIGURE_DEPENDS
    "Drivers/STM32F4xx_HAL_Driver/Src/*.c"
)

set(FREERTOS_SOURCES
    Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    Middlewares/Third_Party/FreeRTOS/Source/list.c
    Middlewares/Third_Party/FreeRTOS/Source/queue.c
    Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    Middlewares/Third_Party/FreeRTOS/Source/timers.c
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
)

set(USB_SOURCES
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
)

add_executable(standard_robot
    ${APP_SOURCES}
    ${HAL_SOURCES}
    ${FREERTOS_SOURCES}
    ${USB_SOURCES}
    gcc/startup_stm32f407xx.c
    gcc/cmsis_dsp_compat.c
    gcc/ahrs_compat.c
)
set_target_properties(standard_robot PROPERTIES OUTPUT_NAME "${FIRMWARE_NAME}")

target_include_directories(standard_robot PRIVATE
    Inc
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
    Middlewares/Third_Party/FreeRTOS/Source/include
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    application
    application/assist
    application/calibrate
    application/chassis
    application/communication
    application/custom_controller
    application/gimbal
    application/IMU
    application/mechanical_arm
    application/music
    application/other
    application/protocol
    application/referee
    application/remote_control
    application/robot_cmd
    application/shoot
    application/typedef
    bsp/boards
    components/devices
    components/algorithm
    components/algorithm/Include
    components/support
    components/controller
)

target_compile_definitions(standard_robot PRIVATE
    USE_HAL_DRIVER
    STM32F407xx
    ARM_MATH_CM4
    ARM_MATH_MATRIX_CHECK
    ARM_MATH_ROUNDING
    __FPU_USED=1U
    __FPU_PRESENT=1U
)

target_compile_options(standard_robot PRIVATE
    ${CPU_FLAGS}
    -ffunction-sections
    -fdata-sections
    -fno-common
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Wno-sign-compare
    $<$<CONFIG:Debug>:-Og;-g3>
    $<$<CONFIG:Release>:-O2;-g>
)

set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/cmake/STM32F407IGHx_FLASH.ld")
set_property(TARGET standard_robot PROPERTY LINK_DEPENDS "${LINKER_SCRIPT}")

target_link_options(standard_robot PRIVATE
    ${CPU_FLAGS}
    -T${LINKER_SCRIPT}
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    -Wl,-Map=${CMAKE_BINARY_DIR}/${FIRMWARE_NAME}.map
    --specs=nano.specs
    --specs=nosys.specs
)
target_link_libraries(standard_robot PRIVATE m)

add_custom_command(TARGET standard_robot POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:standard_robot> "${CMAKE_BINARY_DIR}/${FIRMWARE_NAME}.hex"
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:standard_robot> "${CMAKE_BINARY_DIR}/${FIRMWARE_NAME}.bin"
    COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:standard_robot>
    BYPRODUCTS
        "${CMAKE_BINARY_DIR}/${FIRMWARE_NAME}.hex"
        "${CMAKE_BINARY_DIR}/${FIRMWARE_NAME}.bin"
    COMMENT "Generating firmware HEX and BIN files"
)
