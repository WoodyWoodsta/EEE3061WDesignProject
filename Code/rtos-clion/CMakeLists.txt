cmake_minimum_required(VERSION 3.3)
include(CMakeForceCompiler)
project(rtos_clion)
# -------------------------------------------------------------------

set(CMAKE_SYSTEM_PROCESSOR cortex-m0)
SET(CMAKE_SYSTEM_NAME Generic)

# Target Environment
SET(CMAKE_FIND_ROOT_PATH "C:/Program Files (x86)/GNU Tools ARM Embedded/4.9 2015q2")

# Cross compiler
CMAKE_FORCE_C_COMPILER(arm-none-eabi-gcc GNU)
CMAKE_FORCE_CXX_COMPILER(arm-none-eabi-g++ GNU)

# Executable type
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

# Compiler flags
set(ARM_FLAGS "-mcpu=cortex-m0 -mthumb -Og")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ARM_FLAGS} -std=gnu++11 -fabi-version=0 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -ffunction-sections -g")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 ${ARM_FLAGS} -ffunction-sections -fmessage-length=0 -fsigned-char -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra -g")
set(CMAKE_EXE_LINKER_FLAGS "${ARM_FLAGS} -g -Wl,--library-path=\"${PROJECT_SOURCE_DIR}\" -T mem.ld -T libs.ld -T sections.ld -Xlinker --gc-sections -Wl,-Map=${PROJECT_NAME}.map --specs=nano.specs")

set(CMAKE_C_LINK_EXECUTABLE "<CMAKE_C_COMPILER> <LINK_FLAGS> -o <TARGET> <OBJECTS>")
set(CMAKE_CXX_LINK_EXECUTABLE "<CMAKE_CXX_COMPILER> <LINK_FLAGS> -o <TARGET> <OBJECTS>")
# -------------------------------------------------------------------

# Global definitions
add_definitions(-DDEBUG -DUSE_FULL_ASSERT -DSTM32F051x8 -DHSE_VALUE=8000000)

include_directories(include
    system/include
    system/include/cmsis
    system/include/stm32f0-stdperiph
    lib/Drivers/CMSIS/Device/ST/STM32F0xx/Include
    lib/Drivers/CMSIS/Include
    lib/Drivers/STM32F0xx_HAL_Driver/Inc
    lib/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    lib/Middlewares/Third_Party/FreeRTOS/Source/include
    lib/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0
    lib/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy)

set(SOURCE_FILES
    lib/Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/startup_stm32f0xx.c
    lib/Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/system_stm32f0xx.c
    lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal.c
    lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_adc.c
    lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_adc_ex.c
    lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_cortex.c
    lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_dma.c
    lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash.c
    lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash_ex.c
    lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_gpio.c
    lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr.c
    lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr_ex.c
    lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc.c
    lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc_ex.c
    lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_spi.c
    lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_spi_ex.c
    lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim.c
    lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim_ex.c
    lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_uart.c
    lib/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_uart_ex.c
    lib/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    lib/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0/port.c
    lib/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    lib/Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    lib/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    lib/Middlewares/Third_Party/FreeRTOS/Source/list.c
    lib/Middlewares/Third_Party/FreeRTOS/Source/queue.c
    lib/Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    lib/Middlewares/Third_Party/FreeRTOS/Source/timers.c
    src/_write.c
    src/bossTask_task.c
    src/freertos.c
    src/genericMessaging_lib.c
    src/hal_lib.c
    src/main.c
    src/motorTask_task.c
    src/sensorTask_task.c
    src/stm32f0xx_hal_msp.c
    src/stm32f0xx_it.c
    src/strings_res.c
    src/USARTInTask_task.c
    src/USARTOutTask_task.c
    src/userIOTask_task.c
    src/userTasks_task.c
    src/userTimers_timer.c
    system/src/diag/Trace.c
    system/src/diag/trace_impl.c
    system/src/newlib/_cxx.cpp
    system/src/newlib/_exit.c
    system/src/newlib/_sbrk.c
    system/src/newlib/_startup.c
    system/src/newlib/_syscalls.c
    system/src/newlib/assert.c)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

# Print executable size
add_custom_command(
    TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND arm-none-eabi-size --format=berkeley ${PROJECT_NAME}.elf)

# Search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM	NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY	ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE	ONLY)