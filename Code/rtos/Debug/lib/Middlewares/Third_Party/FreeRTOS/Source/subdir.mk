################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
../lib/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
../lib/Middlewares/Third_Party/FreeRTOS/Source/list.c \
../lib/Middlewares/Third_Party/FreeRTOS/Source/queue.c \
../lib/Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
../lib/Middlewares/Third_Party/FreeRTOS/Source/timers.c 

OBJS += \
./lib/Middlewares/Third_Party/FreeRTOS/Source/croutine.o \
./lib/Middlewares/Third_Party/FreeRTOS/Source/event_groups.o \
./lib/Middlewares/Third_Party/FreeRTOS/Source/list.o \
./lib/Middlewares/Third_Party/FreeRTOS/Source/queue.o \
./lib/Middlewares/Third_Party/FreeRTOS/Source/tasks.o \
./lib/Middlewares/Third_Party/FreeRTOS/Source/timers.o 

C_DEPS += \
./lib/Middlewares/Third_Party/FreeRTOS/Source/croutine.d \
./lib/Middlewares/Third_Party/FreeRTOS/Source/event_groups.d \
./lib/Middlewares/Third_Party/FreeRTOS/Source/list.d \
./lib/Middlewares/Third_Party/FreeRTOS/Source/queue.d \
./lib/Middlewares/Third_Party/FreeRTOS/Source/tasks.d \
./lib/Middlewares/Third_Party/FreeRTOS/Source/timers.d 


# Each subdirectory must supply rules for building sources it contributes
lib/Middlewares/Third_Party/FreeRTOS/Source/%.o: ../lib/Middlewares/Third_Party/FreeRTOS/Source/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DSTM32F051x8 -DUSE_FULL_ASSERT -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f0-stdperiph" -I../lib/Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../lib/Drivers/CMSIS/Include -I../lib/Drivers/STM32F0xx_HAL_Driver/Inc -I../lib/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../lib/Middlewares/Third_Party/FreeRTOS/Source/include -I../lib/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../lib/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


