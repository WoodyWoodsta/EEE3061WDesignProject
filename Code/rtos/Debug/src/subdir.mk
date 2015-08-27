################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/_write.c \
../src/freertos.c \
../src/main.c \
../src/stm32f0xx_hal_msp.c \
../src/stm32f0xx_it.c 

OBJS += \
./src/_write.o \
./src/freertos.o \
./src/main.o \
./src/stm32f0xx_hal_msp.o \
./src/stm32f0xx_it.o 

C_DEPS += \
./src/_write.d \
./src/freertos.d \
./src/main.d \
./src/stm32f0xx_hal_msp.d \
./src/stm32f0xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DSTM32F051x8 -DUSE_FULL_ASSERT -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f0-stdperiph" -I../lib/Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../lib/Drivers/CMSIS/Include -I../lib/Drivers/STM32F0xx_HAL_Driver/Inc -I../lib/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../lib/Middlewares/Third_Party/FreeRTOS/Source/include -I../lib/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0 -I../lib/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


