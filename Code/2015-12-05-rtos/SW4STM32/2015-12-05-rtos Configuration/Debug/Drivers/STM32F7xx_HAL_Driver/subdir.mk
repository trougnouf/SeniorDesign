################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal.c \
/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_cortex.c \
/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma.c \
/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma_ex.c \
/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash.c \
/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash_ex.c \
/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_gpio.c \
/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr.c \
/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr_ex.c \
/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc.c \
/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc_ex.c 

OBJS += \
./Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal.o \
./Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_cortex.o \
./Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_dma.o \
./Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_dma_ex.o \
./Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_flash.o \
./Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_flash_ex.o \
./Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_gpio.o \
./Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_pwr.o \
./Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_pwr_ex.o \
./Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_rcc.o \
./Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_rcc_ex.o 

C_DEPS += \
./Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal.d \
./Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_cortex.d \
./Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_dma.d \
./Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_dma_ex.d \
./Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_flash.d \
./Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_flash_ex.d \
./Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_gpio.d \
./Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_pwr.d \
./Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_pwr_ex.d \
./Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_rcc.d \
./Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_rcc_ex.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal.o: /home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F746xx -I"/home/trougnouf/workspace/2015-12-05-rtos/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Device/ST/STM32F7xx/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_cortex.o: /home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_cortex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F746xx -I"/home/trougnouf/workspace/2015-12-05-rtos/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Device/ST/STM32F7xx/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_cortex.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_dma.o: /home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F746xx -I"/home/trougnouf/workspace/2015-12-05-rtos/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Device/ST/STM32F7xx/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_dma.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_dma_ex.o: /home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma_ex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F746xx -I"/home/trougnouf/workspace/2015-12-05-rtos/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Device/ST/STM32F7xx/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_dma_ex.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_flash.o: /home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F746xx -I"/home/trougnouf/workspace/2015-12-05-rtos/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Device/ST/STM32F7xx/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_flash.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_flash_ex.o: /home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash_ex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F746xx -I"/home/trougnouf/workspace/2015-12-05-rtos/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Device/ST/STM32F7xx/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_flash_ex.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_gpio.o: /home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_gpio.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F746xx -I"/home/trougnouf/workspace/2015-12-05-rtos/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Device/ST/STM32F7xx/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_gpio.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_pwr.o: /home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F746xx -I"/home/trougnouf/workspace/2015-12-05-rtos/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Device/ST/STM32F7xx/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_pwr.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_pwr_ex.o: /home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr_ex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F746xx -I"/home/trougnouf/workspace/2015-12-05-rtos/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Device/ST/STM32F7xx/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_pwr_ex.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_rcc.o: /home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F746xx -I"/home/trougnouf/workspace/2015-12-05-rtos/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Device/ST/STM32F7xx/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_rcc.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_rcc_ex.o: /home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc_ex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F746xx -I"/home/trougnouf/workspace/2015-12-05-rtos/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Device/ST/STM32F7xx/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Drivers/STM32F7xx_HAL_Driver/stm32f7xx_hal_rcc_ex.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


