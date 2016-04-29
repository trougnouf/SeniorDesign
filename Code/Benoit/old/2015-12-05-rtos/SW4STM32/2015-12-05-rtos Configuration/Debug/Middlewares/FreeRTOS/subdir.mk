################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c \
/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/list.c \
/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/port.c \
/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/queue.c \
/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/timers.c 

OBJS += \
./Middlewares/FreeRTOS/cmsis_os.o \
./Middlewares/FreeRTOS/croutine.o \
./Middlewares/FreeRTOS/event_groups.o \
./Middlewares/FreeRTOS/heap_4.o \
./Middlewares/FreeRTOS/list.o \
./Middlewares/FreeRTOS/port.o \
./Middlewares/FreeRTOS/queue.o \
./Middlewares/FreeRTOS/tasks.o \
./Middlewares/FreeRTOS/timers.o 

C_DEPS += \
./Middlewares/FreeRTOS/cmsis_os.d \
./Middlewares/FreeRTOS/croutine.d \
./Middlewares/FreeRTOS/event_groups.d \
./Middlewares/FreeRTOS/heap_4.d \
./Middlewares/FreeRTOS/list.d \
./Middlewares/FreeRTOS/port.d \
./Middlewares/FreeRTOS/queue.d \
./Middlewares/FreeRTOS/tasks.d \
./Middlewares/FreeRTOS/timers.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/FreeRTOS/cmsis_os.o: /home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F746xx -I"/home/trougnouf/workspace/2015-12-05-rtos/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Device/ST/STM32F7xx/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Middlewares/FreeRTOS/cmsis_os.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Middlewares/FreeRTOS/croutine.o: /home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/croutine.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F746xx -I"/home/trougnouf/workspace/2015-12-05-rtos/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Device/ST/STM32F7xx/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Middlewares/FreeRTOS/croutine.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Middlewares/FreeRTOS/event_groups.o: /home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F746xx -I"/home/trougnouf/workspace/2015-12-05-rtos/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Device/ST/STM32F7xx/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Middlewares/FreeRTOS/event_groups.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Middlewares/FreeRTOS/heap_4.o: /home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F746xx -I"/home/trougnouf/workspace/2015-12-05-rtos/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Device/ST/STM32F7xx/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Middlewares/FreeRTOS/heap_4.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Middlewares/FreeRTOS/list.o: /home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/list.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F746xx -I"/home/trougnouf/workspace/2015-12-05-rtos/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Device/ST/STM32F7xx/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Middlewares/FreeRTOS/list.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Middlewares/FreeRTOS/port.o: /home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/port.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F746xx -I"/home/trougnouf/workspace/2015-12-05-rtos/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Device/ST/STM32F7xx/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Middlewares/FreeRTOS/port.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Middlewares/FreeRTOS/queue.o: /home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/queue.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F746xx -I"/home/trougnouf/workspace/2015-12-05-rtos/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Device/ST/STM32F7xx/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Middlewares/FreeRTOS/queue.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Middlewares/FreeRTOS/tasks.o: /home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/tasks.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F746xx -I"/home/trougnouf/workspace/2015-12-05-rtos/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Device/ST/STM32F7xx/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Middlewares/FreeRTOS/tasks.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Middlewares/FreeRTOS/timers.o: /home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/timers.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F746xx -I"/home/trougnouf/workspace/2015-12-05-rtos/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Include" -I"/home/trougnouf/workspace/2015-12-05-rtos/Drivers/CMSIS/Device/ST/STM32F7xx/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"Middlewares/FreeRTOS/timers.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


