/**
  @page FMC_SDRAM_MemRemap SDRAM memory remap functionnalities example
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    FMC/FMC_SDRAM_MemRemap/readme.txt 
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    18-November-2015  
  * @brief   Description of the FMC SDRAM memory remap example.
  ******************************************************************************
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  @endverbatim

@par Example Description

This example guides you through the different configuration steps to use the IS42S32800G 
SDRAM memory (mounted on STM327x6G-EVAL revB evaluation board) as code execution memory. 
In addition, in this example, the SDRAM is used as data memory.

A swap between the FMC SDRAM banks and FMC NOR/PSRAM is implemented in order to enable 
the code execution from SDRAM Banks without modifying the default MPU attribute.
 
At the beginning of the main program the HAL_Init() function is called to reset 
all the peripherals, initialize the Flash interface and the systick.
Then the SystemClock_Config() function is used to configure the system
clock (SYSCLK) to run at 200 MHz.
  
The example scenario does not reflect a real application case, its purpose is to
provide only the procedure to follow to use the external SDRAM as data & execution memory.

This example does not use the default library startup file. It uses a modified 
startup file provided with the example. While startup, the SDRAM memory is configured 
and initialized to be ready to contain data.

The user has to configure his preferred toolchain using the provided linker file.
The RAM zone is modified in order to use the external SDRAM memory as a RAM.
In addition, in the linker file, a new memory section ".sdram" is added, where code
can be executed.

At this stage, all the used data can be located in the external SRAM memory.
In addition, a pragma instruction is used for SystemClock_Config() function to be executed on SDRAM.

The user can use the debugger's watch to evaluate "uwTabAddr" and "MSPValue" variables
values which should be above 0x60000000 (SDRAM offset after implementing memory mapping swap).

Eval board's LEDs can be used to monitor the example status:
1. Check SDRAM as Data Memory:
If uwTabAddr and MSPValue values are in the external SDRAM memory, LED1 is ON, otherwise the LED3 is toggling.

2. Check SDRAM as Execution Memory:
LED3 is ON when executing the SystemClock_Config function from SDRAM.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application need to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@note The STM32F7xx devices can reach a maximum clock frequency of 216MHz but as this example uses SDRAM,
      the system clock is limited to 200MHz. Indeed proper functioning of the SDRAM is only guaranteed 
      at a maximum system clock frequency of 200MHz.
      
@par Directory contents 

  - FMC/FMC_SDRAM_MemRemap/Inc/stm32f7xx_hal_conf.h    HAL configuration file
  - FMC/FMC_SDRAM_MemRemap/Inc/main.h                  Header for main.c module  
  - FMC/FMC_SDRAM_MemRemap/Inc/stm32f7xx_it.h          Interrupt handlers header file
  - FMC/FMC_SDRAM_MemRemap/Src/main.c                  Main program
  - FMC/FMC_SDRAM_MemRemap/Src/stm32f7xx_it.c          Interrupt handlers
  - FMC/FMC_SDRAM_MemRemap/Src/system_stm32f7xx.c      STM32F7xx system source file


@par Hardware and Software environment

  - This example runs on STM32F756xx/STM32F746xx devices.
    
  - This example has been tested with STM327x6G-EVAL board revB and can be
    easily tailored to any other supported device and development board.


@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
