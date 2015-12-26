/**
  @page TIM_DMABurst TIM_DMABurst example
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    TIM/TIM_DMABurst/readme.txt 
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    18-November-2015  
  * @brief   Description of the TIM DMA Burst example.
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

This example shows how to update the TIM2 channel1 period and the duty cycle 
using the TIM2 DMA burst feature.

Every update DMA request, the DMA will do 3 transfers of half words into Timer 
registers beginning from ARR register.
On the DMA update request, 0x0FFF will be transferred into ARR, 0x0000 
will be transferred into RCR, 0x0555 will be transferred into CCR1. 

The TIM2CLK frequency is set to (SystemCoreClock/2), to get TIM2 counter
clock at 20 MHz the Prescaler is computed as following:
- Prescaler = (TIM2CLK / TIM2 counter clock) - 1

SystemCoreClock is set to 216 MHz.

The TIM2 Frequency = TIM2 counter clock/(ARR + 1)
                   = 20 MHz / 4096 = 4.88 KHz

The TIM2 CCR1 register value is equal to 0x555, so the TIM2 Channel 1 generates a 
PWM signal with a frequency equal to 4.88 KHz and a duty cycle equal to 33.33%:
TIM2 Channel1 duty cycle = (TIM2_CCR1/ TIM2_ARR + 1)* 100 = 33.33%

The PWM waveform can be displayed using an oscilloscope.

@note Care must be taken when using HAL_Delay(), this function provides accurate
      delay (in milliseconds) based on variable incremented in SysTick ISR. This
      implies that if HAL_Delay() is called from a peripheral ISR process, then 
      the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application need to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@par Directory contents  

  - TIM/TIM_DMABurst/Inc/stm32f7xx_hal_conf.h    HAL configuration file
  - TIM/TIM_DMABurst/Inc/stm32f7xx_it.h          Interrupt handlers header file
  - TIM/TIM_DMABurst/Inc/main.h                  Header for main.c module  
  - TIM/TIM_DMABurst/Src/stm32f7xx_it.c          Interrupt handlers
  - TIM/TIM_DMABurst/Src/main.c                  Main program
  - TIM/TIM_DMABurst/Src/stm32f7xx_hal_msp.c     HAL MSP file
  - TIM/TIM_DMABurst/Src/system_stm32f7xx.c      STM32F7xx system source file

@par Hardware and Software environment

  - This example runs on STM32756xx devices.
  - In this example, the clock is set to 216 MHz.
    
  - This example has been tested with STMicroelectronics STM327x6G-EVAL revB 
    board and can be easily tailored to any other supported device 
    and development board.

  - STM327x6G-EVAL revB Set-up
    - Connect the TIM2 pin to an oscilloscope to monitor the different waveforms: 
      - TIM2 CH1 (PA.00 (pin 15 in CN5 connector))

@par How to use it ? 

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
