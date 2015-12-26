/**
  @page QSPI_perfs Displays pictures from QSPI flash memory and data transfer performance
  
  @verbatim
  ******************************************************************************
  * @file    QSPI/QSPI_perfs/readme.txt 
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    18-November-2015 
  * @brief   Description of the QSPI performance measurement application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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

@par Application Description

This application describes how to display pictures stored on QSPI flash memory on LCD and
measures data transfer performance between QSPI flash and SDRAM memory.

The application uses DMA2D to render several animated layers on LCD. All pictures and icons
are stored in QSPI flash memory. The DMA2D is used to transfer pictures from QSPI flash memory
to SDRAM memory. During this transfer, the transfer time is measured, then the transfer speed
is calculated and displayed on LCD.

The application needs pictures flashed in QSPI flash memory to work, if using EWARM toolchain, the 
internal flash and QSPI flash programming is done by tool, but if using MDK-ARM or SW4STM32 toolchains,
these tools can't flash QSPI flash. So, in this case ST-link utility has to be used to flash these memories
(see details below).
   
@note the system clock (SYSCLK) is configured to run at 200 MHz. 50 MHz are provided 
      at the output PLL divided by PLL_Q. This frequency permits to reach the 25 MHz clock frequency
      needed for SD operation and in line with microSD specification. 
      
@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.

@note The STM32F7xx devices can reach a maximum clock frequency of 216MHz but as this application uses SDRAM,
      the system clock is limited to 200MHz. Indeed proper functioning of the SDRAM is only guaranteed 
      at a maximum system clock frequency of 200MHz.
      

@par Directory contents

    - QSPI/QSPI_perfs/Inc/main.h                             Main configuration file
    - QSPI/QSPI_perfs/Inc/stm32f7xx_it.h                     Interrupt handlers header file
    - QSPI/QSPI_perfs/Inc/stm32f7xx_hal_conf.h               HAL Configuration file 
    - QSPI/QSPI_perfs/Inc/icon_S.h                           Icon bitmap definition in ARGB4444 format
    - QSPI/QSPI_perfs/Inc/icon_T.h                           Icon bitmap definition in ARGB4444 format
    - QSPI/QSPI_perfs/Inc/icon_M.h                           Icon bitmap definition in ARGB4444 format
    - QSPI/QSPI_perfs/Inc/img1.h                             Picture bitmap definition in RGB565 format
    - QSPI/QSPI_perfs/Inc/img2.h                             Picture bitmap definition in RGB565 format
    - QSPI/QSPI_perfs/Inc/img3.h                             Picture bitmap definition in RGB565 format
    - QSPI/QSPI_perfs/Inc/img4.h                             Picture bitmap definition in RGB565 format
    - QSPI/QSPI_perfs/Inc/img5.h                             Picture bitmap definition in RGB565 format
    - QSPI/QSPI_perfs/Inc/img6.h                             Picture bitmap definition in RGB565 format
    - QSPI/QSPI_perfs/Src/main.c                             Main program 
    - QSPI/QSPI_perfs/Src/main_images.c                      pictures inclusion
    - QSPI/QSPI_perfs/Src/stm32f7xx_it.c                     Interrupt handlers
    - QSPI/QSPI_perfs/Src/system_stm32f7xx.c                 STM32F7xx system clock configuration file


@par Hardware and Software environment  

  - This application runs on STM32F7xx devices.
    
  - This application has been tested with STMicroelectronics STM32746G-DISCOVERY boards and can be
    easily tailored to any other supported device and development board.
    
@par How to use it ?

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - if MDK-ARM or SW4STM32 toolchain used :
   - Open STM32 ST-Link Utility V3.7, click on "External Loader" from the bar menu then check "N25Q128A_STM32F746G-DISCO" box 
   - Connect the STM32746G-DISCOVERY board to PC with USB cable through CN14
   - Use generated hex file with STM32 ST-Link Utility to program both internal Flash and external QSPI memory
 - Run the application 
                
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
                                   