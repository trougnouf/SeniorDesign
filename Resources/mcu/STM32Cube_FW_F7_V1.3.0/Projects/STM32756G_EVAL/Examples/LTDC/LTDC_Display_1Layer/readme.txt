/**
  @page LTDC_Display_1Layer LTDC Display Layer 1 example
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    LTDC/LTDC_Display_1Layer/readme.txt 
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    18-November-2015  
  * @brief   Description of the LTDC Display 1 layer example.
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

@par Example Description

  This example provides a description of how to configure LTDC peripheral to 
  display BMP image of size 480x272 and format RGB888 (24 bits/pixel) on LCD using only one layer.
  In this basic example the goal is to explain the different fields of the LTDC 
  structure. 

  At the beginning of the main program the HAL_Init() function is called to reset 
  all the peripherals, initialize the Flash interface and the systick.
  Then the SystemClock_Config() function is used to configure the system
  clock (SYSCLK) to run at 216 MHz.
 
  After LCD initialization, the LCD layer 1 is configured to display image 
  (modelled by an array) loaded directly from flash memory by LTDC integrated DMA.
  

  LCD_TFT synchronous timings configuration :
  -------------------------------------------
 
                                             Total Width
                          <--------------------------------------------------->
                    Hsync width HBP             Active Width                HFP
                          <---><--><--------------------------------------><-->
                      ____    ____|_______________________________________|____ 
                          |___|   |                                       |    |
                                  |                                       |    |
                      __|         |                                       |    |
         /|\    /|\  |            |                                       |    |
          | VSYNC|   |            |                                       |    |
          |Width\|/  |__          |                                       |    |
          |     /|\     |         |                                       |    |
          |  VBP |      |         |                                       |    |
          |     \|/_____|_________|_______________________________________|    |
          |     /|\     |         | / / / / / / / / / / / / / / / / / / / |    |
          |      |      |         |/ / / / / / / / / / / / / / / / / / / /|    |
 Total    |      |      |         |/ / / / / / / / / / / / / / / / / / / /|    |
 Heigh    |      |      |         |/ / / / / / / / / / / / / / / / / / / /|    |
          |Active|      |         |/ / / / / / / / / / / / / / / / / / / /|    |
          |Heigh |      |         |/ / / / / / Active Display Area / / / /|    |
          |      |      |         |/ / / / / / / / / / / / / / / / / / / /|    |
          |      |      |         |/ / / / / / / / / / / / / / / / / / / /|    |
          |      |      |         |/ / / / / / / / / / / / / / / / / / / /|    |
          |      |      |         |/ / / / / / / / / / / / / / / / / / / /|    |
          |      |      |         |/ / / / / / / / / / / / / / / / / / / /|    |
          |     \|/_____|_________|_______________________________________|    |
          |     /|\     |                                                      |
          |  VFP |      |                                                      |
         \|/    \|/_____|______________________________________________________|
         
         
  Each LCD device has its specific synchronous timings values.
  This example uses AM480272H3TMQW-T01H LCD (MB1046 B-01) and configures 
  the synchronous timings as following :

  Horizontal Synchronization (Hsync) = 41
  Horizontal Back Porch (HBP)        = 2
  Active Width                       = 480
  Horizontal Front Porch (HFP)       = 2

  Vertical Synchronization (Vsync)   = 10
  Vertical Back Porch (VBP)          = 2
  Active Heigh                       = 272
  Vertical Front Porch (VFP)         = 2
  
  LCD_TFT windowing configuration :
  ---------------------------------

  To configure the active display window, this example configures the 
  horizontal/vertical start and stop. 

  HorizontalStart = (Offset_X + Hsync + HBP);
  HorizontalStop  = (Offset_X + Hsync + HBP + Window_Width - 1); 
  VarticalStart   = (Offset_Y + Vsync + VBP);
  VerticalStop    = (Offset_Y + Vsync + VBP + Window_Heigh - 1);
  
  Window_width and Window_heigh should be in line with the image size to be 
  displayed.

STM32 Eval board's LED can be used to monitor the transfer status:
 - LED3 is ON when there is an error in Init process.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application need to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.


@par Directory contents

    - LTDC/LTDC_Display_2Layers/Inc/stm32f7xx_hal_conf.h  HAL configuration file
    - LTDC/LTDC_Display_2Layers/Inc/stm32f7xx_it.h        Interrupt handlers header file
    - LTDC/LTDC_Display_2Layers/Inc/main.h                Main configuration file
    - LTDC/LTDC_Display_2Layers/Src/stm32f7xx_it.c        Interrupt handlers
    - LTDC/LTDC_Display_2Layers/Src/main.c                Main program 
    - LTDC/LTDC_Display_2Layers/Src/stm32f7xx_hal_msp.c   HAL MSP module	
    - LTDC/LTDC_Display_2Layers/Src/system_stm32f7xx.c    STM32F7xx system clock configuration file
    - LTDC/LTDC_Display_2Layers/Inc/RGB565_480x272.h      Image of size 480x272 and format RGB565 to be displayed on LCD

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
                                   