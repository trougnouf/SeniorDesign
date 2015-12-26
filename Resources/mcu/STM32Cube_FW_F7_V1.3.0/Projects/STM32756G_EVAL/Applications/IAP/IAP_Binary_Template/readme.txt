/**
  @page IAP_Binary_Template Binary Template Readme file

  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    IAP/IAP_Binary_Template/readme.txt 
  * @author  MCD Application Team
  * @version V1.0.3
  * @date    18-November-2015
  * @brief   Description of the IAP_Binary_Template directory.
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
  */

@par Example Description

This directory contains a set of sources files that build the application to be
loaded into Flash memory using In-Application Programming (IAP, through USART).

To build such application, some special configuration has to be performed:
1. Set the program load address at 0x08008000, using your toolchain linker file
2. Relocate the vector table at address 0x08008000, using the "NVIC_SetVectorTable"
   function.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application need to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.
      
Four LEDs are toggled with a timing defined by the Delay function.


@par Directory contents 

 - IAP\IAP_Binary_Template/Inc/stm32f7xx_hal_conf.h          HAL Configuration file
 - IAP\IAP_Binary_Template/Inc/main.h                        Header for main.c module
 - IAP\IAP_Binary_Template/Inc/stm32f7xx_it.h                Header for stm32f7xx_it.c
 - IAP\IAP_Binary_Template/Src/main.c                        Main program
 - IAP\IAP_Binary_Template/Src/stm32f7xx_it.c                Interrupt handlers
 - IAP\IAP_Binary_Template/Src/system_stm32f7xx.c            STM32F7xx system clock configuration file 

     
@par Hardware and Software environment

  - This example runs on STM32F756xx/STM32F746xx devices.
    
  - This example has been tested with STM327x6G-EVAL board revB and can be
    easily tailored to any other supported device and development board.

@par How to use it ?  

In order to load the SysTick example with the IAP, you must do the following:
 
 - EWARM:
    - Open the Project.eww workspace
    - Rebuild all files
    - A binary file "STM327x6G_EVAL_SysTick.bin" will be generated under "STM327x6G_EVAL/Exe" folder.  
    - Finally load this image with IAP application

 - MDK-ARM:
    - Open the Project.uvproj project
    - Rebuild all files: Project->Rebuild all target files
    - Go to "/IAP_Binary_Template/MDK-ARM" directory and run "axftobin.bat"
      (Fromelf Exe path might have to be updated in "axftobin.bat" file, according to your Keil setup).
    - A binary file "STM327x6G_EVAL_SysTick.bin" will be generated under "STM327x6G_EVAL" folder. 
    - Finally load this image with IAP application

 - System Workbench for STM32: 
    - Open System Workbench for STM32 toolchain
    - Browse to the SW4STM32 workspace directory, select the project 
      (.project file in \IAP_Binary_Template\SW4STM32 \STM327x6G_EVAL directory).
    - Rebuild all project files: Select the project in the "Project explorer" window 
      then click on Project->build project menu.
    - Load this image with the IAP application (Select option 1 in IAP menu)

 * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
 */
