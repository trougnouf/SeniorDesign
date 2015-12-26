/**
  @page LTDC_PicturesFromSDCard LTDC pictures from SD card application
  
  @verbatim
  ******************************************************************************
  * @file    Display/LTDC_PicturesFromSDCard/readme.txt 
  * @author  MCD Application Team
  * @version V1.0.3
  * @date    18-November-2015
  * @brief   Description of the LTDC pictures from SD card application.
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

This application describes how to display pictures on LCD saved under SD card.
 
The user has to create a "Media" directory under the micro SD card root and
copy inside it the pictures stored in "/BMP_320x240" and "/BMP_480x272" folders provided under 
"/Utilities/Media/Pictures"
   
 
@note : The user can add pictures of his choice but the file type, resolution
        and pixel format described below must be respected.
  
  At the beginning, the program checks the existence of an SD card. If the SD card
  is not inserted, a warning message is displayed on the LCD :" Please insert SD Card. "
  Once the LCD, SD card and file systems initialized and configured, 
  a content check of the "Media" directory is done and the number of ".BMP" files 
  is retained.
  
@note : An associate LCD eval driver is used in this application 
  
@note : The maximum number of BMP files is fixed to 25. It can be raised 
        until reaching the maximum of SD card memory space.

  The following steps are performed to scroll all the images stored in the 
  SD Card :
 
  Step1 :
  ------
  The foreground layer is set, the image copied from SD card to intermediate
  SDRAM memory and then copied to LCD frame buffer. The image is fully visible when 
  the level of transparency is increased until it becomes totally opaque (reaches 255).
 
  Press tamper button to display next image.
 
  Step2 :
  ------
  When the Tamper button is pressed the transparency of the foreground layer
  begins to decrease until becoming totally transparent and the background layer 
  is set, the image copied from SD card to intermediate SDRAM memory and then 
  copied to LCD frame buffer and finally the level of transparency of the 
  background layer increases until it reaches 255 (totally opaque). 
            
  Press tamper button to display next image.
 
  Step3 :
  ------
  When the Tamper button is pressed the transparency of the background layer
  begins to decrease until becoming totally transparent and return to step1.
  
@note the system clock (SYSCLK) is configured to run at 200 MHz and 50 MHz is provided 
      at the output PLL divided by PLL_Q. This frequency permits to reach 25 MHz clock 
      needed for SD operation and in line with microSD specification. 
      
@note 
  => If the "Media" directory is empty, a warning message is displayed on 
     the LCD : "  No Bitmap files...  "
    
  => If the file type stored in the "Media" directory is not supported,
     a warning message is displayed on the LCD : " file type not supported. "
    
  => If the SD card is removed while running the application, no warning message
     is displayed on the LCD. A reset is needed.
   
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

-  LED3 is ON when the uSD disk I/O driver is not Linked;


@par Directory contents

    - Display/LTDC_PicturesFromSDCard/Inc/main.h                             Main configuration file
    - Display/LTDC_PicturesFromSDCard/Inc/stm32f7xx_it.h                     Interrupt handlers header file
    - Display/LTDC_PicturesFromSDCard/Inc/stm32f7xx_hal_conf.h               HAL Configuration file 
    - Display/LTDC_PicturesFromSDCard/Inc/ffconf.h                           FAT file system module configuration file
    - Display/LTDC_PicturesFromSDCard/Inc/fatfs_storage.h                    Header for fatfs_storage.c
    - Display/LTDC_PicturesFromSDCard/Src/main.c                             Main program 
    - Display/LTDC_PicturesFromSDCard/Src/fatfs_storage.c                    Storage (FatFs) driver
    - Display/LTDC_PicturesFromSDCard/Src/stm32f7xx_it.c                     Interrupt handlers
    - Display/LTDC_PicturesFromSDCard/Src/system_stm32f7xx.c                 STM32F7xx system clock configuration file


@par Hardware and Software environment  

  - This application runs on STM32F756xx/STM32F746xx devices.
    
  - This application has been tested with STMicroelectronics STM327x6G-EVAL
    evaluation boards and can be easily tailored to any other supported device 
    and development board.
    
  - STM327x6G-EVAL Set-up
    - Ensure that JP24 is in position 2-3 to use LED1.
    - Ensure that JP23 is in position 2-3 to use LED3. 


@par How to use it ?

In order to make the program work, you must do the following :
 - The bitmap images, available under "/BMP_320x240" and "/BMP_480x272" folders 
   provided under "/Utilities/Media/Pictures" should be copied inside a 
   dedicated directory named "Media" at the micro SD card root.
   The images should have the following properties:
     file type    : *.bmp
     resolution   : up to 480x272 with AM480272H3TMQW LCD and up to 640x480 with 
                    AM-640480G5TNQW-T00H LCD
     pixel format : RGB565, RGB888 or ARGB8888     
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the application 
                
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
                                   