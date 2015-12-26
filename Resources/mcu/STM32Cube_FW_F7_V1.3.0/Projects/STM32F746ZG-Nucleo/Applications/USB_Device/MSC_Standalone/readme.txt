/**
  @page MSC_Standalone USB Device Mass Storage (MSC) example
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    USB_Device/MSC_Standalone/readme.txt
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-November-2015
  * @brief   Description of the USB Device MSC example.
  ******************************************************************************
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  @endverbatim

@par Example Description

This application shows how to use the USB device application based on the Mass Storage Class (MSC).

This is a typical example on how to use the STM32F7xx USB Device peripheral to communicate with a PC
Host using the Bulk Only Transfer (BOT) and Small Computer System Interface (SCSI) transparent commands,
while the microSD card is used as storage media. The STM32 MCU is enumerated as a MSC device using the
native PC Host MSC driver to which the NUCLEO-F746ZG board is connected.

At the beginning of the main program the HAL_Init() function is called to reset all the peripherals,
initialize the Flash interface and the systick. The user is provided with the SystemClock_Config()
function to configure the system clock (SYSCLK) to run at 216 MHz. The Full Speed (FS) USB module uses
internally a 48-MHz clock, which is generated from an integrated PLL.

When the application is started, the user has just to plug the USB cable into a PC host and the device
is automatically detected. A new removable drive appears in the system window and write/read/format
operations can be performed as with any other removable drive.

@note The application needs to ensure that the SysTick time base is set to 1 millisecond
      to have correct HAL configuration.

@note To reduce the example footprint, the toolchain dynamic allocation is replaced by a static allocation
      by returning the address of a pre-defined static buffer with the MSC class structure size.

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.

@par Directory contents

  - USB_Device/MSC_Standalone/Src/main.c                  Main program
  - USB_Device/MSC_Standalone/Src/system_stm32f7xx.c      STM32F7xx system clock configuration file
  - USB_Device/MSC_Standalone/Src/stm32f7xx_it.c          Interrupt handlers
  - USB_Device/MSC_Standalone/Src/usbd_conf.c             General low level driver configuration
  - USB_Device/MSC_Standalone/Src/usbd_desc.c             USB device MSC descriptor
  - USB_Device/MSC_Standalone/Src/ubsd_storage.c          Media Interface Layer
  - USB_Device/MSC_Standalone/Inc/main.h                  Main program header file
  - USB_Device/MSC_Standalone/Inc/stm32f7xx_it.h          Interrupt handlers header file
  - USB_Device/MSC_Standalone/Inc/stm32f7xx_hal_conf.h    HAL configuration file
  - USB_Device/MSC_Standalone/Inc/usbd_conf.h             USB device driver Configuration file
  - USB_Device/MSC_Standalone/Inc/usbd_desc.h             USB device MSC descriptor header file
  - USB_Device/MSC_Standalone/Inc/ubsd_storage.h          Media Interface Layer header file   

	
@par Hardware and Software environment

  - This example runs on STM32F746ZG devices.

  - This example has been tested with a NUCLEO-F746ZG Rev.B board embedding
    a STM32746ZG device and can be easily tailored to any other supported device
    and development board.

  - NUCLEO-F746ZG Set-up
    - Insert a microSD card into the Adafruit shield uSD slot mounted on NUCLEO-F746ZG Rev.B board
    - Connect the NUCLEO-F746ZG board to the PC through 'USB micro A-Male 
      to A-Male' cable to CN13 connector: 

    - Adafruit 1.8" TFT shield must be connected on CN7,CN8, CN9 and CN10 Arduino connectors, 
      for more details please refer to board User manual.

    - Make sure the SB72 is closed and the SB57 and SB80 are opened on the Nucleo_144 board (bottom side)

@par How to use it ?

In order to make the program work, you must do the following:
 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
