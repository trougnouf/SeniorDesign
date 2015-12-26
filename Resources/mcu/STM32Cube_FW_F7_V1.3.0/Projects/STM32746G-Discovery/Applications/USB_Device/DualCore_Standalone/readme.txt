/**
  @page DualCore_Standalone USB Device Dual Core application
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    USB_Device/DualCore_Standalone/readme.txt 
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    18-November-2015 
  * @brief   Description of the USB Device Dual Core application.
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

@par Application Description 

This application is a part of the USB Device Library package using STM32Cube firmware. It describes how to use
USB device application based on the STM32F746xx multi core support feature integrating Mass Storage (MSC) 
and Human Interface (HID) in the same project.

This is a typical application on how to use the STM32F746xx USB OTG Device peripheral, where STM32 is 
enumerated as a MSC device in the High Speed mode, and also as a HID device in the Full Speed mode,
using the native PC Host HID/MSC drivers to which the STM32746G-Discovery board is connected.

At the beginning of the main program the HAL_Init() function is called to reset all the peripherals,
initialize the Flash interface and the systick. The user is provided with the SystemClock_Config()
function to configure the system clock (SYSCLK) to run at 216 MHz. The Full Speed (FS) USB module uses
internally a 48-MHz clock which is coming from a specific output of two PLLs: main PLL or PLL SAI.
In the High Speed (HS) mode the USB clock (60 MHz) is driven by the ULPI.

The 48 MHz clock for the USB FS can be derived from one of the two following sources:
  � PLL clock (clocked by the HSE): If the USB uses the PLL as clock source, the PLL clock must be programmed
    to output 48 MHz frequency (USBCLK = PLLVCO/PLLQ).
  � PLLSAI clock (clocked by the HSE): If the USB uses the PLLSAI as clock source, the PLLSAI clock must be programmed
    to output 48 MHz frequency (USBCLK = PLLSAIVCO/PLLSAIP).
    
When the application is started, the user has just to plug the two USB cables into a PC host and two
USB devices (MSC/HID) are automatically detected. A new removable drive appears in the system window
and write/read/format operations can be performed as with any other removable drive, A new HID Mouse 
device is detected, and the User button mounted on the STM32746G-Discovery board, allows to user to move
the cursor forward and backward. 

@note Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
      based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
      a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
      than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
      
@note The application needs to ensure that the SysTick time base is always set to 1 millisecond
      to have correct HAL operation.
      
For more details about the STM32Cube USB Device library, please refer to UM1734 
"STM32Cube USB Device library".


@par Directory contents

  - USB_Device/DualCore_Standalone/Src/main.c                  Main program
  - USB_Device/DualCore_Standalone/Src/system_stm32f7xx.c      STM32F7xx system clock configuration file
  - USB_Device/DualCore_Standalone/Src/stm32f7xx_it.c          Interrupt handlers
  - USB_Device/DualCore_Standalone/Src/usbd_conf.c             General low level driver configuration
  - USB_Device/DualCore_Standalone/Src/usbd_msc_desc.c         USB device MSC descriptor
  - USB_Device/DualCore_Standalone/Src/usbd_hid_desc.c         USB device HID descriptor
  - USB_Device/DualCore_Standalone/Src/ubsd_storage.c          Media Interface Layer
  - USB_Device/DualCore_Standalone/Inc/main.h                  Main program header file
  - USB_Device/DualCore_Standalone/Inc/stm32f7xx_it.h          Interrupt handlers header file
  - USB_Device/DualCore_Standalone/Inc/stm32f7xx_hal_conf.h    HAL configuration file
  - USB_Device/DualCore_Standalone/Inc/usbd_conf.h             USB device driver Configuration file
  - USB_Device/DualCore_Standalone/Inc/usbd_desc.h             USB device descriptor header file  
  - USB_Device/DualCore_Standalone/Inc/usbd_msc_desc.h         USB device MSC descriptor header file
  - USB_Device/DualCore_Standalone/Inc/usbd_hid_desc.h         USB device HID descriptor header file
  - USB_Device/DualCore_Standalone/Inc/ubsd_storage.h          Media Interface Layer header file 
  
@par Hardware and Software environment

  - This application runs on STM32F746xx devices.
    
  - This application has been tested with STMicroelectronics STM32746G-Discovery
    boards and can be easily tailored to any other supported device and development
    board.

  - STM32746G-Discovery Set-up
    - Insert a microSD card into the STM32746G-Discovery uSD slot (CN3)
    - Connect the STM32746G-Discovery board to the PC through two 'USB micro A-Male 
      to A-Male' cables to the connectors:
      - CN12: to use USB High Speed (HS) 
      - CN13: to use USB Full Speed (FS)            

@par How to use it ?

In order to make the program work, you must do the following :
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the application
 
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
