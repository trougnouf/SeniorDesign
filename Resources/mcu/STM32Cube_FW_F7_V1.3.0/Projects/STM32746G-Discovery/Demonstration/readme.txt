/**
  @page Demo   STM32746G-DISCO Demonstration Firmware
 
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    Demonstrations/readme.txt 
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    18-November-2015
  * @brief   Description of STM32746G-DISCO Demonstration
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

@par Demo Description

The STM32Cube Demonstration platform comes on top of the STM32CubeTM as a firmware
package that offers a full set of software components based on a modules architecture
allowing re-using them separately in standalone applications. All these modules are
managed by the STM32Cube Demonstration kernel allowing to dynamically adding new
modules and access to common resources (storage, graphical components and widgets,
memory management, Real-Time operating system)

The STM32Cube Demonstration platform is built around the powerful graphical library
STemWin and the FreeRTOS real time operating system and uses almost the whole STM32
capability to offer a large scope of usage based on the STM32Cube HAL BSP and several
middleware components.

@par Demo Description

The STM32 F7 demonstration is running on STM32746G-DISCOVERY boards RevB. 

  
Below you find an overview of the different offered module in the demonstration:

 + Video player
 --------------
 The video player module provides a video solution based on the STM32F7xx and STemWin
 movie API. It supports playing movie in emf format.
 
 + Audio player
 --------------
 The audio player module provides a complete audio solution based on the STM32F7xx and
 delivers a high-quality music experience. It supports playing music in WAV format but may
 be extended to support other compressed formats such as MP3 and WMA audio formats.
 The Module supports background mode feature.
 
  + Audio recorder
 -----------------
 The audio recorder module allows recording audio and playing it back.
 
 + VNC Server
 ------------
 The VNC server module allows to control the demo from a remote machine. It is based on
 the TCP/IP LwIP stacks. The background mode is supported.
 
 + Audio Recorder
 ----------------
 The Audio record module allows to record an audio file. The audio format supported is 
 WAV format but may be extended to support other compressed formats such as MP3.
 The recorded files are stored in USB Disk flash(USB High Speed).
  
 + Home alarm
 ------------ 
 Control of Home alarm system, equipped with cameras.
 Static picture shown when a room is selected and then the camera icon pressed
 General room alarm activation/deactivation when pressed.
 
 + Gardening control
 -------------------
 The gardening control module provides a graphic garden watering system behaviour
 
 + Game
 ------
 The game coming in the STM32Cube demonstration is based on the Reversi game. It is a
 strategy board game for two players, played on an 8×8 board. The goal of the game is to
 have the majority of disks turned to display your color when the last playable empty square
 is filled.
 
 + System Info
 --------------  
 The system info module provides information about the core, supported eval boards, 
 CPU speed and demonstration version.  
    
    For more details about the demonstration modules please refers to  STM32CubeF7 demonstration (UM1906)
    
@note The STM32F7xx devices can reach a maximum clock frequency of 216MHz but as this demonstration uses SDRAM,
      the system clock is limited to 200MHz. Indeed proper functioning of the SDRAM is only guaranteed 
      at a maximum system clock frequency of 200MHz.
      
@par Hardware and Software environment

  - This application runs on STM32F746xx devices.  

  - This example has been tested with STMicroelectronics STM32746G_DISCOVERY
    boards RevB and can be easily tailored to any other supported device 
    and development board.


@par How to use it ? 

The QSPI external flash loader is not integrated with supported toolchains, it’s only supported with STM32
ST-Link Utility V3.7.
To load the demonstration, use STM32 ST-Link Utility to program both internal Flash and external QSPI memory.
To edit and debug the demonstration you need first to program the external QSPI memory using STLink utility
and then use your preferred toolchain to update and debug the internal flash content.

In order to program the demonstration you must do the following:
1- Open STM32 ST-Link Utility V3.7, click on "External Loader" from the bar menu then check 
   "N25Q128A_STM32F746G-DISCO" box 
2- Connect the STM32746G-DISCOVERY board to PC with USB cable through CN14
3- Use "STM32CubeDemo_STM32746G-DISCO_V1.1.0.hex" file provided under “Binary” with STM32 ST-Link Utility
   to program both internal Flash and external QSPI memory
4- copy the audio and video files provided under "Utilities/Media/" in the USB key
5- Plug a USB micro A-Male to A-Female cable on CN12 connector
-> The internal Flash and the external QSPI are now programmed and the demonstration is shown on the board.

In order to Edit and debug the program, you must do the following
- if not done, perform step 1, 2, 3, 4 and 5 described above,
- Open your preferred toolchain,
- Use the IDE to update and load the internal flash content, 
- Run the demonstration.
     
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
 
