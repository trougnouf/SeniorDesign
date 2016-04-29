/**
  ******************************************************************************
  * @file    stm32l433xx.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    26-February-2016
  * @brief   CMSIS STM32L433xx Device Peripheral Access Layer Header File.
  *
  *          This file contains:
  *           - Data structures and the address mapping for all peripherals
  *           - Peripheral's registers declarations and bits definition
  *           - Macros to access peripheral�s registers hardware
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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

/** @addtogroup CMSIS_Device
  * @{
  */

/** @addtogroup stm32l433xx
  * @{
  */

#ifndef __STM32L433xx_H
#define __STM32L433xx_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */

/**
  * @brief Configuration of the Cortex-M4 Processor and Core Peripherals
   */
#define __CM4_REV                 0x0001  /*!< Cortex-M4 revision r0p1                       */
#define __MPU_PRESENT             1       /*!< STM32L4XX provides an MPU                     */
#define __NVIC_PRIO_BITS          4       /*!< STM32L4XX uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0       /*!< Set to 1 if different SysTick Config is used  */
#define __FPU_PRESENT             1       /*!< FPU present                                   */

/**
  * @}
  */

/** @addtogroup Peripheral_interrupt_number_definition
  * @{
  */

/**
 * @brief STM32L4XX Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */
typedef enum
{
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Cortex-M4 Non Maskable Interrupt                                */
  HardFault_IRQn              = -13,    /*!< 3 Cortex-M4 Hard Fault Interrupt                                  */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_PVM_IRQn                = 1,      /*!< PVD/PVM1/PVM2/PVM3/PVM4 through EXTI Line detection Interrupts    */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Channel1_IRQn          = 11,     /*!< DMA1 Channel 1 global Interrupt                                   */
  DMA1_Channel2_IRQn          = 12,     /*!< DMA1 Channel 2 global Interrupt                                   */
  DMA1_Channel3_IRQn          = 13,     /*!< DMA1 Channel 3 global Interrupt                                   */
  DMA1_Channel4_IRQn          = 14,     /*!< DMA1 Channel 4 global Interrupt                                   */
  DMA1_Channel5_IRQn          = 15,     /*!< DMA1 Channel 5 global Interrupt                                   */
  DMA1_Channel6_IRQn          = 16,     /*!< DMA1 Channel 6 global Interrupt                                   */
  DMA1_Channel7_IRQn          = 17,     /*!< DMA1 Channel 7 global Interrupt                                   */
  ADC1_IRQn                   = 18,     /*!< ADC1 global Interrupt                                             */
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM15_IRQn         = 24,     /*!< TIM1 Break interrupt and TIM15 global interrupt                   */
  TIM1_UP_TIM16_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM16 global interrupt                  */
  TIM1_TRG_COM_IRQn           = 26,     /*!< TIM1 Trigger and Commutation Interrupt                            */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  SDMMC1_IRQn                 = 49,     /*!< SDMMC1 global Interrupt                                           */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                                   */
  DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                                   */
  DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                                   */
  DMA2_Channel4_IRQn          = 59,     /*!< DMA2 Channel 4 global Interrupt                                   */
  DMA2_Channel5_IRQn          = 60,     /*!< DMA2 Channel 5 global Interrupt                                   */
  COMP_IRQn                   = 64,     /*!< COMP1 and COMP2 Interrupts                                        */
  LPTIM1_IRQn                 = 65,     /*!< LP TIM1 interrupt                                                 */
  LPTIM2_IRQn                 = 66,     /*!< LP TIM2 interrupt                                                 */
  USB_IRQn                    = 67,     /*!< USB event Interrupt                                               */
  DMA2_Channel6_IRQn          = 68,     /*!< DMA2 Channel 6 global interrupt                                   */
  DMA2_Channel7_IRQn          = 69,     /*!< DMA2 Channel 7 global interrupt                                   */
  LPUART1_IRQn                = 70,     /*!< LP UART1 interrupt                                                */
  QUADSPI_IRQn                = 71,     /*!< Quad SPI global interrupt                                         */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  SAI1_IRQn                   = 74,     /*!< Serial Audio Interface 1 global interrupt                         */
  SWPMI1_IRQn                 = 76,     /*!< Serial Wire Interface 1 global interrupt                          */
  TSC_IRQn                    = 77,     /*!< Touch Sense Controller global interrupt                           */
  LCD_IRQn                    = 78,     /*!< LCD global interrupt                                              */
  RNG_IRQn                    = 80,     /*!< RNG global interrupt                                              */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  CRS_IRQn                    = 82      /*!< CRS global interrupt                                              */
} IRQn_Type;

/**
  * @}
  */

#include "core_cm4.h"             /* Cortex-M4 processor and core peripherals */
#include "system_stm32l4xx.h"
#include <stdint.h>

/** @addtogroup Peripheral_registers_structures
  * @{
  */

/**
  * @brief Analog to Digital Converter
  */

typedef struct
{
  __IO uint32_t ISR;          /*!< ADC interrupt and status register,             Address offset: 0x00 */
  __IO uint32_t IER;          /*!< ADC interrupt enable register,                 Address offset: 0x04 */
  __IO uint32_t CR;           /*!< ADC control register,                          Address offset: 0x08 */
  __IO uint32_t CFGR;         /*!< ADC configuration register 1,                  Address offset: 0x0C */
  __IO uint32_t CFGR2;        /*!< ADC configuration register 2,                  Address offset: 0x10 */
  __IO uint32_t SMPR1;        /*!< ADC sampling time register 1,                  Address offset: 0x14 */
  __IO uint32_t SMPR2;        /*!< ADC sampling time register 2,                  Address offset: 0x18 */
       uint32_t RESERVED1;    /*!< Reserved,                                                      0x1C */
  __IO uint32_t TR1;          /*!< ADC analog watchdog 1 threshold register,      Address offset: 0x20 */
  __IO uint32_t TR2;          /*!< ADC analog watchdog 2 threshold register,      Address offset: 0x24 */
  __IO uint32_t TR3;          /*!< ADC analog watchdog 3 threshold register,      Address offset: 0x28 */
       uint32_t RESERVED2;    /*!< Reserved,                                                      0x2C */
  __IO uint32_t SQR1;         /*!< ADC group regular sequencer register 1,        Address offset: 0x30 */
  __IO uint32_t SQR2;         /*!< ADC group regular sequencer register 2,        Address offset: 0x34 */
  __IO uint32_t SQR3;         /*!< ADC group regular sequencer register 3,        Address offset: 0x38 */
  __IO uint32_t SQR4;         /*!< ADC group regular sequencer register 4,        Address offset: 0x3C */
  __IO uint32_t DR;           /*!< ADC group regular data register,               Address offset: 0x40 */
       uint32_t RESERVED3;    /*!< Reserved,                                                      0x44 */
       uint32_t RESERVED4;    /*!< Reserved,                                                      0x48 */
  __IO uint32_t JSQR;         /*!< ADC group injected sequencer register,         Address offset: 0x4C */
       uint32_t RESERVED5[4]; /*!< Reserved,                                               0x50 - 0x5C */
  __IO uint32_t OFR1;         /*!< ADC offset register 1,                         Address offset: 0x60 */
  __IO uint32_t OFR2;         /*!< ADC offset register 2,                         Address offset: 0x64 */
  __IO uint32_t OFR3;         /*!< ADC offset register 3,                         Address offset: 0x68 */
  __IO uint32_t OFR4;         /*!< ADC offset register 4,                         Address offset: 0x6C */
       uint32_t RESERVED6[4]; /*!< Reserved,                                               0x70 - 0x7C */
  __IO uint32_t JDR1;         /*!< ADC group injected rank 1 data register,       Address offset: 0x80 */
  __IO uint32_t JDR2;         /*!< ADC group injected rank 2 data register,       Address offset: 0x84 */
  __IO uint32_t JDR3;         /*!< ADC group injected rank 3 data register,       Address offset: 0x88 */
  __IO uint32_t JDR4;         /*!< ADC group injected rank 4 data register,       Address offset: 0x8C */
       uint32_t RESERVED7[4]; /*!< Reserved,                                             0x090 - 0x09C */
  __IO uint32_t AWD2CR;       /*!< ADC analog watchdog 1 configuration register,  Address offset: 0xA0 */
  __IO uint32_t AWD3CR;       /*!< ADC analog watchdog 3 Configuration Register,  Address offset: 0xA4 */
       uint32_t RESERVED8;    /*!< Reserved,                                                     0x0A8 */
       uint32_t RESERVED9;    /*!< Reserved,                                                     0x0AC */
  __IO uint32_t DIFSEL;       /*!< ADC differential mode selection register,      Address offset: 0xB0 */
  __IO uint32_t CALFACT;      /*!< ADC calibration factors,                       Address offset: 0xB4 */

} ADC_TypeDef;

typedef struct
{
  uint32_t      RESERVED1;    /*!< Reserved,                                      Address offset: ADC1 base address + 0x300 */
  uint32_t      RESERVED2;    /*!< Reserved,                                      Address offset: ADC1 base address + 0x304 */
  __IO uint32_t CCR;          /*!< ADC common configuration register,             Address offset: ADC1 base address + 0x308 */
  uint32_t      RESERVED3;    /*!< Reserved,                                      Address offset: ADC1 base address + 0x30C */
} ADC_Common_TypeDef;


/**
  * @brief Controller Area Network TxMailBox
  */

typedef struct
{
  __IO uint32_t TIR;  /*!< CAN TX mailbox identifier register */
  __IO uint32_t TDTR; /*!< CAN mailbox data length control and time stamp register */
  __IO uint32_t TDLR; /*!< CAN mailbox data low register */
  __IO uint32_t TDHR; /*!< CAN mailbox data high register */
} CAN_TxMailBox_TypeDef;

/**
  * @brief Controller Area Network FIFOMailBox
  */

typedef struct
{
  __IO uint32_t RIR;  /*!< CAN receive FIFO mailbox identifier register */
  __IO uint32_t RDTR; /*!< CAN receive FIFO mailbox data length control and time stamp register */
  __IO uint32_t RDLR; /*!< CAN receive FIFO mailbox data low register */
  __IO uint32_t RDHR; /*!< CAN receive FIFO mailbox data high register */
} CAN_FIFOMailBox_TypeDef;

/**
  * @brief Controller Area Network FilterRegister
  */

typedef struct
{
  __IO uint32_t FR1; /*!< CAN Filter bank register 1 */
  __IO uint32_t FR2; /*!< CAN Filter bank register 1 */
} CAN_FilterRegister_TypeDef;

/**
  * @brief Controller Area Network
  */

typedef struct
{
  __IO uint32_t              MCR;                 /*!< CAN master control register,         Address offset: 0x00          */
  __IO uint32_t              MSR;                 /*!< CAN master status register,          Address offset: 0x04          */
  __IO uint32_t              TSR;                 /*!< CAN transmit status register,        Address offset: 0x08          */
  __IO uint32_t              RF0R;                /*!< CAN receive FIFO 0 register,         Address offset: 0x0C          */
  __IO uint32_t              RF1R;                /*!< CAN receive FIFO 1 register,         Address offset: 0x10          */
  __IO uint32_t              IER;                 /*!< CAN interrupt enable register,       Address offset: 0x14          */
  __IO uint32_t              ESR;                 /*!< CAN error status register,           Address offset: 0x18          */
  __IO uint32_t              BTR;                 /*!< CAN bit timing register,             Address offset: 0x1C          */
  uint32_t                   RESERVED0[88];       /*!< Reserved, 0x020 - 0x17F                                            */
  CAN_TxMailBox_TypeDef      sTxMailBox[3];       /*!< CAN Tx MailBox,                      Address offset: 0x180 - 0x1AC */
  CAN_FIFOMailBox_TypeDef    sFIFOMailBox[2];     /*!< CAN FIFO MailBox,                    Address offset: 0x1B0 - 0x1CC */
  uint32_t                   RESERVED1[12];       /*!< Reserved, 0x1D0 - 0x1FF                                            */
  __IO uint32_t              FMR;                 /*!< CAN filter master register,          Address offset: 0x200         */
  __IO uint32_t              FM1R;                /*!< CAN filter mode register,            Address offset: 0x204         */
  uint32_t                   RESERVED2;           /*!< Reserved, 0x208                                                    */
  __IO uint32_t              FS1R;                /*!< CAN filter scale register,           Address offset: 0x20C         */
  uint32_t                   RESERVED3;           /*!< Reserved, 0x210                                                    */
  __IO uint32_t              FFA1R;               /*!< CAN filter FIFO assignment register, Address offset: 0x214         */
  uint32_t                   RESERVED4;           /*!< Reserved, 0x218                                                    */
  __IO uint32_t              FA1R;                /*!< CAN filter activation register,      Address offset: 0x21C         */
  uint32_t                   RESERVED5[8];        /*!< Reserved, 0x220-0x23F                                              */
  CAN_FilterRegister_TypeDef sFilterRegister[28]; /*!< CAN Filter Register,                 Address offset: 0x240-0x31C   */
} CAN_TypeDef;


/**
  * @brief Comparator
  */

typedef struct
{
  __IO uint32_t CSR;         /*!< COMP control and status register, Address offset: 0x00 */
} COMP_TypeDef;

typedef struct
{
  __IO uint32_t CSR;         /*!< COMP control and status register, used for bits common to several COMP instances, Address offset: 0x00 */
} COMP_Common_TypeDef;

/**
  * @brief CRC calculation unit
  */

typedef struct
{
  __IO uint32_t DR;          /*!< CRC Data register,                           Address offset: 0x00 */
  __IO uint8_t  IDR;         /*!< CRC Independent data register,               Address offset: 0x04 */
  uint8_t       RESERVED0;   /*!< Reserved,                                                    0x05 */
  uint16_t      RESERVED1;   /*!< Reserved,                                                    0x06 */
  __IO uint32_t CR;          /*!< CRC Control register,                        Address offset: 0x08 */
  uint32_t      RESERVED2;   /*!< Reserved,                                                    0x0C */
  __IO uint32_t INIT;        /*!< Initial CRC value register,                  Address offset: 0x10 */
  __IO uint32_t POL;         /*!< CRC polynomial register,                     Address offset: 0x14 */
} CRC_TypeDef;

/**
  * @brief Clock Recovery System
  */
typedef struct 
{
__IO uint32_t CR;            /*!< CRS ccontrol register,              Address offset: 0x00 */
__IO uint32_t CFGR;          /*!< CRS configuration register,         Address offset: 0x04 */
__IO uint32_t ISR;           /*!< CRS interrupt and status register,  Address offset: 0x08 */
__IO uint32_t ICR;           /*!< CRS interrupt flag clear register,  Address offset: 0x0C */
} CRS_TypeDef;

/**
  * @brief Digital to Analog Converter
  */

typedef struct
{
  __IO uint32_t CR;          /*!< DAC control register,                                    Address offset: 0x00 */
  __IO uint32_t SWTRIGR;     /*!< DAC software trigger register,                           Address offset: 0x04 */
  __IO uint32_t DHR12R1;     /*!< DAC channel1 12-bit right-aligned data holding register, Address offset: 0x08 */
  __IO uint32_t DHR12L1;     /*!< DAC channel1 12-bit left aligned data holding register,  Address offset: 0x0C */
  __IO uint32_t DHR8R1;      /*!< DAC channel1 8-bit right aligned data holding register,  Address offset: 0x10 */
  __IO uint32_t DHR12R2;     /*!< DAC channel2 12-bit right aligned data holding register, Address offset: 0x14 */
  __IO uint32_t DHR12L2;     /*!< DAC channel2 12-bit left aligned data holding register,  Address offset: 0x18 */
  __IO uint32_t DHR8R2;      /*!< DAC channel2 8-bit right-aligned data holding register,  Address offset: 0x1C */
  __IO uint32_t DHR12RD;     /*!< Dual DAC 12-bit right-aligned data holding register,     Address offset: 0x20 */
  __IO uint32_t DHR12LD;     /*!< DUAL DAC 12-bit left aligned data holding register,      Address offset: 0x24 */
  __IO uint32_t DHR8RD;      /*!< DUAL DAC 8-bit right aligned data holding register,      Address offset: 0x28 */
  __IO uint32_t DOR1;        /*!< DAC channel1 data output register,                       Address offset: 0x2C */
  __IO uint32_t DOR2;        /*!< DAC channel2 data output register,                       Address offset: 0x30 */
  __IO uint32_t SR;          /*!< DAC status register,                                     Address offset: 0x34 */
  __IO uint32_t CCR;         /*!< DAC calibration control register,                        Address offset: 0x38 */
  __IO uint32_t MCR;         /*!< DAC mode control register,                               Address offset: 0x3C */
  __IO uint32_t SHSR1;       /*!< DAC Sample and Hold sample time register 1,              Address offset: 0x40 */
  __IO uint32_t SHSR2;       /*!< DAC Sample and Hold sample time register 2,              Address offset: 0x44 */
  __IO uint32_t SHHR;        /*!< DAC Sample and Hold hold time register,                  Address offset: 0x48 */
  __IO uint32_t SHRR;        /*!< DAC Sample and Hold refresh time register,               Address offset: 0x4C */
} DAC_TypeDef;


/**
  * @brief Debug MCU
  */

typedef struct
{
  __IO uint32_t IDCODE;      /*!< MCU device ID code,                 Address offset: 0x00 */
  __IO uint32_t CR;          /*!< Debug MCU configuration register,   Address offset: 0x04 */
  __IO uint32_t APB1FZR1;    /*!< Debug MCU APB1 freeze register 1,   Address offset: 0x08 */
  __IO uint32_t APB1FZR2;    /*!< Debug MCU APB1 freeze register 2,   Address offset: 0x0C */
  __IO uint32_t APB2FZ;      /*!< Debug MCU APB2 freeze register,     Address offset: 0x10 */
} DBGMCU_TypeDef;


/**
  * @brief DMA Controller
  */

typedef struct
{
  __IO uint32_t CCR;         /*!< DMA channel x configuration register        */
  __IO uint32_t CNDTR;       /*!< DMA channel x number of data register       */
  __IO uint32_t CPAR;        /*!< DMA channel x peripheral address register   */
  __IO uint32_t CMAR;        /*!< DMA channel x memory address register       */
} DMA_Channel_TypeDef;

typedef struct
{
  __IO uint32_t ISR;         /*!< DMA interrupt status register,                 Address offset: 0x00 */
  __IO uint32_t IFCR;        /*!< DMA interrupt flag clear register,             Address offset: 0x04 */
} DMA_TypeDef;

typedef struct
{
  __IO uint32_t CSELR;       /*!< DMA channel selection register              */
} DMA_Request_TypeDef;

/* Legacy define */
#define DMA_request_TypeDef  DMA_Request_TypeDef

/**
  * @brief External Interrupt/Event Controller
  */

typedef struct
{
  __IO uint32_t IMR1;        /*!< EXTI Interrupt mask register 1,             Address offset: 0x00 */
  __IO uint32_t EMR1;        /*!< EXTI Event mask register 1,                 Address offset: 0x04 */
  __IO uint32_t RTSR1;       /*!< EXTI Rising trigger selection register 1,   Address offset: 0x08 */
  __IO uint32_t FTSR1;       /*!< EXTI Falling trigger selection register 1,  Address offset: 0x0C */
  __IO uint32_t SWIER1;      /*!< EXTI Software interrupt event register 1,   Address offset: 0x10 */
  __IO uint32_t PR1;         /*!< EXTI Pending register 1,                    Address offset: 0x14 */
  uint32_t      RESERVED1;   /*!< Reserved, 0x18                                                   */
  uint32_t      RESERVED2;   /*!< Reserved, 0x1C                                                   */
  __IO uint32_t IMR2;        /*!< EXTI Interrupt mask register 2,             Address offset: 0x20 */
  __IO uint32_t EMR2;        /*!< EXTI Event mask register 2,                 Address offset: 0x24 */
  __IO uint32_t RTSR2;       /*!< EXTI Rising trigger selection register 2,   Address offset: 0x28 */
  __IO uint32_t FTSR2;       /*!< EXTI Falling trigger selection register 2,  Address offset: 0x2C */
  __IO uint32_t SWIER2;      /*!< EXTI Software interrupt event register 2,   Address offset: 0x30 */
  __IO uint32_t PR2;         /*!< EXTI Pending register 2,                    Address offset: 0x34 */
} EXTI_TypeDef;


/**
  * @brief Firewall
  */

typedef struct
{
  __IO uint32_t CSSA;        /*!< Code Segment Start Address register,              Address offset: 0x00 */
  __IO uint32_t CSL;         /*!< Code Segment Length register,                      Address offset: 0x04 */
  __IO uint32_t NVDSSA;      /*!< NON volatile data Segment Start Address register,  Address offset: 0x08 */
  __IO uint32_t NVDSL;       /*!< NON volatile data Segment Length register,         Address offset: 0x0C */
  __IO uint32_t VDSSA ;      /*!< Volatile data Segment Start Address register,      Address offset: 0x10 */
  __IO uint32_t VDSL ;       /*!< Volatile data Segment Length register,             Address offset: 0x14 */
  uint32_t      RESERVED1;   /*!< Reserved1,                                         Address offset: 0x18 */
  uint32_t      RESERVED2;   /*!< Reserved2,                                         Address offset: 0x1C */
  __IO uint32_t CR ;         /*!< Configuration  register,                           Address offset: 0x20 */
} FIREWALL_TypeDef;


/**
  * @brief FLASH Registers
  */

typedef struct
{
  __IO uint32_t ACR;              /*!< FLASH access control register,            Address offset: 0x00 */
  __IO uint32_t PDKEYR;           /*!< FLASH power down key register,            Address offset: 0x04 */
  __IO uint32_t KEYR;             /*!< FLASH key register,                       Address offset: 0x08 */
  __IO uint32_t OPTKEYR;          /*!< FLASH option key register,                Address offset: 0x0C */
  __IO uint32_t SR;               /*!< FLASH status register,                    Address offset: 0x10 */
  __IO uint32_t CR;               /*!< FLASH control register,                   Address offset: 0x14 */
  __IO uint32_t ECCR;             /*!< FLASH ECC register,                       Address offset: 0x18 */
  __IO uint32_t RESERVED1;        /*!< Reserved1,                                Address offset: 0x1C */
  __IO uint32_t OPTR;             /*!< FLASH option register,                    Address offset: 0x20 */
  __IO uint32_t PCROP1SR;         /*!< FLASH bank1 PCROP start address register, Address offset: 0x24 */
  __IO uint32_t PCROP1ER;         /*!< FLASH bank1 PCROP end address register,   Address offset: 0x28 */
  __IO uint32_t WRP1AR;           /*!< FLASH bank1 WRP area A address register,  Address offset: 0x2C */
  __IO uint32_t WRP1BR;           /*!< FLASH bank1 WRP area B address register,  Address offset: 0x30 */
} FLASH_TypeDef;



/**
  * @brief General Purpose I/O
  */

typedef struct
{
  __IO uint32_t MODER;       /*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;      /*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR;     /*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;       /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;         /*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;         /*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint32_t BSRR;        /*!< GPIO port bit set/reset  register,     Address offset: 0x18      */
  __IO uint32_t LCKR;        /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];      /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
  __IO uint32_t BRR;         /*!< GPIO Bit Reset register,               Address offset: 0x28      */

} GPIO_TypeDef;


/**
  * @brief Inter-integrated Circuit Interface
  */

typedef struct
{
  __IO uint32_t CR1;         /*!< I2C Control register 1,            Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< I2C Control register 2,            Address offset: 0x04 */
  __IO uint32_t OAR1;        /*!< I2C Own address 1 register,        Address offset: 0x08 */
  __IO uint32_t OAR2;        /*!< I2C Own address 2 register,        Address offset: 0x0C */
  __IO uint32_t TIMINGR;     /*!< I2C Timing register,               Address offset: 0x10 */
  __IO uint32_t TIMEOUTR;    /*!< I2C Timeout register,              Address offset: 0x14 */
  __IO uint32_t ISR;         /*!< I2C Interrupt and status register, Address offset: 0x18 */
  __IO uint32_t ICR;         /*!< I2C Interrupt clear register,      Address offset: 0x1C */
  __IO uint32_t PECR;        /*!< I2C PEC register,                  Address offset: 0x20 */
  __IO uint32_t RXDR;        /*!< I2C Receive data register,         Address offset: 0x24 */
  __IO uint32_t TXDR;        /*!< I2C Transmit data register,        Address offset: 0x28 */
} I2C_TypeDef;

/**
  * @brief Independent WATCHDOG
  */

typedef struct
{
  __IO uint32_t KR;          /*!< IWDG Key register,       Address offset: 0x00 */
  __IO uint32_t PR;          /*!< IWDG Prescaler register, Address offset: 0x04 */
  __IO uint32_t RLR;         /*!< IWDG Reload register,    Address offset: 0x08 */
  __IO uint32_t SR;          /*!< IWDG Status register,    Address offset: 0x0C */
  __IO uint32_t WINR;        /*!< IWDG Window register,    Address offset: 0x10 */
} IWDG_TypeDef;

/**
  * @brief LCD
  */

typedef struct
{
  __IO uint32_t CR;          /*!< LCD control register,              Address offset: 0x00 */
  __IO uint32_t FCR;         /*!< LCD frame control register,        Address offset: 0x04 */
  __IO uint32_t SR;          /*!< LCD status register,               Address offset: 0x08 */
  __IO uint32_t CLR;         /*!< LCD clear register,                Address offset: 0x0C */
  uint32_t RESERVED;         /*!< Reserved,                          Address offset: 0x10 */
  __IO uint32_t RAM[16];     /*!< LCD display memory,           Address offset: 0x14-0x50 */
} LCD_TypeDef;

/**
  * @brief LPTIMER
  */
typedef struct
{
  __IO uint32_t ISR;         /*!< LPTIM Interrupt and Status register,                Address offset: 0x00 */
  __IO uint32_t ICR;         /*!< LPTIM Interrupt Clear register,                     Address offset: 0x04 */
  __IO uint32_t IER;         /*!< LPTIM Interrupt Enable register,                    Address offset: 0x08 */
  __IO uint32_t CFGR;        /*!< LPTIM Configuration register,                       Address offset: 0x0C */
  __IO uint32_t CR;          /*!< LPTIM Control register,                             Address offset: 0x10 */
  __IO uint32_t CMP;         /*!< LPTIM Compare register,                             Address offset: 0x14 */
  __IO uint32_t ARR;         /*!< LPTIM Autoreload register,                          Address offset: 0x18 */
  __IO uint32_t CNT;         /*!< LPTIM Counter register,                             Address offset: 0x1C */
  __IO uint32_t OR;          /*!< LPTIM Option register,                              Address offset: 0x20 */
} LPTIM_TypeDef;


/**
  * @brief Operational Amplifier (OPAMP)
  */

typedef struct
{
  __IO uint32_t CSR;         /*!< OPAMP control/status register,                     Address offset: 0x00 */
  __IO uint32_t OTR;         /*!< OPAMP offset trimming register for normal mode,    Address offset: 0x04 */
  __IO uint32_t LPOTR;       /*!< OPAMP offset trimming register for low power mode, Address offset: 0x08 */
} OPAMP_TypeDef;

typedef struct
{
  __IO uint32_t CSR;         /*!< OPAMP control/status register, used for bits common to several OPAMP instances, Address offset: 0x00 */
} OPAMP_Common_TypeDef;

/**
  * @brief Power Control
  */

typedef struct
{
  __IO uint32_t CR1;   /*!< PWR power control register 1,        Address offset: 0x00 */
  __IO uint32_t CR2;   /*!< PWR power control register 2,        Address offset: 0x04 */
  __IO uint32_t CR3;   /*!< PWR power control register 3,        Address offset: 0x08 */
  __IO uint32_t CR4;   /*!< PWR power control register 4,        Address offset: 0x0C */
  __IO uint32_t SR1;   /*!< PWR power status register 1,         Address offset: 0x10 */
  __IO uint32_t SR2;   /*!< PWR power status register 2,         Address offset: 0x14 */
  __IO uint32_t SCR;   /*!< PWR power status reset register,     Address offset: 0x18 */
  uint32_t RESERVED;   /*!< Reserved,                            Address offset: 0x1C */
  __IO uint32_t PUCRA; /*!< Pull_up control register of portA,   Address offset: 0x20 */
  __IO uint32_t PDCRA; /*!< Pull_Down control register of portA, Address offset: 0x24 */
  __IO uint32_t PUCRB; /*!< Pull_up control register of portB,   Address offset: 0x28 */
  __IO uint32_t PDCRB; /*!< Pull_Down control register of portB, Address offset: 0x2C */
  __IO uint32_t PUCRC; /*!< Pull_up control register of portC,   Address offset: 0x30 */
  __IO uint32_t PDCRC; /*!< Pull_Down control register of portC, Address offset: 0x34 */
  __IO uint32_t PUCRD; /*!< Pull_up control register of portD,   Address offset: 0x38 */
  __IO uint32_t PDCRD; /*!< Pull_Down control register of portD, Address offset: 0x3C */
  __IO uint32_t PUCRE; /*!< Pull_up control register of portE,   Address offset: 0x40 */
  __IO uint32_t PDCRE; /*!< Pull_Down control register of portE, Address offset: 0x44 */
  uint32_t RESERVED1;  /*!< Reserved,                            Address offset: 0x48 */
  uint32_t RESERVED2;  /*!< Reserved,                            Address offset: 0x4C */
  uint32_t RESERVED3;  /*!< Reserved,                            Address offset: 0x50 */
  uint32_t RESERVED4;  /*!< Reserved,                            Address offset: 0x54 */ 
  __IO uint32_t PUCRH; /*!< Pull_up control register of portH,   Address offset: 0x58 */
  __IO uint32_t PDCRH; /*!< Pull_Down control register of portH, Address offset: 0x5C */
} PWR_TypeDef;


/**
  * @brief QUAD Serial Peripheral Interface
  */

typedef struct
{
  __IO uint32_t CR;          /*!< QUADSPI Control register,                           Address offset: 0x00 */
  __IO uint32_t DCR;         /*!< QUADSPI Device Configuration register,              Address offset: 0x04 */
  __IO uint32_t SR;          /*!< QUADSPI Status register,                            Address offset: 0x08 */
  __IO uint32_t FCR;         /*!< QUADSPI Flag Clear register,                        Address offset: 0x0C */
  __IO uint32_t DLR;         /*!< QUADSPI Data Length register,                       Address offset: 0x10 */
  __IO uint32_t CCR;         /*!< QUADSPI Communication Configuration register,       Address offset: 0x14 */
  __IO uint32_t AR;          /*!< QUADSPI Address register,                           Address offset: 0x18 */
  __IO uint32_t ABR;         /*!< QUADSPI Alternate Bytes register,                   Address offset: 0x1C */
  __IO uint32_t DR;          /*!< QUADSPI Data register,                              Address offset: 0x20 */
  __IO uint32_t PSMKR;       /*!< QUADSPI Polling Status Mask register,               Address offset: 0x24 */
  __IO uint32_t PSMAR;       /*!< QUADSPI Polling Status Match register,              Address offset: 0x28 */
  __IO uint32_t PIR;         /*!< QUADSPI Polling Interval register,                  Address offset: 0x2C */
  __IO uint32_t LPTR;        /*!< QUADSPI Low Power Timeout register,                 Address offset: 0x30 */
} QUADSPI_TypeDef;


/**
  * @brief Reset and Clock Control
  */

typedef struct
{
  __IO uint32_t CR;          /*!< RCC clock control register,                                              Address offset: 0x00 */
  __IO uint32_t ICSCR;       /*!< RCC internal clock sources calibration register,                         Address offset: 0x04 */
  __IO uint32_t CFGR;        /*!< RCC clock configuration register,                                        Address offset: 0x08 */
  __IO uint32_t PLLCFGR;     /*!< RCC system PLL configuration register,                                   Address offset: 0x0C */
  __IO uint32_t PLLSAI1CFGR; /*!< RCC PLL SAI1 configuration register,                                     Address offset: 0x10 */
  uint32_t      RESERVED;    /*!< Reserved,                                                                Address offset: 0x14 */
  __IO uint32_t CIER;        /*!< RCC clock interrupt enable register,                                     Address offset: 0x18 */
  __IO uint32_t CIFR;        /*!< RCC clock interrupt flag register,                                       Address offset: 0x1C */
  __IO uint32_t CICR;        /*!< RCC clock interrupt clear register,                                      Address offset: 0x20 */
  uint32_t      RESERVED0;   /*!< Reserved,                                                                Address offset: 0x24 */
  __IO uint32_t AHB1RSTR;    /*!< RCC AHB1 peripheral reset register,                                      Address offset: 0x28 */
  __IO uint32_t AHB2RSTR;    /*!< RCC AHB2 peripheral reset register,                                      Address offset: 0x2C */
  __IO uint32_t AHB3RSTR;    /*!< RCC AHB3 peripheral reset register,                                      Address offset: 0x30 */
  uint32_t      RESERVED1;   /*!< Reserved,                                                                Address offset: 0x34 */
  __IO uint32_t APB1RSTR1;   /*!< RCC APB1 peripheral reset register 1,                                    Address offset: 0x38 */
  __IO uint32_t APB1RSTR2;   /*!< RCC APB1 peripheral reset register 2,                                    Address offset: 0x3C */
  __IO uint32_t APB2RSTR;    /*!< RCC APB2 peripheral reset register,                                      Address offset: 0x40 */
  uint32_t      RESERVED2;   /*!< Reserved,                                                                Address offset: 0x44 */
  __IO uint32_t AHB1ENR;     /*!< RCC AHB1 peripheral clocks enable register,                              Address offset: 0x48 */
  __IO uint32_t AHB2ENR;     /*!< RCC AHB2 peripheral clocks enable register,                              Address offset: 0x4C */
  __IO uint32_t AHB3ENR;     /*!< RCC AHB3 peripheral clocks enable register,                              Address offset: 0x50 */
  uint32_t      RESERVED3;   /*!< Reserved,                                                                Address offset: 0x54 */
  __IO uint32_t APB1ENR1;    /*!< RCC APB1 peripheral clocks enable register 1,                            Address offset: 0x58 */
  __IO uint32_t APB1ENR2;    /*!< RCC APB1 peripheral clocks enable register 2,                            Address offset: 0x5C */
  __IO uint32_t APB2ENR;     /*!< RCC APB2 peripheral clocks enable register,                              Address offset: 0x60 */
  uint32_t      RESERVED4;   /*!< Reserved,                                                                Address offset: 0x64 */
  __IO uint32_t AHB1SMENR;   /*!< RCC AHB1 peripheral clocks enable in sleep and stop modes register,      Address offset: 0x68 */
  __IO uint32_t AHB2SMENR;   /*!< RCC AHB2 peripheral clocks enable in sleep and stop modes register,      Address offset: 0x6C */
  __IO uint32_t AHB3SMENR;   /*!< RCC AHB3 peripheral clocks enable in sleep and stop modes register,      Address offset: 0x70 */
  uint32_t      RESERVED5;   /*!< Reserved,                                                                Address offset: 0x74 */
  __IO uint32_t APB1SMENR1;  /*!< RCC APB1 peripheral clocks enable in sleep mode and stop modes register 1, Address offset: 0x78 */
  __IO uint32_t APB1SMENR2;  /*!< RCC APB1 peripheral clocks enable in sleep mode and stop modes register 2, Address offset: 0x7C */
  __IO uint32_t APB2SMENR;   /*!< RCC APB2 peripheral clocks enable in sleep mode and stop modes register, Address offset: 0x80 */
  uint32_t      RESERVED6;   /*!< Reserved,                                                                Address offset: 0x84 */
  __IO uint32_t CCIPR;       /*!< RCC peripherals independent clock configuration register,                Address offset: 0x88 */
  __IO uint32_t RESERVED7;   /*!< Reserved,                                                                Address offset: 0x8C */
  __IO uint32_t BDCR;        /*!< RCC backup domain control register,                                      Address offset: 0x90 */
  __IO uint32_t CSR;         /*!< RCC clock control & status register,                                     Address offset: 0x94 */
  __IO uint32_t CRRCR;       /*!< RCC clock recovery RC register,                                          Address offset: 0x98 */
} RCC_TypeDef;

/**
  * @brief Real-Time Clock
  */

typedef struct
{
  __IO uint32_t TR;          /*!< RTC time register,                                         Address offset: 0x00 */
  __IO uint32_t DR;          /*!< RTC date register,                                         Address offset: 0x04 */
  __IO uint32_t CR;          /*!< RTC control register,                                      Address offset: 0x08 */
  __IO uint32_t ISR;         /*!< RTC initialization and status register,                    Address offset: 0x0C */
  __IO uint32_t PRER;        /*!< RTC prescaler register,                                    Address offset: 0x10 */
  __IO uint32_t WUTR;        /*!< RTC wakeup timer register,                                 Address offset: 0x14 */
       uint32_t reserved;    /*!< Reserved  */
  __IO uint32_t ALRMAR;      /*!< RTC alarm A register,                                      Address offset: 0x1C */
  __IO uint32_t ALRMBR;      /*!< RTC alarm B register,                                      Address offset: 0x20 */
  __IO uint32_t WPR;         /*!< RTC write protection register,                             Address offset: 0x24 */
  __IO uint32_t SSR;         /*!< RTC sub second register,                                   Address offset: 0x28 */
  __IO uint32_t SHIFTR;      /*!< RTC shift control register,                                Address offset: 0x2C */
  __IO uint32_t TSTR;        /*!< RTC time stamp time register,                              Address offset: 0x30 */
  __IO uint32_t TSDR;        /*!< RTC time stamp date register,                              Address offset: 0x34 */
  __IO uint32_t TSSSR;       /*!< RTC time-stamp sub second register,                        Address offset: 0x38 */
  __IO uint32_t CALR;        /*!< RTC calibration register,                                  Address offset: 0x3C */
  __IO uint32_t TAMPCR;      /*!< RTC tamper configuration register,                         Address offset: 0x40 */
  __IO uint32_t ALRMASSR;    /*!< RTC alarm A sub second register,                           Address offset: 0x44 */
  __IO uint32_t ALRMBSSR;    /*!< RTC alarm B sub second register,                           Address offset: 0x48 */
  __IO uint32_t OR;          /*!< RTC option register,                                       Address offset: 0x4C */
  __IO uint32_t BKP0R;       /*!< RTC backup register 0,                                     Address offset: 0x50 */
  __IO uint32_t BKP1R;       /*!< RTC backup register 1,                                     Address offset: 0x54 */
  __IO uint32_t BKP2R;       /*!< RTC backup register 2,                                     Address offset: 0x58 */
  __IO uint32_t BKP3R;       /*!< RTC backup register 3,                                     Address offset: 0x5C */
  __IO uint32_t BKP4R;       /*!< RTC backup register 4,                                     Address offset: 0x60 */
  __IO uint32_t BKP5R;       /*!< RTC backup register 5,                                     Address offset: 0x64 */
  __IO uint32_t BKP6R;       /*!< RTC backup register 6,                                     Address offset: 0x68 */
  __IO uint32_t BKP7R;       /*!< RTC backup register 7,                                     Address offset: 0x6C */
  __IO uint32_t BKP8R;       /*!< RTC backup register 8,                                     Address offset: 0x70 */
  __IO uint32_t BKP9R;       /*!< RTC backup register 9,                                     Address offset: 0x74 */
  __IO uint32_t BKP10R;      /*!< RTC backup register 10,                                    Address offset: 0x78 */
  __IO uint32_t BKP11R;      /*!< RTC backup register 11,                                    Address offset: 0x7C */
  __IO uint32_t BKP12R;      /*!< RTC backup register 12,                                    Address offset: 0x80 */
  __IO uint32_t BKP13R;      /*!< RTC backup register 13,                                    Address offset: 0x84 */
  __IO uint32_t BKP14R;      /*!< RTC backup register 14,                                    Address offset: 0x88 */
  __IO uint32_t BKP15R;      /*!< RTC backup register 15,                                    Address offset: 0x8C */
  __IO uint32_t BKP16R;      /*!< RTC backup register 16,                                    Address offset: 0x90 */
  __IO uint32_t BKP17R;      /*!< RTC backup register 17,                                    Address offset: 0x94 */
  __IO uint32_t BKP18R;      /*!< RTC backup register 18,                                    Address offset: 0x98 */
  __IO uint32_t BKP19R;      /*!< RTC backup register 19,                                    Address offset: 0x9C */
  __IO uint32_t BKP20R;      /*!< RTC backup register 20,                                    Address offset: 0xA0 */
  __IO uint32_t BKP21R;      /*!< RTC backup register 21,                                    Address offset: 0xA4 */
  __IO uint32_t BKP22R;      /*!< RTC backup register 22,                                    Address offset: 0xA8 */
  __IO uint32_t BKP23R;      /*!< RTC backup register 23,                                    Address offset: 0xAC */
  __IO uint32_t BKP24R;      /*!< RTC backup register 24,                                    Address offset: 0xB0 */
  __IO uint32_t BKP25R;      /*!< RTC backup register 25,                                    Address offset: 0xB4 */
  __IO uint32_t BKP26R;      /*!< RTC backup register 26,                                    Address offset: 0xB8 */
  __IO uint32_t BKP27R;      /*!< RTC backup register 27,                                    Address offset: 0xBC */
  __IO uint32_t BKP28R;      /*!< RTC backup register 28,                                    Address offset: 0xC0 */
  __IO uint32_t BKP29R;      /*!< RTC backup register 29,                                    Address offset: 0xC4 */
  __IO uint32_t BKP30R;      /*!< RTC backup register 30,                                    Address offset: 0xC8 */
  __IO uint32_t BKP31R;      /*!< RTC backup register 31,                                    Address offset: 0xCC */
} RTC_TypeDef;


/**
  * @brief Serial Audio Interface
  */

typedef struct
{
  __IO uint32_t GCR;         /*!< SAI global configuration register,        Address offset: 0x00 */
} SAI_TypeDef;

typedef struct
{
  __IO uint32_t CR1;         /*!< SAI block x configuration register 1,     Address offset: 0x04 */
  __IO uint32_t CR2;         /*!< SAI block x configuration register 2,     Address offset: 0x08 */
  __IO uint32_t FRCR;        /*!< SAI block x frame configuration register, Address offset: 0x0C */
  __IO uint32_t SLOTR;       /*!< SAI block x slot register,                Address offset: 0x10 */
  __IO uint32_t IMR;         /*!< SAI block x interrupt mask register,      Address offset: 0x14 */
  __IO uint32_t SR;          /*!< SAI block x status register,              Address offset: 0x18 */
  __IO uint32_t CLRFR;       /*!< SAI block x clear flag register,          Address offset: 0x1C */
  __IO uint32_t DR;          /*!< SAI block x data register,                Address offset: 0x20 */
} SAI_Block_TypeDef;


/**
  * @brief Secure digital input/output Interface
  */

typedef struct
{
  __IO uint32_t POWER;          /*!< SDMMC power control register,    Address offset: 0x00 */
  __IO uint32_t CLKCR;          /*!< SDMMC clock control register,    Address offset: 0x04 */
  __IO uint32_t ARG;            /*!< SDMMC argument register,         Address offset: 0x08 */
  __IO uint32_t CMD;            /*!< SDMMC command register,          Address offset: 0x0C */
  __I uint32_t  RESPCMD;        /*!< SDMMC command response register, Address offset: 0x10 */
  __I uint32_t  RESP1;          /*!< SDMMC response 1 register,       Address offset: 0x14 */
  __I uint32_t  RESP2;          /*!< SDMMC response 2 register,       Address offset: 0x18 */
  __I uint32_t  RESP3;          /*!< SDMMC response 3 register,       Address offset: 0x1C */
  __I uint32_t  RESP4;          /*!< SDMMC response 4 register,       Address offset: 0x20 */
  __IO uint32_t DTIMER;         /*!< SDMMC data timer register,       Address offset: 0x24 */
  __IO uint32_t DLEN;           /*!< SDMMC data length register,      Address offset: 0x28 */
  __IO uint32_t DCTRL;          /*!< SDMMC data control register,     Address offset: 0x2C */
  __I uint32_t  DCOUNT;         /*!< SDMMC data counter register,     Address offset: 0x30 */
  __I uint32_t  STA;            /*!< SDMMC status register,           Address offset: 0x34 */
  __IO uint32_t ICR;            /*!< SDMMC interrupt clear register,  Address offset: 0x38 */
  __IO uint32_t MASK;           /*!< SDMMC mask register,             Address offset: 0x3C */
  uint32_t      RESERVED0[2];   /*!< Reserved, 0x40-0x44                                  */
  __I uint32_t  FIFOCNT;        /*!< SDMMC FIFO counter register,     Address offset: 0x48 */
  uint32_t      RESERVED1[13];  /*!< Reserved, 0x4C-0x7C                                  */
  __IO uint32_t FIFO;           /*!< SDMMC data FIFO register,        Address offset: 0x80 */
} SDMMC_TypeDef;


/**
  * @brief Serial Peripheral Interface
  */

typedef struct
{
  __IO uint32_t CR1;         /*!< SPI Control register 1,                              Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< SPI Control register 2,                              Address offset: 0x04 */
  __IO uint32_t SR;          /*!< SPI Status register,                                 Address offset: 0x08 */
  __IO uint32_t DR;          /*!< SPI data register,                                  Address offset: 0x0C */
  __IO uint32_t CRCPR;       /*!< SPI CRC polynomial register,                         Address offset: 0x10 */
  __IO uint32_t RXCRCR;      /*!< SPI Rx CRC register,                                 Address offset: 0x14 */
  __IO uint32_t TXCRCR;      /*!< SPI Tx CRC register,                                 Address offset: 0x18 */
  uint32_t  RESERVED1;       /*!< Reserved,                                            Address offset: 0x1C */
  uint32_t  RESERVED2;       /*!< Reserved,                                            Address offset: 0x20 */
} SPI_TypeDef;


/**
  * @brief Single Wire Protocol Master Interface SPWMI
  */

typedef struct
{
  __IO uint32_t CR;          /*!< SWPMI Configuration/Control register,     Address offset: 0x00 */
  __IO uint32_t BRR;         /*!< SWPMI bitrate register,                   Address offset: 0x04 */
    uint32_t  RESERVED1;     /*!< Reserved, 0x08                                                 */
  __IO uint32_t ISR;         /*!< SWPMI Interrupt and Status register,      Address offset: 0x0C */
  __IO uint32_t ICR;         /*!< SWPMI Interrupt Flag Clear register,      Address offset: 0x10 */
  __IO uint32_t IER;         /*!< SWPMI Interrupt Enable register,          Address offset: 0x14 */
  __IO uint32_t RFL;         /*!< SWPMI Receive Frame Length register,      Address offset: 0x18 */
  __IO uint32_t TDR;         /*!< SWPMI Transmit data register,             Address offset: 0x1C */
  __IO uint32_t RDR;         /*!< SWPMI Receive data register,              Address offset: 0x20 */
  __IO uint32_t OR;          /*!< SWPMI Option register,                    Address offset: 0x24 */
} SWPMI_TypeDef;


/**
  * @brief System configuration controller
  */

typedef struct
{
  __IO uint32_t MEMRMP;      /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
  __IO uint32_t CFGR1;       /*!< SYSCFG configuration register 1,                   Address offset: 0x04      */
  __IO uint32_t EXTICR[4];   /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
  __IO uint32_t SCSR;        /*!< SYSCFG SRAM2 control and status register,          Address offset: 0x18      */
  __IO uint32_t CFGR2;       /*!< SYSCFG configuration register 2,                   Address offset: 0x1C      */
  __IO uint32_t SWPR;        /*!< SYSCFG SRAM2 write protection register,            Address offset: 0x20      */
  __IO uint32_t SKR;         /*!< SYSCFG SRAM2 key register,                         Address offset: 0x24      */
} SYSCFG_TypeDef;


/**
  * @brief TIM
  */

typedef struct
{
  __IO uint32_t CR1;         /*!< TIM control register 1,                   Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< TIM control register 2,                   Address offset: 0x04 */
  __IO uint32_t SMCR;        /*!< TIM slave mode control register,          Address offset: 0x08 */
  __IO uint32_t DIER;        /*!< TIM DMA/interrupt enable register,        Address offset: 0x0C */
  __IO uint32_t SR;          /*!< TIM status register,                      Address offset: 0x10 */
  __IO uint32_t EGR;         /*!< TIM event generation register,            Address offset: 0x14 */
  __IO uint32_t CCMR1;       /*!< TIM capture/compare mode register 1,      Address offset: 0x18 */
  __IO uint32_t CCMR2;       /*!< TIM capture/compare mode register 2,      Address offset: 0x1C */
  __IO uint32_t CCER;        /*!< TIM capture/compare enable register,      Address offset: 0x20 */
  __IO uint32_t CNT;         /*!< TIM counter register,                     Address offset: 0x24 */
  __IO uint32_t PSC;         /*!< TIM prescaler,                            Address offset: 0x28 */
  __IO uint32_t ARR;         /*!< TIM auto-reload register,                 Address offset: 0x2C */
  __IO uint32_t RCR;         /*!< TIM repetition counter register,          Address offset: 0x30 */
  __IO uint32_t CCR1;        /*!< TIM capture/compare register 1,           Address offset: 0x34 */
  __IO uint32_t CCR2;        /*!< TIM capture/compare register 2,           Address offset: 0x38 */
  __IO uint32_t CCR3;        /*!< TIM capture/compare register 3,           Address offset: 0x3C */
  __IO uint32_t CCR4;        /*!< TIM capture/compare register 4,           Address offset: 0x40 */
  __IO uint32_t BDTR;        /*!< TIM break and dead-time register,         Address offset: 0x44 */
  __IO uint32_t DCR;         /*!< TIM DMA control register,                 Address offset: 0x48 */
  __IO uint32_t DMAR;        /*!< TIM DMA address for full transfer,        Address offset: 0x4C */
  __IO uint32_t OR1;         /*!< TIM option register 1,                    Address offset: 0x50 */
  __IO uint32_t CCMR3;       /*!< TIM capture/compare mode register 3,      Address offset: 0x54 */
  __IO uint32_t CCR5;        /*!< TIM capture/compare register5,            Address offset: 0x58 */
  __IO uint32_t CCR6;        /*!< TIM capture/compare register6,            Address offset: 0x5C */
  __IO uint32_t OR2;         /*!< TIM option register 2,                    Address offset: 0x60 */
  __IO uint32_t OR3;         /*!< TIM option register 3,                    Address offset: 0x64 */
} TIM_TypeDef;


/**
  * @brief Touch Sensing Controller (TSC)
  */

typedef struct
{
  __IO uint32_t CR;            /*!< TSC control register,                                     Address offset: 0x00 */
  __IO uint32_t IER;           /*!< TSC interrupt enable register,                            Address offset: 0x04 */
  __IO uint32_t ICR;           /*!< TSC interrupt clear register,                             Address offset: 0x08 */
  __IO uint32_t ISR;           /*!< TSC interrupt status register,                            Address offset: 0x0C */
  __IO uint32_t IOHCR;         /*!< TSC I/O hysteresis control register,                      Address offset: 0x10 */
  uint32_t      RESERVED1;     /*!< Reserved,                                                 Address offset: 0x14 */
  __IO uint32_t IOASCR;        /*!< TSC I/O analog switch control register,                   Address offset: 0x18 */
  uint32_t      RESERVED2;     /*!< Reserved,                                                 Address offset: 0x1C */
  __IO uint32_t IOSCR;         /*!< TSC I/O sampling control register,                        Address offset: 0x20 */
  uint32_t      RESERVED3;     /*!< Reserved,                                                 Address offset: 0x24 */
  __IO uint32_t IOCCR;         /*!< TSC I/O channel control register,                         Address offset: 0x28 */
  uint32_t      RESERVED4;     /*!< Reserved,                                                 Address offset: 0x2C */
  __IO uint32_t IOGCSR;        /*!< TSC I/O group control status register,                    Address offset: 0x30 */
  __IO uint32_t IOGXCR[7];     /*!< TSC I/O group x counter register,                         Address offset: 0x34-4C */
} TSC_TypeDef;

/**
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */

typedef struct
{
  __IO uint32_t CR1;         /*!< USART Control register 1,                 Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< USART Control register 2,                 Address offset: 0x04 */
  __IO uint32_t CR3;         /*!< USART Control register 3,                 Address offset: 0x08 */
  __IO uint32_t BRR;         /*!< USART Baud rate register,                 Address offset: 0x0C */
  __IO uint16_t GTPR;        /*!< USART Guard time and prescaler register,  Address offset: 0x10 */
  uint16_t  RESERVED2;       /*!< Reserved, 0x12                                                 */
  __IO uint32_t RTOR;        /*!< USART Receiver Time Out register,         Address offset: 0x14 */
  __IO uint16_t RQR;         /*!< USART Request register,                   Address offset: 0x18 */
  uint16_t  RESERVED3;       /*!< Reserved, 0x1A                                                 */
  __IO uint32_t ISR;         /*!< USART Interrupt and status register,      Address offset: 0x1C */
  __IO uint32_t ICR;         /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
  __IO uint16_t RDR;         /*!< USART Receive Data register,              Address offset: 0x24 */
  uint16_t  RESERVED4;       /*!< Reserved, 0x26                                                 */
  __IO uint16_t TDR;         /*!< USART Transmit Data register,             Address offset: 0x28 */
  uint16_t  RESERVED5;       /*!< Reserved, 0x2A                                                 */
} USART_TypeDef;

/** 
  * @brief Universal Serial Bus Full Speed Device
  */
  
typedef struct
{
  __IO uint16_t EP0R;            /*!< USB Endpoint 0 register,                Address offset: 0x00 */ 
  __IO uint16_t RESERVED0;       /*!< Reserved */     
  __IO uint16_t EP1R;            /*!< USB Endpoint 1 register,                Address offset: 0x04 */
  __IO uint16_t RESERVED1;       /*!< Reserved */       
  __IO uint16_t EP2R;            /*!< USB Endpoint 2 register,                Address offset: 0x08 */
  __IO uint16_t RESERVED2;       /*!< Reserved */       
  __IO uint16_t EP3R;            /*!< USB Endpoint 3 register,                Address offset: 0x0C */ 
  __IO uint16_t RESERVED3;       /*!< Reserved */       
  __IO uint16_t EP4R;            /*!< USB Endpoint 4 register,                Address offset: 0x10 */
  __IO uint16_t RESERVED4;       /*!< Reserved */       
  __IO uint16_t EP5R;            /*!< USB Endpoint 5 register,                Address offset: 0x14 */
  __IO uint16_t RESERVED5;       /*!< Reserved */       
  __IO uint16_t EP6R;            /*!< USB Endpoint 6 register,                Address offset: 0x18 */
  __IO uint16_t RESERVED6;       /*!< Reserved */       
  __IO uint16_t EP7R;            /*!< USB Endpoint 7 register,                Address offset: 0x1C */
  __IO uint16_t RESERVED7[17];   /*!< Reserved */     
  __IO uint16_t CNTR;            /*!< Control register,                       Address offset: 0x40 */
  __IO uint16_t RESERVED8;       /*!< Reserved */       
  __IO uint16_t ISTR;            /*!< Interrupt status register,              Address offset: 0x44 */
  __IO uint16_t RESERVED9;       /*!< Reserved */       
  __IO uint16_t FNR;             /*!< Frame number register,                  Address offset: 0x48 */
  __IO uint16_t RESERVEDA;       /*!< Reserved */       
  __IO uint16_t DADDR;           /*!< Device address register,                Address offset: 0x4C */
  __IO uint16_t RESERVEDB;       /*!< Reserved */       
  __IO uint16_t BTABLE;          /*!< Buffer Table address register,          Address offset: 0x50 */
  __IO uint16_t RESERVEDC;       /*!< Reserved */       
  __IO uint16_t LPMCSR;          /*!< LPM Control and Status register,        Address offset: 0x54 */
  __IO uint16_t RESERVEDD;       /*!< Reserved */       
  __IO uint16_t BCDR;            /*!< Battery Charging detector register,     Address offset: 0x58 */
  __IO uint16_t RESERVEDE;       /*!< Reserved */       
} USB_TypeDef;

/**
  * @brief VREFBUF
  */

typedef struct
{
  __IO uint32_t CSR;         /*!< VREFBUF control and status register,         Address offset: 0x00 */
  __IO uint32_t CCR;         /*!< VREFBUF calibration and control register,    Address offset: 0x04 */
} VREFBUF_TypeDef;

/**
  * @brief Window WATCHDOG
  */

typedef struct
{
  __IO uint32_t CR;          /*!< WWDG Control register,       Address offset: 0x00 */
  __IO uint32_t CFR;         /*!< WWDG Configuration register, Address offset: 0x04 */
  __IO uint32_t SR;          /*!< WWDG Status register,        Address offset: 0x08 */
} WWDG_TypeDef;

/**
  * @brief RNG
  */

typedef struct
{
  __IO uint32_t CR;  /*!< RNG control register, Address offset: 0x00 */
  __IO uint32_t SR;  /*!< RNG status register,  Address offset: 0x04 */
  __IO uint32_t DR;  /*!< RNG data register,    Address offset: 0x08 */
} RNG_TypeDef;

/**
  * @}
  */

/** @addtogroup Peripheral_memory_map
  * @{
  */
#define FLASH_BASE            ((uint32_t)0x08000000U) /*!< FLASH(up to 1 MB) base address */
#define SRAM1_BASE            ((uint32_t)0x20000000U) /*!< SRAM1(up to 48 KB) base address*/
#define PERIPH_BASE           ((uint32_t)0x40000000U) /*!< Peripheral base address */
#define SRAM2_BASE            ((uint32_t)0x10000000U) /*!< SRAM2(16 KB) base address*/
#define QSPI_R_BASE           ((uint32_t)0xA0001000U) /*!< QUADSPI control registers base address */
#define SRAM1_BB_BASE         ((uint32_t)0x22000000U) /*!< SRAM1(96 KB) base address in the bit-band region */
#define PERIPH_BB_BASE        ((uint32_t)0x42000000U) /*!< Peripheral base address in the bit-band region */
#define SRAM2_BB_BASE         ((uint32_t)0x12000000U) /*!< SRAM2(32 KB) base address in the bit-band region */

/* Legacy defines */
#define SRAM_BASE             SRAM1_BASE
#define SRAM_BB_BASE          SRAM1_BB_BASE

#define SRAM1_SIZE_MAX        ((uint32_t)0x0000C000U) /*!< maximum SRAM1 size (up to 48 KBytes) */
#define SRAM2_SIZE            ((uint32_t)0x00004000U) /*!< SRAM2 size (16 KBytes) */

/*!< Peripheral memory map */
#define APB1PERIPH_BASE        PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000U)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000U)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x08000000U)


/*!< APB1 peripherals */
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000U)
#define TIM6_BASE             (APB1PERIPH_BASE + 0x1000U)
#define TIM7_BASE             (APB1PERIPH_BASE + 0x1400U)
#define LCD_BASE              (APB1PERIPH_BASE + 0x2400U)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800U)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00U)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000U)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800U)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00U)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400U)
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800U)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400U)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800U)
#define I2C3_BASE             (APB1PERIPH_BASE + 0x5C00U)
#define CRS_BASE              (APB1PERIPH_BASE + 0x6000U)
#define CAN1_BASE             (APB1PERIPH_BASE + 0x6400U)
#define USB_BASE              (APB1PERIPH_BASE + 0x6800U)  /*!< USB_IP Peripheral Registers base address */
#define USB_PMAADDR           (APB1PERIPH_BASE + 0x6C00U)  /*!< USB_IP Packet Memory Area base address */
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000U)
#define DAC_BASE              (APB1PERIPH_BASE + 0x7400U)
#define DAC1_BASE             (APB1PERIPH_BASE + 0x7400U)
#define OPAMP_BASE            (APB1PERIPH_BASE + 0x7800U)
#define OPAMP1_BASE           (APB1PERIPH_BASE + 0x7800U)
#define LPTIM1_BASE           (APB1PERIPH_BASE + 0x7C00U)
#define LPUART1_BASE          (APB1PERIPH_BASE + 0x8000U)
#define SWPMI1_BASE           (APB1PERIPH_BASE + 0x8800U)
#define LPTIM2_BASE           (APB1PERIPH_BASE + 0x9400U)


/*!< APB2 peripherals */
#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x0000U)
#define VREFBUF_BASE          (APB2PERIPH_BASE + 0x0030U)
#define COMP1_BASE            (APB2PERIPH_BASE + 0x0200U)
#define COMP2_BASE            (APB2PERIPH_BASE + 0x0204U)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x0400U)
#define FIREWALL_BASE         (APB2PERIPH_BASE + 0x1C00U)
#define SDMMC1_BASE           (APB2PERIPH_BASE + 0x2800U)
#define TIM1_BASE             (APB2PERIPH_BASE + 0x2C00U)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000U)
#define USART1_BASE           (APB2PERIPH_BASE + 0x3800U)
#define TIM15_BASE            (APB2PERIPH_BASE + 0x4000U)
#define TIM16_BASE            (APB2PERIPH_BASE + 0x4400U)
#define SAI1_BASE             (APB2PERIPH_BASE + 0x5400U)
#define SAI1_Block_A_BASE     (SAI1_BASE + 0x004)
#define SAI1_Block_B_BASE     (SAI1_BASE + 0x024)

/*!< AHB1 peripherals */
#define DMA1_BASE             (AHB1PERIPH_BASE)
#define DMA2_BASE             (AHB1PERIPH_BASE + 0x0400U)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x1000U)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x2000U)
#define CRC_BASE              (AHB1PERIPH_BASE + 0x3000U)
#define TSC_BASE              (AHB1PERIPH_BASE + 0x4000U)


#define DMA1_Channel1_BASE    (DMA1_BASE + 0x0008U)
#define DMA1_Channel2_BASE    (DMA1_BASE + 0x001CU)
#define DMA1_Channel3_BASE    (DMA1_BASE + 0x0030U)
#define DMA1_Channel4_BASE    (DMA1_BASE + 0x0044U)
#define DMA1_Channel5_BASE    (DMA1_BASE + 0x0058U)
#define DMA1_Channel6_BASE    (DMA1_BASE + 0x006CU)
#define DMA1_Channel7_BASE    (DMA1_BASE + 0x0080U)
#define DMA1_CSELR_BASE       (DMA1_BASE + 0x00A8U)


#define DMA2_Channel1_BASE    (DMA2_BASE + 0x0008U)
#define DMA2_Channel2_BASE    (DMA2_BASE + 0x001CU)
#define DMA2_Channel3_BASE    (DMA2_BASE + 0x0030U)
#define DMA2_Channel4_BASE    (DMA2_BASE + 0x0044U)
#define DMA2_Channel5_BASE    (DMA2_BASE + 0x0058U)
#define DMA2_Channel6_BASE    (DMA2_BASE + 0x006CU)
#define DMA2_Channel7_BASE    (DMA2_BASE + 0x0080U)
#define DMA2_CSELR_BASE       (DMA2_BASE + 0x00A8U)


/*!< AHB2 peripherals */
#define GPIOA_BASE            (AHB2PERIPH_BASE + 0x0000U)
#define GPIOB_BASE            (AHB2PERIPH_BASE + 0x0400U)
#define GPIOC_BASE            (AHB2PERIPH_BASE + 0x0800U)
#define GPIOD_BASE            (AHB2PERIPH_BASE + 0x0C00U)
#define GPIOE_BASE            (AHB2PERIPH_BASE + 0x1000U)
#define GPIOH_BASE            (AHB2PERIPH_BASE + 0x1C00U)


#define ADC1_BASE             (AHB2PERIPH_BASE + 0x08040000U)
#define ADC1_COMMON_BASE      (AHB2PERIPH_BASE + 0x08040300U)


#define RNG_BASE              (AHB2PERIPH_BASE + 0x08060800U)


/* Debug MCU registers base address */
#define DBGMCU_BASE           ((uint32_t)0xE0042000U)


#define PACKAGE_BASE          ((uint32_t)0x1FFF7500U)        /*!< Package data register base address     */
#define UID_BASE              ((uint32_t)0x1FFF7590U)        /*!< Unique device ID register base address */
#define FLASHSIZE_BASE        ((uint32_t)0x1FFF75E0U)        /*!< Flash size data register base address  */
/**
  * @}
  */

/** @addtogroup Peripheral_declaration
  * @{
  */
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM6                ((TIM_TypeDef *) TIM6_BASE)
#define TIM7                ((TIM_TypeDef *) TIM7_BASE)
#define LCD                 ((LCD_TypeDef *) LCD_BASE)
#define RTC                 ((RTC_TypeDef *) RTC_BASE)
#define WWDG                ((WWDG_TypeDef *) WWDG_BASE)
#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)
#define SPI3                ((SPI_TypeDef *) SPI3_BASE)
#define USART2              ((USART_TypeDef *) USART2_BASE)
#define USART3              ((USART_TypeDef *) USART3_BASE)
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#define I2C3                ((I2C_TypeDef *) I2C3_BASE)
#define CRS                 ((CRS_TypeDef *) CRS_BASE)
#define CAN                 ((CAN_TypeDef *) CAN1_BASE)
#define CAN1                ((CAN_TypeDef *) CAN1_BASE)
#define USB                 ((USB_TypeDef *) USB_BASE)
#define PWR                 ((PWR_TypeDef *) PWR_BASE)
#define DAC                 ((DAC_TypeDef *) DAC1_BASE)
#define DAC1                ((DAC_TypeDef *) DAC1_BASE)
#define OPAMP               ((OPAMP_TypeDef *) OPAMP_BASE)
#define OPAMP1              ((OPAMP_TypeDef *) OPAMP1_BASE)
#define OPAMP1_COMMON       ((OPAMP_Common_TypeDef *) OPAMP1_BASE)
#define LPTIM1              ((LPTIM_TypeDef *) LPTIM1_BASE)
#define LPUART1             ((USART_TypeDef *) LPUART1_BASE)
#define SWPMI1              ((SWPMI_TypeDef *) SWPMI1_BASE)
#define LPTIM2              ((LPTIM_TypeDef *) LPTIM2_BASE)

#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define VREFBUF             ((VREFBUF_TypeDef *) VREFBUF_BASE)
#define COMP1               ((COMP_TypeDef *) COMP1_BASE)
#define COMP2               ((COMP_TypeDef *) COMP2_BASE)
#define COMP12_COMMON       ((COMP_Common_TypeDef *) COMP2_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define FIREWALL            ((FIREWALL_TypeDef *) FIREWALL_BASE)
#define SDMMC1              ((SDMMC_TypeDef *) SDMMC1_BASE)
#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define USART1              ((USART_TypeDef *) USART1_BASE)
#define TIM15               ((TIM_TypeDef *) TIM15_BASE)
#define TIM16               ((TIM_TypeDef *) TIM16_BASE)
#define SAI1                ((SAI_TypeDef *) SAI1_BASE)
#define SAI1_Block_A        ((SAI_Block_TypeDef *)SAI1_Block_A_BASE)
#define SAI1_Block_B        ((SAI_Block_TypeDef *)SAI1_Block_B_BASE)
#define DMA1                ((DMA_TypeDef *) DMA1_BASE)
#define DMA2                ((DMA_TypeDef *) DMA2_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
#define CRC                 ((CRC_TypeDef *) CRC_BASE)
#define TSC                 ((TSC_TypeDef *) TSC_BASE)

#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define ADC1_COMMON         ((ADC_Common_TypeDef *) ADC1_COMMON_BASE)
#define RNG                 ((RNG_TypeDef *) RNG_BASE)


#define DMA1_Channel1       ((DMA_Channel_TypeDef *) DMA1_Channel1_BASE)
#define DMA1_Channel2       ((DMA_Channel_TypeDef *) DMA1_Channel2_BASE)
#define DMA1_Channel3       ((DMA_Channel_TypeDef *) DMA1_Channel3_BASE)
#define DMA1_Channel4       ((DMA_Channel_TypeDef *) DMA1_Channel4_BASE)
#define DMA1_Channel5       ((DMA_Channel_TypeDef *) DMA1_Channel5_BASE)
#define DMA1_Channel6       ((DMA_Channel_TypeDef *) DMA1_Channel6_BASE)
#define DMA1_Channel7       ((DMA_Channel_TypeDef *) DMA1_Channel7_BASE)
#define DMA1_CSELR          ((DMA_request_TypeDef *) DMA1_CSELR_BASE)


#define DMA2_Channel1       ((DMA_Channel_TypeDef *) DMA2_Channel1_BASE)
#define DMA2_Channel2       ((DMA_Channel_TypeDef *) DMA2_Channel2_BASE)
#define DMA2_Channel3       ((DMA_Channel_TypeDef *) DMA2_Channel3_BASE)
#define DMA2_Channel4       ((DMA_Channel_TypeDef *) DMA2_Channel4_BASE)
#define DMA2_Channel5       ((DMA_Channel_TypeDef *) DMA2_Channel5_BASE)
#define DMA2_Channel6       ((DMA_Channel_TypeDef *) DMA2_Channel6_BASE)
#define DMA2_Channel7       ((DMA_Channel_TypeDef *) DMA2_Channel7_BASE)
#define DMA2_CSELR          ((DMA_request_TypeDef *) DMA2_CSELR_BASE)



#define QUADSPI             ((QUADSPI_TypeDef *) QSPI_R_BASE)

#define DBGMCU              ((DBGMCU_TypeDef *) DBGMCU_BASE)

/**
  * @}
  */

/** @addtogroup Exported_constants
  * @{
  */

/** @addtogroup Peripheral_Registers_Bits_Definition
  * @{
  */

/******************************************************************************/
/*                         Peripheral Registers_Bits_Definition               */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                        Analog to Digital Converter                         */
/*                                                                            */
/******************************************************************************/

/*
 * @brief Specific device feature definitions (not present on all devices in the STM32L4 family)
 */
/* Note: No specific macro feature on this device */

/********************  Bit definition for ADC_ISR register  *******************/
#define ADC_ISR_ADRDY           ((uint32_t)0x00000001U) /*!< ADC ready flag */
#define ADC_ISR_EOSMP           ((uint32_t)0x00000002U) /*!< ADC group regular end of sampling flag */
#define ADC_ISR_EOC             ((uint32_t)0x00000004U) /*!< ADC group regular end of unitary conversion flag */
#define ADC_ISR_EOS             ((uint32_t)0x00000008U) /*!< ADC group regular end of sequence conversions flag */
#define ADC_ISR_OVR             ((uint32_t)0x00000010U) /*!< ADC group regular overrun flag */
#define ADC_ISR_JEOC            ((uint32_t)0x00000020U) /*!< ADC group injected end of unitary conversion flag */
#define ADC_ISR_JEOS            ((uint32_t)0x00000040U) /*!< ADC group injected end of sequence conversions flag */
#define ADC_ISR_AWD1            ((uint32_t)0x00000080U) /*!< ADC analog watchdog 1 flag */
#define ADC_ISR_AWD2            ((uint32_t)0x00000100U) /*!< ADC analog watchdog 2 flag */
#define ADC_ISR_AWD3            ((uint32_t)0x00000200U) /*!< ADC analog watchdog 3 flag */
#define ADC_ISR_JQOVF           ((uint32_t)0x00000400U) /*!< ADC group injected contexts queue overflow flag */

/********************  Bit definition for ADC_IER register  *******************/
#define ADC_IER_ADRDYIE         ((uint32_t)0x00000001U) /*!< ADC ready interrupt */
#define ADC_IER_EOSMPIE         ((uint32_t)0x00000002U) /*!< ADC group regular end of sampling interrupt */
#define ADC_IER_EOCIE           ((uint32_t)0x00000004U) /*!< ADC group regular end of unitary conversion interrupt */
#define ADC_IER_EOSIE           ((uint32_t)0x00000008U) /*!< ADC group regular end of sequence conversions interrupt */
#define ADC_IER_OVRIE           ((uint32_t)0x00000010U) /*!< ADC group regular overrun interrupt */
#define ADC_IER_JEOCIE          ((uint32_t)0x00000020U) /*!< ADC group injected end of unitary conversion interrupt */
#define ADC_IER_JEOSIE          ((uint32_t)0x00000040U) /*!< ADC group injected end of sequence conversions interrupt */
#define ADC_IER_AWD1IE          ((uint32_t)0x00000080U) /*!< ADC analog watchdog 1 interrupt */
#define ADC_IER_AWD2IE          ((uint32_t)0x00000100U) /*!< ADC analog watchdog 2 interrupt */
#define ADC_IER_AWD3IE          ((uint32_t)0x00000200U) /*!< ADC analog watchdog 3 interrupt */
#define ADC_IER_JQOVFIE         ((uint32_t)0x00000400U) /*!< ADC group injected contexts queue overflow interrupt */

/* Legacy defines */
#define ADC_IER_ADRDY           (ADC_IER_ADRDYIE)
#define ADC_IER_EOSMP           (ADC_IER_EOSMPIE)
#define ADC_IER_EOC             (ADC_IER_EOCIE)
#define ADC_IER_EOS             (ADC_IER_EOSIE)
#define ADC_IER_OVR             (ADC_IER_OVRIE)
#define ADC_IER_JEOC            (ADC_IER_JEOCIE)
#define ADC_IER_JEOS            (ADC_IER_JEOSIE)
#define ADC_IER_AWD1            (ADC_IER_AWD1IE)
#define ADC_IER_AWD2            (ADC_IER_AWD2IE)
#define ADC_IER_AWD3            (ADC_IER_AWD3IE)
#define ADC_IER_JQOVF           (ADC_IER_JQOVFIE)

/********************  Bit definition for ADC_CR register  ********************/
#define ADC_CR_ADEN             ((uint32_t)0x00000001U) /*!< ADC enable */
#define ADC_CR_ADDIS            ((uint32_t)0x00000002U) /*!< ADC disable */
#define ADC_CR_ADSTART          ((uint32_t)0x00000004U) /*!< ADC group regular conversion start */
#define ADC_CR_JADSTART         ((uint32_t)0x00000008U) /*!< ADC group injected conversion start */
#define ADC_CR_ADSTP            ((uint32_t)0x00000010U) /*!< ADC group regular conversion stop */
#define ADC_CR_JADSTP           ((uint32_t)0x00000020U) /*!< ADC group injected conversion stop */
#define ADC_CR_ADVREGEN         ((uint32_t)0x10000000U) /*!< ADC voltage regulator enable */
#define ADC_CR_DEEPPWD          ((uint32_t)0x20000000U) /*!< ADC deep power down enable */
#define ADC_CR_ADCALDIF         ((uint32_t)0x40000000U) /*!< ADC differential mode for calibration */
#define ADC_CR_ADCAL            ((uint32_t)0x80000000U) /*!< ADC calibration */

/********************  Bit definition for ADC_CFGR register  ******************/
#define ADC_CFGR_DMAEN          ((uint32_t)0x00000001U) /*!< ADC DMA transfer enable */
#define ADC_CFGR_DMACFG         ((uint32_t)0x00000002U) /*!< ADC DMA transfer configuration */

#define ADC_CFGR_RES            ((uint32_t)0x00000018U) /*!< ADC data resolution */
#define ADC_CFGR_RES_0          ((uint32_t)0x00000008U) /*!< bit 0 */
#define ADC_CFGR_RES_1          ((uint32_t)0x00000010U) /*!< bit 1 */

#define ADC_CFGR_ALIGN          ((uint32_t)0x00000020U) /*!< ADC data alignement */

#define ADC_CFGR_EXTSEL         ((uint32_t)0x000003C0U) /*!< ADC group regular external trigger source */
#define ADC_CFGR_EXTSEL_0       ((uint32_t)0x00000040U) /*!< bit 0 */
#define ADC_CFGR_EXTSEL_1       ((uint32_t)0x00000080U) /*!< bit 1 */
#define ADC_CFGR_EXTSEL_2       ((uint32_t)0x00000100U) /*!< bit 2 */
#define ADC_CFGR_EXTSEL_3       ((uint32_t)0x00000200U) /*!< bit 3 */

#define ADC_CFGR_EXTEN          ((uint32_t)0x00000C00U) /*!< ADC group regular external trigger polarity */
#define ADC_CFGR_EXTEN_0        ((uint32_t)0x00000400U) /*!< bit 0 */
#define ADC_CFGR_EXTEN_1        ((uint32_t)0x00000800U) /*!< bit 1 */

#define ADC_CFGR_OVRMOD         ((uint32_t)0x00001000U) /*!< ADC group regular overrun configuration */
#define ADC_CFGR_CONT           ((uint32_t)0x00002000U) /*!< ADC group regular continuous conversion mode */
#define ADC_CFGR_AUTDLY         ((uint32_t)0x00004000U) /*!< ADC low power auto wait */

#define ADC_CFGR_DISCEN         ((uint32_t)0x00010000U) /*!< ADC group regular sequencer discontinuous mode */

#define ADC_CFGR_DISCNUM        ((uint32_t)0x000E0000U) /*!< ADC group regular sequencer discontinuous number of ranks */
#define ADC_CFGR_DISCNUM_0      ((uint32_t)0x00020000U) /*!< bit 0 */
#define ADC_CFGR_DISCNUM_1      ((uint32_t)0x00040000U) /*!< bit 1 */
#define ADC_CFGR_DISCNUM_2      ((uint32_t)0x00080000U) /*!< bit 2 */

#define ADC_CFGR_JDISCEN        ((uint32_t)0x00100000U) /*!< ADC group injected sequencer discontinuous mode */
#define ADC_CFGR_JQM            ((uint32_t)0x00200000U) /*!< ADC group injected contexts queue mode */
#define ADC_CFGR_AWD1SGL        ((uint32_t)0x00400000U) /*!< ADC analog watchdog 1 monitoring a single channel or all channels */
#define ADC_CFGR_AWD1EN         ((uint32_t)0x00800000U) /*!< ADC analog watchdog 1 enable on scope ADC group regular */
#define ADC_CFGR_JAWD1EN        ((uint32_t)0x01000000U) /*!< ADC analog watchdog 1 enable on scope ADC group injected */
#define ADC_CFGR_JAUTO          ((uint32_t)0x02000000U) /*!< ADC group injected automatic trigger mode */

#define ADC_CFGR_AWD1CH         ((uint32_t)0x7C000000U) /*!< ADC analog watchdog 1 monitored channel selection */
#define ADC_CFGR_AWD1CH_0       ((uint32_t)0x04000000U) /*!< bit 0 */
#define ADC_CFGR_AWD1CH_1       ((uint32_t)0x08000000U) /*!< bit 1  */
#define ADC_CFGR_AWD1CH_2       ((uint32_t)0x10000000U) /*!< bit 2  */
#define ADC_CFGR_AWD1CH_3       ((uint32_t)0x20000000U) /*!< bit 3  */
#define ADC_CFGR_AWD1CH_4       ((uint32_t)0x40000000U) /*!< bit 4  */

#define ADC_CFGR_JQDIS          ((uint32_t)0x80000000U) /*!< ADC group injected contexts queue disable */

/********************  Bit definition for ADC_CFGR2 register  *****************/
#define ADC_CFGR2_ROVSE         ((uint32_t)0x00000001U) /*!< ADC oversampler enable on scope ADC group regular */
#define ADC_CFGR2_JOVSE         ((uint32_t)0x00000002U) /*!< ADC oversampler enable on scope ADC group injected */

#define ADC_CFGR2_OVSR          ((uint32_t)0x0000001CU) /*!< ADC oversampling ratio */
#define ADC_CFGR2_OVSR_0        ((uint32_t)0x00000004U) /*!< bit 0 */
#define ADC_CFGR2_OVSR_1        ((uint32_t)0x00000008U) /*!< bit 1 */
#define ADC_CFGR2_OVSR_2        ((uint32_t)0x00000010U) /*!< bit 2 */

#define ADC_CFGR2_OVSS          ((uint32_t)0x000001E0U) /*!< ADC oversampling shift */
#define ADC_CFGR2_OVSS_0        ((uint32_t)0x00000020U) /*!< bit 0 */
#define ADC_CFGR2_OVSS_1        ((uint32_t)0x00000040U) /*!< bit 1 */
#define ADC_CFGR2_OVSS_2        ((uint32_t)0x00000080U) /*!< bit 2 */
#define ADC_CFGR2_OVSS_3        ((uint32_t)0x00000100U) /*!< bit 3 */

#define ADC_CFGR2_TROVS         ((uint32_t)0x00000200U) /*!< ADC oversampling discontinuous mode (triggered mode) for ADC group regular */
#define ADC_CFGR2_ROVSM         ((uint32_t)0x00000400U) /*!< ADC oversampling mode managing interlaced conversions of ADC group regular and group injected */

/********************  Bit definition for ADC_SMPR1 register  *****************/
#define ADC_SMPR1_SMP0          ((uint32_t)0x00000007U) /*!< ADC channel 0 sampling time selection  */
#define ADC_SMPR1_SMP0_0        ((uint32_t)0x00000001U) /*!< bit 0 */
#define ADC_SMPR1_SMP0_1        ((uint32_t)0x00000002U) /*!< bit 1 */
#define ADC_SMPR1_SMP0_2        ((uint32_t)0x00000004U) /*!< bit 2 */

#define ADC_SMPR1_SMP1          ((uint32_t)0x00000038U) /*!< ADC channel 1 sampling time selection  */
#define ADC_SMPR1_SMP1_0        ((uint32_t)0x00000008U) /*!< bit 0 */
#define ADC_SMPR1_SMP1_1        ((uint32_t)0x00000010U) /*!< bit 1 */
#define ADC_SMPR1_SMP1_2        ((uint32_t)0x00000020U) /*!< bit 2 */

#define ADC_SMPR1_SMP2          ((uint32_t)0x000001C0U) /*!< ADC channel 2 sampling time selection  */
#define ADC_SMPR1_SMP2_0        ((uint32_t)0x00000040U) /*!< bit 0 */
#define ADC_SMPR1_SMP2_1        ((uint32_t)0x00000080U) /*!< bit 1 */
#define ADC_SMPR1_SMP2_2        ((uint32_t)0x00000100U) /*!< bit 2 */

#define ADC_SMPR1_SMP3          ((uint32_t)0x00000E00U) /*!< ADC channel 3 sampling time selection  */
#define ADC_SMPR1_SMP3_0        ((uint32_t)0x00000200U) /*!< bit 0 */
#define ADC_SMPR1_SMP3_1        ((uint32_t)0x00000400U) /*!< bit 1 */
#define ADC_SMPR1_SMP3_2        ((uint32_t)0x00000800U) /*!< bit 2 */

#define ADC_SMPR1_SMP4          ((uint32_t)0x00007000U) /*!< ADC channel 4 sampling time selection  */
#define ADC_SMPR1_SMP4_0        ((uint32_t)0x00001000U) /*!< bit 0 */
#define ADC_SMPR1_SMP4_1        ((uint32_t)0x00002000U) /*!< bit 1 */
#define ADC_SMPR1_SMP4_2        ((uint32_t)0x00004000U) /*!< bit 2 */

#define ADC_SMPR1_SMP5          ((uint32_t)0x00038000U) /*!< ADC channel 5 sampling time selection  */
#define ADC_SMPR1_SMP5_0        ((uint32_t)0x00008000U) /*!< bit 0 */
#define ADC_SMPR1_SMP5_1        ((uint32_t)0x00010000U) /*!< bit 1 */
#define ADC_SMPR1_SMP5_2        ((uint32_t)0x00020000U) /*!< bit 2 */

#define ADC_SMPR1_SMP6          ((uint32_t)0x001C0000U) /*!< ADC channel 6 sampling time selection  */
#define ADC_SMPR1_SMP6_0        ((uint32_t)0x00040000U) /*!< bit 0 */
#define ADC_SMPR1_SMP6_1        ((uint32_t)0x00080000U) /*!< bit 1 */
#define ADC_SMPR1_SMP6_2        ((uint32_t)0x00100000U) /*!< bit 2 */

#define ADC_SMPR1_SMP7          ((uint32_t)0x00E00000U) /*!< ADC channel 7 sampling time selection  */
#define ADC_SMPR1_SMP7_0        ((uint32_t)0x00200000U) /*!< bit 0 */
#define ADC_SMPR1_SMP7_1        ((uint32_t)0x00400000U) /*!< bit 1 */
#define ADC_SMPR1_SMP7_2        ((uint32_t)0x00800000U) /*!< bit 2 */

#define ADC_SMPR1_SMP8          ((uint32_t)0x07000000U) /*!< ADC channel 8 sampling time selection  */
#define ADC_SMPR1_SMP8_0        ((uint32_t)0x01000000U) /*!< bit 0 */
#define ADC_SMPR1_SMP8_1        ((uint32_t)0x02000000U) /*!< bit 1 */
#define ADC_SMPR1_SMP8_2        ((uint32_t)0x04000000U) /*!< bit 2 */

#define ADC_SMPR1_SMP9          ((uint32_t)0x38000000U) /*!< ADC channel 9 sampling time selection  */
#define ADC_SMPR1_SMP9_0        ((uint32_t)0x08000000U) /*!< bit 0 */
#define ADC_SMPR1_SMP9_1        ((uint32_t)0x10000000U) /*!< bit 1 */
#define ADC_SMPR1_SMP9_2        ((uint32_t)0x20000000U) /*!< bit 2 */

/********************  Bit definition for ADC_SMPR2 register  *****************/
#define ADC_SMPR2_SMP10         ((uint32_t)0x00000007U) /*!< ADC channel 10 sampling time selection  */
#define ADC_SMPR2_SMP10_0       ((uint32_t)0x00000001U) /*!< bit 0 */
#define ADC_SMPR2_SMP10_1       ((uint32_t)0x00000002U) /*!< bit 1 */
#define ADC_SMPR2_SMP10_2       ((uint32_t)0x00000004U) /*!< bit 2 */

#define ADC_SMPR2_SMP11         ((uint32_t)0x00000038U) /*!< ADC channel 11 sampling time selection  */
#define ADC_SMPR2_SMP11_0       ((uint32_t)0x00000008U) /*!< bit 0 */
#define ADC_SMPR2_SMP11_1       ((uint32_t)0x00000010U) /*!< bit 1 */
#define ADC_SMPR2_SMP11_2       ((uint32_t)0x00000020U) /*!< bit 2 */

#define ADC_SMPR2_SMP12         ((uint32_t)0x000001C0U) /*!< ADC channel 12 sampling time selection  */
#define ADC_SMPR2_SMP12_0       ((uint32_t)0x00000040U) /*!< bit 0 */
#define ADC_SMPR2_SMP12_1       ((uint32_t)0x00000080U) /*!< bit 1 */
#define ADC_SMPR2_SMP12_2       ((uint32_t)0x00000100U) /*!< bit 2 */

#define ADC_SMPR2_SMP13         ((uint32_t)0x00000E00U) /*!< ADC channel 13 sampling time selection  */
#define ADC_SMPR2_SMP13_0       ((uint32_t)0x00000200U) /*!< bit 0 */
#define ADC_SMPR2_SMP13_1       ((uint32_t)0x00000400U) /*!< bit 1 */
#define ADC_SMPR2_SMP13_2       ((uint32_t)0x00000800U) /*!< bit 2 */

#define ADC_SMPR2_SMP14         ((uint32_t)0x00007000U) /*!< ADC channel 14 sampling time selection  */
#define ADC_SMPR2_SMP14_0       ((uint32_t)0x00001000U) /*!< bit 0 */
#define ADC_SMPR2_SMP14_1       ((uint32_t)0x00002000U) /*!< bit 1 */
#define ADC_SMPR2_SMP14_2       ((uint32_t)0x00004000U) /*!< bit 2 */

#define ADC_SMPR2_SMP15         ((uint32_t)0x00038000U) /*!< ADC channel 15 sampling time selection  */
#define ADC_SMPR2_SMP15_0       ((uint32_t)0x00008000U) /*!< bit 0 */
#define ADC_SMPR2_SMP15_1       ((uint32_t)0x00010000U) /*!< bit 1 */
#define ADC_SMPR2_SMP15_2       ((uint32_t)0x00020000U) /*!< bit 2 */

#define ADC_SMPR2_SMP16         ((uint32_t)0x001C0000U) /*!< ADC channel 16 sampling time selection  */
#define ADC_SMPR2_SMP16_0       ((uint32_t)0x00040000U) /*!< bit 0 */
#define ADC_SMPR2_SMP16_1       ((uint32_t)0x00080000U) /*!< bit 1 */
#define ADC_SMPR2_SMP16_2       ((uint32_t)0x00100000U) /*!< bit 2 */

#define ADC_SMPR2_SMP17         ((uint32_t)0x00E00000U) /*!< ADC channel 17 sampling time selection  */
#define ADC_SMPR2_SMP17_0       ((uint32_t)0x00200000U) /*!< bit 0 */
#define ADC_SMPR2_SMP17_1       ((uint32_t)0x00400000U) /*!< bit 1 */
#define ADC_SMPR2_SMP17_2       ((uint32_t)0x00800000U) /*!< bit 2 */

#define ADC_SMPR2_SMP18         ((uint32_t)0x07000000U) /*!< ADC channel 18 sampling time selection  */
#define ADC_SMPR2_SMP18_0       ((uint32_t)0x01000000U) /*!< bit 0 */
#define ADC_SMPR2_SMP18_1       ((uint32_t)0x02000000U) /*!< bit 1 */
#define ADC_SMPR2_SMP18_2       ((uint32_t)0x04000000U) /*!< bit 2 */

/********************  Bit definition for ADC_TR1 register  *******************/
#define ADC_TR1_LT1             ((uint32_t)0x00000FFFU) /*!< ADC analog watchdog 1 threshold low */
#define ADC_TR1_LT1_0           ((uint32_t)0x00000001U) /*!< bit 0 */
#define ADC_TR1_LT1_1           ((uint32_t)0x00000002U) /*!< bit 1 */
#define ADC_TR1_LT1_2           ((uint32_t)0x00000004U) /*!< bit 2 */
#define ADC_TR1_LT1_3           ((uint32_t)0x00000008U) /*!< bit 3 */
#define ADC_TR1_LT1_4           ((uint32_t)0x00000010U) /*!< bit 4 */
#define ADC_TR1_LT1_5           ((uint32_t)0x00000020U) /*!< bit 5 */
#define ADC_TR1_LT1_6           ((uint32_t)0x00000040U) /*!< bit 6 */
#define ADC_TR1_LT1_7           ((uint32_t)0x00000080U) /*!< bit 7 */
#define ADC_TR1_LT1_8           ((uint32_t)0x00000100U) /*!< bit 8 */
#define ADC_TR1_LT1_9           ((uint32_t)0x00000200U) /*!< bit 9 */
#define ADC_TR1_LT1_10          ((uint32_t)0x00000400U) /*!< bit 10 */
#define ADC_TR1_LT1_11          ((uint32_t)0x00000800U) /*!< bit 11 */

#define ADC_TR1_HT1             ((uint32_t)0x0FFF0000U) /*!< ADC Analog watchdog 1 threshold high */
#define ADC_TR1_HT1_0           ((uint32_t)0x00010000U) /*!< bit 0 */
#define ADC_TR1_HT1_1           ((uint32_t)0x00020000U) /*!< bit 1 */
#define ADC_TR1_HT1_2           ((uint32_t)0x00040000U) /*!< bit 2 */
#define ADC_TR1_HT1_3           ((uint32_t)0x00080000U) /*!< bit 3 */
#define ADC_TR1_HT1_4           ((uint32_t)0x00100000U) /*!< bit 4 */
#define ADC_TR1_HT1_5           ((uint32_t)0x00200000U) /*!< bit 5 */
#define ADC_TR1_HT1_6           ((uint32_t)0x00400000U) /*!< bit 6 */
#define ADC_TR1_HT1_7           ((uint32_t)0x00800000U) /*!< bit 7 */
#define ADC_TR1_HT1_8           ((uint32_t)0x01000000U) /*!< bit 8 */
#define ADC_TR1_HT1_9           ((uint32_t)0x02000000U) /*!< bit 9 */
#define ADC_TR1_HT1_10          ((uint32_t)0x04000000U) /*!< bit 10 */
#define ADC_TR1_HT1_11          ((uint32_t)0x08000000U) /*!< bit 11 */

/********************  Bit definition for ADC_TR2 register  *******************/
#define ADC_TR2_LT2             ((uint32_t)0x000000FFU) /*!< ADC analog watchdog 2 threshold low */
#define ADC_TR2_LT2_0           ((uint32_t)0x00000001U) /*!< bit 0 */
#define ADC_TR2_LT2_1           ((uint32_t)0x00000002U) /*!< bit 1 */
#define ADC_TR2_LT2_2           ((uint32_t)0x00000004U) /*!< bit 2 */
#define ADC_TR2_LT2_3           ((uint32_t)0x00000008U) /*!< bit 3 */
#define ADC_TR2_LT2_4           ((uint32_t)0x00000010U) /*!< bit 4 */
#define ADC_TR2_LT2_5           ((uint32_t)0x00000020U) /*!< bit 5 */
#define ADC_TR2_LT2_6           ((uint32_t)0x00000040U) /*!< bit 6 */
#define ADC_TR2_LT2_7           ((uint32_t)0x00000080U) /*!< bit 7 */

#define ADC_TR2_HT2             ((uint32_t)0x00FF0000U) /*!< ADC analog watchdog 2 threshold high */
#define ADC_TR2_HT2_0           ((uint32_t)0x00010000U) /*!< bit 0 */
#define ADC_TR2_HT2_1           ((uint32_t)0x00020000U) /*!< bit 1 */
#define ADC_TR2_HT2_2           ((uint32_t)0x00040000U) /*!< bit 2 */
#define ADC_TR2_HT2_3           ((uint32_t)0x00080000U) /*!< bit 3 */
#define ADC_TR2_HT2_4           ((uint32_t)0x00100000U) /*!< bit 4 */
#define ADC_TR2_HT2_5           ((uint32_t)0x00200000U) /*!< bit 5 */
#define ADC_TR2_HT2_6           ((uint32_t)0x00400000U) /*!< bit 6 */
#define ADC_TR2_HT2_7           ((uint32_t)0x00800000U) /*!< bit 7 */

/********************  Bit definition for ADC_TR3 register  *******************/
#define ADC_TR3_LT3             ((uint32_t)0x000000FFU) /*!< ADC analog watchdog 3 threshold low */
#define ADC_TR3_LT3_0           ((uint32_t)0x00000001U) /*!< bit 0 */
#define ADC_TR3_LT3_1           ((uint32_t)0x00000002U) /*!< bit 1 */
#define ADC_TR3_LT3_2           ((uint32_t)0x00000004U) /*!< bit 2 */
#define ADC_TR3_LT3_3           ((uint32_t)0x00000008U) /*!< bit 3 */
#define ADC_TR3_LT3_4           ((uint32_t)0x00000010U) /*!< bit 4 */
#define ADC_TR3_LT3_5           ((uint32_t)0x00000020U) /*!< bit 5 */
#define ADC_TR3_LT3_6           ((uint32_t)0x00000040U) /*!< bit 6 */
#define ADC_TR3_LT3_7           ((uint32_t)0x00000080U) /*!< bit 7 */

#define ADC_TR3_HT3             ((uint32_t)0x00FF0000U) /*!< ADC analog watchdog 3 threshold high */
#define ADC_TR3_HT3_0           ((uint32_t)0x00010000U) /*!< bit 0 */
#define ADC_TR3_HT3_1           ((uint32_t)0x00020000U) /*!< bit 1 */
#define ADC_TR3_HT3_2           ((uint32_t)0x00040000U) /*!< bit 2 */
#define ADC_TR3_HT3_3           ((uint32_t)0x00080000U) /*!< bit 3 */
#define ADC_TR3_HT3_4           ((uint32_t)0x00100000U) /*!< bit 4 */
#define ADC_TR3_HT3_5           ((uint32_t)0x00200000U) /*!< bit 5 */
#define ADC_TR3_HT3_6           ((uint32_t)0x00400000U) /*!< bit 6 */
#define ADC_TR3_HT3_7           ((uint32_t)0x00800000U) /*!< bit 7 */

/********************  Bit definition for ADC_SQR1 register  ******************/
#define ADC_SQR1_L              ((uint32_t)0x0000000FU) /*!< ADC group regular sequencer scan length */
#define ADC_SQR1_L_0            ((uint32_t)0x00000001U) /*!< bit 0 */
#define ADC_SQR1_L_1            ((uint32_t)0x00000002U) /*!< bit 1 */
#define ADC_SQR1_L_2            ((uint32_t)0x00000004U) /*!< bit 2 */
#define ADC_SQR1_L_3            ((uint32_t)0x00000008U) /*!< bit 3 */

#define ADC_SQR1_SQ1            ((uint32_t)0x000007C0U) /*!< ADC group regular sequencer rank 1 */
#define ADC_SQR1_SQ1_0          ((uint32_t)0x00000040U) /*!< bit 0 */
#define ADC_SQR1_SQ1_1          ((uint32_t)0x00000080U) /*!< bit 1 */
#define ADC_SQR1_SQ1_2          ((uint32_t)0x00000100U) /*!< bit 2 */
#define ADC_SQR1_SQ1_3          ((uint32_t)0x00000200U) /*!< bit 3 */
#define ADC_SQR1_SQ1_4          ((uint32_t)0x00000400U) /*!< bit 4 */

#define ADC_SQR1_SQ2            ((uint32_t)0x0001F000U) /*!< ADC group regular sequencer rank 2 */
#define ADC_SQR1_SQ2_0          ((uint32_t)0x00001000U) /*!< bit 0 */
#define ADC_SQR1_SQ2_1          ((uint32_t)0x00002000U) /*!< bit 1 */
#define ADC_SQR1_SQ2_2          ((uint32_t)0x00004000U) /*!< bit 2 */
#define ADC_SQR1_SQ2_3          ((uint32_t)0x00008000U) /*!< bit 3 */
#define ADC_SQR1_SQ2_4          ((uint32_t)0x00010000U) /*!< bit 4 */

#define ADC_SQR1_SQ3            ((uint32_t)0x007C0000U) /*!< ADC group regular sequencer rank 3 */
#define ADC_SQR1_SQ3_0          ((uint32_t)0x00040000U) /*!< bit 0 */
#define ADC_SQR1_SQ3_1          ((uint32_t)0x00080000U) /*!< bit 1 */
#define ADC_SQR1_SQ3_2          ((uint32_t)0x00100000U) /*!< bit 2 */
#define ADC_SQR1_SQ3_3          ((uint32_t)0x00200000U) /*!< bit 3 */
#define ADC_SQR1_SQ3_4          ((uint32_t)0x00400000U) /*!< bit 4 */

#define ADC_SQR1_SQ4            ((uint32_t)0x1F000000U) /*!< ADC group regular sequencer rank 4 */
#define ADC_SQR1_SQ4_0          ((uint32_t)0x01000000U) /*!< bit 0 */
#define ADC_SQR1_SQ4_1          ((uint32_t)0x02000000U) /*!< bit 1 */
#define ADC_SQR1_SQ4_2          ((uint32_t)0x04000000U) /*!< bit 2 */
#define ADC_SQR1_SQ4_3          ((uint32_t)0x08000000U) /*!< bit 3 */
#define ADC_SQR1_SQ4_4          ((uint32_t)0x10000000U) /*!< bit 4 */

/********************  Bit definition for ADC_SQR2 register  ******************/
#define ADC_SQR2_SQ5            ((uint32_t)0x0000001FU) /*!< ADC group regular sequencer rank 5 */
#define ADC_SQR2_SQ5_0          ((uint32_t)0x00000001U) /*!< bit 0 */
#define ADC_SQR2_SQ5_1          ((uint32_t)0x00000002U) /*!< bit 1 */
#define ADC_SQR2_SQ5_2          ((uint32_t)0x00000004U) /*!< bit 2 */
#define ADC_SQR2_SQ5_3          ((uint32_t)0x00000008U) /*!< bit 3 */
#define ADC_SQR2_SQ5_4          ((uint32_t)0x00000010U) /*!< bit 4 */

#define ADC_SQR2_SQ6            ((uint32_t)0x000007C0U) /*!< ADC group regular sequencer rank 6 */
#define ADC_SQR2_SQ6_0          ((uint32_t)0x00000040U) /*!< bit 0 */
#define ADC_SQR2_SQ6_1          ((uint32_t)0x00000080U) /*!< bit 1 */
#define ADC_SQR2_SQ6_2          ((uint32_t)0x00000100U) /*!< bit 2 */
#define ADC_SQR2_SQ6_3          ((uint32_t)0x00000200U) /*!< bit 3 */
#define ADC_SQR2_SQ6_4          ((uint32_t)0x00000400U) /*!< bit 4 */

#define ADC_SQR2_SQ7            ((uint32_t)0x0001F000U) /*!< ADC group regular sequencer rank 7 */
#define ADC_SQR2_SQ7_0          ((uint32_t)0x00001000U) /*!< bit 0 */
#define ADC_SQR2_SQ7_1          ((uint32_t)0x00002000U) /*!< bit 1 */
#define ADC_SQR2_SQ7_2          ((uint32_t)0x00004000U) /*!< bit 2 */
#define ADC_SQR2_SQ7_3          ((uint32_t)0x00008000U) /*!< bit 3 */
#define ADC_SQR2_SQ7_4          ((uint32_t)0x00010000U) /*!< bit 4 */

#define ADC_SQR2_SQ8            ((uint32_t)0x007C0000U) /*!< ADC group regular sequencer rank 8 */
#define ADC_SQR2_SQ8_0          ((uint32_t)0x00040000U) /*!< bit 0 */
#define ADC_SQR2_SQ8_1          ((uint32_t)0x00080000U) /*!< bit 1 */
#define ADC_SQR2_SQ8_2          ((uint32_t)0x00100000U) /*!< bit 2 */
#define ADC_SQR2_SQ8_3          ((uint32_t)0x00200000U) /*!< bit 3 */
#define ADC_SQR2_SQ8_4          ((uint32_t)0x00400000U) /*!< bit 4 */

#define ADC_SQR2_SQ9            ((uint32_t)0x1F000000U) /*!< ADC group regular sequencer rank 9 */
#define ADC_SQR2_SQ9_0          ((uint32_t)0x01000000U) /*!< bit 0 */
#define ADC_SQR2_SQ9_1          ((uint32_t)0x02000000U) /*!< bit 1 */
#define ADC_SQR2_SQ9_2          ((uint32_t)0x04000000U) /*!< bit 2 */
#define ADC_SQR2_SQ9_3          ((uint32_t)0x08000000U) /*!< bit 3 */
#define ADC_SQR2_SQ9_4          ((uint32_t)0x10000000U) /*!< bit 4 */

/********************  Bit definition for ADC_SQR3 register  ******************/
#define ADC_SQR3_SQ10           ((uint32_t)0x0000001FU) /*!< ADC group regular sequencer rank 10 */
#define ADC_SQR3_SQ10_0         ((uint32_t)0x00000001U) /*!< bit 0 */
#define ADC_SQR3_SQ10_1         ((uint32_t)0x00000002U) /*!< bit 1 */
#define ADC_SQR3_SQ10_2         ((uint32_t)0x00000004U) /*!< bit 2 */
#define ADC_SQR3_SQ10_3         ((uint32_t)0x00000008U) /*!< bit 3 */
#define ADC_SQR3_SQ10_4         ((uint32_t)0x00000010U) /*!< bit 4 */

#define ADC_SQR3_SQ11           ((uint32_t)0x000007C0U) /*!< ADC group regular sequencer rank 11 */
#define ADC_SQR3_SQ11_0         ((uint32_t)0x00000040U) /*!< bit 0 */
#define ADC_SQR3_SQ11_1         ((uint32_t)0x00000080U) /*!< bit 1 */
#define ADC_SQR3_SQ11_2         ((uint32_t)0x00000100U) /*!< bit 2 */
#define ADC_SQR3_SQ11_3         ((uint32_t)0x00000200U) /*!< bit 3 */
#define ADC_SQR3_SQ11_4         ((uint32_t)0x00000400U) /*!< bit 4 */

#define ADC_SQR3_SQ12           ((uint32_t)0x0001F000U) /*!< ADC group regular sequencer rank 12 */
#define ADC_SQR3_SQ12_0         ((uint32_t)0x00001000U) /*!< bit 0 */
#define ADC_SQR3_SQ12_1         ((uint32_t)0x00002000U) /*!< bit 1 */
#define ADC_SQR3_SQ12_2         ((uint32_t)0x00004000U) /*!< bit 2 */
#define ADC_SQR3_SQ12_3         ((uint32_t)0x00008000U) /*!< bit 3 */
#define ADC_SQR3_SQ12_4         ((uint32_t)0x00010000U) /*!< bit 4 */

#define ADC_SQR3_SQ13           ((uint32_t)0x007C0000U) /*!< ADC group regular sequencer rank 13 */
#define ADC_SQR3_SQ13_0         ((uint32_t)0x00040000U) /*!< bit 0 */
#define ADC_SQR3_SQ13_1         ((uint32_t)0x00080000U) /*!< bit 1 */
#define ADC_SQR3_SQ13_2         ((uint32_t)0x00100000U) /*!< bit 2 */
#define ADC_SQR3_SQ13_3         ((uint32_t)0x00200000U) /*!< bit 3 */
#define ADC_SQR3_SQ13_4         ((uint32_t)0x00400000U) /*!< bit 4 */

#define ADC_SQR3_SQ14           ((uint32_t)0x1F000000U) /*!< ADC group regular sequencer rank 14 */
#define ADC_SQR3_SQ14_0         ((uint32_t)0x01000000U) /*!< bit 0 */
#define ADC_SQR3_SQ14_1         ((uint32_t)0x02000000U) /*!< bit 1 */
#define ADC_SQR3_SQ14_2         ((uint32_t)0x04000000U) /*!< bit 2 */
#define ADC_SQR3_SQ14_3         ((uint32_t)0x08000000U) /*!< bit 3 */
#define ADC_SQR3_SQ14_4         ((uint32_t)0x10000000U) /*!< bit 4 */

/********************  Bit definition for ADC_SQR4 register  ******************/
#define ADC_SQR4_SQ15           ((uint32_t)0x0000001FU) /*!< ADC group regular sequencer rank 15 */
#define ADC_SQR4_SQ15_0         ((uint32_t)0x00000001U) /*!< bit 0 */
#define ADC_SQR4_SQ15_1         ((uint32_t)0x00000002U) /*!< bit 1 */
#define ADC_SQR4_SQ15_2         ((uint32_t)0x00000004U) /*!< bit 2 */
#define ADC_SQR4_SQ15_3         ((uint32_t)0x00000008U) /*!< bit 3 */
#define ADC_SQR4_SQ15_4         ((uint32_t)0x00000010U) /*!<5 bit 4 */

#define ADC_SQR4_SQ16           ((uint32_t)0x000007C0U) /*!< ADC group regular sequencer rank 16 */
#define ADC_SQR4_SQ16_0         ((uint32_t)0x00000040U) /*!< bit 0 */
#define ADC_SQR4_SQ16_1         ((uint32_t)0x00000080U) /*!< bit 1 */
#define ADC_SQR4_SQ16_2         ((uint32_t)0x00000100U) /*!< bit 2 */
#define ADC_SQR4_SQ16_3         ((uint32_t)0x00000200U) /*!< bit 3 */
#define ADC_SQR4_SQ16_4         ((uint32_t)0x00000400U) /*!< bit 4 */

/********************  Bit definition for ADC_DR register  ********************/
#define ADC_DR_RDATA            ((uint32_t)0x0000FFFFU) /*!< ADC group regular conversion data */
#define ADC_DR_RDATA_0          ((uint32_t)0x00000001U) /*!< bit 0 */
#define ADC_DR_RDATA_1          ((uint32_t)0x00000002U) /*!< bit 1 */
#define ADC_DR_RDATA_2          ((uint32_t)0x00000004U) /*!< bit 2 */
#define ADC_DR_RDATA_3          ((uint32_t)0x00000008U) /*!< bit 3 */
#define ADC_DR_RDATA_4          ((uint32_t)0x00000010U) /*!< bit 4 */
#define ADC_DR_RDATA_5          ((uint32_t)0x00000020U) /*!< bit 5 */
#define ADC_DR_RDATA_6          ((uint32_t)0x00000040U) /*!< bit 6 */
#define ADC_DR_RDATA_7          ((uint32_t)0x00000080U) /*!< bit 7 */
#define ADC_DR_RDATA_8          ((uint32_t)0x00000100U) /*!< bit 8 */
#define ADC_DR_RDATA_9          ((uint32_t)0x00000200U) /*!< bit 9 */
#define ADC_DR_RDATA_10         ((uint32_t)0x00000400U) /*!< bit 10 */
#define ADC_DR_RDATA_11         ((uint32_t)0x00000800U) /*!< bit 11 */
#define ADC_DR_RDATA_12         ((uint32_t)0x00001000U) /*!< bit 12 */
#define ADC_DR_RDATA_13         ((uint32_t)0x00002000U) /*!< bit 13 */
#define ADC_DR_RDATA_14         ((uint32_t)0x00004000U) /*!< bit 14 */
#define ADC_DR_RDATA_15         ((uint32_t)0x00008000U) /*!< bit 15 */

/********************  Bit definition for ADC_JSQR register  ******************/
#define ADC_JSQR_JL             ((uint32_t)0x00000003U) /*!< ADC group injected sequencer scan length */
#define ADC_JSQR_JL_0           ((uint32_t)0x00000001U) /*!< bit 0 */
#define ADC_JSQR_JL_1           ((uint32_t)0x00000002U) /*!< bit 1 */

#define ADC_JSQR_JEXTSEL        ((uint32_t)0x0000003CU) /*!< ADC group injected external trigger source */
#define ADC_JSQR_JEXTSEL_0      ((uint32_t)0x00000004U) /*!< bit 0 */
#define ADC_JSQR_JEXTSEL_1      ((uint32_t)0x00000008U) /*!< bit 1 */
#define ADC_JSQR_JEXTSEL_2      ((uint32_t)0x00000010U) /*!< bit 2 */
#define ADC_JSQR_JEXTSEL_3      ((uint32_t)0x00000020U) /*!< bit 3 */

#define ADC_JSQR_JEXTEN         ((uint32_t)0x000000C0U) /*!< ADC group injected external trigger polarity */
#define ADC_JSQR_JEXTEN_0       ((uint32_t)0x00000040U) /*!< bit 0 */
#define ADC_JSQR_JEXTEN_1       ((uint32_t)0x00000080U) /*!< bit 1 */

#define ADC_JSQR_JSQ1           ((uint32_t)0x00001F00U) /*!< ADC group injected sequencer rank 1 */
#define ADC_JSQR_JSQ1_0         ((uint32_t)0x00000100U) /*!< bit 0 */
#define ADC_JSQR_JSQ1_1         ((uint32_t)0x00000200U) /*!< bit 1 */
#define ADC_JSQR_JSQ1_2         ((uint32_t)0x00000400U) /*!< bit 2 */
#define ADC_JSQR_JSQ1_3         ((uint32_t)0x00000800U) /*!< bit 3 */
#define ADC_JSQR_JSQ1_4         ((uint32_t)0x00001000U) /*!< bit 4 */

#define ADC_JSQR_JSQ2           ((uint32_t)0x0007C000U) /*!< ADC group injected sequencer rank 2 */
#define ADC_JSQR_JSQ2_0         ((uint32_t)0x00004000U) /*!< bit 0 */
#define ADC_JSQR_JSQ2_1         ((uint32_t)0x00008000U) /*!< bit 1 */
#define ADC_JSQR_JSQ2_2         ((uint32_t)0x00010000U) /*!< bit 2 */
#define ADC_JSQR_JSQ2_3         ((uint32_t)0x00020000U) /*!< bit 3 */
#define ADC_JSQR_JSQ2_4         ((uint32_t)0x00040000U) /*!< bit 4 */

#define ADC_JSQR_JSQ3           ((uint32_t)0x01F00000U) /*!< ADC group injected sequencer rank 3 */
#define ADC_JSQR_JSQ3_0         ((uint32_t)0x00100000U) /*!< bit 0 */
#define ADC_JSQR_JSQ3_1         ((uint32_t)0x00200000U) /*!< bit 1 */
#define ADC_JSQR_JSQ3_2         ((uint32_t)0x00400000U) /*!< bit 2 */
#define ADC_JSQR_JSQ3_3         ((uint32_t)0x00800000U) /*!< bit 3 */
#define ADC_JSQR_JSQ3_4         ((uint32_t)0x01000000U) /*!< bit 4 */

#define ADC_JSQR_JSQ4           ((uint32_t)0x7C000000U) /*!< ADC group injected sequencer rank 4 */
#define ADC_JSQR_JSQ4_0         ((uint32_t)0x04000000U) /*!< bit 0 */
#define ADC_JSQR_JSQ4_1         ((uint32_t)0x08000000U) /*!< bit 1 */
#define ADC_JSQR_JSQ4_2         ((uint32_t)0x10000000U) /*!< bit 2 */
#define ADC_JSQR_JSQ4_3         ((uint32_t)0x20000000U) /*!< bit 3 */
#define ADC_JSQR_JSQ4_4         ((uint32_t)0x40000000U) /*!< bit 4 */


/********************  Bit definition for ADC_OFR1 register  ******************/
#define ADC_OFR1_OFFSET1        ((uint32_t)0x00000FFFU) /*!< ADC offset number 1 offset level */
#define ADC_OFR1_OFFSET1_0      ((uint32_t)0x00000001U) /*!< bit 0 */
#define ADC_OFR1_OFFSET1_1      ((uint32_t)0x00000002U) /*!< bit 1 */
#define ADC_OFR1_OFFSET1_2      ((uint32_t)0x00000004U) /*!< bit 2 */
#define ADC_OFR1_OFFSET1_3      ((uint32_t)0x00000008U) /*!< bit 3 */
#define ADC_OFR1_OFFSET1_4      ((uint32_t)0x00000010U) /*!< bit 4 */
#define ADC_OFR1_OFFSET1_5      ((uint32_t)0x00000020U) /*!< bit 5 */
#define ADC_OFR1_OFFSET1_6      ((uint32_t)0x00000040U) /*!< bit 6 */
#define ADC_OFR1_OFFSET1_7      ((uint32_t)0x00000080U) /*!< bit 7 */
#define ADC_OFR1_OFFSET1_8      ((uint32_t)0x00000100U) /*!< bit 8 */
#define ADC_OFR1_OFFSET1_9      ((uint32_t)0x00000200U) /*!< bit 9 */
#define ADC_OFR1_OFFSET1_10     ((uint32_t)0x00000400U) /*!< bit 10 */
#define ADC_OFR1_OFFSET1_11     ((uint32_t)0x00000800U) /*!< bit 11 */

#define ADC_OFR1_OFFSET1_CH     ((uint32_t)0x7C000000U) /*!< ADC offset number 1 channel selection */
#define ADC_OFR1_OFFSET1_CH_0   ((uint32_t)0x04000000U) /*!<  bit 0 */
#define ADC_OFR1_OFFSET1_CH_1   ((uint32_t)0x08000000U) /*!<  bit 1 */
#define ADC_OFR1_OFFSET1_CH_2   ((uint32_t)0x10000000U) /*!<  bit 2 */
#define ADC_OFR1_OFFSET1_CH_3   ((uint32_t)0x20000000U) /*!<  bit 3 */
#define ADC_OFR1_OFFSET1_CH_4   ((uint32_t)0x40000000U) /*!<  bit 4 */

#define ADC_OFR1_OFFSET1_EN     ((uint32_t)0x80000000U) /*!< ADC offset number 1 enable */

/********************  Bit definition for ADC_OFR2 register  ******************/
#define ADC_OFR2_OFFSET2        ((uint32_t)0x00000FFFU) /*!< ADC offset number 2 offset level */
#define ADC_OFR2_OFFSET2_0      ((uint32_t)0x00000001U) /*!< bit 0 */
#define ADC_OFR2_OFFSET2_1      ((uint32_t)0x00000002U) /*!< bit 1 */
#define ADC_OFR2_OFFSET2_2      ((uint32_t)0x00000004U) /*!< bit 2 */
#define ADC_OFR2_OFFSET2_3      ((uint32_t)0x00000008U) /*!< bit 3 */
#define ADC_OFR2_OFFSET2_4      ((uint32_t)0x00000010U) /*!< bit 4 */
#define ADC_OFR2_OFFSET2_5      ((uint32_t)0x00000020U) /*!< bit 5 */
#define ADC_OFR2_OFFSET2_6      ((uint32_t)0x00000040U) /*!< bit 6 */
#define ADC_OFR2_OFFSET2_7      ((uint32_t)0x00000080U) /*!< bit 7 */
#define ADC_OFR2_OFFSET2_8      ((uint32_t)0x00000100U) /*!< bit 8 */
#define ADC_OFR2_OFFSET2_9      ((uint32_t)0x00000200U) /*!< bit 9 */
#define ADC_OFR2_OFFSET2_10     ((uint32_t)0x00000400U) /*!< bit 10 */
#define ADC_OFR2_OFFSET2_11     ((uint32_t)0x00000800U) /*!< bit 11 */

#define ADC_OFR2_OFFSET2_CH     ((uint32_t)0x7C000000U) /*!< ADC offset number 2 channel selection */
#define ADC_OFR2_OFFSET2_CH_0   ((uint32_t)0x04000000U) /*!< bit 0 */
#define ADC_OFR2_OFFSET2_CH_1   ((uint32_t)0x08000000U) /*!< bit 1 */
#define ADC_OFR2_OFFSET2_CH_2   ((uint32_t)0x10000000U) /*!< bit 2 */
#define ADC_OFR2_OFFSET2_CH_3   ((uint32_t)0x20000000U) /*!< bit 3 */
#define ADC_OFR2_OFFSET2_CH_4   ((uint32_t)0x40000000U) /*!< bit 4 */

#define ADC_OFR2_OFFSET2_EN     ((uint32_t)0x80000000U) /*!< ADC offset number 2 enable */

/********************  Bit definition for ADC_OFR3 register  ******************/
#define ADC_OFR3_OFFSET3        ((uint32_t)0x00000FFFU) /*!< ADC offset number 3 offset level */
#define ADC_OFR3_OFFSET3_0      ((uint32_t)0x00000001U) /*!< bit 0 */
#define ADC_OFR3_OFFSET3_1      ((uint32_t)0x00000002U) /*!< bit 1 */
#define ADC_OFR3_OFFSET3_2      ((uint32_t)0x00000004U) /*!< bit 2 */
#define ADC_OFR3_OFFSET3_3      ((uint32_t)0x00000008U) /*!< bit 3 */
#define ADC_OFR3_OFFSET3_4      ((uint32_t)0x00000010U) /*!< bit 4 */
#define ADC_OFR3_OFFSET3_5      ((uint32_t)0x00000020U) /*!< bit 5 */
#define ADC_OFR3_OFFSET3_6      ((uint32_t)0x00000040U) /*!< bit 6 */
#define ADC_OFR3_OFFSET3_7      ((uint32_t)0x00000080U) /*!< bit 7 */
#define ADC_OFR3_OFFSET3_8      ((uint32_t)0x00000100U) /*!< bit 8 */
#define ADC_OFR3_OFFSET3_9      ((uint32_t)0x00000200U) /*!< bit 9 */
#define ADC_OFR3_OFFSET3_10     ((uint32_t)0x00000400U) /*!< bit 10 */
#define ADC_OFR3_OFFSET3_11     ((uint32_t)0x00000800U) /*!< bit 11 */

#define ADC_OFR3_OFFSET3_CH     ((uint32_t)0x7C000000U) /*!< ADC offset number 3 channel selection */
#define ADC_OFR3_OFFSET3_CH_0   ((uint32_t)0x04000000U) /*!< bit 0 */
#define ADC_OFR3_OFFSET3_CH_1   ((uint32_t)0x08000000U) /*!< bit 1 */
#define ADC_OFR3_OFFSET3_CH_2   ((uint32_t)0x10000000U) /*!< bit 2 */
#define ADC_OFR3_OFFSET3_CH_3   ((uint32_t)0x20000000U) /*!< bit 3 */
#define ADC_OFR3_OFFSET3_CH_4   ((uint32_t)0x40000000U) /*!< bit 4 */

#define ADC_OFR3_OFFSET3_EN     ((uint32_t)0x80000000U) /*!< ADC offset number 3 enable */

/********************  Bit definition for ADC_OFR4 register  ******************/
#define ADC_OFR4_OFFSET4        ((uint32_t)0x00000FFFU) /*!< ADC offset number 4 offset level */
#define ADC_OFR4_OFFSET4_0      ((uint32_t)0x00000001U) /*!< bit 0 */
#define ADC_OFR4_OFFSET4_1      ((uint32_t)0x00000002U) /*!< bit 1 */
#define ADC_OFR4_OFFSET4_2      ((uint32_t)0x00000004U) /*!< bit 2 */
#define ADC_OFR4_OFFSET4_3      ((uint32_t)0x00000008U) /*!< bit 3 */
#define ADC_OFR4_OFFSET4_4      ((uint32_t)0x00000010U) /*!< bit 4 */
#define ADC_OFR4_OFFSET4_5      ((uint32_t)0x00000020U) /*!< bit 5 */
#define ADC_OFR4_OFFSET4_6      ((uint32_t)0x00000040U) /*!< bit 6 */
#define ADC_OFR4_OFFSET4_7      ((uint32_t)0x00000080U) /*!< bit 7 */
#define ADC_OFR4_OFFSET4_8      ((uint32_t)0x00000100U) /*!< bit 8 */
#define ADC_OFR4_OFFSET4_9      ((uint32_t)0x00000200U) /*!< bit 9 */
#define ADC_OFR4_OFFSET4_10     ((uint32_t)0x00000400U) /*!< bit 10 */
#define ADC_OFR4_OFFSET4_11     ((uint32_t)0x00000800U) /*!< bit 11 */

#define ADC_OFR4_OFFSET4_CH     ((uint32_t)0x7C000000U) /*!< ADC offset number 4 channel selection */
#define ADC_OFR4_OFFSET4_CH_0   ((uint32_t)0x04000000U) /*!< bit 0 */
#define ADC_OFR4_OFFSET4_CH_1   ((uint32_t)0x08000000U) /*!< bit 1 */
#define ADC_OFR4_OFFSET4_CH_2   ((uint32_t)0x10000000U) /*!< bit 2 */
#define ADC_OFR4_OFFSET4_CH_3   ((uint32_t)0x20000000U) /*!< bit 3 */
#define ADC_OFR4_OFFSET4_CH_4   ((uint32_t)0x40000000U) /*!< bit 4 */

#define ADC_OFR4_OFFSET4_EN     ((uint32_t)0x80000000U) /*!< ADC offset number 4 enable */

/********************  Bit definition for ADC_JDR1 register  ******************/
#define ADC_JDR1_JDATA          ((uint32_t)0x0000FFFFU) /*!< ADC group injected sequencer rank 1 conversion data */
#define ADC_JDR1_JDATA_0        ((uint32_t)0x00000001U) /*!< bit 0 */
#define ADC_JDR1_JDATA_1        ((uint32_t)0x00000002U) /*!< bit 1 */
#define ADC_JDR1_JDATA_2        ((uint32_t)0x00000004U) /*!< bit 2 */
#define ADC_JDR1_JDATA_3        ((uint32_t)0x00000008U) /*!< bit 3 */
#define ADC_JDR1_JDATA_4        ((uint32_t)0x00000010U) /*!< bit 4 */
#define ADC_JDR1_JDATA_5        ((uint32_t)0x00000020U) /*!< bit 5 */
#define ADC_JDR1_JDATA_6        ((uint32_t)0x00000040U) /*!< bit 6 */
#define ADC_JDR1_JDATA_7        ((uint32_t)0x00000080U) /*!< bit 7 */
#define ADC_JDR1_JDATA_8        ((uint32_t)0x00000100U) /*!< bit 8 */
#define ADC_JDR1_JDATA_9        ((uint32_t)0x00000200U) /*!< bit 9 */
#define ADC_JDR1_JDATA_10       ((uint32_t)0x00000400U) /*!< bit 10 */
#define ADC_JDR1_JDATA_11       ((uint32_t)0x00000800U) /*!< bit 11 */
#define ADC_JDR1_JDATA_12       ((uint32_t)0x00001000U) /*!< bit 12 */
#define ADC_JDR1_JDATA_13       ((uint32_t)0x00002000U) /*!< bit 13 */
#define ADC_JDR1_JDATA_14       ((uint32_t)0x00004000U) /*!< bit 14 */
#define ADC_JDR1_JDATA_15       ((uint32_t)0x00008000U) /*!< bit 15 */

/********************  Bit definition for ADC_JDR2 register  ******************/
#define ADC_JDR2_JDATA          ((uint32_t)0x0000FFFFU) /*!< ADC group injected sequencer rank 2 conversion data */
#define ADC_JDR2_JDATA_0        ((uint32_t)0x00000001U) /*!< bit 0 */
#define ADC_JDR2_JDATA_1        ((uint32_t)0x00000002U) /*!< bit 1 */
#define ADC_JDR2_JDATA_2        ((uint32_t)0x00000004U) /*!< bit 2 */
#define ADC_JDR2_JDATA_3        ((uint32_t)0x00000008U) /*!< bit 3 */
#define ADC_JDR2_JDATA_4        ((uint32_t)0x00000010U) /*!< bit 4 */
#define ADC_JDR2_JDATA_5        ((uint32_t)0x00000020U) /*!< bit 5 */
#define ADC_JDR2_JDATA_6        ((uint32_t)0x00000040U) /*!< bit 6 */
#define ADC_JDR2_JDATA_7        ((uint32_t)0x00000080U) /*!< bit 7 */
#define ADC_JDR2_JDATA_8        ((uint32_t)0x00000100U) /*!< bit 8 */
#define ADC_JDR2_JDATA_9        ((uint32_t)0x00000200U) /*!< bit 9 */
#define ADC_JDR2_JDATA_10       ((uint32_t)0x00000400U) /*!< bit 10 */
#define ADC_JDR2_JDATA_11       ((uint32_t)0x00000800U) /*!< bit 11 */
#define ADC_JDR2_JDATA_12       ((uint32_t)0x00001000U) /*!< bit 12 */
#define ADC_JDR2_JDATA_13       ((uint32_t)0x00002000U) /*!< bit 13 */
#define ADC_JDR2_JDATA_14       ((uint32_t)0x00004000U) /*!< bit 14 */
#define ADC_JDR2_JDATA_15       ((uint32_t)0x00008000U) /*!< bit 15 */

/********************  Bit definition for ADC_JDR3 register  ******************/
#define ADC_JDR3_JDATA          ((uint32_t)0x0000FFFFU) /*!< ADC group injected sequencer rank 3 conversion data */
#define ADC_JDR3_JDATA_0        ((uint32_t)0x00000001U) /*!< bit 0 */
#define ADC_JDR3_JDATA_1        ((uint32_t)0x00000002U) /*!< bit 1 */
#define ADC_JDR3_JDATA_2        ((uint32_t)0x00000004U) /*!< bit 2 */
#define ADC_JDR3_JDATA_3        ((uint32_t)0x00000008U) /*!< bit 3 */
#define ADC_JDR3_JDATA_4        ((uint32_t)0x00000010U) /*!< bit 4 */
#define ADC_JDR3_JDATA_5        ((uint32_t)0x00000020U) /*!< bit 5 */
#define ADC_JDR3_JDATA_6        ((uint32_t)0x00000040U) /*!< bit 6 */
#define ADC_JDR3_JDATA_7        ((uint32_t)0x00000080U) /*!< bit 7 */
#define ADC_JDR3_JDATA_8        ((uint32_t)0x00000100U) /*!< bit 8 */
#define ADC_JDR3_JDATA_9        ((uint32_t)0x00000200U) /*!< bit 9 */
#define ADC_JDR3_JDATA_10       ((uint32_t)0x00000400U) /*!< bit 10 */
#define ADC_JDR3_JDATA_11       ((uint32_t)0x00000800U) /*!< bit 11 */
#define ADC_JDR3_JDATA_12       ((uint32_t)0x00001000U) /*!< bit 12 */
#define ADC_JDR3_JDATA_13       ((uint32_t)0x00002000U) /*!< bit 13 */
#define ADC_JDR3_JDATA_14       ((uint32_t)0x00004000U) /*!< bit 14 */
#define ADC_JDR3_JDATA_15       ((uint32_t)0x00008000U) /*!< bit 15 */

/********************  Bit definition for ADC_JDR4 register  ******************/
#define ADC_JDR4_JDATA          ((uint32_t)0x0000FFFFU) /*!< ADC group injected sequencer rank 4 conversion data */
#define ADC_JDR4_JDATA_0        ((uint32_t)0x00000001U) /*!< bit 0 */
#define ADC_JDR4_JDATA_1        ((uint32_t)0x00000002U) /*!< bit 1 */
#define ADC_JDR4_JDATA_2        ((uint32_t)0x00000004U) /*!< bit 2 */
#define ADC_JDR4_JDATA_3        ((uint32_t)0x00000008U) /*!< bit 3 */
#define ADC_JDR4_JDATA_4        ((uint32_t)0x00000010U) /*!< bit 4 */
#define ADC_JDR4_JDATA_5        ((uint32_t)0x00000020U) /*!< bit 5 */
#define ADC_JDR4_JDATA_6        ((uint32_t)0x00000040U) /*!< bit 6 */
#define ADC_JDR4_JDATA_7        ((uint32_t)0x00000080U) /*!< bit 7 */
#define ADC_JDR4_JDATA_8        ((uint32_t)0x00000100U) /*!< bit 8 */
#define ADC_JDR4_JDATA_9        ((uint32_t)0x00000200U) /*!< bit 9 */
#define ADC_JDR4_JDATA_10       ((uint32_t)0x00000400U) /*!< bit 10 */
#define ADC_JDR4_JDATA_11       ((uint32_t)0x00000800U) /*!< bit 11 */
#define ADC_JDR4_JDATA_12       ((uint32_t)0x00001000U) /*!< bit 12 */
#define ADC_JDR4_JDATA_13       ((uint32_t)0x00002000U) /*!< bit 13 */
#define ADC_JDR4_JDATA_14       ((uint32_t)0x00004000U) /*!< bit 14 */
#define ADC_JDR4_JDATA_15       ((uint32_t)0x00008000U) /*!< bit 15 */

/********************  Bit definition for ADC_AWD2CR register  ****************/
#define ADC_AWD2CR_AWD2CH       ((uint32_t)0x0007FFFFU) /*!< ADC analog watchdog 2 monitored channel selection */
#define ADC_AWD2CR_AWD2CH_0     ((uint32_t)0x00000001U) /*!< ADC analog watchdog 2 monitoring channel 0 */
#define ADC_AWD2CR_AWD2CH_1     ((uint32_t)0x00000002U) /*!< ADC analog watchdog 2 monitoring channel 1 */
#define ADC_AWD2CR_AWD2CH_2     ((uint32_t)0x00000004U) /*!< ADC analog watchdog 2 monitoring channel 2 */
#define ADC_AWD2CR_AWD2CH_3     ((uint32_t)0x00000008U) /*!< ADC analog watchdog 2 monitoring channel 3 */
#define ADC_AWD2CR_AWD2CH_4     ((uint32_t)0x00000010U) /*!< ADC analog watchdog 2 monitoring channel 4 */
#define ADC_AWD2CR_AWD2CH_5     ((uint32_t)0x00000020U) /*!< ADC analog watchdog 2 monitoring channel 5 */
#define ADC_AWD2CR_AWD2CH_6     ((uint32_t)0x00000040U) /*!< ADC analog watchdog 2 monitoring channel 6 */
#define ADC_AWD2CR_AWD2CH_7     ((uint32_t)0x00000080U) /*!< ADC analog watchdog 2 monitoring channel 7 */
#define ADC_AWD2CR_AWD2CH_8     ((uint32_t)0x00000100U) /*!< ADC analog watchdog 2 monitoring channel 8 */
#define ADC_AWD2CR_AWD2CH_9     ((uint32_t)0x00000200U) /*!< ADC analog watchdog 2 monitoring channel 9 */
#define ADC_AWD2CR_AWD2CH_10    ((uint32_t)0x00000400U) /*!< ADC analog watchdog 2 monitoring channel 10 */
#define ADC_AWD2CR_AWD2CH_11    ((uint32_t)0x00000800U) /*!< ADC analog watchdog 2 monitoring channel 11 */
#define ADC_AWD2CR_AWD2CH_12    ((uint32_t)0x00001000U) /*!< ADC analog watchdog 2 monitoring channel 12 */
#define ADC_AWD2CR_AWD2CH_13    ((uint32_t)0x00002000U) /*!< ADC analog watchdog 2 monitoring channel 13 */
#define ADC_AWD2CR_AWD2CH_14    ((uint32_t)0x00004000U) /*!< ADC analog watchdog 2 monitoring channel 14 */
#define ADC_AWD2CR_AWD2CH_15    ((uint32_t)0x00008000U) /*!< ADC analog watchdog 2 monitoring channel 15 */
#define ADC_AWD2CR_AWD2CH_16    ((uint32_t)0x00010000U) /*!< ADC analog watchdog 2 monitoring channel 16 */
#define ADC_AWD2CR_AWD2CH_17    ((uint32_t)0x00020000U) /*!< ADC analog watchdog 2 monitoring channel 17 */
#define ADC_AWD2CR_AWD2CH_18    ((uint32_t)0x00040000U) /*!< ADC analog watchdog 2 monitoring channel 18 */

/********************  Bit definition for ADC_AWD3CR register  ****************/
#define ADC_AWD3CR_AWD3CH       ((uint32_t)0x0007FFFFU) /*!< ADC analog watchdog 3 monitored channel selection */
#define ADC_AWD3CR_AWD3CH_0     ((uint32_t)0x00000001U) /*!< ADC analog watchdog 3 monitoring channel 0 */
#define ADC_AWD3CR_AWD3CH_1     ((uint32_t)0x00000002U) /*!< ADC analog watchdog 3 monitoring channel 1 */
#define ADC_AWD3CR_AWD3CH_2     ((uint32_t)0x00000004U) /*!< ADC analog watchdog 3 monitoring channel 2 */
#define ADC_AWD3CR_AWD3CH_3     ((uint32_t)0x00000008U) /*!< ADC analog watchdog 3 monitoring channel 3 */
#define ADC_AWD3CR_AWD3CH_4     ((uint32_t)0x00000010U) /*!< ADC analog watchdog 3 monitoring channel 4 */
#define ADC_AWD3CR_AWD3CH_5     ((uint32_t)0x00000020U) /*!< ADC analog watchdog 3 monitoring channel 5 */
#define ADC_AWD3CR_AWD3CH_6     ((uint32_t)0x00000040U) /*!< ADC analog watchdog 3 monitoring channel 6 */
#define ADC_AWD3CR_AWD3CH_7     ((uint32_t)0x00000080U) /*!< ADC analog watchdog 3 monitoring channel 7 */
#define ADC_AWD3CR_AWD3CH_8     ((uint32_t)0x00000100U) /*!< ADC analog watchdog 3 monitoring channel 8 */
#define ADC_AWD3CR_AWD3CH_9     ((uint32_t)0x00000200U) /*!< ADC analog watchdog 3 monitoring channel 9 */
#define ADC_AWD3CR_AWD3CH_10    ((uint32_t)0x00000400U) /*!< ADC analog watchdog 3 monitoring channel 10 */
#define ADC_AWD3CR_AWD3CH_11    ((uint32_t)0x00000800U) /*!< ADC analog watchdog 3 monitoring channel 11 */
#define ADC_AWD3CR_AWD3CH_12    ((uint32_t)0x00001000U) /*!< ADC analog watchdog 3 monitoring channel 12 */
#define ADC_AWD3CR_AWD3CH_13    ((uint32_t)0x00002000U) /*!< ADC analog watchdog 3 monitoring channel 13 */
#define ADC_AWD3CR_AWD3CH_14    ((uint32_t)0x00004000U) /*!< ADC analog watchdog 3 monitoring channel 14 */
#define ADC_AWD3CR_AWD3CH_15    ((uint32_t)0x00008000U) /*!< ADC analog watchdog 3 monitoring channel 15 */
#define ADC_AWD3CR_AWD3CH_16    ((uint32_t)0x00010000U) /*!< ADC analog watchdog 3 monitoring channel 16 */
#define ADC_AWD3CR_AWD3CH_17    ((uint32_t)0x00020000U) /*!< ADC analog watchdog 3 monitoring channel 17 */
#define ADC_AWD3CR_AWD3CH_18    ((uint32_t)0x00040000U) /*!< ADC analog watchdog 3 monitoring channel 18 */

/********************  Bit definition for ADC_DIFSEL register  ****************/
#define ADC_DIFSEL_DIFSEL       ((uint32_t)0x0007FFFFU) /*!< ADC channel differential or single-ended mode */
#define ADC_DIFSEL_DIFSEL_0     ((uint32_t)0x00000001U) /*!< bit 0 */
#define ADC_DIFSEL_DIFSEL_1     ((uint32_t)0x00000002U) /*!< bit 1 */
#define ADC_DIFSEL_DIFSEL_2     ((uint32_t)0x00000004U) /*!< bit 2 */
#define ADC_DIFSEL_DIFSEL_3     ((uint32_t)0x00000008U) /*!< bit 3 */
#define ADC_DIFSEL_DIFSEL_4     ((uint32_t)0x00000010U) /*!< bit 4 */
#define ADC_DIFSEL_DIFSEL_5     ((uint32_t)0x00000020U) /*!< bit 5 */
#define ADC_DIFSEL_DIFSEL_6     ((uint32_t)0x00000040U) /*!< bit 6 */
#define ADC_DIFSEL_DIFSEL_7     ((uint32_t)0x00000080U) /*!< bit 7 */
#define ADC_DIFSEL_DIFSEL_8     ((uint32_t)0x00000100U) /*!< bit 8 */
#define ADC_DIFSEL_DIFSEL_9     ((uint32_t)0x00000200U) /*!< bit 9 */
#define ADC_DIFSEL_DIFSEL_10    ((uint32_t)0x00000400U) /*!< bit 10 */
#define ADC_DIFSEL_DIFSEL_11    ((uint32_t)0x00000800U) /*!< bit 11 */
#define ADC_DIFSEL_DIFSEL_12    ((uint32_t)0x00001000U) /*!< bit 12 */
#define ADC_DIFSEL_DIFSEL_13    ((uint32_t)0x00002000U) /*!< bit 13 */
#define ADC_DIFSEL_DIFSEL_14    ((uint32_t)0x00004000U) /*!< bit 14 */
#define ADC_DIFSEL_DIFSEL_15    ((uint32_t)0x00008000U) /*!< bit 15 */
#define ADC_DIFSEL_DIFSEL_16    ((uint32_t)0x00010000U) /*!< bit 16 */
#define ADC_DIFSEL_DIFSEL_17    ((uint32_t)0x00020000U) /*!< bit 17 */
#define ADC_DIFSEL_DIFSEL_18    ((uint32_t)0x00040000U) /*!< bit 18 */

/********************  Bit definition for ADC_CALFACT register  ***************/
#define ADC_CALFACT_CALFACT_S   ((uint32_t)0x0000007FU) /*!< ADC calibration factor in single-ended mode */
#define ADC_CALFACT_CALFACT_S_0 ((uint32_t)0x00000001U) /*!< bit 0 */
#define ADC_CALFACT_CALFACT_S_1 ((uint32_t)0x00000002U) /*!< bit 1 */
#define ADC_CALFACT_CALFACT_S_2 ((uint32_t)0x00000004U) /*!< bit 2 */
#define ADC_CALFACT_CALFACT_S_3 ((uint32_t)0x00000008U) /*!< bit 3 */
#define ADC_CALFACT_CALFACT_S_4 ((uint32_t)0x00000010U) /*!< bit 4 */
#define ADC_CALFACT_CALFACT_S_5 ((uint32_t)0x00000020U) /*!< bit 5 */
#define ADC_CALFACT_CALFACT_S_6 ((uint32_t)0x00000040U) /*!< bit 6 */

#define ADC_CALFACT_CALFACT_D   ((uint32_t)0x007F0000U) /*!< ADC calibration factor in differential mode */
#define ADC_CALFACT_CALFACT_D_0 ((uint32_t)0x00010000U) /*!< bit 0 */
#define ADC_CALFACT_CALFACT_D_1 ((uint32_t)0x00020000U) /*!< bit 1 */
#define ADC_CALFACT_CALFACT_D_2 ((uint32_t)0x00040000U) /*!< bit 2 */
#define ADC_CALFACT_CALFACT_D_3 ((uint32_t)0x00080000U) /*!< bit 3 */
#define ADC_CALFACT_CALFACT_D_4 ((uint32_t)0x00100000U) /*!< bit 4 */
#define ADC_CALFACT_CALFACT_D_5 ((uint32_t)0x00200000U) /*!< bit 5 */
#define ADC_CALFACT_CALFACT_D_6 ((uint32_t)0x00400000U) /*!< bit 6 */

/*************************  ADC Common registers  *****************************/
/********************  Bit definition for ADC_CCR register  *******************/
#define ADC_CCR_CKMODE          ((uint32_t)0x00030000U) /*!< ADC common clock source and prescaler (prescaler only for clock source synchronous) */
#define ADC_CCR_CKMODE_0        ((uint32_t)0x00010000U) /*!< bit 0 */
#define ADC_CCR_CKMODE_1        ((uint32_t)0x00020000U) /*!< bit 1 */

#define ADC_CCR_PRESC           ((uint32_t)0x003C0000U) /*!< ADC common clock prescaler, only for clock source asynchronous */
#define ADC_CCR_PRESC_0         ((uint32_t)0x00040000U) /*!< bit 0 */
#define ADC_CCR_PRESC_1         ((uint32_t)0x00080000U) /*!< bit 1 */
#define ADC_CCR_PRESC_2         ((uint32_t)0x00100000U) /*!< bit 2 */
#define ADC_CCR_PRESC_3         ((uint32_t)0x00200000U) /*!< bit 3 */

#define ADC_CCR_VREFEN          ((uint32_t)0x00400000U) /*!< ADC internal path to VrefInt enable */
#define ADC_CCR_TSEN            ((uint32_t)0x00800000U) /*!< ADC internal path to temperature sensor enable */
#define ADC_CCR_VBATEN          ((uint32_t)0x01000000U) /*!< ADC internal path to battery voltage enable */

/******************************************************************************/
/*                                                                            */
/*                         Controller Area Network                            */
/*                                                                            */
/******************************************************************************/
/*!<CAN control and status registers */
/*******************  Bit definition for CAN_MCR register  ********************/
#define  CAN_MCR_INRQ                        ((uint16_t)0x0001U)            /*!<Initialization Request */
#define  CAN_MCR_SLEEP                       ((uint16_t)0x0002U)            /*!<Sleep Mode Request */
#define  CAN_MCR_TXFP                        ((uint16_t)0x0004U)            /*!<Transmit FIFO Priority */
#define  CAN_MCR_RFLM                        ((uint16_t)0x0008U)            /*!<Receive FIFO Locked Mode */
#define  CAN_MCR_NART                        ((uint16_t)0x0010U)            /*!<No Automatic Retransmission */
#define  CAN_MCR_AWUM                        ((uint16_t)0x0020U)            /*!<Automatic Wakeup Mode */
#define  CAN_MCR_ABOM                        ((uint16_t)0x0040U)            /*!<Automatic Bus-Off Management */
#define  CAN_MCR_TTCM                        ((uint16_t)0x0080U)            /*!<Time Triggered Communication Mode */
#define  CAN_MCR_RESET                       ((uint16_t)0x8000U)            /*!<bxCAN software master reset */

/*******************  Bit definition for CAN_MSR register  ********************/
#define  CAN_MSR_INAK                        ((uint16_t)0x0001U)            /*!<Initialization Acknowledge */
#define  CAN_MSR_SLAK                        ((uint16_t)0x0002U)            /*!<Sleep Acknowledge */
#define  CAN_MSR_ERRI                        ((uint16_t)0x0004U)            /*!<Error Interrupt */
#define  CAN_MSR_WKUI                        ((uint16_t)0x0008U)            /*!<Wakeup Interrupt */
#define  CAN_MSR_SLAKI                       ((uint16_t)0x0010U)            /*!<Sleep Acknowledge Interrupt */
#define  CAN_MSR_TXM                         ((uint16_t)0x0100U)            /*!<Transmit Mode */
#define  CAN_MSR_RXM                         ((uint16_t)0x0200U)            /*!<Receive Mode */
#define  CAN_MSR_SAMP                        ((uint16_t)0x0400U)            /*!<Last Sample Point */
#define  CAN_MSR_RX                          ((uint16_t)0x0800U)            /*!<CAN Rx Signal */

/*******************  Bit definition for CAN_TSR register  ********************/
#define  CAN_TSR_RQCP0                       ((uint32_t)0x00000001U)        /*!<Request Completed Mailbox0 */
#define  CAN_TSR_TXOK0                       ((uint32_t)0x00000002U)        /*!<Transmission OK of Mailbox0 */
#define  CAN_TSR_ALST0                       ((uint32_t)0x00000004U)        /*!<Arbitration Lost for Mailbox0 */
#define  CAN_TSR_TERR0                       ((uint32_t)0x00000008U)        /*!<Transmission Error of Mailbox0 */
#define  CAN_TSR_ABRQ0                       ((uint32_t)0x00000080U)        /*!<Abort Request for Mailbox0 */
#define  CAN_TSR_RQCP1                       ((uint32_t)0x00000100U)        /*!<Request Completed Mailbox1 */
#define  CAN_TSR_TXOK1                       ((uint32_t)0x00000200U)        /*!<Transmission OK of Mailbox1 */
#define  CAN_TSR_ALST1                       ((uint32_t)0x00000400U)        /*!<Arbitration Lost for Mailbox1 */
#define  CAN_TSR_TERR1                       ((uint32_t)0x00000800U)        /*!<Transmission Error of Mailbox1 */
#define  CAN_TSR_ABRQ1                       ((uint32_t)0x00008000U)        /*!<Abort Request for Mailbox 1 */
#define  CAN_TSR_RQCP2                       ((uint32_t)0x00010000U)        /*!<Request Completed Mailbox2 */
#define  CAN_TSR_TXOK2                       ((uint32_t)0x00020000U)        /*!<Transmission OK of Mailbox 2 */
#define  CAN_TSR_ALST2                       ((uint32_t)0x00040000U)        /*!<Arbitration Lost for mailbox 2 */
#define  CAN_TSR_TERR2                       ((uint32_t)0x00080000U)        /*!<Transmission Error of Mailbox 2 */
#define  CAN_TSR_ABRQ2                       ((uint32_t)0x00800000U)        /*!<Abort Request for Mailbox 2 */
#define  CAN_TSR_CODE                        ((uint32_t)0x03000000U)        /*!<Mailbox Code */

#define  CAN_TSR_TME                         ((uint32_t)0x1C000000U)        /*!<TME[2:0] bits */
#define  CAN_TSR_TME0                        ((uint32_t)0x04000000U)        /*!<Transmit Mailbox 0 Empty */
#define  CAN_TSR_TME1                        ((uint32_t)0x08000000U)        /*!<Transmit Mailbox 1 Empty */
#define  CAN_TSR_TME2                        ((uint32_t)0x10000000U)        /*!<Transmit Mailbox 2 Empty */

#define  CAN_TSR_LOW                         ((uint32_t)0xE0000000U)        /*!<LOW[2:0] bits */
#define  CAN_TSR_LOW0                        ((uint32_t)0x20000000U)        /*!<Lowest Priority Flag for Mailbox 0 */
#define  CAN_TSR_LOW1                        ((uint32_t)0x40000000U)        /*!<Lowest Priority Flag for Mailbox 1 */
#define  CAN_TSR_LOW2                        ((uint32_t)0x80000000U)        /*!<Lowest Priority Flag for Mailbox 2 */

/*******************  Bit definition for CAN_RF0R register  *******************/
#define  CAN_RF0R_FMP0                       ((uint8_t)0x03U)               /*!<FIFO 0 Message Pending */
#define  CAN_RF0R_FULL0                      ((uint8_t)0x08U)               /*!<FIFO 0 Full */
#define  CAN_RF0R_FOVR0                      ((uint8_t)0x10U)               /*!<FIFO 0 Overrun */
#define  CAN_RF0R_RFOM0                      ((uint8_t)0x20U)               /*!<Release FIFO 0 Output Mailbox */

/*******************  Bit definition for CAN_RF1R register  *******************/
#define  CAN_RF1R_FMP1                       ((uint8_t)0x03U)               /*!<FIFO 1 Message Pending */
#define  CAN_RF1R_FULL1                      ((uint8_t)0x08U)               /*!<FIFO 1 Full */
#define  CAN_RF1R_FOVR1                      ((uint8_t)0x10U)               /*!<FIFO 1 Overrun */
#define  CAN_RF1R_RFOM1                      ((uint8_t)0x20U)               /*!<Release FIFO 1 Output Mailbox */

/********************  Bit definition for CAN_IER register  *******************/
#define  CAN_IER_TMEIE                       ((uint32_t)0x00000001U)        /*!<Transmit Mailbox Empty Interrupt Enable */
#define  CAN_IER_FMPIE0                      ((uint32_t)0x00000002U)        /*!<FIFO Message Pending Interrupt Enable */
#define  CAN_IER_FFIE0                       ((uint32_t)0x00000004U)        /*!<FIFO Full Interrupt Enable */
#define  CAN_IER_FOVIE0                      ((uint32_t)0x00000008U)        /*!<FIFO Overrun Interrupt Enable */
#define  CAN_IER_FMPIE1                      ((uint32_t)0x00000010U)        /*!<FIFO Message Pending Interrupt Enable */
#define  CAN_IER_FFIE1                       ((uint32_t)0x00000020U)        /*!<FIFO Full Interrupt Enable */
#define  CAN_IER_FOVIE1                      ((uint32_t)0x00000040U)        /*!<FIFO Overrun Interrupt Enable */
#define  CAN_IER_EWGIE                       ((uint32_t)0x00000100U)        /*!<Error Warning Interrupt Enable */
#define  CAN_IER_EPVIE                       ((uint32_t)0x00000200U)        /*!<Error Passive Interrupt Enable */
#define  CAN_IER_BOFIE                       ((uint32_t)0x00000400U)        /*!<Bus-Off Interrupt Enable */
#define  CAN_IER_LECIE                       ((uint32_t)0x00000800U)        /*!<Last Error Code Interrupt Enable */
#define  CAN_IER_ERRIE                       ((uint32_t)0x00008000U)        /*!<Error Interrupt Enable */
#define  CAN_IER_WKUIE                       ((uint32_t)0x00010000U)        /*!<Wakeup Interrupt Enable */
#define  CAN_IER_SLKIE                       ((uint32_t)0x00020000U)        /*!<Sleep Interrupt Enable */

/********************  Bit definition for CAN_ESR register  *******************/
#define  CAN_ESR_EWGF                        ((uint32_t)0x00000001U)        /*!<Error Warning Flag */
#define  CAN_ESR_EPVF                        ((uint32_t)0x00000002U)        /*!<Error Passive Flag */
#define  CAN_ESR_BOFF                        ((uint32_t)0x00000004U)        /*!<Bus-Off Flag */

#define  CAN_ESR_LEC                         ((uint32_t)0x00000070U)        /*!<LEC[2:0] bits (Last Error Code) */
#define  CAN_ESR_LEC_0                       ((uint32_t)0x00000010U)        /*!<Bit 0 */
#define  CAN_ESR_LEC_1                       ((uint32_t)0x00000020U)        /*!<Bit 1 */
#define  CAN_ESR_LEC_2                       ((uint32_t)0x00000040U)        /*!<Bit 2 */

#define  CAN_ESR_TEC                         ((uint32_t)0x00FF0000U)        /*!<Least significant byte of the 9-bit Transmit Error Counter */
#define  CAN_ESR_REC                         ((uint32_t)0xFF000000U)        /*!<Receive Error Counter */

/*******************  Bit definition for CAN_BTR register  ********************/
#define  CAN_BTR_BRP                         ((uint32_t)0x000003FFU)        /*!<Baud Rate Prescaler */
#define  CAN_BTR_TS1_0                       ((uint32_t)0x00010000U)        /*!<Time Segment 1 (Bit 0) */
#define  CAN_BTR_TS1_1                       ((uint32_t)0x00020000U)        /*!<Time Segment 1 (Bit 1) */
#define  CAN_BTR_TS1_2                       ((uint32_t)0x00040000U)        /*!<Time Segment 1 (Bit 2) */
#define  CAN_BTR_TS1_3                       ((uint32_t)0x00080000U)        /*!<Time Segment 1 (Bit 3) */
#define  CAN_BTR_TS1                         ((uint32_t)0x000F0000U)        /*!<Time Segment 1 */
#define  CAN_BTR_TS2_0                       ((uint32_t)0x00100000U)        /*!<Time Segment 2 (Bit 0) */
#define  CAN_BTR_TS2_1                       ((uint32_t)0x00200000U)        /*!<Time Segment 2 (Bit 1) */
#define  CAN_BTR_TS2_2                       ((uint32_t)0x00400000U)        /*!<Time Segment 2 (Bit 2) */
#define  CAN_BTR_TS2                         ((uint32_t)0x00700000U)        /*!<Time Segment 2 */
#define  CAN_BTR_SJW_0                       ((uint32_t)0x01000000U)        /*!<Resynchronization Jump Width (Bit 0) */
#define  CAN_BTR_SJW_1                       ((uint32_t)0x02000000U)        /*!<Resynchronization Jump Width (Bit 1) */
#define  CAN_BTR_SJW                         ((uint32_t)0x03000000U)        /*!<Resynchronization Jump Width */
#define  CAN_BTR_LBKM                        ((uint32_t)0x40000000U)        /*!<Loop Back Mode (Debug) */
#define  CAN_BTR_SILM                        ((uint32_t)0x80000000U)        /*!<Silent Mode */

/*!<Mailbox registers */
/******************  Bit definition for CAN_TI0R register  ********************/
#define  CAN_TI0R_TXRQ                       ((uint32_t)0x00000001U)        /*!<Transmit Mailbox Request */
#define  CAN_TI0R_RTR                        ((uint32_t)0x00000002U)        /*!<Remote Transmission Request */
#define  CAN_TI0R_IDE                        ((uint32_t)0x00000004U)        /*!<Identifier Extension */
#define  CAN_TI0R_EXID                       ((uint32_t)0x001FFFF8U)        /*!<Extended Identifier */
#define  CAN_TI0R_STID                       ((uint32_t)0xFFE00000U)        /*!<Standard Identifier or Extended Identifier */

/******************  Bit definition for CAN_TDT0R register  *******************/
#define  CAN_TDT0R_DLC                       ((uint32_t)0x0000000FU)        /*!<Data Length Code */
#define  CAN_TDT0R_TGT                       ((uint32_t)0x00000100U)        /*!<Transmit Global Time */
#define  CAN_TDT0R_TIME                      ((uint32_t)0xFFFF0000U)        /*!<Message Time Stamp */

/******************  Bit definition for CAN_TDL0R register  *******************/
#define  CAN_TDL0R_DATA0                     ((uint32_t)0x000000FFU)        /*!<Data byte 0 */
#define  CAN_TDL0R_DATA1                     ((uint32_t)0x0000FF00U)        /*!<Data byte 1 */
#define  CAN_TDL0R_DATA2                     ((uint32_t)0x00FF0000U)        /*!<Data byte 2 */
#define  CAN_TDL0R_DATA3                     ((uint32_t)0xFF000000U)        /*!<Data byte 3 */

/******************  Bit definition for CAN_TDH0R register  *******************/
#define  CAN_TDH0R_DATA4                     ((uint32_t)0x000000FFU)        /*!<Data byte 4 */
#define  CAN_TDH0R_DATA5                     ((uint32_t)0x0000FF00U)        /*!<Data byte 5 */
#define  CAN_TDH0R_DATA6                     ((uint32_t)0x00FF0000U)        /*!<Data byte 6 */
#define  CAN_TDH0R_DATA7                     ((uint32_t)0xFF000000U)        /*!<Data byte 7 */

/*******************  Bit definition for CAN_TI1R register  *******************/
#define  CAN_TI1R_TXRQ                       ((uint32_t)0x00000001U)        /*!<Transmit Mailbox Request */
#define  CAN_TI1R_RTR                        ((uint32_t)0x00000002U)        /*!<Remote Transmission Request */
#define  CAN_TI1R_IDE                        ((uint32_t)0x00000004U)        /*!<Identifier Extension */
#define  CAN_TI1R_EXID                       ((uint32_t)0x001FFFF8U)        /*!<Extended Identifier */
#define  CAN_TI1R_STID                       ((uint32_t)0xFFE00000U)        /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT1R register  ******************/
#define  CAN_TDT1R_DLC                       ((uint32_t)0x0000000FU)        /*!<Data Length Code */
#define  CAN_TDT1R_TGT                       ((uint32_t)0x00000100U)        /*!<Transmit Global Time */
#define  CAN_TDT1R_TIME                      ((uint32_t)0xFFFF0000U)        /*!<Message Time Stamp */

/*******************  Bit definition for CAN_TDL1R register  ******************/
#define  CAN_TDL1R_DATA0                     ((uint32_t)0x000000FFU)        /*!<Data byte 0 */
#define  CAN_TDL1R_DATA1                     ((uint32_t)0x0000FF00U)        /*!<Data byte 1 */
#define  CAN_TDL1R_DATA2                     ((uint32_t)0x00FF0000U)        /*!<Data byte 2 */
#define  CAN_TDL1R_DATA3                     ((uint32_t)0xFF000000U)        /*!<Data byte 3 */

/*******************  Bit definition for CAN_TDH1R register  ******************/
#define  CAN_TDH1R_DATA4                     ((uint32_t)0x000000FFU)        /*!<Data byte 4 */
#define  CAN_TDH1R_DATA5                     ((uint32_t)0x0000FF00U)        /*!<Data byte 5 */
#define  CAN_TDH1R_DATA6                     ((uint32_t)0x00FF0000U)        /*!<Data byte 6 */
#define  CAN_TDH1R_DATA7                     ((uint32_t)0xFF000000U)        /*!<Data byte 7 */

/*******************  Bit definition for CAN_TI2R register  *******************/
#define  CAN_TI2R_TXRQ                       ((uint32_t)0x00000001U)        /*!<Transmit Mailbox Request */
#define  CAN_TI2R_RTR                        ((uint32_t)0x00000002U)        /*!<Remote Transmission Request */
#define  CAN_TI2R_IDE                        ((uint32_t)0x00000004U)        /*!<Identifier Extension */
#define  CAN_TI2R_EXID                       ((uint32_t)0x001FFFF8U)        /*!<Extended identifier */
#define  CAN_TI2R_STID                       ((uint32_t)0xFFE00000U)        /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT2R register  ******************/
#define  CAN_TDT2R_DLC                       ((uint32_t)0x0000000FU)        /*!<Data Length Code */
#define  CAN_TDT2R_TGT                       ((uint32_t)0x00000100U)        /*!<Transmit Global Time */
#define  CAN_TDT2R_TIME                      ((uint32_t)0xFFFF0000U)        /*!<Message Time Stamp */

/*******************  Bit definition for CAN_TDL2R register  ******************/
#define  CAN_TDL2R_DATA0                     ((uint32_t)0x000000FFU)        /*!<Data byte 0 */
#define  CAN_TDL2R_DATA1                     ((uint32_t)0x0000FF00U)        /*!<Data byte 1 */
#define  CAN_TDL2R_DATA2                     ((uint32_t)0x00FF0000U)        /*!<Data byte 2 */
#define  CAN_TDL2R_DATA3                     ((uint32_t)0xFF000000U)        /*!<Data byte 3 */

/*******************  Bit definition for CAN_TDH2R register  ******************/
#define  CAN_TDH2R_DATA4                     ((uint32_t)0x000000FFU)        /*!<Data byte 4 */
#define  CAN_TDH2R_DATA5                     ((uint32_t)0x0000FF00U)        /*!<Data byte 5 */
#define  CAN_TDH2R_DATA6                     ((uint32_t)0x00FF0000U)        /*!<Data byte 6 */
#define  CAN_TDH2R_DATA7                     ((uint32_t)0xFF000000U)        /*!<Data byte 7 */

/*******************  Bit definition for CAN_RI0R register  *******************/
#define  CAN_RI0R_RTR                        ((uint32_t)0x00000002U)        /*!<Remote Transmission Request */
#define  CAN_RI0R_IDE                        ((uint32_t)0x00000004U)        /*!<Identifier Extension */
#define  CAN_RI0R_EXID                       ((uint32_t)0x001FFFF8U)        /*!<Extended Identifier */
#define  CAN_RI0R_STID                       ((uint32_t)0xFFE00000U)        /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT0R register  ******************/
#define  CAN_RDT0R_DLC                       ((uint32_t)0x0000000FU)        /*!<Data Length Code */
#define  CAN_RDT0R_FMI                       ((uint32_t)0x0000FF00U)        /*!<Filter Match Index */
#define  CAN_RDT0R_TIME                      ((uint32_t)0xFFFF0000U)        /*!<Message Time Stamp */

/*******************  Bit definition for CAN_RDL0R register  ******************/
#define  CAN_RDL0R_DATA0                     ((uint32_t)0x000000FFU)        /*!<Data byte 0 */
#define  CAN_RDL0R_DATA1                     ((uint32_t)0x0000FF00U)        /*!<Data byte 1 */
#define  CAN_RDL0R_DATA2                     ((uint32_t)0x00FF0000U)        /*!<Data byte 2 */
#define  CAN_RDL0R_DATA3                     ((uint32_t)0xFF000000U)        /*!<Data byte 3 */

/*******************  Bit definition for CAN_RDH0R register  ******************/
#define  CAN_RDH0R_DATA4                     ((uint32_t)0x000000FFU)        /*!<Data byte 4 */
#define  CAN_RDH0R_DATA5                     ((uint32_t)0x0000FF00U)        /*!<Data byte 5 */
#define  CAN_RDH0R_DATA6                     ((uint32_t)0x00FF0000U)        /*!<Data byte 6 */
#define  CAN_RDH0R_DATA7                     ((uint32_t)0xFF000000U)        /*!<Data byte 7 */

/*******************  Bit definition for CAN_RI1R register  *******************/
#define  CAN_RI1R_RTR                        ((uint32_t)0x00000002U)        /*!<Remote Transmission Request */
#define  CAN_RI1R_IDE                        ((uint32_t)0x00000004U)        /*!<Identifier Extension */
#define  CAN_RI1R_EXID                       ((uint32_t)0x001FFFF8U)        /*!<Extended identifier */
#define  CAN_RI1R_STID                       ((uint32_t)0xFFE00000U)        /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT1R register  ******************/
#define  CAN_RDT1R_DLC                       ((uint32_t)0x0000000FU)        /*!<Data Length Code */
#define  CAN_RDT1R_FMI                       ((uint32_t)0x0000FF00U)        /*!<Filter Match Index */
#define  CAN_RDT1R_TIME                      ((uint32_t)0xFFFF0000U)        /*!<Message Time Stamp */

/*******************  Bit definition for CAN_RDL1R register  ******************/
#define  CAN_RDL1R_DATA0                     ((uint32_t)0x000000FFU)        /*!<Data byte 0 */
#define  CAN_RDL1R_DATA1                     ((uint32_t)0x0000FF00U)        /*!<Data byte 1 */
#define  CAN_RDL1R_DATA2                     ((uint32_t)0x00FF0000U)        /*!<Data byte 2 */
#define  CAN_RDL1R_DATA3                     ((uint32_t)0xFF000000U)        /*!<Data byte 3 */

/*******************  Bit definition for CAN_RDH1R register  ******************/
#define  CAN_RDH1R_DATA4                     ((uint32_t)0x000000FFU)        /*!<Data byte 4 */
#define  CAN_RDH1R_DATA5                     ((uint32_t)0x0000FF00U)        /*!<Data byte 5 */
#define  CAN_RDH1R_DATA6                     ((uint32_t)0x00FF0000U)        /*!<Data byte 6 */
#define  CAN_RDH1R_DATA7                     ((uint32_t)0xFF000000U)        /*!<Data byte 7 */

/*!<CAN filter registers */
/*******************  Bit definition for CAN_FMR register  ********************/
#define  CAN_FMR_FINIT                       ((uint8_t)0x01U)               /*!<Filter Init Mode */

/*******************  Bit definition for CAN_FM1R register  *******************/
#define  CAN_FM1R_FBM                        ((uint16_t)0x3FFFU)            /*!<Filter Mode */
#define  CAN_FM1R_FBM0                       ((uint16_t)0x0001U)            /*!<Filter Init Mode bit 0 */
#define  CAN_FM1R_FBM1                       ((uint16_t)0x0002U)            /*!<Filter Init Mode bit 1 */
#define  CAN_FM1R_FBM2                       ((uint16_t)0x0004U)            /*!<Filter Init Mode bit 2 */
#define  CAN_FM1R_FBM3                       ((uint16_t)0x0008U)            /*!<Filter Init Mode bit 3 */
#define  CAN_FM1R_FBM4                       ((uint16_t)0x0010U)            /*!<Filter Init Mode bit 4 */
#define  CAN_FM1R_FBM5                       ((uint16_t)0x0020U)            /*!<Filter Init Mode bit 5 */
#define  CAN_FM1R_FBM6                       ((uint16_t)0x0040U)            /*!<Filter Init Mode bit 6 */
#define  CAN_FM1R_FBM7                       ((uint16_t)0x0080U)            /*!<Filter Init Mode bit 7 */
#define  CAN_FM1R_FBM8                       ((uint16_t)0x0100U)            /*!<Filter Init Mode bit 8 */
#define  CAN_FM1R_FBM9                       ((uint16_t)0x0200U)            /*!<Filter Init Mode bit 9 */
#define  CAN_FM1R_FBM10                      ((uint16_t)0x0400U)            /*!<Filter Init Mode bit 10 */
#define  CAN_FM1R_FBM11                      ((uint16_t)0x0800U)            /*!<Filter Init Mode bit 11 */
#define  CAN_FM1R_FBM12                      ((uint16_t)0x1000U)            /*!<Filter Init Mode bit 12 */
#define  CAN_FM1R_FBM13                      ((uint16_t)0x2000U)            /*!<Filter Init Mode bit 13 */

/*******************  Bit definition for CAN_FS1R register  *******************/
#define  CAN_FS1R_FSC                        ((uint16_t)0x3FFFU)            /*!<Filter Scale Configuration */
#define  CAN_FS1R_FSC0                       ((uint16_t)0x0001U)            /*!<Filter Scale Configuration bit 0 */
#define  CAN_FS1R_FSC1                       ((uint16_t)0x0002U)            /*!<Filter Scale Configuration bit 1 */
#define  CAN_FS1R_FSC2                       ((uint16_t)0x0004U)            /*!<Filter Scale Configuration bit 2 */
#define  CAN_FS1R_FSC3                       ((uint16_t)0x0008U)            /*!<Filter Scale Configuration bit 3 */
#define  CAN_FS1R_FSC4                       ((uint16_t)0x0010U)            /*!<Filter Scale Configuration bit 4 */
#define  CAN_FS1R_FSC5                       ((uint16_t)0x0020U)            /*!<Filter Scale Configuration bit 5 */
#define  CAN_FS1R_FSC6                       ((uint16_t)0x0040U)            /*!<Filter Scale Configuration bit 6 */
#define  CAN_FS1R_FSC7                       ((uint16_t)0x0080U)            /*!<Filter Scale Configuration bit 7 */
#define  CAN_FS1R_FSC8                       ((uint16_t)0x0100U)            /*!<Filter Scale Configuration bit 8 */
#define  CAN_FS1R_FSC9                       ((uint16_t)0x0200U)            /*!<Filter Scale Configuration bit 9 */
#define  CAN_FS1R_FSC10                      ((uint16_t)0x0400U)            /*!<Filter Scale Configuration bit 10 */
#define  CAN_FS1R_FSC11                      ((uint16_t)0x0800U)            /*!<Filter Scale Configuration bit 11 */
#define  CAN_FS1R_FSC12                      ((uint16_t)0x1000U)            /*!<Filter Scale Configuration bit 12 */
#define  CAN_FS1R_FSC13                      ((uint16_t)0x2000U)            /*!<Filter Scale Configuration bit 13 */

/******************  Bit definition for CAN_FFA1R register  *******************/
#define  CAN_FFA1R_FFA                       ((uint16_t)0x3FFFU)            /*!<Filter FIFO Assignment */
#define  CAN_FFA1R_FFA0                      ((uint16_t)0x0001U)            /*!<Filter FIFO Assignment for Filter 0 */
#define  CAN_FFA1R_FFA1                      ((uint16_t)0x0002U)            /*!<Filter FIFO Assignment for Filter 1 */
#define  CAN_FFA1R_FFA2                      ((uint16_t)0x0004U)            /*!<Filter FIFO Assignment for Filter 2 */
#define  CAN_FFA1R_FFA3                      ((uint16_t)0x0008U)            /*!<Filter FIFO Assignment for Filter 3 */
#define  CAN_FFA1R_FFA4                      ((uint16_t)0x0010U)            /*!<Filter FIFO Assignment for Filter 4 */
#define  CAN_FFA1R_FFA5                      ((uint16_t)0x0020U)            /*!<Filter FIFO Assignment for Filter 5 */
#define  CAN_FFA1R_FFA6                      ((uint16_t)0x0040U)            /*!<Filter FIFO Assignment for Filter 6 */
#define  CAN_FFA1R_FFA7                      ((uint16_t)0x0080U)            /*!<Filter FIFO Assignment for Filter 7 */
#define  CAN_FFA1R_FFA8                      ((uint16_t)0x0100U)            /*!<Filter FIFO Assignment for Filter 8 */
#define  CAN_FFA1R_FFA9                      ((uint16_t)0x0200U)            /*!<Filter FIFO Assignment for Filter 9 */
#define  CAN_FFA1R_FFA10                     ((uint16_t)0x0400U)            /*!<Filter FIFO Assignment for Filter 10 */
#define  CAN_FFA1R_FFA11                     ((uint16_t)0x0800U)            /*!<Filter FIFO Assignment for Filter 11 */
#define  CAN_FFA1R_FFA12                     ((uint16_t)0x1000U)            /*!<Filter FIFO Assignment for Filter 12 */
#define  CAN_FFA1R_FFA13                     ((uint16_t)0x2000U)            /*!<Filter FIFO Assignment for Filter 13 */

/*******************  Bit definition for CAN_FA1R register  *******************/
#define  CAN_FA1R_FACT                       ((uint16_t)0x3FFFU)            /*!<Filter Active */
#define  CAN_FA1R_FACT0                      ((uint16_t)0x0001U)            /*!<Filter 0 Active */
#define  CAN_FA1R_FACT1                      ((uint16_t)0x0002U)            /*!<Filter 1 Active */
#define  CAN_FA1R_FACT2                      ((uint16_t)0x0004U)            /*!<Filter 2 Active */
#define  CAN_FA1R_FACT3                      ((uint16_t)0x0008U)            /*!<Filter 3 Active */
#define  CAN_FA1R_FACT4                      ((uint16_t)0x0010U)            /*!<Filter 4 Active */
#define  CAN_FA1R_FACT5                      ((uint16_t)0x0020U)            /*!<Filter 5 Active */
#define  CAN_FA1R_FACT6                      ((uint16_t)0x0040U)            /*!<Filter 6 Active */
#define  CAN_FA1R_FACT7                      ((uint16_t)0x0080U)            /*!<Filter 7 Active */
#define  CAN_FA1R_FACT8                      ((uint16_t)0x0100U)            /*!<Filter 8 Active */
#define  CAN_FA1R_FACT9                      ((uint16_t)0x0200U)            /*!<Filter 9 Active */
#define  CAN_FA1R_FACT10                     ((uint16_t)0x0400U)            /*!<Filter 10 Active */
#define  CAN_FA1R_FACT11                     ((uint16_t)0x0800U)            /*!<Filter 11 Active */
#define  CAN_FA1R_FACT12                     ((uint16_t)0x1000U)            /*!<Filter 12 Active */
#define  CAN_FA1R_FACT13                     ((uint16_t)0x2000U)            /*!<Filter 13 Active */

/*******************  Bit definition for CAN_F0R1 register  *******************/
#define  CAN_F0R1_FB0                        ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F0R1_FB1                        ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F0R1_FB2                        ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F0R1_FB3                        ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F0R1_FB4                        ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F0R1_FB5                        ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F0R1_FB6                        ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F0R1_FB7                        ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F0R1_FB8                        ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F0R1_FB9                        ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F0R1_FB10                       ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F0R1_FB11                       ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F0R1_FB12                       ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F0R1_FB13                       ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F0R1_FB14                       ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F0R1_FB15                       ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F0R1_FB16                       ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F0R1_FB17                       ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F0R1_FB18                       ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F0R1_FB19                       ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F0R1_FB20                       ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F0R1_FB21                       ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F0R1_FB22                       ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F0R1_FB23                       ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F0R1_FB24                       ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F0R1_FB25                       ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F0R1_FB26                       ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F0R1_FB27                       ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F0R1_FB28                       ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F0R1_FB29                       ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F0R1_FB30                       ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F0R1_FB31                       ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F1R1 register  *******************/
#define  CAN_F1R1_FB0                        ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F1R1_FB1                        ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F1R1_FB2                        ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F1R1_FB3                        ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F1R1_FB4                        ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F1R1_FB5                        ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F1R1_FB6                        ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F1R1_FB7                        ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F1R1_FB8                        ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F1R1_FB9                        ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F1R1_FB10                       ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F1R1_FB11                       ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F1R1_FB12                       ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F1R1_FB13                       ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F1R1_FB14                       ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F1R1_FB15                       ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F1R1_FB16                       ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F1R1_FB17                       ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F1R1_FB18                       ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F1R1_FB19                       ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F1R1_FB20                       ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F1R1_FB21                       ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F1R1_FB22                       ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F1R1_FB23                       ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F1R1_FB24                       ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F1R1_FB25                       ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F1R1_FB26                       ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F1R1_FB27                       ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F1R1_FB28                       ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F1R1_FB29                       ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F1R1_FB30                       ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F1R1_FB31                       ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F2R1 register  *******************/
#define  CAN_F2R1_FB0                        ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F2R1_FB1                        ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F2R1_FB2                        ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F2R1_FB3                        ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F2R1_FB4                        ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F2R1_FB5                        ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F2R1_FB6                        ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F2R1_FB7                        ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F2R1_FB8                        ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F2R1_FB9                        ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F2R1_FB10                       ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F2R1_FB11                       ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F2R1_FB12                       ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F2R1_FB13                       ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F2R1_FB14                       ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F2R1_FB15                       ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F2R1_FB16                       ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F2R1_FB17                       ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F2R1_FB18                       ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F2R1_FB19                       ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F2R1_FB20                       ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F2R1_FB21                       ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F2R1_FB22                       ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F2R1_FB23                       ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F2R1_FB24                       ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F2R1_FB25                       ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F2R1_FB26                       ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F2R1_FB27                       ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F2R1_FB28                       ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F2R1_FB29                       ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F2R1_FB30                       ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F2R1_FB31                       ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F3R1 register  *******************/
#define  CAN_F3R1_FB0                        ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F3R1_FB1                        ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F3R1_FB2                        ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F3R1_FB3                        ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F3R1_FB4                        ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F3R1_FB5                        ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F3R1_FB6                        ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F3R1_FB7                        ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F3R1_FB8                        ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F3R1_FB9                        ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F3R1_FB10                       ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F3R1_FB11                       ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F3R1_FB12                       ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F3R1_FB13                       ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F3R1_FB14                       ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F3R1_FB15                       ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F3R1_FB16                       ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F3R1_FB17                       ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F3R1_FB18                       ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F3R1_FB19                       ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F3R1_FB20                       ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F3R1_FB21                       ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F3R1_FB22                       ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F3R1_FB23                       ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F3R1_FB24                       ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F3R1_FB25                       ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F3R1_FB26                       ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F3R1_FB27                       ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F3R1_FB28                       ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F3R1_FB29                       ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F3R1_FB30                       ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F3R1_FB31                       ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F4R1 register  *******************/
#define  CAN_F4R1_FB0                        ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F4R1_FB1                        ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F4R1_FB2                        ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F4R1_FB3                        ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F4R1_FB4                        ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F4R1_FB5                        ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F4R1_FB6                        ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F4R1_FB7                        ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F4R1_FB8                        ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F4R1_FB9                        ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F4R1_FB10                       ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F4R1_FB11                       ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F4R1_FB12                       ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F4R1_FB13                       ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F4R1_FB14                       ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F4R1_FB15                       ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F4R1_FB16                       ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F4R1_FB17                       ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F4R1_FB18                       ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F4R1_FB19                       ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F4R1_FB20                       ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F4R1_FB21                       ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F4R1_FB22                       ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F4R1_FB23                       ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F4R1_FB24                       ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F4R1_FB25                       ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F4R1_FB26                       ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F4R1_FB27                       ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F4R1_FB28                       ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F4R1_FB29                       ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F4R1_FB30                       ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F4R1_FB31                       ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F5R1 register  *******************/
#define  CAN_F5R1_FB0                        ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F5R1_FB1                        ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F5R1_FB2                        ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F5R1_FB3                        ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F5R1_FB4                        ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F5R1_FB5                        ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F5R1_FB6                        ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F5R1_FB7                        ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F5R1_FB8                        ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F5R1_FB9                        ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F5R1_FB10                       ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F5R1_FB11                       ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F5R1_FB12                       ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F5R1_FB13                       ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F5R1_FB14                       ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F5R1_FB15                       ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F5R1_FB16                       ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F5R1_FB17                       ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F5R1_FB18                       ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F5R1_FB19                       ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F5R1_FB20                       ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F5R1_FB21                       ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F5R1_FB22                       ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F5R1_FB23                       ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F5R1_FB24                       ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F5R1_FB25                       ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F5R1_FB26                       ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F5R1_FB27                       ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F5R1_FB28                       ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F5R1_FB29                       ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F5R1_FB30                       ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F5R1_FB31                       ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F6R1 register  *******************/
#define  CAN_F6R1_FB0                        ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F6R1_FB1                        ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F6R1_FB2                        ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F6R1_FB3                        ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F6R1_FB4                        ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F6R1_FB5                        ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F6R1_FB6                        ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F6R1_FB7                        ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F6R1_FB8                        ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F6R1_FB9                        ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F6R1_FB10                       ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F6R1_FB11                       ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F6R1_FB12                       ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F6R1_FB13                       ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F6R1_FB14                       ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F6R1_FB15                       ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F6R1_FB16                       ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F6R1_FB17                       ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F6R1_FB18                       ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F6R1_FB19                       ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F6R1_FB20                       ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F6R1_FB21                       ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F6R1_FB22                       ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F6R1_FB23                       ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F6R1_FB24                       ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F6R1_FB25                       ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F6R1_FB26                       ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F6R1_FB27                       ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F6R1_FB28                       ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F6R1_FB29                       ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F6R1_FB30                       ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F6R1_FB31                       ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F7R1 register  *******************/
#define  CAN_F7R1_FB0                        ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F7R1_FB1                        ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F7R1_FB2                        ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F7R1_FB3                        ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F7R1_FB4                        ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F7R1_FB5                        ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F7R1_FB6                        ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F7R1_FB7                        ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F7R1_FB8                        ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F7R1_FB9                        ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F7R1_FB10                       ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F7R1_FB11                       ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F7R1_FB12                       ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F7R1_FB13                       ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F7R1_FB14                       ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F7R1_FB15                       ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F7R1_FB16                       ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F7R1_FB17                       ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F7R1_FB18                       ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F7R1_FB19                       ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F7R1_FB20                       ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F7R1_FB21                       ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F7R1_FB22                       ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F7R1_FB23                       ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F7R1_FB24                       ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F7R1_FB25                       ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F7R1_FB26                       ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F7R1_FB27                       ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F7R1_FB28                       ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F7R1_FB29                       ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F7R1_FB30                       ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F7R1_FB31                       ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F8R1 register  *******************/
#define  CAN_F8R1_FB0                        ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F8R1_FB1                        ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F8R1_FB2                        ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F8R1_FB3                        ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F8R1_FB4                        ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F8R1_FB5                        ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F8R1_FB6                        ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F8R1_FB7                        ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F8R1_FB8                        ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F8R1_FB9                        ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F8R1_FB10                       ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F8R1_FB11                       ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F8R1_FB12                       ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F8R1_FB13                       ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F8R1_FB14                       ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F8R1_FB15                       ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F8R1_FB16                       ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F8R1_FB17                       ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F8R1_FB18                       ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F8R1_FB19                       ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F8R1_FB20                       ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F8R1_FB21                       ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F8R1_FB22                       ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F8R1_FB23                       ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F8R1_FB24                       ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F8R1_FB25                       ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F8R1_FB26                       ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F8R1_FB27                       ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F8R1_FB28                       ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F8R1_FB29                       ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F8R1_FB30                       ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F8R1_FB31                       ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F9R1 register  *******************/
#define  CAN_F9R1_FB0                        ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F9R1_FB1                        ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F9R1_FB2                        ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F9R1_FB3                        ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F9R1_FB4                        ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F9R1_FB5                        ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F9R1_FB6                        ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F9R1_FB7                        ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F9R1_FB8                        ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F9R1_FB9                        ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F9R1_FB10                       ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F9R1_FB11                       ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F9R1_FB12                       ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F9R1_FB13                       ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F9R1_FB14                       ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F9R1_FB15                       ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F9R1_FB16                       ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F9R1_FB17                       ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F9R1_FB18                       ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F9R1_FB19                       ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F9R1_FB20                       ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F9R1_FB21                       ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F9R1_FB22                       ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F9R1_FB23                       ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F9R1_FB24                       ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F9R1_FB25                       ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F9R1_FB26                       ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F9R1_FB27                       ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F9R1_FB28                       ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F9R1_FB29                       ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F9R1_FB30                       ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F9R1_FB31                       ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F10R1 register  ******************/
#define  CAN_F10R1_FB0                       ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F10R1_FB1                       ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F10R1_FB2                       ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F10R1_FB3                       ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F10R1_FB4                       ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F10R1_FB5                       ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F10R1_FB6                       ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F10R1_FB7                       ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F10R1_FB8                       ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F10R1_FB9                       ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F10R1_FB10                      ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F10R1_FB11                      ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F10R1_FB12                      ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F10R1_FB13                      ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F10R1_FB14                      ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F10R1_FB15                      ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F10R1_FB16                      ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F10R1_FB17                      ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F10R1_FB18                      ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F10R1_FB19                      ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F10R1_FB20                      ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F10R1_FB21                      ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F10R1_FB22                      ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F10R1_FB23                      ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F10R1_FB24                      ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F10R1_FB25                      ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F10R1_FB26                      ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F10R1_FB27                      ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F10R1_FB28                      ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F10R1_FB29                      ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F10R1_FB30                      ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F10R1_FB31                      ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F11R1 register  ******************/
#define  CAN_F11R1_FB0                       ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F11R1_FB1                       ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F11R1_FB2                       ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F11R1_FB3                       ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F11R1_FB4                       ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F11R1_FB5                       ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F11R1_FB6                       ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F11R1_FB7                       ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F11R1_FB8                       ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F11R1_FB9                       ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F11R1_FB10                      ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F11R1_FB11                      ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F11R1_FB12                      ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F11R1_FB13                      ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F11R1_FB14                      ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F11R1_FB15                      ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F11R1_FB16                      ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F11R1_FB17                      ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F11R1_FB18                      ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F11R1_FB19                      ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F11R1_FB20                      ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F11R1_FB21                      ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F11R1_FB22                      ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F11R1_FB23                      ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F11R1_FB24                      ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F11R1_FB25                      ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F11R1_FB26                      ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F11R1_FB27                      ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F11R1_FB28                      ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F11R1_FB29                      ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F11R1_FB30                      ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F11R1_FB31                      ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F12R1 register  ******************/
#define  CAN_F12R1_FB0                       ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F12R1_FB1                       ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F12R1_FB2                       ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F12R1_FB3                       ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F12R1_FB4                       ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F12R1_FB5                       ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F12R1_FB6                       ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F12R1_FB7                       ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F12R1_FB8                       ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F12R1_FB9                       ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F12R1_FB10                      ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F12R1_FB11                      ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F12R1_FB12                      ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F12R1_FB13                      ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F12R1_FB14                      ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F12R1_FB15                      ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F12R1_FB16                      ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F12R1_FB17                      ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F12R1_FB18                      ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F12R1_FB19                      ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F12R1_FB20                      ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F12R1_FB21                      ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F12R1_FB22                      ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F12R1_FB23                      ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F12R1_FB24                      ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F12R1_FB25                      ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F12R1_FB26                      ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F12R1_FB27                      ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F12R1_FB28                      ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F12R1_FB29                      ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F12R1_FB30                      ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F12R1_FB31                      ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F13R1 register  ******************/
#define  CAN_F13R1_FB0                       ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F13R1_FB1                       ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F13R1_FB2                       ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F13R1_FB3                       ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F13R1_FB4                       ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F13R1_FB5                       ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F13R1_FB6                       ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F13R1_FB7                       ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F13R1_FB8                       ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F13R1_FB9                       ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F13R1_FB10                      ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F13R1_FB11                      ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F13R1_FB12                      ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F13R1_FB13                      ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F13R1_FB14                      ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F13R1_FB15                      ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F13R1_FB16                      ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F13R1_FB17                      ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F13R1_FB18                      ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F13R1_FB19                      ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F13R1_FB20                      ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F13R1_FB21                      ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F13R1_FB22                      ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F13R1_FB23                      ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F13R1_FB24                      ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F13R1_FB25                      ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F13R1_FB26                      ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F13R1_FB27                      ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F13R1_FB28                      ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F13R1_FB29                      ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F13R1_FB30                      ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F13R1_FB31                      ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F0R2 register  *******************/
#define  CAN_F0R2_FB0                        ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F0R2_FB1                        ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F0R2_FB2                        ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F0R2_FB3                        ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F0R2_FB4                        ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F0R2_FB5                        ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F0R2_FB6                        ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F0R2_FB7                        ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F0R2_FB8                        ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F0R2_FB9                        ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F0R2_FB10                       ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F0R2_FB11                       ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F0R2_FB12                       ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F0R2_FB13                       ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F0R2_FB14                       ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F0R2_FB15                       ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F0R2_FB16                       ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F0R2_FB17                       ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F0R2_FB18                       ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F0R2_FB19                       ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F0R2_FB20                       ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F0R2_FB21                       ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F0R2_FB22                       ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F0R2_FB23                       ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F0R2_FB24                       ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F0R2_FB25                       ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F0R2_FB26                       ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F0R2_FB27                       ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F0R2_FB28                       ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F0R2_FB29                       ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F0R2_FB30                       ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F0R2_FB31                       ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F1R2 register  *******************/
#define  CAN_F1R2_FB0                        ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F1R2_FB1                        ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F1R2_FB2                        ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F1R2_FB3                        ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F1R2_FB4                        ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F1R2_FB5                        ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F1R2_FB6                        ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F1R2_FB7                        ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F1R2_FB8                        ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F1R2_FB9                        ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F1R2_FB10                       ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F1R2_FB11                       ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F1R2_FB12                       ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F1R2_FB13                       ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F1R2_FB14                       ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F1R2_FB15                       ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F1R2_FB16                       ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F1R2_FB17                       ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F1R2_FB18                       ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F1R2_FB19                       ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F1R2_FB20                       ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F1R2_FB21                       ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F1R2_FB22                       ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F1R2_FB23                       ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F1R2_FB24                       ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F1R2_FB25                       ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F1R2_FB26                       ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F1R2_FB27                       ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F1R2_FB28                       ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F1R2_FB29                       ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F1R2_FB30                       ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F1R2_FB31                       ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F2R2 register  *******************/
#define  CAN_F2R2_FB0                        ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F2R2_FB1                        ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F2R2_FB2                        ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F2R2_FB3                        ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F2R2_FB4                        ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F2R2_FB5                        ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F2R2_FB6                        ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F2R2_FB7                        ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F2R2_FB8                        ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F2R2_FB9                        ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F2R2_FB10                       ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F2R2_FB11                       ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F2R2_FB12                       ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F2R2_FB13                       ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F2R2_FB14                       ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F2R2_FB15                       ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F2R2_FB16                       ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F2R2_FB17                       ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F2R2_FB18                       ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F2R2_FB19                       ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F2R2_FB20                       ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F2R2_FB21                       ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F2R2_FB22                       ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F2R2_FB23                       ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F2R2_FB24                       ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F2R2_FB25                       ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F2R2_FB26                       ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F2R2_FB27                       ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F2R2_FB28                       ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F2R2_FB29                       ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F2R2_FB30                       ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F2R2_FB31                       ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F3R2 register  *******************/
#define  CAN_F3R2_FB0                        ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F3R2_FB1                        ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F3R2_FB2                        ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F3R2_FB3                        ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F3R2_FB4                        ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F3R2_FB5                        ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F3R2_FB6                        ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F3R2_FB7                        ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F3R2_FB8                        ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F3R2_FB9                        ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F3R2_FB10                       ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F3R2_FB11                       ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F3R2_FB12                       ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F3R2_FB13                       ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F3R2_FB14                       ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F3R2_FB15                       ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F3R2_FB16                       ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F3R2_FB17                       ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F3R2_FB18                       ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F3R2_FB19                       ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F3R2_FB20                       ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F3R2_FB21                       ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F3R2_FB22                       ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F3R2_FB23                       ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F3R2_FB24                       ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F3R2_FB25                       ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F3R2_FB26                       ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F3R2_FB27                       ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F3R2_FB28                       ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F3R2_FB29                       ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F3R2_FB30                       ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F3R2_FB31                       ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F4R2 register  *******************/
#define  CAN_F4R2_FB0                        ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F4R2_FB1                        ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F4R2_FB2                        ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F4R2_FB3                        ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F4R2_FB4                        ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F4R2_FB5                        ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F4R2_FB6                        ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F4R2_FB7                        ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F4R2_FB8                        ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F4R2_FB9                        ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F4R2_FB10                       ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F4R2_FB11                       ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F4R2_FB12                       ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F4R2_FB13                       ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F4R2_FB14                       ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F4R2_FB15                       ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F4R2_FB16                       ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F4R2_FB17                       ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F4R2_FB18                       ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F4R2_FB19                       ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F4R2_FB20                       ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F4R2_FB21                       ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F4R2_FB22                       ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F4R2_FB23                       ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F4R2_FB24                       ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F4R2_FB25                       ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F4R2_FB26                       ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F4R2_FB27                       ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F4R2_FB28                       ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F4R2_FB29                       ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F4R2_FB30                       ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F4R2_FB31                       ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F5R2 register  *******************/
#define  CAN_F5R2_FB0                        ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F5R2_FB1                        ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F5R2_FB2                        ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F5R2_FB3                        ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F5R2_FB4                        ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F5R2_FB5                        ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F5R2_FB6                        ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F5R2_FB7                        ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F5R2_FB8                        ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F5R2_FB9                        ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F5R2_FB10                       ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F5R2_FB11                       ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F5R2_FB12                       ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F5R2_FB13                       ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F5R2_FB14                       ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F5R2_FB15                       ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F5R2_FB16                       ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F5R2_FB17                       ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F5R2_FB18                       ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F5R2_FB19                       ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F5R2_FB20                       ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F5R2_FB21                       ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F5R2_FB22                       ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F5R2_FB23                       ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F5R2_FB24                       ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F5R2_FB25                       ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F5R2_FB26                       ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F5R2_FB27                       ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F5R2_FB28                       ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F5R2_FB29                       ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F5R2_FB30                       ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F5R2_FB31                       ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F6R2 register  *******************/
#define  CAN_F6R2_FB0                        ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F6R2_FB1                        ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F6R2_FB2                        ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F6R2_FB3                        ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F6R2_FB4                        ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F6R2_FB5                        ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F6R2_FB6                        ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F6R2_FB7                        ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F6R2_FB8                        ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F6R2_FB9                        ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F6R2_FB10                       ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F6R2_FB11                       ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F6R2_FB12                       ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F6R2_FB13                       ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F6R2_FB14                       ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F6R2_FB15                       ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F6R2_FB16                       ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F6R2_FB17                       ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F6R2_FB18                       ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F6R2_FB19                       ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F6R2_FB20                       ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F6R2_FB21                       ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F6R2_FB22                       ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F6R2_FB23                       ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F6R2_FB24                       ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F6R2_FB25                       ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F6R2_FB26                       ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F6R2_FB27                       ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F6R2_FB28                       ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F6R2_FB29                       ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F6R2_FB30                       ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F6R2_FB31                       ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F7R2 register  *******************/
#define  CAN_F7R2_FB0                        ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F7R2_FB1                        ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F7R2_FB2                        ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F7R2_FB3                        ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F7R2_FB4                        ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F7R2_FB5                        ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F7R2_FB6                        ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F7R2_FB7                        ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F7R2_FB8                        ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F7R2_FB9                        ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F7R2_FB10                       ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F7R2_FB11                       ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F7R2_FB12                       ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F7R2_FB13                       ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F7R2_FB14                       ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F7R2_FB15                       ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F7R2_FB16                       ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F7R2_FB17                       ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F7R2_FB18                       ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F7R2_FB19                       ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F7R2_FB20                       ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F7R2_FB21                       ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F7R2_FB22                       ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F7R2_FB23                       ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F7R2_FB24                       ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F7R2_FB25                       ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F7R2_FB26                       ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F7R2_FB27                       ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F7R2_FB28                       ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F7R2_FB29                       ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F7R2_FB30                       ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F7R2_FB31                       ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F8R2 register  *******************/
#define  CAN_F8R2_FB0                        ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F8R2_FB1                        ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F8R2_FB2                        ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F8R2_FB3                        ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F8R2_FB4                        ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F8R2_FB5                        ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F8R2_FB6                        ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F8R2_FB7                        ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F8R2_FB8                        ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F8R2_FB9                        ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F8R2_FB10                       ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F8R2_FB11                       ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F8R2_FB12                       ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F8R2_FB13                       ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F8R2_FB14                       ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F8R2_FB15                       ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F8R2_FB16                       ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F8R2_FB17                       ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F8R2_FB18                       ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F8R2_FB19                       ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F8R2_FB20                       ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F8R2_FB21                       ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F8R2_FB22                       ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F8R2_FB23                       ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F8R2_FB24                       ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F8R2_FB25                       ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F8R2_FB26                       ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F8R2_FB27                       ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F8R2_FB28                       ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F8R2_FB29                       ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F8R2_FB30                       ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F8R2_FB31                       ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F9R2 register  *******************/
#define  CAN_F9R2_FB0                        ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F9R2_FB1                        ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F9R2_FB2                        ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F9R2_FB3                        ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F9R2_FB4                        ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F9R2_FB5                        ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F9R2_FB6                        ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F9R2_FB7                        ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F9R2_FB8                        ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F9R2_FB9                        ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F9R2_FB10                       ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F9R2_FB11                       ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F9R2_FB12                       ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F9R2_FB13                       ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F9R2_FB14                       ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F9R2_FB15                       ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F9R2_FB16                       ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F9R2_FB17                       ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F9R2_FB18                       ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F9R2_FB19                       ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F9R2_FB20                       ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F9R2_FB21                       ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F9R2_FB22                       ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F9R2_FB23                       ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F9R2_FB24                       ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F9R2_FB25                       ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F9R2_FB26                       ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F9R2_FB27                       ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F9R2_FB28                       ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F9R2_FB29                       ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F9R2_FB30                       ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F9R2_FB31                       ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F10R2 register  ******************/
#define  CAN_F10R2_FB0                       ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F10R2_FB1                       ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F10R2_FB2                       ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F10R2_FB3                       ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F10R2_FB4                       ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F10R2_FB5                       ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F10R2_FB6                       ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F10R2_FB7                       ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F10R2_FB8                       ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F10R2_FB9                       ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F10R2_FB10                      ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F10R2_FB11                      ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F10R2_FB12                      ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F10R2_FB13                      ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F10R2_FB14                      ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F10R2_FB15                      ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F10R2_FB16                      ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F10R2_FB17                      ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F10R2_FB18                      ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F10R2_FB19                      ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F10R2_FB20                      ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F10R2_FB21                      ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F10R2_FB22                      ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F10R2_FB23                      ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F10R2_FB24                      ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F10R2_FB25                      ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F10R2_FB26                      ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F10R2_FB27                      ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F10R2_FB28                      ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F10R2_FB29                      ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F10R2_FB30                      ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F10R2_FB31                      ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F11R2 register  ******************/
#define  CAN_F11R2_FB0                       ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F11R2_FB1                       ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F11R2_FB2                       ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F11R2_FB3                       ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F11R2_FB4                       ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F11R2_FB5                       ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F11R2_FB6                       ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F11R2_FB7                       ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F11R2_FB8                       ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F11R2_FB9                       ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F11R2_FB10                      ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F11R2_FB11                      ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F11R2_FB12                      ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F11R2_FB13                      ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F11R2_FB14                      ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F11R2_FB15                      ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F11R2_FB16                      ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F11R2_FB17                      ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F11R2_FB18                      ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F11R2_FB19                      ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F11R2_FB20                      ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F11R2_FB21                      ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F11R2_FB22                      ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F11R2_FB23                      ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F11R2_FB24                      ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F11R2_FB25                      ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F11R2_FB26                      ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F11R2_FB27                      ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F11R2_FB28                      ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F11R2_FB29                      ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F11R2_FB30                      ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F11R2_FB31                      ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F12R2 register  ******************/
#define  CAN_F12R2_FB0                       ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F12R2_FB1                       ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F12R2_FB2                       ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F12R2_FB3                       ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F12R2_FB4                       ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F12R2_FB5                       ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F12R2_FB6                       ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F12R2_FB7                       ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F12R2_FB8                       ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F12R2_FB9                       ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F12R2_FB10                      ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F12R2_FB11                      ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F12R2_FB12                      ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F12R2_FB13                      ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F12R2_FB14                      ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F12R2_FB15                      ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F12R2_FB16                      ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F12R2_FB17                      ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F12R2_FB18                      ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F12R2_FB19                      ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F12R2_FB20                      ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F12R2_FB21                      ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F12R2_FB22                      ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F12R2_FB23                      ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F12R2_FB24                      ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F12R2_FB25                      ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F12R2_FB26                      ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F12R2_FB27                      ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F12R2_FB28                      ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F12R2_FB29                      ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F12R2_FB30                      ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F12R2_FB31                      ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F13R2 register  ******************/
#define  CAN_F13R2_FB0                       ((uint32_t)0x00000001U)        /*!<Filter bit 0 */
#define  CAN_F13R2_FB1                       ((uint32_t)0x00000002U)        /*!<Filter bit 1 */
#define  CAN_F13R2_FB2                       ((uint32_t)0x00000004U)        /*!<Filter bit 2 */
#define  CAN_F13R2_FB3                       ((uint32_t)0x00000008U)        /*!<Filter bit 3 */
#define  CAN_F13R2_FB4                       ((uint32_t)0x00000010U)        /*!<Filter bit 4 */
#define  CAN_F13R2_FB5                       ((uint32_t)0x00000020U)        /*!<Filter bit 5 */
#define  CAN_F13R2_FB6                       ((uint32_t)0x00000040U)        /*!<Filter bit 6 */
#define  CAN_F13R2_FB7                       ((uint32_t)0x00000080U)        /*!<Filter bit 7 */
#define  CAN_F13R2_FB8                       ((uint32_t)0x00000100U)        /*!<Filter bit 8 */
#define  CAN_F13R2_FB9                       ((uint32_t)0x00000200U)        /*!<Filter bit 9 */
#define  CAN_F13R2_FB10                      ((uint32_t)0x00000400U)        /*!<Filter bit 10 */
#define  CAN_F13R2_FB11                      ((uint32_t)0x00000800U)        /*!<Filter bit 11 */
#define  CAN_F13R2_FB12                      ((uint32_t)0x00001000U)        /*!<Filter bit 12 */
#define  CAN_F13R2_FB13                      ((uint32_t)0x00002000U)        /*!<Filter bit 13 */
#define  CAN_F13R2_FB14                      ((uint32_t)0x00004000U)        /*!<Filter bit 14 */
#define  CAN_F13R2_FB15                      ((uint32_t)0x00008000U)        /*!<Filter bit 15 */
#define  CAN_F13R2_FB16                      ((uint32_t)0x00010000U)        /*!<Filter bit 16 */
#define  CAN_F13R2_FB17                      ((uint32_t)0x00020000U)        /*!<Filter bit 17 */
#define  CAN_F13R2_FB18                      ((uint32_t)0x00040000U)        /*!<Filter bit 18 */
#define  CAN_F13R2_FB19                      ((uint32_t)0x00080000U)        /*!<Filter bit 19 */
#define  CAN_F13R2_FB20                      ((uint32_t)0x00100000U)        /*!<Filter bit 20 */
#define  CAN_F13R2_FB21                      ((uint32_t)0x00200000U)        /*!<Filter bit 21 */
#define  CAN_F13R2_FB22                      ((uint32_t)0x00400000U)        /*!<Filter bit 22 */
#define  CAN_F13R2_FB23                      ((uint32_t)0x00800000U)        /*!<Filter bit 23 */
#define  CAN_F13R2_FB24                      ((uint32_t)0x01000000U)        /*!<Filter bit 24 */
#define  CAN_F13R2_FB25                      ((uint32_t)0x02000000U)        /*!<Filter bit 25 */
#define  CAN_F13R2_FB26                      ((uint32_t)0x04000000U)        /*!<Filter bit 26 */
#define  CAN_F13R2_FB27                      ((uint32_t)0x08000000U)        /*!<Filter bit 27 */
#define  CAN_F13R2_FB28                      ((uint32_t)0x10000000U)        /*!<Filter bit 28 */
#define  CAN_F13R2_FB29                      ((uint32_t)0x20000000U)        /*!<Filter bit 29 */
#define  CAN_F13R2_FB30                      ((uint32_t)0x40000000U)        /*!<Filter bit 30 */
#define  CAN_F13R2_FB31                      ((uint32_t)0x80000000U)        /*!<Filter bit 31 */

/******************************************************************************/
/*                                                                            */
/*                          CRC calculation unit                              */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for CRC_DR register  *********************/
#define  CRC_DR_DR                           ((uint32_t)0xFFFFFFFFU) /*!< Data register bits */

/*******************  Bit definition for CRC_IDR register  ********************/
#define  CRC_IDR_IDR                         ((uint8_t)0xFFU)        /*!< General-purpose 8-bit data register bits */

/********************  Bit definition for CRC_CR register  ********************/
#define  CRC_CR_RESET                        ((uint32_t)0x00000001U) /*!< RESET the CRC computation unit bit */
#define  CRC_CR_POLYSIZE                     ((uint32_t)0x00000018U) /*!< Polynomial size bits */
#define  CRC_CR_POLYSIZE_0                   ((uint32_t)0x00000008U) /*!< Polynomial size bit 0 */
#define  CRC_CR_POLYSIZE_1                   ((uint32_t)0x00000010U) /*!< Polynomial size bit 1 */
#define  CRC_CR_REV_IN                       ((uint32_t)0x00000060U) /*!< REV_IN Reverse Input Data bits */
#define  CRC_CR_REV_IN_0                     ((uint32_t)0x00000020U) /*!< Bit 0 */
#define  CRC_CR_REV_IN_1                     ((uint32_t)0x00000040U) /*!< Bit 1 */
#define  CRC_CR_REV_OUT                      ((uint32_t)0x00000080U) /*!< REV_OUT Reverse Output Data bits */

/*******************  Bit definition for CRC_INIT register  *******************/
#define  CRC_INIT_INIT                       ((uint32_t)0xFFFFFFFFU) /*!< Initial CRC value bits */

/*******************  Bit definition for CRC_POL register  ********************/
#define  CRC_POL_POL                         ((uint32_t)0xFFFFFFFFU) /*!< Coefficients of the polynomial */

/******************************************************************************/
/*                                                                            */
/*                          CRS Clock Recovery System                         */
/******************************************************************************/

/*******************  Bit definition for CRS_CR register  *********************/
#define  CRS_CR_SYNCOKIE                     ((uint32_t)0x00000001U) /*!< SYNC event OK interrupt enable */
#define  CRS_CR_SYNCWARNIE                   ((uint32_t)0x00000002U) /*!< SYNC warning interrupt enable */
#define  CRS_CR_ERRIE                        ((uint32_t)0x00000004U) /*!< SYNC error or trimming error interrupt enable */
#define  CRS_CR_ESYNCIE                      ((uint32_t)0x00000008U) /*!< Expected SYNC interrupt enable */
#define  CRS_CR_CEN                          ((uint32_t)0x00000020U) /*!< Frequency error counter enable */
#define  CRS_CR_AUTOTRIMEN                   ((uint32_t)0x00000040U) /*!< Automatic trimming enable */
#define  CRS_CR_SWSYNC                       ((uint32_t)0x00000080U) /*!< Generate software SYNC event */
#define  CRS_CR_TRIM                         ((uint32_t)0x00003F00U) /*!< HSI48 oscillator smooth trimming */

/*******************  Bit definition for CRS_CFGR register  *********************/
#define  CRS_CFGR_RELOAD                     ((uint32_t)0x0000FFFFU) /*!< Counter reload value */
#define  CRS_CFGR_FELIM                      ((uint32_t)0x00FF0000U) /*!< Frequency error limit */

#define  CRS_CFGR_SYNCDIV                    ((uint32_t)0x07000000U) /*!< SYNC divider */
#define  CRS_CFGR_SYNCDIV_0                  ((uint32_t)0x01000000U) /*!< SYNC divider bit 0 */
#define  CRS_CFGR_SYNCDIV_1                  ((uint32_t)0x02000000U) /*!< SYNC divider bit 1 */
#define  CRS_CFGR_SYNCDIV_2                  ((uint32_t)0x04000000U) /*!< SYNC divider bit 2 */

#define  CRS_CFGR_SYNCSRC                    ((uint32_t)0x30000000U) /*!< SYNC signal source selection */
#define  CRS_CFGR_SYNCSRC_0                  ((uint32_t)0x10000000U) /*!< SYNC signal source selection bit 0 */
#define  CRS_CFGR_SYNCSRC_1                  ((uint32_t)0x20000000U) /*!< SYNC signal source selection bit 1 */

#define  CRS_CFGR_SYNCPOL                    ((uint32_t)0x80000000U) /*!< SYNC polarity selection */
  
/*******************  Bit definition for CRS_ISR register  *********************/
#define  CRS_ISR_SYNCOKF                     ((uint32_t)0x00000001U) /*!< SYNC event OK flag */
#define  CRS_ISR_SYNCWARNF                   ((uint32_t)0x00000002U) /*!< SYNC warning flag */
#define  CRS_ISR_ERRF                        ((uint32_t)0x00000004U) /*!< Error flag */
#define  CRS_ISR_ESYNCF                      ((uint32_t)0x00000008U) /*!< Expected SYNC flag */
#define  CRS_ISR_SYNCERR                     ((uint32_t)0x00000100U) /*!< SYNC error */
#define  CRS_ISR_SYNCMISS                    ((uint32_t)0x00000200U) /*!< SYNC missed */
#define  CRS_ISR_TRIMOVF                     ((uint32_t)0x00000400U) /*!< Trimming overflow or underflow */
#define  CRS_ISR_FEDIR                       ((uint32_t)0x00008000U) /*!< Frequency error direction */
#define  CRS_ISR_FECAP                       ((uint32_t)0xFFFF0000U) /*!< Frequency error capture */

/*******************  Bit definition for CRS_ICR register  *********************/
#define  CRS_ICR_SYNCOKC                     ((uint32_t)0x00000001U) /*!< SYNC event OK clear flag */
#define  CRS_ICR_SYNCWARNC                   ((uint32_t)0x00000002U) /*!< SYNC warning clear flag */
#define  CRS_ICR_ERRC                        ((uint32_t)0x00000004U) /*!< Error clear flag */
#define  CRS_ICR_ESYNCC                      ((uint32_t)0x00000008U) /*!< Expected SYNC clear flag */

/******************************************************************************/
/*                                                                            */
/*                      Digital to Analog Converter                           */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for DAC_CR register  ********************/
#define  DAC_CR_EN1                          ((uint32_t)0x00000001U)        /*!<DAC channel1 enable */
#define  DAC_CR_TEN1                         ((uint32_t)0x00000004U)        /*!<DAC channel1 Trigger enable */

#define  DAC_CR_TSEL1                        ((uint32_t)0x00000038U)        /*!<TSEL1[2:0] (DAC channel1 Trigger selection) */
#define  DAC_CR_TSEL1_0                      ((uint32_t)0x00000008U)        /*!<Bit 0 */
#define  DAC_CR_TSEL1_1                      ((uint32_t)0x00000010U)        /*!<Bit 1 */
#define  DAC_CR_TSEL1_2                      ((uint32_t)0x00000020U)        /*!<Bit 2 */

#define  DAC_CR_WAVE1                        ((uint32_t)0x000000C0U)        /*!<WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable) */
#define  DAC_CR_WAVE1_0                      ((uint32_t)0x00000040U)        /*!<Bit 0 */
#define  DAC_CR_WAVE1_1                      ((uint32_t)0x00000080U)        /*!<Bit 1 */

#define  DAC_CR_MAMP1                        ((uint32_t)0x00000F00U)        /*!<MAMP1[3:0] (DAC channel1 Mask/Amplitude selector) */
#define  DAC_CR_MAMP1_0                      ((uint32_t)0x00000100U)        /*!<Bit 0 */
#define  DAC_CR_MAMP1_1                      ((uint32_t)0x00000200U)        /*!<Bit 1 */
#define  DAC_CR_MAMP1_2                      ((uint32_t)0x00000400U)        /*!<Bit 2 */
#define  DAC_CR_MAMP1_3                      ((uint32_t)0x00000800U)        /*!<Bit 3 */

#define  DAC_CR_DMAEN1                       ((uint32_t)0x00001000U)        /*!<DAC channel1 DMA enable */
#define  DAC_CR_DMAUDRIE1                    ((uint32_t)0x00002000U)        /*!<DAC channel 1 DMA underrun interrupt enable  >*/
#define  DAC_CR_CEN1                         ((uint32_t)0x00004000U)        /*!<DAC channel 1 calibration enable >*/

#define  DAC_CR_EN2                          ((uint32_t)0x00010000U)        /*!<DAC channel2 enable */
#define  DAC_CR_TEN2                         ((uint32_t)0x00040000U)        /*!<DAC channel2 Trigger enable */

#define  DAC_CR_TSEL2                        ((uint32_t)0x00380000U)        /*!<TSEL2[2:0] (DAC channel2 Trigger selection) */
#define  DAC_CR_TSEL2_0                      ((uint32_t)0x00080000U)        /*!<Bit 0 */
#define  DAC_CR_TSEL2_1                      ((uint32_t)0x00100000U)        /*!<Bit 1 */
#define  DAC_CR_TSEL2_2                      ((uint32_t)0x00200000U)        /*!<Bit 2 */

#define  DAC_CR_WAVE2                        ((uint32_t)0x00C00000U)        /*!<WAVE2[1:0] (DAC channel2 noise/triangle wave generation enable) */
#define  DAC_CR_WAVE2_0                      ((uint32_t)0x00400000U)        /*!<Bit 0 */
#define  DAC_CR_WAVE2_1                      ((uint32_t)0x00800000U)        /*!<Bit 1 */

#define  DAC_CR_MAMP2                        ((uint32_t)0x0F000000U)        /*!<MAMP2[3:0] (DAC channel2 Mask/Amplitude selector) */
#define  DAC_CR_MAMP2_0                      ((uint32_t)0x01000000U)        /*!<Bit 0 */
#define  DAC_CR_MAMP2_1                      ((uint32_t)0x02000000U)        /*!<Bit 1 */
#define  DAC_CR_MAMP2_2                      ((uint32_t)0x04000000U)        /*!<Bit 2 */
#define  DAC_CR_MAMP2_3                      ((uint32_t)0x08000000U)        /*!<Bit 3 */

#define  DAC_CR_DMAEN2                       ((uint32_t)0x10000000U)        /*!<DAC channel2 DMA enabled */
#define  DAC_CR_DMAUDRIE2                    ((uint32_t)0x20000000U)        /*!<DAC channel2 DMA underrun interrupt enable  >*/
#define  DAC_CR_CEN2                         ((uint32_t)0x40000000U)        /*!<DAC channel2 calibration enable >*/

/*****************  Bit definition for DAC_SWTRIGR register  ******************/
#define  DAC_SWTRIGR_SWTRIG1                 ((uint32_t)0x00000001U)        /*!<DAC channel1 software trigger */
#define  DAC_SWTRIGR_SWTRIG2                 ((uint32_t)0x00000002U)        /*!<DAC channel2 software trigger */

/*****************  Bit definition for DAC_DHR12R1 register  ******************/
#define  DAC_DHR12R1_DACC1DHR                ((uint32_t)0x00000FFFU)        /*!<DAC channel1 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12L1 register  ******************/
#define  DAC_DHR12L1_DACC1DHR                ((uint32_t)0x0000FFF0U)        /*!<DAC channel1 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8R1 register  ******************/
#define  DAC_DHR8R1_DACC1DHR                 ((uint32_t)0x000000FFU)        /*!<DAC channel1 8-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12R2 register  ******************/
#define  DAC_DHR12R2_DACC2DHR                ((uint32_t)0x00000FFFU)        /*!<DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12L2 register  ******************/
#define  DAC_DHR12L2_DACC2DHR                ((uint32_t)0x0000FFF0U)        /*!<DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8R2 register  ******************/
#define  DAC_DHR8R2_DACC2DHR                 ((uint32_t)0x000000FFU)        /*!<DAC channel2 8-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12RD register  ******************/
#define  DAC_DHR12RD_DACC1DHR                ((uint32_t)0x00000FFFU)        /*!<DAC channel1 12-bit Right aligned data */
#define  DAC_DHR12RD_DACC2DHR                ((uint32_t)0x0FFF0000U)        /*!<DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12LD register  ******************/
#define  DAC_DHR12LD_DACC1DHR                ((uint32_t)0x0000FFF0U)        /*!<DAC channel1 12-bit Left aligned data */
#define  DAC_DHR12LD_DACC2DHR                ((uint32_t)0xFFF00000U)        /*!<DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8RD register  ******************/
#define  DAC_DHR8RD_DACC1DHR                 ((uint32_t)0x000000FFU)        /*!<DAC channel1 8-bit Right aligned data */
#define  DAC_DHR8RD_DACC2DHR                 ((uint32_t)0x0000FF00U)        /*!<DAC channel2 8-bit Right aligned data */

/*******************  Bit definition for DAC_DOR1 register  *******************/
#define  DAC_DOR1_DACC1DOR                   ((uint32_t)0x00000FFFU)        /*!<DAC channel1 data output */

/*******************  Bit definition for DAC_DOR2 register  *******************/
#define  DAC_DOR2_DACC2DOR                   ((uint32_t)0x00000FFFU)        /*!<DAC channel2 data output */

/********************  Bit definition for DAC_SR register  ********************/
#define  DAC_SR_DMAUDR1                      ((uint32_t)0x00002000U)        /*!<DAC channel1 DMA underrun flag */
#define  DAC_SR_CAL_FLAG1                    ((uint32_t)0x00004000U)        /*!<DAC channel1 calibration offset status */
#define  DAC_SR_BWST1                        ((uint32_t)0x20008000U)        /*!<DAC channel1 busy writing sample time flag */

#define  DAC_SR_DMAUDR2                      ((uint32_t)0x20000000U)        /*!<DAC channel2 DMA underrun flag */
#define  DAC_SR_CAL_FLAG2                    ((uint32_t)0x40000000U)        /*!<DAC channel2 calibration offset status */
#define  DAC_SR_BWST2                        ((uint32_t)0x80000000U)        /*!<DAC channel2 busy writing sample time flag */

/*******************  Bit definition for DAC_CCR register  ********************/
#define  DAC_CCR_OTRIM1                      ((uint32_t)0x0000001FU)        /*!<DAC channel1 offset trimming value */
#define  DAC_CCR_OTRIM2                      ((uint32_t)0x001F0000U)        /*!<DAC channel2 offset trimming value */

/*******************  Bit definition for DAC_MCR register  *******************/
#define  DAC_MCR_MODE1                        ((uint32_t)0x00000007U)        /*!<MODE1[2:0] (DAC channel1 mode) */
#define  DAC_MCR_MODE1_0                      ((uint32_t)0x00000001U)        /*!<Bit 0 */
#define  DAC_MCR_MODE1_1                      ((uint32_t)0x00000002U)        /*!<Bit 1 */
#define  DAC_MCR_MODE1_2                      ((uint32_t)0x00000004U)        /*!<Bit 2 */

#define  DAC_MCR_MODE2                        ((uint32_t)0x00070000U)        /*!<MODE2[2:0] (DAC channel2 mode) */
#define  DAC_MCR_MODE2_0                      ((uint32_t)0x00010000U)        /*!<Bit 0 */
#define  DAC_MCR_MODE2_1                      ((uint32_t)0x00020000U)        /*!<Bit 1 */
#define  DAC_MCR_MODE2_2                      ((uint32_t)0x00040000U)        /*!<Bit 2 */

/******************  Bit definition for DAC_SHSR1 register  ******************/
#define  DAC_SHSR1_TSAMPLE1                   ((uint32_t)0x000003FFU)        /*!<DAC channel1 sample time */

/******************  Bit definition for DAC_SHSR2 register  ******************/
#define  DAC_SHSR2_TSAMPLE2                   ((uint32_t)0x000003FFU)        /*!<DAC channel2 sample time */

/******************  Bit definition for DAC_SHHR register  ******************/
#define  DAC_SHHR_THOLD1                      ((uint32_t)0x000003FFU)        /*!<DAC channel1 hold time */
#define  DAC_SHHR_THOLD2                      ((uint32_t)0x03FF0000U)        /*!<DAC channel2 hold time */

/******************  Bit definition for DAC_SHRR register  ******************/
#define  DAC_SHRR_TREFRESH1                   ((uint32_t)0x000000FFU)        /*!<DAC channel1 refresh time */
#define  DAC_SHRR_TREFRESH2                   ((uint32_t)0x00FF0000U)        /*!<DAC channel2 refresh time */



/******************************************************************************/
/*                                                                            */
/*                           DMA Controller (DMA)                             */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for DMA_ISR register  ********************/
#define  DMA_ISR_GIF1                        ((uint32_t)0x00000001U)        /*!< Channel 1 Global interrupt flag */
#define  DMA_ISR_TCIF1                       ((uint32_t)0x00000002U)        /*!< Channel 1 Transfer Complete flag */
#define  DMA_ISR_HTIF1                       ((uint32_t)0x00000004U)        /*!< Channel 1 Half Transfer flag */
#define  DMA_ISR_TEIF1                       ((uint32_t)0x00000008U)        /*!< Channel 1 Transfer Error flag */
#define  DMA_ISR_GIF2                        ((uint32_t)0x00000010U)        /*!< Channel 2 Global interrupt flag */
#define  DMA_ISR_TCIF2                       ((uint32_t)0x00000020U)        /*!< Channel 2 Transfer Complete flag */
#define  DMA_ISR_HTIF2                       ((uint32_t)0x00000040U)        /*!< Channel 2 Half Transfer flag */
#define  DMA_ISR_TEIF2                       ((uint32_t)0x00000080U)        /*!< Channel 2 Transfer Error flag */
#define  DMA_ISR_GIF3                        ((uint32_t)0x00000100U)        /*!< Channel 3 Global interrupt flag */
#define  DMA_ISR_TCIF3                       ((uint32_t)0x00000200U)        /*!< Channel 3 Transfer Complete flag */
#define  DMA_ISR_HTIF3                       ((uint32_t)0x00000400U)        /*!< Channel 3 Half Transfer flag */
#define  DMA_ISR_TEIF3                       ((uint32_t)0x00000800U)        /*!< Channel 3 Transfer Error flag */
#define  DMA_ISR_GIF4                        ((uint32_t)0x00001000U)        /*!< Channel 4 Global interrupt flag */
#define  DMA_ISR_TCIF4                       ((uint32_t)0x00002000U)        /*!< Channel 4 Transfer Complete flag */
#define  DMA_ISR_HTIF4                       ((uint32_t)0x00004000U)        /*!< Channel 4 Half Transfer flag */
#define  DMA_ISR_TEIF4                       ((uint32_t)0x00008000U)        /*!< Channel 4 Transfer Error flag */
#define  DMA_ISR_GIF5                        ((uint32_t)0x00010000U)        /*!< Channel 5 Global interrupt flag */
#define  DMA_ISR_TCIF5                       ((uint32_t)0x00020000U)        /*!< Channel 5 Transfer Complete flag */
#define  DMA_ISR_HTIF5                       ((uint32_t)0x00040000U)        /*!< Channel 5 Half Transfer flag */
#define  DMA_ISR_TEIF5                       ((uint32_t)0x00080000U)        /*!< Channel 5 Transfer Error flag */
#define  DMA_ISR_GIF6                        ((uint32_t)0x00100000U)        /*!< Channel 6 Global interrupt flag */
#define  DMA_ISR_TCIF6                       ((uint32_t)0x00200000U)        /*!< Channel 6 Transfer Complete flag */
#define  DMA_ISR_HTIF6                       ((uint32_t)0x00400000U)        /*!< Channel 6 Half Transfer flag */
#define  DMA_ISR_TEIF6                       ((uint32_t)0x00800000U)        /*!< Channel 6 Transfer Error flag */
#define  DMA_ISR_GIF7                        ((uint32_t)0x01000000U)        /*!< Channel 7 Global interrupt flag */
#define  DMA_ISR_TCIF7                       ((uint32_t)0x02000000U)        /*!< Channel 7 Transfer Complete flag */
#define  DMA_ISR_HTIF7                       ((uint32_t)0x04000000U)        /*!< Channel 7 Half Transfer flag */
#define  DMA_ISR_TEIF7                       ((uint32_t)0x08000000U)        /*!< Channel 7 Transfer Error flag */

/*******************  Bit definition for DMA_IFCR register  *******************/
#define  DMA_IFCR_CGIF1                      ((uint32_t)0x00000001U)        /*!< Channel 1 Global interrupt clearr */
#define  DMA_IFCR_CTCIF1                     ((uint32_t)0x00000002U)        /*!< Channel 1 Transfer Complete clear */
#define  DMA_IFCR_CHTIF1                     ((uint32_t)0x00000004U)        /*!< Channel 1 Half Transfer clear */
#define  DMA_IFCR_CTEIF1                     ((uint32_t)0x00000008U)        /*!< Channel 1 Transfer Error clear */
#define  DMA_IFCR_CGIF2                      ((uint32_t)0x00000010U)        /*!< Channel 2 Global interrupt clear */
#define  DMA_IFCR_CTCIF2                     ((uint32_t)0x00000020U)        /*!< Channel 2 Transfer Complete clear */
#define  DMA_IFCR_CHTIF2                     ((uint32_t)0x00000040U)        /*!< Channel 2 Half Transfer clear */
#define  DMA_IFCR_CTEIF2                     ((uint32_t)0x00000080U)        /*!< Channel 2 Transfer Error clear */
#define  DMA_IFCR_CGIF3                      ((uint32_t)0x00000100U)        /*!< Channel 3 Global interrupt clear */
#define  DMA_IFCR_CTCIF3                     ((uint32_t)0x00000200U)        /*!< Channel 3 Transfer Complete clear */
#define  DMA_IFCR_CHTIF3                     ((uint32_t)0x00000400U)        /*!< Channel 3 Half Transfer clear */
#define  DMA_IFCR_CTEIF3                     ((uint32_t)0x00000800U)        /*!< Channel 3 Transfer Error clear */
#define  DMA_IFCR_CGIF4                      ((uint32_t)0x00001000U)        /*!< Channel 4 Global interrupt clear */
#define  DMA_IFCR_CTCIF4                     ((uint32_t)0x00002000U)        /*!< Channel 4 Transfer Complete clear */
#define  DMA_IFCR_CHTIF4                     ((uint32_t)0x00004000U)        /*!< Channel 4 Half Transfer clear */
#define  DMA_IFCR_CTEIF4                     ((uint32_t)0x00008000U)        /*!< Channel 4 Transfer Error clear */
#define  DMA_IFCR_CGIF5                      ((uint32_t)0x00010000U)        /*!< Channel 5 Global interrupt clear */
#define  DMA_IFCR_CTCIF5                     ((uint32_t)0x00020000U)        /*!< Channel 5 Transfer Complete clear */
#define  DMA_IFCR_CHTIF5                     ((uint32_t)0x00040000U)        /*!< Channel 5 Half Transfer clear */
#define  DMA_IFCR_CTEIF5                     ((uint32_t)0x00080000U)        /*!< Channel 5 Transfer Error clear */
#define  DMA_IFCR_CGIF6                      ((uint32_t)0x00100000U)        /*!< Channel 6 Global interrupt clear */
#define  DMA_IFCR_CTCIF6                     ((uint32_t)0x00200000U)        /*!< Channel 6 Transfer Complete clear */
#define  DMA_IFCR_CHTIF6                     ((uint32_t)0x00400000U)        /*!< Channel 6 Half Transfer clear */
#define  DMA_IFCR_CTEIF6                     ((uint32_t)0x00800000U)        /*!< Channel 6 Transfer Error clear */
#define  DMA_IFCR_CGIF7                      ((uint32_t)0x01000000U)        /*!< Channel 7 Global interrupt clear */
#define  DMA_IFCR_CTCIF7                     ((uint32_t)0x02000000U)        /*!< Channel 7 Transfer Complete clear */
#define  DMA_IFCR_CHTIF7                     ((uint32_t)0x04000000U)        /*!< Channel 7 Half Transfer clear */
#define  DMA_IFCR_CTEIF7                     ((uint32_t)0x08000000U)        /*!< Channel 7 Transfer Error clear */

/*******************  Bit definition for DMA_CCR register  ********************/
#define  DMA_CCR_EN                          ((uint32_t)0x00000001U)        /*!< Channel enable                      */
#define  DMA_CCR_TCIE                        ((uint32_t)0x00000002U)        /*!< Transfer complete interrupt enable  */
#define  DMA_CCR_HTIE                        ((uint32_t)0x00000004U)        /*!< Half Transfer interrupt enable      */
#define  DMA_CCR_TEIE                        ((uint32_t)0x00000008U)        /*!< Transfer error interrupt enable     */
#define  DMA_CCR_DIR                         ((uint32_t)0x00000010U)        /*!< Data transfer direction             */
#define  DMA_CCR_CIRC                        ((uint32_t)0x00000020U)        /*!< Circular mode                       */
#define  DMA_CCR_PINC                        ((uint32_t)0x00000040U)        /*!< Peripheral increment mode           */
#define  DMA_CCR_MINC                        ((uint32_t)0x00000080U)        /*!< Memory increment mode               */

#define  DMA_CCR_PSIZE                       ((uint32_t)0x00000300U)        /*!< PSIZE[1:0] bits (Peripheral size)   */
#define  DMA_CCR_PSIZE_0                     ((uint32_t)0x00000100U)        /*!< Bit 0                               */
#define  DMA_CCR_PSIZE_1                     ((uint32_t)0x00000200U)        /*!< Bit 1                               */

#define  DMA_CCR_MSIZE                       ((uint32_t)0x00000C00U)        /*!< MSIZE[1:0] bits (Memory size)       */
#define  DMA_CCR_MSIZE_0                     ((uint32_t)0x00000400U)        /*!< Bit 0                               */
#define  DMA_CCR_MSIZE_1                     ((uint32_t)0x00000800U)        /*!< Bit 1                               */

#define  DMA_CCR_PL                          ((uint32_t)0x00003000U)        /*!< PL[1:0] bits(Channel Priority level)*/
#define  DMA_CCR_PL_0                        ((uint32_t)0x00001000U)        /*!< Bit 0                               */
#define  DMA_CCR_PL_1                        ((uint32_t)0x00002000U)        /*!< Bit 1                               */

#define  DMA_CCR_MEM2MEM                     ((uint32_t)0x00004000U)        /*!< Memory to memory mode               */

/******************  Bit definition for DMA_CNDTR register  *******************/
#define  DMA_CNDTR_NDT                       ((uint32_t)0x0000FFFFU)        /*!< Number of data to Transfer          */

/******************  Bit definition for DMA_CPAR register  ********************/
#define  DMA_CPAR_PA                         ((uint32_t)0xFFFFFFFFU)        /*!< Peripheral Address                  */

/******************  Bit definition for DMA_CMAR register  ********************/
#define  DMA_CMAR_MA                         ((uint32_t)0xFFFFFFFFU)        /*!< Memory Address                      */


/*******************  Bit definition for DMA_CSELR register  *******************/
#define  DMA_CSELR_C1S                          ((uint32_t)0x0000000FU)          /*!< Channel 1 Selection */
#define  DMA_CSELR_C2S                          ((uint32_t)0x000000F0U)          /*!< Channel 2 Selection */
#define  DMA_CSELR_C3S                          ((uint32_t)0x00000F00U)          /*!< Channel 3 Selection */
#define  DMA_CSELR_C4S                          ((uint32_t)0x0000F000U)          /*!< Channel 4 Selection */
#define  DMA_CSELR_C5S                          ((uint32_t)0x000F0000U)          /*!< Channel 5 Selection */
#define  DMA_CSELR_C6S                          ((uint32_t)0x00F00000U)          /*!< Channel 6 Selection */
#define  DMA_CSELR_C7S                          ((uint32_t)0x0F000000U)          /*!< Channel 7 Selection */


/******************************************************************************/
/*                                                                            */
/*                    External Interrupt/Event Controller                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for EXTI_IMR1 register  ******************/
#define  EXTI_IMR1_IM0                       ((uint32_t)0x00000001U)        /*!< Interrupt Mask on line 0 */
#define  EXTI_IMR1_IM1                       ((uint32_t)0x00000002U)        /*!< Interrupt Mask on line 1 */
#define  EXTI_IMR1_IM2                       ((uint32_t)0x00000004U)        /*!< Interrupt Mask on line 2 */
#define  EXTI_IMR1_IM3                       ((uint32_t)0x00000008U)        /*!< Interrupt Mask on line 3 */
#define  EXTI_IMR1_IM4                       ((uint32_t)0x00000010U)        /*!< Interrupt Mask on line 4 */
#define  EXTI_IMR1_IM5                       ((uint32_t)0x00000020U)        /*!< Interrupt Mask on line 5 */
#define  EXTI_IMR1_IM6                       ((uint32_t)0x00000040U)        /*!< Interrupt Mask on line 6 */
#define  EXTI_IMR1_IM7                       ((uint32_t)0x00000080U)        /*!< Interrupt Mask on line 7 */
#define  EXTI_IMR1_IM8                       ((uint32_t)0x00000100U)        /*!< Interrupt Mask on line 8 */
#define  EXTI_IMR1_IM9                       ((uint32_t)0x00000200U)        /*!< Interrupt Mask on line 9 */
#define  EXTI_IMR1_IM10                      ((uint32_t)0x00000400U)        /*!< Interrupt Mask on line 10 */
#define  EXTI_IMR1_IM11                      ((uint32_t)0x00000800U)        /*!< Interrupt Mask on line 11 */
#define  EXTI_IMR1_IM12                      ((uint32_t)0x00001000U)        /*!< Interrupt Mask on line 12 */
#define  EXTI_IMR1_IM13                      ((uint32_t)0x00002000U)        /*!< Interrupt Mask on line 13 */
#define  EXTI_IMR1_IM14                      ((uint32_t)0x00004000U)        /*!< Interrupt Mask on line 14 */
#define  EXTI_IMR1_IM15                      ((uint32_t)0x00008000U)        /*!< Interrupt Mask on line 15 */
#define  EXTI_IMR1_IM16                      ((uint32_t)0x00010000U)        /*!< Interrupt Mask on line 16 */
#define  EXTI_IMR1_IM17                      ((uint32_t)0x00020000U)        /*!< Interrupt Mask on line 17 */
#define  EXTI_IMR1_IM18                      ((uint32_t)0x00040000U)        /*!< Interrupt Mask on line 18 */
#define  EXTI_IMR1_IM19                      ((uint32_t)0x00080000U)        /*!< Interrupt Mask on line 19 */
#define  EXTI_IMR1_IM20                      ((uint32_t)0x00100000U)        /*!< Interrupt Mask on line 20 */
#define  EXTI_IMR1_IM21                      ((uint32_t)0x00200000U)        /*!< Interrupt Mask on line 21 */
#define  EXTI_IMR1_IM22                      ((uint32_t)0x00400000U)        /*!< Interrupt Mask on line 22 */
#define  EXTI_IMR1_IM23                      ((uint32_t)0x00800000U)        /*!< Interrupt Mask on line 23 */
#define  EXTI_IMR1_IM24                      ((uint32_t)0x01000000U)        /*!< Interrupt Mask on line 24 */
#define  EXTI_IMR1_IM25                      ((uint32_t)0x02000000U)        /*!< Interrupt Mask on line 25 */
#define  EXTI_IMR1_IM26                      ((uint32_t)0x04000000U)        /*!< Interrupt Mask on line 26 */
#define  EXTI_IMR1_IM27                      ((uint32_t)0x08000000U)        /*!< Interrupt Mask on line 27 */
#define  EXTI_IMR1_IM28                      ((uint32_t)0x10000000U)        /*!< Interrupt Mask on line 28 */
#define  EXTI_IMR1_IM31                      ((uint32_t)0x80000000U)        /*!< Interrupt Mask on line 31 */
#define  EXTI_IMR1_IM                        ((uint32_t)0x3FFFFFFFU)        /*!< Interrupt Mask All */

/*******************  Bit definition for EXTI_EMR1 register  ******************/
#define  EXTI_EMR1_EM0                       ((uint32_t)0x00000001U)        /*!< Event Mask on line 0 */
#define  EXTI_EMR1_EM1                       ((uint32_t)0x00000002U)        /*!< Event Mask on line 1 */
#define  EXTI_EMR1_EM2                       ((uint32_t)0x00000004U)        /*!< Event Mask on line 2 */
#define  EXTI_EMR1_EM3                       ((uint32_t)0x00000008U)        /*!< Event Mask on line 3 */
#define  EXTI_EMR1_EM4                       ((uint32_t)0x00000010U)        /*!< Event Mask on line 4 */
#define  EXTI_EMR1_EM5                       ((uint32_t)0x00000020U)        /*!< Event Mask on line 5 */
#define  EXTI_EMR1_EM6                       ((uint32_t)0x00000040U)        /*!< Event Mask on line 6 */
#define  EXTI_EMR1_EM7                       ((uint32_t)0x00000080U)        /*!< Event Mask on line 7 */
#define  EXTI_EMR1_EM8                       ((uint32_t)0x00000100U)        /*!< Event Mask on line 8 */
#define  EXTI_EMR1_EM9                       ((uint32_t)0x00000200U)        /*!< Event Mask on line 9 */
#define  EXTI_EMR1_EM10                      ((uint32_t)0x00000400U)        /*!< Event Mask on line 10 */
#define  EXTI_EMR1_EM11                      ((uint32_t)0x00000800U)        /*!< Event Mask on line 11 */
#define  EXTI_EMR1_EM12                      ((uint32_t)0x00001000U)        /*!< Event Mask on line 12 */
#define  EXTI_EMR1_EM13                      ((uint32_t)0x00002000U)        /*!< Event Mask on line 13 */
#define  EXTI_EMR1_EM14                      ((uint32_t)0x00004000U)        /*!< Event Mask on line 14 */
#define  EXTI_EMR1_EM15                      ((uint32_t)0x00008000U)        /*!< Event Mask on line 15 */
#define  EXTI_EMR1_EM16                      ((uint32_t)0x00010000U)        /*!< Event Mask on line 16 */
#define  EXTI_EMR1_EM17                      ((uint32_t)0x00020000U)        /*!< Event Mask on line 17 */
#define  EXTI_EMR1_EM18                      ((uint32_t)0x00040000U)        /*!< Event Mask on line 18 */
#define  EXTI_EMR1_EM19                      ((uint32_t)0x00080000U)        /*!< Event Mask on line 19 */
#define  EXTI_EMR1_EM20                      ((uint32_t)0x00100000U)        /*!< Event Mask on line 20 */
#define  EXTI_EMR1_EM21                      ((uint32_t)0x00200000U)        /*!< Event Mask on line 21 */
#define  EXTI_EMR1_EM22                      ((uint32_t)0x00400000U)        /*!< Event Mask on line 22 */
#define  EXTI_EMR1_EM23                      ((uint32_t)0x00800000U)        /*!< Event Mask on line 23 */
#define  EXTI_EMR1_EM24                      ((uint32_t)0x01000000U)        /*!< Event Mask on line 24 */
#define  EXTI_EMR1_EM25                      ((uint32_t)0x02000000U)        /*!< Event Mask on line 25 */
#define  EXTI_EMR1_EM26                      ((uint32_t)0x04000000U)        /*!< Event Mask on line 26 */
#define  EXTI_EMR1_EM27                      ((uint32_t)0x08000000U)        /*!< Event Mask on line 27 */
#define  EXTI_EMR1_EM28                      ((uint32_t)0x10000000U)        /*!< Event Mask on line 28 */
#define  EXTI_EMR1_EM31                      ((uint32_t)0x80000000U)        /*!< Event Mask on line 31 */

/******************  Bit definition for EXTI_RTSR1 register  ******************/
#define  EXTI_RTSR1_RT0                      ((uint32_t)0x00000001U)        /*!< Rising trigger event configuration bit of line 0 */
#define  EXTI_RTSR1_RT1                      ((uint32_t)0x00000002U)        /*!< Rising trigger event configuration bit of line 1 */
#define  EXTI_RTSR1_RT2                      ((uint32_t)0x00000004U)        /*!< Rising trigger event configuration bit of line 2 */
#define  EXTI_RTSR1_RT3                      ((uint32_t)0x00000008U)        /*!< Rising trigger event configuration bit of line 3 */
#define  EXTI_RTSR1_RT4                      ((uint32_t)0x00000010U)        /*!< Rising trigger event configuration bit of line 4 */
#define  EXTI_RTSR1_RT5                      ((uint32_t)0x00000020U)        /*!< Rising trigger event configuration bit of line 5 */
#define  EXTI_RTSR1_RT6                      ((uint32_t)0x00000040U)        /*!< Rising trigger event configuration bit of line 6 */
#define  EXTI_RTSR1_RT7                      ((uint32_t)0x00000080U)        /*!< Rising trigger event configuration bit of line 7 */
#define  EXTI_RTSR1_RT8                      ((uint32_t)0x00000100U)        /*!< Rising trigger event configuration bit of line 8 */
#define  EXTI_RTSR1_RT9                      ((uint32_t)0x00000200U)        /*!< Rising trigger event configuration bit of line 9 */
#define  EXTI_RTSR1_RT10                     ((uint32_t)0x00000400U)        /*!< Rising trigger event configuration bit of line 10 */
#define  EXTI_RTSR1_RT11                     ((uint32_t)0x00000800U)        /*!< Rising trigger event configuration bit of line 11 */
#define  EXTI_RTSR1_RT12                     ((uint32_t)0x00001000U)        /*!< Rising trigger event configuration bit of line 12 */
#define  EXTI_RTSR1_RT13                     ((uint32_t)0x00002000U)        /*!< Rising trigger event configuration bit of line 13 */
#define  EXTI_RTSR1_RT14                     ((uint32_t)0x00004000U)        /*!< Rising trigger event configuration bit of line 14 */
#define  EXTI_RTSR1_RT15                     ((uint32_t)0x00008000U)        /*!< Rising trigger event configuration bit of line 15 */
#define  EXTI_RTSR1_RT16                     ((uint32_t)0x00010000U)        /*!< Rising trigger event configuration bit of line 16 */
#define  EXTI_RTSR1_RT18                     ((uint32_t)0x00040000U)        /*!< Rising trigger event configuration bit of line 18 */
#define  EXTI_RTSR1_RT19                     ((uint32_t)0x00080000U)        /*!< Rising trigger event configuration bit of line 19 */
#define  EXTI_RTSR1_RT20                     ((uint32_t)0x00100000U)        /*!< Rising trigger event configuration bit of line 20 */
#define  EXTI_RTSR1_RT21                     ((uint32_t)0x00200000U)        /*!< Rising trigger event configuration bit of line 21 */
#define  EXTI_RTSR1_RT22                     ((uint32_t)0x00400000U)        /*!< Rising trigger event configuration bit of line 22 */

/******************  Bit definition for EXTI_FTSR1 register  ******************/
#define  EXTI_FTSR1_FT0                      ((uint32_t)0x00000001U)        /*!< Falling trigger event configuration bit of line 0 */
#define  EXTI_FTSR1_FT1                      ((uint32_t)0x00000002U)        /*!< Falling trigger event configuration bit of line 1 */
#define  EXTI_FTSR1_FT2                      ((uint32_t)0x00000004U)        /*!< Falling trigger event configuration bit of line 2 */
#define  EXTI_FTSR1_FT3                      ((uint32_t)0x00000008U)        /*!< Falling trigger event configuration bit of line 3 */
#define  EXTI_FTSR1_FT4                      ((uint32_t)0x00000010U)        /*!< Falling trigger event configuration bit of line 4 */
#define  EXTI_FTSR1_FT5                      ((uint32_t)0x00000020U)        /*!< Falling trigger event configuration bit of line 5 */
#define  EXTI_FTSR1_FT6                      ((uint32_t)0x00000040U)        /*!< Falling trigger event configuration bit of line 6 */
#define  EXTI_FTSR1_FT7                      ((uint32_t)0x00000080U)        /*!< Falling trigger event configuration bit of line 7 */
#define  EXTI_FTSR1_FT8                      ((uint32_t)0x00000100U)        /*!< Falling trigger event configuration bit of line 8 */
#define  EXTI_FTSR1_FT9                      ((uint32_t)0x00000200U)        /*!< Falling trigger event configuration bit of line 9 */
#define  EXTI_FTSR1_FT10                     ((uint32_t)0x00000400U)        /*!< Falling trigger event configuration bit of line 10 */
#define  EXTI_FTSR1_FT11                     ((uint32_t)0x00000800U)        /*!< Falling trigger event configuration bit of line 11 */
#define  EXTI_FTSR1_FT12                     ((uint32_t)0x00001000U)        /*!< Falling trigger event configuration bit of line 12 */
#define  EXTI_FTSR1_FT13                     ((uint32_t)0x00002000U)        /*!< Falling trigger event configuration bit of line 13 */
#define  EXTI_FTSR1_FT14                     ((uint32_t)0x00004000U)        /*!< Falling trigger event configuration bit of line 14 */
#define  EXTI_FTSR1_FT15                     ((uint32_t)0x00008000U)        /*!< Falling trigger event configuration bit of line 15 */
#define  EXTI_FTSR1_FT16                     ((uint32_t)0x00010000U)        /*!< Falling trigger event configuration bit of line 16 */
#define  EXTI_FTSR1_FT18                     ((uint32_t)0x00040000U)        /*!< Falling trigger event configuration bit of line 18 */
#define  EXTI_FTSR1_FT19                     ((uint32_t)0x00080000U)        /*!< Falling trigger event configuration bit of line 19 */
#define  EXTI_FTSR1_FT20                     ((uint32_t)0x00100000U)        /*!< Falling trigger event configuration bit of line 20 */
#define  EXTI_FTSR1_FT21                     ((uint32_t)0x00200000U)        /*!< Falling trigger event configuration bit of line 21 */
#define  EXTI_FTSR1_FT22                     ((uint32_t)0x00400000U)        /*!< Falling trigger event configuration bit of line 22 */

/******************  Bit definition for EXTI_SWIER1 register  *****************/
#define  EXTI_SWIER1_SWI0                    ((uint32_t)0x00000001U)        /*!< Software Interrupt on line 0 */
#define  EXTI_SWIER1_SWI1                    ((uint32_t)0x00000002U)        /*!< Software Interrupt on line 1 */
#define  EXTI_SWIER1_SWI2                    ((uint32_t)0x00000004U)        /*!< Software Interrupt on line 2 */
#define  EXTI_SWIER1_SWI3                    ((uint32_t)0x00000008U)        /*!< Software Interrupt on line 3 */
#define  EXTI_SWIER1_SWI4                    ((uint32_t)0x00000010U)        /*!< Software Interrupt on line 4 */
#define  EXTI_SWIER1_SWI5                    ((uint32_t)0x00000020U)        /*!< Software Interrupt on line 5 */
#define  EXTI_SWIER1_SWI6                    ((uint32_t)0x00000040U)        /*!< Software Interrupt on line 6 */
#define  EXTI_SWIER1_SWI7                    ((uint32_t)0x00000080U)        /*!< Software Interrupt on line 7 */
#define  EXTI_SWIER1_SWI8                    ((uint32_t)0x00000100U)        /*!< Software Interrupt on line 8 */
#define  EXTI_SWIER1_SWI9                    ((uint32_t)0x00000200U)        /*!< Software Interrupt on line 9 */
#define  EXTI_SWIER1_SWI10                   ((uint32_t)0x00000400U)        /*!< Software Interrupt on line 10 */
#define  EXTI_SWIER1_SWI11                   ((uint32_t)0x00000800U)        /*!< Software Interrupt on line 11 */
#define  EXTI_SWIER1_SWI12                   ((uint32_t)0x00001000U)        /*!< Software Interrupt on line 12 */
#define  EXTI_SWIER1_SWI13                   ((uint32_t)0x00002000U)        /*!< Software Interrupt on line 13 */
#define  EXTI_SWIER1_SWI14                   ((uint32_t)0x00004000U)        /*!< Software Interrupt on line 14 */
#define  EXTI_SWIER1_SWI15                   ((uint32_t)0x00008000U)        /*!< Software Interrupt on line 15 */
#define  EXTI_SWIER1_SWI16                   ((uint32_t)0x00010000U)        /*!< Software Interrupt on line 16 */
#define  EXTI_SWIER1_SWI18                   ((uint32_t)0x00040000U)        /*!< Software Interrupt on line 18 */
#define  EXTI_SWIER1_SWI19                   ((uint32_t)0x00080000U)        /*!< Software Interrupt on line 19 */
#define  EXTI_SWIER1_SWI20                   ((uint32_t)0x00100000U)        /*!< Software Interrupt on line 20 */
#define  EXTI_SWIER1_SWI21                   ((uint32_t)0x00200000U)        /*!< Software Interrupt on line 21 */
#define  EXTI_SWIER1_SWI22                   ((uint32_t)0x00400000U)        /*!< Software Interrupt on line 22 */

/*******************  Bit definition for EXTI_PR1 register  *******************/
#define  EXTI_PR1_PIF0                       ((uint32_t)0x00000001U)        /*!< Pending bit for line 0 */
#define  EXTI_PR1_PIF1                       ((uint32_t)0x00000002U)        /*!< Pending bit for line 1 */
#define  EXTI_PR1_PIF2                       ((uint32_t)0x00000004U)        /*!< Pending bit for line 2 */
#define  EXTI_PR1_PIF3                       ((uint32_t)0x00000008U)        /*!< Pending bit for line 3 */
#define  EXTI_PR1_PIF4                       ((uint32_t)0x00000010U)        /*!< Pending bit for line 4 */
#define  EXTI_PR1_PIF5                       ((uint32_t)0x00000020U)        /*!< Pending bit for line 5 */
#define  EXTI_PR1_PIF6                       ((uint32_t)0x00000040U)        /*!< Pending bit for line 6 */
#define  EXTI_PR1_PIF7                       ((uint32_t)0x00000080U)        /*!< Pending bit for line 7 */
#define  EXTI_PR1_PIF8                       ((uint32_t)0x00000100U)        /*!< Pending bit for line 8 */
#define  EXTI_PR1_PIF9                       ((uint32_t)0x00000200U)        /*!< Pending bit for line 9 */
#define  EXTI_PR1_PIF10                      ((uint32_t)0x00000400U)        /*!< Pending bit for line 10 */
#define  EXTI_PR1_PIF11                      ((uint32_t)0x00000800U)        /*!< Pending bit for line 11 */
#define  EXTI_PR1_PIF12                      ((uint32_t)0x00001000U)        /*!< Pending bit for line 12 */
#define  EXTI_PR1_PIF13                      ((uint32_t)0x00002000U)        /*!< Pending bit for line 13 */
#define  EXTI_PR1_PIF14                      ((uint32_t)0x00004000U)        /*!< Pending bit for line 14 */
#define  EXTI_PR1_PIF15                      ((uint32_t)0x00008000U)        /*!< Pending bit for line 15 */
#define  EXTI_PR1_PIF16                      ((uint32_t)0x00010000U)        /*!< Pending bit for line 16 */
#define  EXTI_PR1_PIF18                      ((uint32_t)0x00040000U)        /*!< Pending bit for line 18 */
#define  EXTI_PR1_PIF19                      ((uint32_t)0x00080000U)        /*!< Pending bit for line 19 */
#define  EXTI_PR1_PIF20                      ((uint32_t)0x00100000U)        /*!< Pending bit for line 20 */
#define  EXTI_PR1_PIF21                      ((uint32_t)0x00200000U)        /*!< Pending bit for line 21 */
#define  EXTI_PR1_PIF22                      ((uint32_t)0x00400000U)        /*!< Pending bit for line 22 */

/*******************  Bit definition for EXTI_IMR2 register  ******************/
#define  EXTI_IMR2_IM32                      ((uint32_t)0x00000001U)        /*!< Interrupt Mask on line 32 */
#define  EXTI_IMR2_IM33                      ((uint32_t)0x00000002U)        /*!< Interrupt Mask on line 33 */
#define  EXTI_IMR2_IM34                      ((uint32_t)0x00000004U)        /*!< Interrupt Mask on line 34 */
#define  EXTI_IMR2_IM35                      ((uint32_t)0x00000008U)        /*!< Interrupt Mask on line 35 */
#define  EXTI_IMR2_IM36                      ((uint32_t)0x00000010U)        /*!< Interrupt Mask on line 36 */
#define  EXTI_IMR2_IM37                      ((uint32_t)0x00000020U)        /*!< Interrupt Mask on line 37 */
#define  EXTI_IMR2_IM38                      ((uint32_t)0x00000040U)        /*!< Interrupt Mask on line 38 */
#define  EXTI_IMR2_IM39                      ((uint32_t)0x00000080U)        /*!< Interrupt Mask on line 39 */
#define  EXTI_IMR2_IM                        ((uint32_t)0x000000FFU)        /*!< Interrupt Mask on line 39 */

/*******************  Bit definition for EXTI_EMR2 register  ******************/
#define  EXTI_EMR2_EM32                      ((uint32_t)0x00000001U)        /*!< Event Mask on line 32 */
#define  EXTI_EMR2_EM33                      ((uint32_t)0x00000002U)        /*!< Event Mask on line 33 */
#define  EXTI_EMR2_EM34                      ((uint32_t)0x00000004U)        /*!< Event Mask on line 34 */
#define  EXTI_EMR2_EM35                      ((uint32_t)0x00000008U)        /*!< Event Mask on line 35 */
#define  EXTI_EMR2_EM36                      ((uint32_t)0x00000010U)        /*!< Event Mask on line 36 */
#define  EXTI_EMR2_EM37                      ((uint32_t)0x00000020U)        /*!< Event Mask on line 37 */
#define  EXTI_EMR2_EM38                      ((uint32_t)0x00000040U)        /*!< Event Mask on line 38 */
#define  EXTI_EMR2_EM39                      ((uint32_t)0x00000080U)        /*!< Event Mask on line 39 */

/******************  Bit definition for EXTI_RTSR2 register  ******************/
#define  EXTI_RTSR2_RT35                     ((uint32_t)0x00000008U)        /*!< Rising trigger event configuration bit of line 35 */
#define  EXTI_RTSR2_RT36                     ((uint32_t)0x00000010U)        /*!< Rising trigger event configuration bit of line 36 */
#define  EXTI_RTSR2_RT37                     ((uint32_t)0x00000020U)        /*!< Rising trigger event configuration bit of line 37 */
#define  EXTI_RTSR2_RT38                     ((uint32_t)0x00000040U)        /*!< Rising trigger event configuration bit of line 38 */

/******************  Bit definition for EXTI_FTSR2 register  ******************/
#define  EXTI_FTSR2_FT35                     ((uint32_t)0x00000008U)        /*!< Falling trigger event configuration bit of line 35 */
#define  EXTI_FTSR2_FT36                     ((uint32_t)0x00000010U)        /*!< Falling trigger event configuration bit of line 36 */
#define  EXTI_FTSR2_FT37                     ((uint32_t)0x00000020U)        /*!< Falling trigger event configuration bit of line 37 */
#define  EXTI_FTSR2_FT38                     ((uint32_t)0x00000040U)        /*!< Falling trigger event configuration bit of line 38 */

/******************  Bit definition for EXTI_SWIER2 register  *****************/
#define  EXTI_SWIER2_SWI35                   ((uint32_t)0x00000008U)        /*!< Software Interrupt on line 35 */
#define  EXTI_SWIER2_SWI36                   ((uint32_t)0x00000010U)        /*!< Software Interrupt on line 36 */
#define  EXTI_SWIER2_SWI37                   ((uint32_t)0x00000020U)        /*!< Software Interrupt on line 37 */
#define  EXTI_SWIER2_SWI38                   ((uint32_t)0x00000040U)        /*!< Software Interrupt on line 38 */

/*******************  Bit definition for EXTI_PR2 register  *******************/
#define  EXTI_PR2_PIF35                       ((uint32_t)0x00000008U)        /*!< Pending bit for line 35 */
#define  EXTI_PR2_PIF36                       ((uint32_t)0x00000010U)        /*!< Pending bit for line 36 */
#define  EXTI_PR2_PIF37                       ((uint32_t)0x00000020U)        /*!< Pending bit for line 37 */
#define  EXTI_PR2_PIF38                       ((uint32_t)0x00000040U)        /*!< Pending bit for line 38 */


/******************************************************************************/
/*                                                                            */
/*                                    FLASH                                   */
/*                                                                            */
/******************************************************************************/
/*******************  Bits definition for FLASH_ACR register  *****************/
#define FLASH_ACR_LATENCY                    ((uint32_t)0x00000007U)
#define FLASH_ACR_LATENCY_0WS                ((uint32_t)0x00000000U)
#define FLASH_ACR_LATENCY_1WS                ((uint32_t)0x00000001U)
#define FLASH_ACR_LATENCY_2WS                ((uint32_t)0x00000002U)
#define FLASH_ACR_LATENCY_3WS                ((uint32_t)0x00000003U)
#define FLASH_ACR_LATENCY_4WS                ((uint32_t)0x00000004U)
#define FLASH_ACR_PRFTEN                     ((uint32_t)0x00000100U)
#define FLASH_ACR_ICEN                       ((uint32_t)0x00000200U)
#define FLASH_ACR_DCEN                       ((uint32_t)0x00000400U)
#define FLASH_ACR_ICRST                      ((uint32_t)0x00000800U)
#define FLASH_ACR_DCRST                      ((uint32_t)0x00001000U)
#define FLASH_ACR_RUN_PD                     ((uint32_t)0x00002000U)   /*!< Flash power down mode during run */
#define FLASH_ACR_SLEEP_PD                   ((uint32_t)0x00004000U)   /*!< Flash power down mode during sleep */

/*******************  Bits definition for FLASH_SR register  ******************/
#define FLASH_SR_EOP                         ((uint32_t)0x00000001U)
#define FLASH_SR_OPERR                       ((uint32_t)0x00000002U)
#define FLASH_SR_PROGERR                     ((uint32_t)0x00000008U)
#define FLASH_SR_WRPERR                      ((uint32_t)0x00000010U)
#define FLASH_SR_PGAERR                      ((uint32_t)0x00000020U)
#define FLASH_SR_SIZERR                      ((uint32_t)0x00000040U)
#define FLASH_SR_PGSERR                      ((uint32_t)0x00000080U)
#define FLASH_SR_MISERR                      ((uint32_t)0x00000100U)
#define FLASH_SR_FASTERR                     ((uint32_t)0x00000200U)
#define FLASH_SR_RDERR                       ((uint32_t)0x00004000U)
#define FLASH_SR_OPTVERR                     ((uint32_t)0x00008000U)
#define FLASH_SR_BSY                         ((uint32_t)0x00010000U)
#define FLASH_SR_PEMPTY                      ((uint32_t)0x00020000U)

/*******************  Bits definition for FLASH_CR register  ******************/
#define FLASH_CR_PG                          ((uint32_t)0x00000001U)
#define FLASH_CR_PER                         ((uint32_t)0x00000002U)
#define FLASH_CR_MER1                        ((uint32_t)0x00000004U)
#define FLASH_CR_PNB                         ((uint32_t)0x000007F8U)
#define FLASH_CR_STRT                        ((uint32_t)0x00010000U)
#define FLASH_CR_OPTSTRT                     ((uint32_t)0x00020000U)
#define FLASH_CR_FSTPG                       ((uint32_t)0x00040000U)
#define FLASH_CR_EOPIE                       ((uint32_t)0x01000000U)
#define FLASH_CR_ERRIE                       ((uint32_t)0x02000000U)
#define FLASH_CR_RDERRIE                     ((uint32_t)0x04000000U)
#define FLASH_CR_OBL_LAUNCH                  ((uint32_t)0x08000000U)
#define FLASH_CR_OPTLOCK                     ((uint32_t)0x40000000U)
#define FLASH_CR_LOCK                        ((uint32_t)0x80000000U)

/*******************  Bits definition for FLASH_ECCR register  ***************/
#define FLASH_ECCR_ADDR_ECC                 ((uint32_t)0x0007FFFFU)
#define FLASH_ECCR_SYSF_ECC                 ((uint32_t)0x00100000U)
#define FLASH_ECCR_ECCIE                    ((uint32_t)0x01000000U)
#define FLASH_ECCR_ECCC                     ((uint32_t)0x40000000U)
#define FLASH_ECCR_ECCD                     ((uint32_t)0x80000000U)

/*******************  Bits definition for FLASH_OPTR register  ***************/
#define FLASH_OPTR_RDP                      ((uint32_t)0x000000FFU)
#define FLASH_OPTR_BOR_LEV                  ((uint32_t)0x00000700U)
#define FLASH_OPTR_BOR_LEV_0                ((uint32_t)0x00000000U)
#define FLASH_OPTR_BOR_LEV_1                ((uint32_t)0x00000100U)
#define FLASH_OPTR_BOR_LEV_2                ((uint32_t)0x00000200U)
#define FLASH_OPTR_BOR_LEV_3                ((uint32_t)0x00000300U)
#define FLASH_OPTR_BOR_LEV_4                ((uint32_t)0x00000400U)
#define FLASH_OPTR_nRST_STOP                ((uint32_t)0x00001000U)
#define FLASH_OPTR_nRST_STDBY               ((uint32_t)0x00002000U)
#define FLASH_OPTR_nRST_SHDW                ((uint32_t)0x00004000U)
#define FLASH_OPTR_IWDG_SW                  ((uint32_t)0x00010000U)
#define FLASH_OPTR_IWDG_STOP                ((uint32_t)0x00020000U)
#define FLASH_OPTR_IWDG_STDBY               ((uint32_t)0x00040000U)
#define FLASH_OPTR_WWDG_SW                  ((uint32_t)0x00080000U)
#define FLASH_OPTR_nBOOT1                   ((uint32_t)0x00800000U)
#define FLASH_OPTR_SRAM2_PE                 ((uint32_t)0x01000000U)
#define FLASH_OPTR_SRAM2_RST                ((uint32_t)0x02000000U)
#define FLASH_OPTR_nSWBOOT0                 ((uint32_t)0x04000000U)
#define FLASH_OPTR_nBOOT0                   ((uint32_t)0x08000000U)

/******************  Bits definition for FLASH_PCROP1SR register  **********/
#define FLASH_PCROP1SR_PCROP1_STRT          ((uint32_t)0x0000FFFFU)

/******************  Bits definition for FLASH_PCROP1ER register  ***********/
#define FLASH_PCROP1ER_PCROP1_END           ((uint32_t)0x0000FFFFU)
#define FLASH_PCROP1ER_PCROP_RDP            ((uint32_t)0x80000000U)

/******************  Bits definition for FLASH_WRP1AR register  ***************/
#define FLASH_WRP1AR_WRP1A_STRT             ((uint32_t)0x000000FFU)
#define FLASH_WRP1AR_WRP1A_END              ((uint32_t)0x00FF0000U)

/******************  Bits definition for FLASH_WRPB1R register  ***************/
#define FLASH_WRP1BR_WRP1B_STRT             ((uint32_t)0x000000FFU)
#define FLASH_WRP1BR_WRP1B_END              ((uint32_t)0x00FF0000U)




/******************************************************************************/
/*                                                                            */
/*                       General Purpose IOs (GPIO)                           */
/*                                                                            */
/******************************************************************************/
/******************  Bits definition for GPIO_MODER register  *****************/
#define GPIO_MODER_MODE0                    ((uint32_t)0x00000003U)
#define GPIO_MODER_MODE0_0                  ((uint32_t)0x00000001U)
#define GPIO_MODER_MODE0_1                  ((uint32_t)0x00000002U)
#define GPIO_MODER_MODE1                    ((uint32_t)0x0000000CU)
#define GPIO_MODER_MODE1_0                  ((uint32_t)0x00000004U)
#define GPIO_MODER_MODE1_1                  ((uint32_t)0x00000008U)
#define GPIO_MODER_MODE2                    ((uint32_t)0x00000030U)
#define GPIO_MODER_MODE2_0                  ((uint32_t)0x00000010U)
#define GPIO_MODER_MODE2_1                  ((uint32_t)0x00000020U)
#define GPIO_MODER_MODE3                    ((uint32_t)0x000000C0U)
#define GPIO_MODER_MODE3_0                  ((uint32_t)0x00000040U)
#define GPIO_MODER_MODE3_1                  ((uint32_t)0x00000080U)
#define GPIO_MODER_MODE4                    ((uint32_t)0x00000300U)
#define GPIO_MODER_MODE4_0                  ((uint32_t)0x00000100U)
#define GPIO_MODER_MODE4_1                  ((uint32_t)0x00000200U)
#define GPIO_MODER_MODE5                    ((uint32_t)0x00000C00U)
#define GPIO_MODER_MODE5_0                  ((uint32_t)0x00000400U)
#define GPIO_MODER_MODE5_1                  ((uint32_t)0x00000800U)
#define GPIO_MODER_MODE6                    ((uint32_t)0x00003000U)
#define GPIO_MODER_MODE6_0                  ((uint32_t)0x00001000U)
#define GPIO_MODER_MODE6_1                  ((uint32_t)0x00002000U)
#define GPIO_MODER_MODE7                    ((uint32_t)0x0000C000U)
#define GPIO_MODER_MODE7_0                  ((uint32_t)0x00004000U)
#define GPIO_MODER_MODE7_1                  ((uint32_t)0x00008000U)
#define GPIO_MODER_MODE8                    ((uint32_t)0x00030000U)
#define GPIO_MODER_MODE8_0                  ((uint32_t)0x00010000U)
#define GPIO_MODER_MODE8_1                  ((uint32_t)0x00020000U)
#define GPIO_MODER_MODE9                    ((uint32_t)0x000C0000U)
#define GPIO_MODER_MODE9_0                  ((uint32_t)0x00040000U)
#define GPIO_MODER_MODE9_1                  ((uint32_t)0x00080000U)
#define GPIO_MODER_MODE10                   ((uint32_t)0x00300000U)
#define GPIO_MODER_MODE10_0                 ((uint32_t)0x00100000U)
#define GPIO_MODER_MODE10_1                 ((uint32_t)0x00200000U)
#define GPIO_MODER_MODE11                   ((uint32_t)0x00C00000U)
#define GPIO_MODER_MODE11_0                 ((uint32_t)0x00400000U)
#define GPIO_MODER_MODE11_1                 ((uint32_t)0x00800000U)
#define GPIO_MODER_MODE12                   ((uint32_t)0x03000000U)
#define GPIO_MODER_MODE12_0                 ((uint32_t)0x01000000U)
#define GPIO_MODER_MODE12_1                 ((uint32_t)0x02000000U)
#define GPIO_MODER_MODE13                   ((uint32_t)0x0C000000U)
#define GPIO_MODER_MODE13_0                 ((uint32_t)0x04000000U)
#define GPIO_MODER_MODE13_1                 ((uint32_t)0x08000000U)
#define GPIO_MODER_MODE14                   ((uint32_t)0x30000000U)
#define GPIO_MODER_MODE14_0                 ((uint32_t)0x10000000U)
#define GPIO_MODER_MODE14_1                 ((uint32_t)0x20000000U)
#define GPIO_MODER_MODE15                   ((uint32_t)0xC0000000U)
#define GPIO_MODER_MODE15_0                 ((uint32_t)0x40000000U)
#define GPIO_MODER_MODE15_1                 ((uint32_t)0x80000000U)

/* Legacy defines */
#define GPIO_MODER_MODER0                   GPIO_MODER_MODE0
#define GPIO_MODER_MODER0_0                 GPIO_MODER_MODE0_0
#define GPIO_MODER_MODER0_1                 GPIO_MODER_MODE0_1
#define GPIO_MODER_MODER1                   GPIO_MODER_MODE1
#define GPIO_MODER_MODER1_0                 GPIO_MODER_MODE1_0
#define GPIO_MODER_MODER1_1                 GPIO_MODER_MODE1_1
#define GPIO_MODER_MODER2                   GPIO_MODER_MODE2
#define GPIO_MODER_MODER2_0                 GPIO_MODER_MODE2_0
#define GPIO_MODER_MODER2_1                 GPIO_MODER_MODE2_1
#define GPIO_MODER_MODER3                   GPIO_MODER_MODE3
#define GPIO_MODER_MODER3_0                 GPIO_MODER_MODE3_0
#define GPIO_MODER_MODER3_1                 GPIO_MODER_MODE3_1
#define GPIO_MODER_MODER4                   GPIO_MODER_MODE4
#define GPIO_MODER_MODER4_0                 GPIO_MODER_MODE4_0
#define GPIO_MODER_MODER4_1                 GPIO_MODER_MODE4_1
#define GPIO_MODER_MODER5                   GPIO_MODER_MODE5
#define GPIO_MODER_MODER5_0                 GPIO_MODER_MODE5_0
#define GPIO_MODER_MODER5_1                 GPIO_MODER_MODE5_1
#define GPIO_MODER_MODER6                   GPIO_MODER_MODE6
#define GPIO_MODER_MODER6_0                 GPIO_MODER_MODE6_0
#define GPIO_MODER_MODER6_1                 GPIO_MODER_MODE6_1
#define GPIO_MODER_MODER7                   GPIO_MODER_MODE7
#define GPIO_MODER_MODER7_0                 GPIO_MODER_MODE7_0
#define GPIO_MODER_MODER7_1                 GPIO_MODER_MODE7_1
#define GPIO_MODER_MODER8                   GPIO_MODER_MODE8
#define GPIO_MODER_MODER8_0                 GPIO_MODER_MODE8_0
#define GPIO_MODER_MODER8_1                 GPIO_MODER_MODE8_1
#define GPIO_MODER_MODER9                   GPIO_MODER_MODE9
#define GPIO_MODER_MODER9_0                 GPIO_MODER_MODE9_0
#define GPIO_MODER_MODER9_1                 GPIO_MODER_MODE9_1
#define GPIO_MODER_MODER10                  GPIO_MODER_MODE10
#define GPIO_MODER_MODER10_0                GPIO_MODER_MODE10_0
#define GPIO_MODER_MODER10_1                GPIO_MODER_MODE10_1
#define GPIO_MODER_MODER11                  GPIO_MODER_MODE11
#define GPIO_MODER_MODER11_0                GPIO_MODER_MODE11_0
#define GPIO_MODER_MODER11_1                GPIO_MODER_MODE11_1
#define GPIO_MODER_MODER12                  GPIO_MODER_MODE12
#define GPIO_MODER_MODER12_0                GPIO_MODER_MODE12_0
#define GPIO_MODER_MODER12_1                GPIO_MODER_MODE12_1
#define GPIO_MODER_MODER13                  GPIO_MODER_MODE13
#define GPIO_MODER_MODER13_0                GPIO_MODER_MODE13_0
#define GPIO_MODER_MODER13_1                GPIO_MODER_MODE13_1
#define GPIO_MODER_MODER14                  GPIO_MODER_MODE14
#define GPIO_MODER_MODER14_0                GPIO_MODER_MODE14_0
#define GPIO_MODER_MODER14_1                GPIO_MODER_MODE14_1
#define GPIO_MODER_MODER15                  GPIO_MODER_MODE15
#define GPIO_MODER_MODER15_0                GPIO_MODER_MODE15_0
#define GPIO_MODER_MODER15_1                GPIO_MODER_MODE15_1

/******************  Bits definition for GPIO_OTYPER register  ****************/
#define GPIO_OTYPER_OT0                     ((uint32_t)0x00000001U)
#define GPIO_OTYPER_OT1                     ((uint32_t)0x00000002U)
#define GPIO_OTYPER_OT2                     ((uint32_t)0x00000004U)
#define GPIO_OTYPER_OT3                     ((uint32_t)0x00000008U)
#define GPIO_OTYPER_OT4                     ((uint32_t)0x00000010U)
#define GPIO_OTYPER_OT5                     ((uint32_t)0x00000020U)
#define GPIO_OTYPER_OT6                     ((uint32_t)0x00000040U)
#define GPIO_OTYPER_OT7                     ((uint32_t)0x00000080U)
#define GPIO_OTYPER_OT8                     ((uint32_t)0x00000100U)
#define GPIO_OTYPER_OT9                     ((uint32_t)0x00000200U)
#define GPIO_OTYPER_OT10                    ((uint32_t)0x00000400U)
#define GPIO_OTYPER_OT11                    ((uint32_t)0x00000800U)
#define GPIO_OTYPER_OT12                    ((uint32_t)0x00001000U)
#define GPIO_OTYPER_OT13                    ((uint32_t)0x00002000U)
#define GPIO_OTYPER_OT14                    ((uint32_t)0x00004000U)
#define GPIO_OTYPER_OT15                    ((uint32_t)0x00008000U)

/* Legacy defines */
#define GPIO_OTYPER_OT_0                    GPIO_OTYPER_OT0
#define GPIO_OTYPER_OT_1                    GPIO_OTYPER_OT1
#define GPIO_OTYPER_OT_2                    GPIO_OTYPER_OT2
#define GPIO_OTYPER_OT_3                    GPIO_OTYPER_OT3
#define GPIO_OTYPER_OT_4                    GPIO_OTYPER_OT4
#define GPIO_OTYPER_OT_5                    GPIO_OTYPER_OT5
#define GPIO_OTYPER_OT_6                    GPIO_OTYPER_OT6
#define GPIO_OTYPER_OT_7                    GPIO_OTYPER_OT7
#define GPIO_OTYPER_OT_8                    GPIO_OTYPER_OT8
#define GPIO_OTYPER_OT_9                    GPIO_OTYPER_OT9
#define GPIO_OTYPER_OT_10                   GPIO_OTYPER_OT10
#define GPIO_OTYPER_OT_11                   GPIO_OTYPER_OT11
#define GPIO_OTYPER_OT_12                   GPIO_OTYPER_OT12
#define GPIO_OTYPER_OT_13                   GPIO_OTYPER_OT13
#define GPIO_OTYPER_OT_14                   GPIO_OTYPER_OT14
#define GPIO_OTYPER_OT_15                   GPIO_OTYPER_OT15

/******************  Bits definition for GPIO_OSPEEDR register  ***************/
#define GPIO_OSPEEDR_OSPEED0                ((uint32_t)0x00000003U)
#define GPIO_OSPEEDR_OSPEED0_0              ((uint32_t)0x00000001U)
#define GPIO_OSPEEDR_OSPEED0_1              ((uint32_t)0x00000002U)
#define GPIO_OSPEEDR_OSPEED1                ((uint32_t)0x0000000CU)
#define GPIO_OSPEEDR_OSPEED1_0              ((uint32_t)0x00000004U)
#define GPIO_OSPEEDR_OSPEED1_1              ((uint32_t)0x00000008U)
#define GPIO_OSPEEDR_OSPEED2                ((uint32_t)0x00000030U)
#define GPIO_OSPEEDR_OSPEED2_0              ((uint32_t)0x00000010U)
#define GPIO_OSPEEDR_OSPEED2_1              ((uint32_t)0x00000020U)
#define GPIO_OSPEEDR_OSPEED3                ((uint32_t)0x000000C0U)
#define GPIO_OSPEEDR_OSPEED3_0              ((uint32_t)0x00000040U)
#define GPIO_OSPEEDR_OSPEED3_1              ((uint32_t)0x00000080U)
#define GPIO_OSPEEDR_OSPEED4                ((uint32_t)0x00000300U)
#define GPIO_OSPEEDR_OSPEED4_0              ((uint32_t)0x00000100U)
#define GPIO_OSPEEDR_OSPEED4_1              ((uint32_t)0x00000200U)
#define GPIO_OSPEEDR_OSPEED5                ((uint32_t)0x00000C00U)
#define GPIO_OSPEEDR_OSPEED5_0              ((uint32_t)0x00000400U)
#define GPIO_OSPEEDR_OSPEED5_1              ((uint32_t)0x00000800U)
#define GPIO_OSPEEDR_OSPEED6                ((uint32_t)0x00003000U)
#define GPIO_OSPEEDR_OSPEED6_0              ((uint32_t)0x00001000U)
#define GPIO_OSPEEDR_OSPEED6_1              ((uint32_t)0x00002000U)
#define GPIO_OSPEEDR_OSPEED7                ((uint32_t)0x0000C000U)
#define GPIO_OSPEEDR_OSPEED7_0              ((uint32_t)0x00004000U)
#define GPIO_OSPEEDR_OSPEED7_1              ((uint32_t)0x00008000U)
#define GPIO_OSPEEDR_OSPEED8                ((uint32_t)0x00030000U)
#define GPIO_OSPEEDR_OSPEED8_0              ((uint32_t)0x00010000U)
#define GPIO_OSPEEDR_OSPEED8_1              ((uint32_t)0x00020000U)
#define GPIO_OSPEEDR_OSPEED9                ((uint32_t)0x000C0000U)
#define GPIO_OSPEEDR_OSPEED9_0              ((uint32_t)0x00040000U)
#define GPIO_OSPEEDR_OSPEED9_1              ((uint32_t)0x00080000U)
#define GPIO_OSPEEDR_OSPEED10               ((uint32_t)0x00300000U)
#define GPIO_OSPEEDR_OSPEED10_0             ((uint32_t)0x00100000U)
#define GPIO_OSPEEDR_OSPEED10_1             ((uint32_t)0x00200000U)
#define GPIO_OSPEEDR_OSPEED11               ((uint32_t)0x00C00000U)
#define GPIO_OSPEEDR_OSPEED11_0             ((uint32_t)0x00400000U)
#define GPIO_OSPEEDR_OSPEED11_1             ((uint32_t)0x00800000U)
#define GPIO_OSPEEDR_OSPEED12               ((uint32_t)0x03000000U)
#define GPIO_OSPEEDR_OSPEED12_0             ((uint32_t)0x01000000U)
#define GPIO_OSPEEDR_OSPEED12_1             ((uint32_t)0x02000000U)
#define GPIO_OSPEEDR_OSPEED13               ((uint32_t)0x0C000000U)
#define GPIO_OSPEEDR_OSPEED13_0             ((uint32_t)0x04000000U)
#define GPIO_OSPEEDR_OSPEED13_1             ((uint32_t)0x08000000U)
#define GPIO_OSPEEDR_OSPEED14               ((uint32_t)0x30000000U)
#define GPIO_OSPEEDR_OSPEED14_0             ((uint32_t)0x10000000U)
#define GPIO_OSPEEDR_OSPEED14_1             ((uint32_t)0x20000000U)
#define GPIO_OSPEEDR_OSPEED15               ((uint32_t)0xC0000000U)
#define GPIO_OSPEEDR_OSPEED15_0             ((uint32_t)0x40000000U)
#define GPIO_OSPEEDR_OSPEED15_1             ((uint32_t)0x80000000U)

/* Legacy defines */
#define GPIO_OSPEEDER_OSPEEDR0              GPIO_OSPEEDR_OSPEED0
#define GPIO_OSPEEDER_OSPEEDR0_0            GPIO_OSPEEDR_OSPEED0_0
#define GPIO_OSPEEDER_OSPEEDR0_1            GPIO_OSPEEDR_OSPEED0_1
#define GPIO_OSPEEDER_OSPEEDR1              GPIO_OSPEEDR_OSPEED1
#define GPIO_OSPEEDER_OSPEEDR1_0            GPIO_OSPEEDR_OSPEED1_0
#define GPIO_OSPEEDER_OSPEEDR1_1            GPIO_OSPEEDR_OSPEED1_1
#define GPIO_OSPEEDER_OSPEEDR2              GPIO_OSPEEDR_OSPEED2
#define GPIO_OSPEEDER_OSPEEDR2_0            GPIO_OSPEEDR_OSPEED2_0
#define GPIO_OSPEEDER_OSPEEDR2_1            GPIO_OSPEEDR_OSPEED2_1
#define GPIO_OSPEEDER_OSPEEDR3              GPIO_OSPEEDR_OSPEED3
#define GPIO_OSPEEDER_OSPEEDR3_0            GPIO_OSPEEDR_OSPEED3_0
#define GPIO_OSPEEDER_OSPEEDR3_1            GPIO_OSPEEDR_OSPEED3_1
#define GPIO_OSPEEDER_OSPEEDR4              GPIO_OSPEEDR_OSPEED4
#define GPIO_OSPEEDER_OSPEEDR4_0            GPIO_OSPEEDR_OSPEED4_0
#define GPIO_OSPEEDER_OSPEEDR4_1            GPIO_OSPEEDR_OSPEED4_1
#define GPIO_OSPEEDER_OSPEEDR5              GPIO_OSPEEDR_OSPEED5
#define GPIO_OSPEEDER_OSPEEDR5_0            GPIO_OSPEEDR_OSPEED5_0
#define GPIO_OSPEEDER_OSPEEDR5_1            GPIO_OSPEEDR_OSPEED5_1
#define GPIO_OSPEEDER_OSPEEDR6              GPIO_OSPEEDR_OSPEED6
#define GPIO_OSPEEDER_OSPEEDR6_0            GPIO_OSPEEDR_OSPEED6_0
#define GPIO_OSPEEDER_OSPEEDR6_1            GPIO_OSPEEDR_OSPEED6_1
#define GPIO_OSPEEDER_OSPEEDR7              GPIO_OSPEEDR_OSPEED7
#define GPIO_OSPEEDER_OSPEEDR7_0            GPIO_OSPEEDR_OSPEED7_0
#define GPIO_OSPEEDER_OSPEEDR7_1            GPIO_OSPEEDR_OSPEED7_1
#define GPIO_OSPEEDER_OSPEEDR8              GPIO_OSPEEDR_OSPEED8
#define GPIO_OSPEEDER_OSPEEDR8_0            GPIO_OSPEEDR_OSPEED8_0
#define GPIO_OSPEEDER_OSPEEDR8_1            GPIO_OSPEEDR_OSPEED8_1
#define GPIO_OSPEEDER_OSPEEDR9              GPIO_OSPEEDR_OSPEED9
#define GPIO_OSPEEDER_OSPEEDR9_0            GPIO_OSPEEDR_OSPEED9_0
#define GPIO_OSPEEDER_OSPEEDR9_1            GPIO_OSPEEDR_OSPEED9_1
#define GPIO_OSPEEDER_OSPEEDR10             GPIO_OSPEEDR_OSPEED10
#define GPIO_OSPEEDER_OSPEEDR10_0           GPIO_OSPEEDR_OSPEED10_0
#define GPIO_OSPEEDER_OSPEEDR10_1           GPIO_OSPEEDR_OSPEED10_1
#define GPIO_OSPEEDER_OSPEEDR11             GPIO_OSPEEDR_OSPEED11
#define GPIO_OSPEEDER_OSPEEDR11_0           GPIO_OSPEEDR_OSPEED11_0
#define GPIO_OSPEEDER_OSPEEDR11_1           GPIO_OSPEEDR_OSPEED11_1
#define GPIO_OSPEEDER_OSPEEDR12             GPIO_OSPEEDR_OSPEED12
#define GPIO_OSPEEDER_OSPEEDR12_0           GPIO_OSPEEDR_OSPEED12_0
#define GPIO_OSPEEDER_OSPEEDR12_1           GPIO_OSPEEDR_OSPEED12_1
#define GPIO_OSPEEDER_OSPEEDR13             GPIO_OSPEEDR_OSPEED13
#define GPIO_OSPEEDER_OSPEEDR13_0           GPIO_OSPEEDR_OSPEED13_0
#define GPIO_OSPEEDER_OSPEEDR13_1           GPIO_OSPEEDR_OSPEED13_1
#define GPIO_OSPEEDER_OSPEEDR14             GPIO_OSPEEDR_OSPEED14
#define GPIO_OSPEEDER_OSPEEDR14_0           GPIO_OSPEEDR_OSPEED14_0
#define GPIO_OSPEEDER_OSPEEDR14_1           GPIO_OSPEEDR_OSPEED14_1
#define GPIO_OSPEEDER_OSPEEDR15             GPIO_OSPEEDR_OSPEED15
#define GPIO_OSPEEDER_OSPEEDR15_0           GPIO_OSPEEDR_OSPEED15_0
#define GPIO_OSPEEDER_OSPEEDR15_1           GPIO_OSPEEDR_OSPEED15_1

/******************  Bits definition for GPIO_PUPDR register  *****************/
#define GPIO_PUPDR_PUPD0                    ((uint32_t)0x00000003U)
#define GPIO_PUPDR_PUPD0_0                  ((uint32_t)0x00000001U)
#define GPIO_PUPDR_PUPD0_1                  ((uint32_t)0x00000002U)
#define GPIO_PUPDR_PUPD1                    ((uint32_t)0x0000000CU)
#define GPIO_PUPDR_PUPD1_0                  ((uint32_t)0x00000004U)
#define GPIO_PUPDR_PUPD1_1                  ((uint32_t)0x00000008U)
#define GPIO_PUPDR_PUPD2                    ((uint32_t)0x00000030U)
#define GPIO_PUPDR_PUPD2_0                  ((uint32_t)0x00000010U)
#define GPIO_PUPDR_PUPD2_1                  ((uint32_t)0x00000020U)
#define GPIO_PUPDR_PUPD3                    ((uint32_t)0x000000C0U)
#define GPIO_PUPDR_PUPD3_0                  ((uint32_t)0x00000040U)
#define GPIO_PUPDR_PUPD3_1                  ((uint32_t)0x00000080U)
#define GPIO_PUPDR_PUPD4                    ((uint32_t)0x00000300U)
#define GPIO_PUPDR_PUPD4_0                  ((uint32_t)0x00000100U)
#define GPIO_PUPDR_PUPD4_1                  ((uint32_t)0x00000200U)
#define GPIO_PUPDR_PUPD5                    ((uint32_t)0x00000C00U)
#define GPIO_PUPDR_PUPD5_0                  ((uint32_t)0x00000400U)
#define GPIO_PUPDR_PUPD5_1                  ((uint32_t)0x00000800U)
#define GPIO_PUPDR_PUPD6                    ((uint32_t)0x00003000U)
#define GPIO_PUPDR_PUPD6_0                  ((uint32_t)0x00001000U)
#define GPIO_PUPDR_PUPD6_1                  ((uint32_t)0x00002000U)
#define GPIO_PUPDR_PUPD7                    ((uint32_t)0x0000C000U)
#define GPIO_PUPDR_PUPD7_0                  ((uint32_t)0x00004000U)
#define GPIO_PUPDR_PUPD7_1                  ((uint32_t)0x00008000U)
#define GPIO_PUPDR_PUPD8                    ((uint32_t)0x00030000U)
#define GPIO_PUPDR_PUPD8_0                  ((uint32_t)0x00010000U)
#define GPIO_PUPDR_PUPD8_1                  ((uint32_t)0x00020000U)
#define GPIO_PUPDR_PUPD9                    ((uint32_t)0x000C0000U)
#define GPIO_PUPDR_PUPD9_0                  ((uint32_t)0x00040000U)
#define GPIO_PUPDR_PUPD9_1                  ((uint32_t)0x00080000U)
#define GPIO_PUPDR_PUPD10                   ((uint32_t)0x00300000U)
#define GPIO_PUPDR_PUPD10_0                 ((uint32_t)0x00100000U)
#define GPIO_PUPDR_PUPD10_1                 ((uint32_t)0x00200000U)
#define GPIO_PUPDR_PUPD11                   ((uint32_t)0x00C00000U)
#define GPIO_PUPDR_PUPD11_0                 ((uint32_t)0x00400000U)
#define GPIO_PUPDR_PUPD11_1                 ((uint32_t)0x00800000U)
#define GPIO_PUPDR_PUPD12                   ((uint32_t)0x03000000U)
#define GPIO_PUPDR_PUPD12_0                 ((uint32_t)0x01000000U)
#define GPIO_PUPDR_PUPD12_1                 ((uint32_t)0x02000000U)
#define GPIO_PUPDR_PUPD13                   ((uint32_t)0x0C000000U)
#define GPIO_PUPDR_PUPD13_0                 ((uint32_t)0x04000000U)
#define GPIO_PUPDR_PUPD13_1                 ((uint32_t)0x08000000U)
#define GPIO_PUPDR_PUPD14                   ((uint32_t)0x30000000U)
#define GPIO_PUPDR_PUPD14_0                 ((uint32_t)0x10000000U)
#define GPIO_PUPDR_PUPD14_1                 ((uint32_t)0x20000000U)
#define GPIO_PUPDR_PUPD15                   ((uint32_t)0xC0000000U)
#define GPIO_PUPDR_PUPD15_0                 ((uint32_t)0x40000000U)
#define GPIO_PUPDR_PUPD15_1                 ((uint32_t)0x80000000U)

/* Legacy defines */
#define GPIO_PUPDR_PUPDR0                   GPIO_PUPDR_PUPD0
#define GPIO_PUPDR_PUPDR0_0                 GPIO_PUPDR_PUPD0_0
#define GPIO_PUPDR_PUPDR0_1                 GPIO_PUPDR_PUPD0_1
#define GPIO_PUPDR_PUPDR1                   GPIO_PUPDR_PUPD1
#define GPIO_PUPDR_PUPDR1_0                 GPIO_PUPDR_PUPD1_0
#define GPIO_PUPDR_PUPDR1_1                 GPIO_PUPDR_PUPD1_1
#define GPIO_PUPDR_PUPDR2                   GPIO_PUPDR_PUPD2
#define GPIO_PUPDR_PUPDR2_0                 GPIO_PUPDR_PUPD2_0
#define GPIO_PUPDR_PUPDR2_1                 GPIO_PUPDR_PUPD2_1
#define GPIO_PUPDR_PUPDR3                   GPIO_PUPDR_PUPD3
#define GPIO_PUPDR_PUPDR3_0                 GPIO_PUPDR_PUPD3_0
#define GPIO_PUPDR_PUPDR3_1                 GPIO_PUPDR_PUPD3_1
#define GPIO_PUPDR_PUPDR4                   GPIO_PUPDR_PUPD4
#define GPIO_PUPDR_PUPDR4_0                 GPIO_PUPDR_PUPD4_0
#define GPIO_PUPDR_PUPDR4_1                 GPIO_PUPDR_PUPD4_1
#define GPIO_PUPDR_PUPDR5                   GPIO_PUPDR_PUPD5
#define GPIO_PUPDR_PUPDR5_0                 GPIO_PUPDR_PUPD5_0
#define GPIO_PUPDR_PUPDR5_1                 GPIO_PUPDR_PUPD5_1
#define GPIO_PUPDR_PUPDR6                   GPIO_PUPDR_PUPD6
#define GPIO_PUPDR_PUPDR6_0                 GPIO_PUPDR_PUPD6_0
#define GPIO_PUPDR_PUPDR6_1                 GPIO_PUPDR_PUPD6_1
#define GPIO_PUPDR_PUPDR7                   GPIO_PUPDR_PUPD7
#define GPIO_PUPDR_PUPDR7_0                 GPIO_PUPDR_PUPD7_0
#define GPIO_PUPDR_PUPDR7_1                 GPIO_PUPDR_PUPD7_1
#define GPIO_PUPDR_PUPDR8                   GPIO_PUPDR_PUPD8
#define GPIO_PUPDR_PUPDR8_0                 GPIO_PUPDR_PUPD8_0
#define GPIO_PUPDR_PUPDR8_1                 GPIO_PUPDR_PUPD8_1
#define GPIO_PUPDR_PUPDR9                   GPIO_PUPDR_PUPD9
#define GPIO_PUPDR_PUPDR9_0                 GPIO_PUPDR_PUPD9_0
#define GPIO_PUPDR_PUPDR9_1                 GPIO_PUPDR_PUPD9_1
#define GPIO_PUPDR_PUPDR10                  GPIO_PUPDR_PUPD10
#define GPIO_PUPDR_PUPDR10_0                GPIO_PUPDR_PUPD10_0
#define GPIO_PUPDR_PUPDR10_1                GPIO_PUPDR_PUPD10_1
#define GPIO_PUPDR_PUPDR11                  GPIO_PUPDR_PUPD11
#define GPIO_PUPDR_PUPDR11_0                GPIO_PUPDR_PUPD11_0
#define GPIO_PUPDR_PUPDR11_1                GPIO_PUPDR_PUPD11_1
#define GPIO_PUPDR_PUPDR12                  GPIO_PUPDR_PUPD12
#define GPIO_PUPDR_PUPDR12_0                GPIO_PUPDR_PUPD12_0
#define GPIO_PUPDR_PUPDR12_1                GPIO_PUPDR_PUPD12_1
#define GPIO_PUPDR_PUPDR13                  GPIO_PUPDR_PUPD13
#define GPIO_PUPDR_PUPDR13_0                GPIO_PUPDR_PUPD13_0
#define GPIO_PUPDR_PUPDR13_1                GPIO_PUPDR_PUPD13_1
#define GPIO_PUPDR_PUPDR14                  GPIO_PUPDR_PUPD14
#define GPIO_PUPDR_PUPDR14_0                GPIO_PUPDR_PUPD14_0
#define GPIO_PUPDR_PUPDR14_1                GPIO_PUPDR_PUPD14_1
#define GPIO_PUPDR_PUPDR15                  GPIO_PUPDR_PUPD15
#define GPIO_PUPDR_PUPDR15_0                GPIO_PUPDR_PUPD15_0
#define GPIO_PUPDR_PUPDR15_1                GPIO_PUPDR_PUPD15_1

/******************  Bits definition for GPIO_IDR register  *******************/
#define GPIO_IDR_ID0                        ((uint32_t)0x00000001U)
#define GPIO_IDR_ID1                        ((uint32_t)0x00000002U)
#define GPIO_IDR_ID2                        ((uint32_t)0x00000004U)
#define GPIO_IDR_ID3                        ((uint32_t)0x00000008U)
#define GPIO_IDR_ID4                        ((uint32_t)0x00000010U)
#define GPIO_IDR_ID5                        ((uint32_t)0x00000020U)
#define GPIO_IDR_ID6                        ((uint32_t)0x00000040U)
#define GPIO_IDR_ID7                        ((uint32_t)0x00000080U)
#define GPIO_IDR_ID8                        ((uint32_t)0x00000100U)
#define GPIO_IDR_ID9                        ((uint32_t)0x00000200U)
#define GPIO_IDR_ID10                       ((uint32_t)0x00000400U)
#define GPIO_IDR_ID11                       ((uint32_t)0x00000800U)
#define GPIO_IDR_ID12                       ((uint32_t)0x00001000U)
#define GPIO_IDR_ID13                       ((uint32_t)0x00002000U)
#define GPIO_IDR_ID14                       ((uint32_t)0x00004000U)
#define GPIO_IDR_ID15                       ((uint32_t)0x00008000U)

/* Legacy defines */
#define GPIO_IDR_IDR_0                      GPIO_IDR_ID0
#define GPIO_IDR_IDR_1                      GPIO_IDR_ID1
#define GPIO_IDR_IDR_2                      GPIO_IDR_ID2
#define GPIO_IDR_IDR_3                      GPIO_IDR_ID3
#define GPIO_IDR_IDR_4                      GPIO_IDR_ID4
#define GPIO_IDR_IDR_5                      GPIO_IDR_ID5
#define GPIO_IDR_IDR_6                      GPIO_IDR_ID6
#define GPIO_IDR_IDR_7                      GPIO_IDR_ID7
#define GPIO_IDR_IDR_8                      GPIO_IDR_ID8
#define GPIO_IDR_IDR_9                      GPIO_IDR_ID9
#define GPIO_IDR_IDR_10                     GPIO_IDR_ID10
#define GPIO_IDR_IDR_11                     GPIO_IDR_ID11
#define GPIO_IDR_IDR_12                     GPIO_IDR_ID12
#define GPIO_IDR_IDR_13                     GPIO_IDR_ID13
#define GPIO_IDR_IDR_14                     GPIO_IDR_ID14
#define GPIO_IDR_IDR_15                     GPIO_IDR_ID15

/* Old GPIO_IDR register bits definition, maintained for legacy purpose */
#define GPIO_OTYPER_IDR_0                   GPIO_IDR_ID0
#define GPIO_OTYPER_IDR_1                   GPIO_IDR_ID1
#define GPIO_OTYPER_IDR_2                   GPIO_IDR_ID2
#define GPIO_OTYPER_IDR_3                   GPIO_IDR_ID3
#define GPIO_OTYPER_IDR_4                   GPIO_IDR_ID4
#define GPIO_OTYPER_IDR_5                   GPIO_IDR_ID5
#define GPIO_OTYPER_IDR_6                   GPIO_IDR_ID6
#define GPIO_OTYPER_IDR_7                   GPIO_IDR_ID7
#define GPIO_OTYPER_IDR_8                   GPIO_IDR_ID8
#define GPIO_OTYPER_IDR_9                   GPIO_IDR_ID9
#define GPIO_OTYPER_IDR_10                  GPIO_IDR_ID10
#define GPIO_OTYPER_IDR_11                  GPIO_IDR_ID11
#define GPIO_OTYPER_IDR_12                  GPIO_IDR_ID12
#define GPIO_OTYPER_IDR_13                  GPIO_IDR_ID13
#define GPIO_OTYPER_IDR_14                  GPIO_IDR_ID14
#define GPIO_OTYPER_IDR_15                  GPIO_IDR_ID15

/******************  Bits definition for GPIO_ODR register  *******************/
#define GPIO_ODR_OD0                        ((uint32_t)0x00000001U)
#define GPIO_ODR_OD1                        ((uint32_t)0x00000002U)
#define GPIO_ODR_OD2                        ((uint32_t)0x00000004U)
#define GPIO_ODR_OD3                        ((uint32_t)0x00000008U)
#define GPIO_ODR_OD4                        ((uint32_t)0x00000010U)
#define GPIO_ODR_OD5                        ((uint32_t)0x00000020U)
#define GPIO_ODR_OD6                        ((uint32_t)0x00000040U)
#define GPIO_ODR_OD7                        ((uint32_t)0x00000080U)
#define GPIO_ODR_OD8                        ((uint32_t)0x00000100U)
#define GPIO_ODR_OD9                        ((uint32_t)0x00000200U)
#define GPIO_ODR_OD10                       ((uint32_t)0x00000400U)
#define GPIO_ODR_OD11                       ((uint32_t)0x00000800U)
#define GPIO_ODR_OD12                       ((uint32_t)0x00001000U)
#define GPIO_ODR_OD13                       ((uint32_t)0x00002000U)
#define GPIO_ODR_OD14                       ((uint32_t)0x00004000U)
#define GPIO_ODR_OD15                       ((uint32_t)0x00008000U)

/* Legacy defines */
#define GPIO_ODR_ODR_0                      GPIO_ODR_OD0
#define GPIO_ODR_ODR_1                      GPIO_ODR_OD1
#define GPIO_ODR_ODR_2                      GPIO_ODR_OD2
#define GPIO_ODR_ODR_3                      GPIO_ODR_OD3
#define GPIO_ODR_ODR_4                      GPIO_ODR_OD4
#define GPIO_ODR_ODR_5                      GPIO_ODR_OD5
#define GPIO_ODR_ODR_6                      GPIO_ODR_OD6
#define GPIO_ODR_ODR_7                      GPIO_ODR_OD7
#define GPIO_ODR_ODR_8                      GPIO_ODR_OD8
#define GPIO_ODR_ODR_9                      GPIO_ODR_OD9
#define GPIO_ODR_ODR_10                     GPIO_ODR_OD10
#define GPIO_ODR_ODR_11                     GPIO_ODR_OD11
#define GPIO_ODR_ODR_12                     GPIO_ODR_OD12
#define GPIO_ODR_ODR_13                     GPIO_ODR_OD13
#define GPIO_ODR_ODR_14                     GPIO_ODR_OD14
#define GPIO_ODR_ODR_15                     GPIO_ODR_OD15

/* Old GPIO_ODR register bits definition, maintained for legacy purpose */
#define GPIO_OTYPER_ODR_0                   GPIO_ODR_OD0
#define GPIO_OTYPER_ODR_1                   GPIO_ODR_OD1
#define GPIO_OTYPER_ODR_2                   GPIO_ODR_OD2
#define GPIO_OTYPER_ODR_3                   GPIO_ODR_OD3
#define GPIO_OTYPER_ODR_4                   GPIO_ODR_OD4
#define GPIO_OTYPER_ODR_5                   GPIO_ODR_OD5
#define GPIO_OTYPER_ODR_6                   GPIO_ODR_OD6
#define GPIO_OTYPER_ODR_7                   GPIO_ODR_OD7
#define GPIO_OTYPER_ODR_8                   GPIO_ODR_OD8
#define GPIO_OTYPER_ODR_9                   GPIO_ODR_OD9
#define GPIO_OTYPER_ODR_10                  GPIO_ODR_OD10
#define GPIO_OTYPER_ODR_11                  GPIO_ODR_OD11
#define GPIO_OTYPER_ODR_12                  GPIO_ODR_OD12
#define GPIO_OTYPER_ODR_13                  GPIO_ODR_OD13
#define GPIO_OTYPER_ODR_14                  GPIO_ODR_OD14
#define GPIO_OTYPER_ODR_15                  GPIO_ODR_OD15

/******************  Bits definition for GPIO_BSRR register  ******************/
#define GPIO_BSRR_BS0                       ((uint32_t)0x00000001U)
#define GPIO_BSRR_BS1                       ((uint32_t)0x00000002U)
#define GPIO_BSRR_BS2                       ((uint32_t)0x00000004U)
#define GPIO_BSRR_BS3                       ((uint32_t)0x00000008U)
#define GPIO_BSRR_BS4                       ((uint32_t)0x00000010U)
#define GPIO_BSRR_BS5                       ((uint32_t)0x00000020U)
#define GPIO_BSRR_BS6                       ((uint32_t)0x00000040U)
#define GPIO_BSRR_BS7                       ((uint32_t)0x00000080U)
#define GPIO_BSRR_BS8                       ((uint32_t)0x00000100U)
#define GPIO_BSRR_BS9                       ((uint32_t)0x00000200U)
#define GPIO_BSRR_BS10                      ((uint32_t)0x00000400U)
#define GPIO_BSRR_BS11                      ((uint32_t)0x00000800U)
#define GPIO_BSRR_BS12                      ((uint32_t)0x00001000U)
#define GPIO_BSRR_BS13                      ((uint32_t)0x00002000U)
#define GPIO_BSRR_BS14                      ((uint32_t)0x00004000U)
#define GPIO_BSRR_BS15                      ((uint32_t)0x00008000U)
#define GPIO_BSRR_BR0                       ((uint32_t)0x00010000U)
#define GPIO_BSRR_BR1                       ((uint32_t)0x00020000U)
#define GPIO_BSRR_BR2                       ((uint32_t)0x00040000U)
#define GPIO_BSRR_BR3                       ((uint32_t)0x00080000U)
#define GPIO_BSRR_BR4                       ((uint32_t)0x00100000U)
#define GPIO_BSRR_BR5                       ((uint32_t)0x00200000U)
#define GPIO_BSRR_BR6                       ((uint32_t)0x00400000U)
#define GPIO_BSRR_BR7                       ((uint32_t)0x00800000U)
#define GPIO_BSRR_BR8                       ((uint32_t)0x01000000U)
#define GPIO_BSRR_BR9                       ((uint32_t)0x02000000U)
#define GPIO_BSRR_BR10                      ((uint32_t)0x04000000U)
#define GPIO_BSRR_BR11                      ((uint32_t)0x08000000U)
#define GPIO_BSRR_BR12                      ((uint32_t)0x10000000U)
#define GPIO_BSRR_BR13                      ((uint32_t)0x20000000U)
#define GPIO_BSRR_BR14                      ((uint32_t)0x40000000U)
#define GPIO_BSRR_BR15                      ((uint32_t)0x80000000U)

/* Legacy defines */
#define GPIO_BSRR_BS_0                      GPIO_BSRR_BS0
#define GPIO_BSRR_BS_1                      GPIO_BSRR_BS1
#define GPIO_BSRR_BS_2                      GPIO_BSRR_BS2
#define GPIO_BSRR_BS_3                      GPIO_BSRR_BS3
#define GPIO_BSRR_BS_4                      GPIO_BSRR_BS4
#define GPIO_BSRR_BS_5                      GPIO_BSRR_BS5
#define GPIO_BSRR_BS_6                      GPIO_BSRR_BS6
#define GPIO_BSRR_BS_7                      GPIO_BSRR_BS7
#define GPIO_BSRR_BS_8                      GPIO_BSRR_BS8
#define GPIO_BSRR_BS_9                      GPIO_BSRR_BS9
#define GPIO_BSRR_BS_10                     GPIO_BSRR_BS10
#define GPIO_BSRR_BS_11                     GPIO_BSRR_BS11
#define GPIO_BSRR_BS_12                     GPIO_BSRR_BS12
#define GPIO_BSRR_BS_13                     GPIO_BSRR_BS13
#define GPIO_BSRR_BS_14                     GPIO_BSRR_BS14
#define GPIO_BSRR_BS_15                     GPIO_BSRR_BS15
#define GPIO_BSRR_BR_0                      GPIO_BSRR_BR0
#define GPIO_BSRR_BR_1                      GPIO_BSRR_BR1
#define GPIO_BSRR_BR_2                      GPIO_BSRR_BR2
#define GPIO_BSRR_BR_3                      GPIO_BSRR_BR3
#define GPIO_BSRR_BR_4                      GPIO_BSRR_BR4
#define GPIO_BSRR_BR_5                      GPIO_BSRR_BR5
#define GPIO_BSRR_BR_6                      GPIO_BSRR_BR6
#define GPIO_BSRR_BR_7                      GPIO_BSRR_BR7
#define GPIO_BSRR_BR_8                      GPIO_BSRR_BR8
#define GPIO_BSRR_BR_9                      GPIO_BSRR_BR9
#define GPIO_BSRR_BR_10                     GPIO_BSRR_BR10
#define GPIO_BSRR_BR_11                     GPIO_BSRR_BR11
#define GPIO_BSRR_BR_12                     GPIO_BSRR_BR12
#define GPIO_BSRR_BR_13                     GPIO_BSRR_BR13
#define GPIO_BSRR_BR_14                     GPIO_BSRR_BR14
#define GPIO_BSRR_BR_15                     GPIO_BSRR_BR15

/****************** Bit definition for GPIO_LCKR register *********************/
#define GPIO_LCKR_LCK0                      ((uint32_t)0x00000001U)
#define GPIO_LCKR_LCK1                      ((uint32_t)0x00000002U)
#define GPIO_LCKR_LCK2                      ((uint32_t)0x00000004U)
#define GPIO_LCKR_LCK3                      ((uint32_t)0x00000008U)
#define GPIO_LCKR_LCK4                      ((uint32_t)0x00000010U)
#define GPIO_LCKR_LCK5                      ((uint32_t)0x00000020U)
#define GPIO_LCKR_LCK6                      ((uint32_t)0x00000040U)
#define GPIO_LCKR_LCK7                      ((uint32_t)0x00000080U)
#define GPIO_LCKR_LCK8                      ((uint32_t)0x00000100U)
#define GPIO_LCKR_LCK9                      ((uint32_t)0x00000200U)
#define GPIO_LCKR_LCK10                     ((uint32_t)0x00000400U)
#define GPIO_LCKR_LCK11                     ((uint32_t)0x00000800U)
#define GPIO_LCKR_LCK12                     ((uint32_t)0x00001000U)
#define GPIO_LCKR_LCK13                     ((uint32_t)0x00002000U)
#define GPIO_LCKR_LCK14                     ((uint32_t)0x00004000U)
#define GPIO_LCKR_LCK15                     ((uint32_t)0x00008000U)
#define GPIO_LCKR_LCKK                      ((uint32_t)0x00010000U)

/****************** Bit definition for GPIO_AFRL register *********************/
#define GPIO_AFRL_AFSEL0                    ((uint32_t)0x0000000FU)
#define GPIO_AFRL_AFSEL0_0                  ((uint32_t)0x00000001U)
#define GPIO_AFRL_AFSEL0_1                  ((uint32_t)0x00000002U)
#define GPIO_AFRL_AFSEL0_2                  ((uint32_t)0x00000004U)
#define GPIO_AFRL_AFSEL0_3                  ((uint32_t)0x00000008U)
#define GPIO_AFRL_AFSEL1                    ((uint32_t)0x000000F0U)
#define GPIO_AFRL_AFSEL1_0                  ((uint32_t)0x00000010U)
#define GPIO_AFRL_AFSEL1_1                  ((uint32_t)0x00000020U)
#define GPIO_AFRL_AFSEL1_2                  ((uint32_t)0x00000040U)
#define GPIO_AFRL_AFSEL1_3                  ((uint32_t)0x00000080U)
#define GPIO_AFRL_AFSEL2                    ((uint32_t)0x00000F00U)
#define GPIO_AFRL_AFSEL2_0                  ((uint32_t)0x00000100U)
#define GPIO_AFRL_AFSEL2_1                  ((uint32_t)0x00000200U)
#define GPIO_AFRL_AFSEL2_2                  ((uint32_t)0x00000400U)
#define GPIO_AFRL_AFSEL2_3                  ((uint32_t)0x00000800U)
#define GPIO_AFRL_AFSEL3                    ((uint32_t)0x0000F000U)
#define GPIO_AFRL_AFSEL3_0                  ((uint32_t)0x00001000U)
#define GPIO_AFRL_AFSEL3_1                  ((uint32_t)0x00002000U)
#define GPIO_AFRL_AFSEL3_2                  ((uint32_t)0x00004000U)
#define GPIO_AFRL_AFSEL3_3                  ((uint32_t)0x00008000U)
#define GPIO_AFRL_AFSEL4                    ((uint32_t)0x000F0000U)
#define GPIO_AFRL_AFSEL4_0                  ((uint32_t)0x00010000U)
#define GPIO_AFRL_AFSEL4_1                  ((uint32_t)0x00020000U)
#define GPIO_AFRL_AFSEL4_2                  ((uint32_t)0x00040000U)
#define GPIO_AFRL_AFSEL4_3                  ((uint32_t)0x00080000U)
#define GPIO_AFRL_AFSEL5                    ((uint32_t)0x00F00000U)
#define GPIO_AFRL_AFSEL5_0                  ((uint32_t)0x00100000U)
#define GPIO_AFRL_AFSEL5_1                  ((uint32_t)0x00200000U)
#define GPIO_AFRL_AFSEL5_2                  ((uint32_t)0x00400000U)
#define GPIO_AFRL_AFSEL5_3                  ((uint32_t)0x00800000U)
#define GPIO_AFRL_AFSEL6                    ((uint32_t)0x0F000000U)
#define GPIO_AFRL_AFSEL6_0                  ((uint32_t)0x01000000U)
#define GPIO_AFRL_AFSEL6_1                  ((uint32_t)0x02000000U)
#define GPIO_AFRL_AFSEL6_2                  ((uint32_t)0x04000000U)
#define GPIO_AFRL_AFSEL6_3                  ((uint32_t)0x08000000U)
#define GPIO_AFRL_AFSEL7                    ((uint32_t)0xF0000000U)
#define GPIO_AFRL_AFSEL7_0                  ((uint32_t)0x10000000U)
#define GPIO_AFRL_AFSEL7_1                  ((uint32_t)0x20000000U)
#define GPIO_AFRL_AFSEL7_2                  ((uint32_t)0x40000000U)
#define GPIO_AFRL_AFSEL7_3                  ((uint32_t)0x80000000U)

/* Legacy defines */
#define GPIO_AFRL_AFRL0                      GPIO_AFRL_AFSEL0
#define GPIO_AFRL_AFRL1                      GPIO_AFRL_AFSEL1
#define GPIO_AFRL_AFRL2                      GPIO_AFRL_AFSEL2
#define GPIO_AFRL_AFRL3                      GPIO_AFRL_AFSEL3
#define GPIO_AFRL_AFRL4                      GPIO_AFRL_AFSEL4
#define GPIO_AFRL_AFRL5                      GPIO_AFRL_AFSEL5
#define GPIO_AFRL_AFRL6                      GPIO_AFRL_AFSEL6
#define GPIO_AFRL_AFRL7                      GPIO_AFRL_AFSEL7

/****************** Bit definition for GPIO_AFRH register *********************/
#define GPIO_AFRH_AFSEL8                    ((uint32_t)0x0000000FU)
#define GPIO_AFRH_AFSEL8_0                  ((uint32_t)0x00000001U)
#define GPIO_AFRH_AFSEL8_1                  ((uint32_t)0x00000002U)
#define GPIO_AFRH_AFSEL8_2                  ((uint32_t)0x00000004U)
#define GPIO_AFRH_AFSEL8_3                  ((uint32_t)0x00000008U)
#define GPIO_AFRH_AFSEL9                    ((uint32_t)0x000000F0U)
#define GPIO_AFRH_AFSEL9_0                  ((uint32_t)0x00000010U)
#define GPIO_AFRH_AFSEL9_1                  ((uint32_t)0x00000020U)
#define GPIO_AFRH_AFSEL9_2                  ((uint32_t)0x00000040U)
#define GPIO_AFRH_AFSEL9_3                  ((uint32_t)0x00000080U)
#define GPIO_AFRH_AFSEL10                   ((uint32_t)0x00000F00U)
#define GPIO_AFRH_AFSEL10_0                 ((uint32_t)0x00000100U)
#define GPIO_AFRH_AFSEL10_1                 ((uint32_t)0x00000200U)
#define GPIO_AFRH_AFSEL10_2                 ((uint32_t)0x00000400U)
#define GPIO_AFRH_AFSEL10_3                 ((uint32_t)0x00000800U)
#define GPIO_AFRH_AFSEL11                   ((uint32_t)0x0000F000U)
#define GPIO_AFRH_AFSEL11_0                 ((uint32_t)0x00001000U)
#define GPIO_AFRH_AFSEL11_1                 ((uint32_t)0x00002000U)
#define GPIO_AFRH_AFSEL11_2                 ((uint32_t)0x00004000U)
#define GPIO_AFRH_AFSEL11_3                 ((uint32_t)0x00008000U)
#define GPIO_AFRH_AFSEL12                   ((uint32_t)0x000F0000U)
#define GPIO_AFRH_AFSEL12_0                 ((uint32_t)0x00010000U)
#define GPIO_AFRH_AFSEL12_1                 ((uint32_t)0x00020000U)
#define GPIO_AFRH_AFSEL12_2                 ((uint32_t)0x00040000U)
#define GPIO_AFRH_AFSEL12_3                 ((uint32_t)0x00080000U)
#define GPIO_AFRH_AFSEL13                   ((uint32_t)0x00F00000U)
#define GPIO_AFRH_AFSEL13_0                 ((uint32_t)0x00100000U)
#define GPIO_AFRH_AFSEL13_1                 ((uint32_t)0x00200000U)
#define GPIO_AFRH_AFSEL13_2                 ((uint32_t)0x00400000U)
#define GPIO_AFRH_AFSEL13_3                 ((uint32_t)0x00800000U)
#define GPIO_AFRH_AFSEL14                   ((uint32_t)0x0F000000U)
#define GPIO_AFRH_AFSEL14_0                 ((uint32_t)0x01000000U)
#define GPIO_AFRH_AFSEL14_1                 ((uint32_t)0x02000000U)
#define GPIO_AFRH_AFSEL14_2                 ((uint32_t)0x04000000U)
#define GPIO_AFRH_AFSEL14_3                 ((uint32_t)0x08000000U)
#define GPIO_AFRH_AFSEL15                   ((uint32_t)0xF0000000U)
#define GPIO_AFRH_AFSEL15_0                 ((uint32_t)0x10000000U)
#define GPIO_AFRH_AFSEL15_1                 ((uint32_t)0x20000000U)
#define GPIO_AFRH_AFSEL15_2                 ((uint32_t)0x40000000U)
#define GPIO_AFRH_AFSEL15_3                 ((uint32_t)0x80000000U)

/* Legacy defines */
#define GPIO_AFRH_AFRH0                      GPIO_AFRH_AFSEL8
#define GPIO_AFRH_AFRH1                      GPIO_AFRH_AFSEL9
#define GPIO_AFRH_AFRH2                      GPIO_AFRH_AFSEL10
#define GPIO_AFRH_AFRH3                      GPIO_AFRH_AFSEL11
#define GPIO_AFRH_AFRH4                      GPIO_AFRH_AFSEL12
#define GPIO_AFRH_AFRH5                      GPIO_AFRH_AFSEL13
#define GPIO_AFRH_AFRH6                      GPIO_AFRH_AFSEL14
#define GPIO_AFRH_AFRH7                      GPIO_AFRH_AFSEL15

/******************  Bits definition for GPIO_BRR register  ******************/
#define GPIO_BRR_BR0                        ((uint32_t)0x00000001U)
#define GPIO_BRR_BR1                        ((uint32_t)0x00000002U)
#define GPIO_BRR_BR2                        ((uint32_t)0x00000004U)
#define GPIO_BRR_BR3                        ((uint32_t)0x00000008U)
#define GPIO_BRR_BR4                        ((uint32_t)0x00000010U)
#define GPIO_BRR_BR5                        ((uint32_t)0x00000020U)
#define GPIO_BRR_BR6                        ((uint32_t)0x00000040U)
#define GPIO_BRR_BR7                        ((uint32_t)0x00000080U)
#define GPIO_BRR_BR8                        ((uint32_t)0x00000100U)
#define GPIO_BRR_BR9                        ((uint32_t)0x00000200U)
#define GPIO_BRR_BR10                       ((uint32_t)0x00000400U)
#define GPIO_BRR_BR11                       ((uint32_t)0x00000800U)
#define GPIO_BRR_BR12                       ((uint32_t)0x00001000U)
#define GPIO_BRR_BR13                       ((uint32_t)0x00002000U)
#define GPIO_BRR_BR14                       ((uint32_t)0x00004000U)
#define GPIO_BRR_BR15                       ((uint32_t)0x00008000U)

/* Legacy defines */
#define GPIO_BRR_BR_0                       GPIO_BRR_BR0
#define GPIO_BRR_BR_1                       GPIO_BRR_BR1
#define GPIO_BRR_BR_2                       GPIO_BRR_BR2
#define GPIO_BRR_BR_3                       GPIO_BRR_BR3
#define GPIO_BRR_BR_4                       GPIO_BRR_BR4
#define GPIO_BRR_BR_5                       GPIO_BRR_BR5
#define GPIO_BRR_BR_6                       GPIO_BRR_BR6
#define GPIO_BRR_BR_7                       GPIO_BRR_BR7
#define GPIO_BRR_BR_8                       GPIO_BRR_BR8
#define GPIO_BRR_BR_9                       GPIO_BRR_BR9
#define GPIO_BRR_BR_10                      GPIO_BRR_BR10
#define GPIO_BRR_BR_11                      GPIO_BRR_BR11
#define GPIO_BRR_BR_12                      GPIO_BRR_BR12
#define GPIO_BRR_BR_13                      GPIO_BRR_BR13
#define GPIO_BRR_BR_14                      GPIO_BRR_BR14
#define GPIO_BRR_BR_15                      GPIO_BRR_BR15



/******************************************************************************/
/*                                                                            */
/*                      Inter-integrated Circuit Interface (I2C)              */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for I2C_CR1 register  *******************/
#define  I2C_CR1_PE                          ((uint32_t)0x00000001U)        /*!< Peripheral enable                   */
#define  I2C_CR1_TXIE                        ((uint32_t)0x00000002U)        /*!< TX interrupt enable                 */
#define  I2C_CR1_RXIE                        ((uint32_t)0x00000004U)        /*!< RX interrupt enable                 */
#define  I2C_CR1_ADDRIE                      ((uint32_t)0x00000008U)        /*!< Address match interrupt enable      */
#define  I2C_CR1_NACKIE                      ((uint32_t)0x00000010U)        /*!< NACK received interrupt enable      */
#define  I2C_CR1_STOPIE                      ((uint32_t)0x00000020U)        /*!< STOP detection interrupt enable     */
#define  I2C_CR1_TCIE                        ((uint32_t)0x00000040U)        /*!< Transfer complete interrupt enable  */
#define  I2C_CR1_ERRIE                       ((uint32_t)0x00000080U)        /*!< Errors interrupt enable             */
#define  I2C_CR1_DNF                         ((uint32_t)0x00000F00U)        /*!< Digital noise filter                */
#define  I2C_CR1_ANFOFF                      ((uint32_t)0x00001000U)        /*!< Analog noise filter OFF             */
#define  I2C_CR1_SWRST                       ((uint32_t)0x00002000U)        /*!< Software reset                      */
#define  I2C_CR1_TXDMAEN                     ((uint32_t)0x00004000U)        /*!< DMA transmission requests enable    */
#define  I2C_CR1_RXDMAEN                     ((uint32_t)0x00008000U)        /*!< DMA reception requests enable       */
#define  I2C_CR1_SBC                         ((uint32_t)0x00010000U)        /*!< Slave byte control                  */
#define  I2C_CR1_NOSTRETCH                   ((uint32_t)0x00020000U)        /*!< Clock stretching disable            */
#define  I2C_CR1_WUPEN                       ((uint32_t)0x00040000U)        /*!< Wakeup from STOP enable             */
#define  I2C_CR1_GCEN                        ((uint32_t)0x00080000U)        /*!< General call enable                 */
#define  I2C_CR1_SMBHEN                      ((uint32_t)0x00100000U)        /*!< SMBus host address enable           */
#define  I2C_CR1_SMBDEN                      ((uint32_t)0x00200000U)        /*!< SMBus device default address enable */
#define  I2C_CR1_ALERTEN                     ((uint32_t)0x00400000U)        /*!< SMBus alert enable                  */
#define  I2C_CR1_PECEN                       ((uint32_t)0x00800000U)        /*!< PEC enable                          */

/******************  Bit definition for I2C_CR2 register  ********************/
#define  I2C_CR2_SADD                        ((uint32_t)0x000003FFU)        /*!< Slave address (master mode)                             */
#define  I2C_CR2_RD_WRN                      ((uint32_t)0x00000400U)        /*!< Transfer direction (master mode)                        */
#define  I2C_CR2_ADD10                       ((uint32_t)0x00000800U)        /*!< 10-bit addressing mode (master mode)                    */
#define  I2C_CR2_HEAD10R                     ((uint32_t)0x00001000U)        /*!< 10-bit address header only read direction (master mode) */
#define  I2C_CR2_START                       ((uint32_t)0x00002000U)        /*!< START generation                                        */
#define  I2C_CR2_STOP                        ((uint32_t)0x00004000U)        /*!< STOP generation (master mode)                           */
#define  I2C_CR2_NACK                        ((uint32_t)0x00008000U)        /*!< NACK generation (slave mode)                            */
#define  I2C_CR2_NBYTES                      ((uint32_t)0x00FF0000U)        /*!< Number of bytes                                         */
#define  I2C_CR2_RELOAD                      ((uint32_t)0x01000000U)        /*!< NBYTES reload mode                                      */
#define  I2C_CR2_AUTOEND                     ((uint32_t)0x02000000U)        /*!< Automatic end mode (master mode)                        */
#define  I2C_CR2_PECBYTE                     ((uint32_t)0x04000000U)        /*!< Packet error checking byte                              */

/*******************  Bit definition for I2C_OAR1 register  ******************/
#define  I2C_OAR1_OA1                        ((uint32_t)0x000003FFU)        /*!< Interface own address 1   */
#define  I2C_OAR1_OA1MODE                    ((uint32_t)0x00000400U)        /*!< Own address 1 10-bit mode */
#define  I2C_OAR1_OA1EN                      ((uint32_t)0x00008000U)        /*!< Own address 1 enable      */

/*******************  Bit definition for I2C_OAR2 register  ******************/
#define  I2C_OAR2_OA2                        ((uint32_t)0x000000FEU)        /*!< Interface own address 2                        */
#define  I2C_OAR2_OA2MSK                     ((uint32_t)0x00000700U)        /*!< Own address 2 masks                            */
#define  I2C_OAR2_OA2NOMASK                  ((uint32_t)0x00000000U)        /*!< No mask                                        */
#define  I2C_OAR2_OA2MASK01                  ((uint32_t)0x00000100U)        /*!< OA2[1] is masked, Only OA2[7:2] are compared   */
#define  I2C_OAR2_OA2MASK02                  ((uint32_t)0x00000200U)        /*!< OA2[2:1] is masked, Only OA2[7:3] are compared */
#define  I2C_OAR2_OA2MASK03                  ((uint32_t)0x00000300U)        /*!< OA2[3:1] is masked, Only OA2[7:4] are compared */
#define  I2C_OAR2_OA2MASK04                  ((uint32_t)0x00000400U)        /*!< OA2[4:1] is masked, Only OA2[7:5] are compared */
#define  I2C_OAR2_OA2MASK05                  ((uint32_t)0x00000500U)        /*!< OA2[5:1] is masked, Only OA2[7:6] are compared */
#define  I2C_OAR2_OA2MASK06                  ((uint32_t)0x00000600U)        /*!< OA2[6:1] is masked, Only OA2[7] are compared   */
#define  I2C_OAR2_OA2MASK07                  ((uint32_t)0x00000700U)        /*!< OA2[7:1] is masked, No comparison is done      */
#define  I2C_OAR2_OA2EN                      ((uint32_t)0x00008000U)        /*!< Own address 2 enable                           */

/*******************  Bit definition for I2C_TIMINGR register *******************/
#define  I2C_TIMINGR_SCLL                    ((uint32_t)0x000000FFU)        /*!< SCL low period (master mode)  */
#define  I2C_TIMINGR_SCLH                    ((uint32_t)0x0000FF00U)        /*!< SCL high period (master mode) */
#define  I2C_TIMINGR_SDADEL                  ((uint32_t)0x000F0000U)        /*!< Data hold time                */
#define  I2C_TIMINGR_SCLDEL                  ((uint32_t)0x00F00000U)        /*!< Data setup time               */
#define  I2C_TIMINGR_PRESC                   ((uint32_t)0xF0000000U)        /*!< Timings prescaler             */

/******************* Bit definition for I2C_TIMEOUTR register *******************/
#define  I2C_TIMEOUTR_TIMEOUTA               ((uint32_t)0x00000FFFU)        /*!< Bus timeout A                 */
#define  I2C_TIMEOUTR_TIDLE                  ((uint32_t)0x00001000U)        /*!< Idle clock timeout detection  */
#define  I2C_TIMEOUTR_TIMOUTEN               ((uint32_t)0x00008000U)        /*!< Clock timeout enable          */
#define  I2C_TIMEOUTR_TIMEOUTB               ((uint32_t)0x0FFF0000U)        /*!< Bus timeout B                 */
#define  I2C_TIMEOUTR_TEXTEN                 ((uint32_t)0x80000000U)        /*!< Extended clock timeout enable */

/******************  Bit definition for I2C_ISR register  *********************/
#define  I2C_ISR_TXE                         ((uint32_t)0x00000001U)        /*!< Transmit data register empty    */
#define  I2C_ISR_TXIS                        ((uint32_t)0x00000002U)        /*!< Transmit interrupt status       */
#define  I2C_ISR_RXNE                        ((uint32_t)0x00000004U)        /*!< Receive data register not empty */
#define  I2C_ISR_ADDR                        ((uint32_t)0x00000008U)        /*!< Address matched (slave mode)    */
#define  I2C_ISR_NACKF                       ((uint32_t)0x00000010U)        /*!< NACK received flag              */
#define  I2C_ISR_STOPF                       ((uint32_t)0x00000020U)        /*!< STOP detection flag             */
#define  I2C_ISR_TC                          ((uint32_t)0x00000040U)        /*!< Transfer complete (master mode) */
#define  I2C_ISR_TCR                         ((uint32_t)0x00000080U)        /*!< Transfer complete reload        */
#define  I2C_ISR_BERR                        ((uint32_t)0x00000100U)        /*!< Bus error                       */
#define  I2C_ISR_ARLO                        ((uint32_t)0x00000200U)        /*!< Arbitration lost                */
#define  I2C_ISR_OVR                         ((uint32_t)0x00000400U)        /*!< Overrun/Underrun                */
#define  I2C_ISR_PECERR                      ((uint32_t)0x00000800U)        /*!< PEC error in reception          */
#define  I2C_ISR_TIMEOUT                     ((uint32_t)0x00001000U)        /*!< Timeout or Tlow detection flag  */
#define  I2C_ISR_ALERT                       ((uint32_t)0x00002000U)        /*!< SMBus alert                     */
#define  I2C_ISR_BUSY                        ((uint32_t)0x00008000U)        /*!< Bus busy                        */
#define  I2C_ISR_DIR                         ((uint32_t)0x00010000U)        /*!< Transfer direction (slave mode) */
#define  I2C_ISR_ADDCODE                     ((uint32_t)0x00FE0000U)        /*!< Address match code (slave mode) */

/******************  Bit definition for I2C_ICR register  *********************/
#define  I2C_ICR_ADDRCF                      ((uint32_t)0x00000008U)        /*!< Address matched clear flag  */
#define  I2C_ICR_NACKCF                      ((uint32_t)0x00000010U)        /*!< NACK clear flag             */
#define  I2C_ICR_STOPCF                      ((uint32_t)0x00000020U)        /*!< STOP detection clear flag   */
#define  I2C_ICR_BERRCF                      ((uint32_t)0x00000100U)        /*!< Bus error clear flag        */
#define  I2C_ICR_ARLOCF                      ((uint32_t)0x00000200U)        /*!< Arbitration lost clear flag */
#define  I2C_ICR_OVRCF                       ((uint32_t)0x00000400U)        /*!< Overrun/Underrun clear flag */
#define  I2C_ICR_PECCF                       ((uint32_t)0x00000800U)        /*!< PAC error clear flag        */
#define  I2C_ICR_TIMOUTCF                    ((uint32_t)0x00001000U)        /*!< Timeout clear flag          */
#define  I2C_ICR_ALERTCF                     ((uint32_t)0x00002000U)        /*!< Alert clear flag            */

/******************  Bit definition for I2C_PECR register  *********************/
#define  I2C_PECR_PEC                        ((uint32_t)0x000000FFU)        /*!< PEC register */

/******************  Bit definition for I2C_RXDR register  *********************/
#define  I2C_RXDR_RXDATA                     ((uint32_t)0x000000FFU)        /*!< 8-bit receive data */

/******************  Bit definition for I2C_TXDR register  *********************/
#define  I2C_TXDR_TXDATA                     ((uint32_t)0x000000FFU)        /*!< 8-bit transmit data */

/******************************************************************************/
/*                                                                            */
/*                           Independent WATCHDOG                             */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for IWDG_KR register  ********************/
#define  IWDG_KR_KEY                         ((uint32_t)0x0000FFFFU)        /*!<Key value (write only, read 0000h)  */

/*******************  Bit definition for IWDG_PR register  ********************/
#define  IWDG_PR_PR                          ((uint32_t)0x00000007U)        /*!<PR[2:0] (Prescaler divider)         */
#define  IWDG_PR_PR_0                        ((uint32_t)0x00000001U)        /*!<Bit 0 */
#define  IWDG_PR_PR_1                        ((uint32_t)0x00000002U)        /*!<Bit 1 */
#define  IWDG_PR_PR_2                        ((uint32_t)0x00000004U)        /*!<Bit 2 */

/*******************  Bit definition for IWDG_RLR register  *******************/
#define  IWDG_RLR_RL                         ((uint32_t)0x00000FFFU)        /*!<Watchdog counter reload value        */

/*******************  Bit definition for IWDG_SR register  ********************/
#define  IWDG_SR_PVU                         ((uint32_t)0x00000001U)        /*!< Watchdog prescaler value update */
#define  IWDG_SR_RVU                         ((uint32_t)0x00000002U)        /*!< Watchdog counter reload value update */
#define  IWDG_SR_WVU                         ((uint32_t)0x00000004U)        /*!< Watchdog counter window value update */

/*******************  Bit definition for IWDG_KR register  ********************/
#define  IWDG_WINR_WIN                       ((uint32_t)0x00000FFFU)        /*!< Watchdog counter window value */

/******************************************************************************/
/*                                                                            */
/*                                     Firewall                               */
/*                                                                            */
/******************************************************************************/

/*******Bit definition for CSSA;CSL;NVDSSA;NVDSL;VDSSA;VDSL;LSSA;LSL register */
#define  FW_CSSA_ADD                   ((uint32_t)0x00FFFF00U)        /*!< Code Segment Start Address */
#define  FW_CSL_LENG                   ((uint32_t)0x003FFF00U)        /*!< Code Segment Length        */
#define  FW_NVDSSA_ADD                 ((uint32_t)0x00FFFF00U)        /*!< Non Volatile Dat Segment Start Address */
#define  FW_NVDSL_LENG                 ((uint32_t)0x003FFF00U)        /*!< Non Volatile Data Segment Length */
#define  FW_VDSSA_ADD                  ((uint32_t)0x0001FFC0U)        /*!< Volatile Data Segment Start Address */
#define  FW_VDSL_LENG                  ((uint32_t)0x0001FFC0U)        /*!< Volatile Data Segment Length */
#define  FW_LSSA_ADD                   ((uint32_t)0x0007FF80U)        /*!< Library Segment Start Address*/
#define  FW_LSL_LENG                   ((uint32_t)0x0007FF80U)        /*!< Library Segment Length*/

/**************************Bit definition for CR register *********************/
#define  FW_CR_FPA                     ((uint32_t)0x00000001U)         /*!< Firewall Pre Arm*/
#define  FW_CR_VDS                     ((uint32_t)0x00000002U)         /*!< Volatile Data Sharing*/
#define  FW_CR_VDE                     ((uint32_t)0x00000004U)         /*!< Volatile Data Execution*/

/******************************************************************************/
/*                                                                            */
/*                             Power Control                                  */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for PWR_CR1 register  ********************/

#define  PWR_CR1_LPR                         ((uint32_t)0x00004000U)     /*!< Regulator low-power mode */
#define  PWR_CR1_VOS                         ((uint32_t)0x00000600U)     /*!< VOS[1:0] bits (Regulator voltage scaling output selection) */
#define  PWR_CR1_VOS_0                       ((uint32_t)0x00000200U)     /*!< Bit 0 */
#define  PWR_CR1_VOS_1                       ((uint32_t)0x00000400U)     /*!< Bit 1 */
#define  PWR_CR1_DBP                         ((uint32_t)0x00000100U)     /*!< Disable Back-up domain Protection */
#define  PWR_CR1_LPMS                        ((uint32_t)0x00000007U)     /*!< Low-power mode selection field */
#define  PWR_CR1_LPMS_STOP0                  ((uint32_t)0x00000000U)     /*!< Stop 0 mode */
#define  PWR_CR1_LPMS_STOP1                  ((uint32_t)0x00000001U)     /*!< Stop 1 mode */
#define  PWR_CR1_LPMS_STOP2                  ((uint32_t)0x00000002U)     /*!< Stop 2 mode */
#define  PWR_CR1_LPMS_STANDBY                ((uint32_t)0x00000003U)     /*!< Stand-by mode */
#define  PWR_CR1_LPMS_SHUTDOWN               ((uint32_t)0x00000004U)     /*!< Shut-down mode */


/********************  Bit definition for PWR_CR2 register  ********************/
#define  PWR_CR2_USV                         ((uint32_t)0x00000400U)     /*!< VDD USB Supply Valid */
/*!< PVME  Peripheral Voltage Monitor Enable */
#define  PWR_CR2_PVME                        ((uint32_t)0x000000D0U)     /*!< PVM bits field */
#define  PWR_CR2_PVME4                       ((uint32_t)0x00000080U)     /*!< PVM 4 Enable */
#define  PWR_CR2_PVME3                       ((uint32_t)0x00000040U)     /*!< PVM 3 Enable */
#define  PWR_CR2_PVME1                       ((uint32_t)0x00000010U)     /*!< PVM 1 Enable */
/*!< PVD level configuration */
#define  PWR_CR2_PLS                         ((uint32_t)0x0000000EU)     /*!< PVD level selection */
#define  PWR_CR2_PLS_LEV0                    ((uint32_t)0x00000000U)     /*!< PVD level 0 */
#define  PWR_CR2_PLS_LEV1                    ((uint32_t)0x00000002U)     /*!< PVD level 1 */
#define  PWR_CR2_PLS_LEV2                    ((uint32_t)0x00000004U)     /*!< PVD level 2 */
#define  PWR_CR2_PLS_LEV3                    ((uint32_t)0x00000006U)     /*!< PVD level 3 */
#define  PWR_CR2_PLS_LEV4                    ((uint32_t)0x00000008U)     /*!< PVD level 4 */
#define  PWR_CR2_PLS_LEV5                    ((uint32_t)0x0000000AU)     /*!< PVD level 5 */
#define  PWR_CR2_PLS_LEV6                    ((uint32_t)0x0000000CU)     /*!< PVD level 6 */
#define  PWR_CR2_PLS_LEV7                    ((uint32_t)0x0000000EU)     /*!< PVD level 7 */
#define  PWR_CR2_PVDE                        ((uint32_t)0x00000001U)     /*!< Power Voltage Detector Enable */

/********************  Bit definition for PWR_CR3 register  ********************/
#define  PWR_CR3_EIWF                        ((uint32_t)0x00008000U)      /*!< Enable Internal Wake-up line */
#define  PWR_CR3_APC                         ((uint32_t)0x00000400U)      /*!< Apply pull-up and pull-down configuration */
#define  PWR_CR3_RRS                         ((uint32_t)0x00000100U)      /*!< SRAM2 Retention in Stand-by mode */
#define  PWR_CR3_EWUP5                       ((uint32_t)0x00000010U)      /*!< Enable Wake-Up Pin 5 */
#define  PWR_CR3_EWUP4                       ((uint32_t)0x00000008U)      /*!< Enable Wake-Up Pin 4 */
#define  PWR_CR3_EWUP3                       ((uint32_t)0x00000004U)      /*!< Enable Wake-Up Pin 3 */
#define  PWR_CR3_EWUP2                       ((uint32_t)0x00000002U)      /*!< Enable Wake-Up Pin 2 */
#define  PWR_CR3_EWUP1                       ((uint32_t)0x00000001U)      /*!< Enable Wake-Up Pin 1 */
#define  PWR_CR3_EWUP                        ((uint32_t)0x0000001FU)      /*!< Enable Wake-Up Pins  */

/********************  Bit definition for PWR_CR4 register  ********************/
#define  PWR_CR4_VBRS                        ((uint32_t)0x00000200U)      /*!< VBAT Battery charging Resistor Selection */
#define  PWR_CR4_VBE                         ((uint32_t)0x00000100U)      /*!< VBAT Battery charging Enable  */
#define  PWR_CR4_WP5                         ((uint32_t)0x00000010U)      /*!< Wake-Up Pin 5 polarity */
#define  PWR_CR4_WP4                         ((uint32_t)0x00000008U)      /*!< Wake-Up Pin 4 polarity */
#define  PWR_CR4_WP3                         ((uint32_t)0x00000004U)      /*!< Wake-Up Pin 3 polarity */
#define  PWR_CR4_WP2                         ((uint32_t)0x00000002U)      /*!< Wake-Up Pin 2 polarity */
#define  PWR_CR4_WP1                         ((uint32_t)0x00000001U)      /*!< Wake-Up Pin 1 polarity */

/********************  Bit definition for PWR_SR1 register  ********************/
#define   PWR_SR1_WUFI                       ((uint32_t)0x00008000U)     /*!< Wake-Up Flag Internal */
#define   PWR_SR1_SBF                        ((uint32_t)0x00000100U)     /*!< Stand-By Flag */
#define   PWR_SR1_WUF                        ((uint32_t)0x0000001FU)     /*!< Wake-up Flags */
#define   PWR_SR1_WUF5                       ((uint32_t)0x00000010U)     /*!< Wake-up Flag 5 */
#define   PWR_SR1_WUF4                       ((uint32_t)0x00000008U)     /*!< Wake-up Flag 4 */
#define   PWR_SR1_WUF3                       ((uint32_t)0x00000004U)     /*!< Wake-up Flag 3 */
#define   PWR_SR1_WUF2                       ((uint32_t)0x00000002U)     /*!< Wake-up Flag 2 */
#define   PWR_SR1_WUF1                       ((uint32_t)0x00000001U)     /*!< Wake-up Flag 1 */

/********************  Bit definition for PWR_SR2 register  ********************/
#define   PWR_SR2_PVMO4                      ((uint32_t)0x00008000U)     /*!< Peripheral Voltage Monitoring Output 4 */
#define   PWR_SR2_PVMO3                      ((uint32_t)0x00004000U)     /*!< Peripheral Voltage Monitoring Output 3 */
#define   PWR_SR2_PVMO1                      ((uint32_t)0x00001000U)     /*!< Peripheral Voltage Monitoring Output 1 */
#define   PWR_SR2_PVDO                       ((uint32_t)0x00000800U)     /*!< Power Voltage Detector Output */
#define   PWR_SR2_VOSF                       ((uint32_t)0x00000400U)     /*!< Voltage Scaling Flag */
#define   PWR_SR2_REGLPF                     ((uint32_t)0x00000200U)     /*!< Low-power Regulator Flag */
#define   PWR_SR2_REGLPS                     ((uint32_t)0x00000100U)     /*!< Low-power Regulator Started */

/********************  Bit definition for PWR_SCR register  ********************/
#define   PWR_SCR_CSBF                       ((uint32_t)0x00000100U)      /*!< Clear Stand-By Flag */
#define   PWR_SCR_CWUF                       ((uint32_t)0x0000001FU)      /*!< Clear Wake-up Flags  */
#define   PWR_SCR_CWUF5                      ((uint32_t)0x00000010U)      /*!< Clear Wake-up Flag 5 */
#define   PWR_SCR_CWUF4                      ((uint32_t)0x00000008U)      /*!< Clear Wake-up Flag 4 */
#define   PWR_SCR_CWUF3                      ((uint32_t)0x00000004U)      /*!< Clear Wake-up Flag 3 */
#define   PWR_SCR_CWUF2                      ((uint32_t)0x00000002U)      /*!< Clear Wake-up Flag 2 */
#define   PWR_SCR_CWUF1                      ((uint32_t)0x00000001U)      /*!< Clear Wake-up Flag 1 */

/********************  Bit definition for PWR_PUCRA register  ********************/
#define   PWR_PUCRA_PA15                     ((uint32_t)0x00008000U)      /*!< Port PA15 Pull-Up set */
#define   PWR_PUCRA_PA13                     ((uint32_t)0x00002000U)      /*!< Port PA13 Pull-Up set */
#define   PWR_PUCRA_PA12                     ((uint32_t)0x00001000U)      /*!< Port PA12 Pull-Up set */
#define   PWR_PUCRA_PA11                     ((uint32_t)0x00000800U)      /*!< Port PA11 Pull-Up set */
#define   PWR_PUCRA_PA10                     ((uint32_t)0x00000400U)      /*!< Port PA10 Pull-Up set */
#define   PWR_PUCRA_PA9                      ((uint32_t)0x00000200U)      /*!< Port PA9 Pull-Up set  */
#define   PWR_PUCRA_PA8                      ((uint32_t)0x00000100U)      /*!< Port PA8 Pull-Up set  */
#define   PWR_PUCRA_PA7                      ((uint32_t)0x00000080U)      /*!< Port PA7 Pull-Up set  */
#define   PWR_PUCRA_PA6                      ((uint32_t)0x00000040U)      /*!< Port PA6 Pull-Up set  */
#define   PWR_PUCRA_PA5                      ((uint32_t)0x00000020U)      /*!< Port PA5 Pull-Up set  */
#define   PWR_PUCRA_PA4                      ((uint32_t)0x00000010U)      /*!< Port PA4 Pull-Up set  */
#define   PWR_PUCRA_PA3                      ((uint32_t)0x00000008U)      /*!< Port PA3 Pull-Up set  */
#define   PWR_PUCRA_PA2                      ((uint32_t)0x00000004U)      /*!< Port PA2 Pull-Up set  */
#define   PWR_PUCRA_PA1                      ((uint32_t)0x00000002U)      /*!< Port PA1 Pull-Up set  */
#define   PWR_PUCRA_PA0                      ((uint32_t)0x00000001U)      /*!< Port PA0 Pull-Up set  */

/********************  Bit definition for PWR_PDCRA register  ********************/
#define   PWR_PDCRA_PA14                     ((uint32_t)0x00004000U)      /*!< Port PA14 Pull-Down set */
#define   PWR_PDCRA_PA12                     ((uint32_t)0x00001000U)      /*!< Port PA12 Pull-Down set */
#define   PWR_PDCRA_PA11                     ((uint32_t)0x00000800U)      /*!< Port PA11 Pull-Down set */
#define   PWR_PDCRA_PA10                     ((uint32_t)0x00000400U)      /*!< Port PA10 Pull-Down set */
#define   PWR_PDCRA_PA9                      ((uint32_t)0x00000200U)      /*!< Port PA9 Pull-Down set  */
#define   PWR_PDCRA_PA8                      ((uint32_t)0x00000100U)      /*!< Port PA8 Pull-Down set  */
#define   PWR_PDCRA_PA7                      ((uint32_t)0x00000080U)      /*!< Port PA7 Pull-Down set  */
#define   PWR_PDCRA_PA6                      ((uint32_t)0x00000040U)      /*!< Port PA6 Pull-Down set  */
#define   PWR_PDCRA_PA5                      ((uint32_t)0x00000020U)      /*!< Port PA5 Pull-Down set  */
#define   PWR_PDCRA_PA4                      ((uint32_t)0x00000010U)      /*!< Port PA4 Pull-Down set  */
#define   PWR_PDCRA_PA3                      ((uint32_t)0x00000008U)      /*!< Port PA3 Pull-Down set  */
#define   PWR_PDCRA_PA2                      ((uint32_t)0x00000004U)      /*!< Port PA2 Pull-Down set  */
#define   PWR_PDCRA_PA1                      ((uint32_t)0x00000002U)      /*!< Port PA1 Pull-Down set  */
#define   PWR_PDCRA_PA0                      ((uint32_t)0x00000001U)      /*!< Port PA0 Pull-Down set  */

/********************  Bit definition for PWR_PUCRB register  ********************/
#define   PWR_PUCRB_PB15                     ((uint32_t)0x00008000U)      /*!< Port PB15 Pull-Up set */
#define   PWR_PUCRB_PB14                     ((uint32_t)0x00004000U)      /*!< Port PB14 Pull-Up set */
#define   PWR_PUCRB_PB13                     ((uint32_t)0x00002000U)      /*!< Port PB13 Pull-Up set */
#define   PWR_PUCRB_PB12                     ((uint32_t)0x00001000U)      /*!< Port PB12 Pull-Up set */
#define   PWR_PUCRB_PB11                     ((uint32_t)0x00000800U)      /*!< Port PB11 Pull-Up set */
#define   PWR_PUCRB_PB10                     ((uint32_t)0x00000400U)      /*!< Port PB10 Pull-Up set */
#define   PWR_PUCRB_PB9                      ((uint32_t)0x00000200U)      /*!< Port PB9 Pull-Up set  */
#define   PWR_PUCRB_PB8                      ((uint32_t)0x00000100U)      /*!< Port PB8 Pull-Up set  */
#define   PWR_PUCRB_PB7                      ((uint32_t)0x00000080U)      /*!< Port PB7 Pull-Up set  */
#define   PWR_PUCRB_PB6                      ((uint32_t)0x00000040U)      /*!< Port PB6 Pull-Up set  */
#define   PWR_PUCRB_PB5                      ((uint32_t)0x00000020U)      /*!< Port PB5 Pull-Up set  */
#define   PWR_PUCRB_PB4                      ((uint32_t)0x00000010U)      /*!< Port PB4 Pull-Up set  */
#define   PWR_PUCRB_PB3                      ((uint32_t)0x00000008U)      /*!< Port PB3 Pull-Up set  */
#define   PWR_PUCRB_PB2                      ((uint32_t)0x00000004U)      /*!< Port PB2 Pull-Up set  */
#define   PWR_PUCRB_PB1                      ((uint32_t)0x00000002U)      /*!< Port PB1 Pull-Up set  */
#define   PWR_PUCRB_PB0                      ((uint32_t)0x00000001U)      /*!< Port PB0 Pull-Up set  */

/********************  Bit definition for PWR_PDCRB register  ********************/
#define   PWR_PDCRB_PB15                     ((uint32_t)0x00008000U)      /*!< Port PB15 Pull-Down set */
#define   PWR_PDCRB_PB14                     ((uint32_t)0x00004000U)      /*!< Port PB14 Pull-Down set */
#define   PWR_PDCRB_PB13                     ((uint32_t)0x00002000U)      /*!< Port PB13 Pull-Down set */
#define   PWR_PDCRB_PB12                     ((uint32_t)0x00001000U)      /*!< Port PB12 Pull-Down set */
#define   PWR_PDCRB_PB11                     ((uint32_t)0x00000800U)      /*!< Port PB11 Pull-Down set */
#define   PWR_PDCRB_PB10                     ((uint32_t)0x00000400U)      /*!< Port PB10 Pull-Down set */
#define   PWR_PDCRB_PB9                      ((uint32_t)0x00000200U)      /*!< Port PB9 Pull-Down set  */
#define   PWR_PDCRB_PB8                      ((uint32_t)0x00000100U)      /*!< Port PB8 Pull-Down set  */
#define   PWR_PDCRB_PB7                      ((uint32_t)0x00000080U)      /*!< Port PB7 Pull-Down set  */
#define   PWR_PDCRB_PB6                      ((uint32_t)0x00000040U)      /*!< Port PB6 Pull-Down set  */
#define   PWR_PDCRB_PB5                      ((uint32_t)0x00000020U)      /*!< Port PB5 Pull-Down set  */
#define   PWR_PDCRB_PB3                      ((uint32_t)0x00000008U)      /*!< Port PB3 Pull-Down set  */
#define   PWR_PDCRB_PB2                      ((uint32_t)0x00000004U)      /*!< Port PB2 Pull-Down set  */
#define   PWR_PDCRB_PB1                      ((uint32_t)0x00000002U)      /*!< Port PB1 Pull-Down set  */
#define   PWR_PDCRB_PB0                      ((uint32_t)0x00000001U)      /*!< Port PB0 Pull-Down set  */

/********************  Bit definition for PWR_PUCRC register  ********************/
#define   PWR_PUCRC_PC15                     ((uint32_t)0x00008000U)      /*!< Port PC15 Pull-Up set */
#define   PWR_PUCRC_PC14                     ((uint32_t)0x00004000U)      /*!< Port PC14 Pull-Up set */
#define   PWR_PUCRC_PC13                     ((uint32_t)0x00002000U)      /*!< Port PC13 Pull-Up set */
#define   PWR_PUCRC_PC12                     ((uint32_t)0x00001000U)      /*!< Port PC12 Pull-Up set */
#define   PWR_PUCRC_PC11                     ((uint32_t)0x00000800U)      /*!< Port PC11 Pull-Up set */
#define   PWR_PUCRC_PC10                     ((uint32_t)0x00000400U)      /*!< Port PC10 Pull-Up set */
#define   PWR_PUCRC_PC9                      ((uint32_t)0x00000200U)      /*!< Port PC9 Pull-Up set  */
#define   PWR_PUCRC_PC8                      ((uint32_t)0x00000100U)      /*!< Port PC8 Pull-Up set  */
#define   PWR_PUCRC_PC7                      ((uint32_t)0x00000080U)      /*!< Port PC7 Pull-Up set  */
#define   PWR_PUCRC_PC6                      ((uint32_t)0x00000040U)      /*!< Port PC6 Pull-Up set  */
#define   PWR_PUCRC_PC5                      ((uint32_t)0x00000020U)      /*!< Port PC5 Pull-Up set  */
#define   PWR_PUCRC_PC4                      ((uint32_t)0x00000010U)      /*!< Port PC4 Pull-Up set  */
#define   PWR_PUCRC_PC3                      ((uint32_t)0x00000008U)      /*!< Port PC3 Pull-Up set  */
#define   PWR_PUCRC_PC2                      ((uint32_t)0x00000004U)      /*!< Port PC2 Pull-Up set  */
#define   PWR_PUCRC_PC1                      ((uint32_t)0x00000002U)      /*!< Port PC1 Pull-Up set  */
#define   PWR_PUCRC_PC0                      ((uint32_t)0x00000001U)      /*!< Port PC0 Pull-Up set  */

/********************  Bit definition for PWR_PDCRC register  ********************/
#define   PWR_PDCRC_PC15                     ((uint32_t)0x00008000U)      /*!< Port PC15 Pull-Down set */
#define   PWR_PDCRC_PC14                     ((uint32_t)0x00004000U)      /*!< Port PC14 Pull-Down set */
#define   PWR_PDCRC_PC13                     ((uint32_t)0x00002000U)      /*!< Port PC13 Pull-Down set */
#define   PWR_PDCRC_PC12                     ((uint32_t)0x00001000U)      /*!< Port PC12 Pull-Down set */
#define   PWR_PDCRC_PC11                     ((uint32_t)0x00000800U)      /*!< Port PC11 Pull-Down set */
#define   PWR_PDCRC_PC10                     ((uint32_t)0x00000400U)      /*!< Port PC10 Pull-Down set */
#define   PWR_PDCRC_PC9                      ((uint32_t)0x00000200U)      /*!< Port PC9 Pull-Down set  */
#define   PWR_PDCRC_PC8                      ((uint32_t)0x00000100U)      /*!< Port PC8 Pull-Down set  */
#define   PWR_PDCRC_PC7                      ((uint32_t)0x00000080U)      /*!< Port PC7 Pull-Down set  */
#define   PWR_PDCRC_PC6                      ((uint32_t)0x00000040U)      /*!< Port PC6 Pull-Down set  */
#define   PWR_PDCRC_PC5                      ((uint32_t)0x00000020U)      /*!< Port PC5 Pull-Down set  */
#define   PWR_PDCRC_PC4                      ((uint32_t)0x00000010U)      /*!< Port PC4 Pull-Down set  */
#define   PWR_PDCRC_PC3                      ((uint32_t)0x00000008U)      /*!< Port PC3 Pull-Down set  */
#define   PWR_PDCRC_PC2                      ((uint32_t)0x00000004U)      /*!< Port PC2 Pull-Down set  */
#define   PWR_PDCRC_PC1                      ((uint32_t)0x00000002U)      /*!< Port PC1 Pull-Down set  */
#define   PWR_PDCRC_PC0                      ((uint32_t)0x00000001U)      /*!< Port PC0 Pull-Down set  */

/********************  Bit definition for PWR_PUCRD register  ********************/
#define   PWR_PUCRD_PD15                     ((uint32_t)0x00008000U)      /*!< Port PD15 Pull-Up set */
#define   PWR_PUCRD_PD14                     ((uint32_t)0x00004000U)      /*!< Port PD14 Pull-Up set */
#define   PWR_PUCRD_PD13                     ((uint32_t)0x00002000U)      /*!< Port PD13 Pull-Up set */
#define   PWR_PUCRD_PD12                     ((uint32_t)0x00001000U)      /*!< Port PD12 Pull-Up set */
#define   PWR_PUCRD_PD11                     ((uint32_t)0x00000800U)      /*!< Port PD11 Pull-Up set */
#define   PWR_PUCRD_PD10                     ((uint32_t)0x00000400U)      /*!< Port PD10 Pull-Up set */
#define   PWR_PUCRD_PD9                      ((uint32_t)0x00000200U)      /*!< Port PD9 Pull-Up set  */
#define   PWR_PUCRD_PD8                      ((uint32_t)0x00000100U)      /*!< Port PD8 Pull-Up set  */
#define   PWR_PUCRD_PD7                      ((uint32_t)0x00000080U)      /*!< Port PD7 Pull-Up set  */
#define   PWR_PUCRD_PD6                      ((uint32_t)0x00000040U)      /*!< Port PD6 Pull-Up set  */
#define   PWR_PUCRD_PD5                      ((uint32_t)0x00000020U)      /*!< Port PD5 Pull-Up set  */
#define   PWR_PUCRD_PD4                      ((uint32_t)0x00000010U)      /*!< Port PD4 Pull-Up set  */
#define   PWR_PUCRD_PD3                      ((uint32_t)0x00000008U)      /*!< Port PD3 Pull-Up set  */
#define   PWR_PUCRD_PD2                      ((uint32_t)0x00000004U)      /*!< Port PD2 Pull-Up set  */
#define   PWR_PUCRD_PD1                      ((uint32_t)0x00000002U)      /*!< Port PD1 Pull-Up set  */
#define   PWR_PUCRD_PD0                      ((uint32_t)0x00000001U)      /*!< Port PD0 Pull-Up set  */

/********************  Bit definition for PWR_PDCRD register  ********************/
#define   PWR_PDCRD_PD15                     ((uint32_t)0x00008000U)      /*!< Port PD15 Pull-Down set */
#define   PWR_PDCRD_PD14                     ((uint32_t)0x00004000U)      /*!< Port PD14 Pull-Down set */
#define   PWR_PDCRD_PD13                     ((uint32_t)0x00002000U)      /*!< Port PD13 Pull-Down set */
#define   PWR_PDCRD_PD12                     ((uint32_t)0x00001000U)      /*!< Port PD12 Pull-Down set */
#define   PWR_PDCRD_PD11                     ((uint32_t)0x00000800U)      /*!< Port PD11 Pull-Down set */
#define   PWR_PDCRD_PD10                     ((uint32_t)0x00000400U)      /*!< Port PD10 Pull-Down set */
#define   PWR_PDCRD_PD9                      ((uint32_t)0x00000200U)      /*!< Port PD9 Pull-Down set  */
#define   PWR_PDCRD_PD8                      ((uint32_t)0x00000100U)      /*!< Port PD8 Pull-Down set  */
#define   PWR_PDCRD_PD7                      ((uint32_t)0x00000080U)      /*!< Port PD7 Pull-Down set  */
#define   PWR_PDCRD_PD6                      ((uint32_t)0x00000040U)      /*!< Port PD6 Pull-Down set  */
#define   PWR_PDCRD_PD5                      ((uint32_t)0x00000020U)      /*!< Port PD5 Pull-Down set  */
#define   PWR_PDCRD_PD4                      ((uint32_t)0x00000010U)      /*!< Port PD4 Pull-Down set  */
#define   PWR_PDCRD_PD3                      ((uint32_t)0x00000008U)      /*!< Port PD3 Pull-Down set  */
#define   PWR_PDCRD_PD2                      ((uint32_t)0x00000004U)      /*!< Port PD2 Pull-Down set  */
#define   PWR_PDCRD_PD1                      ((uint32_t)0x00000002U)      /*!< Port PD1 Pull-Down set  */
#define   PWR_PDCRD_PD0                      ((uint32_t)0x00000001U)      /*!< Port PD0 Pull-Down set  */

/********************  Bit definition for PWR_PUCRE register  ********************/
#define   PWR_PUCRE_PE15                     ((uint32_t)0x00008000U)      /*!< Port PE15 Pull-Up set */
#define   PWR_PUCRE_PE14                     ((uint32_t)0x00004000U)      /*!< Port PE14 Pull-Up set */
#define   PWR_PUCRE_PE13                     ((uint32_t)0x00002000U)      /*!< Port PE13 Pull-Up set */
#define   PWR_PUCRE_PE12                     ((uint32_t)0x00001000U)      /*!< Port PE12 Pull-Up set */
#define   PWR_PUCRE_PE11                     ((uint32_t)0x00000800U)      /*!< Port PE11 Pull-Up set */
#define   PWR_PUCRE_PE10                     ((uint32_t)0x00000400U)      /*!< Port PE10 Pull-Up set */
#define   PWR_PUCRE_PE9                      ((uint32_t)0x00000200U)      /*!< Port PE9 Pull-Up set  */
#define   PWR_PUCRE_PE8                      ((uint32_t)0x00000100U)      /*!< Port PE8 Pull-Up set  */
#define   PWR_PUCRE_PE7                      ((uint32_t)0x00000080U)      /*!< Port PE7 Pull-Up set  */
#define   PWR_PUCRE_PE6                      ((uint32_t)0x00000040U)      /*!< Port PE6 Pull-Up set  */
#define   PWR_PUCRE_PE5                      ((uint32_t)0x00000020U)      /*!< Port PE5 Pull-Up set  */
#define   PWR_PUCRE_PE4                      ((uint32_t)0x00000010U)      /*!< Port PE4 Pull-Up set  */
#define   PWR_PUCRE_PE3                      ((uint32_t)0x00000008U)      /*!< Port PE3 Pull-Up set  */
#define   PWR_PUCRE_PE2                      ((uint32_t)0x00000004U)      /*!< Port PE2 Pull-Up set  */
#define   PWR_PUCRE_PE1                      ((uint32_t)0x00000002U)      /*!< Port PE1 Pull-Up set  */
#define   PWR_PUCRE_PE0                      ((uint32_t)0x00000001U)      /*!< Port PE0 Pull-Up set  */

/********************  Bit definition for PWR_PDCRE register  ********************/
#define   PWR_PDCRE_PE15                     ((uint32_t)0x00008000U)      /*!< Port PE15 Pull-Down set */
#define   PWR_PDCRE_PE14                     ((uint32_t)0x00004000U)      /*!< Port PE14 Pull-Down set */
#define   PWR_PDCRE_PE13                     ((uint32_t)0x00002000U)      /*!< Port PE13 Pull-Down set */
#define   PWR_PDCRE_PE12                     ((uint32_t)0x00001000U)      /*!< Port PE12 Pull-Down set */
#define   PWR_PDCRE_PE11                     ((uint32_t)0x00000800U)      /*!< Port PE11 Pull-Down set */
#define   PWR_PDCRE_PE10                     ((uint32_t)0x00000400U)      /*!< Port PE10 Pull-Down set */
#define   PWR_PDCRE_PE9                      ((uint32_t)0x00000200U)      /*!< Port PE9 Pull-Down set  */
#define   PWR_PDCRE_PE8                      ((uint32_t)0x00000100U)      /*!< Port PE8 Pull-Down set  */
#define   PWR_PDCRE_PE7                      ((uint32_t)0x00000080U)      /*!< Port PE7 Pull-Down set  */
#define   PWR_PDCRE_PE6                      ((uint32_t)0x00000040U)      /*!< Port PE6 Pull-Down set  */
#define   PWR_PDCRE_PE5                      ((uint32_t)0x00000020U)      /*!< Port PE5 Pull-Down set  */
#define   PWR_PDCRE_PE4                      ((uint32_t)0x00000010U)      /*!< Port PE4 Pull-Down set  */
#define   PWR_PDCRE_PE3                      ((uint32_t)0x00000008U)      /*!< Port PE3 Pull-Down set  */
#define   PWR_PDCRE_PE2                      ((uint32_t)0x00000004U)      /*!< Port PE2 Pull-Down set  */
#define   PWR_PDCRE_PE1                      ((uint32_t)0x00000002U)      /*!< Port PE1 Pull-Down set  */
#define   PWR_PDCRE_PE0                      ((uint32_t)0x00000001U)      /*!< Port PE0 Pull-Down set  */


/********************  Bit definition for PWR_PUCRH register  ********************/
#define   PWR_PUCRH_PH3                      ((uint32_t)0x00000008U)      /*!< Port PH3 Pull-Down set  */
#define   PWR_PUCRH_PH1                      ((uint32_t)0x00000002U)      /*!< Port PH1 Pull-Up set  */
#define   PWR_PUCRH_PH0                      ((uint32_t)0x00000001U)      /*!< Port PH0 Pull-Up set  */

/********************  Bit definition for PWR_PDCRH register  ********************/
#define   PWR_PDCRH_PH3                      ((uint32_t)0x00000008U)      /*!< Port PH3 Pull-Down set  */
#define   PWR_PDCRH_PH1                      ((uint32_t)0x00000002U)      /*!< Port PH1 Pull-Down set  */
#define   PWR_PDCRH_PH0                      ((uint32_t)0x00000001U)      /*!< Port PH0 Pull-Down set  */


/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/
/*
* @brief Specific device feature definitions  (not present on all devices in the STM32L4 family)
*/
#define RCC_HSI48_SUPPORT
#define RCC_PLLP_DIV_2_31_SUPPORT
#define RCC_PLLSAI1P_DIV_2_31_SUPPORT

/********************  Bit definition for RCC_CR register  ********************/
#define  RCC_CR_MSION                        ((uint32_t)0x00000001U)      /*!< Internal Multi Speed oscillator (MSI) clock enable */
#define  RCC_CR_MSIRDY                       ((uint32_t)0x00000002U)      /*!< Internal Multi Speed oscillator (MSI) clock ready flag */
#define  RCC_CR_MSIPLLEN                     ((uint32_t)0x00000004U)      /*!< Internal Multi Speed oscillator (MSI) PLL enable */
#define  RCC_CR_MSIRGSEL                     ((uint32_t)0x00000008U)      /*!< Internal Multi Speed oscillator (MSI) range selection */

/*!< MSIRANGE configuration : 12 frequency ranges available */
#define  RCC_CR_MSIRANGE                     ((uint32_t)0x000000F0U)       /*!< Internal Multi Speed oscillator (MSI) clock Range */
#define  RCC_CR_MSIRANGE_0                   ((uint32_t)0x00000000U)       /*!< Internal Multi Speed oscillator (MSI) clock Range 100 KHz */
#define  RCC_CR_MSIRANGE_1                   ((uint32_t)0x00000010U)       /*!< Internal Multi Speed oscillator (MSI) clock Range 200 KHz */
#define  RCC_CR_MSIRANGE_2                   ((uint32_t)0x00000020U)       /*!< Internal Multi Speed oscillator (MSI) clock Range 400 KHz */
#define  RCC_CR_MSIRANGE_3                   ((uint32_t)0x00000030U)       /*!< Internal Multi Speed oscillator (MSI) clock Range 800 KHz */
#define  RCC_CR_MSIRANGE_4                   ((uint32_t)0x00000040U)       /*!< Internal Multi Speed oscillator (MSI) clock Range 1 MHz */
#define  RCC_CR_MSIRANGE_5                   ((uint32_t)0x00000050U)       /*!< Internal Multi Speed oscillator (MSI) clock Range 2 MHz */
#define  RCC_CR_MSIRANGE_6                   ((uint32_t)0x00000060U)       /*!< Internal Multi Speed oscillator (MSI) clock Range 4  MHz */
#define  RCC_CR_MSIRANGE_7                   ((uint32_t)0x00000070U)       /*!< Internal Multi Speed oscillator (MSI) clock Range 8 KHz */
#define  RCC_CR_MSIRANGE_8                   ((uint32_t)0x00000080U)       /*!< Internal Multi Speed oscillator (MSI) clock Range 16 MHz */
#define  RCC_CR_MSIRANGE_9                   ((uint32_t)0x00000090U)       /*!< Internal Multi Speed oscillator (MSI) clock Range 24 MHz */
#define  RCC_CR_MSIRANGE_10                  ((uint32_t)0x000000A0U)       /*!< Internal Multi Speed oscillator (MSI) clock Range 32 MHz */
#define  RCC_CR_MSIRANGE_11                  ((uint32_t)0x000000B0U)       /*!< Internal Multi Speed oscillator (MSI) clock Range 48  MHz */

#define  RCC_CR_HSION                        ((uint32_t)0x00000100U)       /*!< Internal High Speed oscillator (HSI16) clock enable */
#define  RCC_CR_HSIKERON                     ((uint32_t)0x00000200U)       /*!< Internal High Speed oscillator (HSI16) clock enable for some IPs Kernel */
#define  RCC_CR_HSIRDY                       ((uint32_t)0x00000400U)       /*!< Internal High Speed oscillator (HSI16) clock ready flag */
#define  RCC_CR_HSIASFS                      ((uint32_t)0x00000800U)       /*!< HSI16 Automatic Start from Stop */

#define  RCC_CR_HSEON                        ((uint32_t)0x00010000U)       /*!< External High Speed oscillator (HSE) clock enable */
#define  RCC_CR_HSERDY                       ((uint32_t)0x00020000U)       /*!< External High Speed oscillator (HSE) clock ready */
#define  RCC_CR_HSEBYP                       ((uint32_t)0x00040000U)       /*!< External High Speed oscillator (HSE) clock bypass */
#define  RCC_CR_CSSON                        ((uint32_t)0x00080000U)       /*!< HSE Clock Security System enable */

#define  RCC_CR_PLLON                        ((uint32_t)0x01000000U)       /*!< System PLL clock enable */
#define  RCC_CR_PLLRDY                       ((uint32_t)0x02000000U)       /*!< System PLL clock ready */
#define  RCC_CR_PLLSAI1ON                    ((uint32_t)0x04000000U)       /*!< SAI1 PLL enable */
#define  RCC_CR_PLLSAI1RDY                   ((uint32_t)0x08000000U)       /*!< SAI1 PLL ready */

/********************  Bit definition for RCC_ICSCR register  ***************/
/*!< MSICAL configuration */
#define  RCC_ICSCR_MSICAL                    ((uint32_t)0x000000FFU)       /*!< MSICAL[7:0] bits */
#define  RCC_ICSCR_MSICAL_0                  ((uint32_t)0x00000001U)       /*!<Bit 0 */
#define  RCC_ICSCR_MSICAL_1                  ((uint32_t)0x00000002U)       /*!<Bit 1 */
#define  RCC_ICSCR_MSICAL_2                  ((uint32_t)0x00000004U)       /*!<Bit 2 */
#define  RCC_ICSCR_MSICAL_3                  ((uint32_t)0x00000008U)       /*!<Bit 3 */
#define  RCC_ICSCR_MSICAL_4                  ((uint32_t)0x00000010U)       /*!<Bit 4 */
#define  RCC_ICSCR_MSICAL_5                  ((uint32_t)0x00000020U)       /*!<Bit 5 */
#define  RCC_ICSCR_MSICAL_6                  ((uint32_t)0x00000040U)       /*!<Bit 6 */
#define  RCC_ICSCR_MSICAL_7                  ((uint32_t)0x00000080U)       /*!<Bit 7 */

/*!< MSITRIM configuration */
#define  RCC_ICSCR_MSITRIM                   ((uint32_t)0x0000FF00U)       /*!< MSITRIM[7:0] bits */
#define  RCC_ICSCR_MSITRIM_0                 ((uint32_t)0x00000100U)       /*!<Bit 0 */
#define  RCC_ICSCR_MSITRIM_1                 ((uint32_t)0x00000200U)       /*!<Bit 1 */
#define  RCC_ICSCR_MSITRIM_2                 ((uint32_t)0x00000400U)       /*!<Bit 2 */
#define  RCC_ICSCR_MSITRIM_3                 ((uint32_t)0x00000800U)       /*!<Bit 3 */
#define  RCC_ICSCR_MSITRIM_4                 ((uint32_t)0x00001000U)       /*!<Bit 4 */
#define  RCC_ICSCR_MSITRIM_5                 ((uint32_t)0x00002000U)       /*!<Bit 5 */
#define  RCC_ICSCR_MSITRIM_6                 ((uint32_t)0x00004000U)       /*!<Bit 6 */
#define  RCC_ICSCR_MSITRIM_7                 ((uint32_t)0x00008000U)       /*!<Bit 7 */

/*!< HSICAL configuration */
#define  RCC_ICSCR_HSICAL                    ((uint32_t)0x00FF0000U)       /*!< HSICAL[7:0] bits */
#define  RCC_ICSCR_HSICAL_0                  ((uint32_t)0x00010000U)        /*!<Bit 0 */
#define  RCC_ICSCR_HSICAL_1                  ((uint32_t)0x00020000U)        /*!<Bit 1 */
#define  RCC_ICSCR_HSICAL_2                  ((uint32_t)0x00040000U)        /*!<Bit 2 */
#define  RCC_ICSCR_HSICAL_3                  ((uint32_t)0x00080000U)        /*!<Bit 3 */
#define  RCC_ICSCR_HSICAL_4                  ((uint32_t)0x00100000U)        /*!<Bit 4 */
#define  RCC_ICSCR_HSICAL_5                  ((uint32_t)0x00200000U)        /*!<Bit 5 */
#define  RCC_ICSCR_HSICAL_6                  ((uint32_t)0x00400000U)        /*!<Bit 6 */
#define  RCC_ICSCR_HSICAL_7                  ((uint32_t)0x00800000U)        /*!<Bit 7 */

/*!< HSITRIM configuration */
#define  RCC_ICSCR_HSITRIM                   ((uint32_t)0x1F000000U)       /*!< HSITRIM[4:0] bits */
#define  RCC_ICSCR_HSITRIM_0                 ((uint32_t)0x01000000U)        /*!<Bit 0 */
#define  RCC_ICSCR_HSITRIM_1                 ((uint32_t)0x02000000U)        /*!<Bit 1 */
#define  RCC_ICSCR_HSITRIM_2                 ((uint32_t)0x04000000U)        /*!<Bit 2 */
#define  RCC_ICSCR_HSITRIM_3                 ((uint32_t)0x08000000U)        /*!<Bit 3 */
#define  RCC_ICSCR_HSITRIM_4                 ((uint32_t)0x10000000U)        /*!<Bit 4 */

/********************  Bit definition for RCC_CFGR register  ******************/
/*!< SW configuration */
#define  RCC_CFGR_SW                         ((uint32_t)0x00000003U)      /*!< SW[1:0] bits (System clock Switch) */
#define  RCC_CFGR_SW_0                       ((uint32_t)0x00000001U)      /*!<Bit 0 */
#define  RCC_CFGR_SW_1                       ((uint32_t)0x00000002U)      /*!<Bit 1 */

#define  RCC_CFGR_SW_MSI                     ((uint32_t)0x00000000U)      /*!< MSI oscillator selection as system clock */
#define  RCC_CFGR_SW_HSI                     ((uint32_t)0x00000001U)      /*!< HSI16 oscillator selection as system clock */
#define  RCC_CFGR_SW_HSE                     ((uint32_t)0x00000002U)      /*!< HSE oscillator selection as system clock */
#define  RCC_CFGR_SW_PLL                     ((uint32_t)0x00000003U)      /*!< PLL selection as system clock */

/*!< SWS configuration */
#define  RCC_CFGR_SWS                        ((uint32_t)0x0000000CU)      /*!< SWS[1:0] bits (System Clock Switch Status) */
#define  RCC_CFGR_SWS_0                      ((uint32_t)0x00000004U)      /*!<Bit 0 */
#define  RCC_CFGR_SWS_1                      ((uint32_t)0x00000008U)      /*!<Bit 1 */

#define  RCC_CFGR_SWS_MSI                    ((uint32_t)0x00000000U)      /*!< MSI oscillator used as system clock */
#define  RCC_CFGR_SWS_HSI                    ((uint32_t)0x00000004U)      /*!< HSI16 oscillator used as system clock */
#define  RCC_CFGR_SWS_HSE                    ((uint32_t)0x00000008U)      /*!< HSE oscillator used as system clock */
#define  RCC_CFGR_SWS_PLL                    ((uint32_t)0x0000000CU)      /*!< PLL used as system clock */

/*!< HPRE configuration */
#define  RCC_CFGR_HPRE                       ((uint32_t)0x000000F0U)      /*!< HPRE[3:0] bits (AHB prescaler) */
#define  RCC_CFGR_HPRE_0                     ((uint32_t)0x00000010U)      /*!<Bit 0 */
#define  RCC_CFGR_HPRE_1                     ((uint32_t)0x00000020U)      /*!<Bit 1 */
#define  RCC_CFGR_HPRE_2                     ((uint32_t)0x00000040U)      /*!<Bit 2 */
#define  RCC_CFGR_HPRE_3                     ((uint32_t)0x00000080U)      /*!<Bit 3 */

#define  RCC_CFGR_HPRE_DIV1                  ((uint32_t)0x00000000U)      /*!< SYSCLK not divided */
#define  RCC_CFGR_HPRE_DIV2                  ((uint32_t)0x00000080U)      /*!< SYSCLK divided by 2 */
#define  RCC_CFGR_HPRE_DIV4                  ((uint32_t)0x00000090U)      /*!< SYSCLK divided by 4 */
#define  RCC_CFGR_HPRE_DIV8                  ((uint32_t)0x000000A0U)      /*!< SYSCLK divided by 8 */
#define  RCC_CFGR_HPRE_DIV16                 ((uint32_t)0x000000B0U)      /*!< SYSCLK divided by 16 */
#define  RCC_CFGR_HPRE_DIV64                 ((uint32_t)0x000000C0U)      /*!< SYSCLK divided by 64 */
#define  RCC_CFGR_HPRE_DIV128                ((uint32_t)0x000000D0U)      /*!< SYSCLK divided by 128 */
#define  RCC_CFGR_HPRE_DIV256                ((uint32_t)0x000000E0U)      /*!< SYSCLK divided by 256 */
#define  RCC_CFGR_HPRE_DIV512                ((uint32_t)0x000000F0U)      /*!< SYSCLK divided by 512 */

/*!< PPRE1 configuration */
#define  RCC_CFGR_PPRE1                      ((uint32_t)0x00000700U)      /*!< PRE1[2:0] bits (APB2 prescaler) */
#define  RCC_CFGR_PPRE1_0                    ((uint32_t)0x00000100U)      /*!<Bit 0 */
#define  RCC_CFGR_PPRE1_1                    ((uint32_t)0x00000200U)      /*!<Bit 1 */
#define  RCC_CFGR_PPRE1_2                    ((uint32_t)0x00000400U)      /*!<Bit 2 */

#define  RCC_CFGR_PPRE1_DIV1                 ((uint32_t)0x00000000U)      /*!< HCLK not divided */
#define  RCC_CFGR_PPRE1_DIV2                 ((uint32_t)0x00000400U)      /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE1_DIV4                 ((uint32_t)0x00000500U)      /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE1_DIV8                 ((uint32_t)0x00000600U)      /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE1_DIV16                ((uint32_t)0x00000700U)      /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define  RCC_CFGR_PPRE2                      ((uint32_t)0x00003800U)      /*!< PRE2[2:0] bits (APB2 prescaler) */
#define  RCC_CFGR_PPRE2_0                    ((uint32_t)0x00000800U)      /*!<Bit 0 */
#define  RCC_CFGR_PPRE2_1                    ((uint32_t)0x00001000U)      /*!<Bit 1 */
#define  RCC_CFGR_PPRE2_2                    ((uint32_t)0x00002000U)      /*!<Bit 2 */

#define  RCC_CFGR_PPRE2_DIV1                 ((uint32_t)0x00000000U)      /*!< HCLK not divided */
#define  RCC_CFGR_PPRE2_DIV2                 ((uint32_t)0x00002000U)      /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE2_DIV4                 ((uint32_t)0x00002800U)      /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE2_DIV8                 ((uint32_t)0x00003000U)      /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE2_DIV16                ((uint32_t)0x00003800U)      /*!< HCLK divided by 16 */

#define  RCC_CFGR_STOPWUCK                   ((uint32_t)0x00008000U)      /*!< Wake Up from stop and CSS backup clock selection */

/*!< MCOSEL configuration */
#define  RCC_CFGR_MCOSEL                     ((uint32_t)0x0F000000U)      /*!< MCOSEL [3:0] bits (Clock output selection) */
#define  RCC_CFGR_MCOSEL_0                   ((uint32_t)0x01000000U)      /*!<Bit 0 */
#define  RCC_CFGR_MCOSEL_1                   ((uint32_t)0x02000000U)      /*!<Bit 1 */
#define  RCC_CFGR_MCOSEL_2                   ((uint32_t)0x04000000U)      /*!<Bit 2 */
#define  RCC_CFGR_MCOSEL_3                   ((uint32_t)0x08000000U)      /*!<Bit 3 */

#define  RCC_CFGR_MCOPRE                     ((uint32_t)0x70000000U)      /*!< MCO prescaler */
#define  RCC_CFGR_MCOPRE_0                   ((uint32_t)0x10000000U)      /*!<Bit 0 */
#define  RCC_CFGR_MCOPRE_1                   ((uint32_t)0x20000000U)      /*!<Bit 1 */
#define  RCC_CFGR_MCOPRE_2                   ((uint32_t)0x40000000U)      /*!<Bit 2 */
 
#define  RCC_CFGR_MCOPRE_DIV1                ((uint32_t)0x00000000U)      /*!< MCO is divided by 1 */
#define  RCC_CFGR_MCOPRE_DIV2                ((uint32_t)0x10000000U)      /*!< MCO is divided by 2 */
#define  RCC_CFGR_MCOPRE_DIV4                ((uint32_t)0x20000000U)      /*!< MCO is divided by 4 */
#define  RCC_CFGR_MCOPRE_DIV8                ((uint32_t)0x30000000U)      /*!< MCO is divided by 8 */
#define  RCC_CFGR_MCOPRE_DIV16               ((uint32_t)0x40000000U)      /*!< MCO is divided by 16 */
 
/* Legacy aliases */
#define  RCC_CFGR_MCO_PRE                    RCC_CFGR_MCOPRE
#define  RCC_CFGR_MCO_PRE_1                  RCC_CFGR_MCOPRE_DIV1
#define  RCC_CFGR_MCO_PRE_2                  RCC_CFGR_MCOPRE_DIV2
#define  RCC_CFGR_MCO_PRE_4                  RCC_CFGR_MCOPRE_DIV4
#define  RCC_CFGR_MCO_PRE_8                  RCC_CFGR_MCOPRE_DIV8
#define  RCC_CFGR_MCO_PRE_16                 RCC_CFGR_MCOPRE_DIV16

/********************  Bit definition for RCC_PLLCFGR register  ***************/
#define  RCC_PLLCFGR_PLLSRC                  ((uint32_t)0x00000003U)

#define  RCC_PLLCFGR_PLLSRC_MSI              ((uint32_t)0x00000001U)      /*!< MSI oscillator source clock selected */
#define  RCC_PLLCFGR_PLLSRC_HSI              ((uint32_t)0x00000002U)      /*!< HSI16 oscillator source clock selected */
#define  RCC_PLLCFGR_PLLSRC_HSE              ((uint32_t)0x00000003U)      /*!< HSE oscillator source clock selected */

#define  RCC_PLLCFGR_PLLM                    ((uint32_t)0x00000070U)
#define  RCC_PLLCFGR_PLLM_0                  ((uint32_t)0x00000010U)
#define  RCC_PLLCFGR_PLLM_1                  ((uint32_t)0x00000020U)
#define  RCC_PLLCFGR_PLLM_2                  ((uint32_t)0x00000040U)

#define  RCC_PLLCFGR_PLLN                    ((uint32_t)0x00007F00U)
#define  RCC_PLLCFGR_PLLN_0                  ((uint32_t)0x00000100U)
#define  RCC_PLLCFGR_PLLN_1                  ((uint32_t)0x00000200U)
#define  RCC_PLLCFGR_PLLN_2                  ((uint32_t)0x00000400U)
#define  RCC_PLLCFGR_PLLN_3                  ((uint32_t)0x00000800U)
#define  RCC_PLLCFGR_PLLN_4                  ((uint32_t)0x00001000U)
#define  RCC_PLLCFGR_PLLN_5                  ((uint32_t)0x00002000U)
#define  RCC_PLLCFGR_PLLN_6                  ((uint32_t)0x00004000U)

#define  RCC_PLLCFGR_PLLPEN                  ((uint32_t)0x00010000U)
#define  RCC_PLLCFGR_PLLP                    ((uint32_t)0x00020000U)
#define  RCC_PLLCFGR_PLLQEN                  ((uint32_t)0x00100000U)

#define  RCC_PLLCFGR_PLLQ                    ((uint32_t)0x00600000U)
#define  RCC_PLLCFGR_PLLQ_0                  ((uint32_t)0x00200000U)
#define  RCC_PLLCFGR_PLLQ_1                  ((uint32_t)0x00400000U)

#define  RCC_PLLCFGR_PLLREN                  ((uint32_t)0x01000000U)
#define  RCC_PLLCFGR_PLLR                    ((uint32_t)0x06000000U)
#define  RCC_PLLCFGR_PLLR_0                  ((uint32_t)0x02000000U)
#define  RCC_PLLCFGR_PLLR_1                  ((uint32_t)0x04000000U)

#define  RCC_PLLCFGR_PLLPDIV                 ((uint32_t)0xF8000000U)
#define  RCC_PLLCFGR_PLLPDIV_0               ((uint32_t)0x08000000U)
#define  RCC_PLLCFGR_PLLPDIV_1               ((uint32_t)0x10000000U)
#define  RCC_PLLCFGR_PLLPDIV_2               ((uint32_t)0x20000000U)
#define  RCC_PLLCFGR_PLLPDIV_3               ((uint32_t)0x40000000U)
#define  RCC_PLLCFGR_PLLPDIV_4               ((uint32_t)0x80000000U)

/********************  Bit definition for RCC_PLLSAI1CFGR register  ************/
#define  RCC_PLLSAI1CFGR_PLLSAI1N            ((uint32_t)0x00007F00U)
#define  RCC_PLLSAI1CFGR_PLLSAI1N_0          ((uint32_t)0x00000100U)
#define  RCC_PLLSAI1CFGR_PLLSAI1N_1          ((uint32_t)0x00000200U)
#define  RCC_PLLSAI1CFGR_PLLSAI1N_2          ((uint32_t)0x00000400U)
#define  RCC_PLLSAI1CFGR_PLLSAI1N_3          ((uint32_t)0x00000800U)
#define  RCC_PLLSAI1CFGR_PLLSAI1N_4          ((uint32_t)0x00001000U)
#define  RCC_PLLSAI1CFGR_PLLSAI1N_5          ((uint32_t)0x00002000U)
#define  RCC_PLLSAI1CFGR_PLLSAI1N_6          ((uint32_t)0x00004000U)

#define  RCC_PLLSAI1CFGR_PLLSAI1PEN          ((uint32_t)0x00010000U)
#define  RCC_PLLSAI1CFGR_PLLSAI1P            ((uint32_t)0x00020000U)

#define  RCC_PLLSAI1CFGR_PLLSAI1QEN          ((uint32_t)0x00100000U)
#define  RCC_PLLSAI1CFGR_PLLSAI1Q            ((uint32_t)0x00600000U)
#define  RCC_PLLSAI1CFGR_PLLSAI1Q_0          ((uint32_t)0x00200000U)
#define  RCC_PLLSAI1CFGR_PLLSAI1Q_1          ((uint32_t)0x00400000U)

#define  RCC_PLLSAI1CFGR_PLLSAI1REN          ((uint32_t)0x01000000U)
#define  RCC_PLLSAI1CFGR_PLLSAI1R            ((uint32_t)0x06000000U)
#define  RCC_PLLSAI1CFGR_PLLSAI1R_0          ((uint32_t)0x02000000U)
#define  RCC_PLLSAI1CFGR_PLLSAI1R_1          ((uint32_t)0x04000000U)

#define  RCC_PLLSAI1CFGR_PLLSAI1PDIV         ((uint32_t)0xF8000000U)
#define  RCC_PLLSAI1CFGR_PLLSAI1PDIV_0       ((uint32_t)0x08000000U)
#define  RCC_PLLSAI1CFGR_PLLSAI1PDIV_1       ((uint32_t)0x10000000U)
#define  RCC_PLLSAI1CFGR_PLLSAI1PDIV_2       ((uint32_t)0x20000000U)
#define  RCC_PLLSAI1CFGR_PLLSAI1PDIV_3       ((uint32_t)0x40000000U)
#define  RCC_PLLSAI1CFGR_PLLSAI1PDIV_4       ((uint32_t)0x80000000U)

/********************  Bit definition for RCC_CIER register  ******************/
#define  RCC_CIER_LSIRDYIE                   ((uint32_t)0x00000001U)
#define  RCC_CIER_LSERDYIE                   ((uint32_t)0x00000002U)
#define  RCC_CIER_MSIRDYIE                   ((uint32_t)0x00000004U)
#define  RCC_CIER_HSIRDYIE                   ((uint32_t)0x00000008U)
#define  RCC_CIER_HSERDYIE                   ((uint32_t)0x00000010U)
#define  RCC_CIER_PLLRDYIE                   ((uint32_t)0x00000020U)
#define  RCC_CIER_PLLSAI1RDYIE               ((uint32_t)0x00000040U)
#define  RCC_CIER_LSECSSIE                   ((uint32_t)0x00000200U)
#define  RCC_CIER_HSI48RDYIE                 ((uint32_t)0x00000400U)

/********************  Bit definition for RCC_CIFR register  ******************/
#define  RCC_CIFR_LSIRDYF                    ((uint32_t)0x00000001U)
#define  RCC_CIFR_LSERDYF                    ((uint32_t)0x00000002U)
#define  RCC_CIFR_MSIRDYF                    ((uint32_t)0x00000004U)
#define  RCC_CIFR_HSIRDYF                    ((uint32_t)0x00000008U)
#define  RCC_CIFR_HSERDYF                    ((uint32_t)0x00000010U)
#define  RCC_CIFR_PLLRDYF                    ((uint32_t)0x00000020U)
#define  RCC_CIFR_PLLSAI1RDYF                ((uint32_t)0x00000040U)
#define  RCC_CIFR_CSSF                       ((uint32_t)0x00000100U)
#define  RCC_CIFR_LSECSSF                    ((uint32_t)0x00000200U)
#define  RCC_CIFR_HSI48RDYF                  ((uint32_t)0x00000400U)

/********************  Bit definition for RCC_CICR register  ******************/
#define  RCC_CICR_LSIRDYC                    ((uint32_t)0x00000001U)
#define  RCC_CICR_LSERDYC                    ((uint32_t)0x00000002U)
#define  RCC_CICR_MSIRDYC                    ((uint32_t)0x00000004U)
#define  RCC_CICR_HSIRDYC                    ((uint32_t)0x00000008U)
#define  RCC_CICR_HSERDYC                    ((uint32_t)0x00000010U)
#define  RCC_CICR_PLLRDYC                    ((uint32_t)0x00000020U)
#define  RCC_CICR_PLLSAI1RDYC                ((uint32_t)0x00000040U)
#define  RCC_CICR_CSSC                       ((uint32_t)0x00000100U)
#define  RCC_CICR_LSECSSC                    ((uint32_t)0x00000200U)
#define  RCC_CICR_HSI48RDYC                  ((uint32_t)0x00000400U)

/********************  Bit definition for RCC_AHB1RSTR register  **************/
#define  RCC_AHB1RSTR_DMA1RST                ((uint32_t)0x00000001U)
#define  RCC_AHB1RSTR_DMA2RST                ((uint32_t)0x00000002U)
#define  RCC_AHB1RSTR_FLASHRST               ((uint32_t)0x00000100U)
#define  RCC_AHB1RSTR_CRCRST                 ((uint32_t)0x00001000U)
#define  RCC_AHB1RSTR_TSCRST                 ((uint32_t)0x00010000U)

/********************  Bit definition for RCC_AHB2RSTR register  **************/
#define  RCC_AHB2RSTR_GPIOARST               ((uint32_t)0x00000001U)
#define  RCC_AHB2RSTR_GPIOBRST               ((uint32_t)0x00000002U)
#define  RCC_AHB2RSTR_GPIOCRST               ((uint32_t)0x00000004U)
#define  RCC_AHB2RSTR_GPIODRST               ((uint32_t)0x00000008U)
#define  RCC_AHB2RSTR_GPIOERST               ((uint32_t)0x00000010U)
#define  RCC_AHB2RSTR_GPIOHRST               ((uint32_t)0x00000080U)
#define  RCC_AHB2RSTR_ADCRST                 ((uint32_t)0x00002000U)
#define  RCC_AHB2RSTR_RNGRST                 ((uint32_t)0x00040000U)

/********************  Bit definition for RCC_AHB3RSTR register  **************/
#define  RCC_AHB3RSTR_QSPIRST                ((uint32_t)0x00000100U)

/********************  Bit definition for RCC_APB1RSTR1 register  **************/
#define  RCC_APB1RSTR1_TIM2RST               ((uint32_t)0x00000001U)
#define  RCC_APB1RSTR1_TIM6RST               ((uint32_t)0x00000010U)
#define  RCC_APB1RSTR1_TIM7RST               ((uint32_t)0x00000020U)
#define  RCC_APB1RSTR1_LCDRST                ((uint32_t)0x00000200U)
#define  RCC_APB1RSTR1_SPI2RST               ((uint32_t)0x00004000U)
#define  RCC_APB1RSTR1_SPI3RST               ((uint32_t)0x00008000U)
#define  RCC_APB1RSTR1_USART2RST             ((uint32_t)0x00020000U)
#define  RCC_APB1RSTR1_USART3RST             ((uint32_t)0x00040000U)
#define  RCC_APB1RSTR1_I2C1RST               ((uint32_t)0x00200000U)
#define  RCC_APB1RSTR1_I2C2RST               ((uint32_t)0x00400000U)
#define  RCC_APB1RSTR1_I2C3RST               ((uint32_t)0x00800000U)
#define  RCC_APB1RSTR1_CRSRST                ((uint32_t)0x01000000U)
#define  RCC_APB1RSTR1_CAN1RST               ((uint32_t)0x02000000U)
#define  RCC_APB1RSTR1_USBFSRST              ((uint32_t)0x04000000U)
#define  RCC_APB1RSTR1_PWRRST                ((uint32_t)0x10000000U)
#define  RCC_APB1RSTR1_DAC1RST               ((uint32_t)0x20000000U)
#define  RCC_APB1RSTR1_OPAMPRST              ((uint32_t)0x40000000U)
#define  RCC_APB1RSTR1_LPTIM1RST             ((uint32_t)0x80000000U)

/********************  Bit definition for RCC_APB1RSTR2 register  **************/
#define  RCC_APB1RSTR2_LPUART1RST            ((uint32_t)0x00000001U)
#define  RCC_APB1RSTR2_SWPMI1RST             ((uint32_t)0x00000004U)
#define  RCC_APB1RSTR2_LPTIM2RST             ((uint32_t)0x00000020U)

/********************  Bit definition for RCC_APB2RSTR register  **************/
#define  RCC_APB2RSTR_SYSCFGRST              ((uint32_t)0x00000001U)
#define  RCC_APB2RSTR_SDMMC1RST              ((uint32_t)0x00000400U)
#define  RCC_APB2RSTR_TIM1RST                ((uint32_t)0x00000800U)
#define  RCC_APB2RSTR_SPI1RST                ((uint32_t)0x00001000U)
#define  RCC_APB2RSTR_USART1RST              ((uint32_t)0x00004000U)
#define  RCC_APB2RSTR_TIM15RST               ((uint32_t)0x00010000U)
#define  RCC_APB2RSTR_TIM16RST               ((uint32_t)0x00020000U)
#define  RCC_APB2RSTR_SAI1RST                ((uint32_t)0x00200000U)

/********************  Bit definition for RCC_AHB1ENR register  ***************/
#define  RCC_AHB1ENR_DMA1EN                  ((uint32_t)0x00000001U)
#define  RCC_AHB1ENR_DMA2EN                  ((uint32_t)0x00000002U)
#define  RCC_AHB1ENR_FLASHEN                 ((uint32_t)0x00000100U)
#define  RCC_AHB1ENR_CRCEN                   ((uint32_t)0x00001000U)
#define  RCC_AHB1ENR_TSCEN                   ((uint32_t)0x00010000U)

/********************  Bit definition for RCC_AHB2ENR register  ***************/
#define  RCC_AHB2ENR_GPIOAEN                 ((uint32_t)0x00000001U)
#define  RCC_AHB2ENR_GPIOBEN                 ((uint32_t)0x00000002U)
#define  RCC_AHB2ENR_GPIOCEN                 ((uint32_t)0x00000004U)
#define  RCC_AHB2ENR_GPIODEN                 ((uint32_t)0x00000008U)
#define  RCC_AHB2ENR_GPIOEEN                 ((uint32_t)0x00000010U)
#define  RCC_AHB2ENR_GPIOHEN                 ((uint32_t)0x00000080U)
#define  RCC_AHB2ENR_ADCEN                   ((uint32_t)0x00002000U)
#define  RCC_AHB2ENR_RNGEN                   ((uint32_t)0x00040000U)

/********************  Bit definition for RCC_AHB3ENR register  ***************/
#define  RCC_AHB3ENR_QSPIEN                  ((uint32_t)0x00000100U)

/********************  Bit definition for RCC_APB1ENR1 register  ***************/
#define  RCC_APB1ENR1_TIM2EN                 ((uint32_t)0x00000001U)
#define  RCC_APB1ENR1_TIM6EN                 ((uint32_t)0x00000010U)
#define  RCC_APB1ENR1_TIM7EN                 ((uint32_t)0x00000020U)
#define  RCC_APB1ENR1_LCDEN                  ((uint32_t)0x00000200U)
#define  RCC_APB1ENR1_RTCAPBEN               ((uint32_t)0x00000400U)
#define  RCC_APB1ENR1_WWDGEN                 ((uint32_t)0x00000800U)
#define  RCC_APB1ENR1_SPI2EN                 ((uint32_t)0x00004000U)
#define  RCC_APB1ENR1_SPI3EN                 ((uint32_t)0x00008000U)
#define  RCC_APB1ENR1_USART2EN               ((uint32_t)0x00020000U)
#define  RCC_APB1ENR1_USART3EN               ((uint32_t)0x00040000U)
#define  RCC_APB1ENR1_I2C1EN                 ((uint32_t)0x00200000U)
#define  RCC_APB1ENR1_I2C2EN                 ((uint32_t)0x00400000U)
#define  RCC_APB1ENR1_I2C3EN                 ((uint32_t)0x00800000U)
#define  RCC_APB1ENR1_CRSEN                  ((uint32_t)0x01000000U)
#define  RCC_APB1ENR1_CAN1EN                 ((uint32_t)0x02000000U)
#define  RCC_APB1ENR1_USBFSEN                ((uint32_t)0x04000000U)
#define  RCC_APB1ENR1_PWREN                  ((uint32_t)0x10000000U)
#define  RCC_APB1ENR1_DAC1EN                 ((uint32_t)0x20000000U)
#define  RCC_APB1ENR1_OPAMPEN                ((uint32_t)0x40000000U)
#define  RCC_APB1ENR1_LPTIM1EN               ((uint32_t)0x80000000U)

/********************  Bit definition for RCC_APB1RSTR2 register  **************/
#define  RCC_APB1ENR2_LPUART1EN              ((uint32_t)0x00000001U)
#define  RCC_APB1ENR2_SWPMI1EN               ((uint32_t)0x00000004U)
#define  RCC_APB1ENR2_LPTIM2EN               ((uint32_t)0x00000020U)

/********************  Bit definition for RCC_APB2ENR register  ***************/
#define  RCC_APB2ENR_SYSCFGEN                ((uint32_t)0x00000001U)
#define  RCC_APB2ENR_FWEN                    ((uint32_t)0x00000080U)
#define  RCC_APB2ENR_SDMMC1EN                ((uint32_t)0x00000400U)
#define  RCC_APB2ENR_TIM1EN                  ((uint32_t)0x00000800U)
#define  RCC_APB2ENR_SPI1EN                  ((uint32_t)0x00001000U)
#define  RCC_APB2ENR_USART1EN                ((uint32_t)0x00004000U)
#define  RCC_APB2ENR_TIM15EN                 ((uint32_t)0x00010000U)
#define  RCC_APB2ENR_TIM16EN                 ((uint32_t)0x00020000U)
#define  RCC_APB2ENR_SAI1EN                  ((uint32_t)0x00200000U)

/********************  Bit definition for RCC_AHB1SMENR register  ***************/
#define  RCC_AHB1SMENR_DMA1SMEN              ((uint32_t)0x00000001U)
#define  RCC_AHB1SMENR_DMA2SMEN              ((uint32_t)0x00000002U)
#define  RCC_AHB1SMENR_FLASHSMEN             ((uint32_t)0x00000100U)
#define  RCC_AHB1SMENR_SRAM1SMEN             ((uint32_t)0x00000200U)
#define  RCC_AHB1SMENR_CRCSMEN               ((uint32_t)0x00001000U)
#define  RCC_AHB1SMENR_TSCSMEN               ((uint32_t)0x00010000U)

/********************  Bit definition for RCC_AHB2SMENR register  *************/
#define  RCC_AHB2SMENR_GPIOASMEN             ((uint32_t)0x00000001U)
#define  RCC_AHB2SMENR_GPIOBSMEN             ((uint32_t)0x00000002U)
#define  RCC_AHB2SMENR_GPIOCSMEN             ((uint32_t)0x00000004U)
#define  RCC_AHB2SMENR_GPIODSMEN             ((uint32_t)0x00000008U)
#define  RCC_AHB2SMENR_GPIOESMEN             ((uint32_t)0x00000010U)
#define  RCC_AHB2SMENR_GPIOHSMEN             ((uint32_t)0x00000080U)
#define  RCC_AHB2SMENR_SRAM2SMEN             ((uint32_t)0x00000200U)
#define  RCC_AHB2SMENR_ADCSMEN               ((uint32_t)0x00002000U)
#define  RCC_AHB2SMENR_RNGSMEN               ((uint32_t)0x00040000U)

/********************  Bit definition for RCC_AHB3SMENR register  *************/
#define  RCC_AHB3SMENR_QSPISMEN              ((uint32_t)0x00000100U)

/********************  Bit definition for RCC_APB1SMENR1 register  *************/
#define  RCC_APB1SMENR1_TIM2SMEN             ((uint32_t)0x00000001U)
#define  RCC_APB1SMENR1_TIM6SMEN             ((uint32_t)0x00000010U)
#define  RCC_APB1SMENR1_TIM7SMEN             ((uint32_t)0x00000020U)
#define  RCC_APB1SMENR1_LCDSMEN              ((uint32_t)0x00000200U)
#define  RCC_APB1SMENR1_RTCAPBSMEN           ((uint32_t)0x00000400U)
#define  RCC_APB1SMENR1_WWDGSMEN             ((uint32_t)0x00000800U)
#define  RCC_APB1SMENR1_SPI2SMEN             ((uint32_t)0x00004000U)
#define  RCC_APB1SMENR1_SPI3SMEN             ((uint32_t)0x00008000U)
#define  RCC_APB1SMENR1_USART2SMEN           ((uint32_t)0x00020000U)
#define  RCC_APB1SMENR1_USART3SMEN           ((uint32_t)0x00040000U)
#define  RCC_APB1SMENR1_I2C1SMEN             ((uint32_t)0x00200000U)
#define  RCC_APB1SMENR1_I2C2SMEN             ((uint32_t)0x00400000U)
#define  RCC_APB1SMENR1_I2C3SMEN             ((uint32_t)0x00800000U)
#define  RCC_APB1SMENR1_CRSSMEN              ((uint32_t)0x01000000U)
#define  RCC_APB1SMENR1_CAN1SMEN             ((uint32_t)0x02000000U)
#define  RCC_APB1SMENR1_USBFSSMEN            ((uint32_t)0x04000000U)
#define  RCC_APB1SMENR1_PWRSMEN              ((uint32_t)0x10000000U)
#define  RCC_APB1SMENR1_DAC1SMEN             ((uint32_t)0x20000000U)
#define  RCC_APB1SMENR1_OPAMPSMEN            ((uint32_t)0x40000000U)
#define  RCC_APB1SMENR1_LPTIM1SMEN           ((uint32_t)0x80000000U)

/********************  Bit definition for RCC_APB1SMENR2 register  *************/
#define  RCC_APB1SMENR2_LPUART1SMEN          ((uint32_t)0x00000001U)
#define  RCC_APB1SMENR2_SWPMI1SMEN           ((uint32_t)0x00000004U)
#define  RCC_APB1SMENR2_LPTIM2SMEN           ((uint32_t)0x00000020U)

/********************  Bit definition for RCC_APB2SMENR register  *************/
#define  RCC_APB2SMENR_SYSCFGSMEN            ((uint32_t)0x00000001U)
#define  RCC_APB2SMENR_SDMMC1SMEN            ((uint32_t)0x00000400U)
#define  RCC_APB2SMENR_TIM1SMEN              ((uint32_t)0x00000800U)
#define  RCC_APB2SMENR_SPI1SMEN              ((uint32_t)0x00001000U)
#define  RCC_APB2SMENR_USART1SMEN            ((uint32_t)0x00004000U)
#define  RCC_APB2SMENR_TIM15SMEN             ((uint32_t)0x00010000U)
#define  RCC_APB2SMENR_TIM16SMEN             ((uint32_t)0x00020000U)
#define  RCC_APB2SMENR_SAI1SMEN              ((uint32_t)0x00200000U)

/********************  Bit definition for RCC_CCIPR register  ******************/
#define  RCC_CCIPR_USART1SEL                 ((uint32_t)0x00000003U)
#define  RCC_CCIPR_USART1SEL_0               ((uint32_t)0x00000001U)
#define  RCC_CCIPR_USART1SEL_1               ((uint32_t)0x00000002U)

#define  RCC_CCIPR_USART2SEL                 ((uint32_t)0x0000000CU)
#define  RCC_CCIPR_USART2SEL_0               ((uint32_t)0x00000004U)
#define  RCC_CCIPR_USART2SEL_1               ((uint32_t)0x00000008U)

#define  RCC_CCIPR_USART3SEL                 ((uint32_t)0x00000030U)
#define  RCC_CCIPR_USART3SEL_0               ((uint32_t)0x00000010U)
#define  RCC_CCIPR_USART3SEL_1               ((uint32_t)0x00000020U)

#define  RCC_CCIPR_LPUART1SEL                ((uint32_t)0x00000C00U)
#define  RCC_CCIPR_LPUART1SEL_0              ((uint32_t)0x00000400U)
#define  RCC_CCIPR_LPUART1SEL_1              ((uint32_t)0x00000800U)

#define  RCC_CCIPR_I2C1SEL                   ((uint32_t)0x00003000U)
#define  RCC_CCIPR_I2C1SEL_0                 ((uint32_t)0x00001000U)
#define  RCC_CCIPR_I2C1SEL_1                 ((uint32_t)0x00002000U)

#define  RCC_CCIPR_I2C2SEL                   ((uint32_t)0x0000C000U)
#define  RCC_CCIPR_I2C2SEL_0                 ((uint32_t)0x00004000U)
#define  RCC_CCIPR_I2C2SEL_1                 ((uint32_t)0x00008000U)

#define  RCC_CCIPR_I2C3SEL                   ((uint32_t)0x00030000U)
#define  RCC_CCIPR_I2C3SEL_0                 ((uint32_t)0x00010000U)
#define  RCC_CCIPR_I2C3SEL_1                 ((uint32_t)0x00020000U)

#define  RCC_CCIPR_LPTIM1SEL                 ((uint32_t)0x000C0000U)
#define  RCC_CCIPR_LPTIM1SEL_0               ((uint32_t)0x00040000U)
#define  RCC_CCIPR_LPTIM1SEL_1               ((uint32_t)0x00080000U)

#define  RCC_CCIPR_LPTIM2SEL                 ((uint32_t)0x00300000U)
#define  RCC_CCIPR_LPTIM2SEL_0               ((uint32_t)0x00100000U)
#define  RCC_CCIPR_LPTIM2SEL_1               ((uint32_t)0x00200000U)

#define  RCC_CCIPR_SAI1SEL                   ((uint32_t)0x00C00000U)
#define  RCC_CCIPR_SAI1SEL_0                 ((uint32_t)0x00400000U)
#define  RCC_CCIPR_SAI1SEL_1                 ((uint32_t)0x00800000U)

#define  RCC_CCIPR_CLK48SEL                  ((uint32_t)0x0C000000U)
#define  RCC_CCIPR_CLK48SEL_0                ((uint32_t)0x04000000U)
#define  RCC_CCIPR_CLK48SEL_1                ((uint32_t)0x08000000U)

#define  RCC_CCIPR_ADCSEL                    ((uint32_t)0x30000000U)
#define  RCC_CCIPR_ADCSEL_0                  ((uint32_t)0x10000000U)
#define  RCC_CCIPR_ADCSEL_1                  ((uint32_t)0x20000000U)

#define  RCC_CCIPR_SWPMI1SEL                 ((uint32_t)0x40000000U)

/********************  Bit definition for RCC_BDCR register  ******************/
#define  RCC_BDCR_LSEON                      ((uint32_t)0x00000001U)
#define  RCC_BDCR_LSERDY                     ((uint32_t)0x00000002U)
#define  RCC_BDCR_LSEBYP                     ((uint32_t)0x00000004U)

#define  RCC_BDCR_LSEDRV                     ((uint32_t)0x00000018U)
#define  RCC_BDCR_LSEDRV_0                   ((uint32_t)0x00000008U)
#define  RCC_BDCR_LSEDRV_1                   ((uint32_t)0x00000010U)

#define  RCC_BDCR_LSECSSON                   ((uint32_t)0x00000020U)
#define  RCC_BDCR_LSECSSD                    ((uint32_t)0x00000040U)

#define  RCC_BDCR_RTCSEL                     ((uint32_t)0x00000300U)
#define  RCC_BDCR_RTCSEL_0                   ((uint32_t)0x00000100U)
#define  RCC_BDCR_RTCSEL_1                   ((uint32_t)0x00000200U)

#define  RCC_BDCR_RTCEN                      ((uint32_t)0x00008000U)
#define  RCC_BDCR_BDRST                      ((uint32_t)0x00010000U)
#define  RCC_BDCR_LSCOEN                     ((uint32_t)0x01000000U)
#define  RCC_BDCR_LSCOSEL                    ((uint32_t)0x02000000U)

/********************  Bit definition for RCC_CSR register  *******************/
#define  RCC_CSR_LSION                       ((uint32_t)0x00000001U)
#define  RCC_CSR_LSIRDY                      ((uint32_t)0x00000002U)

#define  RCC_CSR_MSISRANGE                   ((uint32_t)0x00000F00U)
#define  RCC_CSR_MSISRANGE_1                 ((uint32_t)0x00000400U)      /*!< MSI frequency 1MHZ */
#define  RCC_CSR_MSISRANGE_2                 ((uint32_t)0x00000500U)      /*!< MSI frequency 2MHZ */
#define  RCC_CSR_MSISRANGE_4                 ((uint32_t)0x00000600U)      /*!< The default frequency 4MHZ */
#define  RCC_CSR_MSISRANGE_8                 ((uint32_t)0x00000700U)      /*!< MSI frequency 8MHZ */

#define  RCC_CSR_RMVF                        ((uint32_t)0x00800000U)
#define  RCC_CSR_FWRSTF                      ((uint32_t)0x01000000U)
#define  RCC_CSR_OBLRSTF                     ((uint32_t)0x02000000U)
#define  RCC_CSR_PINRSTF                     ((uint32_t)0x04000000U)
#define  RCC_CSR_BORRSTF                     ((uint32_t)0x08000000U)
#define  RCC_CSR_SFTRSTF                     ((uint32_t)0x10000000U)
#define  RCC_CSR_IWDGRSTF                    ((uint32_t)0x20000000U)
#define  RCC_CSR_WWDGRSTF                    ((uint32_t)0x40000000U)
#define  RCC_CSR_LPWRRSTF                    ((uint32_t)0x80000000U)

/********************  Bit definition for RCC_CRRCR register  *****************/
#define  RCC_CRRCR_HSI48ON                   ((uint32_t)0x00000001U)
#define  RCC_CRRCR_HSI48RDY                  ((uint32_t)0x00000002U)

/*!< HSI48CAL configuration */
#define  RCC_CRRCR_HSI48CAL                  ((uint32_t)0x00FF8000U)       /*!< HSI48CAL[8:0] bits */
#define  RCC_CRRCR_HSI48CAL_0                ((uint32_t)0x00001000U)        /*!<Bit 0 */
#define  RCC_CRRCR_HSI48CAL_1                ((uint32_t)0x00010000U)        /*!<Bit 1 */
#define  RCC_CRRCR_HSI48CAL_2                ((uint32_t)0x00020000U)        /*!<Bit 2 */
#define  RCC_CRRCR_HSI48CAL_3                ((uint32_t)0x00040000U)        /*!<Bit 3 */
#define  RCC_CRRCR_HSI48CAL_4                ((uint32_t)0x00080000U)        /*!<Bit 4 */
#define  RCC_CRRCR_HSI48CAL_5                ((uint32_t)0x00100000U)        /*!<Bit 5 */
#define  RCC_CRRCR_HSI48CAL_6                ((uint32_t)0x00200000U)        /*!<Bit 6 */
#define  RCC_CRRCR_HSI48CAL_7                ((uint32_t)0x00400000U)        /*!<Bit 7 */
#define  RCC_CRRCR_HSI48CAL_8                ((uint32_t)0x00800000U)        /*!<Bit 8 */



/******************************************************************************/
/*                                                                            */
/*                                    RNG                                     */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for RNG_CR register  *******************/
#define RNG_CR_RNGEN                         ((uint32_t)0x00000004U)
#define RNG_CR_IE                            ((uint32_t)0x00000008U)

/********************  Bits definition for RNG_SR register  *******************/
#define RNG_SR_DRDY                          ((uint32_t)0x00000001U)
#define RNG_SR_CECS                          ((uint32_t)0x00000002U)
#define RNG_SR_SECS                          ((uint32_t)0x00000004U)
#define RNG_SR_CEIS                          ((uint32_t)0x00000020U)
#define RNG_SR_SEIS                          ((uint32_t)0x00000040U)

/******************************************************************************/
/*                                                                            */
/*                           Real-Time Clock (RTC)                            */
/*                                                                            */
/******************************************************************************/
/*
* @brief Specific device feature definitions
*/
#define RTC_TAMPER1_SUPPORT
#define RTC_TAMPER2_SUPPORT
#define RTC_TAMPER3_SUPPORT
#define RTC_WAKEUP_SUPPORT
#define RTC_BACKUP_SUPPORT

/********************  Bits definition for RTC_TR register  *******************/
#define RTC_TR_PM                            ((uint32_t)0x00400000U)
#define RTC_TR_HT                            ((uint32_t)0x00300000U)
#define RTC_TR_HT_0                          ((uint32_t)0x00100000U)
#define RTC_TR_HT_1                          ((uint32_t)0x00200000U)
#define RTC_TR_HU                            ((uint32_t)0x000F0000U)
#define RTC_TR_HU_0                          ((uint32_t)0x00010000U)
#define RTC_TR_HU_1                          ((uint32_t)0x00020000U)
#define RTC_TR_HU_2                          ((uint32_t)0x00040000U)
#define RTC_TR_HU_3                          ((uint32_t)0x00080000U)
#define RTC_TR_MNT                           ((uint32_t)0x00007000U)
#define RTC_TR_MNT_0                         ((uint32_t)0x00001000U)
#define RTC_TR_MNT_1                         ((uint32_t)0x00002000U)
#define RTC_TR_MNT_2                         ((uint32_t)0x00004000U)
#define RTC_TR_MNU                           ((uint32_t)0x00000F00U)
#define RTC_TR_MNU_0                         ((uint32_t)0x00000100U)
#define RTC_TR_MNU_1                         ((uint32_t)0x00000200U)
#define RTC_TR_MNU_2                         ((uint32_t)0x00000400U)
#define RTC_TR_MNU_3                         ((uint32_t)0x00000800U)
#define RTC_TR_ST                            ((uint32_t)0x00000070U)
#define RTC_TR_ST_0                          ((uint32_t)0x00000010U)
#define RTC_TR_ST_1                          ((uint32_t)0x00000020U)
#define RTC_TR_ST_2                          ((uint32_t)0x00000040U)
#define RTC_TR_SU                            ((uint32_t)0x0000000FU)
#define RTC_TR_SU_0                          ((uint32_t)0x00000001U)
#define RTC_TR_SU_1                          ((uint32_t)0x00000002U)
#define RTC_TR_SU_2                          ((uint32_t)0x00000004U)
#define RTC_TR_SU_3                          ((uint32_t)0x00000008U)

/********************  Bits definition for RTC_DR register  *******************/
#define RTC_DR_YT                            ((uint32_t)0x00F00000U)
#define RTC_DR_YT_0                          ((uint32_t)0x00100000U)
#define RTC_DR_YT_1                          ((uint32_t)0x00200000U)
#define RTC_DR_YT_2                          ((uint32_t)0x00400000U)
#define RTC_DR_YT_3                          ((uint32_t)0x00800000U)
#define RTC_DR_YU                            ((uint32_t)0x000F0000U)
#define RTC_DR_YU_0                          ((uint32_t)0x00010000U)
#define RTC_DR_YU_1                          ((uint32_t)0x00020000U)
#define RTC_DR_YU_2                          ((uint32_t)0x00040000U)
#define RTC_DR_YU_3                          ((uint32_t)0x00080000U)
#define RTC_DR_WDU                           ((uint32_t)0x0000E000U)
#define RTC_DR_WDU_0                         ((uint32_t)0x00002000U)
#define RTC_DR_WDU_1                         ((uint32_t)0x00004000U)
#define RTC_DR_WDU_2                         ((uint32_t)0x00008000U)
#define RTC_DR_MT                            ((uint32_t)0x00001000U)
#define RTC_DR_MU                            ((uint32_t)0x00000F00U)
#define RTC_DR_MU_0                          ((uint32_t)0x00000100U)
#define RTC_DR_MU_1                          ((uint32_t)0x00000200U)
#define RTC_DR_MU_2                          ((uint32_t)0x00000400U)
#define RTC_DR_MU_3                          ((uint32_t)0x00000800U)
#define RTC_DR_DT                            ((uint32_t)0x00000030U)
#define RTC_DR_DT_0                          ((uint32_t)0x00000010U)
#define RTC_DR_DT_1                          ((uint32_t)0x00000020U)
#define RTC_DR_DU                            ((uint32_t)0x0000000FU)
#define RTC_DR_DU_0                          ((uint32_t)0x00000001U)
#define RTC_DR_DU_1                          ((uint32_t)0x00000002U)
#define RTC_DR_DU_2                          ((uint32_t)0x00000004U)
#define RTC_DR_DU_3                          ((uint32_t)0x00000008U)

/********************  Bits definition for RTC_CR register  *******************/
#define RTC_CR_ITSE                          ((uint32_t)0x01000000U)
#define RTC_CR_COE                           ((uint32_t)0x00800000U)
#define RTC_CR_OSEL                          ((uint32_t)0x00600000U)
#define RTC_CR_OSEL_0                        ((uint32_t)0x00200000U)
#define RTC_CR_OSEL_1                        ((uint32_t)0x00400000U)
#define RTC_CR_POL                           ((uint32_t)0x00100000U)
#define RTC_CR_COSEL                         ((uint32_t)0x00080000U)
#define RTC_CR_BCK                           ((uint32_t)0x00040000U)
#define RTC_CR_SUB1H                         ((uint32_t)0x00020000U)
#define RTC_CR_ADD1H                         ((uint32_t)0x00010000U)
#define RTC_CR_TSIE                          ((uint32_t)0x00008000U)
#define RTC_CR_WUTIE                         ((uint32_t)0x00004000U)
#define RTC_CR_ALRBIE                        ((uint32_t)0x00002000U)
#define RTC_CR_ALRAIE                        ((uint32_t)0x00001000U)
#define RTC_CR_TSE                           ((uint32_t)0x00000800U)
#define RTC_CR_WUTE                          ((uint32_t)0x00000400U)
#define RTC_CR_ALRBE                         ((uint32_t)0x00000200U)
#define RTC_CR_ALRAE                         ((uint32_t)0x00000100U)
#define RTC_CR_FMT                           ((uint32_t)0x00000040U)
#define RTC_CR_BYPSHAD                       ((uint32_t)0x00000020U)
#define RTC_CR_REFCKON                       ((uint32_t)0x00000010U)
#define RTC_CR_TSEDGE                        ((uint32_t)0x00000008U)
#define RTC_CR_WUCKSEL                       ((uint32_t)0x00000007U)
#define RTC_CR_WUCKSEL_0                     ((uint32_t)0x00000001U)
#define RTC_CR_WUCKSEL_1                     ((uint32_t)0x00000002U)
#define RTC_CR_WUCKSEL_2                     ((uint32_t)0x00000004U)

/********************  Bits definition for RTC_ISR register  ******************/
#define RTC_ISR_ITSF                         ((uint32_t)0x00020000U)
#define RTC_ISR_RECALPF                      ((uint32_t)0x00010000U)
#define RTC_ISR_TAMP3F                       ((uint32_t)0x00008000U)
#define RTC_ISR_TAMP2F                       ((uint32_t)0x00004000U)
#define RTC_ISR_TAMP1F                       ((uint32_t)0x00002000U)
#define RTC_ISR_TSOVF                        ((uint32_t)0x00001000U)
#define RTC_ISR_TSF                          ((uint32_t)0x00000800U)
#define RTC_ISR_WUTF                         ((uint32_t)0x00000400U)
#define RTC_ISR_ALRBF                        ((uint32_t)0x00000200U)
#define RTC_ISR_ALRAF                        ((uint32_t)0x00000100U)
#define RTC_ISR_INIT                         ((uint32_t)0x00000080U)
#define RTC_ISR_INITF                        ((uint32_t)0x00000040U)
#define RTC_ISR_RSF                          ((uint32_t)0x00000020U)
#define RTC_ISR_INITS                        ((uint32_t)0x00000010U)
#define RTC_ISR_SHPF                         ((uint32_t)0x00000008U)
#define RTC_ISR_WUTWF                        ((uint32_t)0x00000004U)
#define RTC_ISR_ALRBWF                       ((uint32_t)0x00000002U)
#define RTC_ISR_ALRAWF                       ((uint32_t)0x00000001U)

/********************  Bits definition for RTC_PRER register  *****************/
#define RTC_PRER_PREDIV_A                    ((uint32_t)0x007F0000U)
#define RTC_PRER_PREDIV_S                    ((uint32_t)0x00007FFFU)

/********************  Bits definition for RTC_WUTR register  *****************/
#define RTC_WUTR_WUT                         ((uint32_t)0x0000FFFFU)

/********************  Bits definition for RTC_ALRMAR register  ***************/
#define RTC_ALRMAR_MSK4                      ((uint32_t)0x80000000U)
#define RTC_ALRMAR_WDSEL                     ((uint32_t)0x40000000U)
#define RTC_ALRMAR_DT                        ((uint32_t)0x30000000U)
#define RTC_ALRMAR_DT_0                      ((uint32_t)0x10000000U)
#define RTC_ALRMAR_DT_1                      ((uint32_t)0x20000000U)
#define RTC_ALRMAR_DU                        ((uint32_t)0x0F000000U)
#define RTC_ALRMAR_DU_0                      ((uint32_t)0x01000000U)
#define RTC_ALRMAR_DU_1                      ((uint32_t)0x02000000U)
#define RTC_ALRMAR_DU_2                      ((uint32_t)0x04000000U)
#define RTC_ALRMAR_DU_3                      ((uint32_t)0x08000000U)
#define RTC_ALRMAR_MSK3                      ((uint32_t)0x00800000U)
#define RTC_ALRMAR_PM                        ((uint32_t)0x00400000U)
#define RTC_ALRMAR_HT                        ((uint32_t)0x00300000U)
#define RTC_ALRMAR_HT_0                      ((uint32_t)0x00100000U)
#define RTC_ALRMAR_HT_1                      ((uint32_t)0x00200000U)
#define RTC_ALRMAR_HU                        ((uint32_t)0x000F0000U)
#define RTC_ALRMAR_HU_0                      ((uint32_t)0x00010000U)
#define RTC_ALRMAR_HU_1                      ((uint32_t)0x00020000U)
#define RTC_ALRMAR_HU_2                      ((uint32_t)0x00040000U)
#define RTC_ALRMAR_HU_3                      ((uint32_t)0x00080000U)
#define RTC_ALRMAR_MSK2                      ((uint32_t)0x00008000U)
#define RTC_ALRMAR_MNT                       ((uint32_t)0x00007000U)
#define RTC_ALRMAR_MNT_0                     ((uint32_t)0x00001000U)
#define RTC_ALRMAR_MNT_1                     ((uint32_t)0x00002000U)
#define RTC_ALRMAR_MNT_2                     ((uint32_t)0x00004000U)
#define RTC_ALRMAR_MNU                       ((uint32_t)0x00000F00U)
#define RTC_ALRMAR_MNU_0                     ((uint32_t)0x00000100U)
#define RTC_ALRMAR_MNU_1                     ((uint32_t)0x00000200U)
#define RTC_ALRMAR_MNU_2                     ((uint32_t)0x00000400U)
#define RTC_ALRMAR_MNU_3                     ((uint32_t)0x00000800U)
#define RTC_ALRMAR_MSK1                      ((uint32_t)0x00000080U)
#define RTC_ALRMAR_ST                        ((uint32_t)0x00000070U)
#define RTC_ALRMAR_ST_0                      ((uint32_t)0x00000010U)
#define RTC_ALRMAR_ST_1                      ((uint32_t)0x00000020U)
#define RTC_ALRMAR_ST_2                      ((uint32_t)0x00000040U)
#define RTC_ALRMAR_SU                        ((uint32_t)0x0000000FU)
#define RTC_ALRMAR_SU_0                      ((uint32_t)0x00000001U)
#define RTC_ALRMAR_SU_1                      ((uint32_t)0x00000002U)
#define RTC_ALRMAR_SU_2                      ((uint32_t)0x00000004U)
#define RTC_ALRMAR_SU_3                      ((uint32_t)0x00000008U)

/********************  Bits definition for RTC_ALRMBR register  ***************/
#define RTC_ALRMBR_MSK4                      ((uint32_t)0x80000000U)
#define RTC_ALRMBR_WDSEL                     ((uint32_t)0x40000000U)
#define RTC_ALRMBR_DT                        ((uint32_t)0x30000000U)
#define RTC_ALRMBR_DT_0                      ((uint32_t)0x10000000U)
#define RTC_ALRMBR_DT_1                      ((uint32_t)0x20000000U)
#define RTC_ALRMBR_DU                        ((uint32_t)0x0F000000U)
#define RTC_ALRMBR_DU_0                      ((uint32_t)0x01000000U)
#define RTC_ALRMBR_DU_1                      ((uint32_t)0x02000000U)
#define RTC_ALRMBR_DU_2                      ((uint32_t)0x04000000U)
#define RTC_ALRMBR_DU_3                      ((uint32_t)0x08000000U)
#define RTC_ALRMBR_MSK3                      ((uint32_t)0x00800000U)
#define RTC_ALRMBR_PM                        ((uint32_t)0x00400000U)
#define RTC_ALRMBR_HT                        ((uint32_t)0x00300000U)
#define RTC_ALRMBR_HT_0                      ((uint32_t)0x00100000U)
#define RTC_ALRMBR_HT_1                      ((uint32_t)0x00200000U)
#define RTC_ALRMBR_HU                        ((uint32_t)0x000F0000U)
#define RTC_ALRMBR_HU_0                      ((uint32_t)0x00010000U)
#define RTC_ALRMBR_HU_1                      ((uint32_t)0x00020000U)
#define RTC_ALRMBR_HU_2                      ((uint32_t)0x00040000U)
#define RTC_ALRMBR_HU_3                      ((uint32_t)0x00080000U)
#define RTC_ALRMBR_MSK2                      ((uint32_t)0x00008000U)
#define RTC_ALRMBR_MNT                       ((uint32_t)0x00007000U)
#define RTC_ALRMBR_MNT_0                     ((uint32_t)0x00001000U)
#define RTC_ALRMBR_MNT_1                     ((uint32_t)0x00002000U)
#define RTC_ALRMBR_MNT_2                     ((uint32_t)0x00004000U)
#define RTC_ALRMBR_MNU                       ((uint32_t)0x00000F00U)
#define RTC_ALRMBR_MNU_0                     ((uint32_t)0x00000100U)
#define RTC_ALRMBR_MNU_1                     ((uint32_t)0x00000200U)
#define RTC_ALRMBR_MNU_2                     ((uint32_t)0x00000400U)
#define RTC_ALRMBR_MNU_3                     ((uint32_t)0x00000800U)
#define RTC_ALRMBR_MSK1                      ((uint32_t)0x00000080U)
#define RTC_ALRMBR_ST                        ((uint32_t)0x00000070U)
#define RTC_ALRMBR_ST_0                      ((uint32_t)0x00000010U)
#define RTC_ALRMBR_ST_1                      ((uint32_t)0x00000020U)
#define RTC_ALRMBR_ST_2                      ((uint32_t)0x00000040U)
#define RTC_ALRMBR_SU                        ((uint32_t)0x0000000FU)
#define RTC_ALRMBR_SU_0                      ((uint32_t)0x00000001U)
#define RTC_ALRMBR_SU_1                      ((uint32_t)0x00000002U)
#define RTC_ALRMBR_SU_2                      ((uint32_t)0x00000004U)
#define RTC_ALRMBR_SU_3                      ((uint32_t)0x00000008U)

/********************  Bits definition for RTC_WPR register  ******************/
#define RTC_WPR_KEY                          ((uint32_t)0x000000FFU)

/********************  Bits definition for RTC_SSR register  ******************/
#define RTC_SSR_SS                           ((uint32_t)0x0000FFFFU)

/********************  Bits definition for RTC_SHIFTR register  ***************/
#define RTC_SHIFTR_SUBFS                     ((uint32_t)0x00007FFFU)
#define RTC_SHIFTR_ADD1S                     ((uint32_t)0x80000000U)

/********************  Bits definition for RTC_TSTR register  *****************/
#define RTC_TSTR_PM                          ((uint32_t)0x00400000U)
#define RTC_TSTR_HT                          ((uint32_t)0x00300000U)
#define RTC_TSTR_HT_0                        ((uint32_t)0x00100000U)
#define RTC_TSTR_HT_1                        ((uint32_t)0x00200000U)
#define RTC_TSTR_HU                          ((uint32_t)0x000F0000U)
#define RTC_TSTR_HU_0                        ((uint32_t)0x00010000U)
#define RTC_TSTR_HU_1                        ((uint32_t)0x00020000U)
#define RTC_TSTR_HU_2                        ((uint32_t)0x00040000U)
#define RTC_TSTR_HU_3                        ((uint32_t)0x00080000U)
#define RTC_TSTR_MNT                         ((uint32_t)0x00007000U)
#define RTC_TSTR_MNT_0                       ((uint32_t)0x00001000U)
#define RTC_TSTR_MNT_1                       ((uint32_t)0x00002000U)
#define RTC_TSTR_MNT_2                       ((uint32_t)0x00004000U)
#define RTC_TSTR_MNU                         ((uint32_t)0x00000F00U)
#define RTC_TSTR_MNU_0                       ((uint32_t)0x00000100U)
#define RTC_TSTR_MNU_1                       ((uint32_t)0x00000200U)
#define RTC_TSTR_MNU_2                       ((uint32_t)0x00000400U)
#define RTC_TSTR_MNU_3                       ((uint32_t)0x00000800U)
#define RTC_TSTR_ST                          ((uint32_t)0x00000070U)
#define RTC_TSTR_ST_0                        ((uint32_t)0x00000010U)
#define RTC_TSTR_ST_1                        ((uint32_t)0x00000020U)
#define RTC_TSTR_ST_2                        ((uint32_t)0x00000040U)
#define RTC_TSTR_SU                          ((uint32_t)0x0000000FU)
#define RTC_TSTR_SU_0                        ((uint32_t)0x00000001U)
#define RTC_TSTR_SU_1                        ((uint32_t)0x00000002U)
#define RTC_TSTR_SU_2                        ((uint32_t)0x00000004U)
#define RTC_TSTR_SU_3                        ((uint32_t)0x00000008U)

/********************  Bits definition for RTC_TSDR register  *****************/
#define RTC_TSDR_WDU                         ((uint32_t)0x0000E000U)
#define RTC_TSDR_WDU_0                       ((uint32_t)0x00002000U)
#define RTC_TSDR_WDU_1                       ((uint32_t)0x00004000U)
#define RTC_TSDR_WDU_2                       ((uint32_t)0x00008000U)
#define RTC_TSDR_MT                          ((uint32_t)0x00001000U)
#define RTC_TSDR_MU                          ((uint32_t)0x00000F00U)
#define RTC_TSDR_MU_0                        ((uint32_t)0x00000100U)
#define RTC_TSDR_MU_1                        ((uint32_t)0x00000200U)
#define RTC_TSDR_MU_2                        ((uint32_t)0x00000400U)
#define RTC_TSDR_MU_3                        ((uint32_t)0x00000800U)
#define RTC_TSDR_DT                          ((uint32_t)0x00000030U)
#define RTC_TSDR_DT_0                        ((uint32_t)0x00000010U)
#define RTC_TSDR_DT_1                        ((uint32_t)0x00000020U)
#define RTC_TSDR_DU                          ((uint32_t)0x0000000FU)
#define RTC_TSDR_DU_0                        ((uint32_t)0x00000001U)
#define RTC_TSDR_DU_1                        ((uint32_t)0x00000002U)
#define RTC_TSDR_DU_2                        ((uint32_t)0x00000004U)
#define RTC_TSDR_DU_3                        ((uint32_t)0x00000008U)

/********************  Bits definition for RTC_TSSSR register  ****************/
#define RTC_TSSSR_SS                         ((uint32_t)0x0000FFFFU)

/********************  Bits definition for RTC_CAL register  *****************/
#define RTC_CALR_CALP                        ((uint32_t)0x00008000U)
#define RTC_CALR_CALW8                       ((uint32_t)0x00004000U)
#define RTC_CALR_CALW16                      ((uint32_t)0x00002000U)
#define RTC_CALR_CALM                        ((uint32_t)0x000001FFU)
#define RTC_CALR_CALM_0                      ((uint32_t)0x00000001U)
#define RTC_CALR_CALM_1                      ((uint32_t)0x00000002U)
#define RTC_CALR_CALM_2                      ((uint32_t)0x00000004U)
#define RTC_CALR_CALM_3                      ((uint32_t)0x00000008U)
#define RTC_CALR_CALM_4                      ((uint32_t)0x00000010U)
#define RTC_CALR_CALM_5                      ((uint32_t)0x00000020U)
#define RTC_CALR_CALM_6                      ((uint32_t)0x00000040U)
#define RTC_CALR_CALM_7                      ((uint32_t)0x00000080U)
#define RTC_CALR_CALM_8                      ((uint32_t)0x00000100U)

/********************  Bits definition for RTC_TAMPCR register  ***************/
#define RTC_TAMPCR_TAMP3MF                   ((uint32_t)0x01000000U)
#define RTC_TAMPCR_TAMP3NOERASE              ((uint32_t)0x00800000U)
#define RTC_TAMPCR_TAMP3IE                   ((uint32_t)0x00400000U)
#define RTC_TAMPCR_TAMP2MF                   ((uint32_t)0x00200000U)
#define RTC_TAMPCR_TAMP2NOERASE              ((uint32_t)0x00100000U)
#define RTC_TAMPCR_TAMP2IE                   ((uint32_t)0x00080000U)
#define RTC_TAMPCR_TAMP1MF                   ((uint32_t)0x00040000U)
#define RTC_TAMPCR_TAMP1NOERASE              ((uint32_t)0x00020000U)
#define RTC_TAMPCR_TAMP1IE                   ((uint32_t)0x00010000U)
#define RTC_TAMPCR_TAMPPUDIS                 ((uint32_t)0x00008000U)
#define RTC_TAMPCR_TAMPPRCH                  ((uint32_t)0x00006000U)
#define RTC_TAMPCR_TAMPPRCH_0                ((uint32_t)0x00002000U)
#define RTC_TAMPCR_TAMPPRCH_1                ((uint32_t)0x00004000U)
#define RTC_TAMPCR_TAMPFLT                   ((uint32_t)0x00001800U)
#define RTC_TAMPCR_TAMPFLT_0                 ((uint32_t)0x00000800U)
#define RTC_TAMPCR_TAMPFLT_1                 ((uint32_t)0x00001000U)
#define RTC_TAMPCR_TAMPFREQ                  ((uint32_t)0x00000700U)
#define RTC_TAMPCR_TAMPFREQ_0                ((uint32_t)0x00000100U)
#define RTC_TAMPCR_TAMPFREQ_1                ((uint32_t)0x00000200U)
#define RTC_TAMPCR_TAMPFREQ_2                ((uint32_t)0x00000400U)
#define RTC_TAMPCR_TAMPTS                    ((uint32_t)0x00000080U)
#define RTC_TAMPCR_TAMP3TRG                  ((uint32_t)0x00000040U)
#define RTC_TAMPCR_TAMP3E                    ((uint32_t)0x00000020U)
#define RTC_TAMPCR_TAMP2TRG                  ((uint32_t)0x00000010U)
#define RTC_TAMPCR_TAMP2E                    ((uint32_t)0x00000008U)
#define RTC_TAMPCR_TAMPIE                    ((uint32_t)0x00000004U)
#define RTC_TAMPCR_TAMP1TRG                  ((uint32_t)0x00000002U)
#define RTC_TAMPCR_TAMP1E                    ((uint32_t)0x00000001U)

/********************  Bits definition for RTC_ALRMASSR register  *************/
#define RTC_ALRMASSR_MASKSS                  ((uint32_t)0x0F000000U)
#define RTC_ALRMASSR_MASKSS_0                ((uint32_t)0x01000000U)
#define RTC_ALRMASSR_MASKSS_1                ((uint32_t)0x02000000U)
#define RTC_ALRMASSR_MASKSS_2                ((uint32_t)0x04000000U)
#define RTC_ALRMASSR_MASKSS_3                ((uint32_t)0x08000000U)
#define RTC_ALRMASSR_SS                      ((uint32_t)0x00007FFFU)

/********************  Bits definition for RTC_ALRMBSSR register  *************/
#define RTC_ALRMBSSR_MASKSS                  ((uint32_t)0x0F000000U)
#define RTC_ALRMBSSR_MASKSS_0                ((uint32_t)0x01000000U)
#define RTC_ALRMBSSR_MASKSS_1                ((uint32_t)0x02000000U)
#define RTC_ALRMBSSR_MASKSS_2                ((uint32_t)0x04000000U)
#define RTC_ALRMBSSR_MASKSS_3                ((uint32_t)0x08000000U)
#define RTC_ALRMBSSR_SS                      ((uint32_t)0x00007FFFU)

/********************  Bits definition for RTC_0R register  *******************/
#define RTC_OR_OUT_RMP                      ((uint32_t)0x00000002U)
#define RTC_OR_ALARMOUTTYPE                 ((uint32_t)0x00000001U)


/********************  Bits definition for RTC_BKP0R register  ****************/
#define RTC_BKP0R                            ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP1R register  ****************/
#define RTC_BKP1R                            ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP2R register  ****************/
#define RTC_BKP2R                            ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP3R register  ****************/
#define RTC_BKP3R                            ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP4R register  ****************/
#define RTC_BKP4R                            ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP5R register  ****************/
#define RTC_BKP5R                            ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP6R register  ****************/
#define RTC_BKP6R                            ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP7R register  ****************/
#define RTC_BKP7R                            ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP8R register  ****************/
#define RTC_BKP8R                            ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP9R register  ****************/
#define RTC_BKP9R                            ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP10R register  ***************/
#define RTC_BKP10R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP11R register  ***************/
#define RTC_BKP11R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP12R register  ***************/
#define RTC_BKP12R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP13R register  ***************/
#define RTC_BKP13R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP14R register  ***************/
#define RTC_BKP14R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP15R register  ***************/
#define RTC_BKP15R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP16R register  ***************/
#define RTC_BKP16R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP17R register  ***************/
#define RTC_BKP17R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP18R register  ***************/
#define RTC_BKP18R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP19R register  ***************/
#define RTC_BKP19R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP20R register  ***************/
#define RTC_BKP20R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP21R register  ***************/
#define RTC_BKP21R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP22R register  ***************/
#define RTC_BKP22R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP23R register  ***************/
#define RTC_BKP23R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP24R register  ***************/
#define RTC_BKP24R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP25R register  ***************/
#define RTC_BKP25R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP26R register  ***************/
#define RTC_BKP26R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP27R register  ***************/
#define RTC_BKP27R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP28R register  ***************/
#define RTC_BKP28R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP29R register  ***************/
#define RTC_BKP29R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP30R register  ***************/
#define RTC_BKP30R                           ((uint32_t)0xFFFFFFFFU)

/********************  Bits definition for RTC_BKP31R register  ***************/
#define RTC_BKP31R                           ((uint32_t)0xFFFFFFFFU)

/******************** Number of backup registers ******************************/
#define RTC_BKP_NUMBER                       32U

/******************************************************************************/
/*                                                                            */
/*                          Serial Audio Interface                            */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for SAI_GCR register  *******************/
#define  SAI_GCR_SYNCIN                  ((uint32_t)0x00000003U)        /*!<SYNCIN[1:0] bits (Synchronization Inputs)   */
#define  SAI_GCR_SYNCIN_0                ((uint32_t)0x00000001U)        /*!<Bit 0 */
#define  SAI_GCR_SYNCIN_1                ((uint32_t)0x00000002U)        /*!<Bit 1 */

#define  SAI_GCR_SYNCOUT                 ((uint32_t)0x00000030U)        /*!<SYNCOUT[1:0] bits (Synchronization Outputs) */
#define  SAI_GCR_SYNCOUT_0               ((uint32_t)0x00000010U)        /*!<Bit 0 */
#define  SAI_GCR_SYNCOUT_1               ((uint32_t)0x00000020U)        /*!<Bit 1 */

/*******************  Bit definition for SAI_xCR1 register  *******************/
#define  SAI_xCR1_MODE                    ((uint32_t)0x00000003U)        /*!<MODE[1:0] bits (Audio Block Mode)           */
#define  SAI_xCR1_MODE_0                  ((uint32_t)0x00000001U)        /*!<Bit 0 */
#define  SAI_xCR1_MODE_1                  ((uint32_t)0x00000002U)        /*!<Bit 1 */

#define  SAI_xCR1_PRTCFG                  ((uint32_t)0x0000000CU)        /*!<PRTCFG[1:0] bits (Protocol Configuration)   */
#define  SAI_xCR1_PRTCFG_0                ((uint32_t)0x00000004U)        /*!<Bit 0 */
#define  SAI_xCR1_PRTCFG_1                ((uint32_t)0x00000008U)        /*!<Bit 1 */

#define  SAI_xCR1_DS                      ((uint32_t)0x000000E0U)        /*!<DS[1:0] bits (Data Size) */
#define  SAI_xCR1_DS_0                    ((uint32_t)0x00000020U)        /*!<Bit 0 */
#define  SAI_xCR1_DS_1                    ((uint32_t)0x00000040U)        /*!<Bit 1 */
#define  SAI_xCR1_DS_2                    ((uint32_t)0x00000080U)        /*!<Bit 2 */

#define  SAI_xCR1_LSBFIRST                ((uint32_t)0x00000100U)        /*!<LSB First Configuration  */
#define  SAI_xCR1_CKSTR                   ((uint32_t)0x00000200U)        /*!<ClocK STRobing edge      */

#define  SAI_xCR1_SYNCEN                  ((uint32_t)0x00000C00U)        /*!<SYNCEN[1:0](SYNChronization ENable) */
#define  SAI_xCR1_SYNCEN_0                ((uint32_t)0x00000400U)        /*!<Bit 0 */
#define  SAI_xCR1_SYNCEN_1                ((uint32_t)0x00000800U)        /*!<Bit 1 */

#define  SAI_xCR1_MONO                    ((uint32_t)0x00001000U)        /*!<Mono mode                  */
#define  SAI_xCR1_OUTDRIV                 ((uint32_t)0x00002000U)        /*!<Output Drive               */
#define  SAI_xCR1_SAIEN                   ((uint32_t)0x00010000U)        /*!<Audio Block enable         */
#define  SAI_xCR1_DMAEN                   ((uint32_t)0x00020000U)        /*!<DMA enable                 */
#define  SAI_xCR1_NODIV                   ((uint32_t)0x00080000U)        /*!<No Divider Configuration   */

#define  SAI_xCR1_MCKDIV                  ((uint32_t)0x00F00000U)        /*!<MCKDIV[3:0] (Master ClocK Divider)  */
#define  SAI_xCR1_MCKDIV_0                ((uint32_t)0x00100000U)        /*!<Bit 0  */
#define  SAI_xCR1_MCKDIV_1                ((uint32_t)0x00200000U)        /*!<Bit 1  */
#define  SAI_xCR1_MCKDIV_2                ((uint32_t)0x00400000U)        /*!<Bit 2  */
#define  SAI_xCR1_MCKDIV_3                ((uint32_t)0x00800000U)        /*!<Bit 3  */

/*******************  Bit definition for SAI_xCR2 register  *******************/
#define  SAI_xCR2_FTH                     ((uint32_t)0x00000007U)        /*!<FTH[2:0](Fifo THreshold)  */
#define  SAI_xCR2_FTH_0                   ((uint32_t)0x00000001U)        /*!<Bit 0 */
#define  SAI_xCR2_FTH_1                   ((uint32_t)0x00000002U)        /*!<Bit 1 */
#define  SAI_xCR2_FTH_2                   ((uint32_t)0x00000004U)        /*!<Bit 2 */

#define  SAI_xCR2_FFLUSH                  ((uint32_t)0x00000008U)        /*!<Fifo FLUSH                       */
#define  SAI_xCR2_TRIS                    ((uint32_t)0x00000010U)        /*!<TRIState Management on data line */
#define  SAI_xCR2_MUTE                    ((uint32_t)0x00000020U)        /*!<Mute mode                        */
#define  SAI_xCR2_MUTEVAL                 ((uint32_t)0x00000040U)        /*!<Muate value                      */


#define  SAI_xCR2_MUTECNT                 ((uint32_t)0x00001F80U)        /*!<MUTECNT[5:0] (MUTE counter) */
#define  SAI_xCR2_MUTECNT_0               ((uint32_t)0x00000080U)        /*!<Bit 0 */
#define  SAI_xCR2_MUTECNT_1               ((uint32_t)0x00000100U)        /*!<Bit 1 */
#define  SAI_xCR2_MUTECNT_2               ((uint32_t)0x00000200U)        /*!<Bit 2 */
#define  SAI_xCR2_MUTECNT_3               ((uint32_t)0x00000400U)        /*!<Bit 3 */
#define  SAI_xCR2_MUTECNT_4               ((uint32_t)0x00000800U)        /*!<Bit 4 */
#define  SAI_xCR2_MUTECNT_5               ((uint32_t)0x00001000U)        /*!<Bit 5 */

#define  SAI_xCR2_CPL                     ((uint32_t)0x00002000U)        /*!<CPL mode                    */
#define  SAI_xCR2_COMP                    ((uint32_t)0x0000C000U)        /*!<COMP[1:0] (Companding mode) */
#define  SAI_xCR2_COMP_0                  ((uint32_t)0x00004000U)        /*!<Bit 0 */
#define  SAI_xCR2_COMP_1                  ((uint32_t)0x00008000U)        /*!<Bit 1 */


/******************  Bit definition for SAI_xFRCR register  *******************/
#define  SAI_xFRCR_FRL                    ((uint32_t)0x000000FFU)        /*!<FRL[7:0](Frame length)  */
#define  SAI_xFRCR_FRL_0                  ((uint32_t)0x00000001U)        /*!<Bit 0 */
#define  SAI_xFRCR_FRL_1                  ((uint32_t)0x00000002U)        /*!<Bit 1 */
#define  SAI_xFRCR_FRL_2                  ((uint32_t)0x00000004U)        /*!<Bit 2 */
#define  SAI_xFRCR_FRL_3                  ((uint32_t)0x00000008U)        /*!<Bit 3 */
#define  SAI_xFRCR_FRL_4                  ((uint32_t)0x00000010U)        /*!<Bit 4 */
#define  SAI_xFRCR_FRL_5                  ((uint32_t)0x00000020U)        /*!<Bit 5 */
#define  SAI_xFRCR_FRL_6                  ((uint32_t)0x00000040U)        /*!<Bit 6 */
#define  SAI_xFRCR_FRL_7                  ((uint32_t)0x00000080U)        /*!<Bit 7 */

#define  SAI_xFRCR_FSALL                  ((uint32_t)0x00007F00U)        /*!<FRL[6:0] (Frame synchronization active level length)  */
#define  SAI_xFRCR_FSALL_0                ((uint32_t)0x00000100U)        /*!<Bit 0 */
#define  SAI_xFRCR_FSALL_1                ((uint32_t)0x00000200U)        /*!<Bit 1 */
#define  SAI_xFRCR_FSALL_2                ((uint32_t)0x00000400U)        /*!<Bit 2 */
#define  SAI_xFRCR_FSALL_3                ((uint32_t)0x00000800U)        /*!<Bit 3 */
#define  SAI_xFRCR_FSALL_4                ((uint32_t)0x00001000U)        /*!<Bit 4 */
#define  SAI_xFRCR_FSALL_5                ((uint32_t)0x00002000U)        /*!<Bit 5 */
#define  SAI_xFRCR_FSALL_6                ((uint32_t)0x00004000U)        /*!<Bit 6 */

#define  SAI_xFRCR_FSDEF                  ((uint32_t)0x00010000U)        /*!< Frame Synchronization Definition */
#define  SAI_xFRCR_FSPOL                  ((uint32_t)0x00020000U)        /*!<Frame Synchronization POLarity    */
#define  SAI_xFRCR_FSOFF                  ((uint32_t)0x00040000U)        /*!<Frame Synchronization OFFset      */

/******************  Bit definition for SAI_xSLOTR register  *******************/
#define  SAI_xSLOTR_FBOFF                 ((uint32_t)0x0000001FU)        /*!<FRL[4:0](First Bit Offset)  */
#define  SAI_xSLOTR_FBOFF_0               ((uint32_t)0x00000001U)        /*!<Bit 0 */
#define  SAI_xSLOTR_FBOFF_1               ((uint32_t)0x00000002U)        /*!<Bit 1 */
#define  SAI_xSLOTR_FBOFF_2               ((uint32_t)0x00000004U)        /*!<Bit 2 */
#define  SAI_xSLOTR_FBOFF_3               ((uint32_t)0x00000008U)        /*!<Bit 3 */
#define  SAI_xSLOTR_FBOFF_4               ((uint32_t)0x00000010U)        /*!<Bit 4 */

#define  SAI_xSLOTR_SLOTSZ                ((uint32_t)0x000000C0U)        /*!<SLOTSZ[1:0] (Slot size)  */
#define  SAI_xSLOTR_SLOTSZ_0              ((uint32_t)0x00000040U)        /*!<Bit 0 */
#define  SAI_xSLOTR_SLOTSZ_1              ((uint32_t)0x00000080U)        /*!<Bit 1 */

#define  SAI_xSLOTR_NBSLOT                ((uint32_t)0x00000F00U)        /*!<NBSLOT[3:0] (Number of Slot in audio Frame)  */
#define  SAI_xSLOTR_NBSLOT_0              ((uint32_t)0x00000100U)        /*!<Bit 0 */
#define  SAI_xSLOTR_NBSLOT_1              ((uint32_t)0x00000200U)        /*!<Bit 1 */
#define  SAI_xSLOTR_NBSLOT_2              ((uint32_t)0x00000400U)        /*!<Bit 2 */
#define  SAI_xSLOTR_NBSLOT_3              ((uint32_t)0x00000800U)        /*!<Bit 3 */

#define  SAI_xSLOTR_SLOTEN                ((uint32_t)0xFFFF0000U)        /*!<SLOTEN[15:0] (Slot Enable)  */

/*******************  Bit definition for SAI_xIMR register  *******************/
#define  SAI_xIMR_OVRUDRIE                ((uint32_t)0x00000001U)        /*!<Overrun underrun interrupt enable                              */
#define  SAI_xIMR_MUTEDETIE               ((uint32_t)0x00000002U)        /*!<Mute detection interrupt enable                                */
#define  SAI_xIMR_WCKCFGIE                ((uint32_t)0x00000004U)        /*!<Wrong Clock Configuration interrupt enable                     */
#define  SAI_xIMR_FREQIE                  ((uint32_t)0x00000008U)        /*!<FIFO request interrupt enable                                  */
#define  SAI_xIMR_CNRDYIE                 ((uint32_t)0x00000010U)        /*!<Codec not ready interrupt enable                               */
#define  SAI_xIMR_AFSDETIE                ((uint32_t)0x00000020U)        /*!<Anticipated frame synchronization detection interrupt enable   */
#define  SAI_xIMR_LFSDETIE                ((uint32_t)0x00000040U)        /*!<Late frame synchronization detection interrupt enable          */

/********************  Bit definition for SAI_xSR register  *******************/
#define  SAI_xSR_OVRUDR                   ((uint32_t)0x00000001U)         /*!<Overrun underrun                               */
#define  SAI_xSR_MUTEDET                  ((uint32_t)0x00000002U)         /*!<Mute detection                                 */
#define  SAI_xSR_WCKCFG                   ((uint32_t)0x00000004U)         /*!<Wrong Clock Configuration                      */
#define  SAI_xSR_FREQ                     ((uint32_t)0x00000008U)         /*!<FIFO request                                   */
#define  SAI_xSR_CNRDY                    ((uint32_t)0x00000010U)         /*!<Codec not ready                                */
#define  SAI_xSR_AFSDET                   ((uint32_t)0x00000020U)         /*!<Anticipated frame synchronization detection    */
#define  SAI_xSR_LFSDET                   ((uint32_t)0x00000040U)         /*!<Late frame synchronization detection           */

#define  SAI_xSR_FLVL                     ((uint32_t)0x00070000U)         /*!<FLVL[2:0] (FIFO Level Threshold)               */
#define  SAI_xSR_FLVL_0                   ((uint32_t)0x00010000U)         /*!<Bit 0 */
#define  SAI_xSR_FLVL_1                   ((uint32_t)0x00020000U)         /*!<Bit 1 */
#define  SAI_xSR_FLVL_2                   ((uint32_t)0x00040000U)         /*!<Bit 2 */

/******************  Bit definition for SAI_xCLRFR register  ******************/
#define  SAI_xCLRFR_COVRUDR               ((uint32_t)0x00000001U)        /*!<Clear Overrun underrun                               */
#define  SAI_xCLRFR_CMUTEDET              ((uint32_t)0x00000002U)        /*!<Clear Mute detection                                 */
#define  SAI_xCLRFR_CWCKCFG               ((uint32_t)0x00000004U)        /*!<Clear Wrong Clock Configuration                      */
#define  SAI_xCLRFR_CFREQ                 ((uint32_t)0x00000008U)        /*!<Clear FIFO request                                   */
#define  SAI_xCLRFR_CCNRDY                ((uint32_t)0x00000010U)        /*!<Clear Codec not ready                                */
#define  SAI_xCLRFR_CAFSDET               ((uint32_t)0x00000020U)        /*!<Clear Anticipated frame synchronization detection    */
#define  SAI_xCLRFR_CLFSDET               ((uint32_t)0x00000040U)        /*!<Clear Late frame synchronization detection           */

/******************  Bit definition for SAI_xDR register  ******************/
#define  SAI_xDR_DATA                     ((uint32_t)0xFFFFFFFFU)

/******************************************************************************/
/*                                                                            */
/*                          LCD Controller (LCD)                              */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for LCD_CR register  *********************/
#define LCD_CR_LCDEN               ((uint32_t)0x00000001U)     /*!< LCD Enable Bit */
#define LCD_CR_VSEL                ((uint32_t)0x00000002U)     /*!< Voltage source selector Bit */

#define LCD_CR_DUTY                ((uint32_t)0x0000001CU)     /*!< DUTY[2:0] bits (Duty selector) */
#define LCD_CR_DUTY_0              ((uint32_t)0x00000004U)     /*!< Duty selector Bit 0 */
#define LCD_CR_DUTY_1              ((uint32_t)0x00000008U)     /*!< Duty selector Bit 1 */
#define LCD_CR_DUTY_2              ((uint32_t)0x00000010U)     /*!< Duty selector Bit 2 */

#define LCD_CR_BIAS                ((uint32_t)0x00000060U)     /*!< BIAS[1:0] bits (Bias selector) */
#define LCD_CR_BIAS_0              ((uint32_t)0x00000020U)     /*!< Bias selector Bit 0 */
#define LCD_CR_BIAS_1              ((uint32_t)0x00000040U)     /*!< Bias selector Bit 1 */

#define LCD_CR_MUX_SEG             ((uint32_t)0x00000080U)     /*!< Mux Segment Enable Bit */
#define LCD_CR_BUFEN               ((uint32_t)0x00000100U)     /*!< Voltage output buffer enable */

/*******************  Bit definition for LCD_FCR register  ********************/
#define LCD_FCR_HD                 ((uint32_t)0x00000001U)     /*!< High Drive Enable Bit */
#define LCD_FCR_SOFIE              ((uint32_t)0x00000002U)     /*!< Start of Frame Interrupt Enable Bit */
#define LCD_FCR_UDDIE              ((uint32_t)0x00000008U)     /*!< Update Display Done Interrupt Enable Bit */

#define LCD_FCR_PON                ((uint32_t)0x00000070U)     /*!< PON[2:0] bits (Pulse ON Duration) */
#define LCD_FCR_PON_0              ((uint32_t)0x00000010U)     /*!< Bit 0 */
#define LCD_FCR_PON_1              ((uint32_t)0x00000020U)     /*!< Bit 1 */
#define LCD_FCR_PON_2              ((uint32_t)0x00000040U)     /*!< Bit 2 */

#define LCD_FCR_DEAD               ((uint32_t)0x00000380U)     /*!< DEAD[2:0] bits (DEAD Time) */
#define LCD_FCR_DEAD_0             ((uint32_t)0x00000080U)     /*!< Bit 0 */
#define LCD_FCR_DEAD_1             ((uint32_t)0x00000100U)     /*!< Bit 1 */
#define LCD_FCR_DEAD_2             ((uint32_t)0x00000200U)     /*!< Bit 2 */

#define LCD_FCR_CC                 ((uint32_t)0x00001C00U)     /*!< CC[2:0] bits (Contrast Control) */
#define LCD_FCR_CC_0               ((uint32_t)0x00000400U)     /*!< Bit 0 */
#define LCD_FCR_CC_1               ((uint32_t)0x00000800U)     /*!< Bit 1 */
#define LCD_FCR_CC_2               ((uint32_t)0x00001000U)     /*!< Bit 2 */

#define LCD_FCR_BLINKF             ((uint32_t)0x0000E000U)     /*!< BLINKF[2:0] bits (Blink Frequency) */
#define LCD_FCR_BLINKF_0           ((uint32_t)0x00002000U)     /*!< Bit 0 */
#define LCD_FCR_BLINKF_1           ((uint32_t)0x00004000U)     /*!< Bit 1 */
#define LCD_FCR_BLINKF_2           ((uint32_t)0x00008000U)     /*!< Bit 2 */

#define LCD_FCR_BLINK              ((uint32_t)0x00030000U)     /*!< BLINK[1:0] bits (Blink Enable) */
#define LCD_FCR_BLINK_0            ((uint32_t)0x00010000U)     /*!< Bit 0 */
#define LCD_FCR_BLINK_1            ((uint32_t)0x00020000U)     /*!< Bit 1 */

#define LCD_FCR_DIV                ((uint32_t)0x003C0000U)     /*!< DIV[3:0] bits (Divider) */
#define LCD_FCR_PS                 ((uint32_t)0x03C00000U)     /*!< PS[3:0] bits (Prescaler) */

/*******************  Bit definition for LCD_SR register  *********************/
#define LCD_SR_ENS                 ((uint32_t)0x00000001U)     /*!< LCD Enabled Bit */
#define LCD_SR_SOF                 ((uint32_t)0x00000002U)     /*!< Start Of Frame Flag Bit */
#define LCD_SR_UDR                 ((uint32_t)0x00000004U)     /*!< Update Display Request Bit */
#define LCD_SR_UDD                 ((uint32_t)0x00000008U)     /*!< Update Display Done Flag Bit */
#define LCD_SR_RDY                 ((uint32_t)0x00000010U)     /*!< Ready Flag Bit */
#define LCD_SR_FCRSR               ((uint32_t)0x00000020U)     /*!< LCD FCR Register Synchronization Flag Bit */

/*******************  Bit definition for LCD_CLR register  ********************/
#define LCD_CLR_SOFC               ((uint32_t)0x00000002U)     /*!< Start Of Frame Flag Clear Bit */
#define LCD_CLR_UDDC               ((uint32_t)0x00000008U)     /*!< Update Display Done Flag Clear Bit */

/*******************  Bit definition for LCD_RAM register  ********************/
#define LCD_RAM_SEGMENT_DATA       ((uint32_t)0xFFFFFFFFU)     /*!< Segment Data Bits */

/******************************************************************************/
/*                                                                            */
/*                           SDMMC Interface                                  */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for SDMMC_POWER register  ******************/
#define  SDMMC_POWER_PWRCTRL                  ((uint8_t)0x03U)               /*!<PWRCTRL[1:0] bits (Power supply control bits) */
#define  SDMMC_POWER_PWRCTRL_0                ((uint8_t)0x01U)               /*!<Bit 0 */
#define  SDMMC_POWER_PWRCTRL_1                ((uint8_t)0x02U)               /*!<Bit 1 */

/******************  Bit definition for SDMMC_CLKCR register  ******************/
#define  SDMMC_CLKCR_CLKDIV                   ((uint16_t)0x00FFU)            /*!<Clock divide factor             */
#define  SDMMC_CLKCR_CLKEN                    ((uint16_t)0x0100U)            /*!<Clock enable bit                */
#define  SDMMC_CLKCR_PWRSAV                   ((uint16_t)0x0200U)            /*!<Power saving configuration bit  */
#define  SDMMC_CLKCR_BYPASS                   ((uint16_t)0x0400U)            /*!<Clock divider bypass enable bit */

#define  SDMMC_CLKCR_WIDBUS                   ((uint16_t)0x1800U)            /*!<WIDBUS[1:0] bits (Wide bus mode enable bit) */
#define  SDMMC_CLKCR_WIDBUS_0                 ((uint16_t)0x0800U)            /*!<Bit 0 */
#define  SDMMC_CLKCR_WIDBUS_1                 ((uint16_t)0x1000U)            /*!<Bit 1 */

#define  SDMMC_CLKCR_NEGEDGE                  ((uint16_t)0x2000U)            /*!<SDMMC_CK dephasing selection bit */
#define  SDMMC_CLKCR_HWFC_EN                  ((uint16_t)0x4000U)            /*!<HW Flow Control enable          */

/*******************  Bit definition for SDMMC_ARG register  *******************/
#define  SDMMC_ARG_CMDARG                     ((uint32_t)0xFFFFFFFFU)            /*!<Command argument */

/*******************  Bit definition for SDMMC_CMD register  *******************/
#define  SDMMC_CMD_CMDINDEX                   ((uint16_t)0x003FU)            /*!<Command Index                               */

#define  SDMMC_CMD_WAITRESP                   ((uint16_t)0x00C0U)            /*!<WAITRESP[1:0] bits (Wait for response bits) */
#define  SDMMC_CMD_WAITRESP_0                 ((uint16_t)0x0040U)            /*!< Bit 0 */
#define  SDMMC_CMD_WAITRESP_1                 ((uint16_t)0x0080U)            /*!< Bit 1 */

#define  SDMMC_CMD_WAITINT                    ((uint16_t)0x0100U)            /*!<CPSM Waits for Interrupt Request                               */
#define  SDMMC_CMD_WAITPEND                   ((uint16_t)0x0200U)            /*!<CPSM Waits for ends of data transfer (CmdPend internal signal) */
#define  SDMMC_CMD_CPSMEN                     ((uint16_t)0x0400U)            /*!<Command path state machine (CPSM) Enable bit                   */
#define  SDMMC_CMD_SDIOSUSPEND                ((uint16_t)0x0800U)            /*!<SD I/O suspend command                                         */

/*****************  Bit definition for SDMMC_RESPCMD register  *****************/
#define  SDMMC_RESPCMD_RESPCMD                ((uint8_t)0x3FU)               /*!<Response command index */

/******************  Bit definition for SDMMC_RESP0 register  ******************/
#define  SDMMC_RESP0_CARDSTATUS0              ((uint32_t)0xFFFFFFFFU)        /*!<Card Status */

/******************  Bit definition for SDMMC_RESP1 register  ******************/
#define  SDMMC_RESP1_CARDSTATUS1              ((uint32_t)0xFFFFFFFFU)        /*!<Card Status */

/******************  Bit definition for SDMMC_RESP2 register  ******************/
#define  SDMMC_RESP2_CARDSTATUS2              ((uint32_t)0xFFFFFFFFU)        /*!<Card Status */

/******************  Bit definition for SDMMC_RESP3 register  ******************/
#define  SDMMC_RESP3_CARDSTATUS3              ((uint32_t)0xFFFFFFFFU)        /*!<Card Status */

/******************  Bit definition for SDMMC_RESP4 register  ******************/
#define  SDMMC_RESP4_CARDSTATUS4              ((uint32_t)0xFFFFFFFFU)        /*!<Card Status */

/******************  Bit definition for SDMMC_DTIMER register  *****************/
#define  SDMMC_DTIMER_DATATIME                ((uint32_t)0xFFFFFFFFU)        /*!<Data timeout period. */

/******************  Bit definition for SDMMC_DLEN register  *******************/
#define  SDMMC_DLEN_DATALENGTH                ((uint32_t)0x01FFFFFFU)        /*!<Data length value    */

/******************  Bit definition for SDMMC_DCTRL register  ******************/
#define  SDMMC_DCTRL_DTEN                     ((uint16_t)0x0001U)            /*!<Data transfer enabled bit         */
#define  SDMMC_DCTRL_DTDIR                    ((uint16_t)0x0002U)            /*!<Data transfer direction selection */
#define  SDMMC_DCTRL_DTMODE                   ((uint16_t)0x0004U)            /*!<Data transfer mode selection      */
#define  SDMMC_DCTRL_DMAEN                    ((uint16_t)0x0008U)            /*!<DMA enabled bit                   */

#define  SDMMC_DCTRL_DBLOCKSIZE               ((uint16_t)0x00F0U)            /*!<DBLOCKSIZE[3:0] bits (Data block size) */
#define  SDMMC_DCTRL_DBLOCKSIZE_0             ((uint16_t)0x0010U)            /*!<Bit 0 */
#define  SDMMC_DCTRL_DBLOCKSIZE_1             ((uint16_t)0x0020U)            /*!<Bit 1 */
#define  SDMMC_DCTRL_DBLOCKSIZE_2             ((uint16_t)0x0040U)            /*!<Bit 2 */
#define  SDMMC_DCTRL_DBLOCKSIZE_3             ((uint16_t)0x0080U)            /*!<Bit 3 */

#define  SDMMC_DCTRL_RWSTART                  ((uint16_t)0x0100U)            /*!<Read wait start         */
#define  SDMMC_DCTRL_RWSTOP                   ((uint16_t)0x0200U)            /*!<Read wait stop          */
#define  SDMMC_DCTRL_RWMOD                    ((uint16_t)0x0400U)            /*!<Read wait mode          */
#define  SDMMC_DCTRL_SDIOEN                   ((uint16_t)0x0800U)            /*!<SD I/O enable functions */

/******************  Bit definition for SDMMC_DCOUNT register  *****************/
#define  SDMMC_DCOUNT_DATACOUNT               ((uint32_t)0x01FFFFFFU)        /*!<Data count value */

/******************  Bit definition for SDMMC_STA register  ********************/
#define  SDMMC_STA_CCRCFAIL                   ((uint32_t)0x00000001U)        /*!<Command response received (CRC check failed)  */
#define  SDMMC_STA_DCRCFAIL                   ((uint32_t)0x00000002U)        /*!<Data block sent/received (CRC check failed)   */
#define  SDMMC_STA_CTIMEOUT                   ((uint32_t)0x00000004U)        /*!<Command response timeout                      */
#define  SDMMC_STA_DTIMEOUT                   ((uint32_t)0x00000008U)        /*!<Data timeout                                  */
#define  SDMMC_STA_TXUNDERR                   ((uint32_t)0x00000010U)        /*!<Transmit FIFO underrun error                  */
#define  SDMMC_STA_RXOVERR                    ((uint32_t)0x00000020U)        /*!<Received FIFO overrun error                   */
#define  SDMMC_STA_CMDREND                    ((uint32_t)0x00000040U)        /*!<Command response received (CRC check passed)  */
#define  SDMMC_STA_CMDSENT                    ((uint32_t)0x00000080U)        /*!<Command sent (no response required)           */
#define  SDMMC_STA_DATAEND                    ((uint32_t)0x00000100U)        /*!<Data end (data counter, SDIDCOUNT, is zero)   */
#define  SDMMC_STA_STBITERR                   ((uint32_t)0x00000200U)        /*!<Start bit not detected on all data signals in wide bus mode */
#define  SDMMC_STA_DBCKEND                    ((uint32_t)0x00000400U)        /*!<Data block sent/received (CRC check passed)   */
#define  SDMMC_STA_CMDACT                     ((uint32_t)0x00000800U)        /*!<Command transfer in progress                  */
#define  SDMMC_STA_TXACT                      ((uint32_t)0x00001000U)        /*!<Data transmit in progress                     */
#define  SDMMC_STA_RXACT                      ((uint32_t)0x00002000U)        /*!<Data receive in progress                      */
#define  SDMMC_STA_TXFIFOHE                   ((uint32_t)0x00004000U)        /*!<Transmit FIFO Half Empty: at least 8 words can be written into the FIFO */
#define  SDMMC_STA_RXFIFOHF                   ((uint32_t)0x00008000U)        /*!<Receive FIFO Half Full: there are at least 8 words in the FIFO */
#define  SDMMC_STA_TXFIFOF                    ((uint32_t)0x00010000U)        /*!<Transmit FIFO full                            */
#define  SDMMC_STA_RXFIFOF                    ((uint32_t)0x00020000U)        /*!<Receive FIFO full                             */
#define  SDMMC_STA_TXFIFOE                    ((uint32_t)0x00040000U)        /*!<Transmit FIFO empty                           */
#define  SDMMC_STA_RXFIFOE                    ((uint32_t)0x00080000U)        /*!<Receive FIFO empty                            */
#define  SDMMC_STA_TXDAVL                     ((uint32_t)0x00100000U)        /*!<Data available in transmit FIFO               */
#define  SDMMC_STA_RXDAVL                     ((uint32_t)0x00200000U)        /*!<Data available in receive FIFO                */
#define  SDMMC_STA_SDIOIT                     ((uint32_t)0x00400000U)        /*!<SDIO interrupt received                       */

/*******************  Bit definition for SDMMC_ICR register  *******************/
#define  SDMMC_ICR_CCRCFAILC                  ((uint32_t)0x00000001U)        /*!<CCRCFAIL flag clear bit */
#define  SDMMC_ICR_DCRCFAILC                  ((uint32_t)0x00000002U)        /*!<DCRCFAIL flag clear bit */
#define  SDMMC_ICR_CTIMEOUTC                  ((uint32_t)0x00000004U)        /*!<CTIMEOUT flag clear bit */
#define  SDMMC_ICR_DTIMEOUTC                  ((uint32_t)0x00000008U)        /*!<DTIMEOUT flag clear bit */
#define  SDMMC_ICR_TXUNDERRC                  ((uint32_t)0x00000010U)        /*!<TXUNDERR flag clear bit */
#define  SDMMC_ICR_RXOVERRC                   ((uint32_t)0x00000020U)        /*!<RXOVERR flag clear bit  */
#define  SDMMC_ICR_CMDRENDC                   ((uint32_t)0x00000040U)        /*!<CMDREND flag clear bit  */
#define  SDMMC_ICR_CMDSENTC                   ((uint32_t)0x00000080U)        /*!<CMDSENT flag clear bit  */
#define  SDMMC_ICR_DATAENDC                   ((uint32_t)0x00000100U)        /*!<DATAEND flag clear bit  */
#define  SDMMC_ICR_STBITERRC                  ((uint32_t)0x00000200U)        /*!<STBITERR flag clear bit */
#define  SDMMC_ICR_DBCKENDC                   ((uint32_t)0x00000400U)        /*!<DBCKEND flag clear bit  */
#define  SDMMC_ICR_SDIOITC                    ((uint32_t)0x00400000U)        /*!<SDIOIT flag clear bit   */

/******************  Bit definition for SDMMC_MASK register  *******************/
#define  SDMMC_MASK_CCRCFAILIE                ((uint32_t)0x00000001U)        /*!<Command CRC Fail Interrupt Enable          */
#define  SDMMC_MASK_DCRCFAILIE                ((uint32_t)0x00000002U)        /*!<Data CRC Fail Interrupt Enable             */
#define  SDMMC_MASK_CTIMEOUTIE                ((uint32_t)0x00000004U)        /*!<Command TimeOut Interrupt Enable           */
#define  SDMMC_MASK_DTIMEOUTIE                ((uint32_t)0x00000008U)        /*!<Data TimeOut Interrupt Enable              */
#define  SDMMC_MASK_TXUNDERRIE                ((uint32_t)0x00000010U)        /*!<Tx FIFO UnderRun Error Interrupt Enable    */
#define  SDMMC_MASK_RXOVERRIE                 ((uint32_t)0x00000020U)        /*!<Rx FIFO OverRun Error Interrupt Enable     */
#define  SDMMC_MASK_CMDRENDIE                 ((uint32_t)0x00000040U)        /*!<Command Response Received Interrupt Enable */
#define  SDMMC_MASK_CMDSENTIE                 ((uint32_t)0x00000080U)        /*!<Command Sent Interrupt Enable              */
#define  SDMMC_MASK_DATAENDIE                 ((uint32_t)0x00000100U)        /*!<Data End Interrupt Enable                  */
#define  SDMMC_MASK_DBCKENDIE                 ((uint32_t)0x00000400U)        /*!<Data Block End Interrupt Enable            */
#define  SDMMC_MASK_CMDACTIE                  ((uint32_t)0x00000800U)        /*!<CCommand Acting Interrupt Enable           */
#define  SDMMC_MASK_TXACTIE                   ((uint32_t)0x00001000U)        /*!<Data Transmit Acting Interrupt Enable      */
#define  SDMMC_MASK_RXACTIE                   ((uint32_t)0x00002000U)        /*!<Data receive acting interrupt enabled      */
#define  SDMMC_MASK_TXFIFOHEIE                ((uint32_t)0x00004000U)        /*!<Tx FIFO Half Empty interrupt Enable        */
#define  SDMMC_MASK_RXFIFOHFIE                ((uint32_t)0x00008000U)        /*!<Rx FIFO Half Full interrupt Enable         */
#define  SDMMC_MASK_TXFIFOFIE                 ((uint32_t)0x00010000U)        /*!<Tx FIFO Full interrupt Enable              */
#define  SDMMC_MASK_RXFIFOFIE                 ((uint32_t)0x00020000U)        /*!<Rx FIFO Full interrupt Enable              */
#define  SDMMC_MASK_TXFIFOEIE                 ((uint32_t)0x00040000U)        /*!<Tx FIFO Empty interrupt Enable             */
#define  SDMMC_MASK_RXFIFOEIE                 ((uint32_t)0x00080000U)        /*!<Rx FIFO Empty interrupt Enable             */
#define  SDMMC_MASK_TXDAVLIE                  ((uint32_t)0x00100000U)        /*!<Data available in Tx FIFO interrupt Enable */
#define  SDMMC_MASK_RXDAVLIE                  ((uint32_t)0x00200000U)        /*!<Data available in Rx FIFO interrupt Enable */
#define  SDMMC_MASK_SDIOITIE                  ((uint32_t)0x00400000U)        /*!<SDIO Mode Interrupt Received interrupt Enable */

/*****************  Bit definition for SDMMC_FIFOCNT register  *****************/
#define  SDMMC_FIFOCNT_FIFOCOUNT              ((uint32_t)0x00FFFFFFU)        /*!<Remaining number of words to be written to or read from the FIFO */

/******************  Bit definition for SDMMC_FIFO register  *******************/
#define  SDMMC_FIFO_FIFODATA                  ((uint32_t)0xFFFFFFFFU)        /*!<Receive and transmit FIFO data */

/******************************************************************************/
/*                                                                            */
/*                        Serial Peripheral Interface (SPI)                   */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for SPI_CR1 register  ********************/
#define  SPI_CR1_CPHA                        ((uint32_t)0x00000001U)            /*!<Clock Phase      */
#define  SPI_CR1_CPOL                        ((uint32_t)0x00000002U)            /*!<Clock Polarity   */
#define  SPI_CR1_MSTR                        ((uint32_t)0x00000004U)            /*!<Master Selection */

#define  SPI_CR1_BR                          ((uint32_t)0x00000038U)            /*!<BR[2:0] bits (Baud Rate Control) */
#define  SPI_CR1_BR_0                        ((uint32_t)0x00000008U)            /*!<Bit 0 */
#define  SPI_CR1_BR_1                        ((uint32_t)0x00000010U)            /*!<Bit 1 */
#define  SPI_CR1_BR_2                        ((uint32_t)0x00000020U)            /*!<Bit 2 */

#define  SPI_CR1_SPE                         ((uint32_t)0x00000040U)            /*!<SPI Enable                          */
#define  SPI_CR1_LSBFIRST                    ((uint32_t)0x00000080U)            /*!<Frame Format                        */
#define  SPI_CR1_SSI                         ((uint32_t)0x00000100U)            /*!<Internal slave select               */
#define  SPI_CR1_SSM                         ((uint32_t)0x00000200U)            /*!<Software slave management           */
#define  SPI_CR1_RXONLY                      ((uint32_t)0x00000400U)            /*!<Receive only                        */
#define  SPI_CR1_CRCL                        ((uint32_t)0x00000800U)            /*!< CRC Length */
#define  SPI_CR1_CRCNEXT                     ((uint32_t)0x00001000U)            /*!<Transmit CRC next                   */
#define  SPI_CR1_CRCEN                       ((uint32_t)0x00002000U)            /*!<Hardware CRC calculation enable     */
#define  SPI_CR1_BIDIOE                      ((uint32_t)0x00004000U)            /*!<Output enable in bidirectional mode */
#define  SPI_CR1_BIDIMODE                    ((uint32_t)0x00008000U)            /*!<Bidirectional data mode enable      */

/*******************  Bit definition for SPI_CR2 register  ********************/
#define  SPI_CR2_RXDMAEN                     ((uint32_t)0x00000001U)            /*!< Rx Buffer DMA Enable */
#define  SPI_CR2_TXDMAEN                     ((uint32_t)0x00000002U)            /*!< Tx Buffer DMA Enable */
#define  SPI_CR2_SSOE                        ((uint32_t)0x00000004U)            /*!< SS Output Enable */
#define  SPI_CR2_NSSP                        ((uint32_t)0x00000008U)            /*!< NSS pulse management Enable */
#define  SPI_CR2_FRF                         ((uint32_t)0x00000010U)            /*!< Frame Format Enable */
#define  SPI_CR2_ERRIE                       ((uint32_t)0x00000020U)            /*!< Error Interrupt Enable */
#define  SPI_CR2_RXNEIE                      ((uint32_t)0x00000040U)            /*!< RX buffer Not Empty Interrupt Enable */
#define  SPI_CR2_TXEIE                       ((uint32_t)0x00000080U)            /*!< Tx buffer Empty Interrupt Enable */
#define  SPI_CR2_DS                          ((uint32_t)0x00000F00U)            /*!< DS[3:0] Data Size */
#define  SPI_CR2_DS_0                        ((uint32_t)0x00000100U)            /*!< Bit 0 */
#define  SPI_CR2_DS_1                        ((uint32_t)0x00000200U)            /*!< Bit 1 */
#define  SPI_CR2_DS_2                        ((uint32_t)0x00000400U)            /*!< Bit 2 */
#define  SPI_CR2_DS_3                        ((uint32_t)0x00000800U)            /*!< Bit 3 */
#define  SPI_CR2_FRXTH                       ((uint32_t)0x00001000U)            /*!< FIFO reception Threshold */
#define  SPI_CR2_LDMARX                      ((uint32_t)0x00002000U)            /*!< Last DMA transfer for reception */
#define  SPI_CR2_LDMATX                      ((uint32_t)0x00004000U)            /*!< Last DMA transfer for transmission */

/********************  Bit definition for SPI_SR register  ********************/
#define  SPI_SR_RXNE                         ((uint32_t)0x00000001U)            /*!< Receive buffer Not Empty */
#define  SPI_SR_TXE                          ((uint32_t)0x00000002U)            /*!< Transmit buffer Empty */
#define  SPI_SR_CHSIDE                       ((uint32_t)0x00000004U)            /*!< Channel side */
#define  SPI_SR_UDR                          ((uint32_t)0x00000008U)            /*!< Underrun flag */
#define  SPI_SR_CRCERR                       ((uint32_t)0x00000010U)            /*!< CRC Error flag */
#define  SPI_SR_MODF                         ((uint32_t)0x00000020U)            /*!< Mode fault */
#define  SPI_SR_OVR                          ((uint32_t)0x00000040U)            /*!< Overrun flag */
#define  SPI_SR_BSY                          ((uint32_t)0x00000080U)            /*!< Busy flag */
#define  SPI_SR_FRE                          ((uint32_t)0x00000100U)            /*!< TI frame format error */
#define  SPI_SR_FRLVL                        ((uint32_t)0x00000600U)            /*!< FIFO Reception Level */
#define  SPI_SR_FRLVL_0                      ((uint32_t)0x00000200U)            /*!< Bit 0 */
#define  SPI_SR_FRLVL_1                      ((uint32_t)0x00000400U)            /*!< Bit 1 */
#define  SPI_SR_FTLVL                        ((uint32_t)0x00001800U)            /*!< FIFO Transmission Level */
#define  SPI_SR_FTLVL_0                      ((uint32_t)0x00000800U)            /*!< Bit 0 */
#define  SPI_SR_FTLVL_1                      ((uint32_t)0x00001000U)            /*!< Bit 1 */

/********************  Bit definition for SPI_DR register  ********************/
#define  SPI_DR_DR                           ((uint32_t)0x0000FFFFU)            /*!<Data Register           */

/*******************  Bit definition for SPI_CRCPR register  ******************/
#define  SPI_CRCPR_CRCPOLY                   ((uint32_t)0x0000FFFFU)            /*!<CRC polynomial register */

/******************  Bit definition for SPI_RXCRCR register  ******************/
#define  SPI_RXCRCR_RXCRC                    ((uint32_t)0x0000FFFFU)            /*!<Rx CRC Register         */

/******************  Bit definition for SPI_TXCRCR register  ******************/
#define  SPI_TXCRCR_TXCRC                    ((uint32_t)0x0000FFFFU)            /*!<Tx CRC Register         */

/******************************************************************************/
/*                                                                            */
/*                                    QUADSPI                                 */
/*                                                                            */
/******************************************************************************/
/*****************  Bit definition for QUADSPI_CR register  *******************/
#define  QUADSPI_CR_EN                           ((uint32_t)0x00000001U)            /*!< Enable */
#define  QUADSPI_CR_ABORT                        ((uint32_t)0x00000002U)            /*!< Abort request */
#define  QUADSPI_CR_DMAEN                        ((uint32_t)0x00000004U)            /*!< DMA Enable */
#define  QUADSPI_CR_TCEN                         ((uint32_t)0x00000008U)            /*!< Timeout Counter Enable */
#define  QUADSPI_CR_SSHIFT                       ((uint32_t)0x00000010U)            /*!< Sample Shift */
#define  QUADSPI_CR_DFM                          ((uint32_t)0x00000040U)            /*!< Dual-flash mode */
#define  QUADSPI_CR_FSEL                         ((uint32_t)0x00000080U)            /*!< Flash memory selection */
#define  QUADSPI_CR_FTHRES                       ((uint32_t)0x00000F00U)            /*!< FTHRES[3:0] FIFO Level */
#define  QUADSPI_CR_TEIE                         ((uint32_t)0x00010000U)            /*!< Transfer Error Interrupt Enable */
#define  QUADSPI_CR_TCIE                         ((uint32_t)0x00020000U)            /*!< Transfer Complete Interrupt Enable */
#define  QUADSPI_CR_FTIE                         ((uint32_t)0x00040000U)            /*!< FIFO Threshold Interrupt Enable */
#define  QUADSPI_CR_SMIE                         ((uint32_t)0x00080000U)            /*!< Status Match Interrupt Enable */
#define  QUADSPI_CR_TOIE                         ((uint32_t)0x00100000U)            /*!< TimeOut Interrupt Enable */
#define  QUADSPI_CR_APMS                         ((uint32_t)0x00400000U)            /*!< Automatic Polling Mode Stop */
#define  QUADSPI_CR_PMM                          ((uint32_t)0x00800000U)            /*!< Polling Match Mode */
#define  QUADSPI_CR_PRESCALER                    ((uint32_t)0xFF000000U)            /*!< PRESCALER[7:0] Clock prescaler */

/*****************  Bit definition for QUADSPI_DCR register  ******************/
#define  QUADSPI_DCR_CKMODE                      ((uint32_t)0x00000001U)            /*!< Mode 0 / Mode 3 */
#define  QUADSPI_DCR_CSHT                        ((uint32_t)0x00000700U)            /*!< CSHT[2:0]: ChipSelect High Time */
#define  QUADSPI_DCR_CSHT_0                      ((uint32_t)0x00000100U)            /*!< Bit 0 */
#define  QUADSPI_DCR_CSHT_1                      ((uint32_t)0x00000200U)            /*!< Bit 1 */
#define  QUADSPI_DCR_CSHT_2                      ((uint32_t)0x00000400U)            /*!< Bit 2 */
#define  QUADSPI_DCR_FSIZE                       ((uint32_t)0x001F0000U)            /*!< FSIZE[4:0]: Flash Size */

/******************  Bit definition for QUADSPI_SR register  *******************/
#define  QUADSPI_SR_TEF                          ((uint32_t)0x00000001U)             /*!< Transfer Error Flag */
#define  QUADSPI_SR_TCF                          ((uint32_t)0x00000002U)             /*!< Transfer Complete Flag */
#define  QUADSPI_SR_FTF                          ((uint32_t)0x00000004U)             /*!< FIFO Threshlod Flag */
#define  QUADSPI_SR_SMF                          ((uint32_t)0x00000008U)             /*!< Status Match Flag */
#define  QUADSPI_SR_TOF                          ((uint32_t)0x00000010U)             /*!< Timeout Flag */
#define  QUADSPI_SR_BUSY                         ((uint32_t)0x00000020U)             /*!< Busy */
#define  QUADSPI_SR_FLEVEL                       ((uint32_t)0x00001F00U)             /*!< FIFO Threshlod Flag */

/******************  Bit definition for QUADSPI_FCR register  ******************/
#define  QUADSPI_FCR_CTEF                        ((uint32_t)0x00000001U)             /*!< Clear Transfer Error Flag */
#define  QUADSPI_FCR_CTCF                        ((uint32_t)0x00000002U)             /*!< Clear Transfer Complete Flag */
#define  QUADSPI_FCR_CSMF                        ((uint32_t)0x00000008U)             /*!< Clear Status Match Flag */
#define  QUADSPI_FCR_CTOF                        ((uint32_t)0x00000010U)             /*!< Clear Timeout Flag */

/******************  Bit definition for QUADSPI_DLR register  ******************/
#define  QUADSPI_DLR_DL                        ((uint32_t)0xFFFFFFFFU)               /*!< DL[31:0]: Data Length */

/******************  Bit definition for QUADSPI_CCR register  ******************/
#define  QUADSPI_CCR_INSTRUCTION                  ((uint32_t)0x000000FFU)            /*!< INSTRUCTION[7:0]: Instruction */
#define  QUADSPI_CCR_IMODE                        ((uint32_t)0x00000300U)            /*!< IMODE[1:0]: Instruction Mode */
#define  QUADSPI_CCR_IMODE_0                      ((uint32_t)0x00000100U)            /*!< Bit 0 */
#define  QUADSPI_CCR_IMODE_1                      ((uint32_t)0x00000200U)            /*!< Bit 1 */
#define  QUADSPI_CCR_ADMODE                       ((uint32_t)0x00000C00U)            /*!< ADMODE[1:0]: Address Mode */
#define  QUADSPI_CCR_ADMODE_0                     ((uint32_t)0x00000400U)            /*!< Bit 0 */
#define  QUADSPI_CCR_ADMODE_1                     ((uint32_t)0x00000800U)            /*!< Bit 1 */
#define  QUADSPI_CCR_ADSIZE                       ((uint32_t)0x00003000U)            /*!< ADSIZE[1:0]: Address Size */
#define  QUADSPI_CCR_ADSIZE_0                     ((uint32_t)0x00001000U)            /*!< Bit 0 */
#define  QUADSPI_CCR_ADSIZE_1                     ((uint32_t)0x00002000U)            /*!< Bit 1 */
#define  QUADSPI_CCR_ABMODE                       ((uint32_t)0x0000C000U)            /*!< ABMODE[1:0]: Alternate Bytes Mode */
#define  QUADSPI_CCR_ABMODE_0                     ((uint32_t)0x00004000U)            /*!< Bit 0 */
#define  QUADSPI_CCR_ABMODE_1                     ((uint32_t)0x00008000U)            /*!< Bit 1 */
#define  QUADSPI_CCR_ABSIZE                       ((uint32_t)0x00030000U)            /*!< ABSIZE[1:0]: Instruction Mode */
#define  QUADSPI_CCR_ABSIZE_0                     ((uint32_t)0x00010000U)            /*!< Bit 0 */
#define  QUADSPI_CCR_ABSIZE_1                     ((uint32_t)0x00020000U)            /*!< Bit 1 */
#define  QUADSPI_CCR_DCYC                         ((uint32_t)0x007C0000U)            /*!< DCYC[4:0]: Dummy Cycles */
#define  QUADSPI_CCR_DMODE                        ((uint32_t)0x03000000U)            /*!< DMODE[1:0]: Data Mode */
#define  QUADSPI_CCR_DMODE_0                      ((uint32_t)0x01000000U)            /*!< Bit 0 */
#define  QUADSPI_CCR_DMODE_1                      ((uint32_t)0x02000000U)            /*!< Bit 1 */
#define  QUADSPI_CCR_FMODE                        ((uint32_t)0x0C000000U)            /*!< FMODE[1:0]: Functional Mode */
#define  QUADSPI_CCR_FMODE_0                      ((uint32_t)0x04000000U)            /*!< Bit 0 */
#define  QUADSPI_CCR_FMODE_1                      ((uint32_t)0x08000000U)            /*!< Bit 1 */
#define  QUADSPI_CCR_SIOO                         ((uint32_t)0x10000000U)            /*!< SIOO: Send Instruction Only Once Mode */
#define  QUADSPI_CCR_DHHC                         ((uint32_t)0x40000000U)            /*!< DHHC: DDR hold */
#define  QUADSPI_CCR_DDRM                         ((uint32_t)0x80000000U)            /*!< DDRM: Double Data Rate Mode */

/******************  Bit definition for QUADSPI_AR register  *******************/
#define  QUADSPI_AR_ADDRESS                       ((uint32_t)0xFFFFFFFFU)            /*!< ADDRESS[31:0]: Address */

/******************  Bit definition for QUADSPI_ABR register  ******************/
#define  QUADSPI_ABR_ALTERNATE                    ((uint32_t)0xFFFFFFFFU)            /*!< ALTERNATE[31:0]: Alternate Bytes */

/******************  Bit definition for QUADSPI_DR register  *******************/
#define  QUADSPI_DR_DATA                          ((uint32_t)0xFFFFFFFFU)            /*!< DATA[31:0]: Data */

/******************  Bit definition for QUADSPI_PSMKR register  ****************/
#define  QUADSPI_PSMKR_MASK                       ((uint32_t)0xFFFFFFFFU)            /*!< MASK[31:0]: Status Mask */

/******************  Bit definition for QUADSPI_PSMAR register  ****************/
#define  QUADSPI_PSMAR_MATCH                      ((uint32_t)0xFFFFFFFFU)            /*!< MATCH[31:0]: Status Match */

/******************  Bit definition for QUADSPI_PIR register  *****************/
#define  QUADSPI_PIR_INTERVAL                     ((uint32_t)0x0000FFFFU)            /*!< INTERVAL[15:0]: Polling Interval */

/******************  Bit definition for QUADSPI_LPTR register  *****************/
#define  QUADSPI_LPTR_TIMEOUT                     ((uint32_t)0x0000FFFFU)            /*!< TIMEOUT[15:0]: Timeout period */

/******************************************************************************/
/*                                                                            */
/*                                 SYSCFG                                     */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for SYSCFG_MEMRMP register  ***************/
#define SYSCFG_MEMRMP_MEM_MODE          ((uint32_t)0x00000007U) /*!< SYSCFG_Memory Remap Config */
#define SYSCFG_MEMRMP_MEM_MODE_0        ((uint32_t)0x00000001U)
#define SYSCFG_MEMRMP_MEM_MODE_1        ((uint32_t)0x00000002U)
#define SYSCFG_MEMRMP_MEM_MODE_2        ((uint32_t)0x00000004U)



/******************  Bit definition for SYSCFG_CFGR1 register  ******************/
#define SYSCFG_CFGR1_FWDIS                  ((uint32_t)0x00000001U) /*!< FIREWALL access enable*/
#define SYSCFG_CFGR1_BOOSTEN                ((uint32_t)0x00000100U) /*!< I/O analog switch voltage booster enable */
#define SYSCFG_CFGR1_I2C_PB6_FMP            ((uint32_t)0x00010000U) /*!< I2C PB6 Fast mode plus */
#define SYSCFG_CFGR1_I2C_PB7_FMP            ((uint32_t)0x00020000U) /*!< I2C PB7 Fast mode plus */
#define SYSCFG_CFGR1_I2C_PB8_FMP            ((uint32_t)0x00040000U) /*!< I2C PB8 Fast mode plus */
#define SYSCFG_CFGR1_I2C_PB9_FMP            ((uint32_t)0x00080000U) /*!< I2C PB9 Fast mode plus */
#define SYSCFG_CFGR1_I2C1_FMP               ((uint32_t)0x00100000U) /*!< I2C1 Fast mode plus */
#define SYSCFG_CFGR1_I2C2_FMP               ((uint32_t)0x00200000U) /*!< I2C2 Fast mode plus */
#define SYSCFG_CFGR1_I2C3_FMP               ((uint32_t)0x00400000U) /*!< I2C3 Fast mode plus */
#define SYSCFG_CFGR1_FPU_IE_0               ((uint32_t)0x04000000U) /*!<  Invalid operation Interrupt enable */
#define SYSCFG_CFGR1_FPU_IE_1               ((uint32_t)0x08000000U) /*!<  Divide-by-zero Interrupt enable */
#define SYSCFG_CFGR1_FPU_IE_2               ((uint32_t)0x10000000U) /*!<  Underflow Interrupt enable */
#define SYSCFG_CFGR1_FPU_IE_3               ((uint32_t)0x20000000U) /*!<  Overflow Interrupt enable */
#define SYSCFG_CFGR1_FPU_IE_4               ((uint32_t)0x40000000U) /*!<  Input denormal Interrupt enable */
#define SYSCFG_CFGR1_FPU_IE_5               ((uint32_t)0x80000000U) /*!<  Inexact Interrupt enable (interrupt disabled at reset) */

/*****************  Bit definition for SYSCFG_EXTICR1 register  ***************/
#define SYSCFG_EXTICR1_EXTI0            ((uint32_t)0x00000007U) /*!<EXTI 0 configuration */
#define SYSCFG_EXTICR1_EXTI1            ((uint32_t)0x00000070U) /*!<EXTI 1 configuration */
#define SYSCFG_EXTICR1_EXTI2            ((uint32_t)0x00000700U) /*!<EXTI 2 configuration */
#define SYSCFG_EXTICR1_EXTI3            ((uint32_t)0x00007000U) /*!<EXTI 3 configuration */
/**
  * @brief   EXTI0 configuration
  */
#define SYSCFG_EXTICR1_EXTI0_PA         ((uint32_t)0x00000000U) /*!<PA[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PB         ((uint32_t)0x00000001U) /*!<PB[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PC         ((uint32_t)0x00000002U) /*!<PC[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PD         ((uint32_t)0x00000003U) /*!<PD[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PE         ((uint32_t)0x00000004U) /*!<PE[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PH         ((uint32_t)0x00000007U) /*!<PH[0] pin */


/**
  * @brief   EXTI1 configuration
  */
#define SYSCFG_EXTICR1_EXTI1_PA         ((uint32_t)0x00000000U) /*!<PA[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PB         ((uint32_t)0x00000010U) /*!<PB[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PC         ((uint32_t)0x00000020U) /*!<PC[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PD         ((uint32_t)0x00000030U) /*!<PD[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PE         ((uint32_t)0x00000040U) /*!<PE[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PH         ((uint32_t)0x00000070U) /*!<PH[1] pin */

/**
  * @brief   EXTI2 configuration
  */
#define SYSCFG_EXTICR1_EXTI2_PA         ((uint32_t)0x00000000U) /*!<PA[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PB         ((uint32_t)0x00000100U) /*!<PB[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PC         ((uint32_t)0x00000200U) /*!<PC[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PD         ((uint32_t)0x00000300U) /*!<PD[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PE         ((uint32_t)0x00000400U) /*!<PE[2] pin */


/**
  * @brief   EXTI3 configuration
  */
#define SYSCFG_EXTICR1_EXTI3_PA         ((uint32_t)0x00000000U) /*!<PA[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PB         ((uint32_t)0x00001000U) /*!<PB[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PC         ((uint32_t)0x00002000U) /*!<PC[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PD         ((uint32_t)0x00003000U) /*!<PD[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PE         ((uint32_t)0x00004000U) /*!<PE[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PG         ((uint32_t)0x00007000U) /*!<PH[3] pin */


/*****************  Bit definition for SYSCFG_EXTICR2 register  ***************/
#define SYSCFG_EXTICR2_EXTI4            ((uint32_t)0x00000007U) /*!<EXTI 4 configuration */
#define SYSCFG_EXTICR2_EXTI5            ((uint32_t)0x00000070U) /*!<EXTI 5 configuration */
#define SYSCFG_EXTICR2_EXTI6            ((uint32_t)0x00000700U) /*!<EXTI 6 configuration */
#define SYSCFG_EXTICR2_EXTI7            ((uint32_t)0x00007000U) /*!<EXTI 7 configuration */
/**
  * @brief   EXTI4 configuration
  */
#define SYSCFG_EXTICR2_EXTI4_PA         ((uint32_t)0x00000000U) /*!<PA[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PB         ((uint32_t)0x00000001U) /*!<PB[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PC         ((uint32_t)0x00000002U) /*!<PC[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PD         ((uint32_t)0x00000003U) /*!<PD[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PE         ((uint32_t)0x00000004U) /*!<PE[4] pin */

/**
  * @brief   EXTI5 configuration
  */
#define SYSCFG_EXTICR2_EXTI5_PA         ((uint32_t)0x00000000U) /*!<PA[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PB         ((uint32_t)0x00000010U) /*!<PB[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PC         ((uint32_t)0x00000020U) /*!<PC[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PD         ((uint32_t)0x00000030U) /*!<PD[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PE         ((uint32_t)0x00000040U) /*!<PE[5] pin */

/**
  * @brief   EXTI6 configuration
  */
#define SYSCFG_EXTICR2_EXTI6_PA         ((uint32_t)0x00000000U) /*!<PA[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PB         ((uint32_t)0x00000100U) /*!<PB[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PC         ((uint32_t)0x00000200U) /*!<PC[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PD         ((uint32_t)0x00000300U) /*!<PD[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PE         ((uint32_t)0x00000400U) /*!<PE[6] pin */

/**
  * @brief   EXTI7 configuration
  */
#define SYSCFG_EXTICR2_EXTI7_PA         ((uint32_t)0x00000000U) /*!<PA[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PB         ((uint32_t)0x00001000U) /*!<PB[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PC         ((uint32_t)0x00002000U) /*!<PC[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PD         ((uint32_t)0x00003000U) /*!<PD[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PE         ((uint32_t)0x00004000U) /*!<PE[7] pin */


/*****************  Bit definition for SYSCFG_EXTICR3 register  ***************/
#define SYSCFG_EXTICR3_EXTI8            ((uint32_t)0x00000007U) /*!<EXTI 8 configuration */
#define SYSCFG_EXTICR3_EXTI9            ((uint32_t)0x00000070U) /*!<EXTI 9 configuration */
#define SYSCFG_EXTICR3_EXTI10           ((uint32_t)0x00000700U) /*!<EXTI 10 configuration */
#define SYSCFG_EXTICR3_EXTI11           ((uint32_t)0x00007000U) /*!<EXTI 11 configuration */

/**
  * @brief   EXTI8 configuration
  */
#define SYSCFG_EXTICR3_EXTI8_PA         ((uint32_t)0x00000000U) /*!<PA[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PB         ((uint32_t)0x00000001U) /*!<PB[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PC         ((uint32_t)0x00000002U) /*!<PC[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PD         ((uint32_t)0x00000003U) /*!<PD[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PE         ((uint32_t)0x00000004U) /*!<PE[8] pin */

/**
  * @brief   EXTI9 configuration
  */
#define SYSCFG_EXTICR3_EXTI9_PA         ((uint32_t)0x00000000U) /*!<PA[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PB         ((uint32_t)0x00000010U) /*!<PB[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PC         ((uint32_t)0x00000020U) /*!<PC[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PD         ((uint32_t)0x00000030U) /*!<PD[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PE         ((uint32_t)0x00000040U) /*!<PE[9] pin */

/**
  * @brief   EXTI10 configuration
  */
#define SYSCFG_EXTICR3_EXTI10_PA        ((uint32_t)0x00000000U) /*!<PA[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PB        ((uint32_t)0x00000100U) /*!<PB[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PC        ((uint32_t)0x00000200U) /*!<PC[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PD        ((uint32_t)0x00000300U) /*!<PD[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PE        ((uint32_t)0x00000400U) /*!<PE[10] pin */

/**
  * @brief   EXTI11 configuration
  */
#define SYSCFG_EXTICR3_EXTI11_PA        ((uint32_t)0x00000000U) /*!<PA[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PB        ((uint32_t)0x00001000U) /*!<PB[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PC        ((uint32_t)0x00002000U) /*!<PC[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PD        ((uint32_t)0x00003000U) /*!<PD[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PE        ((uint32_t)0x00004000U) /*!<PE[11] pin */

/*****************  Bit definition for SYSCFG_EXTICR4 register  ***************/
#define SYSCFG_EXTICR4_EXTI12           ((uint32_t)0x00000007U) /*!<EXTI 12 configuration */
#define SYSCFG_EXTICR4_EXTI13           ((uint32_t)0x00000070U) /*!<EXTI 13 configuration */
#define SYSCFG_EXTICR4_EXTI14           ((uint32_t)0x00000700U) /*!<EXTI 14 configuration */
#define SYSCFG_EXTICR4_EXTI15           ((uint32_t)0x00007000U) /*!<EXTI 15 configuration */
/**
  * @brief   EXTI12 configuration
  */
#define SYSCFG_EXTICR4_EXTI12_PA        ((uint32_t)0x00000000U) /*!<PA[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PB        ((uint32_t)0x00000001U) /*!<PB[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PC        ((uint32_t)0x00000002U) /*!<PC[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PD        ((uint32_t)0x00000003U) /*!<PD[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PE        ((uint32_t)0x00000004U) /*!<PE[12] pin */

/**
  * @brief   EXTI13 configuration
  */
#define SYSCFG_EXTICR4_EXTI13_PA        ((uint32_t)0x00000000U) /*!<PA[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PB        ((uint32_t)0x00000010U) /*!<PB[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PC        ((uint32_t)0x00000020U) /*!<PC[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PD        ((uint32_t)0x00000030U) /*!<PD[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PE        ((uint32_t)0x00000040U) /*!<PE[13] pin */

/**
  * @brief   EXTI14 configuration
  */
#define SYSCFG_EXTICR4_EXTI14_PA        ((uint32_t)0x00000000U) /*!<PA[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PB        ((uint32_t)0x00000100U) /*!<PB[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PC        ((uint32_t)0x00000200U) /*!<PC[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PD        ((uint32_t)0x00000300U) /*!<PD[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PE        ((uint32_t)0x00000400U) /*!<PE[14] pin */

/**
  * @brief   EXTI15 configuration
  */
#define SYSCFG_EXTICR4_EXTI15_PA        ((uint32_t)0x00000000U) /*!<PA[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PB        ((uint32_t)0x00001000U) /*!<PB[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PC        ((uint32_t)0x00002000U) /*!<PC[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PD        ((uint32_t)0x00003000U) /*!<PD[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PE        ((uint32_t)0x00004000U) /*!<PE[15] pin */

/******************  Bit definition for SYSCFG_SCSR register  ****************/
#define SYSCFG_SCSR_SRAM2ER             ((uint32_t)0x00000001U) /*!< SRAM2 Erase Request */
#define SYSCFG_SCSR_SRAM2BSY            ((uint32_t)0x00000002U) /*!< SRAM2 Erase Ongoing */

/******************  Bit definition for SYSCFG_CFGR2 register  ****************/
#define SYSCFG_CFGR2_CLL                ((uint32_t)0x00000001U) /*!< Core Lockup Lock */
#define SYSCFG_CFGR2_SPL                ((uint32_t)0x00000002U) /*!< SRAM Parity Lock*/
#define SYSCFG_CFGR2_PVDL               ((uint32_t)0x00000004U) /*!<  PVD Lock */
#define SYSCFG_CFGR2_ECCL               ((uint32_t)0x00000008U) /*!< ECC Lock*/
#define SYSCFG_CFGR2_SPF                ((uint32_t)0x00000100U) /*!< SRAM Parity Flag */

/******************  Bit definition for SYSCFG_SWPR register  ****************/
#define SYSCFG_SWPR_PAGE0          ((uint32_t)0x00000001U) /*!< SRAM2 Write protection page 0 */
#define SYSCFG_SWPR_PAGE1          ((uint32_t)0x00000002U) /*!< SRAM2 Write protection page 1 */
#define SYSCFG_SWPR_PAGE2          ((uint32_t)0x00000004U) /*!< SRAM2 Write protection page 2 */
#define SYSCFG_SWPR_PAGE3          ((uint32_t)0x00000008U) /*!< SRAM2 Write protection page 3 */
#define SYSCFG_SWPR_PAGE4          ((uint32_t)0x00000010U) /*!< SRAM2 Write protection page 4 */
#define SYSCFG_SWPR_PAGE5          ((uint32_t)0x00000020U) /*!< SRAM2 Write protection page 5 */
#define SYSCFG_SWPR_PAGE6          ((uint32_t)0x00000040U) /*!< SRAM2 Write protection page 6 */
#define SYSCFG_SWPR_PAGE7          ((uint32_t)0x00000080U) /*!< SRAM2 Write protection page 7 */
#define SYSCFG_SWPR_PAGE8          ((uint32_t)0x00000100U) /*!< SRAM2 Write protection page 8 */
#define SYSCFG_SWPR_PAGE9          ((uint32_t)0x00000200U) /*!< SRAM2 Write protection page 9 */
#define SYSCFG_SWPR_PAGE10         ((uint32_t)0x00000400U) /*!< SRAM2 Write protection page 10*/
#define SYSCFG_SWPR_PAGE11         ((uint32_t)0x00000800U) /*!< SRAM2 Write protection page 11*/
#define SYSCFG_SWPR_PAGE12         ((uint32_t)0x00001000U) /*!< SRAM2 Write protection page 12*/
#define SYSCFG_SWPR_PAGE13         ((uint32_t)0x00002000U) /*!< SRAM2 Write protection page 13*/
#define SYSCFG_SWPR_PAGE14         ((uint32_t)0x00004000U) /*!< SRAM2 Write protection page 14*/
#define SYSCFG_SWPR_PAGE15         ((uint32_t)0x00008000U) /*!< SRAM2 Write protection page 15*/

/******************  Bit definition for SYSCFG_SKR register  ****************/
#define SYSCFG_SKR_KEY             ((uint32_t)0x000000FFU) /*!<  SRAM2 write protection key for software erase  */




/******************************************************************************/
/*                                                                            */
/*                                    TIM                                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for TIM_CR1 register  ********************/
#define  TIM_CR1_CEN               ((uint32_t)0x00000001U)            /*!<Counter enable */
#define  TIM_CR1_UDIS              ((uint32_t)0x00000002U)            /*!<Update disable */
#define  TIM_CR1_URS               ((uint32_t)0x00000004U)            /*!<Update request source */
#define  TIM_CR1_OPM               ((uint32_t)0x00000008U)            /*!<One pulse mode */
#define  TIM_CR1_DIR               ((uint32_t)0x00000010U)            /*!<Direction */

#define  TIM_CR1_CMS               ((uint32_t)0x00000060U)            /*!<CMS[1:0] bits (Center-aligned mode selection) */
#define  TIM_CR1_CMS_0             ((uint32_t)0x00000020U)            /*!<Bit 0 */
#define  TIM_CR1_CMS_1             ((uint32_t)0x00000040U)            /*!<Bit 1 */

#define  TIM_CR1_ARPE              ((uint32_t)0x00000080U)            /*!<Auto-reload preload enable */

#define  TIM_CR1_CKD               ((uint32_t)0x00000300U)            /*!<CKD[1:0] bits (clock division) */
#define  TIM_CR1_CKD_0             ((uint32_t)0x00000100U)            /*!<Bit 0 */
#define  TIM_CR1_CKD_1             ((uint32_t)0x00000200U)            /*!<Bit 1 */

#define  TIM_CR1_UIFREMAP          ((uint32_t)0x00000800U)            /*!<Update interrupt flag remap */

/*******************  Bit definition for TIM_CR2 register  ********************/
#define  TIM_CR2_CCPC              ((uint32_t)0x00000001U)            /*!<Capture/Compare Preloaded Control */
#define  TIM_CR2_CCUS              ((uint32_t)0x00000004U)            /*!<Capture/Compare Control Update Selection */
#define  TIM_CR2_CCDS              ((uint32_t)0x00000008U)            /*!<Capture/Compare DMA Selection */

#define  TIM_CR2_MMS               ((uint32_t)0x00000070U)            /*!<MMS[2:0] bits (Master Mode Selection) */
#define  TIM_CR2_MMS_0             ((uint32_t)0x00000010U)            /*!<Bit 0 */
#define  TIM_CR2_MMS_1             ((uint32_t)0x00000020U)            /*!<Bit 1 */
#define  TIM_CR2_MMS_2             ((uint32_t)0x00000040U)            /*!<Bit 2 */

#define  TIM_CR2_TI1S              ((uint32_t)0x00000080U)            /*!<TI1 Selection */
#define  TIM_CR2_OIS1              ((uint32_t)0x00000100U)            /*!<Output Idle state 1 (OC1 output) */
#define  TIM_CR2_OIS1N             ((uint32_t)0x00000200U)            /*!<Output Idle state 1 (OC1N output) */
#define  TIM_CR2_OIS2              ((uint32_t)0x00000400U)            /*!<Output Idle state 2 (OC2 output) */
#define  TIM_CR2_OIS2N             ((uint32_t)0x00000800U)            /*!<Output Idle state 2 (OC2N output) */
#define  TIM_CR2_OIS3              ((uint32_t)0x00001000U)            /*!<Output Idle state 3 (OC3 output) */
#define  TIM_CR2_OIS3N             ((uint32_t)0x00002000U)            /*!<Output Idle state 3 (OC3N output) */
#define  TIM_CR2_OIS4              ((uint32_t)0x00004000U)            /*!<Output Idle state 4 (OC4 output) */
#define  TIM_CR2_OIS5              ((uint32_t)0x00010000U)            /*!<Output Idle state 5 (OC5 output) */
#define  TIM_CR2_OIS6              ((uint32_t)0x00040000U)            /*!<Output Idle state 6 (OC6 output) */

#define  TIM_CR2_MMS2              ((uint32_t)0x00F00000U)            /*!<MMS[2:0] bits (Master Mode Selection) */
#define  TIM_CR2_MMS2_0            ((uint32_t)0x00100000U)            /*!<Bit 0 */
#define  TIM_CR2_MMS2_1            ((uint32_t)0x00200000U)            /*!<Bit 1 */
#define  TIM_CR2_MMS2_2            ((uint32_t)0x00400000U)            /*!<Bit 2 */
#define  TIM_CR2_MMS2_3            ((uint32_t)0x00800000U)            /*!<Bit 2 */

/*******************  Bit definition for TIM_SMCR register  *******************/
#define  TIM_SMCR_SMS              ((uint32_t)0x00010007U)            /*!<SMS[2:0] bits (Slave mode selection) */
#define  TIM_SMCR_SMS_0            ((uint32_t)0x00000001U)            /*!<Bit 0 */
#define  TIM_SMCR_SMS_1            ((uint32_t)0x00000002U)            /*!<Bit 1 */
#define  TIM_SMCR_SMS_2            ((uint32_t)0x00000004U)            /*!<Bit 2 */
#define  TIM_SMCR_SMS_3            ((uint32_t)0x00010000U)            /*!<Bit 3 */

#define  TIM_SMCR_OCCS             ((uint32_t)0x00000008U)            /*!< OCREF clear selection */

#define  TIM_SMCR_TS               ((uint32_t)0x00000070U)            /*!<TS[2:0] bits (Trigger selection) */
#define  TIM_SMCR_TS_0             ((uint32_t)0x00000010U)            /*!<Bit 0 */
#define  TIM_SMCR_TS_1             ((uint32_t)0x00000020U)            /*!<Bit 1 */
#define  TIM_SMCR_TS_2             ((uint32_t)0x00000040U)            /*!<Bit 2 */

#define  TIM_SMCR_MSM              ((uint32_t)0x00000080U)            /*!<Master/slave mode */

#define  TIM_SMCR_ETF              ((uint32_t)0x00000F00U)            /*!<ETF[3:0] bits (External trigger filter) */
#define  TIM_SMCR_ETF_0            ((uint32_t)0x00000100U)            /*!<Bit 0 */
#define  TIM_SMCR_ETF_1            ((uint32_t)0x00000200U)            /*!<Bit 1 */
#define  TIM_SMCR_ETF_2            ((uint32_t)0x00000400U)            /*!<Bit 2 */
#define  TIM_SMCR_ETF_3            ((uint32_t)0x00000800U)            /*!<Bit 3 */

#define  TIM_SMCR_ETPS             ((uint32_t)0x00003000U)            /*!<ETPS[1:0] bits (External trigger prescaler) */
#define  TIM_SMCR_ETPS_0           ((uint32_t)0x00001000U)            /*!<Bit 0 */
#define  TIM_SMCR_ETPS_1           ((uint32_t)0x00002000U)            /*!<Bit 1 */

#define  TIM_SMCR_ECE              ((uint32_t)0x00004000U)            /*!<External clock enable */
#define  TIM_SMCR_ETP              ((uint32_t)0x00008000U)            /*!<External trigger polarity */

/*******************  Bit definition for TIM_DIER register  *******************/
#define  TIM_DIER_UIE              ((uint32_t)0x00000001U)            /*!<Update interrupt enable */
#define  TIM_DIER_CC1IE            ((uint32_t)0x00000002U)            /*!<Capture/Compare 1 interrupt enable */
#define  TIM_DIER_CC2IE            ((uint32_t)0x00000004U)            /*!<Capture/Compare 2 interrupt enable */
#define  TIM_DIER_CC3IE            ((uint32_t)0x00000008U)            /*!<Capture/Compare 3 interrupt enable */
#define  TIM_DIER_CC4IE            ((uint32_t)0x00000010U)            /*!<Capture/Compare 4 interrupt enable */
#define  TIM_DIER_COMIE            ((uint32_t)0x00000020U)            /*!<COM interrupt enable */
#define  TIM_DIER_TIE              ((uint32_t)0x00000040U)            /*!<Trigger interrupt enable */
#define  TIM_DIER_BIE              ((uint32_t)0x00000080U)            /*!<Break interrupt enable */
#define  TIM_DIER_UDE              ((uint32_t)0x00000100U)            /*!<Update DMA request enable */
#define  TIM_DIER_CC1DE            ((uint32_t)0x00000200U)            /*!<Capture/Compare 1 DMA request enable */
#define  TIM_DIER_CC2DE            ((uint32_t)0x00000400U)            /*!<Capture/Compare 2 DMA request enable */
#define  TIM_DIER_CC3DE            ((uint32_t)0x00000800U)            /*!<Capture/Compare 3 DMA request enable */
#define  TIM_DIER_CC4DE            ((uint32_t)0x00001000U)            /*!<Capture/Compare 4 DMA request enable */
#define  TIM_DIER_COMDE            ((uint32_t)0x00002000U)            /*!<COM DMA request enable */
#define  TIM_DIER_TDE              ((uint32_t)0x00004000U)            /*!<Trigger DMA request enable */

/********************  Bit definition for TIM_SR register  ********************/
#define  TIM_SR_UIF                ((uint32_t)0x00000001U)            /*!<Update interrupt Flag */
#define  TIM_SR_CC1IF              ((uint32_t)0x00000002U)            /*!<Capture/Compare 1 interrupt Flag */
#define  TIM_SR_CC2IF              ((uint32_t)0x00000004U)            /*!<Capture/Compare 2 interrupt Flag */
#define  TIM_SR_CC3IF              ((uint32_t)0x00000008U)            /*!<Capture/Compare 3 interrupt Flag */
#define  TIM_SR_CC4IF              ((uint32_t)0x00000010U)            /*!<Capture/Compare 4 interrupt Flag */
#define  TIM_SR_COMIF              ((uint32_t)0x00000020U)            /*!<COM interrupt Flag */
#define  TIM_SR_TIF                ((uint32_t)0x00000040U)            /*!<Trigger interrupt Flag */
#define  TIM_SR_BIF                ((uint32_t)0x00000080U)            /*!<Break interrupt Flag */
#define  TIM_SR_B2IF               ((uint32_t)0x00000100U)            /*!<Break 2 interrupt Flag */
#define  TIM_SR_CC1OF              ((uint32_t)0x00000200U)            /*!<Capture/Compare 1 Overcapture Flag */
#define  TIM_SR_CC2OF              ((uint32_t)0x00000400U)            /*!<Capture/Compare 2 Overcapture Flag */
#define  TIM_SR_CC3OF              ((uint32_t)0x00000800U)            /*!<Capture/Compare 3 Overcapture Flag */
#define  TIM_SR_CC4OF              ((uint32_t)0x00001000U)            /*!<Capture/Compare 4 Overcapture Flag */
#define  TIM_SR_SBIF               ((uint32_t)0x00002000U)            /*!<System Break interrupt Flag */
#define  TIM_SR_CC5IF              ((uint32_t)0x00010000U)            /*!<Capture/Compare 5 interrupt Flag */
#define  TIM_SR_CC6IF              ((uint32_t)0x00020000U)            /*!<Capture/Compare 6 interrupt Flag */


/*******************  Bit definition for TIM_EGR register  ********************/
#define  TIM_EGR_UG                ((uint32_t)0x00000001U)            /*!<Update Generation */
#define  TIM_EGR_CC1G              ((uint32_t)0x00000002U)            /*!<Capture/Compare 1 Generation */
#define  TIM_EGR_CC2G              ((uint32_t)0x00000004U)            /*!<Capture/Compare 2 Generation */
#define  TIM_EGR_CC3G              ((uint32_t)0x00000008U)            /*!<Capture/Compare 3 Generation */
#define  TIM_EGR_CC4G              ((uint32_t)0x00000010U)            /*!<Capture/Compare 4 Generation */
#define  TIM_EGR_COMG              ((uint32_t)0x00000020U)            /*!<Capture/Compare Control Update Generation */
#define  TIM_EGR_TG                ((uint32_t)0x00000040U)            /*!<Trigger Generation */
#define  TIM_EGR_BG                ((uint32_t)0x00000080U)            /*!<Break Generation */
#define  TIM_EGR_B2G               ((uint32_t)0x00000100U)            /*!<Break 2 Generation */


/******************  Bit definition for TIM_CCMR1 register  *******************/
#define  TIM_CCMR1_CC1S            ((uint32_t)0x00000003U)            /*!<CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define  TIM_CCMR1_CC1S_0          ((uint32_t)0x00000001U)            /*!<Bit 0 */
#define  TIM_CCMR1_CC1S_1          ((uint32_t)0x00000002U)            /*!<Bit 1 */

#define  TIM_CCMR1_OC1FE           ((uint32_t)0x00000004U)            /*!<Output Compare 1 Fast enable */
#define  TIM_CCMR1_OC1PE           ((uint32_t)0x00000008U)            /*!<Output Compare 1 Preload enable */

#define  TIM_CCMR1_OC1M            ((uint32_t)0x00010070U)            /*!<OC1M[2:0] bits (Output Compare 1 Mode) */
#define  TIM_CCMR1_OC1M_0          ((uint32_t)0x00000010U)            /*!<Bit 0 */
#define  TIM_CCMR1_OC1M_1          ((uint32_t)0x00000020U)            /*!<Bit 1 */
#define  TIM_CCMR1_OC1M_2          ((uint32_t)0x00000040U)            /*!<Bit 2 */
#define  TIM_CCMR1_OC1M_3          ((uint32_t)0x00010000U)            /*!<Bit 3 */

#define  TIM_CCMR1_OC1CE           ((uint32_t)0x00000080U)            /*!<Output Compare 1 Clear Enable */

#define  TIM_CCMR1_CC2S            ((uint32_t)0x00000300U)            /*!<CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define  TIM_CCMR1_CC2S_0          ((uint32_t)0x00000100U)            /*!<Bit 0 */
#define  TIM_CCMR1_CC2S_1          ((uint32_t)0x00000200U)            /*!<Bit 1 */

#define  TIM_CCMR1_OC2FE           ((uint32_t)0x00000400U)            /*!<Output Compare 2 Fast enable */
#define  TIM_CCMR1_OC2PE           ((uint32_t)0x00000800U)            /*!<Output Compare 2 Preload enable */

#define  TIM_CCMR1_OC2M            ((uint32_t)0x01007000U)            /*!<OC2M[2:0] bits (Output Compare 2 Mode) */
#define  TIM_CCMR1_OC2M_0          ((uint32_t)0x00001000U)            /*!<Bit 0 */
#define  TIM_CCMR1_OC2M_1          ((uint32_t)0x00002000U)            /*!<Bit 1 */
#define  TIM_CCMR1_OC2M_2          ((uint32_t)0x00004000U)            /*!<Bit 2 */
#define  TIM_CCMR1_OC2M_3          ((uint32_t)0x01000000U)            /*!<Bit 3 */

#define  TIM_CCMR1_OC2CE           ((uint32_t)0x00008000U)            /*!<Output Compare 2 Clear Enable */

/*----------------------------------------------------------------------------*/
#define  TIM_CCMR1_IC1PSC          ((uint32_t)0x0000000CU)            /*!<IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define  TIM_CCMR1_IC1PSC_0        ((uint32_t)0x00000004U)            /*!<Bit 0 */
#define  TIM_CCMR1_IC1PSC_1        ((uint32_t)0x00000008U)            /*!<Bit 1 */

#define  TIM_CCMR1_IC1F            ((uint32_t)0x000000F0U)            /*!<IC1F[3:0] bits (Input Capture 1 Filter) */
#define  TIM_CCMR1_IC1F_0          ((uint32_t)0x00000010U)            /*!<Bit 0 */
#define  TIM_CCMR1_IC1F_1          ((uint32_t)0x00000020U)            /*!<Bit 1 */
#define  TIM_CCMR1_IC1F_2          ((uint32_t)0x00000040U)            /*!<Bit 2 */
#define  TIM_CCMR1_IC1F_3          ((uint32_t)0x00000080U)            /*!<Bit 3 */

#define  TIM_CCMR1_IC2PSC          ((uint32_t)0x00000C00U)            /*!<IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define  TIM_CCMR1_IC2PSC_0        ((uint32_t)0x00000400U)            /*!<Bit 0 */
#define  TIM_CCMR1_IC2PSC_1        ((uint32_t)0x00000800U)            /*!<Bit 1 */

#define  TIM_CCMR1_IC2F            ((uint32_t)0x0000F000U)            /*!<IC2F[3:0] bits (Input Capture 2 Filter) */
#define  TIM_CCMR1_IC2F_0          ((uint32_t)0x00001000U)            /*!<Bit 0 */
#define  TIM_CCMR1_IC2F_1          ((uint32_t)0x00002000U)            /*!<Bit 1 */
#define  TIM_CCMR1_IC2F_2          ((uint32_t)0x00004000U)            /*!<Bit 2 */
#define  TIM_CCMR1_IC2F_3          ((uint32_t)0x00008000U)            /*!<Bit 3 */

/******************  Bit definition for TIM_CCMR2 register  *******************/
#define  TIM_CCMR2_CC3S            ((uint32_t)0x00000003U)            /*!<CC3S[1:0] bits (Capture/Compare 3 Selection) */
#define  TIM_CCMR2_CC3S_0          ((uint32_t)0x00000001U)            /*!<Bit 0 */
#define  TIM_CCMR2_CC3S_1          ((uint32_t)0x00000002U)            /*!<Bit 1 */

#define  TIM_CCMR2_OC3FE           ((uint32_t)0x00000004U)            /*!<Output Compare 3 Fast enable */
#define  TIM_CCMR2_OC3PE           ((uint32_t)0x00000008U)            /*!<Output Compare 3 Preload enable */

#define  TIM_CCMR2_OC3M            ((uint32_t)0x00010070U)            /*!<OC3M[2:0] bits (Output Compare 3 Mode) */
#define  TIM_CCMR2_OC3M_0          ((uint32_t)0x00000010U)            /*!<Bit 0 */
#define  TIM_CCMR2_OC3M_1          ((uint32_t)0x00000020U)            /*!<Bit 1 */
#define  TIM_CCMR2_OC3M_2          ((uint32_t)0x00000040U)            /*!<Bit 2 */
#define  TIM_CCMR2_OC3M_3          ((uint32_t)0x00010000U)            /*!<Bit 3 */

#define  TIM_CCMR2_OC3CE           ((uint32_t)0x00000080U)            /*!<Output Compare 3 Clear Enable */

#define  TIM_CCMR2_CC4S            ((uint32_t)0x00000300U)            /*!<CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define  TIM_CCMR2_CC4S_0          ((uint32_t)0x00000100U)            /*!<Bit 0 */
#define  TIM_CCMR2_CC4S_1          ((uint32_t)0x00000200U)            /*!<Bit 1 */

#define  TIM_CCMR2_OC4FE           ((uint32_t)0x00000400U)            /*!<Output Compare 4 Fast enable */
#define  TIM_CCMR2_OC4PE           ((uint32_t)0x00000800U)            /*!<Output Compare 4 Preload enable */

#define  TIM_CCMR2_OC4M            ((uint32_t)0x01007000U)            /*!<OC4M[2:0] bits (Output Compare 4 Mode) */
#define  TIM_CCMR2_OC4M_0          ((uint32_t)0x00001000U)            /*!<Bit 0 */
#define  TIM_CCMR2_OC4M_1          ((uint32_t)0x00002000U)            /*!<Bit 1 */
#define  TIM_CCMR2_OC4M_2          ((uint32_t)0x00004000U)            /*!<Bit 2 */
#define  TIM_CCMR2_OC4M_3          ((uint32_t)0x01000000U)            /*!<Bit 3 */

#define  TIM_CCMR2_OC4CE           ((uint32_t)0x00008000U)            /*!<Output Compare 4 Clear Enable */

/*----------------------------------------------------------------------------*/
#define  TIM_CCMR2_IC3PSC          ((uint32_t)0x0000000CU)            /*!<IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define  TIM_CCMR2_IC3PSC_0        ((uint32_t)0x00000004U)            /*!<Bit 0 */
#define  TIM_CCMR2_IC3PSC_1        ((uint32_t)0x00000008U)            /*!<Bit 1 */

#define  TIM_CCMR2_IC3F            ((uint32_t)0x000000F0U)            /*!<IC3F[3:0] bits (Input Capture 3 Filter) */
#define  TIM_CCMR2_IC3F_0          ((uint32_t)0x00000010U)            /*!<Bit 0 */
#define  TIM_CCMR2_IC3F_1          ((uint32_t)0x00000020U)            /*!<Bit 1 */
#define  TIM_CCMR2_IC3F_2          ((uint32_t)0x00000040U)            /*!<Bit 2 */
#define  TIM_CCMR2_IC3F_3          ((uint32_t)0x00000080U)            /*!<Bit 3 */

#define  TIM_CCMR2_IC4PSC          ((uint32_t)0x00000C00U)            /*!<IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define  TIM_CCMR2_IC4PSC_0        ((uint32_t)0x00000400U)            /*!<Bit 0 */
#define  TIM_CCMR2_IC4PSC_1        ((uint32_t)0x00000800U)            /*!<Bit 1 */

#define  TIM_CCMR2_IC4F            ((uint32_t)0x0000F000U)            /*!<IC4F[3:0] bits (Input Capture 4 Filter) */
#define  TIM_CCMR2_IC4F_0          ((uint32_t)0x00001000U)            /*!<Bit 0 */
#define  TIM_CCMR2_IC4F_1          ((uint32_t)0x00002000U)            /*!<Bit 1 */
#define  TIM_CCMR2_IC4F_2          ((uint32_t)0x00004000U)            /*!<Bit 2 */
#define  TIM_CCMR2_IC4F_3          ((uint32_t)0x00008000U)            /*!<Bit 3 */

/******************  Bit definition for TIM_CCMR3 register  *******************/
#define  TIM_CCMR3_OC5FE           ((uint32_t)0x00000004U)            /*!<Output Compare 5 Fast enable */
#define  TIM_CCMR3_OC5PE           ((uint32_t)0x00000008U)            /*!<Output Compare 5 Preload enable */

#define  TIM_CCMR3_OC5M            ((uint32_t)0x00010070U)            /*!<OC5M[3:0] bits (Output Compare 5 Mode) */
#define  TIM_CCMR3_OC5M_0          ((uint32_t)0x00000010U)            /*!<Bit 0 */
#define  TIM_CCMR3_OC5M_1          ((uint32_t)0x00000020U)            /*!<Bit 1 */
#define  TIM_CCMR3_OC5M_2          ((uint32_t)0x00000040U)            /*!<Bit 2 */
#define  TIM_CCMR3_OC5M_3          ((uint32_t)0x00010000U)            /*!<Bit 3 */

#define  TIM_CCMR3_OC5CE           ((uint32_t)0x00000080U)            /*!<Output Compare 5 Clear Enable */

#define  TIM_CCMR3_OC6FE           ((uint32_t)0x00000400U)            /*!<Output Compare 6 Fast enable */
#define  TIM_CCMR3_OC6PE           ((uint32_t)0x00000800U)            /*!<Output Compare 6 Preload enable */

#define  TIM_CCMR3_OC6M            ((uint32_t)0x01007000U)            /*!<OC6M[3:0] bits (Output Compare 6 Mode) */
#define  TIM_CCMR3_OC6M_0          ((uint32_t)0x00001000U)            /*!<Bit 0 */
#define  TIM_CCMR3_OC6M_1          ((uint32_t)0x00002000U)            /*!<Bit 1 */
#define  TIM_CCMR3_OC6M_2          ((uint32_t)0x00004000U)            /*!<Bit 2 */
#define  TIM_CCMR3_OC6M_3          ((uint32_t)0x01000000U)            /*!<Bit 3 */

#define  TIM_CCMR3_OC6CE           ((uint32_t)0x00008000U)            /*!<Output Compare 6 Clear Enable */

/*******************  Bit definition for TIM_CCER register  *******************/
#define  TIM_CCER_CC1E             ((uint32_t)0x00000001U)            /*!<Capture/Compare 1 output enable */
#define  TIM_CCER_CC1P             ((uint32_t)0x00000002U)            /*!<Capture/Compare 1 output Polarity */
#define  TIM_CCER_CC1NE            ((uint32_t)0x00000004U)            /*!<Capture/Compare 1 Complementary output enable */
#define  TIM_CCER_CC1NP            ((uint32_t)0x00000008U)            /*!<Capture/Compare 1 Complementary output Polarity */
#define  TIM_CCER_CC2E             ((uint32_t)0x00000010U)            /*!<Capture/Compare 2 output enable */
#define  TIM_CCER_CC2P             ((uint32_t)0x00000020U)            /*!<Capture/Compare 2 output Polarity */
#define  TIM_CCER_CC2NE            ((uint32_t)0x00000040U)            /*!<Capture/Compare 2 Complementary output enable */
#define  TIM_CCER_CC2NP            ((uint32_t)0x00000080U)            /*!<Capture/Compare 2 Complementary output Polarity */
#define  TIM_CCER_CC3E             ((uint32_t)0x00000100U)            /*!<Capture/Compare 3 output enable */
#define  TIM_CCER_CC3P             ((uint32_t)0x00000200U)            /*!<Capture/Compare 3 output Polarity */
#define  TIM_CCER_CC3NE            ((uint32_t)0x00000400U)            /*!<Capture/Compare 3 Complementary output enable */
#define  TIM_CCER_CC3NP            ((uint32_t)0x00000800U)            /*!<Capture/Compare 3 Complementary output Polarity */
#define  TIM_CCER_CC4E             ((uint32_t)0x00001000U)            /*!<Capture/Compare 4 output enable */
#define  TIM_CCER_CC4P             ((uint32_t)0x00002000U)            /*!<Capture/Compare 4 output Polarity */
#define  TIM_CCER_CC4NP            ((uint32_t)0x00008000U)            /*!<Capture/Compare 4 Complementary output Polarity */
#define  TIM_CCER_CC5E             ((uint32_t)0x00010000U)            /*!<Capture/Compare 5 output enable */
#define  TIM_CCER_CC5P             ((uint32_t)0x00020000U)            /*!<Capture/Compare 5 output Polarity */
#define  TIM_CCER_CC6E             ((uint32_t)0x00100000U)            /*!<Capture/Compare 6 output enable */
#define  TIM_CCER_CC6P             ((uint32_t)0x00200000U)            /*!<Capture/Compare 6 output Polarity */

/*******************  Bit definition for TIM_CNT register  ********************/
#define  TIM_CNT_CNT               ((uint32_t)0xFFFFFFFFU)            /*!<Counter Value */
#define  TIM_CNT_UIFCPY            ((uint32_t)0x80000000U)            /*!<Update interrupt flag copy (if UIFREMAP=1) */

/*******************  Bit definition for TIM_PSC register  ********************/
#define  TIM_PSC_PSC               ((uint32_t)0x0000FFFFU)            /*!<Prescaler Value */

/*******************  Bit definition for TIM_ARR register  ********************/
#define  TIM_ARR_ARR               ((uint32_t)0xFFFFFFFFU)            /*!<Actual auto-reload Value */

/*******************  Bit definition for TIM_RCR register  ********************/
#define  TIM_RCR_REP               ((uint32_t)0x0000FFFFU)            /*!<Repetition Counter Value */

/*******************  Bit definition for TIM_CCR1 register  *******************/
#define  TIM_CCR1_CCR1             ((uint32_t)0x0000FFFFU)            /*!<Capture/Compare 1 Value */

/*******************  Bit definition for TIM_CCR2 register  *******************/
#define  TIM_CCR2_CCR2             ((uint32_t)0x0000FFFFU)            /*!<Capture/Compare 2 Value */

/*******************  Bit definition for TIM_CCR3 register  *******************/
#define  TIM_CCR3_CCR3             ((uint32_t)0x0000FFFFU)            /*!<Capture/Compare 3 Value */

/*******************  Bit definition for TIM_CCR4 register  *******************/
#define  TIM_CCR4_CCR4             ((uint32_t)0x0000FFFFU)            /*!<Capture/Compare 4 Value */

/*******************  Bit definition for TIM_CCR5 register  *******************/
#define  TIM_CCR5_CCR5             ((uint32_t)0xFFFFFFFFU)            /*!<Capture/Compare 5 Value */
#define  TIM_CCR5_GC5C1            ((uint32_t)0x20000000U)            /*!<Group Channel 5 and Channel 1 */
#define  TIM_CCR5_GC5C2            ((uint32_t)0x40000000U)            /*!<Group Channel 5 and Channel 2 */
#define  TIM_CCR5_GC5C3            ((uint32_t)0x80000000U)            /*!<Group Channel 5 and Channel 3 */

/*******************  Bit definition for TIM_CCR6 register  *******************/
#define  TIM_CCR6_CCR6             ((uint32_t)0x0000FFFFU)            /*!<Capture/Compare 6 Value */

/*******************  Bit definition for TIM_BDTR register  *******************/
#define  TIM_BDTR_DTG              ((uint32_t)0x000000FFU)            /*!<DTG[0:7] bits (Dead-Time Generator set-up) */
#define  TIM_BDTR_DTG_0            ((uint32_t)0x00000001U)            /*!<Bit 0 */
#define  TIM_BDTR_DTG_1            ((uint32_t)0x00000002U)            /*!<Bit 1 */
#define  TIM_BDTR_DTG_2            ((uint32_t)0x00000004U)            /*!<Bit 2 */
#define  TIM_BDTR_DTG_3            ((uint32_t)0x00000008U)            /*!<Bit 3 */
#define  TIM_BDTR_DTG_4            ((uint32_t)0x00000010U)            /*!<Bit 4 */
#define  TIM_BDTR_DTG_5            ((uint32_t)0x00000020U)            /*!<Bit 5 */
#define  TIM_BDTR_DTG_6            ((uint32_t)0x00000040U)            /*!<Bit 6 */
#define  TIM_BDTR_DTG_7            ((uint32_t)0x00000080U)            /*!<Bit 7 */

#define  TIM_BDTR_LOCK             ((uint32_t)0x00000300U)            /*!<LOCK[1:0] bits (Lock Configuration) */
#define  TIM_BDTR_LOCK_0           ((uint32_t)0x00000100U)            /*!<Bit 0 */
#define  TIM_BDTR_LOCK_1           ((uint32_t)0x00000200U)            /*!<Bit 1 */

#define  TIM_BDTR_OSSI             ((uint32_t)0x00000400U)            /*!<Off-State Selection for Idle mode */
#define  TIM_BDTR_OSSR             ((uint32_t)0x00000800U)            /*!<Off-State Selection for Run mode */
#define  TIM_BDTR_BKE              ((uint32_t)0x00001000U)            /*!<Break enable for Break 1 */
#define  TIM_BDTR_BKP              ((uint32_t)0x00002000U)            /*!<Break Polarity for Break 1 */
#define  TIM_BDTR_AOE              ((uint32_t)0x00004000U)            /*!<Automatic Output enable */
#define  TIM_BDTR_MOE              ((uint32_t)0x00008000U)            /*!<Main Output enable */

#define  TIM_BDTR_BKF              ((uint32_t)0x000F0000U)            /*!<Break Filter for Break 1 */
#define  TIM_BDTR_BK2F             ((uint32_t)0x00F00000U)            /*!<Break Filter for Break 2 */

#define  TIM_BDTR_BK2E             ((uint32_t)0x01000000U)            /*!<Break enable for Break 2 */
#define  TIM_BDTR_BK2P             ((uint32_t)0x02000000U)            /*!<Break Polarity for Break 2 */

/*******************  Bit definition for TIM_DCR register  ********************/
#define  TIM_DCR_DBA               ((uint32_t)0x0000001FU)            /*!<DBA[4:0] bits (DMA Base Address) */
#define  TIM_DCR_DBA_0             ((uint32_t)0x00000001U)            /*!<Bit 0 */
#define  TIM_DCR_DBA_1             ((uint32_t)0x00000002U)            /*!<Bit 1 */
#define  TIM_DCR_DBA_2             ((uint32_t)0x00000004U)            /*!<Bit 2 */
#define  TIM_DCR_DBA_3             ((uint32_t)0x00000008U)            /*!<Bit 3 */
#define  TIM_DCR_DBA_4             ((uint32_t)0x00000010U)            /*!<Bit 4 */

#define  TIM_DCR_DBL               ((uint32_t)0x00001F00U)            /*!<DBL[4:0] bits (DMA Burst Length) */
#define  TIM_DCR_DBL_0             ((uint32_t)0x00000100U)            /*!<Bit 0 */
#define  TIM_DCR_DBL_1             ((uint32_t)0x00000200U)            /*!<Bit 1 */
#define  TIM_DCR_DBL_2             ((uint32_t)0x00000400U)            /*!<Bit 2 */
#define  TIM_DCR_DBL_3             ((uint32_t)0x00000800U)            /*!<Bit 3 */
#define  TIM_DCR_DBL_4             ((uint32_t)0x00001000U)            /*!<Bit 4 */

/*******************  Bit definition for TIM_DMAR register  *******************/
#define  TIM_DMAR_DMAB             ((uint32_t)0x0000FFFFU)            /*!<DMA register for burst accesses */

/*******************  Bit definition for TIM1_OR1 register  *******************/
#define TIM1_OR1_ETR_ADC1_RMP      ((uint32_t)0x00000003U)            /*!<ETR_ADC1_RMP[1:0] bits (TIM1 ETR remap on ADC1) */
#define TIM1_OR1_ETR_ADC1_RMP_0    ((uint32_t)0x00000001U)            /*!<Bit 0 */
#define TIM1_OR1_ETR_ADC1_RMP_1    ((uint32_t)0x00000002U)            /*!<Bit 1 */

#define TIM1_OR1_TI1_RMP           ((uint32_t)0x00000010U)            /*!<TIM1 Input Capture 1 remap */

/*******************  Bit definition for TIM1_OR2 register  *******************/
#define TIM1_OR2_BKINE             ((uint32_t)0x00000001U)            /*!<BRK BKIN input enable */
#define TIM1_OR2_BKCMP1E           ((uint32_t)0x00000002U)            /*!<BRK COMP1 enable */
#define TIM1_OR2_BKCMP2E           ((uint32_t)0x00000004U)            /*!<BRK COMP2 enable */
#define TIM1_OR2_BKINP             ((uint32_t)0x00000200U)            /*!<BRK BKIN input polarity */
#define TIM1_OR2_BKCMP1P           ((uint32_t)0x00000400U)            /*!<BRK COMP1 input polarity */
#define TIM1_OR2_BKCMP2P           ((uint32_t)0x00000800U)            /*!<BRK COMP2 input polarity */

#define TIM1_OR2_ETRSEL            ((uint32_t)0x0001C000U)            /*!<ETRSEL[2:0] bits (TIM1 ETR source selection) */
#define TIM1_OR2_ETRSEL_0          ((uint32_t)0x00004000U)            /*!<Bit 0 */
#define TIM1_OR2_ETRSEL_1          ((uint32_t)0x00008000U)            /*!<Bit 1 */
#define TIM1_OR2_ETRSEL_2          ((uint32_t)0x00010000U)            /*!<Bit 2 */

/*******************  Bit definition for TIM1_OR3 register  *******************/
#define TIM1_OR3_BK2INE            ((uint32_t)0x00000001U)            /*!<BRK2 BKIN2 input enable */
#define TIM1_OR3_BK2CMP1E          ((uint32_t)0x00000002U)            /*!<BRK2 COMP1 enable */
#define TIM1_OR3_BK2CMP2E          ((uint32_t)0x00000004U)            /*!<BRK2 COMP2 enable */
#define TIM1_OR3_BK2INP            ((uint32_t)0x00000200U)            /*!<BRK2 BKIN2 input polarity */
#define TIM1_OR3_BK2CMP1P          ((uint32_t)0x00000400U)            /*!<BRK2 COMP1 input polarity */
#define TIM1_OR3_BK2CMP2P          ((uint32_t)0x00000800U)            /*!<BRK2 COMP2 input polarity */


/*******************  Bit definition for TIM2_OR1 register  *******************/
#define TIM2_OR1_ITR1_RMP          ((uint32_t)0x00000001U)            /*!<TIM2 Internal trigger 1 remap */
#define TIM2_OR1_ETR1_RMP          ((uint32_t)0x00000002U)            /*!<TIM2 External trigger 1 remap */

#define TIM2_OR1_TI4_RMP           ((uint32_t)0x0000000CU)            /*!<TI4_RMP[1:0] bits (TIM2 Input Capture 4 remap) */
#define TIM2_OR1_TI4_RMP_0         ((uint32_t)0x00000004U)            /*!<Bit 0 */
#define TIM2_OR1_TI4_RMP_1         ((uint32_t)0x00000008U)            /*!<Bit 1 */

/*******************  Bit definition for TIM2_OR2 register  *******************/
#define TIM2_OR2_ETRSEL            ((uint32_t)0x0001C000U)            /*!<ETRSEL[2:0] bits (TIM2 ETR source selection) */
#define TIM2_OR2_ETRSEL_0          ((uint32_t)0x00004000U)            /*!<Bit 0 */
#define TIM2_OR2_ETRSEL_1          ((uint32_t)0x00008000U)            /*!<Bit 1 */
#define TIM2_OR2_ETRSEL_2          ((uint32_t)0x00010000U)            /*!<Bit 2 */


/*******************  Bit definition for TIM15_OR1 register  ******************/
#define TIM15_OR1_TI1_RMP          ((uint32_t)0x00000001U)            /*!<TIM15 Input Capture 1 remap */

#define TIM15_OR1_ENCODER_MODE     ((uint32_t)0x00000006U)            /*!<ENCODER_MODE[1:0] bits (TIM15 Encoder mode) */
#define TIM15_OR1_ENCODER_MODE_0   ((uint32_t)0x00000002U)            /*!<Bit 0 */
#define TIM15_OR1_ENCODER_MODE_1   ((uint32_t)0x00000004U)            /*!<Bit 1 */

/*******************  Bit definition for TIM15_OR2 register  ******************/
#define TIM15_OR2_BKINE            ((uint32_t)0x00000001U)            /*!<BRK BKIN input enable */
#define TIM15_OR2_BKCMP1E          ((uint32_t)0x00000002U)            /*!<BRK COMP1 enable */
#define TIM15_OR2_BKCMP2E          ((uint32_t)0x00000004U)            /*!<BRK COMP2 enable */
#define TIM15_OR2_BKINP            ((uint32_t)0x00000200U)            /*!<BRK BKIN input polarity */
#define TIM15_OR2_BKCMP1P          ((uint32_t)0x00000400U)            /*!<BRK COMP1 input polarity */
#define TIM15_OR2_BKCMP2P          ((uint32_t)0x00000800U)            /*!<BRK COMP2 input polarity */

/*******************  Bit definition for TIM16_OR1 register  ******************/
#define TIM16_OR1_TI1_RMP          ((uint32_t)0x00000007U)            /*!<TI1_RMP[2:0] bits (TIM16 Input Capture 1 remap) */
#define TIM16_OR1_TI1_RMP_0        ((uint32_t)0x00000001U)            /*!<Bit 0 */
#define TIM16_OR1_TI1_RMP_1        ((uint32_t)0x00000002U)            /*!<Bit 1 */
#define TIM16_OR1_TI1_RMP_2        ((uint32_t)0x00000004U)            /*!<Bit 2 */

/*******************  Bit definition for TIM16_OR2 register  ******************/
#define TIM16_OR2_BKINE            ((uint32_t)0x00000001U)            /*!<BRK BKIN input enable */
#define TIM16_OR2_BKCMP1E          ((uint32_t)0x00000002U)            /*!<BRK COMP1 enable */
#define TIM16_OR2_BKCMP2E          ((uint32_t)0x00000004U)            /*!<BRK COMP2 enable */
#define TIM16_OR2_BKINP            ((uint32_t)0x00000200U)            /*!<BRK BKIN input polarity */
#define TIM16_OR2_BKCMP1P          ((uint32_t)0x00000400U)            /*!<BRK COMP1 input polarity */
#define TIM16_OR2_BKCMP2P          ((uint32_t)0x00000800U)            /*!<BRK COMP2 input polarity */


/******************************************************************************/
/*                                                                            */
/*                         Low Power Timer (LPTTIM)                           */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for LPTIM_ISR register  *******************/
#define  LPTIM_ISR_CMPM                         ((uint32_t)0x00000001U)            /*!< Compare match */
#define  LPTIM_ISR_ARRM                         ((uint32_t)0x00000002U)            /*!< Autoreload match */
#define  LPTIM_ISR_EXTTRIG                      ((uint32_t)0x00000004U)            /*!< External trigger edge event */
#define  LPTIM_ISR_CMPOK                        ((uint32_t)0x00000008U)            /*!< Compare register update OK */
#define  LPTIM_ISR_ARROK                        ((uint32_t)0x00000010U)            /*!< Autoreload register update OK */
#define  LPTIM_ISR_UP                           ((uint32_t)0x00000020U)            /*!< Counter direction change down to up */
#define  LPTIM_ISR_DOWN                         ((uint32_t)0x00000040U)            /*!< Counter direction change up to down */

/******************  Bit definition for LPTIM_ICR register  *******************/
#define  LPTIM_ICR_CMPMCF                       ((uint32_t)0x00000001U)            /*!< Compare match Clear Flag */
#define  LPTIM_ICR_ARRMCF                       ((uint32_t)0x00000002U)            /*!< Autoreload match Clear Flag */
#define  LPTIM_ICR_EXTTRIGCF                    ((uint32_t)0x00000004U)            /*!< External trigger edge event Clear Flag */
#define  LPTIM_ICR_CMPOKCF                      ((uint32_t)0x00000008U)            /*!< Compare register update OK Clear Flag */
#define  LPTIM_ICR_ARROKCF                      ((uint32_t)0x00000010U)            /*!< Autoreload register update OK Clear Flag */
#define  LPTIM_ICR_UPCF                         ((uint32_t)0x00000020U)            /*!< Counter direction change down to up Clear Flag */
#define  LPTIM_ICR_DOWNCF                       ((uint32_t)0x00000040U)            /*!< Counter direction change up to down Clear Flag */

/******************  Bit definition for LPTIM_IER register ********************/
#define  LPTIM_IER_CMPMIE                       ((uint32_t)0x00000001U)            /*!< Compare match Interrupt Enable */
#define  LPTIM_IER_ARRMIE                       ((uint32_t)0x00000002U)            /*!< Autoreload match Interrupt Enable */
#define  LPTIM_IER_EXTTRIGIE                    ((uint32_t)0x00000004U)            /*!< External trigger edge event Interrupt Enable */
#define  LPTIM_IER_CMPOKIE                      ((uint32_t)0x00000008U)            /*!< Compare register update OK Interrupt Enable */
#define  LPTIM_IER_ARROKIE                      ((uint32_t)0x00000010U)            /*!< Autoreload register update OK Interrupt Enable */
#define  LPTIM_IER_UPIE                         ((uint32_t)0x00000020U)            /*!< Counter direction change down to up Interrupt Enable */
#define  LPTIM_IER_DOWNIE                       ((uint32_t)0x00000040U)            /*!< Counter direction change up to down Interrupt Enable */

/******************  Bit definition for LPTIM_CFGR register *******************/
#define  LPTIM_CFGR_CKSEL                       ((uint32_t)0x00000001U)             /*!< Clock selector */

#define  LPTIM_CFGR_CKPOL                       ((uint32_t)0x00000006U)             /*!< CKPOL[1:0] bits (Clock polarity) */
#define  LPTIM_CFGR_CKPOL_0                     ((uint32_t)0x00000002U)             /*!< Bit 0 */
#define  LPTIM_CFGR_CKPOL_1                     ((uint32_t)0x00000004U)             /*!< Bit 1 */

#define  LPTIM_CFGR_CKFLT                       ((uint32_t)0x00000018U)             /*!< CKFLT[1:0] bits (Configurable digital filter for external clock) */
#define  LPTIM_CFGR_CKFLT_0                     ((uint32_t)0x00000008U)             /*!< Bit 0 */
#define  LPTIM_CFGR_CKFLT_1                     ((uint32_t)0x00000010U)             /*!< Bit 1 */

#define  LPTIM_CFGR_TRGFLT                      ((uint32_t)0x000000C0U)             /*!< TRGFLT[1:0] bits (Configurable digital filter for trigger) */
#define  LPTIM_CFGR_TRGFLT_0                    ((uint32_t)0x00000040U)             /*!< Bit 0 */
#define  LPTIM_CFGR_TRGFLT_1                    ((uint32_t)0x00000080U)             /*!< Bit 1 */

#define  LPTIM_CFGR_PRESC                       ((uint32_t)0x00000E00U)             /*!< PRESC[2:0] bits (Clock prescaler) */
#define  LPTIM_CFGR_PRESC_0                     ((uint32_t)0x00000200U)             /*!< Bit 0 */
#define  LPTIM_CFGR_PRESC_1                     ((uint32_t)0x00000400U)             /*!< Bit 1 */
#define  LPTIM_CFGR_PRESC_2                     ((uint32_t)0x00000800U)             /*!< Bit 2 */

#define  LPTIM_CFGR_TRIGSEL                     ((uint32_t)0x0000E000U)             /*!< TRIGSEL[2:0]] bits (Trigger selector) */
#define  LPTIM_CFGR_TRIGSEL_0                   ((uint32_t)0x00002000U)             /*!< Bit 0 */
#define  LPTIM_CFGR_TRIGSEL_1                   ((uint32_t)0x00004000U)             /*!< Bit 1 */
#define  LPTIM_CFGR_TRIGSEL_2                   ((uint32_t)0x00008000U)             /*!< Bit 2 */

#define  LPTIM_CFGR_TRIGEN                      ((uint32_t)0x00060000U)             /*!< TRIGEN[1:0] bits (Trigger enable and polarity) */
#define  LPTIM_CFGR_TRIGEN_0                    ((uint32_t)0x00020000U)             /*!< Bit 0 */
#define  LPTIM_CFGR_TRIGEN_1                    ((uint32_t)0x00040000U)             /*!< Bit 1 */

#define  LPTIM_CFGR_TIMOUT                      ((uint32_t)0x00080000U)             /*!< Timout enable */
#define  LPTIM_CFGR_WAVE                        ((uint32_t)0x00100000U)             /*!< Waveform shape */
#define  LPTIM_CFGR_WAVPOL                      ((uint32_t)0x00200000U)             /*!< Waveform shape polarity */
#define  LPTIM_CFGR_PRELOAD                     ((uint32_t)0x00400000U)             /*!< Reg update mode */
#define  LPTIM_CFGR_COUNTMODE                   ((uint32_t)0x00800000U)             /*!< Counter mode enable */
#define  LPTIM_CFGR_ENC                         ((uint32_t)0x01000000U)             /*!< Encoder mode enable */

/******************  Bit definition for LPTIM_CR register  ********************/
#define  LPTIM_CR_ENABLE                        ((uint32_t)0x00000001U)             /*!< LPTIMer enable */
#define  LPTIM_CR_SNGSTRT                       ((uint32_t)0x00000002U)             /*!< Timer start in single mode */
#define  LPTIM_CR_CNTSTRT                       ((uint32_t)0x00000004U)             /*!< Timer start in continuous mode */

/******************  Bit definition for LPTIM_CMP register  *******************/
#define  LPTIM_CMP_CMP                          ((uint32_t)0x0000FFFFU)             /*!< Compare register */

/******************  Bit definition for LPTIM_ARR register  *******************/
#define  LPTIM_ARR_ARR                          ((uint32_t)0x0000FFFFU)             /*!< Auto reload register */

/******************  Bit definition for LPTIM_CNT register  *******************/
#define  LPTIM_CNT_CNT                          ((uint32_t)0x0000FFFFU)             /*!< Counter register */

/******************  Bit definition for LPTIM_OR register  *******************/
#define  LPTIM_OR_OR                           ((uint32_t)0x00000003U)               /*!< LPTIMER[1:0] bits (Remap selection) */
#define  LPTIM_OR_OR_0                         ((uint32_t)0x00000001U)               /*!< Bit 0 */
#define  LPTIM_OR_OR_1                         ((uint32_t)0x00000002U)               /*!< Bit 1 */

/******************************************************************************/
/*                                                                            */
/*                      Analog Comparators (COMP)                             */
/*                                                                            */
/******************************************************************************/
/**********************  Bit definition for COMPx_CSR register  ***************/
#define COMP_CSR_EN                    ((uint32_t)0x00000001U) /*!< COMPx enable */

#define COMP_CSR_PWRMODE               ((uint32_t)0x0000000CU) /*!< COMPx power mode */
#define COMP_CSR_PWRMODE_0             ((uint32_t)0x00000004U) /*!< COMPx power mode bit 0 */
#define COMP_CSR_PWRMODE_1             ((uint32_t)0x00000008U) /*!< COMPx power mode bit 1 */

#define COMP_CSR_INMSEL                ((uint32_t)0x00000070U) /*!< COMPx inverting input (minus) selection */
#define COMP_CSR_INMSEL_0              ((uint32_t)0x00000010U) /*!< COMPx inverting input (minus) selection bit 0 */
#define COMP_CSR_INMSEL_1              ((uint32_t)0x00000020U) /*!< COMPx inverting input (minus) selection bit 1 */
#define COMP_CSR_INMSEL_2              ((uint32_t)0x00000040U) /*!< COMPx inverting input (minus) selection bit 2 */

#define COMP_CSR_INPSEL                ((uint32_t)0x00000180U) /*!< COMPx non inverting input (plus) selection */
#define COMP_CSR_INPSEL_0              ((uint32_t)0x00000080U) /*!< COMPx non inverting input (plus) selection bit 0*/
#define COMP_CSR_INPSEL_1              ((uint32_t)0x00000100U) /*!< COMPx non inverting input (plus) selection bit 1*/

#define COMP_CSR_WINMODE               ((uint32_t)0x00000200U) /*!< COMPx window mode. Bit intended to be used with COMP common instance (COMP_Common_TypeDef)  */
#define COMP_CSR_POLARITY              ((uint32_t)0x00008000U) /*!< COMPx output polarity */

#define COMP_CSR_HYST                  ((uint32_t)0x00030000U) /*!< COMPx hysteresis */
#define COMP_CSR_HYST_0                ((uint32_t)0x00010000U) /*!< COMPx hysteresis bit 0 */
#define COMP_CSR_HYST_1                ((uint32_t)0x00020000U) /*!< COMPx hysteresis bit 1 */

#define COMP_CSR_BLANKING              ((uint32_t)0x001C0000U) /*!< COMPx blanking source */
#define COMP_CSR_BLANKING_0            ((uint32_t)0x00040000U) /*!< COMPx blanking source bit 0 */
#define COMP_CSR_BLANKING_1            ((uint32_t)0x00080000U) /*!< COMPx blanking source bit 1 */
#define COMP_CSR_BLANKING_2            ((uint32_t)0x00100000U) /*!< COMPx blanking source bit 2 */

#define COMP_CSR_BRGEN                 ((uint32_t)0x00400000U) /*!< COMPx voltage scaler enable */
#define COMP_CSR_SCALEN                ((uint32_t)0x00800000U) /*!< COMPx scaler bridge enable */

#define COMP_CSR_INMESEL               ((uint32_t)0x06000000U) /*!< COMPx inverting input (minus) extended selection */
#define COMP_CSR_INMESEL_0             ((uint32_t)0x02000000U) /*!< COMPx inverting input (minus) extended selection bit 0*/
#define COMP_CSR_INMESEL_1             ((uint32_t)0x04000000U) /*!< COMPx inverting input (minus) extended selection bit 1*/

#define COMP_CSR_VALUE                 ((uint32_t)0x40000000U) /*!< COMPx value */
#define COMP_CSR_LOCK                  ((uint32_t)0x80000000U) /*!< COMPx lock */

/******************************************************************************/
/*                                                                            */
/*                         Operational Amplifier (OPAMP)                      */
/*                                                                            */
/******************************************************************************/
/*********************  Bit definition for OPAMPx_CSR register  ***************/
#define OPAMP_CSR_OPAMPxEN            ((uint32_t)0x00000001U) /*!< OPAMP enable */
#define OPAMP_CSR_OPALPM              ((uint32_t)0x00000002U) /*!< Operational amplifier Low Power Mode */

#define OPAMP_CSR_OPAMODE             ((uint32_t)0x0000000CU) /*!< Operational amplifier PGA mode */
#define OPAMP_CSR_OPAMODE_0           ((uint32_t)0x00000004U) /*!< Bit 0 */
#define OPAMP_CSR_OPAMODE_1           ((uint32_t)0x00000008U) /*!< Bit 1 */

#define OPAMP_CSR_PGGAIN             ((uint32_t)0x00000030U) /*!< Operational amplifier Programmable amplifier gain value */
#define OPAMP_CSR_PGGAIN_0           ((uint32_t)0x00000010U) /*!< Bit 0 */
#define OPAMP_CSR_PGGAIN_1           ((uint32_t)0x00000020U) /*!< Bit 1 */

#define OPAMP_CSR_VMSEL               ((uint32_t)0x00000300U) /*!< Inverting input selection */
#define OPAMP_CSR_VMSEL_0             ((uint32_t)0x00000100U) /*!< Bit 0 */
#define OPAMP_CSR_VMSEL_1             ((uint32_t)0x00000200U) /*!< Bit 1 */

#define OPAMP_CSR_VPSEL               ((uint32_t)0x00000400U) /*!< Non inverted input selection */
#define OPAMP_CSR_CALON               ((uint32_t)0x00001000U) /*!< Calibration mode enable */
#define OPAMP_CSR_CALSEL              ((uint32_t)0x00002000U) /*!< Calibration selection */
#define OPAMP_CSR_USERTRIM            ((uint32_t)0x00004000U) /*!< User trimming enable */
#define OPAMP_CSR_CALOUT              ((uint32_t)0x00008000U) /*!< Operational amplifier1 calibration output */

/*********************  Bit definition for OPAMP1_CSR register  ***************/
#define OPAMP1_CSR_OPAEN               ((uint32_t)0x00000001U) /*!< Operational amplifier1 Enable */
#define OPAMP1_CSR_OPALPM              ((uint32_t)0x00000002U) /*!< Operational amplifier1 Low Power Mode */

#define OPAMP1_CSR_OPAMODE             ((uint32_t)0x0000000CU) /*!< Operational amplifier1 PGA mode */
#define OPAMP1_CSR_OPAMODE_0           ((uint32_t)0x00000004U) /*!< Bit 0 */
#define OPAMP1_CSR_OPAMODE_1           ((uint32_t)0x00000008U) /*!< Bit 1 */

#define OPAMP1_CSR_PGAGAIN             ((uint32_t)0x00000030U) /*!< Operational amplifier1 Programmable amplifier gain value */
#define OPAMP1_CSR_PGAGAIN_0           ((uint32_t)0x00000010U) /*!< Bit 0 */
#define OPAMP1_CSR_PGAGAIN_1           ((uint32_t)0x00000020U) /*!< Bit 1 */

#define OPAMP1_CSR_VMSEL               ((uint32_t)0x00000300U) /*!< Inverting input selection */
#define OPAMP1_CSR_VMSEL_0             ((uint32_t)0x00000100U) /*!< Bit 0 */
#define OPAMP1_CSR_VMSEL_1             ((uint32_t)0x00000200U) /*!< Bit 1 */

#define OPAMP1_CSR_VPSEL               ((uint32_t)0x00000400U) /*!< Non inverted input selection */
#define OPAMP1_CSR_CALON               ((uint32_t)0x00001000U) /*!< Calibration mode enable */
#define OPAMP1_CSR_CALSEL              ((uint32_t)0x00002000U) /*!< Calibration selection */
#define OPAMP1_CSR_USERTRIM            ((uint32_t)0x00004000U) /*!< User trimming enable */
#define OPAMP1_CSR_CALOUT              ((uint32_t)0x00008000U) /*!< Operational amplifier1 calibration output */
#define OPAMP1_CSR_OPARANGE            ((uint32_t)0x80000000U) /*!< Operational amplifiers power supply range for stability */

/*******************  Bit definition for OPAMP_OTR register  ******************/
#define OPAMP_OTR_TRIMOFFSETN            ((uint32_t)0x0000001FU)        /*!< Trim for NMOS differential pairs */
#define OPAMP_OTR_TRIMOFFSETP            ((uint32_t)0x00001F00U)        /*!< Trim for PMOS differential pairs */

/*******************  Bit definition for OPAMP1_OTR register  ******************/
#define OPAMP1_OTR_TRIMOFFSETN            ((uint32_t)0x0000001FU)        /*!< Trim for NMOS differential pairs */
#define OPAMP1_OTR_TRIMOFFSETP            ((uint32_t)0x00001F00U)        /*!< Trim for PMOS differential pairs */

/*******************  Bit definition for OPAMP_LPOTR register  ****************/
#define OPAMP_LPOTR_TRIMLPOFFSETN        ((uint32_t)0x0000001FU)        /*!< Trim for NMOS differential pairs */
#define OPAMP_LPOTR_TRIMLPOFFSETP        ((uint32_t)0x00001F00U)        /*!< Trim for PMOS differential pairs */

/*******************  Bit definition for OPAMP1_LPOTR register  ****************/
#define OPAMP1_LPOTR_TRIMLPOFFSETN        ((uint32_t)0x0000001FU)        /*!< Trim for NMOS differential pairs */
#define OPAMP1_LPOTR_TRIMLPOFFSETP        ((uint32_t)0x00001F00U)        /*!< Trim for PMOS differential pairs */

/******************************************************************************/
/*                                                                            */
/*                          Touch Sensing Controller (TSC)                    */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for TSC_CR register  *********************/
#define TSC_CR_TSCE                         ((uint32_t)0x00000001U)            /*!<Touch sensing controller enable */
#define TSC_CR_START                        ((uint32_t)0x00000002U)            /*!<Start acquisition */
#define TSC_CR_AM                           ((uint32_t)0x00000004U)            /*!<Acquisition mode */
#define TSC_CR_SYNCPOL                      ((uint32_t)0x00000008U)            /*!<Synchronization pin polarity */
#define TSC_CR_IODEF                        ((uint32_t)0x00000010U)            /*!<IO default mode */

#define TSC_CR_MCV                          ((uint32_t)0x000000E0U)            /*!<MCV[2:0] bits (Max Count Value) */
#define TSC_CR_MCV_0                        ((uint32_t)0x00000020U)            /*!<Bit 0 */
#define TSC_CR_MCV_1                        ((uint32_t)0x00000040U)            /*!<Bit 1 */
#define TSC_CR_MCV_2                        ((uint32_t)0x00000080U)            /*!<Bit 2 */

#define TSC_CR_PGPSC                        ((uint32_t)0x00007000U)            /*!<PGPSC[2:0] bits (Pulse Generator Prescaler) */
#define TSC_CR_PGPSC_0                      ((uint32_t)0x00001000U)            /*!<Bit 0 */
#define TSC_CR_PGPSC_1                      ((uint32_t)0x00002000U)            /*!<Bit 1 */
#define TSC_CR_PGPSC_2                      ((uint32_t)0x00004000U)            /*!<Bit 2 */

#define TSC_CR_SSPSC                        ((uint32_t)0x00008000U)            /*!<Spread Spectrum Prescaler */
#define TSC_CR_SSE                          ((uint32_t)0x00010000U)            /*!<Spread Spectrum Enable */

#define TSC_CR_SSD                          ((uint32_t)0x00FE0000U)            /*!<SSD[6:0] bits (Spread Spectrum Deviation) */
#define TSC_CR_SSD_0                        ((uint32_t)0x00020000U)            /*!<Bit 0 */
#define TSC_CR_SSD_1                        ((uint32_t)0x00040000U)            /*!<Bit 1 */
#define TSC_CR_SSD_2                        ((uint32_t)0x00080000U)            /*!<Bit 2 */
#define TSC_CR_SSD_3                        ((uint32_t)0x00100000U)            /*!<Bit 3 */
#define TSC_CR_SSD_4                        ((uint32_t)0x00200000U)            /*!<Bit 4 */
#define TSC_CR_SSD_5                        ((uint32_t)0x00400000U)            /*!<Bit 5 */
#define TSC_CR_SSD_6                        ((uint32_t)0x00800000U)            /*!<Bit 6 */

#define TSC_CR_CTPL                         ((uint32_t)0x0F000000U)            /*!<CTPL[3:0] bits (Charge Transfer pulse low) */
#define TSC_CR_CTPL_0                       ((uint32_t)0x01000000U)            /*!<Bit 0 */
#define TSC_CR_CTPL_1                       ((uint32_t)0x02000000U)            /*!<Bit 1 */
#define TSC_CR_CTPL_2                       ((uint32_t)0x04000000U)            /*!<Bit 2 */
#define TSC_CR_CTPL_3                       ((uint32_t)0x08000000U)            /*!<Bit 3 */

#define TSC_CR_CTPH                         ((uint32_t)0xF0000000U)            /*!<CTPH[3:0] bits (Charge Transfer pulse high) */
#define TSC_CR_CTPH_0                       ((uint32_t)0x10000000U)            /*!<Bit 0 */
#define TSC_CR_CTPH_1                       ((uint32_t)0x20000000U)            /*!<Bit 1 */
#define TSC_CR_CTPH_2                       ((uint32_t)0x40000000U)            /*!<Bit 2 */
#define TSC_CR_CTPH_3                       ((uint32_t)0x80000000U)            /*!<Bit 3 */

/*******************  Bit definition for TSC_IER register  ********************/
#define TSC_IER_EOAIE                       ((uint32_t)0x00000001U)            /*!<End of acquisition interrupt enable */
#define TSC_IER_MCEIE                       ((uint32_t)0x00000002U)            /*!<Max count error interrupt enable */

/*******************  Bit definition for TSC_ICR register  ********************/
#define TSC_ICR_EOAIC                       ((uint32_t)0x00000001U)            /*!<End of acquisition interrupt clear */
#define TSC_ICR_MCEIC                       ((uint32_t)0x00000002U)            /*!<Max count error interrupt clear */

/*******************  Bit definition for TSC_ISR register  ********************/
#define TSC_ISR_EOAF                        ((uint32_t)0x00000001U)            /*!<End of acquisition flag */
#define TSC_ISR_MCEF                        ((uint32_t)0x00000002U)            /*!<Max count error flag */

/*******************  Bit definition for TSC_IOHCR register  ******************/
#define TSC_IOHCR_G1_IO1                    ((uint32_t)0x00000001U)            /*!<GROUP1_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G1_IO2                    ((uint32_t)0x00000002U)            /*!<GROUP1_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G1_IO3                    ((uint32_t)0x00000004U)            /*!<GROUP1_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G1_IO4                    ((uint32_t)0x00000008U)            /*!<GROUP1_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G2_IO1                    ((uint32_t)0x00000010U)            /*!<GROUP2_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G2_IO2                    ((uint32_t)0x00000020U)            /*!<GROUP2_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G2_IO3                    ((uint32_t)0x00000040U)            /*!<GROUP2_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G2_IO4                    ((uint32_t)0x00000080U)            /*!<GROUP2_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G3_IO1                    ((uint32_t)0x00000100U)            /*!<GROUP3_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G3_IO2                    ((uint32_t)0x00000200U)            /*!<GROUP3_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G3_IO3                    ((uint32_t)0x00000400U)            /*!<GROUP3_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G3_IO4                    ((uint32_t)0x00000800U)            /*!<GROUP3_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G4_IO1                    ((uint32_t)0x00001000U)            /*!<GROUP4_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G4_IO2                    ((uint32_t)0x00002000U)            /*!<GROUP4_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G4_IO3                    ((uint32_t)0x00004000U)            /*!<GROUP4_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G4_IO4                    ((uint32_t)0x00008000U)            /*!<GROUP4_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G5_IO1                    ((uint32_t)0x00010000U)            /*!<GROUP5_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G5_IO2                    ((uint32_t)0x00020000U)            /*!<GROUP5_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G5_IO3                    ((uint32_t)0x00040000U)            /*!<GROUP5_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G5_IO4                    ((uint32_t)0x00080000U)            /*!<GROUP5_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G6_IO1                    ((uint32_t)0x00100000U)            /*!<GROUP6_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G6_IO2                    ((uint32_t)0x00200000U)            /*!<GROUP6_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G6_IO3                    ((uint32_t)0x00400000U)            /*!<GROUP6_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G6_IO4                    ((uint32_t)0x00800000U)            /*!<GROUP6_IO4 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G7_IO1                    ((uint32_t)0x01000000U)            /*!<GROUP7_IO1 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G7_IO2                    ((uint32_t)0x02000000U)            /*!<GROUP7_IO2 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G7_IO3                    ((uint32_t)0x04000000U)            /*!<GROUP7_IO3 schmitt trigger hysteresis mode */
#define TSC_IOHCR_G7_IO4                    ((uint32_t)0x08000000U)            /*!<GROUP7_IO4 schmitt trigger hysteresis mode */

/*******************  Bit definition for TSC_IOASCR register  *****************/
#define TSC_IOASCR_G1_IO1                   ((uint32_t)0x00000001U)            /*!<GROUP1_IO1 analog switch enable */
#define TSC_IOASCR_G1_IO2                   ((uint32_t)0x00000002U)            /*!<GROUP1_IO2 analog switch enable */
#define TSC_IOASCR_G1_IO3                   ((uint32_t)0x00000004U)            /*!<GROUP1_IO3 analog switch enable */
#define TSC_IOASCR_G1_IO4                   ((uint32_t)0x00000008U)            /*!<GROUP1_IO4 analog switch enable */
#define TSC_IOASCR_G2_IO1                   ((uint32_t)0x00000010U)            /*!<GROUP2_IO1 analog switch enable */
#define TSC_IOASCR_G2_IO2                   ((uint32_t)0x00000020U)            /*!<GROUP2_IO2 analog switch enable */
#define TSC_IOASCR_G2_IO3                   ((uint32_t)0x00000040U)            /*!<GROUP2_IO3 analog switch enable */
#define TSC_IOASCR_G2_IO4                   ((uint32_t)0x00000080U)            /*!<GROUP2_IO4 analog switch enable */
#define TSC_IOASCR_G3_IO1                   ((uint32_t)0x00000100U)            /*!<GROUP3_IO1 analog switch enable */
#define TSC_IOASCR_G3_IO2                   ((uint32_t)0x00000200U)            /*!<GROUP3_IO2 analog switch enable */
#define TSC_IOASCR_G3_IO3                   ((uint32_t)0x00000400U)            /*!<GROUP3_IO3 analog switch enable */
#define TSC_IOASCR_G3_IO4                   ((uint32_t)0x00000800U)            /*!<GROUP3_IO4 analog switch enable */
#define TSC_IOASCR_G4_IO1                   ((uint32_t)0x00001000U)            /*!<GROUP4_IO1 analog switch enable */
#define TSC_IOASCR_G4_IO2                   ((uint32_t)0x00002000U)            /*!<GROUP4_IO2 analog switch enable */
#define TSC_IOASCR_G4_IO3                   ((uint32_t)0x00004000U)            /*!<GROUP4_IO3 analog switch enable */
#define TSC_IOASCR_G4_IO4                   ((uint32_t)0x00008000U)            /*!<GROUP4_IO4 analog switch enable */
#define TSC_IOASCR_G5_IO1                   ((uint32_t)0x00010000U)            /*!<GROUP5_IO1 analog switch enable */
#define TSC_IOASCR_G5_IO2                   ((uint32_t)0x00020000U)            /*!<GROUP5_IO2 analog switch enable */
#define TSC_IOASCR_G5_IO3                   ((uint32_t)0x00040000U)            /*!<GROUP5_IO3 analog switch enable */
#define TSC_IOASCR_G5_IO4                   ((uint32_t)0x00080000U)            /*!<GROUP5_IO4 analog switch enable */
#define TSC_IOASCR_G6_IO1                   ((uint32_t)0x00100000U)            /*!<GROUP6_IO1 analog switch enable */
#define TSC_IOASCR_G6_IO2                   ((uint32_t)0x00200000U)            /*!<GROUP6_IO2 analog switch enable */
#define TSC_IOASCR_G6_IO3                   ((uint32_t)0x00400000U)            /*!<GROUP6_IO3 analog switch enable */
#define TSC_IOASCR_G6_IO4                   ((uint32_t)0x00800000U)            /*!<GROUP6_IO4 analog switch enable */
#define TSC_IOASCR_G7_IO1                   ((uint32_t)0x01000000U)            /*!<GROUP7_IO1 analog switch enable */
#define TSC_IOASCR_G7_IO2                   ((uint32_t)0x02000000U)            /*!<GROUP7_IO2 analog switch enable */
#define TSC_IOASCR_G7_IO3                   ((uint32_t)0x04000000U)            /*!<GROUP7_IO3 analog switch enable */
#define TSC_IOASCR_G7_IO4                   ((uint32_t)0x08000000U)            /*!<GROUP7_IO4 analog switch enable */

/*******************  Bit definition for TSC_IOSCR register  ******************/
#define TSC_IOSCR_G1_IO1                    ((uint32_t)0x00000001U)            /*!<GROUP1_IO1 sampling mode */
#define TSC_IOSCR_G1_IO2                    ((uint32_t)0x00000002U)            /*!<GROUP1_IO2 sampling mode */
#define TSC_IOSCR_G1_IO3                    ((uint32_t)0x00000004U)            /*!<GROUP1_IO3 sampling mode */
#define TSC_IOSCR_G1_IO4                    ((uint32_t)0x00000008U)            /*!<GROUP1_IO4 sampling mode */
#define TSC_IOSCR_G2_IO1                    ((uint32_t)0x00000010U)            /*!<GROUP2_IO1 sampling mode */
#define TSC_IOSCR_G2_IO2                    ((uint32_t)0x00000020U)            /*!<GROUP2_IO2 sampling mode */
#define TSC_IOSCR_G2_IO3                    ((uint32_t)0x00000040U)            /*!<GROUP2_IO3 sampling mode */
#define TSC_IOSCR_G2_IO4                    ((uint32_t)0x00000080U)            /*!<GROUP2_IO4 sampling mode */
#define TSC_IOSCR_G3_IO1                    ((uint32_t)0x00000100U)            /*!<GROUP3_IO1 sampling mode */
#define TSC_IOSCR_G3_IO2                    ((uint32_t)0x00000200U)            /*!<GROUP3_IO2 sampling mode */
#define TSC_IOSCR_G3_IO3                    ((uint32_t)0x00000400U)            /*!<GROUP3_IO3 sampling mode */
#define TSC_IOSCR_G3_IO4                    ((uint32_t)0x00000800U)            /*!<GROUP3_IO4 sampling mode */
#define TSC_IOSCR_G4_IO1                    ((uint32_t)0x00001000U)            /*!<GROUP4_IO1 sampling mode */
#define TSC_IOSCR_G4_IO2                    ((uint32_t)0x00002000U)            /*!<GROUP4_IO2 sampling mode */
#define TSC_IOSCR_G4_IO3                    ((uint32_t)0x00004000U)            /*!<GROUP4_IO3 sampling mode */
#define TSC_IOSCR_G4_IO4                    ((uint32_t)0x00008000U)            /*!<GROUP4_IO4 sampling mode */
#define TSC_IOSCR_G5_IO1                    ((uint32_t)0x00010000U)            /*!<GROUP5_IO1 sampling mode */
#define TSC_IOSCR_G5_IO2                    ((uint32_t)0x00020000U)            /*!<GROUP5_IO2 sampling mode */
#define TSC_IOSCR_G5_IO3                    ((uint32_t)0x00040000U)            /*!<GROUP5_IO3 sampling mode */
#define TSC_IOSCR_G5_IO4                    ((uint32_t)0x00080000U)            /*!<GROUP5_IO4 sampling mode */
#define TSC_IOSCR_G6_IO1                    ((uint32_t)0x00100000U)            /*!<GROUP6_IO1 sampling mode */
#define TSC_IOSCR_G6_IO2                    ((uint32_t)0x00200000U)            /*!<GROUP6_IO2 sampling mode */
#define TSC_IOSCR_G6_IO3                    ((uint32_t)0x00400000U)            /*!<GROUP6_IO3 sampling mode */
#define TSC_IOSCR_G6_IO4                    ((uint32_t)0x00800000U)            /*!<GROUP6_IO4 sampling mode */
#define TSC_IOSCR_G7_IO1                    ((uint32_t)0x01000000U)            /*!<GROUP7_IO1 sampling mode */
#define TSC_IOSCR_G7_IO2                    ((uint32_t)0x02000000U)            /*!<GROUP7_IO2 sampling mode */
#define TSC_IOSCR_G7_IO3                    ((uint32_t)0x04000000U)            /*!<GROUP7_IO3 sampling mode */
#define TSC_IOSCR_G7_IO4                    ((uint32_t)0x08000000U)            /*!<GROUP7_IO4 sampling mode */

/*******************  Bit definition for TSC_IOCCR register  ******************/
#define TSC_IOCCR_G1_IO1                    ((uint32_t)0x00000001U)            /*!<GROUP1_IO1 channel mode */
#define TSC_IOCCR_G1_IO2                    ((uint32_t)0x00000002U)            /*!<GROUP1_IO2 channel mode */
#define TSC_IOCCR_G1_IO3                    ((uint32_t)0x00000004U)            /*!<GROUP1_IO3 channel mode */
#define TSC_IOCCR_G1_IO4                    ((uint32_t)0x00000008U)            /*!<GROUP1_IO4 channel mode */
#define TSC_IOCCR_G2_IO1                    ((uint32_t)0x00000010U)            /*!<GROUP2_IO1 channel mode */
#define TSC_IOCCR_G2_IO2                    ((uint32_t)0x00000020U)            /*!<GROUP2_IO2 channel mode */
#define TSC_IOCCR_G2_IO3                    ((uint32_t)0x00000040U)            /*!<GROUP2_IO3 channel mode */
#define TSC_IOCCR_G2_IO4                    ((uint32_t)0x00000080U)            /*!<GROUP2_IO4 channel mode */
#define TSC_IOCCR_G3_IO1                    ((uint32_t)0x00000100U)            /*!<GROUP3_IO1 channel mode */
#define TSC_IOCCR_G3_IO2                    ((uint32_t)0x00000200U)            /*!<GROUP3_IO2 channel mode */
#define TSC_IOCCR_G3_IO3                    ((uint32_t)0x00000400U)            /*!<GROUP3_IO3 channel mode */
#define TSC_IOCCR_G3_IO4                    ((uint32_t)0x00000800U)            /*!<GROUP3_IO4 channel mode */
#define TSC_IOCCR_G4_IO1                    ((uint32_t)0x00001000U)            /*!<GROUP4_IO1 channel mode */
#define TSC_IOCCR_G4_IO2                    ((uint32_t)0x00002000U)            /*!<GROUP4_IO2 channel mode */
#define TSC_IOCCR_G4_IO3                    ((uint32_t)0x00004000U)            /*!<GROUP4_IO3 channel mode */
#define TSC_IOCCR_G4_IO4                    ((uint32_t)0x00008000U)            /*!<GROUP4_IO4 channel mode */
#define TSC_IOCCR_G5_IO1                    ((uint32_t)0x00010000U)            /*!<GROUP5_IO1 channel mode */
#define TSC_IOCCR_G5_IO2                    ((uint32_t)0x00020000U)            /*!<GROUP5_IO2 channel mode */
#define TSC_IOCCR_G5_IO3                    ((uint32_t)0x00040000U)            /*!<GROUP5_IO3 channel mode */
#define TSC_IOCCR_G5_IO4                    ((uint32_t)0x00080000U)            /*!<GROUP5_IO4 channel mode */
#define TSC_IOCCR_G6_IO1                    ((uint32_t)0x00100000U)            /*!<GROUP6_IO1 channel mode */
#define TSC_IOCCR_G6_IO2                    ((uint32_t)0x00200000U)            /*!<GROUP6_IO2 channel mode */
#define TSC_IOCCR_G6_IO3                    ((uint32_t)0x00400000U)            /*!<GROUP6_IO3 channel mode */
#define TSC_IOCCR_G6_IO4                    ((uint32_t)0x00800000U)            /*!<GROUP6_IO4 channel mode */
#define TSC_IOCCR_G7_IO1                    ((uint32_t)0x01000000U)            /*!<GROUP7_IO1 channel mode */
#define TSC_IOCCR_G7_IO2                    ((uint32_t)0x02000000U)            /*!<GROUP7_IO2 channel mode */
#define TSC_IOCCR_G7_IO3                    ((uint32_t)0x04000000U)            /*!<GROUP7_IO3 channel mode */
#define TSC_IOCCR_G7_IO4                    ((uint32_t)0x08000000U)            /*!<GROUP7_IO4 channel mode */

/*******************  Bit definition for TSC_IOGCSR register  *****************/
#define TSC_IOGCSR_G1E                      ((uint32_t)0x00000001U)            /*!<Analog IO GROUP1 enable */
#define TSC_IOGCSR_G2E                      ((uint32_t)0x00000002U)            /*!<Analog IO GROUP2 enable */
#define TSC_IOGCSR_G3E                      ((uint32_t)0x00000004U)            /*!<Analog IO GROUP3 enable */
#define TSC_IOGCSR_G4E                      ((uint32_t)0x00000008U)            /*!<Analog IO GROUP4 enable */
#define TSC_IOGCSR_G5E                      ((uint32_t)0x00000010U)            /*!<Analog IO GROUP5 enable */
#define TSC_IOGCSR_G6E                      ((uint32_t)0x00000020U)            /*!<Analog IO GROUP6 enable */
#define TSC_IOGCSR_G7E                      ((uint32_t)0x00000040U)            /*!<Analog IO GROUP7 enable */
#define TSC_IOGCSR_G1S                      ((uint32_t)0x00010000U)            /*!<Analog IO GROUP1 status */
#define TSC_IOGCSR_G2S                      ((uint32_t)0x00020000U)            /*!<Analog IO GROUP2 status */
#define TSC_IOGCSR_G3S                      ((uint32_t)0x00040000U)            /*!<Analog IO GROUP3 status */
#define TSC_IOGCSR_G4S                      ((uint32_t)0x00080000U)            /*!<Analog IO GROUP4 status */
#define TSC_IOGCSR_G5S                      ((uint32_t)0x00100000U)            /*!<Analog IO GROUP5 status */
#define TSC_IOGCSR_G6S                      ((uint32_t)0x00200000U)            /*!<Analog IO GROUP6 status */
#define TSC_IOGCSR_G7S                      ((uint32_t)0x00400000U)            /*!<Analog IO GROUP7 status */

/*******************  Bit definition for TSC_IOGXCR register  *****************/
#define TSC_IOGXCR_CNT                      ((uint32_t)0x00003FFFU)            /*!<CNT[13:0] bits (Counter value) */

/******************************************************************************/
/*                                                                            */
/*      Universal Synchronous Asynchronous Receiver Transmitter (USART)       */
/*                                                                            */
/******************************************************************************/

/*
* @brief Specific device feature definitions (not present on all devices in the STM32L4 family)
*/

/* Support of TCBGT feature : Supported from USART IP version c7amba_sci3 v1.3 */
#define USART_TCBGT_SUPPORT

/******************  Bit definition for USART_CR1 register  *******************/
#define  USART_CR1_UE                        ((uint32_t)0x00000001U)            /*!< USART Enable */
#define  USART_CR1_UESM                      ((uint32_t)0x00000002U)            /*!< USART Enable in STOP Mode */
#define  USART_CR1_RE                        ((uint32_t)0x00000004U)            /*!< Receiver Enable */
#define  USART_CR1_TE                        ((uint32_t)0x00000008U)            /*!< Transmitter Enable */
#define  USART_CR1_IDLEIE                    ((uint32_t)0x00000010U)            /*!< IDLE Interrupt Enable */
#define  USART_CR1_RXNEIE                    ((uint32_t)0x00000020U)            /*!< RXNE Interrupt Enable */
#define  USART_CR1_TCIE                      ((uint32_t)0x00000040U)            /*!< Transmission Complete Interrupt Enable */
#define  USART_CR1_TXEIE                     ((uint32_t)0x00000080U)            /*!< TXE Interrupt Enable */
#define  USART_CR1_PEIE                      ((uint32_t)0x00000100U)            /*!< PE Interrupt Enable */
#define  USART_CR1_PS                        ((uint32_t)0x00000200U)            /*!< Parity Selection */
#define  USART_CR1_PCE                       ((uint32_t)0x00000400U)            /*!< Parity Control Enable */
#define  USART_CR1_WAKE                      ((uint32_t)0x00000800U)            /*!< Receiver Wakeup method */
#define  USART_CR1_M                         ((uint32_t)0x10001000U)            /*!< Word length */
#define  USART_CR1_M0                        ((uint32_t)0x00001000U)            /*!< Word length - Bit 0 */
#define  USART_CR1_MME                       ((uint32_t)0x00002000U)            /*!< Mute Mode Enable */
#define  USART_CR1_CMIE                      ((uint32_t)0x00004000U)            /*!< Character match interrupt enable */
#define  USART_CR1_OVER8                     ((uint32_t)0x00008000U)            /*!< Oversampling by 8-bit or 16-bit mode */
#define  USART_CR1_DEDT                      ((uint32_t)0x001F0000U)            /*!< DEDT[4:0] bits (Driver Enable Deassertion Time) */
#define  USART_CR1_DEDT_0                    ((uint32_t)0x00010000U)            /*!< Bit 0 */
#define  USART_CR1_DEDT_1                    ((uint32_t)0x00020000U)            /*!< Bit 1 */
#define  USART_CR1_DEDT_2                    ((uint32_t)0x00040000U)            /*!< Bit 2 */
#define  USART_CR1_DEDT_3                    ((uint32_t)0x00080000U)            /*!< Bit 3 */
#define  USART_CR1_DEDT_4                    ((uint32_t)0x00100000U)            /*!< Bit 4 */
#define  USART_CR1_DEAT                      ((uint32_t)0x03E00000U)            /*!< DEAT[4:0] bits (Driver Enable Assertion Time) */
#define  USART_CR1_DEAT_0                    ((uint32_t)0x00200000U)            /*!< Bit 0 */
#define  USART_CR1_DEAT_1                    ((uint32_t)0x00400000U)            /*!< Bit 1 */
#define  USART_CR1_DEAT_2                    ((uint32_t)0x00800000U)            /*!< Bit 2 */
#define  USART_CR1_DEAT_3                    ((uint32_t)0x01000000U)            /*!< Bit 3 */
#define  USART_CR1_DEAT_4                    ((uint32_t)0x02000000U)            /*!< Bit 4 */
#define  USART_CR1_RTOIE                     ((uint32_t)0x04000000U)            /*!< Receive Time Out interrupt enable */
#define  USART_CR1_EOBIE                     ((uint32_t)0x08000000U)            /*!< End of Block interrupt enable */
#define  USART_CR1_M1                        ((uint32_t)0x10000000U)            /*!< Word length - Bit 1 */

/******************  Bit definition for USART_CR2 register  *******************/
#define  USART_CR2_ADDM7                     ((uint32_t)0x00000010U)            /*!< 7-bit or 4-bit Address Detection */
#define  USART_CR2_LBDL                      ((uint32_t)0x00000020U)            /*!< LIN Break Detection Length */
#define  USART_CR2_LBDIE                     ((uint32_t)0x00000040U)            /*!< LIN Break Detection Interrupt Enable */
#define  USART_CR2_LBCL                      ((uint32_t)0x00000100U)            /*!< Last Bit Clock pulse */
#define  USART_CR2_CPHA                      ((uint32_t)0x00000200U)            /*!< Clock Phase */
#define  USART_CR2_CPOL                      ((uint32_t)0x00000400U)            /*!< Clock Polarity */
#define  USART_CR2_CLKEN                     ((uint32_t)0x00000800U)            /*!< Clock Enable */
#define  USART_CR2_STOP                      ((uint32_t)0x00003000U)            /*!< STOP[1:0] bits (STOP bits) */
#define  USART_CR2_STOP_0                    ((uint32_t)0x00001000U)            /*!< Bit 0 */
#define  USART_CR2_STOP_1                    ((uint32_t)0x00002000U)            /*!< Bit 1 */
#define  USART_CR2_LINEN                     ((uint32_t)0x00004000U)            /*!< LIN mode enable */
#define  USART_CR2_SWAP                      ((uint32_t)0x00008000U)            /*!< SWAP TX/RX pins */
#define  USART_CR2_RXINV                     ((uint32_t)0x00010000U)            /*!< RX pin active level inversion */
#define  USART_CR2_TXINV                     ((uint32_t)0x00020000U)            /*!< TX pin active level inversion */
#define  USART_CR2_DATAINV                   ((uint32_t)0x00040000U)            /*!< Binary data inversion */
#define  USART_CR2_MSBFIRST                  ((uint32_t)0x00080000U)            /*!< Most Significant Bit First */
#define  USART_CR2_ABREN                     ((uint32_t)0x00100000U)            /*!< Auto Baud-Rate Enable*/
#define  USART_CR2_ABRMODE                   ((uint32_t)0x00600000U)            /*!< ABRMOD[1:0] bits (Auto Baud-Rate Mode) */
#define  USART_CR2_ABRMODE_0                 ((uint32_t)0x00200000U)            /*!< Bit 0 */
#define  USART_CR2_ABRMODE_1                 ((uint32_t)0x00400000U)            /*!< Bit 1 */
#define  USART_CR2_RTOEN                     ((uint32_t)0x00800000U)            /*!< Receiver Time-Out enable */
#define  USART_CR2_ADD                       ((uint32_t)0xFF000000U)            /*!< Address of the USART node */

/******************  Bit definition for USART_CR3 register  *******************/
#define  USART_CR3_EIE                       ((uint32_t)0x00000001U)            /*!< Error Interrupt Enable */
#define  USART_CR3_IREN                      ((uint32_t)0x00000002U)            /*!< IrDA mode Enable */
#define  USART_CR3_IRLP                      ((uint32_t)0x00000004U)            /*!< IrDA Low-Power */
#define  USART_CR3_HDSEL                     ((uint32_t)0x00000008U)            /*!< Half-Duplex Selection */
#define  USART_CR3_NACK                      ((uint32_t)0x00000010U)            /*!< SmartCard NACK enable */
#define  USART_CR3_SCEN                      ((uint32_t)0x00000020U)            /*!< SmartCard mode enable */
#define  USART_CR3_DMAR                      ((uint32_t)0x00000040U)            /*!< DMA Enable Receiver */
#define  USART_CR3_DMAT                      ((uint32_t)0x00000080U)            /*!< DMA Enable Transmitter */
#define  USART_CR3_RTSE                      ((uint32_t)0x00000100U)            /*!< RTS Enable */
#define  USART_CR3_CTSE                      ((uint32_t)0x00000200U)            /*!< CTS Enable */
#define  USART_CR3_CTSIE                     ((uint32_t)0x00000400U)            /*!< CTS Interrupt Enable */
#define  USART_CR3_ONEBIT                    ((uint32_t)0x00000800U)            /*!< One sample bit method enable */
#define  USART_CR3_OVRDIS                    ((uint32_t)0x00001000U)            /*!< Overrun Disable */
#define  USART_CR3_DDRE                      ((uint32_t)0x00002000U)            /*!< DMA Disable on Reception Error */
#define  USART_CR3_DEM                       ((uint32_t)0x00004000U)            /*!< Driver Enable Mode */
#define  USART_CR3_DEP                       ((uint32_t)0x00008000U)            /*!< Driver Enable Polarity Selection */
#define  USART_CR3_SCARCNT                   ((uint32_t)0x000E0000U)            /*!< SCARCNT[2:0] bits (SmartCard Auto-Retry Count) */
#define  USART_CR3_SCARCNT_0                 ((uint32_t)0x00020000U)            /*!< Bit 0 */
#define  USART_CR3_SCARCNT_1                 ((uint32_t)0x00040000U)            /*!< Bit 1 */
#define  USART_CR3_SCARCNT_2                 ((uint32_t)0x00080000U)            /*!< Bit 2 */
#define  USART_CR3_WUS                       ((uint32_t)0x00300000U)            /*!< WUS[1:0] bits (Wake UP Interrupt Flag Selection) */
#define  USART_CR3_WUS_0                     ((uint32_t)0x00100000U)            /*!< Bit 0 */
#define  USART_CR3_WUS_1                     ((uint32_t)0x00200000U)            /*!< Bit 1 */
#define  USART_CR3_WUFIE                     ((uint32_t)0x00400000U)            /*!< Wake Up Interrupt Enable */
#define  USART_CR3_TCBGTIE                   ((uint32_t)0x01000000U)            /*!< Transmission Complete Before Guard Time Interrupt Enable */

/******************  Bit definition for USART_BRR register  *******************/
#define  USART_BRR_DIV_FRACTION              ((uint16_t)0x000FU)                /*!< Fraction of USARTDIV */
#define  USART_BRR_DIV_MANTISSA              ((uint16_t)0xFFF0U)                /*!< Mantissa of USARTDIV */

/******************  Bit definition for USART_GTPR register  ******************/
#define  USART_GTPR_PSC                      ((uint32_t)0x000000FFU)            /*!< PSC[7:0] bits (Prescaler value) */
#define  USART_GTPR_GT                       ((uint32_t)0x0000FF00U)            /*!< GT[7:0] bits (Guard time value) */


/*******************  Bit definition for USART_RTOR register  *****************/
#define  USART_RTOR_RTO                      ((uint32_t)0x00FFFFFFU)            /*!< Receiver Time Out Value */
#define  USART_RTOR_BLEN                     ((uint32_t)0xFF000000U)            /*!< Block Length */

/*******************  Bit definition for USART_RQR register  ******************/
#define  USART_RQR_ABRRQ                     ((uint16_t)0x0001U)                /*!< Auto-Baud Rate Request */
#define  USART_RQR_SBKRQ                     ((uint16_t)0x0002U)                /*!< Send Break Request */
#define  USART_RQR_MMRQ                      ((uint16_t)0x0004U)                /*!< Mute Mode Request */
#define  USART_RQR_RXFRQ                     ((uint16_t)0x0008U)                /*!< Receive Data flush Request */
#define  USART_RQR_TXFRQ                     ((uint16_t)0x0010U)                /*!< Transmit data flush Request */

/*******************  Bit definition for USART_ISR register  ******************/
#define  USART_ISR_PE                        ((uint32_t)0x00000001U)            /*!< Parity Error */
#define  USART_ISR_FE                        ((uint32_t)0x00000002U)            /*!< Framing Error */
#define  USART_ISR_NE                        ((uint32_t)0x00000004U)            /*!< Noise detected Flag */
#define  USART_ISR_ORE                       ((uint32_t)0x00000008U)            /*!< OverRun Error */
#define  USART_ISR_IDLE                      ((uint32_t)0x00000010U)            /*!< IDLE line detected */
#define  USART_ISR_RXNE                      ((uint32_t)0x00000020U)            /*!< Read Data Register Not Empty */
#define  USART_ISR_TC                        ((uint32_t)0x00000040U)            /*!< Transmission Complete */
#define  USART_ISR_TXE                       ((uint32_t)0x00000080U)            /*!< Transmit Data Register Empty */
#define  USART_ISR_LBDF                      ((uint32_t)0x00000100U)            /*!< LIN Break Detection Flag */
#define  USART_ISR_CTSIF                     ((uint32_t)0x00000200U)            /*!< CTS interrupt flag */
#define  USART_ISR_CTS                       ((uint32_t)0x00000400U)            /*!< CTS flag */
#define  USART_ISR_RTOF                      ((uint32_t)0x00000800U)            /*!< Receiver Time Out */
#define  USART_ISR_EOBF                      ((uint32_t)0x00001000U)            /*!< End Of Block Flag */
#define  USART_ISR_ABRE                      ((uint32_t)0x00004000U)            /*!< Auto-Baud Rate Error */
#define  USART_ISR_ABRF                      ((uint32_t)0x00008000U)            /*!< Auto-Baud Rate Flag */
#define  USART_ISR_BUSY                      ((uint32_t)0x00010000U)            /*!< Busy Flag */
#define  USART_ISR_CMF                       ((uint32_t)0x00020000U)            /*!< Character Match Flag */
#define  USART_ISR_SBKF                      ((uint32_t)0x00040000U)            /*!< Send Break Flag */
#define  USART_ISR_RWU                       ((uint32_t)0x00080000U)            /*!< Receive Wake Up from mute mode Flag */
#define  USART_ISR_WUF                       ((uint32_t)0x00100000U)            /*!< Wake Up from stop mode Flag */
#define  USART_ISR_TEACK                     ((uint32_t)0x00200000U)            /*!< Transmit Enable Acknowledge Flag */
#define  USART_ISR_REACK                     ((uint32_t)0x00400000U)            /*!< Receive Enable Acknowledge Flag */
#define  USART_ISR_TCBGT                     ((uint32_t)0x02000000U)            /*!< Transmission Complete Before Guard Time Completion Flag */

/*******************  Bit definition for USART_ICR register  ******************/
#define  USART_ICR_PECF                      ((uint32_t)0x00000001U)            /*!< Parity Error Clear Flag */
#define  USART_ICR_FECF                      ((uint32_t)0x00000002U)            /*!< Framing Error Clear Flag */
#define  USART_ICR_NCF                       ((uint32_t)0x00000004U)            /*!< Noise detected Clear Flag */
#define  USART_ICR_ORECF                     ((uint32_t)0x00000008U)            /*!< OverRun Error Clear Flag */
#define  USART_ICR_IDLECF                    ((uint32_t)0x00000010U)            /*!< IDLE line detected Clear Flag */
#define  USART_ICR_TCCF                      ((uint32_t)0x00000040U)            /*!< Transmission Complete Clear Flag */
#define  USART_ICR_TCBGTCF                   ((uint32_t)0x00000080U)            /*!< Transmission Complete Before Guard Time Clear Flag */
#define  USART_ICR_LBDCF                     ((uint32_t)0x00000100U)            /*!< LIN Break Detection Clear Flag */
#define  USART_ICR_CTSCF                     ((uint32_t)0x00000200U)            /*!< CTS Interrupt Clear Flag */
#define  USART_ICR_RTOCF                     ((uint32_t)0x00000800U)            /*!< Receiver Time Out Clear Flag */
#define  USART_ICR_EOBCF                     ((uint32_t)0x00001000U)            /*!< End Of Block Clear Flag */
#define  USART_ICR_CMCF                      ((uint32_t)0x00020000U)            /*!< Character Match Clear Flag */
#define  USART_ICR_WUCF                      ((uint32_t)0x00100000U)            /*!< Wake Up from stop mode Clear Flag */

/*******************  Bit definition for USART_RDR register  ******************/
#define  USART_RDR_RDR                       ((uint16_t)0x01FFU)                /*!< RDR[8:0] bits (Receive Data value) */

/*******************  Bit definition for USART_TDR register  ******************/
#define  USART_TDR_TDR                       ((uint16_t)0x01FFU)                /*!< TDR[8:0] bits (Transmit Data value) */

/******************************************************************************/
/*                                                                            */
/*           Single Wire Protocol Master Interface (SWPMI)                    */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for SWPMI_CR register   ********************/
#define  SWPMI_CR_RXDMA                      ((uint32_t)0x00000001U)        /*!<Reception DMA enable                                 */
#define  SWPMI_CR_TXDMA                      ((uint32_t)0x00000002U)        /*!<Transmission DMA enable                              */
#define  SWPMI_CR_RXMODE                     ((uint32_t)0x00000004U)        /*!<Reception buffering mode                             */
#define  SWPMI_CR_TXMODE                     ((uint32_t)0x00000008U)        /*!<Transmission buffering mode                          */
#define  SWPMI_CR_LPBK                       ((uint32_t)0x00000010U)        /*!<Loopback mode enable                                 */
#define  SWPMI_CR_SWPACT                     ((uint32_t)0x00000020U)        /*!<Single wire protocol master interface activate       */
#define  SWPMI_CR_DEACT                      ((uint32_t)0x00000400U)        /*!<Single wire protocol master interface deactivate     */

/*******************  Bit definition for SWPMI_BRR register  ********************/
#define  SWPMI_BRR_BR                        ((uint32_t)0x0000003FU)        /*!<BR[5:0] bits (Bitrate prescaler) */

/*******************  Bit definition for SWPMI_ISR register  ********************/
#define  SWPMI_ISR_RXBFF                     ((uint32_t)0x00000001U)        /*!<Receive buffer full flag        */
#define  SWPMI_ISR_TXBEF                     ((uint32_t)0x00000002U)        /*!<Transmit buffer empty flag      */
#define  SWPMI_ISR_RXBERF                    ((uint32_t)0x00000004U)        /*!<Receive CRC error flag          */
#define  SWPMI_ISR_RXOVRF                    ((uint32_t)0x00000008U)        /*!<Receive overrun error flag      */
#define  SWPMI_ISR_TXUNRF                    ((uint32_t)0x00000010U)        /*!<Transmit underrun error flag    */
#define  SWPMI_ISR_RXNE                      ((uint32_t)0x00000020U)        /*!<Receive data register not empty */
#define  SWPMI_ISR_TXE                       ((uint32_t)0x00000040U)        /*!<Transmit data register empty    */
#define  SWPMI_ISR_TCF                       ((uint32_t)0x00000080U)        /*!<Transfer complete flag          */
#define  SWPMI_ISR_SRF                       ((uint32_t)0x00000100U)        /*!<Slave resume flag               */
#define  SWPMI_ISR_SUSP                      ((uint32_t)0x00000200U)        /*!<SUSPEND flag                    */
#define  SWPMI_ISR_DEACTF                    ((uint32_t)0x00000400U)        /*!<DEACTIVATED flag                */

/*******************  Bit definition for SWPMI_ICR register  ********************/
#define  SWPMI_ICR_CRXBFF                    ((uint32_t)0x00000001U)        /*!<Clear receive buffer full flag       */
#define  SWPMI_ICR_CTXBEF                    ((uint32_t)0x00000002U)        /*!<Clear transmit buffer empty flag     */
#define  SWPMI_ICR_CRXBERF                   ((uint32_t)0x00000004U)        /*!<Clear receive CRC error flag         */
#define  SWPMI_ICR_CRXOVRF                   ((uint32_t)0x00000008U)        /*!<Clear receive overrun error flag     */
#define  SWPMI_ICR_CTXUNRF                   ((uint32_t)0x00000010U)        /*!<Clear transmit underrun error flag   */
#define  SWPMI_ICR_CTCF                      ((uint32_t)0x00000080U)        /*!<Clear transfer complete flag         */
#define  SWPMI_ICR_CSRF                      ((uint32_t)0x00000100U)        /*!<Clear slave resume flag              */

/*******************  Bit definition for SWPMI_IER register  ********************/
#define  SWPMI_IER_SRIE                      ((uint32_t)0x00000100U)        /*!<Slave resume interrupt enable               */
#define  SWPMI_IER_TCIE                      ((uint32_t)0x00000080U)        /*!<Transmit complete interrupt enable          */
#define  SWPMI_IER_TIE                       ((uint32_t)0x00000040U)        /*!<Transmit interrupt enable                   */
#define  SWPMI_IER_RIE                       ((uint32_t)0x00000020U)        /*!<Receive interrupt enable                    */
#define  SWPMI_IER_TXUNRIE                   ((uint32_t)0x00000010U)        /*!<Transmit underrun error interrupt enable    */
#define  SWPMI_IER_RXOVRIE                   ((uint32_t)0x00000008U)        /*!<Receive overrun error interrupt enable      */
#define  SWPMI_IER_RXBERIE                   ((uint32_t)0x00000004U)        /*!<Receive CRC error interrupt enable          */
#define  SWPMI_IER_TXBEIE                    ((uint32_t)0x00000002U)        /*!<Transmit buffer empty interrupt enable      */
#define  SWPMI_IER_RXBFIE                    ((uint32_t)0x00000001U)        /*!<Receive buffer full interrupt enable        */

/*******************  Bit definition for SWPMI_RFL register  ********************/
#define  SWPMI_RFL_RFL                       ((uint32_t)0x0000001FU)        /*!<RFL[4:0] bits (Receive Frame length) */
#define  SWPMI_RFL_RFL_0_1                   ((uint32_t)0x00000003U)        /*!<RFL[1:0] bits (number of relevant bytes for the last SWPMI_RDR register read.) */

/*******************  Bit definition for SWPMI_TDR register  ********************/
#define  SWPMI_TDR_TD                        ((uint32_t)0xFFFFFFFFU)        /*!<Transmit Data Register         */

/*******************  Bit definition for SWPMI_RDR register  ********************/
#define  SWPMI_RDR_RD                        ((uint32_t)0xFFFFFFFFU)        /*!<Receive Data Register          */

/*******************  Bit definition for SWPMI_OR register  ********************/
#define  SWPMI_OR_TBYP                       ((uint32_t)0x00000001U)        /*!<SWP Transceiver Bypass */
#define  SWPMI_OR_CLASS                      ((uint32_t)0x00000002U)        /*!<SWP Voltage Class selection */

/******************************************************************************/
/*                                                                            */
/*                                 VREFBUF                                    */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for VREFBUF_CSR register  ****************/
#define  VREFBUF_CSR_ENVR                    ((uint32_t)0x00000001U)        /*!<Voltage reference buffer enable */
#define  VREFBUF_CSR_HIZ                     ((uint32_t)0x00000002U)        /*!<High impedance mode             */
#define  VREFBUF_CSR_VRS                     ((uint32_t)0x00000004U)        /*!<Voltage reference scale         */
#define  VREFBUF_CSR_VRR                     ((uint32_t)0x00000008U)        /*!<Voltage reference buffer ready  */

/*******************  Bit definition for VREFBUF_CCR register  ******************/
#define  VREFBUF_CCR_TRIM                    ((uint32_t)0x0000003FU)        /*!<TRIM[5:0] bits (Trimming code)  */

/******************************************************************************/
/*                                                                            */
/*                            Window WATCHDOG                                 */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for WWDG_CR register  ********************/
#define  WWDG_CR_T                           ((uint32_t)0x0000007FU)        /*!<T[6:0] bits (7-Bit counter (MSB to LSB)) */
#define  WWDG_CR_T_0                         ((uint32_t)0x00000001U)        /*!<Bit 0 */
#define  WWDG_CR_T_1                         ((uint32_t)0x00000002U)        /*!<Bit 1 */
#define  WWDG_CR_T_2                         ((uint32_t)0x00000004U)        /*!<Bit 2 */
#define  WWDG_CR_T_3                         ((uint32_t)0x00000008U)        /*!<Bit 3 */
#define  WWDG_CR_T_4                         ((uint32_t)0x00000010U)        /*!<Bit 4 */
#define  WWDG_CR_T_5                         ((uint32_t)0x00000020U)        /*!<Bit 5 */
#define  WWDG_CR_T_6                         ((uint32_t)0x00000040U)        /*!<Bit 6 */

#define  WWDG_CR_WDGA                        ((uint32_t)0x00000080U)        /*!<Activation bit */

/*******************  Bit definition for WWDG_CFR register  *******************/
#define  WWDG_CFR_W                          ((uint32_t)0x0000007FU)        /*!<W[6:0] bits (7-bit window value) */
#define  WWDG_CFR_W_0                        ((uint32_t)0x00000001U)        /*!<Bit 0 */
#define  WWDG_CFR_W_1                        ((uint32_t)0x00000002U)        /*!<Bit 1 */
#define  WWDG_CFR_W_2                        ((uint32_t)0x00000004U)        /*!<Bit 2 */
#define  WWDG_CFR_W_3                        ((uint32_t)0x00000008U)        /*!<Bit 3 */
#define  WWDG_CFR_W_4                        ((uint32_t)0x00000010U)        /*!<Bit 4 */
#define  WWDG_CFR_W_5                        ((uint32_t)0x00000020U)        /*!<Bit 5 */
#define  WWDG_CFR_W_6                        ((uint32_t)0x00000040U)        /*!<Bit 6 */

#define  WWDG_CFR_WDGTB                      ((uint32_t)0x00000180U)        /*!<WDGTB[1:0] bits (Timer Base) */
#define  WWDG_CFR_WDGTB_0                    ((uint32_t)0x00000080U)        /*!<Bit 0 */
#define  WWDG_CFR_WDGTB_1                    ((uint32_t)0x00000100U)        /*!<Bit 1 */

#define  WWDG_CFR_EWI                        ((uint32_t)0x00000200U)        /*!<Early Wakeup Interrupt */

/*******************  Bit definition for WWDG_SR register  ********************/
#define  WWDG_SR_EWIF                        ((uint32_t)0x00000001U)        /*!<Early Wakeup Interrupt Flag */


/******************************************************************************/
/*                                                                            */
/*                                 Debug MCU                                  */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for DBGMCU_IDCODE register  *************/
#define  DBGMCU_IDCODE_DEV_ID                ((uint32_t)0x00000FFFU)
#define  DBGMCU_IDCODE_REV_ID                ((uint32_t)0xFFFF0000U)

/********************  Bit definition for DBGMCU_CR register  *****************/
#define  DBGMCU_CR_DBG_SLEEP                 ((uint32_t)0x00000001U)
#define  DBGMCU_CR_DBG_STOP                  ((uint32_t)0x00000002U)
#define  DBGMCU_CR_DBG_STANDBY               ((uint32_t)0x00000004U)
#define  DBGMCU_CR_TRACE_IOEN                ((uint32_t)0x00000020U)

#define  DBGMCU_CR_TRACE_MODE                ((uint32_t)0x000000C0U)
#define  DBGMCU_CR_TRACE_MODE_0              ((uint32_t)0x00000040U)/*!<Bit 0 */
#define  DBGMCU_CR_TRACE_MODE_1              ((uint32_t)0x00000080U)/*!<Bit 1 */

/********************  Bit definition for DBGMCU_APB1FZR1 register  ***********/
#define  DBGMCU_APB1FZR1_DBG_TIM2_STOP       ((uint32_t)0x00000001U)
#define  DBGMCU_APB1FZR1_DBG_TIM6_STOP       ((uint32_t)0x00000010U)
#define  DBGMCU_APB1FZR1_DBG_TIM7_STOP       ((uint32_t)0x00000020U)
#define  DBGMCU_APB1FZR1_DBG_RTC_STOP        ((uint32_t)0x00000400U)
#define  DBGMCU_APB1FZR1_DBG_WWDG_STOP       ((uint32_t)0x00000800U)
#define  DBGMCU_APB1FZR1_DBG_IWDG_STOP       ((uint32_t)0x00001000U)
#define  DBGMCU_APB1FZR1_DBG_I2C1_STOP       ((uint32_t)0x00200000U)
#define  DBGMCU_APB1FZR1_DBG_I2C2_STOP       ((uint32_t)0x00400000U)
#define  DBGMCU_APB1FZR1_DBG_I2C3_STOP       ((uint32_t)0x00800000U)
#define  DBGMCU_APB1FZR1_DBG_CAN_STOP        ((uint32_t)0x02000000U)
#define  DBGMCU_APB1FZR1_DBG_LPTIM1_STOP     ((uint32_t)0x80000000U)

/********************  Bit definition for DBGMCU_APB1FZR2 register  **********/
#define  DBGMCU_APB1FZR2_DBG_LPTIM2_STOP     ((uint32_t)0x00000020U)

/********************  Bit definition for DBGMCU_APB2FZ register  ************/
#define  DBGMCU_APB2FZ_DBG_TIM1_STOP         ((uint32_t)0x00000800U)
#define  DBGMCU_APB2FZ_DBG_TIM15_STOP        ((uint32_t)0x00010000U)
#define  DBGMCU_APB2FZ_DBG_TIM16_STOP        ((uint32_t)0x00020000U)

/******************************************************************************/
/*                                                                            */
/*                         USB Device FS Endpoint registers                   */
/*                                                                            */
/******************************************************************************/
#define USB_EP0R                             USB_BASE                    /*!< endpoint 0 register address */
#define USB_EP1R                             (USB_BASE + 0x0x00000004)   /*!< endpoint 1 register address */
#define USB_EP2R                             (USB_BASE + 0x0x00000008)   /*!< endpoint 2 register address */
#define USB_EP3R                             (USB_BASE + 0x0x0000000C)   /*!< endpoint 3 register address */
#define USB_EP4R                             (USB_BASE + 0x0x00000010)   /*!< endpoint 4 register address */
#define USB_EP5R                             (USB_BASE + 0x0x00000014)   /*!< endpoint 5 register address */
#define USB_EP6R                             (USB_BASE + 0x0x00000018)   /*!< endpoint 6 register address */
#define USB_EP7R                             (USB_BASE + 0x0x0000001C)   /*!< endpoint 7 register address */

/* bit positions */ 
#define USB_EP_CTR_RX                        ((uint16_t)0x8000U)          /*!<  EndPoint Correct TRansfer RX */
#define USB_EP_DTOG_RX                       ((uint16_t)0x4000U)          /*!<  EndPoint Data TOGGLE RX */
#define USB_EPRX_STAT                        ((uint16_t)0x3000U)          /*!<  EndPoint RX STATus bit field */
#define USB_EP_SETUP                         ((uint16_t)0x0800U)          /*!<  EndPoint SETUP */
#define USB_EP_T_FIELD                       ((uint16_t)0x0600U)          /*!<  EndPoint TYPE */
#define USB_EP_KIND                          ((uint16_t)0x0100U)          /*!<  EndPoint KIND */
#define USB_EP_CTR_TX                        ((uint16_t)0x0080U)          /*!<  EndPoint Correct TRansfer TX */
#define USB_EP_DTOG_TX                       ((uint16_t)0x0040U)          /*!<  EndPoint Data TOGGLE TX */
#define USB_EPTX_STAT                        ((uint16_t)0x0030U)          /*!<  EndPoint TX STATus bit field */
#define USB_EPADDR_FIELD                     ((uint16_t)0x000FU)          /*!<  EndPoint ADDRess FIELD */

/* EndPoint REGister MASK (no toggle fields) */
#define USB_EPREG_MASK     (USB_EP_CTR_RX|USB_EP_SETUP|USB_EP_T_FIELD|USB_EP_KIND|USB_EP_CTR_TX|USB_EPADDR_FIELD)
                                                                         /*!< EP_TYPE[1:0] EndPoint TYPE */
#define USB_EP_TYPE_MASK                     ((uint16_t)0x0600U)          /*!< EndPoint TYPE Mask */
#define USB_EP_BULK                          ((uint16_t)0x0000U)          /*!< EndPoint BULK */
#define USB_EP_CONTROL                       ((uint16_t)0x0200U)          /*!< EndPoint CONTROL */
#define USB_EP_ISOCHRONOUS                   ((uint16_t)0x0400U)          /*!< EndPoint ISOCHRONOUS */
#define USB_EP_INTERRUPT                     ((uint16_t)0x0600U)          /*!< EndPoint INTERRUPT */
#define USB_EP_T_MASK                        ((uint16_t) ~USB_EP_T_FIELD & USB_EPREG_MASK)
                                                                 
#define USB_EPKIND_MASK                      ((uint16_t)~USB_EP_KIND & USB_EPREG_MASK) /*!< EP_KIND EndPoint KIND */
                                                                         /*!< STAT_TX[1:0] STATus for TX transfer */
#define USB_EP_TX_DIS                        ((uint16_t)0x0000U)          /*!< EndPoint TX DISabled */
#define USB_EP_TX_STALL                      ((uint16_t)0x0010U)          /*!< EndPoint TX STALLed */
#define USB_EP_TX_NAK                        ((uint16_t)0x0020U)          /*!< EndPoint TX NAKed */
#define USB_EP_TX_VALID                      ((uint16_t)0x0030U)          /*!< EndPoint TX VALID */
#define USB_EPTX_DTOG1                       ((uint16_t)0x0010U)          /*!< EndPoint TX Data TOGgle bit1 */
#define USB_EPTX_DTOG2                       ((uint16_t)0x0020U)          /*!< EndPoint TX Data TOGgle bit2 */
#define USB_EPTX_DTOGMASK  (USB_EPTX_STAT|USB_EPREG_MASK)
                                                                         /*!< STAT_RX[1:0] STATus for RX transfer */
#define USB_EP_RX_DIS                        ((uint16_t)0x0000U)          /*!< EndPoint RX DISabled */
#define USB_EP_RX_STALL                      ((uint16_t)0x1000U)          /*!< EndPoint RX STALLed */
#define USB_EP_RX_NAK                        ((uint16_t)0x2000U)          /*!< EndPoint RX NAKed */
#define USB_EP_RX_VALID                      ((uint16_t)0x3000U)          /*!< EndPoint RX VALID */
#define USB_EPRX_DTOG1                       ((uint16_t)0x1000U)          /*!< EndPoint RX Data TOGgle bit1 */
#define USB_EPRX_DTOG2                       ((uint16_t)0x2000U)          /*!< EndPoint RX Data TOGgle bit1 */
#define USB_EPRX_DTOGMASK  (USB_EPRX_STAT|USB_EPREG_MASK)

/******************************************************************************/
/*                                                                            */
/*                         USB Device FS General registers                    */
/*                                                                            */
/******************************************************************************/
#define USB_CNTR                             (USB_BASE + 0x00000040U)     /*!< Control register */
#define USB_ISTR                             (USB_BASE + 0x00000044U)     /*!< Interrupt status register */
#define USB_FNR                              (USB_BASE + 0x00000048U)     /*!< Frame number register */
#define USB_DADDR                            (USB_BASE + 0x0000004CU)     /*!< Device address register */
#define USB_BTABLE                           (USB_BASE + 0x00000050U)     /*!< Buffer Table address register */
#define USB_LPMCSR                           (USB_BASE + 0x00000054U)     /*!< LPM Control and Status register */
#define USB_BCDR                             (USB_BASE + 0x00000058U)     /*!< Battery Charging detector register*/

/******************  Bits definition for USB_CNTR register  *******************/
#define USB_CNTR_CTRM                        ((uint16_t)0x8000U)          /*!< Correct TRansfer Mask */
#define USB_CNTR_PMAOVRM                     ((uint16_t)0x4000U)          /*!< DMA OVeR/underrun Mask */
#define USB_CNTR_ERRM                        ((uint16_t)0x2000U)          /*!< ERRor Mask */
#define USB_CNTR_WKUPM                       ((uint16_t)0x1000U)          /*!< WaKe UP Mask */
#define USB_CNTR_SUSPM                       ((uint16_t)0x0800U)          /*!< SUSPend Mask */
#define USB_CNTR_RESETM                      ((uint16_t)0x0400U)          /*!< RESET Mask   */
#define USB_CNTR_SOFM                        ((uint16_t)0x0200U)          /*!< Start Of Frame Mask */
#define USB_CNTR_ESOFM                       ((uint16_t)0x0100U)          /*!< Expected Start Of Frame Mask */
#define USB_CNTR_L1REQM                      ((uint16_t)0x0080U)          /*!< LPM L1 state request interrupt mask */
#define USB_CNTR_L1RESUME                    ((uint16_t)0x0020U)          /*!< LPM L1 Resume request */
#define USB_CNTR_RESUME                      ((uint16_t)0x0010U)          /*!< RESUME request */
#define USB_CNTR_FSUSP                       ((uint16_t)0x0008U)          /*!< Force SUSPend */
#define USB_CNTR_LPMODE                      ((uint16_t)0x0004U)          /*!< Low-power MODE */
#define USB_CNTR_PDWN                        ((uint16_t)0x0002U)          /*!< Power DoWN */
#define USB_CNTR_FRES                        ((uint16_t)0x0001U)          /*!< Force USB RESet */

/******************  Bits definition for USB_ISTR register  *******************/
#define USB_ISTR_EP_ID                       ((uint16_t)0x000FU)          /*!< EndPoint IDentifier (read-only bit)  */
#define USB_ISTR_DIR                         ((uint16_t)0x0010U)          /*!< DIRection of transaction (read-only bit)  */
#define USB_ISTR_L1REQ                       ((uint16_t)0x0080U)          /*!< LPM L1 state request  */
#define USB_ISTR_ESOF                        ((uint16_t)0x0100U)          /*!< Expected Start Of Frame (clear-only bit) */
#define USB_ISTR_SOF                         ((uint16_t)0x0200U)          /*!< Start Of Frame (clear-only bit) */
#define USB_ISTR_RESET                       ((uint16_t)0x0400U)          /*!< RESET (clear-only bit) */
#define USB_ISTR_SUSP                        ((uint16_t)0x0800U)          /*!< SUSPend (clear-only bit) */
#define USB_ISTR_WKUP                        ((uint16_t)0x1000U)          /*!< WaKe UP (clear-only bit) */
#define USB_ISTR_ERR                         ((uint16_t)0x2000U)          /*!< ERRor (clear-only bit) */
#define USB_ISTR_PMAOVR                      ((uint16_t)0x4000U)          /*!< DMA OVeR/underrun (clear-only bit) */
#define USB_ISTR_CTR                         ((uint16_t)0x8000U)          /*!< Correct TRansfer (clear-only bit) */

#define USB_CLR_L1REQ                        (~USB_ISTR_L1REQ)           /*!< clear LPM L1  bit */
#define USB_CLR_ESOF                         (~USB_ISTR_ESOF)            /*!< clear Expected Start Of Frame bit */
#define USB_CLR_SOF                          (~USB_ISTR_SOF)             /*!< clear Start Of Frame bit */
#define USB_CLR_RESET                        (~USB_ISTR_RESET)           /*!< clear RESET bit */
#define USB_CLR_SUSP                         (~USB_ISTR_SUSP)            /*!< clear SUSPend bit */
#define USB_CLR_WKUP                         (~USB_ISTR_WKUP)            /*!< clear WaKe UP bit */
#define USB_CLR_ERR                          (~USB_ISTR_ERR)             /*!< clear ERRor bit */
#define USB_CLR_PMAOVR                       (~USB_ISTR_PMAOVR)          /*!< clear DMA OVeR/underrun bit*/
#define USB_CLR_CTR                          (~USB_ISTR_CTR)             /*!< clear Correct TRansfer bit */

/******************  Bits definition for USB_FNR register  ********************/
#define USB_FNR_FN                           ((uint16_t)0x07FFU)          /*!< Frame Number */
#define USB_FNR_LSOF                         ((uint16_t)0x1800U)          /*!< Lost SOF */
#define USB_FNR_LCK                          ((uint16_t)0x2000U)          /*!< LoCKed */
#define USB_FNR_RXDM                         ((uint16_t)0x4000U)          /*!< status of D- data line */
#define USB_FNR_RXDP                         ((uint16_t)0x8000U)          /*!< status of D+ data line */

/******************  Bits definition for USB_DADDR register    ****************/
#define USB_DADDR_ADD                        ((uint8_t)0x7FU)             /*!< ADD[6:0] bits (Device Address) */
#define USB_DADDR_ADD0                       ((uint8_t)0x01U)             /*!< Bit 0 */
#define USB_DADDR_ADD1                       ((uint8_t)0x02U)             /*!< Bit 1 */
#define USB_DADDR_ADD2                       ((uint8_t)0x04U)             /*!< Bit 2 */
#define USB_DADDR_ADD3                       ((uint8_t)0x08U)             /*!< Bit 3 */
#define USB_DADDR_ADD4                       ((uint8_t)0x10U)             /*!< Bit 4 */
#define USB_DADDR_ADD5                       ((uint8_t)0x20U)             /*!< Bit 5 */
#define USB_DADDR_ADD6                       ((uint8_t)0x40U)             /*!< Bit 6 */

#define USB_DADDR_EF                         ((uint8_t)0x80U)             /*!< Enable Function */

/******************  Bit definition for USB_BTABLE register  ******************/
#define USB_BTABLE_BTABLE                    ((uint16_t)0xFFF8U)          /*!< Buffer Table */

/******************  Bits definition for USB_BCDR register  *******************/
#define USB_BCDR_BCDEN                       ((uint16_t)0x0001U)          /*!< Battery charging detector (BCD) enable */
#define USB_BCDR_DCDEN                       ((uint16_t)0x0002U)          /*!< Data contact detection (DCD) mode enable */
#define USB_BCDR_PDEN                        ((uint16_t)0x0004U)          /*!< Primary detection (PD) mode enable */  
#define USB_BCDR_SDEN                        ((uint16_t)0x0008U)          /*!< Secondary detection (SD) mode enable */ 
#define USB_BCDR_DCDET                       ((uint16_t)0x0010U)          /*!< Data contact detection (DCD) status */ 
#define USB_BCDR_PDET                        ((uint16_t)0x0020U)          /*!< Primary detection (PD) status */ 
#define USB_BCDR_SDET                        ((uint16_t)0x0040U)          /*!< Secondary detection (SD) status */  
#define USB_BCDR_PS2DET                      ((uint16_t)0x0080U)          /*!< PS2 port or proprietary charger detected */  
#define USB_BCDR_DPPU                        ((uint16_t)0x8000U)          /*!< DP Pull-up Enable */  

/*******************  Bit definition for LPMCSR register  *********************/
#define USB_LPMCSR_LMPEN                     ((uint16_t)0x0001U)          /*!< LPM support enable  */
#define USB_LPMCSR_LPMACK                    ((uint16_t)0x0002U)          /*!< LPM Token acknowledge enable*/
#define USB_LPMCSR_REMWAKE                   ((uint16_t)0x0008U)          /*!< bRemoteWake value received with last ACKed LPM Token */ 
#define USB_LPMCSR_BESL                      ((uint16_t)0x00F0U)          /*!< BESL value received with last ACKed LPM Token  */ 

/*!< Buffer descriptor table */
/*****************  Bit definition for USB_ADDR0_TX register  *****************/
#define USB_ADDR0_TX_ADDR0_TX               ((uint32_t)0x0000FFFEU)        /*!< Transmission Buffer Address 0 */

/*****************  Bit definition for USB_ADDR1_TX register  *****************/
#define USB_ADDR1_TX_ADDR1_TX               ((uint32_t)0x0000FFFEU)        /*!< Transmission Buffer Address 1 */

/*****************  Bit definition for USB_ADDR2_TX register  *****************/
#define USB_ADDR2_TX_ADDR2_TX               ((uint32_t)0x0000FFFEU)        /*!< Transmission Buffer Address 2 */

/*****************  Bit definition for USB_ADDR3_TX register  *****************/
#define USB_ADDR3_TX_ADDR3_TX               ((uint32_t)0x0000FFFEU)        /*!< Transmission Buffer Address 3 */

/*****************  Bit definition for USB_ADDR4_TX register  *****************/
#define USB_ADDR4_TX_ADDR4_TX               ((uint32_t)0x0000FFFEU)        /*!< Transmission Buffer Address 4 */

/*****************  Bit definition for USB_ADDR5_TX register  *****************/
#define USB_ADDR5_TX_ADDR5_TX               ((uint32_t)0x0000FFFEU)        /*!< Transmission Buffer Address 5 */

/*****************  Bit definition for USB_ADDR6_TX register  *****************/
#define USB_ADDR6_TX_ADDR6_TX               ((uint32_t)0x0000FFFEU)        /*!< Transmission Buffer Address 6 */

/*****************  Bit definition for USB_ADDR7_TX register  *****************/
#define USB_ADDR7_TX_ADDR7_TX               ((uint32_t)0x0000FFFEU)        /*!< Transmission Buffer Address 7 */

/*----------------------------------------------------------------------------*/

/*****************  Bit definition for USB_COUNT0_TX register  ****************/
#define USB_COUNT0_TX_COUNT0_TX             ((uint32_t)0x000003FFU)        /*!< Transmission Byte Count 0 */

/*****************  Bit definition for USB_COUNT1_TX register  ****************/
#define USB_COUNT1_TX_COUNT1_TX             ((uint32_t)0x000003FFU)        /*!< Transmission Byte Count 1 */

/*****************  Bit definition for USB_COUNT2_TX register  ****************/
#define USB_COUNT2_TX_COUNT2_TX             ((uint32_t)0x000003FFU)        /*!< Transmission Byte Count 2 */

/*****************  Bit definition for USB_COUNT3_TX register  ****************/
#define USB_COUNT3_TX_COUNT3_TX             ((uint32_t)0x000003FFU)        /*!< Transmission Byte Count 3 */

/*****************  Bit definition for USB_COUNT4_TX register  ****************/
#define USB_COUNT4_TX_COUNT4_TX             ((uint32_t)0x000003FFU)        /*!< Transmission Byte Count 4 */

/*****************  Bit definition for USB_COUNT5_TX register  ****************/
#define USB_COUNT5_TX_COUNT5_TX             ((uint32_t)0x000003FFU)        /*!< Transmission Byte Count 5 */

/*****************  Bit definition for USB_COUNT6_TX register  ****************/
#define USB_COUNT6_TX_COUNT6_TX             ((uint32_t)0x000003FFU)        /*!< Transmission Byte Count 6 */

/*****************  Bit definition for USB_COUNT7_TX register  ****************/
#define USB_COUNT7_TX_COUNT7_TX             ((uint32_t)0x000003FFU)        /*!< Transmission Byte Count 7 */

/*----------------------------------------------------------------------------*/

/****************  Bit definition for USB_COUNT0_TX_0 register  ***************/
#define USB_COUNT0_TX_0_COUNT0_TX_0         ((uint32_t)0x000003FFU)        /*!< Transmission Byte Count 0 (low) */

/****************  Bit definition for USB_COUNT0_TX_1 register  ***************/
#define USB_COUNT0_TX_1_COUNT0_TX_1         ((uint32_t)0x03FF0000U)        /*!< Transmission Byte Count 0 (high) */

/****************  Bit definition for USB_COUNT1_TX_0 register  ***************/
#define USB_COUNT1_TX_0_COUNT1_TX_0         ((uint32_t)0x000003FFU)        /*!< Transmission Byte Count 1 (low) */

/****************  Bit definition for USB_COUNT1_TX_1 register  ***************/
#define USB_COUNT1_TX_1_COUNT1_TX_1         ((uint32_t)0x03FF0000U)        /*!< Transmission Byte Count 1 (high) */

/****************  Bit definition for USB_COUNT2_TX_0 register  ***************/
#define USB_COUNT2_TX_0_COUNT2_TX_0         ((uint32_t)0x000003FFU)        /*!< Transmission Byte Count 2 (low) */

/****************  Bit definition for USB_COUNT2_TX_1 register  ***************/
#define USB_COUNT2_TX_1_COUNT2_TX_1         ((uint32_t)0x03FF0000U)        /*!< Transmission Byte Count 2 (high) */

/****************  Bit definition for USB_COUNT3_TX_0 register  ***************/
#define USB_COUNT3_TX_0_COUNT3_TX_0         ((uint32_t)0x000003FFU)        /*!< Transmission Byte Count 3 (low) */

/****************  Bit definition for USB_COUNT3_TX_1 register  ***************/
#define USB_COUNT3_TX_1_COUNT3_TX_1         ((uint32_t)0x03FF0000U)        /*!< Transmission Byte Count 3 (high) */

/****************  Bit definition for USB_COUNT4_TX_0 register  ***************/
#define USB_COUNT4_TX_0_COUNT4_TX_0         ((uint32_t)0x000003FFU)        /*!< Transmission Byte Count 4 (low) */

/****************  Bit definition for USB_COUNT4_TX_1 register  ***************/
#define USB_COUNT4_TX_1_COUNT4_TX_1         ((uint32_t)0x03FF0000U)        /*!< Transmission Byte Count 4 (high) */

/****************  Bit definition for USB_COUNT5_TX_0 register  ***************/
#define USB_COUNT5_TX_0_COUNT5_TX_0         ((uint32_t)0x000003FFU)        /*!< Transmission Byte Count 5 (low) */

/****************  Bit definition for USB_COUNT5_TX_1 register  ***************/
#define USB_COUNT5_TX_1_COUNT5_TX_1         ((uint32_t)0x03FF0000U)        /*!< Transmission Byte Count 5 (high) */

/****************  Bit definition for USB_COUNT6_TX_0 register  ***************/
#define USB_COUNT6_TX_0_COUNT6_TX_0         ((uint32_t)0x000003FFU)        /*!< Transmission Byte Count 6 (low) */

/****************  Bit definition for USB_COUNT6_TX_1 register  ***************/
#define USB_COUNT6_TX_1_COUNT6_TX_1         ((uint32_t)0x03FF0000U)        /*!< Transmission Byte Count 6 (high) */

/****************  Bit definition for USB_COUNT7_TX_0 register  ***************/
#define USB_COUNT7_TX_0_COUNT7_TX_0         ((uint32_t)0x000003FFU)        /*!< Transmission Byte Count 7 (low) */

/****************  Bit definition for USB_COUNT7_TX_1 register  ***************/
#define USB_COUNT7_TX_1_COUNT7_TX_1         ((uint32_t)0x03FF0000U)        /*!< Transmission Byte Count 7 (high) */

/*----------------------------------------------------------------------------*/

/*****************  Bit definition for USB_ADDR0_RX register  *****************/
#define USB_ADDR0_RX_ADDR0_RX               ((uint32_t)0x0000FFFEU)        /*!< Reception Buffer Address 0 */

/*****************  Bit definition for USB_ADDR1_RX register  *****************/
#define USB_ADDR1_RX_ADDR1_RX               ((uint32_t)0x0000FFFEU)        /*!< Reception Buffer Address 1 */

/*****************  Bit definition for USB_ADDR2_RX register  *****************/
#define USB_ADDR2_RX_ADDR2_RX               ((uint32_t)0x0000FFFEU)        /*!< Reception Buffer Address 2 */

/*****************  Bit definition for USB_ADDR3_RX register  *****************/
#define USB_ADDR3_RX_ADDR3_RX               ((uint32_t)0x0000FFFEU)        /*!< Reception Buffer Address 3 */

/*****************  Bit definition for USB_ADDR4_RX register  *****************/
#define USB_ADDR4_RX_ADDR4_RX               ((uint32_t)0x0000FFFEU)        /*!< Reception Buffer Address 4 */

/*****************  Bit definition for USB_ADDR5_RX register  *****************/
#define USB_ADDR5_RX_ADDR5_RX               ((uint32_t)0x0000FFFEU)        /*!< Reception Buffer Address 5 */

/*****************  Bit definition for USB_ADDR6_RX register  *****************/
#define USB_ADDR6_RX_ADDR6_RX               ((uint32_t)0x0000FFFEU)        /*!< Reception Buffer Address 6 */

/*****************  Bit definition for USB_ADDR7_RX register  *****************/
#define USB_ADDR7_RX_ADDR7_RX               ((uint32_t)0x0000FFFEU)        /*!< Reception Buffer Address 7 */

/*----------------------------------------------------------------------------*/

/*****************  Bit definition for USB_COUNT0_RX register  ****************/
#define USB_COUNT0_RX_COUNT0_RX             ((uint32_t)0x000003FFU)        /*!< Reception Byte Count */

#define USB_COUNT0_RX_NUM_BLOCK             ((uint32_t)0x00007C00U)        /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define USB_COUNT0_RX_NUM_BLOCK_0           ((uint32_t)0x00000400U)        /*!< Bit 0 */
#define USB_COUNT0_RX_NUM_BLOCK_1           ((uint32_t)0x00000800U)        /*!< Bit 1 */
#define USB_COUNT0_RX_NUM_BLOCK_2           ((uint32_t)0x00001000U)        /*!< Bit 2 */
#define USB_COUNT0_RX_NUM_BLOCK_3           ((uint32_t)0x00002000U)        /*!< Bit 3 */
#define USB_COUNT0_RX_NUM_BLOCK_4           ((uint32_t)0x00004000U)        /*!< Bit 4 */

#define USB_COUNT0_RX_BLSIZE                ((uint32_t)0x00008000U)        /*!< BLock SIZE */

/*****************  Bit definition for USB_COUNT1_RX register  ****************/
#define USB_COUNT1_RX_COUNT1_RX             ((uint32_t)0x000003FFU)        /*!< Reception Byte Count */

#define USB_COUNT1_RX_NUM_BLOCK             ((uint32_t)0x00007C00U)        /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define USB_COUNT1_RX_NUM_BLOCK_0           ((uint32_t)0x00000400U)        /*!< Bit 0 */
#define USB_COUNT1_RX_NUM_BLOCK_1           ((uint32_t)0x00000800U)        /*!< Bit 1 */
#define USB_COUNT1_RX_NUM_BLOCK_2           ((uint32_t)0x00001000U)        /*!< Bit 2 */
#define USB_COUNT1_RX_NUM_BLOCK_3           ((uint32_t)0x00002000U)        /*!< Bit 3 */
#define USB_COUNT1_RX_NUM_BLOCK_4           ((uint32_t)0x00004000U)        /*!< Bit 4 */

#define USB_COUNT1_RX_BLSIZE                ((uint32_t)0x00008000U)        /*!< BLock SIZE */

/*****************  Bit definition for USB_COUNT2_RX register  ****************/
#define USB_COUNT2_RX_COUNT2_RX             ((uint32_t)0x000003FFU)        /*!< Reception Byte Count */

#define USB_COUNT2_RX_NUM_BLOCK             ((uint32_t)0x00007C00U)        /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define USB_COUNT2_RX_NUM_BLOCK_0           ((uint32_t)0x00000400U)        /*!< Bit 0 */
#define USB_COUNT2_RX_NUM_BLOCK_1           ((uint32_t)0x00000800U)        /*!< Bit 1 */
#define USB_COUNT2_RX_NUM_BLOCK_2           ((uint32_t)0x00001000U)        /*!< Bit 2 */
#define USB_COUNT2_RX_NUM_BLOCK_3           ((uint32_t)0x00002000U)        /*!< Bit 3 */
#define USB_COUNT2_RX_NUM_BLOCK_4           ((uint32_t)0x00004000U)        /*!< Bit 4 */

#define USB_COUNT2_RX_BLSIZE                ((uint32_t)0x00008000U)        /*!< BLock SIZE */

/*****************  Bit definition for USB_COUNT3_RX register  ****************/
#define USB_COUNT3_RX_COUNT3_RX             ((uint32_t)0x000003FFU)        /*!< Reception Byte Count */

#define USB_COUNT3_RX_NUM_BLOCK             ((uint32_t)0x00007C00U)        /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define USB_COUNT3_RX_NUM_BLOCK_0           ((uint32_t)0x00000400U)        /*!< Bit 0 */
#define USB_COUNT3_RX_NUM_BLOCK_1           ((uint32_t)0x00000800U)        /*!< Bit 1 */
#define USB_COUNT3_RX_NUM_BLOCK_2           ((uint32_t)0x00001000U)        /*!< Bit 2 */
#define USB_COUNT3_RX_NUM_BLOCK_3           ((uint32_t)0x00002000U)        /*!< Bit 3 */
#define USB_COUNT3_RX_NUM_BLOCK_4           ((uint32_t)0x00004000U)        /*!< Bit 4 */

#define USB_COUNT3_RX_BLSIZE                ((uint32_t)0x00008000U)        /*!< BLock SIZE */

/*****************  Bit definition for USB_COUNT4_RX register  ****************/
#define USB_COUNT4_RX_COUNT4_RX             ((uint32_t)0x000003FFU)        /*!< Reception Byte Count */

#define USB_COUNT4_RX_NUM_BLOCK             ((uint32_t)0x00007C00U)        /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define USB_COUNT4_RX_NUM_BLOCK_0           ((uint32_t)0x00000400U)        /*!< Bit 0 */
#define USB_COUNT4_RX_NUM_BLOCK_1           ((uint32_t)0x00000800U)        /*!< Bit 1 */
#define USB_COUNT4_RX_NUM_BLOCK_2           ((uint32_t)0x00001000U)        /*!< Bit 2 */
#define USB_COUNT4_RX_NUM_BLOCK_3           ((uint32_t)0x00002000U)        /*!< Bit 3 */
#define USB_COUNT4_RX_NUM_BLOCK_4           ((uint32_t)0x00004000U)        /*!< Bit 4 */

#define USB_COUNT4_RX_BLSIZE                ((uint32_t)0x00008000U)        /*!< BLock SIZE */

/*****************  Bit definition for USB_COUNT5_RX register  ****************/
#define USB_COUNT5_RX_COUNT5_RX             ((uint32_t)0x000003FFU)        /*!< Reception Byte Count */

#define USB_COUNT5_RX_NUM_BLOCK             ((uint32_t)0x00007C00U)        /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define USB_COUNT5_RX_NUM_BLOCK_0           ((uint32_t)0x00000400U)        /*!< Bit 0 */
#define USB_COUNT5_RX_NUM_BLOCK_1           ((uint32_t)0x00000800U)        /*!< Bit 1 */
#define USB_COUNT5_RX_NUM_BLOCK_2           ((uint32_t)0x00001000U)        /*!< Bit 2 */
#define USB_COUNT5_RX_NUM_BLOCK_3           ((uint32_t)0x00002000U)        /*!< Bit 3 */
#define USB_COUNT5_RX_NUM_BLOCK_4           ((uint32_t)0x00004000U)        /*!< Bit 4 */

#define USB_COUNT5_RX_BLSIZE                ((uint32_t)0x00008000U)        /*!< BLock SIZE */

/*****************  Bit definition for USB_COUNT6_RX register  ****************/
#define USB_COUNT6_RX_COUNT6_RX             ((uint32_t)0x000003FFU)        /*!< Reception Byte Count */

#define USB_COUNT6_RX_NUM_BLOCK             ((uint32_t)0x00007C00U)        /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define USB_COUNT6_RX_NUM_BLOCK_0           ((uint32_t)0x00000400U)        /*!< Bit 0 */
#define USB_COUNT6_RX_NUM_BLOCK_1           ((uint32_t)0x00000800U)        /*!< Bit 1 */
#define USB_COUNT6_RX_NUM_BLOCK_2           ((uint32_t)0x00001000U)        /*!< Bit 2 */
#define USB_COUNT6_RX_NUM_BLOCK_3           ((uint32_t)0x00002000U)        /*!< Bit 3 */
#define USB_COUNT6_RX_NUM_BLOCK_4           ((uint32_t)0x00004000U)        /*!< Bit 4 */

#define USB_COUNT6_RX_BLSIZE                ((uint32_t)0x00008000U)        /*!< BLock SIZE */

/*****************  Bit definition for USB_COUNT7_RX register  ****************/
#define USB_COUNT7_RX_COUNT7_RX             ((uint32_t)0x000003FFU)        /*!< Reception Byte Count */

#define USB_COUNT7_RX_NUM_BLOCK             ((uint32_t)0x00007C00U)        /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define USB_COUNT7_RX_NUM_BLOCK_0           ((uint32_t)0x00000400U)        /*!< Bit 0 */
#define USB_COUNT7_RX_NUM_BLOCK_1           ((uint32_t)0x00000800U)        /*!< Bit 1 */
#define USB_COUNT7_RX_NUM_BLOCK_2           ((uint32_t)0x00001000U)        /*!< Bit 2 */
#define USB_COUNT7_RX_NUM_BLOCK_3           ((uint32_t)0x00002000U)        /*!< Bit 3 */
#define USB_COUNT7_RX_NUM_BLOCK_4           ((uint32_t)0x00004000U)        /*!< Bit 4 */

#define USB_COUNT7_RX_BLSIZE                ((uint32_t)0x00008000U)        /*!< BLock SIZE */

/*----------------------------------------------------------------------------*/

/****************  Bit definition for USB_COUNT0_RX_0 register  ***************/
#define USB_COUNT0_RX_0_COUNT0_RX_0         ((uint32_t)0x000003FFU)        /*!< Reception Byte Count (low) */

#define USB_COUNT0_RX_0_NUM_BLOCK_0         ((uint32_t)0x00007C00U)        /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define USB_COUNT0_RX_0_NUM_BLOCK_0_0       ((uint32_t)0x00000400U)        /*!< Bit 0 */
#define USB_COUNT0_RX_0_NUM_BLOCK_0_1       ((uint32_t)0x00000800U)        /*!< Bit 1 */
#define USB_COUNT0_RX_0_NUM_BLOCK_0_2       ((uint32_t)0x00001000U)        /*!< Bit 2 */
#define USB_COUNT0_RX_0_NUM_BLOCK_0_3       ((uint32_t)0x00002000U)        /*!< Bit 3 */
#define USB_COUNT0_RX_0_NUM_BLOCK_0_4       ((uint32_t)0x00004000U)        /*!< Bit 4 */

#define USB_COUNT0_RX_0_BLSIZE_0            ((uint32_t)0x00008000U)        /*!< BLock SIZE (low) */

/****************  Bit definition for USB_COUNT0_RX_1 register  ***************/
#define USB_COUNT0_RX_1_COUNT0_RX_1         ((uint32_t)0x03FF0000U)        /*!< Reception Byte Count (high) */

#define USB_COUNT0_RX_1_NUM_BLOCK_1         ((uint32_t)0x7C000000U)        /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define USB_COUNT0_RX_1_NUM_BLOCK_1_0       ((uint32_t)0x04000000U)        /*!< Bit 1 */
#define USB_COUNT0_RX_1_NUM_BLOCK_1_1       ((uint32_t)0x08000000U)        /*!< Bit 1 */
#define USB_COUNT0_RX_1_NUM_BLOCK_1_2       ((uint32_t)0x10000000U)        /*!< Bit 2 */
#define USB_COUNT0_RX_1_NUM_BLOCK_1_3       ((uint32_t)0x20000000U)        /*!< Bit 3 */
#define USB_COUNT0_RX_1_NUM_BLOCK_1_4       ((uint32_t)0x40000000U)        /*!< Bit 4 */

#define USB_COUNT0_RX_1_BLSIZE_1            ((uint32_t)0x80000000U)        /*!< BLock SIZE (high) */

/****************  Bit definition for USB_COUNT1_RX_0 register  ***************/
#define USB_COUNT1_RX_0_COUNT1_RX_0         ((uint32_t)0x000003FFU)        /*!< Reception Byte Count (low) */

#define USB_COUNT1_RX_0_NUM_BLOCK_0         ((uint32_t)0x00007C00U)        /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define USB_COUNT1_RX_0_NUM_BLOCK_0_0       ((uint32_t)0x00000400U)        /*!< Bit 0 */
#define USB_COUNT1_RX_0_NUM_BLOCK_0_1       ((uint32_t)0x00000800U)        /*!< Bit 1 */
#define USB_COUNT1_RX_0_NUM_BLOCK_0_2       ((uint32_t)0x00001000U)        /*!< Bit 2 */
#define USB_COUNT1_RX_0_NUM_BLOCK_0_3       ((uint32_t)0x00002000U)        /*!< Bit 3 */
#define USB_COUNT1_RX_0_NUM_BLOCK_0_4       ((uint32_t)0x00004000U)        /*!< Bit 4 */

#define USB_COUNT1_RX_0_BLSIZE_0            ((uint32_t)0x00008000U)        /*!< BLock SIZE (low) */

/****************  Bit definition for USB_COUNT1_RX_1 register  ***************/
#define USB_COUNT1_RX_1_COUNT1_RX_1         ((uint32_t)0x03FF0000U)        /*!< Reception Byte Count (high) */

#define USB_COUNT1_RX_1_NUM_BLOCK_1         ((uint32_t)0x7C000000U)        /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define USB_COUNT1_RX_1_NUM_BLOCK_1_0       ((uint32_t)0x04000000U)        /*!< Bit 0 */
#define USB_COUNT1_RX_1_NUM_BLOCK_1_1       ((uint32_t)0x08000000U)        /*!< Bit 1 */
#define USB_COUNT1_RX_1_NUM_BLOCK_1_2       ((uint32_t)0x10000000U)        /*!< Bit 2 */
#define USB_COUNT1_RX_1_NUM_BLOCK_1_3       ((uint32_t)0x20000000U)        /*!< Bit 3 */
#define USB_COUNT1_RX_1_NUM_BLOCK_1_4       ((uint32_t)0x40000000U)        /*!< Bit 4 */

#define USB_COUNT1_RX_1_BLSIZE_1            ((uint32_t)0x80000000U)        /*!< BLock SIZE (high) */

/****************  Bit definition for USB_COUNT2_RX_0 register  ***************/
#define USB_COUNT2_RX_0_COUNT2_RX_0         ((uint32_t)0x000003FFU)        /*!< Reception Byte Count (low) */

#define USB_COUNT2_RX_0_NUM_BLOCK_0         ((uint32_t)0x00007C00U)        /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define USB_COUNT2_RX_0_NUM_BLOCK_0_0       ((uint32_t)0x00000400U)        /*!< Bit 0 */
#define USB_COUNT2_RX_0_NUM_BLOCK_0_1       ((uint32_t)0x00000800U)        /*!< Bit 1 */
#define USB_COUNT2_RX_0_NUM_BLOCK_0_2       ((uint32_t)0x00001000U)        /*!< Bit 2 */
#define USB_COUNT2_RX_0_NUM_BLOCK_0_3       ((uint32_t)0x00002000U)        /*!< Bit 3 */
#define USB_COUNT2_RX_0_NUM_BLOCK_0_4       ((uint32_t)0x00004000U)        /*!< Bit 4 */

#define USB_COUNT2_RX_0_BLSIZE_0            ((uint32_t)0x00008000U)        /*!< BLock SIZE (low) */

/****************  Bit definition for USB_COUNT2_RX_1 register  ***************/
#define USB_COUNT2_RX_1_COUNT2_RX_1         ((uint32_t)0x03FF0000U)        /*!< Reception Byte Count (high) */

#define USB_COUNT2_RX_1_NUM_BLOCK_1         ((uint32_t)0x7C000000U)        /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define USB_COUNT2_RX_1_NUM_BLOCK_1_0       ((uint32_t)0x04000000U)        /*!< Bit 0 */
#define USB_COUNT2_RX_1_NUM_BLOCK_1_1       ((uint32_t)0x08000000U)        /*!< Bit 1 */
#define USB_COUNT2_RX_1_NUM_BLOCK_1_2       ((uint32_t)0x10000000U)        /*!< Bit 2 */
#define USB_COUNT2_RX_1_NUM_BLOCK_1_3       ((uint32_t)0x20000000U)        /*!< Bit 3 */
#define USB_COUNT2_RX_1_NUM_BLOCK_1_4       ((uint32_t)0x40000000U)        /*!< Bit 4 */

#define USB_COUNT2_RX_1_BLSIZE_1            ((uint32_t)0x80000000U)        /*!< BLock SIZE (high) */

/****************  Bit definition for USB_COUNT3_RX_0 register  ***************/
#define USB_COUNT3_RX_0_COUNT3_RX_0         ((uint32_t)0x000003FFU)        /*!< Reception Byte Count (low) */

#define USB_COUNT3_RX_0_NUM_BLOCK_0         ((uint32_t)0x00007C00U)        /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define USB_COUNT3_RX_0_NUM_BLOCK_0_0       ((uint32_t)0x00000400U)        /*!< Bit 0 */
#define USB_COUNT3_RX_0_NUM_BLOCK_0_1       ((uint32_t)0x00000800U)        /*!< Bit 1 */
#define USB_COUNT3_RX_0_NUM_BLOCK_0_2       ((uint32_t)0x00001000U)        /*!< Bit 2 */
#define USB_COUNT3_RX_0_NUM_BLOCK_0_3       ((uint32_t)0x00002000U)        /*!< Bit 3 */
#define USB_COUNT3_RX_0_NUM_BLOCK_0_4       ((uint32_t)0x00004000U)        /*!< Bit 4 */

#define USB_COUNT3_RX_0_BLSIZE_0            ((uint32_t)0x00008000U)        /*!< BLock SIZE (low) */

/****************  Bit definition for USB_COUNT3_RX_1 register  ***************/
#define USB_COUNT3_RX_1_COUNT3_RX_1         ((uint32_t)0x03FF0000U)        /*!< Reception Byte Count (high) */

#define USB_COUNT3_RX_1_NUM_BLOCK_1         ((uint32_t)0x7C000000U)        /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define USB_COUNT3_RX_1_NUM_BLOCK_1_0       ((uint32_t)0x04000000U)        /*!< Bit 0 */
#define USB_COUNT3_RX_1_NUM_BLOCK_1_1       ((uint32_t)0x08000000U)        /*!< Bit 1 */
#define USB_COUNT3_RX_1_NUM_BLOCK_1_2       ((uint32_t)0x10000000U)        /*!< Bit 2 */
#define USB_COUNT3_RX_1_NUM_BLOCK_1_3       ((uint32_t)0x20000000U)        /*!< Bit 3 */
#define USB_COUNT3_RX_1_NUM_BLOCK_1_4       ((uint32_t)0x40000000U)        /*!< Bit 4 */

#define USB_COUNT3_RX_1_BLSIZE_1            ((uint32_t)0x80000000U)        /*!< BLock SIZE (high) */

/****************  Bit definition for USB_COUNT4_RX_0 register  ***************/
#define USB_COUNT4_RX_0_COUNT4_RX_0         ((uint32_t)0x000003FFU)        /*!< Reception Byte Count (low) */

#define USB_COUNT4_RX_0_NUM_BLOCK_0         ((uint32_t)0x00007C00U)        /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define USB_COUNT4_RX_0_NUM_BLOCK_0_0       ((uint32_t)0x00000400U)        /*!< Bit 0 */
#define USB_COUNT4_RX_0_NUM_BLOCK_0_1       ((uint32_t)0x00000800U)        /*!< Bit 1 */
#define USB_COUNT4_RX_0_NUM_BLOCK_0_2       ((uint32_t)0x00001000U)        /*!< Bit 2 */
#define USB_COUNT4_RX_0_NUM_BLOCK_0_3       ((uint32_t)0x00002000U)        /*!< Bit 3 */
#define USB_COUNT4_RX_0_NUM_BLOCK_0_4       ((uint32_t)0x00004000U)        /*!< Bit 4 */

#define USB_COUNT4_RX_0_BLSIZE_0            ((uint32_t)0x00008000U)        /*!< BLock SIZE (low) */

/****************  Bit definition for USB_COUNT4_RX_1 register  ***************/
#define USB_COUNT4_RX_1_COUNT4_RX_1         ((uint32_t)0x03FF0000U)        /*!< Reception Byte Count (high) */

#define USB_COUNT4_RX_1_NUM_BLOCK_1         ((uint32_t)0x7C000000U)        /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define USB_COUNT4_RX_1_NUM_BLOCK_1_0       ((uint32_t)0x04000000U)        /*!< Bit 0 */
#define USB_COUNT4_RX_1_NUM_BLOCK_1_1       ((uint32_t)0x08000000U)        /*!< Bit 1 */
#define USB_COUNT4_RX_1_NUM_BLOCK_1_2       ((uint32_t)0x10000000U)        /*!< Bit 2 */
#define USB_COUNT4_RX_1_NUM_BLOCK_1_3       ((uint32_t)0x20000000U)        /*!< Bit 3 */
#define USB_COUNT4_RX_1_NUM_BLOCK_1_4       ((uint32_t)0x40000000U)        /*!< Bit 4 */

#define USB_COUNT4_RX_1_BLSIZE_1            ((uint32_t)0x80000000U)        /*!< BLock SIZE (high) */

/****************  Bit definition for USB_COUNT5_RX_0 register  ***************/
#define USB_COUNT5_RX_0_COUNT5_RX_0         ((uint32_t)0x000003FFU)        /*!< Reception Byte Count (low) */

#define USB_COUNT5_RX_0_NUM_BLOCK_0         ((uint32_t)0x00007C00U)        /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define USB_COUNT5_RX_0_NUM_BLOCK_0_0       ((uint32_t)0x00000400U)        /*!< Bit 0 */
#define USB_COUNT5_RX_0_NUM_BLOCK_0_1       ((uint32_t)0x00000800U)        /*!< Bit 1 */
#define USB_COUNT5_RX_0_NUM_BLOCK_0_2       ((uint32_t)0x00001000U)        /*!< Bit 2 */
#define USB_COUNT5_RX_0_NUM_BLOCK_0_3       ((uint32_t)0x00002000U)        /*!< Bit 3 */
#define USB_COUNT5_RX_0_NUM_BLOCK_0_4       ((uint32_t)0x00004000U)        /*!< Bit 4 */

#define USB_COUNT5_RX_0_BLSIZE_0            ((uint32_t)0x00008000U)        /*!< BLock SIZE (low) */

/****************  Bit definition for USB_COUNT5_RX_1 register  ***************/
#define USB_COUNT5_RX_1_COUNT5_RX_1         ((uint32_t)0x03FF0000U)        /*!< Reception Byte Count (high) */

#define USB_COUNT5_RX_1_NUM_BLOCK_1         ((uint32_t)0x7C000000U)        /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define USB_COUNT5_RX_1_NUM_BLOCK_1_0       ((uint32_t)0x04000000U)        /*!< Bit 0 */
#define USB_COUNT5_RX_1_NUM_BLOCK_1_1       ((uint32_t)0x08000000U)        /*!< Bit 1 */
#define USB_COUNT5_RX_1_NUM_BLOCK_1_2       ((uint32_t)0x10000000U)        /*!< Bit 2 */
#define USB_COUNT5_RX_1_NUM_BLOCK_1_3       ((uint32_t)0x20000000U)        /*!< Bit 3 */
#define USB_COUNT5_RX_1_NUM_BLOCK_1_4       ((uint32_t)0x40000000U)        /*!< Bit 4 */

#define USB_COUNT5_RX_1_BLSIZE_1            ((uint32_t)0x80000000U)        /*!< BLock SIZE (high) */

/***************  Bit definition for USB_COUNT6_RX_0  register  ***************/
#define USB_COUNT6_RX_0_COUNT6_RX_0         ((uint32_t)0x000003FFU)        /*!< Reception Byte Count (low) */

#define USB_COUNT6_RX_0_NUM_BLOCK_0         ((uint32_t)0x00007C00U)        /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define USB_COUNT6_RX_0_NUM_BLOCK_0_0       ((uint32_t)0x00000400U)        /*!< Bit 0 */
#define USB_COUNT6_RX_0_NUM_BLOCK_0_1       ((uint32_t)0x00000800U)        /*!< Bit 1 */
#define USB_COUNT6_RX_0_NUM_BLOCK_0_2       ((uint32_t)0x00001000U)        /*!< Bit 2 */
#define USB_COUNT6_RX_0_NUM_BLOCK_0_3       ((uint32_t)0x00002000U)        /*!< Bit 3 */
#define USB_COUNT6_RX_0_NUM_BLOCK_0_4       ((uint32_t)0x00004000U)        /*!< Bit 4 */

#define USB_COUNT6_RX_0_BLSIZE_0            ((uint32_t)0x00008000U)        /*!< BLock SIZE (low) */

/****************  Bit definition for USB_COUNT6_RX_1 register  ***************/
#define USB_COUNT6_RX_1_COUNT6_RX_1         ((uint32_t)0x03FF0000U)        /*!< Reception Byte Count (high) */

#define USB_COUNT6_RX_1_NUM_BLOCK_1         ((uint32_t)0x7C000000U)        /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define USB_COUNT6_RX_1_NUM_BLOCK_1_0       ((uint32_t)0x04000000U)        /*!< Bit 0 */
#define USB_COUNT6_RX_1_NUM_BLOCK_1_1       ((uint32_t)0x08000000U)        /*!< Bit 1 */
#define USB_COUNT6_RX_1_NUM_BLOCK_1_2       ((uint32_t)0x10000000U)        /*!< Bit 2 */
#define USB_COUNT6_RX_1_NUM_BLOCK_1_3       ((uint32_t)0x20000000U)        /*!< Bit 3 */
#define USB_COUNT6_RX_1_NUM_BLOCK_1_4       ((uint32_t)0x40000000U)        /*!< Bit 4 */

#define USB_COUNT6_RX_1_BLSIZE_1            ((uint32_t)0x80000000U)        /*!< BLock SIZE (high) */

/***************  Bit definition for USB_COUNT7_RX_0 register  ****************/
#define USB_COUNT7_RX_0_COUNT7_RX_0         ((uint32_t)0x000003FFU)        /*!< Reception Byte Count (low) */

#define USB_COUNT7_RX_0_NUM_BLOCK_0         ((uint32_t)0x00007C00U)        /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define USB_COUNT7_RX_0_NUM_BLOCK_0_0       ((uint32_t)0x00000400U)        /*!< Bit 0 */
#define USB_COUNT7_RX_0_NUM_BLOCK_0_1       ((uint32_t)0x00000800U)        /*!< Bit 1 */
#define USB_COUNT7_RX_0_NUM_BLOCK_0_2       ((uint32_t)0x00001000U)        /*!< Bit 2 */
#define USB_COUNT7_RX_0_NUM_BLOCK_0_3       ((uint32_t)0x00002000U)        /*!< Bit 3 */
#define USB_COUNT7_RX_0_NUM_BLOCK_0_4       ((uint32_t)0x00004000U)        /*!< Bit 4 */

#define USB_COUNT7_RX_0_BLSIZE_0            ((uint32_t)0x00008000U)        /*!< BLock SIZE (low) */

/***************  Bit definition for USB_COUNT7_RX_1 register  ****************/
#define USB_COUNT7_RX_1_COUNT7_RX_1         ((uint32_t)0x03FF0000U)        /*!< Reception Byte Count (high) */

#define USB_COUNT7_RX_1_NUM_BLOCK_1         ((uint32_t)0x7C000000U)        /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define USB_COUNT7_RX_1_NUM_BLOCK_1_0       ((uint32_t)0x04000000U)        /*!< Bit 0 */
#define USB_COUNT7_RX_1_NUM_BLOCK_1_1       ((uint32_t)0x08000000U)        /*!< Bit 1 */
#define USB_COUNT7_RX_1_NUM_BLOCK_1_2       ((uint32_t)0x10000000U)        /*!< Bit 2 */
#define USB_COUNT7_RX_1_NUM_BLOCK_1_3       ((uint32_t)0x20000000U)        /*!< Bit 3 */
#define USB_COUNT7_RX_1_NUM_BLOCK_1_4       ((uint32_t)0x40000000U)        /*!< Bit 4 */

#define USB_COUNT7_RX_1_BLSIZE_1            ((uint32_t)0x80000000U)        /*!< BLock SIZE (high) */


/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup Exported_macros
  * @{
  */

/******************************* ADC Instances ********************************/
#define IS_ADC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == ADC1)

#define IS_ADC_COMMON_INSTANCE(INSTANCE) ((INSTANCE) == ADC1_COMMON)

/******************************** CAN Instances ******************************/
#define IS_CAN_ALL_INSTANCE(INSTANCE) ((INSTANCE) == CAN1)

/******************************** COMP Instances ******************************/
#define IS_COMP_ALL_INSTANCE(INSTANCE) (((INSTANCE) == COMP1) || \
                                        ((INSTANCE) == COMP2))

#define IS_COMP_COMMON_INSTANCE(INSTANCE) ((INSTANCE) == COMP12_COMMON)

/******************** COMP Instances with window mode capability **************/
#define IS_COMP_WINDOWMODE_INSTANCE(INSTANCE) ((INSTANCE) == COMP2)

/******************************* CRC Instances ********************************/
#define IS_CRC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == CRC)

/******************************* DAC Instances ********************************/
#define IS_DAC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == DAC1)

/******************************** DMA Instances *******************************/
#define IS_DMA_ALL_INSTANCE(INSTANCE) (((INSTANCE) == DMA1_Channel1) || \
                                       ((INSTANCE) == DMA1_Channel2) || \
                                       ((INSTANCE) == DMA1_Channel3) || \
                                       ((INSTANCE) == DMA1_Channel4) || \
                                       ((INSTANCE) == DMA1_Channel5) || \
                                       ((INSTANCE) == DMA1_Channel6) || \
                                       ((INSTANCE) == DMA1_Channel7) || \
                                       ((INSTANCE) == DMA2_Channel1) || \
                                       ((INSTANCE) == DMA2_Channel2) || \
                                       ((INSTANCE) == DMA2_Channel3) || \
                                       ((INSTANCE) == DMA2_Channel4) || \
                                       ((INSTANCE) == DMA2_Channel5) || \
                                       ((INSTANCE) == DMA2_Channel6) || \
                                       ((INSTANCE) == DMA2_Channel7))

/******************************* GPIO Instances *******************************/
#define IS_GPIO_ALL_INSTANCE(INSTANCE) (((INSTANCE) == GPIOA) || \
                                        ((INSTANCE) == GPIOB) || \
                                        ((INSTANCE) == GPIOC) || \
                                        ((INSTANCE) == GPIOD) || \
                                        ((INSTANCE) == GPIOE) || \
                                        ((INSTANCE) == GPIOH))

/******************************* GPIO AF Instances ****************************/
/* On L4, all GPIO Bank support AF */
#define IS_GPIO_AF_INSTANCE(INSTANCE)   IS_GPIO_ALL_INSTANCE(INSTANCE)

/**************************** GPIO Lock Instances *****************************/
/* On L4, all GPIO Bank support the Lock mechanism */
#define IS_GPIO_LOCK_INSTANCE(INSTANCE) IS_GPIO_ALL_INSTANCE(INSTANCE)

/******************************** I2C Instances *******************************/
#define IS_I2C_ALL_INSTANCE(INSTANCE) (((INSTANCE) == I2C1) || \
                                       ((INSTANCE) == I2C2) || \
                                       ((INSTANCE) == I2C3))

/****************** I2C Instances : wakeup capability from stop modes *********/
#define IS_I2C_WAKEUP_FROMSTOP_INSTANCE(INSTANCE) IS_I2C_ALL_INSTANCE(INSTANCE)

/******************************* LCD Instances ********************************/
#define IS_LCD_ALL_INSTANCE(INSTANCE) ((INSTANCE) == LCD)

/****************************** OPAMP Instances *******************************/
#define IS_OPAMP_ALL_INSTANCE(INSTANCE) ((INSTANCE) == OPAMP1)

#define IS_OPAMP_COMMON_INSTANCE(INSTANCE) ((INSTANCE) == OPAMP1_COMMON)

/******************************* QSPI Instances *******************************/
#define IS_QSPI_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == QUADSPI)

/******************************* RNG Instances ********************************/
#define IS_RNG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == RNG)

/****************************** RTC Instances *********************************/
#define IS_RTC_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == RTC)

/******************************** SAI Instances *******************************/
#define IS_SAI_ALL_INSTANCE(INSTANCE) (((INSTANCE) == SAI1_Block_A) || \
                                       ((INSTANCE) == SAI1_Block_B))

/****************************** SDMMC Instances *******************************/
#define IS_SDMMC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == SDMMC1)

/****************************** SMBUS Instances *******************************/
#define IS_SMBUS_ALL_INSTANCE(INSTANCE) (((INSTANCE) == I2C1) || \
                                         ((INSTANCE) == I2C2) || \
                                         ((INSTANCE) == I2C3))

/******************************** SPI Instances *******************************/
#define IS_SPI_ALL_INSTANCE(INSTANCE) (((INSTANCE) == SPI1) || \
                                       ((INSTANCE) == SPI2) || \
                                       ((INSTANCE) == SPI3))

/******************************** SWPMI Instances *****************************/
#define IS_SWPMI_INSTANCE(INSTANCE)  ((INSTANCE) == SWPMI1)

/****************** LPTIM Instances : All supported instances *****************/
#define IS_LPTIM_INSTANCE(INSTANCE)     (((INSTANCE) == LPTIM1) || \
                                         ((INSTANCE) == LPTIM2))

/****************** TIM Instances : All supported instances *******************/
#define IS_TIM_INSTANCE(INSTANCE)       (((INSTANCE) == TIM1)   || \
                                         ((INSTANCE) == TIM2)   || \
                                         ((INSTANCE) == TIM6)   || \
                                         ((INSTANCE) == TIM7)   || \
                                         ((INSTANCE) == TIM15)  || \
                                         ((INSTANCE) == TIM16))

/****************** TIM Instances : supporting 32 bits counter ****************/
#define IS_TIM_32B_COUNTER_INSTANCE(INSTANCE) ((INSTANCE) == TIM2)

/****************** TIM Instances : supporting the break function *************/
#define IS_TIM_BREAK_INSTANCE(INSTANCE)    (((INSTANCE) == TIM1)    || \
                                            ((INSTANCE) == TIM15)   || \
                                            ((INSTANCE) == TIM16))

/************** TIM Instances : supporting Break source selection *************/
#define IS_TIM_BREAKSOURCE_INSTANCE(INSTANCE) (((INSTANCE) == TIM1)   || \
                                               ((INSTANCE) == TIM15)  || \
                                               ((INSTANCE) == TIM16))

/****************** TIM Instances : supporting 2 break inputs *****************/
#define IS_TIM_BKIN2_INSTANCE(INSTANCE)    ((INSTANCE) == TIM1)

/************* TIM Instances : at least 1 capture/compare channel *************/
#define IS_TIM_CC1_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)   || \
                                         ((INSTANCE) == TIM2)   || \
                                         ((INSTANCE) == TIM15)  || \
                                         ((INSTANCE) == TIM16))

/************ TIM Instances : at least 2 capture/compare channels *************/
#define IS_TIM_CC2_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)   || \
                                         ((INSTANCE) == TIM2)   || \
                                         ((INSTANCE) == TIM15))

/************ TIM Instances : at least 3 capture/compare channels *************/
#define IS_TIM_CC3_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)   || \
                                         ((INSTANCE) == TIM2))

/************ TIM Instances : at least 4 capture/compare channels *************/
#define IS_TIM_CC4_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)   || \
                                         ((INSTANCE) == TIM2))

/****************** TIM Instances : at least 5 capture/compare channels *******/
#define IS_TIM_CC5_INSTANCE(INSTANCE)   ((INSTANCE) == TIM1)

/****************** TIM Instances : at least 6 capture/compare channels *******/
#define IS_TIM_CC6_INSTANCE(INSTANCE)   ((INSTANCE) == TIM1)

/************ TIM Instances : DMA requests generation (TIMx_DIER.COMDE) *******/
#define IS_TIM_CCDMA_INSTANCE(INSTANCE)    (((INSTANCE) == TIM1)   || \
                                            ((INSTANCE) == TIM15)  || \
                                            ((INSTANCE) == TIM16))

/****************** TIM Instances : DMA requests generation (TIMx_DIER.UDE) ***/
#define IS_TIM_DMA_INSTANCE(INSTANCE)      (((INSTANCE) == TIM1)   || \
                                            ((INSTANCE) == TIM2)   || \
                                            ((INSTANCE) == TIM6)   || \
                                            ((INSTANCE) == TIM7)   || \
                                            ((INSTANCE) == TIM15)  || \
                                            ((INSTANCE) == TIM16))

/************ TIM Instances : DMA requests generation (TIMx_DIER.CCxDE) *******/
#define IS_TIM_DMA_CC_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)   || \
                                            ((INSTANCE) == TIM2)   || \
                                            ((INSTANCE) == TIM15)  || \
                                            ((INSTANCE) == TIM16))

/******************** TIM Instances : DMA burst feature ***********************/
#define IS_TIM_DMABURST_INSTANCE(INSTANCE) (((INSTANCE) == TIM1)   || \
                                            ((INSTANCE) == TIM2)   || \
                                            ((INSTANCE) == TIM15)  || \
                                            ((INSTANCE) == TIM16))

/******************* TIM Instances : output(s) available **********************/
#define IS_TIM_CCX_INSTANCE(INSTANCE, CHANNEL) \
    ((((INSTANCE) == TIM1) &&                  \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4) ||          \
      ((CHANNEL) == TIM_CHANNEL_5) ||          \
      ((CHANNEL) == TIM_CHANNEL_6)))           \
     ||                                        \
     (((INSTANCE) == TIM2) &&                  \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4)))           \
     ||                                        \
     (((INSTANCE) == TIM15) &&                 \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2)))           \
     ||                                        \
     (((INSTANCE) == TIM16) &&                 \
     (((CHANNEL) == TIM_CHANNEL_1))))

/****************** TIM Instances : supporting complementary output(s) ********/
#define IS_TIM_CCXN_INSTANCE(INSTANCE, CHANNEL) \
   ((((INSTANCE) == TIM1) &&                    \
     (((CHANNEL) == TIM_CHANNEL_1) ||           \
      ((CHANNEL) == TIM_CHANNEL_2) ||           \
      ((CHANNEL) == TIM_CHANNEL_3)))            \
    ||                                          \
    (((INSTANCE) == TIM15) &&                   \
     ((CHANNEL) == TIM_CHANNEL_1))              \
    ||                                          \
    (((INSTANCE) == TIM16) &&                   \
     ((CHANNEL) == TIM_CHANNEL_1)))

/****************** TIM Instances : supporting clock division *****************/
#define IS_TIM_CLOCK_DIVISION_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)    || \
                                                    ((INSTANCE) == TIM2)    || \
                                                    ((INSTANCE) == TIM15)   || \
                                                    ((INSTANCE) == TIM16))

/****** TIM Instances : supporting external clock mode 1 for ETRF input *******/
#define IS_TIM_CLOCKSOURCE_ETRMODE1_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                                        ((INSTANCE) == TIM2) || \
                                                        ((INSTANCE) == TIM15))

/****** TIM Instances : supporting external clock mode 2 for ETRF input *******/
#define IS_TIM_CLOCKSOURCE_ETRMODE2_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
                                                        ((INSTANCE) == TIM2))

/****************** TIM Instances : supporting external clock mode 1 for TIX inputs*/
#define IS_TIM_CLOCKSOURCE_TIX_INSTANCE(INSTANCE)      (((INSTANCE) == TIM1) || \
                                                        ((INSTANCE) == TIM2) || \
                                                        ((INSTANCE) == TIM15))

/****************** TIM Instances : supporting internal trigger inputs(ITRX) *******/
#define IS_TIM_CLOCKSOURCE_ITRX_INSTANCE(INSTANCE)     (((INSTANCE) == TIM1) || \
                                                        ((INSTANCE) == TIM2) || \
                                                        ((INSTANCE) == TIM15))

/****************** TIM Instances : supporting combined 3-phase PWM mode ******/
#define IS_TIM_COMBINED3PHASEPWM_INSTANCE(INSTANCE) ((INSTANCE) == TIM1)

/****************** TIM Instances : supporting commutation event generation ***/
#define IS_TIM_COMMUTATION_EVENT_INSTANCE(INSTANCE) (((INSTANCE) == TIM1)   || \
                                                     ((INSTANCE) == TIM15)  || \
                                                     ((INSTANCE) == TIM16))

/****************** TIM Instances : supporting counting mode selection ********/
#define IS_TIM_COUNTER_MODE_SELECT_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1) || \
                                                        ((INSTANCE) == TIM2))

/****************** TIM Instances : supporting encoder interface **************/
#define IS_TIM_ENCODER_INTERFACE_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1)  || \
                                                      ((INSTANCE) == TIM2))

/****************** TIM Instances : supporting Hall sensor interface **********/
#define IS_TIM_HALL_SENSOR_INTERFACE_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1)  || \
                                                      ((INSTANCE) == TIM2))

/**************** TIM Instances : external trigger input available ************/
#define IS_TIM_ETR_INSTANCE(INSTANCE)      (((INSTANCE) == TIM1)  || \
                                            ((INSTANCE) == TIM2))

/************* TIM Instances : supporting ETR source selection ***************/
#define IS_TIM_ETRSEL_INSTANCE(INSTANCE)    (((INSTANCE) == TIM1)  || \
                                             ((INSTANCE) == TIM2))
      
/****** TIM Instances : Master mode available (TIMx_CR2.MMS available )********/
#define IS_TIM_MASTER_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)  || \
                                            ((INSTANCE) == TIM2)  || \
                                            ((INSTANCE) == TIM6)  || \
                                            ((INSTANCE) == TIM7)  || \
                                            ((INSTANCE) == TIM15))

/*********** TIM Instances : Slave mode available (TIMx_SMCR available )*******/
#define IS_TIM_SLAVE_INSTANCE(INSTANCE)    (((INSTANCE) == TIM1)  || \
                                            ((INSTANCE) == TIM2)  || \
                                            ((INSTANCE) == TIM15))

/****************** TIM Instances : supporting OCxREF clear *******************/
#define IS_TIM_OCXREF_CLEAR_INSTANCE(INSTANCE)        (((INSTANCE) == TIM1) || \
                                                       ((INSTANCE) == TIM2))

/****************** TIM Instances : remapping capability **********************/
#define IS_TIM_REMAP_INSTANCE(INSTANCE)    (((INSTANCE) == TIM1)  || \
                                            ((INSTANCE) == TIM2)  || \
                                            ((INSTANCE) == TIM15) || \
                                            ((INSTANCE) == TIM16))

/****************** TIM Instances : supporting repetition counter *************/
#define IS_TIM_REPETITION_COUNTER_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1)  || \
                                                       ((INSTANCE) == TIM15) || \
                                                       ((INSTANCE) == TIM16))

/****************** TIM Instances : supporting synchronization ****************/
#define IS_TIM_SYNCHRO_INSTANCE(INSTANCE)  IS_TIM_MASTER_INSTANCE(INSTANCE)

/****************** TIM Instances : supporting ADC triggering through TRGO2 ***/
#define IS_TIM_TRGO2_INSTANCE(INSTANCE)    ((INSTANCE) == TIM1)

/******************* TIM Instances : Timer input XOR function *****************/
#define IS_TIM_XOR_INSTANCE(INSTANCE)      (((INSTANCE) == TIM1)   || \
                                            ((INSTANCE) == TIM2)   || \
                                            ((INSTANCE) == TIM15))

/****************************** TSC Instances *********************************/
#define IS_TSC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == TSC)

/******************** USART Instances : Synchronous mode **********************/
#define IS_USART_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                     ((INSTANCE) == USART2) || \
                                     ((INSTANCE) == USART3))

/******************** UART Instances : Asynchronous mode **********************/
#define IS_UART_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                    ((INSTANCE) == USART2) || \
                                    ((INSTANCE) == USART3))

/****************** UART Instances : Auto Baud Rate detection ****************/
#define IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                                            ((INSTANCE) == USART2) || \
                                                            ((INSTANCE) == USART3))

/****************** UART Instances : Driver Enable *****************/
#define IS_UART_DRIVER_ENABLE_INSTANCE(INSTANCE)     (((INSTANCE) == USART1) || \
                                                      ((INSTANCE) == USART2) || \
                                                      ((INSTANCE) == USART3) || \
                                                      ((INSTANCE) == LPUART1))

/******************** UART Instances : Half-Duplex mode **********************/
#define IS_UART_HALFDUPLEX_INSTANCE(INSTANCE)   (((INSTANCE) == USART1) || \
                                                 ((INSTANCE) == USART2) || \
                                                 ((INSTANCE) == USART3) || \
                                                 ((INSTANCE) == LPUART1))

/****************** UART Instances : Hardware Flow control ********************/
#define IS_UART_HWFLOW_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                           ((INSTANCE) == USART2) || \
                                           ((INSTANCE) == USART3) || \
                                           ((INSTANCE) == LPUART1))

/******************** UART Instances : LIN mode **********************/
#define IS_UART_LIN_INSTANCE(INSTANCE)   (((INSTANCE) == USART1) || \
                                          ((INSTANCE) == USART2) || \
                                          ((INSTANCE) == USART3))

/******************** UART Instances : Wake-up from Stop mode **********************/
#define IS_UART_WAKEUP_FROMSTOP_INSTANCE(INSTANCE)   (((INSTANCE) == USART1) || \
                                                      ((INSTANCE) == USART2) || \
                                                      ((INSTANCE) == USART3) || \
                                                      ((INSTANCE) == LPUART1))

/*********************** UART Instances : IRDA mode ***************************/
#define IS_IRDA_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                    ((INSTANCE) == USART2) || \
                                    ((INSTANCE) == USART3))

/********************* USART Instances : Smard card mode ***********************/
#define IS_SMARTCARD_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                         ((INSTANCE) == USART2) || \
                                         ((INSTANCE) == USART3))

/******************** LPUART Instance *****************************************/
#define IS_LPUART_INSTANCE(INSTANCE)    ((INSTANCE) == LPUART1)

/****************************** IWDG Instances ********************************/
#define IS_IWDG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == IWDG)

/****************************** WWDG Instances ********************************/
#define IS_WWDG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == WWDG)

/******************************* USB Instances *******************************/
#define IS_USB_ALL_INSTANCE(INSTANCE) ((INSTANCE) == USB)

/**
  * @}
  */


/******************************************************************************/
/*  For a painless codes migration between the STM32L4xx device product       */
/*  lines, the aliases defined below are put in place to overcome the         */
/*  differences in the interrupt handlers and IRQn definitions.               */
/*  No need to update developed interrupt code when moving across             */ 
/*  product lines within the same STM32L4 Family                              */
/******************************************************************************/

/* Aliases for __IRQn */
#define ADC1_2_IRQn                    ADC1_IRQn
#define TIM1_TRG_COM_TIM17_IRQn        TIM1_TRG_COM_IRQn
#define USB_FS_IRQn                    USB_IRQn

/* Aliases for __IRQHandler */
#define ADC1_2_IRQHandler              ADC1_IRQHandler
#define TIM1_TRG_COM_TIM17_IRQHandler  TIM1_TRG_COM_IRQHandler
#define USB_FS_IRQHandler              USB_IRQHandler

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __STM32L433xx_H */

/**
  * @}
  */

  /**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
