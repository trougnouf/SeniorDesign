/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define UART_TX_Laser_Pin GPIO_PIN_0
#define UART_TX_Laser_GPIO_Port GPIOA
#define UART_RX_Laser_Pin GPIO_PIN_1
#define UART_RX_Laser_GPIO_Port GPIOA
#define SPI_CLK_Compass_Pin GPIO_PIN_5
#define SPI_CLK_Compass_GPIO_Port GPIOA
#define SPI_MOSI_Compass_Pin GPIO_PIN_7
#define SPI_MOSI_Compass_GPIO_Port GPIOA
#define COMP_INT_Pin GPIO_PIN_4
#define COMP_INT_GPIO_Port GPIOC
#define TRIG_Compass_Pin GPIO_PIN_5
#define TRIG_Compass_GPIO_Port GPIOC
#define USART1_TX_Debug_Pin GPIO_PIN_9
#define USART1_TX_Debug_GPIO_Port GPIOA
#define COMP_CS_Pin GPIO_PIN_15
#define COMP_CS_GPIO_Port GPIOA
#define SPI_MISO_Compass_Pin GPIO_PIN_4
#define SPI_MISO_Compass_GPIO_Port GPIOB
#define USART1_RX_Debug_Pin GPIO_PIN_7
#define USART1_RX_Debug_GPIO_Port GPIOB
#define HEARTBEAT_Pin GPIO_PIN_9
#define HEARTBEAT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
