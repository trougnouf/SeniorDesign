/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdint.h>

#define STATUS_DELAY	10000

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
osThreadId fsTaskHandle;
osThreadId laserTaskHandle;
osThreadId compassTaskHandle;
osThreadId vibratorTaskHandle;

volatile uint8_t vibrationLv;
uint8_t debugging;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void StartFSTask(void const * argument);
void StartLaserTask(void const * argument);
void StartCompassTask(void const * argument);
void StartVibratorTask(void const * argument);

void compass_takemeasurement(uint8_t * inbuf, uint8_t * outbuf);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */
  //HAL_SD_Init(&hsd1, &SDCardInfo1);
  //HAL_SD_WideBusOperation_Config(&hsd1, SDMMC_BUS_WIDE_4B);
  vibrationLv = 0;
  debugging = 1;
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  //osThreadDef(fsTask, startFSTask, osPriorityNormal, 0, 128);
  //fsTaskHandle = osThreadCreate(osThread(fsTask), NULL);
  
  osThreadDef(laserTask, StartLaserTask, osPriorityNormal, 0, 128);
  laserTaskHandle = osThreadCreate(osThread(laserTask), NULL);
  
  osThreadDef(compassTask, StartCompassTask, osPriorityBelowNormal, 0, 128);
  compassTaskHandle = osThreadCreate(osThread(compassTask), NULL);
  
  osThreadDef(vibratorTask, StartVibratorTask, osPriorityNormal, 0, 64);
  vibratorTaskHandle = osThreadCreate(osThread(vibratorTask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_UART4;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  __HAL_RCC_PWR_CLK_ENABLE();

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  HAL_SPI_Init(&hspi2);

}

/* UART4 init function */
void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 19200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart4);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PH0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : PH1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : VIBRATOR_Pin LED_Pin COMP_CS_Pin */
  GPIO_InitStruct.Pin = VIBRATOR_Pin|LED_Pin|COMP_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : COMP_INT_Pin INT2_Compass_Pin */
  GPIO_InitStruct.Pin = COMP_INT_Pin|INT2_Compass_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : FIVEVOLTSEN_Pin */
  GPIO_InitStruct.Pin = FIVEVOLTSEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FIVEVOLTSEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, VIBRATOR_Pin|LED_Pin|COMP_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FIVEVOLTSEN_GPIO_Port, FIVEVOLTSEN_Pin, GPIO_PIN_RESET);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

// Interrupts

void EXTI0_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI0_IRQn) != RESET)
  {
    osThreadSuspend(laserTaskHandle);
    osThreadSuspend(vibratorTaskHandle);
    HAL_GPIO_WritePin(VIBRATOR_GPIO_Port, VIBRATOR_Pin, GPIO_PIN_RESET);
    osThreadResume(compassTaskHandle);
  }
  EXTI_ClearITPendingBit(EXTI0_IRQn);
}

// Functions
// Useful debugging info
void compass_dbg_uartout(uint8_t * inBuf, uint8_t * outBuf, uint8_t * msg)
{
  if(outBuf[0] == 62)	// SSMM
  {
    msg = "SMM:\t";
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  }
  else if(outBuf[0] == 78)	// RM
  {
    msg = "RM:\t";
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  }
  else if(outBuf[0] == 240)	// RST
  {
    msg = "RST:\t";
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  }
  else if(outBuf[0] == 80)	// RR
  {
    msg = "RR:\t";
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  }
  if(inBuf[0] & 128)	msg = "BM\t";
  else				msg = "  \t";
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  if(inBuf[0] & 64)	msg = "WC\t";
  else				msg = "  \t";
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  if(inBuf[0] & 32)	msg = "SM\t";
  else				msg = "  \t";
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  if(inBuf[0] & 16)	msg = "ER\t";
  else				msg = "  \t";
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  if(inBuf[0] & 8)	msg = "SD\t";
  else				msg = "  \t";
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  if(inBuf[0] & 4)	msg = "RS\t";
  else				msg = "  \t";
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  if(inBuf[0] & 2)	msg = "D1\t";
  else				msg = "  \t";
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  if(inBuf[0] & 1)	msg = "D0\t";
  else				msg = "  \t";
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  if(HAL_GPIO_ReadPin(COMP_INT_GPIO_Port, COMP_INT_Pin))
  {
    msg = "RDY\t";
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  }
  msg = "\n\n\r";
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
}

// I/O
void compass_send_SPI(uint8_t * inBuf, uint8_t inLength, uint8_t * outBuf, uint8_t outLength)
{
  HAL_GPIO_WritePin(COMP_CS_GPIO_Port, COMP_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, outBuf, outLength, 1000);
  HAL_SPI_Receive(&hspi2, inBuf, inLength, 1000);
  HAL_GPIO_WritePin(COMP_CS_GPIO_Port, COMP_CS_Pin, GPIO_PIN_SET);
}

// Print results as integers
void compass_uart(int16_t * xyz, uint8_t * msg, int16_t * angle)
{
  sprintf(msg, "x: %5d\t", xyz[0]);
  if(HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0x1000) != HAL_OK)
    return;
  sprintf(msg, "y: %5d\t", xyz[1]);
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  sprintf(msg, "z: %5d\t", xyz[2]);
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  sprintf(msg, "a: %d\t", *angle);
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  msg = "\n\r";
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
}

// Convert raw input bits to integers
void compass_processInput(uint8_t * inbuf, int16_t * xyz, int16_t * minmaxxyz)
{
  // FL is off by 2 degrees W: Don't care.
  for(uint8_t i=0; i<3; i++)
  {
    // 2's complement binaries to int16_t
    xyz[i] = ((int16_t)inbuf[i*2+1] << 8 | inbuf[i*2+2]);
    // (optional) use GAIN_SEL to find "real" value
    xyz[i] *= i==2?.294:.161;
    // Update minmaxxyz
    if(xyz[i] < minmaxxyz[2*i])	minmaxxyz[2*i] = xyz[i];
    if(xyz[i] > minmaxxyz[2*i+1])	minmaxxyz[2*i+1] = xyz[i];
    // Normalize results
    xyz[i] -= (minmaxxyz[2*i+1]+minmaxxyz[2*i])/2;
  }
}

/*
* Calibration: Do a bunch of measurements
*
* Measurements/normalization: Save min and max. Med = 0
*/

void compass_calibrate(uint8_t * inbuf, uint8_t * outbuf, int16_t * xyz, int16_t * minmaxxyz)
{
  compass_takemeasurement(inbuf, outbuf);
  for(uint8_t i=0; i<3; i++)
  {
    // 2's complement binaries to int16_t
    xyz[i] = ((int16_t)inbuf[i*2+1] << 8 | inbuf[i*2+2]);
    // (optional) use GAIN_SEL to find "real" value
    xyz[i] *= i==2?.294:.161;
    // Update minmaxxyz
    minmaxxyz[2*i] = xyz[i];
    minmaxxyz[2*1+1] = xyz[i];
  }
  for(uint16_t i=0; i<100; i++)	// TODO replace this with pushbutton
  {
    compass_takemeasurement(inbuf, outbuf);
    compass_processInput(inbuf, xyz, minmaxxyz);
    osDelay(100);
  }
}

void compass_takemeasurement(uint8_t * inbuf, uint8_t * outbuf)
{
  // Start single measurement mode
  outbuf[0] = 62;	// 0011zyxt
  compass_send_SPI(inbuf, 1, outbuf, 1);
  //compass_dbg_uartout(inbuf, outbuf, msg);
  
  while(!HAL_GPIO_ReadPin(COMP_INT_GPIO_Port, COMP_INT_Pin));
  
  //Read measurement
  outbuf[0] = 78;
  compass_send_SPI(inbuf, 7, outbuf, 1);
  //compass_dbg_uartout(inbuf, outbuf, msg);
}

void compass_getAngle(int16_t * angle, int16_t * xyz)
{
  volatile double tmp = atan((double)xyz[0]/(double)xyz[1]);
  tmp *= (180/3.1428);
  tmp = 90 - tmp;
  if(xyz[1] > 0)	*angle = (int16_t)(90.0 - (atan((double)xyz[0]/(double)xyz[1]) * (180.0 / 3.1428)));
  else if(xyz[1] < 0)	*angle = (int16_t)(270.0 - (atan((double)xyz[0]/(double)xyz[1]) * (180.0 / 3.1428)));
  else if(xyz[0]<0)	*angle = 180;
  else	*angle = 0;
}

void compass_vibrateAngle(int16_t * angle)
{
  uint8_t clockangle = *angle/30;
  uint8_t i;
  for(;clockangle > 0; clockangle--)
  {
    for(i=255; i>0; i--)
    {
      HAL_GPIO_WritePin(VIBRATOR_GPIO_Port, VIBRATOR_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(VIBRATOR_GPIO_Port, VIBRATOR_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(VIBRATOR_GPIO_Port, VIBRATOR_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(VIBRATOR_GPIO_Port, VIBRATOR_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(VIBRATOR_GPIO_Port, VIBRATOR_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(VIBRATOR_GPIO_Port, VIBRATOR_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(VIBRATOR_GPIO_Port, VIBRATOR_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(VIBRATOR_GPIO_Port, VIBRATOR_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(VIBRATOR_GPIO_Port, VIBRATOR_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(VIBRATOR_GPIO_Port, VIBRATOR_Pin, GPIO_PIN_RESET);
    }
    osDelay(250);
  }
}

// Threads

void StartVibratorTask(void const * argument)
{
  uint8_t i,j;
  osThreadSuspend(vibratorTaskHandle);
  // 3V motor, 3.3V max source. 91% max DC
  for(;;)
  {
    if(!vibrationLv)
    {
      HAL_GPIO_WritePin(VIBRATOR_GPIO_Port, VIBRATOR_Pin, GPIO_PIN_RESET);
      continue;
    }

    for(i=0; i<10; i++)
    {
      if(vibrationLv > i)
	HAL_GPIO_WritePin(VIBRATOR_GPIO_Port, VIBRATOR_Pin, GPIO_PIN_SET);
      else
	HAL_GPIO_WritePin(VIBRATOR_GPIO_Port, VIBRATOR_Pin, GPIO_PIN_RESET);
    }
  }
}

void StartFSTask(void const * argument)
{
  for(;;)
  {
  }
}

void StartCompassTask(void const * argument)
{
  osThreadSuspend(compassTaskHandle);
  
  osThreadSuspend(defaultTaskHandle);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
  uint8_t inbuf[7];
  uint8_t outbuf[2];
  int16_t xyz[3];
  int16_t minmaxxyz[6];	// xmin xmax ymin ymax zmin zmax
  int16_t angle;
  uint8_t * msg = malloc(10);
  
  // Reset: 11110000 = 0xf0
  outbuf[0] = 0xf0;
  compass_send_SPI(inbuf, 1, outbuf, 1);
  compass_dbg_uartout(inbuf, outbuf, msg);
  
  compass_calibrate(inbuf, outbuf, xyz, minmaxxyz);
  
  /*
  // Read register 0
  outbuf[0] = 80;
  outbuf[1] = 0 << 2;
  compass_send_SPI(inbuf, 3, outbuf, 2);
  compass_dbg_uartout(inbuf, outbuf, msg);
  */
  
  
  
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
  osThreadResume(defaultTaskHandle);
  osThreadResume(laserTaskHandle);
  osThreadSuspend(compassTaskHandle);
  
  /* Infinite loop */
  for(;;)
  {
    compass_takemeasurement(inbuf, outbuf);
    compass_processInput(inbuf, xyz, minmaxxyz);
    compass_getAngle(&angle, xyz);
    if(debugging)	compass_uart(xyz, msg, &angle);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    osThreadResume(laserTaskHandle);
    osThreadSuspend(compassTaskHandle);
  }
}

void StartLaserTask(void const * argument)
{
  HAL_GPIO_WritePin(FIVEVOLTSEN_GPIO_Port, FIVEVOLTSEN_Pin, GPIO_PIN_SET);
  osDelay(50);
  uint8_t in[6];
  in[0] = '0';
  uint8_t * nl = "\n\r";
  uint8_t tabs[10];
  uint8_t inSize;
  HAL_StatusTypeDef status;
  double distance = 0; //(debug) was volatile
  sprintf(tabs, "\t\t\t\t\t\t\td: ");
  for(;;)
  {
    osThreadSuspend(vibratorTaskHandle);
    HAL_GPIO_WritePin(VIBRATOR_GPIO_Port, VIBRATOR_Pin, GPIO_PIN_RESET);
    if(HAL_UART_Transmit(&huart4, &in[0], 1, 1000) == HAL_OK)// send request
    {
      // Receive
      for (inSize = 0; inSize < 7; inSize++)	// max = 25.00\n\r
      {
	status = HAL_UART_Receive(&huart4, &in[inSize], 1, 0x1000);
	if(in[inSize] == '\n')	break;
	if(status == HAL_TIMEOUT)       break;
      }
      if(status == HAL_TIMEOUT)
      {
	HAL_GPIO_WritePin(VIBRATOR_GPIO_Port, VIBRATOR_Pin, GPIO_PIN_RESET);
	osDelay(1000);
	continue;
      }
      // Send result to PC (opt)
	HAL_UART_Transmit(&huart2, (uint8_t *)tabs, 10, 0x100);
	HAL_UART_Transmit(&huart2, (uint8_t *)in, inSize-1, 0x100);
	HAL_UART_Transmit(&huart2, (uint8_t *)nl, 3, 0x100);
      //strToUART(nl);	// makes debugging prettier
      
      distance = atof(in);
    }
    // Process result
    
    //vibrationLv = - log(distance/30) / log(2);
    // The above function does not process correctly. Doing it by hand:
    if(distance <= .25)	vibrationLv = 9;
    else if(distance <= .5)	vibrationLv = 8;
    else if(distance <= 1)	vibrationLv = 7;
    else if(distance <= 2)	vibrationLv = 6;
    else if(distance <= 3)	vibrationLv = 5;
    else if(distance <= 4)	vibrationLv = 4;
    else if(distance <= 5)	vibrationLv = 3;
    else if(distance <= 10)	vibrationLv = 2;
    else if(distance <= 20)	vibrationLv = 1;
    else			vibrationLv = 0;
    /*
    if(distance >= 0.666 * 9)  vibrationLv = 0;
    else        vibrationLv = 10 - (distance / 0.666);
    if(vibrationLv == 10)	vibrationLv = 9;
    */
    //if(distance < 0.3048)			laserOut_1ft();
    //else if(distance < 0.6096)	laserOut_2ft();
    //else if(distance < 0.9144)	laserOut_3ft();
    //else							laserOut_safe();
    // delay before checking again to save precious battery life
    //osDelay(150);
    osThreadResume(vibratorTaskHandle);
    osDelay(300);
  }
}
/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /*FATFS SDFatFS;
  FIL hello;
  
  
  if(f_mount(&SDFatFS, (TCHAR const*)SD_Path, 0) != FR_OK)
    ;//strToUART("mount failure\n");
  else
  {
  uint8_t failCounter = 0;
  notready:
    volatile FRESULT SDstatus = f_open(&hello, "hello2.txt", FA_OPEN_ALWAYS);
    //strToUART("mount success\n");
    if(SDstatus == FR_OK)
    {
      //strToUART("opened hello.txt\n");
      volatile char * str;
      //int readcount;
      TCHAR * buffer[10];
      str = f_gets((char*)buffer, hello.fsize, &hello);
      //strToUART(str);
      f_close(&hello);
    }
    else if(failCounter<5)
    {
      failCounter++;
      goto notready;//strToUART("could not open hello.txt\n");
    }
  }
*/
  
  // This is useless, transmit never fails. Use button instead.
  //if(HAL_UART_Transmit(&huart2, (uint8_t*)debugging, 1, 0x1000) != HAL_OK)
    //debugging = 0;
  
  // Disable BT GP2 (USB update)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  
  /* Infinite loop */
  for(;;)
  {
    osDelay(STATUS_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    osDelay(STATUS_DELAY/10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    
  }
  /* USER CODE END 5 */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
