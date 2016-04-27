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
#include <string.h>
#include <math.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId rLEDTaskHandle;
osThreadId lLEDTaskHandle;
osThreadId tLEDTaskHandle;
osThreadId bLEDTaskHandle;
osThreadId thandlerTaskHandle;
osThreadId compassTaskHandle;
osThreadId gpsTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint8_t blinking;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartRLEDTask(void const * argument);
void StartLLEDTask(void const * argument);
void StartTLEDTask(void const * argument);
void StartBLEDTask(void const * argument);
void StartTHandlerTask(void const * argument);
void StartCompassTask(void const * argument);
void StartGPSTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int calculateditance(double lat1, double lat2, double long1, double long2);
int calculateangle( double lat1, double lat2, double long1, double long2);
void Directions(double *Mylat, double *Mylong, char *in);
void GPSReceive(double *Mylat, double *Mylong, char *in);
void compass_takemeasurement(uint8_t * inbuf, uint8_t * outbuf);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
  if(blinking)	return;
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  if (GPIO_Pin == GPIO_PIN_0)
  {
    blinking = 128;
  }
  else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11))
  {
    blinking = 64;
  }
  else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12))
  {
    blinking = 32;
  }
  HAL_Delay(200);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  osThreadResume(defaultTaskHandle);
  /*
  osThreadSuspend(defaultTaskHandle);
  //osThreadResume(thandlerTaskHandle);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);	
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);	
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);	
  HAL_Delay(100);
  if (GPIO_Pin == GPIO_PIN_0)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);	
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);	
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);	
  HAL_Delay(100);
  //osThreadSuspend(laserTaskHandle);
  //osThreadSuspend(vibratorTaskHandle);
  //HAL_GPIO_WritePin(VIBRATOR_GPIO_Port, VIBRATOR_Pin, GPIO_PIN_RESET);
  
  //HAL_Delay(100);
  //osThreadResume(testTaskHandle);
    }
  osThreadResume(rLEDTaskHandle);
  osThreadResume(bLEDTaskHandle);
  osThreadResume(thandlerTaskHandle);
  
  */
}

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
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  blinking = 0;
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of rLEDTask */
  osThreadDef(rLEDTask, StartRLEDTask, osPriorityNormal, 0, 128);
  rLEDTaskHandle = osThreadCreate(osThread(rLEDTask), NULL);

  /* definition and creation of lLEDTask */
  osThreadDef(lLEDTask, StartLLEDTask, osPriorityNormal, 0, 128);
  lLEDTaskHandle = osThreadCreate(osThread(lLEDTask), NULL);

  /* definition and creation of tLEDTask */
  osThreadDef(tLEDTask, StartTLEDTask, osPriorityNormal, 0, 128);
  tLEDTaskHandle = osThreadCreate(osThread(tLEDTask), NULL);

  /* definition and creation of bLEDTask */
  osThreadDef(bLEDTask, StartBLEDTask, osPriorityNormal, 0, 128);
  bLEDTaskHandle = osThreadCreate(osThread(bLEDTask), NULL);

  /* definition and creation of thandlerTask */
  /*osThreadDef(thandlerTask, StartTHandlerTask, osPriorityNormal, 0, 128);
  thandlerTaskHandle = osThreadCreate(osThread(thandlerTask), NULL);*/

  /* definition and creation of compassTask */
  /*osThreadDef(compassTask, StartCompassTask, osPriorityNormal, 0, 128);
  compassTaskHandle = osThreadCreate(osThread(compassTask), NULL);*/

  /* definition and creation of gpsTask */
  /*osThreadDef(gpsTask, StartGPSTask, osPriorityNormal, 0, 128);
  gpsTaskHandle = osThreadCreate(osThread(gpsTask), NULL);*/

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
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

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart1);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_7B;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ledlef_Pin|ledbot_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ledtop_Pin|ledrig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|COMP_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ledlef_Pin ledbot_Pin */
  GPIO_InitStruct.Pin = ledlef_Pin|ledbot_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PH0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : COMP_INT_Pin */
  GPIO_InitStruct.Pin = COMP_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(COMP_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ledtop_Pin ledrig_Pin */
  GPIO_InitStruct.Pin = ledtop_Pin|ledrig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin COMP_CS_Pin */
  GPIO_InitStruct.Pin = LED_Pin|COMP_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
int calculateditance(double lat1, double lat2, double long1, double long2){
  double  pi = 3.1415926535;
  double distanceKm,a;
  double c;
  
  double R = 6371;
  double dlong;
  double dlat;
  
  lat1 = lat1*(pi/180);
  lat2 = lat2*(pi/180);
  
  long1= long1*(pi/180);
  long2= long2*(pi/180);
  dlong = (long2 - long1);
  dlat  = (lat2 - lat1);
  
  a = sin(dlat/2)*sin(dlat/2) + cos(lat1)*cos(lat2)*sin(dlong/2)*sin(dlong/2);
  c = 2 * atan2( sqrt(a), sqrt(1-a) );
  distanceKm = R * c;
  
  double DistanceMeters= distanceKm*1000;
  return DistanceMeters;
  
  
}

int calculateangle( double lat1, double lat2, double long1, double long2){
  
  double  pi = 3.1415926535;
  double angle = 0;
  double degree;
  double dlong;
  double dlat;
  
  lat1 = lat1*(pi/180);
  lat2 = lat2*(pi/180);
  
  long1= long1*(pi/180);
  long2= long2*(pi/180);
  
  dlong = (long2 - long1);
  dlat  = (lat2 - lat1);
  
  angle =  atan2(sin(dlong)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(dlong));
  degree = round(angle*(180/pi));
  int degreerounded = degree;
  degreerounded = (degreerounded+360)%360;
  return degreerounded;
  
}
void Directions(double *Mylat, double *Mylong, char *in)
{
  char latitud[10];
  char longitud[10];
  
  strncpy(latitud,in+18,9);    //extracting latitud from current position;
  strncpy(longitud,in+30,10);  //extracting longitud
  
  *Mylong = atof(longitud)/100;
  *Mylat =  atof(latitud)/100;
  if(longitud[0]=='0')   // if longitud[0]==0,  this means the longitud is negative, thats why im multiplying by -1
  {
    *Mylong *= -1;
  }	
}


void GPSReceive(double *Mylat, double *Mylong, char *in){
  volatile HAL_StatusTypeDef status;
  uint8_t inSize;
receivebeg:
  
  while(in[0] != '$')
  {
    status = HAL_UART_Receive(&huart1, &in[0], 1, 0x100);
    if(status == HAL_TIMEOUT)
    {
      osDelay(200);
    }
  }
  // Receive
  
  for (inSize = 1; inSize < 40 && status != HAL_TIMEOUT; inSize++)
  {
    status = HAL_UART_Receive(&huart1, &in[inSize], 1, 0x100);
    
    if (in[inSize] == '\n')
      break;
  }
  
  
  HAL_UART_Transmit(&huart2, (uint8_t *)in, inSize-1, 0x100);
  
  if(in[3]== 'G' && in[4]=='G' && in[5]=='A')
  {
    Directions(Mylat,Mylong, in);
    
  }
  
  else
  {
    osDelay(200);
    in[0] = 0;
    goto receivebeg;
  } 
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
  uint16_t i;
  osDelay(500);
  for(;clockangle > 0; clockangle--)
  {
    for(i=35535; i>0; i--)
    {

    }
    osDelay(300);
  }
  osDelay(1000);
}

/*
TODO: 
interupt 1:
save current location

interupt 2:
get direction to hard saved location

interupt 3:
get direction to soft saved location
*/

void angleToLED(uint16_t angle)
{
  if(angle<90 || angle >270)
    osThreadResume(tLEDTaskHandle);
  if(angle < 180)
    osThreadResume(rLEDTaskHandle);
  if(angle > 180)
    osThreadResume(lLEDTaskHandle);
  if(angle > 90 && angle < 270)
    osThreadResume(bLEDTaskHandle);
}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  

  /* USER CODE BEGIN 5 */
  
  double softlat;
  double softlong;
  
  /*
  	Calibrate compass
  */
  
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
  
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
  
  
  /*
  	Initialize GPS
  */
  
      struct route1{
    
    double firstpointlat;
    double firstpointlong;
    double secondpointlat;
    double secondpointlong;
    double thirdpointlat;
    double thirdpointlong;
    
  }points;
  
  
  
  points.firstpointlat =   28.600640;
  points.firstpointlong = -81.196905;
  //points.firstpointlat =   28.600969;
  //points.firstpointlong = -81.197758;
  
  points.secondpointlat =  28.600642;
  points.secondpointlong = -81.196900;
  
  points.thirdpointlat   = 28.601333;
  points.thirdpointlong  = -81.196601;
  
  
  char in[39];
//char latitud[10];
  //char longitud[10];
  
  double  currentlatitud=0;
  double currentlongitud=0;
  char *pop = "Just testing123!\n\r";
  
  in[0] = '0';
  uint8_t * nl = " - \r\n";
  uint8_t * nl2= "\r";
  uint8_t inSize;
  
  int totaldistance =0;
  int  test=0;
  
  // Config GPS (GGA, .5Hz)
  uint8_t *onlygga ="$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29<CR><LF>";
  uint8_t *updatarate = "$PMTK220,2000*1C<CR><LF>";   //.5 times per second update  ---> .5Hz
  HAL_UART_Transmit(&huart1, (uint8_t *)onlygga, 13, 0x1000);
  HAL_UART_Transmit(&huart1, (uint8_t *)nl,1, 0x1000);
  
  //osThreadSuspend(defaultTaskHandle);
  for(;;)
  {
    /*
    if(!(blinking&224) || blinking|7)
    {
      osDelay(1000);
      continue;
    }
    */
    // get compass data
    compass_takemeasurement(inbuf, outbuf);
    compass_processInput(inbuf, xyz, minmaxxyz);
    compass_getAngle(&angle, xyz);
    
    // get gps data
    //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    GPSReceive(&currentlatitud, &currentlongitud,in);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    osDelay(100);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    osDelay(100);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    osDelay(100);
    
    // button 1: calculate hard angle - real angle
    if(blinking == 128)	// but 1
    {
      osDelay(100);
      int destAngle = calculateangle(currentlatitud, 28.600640, currentlongitud, -81.196905);
	destAngle = (angle>destAngle)?360-angle+destAngle:destAngle-angle;
	angleToLED(destAngle);
      
      blinking &= 127;
    }
    // button 2: calculate soft angle - real angle
    else if(blinking == 64)	// but 2
    {
      osDelay(100);
        int destAngle = calculateangle(currentlatitud, softlat, currentlongitud, softlong);
	destAngle = (angle>destAngle)?360-angle+destAngle:destAngle-angle;
	angleToLED(destAngle);
      blinking &= 191;
    }
    // button 3: save soft location
    else if(blinking == 32)	// but 3
    {
      osDelay(100);
      softlat = currentlatitud;
      softlong = currentlongitud;
      blinking = 0;
    }
    //else	osDelay(1000);
    osThreadSuspend(defaultTaskHandle);
  }
  
  /* USER CODE END 5 */ 
}

/* StartRLEDTask function */
void StartRLEDTask(void const * argument)
{
  /* USER CODE BEGIN StartRLEDTask */
  osThreadSuspend(rLEDTaskHandle);
  /* Infinite loop */
  for(;;)
  {
    blinking |= 1;
    for(uint8_t i = 0; i < 50; i++)
    {
      HAL_GPIO_WritePin(ledrig_GPIO_Port, ledrig_Pin, GPIO_PIN_RESET);
      osDelay(100);
      HAL_GPIO_WritePin(ledrig_GPIO_Port, ledrig_Pin, GPIO_PIN_SET);
      osDelay(100);
    }
    blinking &= 254;
    osThreadSuspend(rLEDTaskHandle);
    
  }
  /* USER CODE END StartRLEDTask */
}

/* StartLLEDTask function */
void StartLLEDTask(void const * argument)
{
  /* USER CODE BEGIN StartLLEDTask */
  osThreadSuspend(lLEDTaskHandle);
  /* Infinite loop */
  for(;;)
  {
    blinking |= 2;
    for(uint8_t i = 0; i < 50; i++)
    {
      HAL_GPIO_WritePin(ledlef_GPIO_Port, ledlef_Pin, GPIO_PIN_RESET);
      osDelay(100);
      HAL_GPIO_WritePin(ledlef_GPIO_Port, ledlef_Pin, GPIO_PIN_SET);
      osDelay(100);
    }
    blinking &= 253;
    osThreadSuspend(lLEDTaskHandle);
    
  }
  /* USER CODE END StartLLEDTask */
}

/* StartTLEDTask function */
void StartTLEDTask(void const * argument)
{
  /* USER CODE BEGIN StartTLEDTask */
  osThreadSuspend(tLEDTaskHandle);
  /* Infinite loop */
  for(;;)
  {
    blinking |= 4;
    for(uint8_t i = 0; i < 50; i++)
    {
      HAL_GPIO_WritePin(ledtop_GPIO_Port, ledtop_Pin, GPIO_PIN_RESET);
      osDelay(100);
      HAL_GPIO_WritePin(ledtop_GPIO_Port, ledtop_Pin, GPIO_PIN_SET);
      osDelay(100);
    }
    blinking &= 251;
    osThreadSuspend(tLEDTaskHandle);
    
  }
  /* USER CODE END StartTLEDTask */
}

/* StartBLEDTask function */
void StartBLEDTask(void const * argument)
{
  /* USER CODE BEGIN StartBLEDTask */
  osThreadSuspend(bLEDTaskHandle);
  /* Infinite loop */
  for(;;)
  {
    blinking |= 8;
    for(uint8_t i = 0; i < 50; i++)
    {
      HAL_GPIO_WritePin(ledbot_GPIO_Port, ledbot_Pin, GPIO_PIN_RESET);
      osDelay(100);
      HAL_GPIO_WritePin(ledbot_GPIO_Port, ledbot_Pin, GPIO_PIN_SET);
      osDelay(100);
    }
    blinking &= 247;
    osThreadSuspend(bLEDTaskHandle);
    
  }
  /* USER CODE END StartBLEDTask */
}

/* StartTHandlerTask function */
void StartTHandlerTask(void const * argument)
{
  /* USER CODE BEGIN StartTHandlerTask */
  osThreadSuspend(thandlerTaskHandle);
  /* Infinite loop */
  for(;;)
  {
    //if(!blinking) osThreadResume(defaultTaskHandle);
    osDelay(30000);
  }
  /* USER CODE END StartTHandlerTask */
}

/* StartCompassTask function */
void StartCompassTask(void const * argument)
{
  /* USER CODE BEGIN StartCompassTask */
  
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
  osThreadSuspend(compassTaskHandle);
  
  /* Infinite loop */
  for(;;)
  {
    compass_takemeasurement(inbuf, outbuf);
    compass_processInput(inbuf, xyz, minmaxxyz);
    compass_getAngle(&angle, xyz);
    //compass_vibrateAngle(&angle);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    //osThreadResume(defaultTaskHandle);
    //osThreadResume(laserTaskHandle);
    osThreadSuspend(compassTaskHandle);
  }
  /* USER CODE END StartCompassTask */
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
