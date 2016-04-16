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
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int calculateditance(double lat1, double lat2, double long1, double long2);
int calculateangle( double lat1, double lat2, double long1, double long2);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  
  /* USER CODE BEGIN 2 */
  
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
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);
  
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
  
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 3, 0);
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
  huart2.Init.BaudRate = 57600;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  
}

/* USER CODE BEGIN 4 */

int calculateditance(double lat1, double lat2, double long1, double long2){
  double  pi = 3.1415926535;
  double distanceKm,distanceMiles,a,c;
  
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
  
  volatile double DistanceMeters= distanceKm*1000;
  
  
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
  
  //printf(" The angle degree  is %d\n",degreerounded);
  return degreerounded;
  
}


/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  
  /* USER CODE BEGIN 5 */
  struct route1{
    
    double firstpointlat;
    double firstpointlong;
    double secondpointlat;
    double secondpointlong;
    double thirdpointlat;
    double thirdpointlong;
    
  }points;
  
  
  points.firstpointlat =   28.600969;
  points.firstpointlong = -81.197758;
  
  points.secondpointlat =  28.600642;
  points.secondpointlong = -81.196900;
  
  points.thirdpointlat   = 28.601333;
  points.thirdpointlong  = -81.196601;
  
  
  
  
  
  
  
  
  volatile HAL_StatusTypeDef status;
  char in[39];
  
  
  char latitud[10];
  char longitud[10];
  
  double  currentlatitud=0;
  double currentlongitud=0;
  
  
  
  
  char *pop = "Just testing123!\n\r";
  
  
  
  in[0] = '0';
  uint8_t * nl = " - \r\n";
  uint8_t * nl2= "\r";
  uint8_t inSize;
  
  // Config GPS (GGA, .5Hz)
  uint8_t *onlygga ="$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29<CR><LF>";
  uint8_t *updatarate = "$PMTK220,2000*1C<CR><LF>";   //5 times per second update  ---> 5Hz
  HAL_UART_Transmit(&huart1, (uint8_t *)onlygga, 13, 0x1000);
  HAL_UART_Transmit(&huart1, (uint8_t *)nl,1, 0x1000);
  
  
  for(;;)
  {
    while(in[0] != '$')
    {
      status = HAL_UART_Receive(&huart1, &in[0], 1, 0x100);
      if(status == HAL_TIMEOUT)
	osDelay(500);
    }
    // Receive
    
    
    
    for (inSize = 1; inSize < 40 && status != HAL_TIMEOUT; inSize++)
    {
      status = HAL_UART_Receive(&huart1, &in[inSize], 1, 0x100);
      
      if (in[inSize] == '\n')
	
	break;
    }
    
    
    //HAL_UART_Transmit(&huart2, (uint8_t *)pop, 13, 0x1000);
    
    HAL_UART_Transmit(&huart2, (uint8_t *)in, inSize-1, 0x100);
    
    
    
    
    if(in[3]== 'G' && in[4]=='G' && in[5]=='A')
    {
      
      
      
      
      strncpy(latitud,in+18,9);    //extracting latitud from current position;
      strncpy(longitud,in+30,10);  //extracting longitud
      
      currentlongitud = atof(longitud)/100;
      currentlatitud = atof(latitud)/100;
      if(longitud[0]=='0')   // if longitud[0]==0,  this means the longitud is negative, thats why im multiplying by -1
      {
	
	currentlongitud *= -1;
	
	
      }	
      
    }
    
    else
    {
      osDelay(2000);
      continue;
    }
    
    
    
      double  approximation =6;
      
      //calculate distance from current location to the fisrt coordinate
      int distancetofirstpoint = calculateditance(currentlatitud,points.firstpointlat,currentlongitud,points.firstpointlong);
      int angleToFirstPoint = calculateangle(currentlatitud,points.firstpointlat,currentlongitud,points.firstpointlong);
      if(distancetofirstpoint <= approximation){
	// check next direction
	printf("close to point one");

    
  }

  
  
  
  //???
  {
    
    
    HAL_UART_Transmit(&huart2, (uint8_t *)nl,1, 0x1000);
    HAL_UART_Transmit(&huart2, (uint8_t *)latitud,9, 0x100);
    HAL_UART_Transmit(&huart2, (uint8_t *)nl,1, 0x1000);
    HAL_UART_Transmit(&huart2, (uint8_t *)nl,1, 0x1000);
    HAL_UART_Transmit(&huart2, (uint8_t *)longitud,10, 0x100);
    HAL_UART_Transmit(&huart2, (uint8_t *)nl,1, 0x1000);
    //HAL_UART_Transmit(&huart2, (uint8_t *)s,2, 0x100);
    HAL_UART_Transmit(&huart2, (uint8_t *)nl,1, 0x1000);
    //HAL_UART_Transmit(&huart2, (uint8_t *)lat2,2, 0x1000);
    //HAL_UART_Transmit(&huart2,(uint32_t *)lat2,7, 0x1000);
    //HAL_UART_Transmit(&huart2, (uint32_t *)distanceKm,15, 0x1000);
  }
  
  in[0] = '0';
  osDelay(150);
  
  
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
