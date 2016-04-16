LED:
HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET);
  
  
  
UART transmission:
Discovery board: use PB7 and PA9 with USART1
Nucleo: use PA2 and PA3

Nucleo has USART1 on PA9 and PA10

char* msg = "Hello world\n\r";
HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg1), 0xFFFF);

or create a function such as:
void printUART(char * msg)
{
	HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), 0xFFFF);
}




FreeRTOS:

change stm32f7xx_it.c to:
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

Create a thread: put the following after preprocessor:
osThreadId tNameThreadHandle;

Then the following before main():
static void tNameThread(void const *argument);

and add these to the main function prior to osKernelStart():
osThreadDef(tName, tNameThread, osPriorityLow, 0, configMINIMAL_STACK_SIZE);
tNameThreadHandle = osThreadCreate(osThread(tName), NULL);

Finally create a new thread "function":
static void tNameThread(void const *argument)
{
	//initialization
	for(;;)
	{
		// infinite loop
		osDelay(10); // delay if necessary
	}
}

eg:
osThreadId hearbeatThreadHandle;
void heartbeatThread(void const * argument);
osThreadDef(heartbeat, heartbeatThread, osPriorityLow, 0, configMINIMAL_STACK_SIZE);
heartbeatThreadHandle = osThreadCreate(osThread(heartbeat),NULL);
void heartbeatThread(void const *argument)
{
	char* pulse = "Still beating.\n\r";
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	for(;;)
	{
		HAL_UART_Transmit(&huart6, (uint8_t*)pulse, strlen(pulse), 0xFFFF);
		HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_6);
		osDelay(10); // delay if necessary
	}
}


