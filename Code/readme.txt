LED:
HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_2, GPIO_PIN_RESET);
  
  
  
UART transmission:
char* msg6 = "Hello world6\n\r";
HAL_UART_Transmit(&huart6, (uint8_t*)msg6, strlen(msg6), 0xFFFF);




FreeRTOS:

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
