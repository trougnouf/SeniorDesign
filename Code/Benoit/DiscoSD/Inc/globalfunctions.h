#ifndef __globalfunctions_H
#define __globalfunctions_H

static void MX_USART1_UART_Init(void);
static void strToUART(char * str);
UART_HandleTypeDef huart1;

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart1);

}

static void strToUART(char * msg)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 0xFFFF);
}

#endif
