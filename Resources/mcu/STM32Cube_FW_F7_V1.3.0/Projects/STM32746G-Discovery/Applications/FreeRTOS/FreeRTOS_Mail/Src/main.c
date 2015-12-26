/**
  ******************************************************************************
  * @file    FreeRTOS/FreeRTOS_Mail/Src/main.c
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    18-November-2015 
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct 
{ /* Mail object structure */
  uint32_t var1; /* var1 is a uint32_t */
  uint32_t var2; /* var2 is a uint32_t */
  uint8_t var3; /* var3 is a uint8_t */
} Amail_TypeDef;

/* Private define ------------------------------------------------------------*/
#define blckqSTACK_SIZE   configMINIMAL_STACK_SIZE
#define MAIL_SIZE        (uint32_t) 1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
osMailQId mailId;

uint32_t ProducerValue1 = 0, ProducerValue2 = 0;
uint8_t ProducerValue3 = 0;
uint32_t ConsumerValue1 = 0, ConsumerValue2 = 0;
uint8_t ConsumerValue3 = 0;

/* Private function prototypes -----------------------------------------------*/

/* Thread function that creates a mail and posts it on a mail queue. */
static void MailQueueProducer (const void *argument);

/* Thread function that receives mail , remove it  from a mail queue and checks that
it is the expected mail. */
static void MailQueueConsumer (const void *argument);

static void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* Enable the CPU Cache */
  CPU_CACHE_Enable();

  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator on ITCM interface
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();  
  
  /* Configure the system clock to 216 Mhz */
  SystemClock_Config();
  
  /* Initialize LED1 */
  BSP_LED_Init(LED1);
  
  /* Create the mail queue used by the two tasks to pass the struct Amail_TypeDef */
  osMailQDef(mail, MAIL_SIZE, Amail_TypeDef); /* Define mail queue */
  
  mailId = osMailCreate(osMailQ(mail), NULL); /* create mail queue */
  
  /* Note the producer has a lower priority than the consumer when the tasks are
     spawned. */
  osThreadDef(QCons, MailQueueConsumer, osPriorityBelowNormal, 0, blckqSTACK_SIZE);
  osThreadCreate(osThread(QCons), NULL);
  
  osThreadDef(QProd, MailQueueProducer, osPriorityBelowNormal, 0, blckqSTACK_SIZE);
  osThreadCreate(osThread(QProd), NULL);
  
  /* Start scheduler */
  osKernelStart ();

  /* We should never get here as control is now taken by the scheduler */
  for(;;);
}

/**
  * @brief  Mail Producer Thread.
  * @param  argument: Not used
  * @retval None
  */
static void MailQueueProducer(const void *argument)
{
  Amail_TypeDef *pTMail;
  
  for(;;)
  {	

    pTMail = osMailAlloc(mailId, osWaitForever); /* Allocate memory */
    pTMail->var1 = ProducerValue1; /* Set the mail content */
    pTMail->var2 = ProducerValue2;
    pTMail->var3 = ProducerValue3;
    
    if(osMailPut(mailId, pTMail) != osOK) /* Send Mail */  
    {
      /* LED1 is turned On to indicate error */
      BSP_LED_On(LED1);
    }
    else
    {
      /* Increment the variables we are going to post next time round.  The
      consumer will expect the numbers to follow in numerical order. */
      ++ProducerValue1;
      ProducerValue2 += 2;
      ProducerValue3 += 3;
      
      /* Toggle LED1 to indicate a correct number received  */
      BSP_LED_Toggle(LED1);

      osDelay(250);
    }
  }
}

/**
  * @brief  Mail Consumer Thread.
  * @param  argument: Not used
  * @retval None
  */
static void MailQueueConsumer (const void *argument)
{
  osEvent event;
  Amail_TypeDef *pRMail;
  
  for(;;)
  {
    /* Get the message from the queue */
    event = osMailGet(mailId, osWaitForever); /* wait for mail */
    
    if(event.status == osEventMail)
    {
      pRMail = event.value.p;
      
      if((pRMail->var1 != ConsumerValue1) || (pRMail->var2 != ConsumerValue2) || (pRMail->var3 != ConsumerValue3))
      {
        /* Catch-up. */
        ConsumerValue1 = pRMail->var1;
        ConsumerValue2 = pRMail->var2;
        ConsumerValue3 = pRMail->var3;
        
        /* LED1 is turned On to indicate error */
        BSP_LED_On(LED1);
      }
      else
      {  
        /* Calculate values we expect to remove from the mail queue next time
        round. */
        ++ConsumerValue1;
        ConsumerValue2 += 2;
        ConsumerValue3 += 3;        
      }

      osMailFree(mailId, pRMail); /* free memory allocated for mail */       
    }		
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);

}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
