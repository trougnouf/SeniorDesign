/**
  ******************************************************************************
  * @file    Audio/Audio_playback_and_record/Src/main.c
  * @author  MCD Application Team
  * @version V1.0.3
  * @date    18-November-2015 
  * @brief   Audio playback and record main file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
#include "waveplayer.h"
#include "waverecorder.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USBH_HandleTypeDef hUSBHost;
AUDIO_ApplicationTypeDef appli_state = APPLICATION_IDLE;
static volatile uint32_t MfxToggleLed = 0;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);
static void AUDIO_InitApplication(void);
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
       - Global MSP (MCU Support Package) initialization
     */   
  HAL_Init();
  
  /* Configure the system clock to 200 MHz */
  SystemClock_Config(); 

  /* Init Audio Application */
  AUDIO_InitApplication();

  /* Init Host Library */
  USBH_Init(&hUSBHost, USBH_UserProcess, 0);

  /* Add Supported Class */
  USBH_RegisterClass(&hUSBHost, USBH_MSC_CLASS);
  
  /* Start Host Process */
  USBH_Start(&hUSBHost);
  
  /* Run Application (Blocking mode) */
  while (1)
  {
    /* USB Host Background task */
    USBH_Process(&hUSBHost);
    
    /* AUDIO Menu Process */
    AUDIO_MenuProcess();

    if ( MfxToggleLed == 1)
    { 
#if defined(USE_STM32756G_EVAL_REVA)
      /* On RevA board, as LED1 is connected to MFX, the toggle is performed in main loop */
      BSP_LED_Toggle(LED1);
#else
      /* On RevB board, as LED1 is connected to GPIO, it is toggled in ISR,
       * the LED4 is toggled in main loop as it is connected to MFX */
      BSP_LED_Toggle(LED4);
#endif
      MfxToggleLed = 0;
    }
  }
}

/**
  * @brief  Toggles LEDs to show user input state.
  * @param  None
  * @retval None
  */
void Toggle_Leds(void)
{
  static uint32_t ticks = 0;
  
  if(ticks++ > 400)
  {
    /* Led2 and Led4 are on MFX (IO expander). Communication with Mfx is done by I2C. 
    Often the sw requires ISRs (irq service routines) to be quick while communication 
    with I2C can be considered relatively long (hundreds of usec depending on I2C clk). 
    In order to avoid blocking I2C communication on interrupt service routines 
    here we propose an example of implementation which is a mix between pooling and exit:
    On ISR a flag is set (MfxToggleLed), the main loop polls on the flag;
    Mcu communicates with Mfx only when the flag has been set. */
    MfxToggleLed = 1;
#if !defined(USE_STM32756G_EVAL_REVA)
    /* NOTE: Led1 and Led3 can toggle on SysTick without performance drawback
             because they are plugged on GPIO pins */
    BSP_LED_Toggle(LED1); 		
#endif
    ticks = 0;
  }
}

/*******************************************************************************
                            Static Function
*******************************************************************************/

/**
  * @brief  Audio Application Init.
  * @param  None
  * @retval None
  */
static void AUDIO_InitApplication(void)
{
  /* Configure Key Button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);              
  
  /* Configure IO and LED1 */
  BSP_IO_Init();
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED4);
 
  /* Configure Joystick in EXTI mode */
  BSP_JOY_Init(JOY_MODE_EXTI);
  
  /* Camera has to be powered down as some signals use same GPIOs between
   * I2S signals and camera bus. Camera drives its signals to low impedance
   * when powered ON. So the camera is powered off to let its signals
   * in high impedance */

  /* Camera power down sequence */
  BSP_IO_ConfigPin(RSTI_PIN, IO_MODE_OUTPUT);
  BSP_IO_ConfigPin(XSDN_PIN, IO_MODE_OUTPUT);

  /* De-assert the camera STANDBY pin (active high) */
  BSP_IO_WritePin(XSDN_PIN, BSP_IO_PIN_RESET);

  /* Assert the camera RSTI pin (active low) */
  BSP_IO_WritePin(RSTI_PIN, BSP_IO_PIN_RESET);

  /* Initialize the LCD */
  BSP_LCD_Init();
  
  /* LCD Layer Initialization */
  BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS); 
  
  /* Select the LCD Layer */
  BSP_LCD_SelectLayer(1);
  
  /* Enable the display */
  BSP_LCD_DisplayOn();
  
  /* Init the LCD Log module */
  LCD_LOG_Init();
  
  LCD_LOG_SetHeader((uint8_t *)"Audio Playback and Record Application");
  
  LCD_UsrLog("USB Host library started.\n"); 
  
  /* Start Audio interface */
  USBH_UsrLog("Starting Audio Demo");
  
  /* Init Audio interface */
  AUDIO_PLAYER_Init();
  
  /* Start Audio interface */
  AUDIO_MenuInit();
}

/**
  * @brief  User Process
  * @param  phost: Host Handle
  * @param  id: Host Library user message ID
  * @retval None
  */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id)
{
  switch(id)
  { 
  case HOST_USER_SELECT_CONFIGURATION:
    break;
    
  case HOST_USER_DISCONNECTION:
    appli_state = APPLICATION_DISCONNECT;
    break;

  case HOST_USER_CLASS_ACTIVE:
    appli_state = APPLICATION_READY;
    break;
 
  case HOST_USER_CONNECTION:
    appli_state = APPLICATION_START;
    break;
   
  default:
    break; 
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 200000000
  *            HCLK(Hz)                       = 200000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 400
  *            PLL_P                          = 2
  *            PLL_Q                          = 8
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 6
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;

  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Activate the OverDrive to reach the 200 MHz Frequency */
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Select PLLSAI output as USB clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 4; 
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
  ret = HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
}

/**
  * @brief  Clock Config.
  * @param  hsai: might be required to set audio peripheral predivider if any.
  * @param  AudioFreq: Audio frequency used to play the audio stream.
  * @note   This API is called by BSP_AUDIO_OUT_Init() and BSP_AUDIO_OUT_SetFrequency()
  *         Being __weak it can be overwritten by the application     
  * @retval None
  */
void BSP_AUDIO_OUT_ClockConfig(SAI_HandleTypeDef *hsai, uint32_t AudioFreq, void *Params)
{
  RCC_PeriphCLKInitTypeDef RCC_ExCLKInitStruct;

  HAL_RCCEx_GetPeriphCLKConfig(&RCC_ExCLKInitStruct);
  
  /* Set the PLL configuration according to the audio frequency */
  if((AudioFreq == AUDIO_FREQUENCY_11K) || (AudioFreq == AUDIO_FREQUENCY_22K) || (AudioFreq == AUDIO_FREQUENCY_44K))
  {
    /* Configure PLLSAI prescalers */
    /* PLLI2S_VCO: VCO_429M
    SAI_CLK(first level) = PLLI2S_VCO/PLLSAIQ = 429/2 = 214.5 Mhz
    SAI_CLK_x = SAI_CLK(first level)/PLLI2SDivQ = 214.5/19 = 11.289 Mhz */
    RCC_ExCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
    RCC_ExCLKInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S;
    RCC_ExCLKInitStruct.PLLI2S.PLLI2SP = 8;
    RCC_ExCLKInitStruct.PLLI2S.PLLI2SN = 429;
    RCC_ExCLKInitStruct.PLLI2S.PLLI2SQ = 2;
    RCC_ExCLKInitStruct.PLLI2SDivQ = 19;
    HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct);
  }
  else /* AUDIO_FREQUENCY_8K, AUDIO_FREQUENCY_16K, AUDIO_FREQUENCY_48K), AUDIO_FREQUENCY_96K */
  {
    /* SAI clock config
    PLLI2S_VCO: VCO_344M
    SAI_CLK(first level) = PLLI2S_VCO/PLLSAIQ = 344/7 = 49.142 Mhz
    SAI_CLK_x = SAI_CLK(first level)/PLLI2SDivQ = 49.142/1 = 49.142 Mhz */
    RCC_ExCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
    RCC_ExCLKInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S;
    RCC_ExCLKInitStruct.PLLI2S.PLLI2SP = 8;
    RCC_ExCLKInitStruct.PLLI2S.PLLI2SN = 344;
    RCC_ExCLKInitStruct.PLLI2S.PLLI2SQ = 7;
    RCC_ExCLKInitStruct.PLLI2SDivQ = 1;
    HAL_RCCEx_PeriphCLKConfig(&RCC_ExCLKInitStruct);
  }
}

/**
  * @brief  Clock Config.
  * @param  hltdc: LTDC handle
  * @note   This API is called by BSP_LCD_Init()
  * @retval None
  */
void BSP_LCD_ClockConfig(LTDC_HandleTypeDef *hltdc, void *Params)
{
  static RCC_PeriphCLKInitTypeDef  periph_clk_init_struct;

  if(stmpe811_ts_drv.ReadID(TS_I2C_ADDRESS) == STMPE811_ID)
  {
    /* AMPIRE480272 LCD clock configuration */
    /* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
    /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 192 Mhz */
    /* PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 192/5 = 38.4 Mhz */
    /* LTDC clock frequency = PLLLCDCLK / LTDC_PLLSAI_DIVR_4 = 38.4/4 = 9.6Mhz */
    periph_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
    periph_clk_init_struct.PLLSAI.PLLSAIN = 192;
    periph_clk_init_struct.PLLSAI.PLLSAIR = AMPIRE480272_FREQUENCY_DIVIDER;
    periph_clk_init_struct.PLLSAIDivR = RCC_PLLSAIDIVR_4;
    HAL_RCCEx_PeriphCLKConfig(&periph_clk_init_struct);
  }
  else
  {
    /* AMPIRE640480 LCD clock configuration */
    /* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
    /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 192 Mhz */
    /* PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 192/4 = 48 Mhz */
    /* LTDC clock frequency = PLLLCDCLK / LTDC_PLLSAI_DIVR_2 = 48/2 = 24 Mhz */
    periph_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
    periph_clk_init_struct.PLLSAI.PLLSAIN = 192;
    periph_clk_init_struct.PLLSAI.PLLSAIR = 4;
    periph_clk_init_struct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
    HAL_RCCEx_PeriphCLKConfig(&periph_clk_init_struct);
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
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
  {
  }
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
