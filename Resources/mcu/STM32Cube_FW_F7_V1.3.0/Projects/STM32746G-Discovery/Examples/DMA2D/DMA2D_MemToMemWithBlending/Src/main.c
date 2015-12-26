/**
  ******************************************************************************
  * @file    DMA2D/DMA2D_MemToMemWithBlending/Src/main.c
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    18-November-2015 
  * @brief   This example provides a description of how to configure
  *          DMA2D peripheral in Memory to Memory with blending transfer mode.
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

#include "main.h"

#include "RGB565_240x130_1.h"
#include "RGB565_240x130_2.h"

/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @addtogroup DMA2D_MemToMemWithBlending
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
DMA2D_HandleTypeDef Dma2dHandle;

/* DMA2D output address in SRAM : this is the buffer displayed on LCD screen */
/* The buffer in SRAM is 240x130x2 = 60 KBytes                               */
uint32_t aBlendedImage[(LAYER_SIZE_X * LAYER_SIZE_Y * LAYER_BYTE_PER_PIXEL) / 4];


/* Private function prototypes -----------------------------------------------*/
static void LCD_Config(void);

static void DMA2D_Config(void);
static void TransferError(DMA2D_HandleTypeDef* Dma2dHandle);
static void TransferComplete(DMA2D_HandleTypeDef* Dma2dHandle);
static void SystemClock_Config(void);
static void OnError_Handler(uint32_t condition);
static void MPU_Config(void);
static void CPU_CACHE_Enable(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
  HAL_StatusTypeDef hal_status = HAL_OK;
  /* Configure the MPU attributes as Write Through */
  MPU_Config();

  /* Enable the CPU Cache */
  CPU_CACHE_Enable();
  
  /* STM32F7xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 216 MHz */
  SystemClock_Config();


  
  /*## LTDC Clock Configuration ###########################################*/
  /* LCD clock configuration */
  /* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
  /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 192 Mhz */
  /* PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 192/5 = 38.4 Mhz */
  /* LTDC clock frequency = PLLLCDCLK / LTDC_PLLSAI_DIVR_4 = 38.4/4 = 9.6Mhz */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_4;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
    
  /* Configure LED1 */
  BSP_LED_Init(LED1);

  /*##-1- LCD Configuration ##################################################*/
  LCD_Config();

  /*##-2- Configure DMA2D : Configure foreground and background ##############*/
  DMA2D_Config();

  /*##-3- Start DMA2D transfer in interrupt mode ################################################*/
  /*## RGB565_240x130_1[] is the foreground layer and RGB565_240x130_2[] is the background layer */
  hal_status = HAL_DMA2D_BlendingStart_IT(&Dma2dHandle,
                                          (uint32_t)&RGB565_240x130_1,
                                          (uint32_t)&RGB565_240x130_2,
                                          (uint32_t)&aBlendedImage,
                                          LAYER_SIZE_X,
                                          LAYER_SIZE_Y);
  OnError_Handler(hal_status != HAL_OK);


  while (1)
  {
   ;
  }
}


/**
  * @brief DMA2D configuration.
  * @note  This function Configure the DMA2D peripheral :
  *        1) Configure the Transfer mode as memory to memory with blending.
  *        2) Configure the output color mode as RGB565 pixel format.
  *        3) Configure the Foreground
  *          - Foreground image is loaded from FLASH memory (RGB565_240x130_2[])
  *          - constant alpha value (decreased to see the background)
  *          - color mode as RGB565 pixel format
  *        4) Configure the Background
  *          - Background image loaded from FLASH memory (RGB565_240x130_1[])
  *          - color mode as RGB565 pixel format
  * @retval
  *  None
  */
static void DMA2D_Config(void)
{
  HAL_StatusTypeDef hal_status = HAL_OK;

  /* Configure the DMA2D Mode, Color Mode and output offset */
  Dma2dHandle.Init.Mode         = DMA2D_M2M_BLEND; /* DMA2D mode Memory to Memory with Blending */
  Dma2dHandle.Init.ColorMode    = DMA2D_RGB565; /* output format of DMA2D */
  Dma2dHandle.Init.OutputOffset = 0x0;  /* No offset in output */

  /* DMA2D Callbacks Configuration */
  Dma2dHandle.XferCpltCallback  = TransferComplete;
  Dma2dHandle.XferErrorCallback = TransferError;

  /* Foreground layer Configuration */
  Dma2dHandle.LayerCfg[1].AlphaMode = DMA2D_REPLACE_ALPHA;
  Dma2dHandle.LayerCfg[1].InputAlpha = 0x7F; /* 127 : semi-transparent */
  Dma2dHandle.LayerCfg[1].InputColorMode = CM_RGB565;
  Dma2dHandle.LayerCfg[1].InputOffset = 0x0; /* No offset in input */

  /* Background layer Configuration */
  Dma2dHandle.LayerCfg[0].AlphaMode = DMA2D_REPLACE_ALPHA;
  Dma2dHandle.LayerCfg[0].InputAlpha = 0x7F; /* 127 : semi-transparent */
  Dma2dHandle.LayerCfg[0].InputColorMode = CM_RGB565;
  Dma2dHandle.LayerCfg[0].InputOffset = 0x0; /* No offset in input */

  Dma2dHandle.Instance = DMA2D;

  /* DMA2D Initialization */
  hal_status = HAL_DMA2D_Init(&Dma2dHandle);
  OnError_Handler(hal_status != HAL_OK);

  /* Apply DMA2D Foreground configuration */
  HAL_DMA2D_ConfigLayer(&Dma2dHandle, 1);

  /* Apply DMA2D Background configuration */
  HAL_DMA2D_ConfigLayer(&Dma2dHandle, 0);
}

/**
  * @brief  On Error Handler on condition TRUE.
  * @param  condition : Can be TRUE or FALSE
  * @retval None
  */
static void OnError_Handler(uint32_t condition)
{
  if(condition)
  {
    while(1)
    {
      /* Toggle LED1 with a period of 200 ms */
      BSP_LED_Toggle(LED1);
      HAL_Delay(200);
    }
  }
}

/**
  * @brief LCD configuration.
  * @note  This function Configure the LTDC peripheral to display output of DMA2D operation on glass.
  *        That is an image of size 240x130 in format RGB565 from buffer in internal SRAM aBlendedImage[].
  *        1) Configure the Pixel Clock for the LCD
  *        2) Configure the LTDC Timing and Polarity
  *        3) Configure the LTDC Layer 2 :
  *           - RGB565 as pixel format
  *           - The frame buffer is located at internal SRAM : The output of DMA2D transfer
  *             which is aBlendedImage[].
  *           - The Layer size configuration : 240x130
  * @retval
  *  None
  */
static void LCD_Config(void)
{
  static LTDC_HandleTypeDef        hltdc_F;
  static LTDC_LayerCfgTypeDef      pLayerCfg;
  HAL_StatusTypeDef hal_status = HAL_OK;

/* LTDC Initialization -------------------------------------------------------*/

  /* Polarity configuration */
  /* Initialize the horizontal synchronization polarity as active low */
  hltdc_F.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  /* Initialize the vertical synchronization polarity as active low */
  hltdc_F.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  /* Initialize the data enable polarity as active low */
  hltdc_F.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  /* Initialize the pixel clock polarity as input pixel clock */
  hltdc_F.Init.PCPolarity = LTDC_PCPOLARITY_IPC;

  /* Timing configuration for RK043FN48H 480x272 LCD */
  /* Horizontal synchronization width = Hsync - 1 */
  hltdc_F.Init.HorizontalSync = 40;
  /* Vertical synchronization height = Vsync - 1 */
  hltdc_F.Init.VerticalSync = 9;
  /* Accumulated horizontal back porch = Hsync + HBP - 1 */
  hltdc_F.Init.AccumulatedHBP = 53;
  /* Accumulated vertical back porch = Vsync + VBP - 1 */
  hltdc_F.Init.AccumulatedVBP = 11;
  /* Accumulated active width = Hsync + HBP + Active Width - 1 */
  hltdc_F.Init.AccumulatedActiveH = 283;
  /* Accumulated active height = Vsync + VBP + Active Height - 1 */
  hltdc_F.Init.AccumulatedActiveW = 533;
  /* Total height = Vsync + VBP + Active Height + VFP - 1 */
  hltdc_F.Init.TotalHeigh = 285;
  /* Total width = Hsync + HBP + Active Width + HFP - 1 */
  hltdc_F.Init.TotalWidth = 565;

  /* Configure R,G,B component values for LCD background color */
  hltdc_F.Init.Backcolor.Blue = 0;
  hltdc_F.Init.Backcolor.Green = 0;
  hltdc_F.Init.Backcolor.Red = 0;

  hltdc_F.Instance = LTDC;

  /* Layer1 Configuration ------------------------------------------------------*/

  /* Windowing configuration */
  /*
     WindowX0 = Horizontal start
     WindowX1 = Horizontal stop
     WindowY0 = Vertical start
     WindowY1 = Vertical stop
     Display image 240x130 at start position on display (120, 70).
  */
  pLayerCfg.WindowX0 = 120;
  pLayerCfg.WindowX1 = 360;
  pLayerCfg.WindowY0 = 70;
  pLayerCfg.WindowY1 = 200;

  /* Pixel Format configuration */
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;

  /* Start Address configuration : frame buffer is located in SRAM memory */
  /* Output of DMA2D operation */
  pLayerCfg.FBStartAdress = (uint32_t)&aBlendedImage;

  /* Alpha constant (255 = totally opaque) */
  pLayerCfg.Alpha = 255;

  /* Default Color configuration (configure A,R,G,B component values) */
  pLayerCfg.Alpha0 = 0; /* fully transparent */
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;

  /* Configure blending factors */
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;

  /* Configure the number of lines and number of pixels per line : 240x130 */
  pLayerCfg.ImageWidth  = LAYER_SIZE_X;
  pLayerCfg.ImageHeight = LAYER_SIZE_Y;

  /* Configure the LTDC */
  hal_status  = HAL_LTDC_Init(&hltdc_F);
  OnError_Handler(hal_status != HAL_OK);

  /* Assert display enable LCD_DISP pin */
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_SET);

  /* Assert backlight LCD_BL_CTRL pin */
  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_3, GPIO_PIN_SET);
  /* Configure the single Layer 1 */
  hal_status  = HAL_LTDC_ConfigLayer(&hltdc_F, &pLayerCfg, 1);
  OnError_Handler(hal_status != HAL_OK);
}
/**
  * @brief  DMA2D Transfer completed callback
  * @param  hdma2d: DMA2D handle.
  * @note   This example shows a simple way to report end of DMA2D transfer, and
  *         you can add your own implementation.
  * @retval None
  */
static void TransferComplete(DMA2D_HandleTypeDef *hdma2d)
{
  /* Turn LED1 On */
  BSP_LED_On(LED1);
}

/**
  * @brief  DMA2D error callbacks
  * @param  hdma2d: DMA2D handle
  * @note   This example shows a simple way to report DMA2D transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
static void TransferError(DMA2D_HandleTypeDef *hdma2d)
{
  while (1)
  {
    /* Toggle LED1 with a period of 1 s */
    BSP_LED_Toggle(LED1);
    HAL_Delay(1000);
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
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* Activate the OverDrive to reach the 216 MHz Frequency */  
  ret = HAL_PWREx_EnableOverDrive();
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
  
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  if(ret != HAL_OK)
  {
    while(1) { ; }
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
  * @brief  Configure the MPU attributes as Write Through for SRAM1/2.
  * @note   The Base Address is 0x20010000 since this memory interface is the AXI.
  *         The Region Size is 256KB, it is related to SRAM1 and SRAM2  memory size.
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;
  
  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU attributes as WT for SRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x20010000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

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

/**
  * @}
  */

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
