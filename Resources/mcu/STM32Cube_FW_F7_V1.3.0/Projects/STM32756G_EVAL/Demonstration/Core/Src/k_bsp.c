/**
  ******************************************************************************
  * @file    k_bsp.c
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    18-November-2015  
  * @brief   This file provides the kernel bsp functions
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
#include "k_bsp.h"
#include "GUI.h"
#include "DIALOG.h"

/** @addtogroup CORE
  * @{
  */

/** @defgroup KERNEL_BSP
  * @brief Kernel BSP routines
  * @{
  */

/* External variables --------------------------------------------------------*/
  extern uint8_t SelLayer;

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes LEDs, SDRAM, touch screen, CRC and SRAM.
  * @param  None 
  * @retval None
  */
void k_BspInit(void)
{
  
  BSP_IO_Init();
  
  /* Initialize the NOR */
  BSP_NOR_Init();
  
  /* Initialize the SDRAM */
  BSP_SDRAM_Init();  
  
  /* Initialize the Touch screen */
  BSP_TS_Init(640, 480);
  
  /* Enable CRC to Unlock GUI */
 __HAL_RCC_CRC_CLK_ENABLE();
  
 /* Enable Back up SRAM */
__HAL_RCC_BKPSRAM_CLK_ENABLE();
  
}

/**
  * @brief  Read the coordinate of the point touched and assign their
  *         value to the variables u32_TSXCoordinate and u32_TSYCoordinate
  * @param  None
  * @retval None
  */
void k_TouchUpdate(void)
{
  static GUI_PID_STATE TS_State = {0, 0, 0, 0};
  __IO TS_StateTypeDef  ts;
  uint16_t xDiff, yDiff;  

  BSP_TS_GetState((TS_StateTypeDef *)&ts);

  if((ts.x >= LCD_GetXSize()) ||(ts.y >= LCD_GetYSize()) ) 
  {
    ts.x = 0;
    ts.y = 0;
    ts.TouchDetected =0;
  }

  xDiff = (TS_State.x > ts.x) ? (TS_State.x - ts.x) : (ts.x - TS_State.x);
  yDiff = (TS_State.y > ts.y) ? (TS_State.y - ts.y) : (ts.y - TS_State.y);
  
  
  if((TS_State.Pressed != ts.TouchDetected ) ||
     (xDiff > 30 )||
      (yDiff > 30))
  {
    TS_State.Pressed = ts.TouchDetected;
    TS_State.Layer = SelLayer;
    if(ts.TouchDetected) 
    {
      TS_State.x = ts.x;
      TS_State.y = ts.y;
      GUI_TOUCH_StoreStateEx(&TS_State);
    }
    else
    {
      GUI_TOUCH_StoreStateEx(&TS_State);
      TS_State.x = 0;
      TS_State.y = 0;
    }
  }
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
