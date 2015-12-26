/**
  ******************************************************************************
  * @file    USB_Host/DualCore_Standalone/Src/dual_menu.c 
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    18-November-2015 
  * @brief   Dual Core Process
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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t prev_select = 0;
DEMO_StateMachine demo;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Menu Init
  * @param  None
  * @retval None
  */
void DUAL_MenuInit(void)
{
  /* Start HID Interface */
  LCD_UsrLogY("USB Host Full speed initialized.\n");
  /* Start MSC Interface */
  LCD_UsrLogY("USB Host High speed initialized.\n");
}

/**
  * @brief  Manages DUAL Menu Process.
  * @param  None
  * @retval None
  */
void DUAL_MenuProcess(void)
{
  switch(demo.state)
  {
  case DEMO_IDLE:
    LCD_LOG_ClearTextZone();
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    LCD_UsrLog("Select an operation to continue:\n");
    
    BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
    BSP_LCD_DisplayStringAtLine(19, (uint8_t *)" 1 - Press User button to start HID demo");
    BSP_LCD_DisplayStringAtLine(20, (uint8_t *)" 2 - Press and hold User button to start MSC demo");
    
    /* Prevent debounce effect for user key */
    HAL_Delay(400);
    
    demo.state = DEMO_WAIT;
    break;
    
  case DEMO_WAIT:
    /* Check if User button is pressed */
    if(BSP_PB_GetState(BUTTON_TAMPER) == GPIO_PIN_SET)
    {
      /* To start MSC demo the User button should be kept pressed at least 2s */
      HAL_Delay (500);
      
      if(BSP_PB_GetState(BUTTON_TAMPER) == GPIO_PIN_SET)
      {
        demo.state = DEMO_MSC_START;
        while(BSP_PB_GetState(BUTTON_TAMPER) != RESET)
        {
        }
      }
      else
      {
        demo.state = DEMO_HID_START;
      }
      LCD_LOG_ClearTextZone();
    }
    break;
    
  case DEMO_MSC_START:
    if(Appli_HS_state == APPLICATION_HS_READY) 
    {
      /* If HS device connected*/
      demo.state = DEMO_MSC; 
      demo.msc_state = APPLI_MSC_START;
      
      BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
      BSP_LCD_DisplayStringAtLine(4, (uint8_t *)"Mass Storage demo:                                                            "); 
    }
    else
    {
      LCD_ErrLog("No Mass storage device connected.\n");
      demo.state = DEMO_WAIT; 
    }
    break;
    
  case DEMO_HID_START:
    if(Appli_FS_state == APPLICATION_FS_READY)
    {
      /*if FS device connected*/
      demo.state = DEMO_HID; 
      demo.hid_state = APPLI_HID_START;
      
      BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
      BSP_LCD_DisplayStringAtLine(4, (uint8_t *)"Mass Storage demo:                                                            "); 
    }
    else
    {
      LCD_ErrLog("No HID device connected.\n");
      demo.state = DEMO_WAIT; 
    }
    break;
    
  case DEMO_MSC:
    MSC_MenuProcess();
    break;
    
  case DEMO_HID:
    HID_MenuProcess();
    break;
        
  default:
    break;
  }
  
  if(Appli_FS_state == APPLICATION_FS_DISCONNECT)
  {
    Appli_FS_state = APPLICATION_IDLE; 
    LCD_LOG_ClearTextZone();
    LCD_ErrLog("USB device FS disconnected!\n");
    demo.state = DEMO_IDLE;
  }
  
  if(Appli_HS_state == APPLICATION_HS_DISCONNECT)
  {
    Appli_HS_state = APPLICATION_IDLE; 
    LCD_LOG_ClearTextZone();
    LCD_ErrLog("USB device HS disconnected!\n");
    demo.state = DEMO_IDLE;
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
