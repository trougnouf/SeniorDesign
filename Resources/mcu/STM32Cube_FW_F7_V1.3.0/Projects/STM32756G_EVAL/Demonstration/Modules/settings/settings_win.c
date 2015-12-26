/**
  ******************************************************************************
  * @file    settings_win.c
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    18-November-2015  
  * @brief   settings functions
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
#include "dialog.h"
#include "k_module.h"
#include "settings_res.c"
#include "k_modules_res.h"
#include "cpu_utils.h"

/** @addtogroup SETTINGS_MODULE
  * @{
  */

/** @defgroup SETTINGS
  * @brief settings routines
  * @{
  */
  
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void Startup(WM_HWIN hWin, uint16_t xpos, uint16_t ypos);

/* Private typedef -----------------------------------------------------------*/
K_ModuleItem_Typedef  settings_board =
{
  7,
  "system info",
  settings,
  0,
  Startup,
  NULL,
}
;

/* Private defines -----------------------------------------------------------*/
#define ID_WINDOW_0               (GUI_ID_USER + 0x00)
#define ID_IMAGE_0                (GUI_ID_USER + 0x01)

#define ID_TEXT_BOARD             (GUI_ID_USER + 0x02)
#define ID_TEXT_CORE              (GUI_ID_USER + 0x03)
#define ID_TEXT_CPU               (GUI_ID_USER + 0x04)
#define ID_TEXT_VERSION           (GUI_ID_USER + 0x05)
#define ID_TEXT_COPYRIGHT         (GUI_ID_USER + 0x06)
#define ID_TEXT_BOARD_1           (GUI_ID_USER + 0x07)
#define ID_TEXT_BOARD_2           (GUI_ID_USER + 0x0C)

#define ID_TEXT_CORE_1            (GUI_ID_USER + 0x08)
#define ID_TEXT_CPU_1             (GUI_ID_USER + 0x09)
#define ID_TEXT_VERSION_1         (GUI_ID_USER + 0x0A)
#define ID_TEXT_TITLE             (GUI_ID_USER + 0x0B)

#define ID_BUTTON_EXIT            (GUI_ID_USER + 0x20)

#define ID_IMAGE_BOARD            (GUI_ID_USER + 0x21)
#define ID_IMAGE_MCU              (GUI_ID_USER + 0x22)
#define ID_IMAGE_CPU              (GUI_ID_USER + 0x23)
#define ID_IMAGE_FVERSION         (GUI_ID_USER + 0x24)

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const GUI_WIDGET_CREATE_INFO _aDialog[] = 
{
  { WINDOW_CreateIndirect, "", ID_WINDOW_0,                    0,    0,  640, 480, 0, 0x64, 0 }, 
  { IMAGE_CreateIndirect, "Image", ID_IMAGE_0,                 33,  180, 574, 51,  0, 0,    0 },
  { TEXT_CreateIndirect, "System Information", ID_TEXT_TITLE,  200,  40, 300, 40,  0, 0x0,  0 },
  { TEXT_CreateIndirect, "Board", ID_TEXT_BOARD,               65,  150, 100, 20,  0, 0x0,  0 },
  { TEXT_CreateIndirect, "Core", ID_TEXT_CORE,                 220, 150, 100, 20,  0, 0x0,  0 },
  { TEXT_CreateIndirect, "CPU Speed", ID_TEXT_CPU,             350, 150, 100, 20,  0, 0x0,  0 },
  { TEXT_CreateIndirect, "Firm. Version", ID_TEXT_VERSION,   505, 150, 100, 20,  0, 0x0,  0 },  

  { TEXT_CreateIndirect, " STM32746G", ID_TEXT_BOARD_1,  37,  295, 100, 20, 0, 0x0, 0 },
  { TEXT_CreateIndirect, " STM32756G",  ID_TEXT_BOARD_2,  37,  315, 100, 20, 0, 0x0, 0 },
  { TEXT_CreateIndirect, " STM32F7", ID_TEXT_CORE_1,     205, 310, 100, 20, 0, 0x0, 0 },
  { TEXT_CreateIndirect, " 200MHz", ID_TEXT_CPU_1,       360, 310, 100, 20, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "V1.2.1 ", ID_TEXT_VERSION_1,    530, 310, 120, 20, 0, 0x0, 0 }, 
  
  { TEXT_CreateIndirect, "Copyright (c) STMicroelectronics 2015", ID_TEXT_COPYRIGHT, 400, 450, 240, 20, 0, 0x0, 0 },
};

static WM_HWIN SystemWin;
static WM_HTIMER                  hTimer; 
uint32_t frame = 0;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Paints exit button
  * @param  hObj: button handle
  * @retval None
  */
static void _OnPaint_exit(BUTTON_Handle hObj) {

  GUI_SetBkColor(FRAMEWIN_GetDefaultClientColor());
  GUI_Clear();

  GUI_SetColor(0x00DCA939);
  GUI_AA_FillCircle(70, 0, 70);

  GUI_SetBkColor(0x00DCA939);
  GUI_SetColor(GUI_WHITE);
  GUI_SetFont(&GUI_Font20B_1);
  GUI_DispStringAt("Menu", 10, 15);
}

/**
  * @brief  callback for Exit button
  * @param  pMsg: pointer to data structure of type WM_MESSAGE
  * @retval None
  */
static void _cbButton_exit(WM_MESSAGE * pMsg) {
  switch (pMsg->MsgId) {
    case WM_PAINT:
      _OnPaint_exit(pMsg->hWin);
      break;
    default:
      /* The original callback */
      BUTTON_Callback(pMsg);
      break;
  }
}

static void _cbDialog(WM_MESSAGE * pMsg) {
  WM_HWIN hItem;
  int Id, NCode;

  
  switch (pMsg->MsgId) {
  case WM_INIT_DIALOG:
    
    hItem = BUTTON_CreateEx(570, 0, 70, 70, pMsg->hWin, WM_CF_SHOW, 0, ID_BUTTON_EXIT);
    WM_SetCallback(hItem, _cbButton_exit);
    
    /* Initialization of 'Board : STM324x9I' */
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_BOARD);
    TEXT_SetFont(hItem, &GUI_FontITCAvantGardeStdBk20);
    TEXT_SetTextColor(hItem, 0x00522000);

    /* Initialization of 'Core: STM32F-4 Series' */
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_CORE);
    TEXT_SetFont(hItem, &GUI_FontITCAvantGardeStdBk20);
    TEXT_SetTextColor(hItem, 0x00522000);

    /* Initialization of 'CPU Speed : 180MHz' */
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_CPU);
    TEXT_SetFont(hItem, &GUI_FontITCAvantGardeStdBk20);
    TEXT_SetTextColor(hItem, 0x00522000);

    /* Initialization of 'Firmware Version : 1.0' */
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_VERSION);
    TEXT_SetFont(hItem, &GUI_FontITCAvantGardeStdBk20);
    TEXT_SetTextColor(hItem, 0x00522000); 

    /* Initialization of 'Board : STM324x9I' */
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_BOARD_1);
    TEXT_SetFont(hItem, &GUI_FontLubalGraphXLtBT20);
    TEXT_SetTextColor(hItem, 0x00DCA939);

    /* Initialization of 'Board : STM324x9I' */
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_BOARD_2);
    TEXT_SetFont(hItem, &GUI_FontLubalGraphXLtBT20);
    TEXT_SetTextColor(hItem, 0x00DCA939);    
    
    /* Initialization of 'Core: STM32F-4 Series' */
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_CORE_1);
    TEXT_SetFont(hItem, &GUI_FontLubalGraphXLtBT20);
    TEXT_SetTextColor(hItem, 0x00DCA939);

    /* Initialization of 'CPU Speed : 180MHz' */
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_CPU_1);
    TEXT_SetFont(hItem, &GUI_FontLubalGraphXLtBT20);
    TEXT_SetTextColor(hItem, 0x00DCA939);

    /* Initialization of 'Firmware Version : 1.0' */
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_VERSION_1);
    TEXT_SetFont(hItem, &GUI_FontLubalGraphXLtBT20);
    TEXT_SetTextColor(hItem, 0x00DCA939); 
    
    /* ST Copyright */
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_COPYRIGHT);
    TEXT_SetFont(hItem, GUI_FONT_16_ASCII);
    TEXT_SetTextColor(hItem, 0x00522000);

    /* ST Copyright */
    hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_TITLE);
    TEXT_SetFont(hItem, GUI_FONT_32_ASCII);
    TEXT_SetTextColor(hItem, 0x00DCA939);

    IMAGE_CreateEx(37,  190, 100, 100, pMsg->hWin, WM_CF_SHOW, 0, ID_IMAGE_BOARD);  
    IMAGE_CreateEx(192, 190, 100, 100, pMsg->hWin, WM_CF_SHOW, 0, ID_IMAGE_MCU);
    IMAGE_CreateEx(347, 190, 100, 100, pMsg->hWin, WM_CF_SHOW, 0, ID_IMAGE_CPU);
    IMAGE_CreateEx(502, 190, 100, 100, pMsg->hWin, WM_CF_SHOW, 0, ID_IMAGE_FVERSION);
    
    break;     
    
  case WM_PAINT:    
    GUI_SetColor(0x00DCA939);
    GUI_AA_DrawRoundedRect(20,  140, 155, 340, 30);
    GUI_AA_DrawRoundedRect(175, 140, 310, 340, 30);
    GUI_AA_DrawRoundedRect(330, 140, 465, 340, 30);
    GUI_AA_DrawRoundedRect(485, 140, 620, 340, 30);
    
    break;     

  case WM_TIMER:
    /* draw */
    
    if(frame < 5)
    {
      WM_RestartTimer(pMsg->Data.v, 100);
      
      hItem = WM_GetDialogItem(pMsg->hWin, ID_IMAGE_BOARD);
      IMAGE_SetBitmap(hItem, open_board[frame]);
      
      hItem = WM_GetDialogItem(pMsg->hWin, ID_IMAGE_MCU);
      IMAGE_SetBitmap(hItem, open_mcu[frame]);
      
      hItem = WM_GetDialogItem(pMsg->hWin, ID_IMAGE_CPU);
      IMAGE_SetBitmap(hItem, open_cpu[frame]);
      
      hItem = WM_GetDialogItem(pMsg->hWin, ID_IMAGE_FVERSION);
      IMAGE_SetBitmap(hItem, open_fversion[frame]);
      
      frame++;
    }
    else
    {
      if(hTimer != 0)
      {
        WM_DeleteTimer(hTimer);
        hTimer = 0;
      }  
    }    
    break;
    
  case WM_DELETE:
    if(hTimer != 0)
    {
      WM_DeleteTimer(hTimer);
      hTimer = 0;
    }    
   
     break;
    
  case WM_NOTIFY_PARENT:
    Id    = WM_GetId(pMsg->hWinSrc);    /* Id of widget */
    NCode = pMsg->Data.v;               /* Notification code */
       
    switch(Id) {
    case ID_BUTTON_EXIT: 
      switch(NCode) {
      case WM_NOTIFICATION_RELEASED:
        if(hTimer != 0)
        {
          WM_DeleteTimer(hTimer);
          hTimer = 0;
        }         
        GUI_EndDialog(pMsg->hWin, 0);

        break;
      }
      break;      
    }
    break;
  default:
    WM_DefaultProc(pMsg);
    break;
  }    
}


/**
  * @brief  Game window Startup
  * @param  hWin: pointer to the parent handle.
  * @param  xpos: X position 
  * @param  ypos: Y position
  * @retval None
  */
static void Startup(WM_HWIN hWin, uint16_t xpos, uint16_t ypos)
{
  frame = 0;
  SystemWin = GUI_CreateDialogBox(_aDialog, GUI_COUNTOF(_aDialog), _cbDialog, hWin, xpos, ypos);
  hTimer = WM_CreateTimer(SystemWin, 0, 100, 0);
}

/**
  * @}
  */

/**
  * @}
  */
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
