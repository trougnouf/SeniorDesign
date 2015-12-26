/**
  ******************************************************************************
  * @file    videoplayer_win.c
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    18-November-2015  
  * @brief   Video player functions
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
#include "video_player_res.c"
#include "k_modules_res.h"
#include "k_storage.h"
#include "k_rtc.h"
#include <stdio.h>
#include "cpu_utils.h"
#include "STSLIDER_SKIN.h"
/** @addtogroup VIDEO_PLAYER_MODULE
  * @{
  */

/** @defgroup VIDEO_PLAYER
  * @brief video player routines
  * @{
  */

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void Startup(WM_HWIN hWin, uint16_t xpos, uint16_t ypos);
static void FILEMGR_GetFileOnly (char *file, char *path);
static void _StopPlay(WM_HWIN *hmovie, FIL * file) ;
static uint8_t _StartPlay(WM_HWIN *hmovie, char * filename, FIL * file, uint16_t x0, uint16_t y0) ;
static void _cbMediaConnection(WM_MESSAGE * pMsg);
/* Private typedef -----------------------------------------------------------*/
K_ModuleItem_Typedef  video_player_board =
{
  1,
  "Video Player",
  open_video,
  0,
  Startup,
  NULL,
}
;

/* Private defines -----------------------------------------------------------*/
#define VIDEO_FRAME_ID            (GUI_ID_USER + 0x00)
#define VIDEO_TITLE_ID            (GUI_ID_USER + 0x01)
#define SELECT_FILE1_ID           (GUI_ID_USER + 0x02)
#define PLAY_VIDEO1_ID            (GUI_ID_USER + 0x03)
#define FPS1_CAPTION_ID           (GUI_ID_USER + 0x06)
#define FPS1_VALUE_ID             (GUI_ID_USER + 0x07)
#define LostFrames11_CAPTION_ID   (GUI_ID_USER + 0x0B)
#define LostFrames11_VALUE_ID     (GUI_ID_USER + 0x0C)

#define FILENAME1_ID              (GUI_ID_USER + 0x19)
#define ID_BUTTON_EXIT            (GUI_ID_USER + 0x1B)
#define ID_BUTTON_EXIT_FROM_VIDEO (GUI_ID_USER + 0x1C)
#define PLAY_VIDEO_ID             (GUI_ID_USER + 0x1D)
#define ID_BUTTON_STOP            (GUI_ID_USER + 0x1E)
#define ID_VIDEO_LIST             (GUI_ID_USER + 0x1F)
#define ID_BUTTON_MENU            (GUI_ID_USER + 0x20)

#define ID_ICONVIEW_SUBMENU       (GUI_ID_USER + 0x30)

#define ID_BUTTON_OKPL            (GUI_ID_USER + 0x31)
#define ID_BUTTON_ADDPL           (GUI_ID_USER + 0x32)
#define ID_BUTTON_NEXT            (GUI_ID_USER + 0x33)
#define ID_BUTTON_PREVIOUS        (GUI_ID_USER + 0x34)
#define ID_BUTTON_BACK            (GUI_ID_USER + 0x35)
#define ID_BUTTON_FORE            (GUI_ID_USER + 0x36)

#define ID_FRAMEWIN_0               (GUI_ID_USER + 0x37)
#define ID_BUTTON_OK_MESSAGE        (GUI_ID_USER + 0x38)
#define ID_TEXT_FILENAME            (GUI_ID_USER + 0x39)

#define ID_TIME                     (GUI_ID_USER + 0x40)
#define ID_ELAPSED_TIME             (GUI_ID_USER + 0x41)

#define ID_VIDEO_NAME               (GUI_ID_USER + 0x42)
#define ID_FPS                      (GUI_ID_USER + 0x43)
#define ID_FEATURES_FPU             (GUI_ID_USER + 0x44)
#define ID_FEATURES_IC              (GUI_ID_USER + 0x45)
#define ID_FEATURES_DC              (GUI_ID_USER + 0x46)
#define ID_FEATURES_CPU             (GUI_ID_USER + 0x47)
#define ID_SLIDER_DURATION          (GUI_ID_USER + 0x48)
#define ID_VIDEO_FORMAT             (GUI_ID_USER + 0x49)
#define VIDEO_IDLE             0
#define VIDEO_PLAY             1
#define VIDEO_STOP             2
#define VIDEO_PAUSE            3

#define VIDEO_FRAME_TIME       40

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const GUI_WIDGET_CREATE_INFO _aDialog[] = {
  { WINDOW_CreateIndirect, "Window", VIDEO_FRAME_ID, 0, 0, 640, 480, 0, 0x0, 0 },
};

/* Dialog Create */
static const GUI_WIDGET_CREATE_INFO _aFileInfoDialogCreate[] = {
  { FRAMEWIN_CreateIndirect, "Video player", ID_FRAMEWIN_0, 0, 0, 430, 300, 0, 0x0, 0 },
  { TEXT_CreateIndirect, "Please Add Video file to playList", ID_TEXT_FILENAME, 75, 120, 300, 20, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "OK", ID_BUTTON_OK_MESSAGE, 170, 225, 90, 90, 0, 0x0, 0 },
};

static const GUI_WIDGET_CREATE_INFO _aPlaylistDialogCreate[] = {
  { FRAMEWIN_CreateIndirect, "Playlist", ID_FRAMEWIN_0, 0, 0, 430, 300, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "OK", ID_BUTTON_OKPL, 380, 220, 90, 90, 0, 0x0, 0 },
  { BUTTON_CreateIndirect, "Add", ID_BUTTON_ADDPL, (-45), 220, 90, 90, 0, 0x0, 0 },  
};


static GUI_MOVIE_HANDLE hvideo;
static GUI_MOVIE_HANDLE playbackwin;
static WM_HWIN  VideoWin, hFrame;
uint8_t         VideoPlayer_State = VIDEO_IDLE;

static char const                *apDrives[1] = {"0:"};
static const char                acMask_video[] = ".video";

FIL                               Video_File;

uint8_t                          Video_Path[256];

uint8_t                          SelLayer = 0;
static __IO uint32_t              AddStatus = 0;
static FILELIST_FileTypeDef       VideoList;
static  CHOOSEFILE_INFO           FileInfo;
static uint32_t                   file_pos = 0;
static WM_HTIMER                  hTimer; 
static uint32_t                   elapsed_time = 0;
GUI_MOVIE_INFO   Video_Info;
  uint32_t step = 0;
static __IO uint32_t FrameIndex = 0, PrevFrameIndex = 0;
static __IO uint32_t LostFrames = 0, previous_total = 0;

/**
  * @brief  Paints exit button
  * @param  hObj: button handle
  * @retval None
  */
static void _OnPaint_exit(BUTTON_Handle hObj) {
  
  GUI_SetBkColor(GUI_WHITE);
  GUI_Clear();

  GUI_SetColor(0x00DCA939);
  GUI_AA_FillCircle(65, 0, 65);

  GUI_SetBkColor(0x00DCA939);
  GUI_SetColor(GUI_WHITE);
  GUI_SetFont(&GUI_Font20B_1);
  GUI_DispStringAt("Menu", 12, 15);
}

/**
  * @brief  Paints exit button
  * @param  hObj: button handle
  * @retval None
  */
static void _OnPaint_exit1(BUTTON_Handle hObj) {

  GUI_SetBkColor(GUI_TRANSPARENT);
  GUI_Clear();

  GUI_SetColor(0x00DCA939);
  GUI_AA_FillCircle(0, 65, 65);

  GUI_SetBkColor(0x00DCA939);
  GUI_SetColor(GUI_WHITE);
  GUI_SetFont(&GUI_Font20B_1);
  GUI_DispStringAt("back", 5, 25);
}

/**
  * @brief  Paints exit button
  * @param  hObj: button handle
  * @retval None
  */
static void _OnPaint_menu(BUTTON_Handle hObj) {

  GUI_SetBkColor(GUI_TRANSPARENT);
  GUI_Clear();

  GUI_SetColor(0x00DCA939);
  GUI_AA_FillCircle(65, 0, 65);

  GUI_SetBkColor(0x00DCA939);
  GUI_SetColor(GUI_WHITE);
  GUI_SetFont(&GUI_Font20B_1);
  GUI_DispStringAt("Menu", 20, 15);
}

/**
  * @brief  Paints play button
  * @param  hObj: button handle
  * @retval None
  */
static void _OnPaint_play(BUTTON_Handle hObj) {
  
  if(VideoPlayer_State != VIDEO_PLAY)
  {
    GUI_DrawBitmap(&bmplay, 0, 0);
  }
  else
  {
    GUI_DrawBitmap(&bmpause, 0, 0);
  }
}

/**
  * @brief  Paints play button
  * @param  hObj: button handle
  * @retval None
  */
static void _OnPaint_stop(BUTTON_Handle hObj) {
  
  GUI_DrawBitmap(&bmstop, 0, 0);
}

/**
  * @brief  Paints back button
  * @param  hObj: button handle
  * @retval None
  */
static void _OnPaint_back(BUTTON_Handle hObj) {
  
  GUI_DrawBitmap(&bmback, 0, 0);
}

/**
  * @brief  Paints foreward button
  * @param  hObj: button handle
  * @retval None
  */
static void _OnPaint_fore(BUTTON_Handle hObj) {
  
  GUI_DrawBitmap(&bmfore, 0, 0);
}
/**
  * @brief  callback for start button
  * @param  pMsg: pointer to data structure of type WM_MESSAGE
  * @retval None
  */
static void _cbButton_play(WM_MESSAGE * pMsg) {
  switch (pMsg->MsgId) {
  case WM_PAINT:
    _OnPaint_play(pMsg->hWin);
    break;
  default:
    /* The original callback */
    BUTTON_Callback(pMsg);
    break;
  }
}
/**
  * @brief  callback for back button
  * @param  pMsg: pointer to data structure of type WM_MESSAGE
  * @retval None
  */
static void _cbButton_back(WM_MESSAGE * pMsg) {
  switch (pMsg->MsgId) {
  case WM_PAINT:
    _OnPaint_back(pMsg->hWin);
    break;
  default:
    /* The original callback */
    BUTTON_Callback(pMsg);
    break;
  }
}

/**
  * @brief  callback for foreward button
  * @param  pMsg: pointer to data structure of type WM_MESSAGE
  * @retval None
  */
static void _cbButton_fore(WM_MESSAGE * pMsg) {
  switch (pMsg->MsgId) {
  case WM_PAINT:
    _OnPaint_fore(pMsg->hWin);
    break;
  default:
    /* The original callback */
    BUTTON_Callback(pMsg);
    break;
  }
}
/**
  * @brief  callback for start button
  * @param  pMsg: pointer to data structure of type WM_MESSAGE
  * @retval None
  */
static void _cbButton_stop(WM_MESSAGE * pMsg) {
  switch (pMsg->MsgId) {
  case WM_PAINT:
    _OnPaint_stop(pMsg->hWin);
    break;
  default:
    /* The original callback */
    BUTTON_Callback(pMsg);
    break;
  }
}

/**
  * @brief  callback for Exit button
  * @param  pMsg: pointer to data structure of type WM_MESSAGE
  * @retval None
  */
static void _cbButton_exit1(WM_MESSAGE * pMsg) {
  switch (pMsg->MsgId) {
  case WM_PAINT:
    _OnPaint_exit1(pMsg->hWin);
    break;
  default:
    /* The original callback */
    BUTTON_Callback(pMsg);
    break;
  }
}

/**
  * @brief  callback for menu button
  * @param  pMsg: pointer to data structure of type WM_MESSAGE
  * @retval None
  */
static void _cbButton_menu(WM_MESSAGE * pMsg) {
  switch (pMsg->MsgId) {
  case WM_PAINT:
    _OnPaint_menu(pMsg->hWin);
    break;
  default:
    /* The original callback */
    BUTTON_Callback(pMsg);
    break;
  }
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


/**
  * @brief  Paints add to playlist button
  * @param  hObj: button handle
  * @retval None
  */
static void _OnPaint_add2PL(BUTTON_Handle hObj) {

  GUI_SetBkColor(GUI_WHITE);
  GUI_Clear();

  GUI_SetColor(0x00DCA939);
  GUI_AA_FillCircle(45, 45, 45);

  GUI_SetBkColor(0x00DCA939);
  GUI_SetColor(GUI_WHITE);
  GUI_SetFont(&GUI_Font20B_1);
  GUI_DispStringAt("Add", 47, 20);
}

/**
  * @brief  Paints ok playlist button
  * @param  hObj: button handle
  * @retval None
  */
static void _OnPaint_okPL(BUTTON_Handle hObj) {

  GUI_SetBkColor(GUI_WHITE);
  GUI_Clear();

  GUI_SetColor(0x00DCA939);
  GUI_AA_FillCircle(45, 45, 45);

  GUI_SetBkColor(0x00DCA939);
  GUI_SetColor(GUI_WHITE);
  GUI_SetFont(&GUI_Font20B_1);
  GUI_DispStringAt("OK", 10, 20);
}

/**
  * @brief  Paints ok playlist button
  * @param  hObj: button handle
  * @retval None
  */
static void _OnPaint_okMessage(BUTTON_Handle hObj) {

  GUI_SetBkColor(GUI_WHITE);
  GUI_Clear();

  GUI_SetColor(0x00DCA939);
  GUI_AA_FillCircle(45, 45, 45);

  GUI_SetBkColor(0x00DCA939);
  GUI_SetColor(GUI_WHITE);
  GUI_SetFont(&GUI_Font20B_1);
  GUI_DispStringAt("OK", 30, 20);
}

/**
  * @brief  callback for add to playlist button
  * @param  pMsg: pointer to data structure of type WM_MESSAGE
  * @retval None
  */
static void _cbButton_add2PL(WM_MESSAGE * pMsg) {
  switch (pMsg->MsgId) {
  case WM_PAINT:
    _OnPaint_add2PL(pMsg->hWin);
    break;
  default:
    /* The original callback */
    BUTTON_Callback(pMsg);
    break;
  }
}

/**
  * @brief  callback for ok playlist button
  * @param  pMsg: pointer to data structure of type WM_MESSAGE
  * @retval None
  */
static void _cbButton_okPL(WM_MESSAGE * pMsg) {
  switch (pMsg->MsgId) {
  case WM_PAINT:
    _OnPaint_okPL(pMsg->hWin);
    break;
  default:
    /* The original callback */
    BUTTON_Callback(pMsg);
    break;
  }
}

/**
  * @brief  callback for ok message button
  * @param  pMsg: pointer to data structure of type WM_MESSAGE
  * @retval None
  */
static void _cbButton_okMessage(WM_MESSAGE * pMsg) {
  switch (pMsg->MsgId) {
  case WM_PAINT:
    _OnPaint_okMessage(pMsg->hWin);
    break;
  default:
    /* The original callback */
    BUTTON_Callback(pMsg);
    break;
  }
}
/**
  * @brief  callback for video window
  * @param  pMsg: pointer to a data structure of type WM_MESSAGE
  * @retval None
  */
static void _cbTouch(WM_MESSAGE * pMsg) 
{
  const GUI_PID_STATE * pState;
  static uint8_t toggle = 0;
  
  switch (pMsg->MsgId) 
  {
  case  WM_TOUCH:
    pState = (const GUI_PID_STATE *)pMsg->Data.p;
    if (pState) 
    {
      if (pState->Pressed == 1)
      {
        if(!toggle)
        {
          GUI_SetLayerVisEx (1, 0);
        }
        else
        {
          GUI_SetLayerVisEx (1, 1);
        }
        toggle ^= 1;
        WM_Exec();
      }
    }
    break; 
    
  default:
    WM_DefaultProc(pMsg);
    break;    
  }
}

/**
  * @brief  Callback function of the media connection status
  * @param  pMsg: pointer to data structure of type WM_MESSAGE
  * @retval None
  */
static void _cbMediaConnection(WM_MESSAGE * pMsg) 
{
  static WM_HTIMER      hStatusTimer;  
  static uint8_t        prev_usb_status = 0;
   
  switch (pMsg->MsgId) 
  {
  case WM_CREATE:
    prev_usb_status = k_StorageGetStatus(USB_DISK_UNIT);    
    hStatusTimer = WM_CreateTimer(pMsg->hWin, 0, 500, 0);      
    break;
    
  case WM_TIMER:
  if(prev_usb_status != k_StorageGetStatus(USB_DISK_UNIT))
    {
      prev_usb_status = k_StorageGetStatus(USB_DISK_UNIT);
      if(VideoList.file[file_pos].name[0] == '0')
      { 
        if(VideoPlayer_State != VIDEO_IDLE)
        {
          _StopPlay(&hvideo, &Video_File);
        }
        
        VideoList.ptr = 0;
        
        GUI_EndDialog(playbackwin, 0);
        SelLayer = 0;
        GUI_SetLayerVisEx (1, 0);
        GUI_SelectLayer(0); 
        WM_HideWindow(hFrame);        
        WM_Exec();        
        WM_DeleteWindow(hFrame);
        WM_InvalidateWindow(VideoWin);
        break; 
      }
    }
    WM_RestartTimer(pMsg->Data.v, 500);
    break;
    
  case WM_DELETE:
    if(hStatusTimer != 0)
    {
      WM_DeleteTimer(hStatusTimer);
      hStatusTimer = 0;
    }
    break;   
    
  default:
    WM_DefaultProc(pMsg);
  }
}

/**
  * @brief  Get data.
  * @param  p: Handle to file structure
  * @param  ppData: pointer to data buffer to be read
  * @param  NumBytesReq: number of bytes to be read
  * @param  Off: offset in the file    
  * @retval number of read bytes
  */
static int _GetData(void * p, const U8 ** ppData, unsigned NumBytesReq, U32 Off)
{
  unsigned int NumBytesRead;
  
  /* Set file pointer to the required position */
  f_lseek((FIL *)p, Off);
  
  /* Read data into buffer */
  f_read((FIL *)p, (U8 *)*ppData, NumBytesReq, &NumBytesRead);
   
  /* Return number of available bytes */
  return NumBytesRead;  
}

/**
  * @brief  callback for video window
  * @param  pMsg: pointer to a data structure of type WM_MESSAGE
  * @retval None
  */
static void _cbplaybackwin(WM_MESSAGE * pMsg) 
{
  WM_HWIN hItem;
  int Id, NCode;
  char tmp[64];
  
  switch (pMsg->MsgId) 
  {
  case WM_CREATE:
    SelLayer = 1;    
   
    hItem = BUTTON_CreateEx(398 - 160, 442, 40, 40, pMsg->hWin, WM_CF_SHOW, 0, ID_BUTTON_BACK);
    WM_SetCallback(hItem, _cbButton_back);
    
    hItem = BUTTON_CreateEx(449 - 160, 442, 40, 40, pMsg->hWin, WM_CF_SHOW, 0, ID_BUTTON_STOP);
    WM_SetCallback(hItem, _cbButton_stop); 

    hItem = BUTTON_CreateEx(495 - 160, 441, 40, 40, pMsg->hWin, WM_CF_SHOW, 0, PLAY_VIDEO_ID);
    WM_SetCallback(hItem, _cbButton_play);
        
    hItem = BUTTON_CreateEx(540 - 160, 442, 40, 40, pMsg->hWin, WM_CF_SHOW, 0, ID_BUTTON_FORE);
    WM_SetCallback(hItem, _cbButton_fore);
    
    hItem = SLIDER_CreateEx(72, 420, 510, 13, pMsg->hWin, WM_CF_SHOW, 0, ID_SLIDER_DURATION);
    SLIDER_SetBkColor(hItem, GUI_TRANSPARENT);
    SLIDER_SetFocusColor (hItem, 0x00DCA939);
    SLIDER_SetValue(hItem, 0);
    SLIDER_SetWidth(hItem, 0);
    SLIDER_SetSTSkin(hItem);    
       
    hItem = BUTTON_CreateEx(568, 0, 70, 70, pMsg->hWin, WM_CF_SHOW, 0, ID_BUTTON_MENU);
    WM_SetCallback(hItem, _cbButton_menu);     
    
    hItem = BUTTON_CreateEx(0, 410, 70, 70, pMsg->hWin, WM_CF_SHOW, 0, ID_BUTTON_EXIT_FROM_VIDEO);
    WM_SetCallback(hItem, _cbButton_exit1);   
    
    /* Title Initialization in play list */   
    hItem = TEXT_CreateEx(10, 20, 100, 40, pMsg->hWin, WM_CF_SHOW, 0, ID_ELAPSED_TIME, "00:00");
    TEXT_SetFont(hItem, GUI_FONT_20B_1);
    TEXT_SetTextColor(hItem, GUI_WHITE);     
    
    /* Title Initialization in play list */   
    hItem = TEXT_CreateEx(50, 40, 50, 30, pMsg->hWin, WM_CF_SHOW, 0, ID_TIME, "00:00");
    TEXT_SetFont(hItem, GUI_FONT_16B_1);
    TEXT_SetTextColor(hItem, GUI_WHITE);  
    
    hItem = TEXT_CreateEx(150, 20, 350, 30, pMsg->hWin, WM_CF_SHOW, TEXT_CF_HCENTER, ID_VIDEO_NAME, "File Name");
    TEXT_SetFont(hItem, GUI_FONT_20B_1);
    TEXT_SetTextColor(hItem, GUI_WHITE); 
    
    hItem = TEXT_CreateEx(150, 48, 350, 30, pMsg->hWin, WM_CF_SHOW, TEXT_CF_HCENTER, ID_VIDEO_FORMAT, "Format : MJPEG");
    TEXT_SetFont(hItem, GUI_FONT_13B_1);
    TEXT_SetTextColor(hItem, GUI_WHITE);
    
    hItem = TEXT_CreateEx(548, 200, 260, 20, pMsg->hWin, WM_CF_SHOW, TEXT_CF_LEFT, ID_FPS, "Rate : 20 fps");
    TEXT_SetFont(hItem, GUI_FONT_13B_1);
    TEXT_SetTextColor(hItem, GUI_WHITE);   
    
    hItem = TEXT_CreateEx(548, 220, 260, 20, pMsg->hWin, WM_CF_SHOW, TEXT_CF_LEFT, ID_FEATURES_FPU, "FPU : ON");
    TEXT_SetFont(hItem, GUI_FONT_13B_1);
    TEXT_SetTextColor(hItem, GUI_WHITE); 
    
    hItem = TEXT_CreateEx(548, 240, 260, 20, pMsg->hWin, WM_CF_SHOW, TEXT_CF_LEFT, ID_FEATURES_IC, "I-Cache : ON");
    TEXT_SetFont(hItem, GUI_FONT_13B_1);
    TEXT_SetTextColor(hItem, GUI_WHITE);     
    
    hItem = TEXT_CreateEx(548, 260, 260, 20, pMsg->hWin, WM_CF_SHOW, TEXT_CF_LEFT, ID_FEATURES_DC, "D-Cache : ON");
    TEXT_SetFont(hItem, GUI_FONT_13B_1);
    TEXT_SetTextColor(hItem, GUI_WHITE);  

    hItem = TEXT_CreateEx(548, 280, 260, 20, pMsg->hWin, WM_CF_SHOW, TEXT_CF_LEFT, ID_FEATURES_CPU, "MCU Load : 0%");
    TEXT_SetFont(hItem, GUI_FONT_13B_1);
    TEXT_SetTextColor(hItem, GUI_WHITE);       
    
    break;
    
  case WM_TIMER:
    { 
      WM_RestartTimer(pMsg->Data.v, 1000);
      /* show elapsed time */
      hItem = WM_GetDialogItem(pMsg->hWin, ID_ELAPSED_TIME);
      elapsed_time++;
      sprintf((char *)tmp , "%02lu:%02lu", elapsed_time/60, elapsed_time%60 );
      TEXT_SetText(hItem, tmp);
      sprintf((char *)tmp , "MCU Load : %d%%", ((osGetCPUUsage() > 90) ? (osGetCPUUsage() - 10) : osGetCPUUsage()));
      hItem = WM_GetDialogItem(pMsg->hWin, ID_FEATURES_CPU);
      TEXT_SetText(hItem, tmp);
      
      hItem = WM_GetDialogItem(pMsg->hWin, ID_SLIDER_DURATION);
      SLIDER_SetValue(hItem, (GUI_MOVIE_GetFrameIndex(hvideo)* 100)/ Video_Info.NumFrames);
      
      if(step == 0)
      {
        step = ((GUI_MOVIE_GetFrameIndex(hvideo)* 100)/ Video_Info.NumFrames);
      }

      if(((GUI_MOVIE_GetFrameIndex(hvideo)* 100)/ Video_Info.NumFrames) > (100 - 2*step - 1))
      {
        SLIDER_SetValue(hItem, 100);
        GUI_Exec();
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
    
  case WM_PAINT: 
    GUI_SetColor(0xFF000000 | GUI_BLACK);
    GUI_SetBkColor(GUI_TRANSPARENT);
    GUI_Clear();

    GUI_SetColor(GUI_BLACK);
    GUI_DrawHLine(0, 0, 640);
    GUI_DrawHLine(479, 0, 640);
    GUI_DrawLine(639, 0, 639, 480);   
    GUI_DrawLine(0, 0, 0, 480);

    /* Background for total time */
    GUI_SetColor(0x00DCA939);
    GUI_AA_FillRoundedRect((-30), 5, 90, 65, 30);
    
    /*Video Information */
    GUI_AA_FillRoundedRect(150, 5, 500, 65, 30);
    
    /*H/W Information */
    GUI_AA_FillRoundedRect(538, 180, 780, 320, 30);    
    
    break;
    
    
  case WM_NOTIFY_PARENT:
    Id    = WM_GetId(pMsg->hWinSrc);    /* Id of widget */
    NCode = pMsg->Data.v;               /* Notification code */
 
    
    switch(Id) {
      
    case ID_BUTTON_EXIT_FROM_VIDEO: 
      switch(NCode) {
      case WM_NOTIFICATION_RELEASED:

        if(VideoPlayer_State != VIDEO_IDLE)
        {
          _StopPlay(&hvideo, &Video_File);
        }
        
        GUI_EndDialog(pMsg->hWin, 0);
        SelLayer = 0;
        GUI_SetLayerVisEx (1, 0);
        GUI_SelectLayer(0); 
        WM_HideWindow(hFrame);        
        WM_Exec();        
        WM_DeleteWindow(hFrame);
        WM_InvalidateWindow(VideoWin);
        break;
      }
      break;

    case ID_BUTTON_MENU: 
      switch(NCode) {
      case WM_NOTIFICATION_RELEASED:

        if(VideoPlayer_State != VIDEO_IDLE)
        {
          _StopPlay(&hvideo, &Video_File);
        }
        
        GUI_EndDialog(pMsg->hWin, 0);
        SelLayer = 0;
        GUI_SetLayerVisEx (1, 0);
        GUI_SelectLayer(0); 
        WM_DeleteWindow(hFrame);
        WM_HideWindow(VideoWin);        
        WM_Exec();
        WM_DeleteWindow(VideoWin);
        break;
      }
      break;
      
    case ID_BUTTON_STOP: 
      switch(NCode) {
      case WM_NOTIFICATION_RELEASED:
        if(VideoPlayer_State != VIDEO_IDLE)
        {
          VideoPlayer_State = VIDEO_STOP;
          _StopPlay(&hvideo, &Video_File);
        }
         WM_InvalidateWindow(pMsg->hWin);
        break;
      }
      break;
     
    case ID_BUTTON_FORE: 
      switch(NCode) {
      case WM_NOTIFICATION_RELEASED:
        if(VideoPlayer_State == VIDEO_PLAY)
        {
          if(file_pos < (VideoList.ptr - 1))
          {
            file_pos++;
          }
          else 
          {        
            file_pos = 0; 
          }  
          _StopPlay(&hvideo, &Video_File);
          _StartPlay(&hvideo, (char *)VideoList.file[file_pos].name, &Video_File, 0, 0);

        }
        break;
      }
      break;
      
    /* Notifications sent by 'progress' Slider */
    case ID_SLIDER_DURATION: 
      if(NCode == WM_NOTIFICATION_CLICKED)
      {
        hItem = WM_GetDialogItem(pMsg->hWin, ID_SLIDER_DURATION);
        int32_t newpos;
        if(VideoPlayer_State == VIDEO_PLAY)
        {
          GUI_MOVIE_Pause(hvideo);
          hItem = WM_GetDialogItem(pMsg->hWin, ID_SLIDER_DURATION);
          newpos = (SLIDER_GetValue(hItem) * Video_Info.NumFrames)/100;
          GUI_MOVIE_GotoFrame(hvideo, newpos);
          GUI_MOVIE_Play(hvideo);
          elapsed_time = (Video_Info.msPerFrame * newpos)/ 1000;
          hItem = WM_GetDialogItem(playbackwin, ID_ELAPSED_TIME);
          sprintf((char *)tmp , "%02lu:%02lu", elapsed_time/60, elapsed_time%60 );
          TEXT_SetText(hItem, tmp); 
          WM_InvalidateWindow(hItem);
          WM_Paint(hItem);
        }
      }
      break;
      
      
    case ID_BUTTON_BACK: 
      switch(NCode) {
      case WM_NOTIFICATION_RELEASED:
        if(VideoPlayer_State == VIDEO_PLAY)
        {
          if(file_pos > 0)
          {   
            file_pos--;
          }
          else 
          {        
            file_pos = VideoList.ptr - 1; 
          }  
          
          _StopPlay(&hvideo, &Video_File);
          _StartPlay(&hvideo, (char *)VideoList.file[file_pos].name, &Video_File, 0, 0); 
        }
        break;
      }
      break;
      
    case PLAY_VIDEO_ID: 
      switch(NCode) {
      case WM_NOTIFICATION_RELEASED:
          if((VideoPlayer_State == VIDEO_IDLE) || (VideoPlayer_State == VIDEO_PAUSE))
          {
            hTimer = WM_CreateTimer(playbackwin, 0, 1000, 0);  
            GUI_MOVIE_Play(hvideo); 
            VideoPlayer_State = VIDEO_PLAY;
          }
          else if(VideoPlayer_State == VIDEO_STOP)
          {
            _StartPlay(&hvideo, (char *)VideoList.file[file_pos].name, &Video_File, 0, 0);             
            VideoPlayer_State = VIDEO_PLAY;
          }
          else
          {
            GUI_MOVIE_Pause(hvideo);
            VideoPlayer_State = VIDEO_PAUSE; 
            if(hTimer != 0)
            {
              WM_DeleteTimer(hTimer);
              hTimer = 0;
            }            
          }
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
  * @brief  Callback routine of Info dialog
  * @param  pMsg: pointer to data structure of type WM_MESSAGE 
  * @retval None
  */
static void _cbFileInfoDialog(WM_MESSAGE * pMsg) {
  int     NCode;
  int     Id;
  WM_HWIN hItem;
  
  switch (pMsg->MsgId) {
  case WM_INIT_DIALOG:
    
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_OK_MESSAGE);
    WM_SetCallback(hItem, _cbButton_okMessage);
    break;
        
  case WM_PAINT:    
    GUI_DrawBitmap(&bmwarning, 180, 40);
    break;
    
  case WM_NOTIFY_PARENT:
    Id    = WM_GetId(pMsg->hWinSrc);
    NCode = pMsg->Data.v;
    switch(Id) {
    case ID_BUTTON_OK_MESSAGE: /* Notifications sent by 'OK' */
      switch(NCode) {
      case WM_NOTIFICATION_RELEASED:
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
  * @brief  Callback routine of Info dialog
  * @param  pMsg: pointer to data structure of type WM_MESSAGE 
  * @retval None
  */
static void _cbPlaylistDialog(WM_MESSAGE * pMsg) {
  int     NCode;
  int     Id;
  int     r;
  WM_HWIN hItem;

  static char tmp[64];
  uint32_t i = 0;
  
  switch (pMsg->MsgId) {
  case WM_INIT_DIALOG:
    
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_OKPL);
    WM_SetCallback(hItem, _cbButton_okPL);
    
    hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_ADDPL);
    WM_SetCallback(hItem, _cbButton_add2PL);   
    
    
    hItem = LISTVIEW_CreateEx(60, 30, 300, 250, pMsg->hWin, WM_CF_SHOW, 0, ID_VIDEO_LIST);

    LISTVIEW_SetFont(hItem, &GUI_Font16_1);
    LISTVIEW_SetTextColor(hItem, LISTVIEW_CI_UNSEL, 0x00DCA939);
    LISTVIEW_SetTextColor(hItem, LISTVIEW_CI_SEL, GUI_WHITE);
    LISTVIEW_SetHeaderHeight(hItem, 0);
    LISTVIEW_AddColumn(hItem, 300, "", GUI_TA_VCENTER | GUI_TA_LEFT);
    LISTVIEW_SetGridVis(hItem, 0);
    LISTVIEW_SetAutoScrollV(hItem, 1);  
    WIDGET_SetEffect(hItem, &WIDGET_Effect_None);
    LISTVIEW_SetTextAlign(hItem, 0, GUI_TA_HCENTER); 
    
    if(VideoList.ptr > 0)
    {
      for(i=0; i<VideoList.ptr; i++)
      {
        strcpy(FileInfo.pRoot, (char *)VideoList.file[i].name);
        FILEMGR_GetFileOnly ((char *)tmp, (char *)FileInfo.pRoot);
        hItem = WM_GetDialogItem(pMsg->hWin, ID_VIDEO_LIST);
        
        LISTVIEW_AddRow(hItem, NULL);         
        LISTVIEW_SetItemText(hItem, 0, i, (char *)tmp);
      }
    }     
    break;
    
  case WM_NOTIFY_PARENT:
    Id    = WM_GetId(pMsg->hWinSrc);
    NCode = pMsg->Data.v;   
    switch(Id) {
    case ID_BUTTON_OKPL: /* Notifications sent by 'OK' */
      switch(NCode) {
      case WM_NOTIFICATION_RELEASED:
        GUI_EndDialog(pMsg->hWin, 0); 
        break;
      }
      break;
      
    case ID_BUTTON_ADDPL: 
      switch(NCode) {
      case WM_NOTIFICATION_RELEASED:
        FileInfo.pfGetData = k_GetData;
        FileInfo.pMask = acMask_video;     
        hItem = CHOOSEFILE_Create(pMsg->hWin, 48, 30, 330, 240, apDrives, GUI_COUNTOF(apDrives), 0, "Video files", 0, &FileInfo);
        WM_MakeModal(hItem);
        r = GUI_ExecCreatedDialog(hItem);
        if (r == 0) 
        {
          if((strstr(FileInfo.pRoot, ".emf")) || (strstr(FileInfo.pRoot, ".EMF")))
          {
            if(VideoList.ptr < FILEMGR_LIST_DEPDTH)
            {
              strcpy((char *)VideoList.file[VideoList.ptr].name, FileInfo.pRoot);
              FILEMGR_GetFileOnly ((char *)tmp, (char *)FileInfo.pRoot);
              hItem = WM_GetDialogItem(pMsg->hWin, ID_VIDEO_LIST);
              
              LISTVIEW_AddRow(hItem, NULL);         
              LISTVIEW_SetItemText(hItem, 0, VideoList.ptr, (char *)tmp);
              VideoList.ptr++;
            }
          }
          WM_InvalidateWindow(pMsg->hWin);
        }
        
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
  * @brief  Retrieve the file name from a full file path
  * @param  file: pointer to base path
  * @param  path: pointer to full path
  * @retval None
*/
static void FILEMGR_GetFileOnly (char *file, char *path)
{
  char *baseName1, *baseName2;
  baseName1 = strrchr(path,'/');
  baseName2 = strrchr(path,':');
  
  if(baseName1++) 
  { 
    strcpy(file, baseName1);
  }
  else 
  {
    if (baseName2++) 
    {
      strcpy(file, baseName2);
    }
    else
    {
      strcpy(file, path);
    }
  }
}

/**
  * @brief  Callback function for movie player. Uses multiple buffering if
  *         available to avoid tearing effects.
  * @param  hMem:         pointer to parent handle
  * @param  Notification: notification ID
  * @param  CurrentFrame: current playing frame
  * @retval None
  */
void _cbNotify(GUI_HMEM hMem, int Notification, U32 CurrentFrame) 
{ 

  char tmp[20]; 
  WM_HWIN  hItem;
  switch (Notification) 
  {   
  case GUI_MOVIE_NOTIFICATION_PREDRAW:
    GUI_SelectLayer(0); 
    GUI_MULTIBUF_Begin();
    
    
      FrameIndex = GUI_MOVIE_GetFrameIndex(hvideo);
      
      if(FrameIndex < PrevFrameIndex)
      {
        PrevFrameIndex = FrameIndex;
        previous_total = 0;
      }
      
      LostFrames += (FrameIndex - PrevFrameIndex);
      if((FrameIndex - previous_total) >= VIDEO_FRAME_TIME)
      {
        hItem = WM_GetDialogItem(playbackwin, ID_FPS);
        if((VIDEO_FRAME_TIME - LostFrames + 1) < VIDEO_FRAME_TIME)
        {
          sprintf(tmp, "Rate : %lu fps", VIDEO_FRAME_TIME - LostFrames + 4 < 15 ? 15 : VIDEO_FRAME_TIME - LostFrames + 4);
          TEXT_SetText(hItem, tmp);
        }
        LostFrames = 0;
        previous_total = FrameIndex;
      }
      PrevFrameIndex = FrameIndex + 1;    
    
    break;
    
  case GUI_MOVIE_NOTIFICATION_POSTDRAW:
    GUI_MULTIBUF_End();
    GUI_SelectLayer(1); 
    break;
    
  case GUI_MOVIE_NOTIFICATION_STOP:
    {
      
      if(GUI_MOVIE_GetFrameIndex(hvideo) >= (Video_Info.NumFrames - 1))
      { 
        _StopPlay(&hvideo, &Video_File);
        
        if(file_pos < (VideoList.ptr - 1))
        {
          file_pos++;
        }
        else 
        {        
          file_pos = 0; 
        }        
        _StartPlay(&hvideo, (char *)VideoList.file[file_pos].name, &Video_File, 0, 0);        
      }
    }
    break;
  }
}


/**
  * @brief  Start play
  * @param  filename: pointer to the video file name
  * @retval None
  */
static uint8_t _StartPlay(GUI_MOVIE_HANDLE *hmovie, char * filename, FIL * file, uint16_t x0, uint16_t y0) 
{

  static char tmp[FILEMGR_FILE_NAME_SIZE]; 
  int ms, frames;
  uint32_t duration;
  WM_HWIN  hItem;
  
  if(f_open(file, filename, FA_OPEN_EXISTING | FA_READ) == FR_OK)
  {
    
    
    PrevFrameIndex = 0;
    LostFrames = 0;
    previous_total = 0;
    
    GUI_MOVIE_GetInfoEx(_GetData, file, &Video_Info);
    
    *hmovie = GUI_MOVIE_CreateEx(_GetData, file, _cbNotify) ;
    GUI_MOVIE_SetPeriod (*hmovie, VIDEO_FRAME_TIME);
    
    if((Video_Info.xSize == 0) || (Video_Info.ySize == 0) || 
       (Video_Info.xSize > 1024) || (Video_Info.ySize > 768))
    {
      GUI_MOVIE_Delete(*hmovie);
      return 1;
    }
    
    GUI_MOVIE_Show(*hmovie, ((LCD_GetXSize() - Video_Info.xSize)/2), ((LCD_GetYSize() - Video_Info.ySize)/2), 0); 
    
    hTimer = WM_CreateTimer(playbackwin, 0, 1000, 0);  
    
    /* Update video total time */
    hItem = WM_GetDialogItem(playbackwin, ID_TIME);
    ms = Video_Info.msPerFrame;
    frames = Video_Info.NumFrames;
    duration = (frames * ms)/1000;
    sprintf((char *)tmp , "%02lu:%02lu", duration/60, duration%60 );
    TEXT_SetText(hItem, tmp);          
    WM_InvalidateWindow(playbackwin);  
    elapsed_time = 0;
    
    hItem = WM_GetDialogItem(playbackwin, ID_VIDEO_NAME);
    FILEMGR_GetFileOnly (tmp, filename);
    TEXT_SetText(hItem, tmp);
    
    hItem = WM_GetDialogItem(playbackwin, ID_ELAPSED_TIME);
    TEXT_SetText(hItem, "00:00");
    WM_InvalidateWindow(hItem);
    WM_Paint(hItem);
  }
  return 0;
}


/**
  * @brief  Stop play 
  * @param  None
  * @retval None
  */
static void _StopPlay(GUI_MOVIE_HANDLE *hmovie, FIL * file) 
{
  
  if(hTimer != 0)
  {
    WM_DeleteTimer(hTimer);
    hTimer = 0;
  }
  
  if(*hmovie != 0)
  {
    GUI_MOVIE_Delete (*hmovie);
    *hmovie = 0;
  }
  f_close(file); 

  WM_InvalidateWindow(hFrame);  
}

/**
  * @brief  callback for video window
  * @param  pMsg: pointer to a data structure of type WM_MESSAGE
  * @retval None
  */
static void _cbVideoWindow(WM_MESSAGE * pMsg) 
{
  GUI_RECT r;
 
  switch (pMsg->MsgId) 
  {
  case WM_PAINT:
    WM_GetInsideRect(&r);
    GUI_ClearRectEx(&r);
    break;
    
  default:
    WM_DefaultProc(pMsg);
  }
}

/**
  * @brief  Callback routine of the dialog
  * @param  pMsg: pointer to a data structure of type WM_MESSAGE
  * @retval None
  */
static void _cbDialog(WM_MESSAGE * pMsg) {
  WM_HWIN  hItem;
  int Id, NCode;

  static uint8_t sel = 0;
  
  switch (pMsg->MsgId) {
  case WM_INIT_DIALOG:

    memset(Video_Path, 0, 256);   
    
    hItem = BUTTON_CreateEx(574, 0, 65, 65, pMsg->hWin, WM_CF_SHOW, 0, ID_BUTTON_EXIT);
    WM_SetCallback(hItem, _cbButton_exit); 
    
    hItem = ST_AnimatedIconView_CreateEx(100, 
                                         70, 
                                         LCD_GetXSize() - 0, 
                                         LCD_GetYSize() - 30, 
                                         pMsg->hWin, 
                                         WM_CF_SHOW | WM_CF_HASTRANS ,
                                         0,
                                         ID_ICONVIEW_SUBMENU, 
                                         200, 
                                         250, 5, 5);    
    
    
    ST_AnimatedIconView_SetDualFont(hItem, GUI_FONT_20_1, GUI_FONT_20_1);
    
    ST_AnimatedIconView_SetSpace(hItem, GUI_COORD_Y, 5);
    ST_AnimatedIconView_SetSpace(hItem, GUI_COORD_X, 25);
    ST_AnimatedIconView_SetFrame(hItem, GUI_COORD_Y, 10);
    ST_AnimatedIconView_SetFrame(hItem, GUI_COORD_X, 5);
    
    ST_AnimatedIconView_SetSel(hItem, -1);
    
    ST_AnimatedIconView_SetTextColor(hItem, ICONVIEW_CI_UNSEL, 0x00DCA939);
    ST_AnimatedIconView_SetBkColor(hItem, ICONVIEW_CI_UNSEL, GUI_WHITE);
    ST_AnimatedIconView_SetBkColor(hItem, ICONVIEW_CI_SEL, GUI_WHITE);
    
    ST_AnimatedIconView_SetDualTextColor(hItem, ICONVIEW_CI_SEL, 0x00DCA939, 0x00522000);  
    
    ST_AnimatedIconView_AddIcon(hItem, open_file, 0, "Play video");   
    ST_AnimatedIconView_AddIcon(hItem, add_video, 0, "Add to playlist");    
    
    break;     

  case WM_PAINT: 
    GUI_SetColor(GUI_BLACK);
    GUI_DrawLine(639, 0, 639, 480);   

    break;
    
  case WM_NOTIFY_PARENT:
    Id    = WM_GetId(pMsg->hWinSrc);    /* Id of widget */
    NCode = pMsg->Data.v;               /* Notification code */   
    
    switch(Id) {
    case ID_BUTTON_EXIT: 
      switch(NCode) {
      case WM_NOTIFICATION_RELEASED:
        GUI_EndDialog(pMsg->hWin, 0);
        break;
      }
      break; 
      
      
    case ID_ICONVIEW_SUBMENU: 
      switch(NCode) {
      case WM_NOTIFICATION_RELEASED:      
        sel = ST_AnimatedIconView_GetSel(pMsg->hWinSrc);
        
        if(sel == 0)
        {

          osDelay(100);  
          /* Playlist not empty, so start play first item */
          if(VideoList.ptr > 0)
          {  
            GUI_SetLayerVisEx (1, 1);
            GUI_SelectLayer(1); 
            playbackwin = WM_CreateWindowAsChild(0, 0, 640, 480, WM_GetDesktopWindowEx(1), WM_CF_SHOW, _cbplaybackwin , 0);
            WM_CreateWindowAsChild(0, 70, 640, 300, WM_GetDesktopWindowEx(1), WM_CF_SHOW | WM_CF_HASTRANS, _cbTouch , 0);             
            GUI_SelectLayer(0);
            _StartPlay(&hvideo, (char *)VideoList.file[0].name, &Video_File, 0, 0);
            VideoPlayer_State = VIDEO_PLAY;
            hFrame = WM_CreateWindowAsChild(0, 0, 640, 480,pMsg->hWin, WM_CF_SHOW, _cbVideoWindow , 0);    
            GUI_SelectLayer(1);
            
          }
          else
          {/* There is no item yet in the playlist: Show hint message */
            hItem = GUI_CreateDialogBox(_aFileInfoDialogCreate, 
                                GUI_COUNTOF(_aFileInfoDialogCreate), 
                                _cbFileInfoDialog, 
                                pMsg->hWin, 
                                100, 
                                80);
            WM_MakeModal(hItem);
          }
        }
        else /* Add file to playlist icon item action */
        {
            hItem = GUI_CreateDialogBox(_aPlaylistDialogCreate, 
                                GUI_COUNTOF(_aPlaylistDialogCreate), 
                                _cbPlaylistDialog, 
                                pMsg->hWin, 
                                100, 
                                80);
            WM_MakeModal(hItem);
        }
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
  * @brief  Add entire folder to play list.
  * @param  Foldername: pointer to folder name.
  * @retval None
  */
static void _AddEntireFolder(char *Foldername)
{
  FRESULT res;
  FILINFO fno;
  DIR dir;
  char *fn;
  static char tmp[FILEMGR_FILE_NAME_SIZE]; 
  
#if _USE_LFN
  static char lfn[_MAX_LFN];
  fno.lfname = lfn;
  fno.lfsize = sizeof(lfn);
#endif
  
  res = f_opendir(&dir, Foldername);
  
  if (res == FR_OK)
  {
    
    while (1)
    {
      res = f_readdir(&dir, &fno);
      
      if (res != FR_OK || fno.fname[0] == 0)
      {
        break;
      }
      if (fno.fname[0] == '.')
      {
        continue;
      }
#if _USE_LFN
      fn = *fno.lfname ? fno.lfname : fno.fname;
#else
      fn = fno.fname;
#endif
      
      if (VideoList.ptr < FILEMGR_LIST_DEPDTH)
      {
        if ((fno.fattrib & AM_DIR) == 0)
        {
          if((strstr(fn, ".emf")) || (strstr(fn, ".EMF")))
          {
            strcpy(tmp, Foldername);
            strcat(tmp, "/");
            strcat(tmp, fn);
            
            strncpy((char *)VideoList.file[VideoList.ptr].name, (char *)tmp, FILEMGR_FILE_NAME_SIZE);
            VideoList.ptr++;
          }
        }
      }   
    }
  }
  f_closedir(&dir);
}

/**
  * @brief  Video window Starup
  * @param  hWin: pointer to the parent handle.
  * @param  xpos: X position 
  * @param  ypos: Y position
  * @retval None
  */
static void Startup(WM_HWIN hWin, uint16_t xpos, uint16_t ypos)
{
  VideoPlayer_State = VIDEO_IDLE;  
  AddStatus = 0;
  VideoList.ptr = 0;
  _AddEntireFolder("0:");
  _AddEntireFolder("0:/Video");
  VideoWin = GUI_CreateDialogBox(_aDialog, GUI_COUNTOF(_aDialog), _cbDialog, hWin, xpos, ypos);
  
  WM_CreateWindowAsChild(639, 479, 1, 1, VideoWin, WM_CF_SHOW | WM_CF_HASTRANS, _cbMediaConnection , 0);
}
/**
  * @}
  */

/**
  * @}
  */
  

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

