/**
  ******************************************************************************
  * @file    Audio/Audio_playback_and_record/Src/menu.c 
  * @author  MCD Application Team
  * @version V1.0.3
  * @date    18-November-2015
  * @brief   This file implements Menu Functions
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
/* Global extern variables ---------------------------------------------------*/
static volatile uint8_t MfxIrqReceived = 0;

/* Private variables ---------------------------------------------------------*/
AUDIO_DEMO_StateMachine     AudioDemo;
uint8_t                     PrevSelect = 0;
AUDIO_DEMO_SelectMode       AudioSelectMode;
AUDIO_PLAYBACK_StateTypeDef AudioState;

uint8_t *AUDIO_main_menu[] = 
{
  (uint8_t *)"      1 - Explore Audio File                                         ",
  (uint8_t *)"      2 - Start Audio Player                                         ",
  (uint8_t *)"      3 - Start Audio Recorder                                       ",
};

/* Private function prototypes -----------------------------------------------*/
static void AUDIO_SelectItem(uint8_t **menu, uint8_t item);
static void AUDIO_MenuProbeKey(JOYState_TypeDef state);
static void AUDIO_ChangeSelectMode(AUDIO_DEMO_SelectMode select_mode);
static void LCD_ClearTextZone(void);
static void Joystick_AudioMenu(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Demo state machine.
  * @param  None
  * @retval None
  */
void AUDIO_MenuInit(void)
{
  AudioDemo.state = AUDIO_DEMO_IDLE;
  AudioSelectMode = AUDIO_SELECT_MENU;
  AUDIO_SelectItem(AUDIO_main_menu, 0); 
}

/**
  * @brief  Manages AUDIO Menu Process.
  * @param  None
  * @retval None
  */
void AUDIO_MenuProcess(void)
{
  uint32_t ITstatus = 0;
  AUDIO_ErrorTypeDef  status;
  static uint32_t debounce_time = 0;
  
  if(appli_state == APPLICATION_READY)
  { 
    switch(AudioDemo.state)
    {
    case AUDIO_DEMO_IDLE:
      AUDIO_SelectItem(AUDIO_main_menu, 0); 
      if(AUDIO_ShowWavFiles() > 0)
      {
        LCD_ErrLog("There is no WAV file on the USB Key.\n");         
        AUDIO_ChangeSelectMode(AUDIO_SELECT_MENU); 
        AudioDemo.state = AUDIO_DEMO_IDLE;
      }
      else
      {
        AudioDemo.state = AUDIO_DEMO_WAIT;
      }
      
      AudioDemo.select = 0;
      BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
      BSP_LCD_DisplayStringAtLine(17, (uint8_t *)"Use [Joystick Left/Right] to scroll up/down");
      BSP_LCD_DisplayStringAtLine(18, (uint8_t *)"Use [Joystick Up/Down] to scroll Audio Playback and Record menu");
      break;    
      
    case AUDIO_DEMO_WAIT:
      if(AudioDemo.select != PrevSelect)
      {
        PrevSelect = AudioDemo.select;
        AUDIO_SelectItem(AUDIO_main_menu, AudioDemo.select & 0x7F);
        
        /* Handle select item */
        if(AudioDemo.select & 0x80)
        {
          
          switch(AudioDemo.select & 0x7F)
          {
          case 0:
            AudioDemo.state = AUDIO_DEMO_EXPLORE;  
            break;
            
          case 1:         
            /* Display HMI messages */
            BSP_LCD_SetTextColor(LCD_COLOR_GREEN);          
            BSP_LCD_DisplayStringAtLine(14 ,(uint8_t *)"                                                               ");
            BSP_LCD_DisplayStringAtLine(15 ,(uint8_t *)"                                                               ");
            BSP_LCD_DisplayStringAtLine(16 ,(uint8_t *)"                                                               ");
            BSP_LCD_DisplayStringAtLine(17 ,(uint8_t *)"                                                               ");
            BSP_LCD_DisplayStringAtLine(18 ,(uint8_t *)"Use [User Key] To Stop and return from player/recorder         ");
            BSP_LCD_SetTextColor(LCD_COLOR_WHITE); 
            
            /* Set PLAYBACK state and start playing 1st file */ 
            AudioState = AUDIO_STATE_IDLE;
            AudioDemo.state = AUDIO_DEMO_PLAYBACK;
            AUDIO_ChangeSelectMode(AUDIO_PLAYBACK_CONTROL);           
            break;
            
          case 2:
            /* Display HMI messages */
            BSP_LCD_SetTextColor(LCD_COLOR_GREEN);          
            BSP_LCD_DisplayStringAtLine(14 ,(uint8_t *)"                                                               ");
            BSP_LCD_DisplayStringAtLine(15 ,(uint8_t *)"                                                               ");
            BSP_LCD_DisplayStringAtLine(16 ,(uint8_t *)"                                                               ");
            BSP_LCD_DisplayStringAtLine(17 ,(uint8_t *)"                                                               ");
            BSP_LCD_DisplayStringAtLine(18 ,(uint8_t *)"Use [User Key] To Stop and return from player/recorder         ");
            BSP_LCD_SetTextColor(LCD_COLOR_WHITE); 
            
            /* Set PLAYBACK state and start playing 1st file */ 
            AudioState = AUDIO_STATE_IDLE;
            AudioDemo.state = AUDIO_DEMO_IN;  
            AUDIO_ChangeSelectMode(AUDIO_PLAYBACK_CONTROL); 
            break;
            
          default:
            break;
          }
        }
      }
      break;
      
    case AUDIO_DEMO_EXPLORE: 
      if(appli_state == APPLICATION_READY)
      {
        if(AUDIO_ShowWavFiles() > 0)
        {
          LCD_ErrLog("There is no WAV file on the USB Key.\n");         
          AUDIO_ChangeSelectMode(AUDIO_SELECT_MENU); 
          AudioDemo.state = AUDIO_DEMO_IDLE;
        }
        else
        {
          AudioDemo.state = AUDIO_DEMO_WAIT;
        }
      }
      else
      {
        AudioDemo.state = AUDIO_DEMO_WAIT;
      }
      break; 
      
    case AUDIO_DEMO_PLAYBACK:
      if(appli_state == APPLICATION_READY)
      {
        if(AudioState == AUDIO_STATE_IDLE)
        {
          /* Start Playing */
          AudioState = AUDIO_STATE_INIT;
          if(AUDIO_PLAYER_Start(0) == AUDIO_ERROR_IO)
          {
            AUDIO_ChangeSelectMode(AUDIO_SELECT_MENU); 
            AudioDemo.state = AUDIO_DEMO_IDLE;
          }
          else
          {
            BSP_LCD_SetTextColor(LCD_COLOR_YELLOW); 
            BSP_LCD_DisplayStringAtLine(10, (uint8_t *)"[  UP   ] : Volume +");
            BSP_LCD_DisplayStringAtLine(11, (uint8_t *)"[ DOWN  ] : Volume -");
            BSP_LCD_DisplayStringAtLine(12, (uint8_t *)"[ LEFT  ] : Previous");
            BSP_LCD_DisplayStringAtLine(13, (uint8_t *)"[ RIGHT ] : Next");
            BSP_LCD_DisplayStringAtLine(14, (uint8_t *)"[  SEL  ] : Pause/Resume");
            BSP_LCD_SetTextColor(LCD_COLOR_WHITE);           
          }
        }
        else /* Not idle */
        {
          if(AUDIO_PLAYER_Process() == AUDIO_ERROR_IO)
          {
            AUDIO_ChangeSelectMode(AUDIO_SELECT_MENU);  
            AudioDemo.state = AUDIO_DEMO_IDLE;
          }
        }
      }
      else
      {
        AudioDemo.state = AUDIO_DEMO_WAIT;
      }
      break; 
      
    case AUDIO_DEMO_IN:
      if(appli_state == APPLICATION_READY)
      {
        if(AudioState == AUDIO_STATE_IDLE)
        {
          /* Start Playing */
          AudioState = AUDIO_STATE_INIT;
          /* Configure the audio recorder: sampling frequency, bits-depth, number of channels */
          if(AUDIO_REC_Start() == AUDIO_ERROR_IO)
          {
            AUDIO_ChangeSelectMode(AUDIO_SELECT_MENU); 
            AudioDemo.state = AUDIO_DEMO_IDLE;
          }
          else
          {
            BSP_LCD_SetTextColor(LCD_COLOR_YELLOW); 
            BSP_LCD_DisplayStringAtLine(11, (uint8_t *)"[  UP   ] : Volume +");
            BSP_LCD_DisplayStringAtLine(12, (uint8_t *)"[ DOWN  ] : Volume -");
            BSP_LCD_DisplayStringAtLine(13, (uint8_t *)"[  SEL  ] : Pause/Resume");
            BSP_LCD_SetTextColor(LCD_COLOR_WHITE); 
          }
        }
        else /* Not idle */
        {
          status = AUDIO_REC_Process();
          if((status == AUDIO_ERROR_IO) || (status == AUDIO_ERROR_EOF))
          {
            AUDIO_ChangeSelectMode(AUDIO_SELECT_MENU);  
            AudioDemo.state = AUDIO_DEMO_IDLE;
          }
        }
      }
      else
      {
        AudioDemo.state = AUDIO_DEMO_WAIT;
      }
      break;
      
    default:
      break;
    }
  }

  if(appli_state == APPLICATION_DISCONNECT)
  {
    appli_state = APPLICATION_IDLE;     
    AUDIO_ChangeSelectMode(AUDIO_SELECT_MENU); 
    BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);    
  }
  AudioDemo.select &= 0x7F;

  if (MfxIrqReceived == 1)
  {
      MfxIrqReceived = 0;
      ITstatus = BSP_IO_ITGetStatus(JOY_ALL_PINS);
      if (ITstatus)
      {
        /* Prevent debounce effect for user key */
        if((HAL_GetTick() - debounce_time) > 200)
        {
          debounce_time = HAL_GetTick();
          Joystick_AudioMenu();
        }
        /* Clear joystick interrupt pending bits */
        BSP_IO_ITClear();
      }
  }
} 


/**
  * @brief  Joystick audio menu
  * @param  None
  * @retval None
  */
void Joystick_AudioMenu(void)
{
  static JOYState_TypeDef JoyState = JOY_NONE;

    /* Get the Joystick State */
    JoyState = BSP_JOY_GetState();

    
    if(AudioSelectMode == AUDIO_SELECT_MENU)
    {  
      AUDIO_MenuProbeKey(JoyState); 
      
      switch(JoyState)
      {
      case JOY_LEFT:
        LCD_LOG_ScrollBack();
        break;
        
      case JOY_RIGHT:
        LCD_LOG_ScrollForward();
        break;          
        
      default:
        break;           
      }
    }
    else if(AudioSelectMode == AUDIO_PLAYBACK_CONTROL)
    {
      AUDIO_PlaybackProbeKey(JoyState);
    }

}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t debounce_time = 0;

  if(GPIO_Pin == MFX_IRQOUT_PIN)
  {
    /* The different functionalities of MFX (TS, Joystick, SD detection, etc, ) 
     can be configured in EXTI mode to generate an IRQ on IO transition events. 
     Querying the IO states is based on I2C communication that is not recommended 
     to be called within ISR context to prevent blocking the interrupt. 
     HMI devices such as TS, Joystick, SD detection are relatively slow and heavy 
     processes. So, it is recommended to use polling and interrupt based modes 
     together: the query on IO states in the polling statement is triggered through 
     a flag that is set in the ISR */

    /* Here is an example of implementation based on a mix between polling and 
     interrupt mode: On ISR a flag is set (MfxIrqAsserted) and in the main loop, 
     the flag is checked before handling the I2C process*/
    MfxIrqReceived = 1;
  }
  
  if(AudioDemo.state == AUDIO_DEMO_PLAYBACK)
  {
    if(GPIO_Pin == KEY_BUTTON_PIN)
    { 
      /* Prevent debounce effect for user key */
      if((HAL_GetTick() - debounce_time) > 50)
      {
        debounce_time = HAL_GetTick();
      }
      else
      {
        return;
      }
      
      /* Change the selection type */
      if(AudioSelectMode == AUDIO_SELECT_MENU)
      {
        AUDIO_ChangeSelectMode(AUDIO_PLAYBACK_CONTROL); 
      }
      else if(AudioSelectMode == AUDIO_PLAYBACK_CONTROL)
      {       
        AUDIO_PLAYER_Stop();
      }
    }
  }

  if(AudioDemo.state == AUDIO_DEMO_IN)
  {
    if(GPIO_Pin == KEY_BUTTON_PIN)
    { 
      /* Prevent debounce effect for user key */
      if((HAL_GetTick() - debounce_time) > 50)
      {
        debounce_time = HAL_GetTick();
      }
      else
      {
        return;
      }
      
      AudioState = AUDIO_STATE_STOP;
    }
  }
}

/*******************************************************************************
                            Static Functions
*******************************************************************************/

/**
  * @brief  Changes the selection mode.
  * @param  select_mode: Selection mode
  * @retval None
  */
static void AUDIO_ChangeSelectMode(AUDIO_DEMO_SelectMode select_mode)
{
  if(select_mode == AUDIO_SELECT_MENU)
  {
    AUDIO_SelectItem(AUDIO_main_menu, 0x00);
    LCD_LOG_UpdateDisplay(); 
    AudioDemo.state = AUDIO_DEMO_IDLE; 
    AUDIO_PLAYER_Stop();
  }
  else if(select_mode == AUDIO_PLAYBACK_CONTROL)
  {
    LCD_ClearTextZone();
    AUDIO_SelectItem(AUDIO_main_menu, 0xFF);     
  }
  AudioSelectMode = select_mode; 
  AudioDemo.select = 0;
}

/**
  * @brief  Manages the menu on the screen.
  * @param  menu: Menu table
  * @param  item: Selected item to be highlighted
  * @retval None
  */
static void AUDIO_SelectItem(uint8_t **menu, uint8_t item)
{
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  
  switch(item)
  {
  case 0: 
    BSP_LCD_SetBackColor(LCD_COLOR_MAGENTA);
    BSP_LCD_DisplayStringAtLine(19, menu [0]);
    BSP_LCD_SetBackColor(LCD_COLOR_BLUE);    
    BSP_LCD_DisplayStringAtLine(20, menu [1]);
    BSP_LCD_DisplayStringAtLine(21, menu [2]);
    break;
    
  case 1: 
    BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
    BSP_LCD_DisplayStringAtLine(19, menu [0]);
    BSP_LCD_SetBackColor(LCD_COLOR_MAGENTA);    
    BSP_LCD_DisplayStringAtLine(20, menu [1]);
    BSP_LCD_SetBackColor(LCD_COLOR_BLUE);  
    BSP_LCD_DisplayStringAtLine(21, menu [2]); 
    break;
    
  case 2: 
    BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
    BSP_LCD_DisplayStringAtLine(19, menu [0]);  
    BSP_LCD_DisplayStringAtLine(20, menu [1]);
    BSP_LCD_SetBackColor(LCD_COLOR_MAGENTA);  
    BSP_LCD_DisplayStringAtLine(21, menu [2]); 
    break;
    
  default:
    BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
    BSP_LCD_DisplayStringAtLine(19, menu [0]);
    BSP_LCD_DisplayStringAtLine(20, menu [1]);
    BSP_LCD_DisplayStringAtLine(21, menu [2]); 
    break; 
  }
  BSP_LCD_SetBackColor(LCD_COLOR_BLACK); 
}

/**
  * @brief  Probes the Audio joystick state.
  * @param  state: Joystick state
  * @retval None
  */
static void AUDIO_MenuProbeKey(JOYState_TypeDef state)
{
  /* Handle Menu inputs */
  if((state == JOY_UP) && (AudioDemo.select > 0))
  {
    AudioDemo.select--;
  }
  else if((state == JOY_DOWN) && (AudioDemo.select < 2))
  {
    AudioDemo.select++;
  }
  else if(state == JOY_SEL)
  {
    AudioDemo.select |= 0x80;
  }  
}

/**
  * @brief  Clears the text zone.
  * @param  None
  * @retval None
  */
static void LCD_ClearTextZone(void)
{
  uint8_t i = 0;
  
  for(i= 0; i < 13; i++)
  {
    BSP_LCD_ClearStringLine(i + 3);
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
