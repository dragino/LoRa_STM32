/**
 ******************************************************************************
 * @file    low_power_manager.c
 * @author  MCD Application Team
 * @version V1.1.4
 * @date    08-January-2018
 * @brief   Low Power Manager
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "low_power_manager.h"

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint32_t StopModeDisable = 0;
static uint32_t OffModeDisable = 0;

/* Global variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Functions Definition ------------------------------------------------------*/
void LPM_SetOffMode(LPM_Id_t id, LPM_SetMode_t mode)
{
  BACKUP_PRIMASK();
  
  DISABLE_IRQ( );
  
  
  switch(mode)
  {
    case LPM_Disable:
    {
      OffModeDisable |= (uint32_t)id;
      break;
    }
    case LPM_Enable:
    {
      OffModeDisable &= ~(uint32_t)id;
      break;
    }
    default:
      break;
  }
  
  RESTORE_PRIMASK( );

  return;
}

void LPM_SetStopMode(LPM_Id_t id, LPM_SetMode_t mode)
{
  BACKUP_PRIMASK();
  
  DISABLE_IRQ( );
  
  
  switch(mode)
  {
    case LPM_Disable:
    {
      StopModeDisable |= (uint32_t)id;
      break;
    }
    case LPM_Enable:
    {
      StopModeDisable &= ~(uint32_t)id;
      break;
    }
    default:
      break;
  }
  RESTORE_PRIMASK( );

  return;
}

void LPM_EnterLowPower(void)
{
  if( StopModeDisable )
  {
    /**
     * SLEEP mode is required
     */
    LPM_EnterSleepMode();
    LPM_ExitSleepMode();
  }
  else
  { 
    if( OffModeDisable )
    {
      /**
       * STOP mode is required
       */
      LPM_EnterStopMode();
      LPM_ExitStopMode();
    }
    else
    {
      /**
       * OFF mode is required
       */
      LPM_EnterOffMode();
      LPM_ExitOffMode();
    }
  }

  return;
}

LPM_GetMode_t LPM_GetMode(void)
{
  LPM_GetMode_t mode_selected;

  BACKUP_PRIMASK();
  
  DISABLE_IRQ( );

  if(StopModeDisable )
  {
    mode_selected = LPM_SleepMode;
  }
  else
  {
    if(OffModeDisable)
    {
      mode_selected = LPM_StopMode;
    }
    else
    {
      mode_selected = LPM_OffMode;
    }
  }

  RESTORE_PRIMASK( );

  return mode_selected;
}

__weak void LPM_EnterSleepMode(void) {}
__weak void LPM_ExitSleepMode(void) {}
__weak void LPM_EnterStopMode(void) {}
__weak void LPM_ExitStopMode(void) {}
__weak void LPM_EnterOffMode(void) {}
__weak void LPM_ExitOffMode(void) {}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
