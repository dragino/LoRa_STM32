/**
  ******************************************************************************
  * @file    trace.c
  * @author  MCD Application Team
  * @brief   This file contains the Interface with BLE Drivers functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright c 2017 STMicroelectronics International N.V.
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
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include "utilities.h"
#include "queue.h"
#include "trace.h"
#include "low_power_manager.h"
#include "debug.h"
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

#define TEMPBUFSIZE 256

/* Private variables ---------------------------------------------------------*/
static queue_t MsgTraceQueue;
static uint8_t MsgTraceQueueBuff[DBG_TRACE_MSG_QUEUE_SIZE];

__IO ITStatus TracePeripheralReady = SET;

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief  Trace buffer Transfer completed callback
 * @param  none
 * @note   Indicate the end of the transmission of a  trace buffer. If queue
 *         contains new trace data to transmit, start a new transmission.
 * @retval None
 */
static void Trace_TxCpltCallback(void);

/* Functions Definition ------------------------------------------------------*/
void TraceInit( void )
{
  OutputInit(Trace_TxCpltCallback);

  CircularQueue_Init(&MsgTraceQueue, MsgTraceQueueBuff, DBG_TRACE_MSG_QUEUE_SIZE, 0, CIRCULAR_QUEUE_SPLIT_IF_WRAPPING_FLAG);

  return;
}

int32_t TraceSend( const char *strFormat, ...)
{
  int32_t status =0;
  char buf[TEMPBUFSIZE];
  va_list vaArgs;
  uint8_t* buffer;
  va_start( vaArgs, strFormat);
  uint16_t bufSize=vsnprintf(buf,TEMPBUFSIZE,strFormat, vaArgs);
  va_end(vaArgs);
  
  BACKUP_PRIMASK();
  
  DISABLE_IRQ(); /**< Disable all interrupts by setting PRIMASK bit on Cortex*/
  //DBG_GPIO_SET(GPIOB, GPIO_PIN_15);
  //DBG_GPIO_RST(GPIOB, GPIO_PIN_15);
  buffer=CircularQueue_Add(&MsgTraceQueue,(uint8_t*)buf, bufSize,1);
  
  if ((buffer!=NULL) && (TracePeripheralReady==SET))
  {
    buffer=CircularQueue_Sense(&MsgTraceQueue,&bufSize);
    TracePeripheralReady = RESET;
    //DBG_GPIO_RST(GPIOB, GPIO_PIN_12);
    LPM_SetStopMode(LPM_UART_TX_Id , LPM_Disable );

    RESTORE_PRIMASK();
    OutputTrace((uint8_t*)buffer, bufSize);
  }
  else
  {
    RESTORE_PRIMASK();
  }
  
  if (buffer!=NULL)
  {
    status=0;
  }
  else
  {
    status =-1;
  }
  return status;
}

const char *TraceGetFileName(const char *fullpath)
{
  const char *ret = fullpath;

  if (strrchr(fullpath, '\\') != NULL)
  {
    ret = strrchr(fullpath, '\\') + 1;
  }
  else if (strrchr(fullpath, '/') != NULL)
  {
    ret = strrchr(fullpath, '/') + 1;
  }

  return ret;
}

/* Private Functions Definition ------------------------------------------------------*/

static void Trace_TxCpltCallback(void)
{
  uint8_t* buffer;
  uint16_t bufSize;

  BACKUP_PRIMASK();

  DISABLE_IRQ(); /**< Disable all interrupts by setting PRIMASK bit on Cortex*/
  /* Remove element just sent to UART */
  CircularQueue_Remove(&MsgTraceQueue,&bufSize);
  //DBG_GPIO_SET(GPIOB, GPIO_PIN_13);
  //DBG_GPIO_RST(GPIOB, GPIO_PIN_13);
  /* Sense if new data to be sent */
  buffer=CircularQueue_Sense(&MsgTraceQueue,&bufSize);

  if ( buffer != NULL) 
  {
    RESTORE_PRIMASK();
    //DBG_GPIO_SET(GPIOB, GPIO_PIN_14);
    //DBG_GPIO_RST(GPIOB, GPIO_PIN_14);
    OutputTrace((uint8_t*)buffer, bufSize);
  }
  else
  {
    //DBG_GPIO_SET(GPIOB, GPIO_PIN_12);

    LPM_SetStopMode(LPM_UART_TX_Id , LPM_Enable );
    TracePeripheralReady = SET;
    RESTORE_PRIMASK();
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
