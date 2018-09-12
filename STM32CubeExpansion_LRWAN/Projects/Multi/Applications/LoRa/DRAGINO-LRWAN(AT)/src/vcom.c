 /******************************************************************************
  * @file    vcom.c
  * @author  MCD Application Team
  * @version V1.1.2
  * @date    08-September-2017
  * @brief   manages virtual com port
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
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
  
#include "hw.h"
#include "vcom.h"
#include <stdarg.h>
#include "tiny_vsnprintf.h"
#include "low_power.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* based on UART_HandleTypeDef */
static struct {
  char buffTx[256];                   /**< buffer to transmit */
  char buffRx[256];                   /**< Circular buffer of received chars */
  int rx_idx_free;                    /**< 1st free index in BuffRx */
  int rx_idx_toread;                  /**< next char to read in buffRx, when not rx_idx_free */
  HW_LockTypeDef Lock;                /**< Locking object */

  __IO HAL_UART_StateTypeDef gState;  /**< UART state information related to global Handle management
                                           and also related to Tx operations. */
  __IO HAL_UART_StateTypeDef RxState; /**< UART state information related to Rx operations. */
} uart_context;

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief  Transmit uart_context.buffTx from start to len - 1
 * @param  1st index to transmit
 * @param  Last index not to transmit
 * @return Last index not transmitted
 */
static int buffer_transmit(int start, int len);

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BUFSIZE 256
//#define USARTX_IRQn USART1_IRQn
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* buffer */
static char buff[BUFSIZE];
/* buffer write index*/
__IO uint16_t iw=0;
/* buffer read index*/
static uint16_t ir=0;
/* Uart Handle */
UART_HandleTypeDef UartHandle;
UART_WakeUpTypeDef WakeUpSelection; 
/* Private function prototypes -----------------------------------------------*/
/* Functions Definition ------------------------------------------------------*/

void vcom_Init(void)
{
  /*## Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = ODD parity
      - BaudRate = 921600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
	
	__HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);
	__HAL_RCC_LPUART1_CONFIG(RCC_LPUART1CLKSOURCE_HSI);
  UartHandle.Instance        = USARTX;
  
  UartHandle.Init.BaudRate   = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }	

  /* set the wake-up event:
   * specify wake-up on start bit */
  WakeUpSelection.WakeUpEvent = UART_WAKEUP_ON_STARTBIT;
  if (HAL_UARTEx_StopModeWakeUpSourceConfig(&UartHandle, WakeUpSelection)!= HAL_OK)
  {
    Error_Handler(); 
  }
 		/* Enable UART Stop Mode. */
  HAL_UARTEx_EnableStopMode(&UartHandle);
	
	  /* Enable the UART Wake UP from stop mode Interrupt */
  __HAL_UART_ENABLE_IT(&UartHandle, UART_IT_WUF);
	
  __HAL_UART_ENABLE(&UartHandle);

	  /* make sure that no UART transfer is on-going */ 
   while(__HAL_UART_GET_FLAG(&UartHandle, USART_ISR_BUSY) == SET);
   /* make sure that UART is ready to receive
   * (test carried out again later in HAL_UARTEx_StopModeWakeUpSourceConfig) */   
   while(__HAL_UART_GET_FLAG(&UartHandle, USART_ISR_REACK) == RESET);
	
	 uart_context.gState = HAL_UART_STATE_READY;
   uart_context.RxState = HAL_UART_STATE_READY;
}


void vcom_DeInit(void)
{
#if 1
  HAL_UART_DeInit(&UartHandle);
#endif
}
void vcom_Send_f(const char *format, ...)
{
  va_list args;
  va_start(args, format);
  
  /*convert into string at buff[0] of length iw*/
  iw = vsprintf(&buff[0], format, args);
  
  HAL_UART_Transmit(&UartHandle,(uint8_t *)&buff[0], iw, 300);
  
  va_end(args);
}
void vcom_Send(const char *format, ...)
{
  va_list args;
  static __IO uint16_t len = 0;
  uint16_t current_len;
  int start;
  int stop;

  va_start(args, format);

  BACKUP_PRIMASK();
  DISABLE_IRQ();
  if (len != 0)
  {
    if (len != sizeof(uart_context.buffTx))
    {
      current_len = len; /* use current_len instead of volatile len in below computation */
      len = current_len + tiny_vsnprintf_like(uart_context.buffTx + current_len, \
                                              sizeof(uart_context.buffTx) - current_len, format, args);
    }
    RESTORE_PRIMASK();
    va_end(args);
    return;
  }
  else
   {
    len = tiny_vsnprintf_like(uart_context.buffTx, sizeof(uart_context.buffTx), format, args);
  }

  current_len = len;
  RESTORE_PRIMASK();

  start = 0;
  
  do
  {
    stop = buffer_transmit(start, current_len);

    {  
      BACKUP_PRIMASK();
      DISABLE_IRQ();
      if (len == stop)
      {
        len = 0;
        RESTORE_PRIMASK();
      }
      else
      {
        start = stop;
        current_len = len;
        RESTORE_PRIMASK();
      }
    }
  } while (current_len != stop);

  va_end(args);
}
/* Private functions Definition ------------------------------------------------------*/

static int buffer_transmit(int start, int len)
{
  int i;
  for (i = start; i < len; i++)
  {
		WRITE_REG(USARTX->ICR, USART_ICR_TCCF);
		
		//__HAL_UART_CLEAR_FLAG(&UartHandle,UART_CLEAR_TCF);
    
    USARTX->TDR = uart_context.buffTx[i];
		
    while (READ_BIT(USARTX->ISR, USART_ISR_TC) == (USART_ISR_TC) != SET)
    {
      ;
    }
  }
  WRITE_REG(USARTX->ICR, USART_ICR_TCCF);
  return len;
}

FlagStatus IsNewCharReceived(void)
{
  FlagStatus status;
  
  BACKUP_PRIMASK();
  DISABLE_IRQ();
  
  status = ((uart_context.rx_idx_toread == uart_context.rx_idx_free) ? RESET : SET);
  
  RESTORE_PRIMASK();
  return status;
}
uint8_t GetNewChar(void)
{
  uint8_t NewChar;

  BACKUP_PRIMASK();
  DISABLE_IRQ();

  NewChar = uart_context.buffRx[uart_context.rx_idx_toread];
  uart_context.rx_idx_toread = (uart_context.rx_idx_toread + 1) % sizeof(uart_context.buffRx);

  RESTORE_PRIMASK();
  return NewChar;
}
void vcom_ReceiveInit(void)
{
  if (uart_context.RxState != HAL_UART_STATE_READY)
  {
    return;
  }

  /* Process Locked */
  HW_LOCK(&uart_context);

  uart_context.RxState = HAL_UART_STATE_BUSY_RX;

  /* Enable the UART Parity Error Interrupt */
  SET_BIT(USARTX->CR1, USART_CR1_PEIE);
  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */

  SET_BIT(USARTX->CR3, USART_CR3_EIE);
  /* Process Unlocked */
  HW_UNLOCK(&uart_context);
}
static void receive(char rx)
{
  int next_free;

  /** no need to clear the RXNE flag because it is auto cleared by reading the data*/
  uart_context.buffRx[uart_context.rx_idx_free] = rx;
  next_free = (uart_context.rx_idx_free + 1) % sizeof(uart_context.buffRx);
  if (next_free != uart_context.rx_idx_toread)
  {
    /* this is ok to read as there is no buffer overflow in input */
    uart_context.rx_idx_free = next_free;
  }
  else
  {
    /* force the end of a command in case of overflow so that we can process it */
    uart_context.buffRx[uart_context.rx_idx_free] = '\r';
    PRINTF("uart_context.buffRx buffer overflow %d\r\n");
  }
}
void vcom_Print( void)
{
  char* CurChar;
  while( ( (iw+BUFSIZE-ir)%BUFSIZE) >0 )
  {
    BACKUP_PRIMASK();
    DISABLE_IRQ();
    
    CurChar = &buff[ir];
    ir= (ir+1) %BUFSIZE;
    
    RESTORE_PRIMASK();
    
    HAL_UART_Transmit(&UartHandle,(uint8_t *) CurChar, 1, 300);    
  }
  HAL_NVIC_ClearPendingIRQ(USARTX_IRQn);
}
void vcom_IRQHandler(UART_HandleTypeDef *huart)
{
	int rx_ready = 0;
  char rx;
	uint32_t isrflags   = READ_REG(huart->Instance->ISR);
  uint32_t cr1its     = READ_REG(huart->Instance->CR1);
  uint32_t cr3its = READ_REG(huart->Instance->CR3);;
  uint32_t errorflags;
	
	    /* UART wakeup from Stop mode interrupt occurred ---------------------------*/
    if(((isrflags & USART_ISR_WUF) != RESET) && ((cr3its & USART_CR3_WUFIE) != RESET))
    {
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_WUF);
      
       /* forbid stop mode */
      LowPower_Disable(e_LOW_POWER_UART);  
       
      /* Enable the UART Data Register not empty Interrupts */
      SET_BIT(huart->Instance->CR1, USART_CR1_RXNEIE);
    
      /* Set the UART state ready to be able to start again the process */
      huart->gState  = HAL_UART_STATE_READY;
      huart->RxState = HAL_UART_STATE_READY;
    }
		
		/* UART in mode Receiver ---------------------------------------------------*/
    if(((isrflags & USART_ISR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
    {
		        /*RXNE flag is auto cleared by reading the data*/
             rx= (uint8_t)READ_REG(huart->Instance->RDR);	                        
             /* allow stop mode*/
             LowPower_Enable(e_LOW_POWER_UART);                       			
		         rx_ready = 1;
		}
			/* If error occurs */
     errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));
     if (errorflags != RESET)
     {
			 printf("Error when received");
	   /* Error on receiving */ 
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_PEF);	  
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_FEF);
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF);     
        __HAL_UART_CLEAR_IT(huart, UART_CLEAR_NEF);
	   rx_ready = 1;
	   }
	if(rx_ready)
	{
	  receive(rx);
		//PRINTF("%c",rx);
	}
}
void vcom_Send_Lp( char *format, ... )
{
  va_list args;
  va_start(args, format);
  uint8_t len;
  uint8_t lenTop;
  char tempBuff[128];
  
  BACKUP_PRIMASK();
  DISABLE_IRQ();
  
  /*convert into string at buff[0] of length iw*/
  len = vsprintf(&tempBuff[0], format, args);
  
  if (iw+len<BUFSIZE)
  {
    memcpy( &buff[iw], &tempBuff[0], len);
    iw+=len;
  }
  else
  {
    lenTop=BUFSIZE-iw;
    memcpy( &buff[iw], &tempBuff[0], lenTop);
    len-=lenTop;
    memcpy( &buff[0], &tempBuff[lenTop], len);
    iw = len;
  }
  RESTORE_PRIMASK();  
  
  va_end(args);
}
/**
  * @brief UART MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - NVIC configuration for UART interrupt request enable
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{ 
  /*##-2- Configure peripheral GPIO ##########################################*/  
  vcom_IoInit();
}

void vcom_IoInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct={0};
    /* Enable GPIO TX/RX clock */
  USARTX_TX_GPIO_CLK_ENABLE();
  USARTX_RX_GPIO_CLK_ENABLE();
	  /* Enable USARTX clock */
  USARTX_CLK_ENABLE(); 
    /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = USARTX_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = USARTX_TX_AF;

  HAL_GPIO_Init(USARTX_TX_GPIO_PORT, &GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USARTX_RX_PIN;
  GPIO_InitStruct.Alternate = USARTX_RX_AF;

  HAL_GPIO_Init(USARTX_RX_GPIO_PORT, &GPIO_InitStruct);
	
	HAL_NVIC_SetPriority(USARTX_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USARTX_IRQn);
 
}

void vcom_IoDeInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure={0};
  
  USARTX_TX_GPIO_CLK_ENABLE();
  USARTX_RX_GPIO_CLK_ENABLE();

  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  
  GPIO_InitStructure.Pin =  USARTX_TX_PIN ;
  HAL_GPIO_Init(  USARTX_TX_GPIO_PORT, &GPIO_InitStructure );
  
  GPIO_InitStructure.Pin =  USARTX_RX_PIN ;
  HAL_GPIO_Init(  USARTX_RX_GPIO_PORT, &GPIO_InitStructure ); 
}

/**
  * @brief UART MSP DeInit
  * @param huart: uart handle
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  vcom_IoDeInit( );
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
