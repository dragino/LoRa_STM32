 /******************************************************************************
  * @file    lubcos.c
  * @author  Preddata
  * @version V1.1.0
  * @date    2022-02-21
  * @brief   manages the sensors on the application
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics International N.V. 
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
#include <stdlib.h>
	
#include "hw.h"
#include "vcom.h"
#include "lubcos.h"
#include "delay.h"
#include "string.h"

// #include "timeServer.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef UartHandle1;


void lubcos_read_serial(lubcos_serial_reading_t *lubcos)
{
	uart1_IoInit();
	at_lubcos_data_receive(lubcos, 1000);
	uart1_IoDeInit();
}

// Tutorial com strings
// https://simonmartin.ch/resources/stm32/dl/STM32%20Tutorial%2003%20-%20UART%20Communication%20using%20HAL%20(and%20FreeRTOS).pdf
void uart1_init_lubcos(void)
{
	UartHandle1.Instance                    = USART1;

  UartHandle1.Init.BaudRate               = 9600;
  UartHandle1.Init.WordLength             = UART_WORDLENGTH_8B;
  UartHandle1.Init.StopBits               = UART_STOPBITS_1;
  UartHandle1.Init.Parity                 = UART_PARITY_NONE;
  UartHandle1.Init.Mode                   = UART_MODE_TX_RX;
  UartHandle1.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
	
	
	if(HAL_UART_Init(&UartHandle1) != HAL_OK)
  {
    Error_Handler();
  }
}
