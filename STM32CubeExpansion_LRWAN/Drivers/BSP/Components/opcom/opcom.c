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
#include "opcom.h"
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


void opcom_read_serial(opcom_serial_reading_t *opcom)
{
	uart1_IoInit();
	at_opcom_data_receive(opcom, 1000);
	uart1_IoDeInit();
}

// Tutorial com strings
// https://simonmartin.ch/resources/stm32/dl/STM32%20Tutorial%2003%20-%20UART%20Communication%20using%20HAL%20(and%20FreeRTOS).pdf
void uart1_init_opcom(void)
{
	UartHandle1.Instance                    = USART1;

  UartHandle1.Init.BaudRate               = 57600;
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


void opcom_GPIO_IoDeInit( void )
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
  HW_GPIO_Init( GPIOB, GPIO_PIN_6, &GPIO_InitStruct );
}


void opcom_GPIO_EXTI_FALLINGInit( void )
{
	
	opcom_GPIO_IoDeInit();
	
	GPIO_InitTypeDef GPIO_InitStruct={0};
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	GPIO_InitStruct.Mode	= GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull	= GPIO_PULLUP;
	GPIO_InitStruct.Pin		= GPIO_PIN_6;

  HW_GPIO_Init( GPIOB, GPIO_PIN_6, &GPIO_InitStruct );
	
	/* Enable and set EXTI lines 4 to 15 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}


void opcom_GPIO_INPUTS_IoInit(void)
{
	
	opcom_GPIO_IoDeInit();
	
	
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	GPIO_InitTypeDef GPIO_InitStruct1={0};
	GPIO_InitTypeDef GPIO_InitStruct2={0};
	GPIO_InitTypeDef GPIO_InitStruct3={0};

	
	//PB6
	GPIO_InitStruct1.Pin = GPIO_PIN_6;
	GPIO_InitStruct1.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct1.Pull = GPIO_PULLUP;
  GPIO_InitStruct1.Speed = GPIO_SPEED_HIGH;
  HW_GPIO_Init( GPIOB, GPIO_PIN_6, &GPIO_InitStruct1 );
	
	
	//PB7
	GPIO_InitStruct2.Pin = GPIO_PIN_7;
	GPIO_InitStruct2.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct2.Pull = GPIO_PULLUP;
  GPIO_InitStruct2.Speed = GPIO_SPEED_HIGH;
  HW_GPIO_Init( GPIOB, GPIO_PIN_7, &GPIO_InitStruct2 );
	
	
	//PB3
	GPIO_InitStruct3.Pin = GPIO_PIN_3;
	GPIO_InitStruct3.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct3.Pull = GPIO_PULLUP;
  GPIO_InitStruct3.Speed = GPIO_SPEED_HIGH;
  HW_GPIO_Init( GPIOB, GPIO_PIN_3, &GPIO_InitStruct3 );
}
