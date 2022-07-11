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
#include "digital_inputs.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

void GPIOB_INPUT_PULLUP_PIN_IoDeInit(uint16_t PIN)
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
  HW_GPIO_Init( GPIOB, PIN, &GPIO_InitStruct );
}


void GPIOB_INPUT_PULLUP_PIN_IoInit(uint16_t PIN)
{
	// Desativa configuracoes anteriores
	GPIOB_INPUT_PULLUP_PIN_IoDeInit(PIN);
	
	
	// Inicializa clock
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	
	// Inicializa porta
	GPIO_InitTypeDef GPIO_InitStruct={0};
	
	GPIO_InitStruct.Pin = PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HW_GPIO_Init( GPIOB, PIN, &GPIO_InitStruct );
}


void GPIOB_EXTI_FALLINGEDGE_PULLUP_PIN_Init(uint16_t PIN)
{
	// Desativa configuracoes anteriores
	GPIOB_INPUT_PULLUP_PIN_IoDeInit(PIN);
	
	
	// Inicializa clock
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	
	// Inicializa Interrupcao
	GPIO_InitTypeDef GPIO_InitStruct={0};
	
	
	GPIO_InitStruct.Mode	= GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull	= GPIO_PULLUP;
	GPIO_InitStruct.Pin		= PIN;
  HW_GPIO_Init( GPIOB, PIN, &GPIO_InitStruct );
	
	/* Enable and set EXTI lines 4 to 15 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}
