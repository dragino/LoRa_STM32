 /******************************************************************************
  * @file    ogpio_exti.c
  * @author  MCD Application Team
  * @version V1.1.2
  * @date    01-June-2017
  * @brief   manages the sensors on the application
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
  
  /* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "gpio_exti.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

void GPIO_EXTI4_IoInit(uint8_t state)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_EXTI4_CLK_ENABLE();	
	
	GPIO_InitStruct.Pin = GPIO_EXTI4_PIN;	
	
	if(state == 0)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	}
	else if(state == 1)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	}
	else if(state == 2)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	}
	else if(state == 3)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	}	
	
	HAL_GPIO_Init(GPIO_EXTI4_PORT, &GPIO_InitStruct);
	
	if(state != 0)
	{
		/* EXTI interrupt init*/
		HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
		HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
	}
}

void GPIO_EXTI14_IoInit(uint8_t state)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_EXTI14_CLK_ENABLE();	
	
  GPIO_InitStruct.Pin = GPIO_EXTI14_PIN;	
	
	if(state == 0)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	}
	else if(state == 1)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	}
	else if(state == 2)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	}
	else if(state == 3)
  {
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	}	
	
	HAL_GPIO_Init(GPIO_EXTI14_PORT, &GPIO_InitStruct);
	
	if(state != 0)
	{
		/* EXTI interrupt init*/
		HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
		HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
	}
}

void GPIO_EXTI15_IoInit(uint8_t state)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_EXTI15_CLK_ENABLE();	
	
	GPIO_InitStruct.Pin = GPIO_EXTI15_PIN;	
	
	if(state == 0)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	}
	else if(state == 1)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	}
	else if(state == 2)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	}
	else if(state == 3)
	{
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	}	
	
	HAL_GPIO_Init(GPIO_EXTI15_PORT, &GPIO_InitStruct);
	
	if(state != 0)
	{
		/* EXTI interrupt init*/
		HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
		HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
	}
}

void GPIO_INPUT_IoInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	GPIO_INPUT_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = GPIO_INPUT_PIN1;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;

  HW_GPIO_Init( GPIO_INPUT_PORT, GPIO_INPUT_PIN1, &GPIO_InitStruct );
}

void GPIO_INPUT_DeIoInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	GPIO_INPUT_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = GPIO_INPUT_PIN1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  HW_GPIO_Init( GPIO_INPUT_PORT, GPIO_INPUT_PIN1, &GPIO_InitStruct );
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
