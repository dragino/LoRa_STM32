 /******************************************************************************
  * @file    ult.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    17-April-2018
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
#include "ult.h"
#include "delay.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;
extern bool debug_flags;

void GPIO_ULT_INPUT_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	ULT_Echo_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = ULT_Echo_PIN;
	GPIO_InitStruct.Mode =GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

  HW_GPIO_Init( ULT_Echo_PORT, ULT_Echo_PIN, &GPIO_InitStruct );
}

void GPIO_ULT_OUTPUT_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	ULT_TRIG_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = ULT_TRIG_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

  HW_GPIO_Init( ULT_TRIG_PORT, ULT_TRIG_PIN, &GPIO_InitStruct );
}

void GPIO_ULT_INPUT_DeInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct={0};	
	ULT_Echo_CLK_ENABLE();
	
  GPIO_InitStruct.Pin = ULT_Echo_PIN ;
	GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ULT_Echo_PORT, &GPIO_InitStruct); 
}

void GPIO_ULT_OUTPUT_DeInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	ULT_TRIG_CLK_ENABLE();

  GPIO_InitStruct.Pin = ULT_TRIG_PIN ;
	GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ULT_TRIG_PORT, &GPIO_InitStruct); 	
}

void TIM2_Init(void)
{	
	__HAL_RCC_TIM2_CLK_ENABLE();
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 159;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff;
  htim2.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);
	__HAL_TIM_DISABLE(&htim2);
}

uint16_t ULT_test(void)
{
	uint32_t time;
	uint16_t distance;
	uint8_t  ult_flags=0;
	
	if(HAL_GPIO_ReadPin(ULT_Echo_PORT, ULT_Echo_PIN)==RESET)
	{
		 ult_flags=0;
	}
	else
	{
		 ult_flags=1;
	}
	
	if(ult_flags==0)
	{
		HAL_GPIO_WritePin(ULT_TRIG_PORT,ULT_TRIG_PIN,GPIO_PIN_SET);
		DelayMs(1);	
		HAL_GPIO_WritePin(ULT_TRIG_PORT,ULT_TRIG_PIN,GPIO_PIN_RESET);
	
		while(!HAL_GPIO_ReadPin(ULT_Echo_PORT, ULT_Echo_PIN));
		__HAL_TIM_SET_COUNTER(&htim2,0);
		__HAL_TIM_ENABLE(&htim2);
		while(HAL_GPIO_ReadPin(ULT_Echo_PORT, ULT_Echo_PIN));
		time=__HAL_TIM_GetCounter(&htim2)+(int)((__HAL_TIM_GetCounter(&htim2))/33);

		__HAL_TIM_DISABLE(&htim2);

//	PPRINTF("TIME=%d\r\n",time);
		distance=(170*(time*5/(float)1000)+5);

		if((distance<240)||(distance>6000))
		{
			if(debug_flags==1)
			{			
				PPRINTF("\r\n");				
				PRINTF("Distance is out of range\r\n",distance);
			}
			distance=65535;
			return distance;
		}
		else
		{
			if(debug_flags==1)
			{			
				PPRINTF("\r\n");				
				PRINTF("Distance=%d mm\r\n",distance);
			}
			return distance;
		}
	}	
	else
	{
		if(debug_flags==1)
		{		
			PPRINTF("\r\n");				
			PRINTF("ULT is not connect\r\n");
		}
		distance=4095;
		return distance;
	}
}
