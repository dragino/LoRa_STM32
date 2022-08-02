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
#include "timeServer.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint16_t ms_sum=0;
TIM_HandleTypeDef htim3;
extern uint16_t power_time;

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
  HAL_GPIO_Init(ULT_Echo_PORT, &GPIO_InitStruct); 
}

void GPIO_ULT_OUTPUT_DeInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	ULT_TRIG_CLK_ENABLE();

  GPIO_InitStruct.Pin = ULT_TRIG_PIN ;
	GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(ULT_TRIG_PORT, &GPIO_InitStruct); 	
}

void TIM3_Init(void)
{		
	__HAL_RCC_TIM3_CLK_ENABLE();
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 32-1;  //1 millisecond timing
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;   //1000 microseconds produce a count interrupt
  htim3.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
  if(HAL_TIM_Base_Init(&htim3)!= HAL_OK)
  {
    Error_Handler();
  }
	
	HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim3)
	{
		ms_sum++;
	}
}

uint16_t ULT_test(void)
{
	uint16_t temp_ult[6],distance;
	uint8_t  ult_flags=0;
	
	if((HAL_GPIO_ReadPin(ULT_Echo_PORT, ULT_Echo_PIN)==RESET)&&(power_time!=0))
	{
		 ult_flags=0;
	}
	else
	{
		 ult_flags=1;
	}
	
	if(ult_flags==0)
	{
		ult_test_temp(temp_ult);
		distance=ADC_Average(temp_ult);
	
		if((distance<240)||(distance>6000))
		{
				distance=0;
				return distance;
		}
		else
		{
				return distance;
		}
	}	
	else
	{
		distance=65535;
		return distance;
	}
}

void ult_test_temp(uint16_t ultdata[])
{	
	uint8_t read_sensor_flag=0;
	for(uint8_t k=0;k<6;k++)
	{
		uint32_t currentTime = 0;
		uint16_t time_usnum=0;
		ms_sum=0;
		__HAL_TIM_SET_COUNTER(&htim3,0);    //begin	
		HAL_GPIO_WritePin(ULT_TRIG_PORT,ULT_TRIG_PIN,GPIO_PIN_SET); //give a signal
		HAL_Delay(1);	
		HAL_GPIO_WritePin(ULT_TRIG_PORT,ULT_TRIG_PIN,GPIO_PIN_RESET);
		
		currentTime = TimerGetCurrentTime();		
		while(!HAL_GPIO_ReadPin(ULT_Echo_PORT, ULT_Echo_PIN)) //wait the response signal  
		{
			if(TimerGetElapsedTime(currentTime) >= 1000)
			{
				read_sensor_flag=1;
				break;
			}			
		}      
		
		HAL_TIM_Base_Start_IT(&htim3);	
		
		currentTime = TimerGetCurrentTime();
		while(HAL_GPIO_ReadPin(ULT_Echo_PORT, ULT_Echo_PIN))  //wait the response signal
		{
			if(TimerGetElapsedTime(currentTime) >= 1000)
			{
				read_sensor_flag=1;
				break;
			}					
		}
		HAL_TIM_Base_Stop_IT(&htim3);                          //end
		time_usnum=__HAL_TIM_GetCounter(&htim3);

		if(read_sensor_flag==1)
		{
			break;
		}
		
		ultdata[k]=(((ms_sum*1000+time_usnum)*0.0343)/2.0)*10.0; //(time(us)*0.0343cm/us)/2	
		HAL_Delay(50);
	}
}

void ADC_Dxpd(uint16_t adc_nums[])
{
	int i, j, temp, isSorted;  
	for(i=0; i<6-1; i++)
	{
		isSorted = 1;  
		for(j=0; j<6-1-i; j++)
		{
			if(adc_nums[j] > adc_nums[j+1])
			{
				temp = adc_nums[j];
				adc_nums[j] = adc_nums[j+1];
				adc_nums[j+1] = temp;
				isSorted = 0; 
			}
		}
		if(isSorted) break;
	}
}

uint16_t ADC_Average(uint16_t adc_nums[])
{
	uint32_t sum = 0;

	ADC_Dxpd(adc_nums);
	
	for(uint8_t i=1; i<5; i++)
	{
		sum = sum + adc_nums[i];
	}
	
	return sum/4;
}
