 /******************************************************************************
  * @file    sht20.c
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

#include "sht20.h"
#include "timeServer.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* I2C handler declaration */
#ifdef USE_SHT
I2C_HandleTypeDef I2cHandle1;
/* I2C TIMING Register define when I2C clock source is SYSCLK */
/* I2C TIMING is calculated in case of the I2C Clock source is the SYSCLK = 32 MHz */
#define I2C_TIMING    0x10A13E56 /* 100 kHz with analog Filter ON, Rise Time 400ns, Fall Time 100ns */ 
//#define I2C_TIMING      0x00B1112E /* 400 kHz with analog Filter ON, Rise Time 250ns, Fall Time 100ns */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
void  BSP_sht20_Init( void )
{
  /*##-1- Configure the I2C peripheral ######################################*/
  I2cHandle1.Instance              = I2Cx;
  I2cHandle1.Init.Timing           = I2C_TIMING;
  I2cHandle1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
  I2cHandle1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  I2cHandle1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  I2cHandle1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;  
  I2cHandle1.Init.OwnAddress1      = 0xF0;
  I2cHandle1.Init.OwnAddress2      = 0xFE;
  
  if(HAL_I2C_Init(&I2cHandle1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Enable the Analog I2C Filter */
  HAL_I2CEx_ConfigAnalogFilter(&I2cHandle1,I2C_ANALOGFILTER_ENABLE);
  /* Infinite loop */
}
/**
  * @brief I2C MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - DMA configuration for transmission request by peripheral 
  *           - NVIC configuration for DMA interrupt request enable
  * @param hi2c: I2C handle pointer
  * @retval None
  */

float SHT20_RH(void)
{
	  uint8_t txdata[1]={0xf5};//Humidity measurement
		uint8_t rxdata[2];
		uint16_t AD_code;
		float hum;
		bool read_status=1;
		
		uint32_t currentTime = TimerGetCurrentTime();
		while(HAL_I2C_Master_Transmit(&I2cHandle1,0x80,txdata,1,1000) != HAL_OK)
    {
			  if(TimerGetElapsedTime(currentTime) >= 300)
				{
					read_status=0;
					break;
				}
        if(HAL_I2C_GetError(&I2cHandle1) != HAL_I2C_ERROR_AF)
        {}
    }

		currentTime = TimerGetCurrentTime();
		while(HAL_I2C_Master_Receive(&I2cHandle1,0x81,rxdata,2,1000) != HAL_OK)
    {
			  if(TimerGetElapsedTime(currentTime) >= 1000)
				{
					read_status=0;
					break;
				}
        if(HAL_I2C_GetError(&I2cHandle1) != HAL_I2C_ERROR_AF)
        {}
    }

	if(read_status==1)
	{
		AD_code=(rxdata[0]<<8)+rxdata[1];
		AD_code &=~0x0003;   //14bit

		hum=AD_code*125.0/65536-6;
		
		if(hum>100)
		{
			hum=100;
		}
		else if(hum<0)
		{
			hum=0;
		}
	}
	else
	{
		hum=6553.5;
	}
	return hum;
}

float SHT20_RT(void)
{
	  uint8_t txdata[1]={0xf3};//Temperature measurement
		uint8_t rxdata[2];
		uint16_t AD_code;
		float tem;
		bool read_status=1;
		
	  uint32_t currentTime = TimerGetCurrentTime();		
		while(HAL_I2C_Master_Transmit(&I2cHandle1,0x80,txdata,1,1000) != HAL_OK)
    {
			  if(TimerGetElapsedTime(currentTime) >= 300)
				{
					read_status=0;
					break;
				}
        if(HAL_I2C_GetError(&I2cHandle1) != HAL_I2C_ERROR_AF)
        {}
    }

		currentTime = TimerGetCurrentTime();		
		while(HAL_I2C_Master_Receive(&I2cHandle1,0x81,rxdata,2,1000) != HAL_OK)
    {
			  if(TimerGetElapsedTime(currentTime) >= 1000)
				{		
					read_status=0;
					break;
				}
        if(HAL_I2C_GetError(&I2cHandle1) != HAL_I2C_ERROR_AF)
        {}
    }
		
	if(read_status==1)
	{		
		AD_code=(rxdata[0]<<8)+rxdata[1];
		AD_code &=~0x0003;   //14bit
		tem=AD_code*175.72/65536-46.85;
		
		if(tem>125)
		{
			tem=125;
		}
		else if(tem<-40)
		{
			tem=-40;
		}
	}
	else 
	{
		tem=3276.7;
	}
	
	return tem;	
}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

