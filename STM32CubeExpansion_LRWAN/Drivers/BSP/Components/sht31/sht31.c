 /******************************************************************************
  * @file    sht31.c
  * @author  MCD Application Team
  * @version V1.1.2
  * @date    30-Novermber-2018
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

#include "sht31.h"
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
static bool sht31_status=1;
static uint8_t rxdatas[6];
float sht31_tem,sht31_hum;
I2C_HandleTypeDef I2cHandle2;
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
void  BSP_sht31_Init( void )
{
  /*##-1- Configure the I2C peripheral ######################################*/
  I2cHandle2.Instance              = I2Cx;
  I2cHandle2.Init.Timing           = I2C_TIMING;
  I2cHandle2.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle2.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
  I2cHandle2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  I2cHandle2.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  I2cHandle2.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;  
  I2cHandle2.Init.OwnAddress1      = 0xF0;
  I2cHandle2.Init.OwnAddress2      = 0xFE;
  
  if(HAL_I2C_Init(&I2cHandle2) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Enable the Analog I2C Filter */
  HAL_I2CEx_ConfigAnalogFilter(&I2cHandle2,I2C_ANALOGFILTER_ENABLE);
  /* Infinite loop */
}

void SHT31_Read(uint8_t rxdata[])
{
		uint8_t SHT3X_Modecommand_Buffer[2]={0x2c,0x06};  //Measurement Commands for Single Shot Data Acquisition Mode

		uint32_t currentTime = TimerGetCurrentTime();
		while(HAL_I2C_Master_Transmit(&I2cHandle2,0x88,SHT3X_Modecommand_Buffer,2,1000)!= HAL_OK)//work mode
		{
			  if(TimerGetElapsedTime(currentTime) >= 500)
				{
					sht31_status=0;
					break;
				}
        if(HAL_I2C_GetError(&I2cHandle2) != HAL_I2C_ERROR_AF)
        {}			
		} 
		
		currentTime = TimerGetCurrentTime();		
		while(HAL_I2C_Master_Receive(&I2cHandle2,0x89,rxdata,6,3000) != HAL_OK)  
    {
			  if(TimerGetElapsedTime(currentTime) >= 4000)
				{
					sht31_status=0;
					break;
				}
        if(HAL_I2C_GetError(&I2cHandle2) != HAL_I2C_ERROR_AF)
        {}
    }	
}

void tran_SHT31data(void)
{
	for(int i=0;i<6;i++)
	{
		rxdatas[i]=0x00;
	}
	
	sht31_status=1;
	SHT31_Read(rxdatas);	
	
	if(sht31_status==1)
	{
		sht31_tem=((rxdatas[0]<<8)+rxdatas[1])*175.0/(65536-1)-45.0;
		sht31_hum=((rxdatas[3]<<8)+rxdatas[4])*100.0/(65536-1);
		if(sht31_hum>100)	
		{
			sht31_hum=100;
		}
		else if(sht31_hum<0)
		{		
      sht31_hum=0;
		}	
		
		if(sht31_tem>125)
		{
			sht31_tem=125;
		}
		else if(sht31_tem<-40)
		{
			sht31_tem=-40;
		}		
	}
	else
	{
		sht31_tem=3276.7;
		sht31_hum=6553.5;
	}
}

#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

