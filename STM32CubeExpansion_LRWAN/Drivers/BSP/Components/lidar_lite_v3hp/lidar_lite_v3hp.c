 /******************************************************************************
  * @file    LIDAR_Lite_v3HP.c
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

#include "lidar_lite_v3hp.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* I2C handler declaration */
extern bool debug_flags;
I2C_HandleTypeDef I2cHandle3;
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
void BSP_lidar_Init(void)
{
  /*##-1- Configure the I2C peripheral ######################################*/
  I2cHandle3.Instance              = I2Cx;
  I2cHandle3.Init.Timing           = I2C_TIMING;
  I2cHandle3.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle3.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
  I2cHandle3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  I2cHandle3.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  I2cHandle3.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;  
  I2cHandle3.Init.OwnAddress1      = 0xF0;
  I2cHandle3.Init.OwnAddress2      = 0xFE;
  
  if(HAL_I2C_Init(&I2cHandle3) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Enable the Analog I2C Filter */
  HAL_I2CEx_ConfigAnalogFilter(&I2cHandle3,I2C_ANALOGFILTER_ENABLE);
  /* Infinite loop */
}

void LidarLite_init(void) //Default mode, balanced performance
{
    uint8_t sigCountMax[1]={0x80};
    uint8_t acqConfigReg[1]={0x08};
    uint8_t refCountMax[1]={0x05};
    uint8_t thresholdBypass[1]={0x00};	
    waitbusy(); 
    HAL_I2C_Mem_Write(&I2cHandle3,0xc4,0x02,1,sigCountMax,1,1000);
    waitbusy(); 
    HAL_I2C_Mem_Write(&I2cHandle3,0xc4,0x04,1,acqConfigReg,1,1000);
    waitbusy(); 
    HAL_I2C_Mem_Write(&I2cHandle3,0xc4,0x12,1,refCountMax,1,1000);	
    waitbusy(); 
    HAL_I2C_Mem_Write(&I2cHandle3,0xc4,0x1c,1,thresholdBypass,1,1000);
    waitbusy(); 
}

uint16_t LidarLite(void)
{
	  uint8_t dataByte[1]={0x04};
		uint8_t rxdata1[1];
		uint8_t rxdata2[1];
		uint16_t distance;
    waitbusy(); 		
    HAL_I2C_Mem_Write(&I2cHandle3,0xc4,0x00,1,dataByte,1,1000);	
		if(waitbusy()<9999)
		{		
			HAL_I2C_Mem_Read(&I2cHandle3,0xc5,0x0f,1,rxdata1,1,1000);
			HAL_I2C_Mem_Read(&I2cHandle3,0xc5,0x10,1,rxdata2,1,1000);
			distance=(rxdata1[0]<<8)+rxdata2[0];
			if(distance>4000)
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
					PRINTF("Distance =%dcm\r\n",distance);
				}
				return distance*10;	
			}			
		}
		else
		{
			if(debug_flags==1)
			{			
				PPRINTF("\r\n");					
				PRINTF("lidar_lite is not connect\r\n");
			}
	   distance=4095;
	   return distance;			
		}
}

uint16_t waitbusy(void)
{
  uint16_t busyCounter = 0;
	uint8_t busy[1]={0x01};
  while (busy[0])      
  {
   if (busyCounter > 9999)
   {
		return busyCounter;			 
   }
	 HAL_I2C_Mem_Read(&I2cHandle3,0xc5,0x01,1,busy,1,1000);
	 busy[0] &=0x01;
	 busyCounter++;
  }
	return busyCounter;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

