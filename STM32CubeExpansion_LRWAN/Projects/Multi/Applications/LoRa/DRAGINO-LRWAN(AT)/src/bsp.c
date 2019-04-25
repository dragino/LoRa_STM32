 /******************************************************************************
  * @file    bsp.c
  * @author  MCD Application Team
  * @version V1.1.4
  * @date    08-January-2018
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
#include <string.h>
#include <stdlib.h>
#include "hw.h"
#include "timeServer.h"
#include "bsp.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#if defined(LoRa_Sensor_Node)
#include "ds18b20.h"
#include "oil_float.h"
#include "gpio_exti.h"
#include "sht20.h"
#include "sht31.h"
#include "pwr_out.h"
#include "ult.h"
#endif
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint16_t ult;
static __IO uint16_t AD_code1=0;
static __IO uint16_t AD_code2=0;

#ifdef USE_SHT
static int flags=0;
#endif

//static GPIO_InitTypeDef  GPIO_InitStruct;
extern uint16_t batteryLevel_mV;

#ifdef USE_SHT
extern I2C_HandleTypeDef I2cHandle1;
extern I2C_HandleTypeDef I2cHandle2;
#endif

extern void Read_Config(void);
extern uint8_t mode;

void BSP_sensor_Read( sensor_t *sensor_data)
{
 	#if defined(LoRa_Sensor_Node)
	
	HAL_GPIO_WritePin(PWR_OUT_PORT,PWR_OUT_PIN,GPIO_PIN_RESET);//Enable 5v power supply
	
	sensor_data->temp1=DS18B20_GetTemp_SkipRom();
	
	HAL_GPIO_WritePin(OIL_CONTROL_PORT,OIL_CONTROL_PIN,GPIO_PIN_RESET);
	AD_code1=HW_AdcReadChannel( ADC_Channel_Oil );
	HAL_GPIO_WritePin(OIL_CONTROL_PORT,OIL_CONTROL_PIN,GPIO_PIN_SET);
	
	HW_GetBatteryLevel( );
	sensor_data->oil=AD_code1*batteryLevel_mV/4095;
	
	sensor_data->in1=HAL_GPIO_ReadPin(GPIO_INPUT_PORT,GPIO_INPUT_PIN1);
	
	 if(mode==1)
	 {		
   #ifdef USE_SHT
	 if(flags==0)
	 {
		 sensor_data->temp_sht=6553.5;
		 sensor_data->hum_sht=6553.5;
	 } 
	 if(flags==1)
	 {
	float temp2,hum1;
	temp2=SHT20_RT();//get temperature
	hum1=SHT20_RH(); //get humidity
	sensor_data->temp_sht=temp2;
	sensor_data->hum_sht=hum1;
   }
	if(flags==2)
	{
	float temp2,hum1;
	temp2=SHT31_RT();//get temperature
	hum1=SHT31_RH(); //get humidity
	sensor_data->temp_sht=temp2;
	sensor_data->hum_sht=hum1;
	}
	 #endif
		}
	 
		if(mode==2)
	 {
		 ult=ULT_test();
	 }
	 
	 HAL_GPIO_WritePin(PWR_OUT_PORT,PWR_OUT_PIN,GPIO_PIN_SET);//Disable 5v power supply
	 
	#endif
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;
  
  /*##-1- Configure the I2C clock source. The clock is derived from the SYSCLK #*/
  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2Cx;
  RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2CxCLKSOURCE_SYSCLK;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

  /*##-2- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  I2Cx_SCL_GPIO_CLK_ENABLE();
  I2Cx_SDA_GPIO_CLK_ENABLE();
  /* Enable I2Cx clock */
  I2Cx_CLK_ENABLE();
  
  /*##-3- Configure peripheral GPIO ##########################################*/  
  /* I2C TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = I2Cx_SCL_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = I2Cx_SCL_SDA_AF;
  HAL_GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStruct);
    
  /* I2C RX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = I2Cx_SDA_PIN;
  GPIO_InitStruct.Alternate = I2Cx_SCL_SDA_AF;
  HAL_GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStruct);
	
  /* NVIC for I2Cx */
  HAL_NVIC_SetPriority(I2Cx_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(I2Cx_IRQn);
}
/**
  * @brief I2C MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  /*##-1- Reset peripherals ##################################################*/
  I2Cx_FORCE_RESET();
  I2Cx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks #################################*/
  /* Configure I2C Tx as alternate function  */
  HAL_GPIO_DeInit(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_PIN);
  /* Configure I2C Rx as alternate function  */
  HAL_GPIO_DeInit(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_PIN);
}

void  BSP_sensor_Init( void  )
{
  #if defined(LoRa_Sensor_Node)
//	 while(DS18B20_Init()==1);
	 Read_Config();
	
	 GPIO_EXTI_IoInit();
	 GPIO_INPUT_IoInit();
	 BSP_oil_float_Init();
	
	 if(mode==1)
	 {	 
	 #ifdef USE_SHT
	 int sum1=0,sum2=0;
	 uint8_t txdata1[1]={0xE7},txdata2[2]={0xF3,0x2D};
	 
	 BSP_sht20_Init();
 
		 while(HAL_I2C_Master_Transmit(&I2cHandle1,0x80,txdata1,1,1000) != HAL_OK)
	 {
		 sum1++;
		 if(sum1==100)
		 {
			 flags=0;
			 break;
		 }
	 }
	 if(HAL_I2C_Master_Transmit(&I2cHandle1,0x80,txdata1,1,1000) == HAL_OK)
	 {
		 flags=1;
	   PRINTF(" Use Sensor is STH20\n\r");
	 }
	 
	 if(flags==0)
	 {
		 
		 BSP_sht31_Init();
		 
	 	 while(HAL_I2C_Master_Transmit(&I2cHandle2,0x88,txdata2,2,1000) != HAL_OK) 
	 {
		 sum2++;
		 if(sum2==100)
		 {
			 flags=0;
			 break;
		 }
	 }
	 if(HAL_I2C_Master_Transmit(&I2cHandle2,0x88,txdata2,2,1000) == HAL_OK)
	 {
		 flags=2;
		 PRINTF("  Use Sensor is STH31\n\r");
	 }
   }
	 
	 if(flags==0)
	 {
		 PRINTF("  IIC is not connect\n\r");
	 }	
	 #endif
  }
	if(mode==2)
	{ 
		GPIO_ULT_INPUT_Init();
		GPIO_ULT_OUTPUT_Init();
		TIM2_Init();
	}
	pwr_control_IoInit();
	#endif
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
