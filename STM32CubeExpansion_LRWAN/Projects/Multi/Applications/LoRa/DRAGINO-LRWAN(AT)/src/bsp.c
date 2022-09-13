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
#include "delay.h"
#include "vcom.h"
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
#include "lidar_lite_v3hp.h"
#include "weight.h"
#include "iwdg.h"
#include "bh1750.h"
#include "tfsensor.h"
#include "digital_inputs.h"
#include "lubcos.h"
#include "opcom.h"
#endif
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
bool bh1750flags=0;
uint8_t mode2_flag=0;
static __IO uint16_t AD_code1=0;
__IO uint16_t AD_code2=0;
__IO uint16_t AD_code3=0;

#ifdef USE_SHT
uint8_t flags=0;
#endif

//static GPIO_InitTypeDef  GPIO_InitStruct;
extern uint16_t batteryLevel_mV;

// Maximo modo de operacao
int MAX_WORK_MODE = 8;

#ifdef USE_SHT
extern float sht31_tem,sht31_hum;
extern I2C_HandleTypeDef I2cHandle1;
extern I2C_HandleTypeDef I2cHandle2;
extern I2C_HandleTypeDef I2cHandle3;
tfsensor_reading_t reading_t;


lubcos_serial_reading_t lubcos_reading;
opcom_serial_reading_t opcom_reading;


#endif

extern void Read_Config(void);
extern uint8_t mode;
extern uint8_t inmode;
extern uint16_t power_time;

void BSP_sensor_Read( sensor_t *sensor_data)
{
 	#if defined(LoRa_Sensor_Node)
	
	HAL_GPIO_WritePin(PWR_OUT_PORT,PWR_OUT_PIN,GPIO_PIN_RESET);//Enable 5v power supply

	IWDG_Refresh();	
	DelayMs(500);
	if(power_time!=0)
	{
		for(int i=0;i<=(int)(power_time/100);i++)
		{
			 DelayMs(100);
       if((i%99==0)&&(i!=0))
			 {
					IWDG_Refresh();		 
			 }				 
		}
	}
	
	HAL_GPIO_WritePin(OIL_CONTROL_PORT,OIL_CONTROL_PIN,GPIO_PIN_RESET);	
	HW_GetBatteryLevel( );	
	AD_code1=HW_AdcReadChannel( ADC_Channel_Oil );  //PA0
	
	HAL_GPIO_WritePin(OIL_CONTROL_PORT,OIL_CONTROL_PIN,GPIO_PIN_SET);
	
	sensor_data->oil=AD_code1*batteryLevel_mV/4095;
	
	sensor_data->in1=HAL_GPIO_ReadPin(GPIO_INPUT_PORT,GPIO_INPUT_PIN1);

	sensor_data->temp1=DS18B20_GetTemp_SkipRom(1);
	
	 if((mode==1)||(mode==3))
	 {		
		#ifdef USE_SHT
		if(flags==0)
		{
			sensor_data->temp_sht=6553.5;
			sensor_data->hum_sht=6553.5;
		} 
		else if(flags==1)
		{
			float temp2,hum1;
			HAL_I2C_MspInit(&I2cHandle1);
			temp2=SHT20_RT();//get temperature
			hum1=SHT20_RH(); //get humidity
			sensor_data->temp_sht=temp2;
			sensor_data->hum_sht=hum1;
		}
		else if(flags==2)
		{			
			HAL_I2C_MspInit(&I2cHandle2);			
			tran_SHT31data();		
			sensor_data->temp_sht=sht31_tem;
			sensor_data->hum_sht=sht31_hum;
		}
		else if(flags==3)
		{		
			bh1750flags=1;		
			I2C_IoInit();
			sensor_data->illuminance=bh1750_read();
			I2C_DoInit();					
		}		
		#endif
		}
	 
		else if(mode==2)
	 {
		 if(mode2_flag==1)
		 {
			HAL_I2C_MspInit(&I2cHandle3);		
			LidarLite_init();			 
			sensor_data -> distance_mm=LidarLite();				 
		 }
		 else if(mode2_flag==2)
		 {
			GPIO_ULT_INPUT_Init();
			GPIO_ULT_OUTPUT_Init();			 
			sensor_data -> distance_mm=ULT_test();
			GPIO_ULT_INPUT_DeInit();
			GPIO_ULT_OUTPUT_DeInit();	
		 }
		 else if(mode2_flag==3)
		 {
			tfsensor_read_distance(&reading_t);	
		  sensor_data -> distance_mm = reading_t.distance_mm;		
			sensor_data -> distance_signal_strengh = reading_t.distance_signal_strengh;					
		 }
		 else
		 {
			sensor_data -> distance_mm = 65535;		
			sensor_data -> distance_signal_strengh = 65535;			
		 }			 
	  }

   else if(mode==4)
   {
		sensor_data->temp2=DS18B20_GetTemp_SkipRom(2);
		sensor_data->temp3=DS18B20_GetTemp_SkipRom(3);	 
	 }	
	 
	 else if(mode==5)
   {
		WEIGHT_SCK_Init();
		WEIGHT_DOUT_Init();		 
		Get_Weight();		
		WEIGHT_SCK_DeInit();
		WEIGHT_DOUT_DeInit();		 
	 }		 
	 
	  if(mode==3)
	 {	
	   AD_code2=HW_AdcReadChannel( ADC_Channel_IN1 );  //PA1
	   sensor_data->ADC_1=AD_code2*batteryLevel_mV/4095;
		 
	   AD_code3=HW_AdcReadChannel( ADC_Channel_IN4 );	//PA4
	   sensor_data->ADC_2=AD_code3*batteryLevel_mV/4095;  			
	 }	 

		// Artur Gussi de Oliveira - Preddata - 20020218
	 // LubCos
	 if(mode==7)
	 {
		 
		 lubcos_read_serial(&lubcos_reading);

		 sensor_data->horario							= lubcos_reading.horario;
		 sensor_data->temperatura					= lubcos_reading.temperatura;
		 sensor_data->temperatura_sensor	= lubcos_reading.temperatura_sensor;
		 sensor_data->umidade_relativa		= lubcos_reading.umidade_relativa;
		 sensor_data->umidade_absoluta		= lubcos_reading.umidade_absoluta;

		 
		 GPIOB_INPUT_PULLUP_PIN_IoInit( GPIO_PIN_6 );
		 GPIOB_INPUT_PULLUP_PIN_IoInit( GPIO_PIN_7 );
		 GPIOB_INPUT_PULLUP_PIN_IoInit( GPIO_PIN_3 );
		 
		 bool DI1	= HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6);
		 bool DI2	= HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7);
		 bool DI3	= HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3);
		 
		 // Entradas com pull-up ativas da "0". Inverte para dar resultado 1
		 sensor_data->in1 = (!DI3)<<2 | (!DI2)<<1 | (!DI1);
	 }
	 
	 // OPCom
	 if(mode==8)
	 {
		 
		 opcom_read_serial(&opcom_reading);
		 
		 sensor_data->horario	= opcom_reading.horario;
		 sensor_data->findex	= opcom_reading.findex;
		 sensor_data->v_4um		= opcom_reading.v_4um;
		 sensor_data->v_6um		= opcom_reading.v_6um;
		 sensor_data->v_14um	= opcom_reading.v_14um;
		 sensor_data->v_21um	= opcom_reading.v_21um;
		 
		 
		 GPIOB_INPUT_PULLUP_PIN_IoInit( GPIO_PIN_6 );
		 GPIOB_INPUT_PULLUP_PIN_IoInit( GPIO_PIN_7 );
		 GPIOB_INPUT_PULLUP_PIN_IoInit( GPIO_PIN_3 );
		 
		 bool DI1	= HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6);
		 bool DI2	= HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7);
		 bool DI3	= HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3);
		 
		 // Entradas com pull-up ativas da "0". Inverte para dar resultado 1
		 sensor_data->in1 = (!DI3)<<2 | (!DI2)<<1 | (!DI1);
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
	
	 pwr_control_IoInit();		
	
	if((mode==1)||(mode==3))
	{	 
	 #ifdef USE_SHT
	 uint8_t txdata1[1]={0xE7},txdata2[2]={0xF3,0x2D};
	 
	 BSP_sht20_Init();

	 uint32_t currentTime = TimerGetCurrentTime();	 
	 while(HAL_I2C_Master_Transmit(&I2cHandle1,0x80,txdata1,1,1000) != HAL_OK)
	 {
			if(TimerGetElapsedTime(currentTime) >= 500)
			{
			 flags=0;
			 break;
		 }
	 }
	 if(HAL_I2C_Master_Transmit(&I2cHandle1,0x80,txdata1,1,1000) == HAL_OK)
	 {
		 flags=1;
	   PRINTF(" Use Sensor is STH2x\n\r");
	 }
	 
	 if(flags==0)
	 {	 
		 HAL_I2C_MspDeInit(&I2cHandle1);	 
		 BSP_sht31_Init();

		 currentTime = TimerGetCurrentTime();		 
	 	 while(HAL_I2C_Master_Transmit(&I2cHandle2,0x88,txdata2,2,1000) != HAL_OK) 
		 {
			if(TimerGetElapsedTime(currentTime) >= 500)
			{
			 flags=0;
			 break;
			}
		 }
	 if(HAL_I2C_Master_Transmit(&I2cHandle2,0x88,txdata2,2,1000) == HAL_OK)
	 {
		 flags=2;
		 PRINTF("  Use Sensor is STH3x\n\r");
	 }	 
   }
	 
	 if(flags==0)
	 {
		 float luxtemp;
		 HAL_I2C_MspDeInit(&I2cHandle2);
		 I2C_IoInit();
		 luxtemp=bh1750_read();
		 I2C_DoInit();
		 if(luxtemp!=65535)
		 {
			flags=3;
			PRINTF("  Use Sensor is BH1750\n\r");			 
		 }
	 }
	 
	 if(flags==0)
	 {
		 PRINTF("  No I2C device detected\n\r");
	 }
	 #endif
   }
	 
	else if(mode==2)
	{	
	  uint8_t dataByte[1]={0x00};		
	  HAL_GPIO_WritePin(PWR_OUT_PORT,PWR_OUT_PIN,GPIO_PIN_RESET);//Enable 5v power supply	
    BSP_lidar_Init();
    waitbusy(); 	
    HAL_I2C_Mem_Write(&I2cHandle3,0xc4,0x00,1,dataByte,1,1000);	
	  if(waitbusy()<9999)
	  {
     mode2_flag=1;		
     HAL_GPIO_WritePin(PWR_OUT_PORT,PWR_OUT_PIN,GPIO_PIN_SET);//Disable 5v power supply				
		 PRINTF("  Use Sensor is LIDAR_Lite_v3HP\n\r");
	  }		
	  else
	  {		 
		 HAL_I2C_MspDeInit(&I2cHandle3);	 
		 TIM2_Init();
		 GPIO_ULT_INPUT_Init();
		 GPIO_ULT_OUTPUT_Init();	
			
		 if(HAL_GPIO_ReadPin(ULT_Echo_PORT, ULT_Echo_PIN)==RESET)
	   {  
			mode2_flag=2;	 			 
			PRINTF("  Use Sensor is ultrasonic distance measurement\n\r");				 
		 }	
		 GPIO_ULT_INPUT_DeInit();
		 GPIO_ULT_OUTPUT_DeInit();
		 
		 if(mode2_flag==0)
		 {
			__HAL_RCC_TIM2_CLK_DISABLE();			 
			if(check_deceive()==1)
			{
				mode2_flag=3;
				PRINTF("  Use Sensor is TF-series sensor\n\r");	
			}	
			else
			{	
		    PRINTF("  No distance measurement device detected\n\r");					
			}	
			HAL_GPIO_WritePin(PWR_OUT_PORT,PWR_OUT_PIN,GPIO_PIN_SET);//Disable 5v power supply						
		 }
	  }
	}
	else if(mode==5)
	{
		WEIGHT_SCK_Init();
		WEIGHT_DOUT_Init();
		Get_Maopi();
    DelayMs(500);
		Get_Maopi();		
		PPRINTF("  Use Sensor is HX711\n\r");			
	}
	
	switch(inmode)
	{
		case 0:
			GPIO_EXTI_IoDeInit();
	  break;
		case 1:
			GPIO_EXTI_RISING_FALLINGInit();
	  break;
		case 2:
			GPIO_EXTI_FALLINGInit();
	  break;
		case 3:
			GPIO_EXTI_RISINGInit();
	  break;
		default:
	  break;
	}
	
	 // Artur Gussi de Oliveira - Preddata - 20020218
	//LubCos
	if(mode == 7)
	{
		uart1_init_lubcos();
		PPRINTF("  Use Sensor is LubCos\n\r");
	}
	//OPCom
	else if(mode == 8)
	{
		uart1_init_opcom();
		PPRINTF("  Use Sensor is OPCom\n\r");
	}
	else
	{
	 GPIO_INPUT_IoInit();
	 BSP_oil_float_Init();
	}
	
	#endif
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
