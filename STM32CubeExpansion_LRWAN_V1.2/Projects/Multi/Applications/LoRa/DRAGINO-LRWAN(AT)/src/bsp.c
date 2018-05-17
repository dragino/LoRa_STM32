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
#endif
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static __IO uint16_t AD_code1=0;
static __IO uint16_t AD_code2=0;


extern uint16_t batteryLevel_mV;
void BSP_sensor_Read( sensor_t *sensor_data)
{
 	#if defined(LoRa_Sensor_Node)
	sensor_data->temp1=DS18B20_GetTemp_SkipRom();
	
	HAL_GPIO_WritePin(OIL_CONTROL_PORT,OIL_CONTROL_PIN,GPIO_PIN_RESET);
	AD_code1=HW_AdcReadChannel( ADC_Channel_Oil );
	HAL_GPIO_WritePin(OIL_CONTROL_PORT,OIL_CONTROL_PIN,GPIO_PIN_SET);

	HW_GetBatteryLevel( );
	sensor_data->oil=AD_code1*batteryLevel_mV/4095;
	
	sensor_data->in1=HAL_GPIO_ReadPin(GPIO_INPUT_PORT,GPIO_INPUT_PIN1);
	
	
	 #ifdef USE_SHT20
	float temp2,hum1;
	temp2=SHT20_RT();//get temperature
	hum1=SHT20_RH(); //get humidity
	sensor_data->temp_sht=temp2;
	sensor_data->hum_sht=hum1;

	 #endif
	 
	#endif
}

void  BSP_sensor_Init( void  )
{
  	#if defined(LoRa_Sensor_Node)
//	 while(DS18B20_Init()==1);
   BSP_oil_float_Init();
	 GPIO_EXTI_IoInit();
	 GPIO_INPUT_IoInit();
	
	 #ifdef USE_SHT20
	 BSP_sht20_Init();
	 #endif
	
	#endif
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
