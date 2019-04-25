/******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.1.4
  * @date    08-January-2018
  * @brief   this is the main!
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
#include "low_power_manager.h"
#include "lora.h"
#include "bsp.h"
#include "timeServer.h"
#include "vcom.h"
#include "version.h"
#include "command.h"
#include "at.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
uint32_t APP_TX_DUTYCYCLE=30000;
uint32_t ServerSetTDC;
/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE LORAWAN_ADR_ON
/*!
 * LoRaWAN Default data Rate Data Rate
 * @note Please note that LORAWAN_DEFAULT_DATA_RATE is used only when ADR is disabled 
 */
#define LORAWAN_DEFAULT_DATA_RATE DR_0
/*!
 * LoRaWAN application port is 0 to 255
 * @note do not use 256. It is reserved for certification
 */
#define LORAWAN_APP_PORT                            2
/*!
 * Number of trials for the join request.
 */
#define JOINREQ_NBTRIALS                            200
/*!
 * LoRaWAN default endNode class port
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_A
/*!
 * LoRaWAN default confirm state
 */
#define LORAWAN_DEFAULT_CONFIRM_MSG_STATE           LORAWAN_UNCONFIRMED_MSG
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFF_SIZE                           64
/*!
 * User application data
 */
static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];

int exti_flag=0;
uint8_t TDC_flag=0;
uint16_t batteryLevel_mV;
void send_exti(void);
extern uint8_t mode;
extern uint16_t ult;

/*!
 * User application data structure
 */
static lora_AppData_t AppData={ AppDataBuff,  0 ,0 };
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* call back when LoRa endNode has received a frame*/
static void LORA_RxData( lora_AppData_t *AppData);

/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined( void );

/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass ( DeviceClass_t Class );

/* LoRa endNode send request*/
static void Send( void );

#if defined(LoRa_Sensor_Node)
/* start the tx process*/
static void LoraStartTx(TxEventType_t EventType);

static TimerEvent_t TxTimer;

/* tx timer callback function*/
static void OnTxTimerEvent( void );

#endif

/* Private variables ---------------------------------------------------------*/
/* load Main call backs structure*/
static LoRaMainCallback_t LoRaMainCallbacks ={ HW_GetBatteryLevel,
                                               HW_GetTemperatureLevel,
                                               HW_GetUniqueId,
                                               HW_GetRandomSeed,
                                               LORA_RxData,
                                               LORA_HasJoined,
                                               LORA_ConfirmClass};

/* !
 *Initialises the Lora Parameters
 */
static  LoRaParam_t LoRaParamInit= {LORAWAN_ADR_STATE,
                                    LORAWAN_DEFAULT_DATA_RATE,  
                                    LORAWAN_PUBLIC_NETWORK,
                                    JOINREQ_NBTRIALS};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main( void )
{
  /* STM32 HAL library initialization*/
  HAL_Init( );
  
  /* Configure the system clock*/
  SystemClock_Config( );
  
  /* Configure the debug mode*/
  DBG_Init( );
  
  /* Configure the hardware*/
  HW_Init( );
 
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
   CMD_Init();
	
  /*Disbale Stand-by mode*/
  LPM_SetOffMode(LPM_APPLI_Id , LPM_Disable );
  
  /* Configure the Lora Stack*/
  LORA_Init( &LoRaMainCallbacks, &LoRaParamInit);
  
  while( 1 )
  {
		/* Handle UART commands */
    CMD_Process();
		
		#if defined(LoRa_Sensor_Node)
		send_exti();
		#endif
		
    DISABLE_IRQ( );
    /*
     * if an interrupt has occurred after DISABLE_IRQ, it is kept pending
     * and cortex will not enter low power anyway
     * don't go in low power mode if we just received a char
     */
#ifndef LOW_POWER_DISABLE
    LPM_EnterLowPower();
#endif
    ENABLE_IRQ();
    
    /* USER CODE BEGIN 2 */
    /* USER CODE END 2 */
  }
}

static void LORA_HasJoined( void )
{
  AT_PRINTF("JOINED\n\r");
	Read_Config();
  LORA_RequestClass( LORAWAN_DEFAULT_CLASS );
	
	#if defined(LoRa_Sensor_Node) /*LSN50 Preprocessor compile swicth:hw_conf.h*/
	LoraStartTx( TX_ON_TIMER);    
	#endif
	
	#if defined(AT_Data_Send)     /*LoRa ST Module*/
	AT_PRINTF("Please using AT+SEND or AT+SENDB to send you data!\n\r");
	#endif
}

static void Send( void )
{
  sensor_t sensor_data;
  
  if ( LORA_JoinStatus () != LORA_SET)
  {
    /*Not joined, try again later*/
    return;
  }

	BSP_sensor_Read( &sensor_data );
	
	#if defined(LoRa_Sensor_Node)
	
	uint32_t i = 0;

  AppData.Port = lora_config_application_port_get();
	
	HW_GetBatteryLevel( );

  if(mode==1)
	{		
	AppData.Buff[i++] =(batteryLevel_mV>>8);       //level of battery in mV
	AppData.Buff[i++] =batteryLevel_mV & 0xFF;
	
	AppData.Buff[i++]=(int)(sensor_data.temp1*10)>>8;     //DS18B20
  AppData.Buff[i++]=(int)(sensor_data.temp1*10);
	
  AppData.Buff[i++] =(int)(sensor_data.oil)>>8;          //oil float
	AppData.Buff[i++] =(int)sensor_data.oil;
	
	if(exti_flag==1)
	{
	  AppData.Buff[i++]=(sensor_data.in1<<1)|0x01;    //Digital Input and EXTI Trigger status
		exti_flag=0;
	}
	else
	{
		AppData.Buff[i++]=(sensor_data.in1<<1)|0x00;
	}
	
	#if defined USE_SHT
	
	AppData.Buff[i++] =(int)(sensor_data.temp_sht*10)>>8;      //SHT20
	AppData.Buff[i++] =(int)(sensor_data.temp_sht*10);
	AppData.Buff[i++] =(int)(sensor_data.hum_sht*10)>>8; 
	AppData.Buff[i++] =(int)(sensor_data.hum_sht*10);
	
	#endif
	}
	
	if(mode==2)
	{
	AppData.Buff[i++] =(batteryLevel_mV>>8);       //level of battery in mV
	AppData.Buff[i++] =batteryLevel_mV & 0xFF;
	
	AppData.Buff[i++]=(int)(sensor_data.temp1*10)>>8;     //DS18B20
  AppData.Buff[i++]=(int)(sensor_data.temp1*10);
	
  AppData.Buff[i++] =(int)(sensor_data.oil)>>8;          //oil float
	AppData.Buff[i++] =(int)sensor_data.oil;
	
	if(exti_flag==1)
	{
	  AppData.Buff[i++]=(sensor_data.in1<<1)|0x01|0x04;    //Digital Input and EXTI Trigger status
		exti_flag=0;
	}
	else
	{
		AppData.Buff[i++]=(sensor_data.in1<<1)|0x00|0x04;
	}	
	AppData.Buff[i++]=(int)(ult)>>8;
	AppData.Buff[i++]=(int)(ult);	
	AppData.Buff[i++] = 0xFF; 
	AppData.Buff[i++] = 0xFF;	
	}
	AppData.BuffSize = i;
  LORA_send( &AppData, lora_config_reqack_get());
	#endif
	
}


static void LORA_RxData( lora_AppData_t *AppData )
{
  set_at_receive(AppData->Port, AppData->Buff, AppData->BuffSize);
	AT_PRINTF("Receive data\n\r");
	AT_PRINTF("%d:",AppData->Port);
	 for (int i = 0; i < AppData->BuffSize; i++)
  {
    AT_PRINTF("%02x", AppData->Buff[i]);
  }
	AT_PRINTF("\n\r");
	
 switch(AppData->Buff[0] & 0xff)
      {		
				case 1:
				{
					if( AppData->BuffSize == 4 )
					{
					  ServerSetTDC=( AppData->Buff[1]<<16 | AppData->Buff[2]<<8 | AppData->Buff[3] );//S
					
						if(ServerSetTDC<5)
						{
							PRINTF("TDC setting must be more than 4S\n\r");
						}
						else
						{
					    TDC_flag=1;
			        APP_TX_DUTYCYCLE=ServerSetTDC*1000;
						}
					}
					break;
				}
				
			case 4:
			{
				if( AppData->BuffSize == 2 )
					{
					  if(AppData->Buff[1]==0xFF)
					  {
					    NVIC_SystemReset();
					  }
				  }
					break;
			}
				
				default:
					break;
			}
	
	if(TDC_flag==1)
	{
		Store_Config();
		TimerInit( &TxTimer, OnTxTimerEvent );
    TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);		
    TimerStart( &TxTimer); 
		TDC_flag=0;
	}	
	
}

#if defined(LoRa_Sensor_Node)
static void OnTxTimerEvent( void )
{
	
	TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
	
  /*Wait for next tx slot*/
  TimerStart( &TxTimer);
	
	Send( );
}

static void LoraStartTx(TxEventType_t EventType)
{
  if (EventType == TX_ON_TIMER)
  {
    /* send everytime timer elapses */
    TimerInit( &TxTimer, OnTxTimerEvent );
    TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE); 
    OnTxTimerEvent();
  }
}
#endif

static void LORA_ConfirmClass ( DeviceClass_t Class )
{
  PRINTF("switch to class %c done\n\r","ABC"[Class] );

  /*Optionnal*/
  /*informs the server that switch has occurred ASAP*/
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;
  
  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
}

void send_exti(void)
{
	if(exti_flag==1)
	{
	 Send( );
	}
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
