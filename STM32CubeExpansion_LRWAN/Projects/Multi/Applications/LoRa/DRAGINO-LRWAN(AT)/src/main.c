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
#include "flash_eraseprogram.h"
#include "low_power_manager.h"
#include "lora.h"
#include "bsp.h"
#include "timeServer.h"
#include "vcom.h"
#include "version.h"
#include "command.h"
#include "at.h"
#include "gpio_exti.h"
#include "weight.h"
#include "iwdg.h"
#include "delay.h"

#include "digital_inputs.h"
#include "float_encode.h"

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
#define LORAWAN_APP_DATA_BUFF_SIZE                           256
/*!
 * User application data
 */
static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];
uint8_t switch_status=0,normal_status=0;
bool is_check_exit=0;
bool rxpr_flags=0;
int exti_flag=0;
uint32_t COUNT;
uint8_t TDC_flag=0;
uint8_t join_flag=0;
uint8_t atz_flags=0;
uint16_t batteryLevel_mV;
uint8_t payloadlens;
bool is_time_to_IWDG_Refresh=0;
bool joined_flags=0;
bool is_there_data=0;
bool is_time_to_rejoin=0;
bool JoinReq_NbTrails_over=0;
bool unconfirmed_downlink_data_ans_status=0,confirmed_downlink_data_ans_status=0;
bool rejoin_status=0;
bool rejoin_keep_status=0;
bool MAC_COMMAND_ANS_status=0;
bool uplink_data_status=0;
uint8_t response_level=0;
uint16_t REJOIN_TX_DUTYCYCLE=20;//min

uint8_t interrupcao_flag=0;

void send_exti(void);
extern bool bh1750flags;
extern uint8_t mode;
extern uint8_t mode2_flag;
extern __IO uint16_t AD_code2;
extern __IO uint16_t AD_code3;
extern uint8_t inmode;
extern float GapValue;
extern int32_t Weight_Shiwu;
extern uint16_t power_time;
extern bool rx2_flags;
extern uint32_t LoRaMacState;
extern uint8_t dwelltime;
extern bool debug_flags;
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
static void LoraStartjoin(TxEventType_t EventType);
static void StartIWDGRefresh(TxEventType_t EventType);
static void LoraStartRejoin(TxEventType_t EventType);

TimerEvent_t TxTimer;
static TimerEvent_t TxTimer2;
static TimerEvent_t IWDGRefreshTimer;//watch dog
TimerEvent_t ReJoinTimer;

/* tx timer callback function*/
static void OnTxTimerEvent( void );
static void OnTxTimerEvent2( void );
static void OnIWDGRefreshTimeoutEvent(void);
static void OnReJoinTimerEvent( void );
#endif
extern void printf_joinmessage(void);

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

/*!   DEFINIDO NO LoRaMac.c
 * LoRaMac internal states
 */
enum eLoRaMacState
{
    LORAMAC_IDLE          = 0x00000000,
    LORAMAC_TX_RUNNING    = 0x00000001,
    LORAMAC_RX            = 0x00000002,
    LORAMAC_ACK_REQ       = 0x00000004,
    LORAMAC_ACK_RETRY     = 0x00000008,
    LORAMAC_TX_DELAYED    = 0x00000010,
    LORAMAC_TX_CONFIG     = 0x00000020,
    LORAMAC_RX_ABORT      = 0x00000040,
};	

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
	
	iwdg_init();		
	
 	StartIWDGRefresh(TX_ON_EVENT); 

	new_firmware_update();
		
  /*Disbale Stand-by mode*/
  LPM_SetOffMode(LPM_APPLI_Id , LPM_Disable );
  
  /* Configure the Lora Stack*/
  LORA_Init( &LoRaMainCallbacks, &LoRaParamInit);
  
  while( 1 )
  {
		/* Handle UART commands */
    CMD_Process();

		if(joined_flags==1)
		{
			send_exti();

			if(atz_flags==1)
			{
				DelayMs(500);
				AppData.Buff[0]=0x11;
	      AppData.BuffSize=1;
	      AppData.Port = 2;
	      LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
				atz_flags++;
			}
		  else if((atz_flags==2)&&(( LoRaMacState & 0x00000001 ) != 0x00000001))
			{
				NVIC_SystemReset();
			}		
			
			if(uplink_data_status==1)
			{
				Send();
				uplink_data_status=0;
			}	
      
      if((is_check_exit==1)&&(( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
			{
				normal_status=HAL_GPIO_ReadPin(GPIO_EXTI_PORT,GPIO_EXTI_PIN);
				if(switch_status!=normal_status)
				{
					switch_status=normal_status;
					uplink_data_status=1;
				}
				is_check_exit=0;
			}	
			
			#ifdef REGION_US915
			if(MAC_COMMAND_ANS_status==1)
			{		
				if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
				{          
					MibRequestConfirm_t mib;
			
			    mib.Type=MIB_CHANNELS_DATARATE;
			    LoRaMacMibGetRequestConfirm(&mib);
					
				  if(mib.Param.ChannelsDatarate==0)
			    {
						AppData.Buff[0]=0x00;
						AppData.BuffSize=1;
						AppData.Port = 2;							
	          LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
						MAC_COMMAND_ANS_status=0;
				  }
				}		  
			}
			#elif defined( REGION_AS923 )	|| defined( REGION_AU915 )		
			if((MAC_COMMAND_ANS_status==1)&&(dwelltime==1))
			{		
				if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
				{          
					MibRequestConfirm_t mib;
			
			    mib.Type=MIB_CHANNELS_DATARATE;
			    LoRaMacMibGetRequestConfirm(&mib);
					
				  if(mib.Param.ChannelsDatarate==0)
			    {
						AppData.Buff[0]=0x00;
						AppData.BuffSize=1;
						AppData.Port = 2;							
	          LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
						MAC_COMMAND_ANS_status=0;
				  }
				}		  
			}		
			#endif
		
			if((MAC_COMMAND_ANS_status==1 && response_level==3) 
					|| (unconfirmed_downlink_data_ans_status==1 && response_level==1 && is_there_data==1 ) 
					|| (confirmed_downlink_data_ans_status==1 && response_level==2 && is_there_data==1 )
					||(((MAC_COMMAND_ANS_status==1)||(confirmed_downlink_data_ans_status==1&&is_there_data==1))&&(response_level==4)))
			{
				if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
				{
					MAC_COMMAND_ANS_status=0;
					unconfirmed_downlink_data_ans_status=0;
					confirmed_downlink_data_ans_status=0;
					is_there_data=0;
					AppData.Buff[0]=0x00;
					AppData.BuffSize=1;
					AppData.Port = 2;							
					LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
				}
			}
		
		}
		
		if(is_time_to_rejoin==1)
		{
			if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
			{
			  is_time_to_rejoin=0;
			  LORA_Join();
			}
		}
		
		if(JoinReq_NbTrails_over==1)
		{
			JoinReq_NbTrails_over=0;
			
			rejoin_keep_status=1;
			
			if(REJOIN_TX_DUTYCYCLE>0)
			{
			  LoraStartRejoin(TX_ON_EVENT);
			}
		}
		
		if(rejoin_status==1)
		{
			rejoin_keep_status=1;
			TimerStop(&TxTimer);
			LORA_Join();
		}
		
		if(is_time_to_IWDG_Refresh==1)
		{
			IWDG_Refresh();			
			is_time_to_IWDG_Refresh=0;
		}				
						
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
	rx2_flags=1;

	Read_Config();
	
	joined_flags=1;
	
  AT_PRINTF("JOINED\r\n");

	rejoin_keep_status=0;
	
	if((lora_config_otaa_get() == LORA_ENABLE ? 1 : 0))
	{
		printf_joinmessage();
	}		
	
	TimerStop(&ReJoinTimer);	
	
  LORA_RequestClass( LORAWAN_DEFAULT_CLASS );
	
	#if defined(LoRa_Sensor_Node) /*LSN50 Preprocessor compile swicth:hw_conf.h*/
	LoraStartjoin( TX_ON_TIMER);	
	#endif
	
	#if defined(AT_Data_Send)     /*LoRa ST Module*/
	AT_PRINTF("Please using AT+SEND or AT+SENDB to send you data!\n\r");
	#endif
}

static void Send( void )
{
  sensor_t sensor_data; 
	is_there_data=0;		
  if ( LORA_JoinStatus () != LORA_SET)
  {
    /*Not joined, try again later*/
    return;
  }
	
	BSP_sensor_Read( &sensor_data );
	
	#if defined(LoRa_Sensor_Node)
	
	uint32_t i = 0;

  AppData.Port = lora_config_application_port_get();

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
			AppData.Buff[i++]=(switch_status<<7)|(sensor_data.in1<<1)|0x01;
			exti_flag=0;
		}
		else
		{
			switch_status=HAL_GPIO_ReadPin(GPIO_EXTI_PORT,GPIO_EXTI_PIN);							
			AppData.Buff[i++]=(switch_status<<7)|(sensor_data.in1<<1);
		}
	
		#if defined USE_SHT
		if(bh1750flags==1)
		{
			AppData.Buff[i++] =(int)(sensor_data.illuminance)>>8;      
			AppData.Buff[i++] =(int)(sensor_data.illuminance);
			AppData.Buff[i++] = 0x00;   
			AppData.Buff[i++] = 0x00;				
		}	
		else
		{
			AppData.Buff[i++] =(int)(sensor_data.temp_sht*10)>>8;      
			AppData.Buff[i++] =(int)(sensor_data.temp_sht*10);
			AppData.Buff[i++] =(int)(sensor_data.hum_sht*10)>>8;   
			AppData.Buff[i++] =(int)(sensor_data.hum_sht*10);
		}
		#endif
	}
	
	else if(mode==2)
	{
		AppData.Buff[i++] =(batteryLevel_mV>>8);       //level of battery in mV
		AppData.Buff[i++] =batteryLevel_mV & 0xFF;
	
		AppData.Buff[i++]=(int)(sensor_data.temp1*10)>>8;     //DS18B20
		AppData.Buff[i++]=(int)(sensor_data.temp1*10);
	
		AppData.Buff[i++] =(int)(sensor_data.oil)>>8;          //oil float
		AppData.Buff[i++] =(int)sensor_data.oil;

		if(exti_flag==1)
		{
			AppData.Buff[i++]=(switch_status<<7)|(sensor_data.in1<<1)|0x01|0x04;
			exti_flag=0;
		}
		else
		{
			switch_status=HAL_GPIO_ReadPin(GPIO_EXTI_PORT,GPIO_EXTI_PIN);							
			AppData.Buff[i++]=(switch_status<<7)|(sensor_data.in1<<1)|0x04;
		}

		AppData.Buff[i++]=(sensor_data.distance_mm)>>8;
		AppData.Buff[i++]=(sensor_data.distance_mm);	
		if(mode2_flag==3)
		{
		  AppData.Buff[i++]=(sensor_data.distance_signal_strengh)>>8;
		  AppData.Buff[i++]=(sensor_data.distance_signal_strengh);				
		}
		else
		{
			AppData.Buff[i++] =0xff; 
			AppData.Buff[i++] =0xff;
		}			
	}
	
	else if(mode==3)
	{
		AppData.Buff[i++] =(int)(sensor_data.oil)>>8;          //oil float
		AppData.Buff[i++] =(int)sensor_data.oil;
	
		AppData.Buff[i++] =(int)(sensor_data.ADC_1)>>8;     
		AppData.Buff[i++] =(int)(sensor_data.ADC_1);
		AppData.Buff[i++] =(int)(sensor_data.ADC_2)>>8; 
		AppData.Buff[i++] =(int)(sensor_data.ADC_2);

		if(exti_flag==1)
		{
			AppData.Buff[i++]=(switch_status<<7)|(sensor_data.in1<<1)|0x01|0x08;
			exti_flag=0;
		}
		else
		{
			switch_status=HAL_GPIO_ReadPin(GPIO_EXTI_PORT,GPIO_EXTI_PIN);							
			AppData.Buff[i++]=(switch_status<<7)|(sensor_data.in1<<1)|0x08;
		}
	
		#if defined USE_SHT
		if(bh1750flags==1)
		{
			AppData.Buff[i++] =(int)(sensor_data.illuminance)>>8;      
			AppData.Buff[i++] =(int)(sensor_data.illuminance);
			AppData.Buff[i++] = 0x00;   
			AppData.Buff[i++] = 0x00;				
		}	
		else
		{
			AppData.Buff[i++] =(int)(sensor_data.temp_sht*10)>>8;      
			AppData.Buff[i++] =(int)(sensor_data.temp_sht*10);
			AppData.Buff[i++] =(int)(sensor_data.hum_sht*10)>>8;   
			AppData.Buff[i++] =(int)(sensor_data.hum_sht*10);
			}
		#endif
	
		AppData.Buff[i++] =(int)(batteryLevel_mV/100);	
	}
	
  else if(mode==4)
	{		
		AppData.Buff[i++] =(batteryLevel_mV>>8);       //level of battery in mV
		AppData.Buff[i++] =batteryLevel_mV & 0xFF;
	
		AppData.Buff[i++]=(int)(sensor_data.temp1*10)>>8;     //DS18B20
		AppData.Buff[i++]=(int)(sensor_data.temp1*10);
	
		AppData.Buff[i++] =(int)(sensor_data.oil)>>8;          //oil float
		AppData.Buff[i++] =(int)sensor_data.oil;

		if(exti_flag==1)
		{
			AppData.Buff[i++]=(switch_status<<7)|(sensor_data.in1<<1)|0x01|0x0C;
			exti_flag=0;
		}
		else
		{
			switch_status=HAL_GPIO_ReadPin(GPIO_EXTI_PORT,GPIO_EXTI_PIN);				
			AppData.Buff[i++]=(switch_status<<7)|(sensor_data.in1<<1)|0x0C;
		}

		AppData.Buff[i++]=(int)(sensor_data.temp2*10)>>8;     //DS18B20
		AppData.Buff[i++]=(int)(sensor_data.temp2*10);
		AppData.Buff[i++]=(int)(sensor_data.temp3*10)>>8;     //DS18B20
		AppData.Buff[i++]=(int)(sensor_data.temp3*10);
	
	}	
	
	else if(mode==5)
	{
		AppData.Buff[i++] =(batteryLevel_mV>>8);       //level of battery in mV
		AppData.Buff[i++] =batteryLevel_mV & 0xFF;
	
		AppData.Buff[i++]=(int)(sensor_data.temp1*10)>>8;     //DS18B20
		AppData.Buff[i++]=(int)(sensor_data.temp1*10);
	
		AppData.Buff[i++] =(int)(sensor_data.oil)>>8;          //oil float
		AppData.Buff[i++] =(int)sensor_data.oil;

		if(exti_flag==1)
		{
			AppData.Buff[i++]=(switch_status<<7)|(sensor_data.in1<<1)|0x01|0x10;
			exti_flag=0;
		}
		else
		{
			switch_status=HAL_GPIO_ReadPin(GPIO_EXTI_PORT,GPIO_EXTI_PIN);							
			AppData.Buff[i++]=(switch_status<<7)|(sensor_data.in1<<1)|0x10;
		}

		AppData.Buff[i++]=(int)(Weight_Shiwu)>>8;
		AppData.Buff[i++]=(int)(Weight_Shiwu);	
		AppData.Buff[i++] = 0xFF; 
		AppData.Buff[i++] = 0xFF;		
	}
	
	else if(mode==6)
	{	
		AppData.Buff[i++] =(batteryLevel_mV>>8);       //level of battery in mV
		AppData.Buff[i++] =batteryLevel_mV & 0xFF;
		
		AppData.Buff[i++]=(int)(sensor_data.temp1*10)>>8;     //DS18B20
		AppData.Buff[i++]=(int)(sensor_data.temp1*10);
	
		AppData.Buff[i++] =(int)(sensor_data.oil)>>8;          //oil float
		AppData.Buff[i++] =(int)sensor_data.oil;

		AppData.Buff[i++]=(sensor_data.in1<<1)|0x14;
		
		AppData.Buff[i++] = (int)(COUNT)>>24;
		AppData.Buff[i++] =	(int)(COUNT)>>16;	
		AppData.Buff[i++] = (int)(COUNT)>>8;
		AppData.Buff[i++] =	(int)(COUNT);  	
	}
	
	else if(mode==7)
	{	
		AppData.Buff[i++] =(batteryLevel_mV>>8);       //level of battery in mV
		AppData.Buff[i++] =batteryLevel_mV & 0xFF;
				
		int temperatura;				//T
		int temperatura_sensor; //PCBT
		int umidade_relativa;		//RH
		int umidade_absoluta;		//AH
		int horario;						//Time
		
		//PPRINTF("%f\r\n%f\r\n%f\r\n%f\r\n%f\r\n", sensor_data.horario, sensor_data.temperatura, sensor_data.temperatura_sensor, sensor_data.umidade_relativa, sensor_data.umidade_absoluta);
		// Tramsformacao de float para inteiro em representacao de pontoFlutuante
		temperatura					= calculaPontoFlutuante16Bits(sensor_data.temperatura);
		temperatura_sensor	= calculaPontoFlutuante16Bits(sensor_data.temperatura_sensor);
		umidade_relativa		= calculaPontoFlutuante16Bits(sensor_data.umidade_relativa);
		umidade_absoluta		= calculaPontoFlutuante16Bits(sensor_data.umidade_absoluta);
		horario							= calculaPontoFlutuante32Bits(sensor_data.horario);
		//PPRINTF("%i\r\n%i\r\n%i\r\n%i\r\n%i\r\n", horario, temperatura, temperatura_sensor, umidade_relativa, umidade_absoluta);

		
		AppData.Buff[i++]=(temperatura)>>8;
		AppData.Buff[i++]=(temperatura) & 0xFF;	
		
		AppData.Buff[i++]=(temperatura_sensor)>>8;
		AppData.Buff[i++]=(temperatura_sensor) & 0xFF;
		
		AppData.Buff[i++]=(umidade_relativa)>>8;
		AppData.Buff[i++]=(umidade_relativa) & 0xFF;
			
		AppData.Buff[i++]=(umidade_absoluta)>>8;
		AppData.Buff[i++]=(umidade_absoluta) & 0xFF;
		
		
		AppData.Buff[i++] = (sensor_data.in1<<4 | 1);
		
		
		// Enviando primeira mensagem
		AppData.BuffSize = i;
		payloadlens=i;
		LORA_send( &AppData, lora_config_reqack_get());
		
		// Espera ate Lora ser liberado para nova mensagem
		while ( LoRaMacState != LORAMAC_IDLE )
    {
       lora_config_reqack_get();
    }

		// Voltando ao tamanho do buffer normal (-8 eh o numero de casas que o i++ aconteceu)
		int stopLoop = i;
		for(int j=i; j>stopLoop-11; j--)
    {
			//PPRINTF("%x", AppData.Buff[i]);
			AppData.Buff[i--] = 0;
		}
				
		AppData.Buff[i++]=(horario)>>24       ;
		AppData.Buff[i++]=(horario)>>16 & 0xFF;
		
		AppData.Buff[i++]=(horario)>> 8 & 0xFF;
		AppData.Buff[i++]=(horario)     & 0xFF;
		
		AppData.Buff[i++]=0xFF;
		AppData.Buff[i++]=0xFF;
		
		AppData.Buff[i++]=0xFF;
		AppData.Buff[i++]=0xFF;
		
		AppData.Buff[i++]=0xFF;
		AppData.Buff[i++]=0xFF;
		
		// Primeiro caractere as 3 entradas digitais e no segundo caractere o numero da mensagem
		AppData.Buff[i++] = 2;
	}
	
	
	else if(mode==8)
	{	
		AppData.Buff[i++] =(batteryLevel_mV>>8);       //level of battery in mV
		AppData.Buff[i++] =batteryLevel_mV & 0xFF;
		
		int horario; //Time (ja tem no lubcos)
		int v_4um; 		//ISO4um
		int v_6um; 		//ISO6um
		int v_14um; 	//ISO14um
		int v_21um; 	//ISO21um
		int findex; 	//FIndex
		
		// Tramsformacao de float para inteiro em representacao de pontoFlutuante
		v_4um		= calculaPontoFlutuante16Bits(sensor_data.v_4um);
		v_6um		= calculaPontoFlutuante16Bits(sensor_data.v_6um);
		v_14um	= calculaPontoFlutuante16Bits(sensor_data.v_14um);
		v_21um	= calculaPontoFlutuante16Bits(sensor_data.v_21um);
		findex	= calculaPontoFlutuante32Bits(sensor_data.findex);
		horario = calculaPontoFlutuante32Bits(sensor_data.horario);
		//PPRINTF("%f\r\n%f\r\n%f\r\n%f\r\n%f\r\n%f\r\n", sensor_data.v_4um, sensor_data.v_6um, sensor_data.v_14um, sensor_data.v_21um, sensor_data.findex, sensor_data.horario);
		//PPRINTF("%i\r\n%i\r\n%i\r\n%i\r\n%i\r\n%i\r\n", v_4um, v_6um, v_14um, v_21um, findex, horario);
		//PPRINTF("%f\r\n%i\r\n", sensor_data.v_4um, v_4um );
		
		
		AppData.Buff[i++]=(v_4um)>>8;
		AppData.Buff[i++]=(v_4um) & 0xFF;	
		
		AppData.Buff[i++]=(v_6um)>>8;
		AppData.Buff[i++]=(v_6um) & 0xFF;
		
		AppData.Buff[i++]=(v_14um)>>8;
		AppData.Buff[i++]=(v_14um) & 0xFF;		
		
		AppData.Buff[i++]=(v_21um)>>8;
		AppData.Buff[i++]=(v_21um) & 0xFF;
		
		AppData.Buff[i++]=1;
		
		// Enviando primeira mensagem
		AppData.BuffSize = i;
		payloadlens=i;
		LORA_send( &AppData, lora_config_reqack_get());
		
		// Espera ate Lora ser liberado para nova mensagem
		while ( LoRaMacState != LORAMAC_IDLE )
    {
       lora_config_reqack_get();
    }

		// Voltando ao tamanho do buffer normal (-8 eh o numero de casas que o i++ aconteceu)
		int stopLoop = i;
		for(int j=i; j>stopLoop-11; j--)
    {
			AppData.Buff[i--] = 0;
		}

		
		AppData.Buff[i++]=(findex)>>24       ;
		AppData.Buff[i++]=(findex)>>16 & 0xFF;
		
		AppData.Buff[i++]=(findex)>> 8 & 0xFF;
		AppData.Buff[i++]=(findex)     & 0xFF;
		
		AppData.Buff[i++]=(horario)>>24       ;
		AppData.Buff[i++]=(horario)>>16 & 0xFF;
		
		AppData.Buff[i++]=(horario)>> 8 & 0xFF;
		AppData.Buff[i++]=(horario)     & 0xFF;
		
		AppData.Buff[i++]=0xFF;
		AppData.Buff[i++]=0xFF;
		
		
		// Primeiro caractere as 3 entradas digitais e no segundo caractere o numero da mensagem
		AppData.Buff[i++] = (sensor_data.in1<<4 | 2);	
	}
	
	
	// Enviando mensagem LoRa em todos os modos
	AppData.BuffSize = i;
	payloadlens=i;
  LORA_send( &AppData, lora_config_reqack_get());
	#endif
	
	if (mode == 7 || mode == 8)
	{
		// Espera ate Lora ser liberado para nova mensagem
		while ( LoRaMacState != LORAMAC_IDLE )
    {
       lora_config_reqack_get();
    }
		
		// ativa interrupcao
		//GPIOB_EXTI_FALLINGEDGE_PULLUP_PIN_Init( GPIO_PIN_6 );
		interrupcao_flag = 0;
	}
	
}


static void LORA_RxData( lora_AppData_t *AppData )
{
	is_there_data=1;
		
  set_at_receive(AppData->Port, AppData->Buff, AppData->BuffSize);
	
	switch(AppData->Buff[0] & 0xff)
  {		
	  case 0x01:   
	  {
		  if( AppData->BuffSize == 4 )  //---->AT+TDC
			{
			  ServerSetTDC=( AppData->Buff[1]<<16 | AppData->Buff[2]<<8 | AppData->Buff[3] );//S
					
				if(ServerSetTDC<6)
				{
		      APP_TX_DUTYCYCLE=6000;							
				}
				else
				{
				  TDC_flag=1;
			    APP_TX_DUTYCYCLE=ServerSetTDC*1000;
			  }
				rxpr_flags=1;							
			}
		  break;
		}
				
		case 0x04:  
		{
			if( AppData->BuffSize == 2 )
			{
				if(AppData->Buff[1]==0xFF)  //---->ATZ
			  {
					atz_flags=1;
					rxpr_flags=1;		
				}
				else if(AppData->Buff[1]==0xFE)  //---->AT+FDR
				{			
					FLASH_erase(0x8018F80);//page 799					
				  FLASH_program_on_addr(0x8018F80,0x12);	
          FLASH_erase(FLASH_USER_START_ADDR_CONFIG);//Page800 					
					atz_flags=1;						
					rxpr_flags=1;								
				}
			}
			break;
	  }
		
		case 0x05:
		{
			if( AppData->BuffSize == 2 )   
			{
				if(AppData->Buff[1]==0x01)    //---->AT+CFM=1
				{
					lora_config_reqack_set(LORAWAN_CONFIRMED_MSG);
					Store_Config();
					rxpr_flags=1;	
				}
			  else if(AppData->Buff[1]==0x00)  //---->AT+CFM=0
				{
					lora_config_reqack_set(LORAWAN_UNCONFIRMED_MSG);
					Store_Config();
					rxpr_flags=1;	
				}					
		  }
			break;
	  }	
		
    case 0x06:
    {
			if( AppData->BuffSize == 4 )
			{
			  if((AppData->Buff[1]==0x00)&&(AppData->Buff[2]==0x00)&&(AppData->Buff[3]==0x00))   //---->AT+INTMOD=0
				{
					GPIO_EXTI_IoDeInit();
					inmode=0;
					Store_Config();
					rxpr_flags=1;		
				}
				else if((AppData->Buff[1]==0x00)&&(AppData->Buff[2]==0x00)&&(AppData->Buff[3]==0x01))    //---->AT+INTMOD=1
				{
				  GPIO_EXTI_RISING_FALLINGInit();
					inmode=1;
				  Store_Config();
				  rxpr_flags=1;		
				}
				else if((AppData->Buff[1]==0x00)&&(AppData->Buff[2]==0x00)&&(AppData->Buff[3]==0x02))    //---->AT+INTMOD=2
				{
					GPIO_EXTI_FALLINGInit();
					inmode=2;
					Store_Config();
					rxpr_flags=1;								
				} 
				else if((AppData->Buff[1]==0x00)&&(AppData->Buff[2]==0x00)&&(AppData->Buff[3]==0x03))    //---->AT+INTMOD=3
				{
					GPIO_EXTI_RISINGInit();
					inmode=3;
				  Store_Config();
					rxpr_flags=1;								
				}							
			}
			break;	
		}		
		
		case 0x07:
		{
			if( AppData->BuffSize == 3 )
			{	
			  power_time=(AppData->Buff[1]<<8) | AppData->Buff[2];  //---->AT+5VT
				Store_Config();
			  rxpr_flags=1;							
			}
			break;									
		}
		
    case 0x08:			
		{
			if(mode==5)
			{
				if((AppData->BuffSize == 2 )&&(AppData->Buff[1]==0x01))   //---->AT+WEIGRE
				{	
				  weightreset();
				  rxpr_flags=1;							
				}
				else if((AppData->BuffSize == 4 )&&(AppData->Buff[1]==0x02))  //---->AT+WEIGAP
				{
				  GapValue=(float)((AppData->Buff[2]<<8 | AppData->Buff[3])/10.0);
					Store_Config();					
					rxpr_flags=1;							
				}
			}
			break;			
	  }		
		
		case 0x0A:
		{
			if( AppData->BuffSize == 2 )         
			{	
				if((AppData->Buff[1]>=0x01)&&(AppData->Buff[1]<=0x06))    //---->AT+MOD
				{
					mode=AppData->Buff[1];
					Store_Config();
					atz_flags=1;						
					rxpr_flags=1;	
				}						 
			}				
			break;
		}
		
    case 0x20:			
		{
			if( AppData->BuffSize == 2 )
			{		
				if((AppData->Buff[1]==0x00)||(AppData->Buff[1]==0x01))    
				{
					if(AppData->Buff[1]==0x01)       //---->AT+NJM=1
					{
						lora_config_otaa_set(LORA_ENABLE);
					}
					else                             //---->AT+NJM=0
					{
						lora_config_otaa_set(LORA_DISABLE);							
					}
					Store_Config();
					atz_flags=1;
					rxpr_flags=1;		
				}						 
			}
			break;				
		}	
		
		case 0x21:
		{
		  if( (AppData->BuffSize == 2) && (AppData->Buff[1]<=4) )
			{
				response_level=( AppData->Buff[1] );//0~4					//---->AT+RPL
				Store_Config();
				rxpr_flags=1;							
			}
			break;
		}		
		
    case 0x22:			
		{
			MibRequestConfirm_t mib;
			if(( AppData->BuffSize == 2 )&&(AppData->Buff[1]==0x01))   //---->AT+ADR=1
			{		
				mib.Type = MIB_ADR;
				mib.Param.AdrEnable =AppData->Buff[1];
				LoRaMacMibSetRequestConfirm( &mib );					
				Store_Config();
				rxpr_flags=1;								
			}
			else if((AppData->BuffSize == 4 )&&(AppData->Buff[1]==0x00))   //---->AT+ADR=0
			{
				mib.Type = MIB_ADR;					
				mib.Param.AdrEnable = AppData->Buff[1];
				LoRaMacMibSetRequestConfirm( &mib );						
				if(AppData->Buff[2]!=0xff)                //---->AT+DR
				{
				 uint8_t datarate=AppData->Buff[2];
				 #if defined( REGION_AS923 )
				     if(datarate<8)
						 {
								lora_config_tx_datarate_set(AppData->Buff[2]);							
						 }
				 #elif defined( REGION_AU915 )
						 if((datarate!=7)&&(datarate<14))
						 {
								lora_config_tx_datarate_set(AppData->Buff[2]);							
						 }
				 #elif defined( REGION_CN470 )
					   if(datarate<6)	
						 {
								lora_config_tx_datarate_set(AppData->Buff[2]);							
						 }
				 #elif defined( REGION_CN779 )
						 if(datarate<8)
						 {
								lora_config_tx_datarate_set(AppData->Buff[2]);							
						 }
         #elif defined( REGION_EU433 )
						 if(datarate<8)
						 {
								lora_config_tx_datarate_set(AppData->Buff[2]);							
						 }
				 #elif defined( REGION_IN865 )
						 if(datarate<8)
						 {
								lora_config_tx_datarate_set(AppData->Buff[2]);							
						 }
				 #elif defined( REGION_EU868 )
						 if(datarate<8)
						 {
								lora_config_tx_datarate_set(AppData->Buff[2]);							
						 }
				 #elif defined( REGION_KR920 )
						 if(datarate<6)
						 {
								lora_config_tx_datarate_set(AppData->Buff[2]);							
						 }
				 #elif defined( REGION_US915 )
						 if(((datarate<5)||(datarate>7))&&(datarate<14))
						 {
								lora_config_tx_datarate_set(AppData->Buff[2]);							
						 }
				 #elif defined( REGION_RU864 )
						 if(datarate<8)
						 {
								lora_config_tx_datarate_set(AppData->Buff[2]);							
						 }
				 #elif defined( REGION_KZ865 )
						 if(datarate<8)
						 {
								lora_config_tx_datarate_set(AppData->Buff[2]);							
						 }
				 #endif
								
				}
				if(AppData->Buff[3]!=0xff)                //---->AT+TXP
				{
					mib.Type = MIB_CHANNELS_TX_POWER;						
					mib.Param.ChannelsTxPower=AppData->Buff[3];
					LoRaMacMibSetRequestConfirm( &mib );							
				}				
				Store_Config();
				rxpr_flags=1;									
			 }
			 break;				
		}			
		
    case 0x23:			
		{
			if( AppData->BuffSize == 2 )
			{		
				lora_config_application_port_set(AppData->Buff[1]);    //---->AT+PORT
				Store_Config();
				rxpr_flags=1;							 
			}
			break;					
		}	
		
    case 0x24:			
	  {
		  #if defined( REGION_US915 )	|| defined( REGION_AU915 ) ||	defined( REGION_CN470 ) 		
			if( AppData->BuffSize == 2 )
			{		
			  if(AppData->Buff[1]<=0x0C)
				{
					customize_set8channel_set(AppData->Buff[1]);    //---->AT+CHE
					Store_Config();		
					rxpr_flags=1;		
				}				
			}
		  #endif	
			break;				
	  }	
		
    case 0x25:			
		{
		 #if defined( REGION_AS923 )	|| defined( REGION_AU915 )
		 if( AppData->BuffSize == 2 )
		 {				
			 if((AppData->Buff[1]==0x00)||(AppData->Buff[1]==0x01))   //---->AT+DWELLT
			 {
				 dwelltime=AppData->Buff[1];
			 	 Store_Config();
				 atz_flags=1;
				 rxpr_flags=1;		
			 }						
		 }
		 #endif	
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
		TimerStart( &IWDGRefreshTimer);		
		TDC_flag=0;
	}	

	AT_PRINTF("\r\n");		
	AT_PRINTF("Receive data\n\r");
	if((AppData->BuffSize<=8)&&(rxpr_flags==1))
	{		
		AT_PRINTF("%d:",AppData->Port);		
		for (int i = 0; i < AppData->BuffSize; i++)
		{
			AT_PRINTF("%02x ", AppData->Buff[i]);
		}
		AT_PRINTF("\n\r");
	}
	else
	{
		/*
		// Inicializa clock
		__HAL_RCC_GPIOB_CLK_ENABLE();
		
		
		// Inicializa porta
		GPIO_InitTypeDef GPIO_InitStruct={0};
		
		GPIO_InitStruct.Pin = GPIO_PIN_4;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		HW_GPIO_Init( GPIOB, GPIO_PIN_4, &GPIO_InitStruct );
		
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
		
		*/
		AT_PRINTF("BuffSize:%d,Run AT+RECVB=? to see detail\r\n",AppData->BuffSize);		
		
	}
	rxpr_flags=0;	
}

#if defined(LoRa_Sensor_Node)
static void OnTxTimerEvent( void )
{
	
	TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
	
  /*Wait for next tx slot*/
  TimerStart( &TxTimer);

	if((exti_flag==1)&&(( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
	{

	}
	else if((exti_flag==0)&&(( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
	{	
	  uplink_data_status=1;
	}
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
static void OnTxTimerEvent2( void )
{
	if(join_flag==0)
	{
		TimerSetValue( &TxTimer2,  500);
	
		/*Wait for next tx slot*/
		TimerStart( &TxTimer2);
		
		join_flag++;
	}
	else if(join_flag==1)
	{
		LoraStartTx(TX_ON_TIMER);
		join_flag++;
	}
}

static void LoraStartjoin(TxEventType_t EventType)
{
  if (EventType == TX_ON_TIMER)
  {
    /* send everytime timer elapses */
    TimerInit( &TxTimer2, OnTxTimerEvent2 );
    TimerSetValue( &TxTimer2, 500); 
	
    OnTxTimerEvent2();
  }
}

static void OnIWDGRefreshTimeoutEvent( void )
{
	TimerSetValue( &IWDGRefreshTimer,  18000);

  TimerStart( &IWDGRefreshTimer);

	is_time_to_IWDG_Refresh=1;
}

static void StartIWDGRefresh(TxEventType_t EventType)
{
  if (EventType == TX_ON_EVENT)
  {
    /* send everytime timer elapses */
    TimerInit( &IWDGRefreshTimer, OnIWDGRefreshTimeoutEvent );
    TimerSetValue( &IWDGRefreshTimer,  18000); 
		TimerStart( &IWDGRefreshTimer);
  }
}

static void OnReJoinTimerEvent( void )
{
	TimerStop( &ReJoinTimer);
	
	is_time_to_rejoin=1;
}

static void LoraStartRejoin(TxEventType_t EventType)
{
  if (EventType == TX_ON_EVENT)
  {
    /* send everytime timer elapses */
    TimerInit( &ReJoinTimer, OnReJoinTimerEvent );
    TimerSetValue( &ReJoinTimer,  REJOIN_TX_DUTYCYCLE*60000); 
		TimerStart( &ReJoinTimer);
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
		 if(mode!=6)
		 {
			 is_check_exit=1;
			 if((( LoRaMacState & 0x00000001 ) != 0x00000001) && (( LoRaMacState & 0x00000010 ) != 0x00000010))
			 {
				 uplink_data_status=1;
			 }
			 else if(inmode==1)
			 {
				 exti_flag=0;
			 }
		 }
		 else
		 {
			if(debug_flags==1)
			{
				PPRINTF("COUNT is %d\r\n",COUNT);	
			}
      if((COUNT%30==0)&&(COUNT!=0))
      {
				IWDG_Refresh();				
			}				 
			exti_flag=0;			 
		 }
	}
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
