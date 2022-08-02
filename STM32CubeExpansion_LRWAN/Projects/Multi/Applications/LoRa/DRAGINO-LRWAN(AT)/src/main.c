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
static uint8_t downlink_command_buffer[4];
static uint8_t downlink_command_buffersize=0;
static uint8_t downlinklens;
static uint8_t downlink_send[51];
uint8_t switch_status=0,switch_status2=0,switch_status3=0;
static uint8_t normal_status=0,normal2_status=0,normal3_status=0;
bool is_check_exit=0;
bool rxpr_flags=0;
int exti_flag=0,exti_flag2=0,exti_flag3=0;
uint32_t COUNT=0,COUNT2=0;
uint8_t TDC_flag=0;
uint8_t join_flag=0;
uint8_t atz_flags=0;
uint16_t batteryLevel_mV;
uint8_t payloadlens;
bool is_time_to_IWDG_Refresh=0;
bool join_network=0;
bool joined_flags=0;
bool joined_led_flags=0;
bool joined_led_end=0;
bool is_there_data=0;
bool is_time_to_rejoin=0;
bool JoinReq_NbTrails_over=0;
bool unconfirmed_downlink_data_ans_status=0,confirmed_downlink_data_ans_status=0;
bool rejoin_status=0;
bool rejoin_keep_status=0;
bool MAC_COMMAND_ANS_status=0;
bool uplink_data_status=0;
bool downlink_data_status=0;
bool is_time_to_reply_downlink=0;
bool uplink_message_data_status=0;
uint8_t response_level=0;
uint16_t REJOIN_TX_DUTYCYCLE=20;//min
uint32_t Automatic_fdr_network[1]={0x12};

uint8_t downlink_detect_switch=0;
uint16_t downlink_detect_timeout=0;
uint8_t downlink_received_status=0;
uint8_t LoRaMacState_error_times=0;

uint8_t confirmed_uplink_counter_retransmission_increment_switch=0;
uint8_t confirmed_uplink_retransmission_nbtrials=0;

uint8_t LinkADR_NbTrans_uplink_counter_retransmission_increment_switch=0;
uint8_t LinkADR_NbTrans_retransmission_nbtrials=0;

uint8_t unconfirmed_uplink_change_to_confirmed_uplink_status=0;
uint16_t unconfirmed_uplink_change_to_confirmed_uplink_timeout=0;

extern bool bh1750flags;
extern uint8_t mode;
extern uint8_t mode2_flag;
extern uint8_t inmode,inmode2,inmode3;
extern float GapValue;
extern uint16_t power_time;
extern bool message_flags;
extern uint32_t LoRaMacState;
extern uint8_t dwelltime;
extern bool debug_flags;
extern bool mac_response_flag;

static void send_exti(void);
static void Flash_copy_key_to_EEPROM(void);
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
static void Send_device_status(void);
static void Send_reply_downlink( void );

#if defined(LoRa_Sensor_Node)
/* start the tx process*/
static void LoraStartTx(void);
static void LoraStartjoin(void);
static void StartIWDGRefresh(void);
static void LoraStartRejoin(void);
static void StartDownlinkDetect(void);
static void StartUnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer(void);

TimerEvent_t TxTimer;
static TimerEvent_t TxTimer2;
static TimerEvent_t IWDGRefreshTimer;//watch dog
TimerEvent_t ReJoinTimer;
TimerEvent_t NetworkJoinedLedTimer;
TimerEvent_t DownlinkDetectTimeoutTimer;
TimerEvent_t UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer;

/* tx timer callback function*/
static void OnTxTimerEvent( void );
static void OnTxTimerEvent2( void );
static void OnIWDGRefreshTimeoutEvent(void);
static void OnReJoinTimerEvent( void );
static void OnNetworkJoinedLedEvent(void);
static void OnDownlinkDetectTimeoutEvent( void );
static void UnconfirmedUplinkChangeToConfirmedUplinkTimeoutEvent( void );
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
	
 	StartIWDGRefresh(); 
		
	Flash_copy_key_to_EEPROM();
	
	new_firmware_update();
	
  /*Disbale Stand-by mode*/
  LPM_SetOffMode(LPM_APPLI_Id , LPM_Disable );
	
  /* Configure the Lora Stack*/
  LORA_Init( &LoRaMainCallbacks, &LoRaParamInit);
	
  while( 1 )
  {
		/* Handle UART commands */
    CMD_Process();

		if(joined_led_flags==1)
		{
			joined_led_flags=0;
			joined_led_end=1;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
			TimerInit( &NetworkJoinedLedTimer, OnNetworkJoinedLedEvent );
			TimerSetValue( &NetworkJoinedLedTimer, 5000);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET); 
			TimerStart( &NetworkJoinedLedTimer );
		}	
		
		if(is_time_to_rejoin==1)
		{
			if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
			{
				LoRaMacState_error_times=0;
				unconfirmed_uplink_change_to_confirmed_uplink_status=0;
			  is_time_to_rejoin=0;
			  LORA_Join();
			}
		}
		
		if(rejoin_status==1)
		{
			if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
			{
				rejoin_keep_status=1;
				LoRaMacState_error_times=0;
				unconfirmed_uplink_change_to_confirmed_uplink_status=0;
				TimerStop(&TxTimer);
				TimerStop(&DownlinkDetectTimeoutTimer);
				TimerStop(&UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer);
			  LORA_Join();
			}
		}
		
		if(joined_flags==1)
		{
			send_exti();

			if(LoRaMacState_error_times>=5)
			{
				LoRaMacState_error_times=0;
				HAL_Delay(100);
				NVIC_SystemReset();
			}
		
			if(atz_flags==1)
			{
				HAL_Delay(500);
				AppData.Buff[0]=0x11;
	      AppData.BuffSize=1;
	      AppData.Port = 4;
	      LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
				atz_flags++;
			}
		  else if((atz_flags==2)&&(( LoRaMacState & 0x00000001 ) != 0x00000001))
			{
				NVIC_SystemReset();
			}		
			
			if((uplink_data_status==1)&&(( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
			{
				Send();
				uplink_data_status=0;
			}	
      
      if(is_check_exit==1)
			{
				if((( LoRaMacState & 0x00000001 ) != 0x00000001) &&(( LoRaMacState & 0x00000010 ) != 0x00000010))
				{
					normal_status=HAL_GPIO_ReadPin(GPIO_EXTI14_PORT,GPIO_EXTI14_PIN);
					normal2_status=HAL_GPIO_ReadPin(GPIO_EXTI15_PORT,GPIO_EXTI15_PIN);
					normal3_status=HAL_GPIO_ReadPin(GPIO_EXTI4_PORT,GPIO_EXTI4_PIN);
					is_check_exit=0;
					if(((switch_status!=normal_status)&&(mode!=6)&&(mode!=9)&&(inmode==1))||
						((switch_status2!=normal2_status)&&(mode==7)&&(inmode2==1))||
					  ((switch_status3!=normal3_status)&&((mode==7)||(mode==9))&&(inmode3==1)))
					{
						switch_status=normal_status;
						switch_status2=normal2_status;
						switch_status3=normal3_status;
						uplink_data_status=1;
					}		
				}
			}	
			
			#ifdef REGION_US915
			if((MAC_COMMAND_ANS_status==1)&&(mac_response_flag==0))
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
						AppData.Port = 4;							
	          LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
						MAC_COMMAND_ANS_status=0;
				  }
				}		  
			}
			#elif defined( REGION_AS923 )	|| defined( REGION_AU915 )		
			if((MAC_COMMAND_ANS_status==1)&&(dwelltime==1)&&(mac_response_flag==0))
			{		
				if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
				{          
					MibRequestConfirm_t mib;
			
			    mib.Type=MIB_CHANNELS_DATARATE;
			    LoRaMacMibGetRequestConfirm(&mib);
					
				  if(mib.Param.ChannelsDatarate==2)
			    {
						AppData.Buff[0]=0x00;
						AppData.BuffSize=1;
						AppData.Port = 4;							
	          LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
						MAC_COMMAND_ANS_status=0;
				  }
				}		  
			}		
			#endif
		
			if((MAC_COMMAND_ANS_status==1 && response_level==3) 
					|| (unconfirmed_downlink_data_ans_status==1 && is_there_data==1 && response_level==1) 
					|| (confirmed_downlink_data_ans_status==1 && is_there_data==1 && response_level==2)
					||(((MAC_COMMAND_ANS_status==1)||(confirmed_downlink_data_ans_status==1&&is_there_data==1))&&(response_level==4))
			    ||(((MAC_COMMAND_ANS_status==1)||(is_there_data==1))&&(response_level==5)))
			{
				if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
				{
					MAC_COMMAND_ANS_status=0;
					unconfirmed_downlink_data_ans_status=0;
					confirmed_downlink_data_ans_status=0;
					is_there_data=0;
					if(downlink_data_status==1)
					{
						downlink_data_status=0;
						Send_reply_downlink();
					}
					else
					{
						AppData.Buff[0]=0x00;
						AppData.BuffSize=1;
						AppData.Port = 4;							
						LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
					}
				}
			}
	
			if(downlink_received_status==1 && LORA_JoinStatus () == LORA_SET && downlink_detect_switch==1 && downlink_detect_timeout>0 && unconfirmed_uplink_change_to_confirmed_uplink_timeout>0)
			{
				downlink_received_status=0;
				unconfirmed_uplink_change_to_confirmed_uplink_status=0;
				TimerSetValue( &DownlinkDetectTimeoutTimer,  downlink_detect_timeout*60000); 
				TimerStart( &DownlinkDetectTimeoutTimer);
				
				if(lora_config_reqack_get()==LORAWAN_UNCONFIRMED_MSG)
				{
					TimerSetValue( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer,  unconfirmed_uplink_change_to_confirmed_uplink_timeout*60000); 
					TimerStart( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer);
				}
			}	
		
			if(is_time_to_reply_downlink==1)
			{
				if((( LoRaMacState & 0x00000001 ) != 0x00000001)&&(( LoRaMacState & 0x00000010 ) != 0x00000010))
				{
					is_time_to_reply_downlink=0;
					
					for(int i=0;i<downlink_command_buffersize;i++)
					{
						AppData.Buff[i]=downlink_command_buffer[i];
					}
					
					AppData.BuffSize=downlink_command_buffersize;
					AppData.Port = 12;
					LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
				}
			}		

			if((uplink_message_data_status==1)&&(( LoRaMacState & 0x00000001 ) != 0x00000001) &&(( LoRaMacState & 0x00000010 ) != 0x00000010))
			{				
        Send_device_status();			
				uplink_message_data_status=0;
			}			
		}
		
		if(JoinReq_NbTrails_over==1)
		{
			JoinReq_NbTrails_over=0;
			
			rejoin_keep_status=1;
			
			if(REJOIN_TX_DUTYCYCLE==0)
			{
				REJOIN_TX_DUTYCYCLE=20;
			}
			LoraStartRejoin();
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
	exti_flag=0;
	exti_flag2=0;
	exti_flag3=0;	
	joined_flags=1;
  joined_led_flags=1;
	join_network=1;
	
  AT_PRINTF("JOINED\r\n");

	rejoin_keep_status=0;
	
	if((lora_config_otaa_get() == LORA_ENABLE ? 1 : 0))
	{
		printf_joinmessage();
	}		
	
	TimerStop(&ReJoinTimer);	
	
  LORA_RequestClass( LORAWAN_DEFAULT_CLASS );
	
	#if defined(LoRa_Sensor_Node) /*LSN50 Preprocessor compile swicth:hw_conf.h*/
	LoraStartjoin();	
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
	
	BSP_sensor_Read( &sensor_data,message_flags );
	message_flags=0;
	
	uint8_t exit_temp,exit2_temp,exit3_temp;
	exit_temp=exti_flag;
	exit2_temp=exti_flag2;
	exit3_temp=exti_flag3;
	
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

		if(exit_temp==0)
		{
			switch_status=HAL_GPIO_ReadPin(GPIO_EXTI14_PORT,GPIO_EXTI14_PIN);		
		}
		AppData.Buff[i++]=(switch_status<<7)|(sensor_data.in1<<1)|(exit_temp&0x01);
	
		#if defined USE_SHT
		if(bh1750flags==1)
		{
			AppData.Buff[i++] =(sensor_data.illuminance)>>8;      
			AppData.Buff[i++] =(sensor_data.illuminance);
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
		
//		if(exit2_temp==0)
//		{
//			switch_status2=HAL_GPIO_ReadPin(GPIO_EXTI15_PORT,GPIO_EXTI15_PIN);	
//		}
//		if(exit3_temp==0)
//		{
//			switch_status3=HAL_GPIO_ReadPin(GPIO_EXTI4_PORT,GPIO_EXTI4_PIN);		
//		}	
//		
//  	AppData.Buff[i++] =(exit2_temp<<4)|switch_status2;           				
//	  AppData.Buff[i++] =(exit3_temp<<4)|switch_status3;   		
	}
	
	else if(mode==2)
	{
		AppData.Buff[i++] =(batteryLevel_mV>>8);       //level of battery in mV
		AppData.Buff[i++] =batteryLevel_mV & 0xFF;
	
		AppData.Buff[i++]=(int)(sensor_data.temp1*10)>>8;     //DS18B20
		AppData.Buff[i++]=(int)(sensor_data.temp1*10);
	
		AppData.Buff[i++] =(int)(sensor_data.oil)>>8;          //oil float
		AppData.Buff[i++] =(int)sensor_data.oil;

		if(exit_temp==0)
		{
			switch_status=HAL_GPIO_ReadPin(GPIO_EXTI14_PORT,GPIO_EXTI14_PIN);		
		}
		AppData.Buff[i++]=(switch_status<<7)|(sensor_data.in1<<1)|0x04|(exit_temp&0x01);

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

		if(exit_temp==0)
		{
			switch_status=HAL_GPIO_ReadPin(GPIO_EXTI14_PORT,GPIO_EXTI14_PIN);		
		}
		AppData.Buff[i++]=(switch_status<<7)|(sensor_data.in1<<1)|0x08|(exit_temp&0x01);
	
		#if defined USE_SHT
		if(bh1750flags==1)
		{
			AppData.Buff[i++] =(sensor_data.illuminance)>>8;      
			AppData.Buff[i++] =(sensor_data.illuminance);
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

		if(exit_temp==0)
		{
			switch_status=HAL_GPIO_ReadPin(GPIO_EXTI14_PORT,GPIO_EXTI14_PIN);		
		}
		AppData.Buff[i++]=(switch_status<<7)|(sensor_data.in1<<1)|0x0C|(exit_temp&0x01);

		AppData.Buff[i++]=(int)(sensor_data.temp2*10)>>8;     //PA9-DS18B20
		AppData.Buff[i++]=(int)(sensor_data.temp2*10);
		AppData.Buff[i++]=(int)(sensor_data.temp3*10)>>8;     //PA10-DS18B20
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

		if(exit_temp==0)
		{
			switch_status=HAL_GPIO_ReadPin(GPIO_EXTI14_PORT,GPIO_EXTI14_PIN);		
		}
		AppData.Buff[i++]=(switch_status<<7)|(sensor_data.in1<<1)|0x10|(exit_temp&0x01);

		AppData.Buff[i++] =(int)((sensor_data.Weight)>>8);
		AppData.Buff[i++] =(int)(sensor_data.Weight);	
		AppData.Buff[i++] =(int)((sensor_data.Weight)>>24);
		AppData.Buff[i++] =(int)((sensor_data.Weight)>>16);	
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
		
		AppData.Buff[i++] = (uint8_t)((COUNT)>>24);
		AppData.Buff[i++] =	(uint8_t)((COUNT)>>16);	
		AppData.Buff[i++] = (uint8_t)((COUNT)>>8);
		AppData.Buff[i++] =	(uint8_t)(COUNT);  	
	}
	
	else if(mode==7)
	{
		AppData.Buff[i++] =(batteryLevel_mV>>8);       //level of battery in mV
		AppData.Buff[i++] =batteryLevel_mV & 0xFF;
		
		AppData.Buff[i++]=(int)(sensor_data.temp1*10)>>8;     //DS18B20
		AppData.Buff[i++]=(int)(sensor_data.temp1*10);
	
		AppData.Buff[i++] =(int)(sensor_data.oil)>>8;          //oil float
		AppData.Buff[i++] =(int)sensor_data.oil;

		if(exit_temp==0)
		{
			switch_status=HAL_GPIO_ReadPin(GPIO_EXTI14_PORT,GPIO_EXTI14_PIN);		
		}
		if(exit2_temp==0)
		{
			switch_status2=HAL_GPIO_ReadPin(GPIO_EXTI15_PORT,GPIO_EXTI15_PIN);	
		}
		if(exit3_temp==0)
		{
			switch_status3=HAL_GPIO_ReadPin(GPIO_EXTI4_PORT,GPIO_EXTI4_PIN);		
		}	
		
		AppData.Buff[i++] =(switch_status<<7)|(sensor_data.in1<<1)|0x18|(exit_temp&0x01);			
  	AppData.Buff[i++] =(exit2_temp<<4)|switch_status2;           				
	  AppData.Buff[i++] =(exit3_temp<<4)|switch_status3;   
		
		AppData.Buff[i++] =0xff; 
		AppData.Buff[i++] =0xff;			
	}
	
	else if(mode==8)
	{	
		AppData.Buff[i++] =(batteryLevel_mV>>8);       //level of battery in mV
		AppData.Buff[i++] =batteryLevel_mV & 0xFF;
	
		AppData.Buff[i++]=(int)(sensor_data.temp1*10)>>8;     //DS18B20
		AppData.Buff[i++]=(int)(sensor_data.temp1*10);	
		
		AppData.Buff[i++] =(int)(sensor_data.oil)>>8;          //oil float
		AppData.Buff[i++] =(int)sensor_data.oil;

		if(exit_temp==0)
		{
			switch_status=HAL_GPIO_ReadPin(GPIO_EXTI14_PORT,GPIO_EXTI14_PIN);		
		}
		AppData.Buff[i++]=(switch_status<<7)|(sensor_data.in1<<1)|0x1C|(exit_temp&0x01);
	
		AppData.Buff[i++] =(int)(sensor_data.ADC_1)>>8;     
		AppData.Buff[i++] =(int)(sensor_data.ADC_1);
		AppData.Buff[i++] =(int)(sensor_data.ADC_2)>>8; 
		AppData.Buff[i++] =(int)(sensor_data.ADC_2);		
	}

	else if(mode==9)
	{	
		AppData.Buff[i++] =(batteryLevel_mV>>8);       //level of battery in mV
		AppData.Buff[i++] =batteryLevel_mV & 0xFF;
	
		AppData.Buff[i++]=(int)(sensor_data.temp1*10)>>8;     //DS18B20
		AppData.Buff[i++]=(int)(sensor_data.temp1*10);

		AppData.Buff[i++]=(int)(sensor_data.temp2*10)>>8;     //PA9-DS18B20
		AppData.Buff[i++]=(int)(sensor_data.temp2*10);
		
		if(exit3_temp==0)
		{
			switch_status3=HAL_GPIO_ReadPin(GPIO_EXTI4_PORT,GPIO_EXTI4_PIN);		
		}	
		AppData.Buff[i++]=(switch_status3<<7)|(sensor_data.in1<<1)|0x20|(exit3_temp&0x01);		
		
		AppData.Buff[i++]=(int)(sensor_data.temp3*10)>>8;     //PA10-DS18B20
		AppData.Buff[i++]=(int)(sensor_data.temp3*10);
		
		AppData.Buff[i++] = (uint8_t)((COUNT)>>24);
		AppData.Buff[i++] =	(uint8_t)((COUNT)>>16);	
		AppData.Buff[i++] = (uint8_t)((COUNT)>>8);
		AppData.Buff[i++] =	(uint8_t)(COUNT); 	
	
		AppData.Buff[i++] = (uint8_t)((COUNT2)>>24);
		AppData.Buff[i++] =	(uint8_t)((COUNT2)>>16);	
		AppData.Buff[i++] = (uint8_t)((COUNT2)>>8);
		AppData.Buff[i++] =	(uint8_t)(COUNT2); 		
	}
	
	if(exit_temp==1)
	{
		exti_flag=0;
	}
	if(exit2_temp==1)
	{
		exti_flag2=0;
	}
	if(exit3_temp==1)
	{
		exti_flag3=0;
	}
	
	if((inmode==1)||(inmode2==1)||(inmode3==1))
	{
		is_check_exit=1;
	}
	
	AppData.BuffSize = i;
	payloadlens=i;
	
  if(unconfirmed_uplink_change_to_confirmed_uplink_status==1)
  {
		LORA_send( &AppData, LORAWAN_CONFIRMED_MSG);
	}
	else
	{	
		LORA_send( &AppData, lora_config_reqack_get());
	}
	#endif	
}

static void Send_device_status(void)
{
  device_t device_data; 
  uint32_t tdc_temp;
	tdc_temp=APP_TX_DUTYCYCLE/1000;
	
	is_there_data=0;		
  if ( LORA_JoinStatus () != LORA_SET)
  {
    /*Not joined, try again later*/
    return;
  }
	
	Device_status(&device_data);
	
	uint32_t i = 0;

  AppData.Port = 5;
	
	AppData.Buff[i++] = device_data.freq_band;
	
	AppData.Buff[i++] = device_data.sub_band;
	
	AppData.Buff[i++] = (device_data.firm_ver>>8)&0xff;
	AppData.Buff[i++] =  device_data.firm_ver&0xff;
	
	AppData.Buff[i++] = (tdc_temp>>16)&0xff;
	AppData.Buff[i++] = (tdc_temp>>8)&0xff;
	AppData.Buff[i++] =  tdc_temp&0xff;
	
	AppData.BuffSize = i;
	payloadlens=i;
  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);		  
}

static void Send_reply_downlink( void )
{
	is_there_data=0;  	
  if ( LORA_JoinStatus () != LORA_SET)
  {
    /*Not joined, try again later*/
    return;
  }

	uint32_t i = 0;

	AppData.Port = 100;	
	
	for(int j=0;j<downlinklens;j++)
	{
		AppData.Buff[i++] = downlink_send[j];
		downlink_send[j]=0x00;
	}	
		
	downlinklens=0;
  AppData.BuffSize = i;
	payloadlens=i;
	LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);	
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
					EEPROM_program(DATA_EEPROM_BASE,Automatic_fdr_network,1);	
					EEPROM_erase_lora_config();							
					atz_flags=1;						
					rxpr_flags=1;								
				}
			}
			break;
	  }
		
		case 0x05:
		{
			is_time_to_reply_downlink=1;
					
			if( AppData->BuffSize == 4 )   //---->AT+CFM
			{
				if(AppData->Buff[1]<2)
				{
					if(AppData->Buff[1]==0x01)
					{
						lora_config_reqack_set(LORAWAN_CONFIRMED_MSG);
						confirmed_uplink_retransmission_nbtrials=AppData->Buff[2];
		        confirmed_uplink_counter_retransmission_increment_switch=AppData->Buff[3];
					}
					else if(AppData->Buff[1]==0x00)
					{
						confirmed_uplink_retransmission_nbtrials=AppData->Buff[2];
		        confirmed_uplink_counter_retransmission_increment_switch=AppData->Buff[3];
						lora_config_reqack_set(LORAWAN_UNCONFIRMED_MSG);
					}
					EEPROM_Store_Config();
					rxpr_flags=1;	

					downlink_command_buffersize=AppData->BuffSize;
					for(int i=0;i<AppData->BuffSize;i++)
					{
						downlink_command_buffer[i]=AppData->Buff[i];
					}					
				}
			}
			else if(AppData->BuffSize == 2)
			{
				if(AppData->Buff[1]==0x01)
				{
					lora_config_reqack_set(LORAWAN_CONFIRMED_MSG);
				}
				else if(AppData->Buff[1]==0x00)
				{
					lora_config_reqack_set(LORAWAN_UNCONFIRMED_MSG);
				}
				confirmed_uplink_counter_retransmission_increment_switch=0;
				confirmed_uplink_retransmission_nbtrials=7;
				EEPROM_Store_Config();
				rxpr_flags=1;		
					
				downlink_command_buffersize=AppData->BuffSize;
				for(int i=0;i<AppData->BuffSize;i++)
				{
					downlink_command_buffer[i]=AppData->Buff[i];
				}
			}
			else
			{
				is_time_to_reply_downlink=0;
			}
					
			break;
	  }	
		
    case 0x06:
    {
			if( AppData->BuffSize == 4 )
			{
			  if((AppData->Buff[1]==0x00)&&(AppData->Buff[2]==0x00)&&(AppData->Buff[3]<=0x03))   		  //---->AT+INTMOD1
				{
					inmode=AppData->Buff[3];
					GPIO_EXTI14_IoInit(inmode);
					EEPROM_Store_Config();
					rxpr_flags=1;		
				}	
			  else if((AppData->Buff[1]==0x00)&&(AppData->Buff[2]==0x01)&&(AppData->Buff[3]<=0x03))   //---->AT+INTMOD2
				{
					inmode2=AppData->Buff[3];
					GPIO_EXTI15_IoInit(inmode2);
					EEPROM_Store_Config();
					rxpr_flags=1;						
				}		
			  else if((AppData->Buff[1]==0x00)&&(AppData->Buff[2]==0x02)&&(AppData->Buff[3]<=0x03))   //---->AT+INTMOD3
				{
					inmode3=AppData->Buff[3];
					GPIO_EXTI4_IoInit(inmode3);
					EEPROM_Store_Config();
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
				EEPROM_Store_Config();
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
					EEPROM_Store_Config();					
					rxpr_flags=1;							
				}
			}
			break;			
	  }		
		
		case 0x09:
		{
			if( AppData->BuffSize == 6 )             //---->AT+SETCNT
			{	
				if(AppData->Buff[1]==0x01)
				{				 
					COUNT=AppData->Buff[2]<<24 | AppData->Buff[3]<<16 | AppData->Buff[4]<<8 | AppData->Buff[5];
					rxpr_flags=1;								
				}
				else if(AppData->Buff[1]==0x02)
				{					 
					COUNT2=AppData->Buff[2]<<24 | AppData->Buff[3]<<16 | AppData->Buff[4]<<8 | AppData->Buff[5];
				  rxpr_flags=1;							
				}
			}				
			break;				
		}
		
		case 0x0A:
		{
			if( AppData->BuffSize == 2 )         
			{	
				if((AppData->Buff[1]>=0x01)&&(AppData->Buff[1]<=0x09))    //---->AT+MOD
				{
					mode=AppData->Buff[1];
					EEPROM_Store_Config();
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
					EEPROM_Store_Config();
					atz_flags=1;
					rxpr_flags=1;		
				}						 
			}
			break;				
		}	
		
		case 0x21:
		{
		  if( (AppData->BuffSize == 2) && (AppData->Buff[1]<=5) ) //---->AT+RPL
			{
				response_level=( AppData->Buff[1] );
				EEPROM_Store_Config();
				rxpr_flags=1;							
			}
			else if( (AppData->BuffSize == 3) && (AppData->Buff[1]==0x00) && (AppData->Buff[2]<=1))  //---->AT+DISMACANS
			{
				mac_response_flag=AppData->Buff[2];
				EEPROM_Store_Config();
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
				EEPROM_Store_Config();
				rxpr_flags=1;								
			}
			else if((AppData->BuffSize == 4 )&&(AppData->Buff[1]==0x00))   //---->AT+ADR=0
			{
				uint8_t downlink_data_rate=AppData->Buff[2];
				mib.Type = MIB_ADR;					
				mib.Param.AdrEnable = AppData->Buff[1];
				LoRaMacMibSetRequestConfirm( &mib );	
						
				#if defined(REGION_US915)
				if(downlink_data_rate>3)
				{
					downlink_data_rate=3;   
				}
				#elif defined(REGION_AS923) || defined(REGION_AU915)
				if(dwelltime==1)
				{
					if(downlink_data_rate>5)
					{
						downlink_data_rate=5;
					}
				  else if(downlink_data_rate<2)
					{
						downlink_data_rate=2;
					}
				}
				#else
				if(downlink_data_rate>5)
				{
					downlink_data_rate=5;
				}
				#endif	
						
				lora_config_tx_datarate_set(downlink_data_rate) ;
								
				if(AppData->Buff[3]!=0xff)                //---->AT+TXP
				{
					mib.Type = MIB_CHANNELS_TX_POWER;						
					mib.Param.ChannelsTxPower=AppData->Buff[3];
					LoRaMacMibSetRequestConfirm( &mib );							
				}				
				EEPROM_Store_Config();
				rxpr_flags=1;									
			 }
			 break;				
		}			
		
    case 0x23:			
		{
			if(( AppData->BuffSize == 2 )&&(AppData->Buff[1]!=0x00))
			{		
				lora_config_application_port_set(AppData->Buff[1]);    //---->AT+PORT
				EEPROM_Store_Config();
				rxpr_flags=1;							 
			}
			break;					
		}	
		
    case 0x24:			
	  {
			if( AppData->BuffSize == 2 )    //---->AT+CHE
			{
				#if defined( REGION_US915 )	|| defined( REGION_AU915 )
				if(AppData->Buff[1]<9)
				{
					customize_set8channel_set(AppData->Buff[1]);
					EEPROM_Store_Config();
					atz_flags=1;
				  rxpr_flags=1;
				}
				#elif defined( REGION_CN470 )
				if(AppData->Buff[1]<13)
				{
					customize_set8channel_set(AppData->Buff[1]);
					EEPROM_Store_Config();
					atz_flags=1;
					rxpr_flags=1;
				}
				#endif
			}
			break;		
	  }	
		
    case 0x25:			
		{
		 #if defined( REGION_AS923 )	|| defined( REGION_AU915 )  //---->AT+DWELLT
		 if( AppData->BuffSize == 2 )
		 {				
			 if((AppData->Buff[1]==0x00)||(AppData->Buff[1]==0x01))   
			 {
				 dwelltime=AppData->Buff[1];
			 	 EEPROM_Store_Config();	
				 atz_flags=1;
				 rxpr_flags=1;		
			 }						
		 }
		 #endif	
		 break;				
		}	
		
    case 0x26:
		{
			if(( AppData->BuffSize == 2 )&&(AppData->Buff[1]==0x01))  
			{
				uplink_message_data_status=1;		
				rxpr_flags=1;					
			}
		  else if( AppData->BuffSize == 3 )    //---->AT+RJTDC
			{
				uint16_t value;			
						
				value=( AppData->Buff[1]<<8 | AppData->Buff[2] );//1~65535
						
				if(value>0)
				{
					REJOIN_TX_DUTYCYCLE=value;
					EEPROM_Store_Config();
					rxpr_flags=1;
				}					
			}
			break;				
		}		
		
		case 0x32:
		{
		  if( AppData->BuffSize == 6 )    //---->AT+DDETECT
			{
       uint16_t value;						
		 	 value=AppData->Buff[1];
			 if(value<2)
			 {
				 downlink_detect_switch=value;
			 }
						
       value=AppData->Buff[2]<<8|AppData->Buff[3];
			 if(value>0)
			 {
				 unconfirmed_uplink_change_to_confirmed_uplink_timeout=value;
			 }
						
       value=AppData->Buff[4]<<8|AppData->Buff[5];
			 if(value>0)
			 {
			   downlink_detect_timeout=value;
			 }
						
			 if(downlink_detect_switch==0)
			 {
					TimerStop(&DownlinkDetectTimeoutTimer);
					TimerStop(&UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer);
			 }
			 else
			 {
					TimerSetValue(&DownlinkDetectTimeoutTimer,downlink_detect_timeout*60000);
					TimerStart(&DownlinkDetectTimeoutTimer);
							
					if(lora_config_reqack_get()==LORAWAN_UNCONFIRMED_MSG)
					{
						TimerSetValue(&UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer,unconfirmed_uplink_change_to_confirmed_uplink_timeout*60000); 
						TimerStart(&UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer);
					}	
				}
						
				EEPROM_Store_Config();	
				rxpr_flags=1;				
			}
			break;
		}
				
		case 0x33:
		{
			if( AppData->BuffSize == 3 )  //---->AT+SETMAXNBTRANS
			{
				LinkADR_NbTrans_retransmission_nbtrials=AppData->Buff[1];
				LinkADR_NbTrans_uplink_counter_retransmission_increment_switch=AppData->Buff[2];
						
				if(LinkADR_NbTrans_retransmission_nbtrials==0)
				{
					LinkADR_NbTrans_retransmission_nbtrials=1;
				}
						
				if(LinkADR_NbTrans_retransmission_nbtrials>15)
				{
					LinkADR_NbTrans_retransmission_nbtrials=15;
				}
						
				if(LinkADR_NbTrans_uplink_counter_retransmission_increment_switch>1)
				{
					LinkADR_NbTrans_uplink_counter_retransmission_increment_switch=1;
				}
				EEPROM_Store_Config();
				rxpr_flags=1;		
			}
			break;
		}
	
		default:
		break;
	}	
	
	if(TDC_flag==1)
	{
		EEPROM_Store_Config();
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
		AT_PRINTF("BuffSize:%d,Run AT+RECVB=? to see detail\r\n",AppData->BuffSize);				
	}
	
	if((response_level!=0)&&(response_level!=3))
  {
		if(rxpr_flags==1)
		{
			downlink_send[0]=0x01;
		}
		else 
		{
			downlink_send[0]=0x00;
		}
		downlinklens=1;
		for (uint8_t g = 0; g < AppData->BuffSize; g++)
		{
			if(downlinklens<51)
			{
				downlink_send[downlinklens++]=AppData->Buff[g];
			}
		}
		is_time_to_reply_downlink=0;
		if((AppData->BuffSize==2)&&(AppData->Buff[0]==0x26)&&(AppData->Buff[1]==0x01))
		{
			is_there_data=0;
			MAC_COMMAND_ANS_status=0;
		}
		else
		{
			downlink_data_status=1;
		}		
	}
	
	rxpr_flags=0;	
}

#if defined(LoRa_Sensor_Node)
static void OnTxTimerEvent( void )
{
	
	TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
	
  /*Wait for next tx slot*/
  TimerStart( &TxTimer);

  uplink_data_status=1;
		
	if((( LoRaMacState & 0x00000001 ) == 0x00000001)||(( LoRaMacState & 0x00000010 ) == 0x00000010))
	{
		LoRaMacState_error_times++;
	}
}

static void LoraStartTx(void)
{
  /* send everytime timer elapses */
  TimerInit( &TxTimer, OnTxTimerEvent );
  TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE); 
  OnTxTimerEvent();
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
		TimerStop( &TxTimer2);			
		if(downlink_detect_switch==1)
		{
			if(lora_config_reqack_get()==LORAWAN_UNCONFIRMED_MSG)
			{
				StartUnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer();
				HAL_Delay(5);
			}
			StartDownlinkDetect();
			HAL_Delay(5);
		}
		LoraStartTx();
		HAL_Delay(5);
		join_flag=0;
	}
}

static void LoraStartjoin(void)
{
  /* send everytime timer elapses */
  TimerInit( &TxTimer2, OnTxTimerEvent2 );
  TimerSetValue( &TxTimer2, 500); 
  OnTxTimerEvent2();
}

static void OnIWDGRefreshTimeoutEvent( void )
{
	TimerSetValue( &IWDGRefreshTimer,  18000);

  TimerStart( &IWDGRefreshTimer);

	is_time_to_IWDG_Refresh=1;
}

static void StartIWDGRefresh(void)
{
  /* send everytime timer elapses */
  TimerInit( &IWDGRefreshTimer, OnIWDGRefreshTimeoutEvent );
  TimerSetValue( &IWDGRefreshTimer,  18000); 
  TimerStart( &IWDGRefreshTimer);
}

static void OnReJoinTimerEvent( void )
{
	TimerStop( &ReJoinTimer);
	
	is_time_to_rejoin=1;
}

static void LoraStartRejoin(void)
{
  /* send everytime timer elapses */
  TimerInit( &ReJoinTimer, OnReJoinTimerEvent );
  TimerSetValue( &ReJoinTimer,  REJOIN_TX_DUTYCYCLE*60000); 
  TimerStart( &ReJoinTimer);
}

static void OnNetworkJoinedLedEvent(void)
{
	TimerStop(&NetworkJoinedLedTimer);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
	joined_led_end=0;
}

static void OnDownlinkDetectTimeoutEvent( void )
{
	if((lora_config_otaa_get() == LORA_ENABLE ? 1 : 0))
	{
		rejoin_status=1;
	}
	
  /*Wait for next tx slot*/
  TimerStop( &DownlinkDetectTimeoutTimer);
}

static void StartDownlinkDetect(void)
{
  /* send everytime timer elapses */
  TimerInit( &DownlinkDetectTimeoutTimer, OnDownlinkDetectTimeoutEvent );
  TimerSetValue( &DownlinkDetectTimeoutTimer,  downlink_detect_timeout*60000); 
  TimerStart( &DownlinkDetectTimeoutTimer);
}

static void UnconfirmedUplinkChangeToConfirmedUplinkTimeoutEvent( void )
{
	unconfirmed_uplink_change_to_confirmed_uplink_status=1;
	
  /*Wait for next tx slot*/
  TimerStop( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer);
}

static void StartUnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer(void)
{
  /* send everytime timer elapses */
  TimerInit( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer, UnconfirmedUplinkChangeToConfirmedUplinkTimeoutEvent );
  TimerSetValue( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer,  unconfirmed_uplink_change_to_confirmed_uplink_timeout*60000); 
  TimerStart( &UnconfirmedUplinkChangeToConfirmedUplinkTimeoutTimer);
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
		 if((mode!=6)&&(mode!=9))
		 {
			 if((( LoRaMacState & 0x00000001 ) != 0x00000001) &&(( LoRaMacState & 0x00000010 ) != 0x00000010))
			 {	
			  uplink_data_status=1;
			 }
		 }
		 else
		 {
			if(debug_flags==1)
			{
				PPRINTF("COUNT1 is %u\r\n",COUNT);	
			}	 
			exti_flag=0;			 
		 }
	}
	
	if(exti_flag2==1)
	{
		if(mode!=9)
		{
			if((( LoRaMacState & 0x00000001 ) != 0x00000001) && (( LoRaMacState & 0x00000010 ) != 0x00000010))
			{
				uplink_data_status=1;
			}		
		}
		else
		{
			if(debug_flags==1)
			{
				PPRINTF("COUNT2 is %u\r\n",COUNT2);	
			}	 
			exti_flag2=0;				
		}
	}
	
	
	if(exti_flag3==1)
	{
		if((( LoRaMacState & 0x00000001 ) != 0x00000001) && (( LoRaMacState & 0x00000010 ) != 0x00000010))
	  {
		  uplink_data_status=1;
		}		
	}
}

static void Flash_copy_key_to_EEPROM(void)
{                      
	if((*(__IO uint32_t *)DATA_EEPROM_BASE==0x00)   //baseflag
	   &&(((*(__IO uint32_t *)0x08080004==0x00))&&((*(__IO uint32_t *)0x08080008==0x00)))  //DEUI
	   &&((*(__IO uint32_t *)0x0808000C==0x00)))  //DADDR
	{	  
	  Flash_Read_key();
	  EEPROM_Store_key();
		HAL_Delay(200);
	}
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
