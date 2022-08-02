/*******************************************************************************
 * @file    at.c
 * @author  MCD Application Team
 * @version V1.1.4
 * @date    08-January-2018
 * @brief   at command API
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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "at.h"
#include "utilities.h"
#include "lora.h"
#include "LoRaMacTest.h"
#include "radio.h"
#include "vcom.h"
#include "tiny_sscanf.h"
#include "version.h"
#include "hw_msp.h"
#include "flash_eraseprogram.h"
#include "command.h"
#include "timeServer.h"
#include "delay.h"
#include "gpio_exti.h"
#include "weight.h"
#include "bsp.h"

bool debug_flags=0;
bool message_flags=0;
uint8_t symbtime1_value=0;  //RX1windowtimeout 
uint8_t flag1=0;
uint8_t symbtime2_value=0;  //RX2windowtimeout 
uint8_t flag2=0;
uint16_t power_time=0;
uint8_t dwelltime;
/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/**
 * @brief Max size of the data that can be received
 */
#define MAX_RECEIVED_DATA 255
extern uint8_t RX2DR_setting_status;
extern bool down_check;
extern bool mac_response_flag;
extern uint32_t LoRaMacState;
extern uint32_t APP_TX_DUTYCYCLE;
extern uint8_t mode;
extern uint8_t inmode,inmode2,inmode3;
extern float GapValue;
extern void Get_Maopi(void);
extern bool fdr_flags;
extern uint16_t REJOIN_TX_DUTYCYCLE;
extern uint8_t response_level;
extern uint8_t decrypt_flag;
extern uint32_t COUNT,COUNT2;
extern bool uplink_data_status;
extern TimerEvent_t MacStateCheckTimer;
extern TimerEvent_t TxDelayedTimer;
extern TimerEvent_t AckTimeoutTimer;
extern TimerEvent_t RxWindowTimer1;
extern TimerEvent_t RxWindowTimer2;
extern TimerEvent_t TxTimer;
extern TimerEvent_t ReJoinTimer;
extern LoRaMainCallback_t *LoRaMainCallbacks;

extern uint8_t downlink_detect_switch;
extern uint16_t downlink_detect_timeout;

extern uint8_t confirmed_uplink_counter_retransmission_increment_switch;
extern uint8_t confirmed_uplink_retransmission_nbtrials;

extern uint8_t LinkADR_NbTrans_uplink_counter_retransmission_increment_switch;
extern uint8_t LinkADR_NbTrans_retransmission_nbtrials;
extern uint16_t unconfirmed_uplink_change_to_confirmed_uplink_timeout;
/* Private macro -------------------------------------------------------------*/
/**
 * @brief Macro to return when an error occurs
 */
#define CHECK_STATUS(status) do {                    \
    ATEerror_t at_status = translate_status(status); \
    if (at_status != AT_OK) { return at_status; }    \
  } while (0)

/* Private variables ---------------------------------------------------------*/
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFF_SIZE                           64

/*!
 * User application data
 */
static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];

/*!
 * User application data structure
 */
static lora_AppData_t AppData={ AppDataBuff,  0 ,0 };
/**
 * @brief Buffer that contains the last received data
 */
static char ReceivedData[MAX_RECEIVED_DATA];

/**
 * @brief Size if the buffer that contains the last received data
 */
static unsigned ReceivedDataSize = 0;

/**
 * @brief Application port the last received data were on
 */
static uint8_t ReceivedDataPort;

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief  Translate a LoRaMacStatus_t into an ATEerror_t
 * @param  the LoRaMacStatus_t status
 * @retval the corresponding ATEerror_t code
 */
static ATEerror_t translate_status(LoRaMacStatus_t status);

/**
 * @brief  Get 16 bytes values in hexa
 * @param  The string containing the 16 bytes, something like ab:cd:01:...
 * @param  The buffer that will contain the bytes read
 * @retval The number of bytes read
 */
static int sscanf_16_hhx(const char *from, uint8_t *pt,uint8_t num);

/**
 * @brief  Print 16 bytes as %02x
 * @param  the pointer containing the 16 bytes to print
 * @retval None
 */
static void print_16_02x(uint8_t *pt);

/**
 * @brief  Get 4 bytes values in hexa
 * @param  The string containing the 16 bytes, something like ab:cd:01:21
 * @param  The buffer that will contain the bytes read
 * @retval The number of bytes read
 */
static int sscanf_uint32_as_hhx(const char *from, uint32_t *value,uint8_t num);

/**
 * @brief  Print 4 bytes as %02x
 * @param  the value containing the 4 bytes to print
 * @retval None
 */
static void print_uint32_as_02x(uint32_t value);

static int sscanf_8_hhx(const char *from, uint8_t *pt,uint8_t num);
/**
 * @brief  Print 8 bytes as %02x
 * @param  the pointer containing the 8 bytes to print
 * @retval None
 */
static void print_8_02x(uint8_t *pt);

/**
 * @brief  Print an int
 * @param  the value to print
 * @retval None
 */
static void print_d(int value);

/**
 * @brief  Print an unsigned int
 * @param  the value to print
 * @retval None
 */
static void print_u(unsigned int value);

static uint8_t changeform(const char *from,uint8_t *pt,uint8_t number);
/* Exported functions ------------------------------------------------------- */

void set_at_receive(uint8_t AppPort, uint8_t* Buff, uint8_t BuffSize)
{
  if (MAX_RECEIVED_DATA <= BuffSize)
    BuffSize = MAX_RECEIVED_DATA;
  memcpy1((uint8_t *)ReceivedData, Buff, BuffSize);
  ReceivedDataSize = BuffSize;
  ReceivedDataPort = AppPort;
}

ATEerror_t at_return_ok(const char *param)
{
  return AT_OK;
}

ATEerror_t at_return_error(const char *param)
{
  return AT_ERROR;
}

ATEerror_t at_reset(const char *param)
{
  NVIC_SystemReset();
  return AT_OK;
}

ATEerror_t at_DEBUG_run(const char *param)
{
  debug_flags=1;
	PPRINTF("Enter Debug mode\r\n");			
	debug_flags=1;
	
  return AT_OK;	
}

ATEerror_t at_FDR(const char *param)
{
	EEPROM_erase_one_address(DATA_EEPROM_BASE);
	EEPROM_erase_lora_config();
	AT_PRINTF("OK\n\r");
	HAL_Delay(50);
	NVIC_SystemReset();
  return AT_OK;
}

ATEerror_t at_DevEUI_get(const char *param)
{
  print_8_02x(lora_config_deveui_get());
  return AT_OK;
}

ATEerror_t at_DevEUI_set(const char *param)
{
  uint8_t DevEUI[8];
  if (sscanf_8_hhx(param, DevEUI,2) != 8)
  {
		if (sscanf_8_hhx(param, DevEUI,1) != 8)
		{

		}
  }
  
  lora_config_deveui_set(DevEUI);
  return AT_OK;
}

ATEerror_t at_AppEUI_get(const char *param)
{
  print_8_02x(lora_config_appeui_get());
  return AT_OK;
}

ATEerror_t at_AppEUI_set(const char *param)
{
  uint8_t AppEui[8];
  if (sscanf_8_hhx(param, AppEui,2) != 8)
  {
		if (sscanf_8_hhx(param, AppEui,1) != 8)
		{		

		}
  }
  
  lora_config_appeui_set(AppEui);
  return AT_OK;
}

ATEerror_t at_DevAddr_set(const char *param)
{
  uint32_t DevAddr;
  if (sscanf_uint32_as_hhx(param, &DevAddr,2) != 4)
  {
		if (sscanf_uint32_as_hhx(param, &DevAddr,1) != 4)
		{		

		}
  }
  lora_config_devaddr_set(DevAddr);
  return AT_OK;
}

ATEerror_t at_DevAddr_get(const char *param)
{
  lora_config_devaddr_get();
  return AT_OK;
}

ATEerror_t at_AppKey_get(const char *param)
{
  print_16_02x(lora_config_appkey_get());
  return AT_OK;
}

ATEerror_t at_AppKey_set(const char *param)
{
  uint8_t AppKey[16];
  if (sscanf_16_hhx(param, AppKey,2) != 16)
  {
		if (sscanf_16_hhx(param, AppKey,1) != 16)
		{		

		}
  }
  
  lora_config_appkey_set(AppKey);
  return AT_OK;
}

ATEerror_t at_NwkSKey_get(const char *param)
{
	print_16_02x(lora_config_nwkskey_get());
  return AT_OK;
}

ATEerror_t at_NwkSKey_set(const char *param)
{
	uint8_t NwkSKey[16];
  if (sscanf_16_hhx(param, NwkSKey,2) != 16)
  {
		if (sscanf_16_hhx(param, NwkSKey,1) != 16)
		{		

		}
  }
  
  lora_config_nwkskey_set(NwkSKey);
  return AT_OK;
}

ATEerror_t at_AppSKey_get(const char *param)
{
	print_16_02x(lora_config_appskey_get());
  return AT_OK;
}

ATEerror_t at_AppSKey_set(const char *param)
{
	uint8_t AppSKey[16];
  if (sscanf_16_hhx(param, AppSKey,2) != 16)
  {
		if (sscanf_16_hhx(param, AppSKey,1) != 16)
		{		

		}
  }
  
  lora_config_appskey_set(AppSKey);
  return AT_OK;
}

ATEerror_t at_ADR_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_ADR;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_d(mib.Param.AdrEnable);

  return AT_OK;
}

ATEerror_t at_ADR_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_ADR;

  switch (param[0])
  {
    case '0':
    case '1':
      mib.Param.AdrEnable = param[0] - '0';
      status = LoRaMacMibSetRequestConfirm(&mib);
      CHECK_STATUS(status);
      break;
    default:
      return AT_PARAM_ERROR;
  }

  return AT_OK;
}

ATEerror_t at_TransmitPower_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_CHANNELS_TX_POWER;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_d(mib.Param.ChannelsTxPower);

  return AT_OK;
}

ATEerror_t at_TransmitPower_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_CHANNELS_TX_POWER;
  if (tiny_sscanf(param, "%hhu", &mib.Param.ChannelsTxPower) != 1)
  {
    return AT_PARAM_ERROR;
  }
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

	PPRINTF("Attention:Take effect after AT+ADR=0\r\n");
  return AT_OK;
}

ATEerror_t at_DataRate_get(const char *param)
{

  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_CHANNELS_DATARATE;

  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_d(mib.Param.ChannelsDatarate);

  return AT_OK;
}

ATEerror_t at_DataRate_set(const char *param)
{
  int8_t datarate;
	
  if (tiny_sscanf(param, "%hhu", &datarate) != 1)
  {
    return AT_PARAM_ERROR;
  }

#if defined( REGION_AS923 )
	  if(datarate>=8)
		{
    return AT_PARAM_NOT_Range;			
		}
#elif defined( REGION_AU915 )
	  if((datarate==7)||(datarate>=14))
		{
    return AT_PARAM_NOT_Range;			
		}
#elif defined( REGION_CN470 )
	  if(datarate>=6)	
		{
    return AT_PARAM_NOT_Range;			
		}
#elif defined( REGION_CN779 )
	  if(datarate>=8)
		{
    return AT_PARAM_NOT_Range;			
		}
#elif defined( REGION_EU433 )
	  if(datarate>=8)
		{
    return AT_PARAM_NOT_Range;			
		}
#elif defined( REGION_IN865 )
	  if(datarate>=8)
		{
    return AT_PARAM_NOT_Range;			
		}
#elif defined( REGION_EU868 )
	  if(datarate>=8)
		{
    return AT_PARAM_NOT_Range;			
		}
#elif defined( REGION_KR920 )
	  if(datarate>=6)
		{
    return AT_PARAM_NOT_Range;			
		}
#elif defined( REGION_US915 )
	  if(((datarate>=5)&&(datarate<=7))||(datarate>=14))
		{
    return AT_PARAM_NOT_Range;			
		}
#elif defined( REGION_RU864 )
	  if(datarate>=8)
		{
    return AT_PARAM_NOT_Range;			
		}
#elif defined( REGION_KZ865 )
	  if(datarate>=8)
		{
    return AT_PARAM_NOT_Range;			
		}	
#endif
	
  lora_config_tx_datarate_set(datarate) ;
	
	PPRINTF("Attention:Take effect after AT+ADR=0\r\n");
  return AT_OK;
}

ATEerror_t at_DutyCycle_set(const char *param)
{
  switch (param[0])
  {
    case '0':
      lora_config_duty_cycle_set(LORA_DISABLE);
      break;
    case '1':
      lora_config_duty_cycle_set(LORA_ENABLE);
      break;
    default:
      return AT_PARAM_ERROR;
  }

  return AT_OK;
}

ATEerror_t at_DutyCycle_get(const char *param)
{
  if (lora_config_duty_cycle_get() == LORA_ENABLE)
    AT_PRINTF("1\r\n");
  else
    AT_PRINTF("0\r\n");

  return AT_OK;
}


ATEerror_t at_PublicNetwork_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_PUBLIC_NETWORK;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_d(mib.Param.EnablePublicNetwork);
	
  return AT_OK;
}

ATEerror_t at_PublicNetwork_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_PUBLIC_NETWORK;
  switch (param[0])
  {
    case '0':
    case '1':
      mib.Param.EnablePublicNetwork = param[0] - '0';
      status = LoRaMacMibSetRequestConfirm(&mib);
      CHECK_STATUS(status);
      break;
    default:
      return AT_PARAM_ERROR;
  }

  return AT_OK;
}

ATEerror_t at_Rx2Frequency_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_RX2_CHANNEL;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_d(mib.Param.Rx2Channel.Frequency);

  return AT_OK;
}

ATEerror_t at_Rx2Frequency_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_RX2_CHANNEL;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);

  if (tiny_sscanf(param, "%lu", &mib.Param.Rx2Channel.Frequency) != 1)
  {
    return AT_PARAM_ERROR;
  }

  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  return AT_OK;
}

ATEerror_t at_Rx2DataRate_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_RX2_CHANNEL;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_d(mib.Param.Rx2Channel.Datarate);

  return AT_OK;
}

ATEerror_t at_Rx2DataRate_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_RX2_CHANNEL;

  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);

  if (tiny_sscanf(param, "%hhu", &mib.Param.Rx2Channel.Datarate) != 1)
  {
    return AT_PARAM_ERROR;
  }

	RX2DR_setting_status=1;
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  return AT_OK;
}

ATEerror_t at_Rx1Delay_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_RECEIVE_DELAY_1;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_u(mib.Param.ReceiveDelay1);

  return AT_OK;
}

ATEerror_t at_Rx1Delay_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_RECEIVE_DELAY_1;
  if (tiny_sscanf(param, "%lu", &mib.Param.ReceiveDelay1) != 1)
  {
    return AT_PARAM_ERROR;
  }
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  return AT_OK;
}

ATEerror_t at_Rx2Delay_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_RECEIVE_DELAY_2;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_u(mib.Param.ReceiveDelay2);

  return AT_OK;
}

ATEerror_t at_Rx2Delay_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_RECEIVE_DELAY_2;
  if (tiny_sscanf(param, "%lu", &mib.Param.ReceiveDelay2) != 1)
  {
    return AT_PARAM_ERROR;
  }
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  return AT_OK;
}

ATEerror_t at_JoinAcceptDelay1_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_JOIN_ACCEPT_DELAY_1;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_u(mib.Param.JoinAcceptDelay1);

  return AT_OK;
}

ATEerror_t at_JoinAcceptDelay1_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_JOIN_ACCEPT_DELAY_1;
  if (tiny_sscanf(param, "%lu", &mib.Param.JoinAcceptDelay1) != 1)
  {
    return AT_PARAM_ERROR;
  }
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

	PPRINTF("Attention:Take effect after ATZ\r\n");	
  return AT_OK;
}

ATEerror_t at_JoinAcceptDelay2_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_JOIN_ACCEPT_DELAY_2;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_u(mib.Param.JoinAcceptDelay2);

  return AT_OK;
}

ATEerror_t at_JoinAcceptDelay2_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_JOIN_ACCEPT_DELAY_2;
  if (tiny_sscanf(param, "%lu", &mib.Param.JoinAcceptDelay2) != 1)
  {
    return AT_PARAM_ERROR;
  }
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

	PPRINTF("Attention:Take effect after ATZ\r\n");	
  return AT_OK;
}

ATEerror_t at_NetworkJoinMode_get(const char *param)
{
  print_d((lora_config_otaa_get() == LORA_ENABLE ? 1 : 0));
  return AT_OK;
}

ATEerror_t at_NetworkJoinMode_set(const char *param)
{
  LoraState_t status;

  switch (param[0])
  {
    case '0':
      status = LORA_DISABLE;
      break;
    case '1':
      status = LORA_ENABLE;
      break;
    default:
      return AT_PARAM_ERROR;
  }

  lora_config_otaa_set(status);

	PPRINTF("Attention:Take effect after ATZ\r\n");	
  return AT_OK;
}

ATEerror_t at_NetworkID_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_NET_ID;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_uint32_as_02x(mib.Param.NetID);

  return AT_OK;
}

ATEerror_t at_NetworkID_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_NET_ID;
  if (sscanf_uint32_as_hhx(param, &mib.Param.NetID,2) != 4)
  {
		if (sscanf_uint32_as_hhx(param, &mib.Param.NetID,1) != 4)
		{		

		}
  }
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  return AT_OK;
}

ATEerror_t at_UplinkCounter_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_UPLINK_COUNTER;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_u(mib.Param.UpLinkCounter);

  return AT_OK;
}

ATEerror_t at_UplinkCounter_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_UPLINK_COUNTER;
  if (tiny_sscanf(param, "%lu", &mib.Param.UpLinkCounter) != 1)
  {
    return AT_PARAM_ERROR;
  }
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  return AT_OK;
}

ATEerror_t at_DownlinkCounter_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_DOWNLINK_COUNTER;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  print_u(mib.Param.DownLinkCounter);

  return AT_OK;
}

ATEerror_t at_DownlinkCounter_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_DOWNLINK_COUNTER;
  if (tiny_sscanf(param, "%lu", &mib.Param.DownLinkCounter) != 1)
  {
    return AT_PARAM_ERROR;
  }
  status = LoRaMacMibSetRequestConfirm(&mib);
  CHECK_STATUS(status);

  return AT_OK;
}

ATEerror_t at_DeviceClass_get(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_DEVICE_CLASS;
  status = LoRaMacMibGetRequestConfirm(&mib);
  CHECK_STATUS(status);
  AT_PRINTF("%c\r\n", 'A' + mib.Param.Class);

  return AT_OK;
}

ATEerror_t at_DeviceClass_set(const char *param)
{
  MibRequestConfirm_t mib;
  LoRaMacStatus_t status;

  mib.Type = MIB_DEVICE_CLASS;
  switch (param[0])
  {
    case 'A':
    case 'B':
    case 'C':
      /* assume CLASS_A == 0, CLASS_B == 1, etc, which is the case for now */
      mib.Param.Class = (DeviceClass_t)(param[0] - 'A');
      status = LoRaMacMibSetRequestConfirm(&mib);
      CHECK_STATUS(status);
      break;
    default:
      return AT_PARAM_ERROR;
  }

	PPRINTF("Attention:Take effect after ATZ\r\n");
  return AT_OK;
}

ATEerror_t at_Join(const char *param)
{
  LORA_Join();

  return AT_OK;
}

ATEerror_t at_NetworkJoinStatus(const char *param)
{
  MibRequestConfirm_t mibReq;
  LoRaMacStatus_t status;

  mibReq.Type = MIB_NETWORK_JOINED;
  status = LoRaMacMibGetRequestConfirm(&mibReq);

  if (status == LORAMAC_STATUS_OK)
  {
    print_d(mibReq.Param.IsNetworkJoined ? 1 : 0);
    return AT_OK;
  }
  return AT_ERROR;
}

ATEerror_t at_SendBinary(const char *param)
{
  LoraErrorStatus status;
  const char *buf= param;
  unsigned char bufSize= strlen(param);
  uint32_t appPort;
  unsigned size=0;
  char hex[3];
  
    /* read and set the application port */
  if (1 != tiny_sscanf(buf, "%u:", &appPort))
  {
    PRINTF("AT+SEND without the application port");
    return AT_PARAM_ERROR;
  }
  
  /* skip the application port */
  while (('0' <= buf[0]) && (buf[0] <= '9'))
  {
    buf ++;
    bufSize --;
  };
  
  if (buf[0] != ':')
  {
    PRINTF("AT+SEND without the application port");
    return AT_PARAM_ERROR;
  }
  else
  {
    /*ok skip the char ':' */
    buf ++;
    bufSize --;
  }

  hex[2] = 0;
  while ((size < LORAWAN_APP_DATA_BUFF_SIZE) && (bufSize > 1))
  {
    hex[0] = buf[size*2];
    hex[1] = buf[size*2+1];
    if (tiny_sscanf(hex, "%hhx", &AppData.Buff[size]) != 1)
    {
      return AT_PARAM_ERROR;
    }
    size++;
    bufSize -= 2;
  }
  if (bufSize != 0)
  {
    return AT_PARAM_ERROR;
  }
  
  AppData.BuffSize = size;
  AppData.Port= appPort;

  status = LORA_send( &AppData, lora_config_reqack_get() );
  
  if (status == LORA_SUCCESS)
  {
    return AT_OK;
  }
  else
  {
    return AT_ERROR;
  }
}

ATEerror_t at_Send(const char *param)
{
  LoraErrorStatus status;
  const char *buf= param;
  unsigned char bufSize= strlen(param);
  uint32_t appPort;
  
    /* read and set the application port */
  if (1 != tiny_sscanf(buf, "%u:", &appPort))
  {
    PRINTF("AT+SEND without the application port");
    return AT_PARAM_ERROR;
  }
  
  /* skip the application port */
  while (('0' <= buf[0]) && (buf[0] <= '9'))
  {
    buf ++;
    bufSize --;
  };
  
  if (buf[0] != ':')
  {
    PRINTF("AT+SEND without the application port");
    return AT_PARAM_ERROR;
  }
  else
  {
    /*ok skip the char ':' */
    buf ++;
    bufSize --;
  }
  
  if (bufSize > LORAWAN_APP_DATA_BUFF_SIZE)
  {
    bufSize = LORAWAN_APP_DATA_BUFF_SIZE;
  }
  memcpy1(AppData.Buff, (uint8_t *)buf, bufSize);
  AppData.BuffSize = bufSize;
  AppData.Port= appPort;
  
  status = LORA_send( &AppData, lora_config_reqack_get() );
  
  if (status == LORA_SUCCESS)
  {
    return AT_OK;
  }
  else
  {
    return AT_ERROR;
  }
}

ATEerror_t at_ReceiveBinary(const char *param)
{
  unsigned i;
  
  AT_PRINTF("%d:", ReceivedDataPort);
  for (i = 0; i < ReceivedDataSize; i++)
  {
    AT_PRINTF("%02x ", ReceivedData[i]);
  }
  AT_PRINTF("\r\n");
  ReceivedDataSize = 0;

  return AT_OK;
}

ATEerror_t at_Receive(const char *param)
{
  AT_PRINTF("%d:", ReceivedDataPort);
  if (ReceivedDataSize)
  {
    AT_PRINTF("%s", ReceivedData);
    ReceivedDataSize = 0;
  }
  AT_PRINTF("\r\n");

  return AT_OK;
}

ATEerror_t at_DwellTime_set(const char *param)
{
#if defined( REGION_AS923 )	|| defined( REGION_AU915 )
	uint8_t dwelltime_temp;
	if (tiny_sscanf(param, "%d", &dwelltime_temp) != 1)	
	{
    return AT_PARAM_ERROR;		
	}
	
	if((dwelltime_temp==1)||(dwelltime_temp==0))
	{
		dwelltime=dwelltime_temp;
	}
	else
	{
    return AT_PARAM_ERROR;			
	}

	PPRINTF("Attention:Take effect after ATZ\r\n");
	
	return AT_OK;
#else
    PPRINTF("This Channel Plan is not support\r\n");
		return AT_OK;	
#endif	
}

ATEerror_t at_DwellTime_get(const char *param)
{
  print_d(dwelltime);	
  return AT_OK;	
}

ATEerror_t at_RJTDC_set(const char *param)
{ 
	uint16_t time=0;
	if (tiny_sscanf(param, "%lu", &time) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	if(time>0 && time<=65535)
	{
		REJOIN_TX_DUTYCYCLE=time;
	}
	else
	{
		PRINTF("The RJTDC range is 1 to 65535\n\r");
		return AT_PARAM_ERROR;	
	}
	
	return AT_OK;
}

ATEerror_t at_RJTDC_get(const char *param)
{ 
	print_d(REJOIN_TX_DUTYCYCLE);
	return AT_OK;
}

ATEerror_t at_RPL_set(const char *param)
{ 
	uint8_t time=0;
	if (tiny_sscanf(param, "%d", &time) != 1)
	{
		return AT_PARAM_ERROR;
	}
	
	/*
	 0 : null
	 1 : unconfirm downlink data
	 2 : confirm downlink data
	 3 : MAC command
	 4 : MAC command or confirm downlink data
	 5 : Downlink data
	 */
	if(time<=5)
	{
		response_level=time;
	}
	else
	{
		PRINTF("The response level range is 0 to 5\n\r");
		return AT_PARAM_ERROR;	
	}

	return AT_OK;
}

ATEerror_t at_RPL_get(const char *param)
{ 
	print_d(response_level);
	
	return AT_OK;
}

ATEerror_t at_version_get(const char *param)
{
  AT_PRINTF(AT_VERSION_STRING" ");
	region_printf();
	
  return AT_OK;
}

ATEerror_t at_ack_set(const char *param)
{
	uint8_t mode,trials,increment_switch;
	if(tiny_sscanf(param, "%d,%d,%d", &mode,&trials,&increment_switch) != 3)
	{
		if(tiny_sscanf(param, "%d", &mode) != 1)
		{
			return AT_PARAM_ERROR;
		}
		else
		{
			trials=7;
			increment_switch=0;
		}
	}
	
	if(mode>1 || trials>7 || increment_switch>1)
	{
		return AT_PARAM_ERROR;
	}
	
	if(mode==0)
	{
		lora_config_reqack_set(LORAWAN_UNCONFIRMED_MSG);
		confirmed_uplink_retransmission_nbtrials=trials;
		confirmed_uplink_counter_retransmission_increment_switch=0;
	}
	else if(mode==1)
	{
		lora_config_reqack_set(LORAWAN_CONFIRMED_MSG);
		confirmed_uplink_retransmission_nbtrials=trials;
		confirmed_uplink_counter_retransmission_increment_switch=increment_switch;
	}

  return AT_OK;
}

ATEerror_t at_ack_get(const char *param)
{
  AT_PRINTF("%d,%d,%d\r\n", lora_config_reqack_get(),confirmed_uplink_retransmission_nbtrials,confirmed_uplink_counter_retransmission_increment_switch);
  return AT_OK;
}

ATEerror_t at_isack_get(const char *param)
{
  print_d(((lora_config_isack_get() == LORA_ENABLE) ? 1 : 0));
  return AT_OK;
}

ATEerror_t at_snr_get(const char *param)
{
  print_u(lora_config_snr_get());
  return AT_OK;
}

ATEerror_t at_rssi_get(const char *param)
{
  print_d(lora_config_rssi_get());
  return AT_OK;
}

ATEerror_t at_CFG_run(const char *param)
{
	MibRequestConfirm_t mibReq;
  LoRaMacStatus_t mac_status;
	
	mibReq.Type = MIB_NETWORK_JOINED;
  mac_status = LoRaMacMibGetRequestConfirm(&mibReq);

	if(mibReq.Param.IsNetworkJoined == 1)
	{	
		if(( LoRaMacState & 0x00000001 ) == 0x00000001)
		{
			return AT_BUSY_ERROR;
		}	
	}
	
	PPRINTF("\n\rStop Tx events,Please wait for all configurations to print\r\n");	
	PPRINTF("\r\n");
	TimerStop(&MacStateCheckTimer);
	TimerStop(&TxDelayedTimer);
	TimerStop(&AckTimeoutTimer);

	TimerStop(&RxWindowTimer1);
	TimerStop(&RxWindowTimer2);
	TimerStop(&TxTimer);	
	
	printf_all_config();
	HAL_Delay(500);	
	
	if (mac_status == LORAMAC_STATUS_OK)
	{
		if(mibReq.Param.IsNetworkJoined == 1)
		{
				PPRINTF("\n\rStart Tx events\r\n");
				TimerStart(&TxTimer);
		}
		else
		{
		  if(fdr_flags==0)
	    {			
				 PPRINTF("\n\rStart Tx events\r\n");				
				 TimerStart(&TxDelayedTimer);				 
			}
		}
	}			
		
	 return AT_OK;	
}

ATEerror_t at_TDC_set(const char *param)
{ 
	uint32_t txtimeout;
	
	if (tiny_sscanf(param, "%lu", &txtimeout) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	if(txtimeout<6000)
	{
		PRINTF("TDC setting must be more than 6S\n\r");
		APP_TX_DUTYCYCLE=6000;
		return AT_PARAM_ERROR;
	}
	
	APP_TX_DUTYCYCLE=txtimeout;
	
	return AT_OK;
}

ATEerror_t at_TDC_get(const char *param)
{ 
	print_u(APP_TX_DUTYCYCLE);
	return AT_OK;
}

ATEerror_t at_application_port_set(const char *param)
{
	 int8_t application_port;

  if (tiny_sscanf(param, "%hhu", &application_port) != 1)
  {
    return AT_PARAM_ERROR;
  }
  
  lora_config_application_port_set(application_port) ;

  return AT_OK;
}

ATEerror_t at_application_port_get(const char *param)
{
	print_d(lora_config_application_port_get());
	return AT_OK;
}

ATEerror_t at_CHE_set(const char *param)
{
	uint8_t fre;
	if (tiny_sscanf(param, "%d", &fre) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	#if defined ( REGION_CN470 )
	if((fre>12)||((fre>=1)&&(fre<=10)))
	{
		fre=1;
	  PPRINTF("Error Subband, must be 0 or 11,12\r\n");	
	}
	else
	{
  	PPRINTF("Attention:Take effect after ATZ\r\n");	
	}
	#elif defined ( REGION_US915 )
	if(fre>8)
	{
		fre=1;
	  PPRINTF("Error Subband, must be 0 ~ 8\r\n");
	}
	else
	{
  	PPRINTF("Attention:Take effect after ATZ\r\n");	
	}	
	#elif defined ( REGION_AU915 )
	if(fre>8)
	{
		fre=1;
	  PPRINTF("Error Subband, must be 0 ~ 8\r\n");		
	}
	else
	{
  	PPRINTF("Attention:Take effect after ATZ\r\n");	
	}	
	#else
	fre=0;
	#endif
	
	customize_set8channel_set(fre);

	return AT_OK;
}

ATEerror_t at_CHE_get(const char *param)
{ 
  print_d(customize_set8channel_get());
	uint8_t i;
	double fre1;
	double j,k,l;
	
	i=customize_set8channel_get();
	
	#if defined ( REGION_CN470 )
	  j=470.3;k=1.6;l=0.2;
	#elif defined ( REGION_US915 )
	if(i==9)
	{
    j=902.2;k=0.1;l=1.6;
	}
	else
	{j=902.3;k=1.6;l=0.2;}
	#else
	if(i==10)
	{
    j=915.0;k=0.1;l=1.6;
	}
	else
	{j=915.2;k=1.6;l=0.2;}
	#endif
	
	if(i)
	{
	  fre1=j+(i-1)*k;
	  for(int i=0;i<8;i++)
	  {		
		  AT_PRINTF("%.1f ",fre1);
		  fre1=fre1+l;
	  }
	  AT_PRINTF("\n\r");
  }
	 else AT_PRINTF("Use default channel\r\n");
	
	return AT_OK;
}

ATEerror_t at_CHS_set(const char *param)
{
	uint32_t fre;
	if (tiny_sscanf(param, "%lu", &fre) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	if((100000000<fre&&fre<999999999)||fre==0)
	{
	  customize_freq1_set(fre);
	}
	else 
	{
		return AT_PARAM_ERROR;
	}
	
	return AT_OK;
}

ATEerror_t at_CHS_get(const char *param)
{ 
	print_u(customize_freq1_get());
	return AT_OK;
}

ATEerror_t at_symbtimeout1LSB_get(const char *param)
{ 
   print_d(symbtime1_value);
	 return AT_OK;
}

ATEerror_t at_symbtimeout1LSB_set(const char *param)
{ 
	int symbtime1;
	if (tiny_sscanf(param, "%d", &symbtime1) != 1)
  {
    return AT_PARAM_ERROR;
  }
	if ((symbtime1>=0)&&(symbtime1<=255))
  { 
		 flag1=1;
		 symbtime1_value=symbtime1;
  }
	else
	{
    return AT_PARAM_ERROR;
	}
	
	return AT_OK;
}

ATEerror_t at_symbtimeout2LSB_get(const char *param)
{ 
   print_d(symbtime2_value);
	 return AT_OK;
}

ATEerror_t at_symbtimeout2LSB_set(const char *param)
{ 
	int symbtime2;
	if (tiny_sscanf(param, "%d", &symbtime2) != 1)
  {
    return AT_PARAM_ERROR;
  }
	if ((symbtime2>=0)&&(symbtime2<=255))
  { 
		 flag2=1;
		 symbtime2_value=symbtime2;
  }
	else
	{
    return AT_PARAM_ERROR;
	}
	
	return AT_OK;
}

ATEerror_t at_decrypt_set(const char *param)
{
	uint8_t temp;
	if (tiny_sscanf(param, "%d", &temp) != 1)
  {
    return AT_PARAM_ERROR;
  }
	
	if(temp<=1)
	{
		decrypt_flag=temp;
	}
	else
	{
		return AT_PARAM_ERROR;
	}
	
	return AT_OK;	
}

ATEerror_t at_decrypt_get(const char *param)
{
	PPRINTF("%d\r\n",decrypt_flag);
	return AT_OK;	
}

ATEerror_t at_MOD_set(const char *param)
{ 
	uint8_t workmode;
	if (tiny_sscanf(param, "%d", &workmode) != 1)
  {
    return AT_PARAM_ERROR;
  }
	if ((workmode>=1)&&(workmode<=9))
  {
    mode=workmode;	
  	PPRINTF("Attention:Take effect after ATZ\r\n");			
	}
	else
	{
		PPRINTF("Mode of range is 1 to 9\r\n");	
    return AT_PARAM_ERROR;
	}
	
	return AT_OK;
}

ATEerror_t at_MOD_get(const char *param)
{ 
	print_d(mode);
	return AT_OK;
}


ATEerror_t at_INTMOD1_set(const char *param)
{ 
	uint8_t interrputmode;
	if (tiny_sscanf(param, "%d", &interrputmode) != 1)
  {
    return AT_PARAM_ERROR;
  }
	if (interrputmode<=3)
  {
    inmode=interrputmode;		
	}
	else
	{
		PPRINTF("INTMode1 of range is 0 to 3\r\n");			
    return AT_PARAM_ERROR;
	}
	
	GPIO_EXTI14_IoInit(inmode);
	
	return AT_OK;
}

ATEerror_t at_INTMOD1_get(const char *param)
{ 
	print_d(inmode);
	return AT_OK;
}

ATEerror_t at_INTMOD2_set(const char *param)
{
	uint8_t interrputmode;
	if (tiny_sscanf(param, "%d", &interrputmode) != 1)
  {
    return AT_PARAM_ERROR;
  }
	if (interrputmode<=3)
  {
    inmode2=interrputmode;		
	}
	else
	{
		PPRINTF("INTMode2 of range is 0 to 3\r\n");			
    return AT_PARAM_ERROR;
	}
	
	GPIO_EXTI15_IoInit(inmode2);
	
	return AT_OK;	
}

ATEerror_t at_INTMOD2_get(const char *param)
{
	print_d(inmode2);
	return AT_OK;	
}

ATEerror_t at_INTMOD3_set(const char *param)
{
	uint8_t interrputmode;
	if (tiny_sscanf(param, "%d", &interrputmode) != 1)
  {
    return AT_PARAM_ERROR;
  }
	if (interrputmode<=3)
  {
    inmode3=interrputmode;		
	}
	else
	{
		PPRINTF("INTMode3 of range is 0 to 3\r\n");			
    return AT_PARAM_ERROR;
	}
	
	GPIO_EXTI4_IoInit(inmode3);
	
	return AT_OK;	
}

ATEerror_t at_INTMOD3_get(const char *param)
{
	print_d(inmode3);
	return AT_OK;	
}

ATEerror_t at_weightreset(const char *param)
{
	weightreset();
	
	return AT_OK;	
}

ATEerror_t at_weight_GapValue_set(const char *param)
{
	uint16_t gapvalue_a;
	uint8_t  gapvalue_b;
	
	if (tiny_sscanf(param, "%d.%d", &gapvalue_a,&gapvalue_b) != 2)
  {
    return AT_PARAM_ERROR;
  }	
	
	if(gapvalue_b>=10)
  {
    return AT_PARAM_ERROR;
  }	
	
	GapValue=gapvalue_a+((float)gapvalue_b/10.0);
	Get_Weight();
	
  return AT_OK;	
}

ATEerror_t at_weight_GapValue_get(const char *param)
{
	PPRINTF("%0.1f\r\n",GapValue);
	
  return AT_OK;		
}

ATEerror_t at_5Vtime_set(const char *param)
{
	uint16_t power_5v;
	if (tiny_sscanf(param, "%d", &power_5v) != 1)
  {
    return AT_PARAM_ERROR;
  }	
	
	power_time=power_5v;
	
  return AT_OK;		
}

ATEerror_t at_5Vtime_get(const char *param)
{
	print_d(power_time);
	return AT_OK;
}

ATEerror_t at_SETCNT_set(const char *param)
{
	uint8_t coun;
	uint32_t count_temp;
  if (tiny_sscanf(param,"%d,%lu",&coun,&count_temp) != 2)
	{
		return AT_PARAM_ERROR;
	}
			
	if(coun==1)
	{
		COUNT=count_temp;
	}
	else if(coun==2)
	{
		COUNT2=count_temp;			
	}		
	
	return AT_OK;
}

ATEerror_t at_getsensorvaule_set(const char *param)
{
	uint8_t stus;
	if (tiny_sscanf(param, "%d", &stus) != 1)
  {
    return AT_PARAM_ERROR;
  }	
	
	if(stus==0)
	{
		if(( LoRaMacState & 0x00000001 ) == 0x00000001)
		{
			return AT_BUSY_ERROR;
		}	
		sensor_t sensor_message;
		BSP_sensor_Read(&sensor_message,1);
	}
	else if(stus==1)
	{
		message_flags=1;
		uplink_data_status=1;
	}
	else
	{
		return AT_PARAM_ERROR;
	}
	
	return AT_OK;	
}

ATEerror_t at_downlink_detect_set(const char *param)
{
	uint8_t status;
	uint16_t timeout1=0,timeout2=0;
	if (tiny_sscanf(param, "%d,%lu,%lu", &status,&timeout1,&timeout2) != 3)
  {
    return AT_PARAM_ERROR;
  }
	
	if(status<2)
	{
		downlink_detect_switch=status;
	}
	else
		return AT_PARAM_ERROR;
	
	if(timeout1>0 && timeout1<=65535 && timeout2>0 && timeout2<=65535)
	{
		unconfirmed_uplink_change_to_confirmed_uplink_timeout=timeout1;
		downlink_detect_timeout=timeout2;
	}
	else
	{
		PRINTF("The timeout range is 1 to 65535\n\r");
		return AT_PARAM_ERROR;	
	}
	
	PPRINTF("Attention:Take effect after ATZ\r\n");
	
	return AT_OK;	
}

ATEerror_t at_downlink_detect_get(const char *param)
{
	AT_PRINTF("%d,%d,%d\r\n", downlink_detect_switch,unconfirmed_uplink_change_to_confirmed_uplink_timeout,downlink_detect_timeout);
	return AT_OK;	
}

ATEerror_t at_setmaxnbtrans_set(const char *param)
{
	uint8_t trials,increment_switch;
	if(tiny_sscanf(param, "%d,%d", &trials,&increment_switch) != 2)
	{
		return AT_PARAM_ERROR;
	}
	
	if(trials<1 || trials>15 || increment_switch>1)
	{
		return AT_PARAM_ERROR;
	}
	
	LinkADR_NbTrans_retransmission_nbtrials=trials;
  LinkADR_NbTrans_uplink_counter_retransmission_increment_switch=increment_switch;

  return AT_OK;	
}

ATEerror_t at_setmaxnbtrans_get(const char *param)
{
  AT_PRINTF("%d,%d\r\n", LinkADR_NbTrans_retransmission_nbtrials,LinkADR_NbTrans_uplink_counter_retransmission_increment_switch);
  return AT_OK;	
}

ATEerror_t at_disdownlinkcheck_set(const char *param)
{
	uint8_t temp1;
	if (tiny_sscanf(param, "%d", &temp1) != 1)
  {
		return AT_PARAM_ERROR;
	}		
	
	if(temp1<=1)
	{
		down_check=temp1;
	}
	else
	{
		return AT_PARAM_ERROR;
	}
	
	return AT_OK;			
}

ATEerror_t at_disdownlinkcheck_get(const char *param)
{
	PPRINTF("%d\r\n",down_check);
	return AT_OK;			
}

ATEerror_t at_dismac_answer_set(const char *param)
{
	uint8_t temp1;
	if (tiny_sscanf(param, "%d", &temp1) != 1)
  {
		return AT_PARAM_ERROR;
	}		
	
	if(temp1<=1)
	{
		mac_response_flag=temp1;
	}
	else
	{
		return AT_PARAM_ERROR;
	}
	
	return AT_OK;		
}

ATEerror_t at_dismac_answer_get(const char *param)
{
	PPRINTF("%d\r\n",mac_response_flag);
	return AT_OK;		
}

ATEerror_t at_rxdata_test(const char *param)
{
  const char *buf= param;
  unsigned char bufSize= strlen(param);
  unsigned size=0;
  char hex[3];
  uint8_t buff[30];
	uint8_t length=0;

  hex[2] = 0;
  while ((size < 255) && (bufSize > 1))
  {
    hex[0] = buf[size*2];
    hex[1] = buf[size*2+1];
    if (tiny_sscanf(hex, "%hhx", &buff[size]) != 1)
    {
      return AT_PARAM_ERROR;
    }
    size++;
    bufSize -= 2;
  }
	
  if (bufSize != 0)
  {
    return AT_PARAM_ERROR;
  }

	length=size;	
	lora_AppData_t rxappData;
	rxappData.Port = 2;
  rxappData.BuffSize = length;
	rxappData.Buff=buff;
  LoRaMainCallbacks->LORA_RxData( &rxappData );
	
	return AT_OK;		
}

/* Private functions ---------------------------------------------------------*/

static ATEerror_t translate_status(LoRaMacStatus_t status)
{
  if (status == LORAMAC_STATUS_BUSY)
  {
    return AT_BUSY_ERROR;
  }
  if (status == LORAMAC_STATUS_PARAMETER_INVALID)
  {
    return AT_PARAM_ERROR;
  }
  if (status == LORAMAC_STATUS_NO_NETWORK_JOINED)
  {
    return AT_NO_NET_JOINED;
  }
		if(status == LORAMAC_STATUS_PARAMETER_NOT_in_Range)
	{
    return AT_PARAM_NOT_Range;		
	}
	if(status == LORAMAC_STATUS_PARAMETER_is_used)
	{
    return AT_PARAM_FDR;		
	}	
  if (status != LORAMAC_STATUS_OK)
  {
    return AT_ERROR;
  }
  return AT_OK;
}

static int sscanf_16_hhx(const char *from, uint8_t *pt,uint8_t num)
{
	if(num==1)
	{
		return tiny_sscanf(from, "%hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx",
                      &pt[0], &pt[1], &pt[2], &pt[3], &pt[4], &pt[5], &pt[6],
                      &pt[7], &pt[8], &pt[9], &pt[10], &pt[11], &pt[12], &pt[13],
                      &pt[14], &pt[15]);
	}
	else if(num==2)
	{	
		if(changeform(from,pt,16)==1)
		{
			return 0;
		}
		else
		{
			return changeform(from,pt,16);
		}
	}
	return 0;
}

static void print_16_02x(uint8_t *pt)
{
  AT_PRINTF("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\r\n",
            pt[0], pt[1], pt[2], pt[3],
            pt[4], pt[5], pt[6], pt[7],
            pt[8], pt[9], pt[10], pt[11],
            pt[12], pt[13], pt[14], pt[15]);
}

static int sscanf_uint32_as_hhx(const char *from, uint32_t *value,uint8_t num)
{
	if(num==1)
	{	
		return tiny_sscanf(from, "%hhx %hhx %hhx %hhx",
                      &((unsigned char *)(value))[3],
                      &((unsigned char *)(value))[2],
                      &((unsigned char *)(value))[1],
                      &((unsigned char *)(value))[0]);
  }
	else if(num==2)
	{
		uint8_t pt[4];	
		uint8_t return_temp;
		
		if(changeform(from,pt,4)==1)
		{
			return 0;
		}
		else
		{
			return_temp=changeform(from,pt,4);
			((unsigned char *)(value))[3]=pt[0];
			((unsigned char *)(value))[2]=pt[1];
			((unsigned char *)(value))[1]=pt[2];
			((unsigned char *)(value))[0]=pt[3];			
			return return_temp;
		}	
	}	
	return 0;	
}

static void print_uint32_as_02x(uint32_t value)
{
  AT_PRINTF("%02x %02x %02x %02x\r\n",
            (unsigned)((unsigned char *)(&value))[3],
            (unsigned)((unsigned char *)(&value))[2],
            (unsigned)((unsigned char *)(&value))[1],
            (unsigned)((unsigned char *)(&value))[0]);
}

static int sscanf_8_hhx(const char *from, uint8_t *pt,uint8_t num)
{
	if(num==1)
	{		
		return tiny_sscanf(from, "%hhx %hhx %hhx %hhx %hhx %hhx %hhx %hhx",
                      &pt[0], &pt[1], &pt[2], &pt[3], &pt[4], &pt[5], &pt[6], &pt[7]);
	}
	else if(num==2)
	{		
		if(changeform(from,pt,8)==1)
		{
			return 0;
		}
		else
		{
			return changeform(from,pt,8);
		}
	}	
	return 0;
}

static void print_8_02x(uint8_t *pt)
{
  AT_PRINTF("%02x %02x %02x %02x %02x %02x %02x %02x\r\n",
            pt[0], pt[1], pt[2], pt[3], pt[4], pt[5], pt[6], pt[7]);
}

static void print_d(int value)
{
  AT_PRINTF("%d\r\n", value);
}

static void print_u(unsigned int value)
{
  AT_PRINTF("%u\r\n", value);
}

static uint8_t changeform(const char *from,uint8_t *pt,uint8_t number)
{
  char *p[16];
  char temp[2]="";
	uint8_t k=0;
	uint8_t l=0;
	
	if(strlen(from)==number*2)
	{
	  for(int i=0;i<strlen(from);i++)
	  {
			temp[l++]=from[i];
			if(l%2==0)
			{
				p[k++]=temp;
				sscanf(p[k-1],"%hhx",&pt[k-1]);					
				l=0;
			}				
		}
		return number;
	}
	else
	{
		if(strlen(from)!=(number*3-1))
		{
			PPRINTF("AT_PARAM_ERROR(Incorrect Length)\r\n");
		}			
		return 1;
	}
}

void weightreset(void)
{
	HAL_GPIO_WritePin(PWR_OUT_PORT,PWR_OUT_PIN,GPIO_PIN_RESET);//Enable 5v power supply
	WEIGHT_SCK_Init();
	WEIGHT_DOUT_Init();
	Get_Maopi();	
  HAL_Delay(500);
  Get_Maopi();
	WEIGHT_SCK_DeInit();
	WEIGHT_DOUT_DeInit();		
	HAL_GPIO_WritePin(PWR_OUT_PORT,PWR_OUT_PIN,GPIO_PIN_SET);//Disable 5v power supply
}
