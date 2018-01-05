 /*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: LoRaMac classA device implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Wael Guibene
*/
/******************************************************************************
  * @file    lora.h
  * @author  MCD Application Team
  * @version V1.1.2
  * @date    08-September-2017
  * @brief   lora API to drive the lora state Machine
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
/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __LORA_MAIN_H__
#define __LORA_MAIN_H__

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
#include "Commissioning.h"
#include "utilities.h"
#include "LoRaMac.h"
#include "region/Region.h"

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

 
 /**
   * Lora Configuration
   */
 typedef struct
 {
   FunctionalState otaa;        /*< ENABLE if over the air activation, DISABLE otherwise */
   FunctionalState duty_cycle;  /*< ENABLE if dutycyle is on, DISABLE otherwise */
	 uint32_t DevAddr;          /*< Device  Address*/
   uint8_t DevEui[8];           /*< Device EUI */
   uint8_t AppEui[8];           /*< Application EUI */
   uint8_t AppKey[16];          /*< Application Key */
   uint8_t NwkSKey[16];         /*< Network Session Key */
   uint8_t AppSKey[16];         /*< Application Session Key */
   int16_t Rssi;                /*< Rssi of the received packet */
   uint8_t Snr;                 /*< Snr of the received packet */
   uint8_t application_port;    /*< Application port we will receive to */
   FunctionalState ReqAck;      /*< ENABLE if acknowledge is requested */
   McpsConfirm_t *McpsConfirm;  /*< pointer to the confirm structure */
 } lora_configuration_t;
 
/*!
 * Application Data structure
 */
typedef struct
{
  /*point to the LoRa App data buffer*/
  uint8_t* Buff;
  /*LoRa App data buffer size*/
  uint8_t BuffSize;
  /*Port on which the LoRa App is data is sent/ received*/
  uint8_t Port;
  
} lora_AppData_t;

/*!
 * LoRa State Machine states 
 */
typedef enum eDevicState
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_JOINED,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
} DeviceState_t;

/*!
 * LoRa State Machine states 
 */
typedef enum eTxEventType
{
/*!
 * @brief AppdataTransmition issue based on timer every TxDutyCycleTime
 */
    TX_ON_TIMER,
/*!
 * @brief AppdataTransmition external event plugged on OnSendEvent( )
 */
    TX_ON_EVENT
} TxEventType_t;


/*!
 * LoRa State Machine states 
 */
typedef struct sLoRaParam
{
/*!
 * @brief Event type
 *
 * @retval value  battery level ( 0: very low, 254: fully charged )
 */
    TxEventType_t TxEvent;
/*!
 * @brief Application data transmission duty cycle in ms
 *
 * @note when TX_ON_TIMER Event type is selected
 */
    uint32_t TxDutyCycleTime;
/*!
 * @brief LoRaWAN device class
 */
    DeviceClass_t Class;
/*!
 * @brief Activation state of adaptativeDatarate
 */
    bool AdrEnable;
/*!
 * @brief Uplink datarate, if AdrEnable is off
 */
    int8_t TxDatarate;
/*!
 * @brief Enable or disable a public network
 *
 */
    bool EnablePublicNetwork;
/*!
 * @brief Number of trials for the join request.
 */
    uint8_t NbTrials;

} LoRaParam_t;

/* Lora Main callbacks*/
typedef struct sLoRaMainCallback
{
/*!
 * @brief Get the current battery level
 *
 * @retval value  battery level ( 0: very low, 254: fully charged )
 */
    uint8_t ( *BoardGetBatteryLevel )( void );
  
/*!
 * @brief Gets the board 64 bits unique ID 
 *
 * @param [IN] id Pointer to an array that will contain the Unique ID
 */
    void    ( *BoardGetUniqueId ) ( uint8_t *id);
  /*!
 * Returns a pseudo random seed generated using the MCU Unique ID
 *
 * @retval seed Generated pseudo random seed
 */
    uint32_t ( *BoardGetRandomSeed ) (void);
/*!
 * @brief Prepares Tx Data to be sent on Lora network 
 *
 * @param [IN] AppData is a buffer to fill
 *
 * @param [IN] port is a Application port on wicth Appdata will be sent
 *
 * @param [IN] length of the AppDataBuffer to send
 *
 * @param [IN] requests a confirmed Frame from the Network
 */
  void ( *LoraTxData ) ( lora_AppData_t *AppData, FunctionalState* IsTxConfirmed);
/*!
 * @brief Process Rx Data received from Lora network 
 *
 * @param [IN] AppData is a buffer to process
 *
 * @param [IN] port is a Application port on wicth Appdata will be sent
 *
 * @param [IN] length is the number of recieved bytes
 */
    void ( *LoraRxData ) ( lora_AppData_t *AppData);
  
} LoRaMainCallback_t;



/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 
/**
 * @brief Lora Join procedure
 * @param [IN] none
 * @retval status (ok or busy)
 */
LoRaMacStatus_t lora_join(void);

/**
 * @brief Lora Send command
 * @param [IN] buf Pointer to buffer of data
 * @param [IN] size Size of data
 * @param [IN] binary Whether buffer contains raw data or a string of hexadecimal values (ie binary data)
 * @retval LoRa status
 */
LoRaMacStatus_t lora_send(const char *buf, unsigned size, unsigned binary);

/**
 * @brief Lora Send EXTI Data
 * @param [IN] none
 * @retval LoRa status
 */
LoRaMacStatus_t lora_send_exti(void);

/**
 * @brief Lora Initialisation
 * @param [IN] LoRaMainCallback_t
 * @param [IN] application parmaters
 * @retval none
 */
void lora_Init (LoRaMainCallback_t *callbacks, LoRaParam_t* LoRaParamInit );

/**
 * @brief run Lora classA state Machine 
 * @param [IN] none
 * @retval none
 */
void lora_fsm( void );

/**
 * @brief functionl requesting loRa state machine to send data 
 * @note function to link in mode TX_ON_EVENT 
 * @param  none
 * @retval none
 */
void OnSendEvent( void );


/**
 * @brief API returns the state of the lora state machine
 * @note return @DeviceState_t state
 * @param [IN] none
 * @retval return @FlagStatus
  */
DeviceState_t lora_getDeviceState( void );

/**
  * @brief  Set join activation process: OTAA vs ABP
  * @param  Over The Air Activation status to set: enable or disable
  * @retval None
  */
void lora_config_otaa_set(FunctionalState otaa);

/**
  * @brief  Get join activation process: OTAA vs ABP
  * @param  None
  * @retval ENABLE if OTAA is used, DISABLE if ABP is used
  */
FunctionalState lora_config_otaa_get(void);

/**
  * @brief  Set duty cycle: ENABLE or DISABLE
  * @param  Duty cycle to set: enable or disable
  * @retval None
  */
void lora_config_duty_cycle_set(FunctionalState duty_cycle);

/**
  * @brief  Get Duty cycle: OTAA vs ABP
  * @param  None
  * @retval ENABLE / DISABLE
  */
FunctionalState lora_config_duty_cycle_get(void);

/**
  * @brief  Get Device EUI
  * @param  None
  * @retval DevEUI
  */
uint8_t *lora_config_deveui_get(void);

/**
  * @brief  Set Device EUI
  * @param  None
  * @retval DevEUI
  */
void lora_config_deveui_set(uint8_t deveui[8]);

/**
  * @brief  Get Application EUI
  * @param  None
  * @retval AppEUI
  */
uint8_t *lora_config_appeui_get(void);

/**
  * @brief  Set Application EUI
  * @param  AppEUI
  * @retval Nonoe
  */
void lora_config_appeui_set(uint8_t appeui[8]);

/**
  * @brief  Get 
  * @param  None
  * @retval 
  */
void lora_config_appskey_set(uint8_t appskey[16]);

/**
  * @brief  Get 
  * @param  None
  * @retval 
  */
uint8_t *lora_config_appskey_get(void);
	
/**
  * @brief  Get 
  * @param  None
  * @retval 
  */
void lora_config_nwkskey_set(uint8_t nwkskey[16]);

/**
  * @brief  Get 
  * @param  None
  * @retval 
  */
uint8_t *lora_config_nwkskey_get(void);

/**
  * @brief  Get Application Key
  * @param  None
  * @retval AppKey
  */
uint8_t *lora_config_appkey_get(void);

/**
  * @brief  Set Application Key
  * @param  AppKey
  * @retval None
  */
void lora_config_appkey_set(uint8_t appkey[16]);

/**
  * @brief  get 
  * @param  
  * @retval None
  */
void lora_config_devaddr_get(void);


/**
  * @brief  set 
  * @param  
  * @retval None
  */
void lora_config_devaddr_set(uint32_t devaddr);
/**
 * @brief  Set whether or not acknowledgement is required
 * @param  ENABLE or DISABLE
 * @retval None
 */
void lora_config_reqack_set(FunctionalState reqack);

/**
 * @brief  Get whether or not acknowledgement is required
 * @param  None
 * @retval ENABLE or DISABLE
 */
FunctionalState lora_config_reqack_get(void);

/**
 * @brief  Get the SNR of the last received data
 * @param  None
 * @retval SNR
 */
int8_t lora_config_snr_get(void);

/**
 * @brief  Get the RSSI of the last received data
 * @param  None
 * @retval RSSI
 */
int16_t lora_config_rssi_get(void);

/**
 * @brief  Get whether or not the last sent data were acknowledged
 * @param  None
 * @retval ENABLE if so, DISABLE otherwise
 */
FunctionalState lora_config_isack_get(void);

/**
 * @brief  Set the application port we will receive the data to
 * @param  The application port
 * @retval None
 */
void lora_config_application_port_set(uint8_t application_port);

/**
 * @brief  Get the application port we will receive the data to
 * @param  None
 * @retval The application port
 */
uint8_t lora_config_application_port_get(void);

/**
 * @brief  Launch LoraWan certification tests
 * @param  None
 * @retval The application port
 */ 
void lora_wan_certif(void);

#ifdef __cplusplus
}
#endif

#endif /*__LORA_MAIN_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
