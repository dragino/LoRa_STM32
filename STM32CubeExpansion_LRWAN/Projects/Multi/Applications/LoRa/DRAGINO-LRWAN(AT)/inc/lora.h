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
  * @version V1.1.4
  * @date    08-January-2018
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
#include "LoRaMac.h"
#include "region/Region.h"

/* Exported constants --------------------------------------------------------*/
   /*!
 * LoRaWAN confirmed messages
 */

#define LORAWAN_ADR_ON                              1
#define LORAWAN_ADR_OFF                             0
/* Exported types ------------------------------------------------------------*/


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

typedef enum 
{
  LORA_RESET = 0, 
  LORA_SET = !LORA_RESET
} LoraFlagStatus;

typedef enum 
{
  LORA_DISABLE = 0, 
  LORA_ENABLE = !LORA_DISABLE
} LoraState_t;

typedef enum 
{
  LORA_ERROR = -1, 
  LORA_SUCCESS = 0
} LoraErrorStatus;

typedef enum 
{
  LORAWAN_UNCONFIRMED_MSG = 0, 
  LORAWAN_CONFIRMED_MSG = !LORAWAN_UNCONFIRMED_MSG
} LoraConfirm_t;

typedef enum 
{
  LORA_TRUE = 0, 
  LORA_FALSE = !LORA_TRUE
} LoraBool_t;

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
 * \brief Get the current temperature
 *
 * \retval value  temperature in degreeCelcius( q7.8 )
 */
  uint16_t ( *BoardGetTemperatureLevel)( void );
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
 * @brief Process Rx Data received from Lora network 
 *
 * @param [IN] AppData structure
 *
 */
    void ( *LORA_RxData ) ( lora_AppData_t *AppData);
    
    /*!
 * @brief callback indicating EndNode has jsu joiny 
 *
 * @param [IN] None
 */
    void ( *LORA_HasJoined)( void );
    /*!
 * @brief Confirms the class change 
 *
 * @param [IN] AppData is a buffer to process
 *
 * @param [IN] port is a Application port on wicth Appdata will be sent
 *
 * @param [IN] length is the number of recieved bytes
 */
    void ( *LORA_ConfirmClass) ( DeviceClass_t Class );
  
} LoRaMainCallback_t;



/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 
/**
 * @brief Lora Initialisation
 * @param [IN] LoRaMainCallback_t
 * @param [IN] application parmaters
 * @retval none
 */
void LORA_Init (LoRaMainCallback_t *callbacks, LoRaParam_t* LoRaParam );
void fdr_config(void);
/**
 * @brief run Lora classA state Machine 
 * @param [IN] none
 * @retval none
 */
LoraErrorStatus LORA_send(lora_AppData_t* AppData, LoraConfirm_t IsTxConfirmed);

/**
 * @brief Join a Lora Network in classA
 * @Note if the device is ABP, this is a pass through functon
 * @param [IN] none
 * @retval none
 */
void LORA_Join( void);

/**
 * @brief Check whether the Device is joined to the network
 * @param [IN] none
 * @retval returns LORA_SET if joined
 */
LoraFlagStatus LORA_JoinStatus( void);

/**
 * @brief change Lora Class
 * @Note callback LORA_ConfirmClass informs upper layer that the change has occured
 * @Note Only switch from class A to class B/C OR from  class B/C to class A is allowed
 * @Attention can be calld only in LORA_ClassSwitchSlot or LORA_RxData callbacks
 * @param [IN] DeviceClass_t NewClass
 * @retval LoraErrorStatus
 */
LoraErrorStatus LORA_RequestClass( DeviceClass_t newClass );

/**
 * @brief get the current Lora Class
 * @param [IN] DeviceClass_t NewClass
 * @retval None
 */
void LORA_GetCurrentClass( DeviceClass_t *currentClass );
/**
  * @brief  Set join activation process: OTAA vs ABP
  * @param  Over The Air Activation status to set: enable or disable
  * @retval None
  */
void lora_config_otaa_set(LoraState_t otaa);

/**
  * @brief  Get join activation process: OTAA vs ABP
  * @param  None
  * @retval ENABLE if OTAA is used, DISABLE if ABP is used
  */
LoraState_t lora_config_otaa_get(void);

/**
  * @brief  Set duty cycle: ENABLE or DISABLE
  * @param  Duty cycle to set: enable or disable
  * @retval None
  */
void lora_config_duty_cycle_set(LoraState_t duty_cycle);

/**
  * @brief  Get Duty cycle: OTAA vs ABP
  * @param  None
  * @retval ENABLE / DISABLE
  */
LoraState_t lora_config_duty_cycle_get(void);

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
 * @brief  Set whether or not acknowledgement is required
 * @param  ENABLE or DISABLE
 * @retval None
 */
void lora_config_reqack_set(LoraConfirm_t reqack);

/**
 * @brief  Get whether or not acknowledgement is required
 * @param  None
 * @retval ENABLE or DISABLE
 */
LoraConfirm_t lora_config_reqack_get(void);

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
LoraState_t lora_config_isack_get(void);

/**
 * @brief  Launch LoraWan certification tests
 * @param  None
 * @retval The application port
 */ 
void lora_wan_certif(void);

/**
 * @brief  set tx datarate
 * @param  None
 * @retval The application port
 */ 
void lora_config_tx_datarate_set(int8_t TxDataRate);

/**
 * @brief  get tx datarate
 * @param  None
 * @retval tx datarate
 */ 
int8_t lora_config_tx_datarate_get(void );

/**
  * @brief  set 
  * @param  
  * @retval None
  */
void lora_config_application_port_set(int8_t application_port);

/**
  * @brief  get 
  * @param  
  * @retval None
  */
int8_t lora_config_application_port_get(void );

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

void Store_key(void);
void Store_Config(void);
void store_data(uint8_t size,uint8_t *data1,uint32_t data2);
void read_data(uint8_t size,uint8_t *data1,uint32_t data3,uint32_t data4,uint32_t data5,uint32_t data6);	 
void Read_Config(void);
void key_printf(void);
uint16_t string_touint(void);
void new_firmware_update(void);

uint32_t customize_freq1_get(void);

void customize_freq1_set(uint32_t Freq);

uint32_t customize_set8channel_get(void);

void customize_set8channel_set(uint8_t Freq);

void region_printf(void);

#ifdef __cplusplus
}
#endif

#endif /*__LORA_MAIN_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
