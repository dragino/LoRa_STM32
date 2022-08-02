/*******************************************************************************
 * @file    at.h
 * @author  MCD Application Team
 * @version V1.1.4
 * @date    08-January-2018
 * @brief   Header for driver at.c module
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
#ifndef __AT_H__
#define __AT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
/*
 * AT Command Id errors. Note that they are in sync with ATError_description static array
 * in command.c
 */
typedef enum eATEerror
{
  AT_OK = 0,
  AT_ERROR,
  AT_PARAM_ERROR,
  AT_PARAM_NOT_Range,	
  AT_PARAM_FDR,		
  AT_BUSY_ERROR,
  AT_TEST_PARAM_OVERFLOW,
  AT_NO_NET_JOINED,
  AT_RX_ERROR,
  AT_MAX,
} ATEerror_t;

/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* AT printf */
#define AT_PRINTF     PPRINTF

/* AT Command strings. Commands start with AT */
#define AT_RESET      "Z"
#define AT_FDR        "+FDR"
#define AT_DEUI       "+DEUI"
#define AT_DADDR      "+DADDR"
#define AT_APPKEY     "+APPKEY"
#define AT_NWKSKEY    "+NWKSKEY"
#define AT_APPSKEY    "+APPSKEY"
#define AT_APPEUI     "+APPEUI"
#define AT_ADR        "+ADR"
#define AT_TXP        "+TXP"
#define AT_DR         "+DR"
#define AT_DCS        "+DCS"
#define AT_PNM        "+PNM"
#define AT_RX2FQ      "+RX2FQ"
#define AT_RX2DR      "+RX2DR"
#define AT_RX1DL      "+RX1DL"
#define AT_RX2DL      "+RX2DL"
#define AT_JN1DL      "+JN1DL"
#define AT_JN2DL      "+JN2DL"
#define AT_NJM        "+NJM"
#define AT_NWKID      "+NWKID"
#define AT_FCU        "+FCU"
#define AT_FCD        "+FCD"
#define AT_CLASS      "+CLASS"
#define AT_JOIN       "+JOIN"
#define AT_NJS        "+NJS"
#define AT_SENDB      "+SENDB"
#define AT_SEND       "+SEND"
#define AT_RECVB      "+RECVB"
#define AT_RECV       "+RECV"
#define AT_DWELLT     "+DWELLT"
#define AT_VER        "+VER"
#define AT_CFM        "+CFM"
#define AT_CFS        "+CFS"
#define AT_SNR        "+SNR"
#define AT_RSSI       "+RSSI"
#define AT_RJTDC      "+RJTDC"
#define AT_RPL        "+RPL"
#define AT_DEBUG      "+DEBUG"
#define AT_TDC        "+TDC"
#define AT_PORT       "+PORT"
#define AT_CHS        "+CHS"
#define AT_CHE        "+CHE"
#define AT_CFG        "+CFG"
#define AT_RX1WTO     "+RX1WTO"
#define AT_RX2WTO     "+RX2WTO"
#define AT_DECRYPT    "+DECRYPT"
#define AT_MOD        "+MOD"
#define AT_INTMOD1    "+INTMOD1"
#define AT_INTMOD2    "+INTMOD2"
#define AT_INTMOD3    "+INTMOD3"
#define AT_WEIGRE     "+WEIGRE"
#define AT_WEIGAP     "+WEIGAP"
#define AT_5VT        "+5VT"
#define AT_SETCNT     "+SETCNT"
#define AT_DDETECT    "+DDETECT"
#define AT_SETMAXNBTRANS    "+SETMAXNBTRANS"
#define AT_GETSENSORVALUE   "+GETSENSORVALUE"
#define AT_DISFCNTCHECK 		"+DISFCNTCHECK"
#define AT_DISMACANS 	   	  "+DISMACANS"
#define AT_RXDATEST  			  "+RXDATEST"

/* Exported functions ------------------------------------------------------- */
void weightreset(void);
/**
 * @brief  Store the received data
 * @param  Application port
 * @param  Buffer of the received data
 * @param  Size of the received data
 * @retval None
 */
void set_at_receive(uint8_t AppPort, uint8_t* Buff, uint8_t BuffSize);

/**
 * @brief  Return AT_OK in all cases
 * @param  Param string of the AT command - unused
 * @retval AT_OK
 */
ATEerror_t at_return_ok(const char *param);

/**
 * @brief  Return AT_ERROR in all cases
 * @param  Param string of the AT command - unused
 * @retval AT_ERROR
 */
ATEerror_t at_return_error(const char *param);

/**
 * @brief  Trig a reset of the MCU
 * @param  Param string of the AT command - unused
 * @retval AT_OK
 */
ATEerror_t at_reset(const char *param);

/**
 * @brief  Flash erase
 * @param  Param string of the AT command - unused
 * @retval AT_OK
 */
ATEerror_t at_FDR(const char *param);

/**
 * @brief  Print Device EUI
 * @param  Param string of the AT command - unused
 * @retval AT_OK
 */
ATEerror_t at_DevEUI_get(const char *param);

/**
 * @brief  Set Device EUI
 * @param  Param string of the AT command
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_DevEUI_set(const char *param);

/**
 * @brief  Print Application EUI
 * @param  Param string of the AT command
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_AppEUI_get(const char *param);

/**
 * @brief  Set Application EUI
 * @param  Param string of the AT command
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_AppEUI_set(const char *param);

/**
 * @brief  Set DevAddr
 * @param  String pointing to provided DevAddr
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_DevAddr_set(const char *param);

/**
 * @brief  Print the DevAddr
 * @param  String pointing to provided DevAddr
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_DevAddr_get(const char *param);

/**
 * @brief  Print Application Key
 * @param  Param string of the AT command
 * @retval AT_OK
 */
ATEerror_t at_AppKey_get(const char *param);

/**
 * @brief  Set Application Key
 * @param  Param string of the AT command
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_AppKey_set(const char *param);

/**
 * @brief  Print Network Session Key
 * @param  String pointing to provided DevAddr
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_NwkSKey_get(const char *param);

/**
 * @brief  Set Network Session Key
 * @param  String pointing to provided DevAddr
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_NwkSKey_set(const char *param);

/**
 * @brief  Print Application Session Key
 * @param  String pointing to provided DevAddr
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_AppSKey_get(const char *param);

/**
 * @brief  Set Application Session Key
 * @param  String pointing to provided DevAddr
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_AppSKey_set(const char *param);

/**
 * @brief  Print Adaptative Data Rate setting
 * @param  String pointing to provided ADR setting
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_ADR_get(const char *param);

/**
 * @brief  Set Adaptative Data Rate setting
 * @param  String pointing to provided ADR setting
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_ADR_set(const char *param);

/**
 * @brief  Print Transmit Power
 * @param  String pointing to provided power
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_TransmitPower_get(const char *param);

/**
 * @brief  Set Transmit Power
 * @param  String pointing to provided power
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_TransmitPower_set(const char *param);

/**
 * @brief  Print Data Rate
 * @param  String pointing to provided rate
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_DataRate_get(const char *param);

/**
 * @brief  Set Data Rate
 * @param  String pointing to provided rate
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_DataRate_set(const char *param);

/**
 * @brief  Set ETSI Duty Cycle parameter
 * @param  String pointing to provided Duty Cycle value
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_DutyCycle_set(const char *param);

/**
 * @brief  Get ETSI Duty Cycle parameter
 * @param  0 if disable, 1 if enable
 * @retval AT_OK
 */
ATEerror_t at_DutyCycle_get(const char *param);

/**
 * @brief  Print Public Network setting
 * @param  String pointing to provided Network setting
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_PublicNetwork_get(const char *param);

/**
 * @brief  Set Public Network setting
 * @param  String pointing to provided Network setting
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_PublicNetwork_set(const char *param);

/**
 * @brief  Print Rx2 window frequency
 * @param  String pointing to parameter
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_Rx2Frequency_get(const char *param);

/**
 * @brief  Set Rx2 window frequency
 * @param  String pointing to parameter
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_Rx2Frequency_set(const char *param);

/**
 * @brief  Print Rx2 window data rate
 * @param  String pointing to parameter
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_Rx2DataRate_get(const char *param);

/**
 * @brief  Set Rx2 window data rate
 * @param  String pointing to parameter
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_Rx2DataRate_set(const char *param);

/**
 * @brief  Print the delay between the end of the Tx and the Rx Window 1
 * @param  String pointing to provided param
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_Rx1Delay_get(const char *param);

/**
 * @brief  Set the delay between the end of the Tx and the Rx Window 1
 * @param  String pointing to provided param
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_Rx1Delay_set(const char *param);

/**
 * @brief  Print the delay between the end of the Tx and the Rx Window 2
 * @param  String pointing to provided param
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_Rx2Delay_get(const char *param);

/**
 * @brief  Set the delay between the end of the Tx and the Rx Window 2
 * @param  String pointing to provided param
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_Rx2Delay_set(const char *param);

/**
 * @brief  Print the delay between the end of the Tx and the Join Rx Window 1
 * @param  String pointing to provided param
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_JoinAcceptDelay1_get(const char *param);

/**
 * @brief  Set the delay between the end of the Tx and the Join Rx Window 1
 * @param  String pointing to provided param
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_JoinAcceptDelay1_set(const char *param);

/**
 * @brief  Print the delay between the end of the Tx and the Join Rx Window 2
 * @param  String pointing to provided param
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_JoinAcceptDelay2_get(const char *param);

/**
 * @brief  Set the delay between the end of the Tx and the Join Rx Window 2
 * @param  String pointing to provided param
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_JoinAcceptDelay2_set(const char *param);

/**
 * @brief  Print network join mode
 * @param  String pointing to provided param
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_NetworkJoinMode_get(const char *param);

/**
 * @brief  Set network join mode
 * @param  String pointing to provided param: "1" for OTAA, "0" for ABP
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_NetworkJoinMode_set(const char *param);

/**
 * @brief  Print the Network ID
 * @param  String pointing to provided parameter
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_NetworkID_get(const char *param);

/**
 * @brief  Set the Network ID
 * @param  String pointing to provided parameter
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_NetworkID_set(const char *param);


/**
 * @brief  Print the Uplink Counter
 * @param  String pointing to provided param
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_UplinkCounter_get(const char *param);

/**
 * @brief  Set the Uplink Counter
 * @param  String pointing to provided param
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_UplinkCounter_set(const char *param);

/**
 * @brief  Print the Downlink Counter
 * @param  String pointing to provided param
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_DownlinkCounter_get(const char *param);

/**
 * @brief  Set the Downlink Counter
 * @param  String pointing to provided param
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_DownlinkCounter_set(const char *param);

/**
 * @brief  Print the Device Class
 * @param  String pointing to provided param
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_DeviceClass_get(const char *param);

/**
 * @brief  Set the Device Class
 * @param  String pointing to provided param
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_DeviceClass_set(const char *param);

/**
 * @brief  Join a network
 * @param  String parameter
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_Join(const char *param);

/**
 * @brief  Print the join status
 * @param  String parameter
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_NetworkJoinStatus(const char *param);

/**
 * @brief  Send a message
 * @param  String parameter of hexadecimal value
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_SendBinary(const char *param);

/**
 * @brief  Print last received message
 * @param  String parameter
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_Send(const char *param);

/**
 * @brief  Print last received data in binary format with hexadecimal value
 * @param  String parameter
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_ReceiveBinary(const char *param);

/**
 * @brief  Print last received data
 * @param  String parameter
 * @retval AT_OK if OK, or an appropriate AT_xxx error code
 */
ATEerror_t at_Receive(const char *param);

/**
 * @brief  Print the version of the AT_Slave FW
 * @param  String parameter
 * @retval AT_OK
 */
ATEerror_t at_version_get(const char *param);

/**
 * @brief  Set if message acknowledgment is required (1) or not (0)
 * @param  String parameter
 * @retval AT_OK
 */
ATEerror_t at_ack_set(const char *param);

/**
 * @brief  Get if message acknowledgment is required (1) or not (0)
 * @param  String parameter
 * @retval AT_OK
 */
ATEerror_t at_ack_get(const char *param);

/**
 * @brief  Get if the last message has been acknowledged or not
 * @param  String parameter
 * @retval AT_OK
 */
ATEerror_t at_isack_get(const char *param);

/**
 * @brief  Get the SNR
 * @param  String parameter
 * @retval AT_OK
 */
ATEerror_t at_snr_get(const char *param);

/**
 * @brief  Get the RSSI
 * @param  String parameter
 * @retval AT_OK
 */
ATEerror_t at_rssi_get(const char *param);

/**
 * @brief  
 * @param  String parameter
 * @retval AT_OK
 */
ATEerror_t at_RJTDC_get(const char *param);

/**
 * @brief  
 * @param  String parameter
 * @retval AT_OK
 */
ATEerror_t at_RJTDC_set(const char *param);

/**
 * @brief  
 * @param  String parameter
 * @retval AT_OK
 */
ATEerror_t at_RPL_get(const char *param);

/**
 * @brief  
 * @param  String parameter
 * @retval AT_OK
 */
ATEerror_t at_RPL_set(const char *param);

/**
 * @brief  Set Rx or Tx test config
 * @param  String parameter
 * @retval AT_OK
 */
ATEerror_t at_test_set_lora_config(const char *param);

/**
 * @brief  Get Rx or Tx test config
 * @param  String parameter
 * @retval AT_OK
 */
ATEerror_t at_test_get_lora_config(const char *param);

///**
// * @brief  set the Modem in Certif Mode
// * @param  String parameter
// * @retval AT_OK
// */
//ATEerror_t at_Certif( const char *param );

/**
 * @brief  
 * @param  String parameter
 * @retval AT_OK
 */
ATEerror_t at_TDC_get(const char *param);

/**
 * @brief  
 * @param  String parameter
 * @retval AT_OK
 */
ATEerror_t at_TDC_set(const char *param);

/**
 * @brief  
 * @param  String parameter
 * @retval AT_OK
 */
ATEerror_t at_application_port_get(const char *param);

/**
 * @brief  
 * @param  String parameter
 * @retval AT_OK
 */
	
ATEerror_t at_decrypt_set(const char *param);

ATEerror_t at_decrypt_get(const char *param);

ATEerror_t at_CFG_run(const char *param);
 
ATEerror_t at_DEBUG_run(const char *param);
	
ATEerror_t at_DwellTime_set(const char *param);

ATEerror_t at_DwellTime_get(const char *param);

ATEerror_t at_application_port_set(const char *param);

ATEerror_t at_CHS_get(const char *param);

ATEerror_t at_CHS_set(const char *param);

ATEerror_t at_CHE_get(const char *param);

ATEerror_t at_CHE_set(const char *param);

ATEerror_t at_symbtimeout1LSB_get(const char *param);

ATEerror_t at_symbtimeout1LSB_set(const char *param);

ATEerror_t at_symbtimeout2LSB_get(const char *param);

ATEerror_t at_symbtimeout2LSB_set(const char *param);

ATEerror_t at_MOD_set(const char *param);

ATEerror_t at_MOD_get(const char *param);

ATEerror_t at_INTMOD1_set(const char *param);

ATEerror_t at_INTMOD1_get(const char *param);

ATEerror_t at_INTMOD2_set(const char *param);

ATEerror_t at_INTMOD2_get(const char *param);

ATEerror_t at_INTMOD3_set(const char *param);

ATEerror_t at_INTMOD3_get(const char *param);

ATEerror_t at_weightreset(const char *param);

ATEerror_t at_weight_GapValue_set(const char *param);

ATEerror_t at_weight_GapValue_get(const char *param);

ATEerror_t at_5Vtime_set(const char *param);

ATEerror_t at_5Vtime_get(const char *param);

ATEerror_t at_SETCNT_set(const char *param);

ATEerror_t at_getsensorvaule_set(const char *param);

ATEerror_t at_downlink_detect_set(const char *param);

ATEerror_t at_downlink_detect_get(const char *param);

ATEerror_t at_setmaxnbtrans_set(const char *param);

ATEerror_t at_setmaxnbtrans_get(const char *param);

ATEerror_t at_disdownlinkcheck_set(const char *param);

ATEerror_t at_disdownlinkcheck_get(const char *param);

ATEerror_t at_dismac_answer_set(const char *param);

ATEerror_t at_dismac_answer_get(const char *param);

ATEerror_t at_rxdata_test(const char *param);

#ifdef __cplusplus
}
#endif

#endif /* __AT_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
