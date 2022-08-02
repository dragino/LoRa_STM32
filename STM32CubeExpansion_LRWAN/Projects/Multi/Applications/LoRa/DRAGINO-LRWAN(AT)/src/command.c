/*******************************************************************************
 * @file    command.c
 * @author  MCD Application Team
 * @version V1.1.4
 * @date    10-July-2018
 * @brief   main command driver dedicated to command AT
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
#include <stdlib.h>
#include "at.h"
#include "hw.h"
#include "command.h"
#include "lora.h"
#include "delay.h"

/* comment the following to have help message */
/* #define NO_HELP */
/* #define NO_KEY_ADDR_EUI */

/* Private typedef -----------------------------------------------------------*/
/**
 * @brief  Structure defining an AT Command
 */
struct ATCommand_s {
  const char *string;                       /*< command string, after the "AT" */
  const int size_string;                    /*< size of the command string, not including the final \0 */
  ATEerror_t (*get)(const char *param);     /*< =? after the string to get the current value*/
  ATEerror_t (*set)(const char *param);     /*< = (but not =?\0) after the string to set a value */
  ATEerror_t (*run)(const char *param);     /*< \0 after the string - run the command */
#if !defined(NO_HELP)
  const char *help_string;                  /*< to be printed when ? after the string */
#endif
};

/* Private define ------------------------------------------------------------*/
#define CMD_SIZE 128
extern uint8_t parse_flag;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/**
 * @brief  Array corresponding to the description of each possible AT Error
 */
static const char *const ATError_description[] =
{
  "\r\nOK\r\n",                     /* AT_OK */
  "\r\nAT_ERROR\r\n",               /* AT_ERROR */
  "\r\nAT_PARAM_ERROR\r\n",         /* AT_PARAM_ERROR */
  "\r\nERROR:Not in Range\r\n",     /* AT_PARAM_NOT in Range */	
  "\r\nERROR:Run AT+FDR first\r\n", /* AT_PARAM_FDR */		
  "\r\nAT_BUSY_ERROR\r\n",          /* AT_BUSY_ERROR */
  "\r\nAT_TEST_PARAM_OVERFLOW\r\n", /* AT_TEST_PARAM_OVERFLOW */
  "\r\nAT_NO_NETWORK_JOINED\r\n",   /* AT_NO_NET_JOINED */
  "\r\nAT_RX_ERROR\r\n",            /* AT_RX_ERROR */
  "\r\nerror unknown\r\n",          /* AT_MAX */
};

/**
 * @brief  Array of all supported AT Commands
 */
static const struct ATCommand_s ATCommand[] =
{
 {
    .string = AT_DEBUG,
    .size_string = sizeof(AT_DEBUG) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_DEBUG ":Set more info input\r\n",
#endif
    .get = at_return_error,
    .set = at_return_error,
    .run = at_DEBUG_run,
  },
	
  {
    .string = AT_RESET,
    .size_string = sizeof(AT_RESET) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_RESET ": Trig a reset of the MCU\r\n",
#endif
    .get = at_return_error,
    .set = at_return_error,
    .run = at_reset,
  },
	
	{
	  .string = AT_FDR,
    .size_string = sizeof(AT_FDR) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_FDR ": Reset Parameters to Factory Default, Keys Reserve\r\n",
#endif
    .get = at_return_error,
    .set = at_return_error,
    .run = at_FDR,
  },

#ifndef NO_KEY_ADDR_EUI
  {
    .string = AT_DEUI,
    .size_string = sizeof(AT_DEUI) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_DEUI ": Get or Set the Device EUI\r\n",
#endif
    .get = at_DevEUI_get,
    .set = at_DevEUI_set,
    .run = at_return_error,
  },
#endif

#ifndef NO_KEY_ADDR_EUI
  {
    .string = AT_APPEUI,
    .size_string = sizeof(AT_APPEUI) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_APPEUI ": Get or Set the Application EUI\r\n",
#endif
    .get = at_AppEUI_get,
    .set = at_AppEUI_set,
    .run = at_return_error,
  },
#endif
  
#ifndef NO_KEY_ADDR_EUI
  {
    .string = AT_APPKEY,
    .size_string = sizeof(AT_APPKEY) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_APPKEY ": Get or Set the Application Key\r\n",
#endif
    .get = at_AppKey_get,
    .set = at_AppKey_set,
    .run = at_return_error,
  },
#endif
  
#ifndef NO_KEY_ADDR_EUI
  {
    .string = AT_DADDR,
    .size_string = sizeof(AT_DADDR) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_DADDR ": Get or Set the Device Address\r\n",
#endif
    .get = at_DevAddr_get,
    .set = at_DevAddr_set,
    .run = at_return_error,
  },
#endif
  
#ifndef NO_KEY_ADDR_EUI
  {
    .string = AT_NWKSKEY,
    .size_string = sizeof(AT_NWKSKEY) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_NWKSKEY ": Get or Set the Network Session Key\r\n",
#endif
    .get = at_NwkSKey_get,
    .set = at_NwkSKey_set,
    .run = at_return_error,
  },
#endif
  
#ifndef NO_KEY_ADDR_EUI
  {
    .string = AT_APPSKEY,
    .size_string = sizeof(AT_APPSKEY) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_APPSKEY ": Get or Set the Application Session Key\r\n",
#endif
    .get = at_AppSKey_get,
    .set = at_AppSKey_set,
    .run = at_return_error,
  },
#endif

  {
    .string = AT_ADR,
    .size_string = sizeof(AT_ADR) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_ADR ": Get or Set the Adaptive Data Rate setting. (0: off, 1: on)\r\n",
#endif
    .get = at_ADR_get,
    .set = at_ADR_set,
    .run = at_return_error,
  },
  
  {
    .string = AT_TXP,
    .size_string = sizeof(AT_TXP) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_TXP ": Get or Set the Transmit Power (0-5, MAX:0, MIN:5, according to LoRaWAN Spec)\r\n",
#endif
    .get = at_TransmitPower_get,
    .set = at_TransmitPower_set,
    .run = at_return_error,
  },

  {
    .string = AT_DR,
    .size_string = sizeof(AT_DR) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_DR ": Get or Set the Data Rate. (0-7 corresponding to DR_X)\r\n",
#endif
    .get = at_DataRate_get,
    .set = at_DataRate_set,
    .run = at_return_error,
  },

  {
    .string = AT_DCS,
    .size_string = sizeof(AT_DCS) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_DCS ": Get or Set the ETSI Duty Cycle setting - 0=disable, 1=enable - Only for testing\r\n",
#endif
    .get = at_DutyCycle_get,
    .set = at_DutyCycle_set,
    .run = at_return_error,
  },

  {
    .string = AT_PNM,
    .size_string = sizeof(AT_PNM) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_PNM ": Get or Set the public network mode. (0: off, 1: on)\r\n",
#endif
    .get = at_PublicNetwork_get,
    .set = at_PublicNetwork_set,
    .run = at_return_error,
  },

  {
    .string = AT_RX2FQ,
    .size_string = sizeof(AT_RX2FQ) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_RX2FQ ": Get or Set the Rx2 window frequency\r\n",
#endif
    .get = at_Rx2Frequency_get,
    .set = at_Rx2Frequency_set,
    .run = at_return_error,
  },

  {
    .string = AT_RX2DR,
    .size_string = sizeof(AT_RX2DR) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_RX2DR ": Get or Set the Rx2 window data rate (0-7 corresponding to DR_X)\r\n",
#endif
    .get = at_Rx2DataRate_get,
    .set = at_Rx2DataRate_set,
    .run = at_return_error,
  },

  {
    .string = AT_RX1DL,
    .size_string = sizeof(AT_RX1DL) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_RX1DL ": Get or Set the delay between the end of the Tx and the Rx Window 1 in ms\r\n",
#endif
    .get = at_Rx1Delay_get,
    .set = at_Rx1Delay_set,
    .run = at_return_error,
  },

  {
    .string = AT_RX2DL,
    .size_string = sizeof(AT_RX2DL) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_RX2DL ": Get or Set the delay between the end of the Tx and the Rx Window 2 in ms\r\n",
#endif
    .get = at_Rx2Delay_get,
    .set = at_Rx2Delay_set,
    .run = at_return_error,
  },

  {
    .string = AT_JN1DL,
    .size_string = sizeof(AT_JN1DL) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_JN1DL ": Get or Set the Join Accept Delay between the end of the Tx and the Join Rx Window 1 in ms\r\n",
#endif
    .get = at_JoinAcceptDelay1_get,
    .set = at_JoinAcceptDelay1_set,
    .run = at_return_error,
  },

  {
    .string = AT_JN2DL,
    .size_string = sizeof(AT_JN2DL) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_JN2DL ": Get or Set the Join Accept Delay between the end of the Tx and the Join Rx Window 2 in ms\r\n",
#endif
    .get = at_JoinAcceptDelay2_get,
    .set = at_JoinAcceptDelay2_set,
    .run = at_return_error,
  },

  {
    .string = AT_NJM,
    .size_string = sizeof(AT_NJM) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_NJM ": Get or Set the Network Join Mode. (0: ABP, 1: OTAA)\r\n",
#endif
    .get = at_NetworkJoinMode_get,
    .set = at_NetworkJoinMode_set,
    .run = at_return_error,
  },

#ifndef NO_KEY_ADDR_EUI
  {
    .string = AT_NWKID,
    .size_string = sizeof(AT_NWKID) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_NWKID ": Get or Set the Network ID\r\n",
#endif
    .get = at_NetworkID_get,
    .set = at_NetworkID_set,
    .run = at_return_error,
  },
#endif
  
  {
    .string = AT_FCU,
    .size_string = sizeof(AT_FCU) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_FCU ": Get or Set the Frame Counter Uplink\r\n",
#endif
    .get = at_UplinkCounter_get,
    .set = at_UplinkCounter_set,
    .run = at_return_error,
  },

  {
    .string = AT_FCD,
    .size_string = sizeof(AT_FCD) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_FCD ": Get or Set the Frame Counter Downlink\r\n",
#endif
    .get = at_DownlinkCounter_get,
    .set = at_DownlinkCounter_set,
    .run = at_return_error,
  },

  {
    .string = AT_CLASS,
    .size_string = sizeof(AT_CLASS) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_CLASS ": Get or Set the Device Class\r\n",
#endif
    .get = at_DeviceClass_get,
    .set = at_DeviceClass_set,
    .run = at_return_error,
  },

  {
    .string = AT_JOIN,
    .size_string = sizeof(AT_JOIN) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_JOIN ": Join network\r\n",
#endif
    .get = at_return_error,
    .set = at_return_error,
    .run = at_Join,
  },

  {
    .string = AT_NJS,
    .size_string = sizeof(AT_NJS) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_NJS ": Get the join status\r\n",
#endif
    .get = at_NetworkJoinStatus,
    .set = at_return_error,
    .run = at_return_error,
  },

  {
    .string = AT_SENDB,
    .size_string = sizeof(AT_SENDB) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_SENDB ": Send hexadecimal data along with the application port\r\n",
#endif
    .get = at_return_error,
    .set = at_SendBinary,
    .run = at_return_error,
  },

  {
    .string = AT_SEND,
    .size_string = sizeof(AT_SEND) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_SEND ": Send text data along with the application port\r\n",
#endif
    .get = at_return_error,
    .set = at_Send,
    .run = at_return_error,
  },

  {
    .string = AT_RECVB,
    .size_string = sizeof(AT_RECVB) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_RECVB ": Print last received data in binary format (with hexadecimal values)\r\n",
#endif
    .get = at_ReceiveBinary,
    .set = at_return_error,
    .run = at_ReceiveBinary,
  },

  {
    .string = AT_RECV,
    .size_string = sizeof(AT_RECV) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_RECV ": Print last received data in raw format\r\n",
#endif
    .get = at_Receive,
    .set = at_return_error,
    .run = at_Receive,
  },

	#if defined ( REGION_AU915 ) || defined ( REGION_AS923 )	
	{
    .string = AT_DWELLT,
    .size_string = sizeof(AT_DWELLT) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_DWELLT ": Get or set UplinkDwellTime\r\n",
#endif
    .get = at_DwellTime_get,
    .set = at_DwellTime_set,
    .run = at_return_error,
  },
#endif

	{
    .string = AT_RJTDC,
    .size_string = sizeof(AT_RJTDC) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_RJTDC ": Get or set the ReJoin data transmission interval in min\r\n",
#endif
    .get = at_RJTDC_get,
    .set = at_RJTDC_set,
    .run = at_return_error,
  },
	
	{
    .string = AT_RPL,
    .size_string = sizeof(AT_RPL) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_RPL ": Get or set response level\r\n",
#endif
    .get = at_RPL_get,
    .set = at_RPL_set,
    .run = at_return_error,
  },
	
  {
    .string = AT_VER,
    .size_string = sizeof(AT_VER) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_VER ": Get current image version and Frequency Band\r\n",
#endif
    .get = at_version_get,
    .set = at_return_error,
    .run = at_return_error,
  },
  
  {
    .string = AT_CFM,
    .size_string = sizeof(AT_CFM) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_CFM ": Get or Set the confirmation mode (0-1)\r\n",
#endif
    .get = at_ack_get,
    .set = at_ack_set,
    .run = at_return_error,
  },
  
  {
    .string = AT_CFS,
    .size_string = sizeof(AT_CFS) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_CFS ": Get confirmation status of the last AT+SEND (0-1)\r\n",
#endif
    .get = at_isack_get,
    .set = at_return_error,
    .run = at_return_error,
  },
  
  {
    .string = AT_SNR,
    .size_string = sizeof(AT_SNR) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_SNR ": Get the SNR of the last received packet\r\n",
#endif
    .get = at_snr_get,
    .set = at_return_error,
    .run = at_return_error,
  },
  
  {
    .string = AT_RSSI,
    .size_string = sizeof(AT_RSSI) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_RSSI ": Get the RSSI of the last received packet\r\n",
#endif
    .get = at_rssi_get,
    .set = at_return_error,
    .run = at_return_error,
  },

	{
    .string = AT_TDC,
    .size_string = sizeof(AT_TDC) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_TDC ": Get or set the application data transmission interval in ms\r\n",
#endif
    .get = at_TDC_get,
    .set = at_TDC_set,
    .run = at_return_error,
  },
	
	{
    .string = AT_PORT,
    .size_string = sizeof(AT_PORT) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_PORT ": Get or set the application port\r\n",
#endif
    .get = at_application_port_get,
    .set = at_application_port_set,
    .run = at_return_error,
  },
	
  {
	  .string = AT_RX1WTO,
    .size_string = sizeof(AT_RX1WTO) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_RX1WTO ": Get or Set the number of symbols to detect and timeout from RXwindow1(0 to 255)\r\n",
#endif
    .get = at_symbtimeout1LSB_get,
    .set = at_symbtimeout1LSB_set,
    .run = at_return_error,
	},
	
	{
	  .string = AT_RX2WTO,
    .size_string = sizeof(AT_RX2WTO) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_RX2WTO ": Get or Set the number of symbols to detect and timeout from RXwindow2(0 to 255)\r\n",
#endif
    .get = at_symbtimeout2LSB_get,
    .set = at_symbtimeout2LSB_set,
    .run = at_return_error,
	 },
	
	{
	  .string = AT_DECRYPT,
    .size_string = sizeof(AT_DECRYPT) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_DECRYPT ": Get or Set the Decrypt the uplink payload(0:Disable,1:Enable)\r\n",
#endif
    .get = at_decrypt_get,
    .set = at_decrypt_set,
    .run = at_return_error,
	 },
	
		{
	  .string = AT_MOD,
    .size_string = sizeof(AT_MOD) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_MOD ":Get or Set the work mode\r\n",
#endif
    .get = at_MOD_get,
    .set = at_MOD_set,
    .run = at_return_error,
	},

		{
	  .string = AT_INTMOD1,
    .size_string = sizeof(AT_INTMOD1) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_INTMOD1 ":Get or Set the trigger interrupt mode for PB14(0:Disable,1:falling or rising,2:falling,3:rising)\r\n",
#endif
    .get = at_INTMOD1_get,
    .set = at_INTMOD1_set,
    .run = at_return_error,
	},

		{
	  .string = AT_INTMOD2,
    .size_string = sizeof(AT_INTMOD2) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_INTMOD2 ":Get or Set the trigger interrupt mode for PB15(0:Disable,1:falling or rising,2:falling,3:rising)\r\n",
#endif
    .get = at_INTMOD2_get,
    .set = at_INTMOD2_set,
    .run = at_return_error,
	},
		
		{
	  .string = AT_INTMOD3,
    .size_string = sizeof(AT_INTMOD3) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_INTMOD3 ":Get or Set the trigger interrupt mode for PA4(0:Disable,1:falling or rising,2:falling,3:rising)\r\n",
#endif
    .get = at_INTMOD3_get,
    .set = at_INTMOD3_set,
    .run = at_return_error,
	},

		{
	  .string = AT_WEIGRE,
    .size_string = sizeof(AT_WEIGRE) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_WEIGRE ":Set the weight to 0g\r\n",
#endif
    .get = at_return_error,
    .set = at_return_error,
    .run = at_weightreset,
	},

		{
	  .string = AT_WEIGAP,
    .size_string = sizeof(AT_WEIGAP) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_WEIGAP ":Get or Set the GapValue of weight\r\n",
#endif
    .get = at_weight_GapValue_get,
    .set = at_weight_GapValue_set,
    .run = at_return_error,
	},

		{
	  .string = AT_5VT,
    .size_string = sizeof(AT_5VT) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_5VT ":Get or Set extend the time of 5V power\r\n",
#endif
    .get = at_5Vtime_get,
    .set = at_5Vtime_set,
    .run = at_return_error,
	},

		{
	  .string = AT_SETCNT,
    .size_string = sizeof(AT_SETCNT) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_SETCNT ": Get or set the count at present\r\n",
#endif
    .get = at_return_error,
    .set = at_SETCNT_set,
    .run = at_return_error,
	},	
		
	{
	  .string = AT_CHS,
    .size_string = sizeof(AT_CHS) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_CHS ": Get or Set Frequency (Unit: Hz) for Single Channel Mode\r\n",
#endif
    .get = at_CHS_get,
    .set = at_CHS_set,
    .run = at_return_error,
	},
	
	#if defined( REGION_US915 ) || defined ( REGION_AU915 ) || defined ( REGION_CN470 )
	{
	  .string = AT_CHE,
    .size_string = sizeof(AT_CHE) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_CHE ": Get or Set eight channels mode,Only for US915,AU915,CN470\r\n",
#endif
    .get = at_CHE_get,
    .set = at_CHE_set,
    .run = at_return_error,
	},
	#endif	

	{
	  .string = AT_GETSENSORVALUE,
    .size_string = sizeof(AT_GETSENSORVALUE) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_GETSENSORVALUE ": Returns the current sensor value\r\n",
#endif
    .get = at_return_error,
    .set = at_getsensorvaule_set,
    .run = at_return_error,
	},
	
	{
    .string = AT_DDETECT,
    .size_string = sizeof(AT_DDETECT) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_DDETECT ": Get or set the downlink detection\r\n",
#endif
    .get = at_downlink_detect_get,
    .set = at_downlink_detect_set,
    .run = at_return_error,
  },
	
	{
    .string = AT_SETMAXNBTRANS,
    .size_string = sizeof(AT_SETMAXNBTRANS) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_SETMAXNBTRANS ": Get or set the max nbtrans in LinkADR\r\n",
#endif
    .get = at_setmaxnbtrans_get,
    .set = at_setmaxnbtrans_set,
    .run = at_return_error,
  },

		{
	  .string = AT_DISFCNTCHECK,
    .size_string = sizeof(AT_DISFCNTCHECK) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_DISFCNTCHECK ": Get or set the Downlink Frame Check(0:Enable,1:Disable)\r\n",
#endif
    .get = at_disdownlinkcheck_get,
    .set = at_disdownlinkcheck_set,
    .run = at_return_error,
	},	

		{
	  .string = AT_DISMACANS,
    .size_string = sizeof(AT_DISMACANS) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_DISMACANS ": Get or set the MAC ANS switch(0:Enable,1:Disable)\r\n",
#endif
    .get = at_dismac_answer_get,
    .set = at_dismac_answer_set,
    .run = at_return_error,
	},	

		{
	  .string = AT_RXDATEST,
    .size_string = sizeof(AT_RXDATEST) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_RXDATEST ": Set the downlink data detection\r\n",
#endif
    .get = at_return_error,
    .set = at_rxdata_test,
    .run = at_return_error,
	},	
		
	{
	  .string = AT_CFG,
    .size_string = sizeof(AT_CFG) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_CFG ": Print all configurations\r\n",
#endif
    .get = at_return_error,
    .set = at_return_error,
    .run = at_CFG_run,
	},
};


/* Private function prototypes -----------------------------------------------*/

/**
 * @brief  Print a string corresponding to an ATEerror_t
 * @param  The AT error code
 * @retval None
 */
static void com_error(ATEerror_t error_type);

/**
 * @brief  Parse a command and process it
 * @param  The command
 * @retval None
 */
static void parse_cmd(const char *cmd);

/* Exported functions ---------------------------------------------------------*/
static void CMD_GetChar( uint8_t* rxChar);
static char command[CMD_SIZE];
static unsigned i = 0;  
static FlagStatus IsCharReceived=RESET;  

void CMD_Init(void)
{
  vcom_ReceiveInit( CMD_GetChar );
  IsCharReceived=RESET;
}

static void CMD_GetChar( uint8_t* rxChar)
{
  command[i] = *rxChar;
  IsCharReceived=SET;
}

void CMD_Process(void)
{
/* Process all commands */
  if (IsCharReceived==SET)
  {
    //ENTER_CRITICAL
    IsCharReceived=RESET;
    //EXIT CRITICAL
#if 0 /* echo On    */
  PRINTF("%c", command[i]);
#endif

    if (command[i] == AT_ERROR_RX_CHAR)
    {
      i = 0;
      com_error(AT_RX_ERROR);
    }
    else if ((command[i] == '\r') || (command[i] == '\n'))
    {
      if (i != 0)
      {
        command[i] = '\0';
				i = 0;				
        parse_cmd(command);
      }
    }
    else if (i == (CMD_SIZE - 1))
    {
      i = 0;
      com_error(AT_TEST_PARAM_OVERFLOW);
    }
    else
    {
      i++;
    }
  }
}

/* Private functions ---------------------------------------------------------*/

static void com_error(ATEerror_t error_type)
{
  if (error_type > AT_MAX)
  {
    error_type = AT_MAX;
  }
  AT_PRINTF(ATError_description[error_type]);
}


static void parse_cmd(const char *cmd)
{
	uint8_t store_config_status=1;
  ATEerror_t status = AT_OK;
  const struct ATCommand_s *Current_ATCommand;
  int i;

  if ((cmd[0] != 'A') || (cmd[1] != 'T'))
  {
    status = AT_ERROR;
  }
  else
  if (cmd[2] == '\0')
  {
    /* status = AT_OK; */
  }
  else
  if (cmd[2] == '?')
  {
#ifdef NO_HELP
#else
    PPRINTF("AT+<CMD>?        : Help on <CMD>\r\n"
              "AT+<CMD>         : Run <CMD>\r\n"
              "AT+<CMD>=<value> : Set the value\r\n"
              "AT+<CMD>=?       : Get the value\r\n");
    for (i = 0; i < (sizeof(ATCommand) / sizeof(struct ATCommand_s)); i++)
    {
      PPRINTF(ATCommand[i].help_string);
    }
#endif
  }
  else
  {
    /* point to the start of the command, excluding AT */
    status = AT_ERROR;
    cmd += 2;
    for (i = 0; i < (sizeof(ATCommand) / sizeof(struct ATCommand_s)); i++)
    {
      if (strncmp(cmd, ATCommand[i].string, ATCommand[i].size_string) == 0)
      {
        Current_ATCommand = &(ATCommand[i]);
        /* point to the string after the command to parse it */
        cmd += Current_ATCommand->size_string;

        /* parse after the command */
        switch (cmd[0])
        {
          case '\0':    /* nothing after the command */
            status = Current_ATCommand->run(cmd);
            break;
          case '=':
            if ((cmd[1] == '?') && (cmd[2] == '\0'))
            {
              status = Current_ATCommand->get(cmd + 1);
            }
            else
            {
              status = Current_ATCommand->set(cmd + 1);
							cmd=Current_ATCommand->string;
							
							if((strcmp(cmd,AT_DEUI)==0)||(strcmp(cmd,AT_APPEUI)==0)||(strcmp(cmd,AT_APPKEY)==0)
								||(strcmp(cmd,AT_DADDR)==0)||(strcmp(cmd,AT_NWKSKEY)==0)||(strcmp(cmd,AT_APPSKEY)==0))
							{
								store_config_status=2;
							}
	
							if((strcmp(cmd,AT_SENDB)==0)||(strcmp(cmd,AT_SEND)==0)||(strcmp(cmd,AT_RXDATEST)==0)||(strcmp(cmd,AT_SETCNT)==0)||(strcmp(cmd,AT_GETSENSORVALUE)==0))
							{
								store_config_status=0;
							}
							
							if(store_config_status>0)
							{
								if(status==AT_OK)
								{
									if(store_config_status==1)
									{
										EEPROM_Store_Config();
									}
									else if(store_config_status==2)
									{
										EEPROM_Store_key();
									}
								}
							}
            }
            break;
          case '?':
#ifndef NO_HELP
            AT_PRINTF(Current_ATCommand->help_string);
#endif
            status = AT_OK;
            break;
          default:
            /* not recognized */
            break;
        }

        /* we end the loop as the command was found */
        break;
		 }
    }
  }

  com_error(status);
}

uint8_t printf_all_config(void)
{
	char *cmd;
	for (int j = 0; j< (sizeof(ATCommand) / sizeof(struct ATCommand_s)); j++)
	{
		if((ATCommand[j].get)!=at_return_error)
		{
			PPRINTF("AT%s=",ATCommand[j].string);
			ATCommand[j].get(( char *)cmd);		
			HAL_Delay(50);				
		}
	}
	
	return 1;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
