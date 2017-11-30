/*******************************************************************************
  * @file    atcmd_modem.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    01-June-2017
  * @brief   Header for AT commands definition 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#ifndef __ATCMD_MODEM_H__
#define __ATCMD_MODEM_H__

#ifdef __cplusplus
 extern "C" {
#endif 



      /**********************************************************************/ 
      /*    AT commands specification for WM_SG_SM_42USI modem              */
      /*               - set of commands                    */
      /*               - return code error                                  */
      /**********************************************************************/


//#ifdef WM_SG_SM_42   /*USI modem*/

#ifdef AT_CMD_INDEX
/*
 * AT Command Index . In direct relationship with "CmdTab" static array
 * in atcmd.c file
 */
typedef enum ATCmd
{
 AT,                 /* OK*/
 
 AT_FWVERSION,       /* new one*/
 
 AT_RESET,           /* OK*/
 
 AT_BAND,            /* new one*/ 
 
 AT_JOIN,            /* same than Murata with additional parameter ABP or OTAA -> OK*/

 AT_NJS,             /*KO - there is not equivalent in USI*/
 
 AT_DEUI,            /*OK - USI equivalent EUI*/          
 
 AT_DADDR,           /*OK*/
 
 AT_APPKEY,          /*OK - USI equivalent AK*/
 
 AT_NWKSKEY,         /*OK - USI equivalent NSK*/
 
 AT_APPSKEY,       /*OK - USI equivalent ASK*/
 
 AT_APPEUI,          /*OK*/
 
 AT_ADR,             /*OK*/ 
 
 AT_TXP,             /*OK*/
 
 AT_DR,              /*OK*/
 
 AT_DCS,             /*OK - USI equivalent DC*/
 
 AT_PNM,             /*OK - USI equivalent NTYP*/  
 
 AT_RX2FQ,       /*???????  */
 
 AT_RX2DR,       /*OK*/
 
 AT_RX1DL,           /*OK - USI equivalent RX1DT*/
 
 AT_RX2DL,           /*OK - USI equivalent RX2DT*/
 
 AT_JN1DL,           /*OK - USI equivalent JRX1DT*/
 
 AT_JN2DL,           /*OK - USI equivalent JRX2DT*/
 
 AT_NJM,             /* is replaced by the combo join ?????? */
 
 AT_NWKID,           /*??????????*/
 AT_FCU,             /*?????????*/
 AT_FCD,             /*????????????*/
 
 AT_CLASS,            /*OK*/
 
/* AT_SENDB,*/            
 AT_SEND,             /*just one send mode- binary with scalar format port,data,ack*/   
 
 AT_TXT,              /*new one -- to send text. look like AT_SEND for murata*/
 
 AT_RECVB,            /* ??????????*/
 AT_RECV,     /*  ????????????*/
 AT_CFM,              /* include in the send command*/
 AT_CFS,              /*??????????????*/
 
 AT_BAT,                /*OK*/ 
 
 AT_RSSI,              /* OK*/
 
 AT_SNR,        /* OK*/
 
 AT_VER,               /*OK*/
 
 AT_WDCT,             /* new one*/
 
 AT_DEFMODE,          /* new one*/
 
 AT_WDG,              /* new one*/
 
 AT_ATE,              /*new one*/
 
 AT_SLEEP,            /*new one*/
 
 AT_PS,               /*new one*/    
 
 AT_END_AT
} ATCmd_t; 

#endif



#ifdef AT_CMD_STRING
/*list of AT string cmd supported by the USI LoRa modem*/
static char *CmdTab[] = { 
  {""},
  {"I"},           /* firmware version of USI loara module*/
  {"Z"},
  {"+BAND"},       /* +BAND country band - default 868 band*/
  
  {"+JOIN"},       /* +JOIN*/
  
  {"+NJS"},        /* +NJS --> ????*/
  
  {"+EUI"},       /* +EUI device ID*/
  
  {"+ADDR"},      /* +ADDR device Address*/
  
  {"+AK"},         /* +AK application key*/
  
  {"+NSK"},        /* +NSK Network session Key*/
  
  {"+ASK"},        /* +ASK application Session key*/
  
  {"+APPEUI"},     /* +APPEUI application Identifier*/
  
  {"+ADR"},        /* +ADR adaptive data rate*/
  
  {"+TXP"},        /* +TXP transmit Tx power*/
  
  {"+DR"},         /* +DR data rate*/
  
  {"+DC" },       /* +DC duty cycle settings*/
  
  {"+NTYP"},        /* +NTYP (replace+PNM) public network*/
  
  {"+RX2FQ"},      /* +RF2FQ Rx2 window frequency -->   ??????*/
  
  {"+RX2DR"},      /* +RX2DR data rate of Rx window*/
  
  {"+RX1DT"},      /* +RX1DT Delay of the Rx1 window*/
  
  {"+RX2DT"},      /* +RX2DT delay of the Rx2 window*/
  
  {"+JRX1DT"},      /* +JRX1DT Join delay on Rx Wind 1*/
  
  {"+JRX2DT"},      /* +JRX2DT Join delay on Rx Wind 2*/
  
  {"+NJM"},        /* +NJM Nwk Join Mode*/
  {"+NWKID"},      /* +NWKID Network ID */
  {"+FCU"},        /* +FCU uplink frame counter */
  {"+FCD"},        /* +FCD downlink frame counter */ 
  
  {"+CLASS"},      /* +CLASS LoRa class*/
  
/*  {"+SENDB"}, */     /* +SENDB send data binary format*/   /* sendB replaced by Send*/
  {"+SEND"},       /* +SEND send data in raw format*/    /* no sendb anymore with USI*/
  
  {"+TXT"} ,       /* +TXT  transmit text packet*/
  
  {"+RECVB"},      /* +RECVB received data in binary format*/
  {"+RECV"},       /* +RECV received data in raw format*/
  {"+CFM"},        /* +CFM  confirmation mode*/
  {"+CFS"},        /* +CFS  confirm status*/
  
  {"+BAT"},        /* +BAT  battery level*/
  
  {"+RSSI"},       /* +RSSI Signal strength indicator on received radio signal*/
  
  {"+SNR"},        /* +SNR  Signal to Noice ratio*/
  
  {"+VER"},         /* LoRaWAN version */
  
  {"+WDCT"},        /* for update the configuration table*/
  
  {"+DEFMODE"},     /* fro set the default operationmode - 6 = LoRA WAN mode*/
  
  {"+WDG"},         /* for enabling/disabling the watchdog*/
  
  {"E"},            /* for enabling/deisabling echo mode*/
  
  {"+SLEEP"},       /* for enter immediatly in sleep mode (slave) following the power control setting*/
  
  {"+PS"}           /* for reead or set the MCU power control (Slave)*/
};

#endif

#ifdef AT_ERROR_INDEX
/*
 * AT Command Index errors. In direct relationship with ATE_RetCode static array
 * in atcmd.c file
 */
typedef enum eATEerror
{
  AT_OK = 0,
  AT_ERROR_UNKNOW,                /* = -1 is USI  error code*/
  AT_ERROR_UNKNOW_COMMAND,        /* = -2   */
  AT_ERROR_LESS_ARGUMENTS,        /* = -3,  */
  AT_ERROR_MORE_ARGUMENETS,       /* = -4,  */
  AT_ERROR_INVALID_ARGUMENTS,     /* = -5,  */
  AT_ERROR_NOT_SUPPORTED,         /* = -6,  */
  AT_ERROR_OUT_OF_RANGE,          /* = -7,  */
  AT_ERROR_RX_TIMEOUT,            /* = -8,  */
  AT_ERROR_RX_ERROR,              /* = -9,  */
  AT_ERROR_TX_TIMEOUT,            /* = -10, */
  AT_ERROR_TX_ERROR,              /* = -11, */
  AT_ERROR_RF_BUSY,               /* = -12, */
  AT_ERROR_TIMEOUT,               /* = -13, */
  AT_ERROR_NO_ARGUMENETS_NEEDED,  /* = -14, */
  AT_ERROR_HAL_ERROR,             /* = -15, */ 
  AT_ERROR_INVALID_HEX_FORMAT,    /* = -16, */
  AT_ERROR_OUT_OF_ADDRESS,        /* = -17, */ 
  AT_ERROR_WAN_SEND,              /* = -100,*/
  AT_ERROR_WAN_GETPARAM,          /* = -101,*/
  AT_ERROR_WAN_SETPARAM,          /* = -102,*/
  AT_WAN_NON_JOINED,              /* = -103,*/ 
  AT_END_ERROR,
  AT_UART_LINK_ERROR,    /*additional return code to notify error on UART link*/
  AT_TEST_PARAM_OVERFLOW, /* additonal return code to be compatible whatevery the device modem*/
  AT_JOIN_SLEEP_TRANSITION, /*additional return code to manage the Join request transaction*/
} ATEerror_t;

#endif

#ifdef AT_ERROR_STRING
/*RetCode used to compare the return code from modem*/
static ATE_RetCode_t ATE_RetCode[] = {
  {{"OK\r\n"},{sizeof("OK\r\n")},{AT_OK}},
  {{"ERROR_UNKNOW\r"},{sizeof("ERROR_UNKNOW\r")},{AT_ERROR_UNKNOW}},
  {{"\rERROR_UNKNOW_COMMAND\r"},{sizeof("\rERROR_UNKNOW_COMMAND\r")},{AT_ERROR_UNKNOW_COMMAND}},  
  {{"ERROR_LESS_ARGUMENETS\r\n"},{sizeof("ERROR_LESS_ARGUMENETS\r\n")},{AT_ERROR_LESS_ARGUMENTS}},
  {{"ERROR_MORE_ARGUMENETS\r"},{sizeof("ERROR_MORE_ARGUMENETS\r")},{AT_ERROR_MORE_ARGUMENETS}},  
  {{"ERROR_INVALID_ARGUMENTS\r"},{sizeof("ERROR_INVALID_ARGUMENTS\r")},{AT_ERROR_INVALID_ARGUMENTS}},
  {{"AT_ERROR_NOT_SUPPORTED\r"},{sizeof("AT_ERROR_NOT_SUPPORTED\r")},{AT_ERROR_NOT_SUPPORTED}},  
  {{"ERROR_OUT_OF_RANGE\r"},{sizeof("ERROR_OUT_OF_RANGE\r")},{AT_ERROR_OUT_OF_RANGE}},
  {{"ERROR_RX_TIMEOUT\r"},{sizeof("ERROR_RX_TIMEOUT\r")},{AT_ERROR_RX_TIMEOUT}},
  {{"ERROR_RX_ERROR\r"},{sizeof("ERROR_RX_ERROR\r")},{AT_ERROR_RX_ERROR}},  
  {{"ERROR_TX_TIMEOUT\r"},{sizeof("ERROR_TX_TIMEOUT\r")},{AT_ERROR_TX_TIMEOUT}},
  {{"ERROR_TX_ERROR\r"},{sizeof("ERROR_TX_ERROR\r")},{AT_ERROR_TX_ERROR}},
  {{"ERROR_RF_BUSY\r"},{sizeof("ERROR_RF_BUSY\r")},{AT_ERROR_RF_BUSY}},  
  {{"ERROR_TIMEOUT\r"},{sizeof("ERROR_TIMEOUT\r")},{AT_ERROR_TIMEOUT}},
  {{"ERROR_NO_ARGUMENETS_NEEDED\r"},{sizeof("ERROR_NO_ARGUMENETS_NEEDED\r")},{AT_ERROR_NO_ARGUMENETS_NEEDED}},
  {{"AT_ERROR_HAL_ERROR\r"},{sizeof("AT_ERROR_HAL_ERROR\r")},{AT_ERROR_HAL_ERROR}},  
  {{"ERROR_INVALID_HEX_FORMAT\r"},{sizeof("ERROR_INVALID_HEX_FORMAT\r")},{AT_ERROR_INVALID_HEX_FORMAT}},
  {{"ERROR_OUT_OF_ADDRESS\r"},{sizeof("ERROR_OUT_OF_ADDRESS\r")},{AT_ERROR_OUT_OF_ADDRESS}},  
  {{"ERROR_WAN_SEND\r"},{sizeof("ERROR_WAN_SEND\r")},{AT_ERROR_WAN_SEND}}, 
  {{"ERROR_WAN_GETPARAM\r"},{sizeof("ERROR_WAN_GETPARAM\r")},{AT_ERROR_WAN_GETPARAM}}, 
  {{"ERROR_WAN_SETPARAM\r"},{sizeof("ERROR_WAN_SETPARAM\r")},{AT_ERROR_WAN_SETPARAM}}, 
  {{"ERROR_WAN_NON_JOINED\r"},{sizeof("ERROR_WAN_NON_JOINED\r")},{AT_WAN_NON_JOINED}},   
  {{"unknown error\r"},{sizeof("unknown error\r")},{AT_END_ERROR}}};

#endif


#ifdef AT_CMD_MARKER  
/* Marker to design the AT command string*/  
#define AT_HEADER       "AT"
#define AT_SET_MARKER   "="
#define AT_GET_MARKER   ""
#define AT_NULL_MARKER  ""
#define AT_COLON        ":" 
#define AT_COMMA        ","
#define AT_TAIL         "\r"
#define AT_SEPARATOR    "" 
#define AT_FRAME_KEY  "%hhx,%hhx,%hhx,%hhx,%hhx,%hhx,%hhx,%hhx,%hhx,%hhx,%hhx,%hhx,%hhx,%hhx,%hhx,%hhx"
#define AT_FRAME_KEY_OFFSET  2
  
#endif

//#endif /*endif WM_SG_SM_42*/    


#ifdef __cplusplus
}
#endif

#endif /*__ATCMD_MODEM_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
