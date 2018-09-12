/*******************************************************************************
  * @file    atcmd_modem.h
  * @author  MCD Application Team
  * @version V1.0.3
  * @date    20-December-2017
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

#ifdef AT_CMD_INDEX
/*
 * AT Command Index . In direct relationship with "CmdTab" static array
 * in atcmd.c file
 */
typedef enum
{
  AT,
  AT_RESET,
  AT_BAND,
  AT_JOIN,
  AT_NJS,         /*KO*/
  AT_DEUI,        /*AT+ID=DEVEUI,*/
  AT_DADDR,       /*AT+ID=DEVADDR,*/
  AT_APPKEY,
  AT_NWKSKEY,
  AT_APPSKEY,
  AT_APPEUI,
  AT_ADR,
  AT_TXP,         /*TX power*/
  AT_DR,
  AT_DCS,         /*DutyCycle settings*/
  AT_PNM,         /*Public network mode*/
  AT_RX2FQ,       /*KO, include in the RX2 command*/
  AT_RX2DR,       /*KO, include in the RX2 command*/
  AT_RX1DL,
  AT_RX2DL,
  AT_JN1DL,
  AT_JN2DL,
  AT_NJM,
  AT_NWKID,       /*KO*/
  AT_FCU,         /*KO*/
  AT_FCD,         /*KO*/
  AT_CLASS,
  AT_SENDB,
  AT_SEND,
  AT_RECVB,       /*KO*/
  AT_RECV,        /*KO*/
  AT_CFM,         /*include in the send command*/
  AT_CFS,         /*KO*/
  AT_BAT,
  AT_RSSI,        /*KO, include in the received data frame*/
  AT_SNR,         /*KO, include in the received data frame*/
  AT_VER,
  AT_WDCT,        /*KO*/
  AT_DEFMODE,      /*KO*/
  AT_WDG,
  AT_ATE,         /*KO*/
  AT_SLEEP,
  AT_PS,          /*KO*/
  AT_RX2,         /*Combine AT_RX2FQ with AT_RX2DR*/
  AT_FDEFAULT,
  AT_CMSG,
  AT_CMSGHEX,
  AT_PORT,
  AT_LEN,
  AT_VDD,
  AT_MAX,         /* Last command for searching, not real command */
}ATCmd_t;

#endif

#ifdef AT_CMD_STRING
/*list of AT string cmd supported by the RisingHF LoRa modem*/
static char *CmdTab[]
#ifdef __GNUC__
__attribute__ ((unused))
#endif
					= {
  "",

  "+RESET",

  "+DR",              /* +BAND country band */

  "+JOIN",            /* +JOIN*/

  "+NJS",             /* KO */

  "+ID=DevEui",       /* +ID device ID*/

  "+ID=DevAddr",      /* +ID device Address*/

  "+KEY=APPKEY",      /* +KEY application key*/

  "+KEY=NWKKEY",      /* +KEY Network session Key*/

  "+KEY=APPSKEY",     /* +KEY application Session key*/

  "+ID=APPEUI",       /* +ID application Identifier*/

  "+ADR",             /* +ADR adaptive data rate*/

  "+POWER",           /* +POWER transmit Tx power*/

  "+DR",              /* +DR data rate*/

  "+LW=DC" ,          /* +LW duty cycle settings*/

  "+LW=NET",          /* +LW public network or private network*/

  "+RX2FQ",           /* KO */

  "+RX2DR",           /* KO */

  "+DELAY=RX1",       /* +DELAY Delay of the Rx1 window*/

  "+DELAY=RX2",       /* +DELAY delay of the Rx2 window*/

  "+DELAY=JRX1",      /* +DELAY Join delay on Rx Wind 1*/

  "+DELAY=JRX2",      /* +DELAY Join delay on Rx Wind 2*/

  "+MODE",            /* +MODE Nwk Join Mode*/

  "+NWKID",           /* KO */

  "+FCU",             /* KO */

  "+FCD",             /* KO */

  "+CLASS",           /* +CLASS LoRa class*/

  "+MSGHEX",          /* +MSGHEX send data binary format*/

  "+MSG",             /* +MSG send data in raw format*/

  "+RECVB",           /* KO */

  "+RECV",            /* KO */

  "+CFM",             /* KO, include in the data frame*/

  "+CFS",             /* KO */

  "+LW=BAT",          /* +LW  battery level*/

  "+RSSI",            /* KO */

  "+SNR",             /* KO */

  "+VER",             /* Firmware version */

  "+WDCT",            /* KO */

  "+DEFMODE",         /* KO */

  "+WDT",

  "E",                /* KO */

  "+LOWPOWER",        /* for enter immediatly in sleep mode (slave) following the power control setting*/

  "+PS",              /* KO*/

  "+RXWIN2",          /* +RXWIN2, RX2 windouw */

  "+FDEFAULT",        /* Set the module default status */

  "+CMSG",            /* Send the confirmed data frame */

  "+CMSGHEX",         /* Send the confirmed HEX data frame  */

  "+PORT",            /* Set the port in sending data */

  "+LW=LEN",

  "+VDD"              /* Get the vdd */
};

#endif

#ifdef AT_ERROR_INDEX
/*
 * AT Command Index errors. In direct relationship with ATE_RetCode static array
 * in atcmd.c file
 */
typedef enum
{
  ATCTL_RET_TIMEOUT = -3,         /* RX data timeout */
  ATCTL_RET_ERR = -2,             /* Unknown command */
  ATCTL_RET_CMD_ERR = -1,         /* Get command +CMD: ERROR(x) */
  ATCTL_RET_IDLE = 0,
  ATCTL_RET_CMD_OK,               /* Command is OK, but can't parse */
  ATCTL_RET_CMD_AT,
  ATCTL_RET_CMD_ID,
  ATCTL_RET_CMD_VER,
  ATCTL_RET_CMD_RTC,
  ATCTL_RET_CMD_VDD,
  ATCTL_RET_CMD_MSG,
  ATCTL_RET_CMD_JOIN,
  ATCTL_RET_CMD_EEPROM,
  ATCTL_RET_CMD_DR,
  ATCTL_RET_CMD_MODE,
  ATCTL_RET_CMD_LW,
  ATCTL_RET_CMD_DELAY,
}atctl_ret_t,ATEerror_t;

#endif

#ifdef AT_CMD_MARKER
/* Marker to design the AT command string*/
#define AT_HEADER       "AT"
#define AT_SET_MARKER   "="
#define AT_GET_MARKER   ""
#define AT_NULL_MARKER  ""
#define AT_COLON        ":"
#define AT_COMMA        ","
#define AT_TAIL         "\r\n"
#define AT_SEPARATOR    ""
#define AT_FRAME_KEY  "%hhx,%hhx,%hhx,%hhx,%hhx,%hhx,%hhx,%hhx,%hhx,%hhx,%hhx,%hhx,%hhx,%hhx,%hhx,%hhx"
#define AT_FRAME_KEY_OFFSET  2

#define AT_ON           "ON"
#define AT_OFF          "OFF"

#define AT_LPAUTOOFF    "AUTOOFF"
#define AT_LPAUTOON     "AUTOON"

#define AT_ABP          "LWABP"
#define AT_OTAA         "LWOTAA"

#define AT_CLASS_A      "A"
#define AT_CLASS_C      "C"

#endif




#ifdef __cplusplus
}
#endif

#endif /*__ATCMD_MODEM_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
