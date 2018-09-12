/*******************************************************************************
  * @file    lrwan_ns1_atcmd.h
  * @author  MCD Application Team
  * @version V1.0.3
  * @date    20-December-2017
  * @brief   Header for driver lrwan_ns1_atcmd.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LRWAN_NS1_ATCMD__
#define __LRWAN_NS1_ATCMD__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32l0xx_hal.h"

/* Exported types ------------------------------------------------------------*/
typedef enum ATGroup
{
  AT_CTRL = 0,
  AT_SET,
  AT_GET,
} ATGroup_t;

typedef enum Marker_s
{
  CTRL_MARKER = 0,
  SET_MARKER,
  GET_MARKER,
} Marker_t;


/****************************************************************************/
/*here we have to include a list of AT cmd by the way of #include<file>     */
/*this file will be preprocessed for enum ATCmd, enum eATerror and AT marker*/
/*define                                                                    */
/****************************************************************************/


#define  AT_ERROR_INDEX
#define  AT_CMD_STRING
#define  AT_CMD_INDEX
#define  AT_CMD_MARKER
#include "atcmd_modem.h"

/* Private define ------------------------------------------------------------*/
#define DATA_RX_MAX_BUFF_SIZE    300       /*Max size of the received buffer*/
                                          /*to optimize we can match with device key sizeof*/

#define DATA_TX_MAX_BUFF_SIZE    300       /*Max size of the transmit buffer*/
                                          /*it is the worst-case when sending*/
                                          /*a max payload equal to 64 bytes*/

#define ATCTL_CMD_BUF_SIZE              (250)
#define ATCTL_DL_BUF_SIZE               (100)
#define ATCTL_RX_TIMEOUT                (300)       /* ms */
#define ATCTL_CMD_MAX_SIZE              (10)

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
  ATCTL_MSG_START,
  ATCTL_MSG_TX,
  ATCTL_MSG_WAIT_ACK,
  ATCTL_MSG_RX,
  ATCTL_MSG_DONE,
  ATCTL_MSG_BUSY,
  ATCTL_MSG_LEN,
  ATCTL_MSG_DR,
  ATCTL_MSG_NO_CH,
  ATCTL_MSG_NO_BAND,
  ATCTL_MSG_NO_NET,                   /* Please join network first */
  ATCTL_MSG_JOINED,
  ATCTL_MSG_JOIN_ALREADY,
  ATCTL_MSG_JOIN_FORCE,
  ATCTL_MSG_JOIN_NORMAL,
  ATCTL_MSG_JOIN_FAILED,
  ATCTL_MSG_JOIN_NETID,
  ATCTL_MSG_JOIN_NOT_OTAA,
  ATCTL_MSG_ACK_RECEIVED,
  ATCTL_MSG_RXWIN,
  ATCTL_MSG_MACCMD,
} atctl_msg_sta_t;

typedef union
{
  int err;
  int ver[3];
  struct
  {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
  } rtc;
  float vdd;
  struct
  {
    atctl_msg_sta_t sta;
    bool ack;
    int16_t rssi;
    int8_t snr;
    uint8_t win;
    uint8_t port;
    uint8_t *buf;
    uint16_t size;
  } msg;
  struct
  {
    atctl_msg_sta_t sta;
    uint32_t netid;
    uint8_t devaddr[4];
  } join;
  struct
  {
    uint8_t addr;
    uint8_t val;
  } eeprom;
  struct
  {
    uint8_t devaddr[4];
    uint8_t deveui[8];
    uint8_t appeui[8];
  } id;
  struct
  {
    uint8_t dr_value;
    uint8_t dr_type;
  } dr;
  int mode;
  struct
  {
    uint8_t len;
    int8_t thld;
    uint16_t ul_counter;
    uint16_t dl_counter;
  } lw;
  struct
  {
    uint16_t rx1;
    uint16_t rx2;
    uint16_t jrx1;
    uint16_t jrx2;
  } delay;
} atctl_data_t;

typedef atctl_ret_t (*atctl_func)(char *, int, atctl_data_t *);

typedef struct
{
  ATCmd_t cmd;
  char *name;
  atctl_func func;
} atctl_cmd_list_t;

typedef enum
{
  ATCTL_RX_HEAD,
  ATCTL_RX_CMD,
  ATCTL_RX_DONE,
  ATCTL_PARSE_DONE,
} atctl_sta_t;

/*type definition for SENDB command*/
typedef struct sSendDataBinary
{
  char *Buffer;
  uint8_t DataSize;
} sSendDataBinary_t,sSendDataString_t;

/* LoRa modem band plans */
typedef enum eBandPlans
{
  EU868,
  US915,
  US915HYBRID,
  CN779,
  EU433,
  AU915,
  AU915OLD,
  CN470,
  AS923,
  KR920,
  IN865,
  CN470PREQUEL,
  STE920,
  BAND_MAX,
} BandPlans_t;

uint16_t at_cmd_vprintf(const char *format, ...);

/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* AT printf */
#define AT_VPRINTF(...)    at_cmd_vprintf(__VA_ARGS__)

#define AT_VSSCANF(...)    tiny_sscanf(__VA_ARGS__)



/* Exported functions ------------------------------------------------------- */

/******************************************************************************
 * @brief  Configures modem UART interface.
 * @param  None
 * @retval AT_OK in case of success
 * @retval HAL_ERROR in case of failure
*****************************************************************************/
HAL_StatusTypeDef Modem_IO_Init( void ) ;

/******************************************************************************
 * @brief  Deinitialise modem UART interface.
 * @param  None
 * @retval None
*****************************************************************************/
void Modem_IO_DeInit( void ) ;

/******************************************************************************
* @brief This function receives response from the slave device and parse
* @param Cmd: command type
* @param *dt: atctl_data_t type
* @retval LoRa return code
******************************************************************************/
atctl_ret_t at_cmd_receive(ATCmd_t Cmd, atctl_data_t *dt);

/******************************************************************************
* @brief This function receives response from the slave device
* @param viod
* @retval LoRa return code
******************************************************************************/
atctl_ret_t at_cmd_receive_evt(void);

/**************************************************************
* @brief  Parse the data stored in buffer
* @param  *dt: atctl_data_t
* @retval LoRa return code
**************************************************************/
atctl_ret_t atctl_rx(atctl_data_t *dt, int timeout);

/**************************************************************
* @brief  Send the command and receive
* @param  *dt: atctl_data_t
* @retval LoRa return code
**************************************************************/
atctl_ret_t atctl_tx(atctl_data_t *dt, ATCmd_t cmd, char *fmt, ...);

HAL_StatusTypeDef at_printf_send(uint8_t *buf, uint16_t len);

ATEerror_t  Modem_AT_Cmd(ATGroup_t at_group, ATCmd_t Cmd, void *pdata );

#ifdef __cplusplus
}
#endif

#endif /* __LRWAN_NS2_ATCMD__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
