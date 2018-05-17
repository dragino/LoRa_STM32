/*******************************************************************************
* @file    lrwan_ns1_atcmd.c
* @author  MCD Application Team
* @version V1.0.3
* @date    20-December-2017
* @brief   at command API
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

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "stm32l0xx_hal.h"
#include "hw_conf.h"
#include "hw_usart.h"
#include "lrwan_ns1_atcmd.h"
#include "tiny_sscanf.h"
#include "timeServer.h"

#include <stdarg.h>
#include "tiny_vsnprintf.h"
#include "debug.h"

#define ATCTL_WAKEUP    1

/*Globle variables------------------------------------------------------------*/
uint32_t record_num = 0;
uint8_t atctl_dl_buf[256];
char LoRa_AT_Cmd_Buff[DATA_TX_MAX_BUFF_SIZE];    /* Buffer used for AT cmd transmission */
char response[DATA_RX_MAX_BUFF_SIZE];   /*not only for return code but also for return value: exemple KEY*/

/* Private functions ---------------------------------------------------------*/
static atctl_ret_t atctl_parse(char *buf, int len, atctl_data_t *dt);
static atctl_ret_t atctl_at(char *buf, int len, atctl_data_t *dt);
static atctl_ret_t atctl_id(char *buf, int len, atctl_data_t *dt);
static atctl_ret_t atctl_ver(char *buf, int len, atctl_data_t *dt);
static atctl_ret_t atctl_vdd(char *buf, int len, atctl_data_t *dt);
static atctl_ret_t atctl_msg(char *buf, int len, atctl_data_t *dt);
static atctl_ret_t atctl_join(char *buf, int len, atctl_data_t *dt);
static atctl_ret_t atctl_dr(char *buf, int len, atctl_data_t *dt);
static atctl_ret_t atctl_mode(char *buf, int len, atctl_data_t *dt);
static atctl_ret_t atctl_lw(char *buf, int len, atctl_data_t *dt);
static atctl_ret_t atctl_delay(char *buf, int len, atctl_data_t *dt);
static HAL_StatusTypeDef at_cmd_send(uint16_t len);

static uint8_t at_cmd_format(ATCmd_t Cmd, void *ptr, Marker_t Marker);

/* Private variables ---------------------------------------------------------*/
static uint16_t Offset = 0;   /*write position needed for sendb command*/
static uint8_t aRxBuffer[5];  /* Buffer used for Rx input character */
static const atctl_cmd_list_t atctl_cmd_list[] = {
  {AT,             "AT",          atctl_at},
  {AT_FDEFAULT,    "FDEFAULT",    NULL},
  {AT_RESET,       "RESET",       NULL},
  {AT_SLEEP,       "LOWPOWER",    NULL},
  {AT_VER,         "VER",         atctl_ver},

  {AT_DADDR,       "ID",         atctl_id},
  {AT_DEUI,        "ID",         atctl_id},
  {AT_APPEUI,      "ID",         atctl_id},

  {AT_SEND,        "MSG",         atctl_msg},
  {AT_SENDB,       "MSGHEX",      atctl_msg},
  {AT_CMSG,        "CMSG",        atctl_msg},
  {AT_CMSGHEX,     "CMSGHEX",     atctl_msg},

  {AT_ADR,         "ADR",         NULL},
  {AT_DR,          "DR",          atctl_dr},
  {AT_TXP,         "POWER",       NULL},
  {AT_RX2,         "RXWIN2",      NULL},
  {AT_PORT,        "PORT",        NULL},

  {AT_LEN,         "LW",          atctl_lw},

  {AT_NJM,        "MODE",         atctl_mode},
  {AT_APPKEY,      "KEY",         NULL},
  {AT_NWKSKEY,     "KEY",         NULL},
  {AT_APPSKEY,     "KEY",         NULL},
  {AT_CLASS,       "CLASS",       NULL},
  {AT_JOIN,        "JOIN",        atctl_join},
  {AT_RX1DL,       "DELAY",       atctl_delay},
  {AT_RX2DL,       "DELAY",       atctl_delay},
  {AT_JN1DL,       "DELAY",       atctl_delay},
  {AT_JN2DL,       "DELAY",       atctl_delay},
  {AT_VDD,         "VDD",         atctl_vdd},
  {AT_WDG,         "WDT",         NULL},
};

static const struct atctl_msg_str
{
  const char *info;
  atctl_msg_sta_t type;
} atctl_msg_str[] = {
  {"Start",                               ATCTL_MSG_START},
  {"Done",                                ATCTL_MSG_DONE},
  {"LoRaWAN modem is busy",               ATCTL_MSG_BUSY},

  {"TX ",                                 ATCTL_MSG_TX},
  {"Wait ACK",                            ATCTL_MSG_WAIT_ACK},
  {"PORT",                                ATCTL_MSG_RX},
  {"RXWIN",                               ATCTL_MSG_RXWIN},
  {"MACCMD",                              ATCTL_MSG_MACCMD},

  {"Network joined",                      ATCTL_MSG_JOINED},
  {"Joined already",                      ATCTL_MSG_JOIN_ALREADY},
  {"NORMAL",                              ATCTL_MSG_JOIN_NORMAL},
  {"FORCE",                               ATCTL_MSG_JOIN_FORCE},
  {"Join failed",                         ATCTL_MSG_JOIN_FAILED},
  {"NetID",                               ATCTL_MSG_JOIN_NETID},
  {"Not in OTAA mode",                    ATCTL_MSG_JOIN_NOT_OTAA},

  {"Please join network first",           ATCTL_MSG_NO_NET},
  {"DR error",                            ATCTL_MSG_DR},
  {"Length error",                        ATCTL_MSG_LEN},
  {"No free channel",                     ATCTL_MSG_NO_CH},
  {"No band in",                          ATCTL_MSG_NO_BAND},
  {"ACK Received",                        ATCTL_MSG_ACK_RECEIVED},
};

static const struct band_str
{
  BandPlans_t BandPlan;
  const char *info;
} band_str[] = {
  {EU868,                         "EU868"},
  {US915,                         "US915"},
  {US915HYBRID,                   "US915HYBRID"},
  {CN779,                         "CN779"},
  {EU433,                         "EU433"},
  {AU915,                         "AU915"},
  {AU915OLD,                      "AU915OLD"},
  {CN470,                         "CN470"},
  {AS923,                         "AS923"},
  {KR920,                         "KR920"},
  {IN865,                         "IN865"},
  {CN470PREQUEL,                  "CN470PREQUEL"},
  {STE920,                        "STE920"},
};

static atctl_sta_t atctl_sta;
static char atctl_rx_buf[ATCTL_CMD_BUF_SIZE];
static volatile int16_t atctl_rx_wr_index;
static uint8_t atctl_rx_tmp_buf[ATCTL_CMD_BUF_SIZE];
static volatile int16_t atctl_rx_tmp_wr_index, atctl_rx_tmp_rd_index, atctl_rx_tmp_cnt;

extern atctl_data_t dt;

/**************************************************************
* @brief  Parse the received data
* @param  *buf: point to the data
* @param  len: length of the data
* @param  *dt: point to area used to store the parsed data
* @retval LoRa return code
**************************************************************/
static atctl_ret_t atctl_parse(char *buf, int len, atctl_data_t *dt)
{
  int i, cmdlen;

  i = sscanf((char*)buf, "%*s ERROR(%d)", &dt->err);
  if (i == 1)
  {
    return ATCTL_RET_CMD_ERR;
  }

  for (i = 0; i < sizeof(atctl_cmd_list) / sizeof(atctl_cmd_list_t); i ++)
  {
    cmdlen = strlen(atctl_cmd_list[i].name);
    if ( 0 == strncasecmp(atctl_cmd_list[i].name, (char *)buf + 1, cmdlen) )
    {
      if( atctl_cmd_list[i].func == NULL )
      {
        return ATCTL_RET_CMD_OK;
      }
      return (atctl_cmd_list[i].func(buf, len, dt));
    }
  }

  return ATCTL_RET_ERR;
}

/**************************************************************
* @brief  Parse the AT command received data
* @param  *buf: point to the data
* @param  len: length of the data
* @param  *dt: point to area used to store the parsed data
* @retval LoRa return code
**************************************************************/
static atctl_ret_t atctl_at(char *buf, int len, atctl_data_t *dt)
{
  int i;
  uint8_t p[2];
  i = sscanf((char*)buf, "%*s %c%c", &p[0], &p[1]);
  if (i == 2)
  {
    if (p[0] == 'O' && p[1] == 'K')
    {
      return ATCTL_RET_CMD_AT;
    }
  }
  return ATCTL_RET_CMD_OK;
}

/**************************************************************
* @brief  Parse the AT+VER command received data
* @param  *buf: point to the data
* @param  len: length of the data
* @param  *dt: point to area used to store the parsed data
* @retval LoRa return code
**************************************************************/
static atctl_ret_t atctl_ver(char *buf, int len, atctl_data_t *dt)
{
  int i;
  i = sscanf((char*)buf, "%*s %d.%d.%d", &dt->ver[0], &dt->ver[1], &dt->ver[2]);
  if (i == 3)
  {
    return ATCTL_RET_CMD_VER;
  }
  return ATCTL_RET_CMD_OK;
}

/**************************************************************
* @brief  Parse the AT+VDD command received data
* @param  *buf: point to the data
* @param  len: length of the data
* @param  *dt: point to area used to store the parsed data
* @retval LoRa return code
**************************************************************/
static atctl_ret_t atctl_vdd(char *buf, int len, atctl_data_t *dt)
{
  int i;
  struct
  {
    int integer;
    int fractional;
  }vdd;
  i = sscanf((char*)buf, "%*s %d.%dV", &vdd.integer, &vdd.fractional);
  if (i == 2)
  {
    dt->vdd = (float)vdd.integer + (float)vdd.fractional / 100.0;
    return ATCTL_RET_CMD_VDD;
  }
  return ATCTL_RET_CMD_OK;
}

/**************************************************************
* @brief  Parse the AT+MODE command received data
* @param  *buf: point to the data
* @param  len: length of the data
* @param  *dt: point to area used to store the parsed data
* @retval LoRa return code
**************************************************************/
static atctl_ret_t atctl_mode(char *buf, int len, atctl_data_t *dt)
{
  int i;
  char str_mode[4] = {0};
  char otaa_mode[] = "OTAA";
  char abp_mode[] = "ABP";
  i = sscanf((char*)buf, "%*s LW%s", str_mode);
  if (i == 1)
  {
    if (!memcmp(otaa_mode, str_mode, strlen(otaa_mode)))
    {
      dt->mode = 1;
    } else if (!memcmp(abp_mode, str_mode, strlen(abp_mode)))
    {
      dt->mode = 0;
    }
    return ATCTL_RET_CMD_MODE;
  }
  return ATCTL_RET_CMD_OK;
}

/**************************************************************
* @brief  Parse the AT+LW command received data
* @param  *buf: point to the data
* @param  len: length of the data
* @param  *dt: point to area used to store the parsed data
* @retval LoRa return code
**************************************************************/
static atctl_ret_t atctl_lw(char *buf, int len, atctl_data_t *dt)
{
  int i;
  int ul = 0;
  int dl = 0;
  int thld = 0;
  int length = 0;
  i = sscanf((char*)buf, "%*s ULDL%*s %d%*s %d", &ul, &dl);
  if (i == 2)
  {
    dt->lw.ul_counter = ul;
    dt->lw.dl_counter = dl;
    return ATCTL_RET_CMD_LW;
  }

  i = sscanf((char*)buf, "%*s THLD%*s -%d", &thld);
  if (i == 1)
  {
    dt->lw.thld = -thld;
    return ATCTL_RET_CMD_LW;
  }

  i = sscanf((char*)buf, "%*s LEN%*s %d", &length);
  if (i == 1)
  {
    dt->lw.len = length;
    return ATCTL_RET_CMD_LW;
  }
  return ATCTL_RET_CMD_OK;
}

/**************************************************************
* @brief  Parse the AT+DELAY command received data
* @param  *buf: point to the data
* @param  len: length of the data
* @param  *dt: point to area used to store the parsed data
* @retval LoRa return code
**************************************************************/
static atctl_ret_t atctl_delay(char *buf, int len, atctl_data_t *dt)
{
  int i;
  int rx1 = 0;
  int rx2 = 0;
  int jrx1 = 0;
  int jrx2 = 0;

  i = sscanf((char*)buf, "%*s RX1,%d", &rx1);
  if (i == 1)
  {
    dt->delay.rx1 = rx1;
    return ATCTL_RET_CMD_DELAY;
  }

  i = sscanf((char*)buf, "%*s RX2,%d", &rx2);
  if (i == 1)
  {
    dt->delay.rx2 = rx2;
    return ATCTL_RET_CMD_DELAY;
  }

  i = sscanf((char*)buf, "%*s JRX1,%d", &jrx1);
  if (i == 1)
  {
    dt->delay.jrx1 = jrx1;
    return ATCTL_RET_CMD_DELAY;
  }

  i = sscanf((char*)buf, "%*s JRX2,%d", &jrx2);
  if (i == 1)
  {
    dt->delay.jrx2 = jrx2;
    return ATCTL_RET_CMD_DELAY;
  }
  return ATCTL_RET_CMD_OK;
}

/**************************************************************
* @brief  Parse the AT+DR command received data
* @param  *buf: point to the data
* @param  len: length of the data
* @param  *dt: point to area used to store the parsed data
* @retval LoRa return code
**************************************************************/
static atctl_ret_t atctl_dr(char *buf, int len, atctl_data_t *dt)
{
  int i;
  int dr_value = 0;
  uint8_t type_str[20] = {0};

  i = sscanf((char*)buf, "%*s DR%d", (&dr_value));
  if (i == 1)
  {
    dt->dr.dr_value = dr_value;
    return ATCTL_RET_CMD_DR;
  }

  i = sscanf((char*)buf, "%*s %s", type_str);
  if (i == 1)
  {
    /* need to do something about dr type */
    return ATCTL_RET_CMD_DR;
  }

  i = sscanf((char*)buf, "%*s %*s %*s  %*s %s", type_str);
  if (i == 1)
  {
    /* need to do something about dr type */
    return ATCTL_RET_CMD_DR;
  }

  return ATCTL_RET_CMD_OK;
}

/**************************************************************
* @brief  Parse the AT+ID command received data
* @param  *buf: point to the data
* @param  len: length of the data
* @param  *dt: point to area used to store the parsed data
* @retval LoRa return code
**************************************************************/
static atctl_ret_t atctl_id(char *buf, int len, atctl_data_t *dt)
{
  int i;
  int p[8];

  i = sscanf((char*)buf, "%*s DevAddr, %02x:%02x:%02x:%02x", &p[0], &p[1], &p[2], &p[3]);
  if (i == 4)
  {
    for (i = 0; i < 4; i ++)
    {
      dt->id.devaddr[i] = (uint8_t)p[i];
    }
    return ATCTL_RET_CMD_ID;
  }

  i = sscanf((char*)buf, "%*s DevEui, %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", \
                                      &p[0], &p[1], &p[2], &p[3], &p[4], &p[5], &p[6], &p[7]);
  if (i == 8)
  {
    for (i = 0; i < 8; i ++)
    {
      dt->id.deveui[i] = (uint8_t)p[i];
    }
    return ATCTL_RET_CMD_ID;
  }

  i = sscanf((char*)buf, "%*s AppEui, %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", \
                                      &p[0], &p[1], &p[2], &p[3], &p[4], &p[5], &p[6], &p[7]);
  if (i == 8)
  {
    for (i = 0; i < 8; i ++)
    {
      dt->id.appeui[i] = (uint8_t)p[i];
    }
    return ATCTL_RET_CMD_ID;
  }

  return ATCTL_RET_CMD_OK;
}

/**************************************************************
* @brief  Parse the AT+JOIN command received data
* @param  *buf: point to the data
* @param  len: length of the data
* @param  *dt: point to area used to store the parsed data
* @retval LoRa return code
**************************************************************/
static atctl_ret_t atctl_join(char *buf, int len, atctl_data_t *dt)
{
  int i, j, index, ret;
  int slen;
  int x0;
  int p[4];
  static int delay_time;

  for (index = 0; index < ATCTL_CMD_MAX_SIZE; index ++)
  {
    if(buf[index] == ':')
    {
      break;
    }
  }

  for (i = 0; i < sizeof(atctl_msg_str) / sizeof(struct atctl_msg_str); i ++)
  {
    slen = strlen(atctl_msg_str[i].info);
    if( 0 == strncasecmp(atctl_msg_str[i].info, (char *)buf + index + 2, slen) )
    {
      dt->join.sta =  atctl_msg_str[i].type;
      switch (dt->join.sta)
      {
        case ATCTL_MSG_JOIN_NETID:
          ret = sscanf((char*)buf, "%*s %*s %x %*s %x:%x:%x:%x", \
                                                    &x0, &p[0], &p[1], &p[2], &p[3]);
          if (ret != 5)
          {
            return ATCTL_RET_CMD_OK;
          }
          dt->join.netid = (uint32_t)x0;
          for (j = 0; j < 4;  j++)
          {
            dt->join.devaddr[j] = (uint8_t)p[j];
          }
          break;
        case ATCTL_MSG_NO_BAND:
          ret = sscanf((char *)buf, "%*s %*s %*s %*s %d", &delay_time);
          break;
        default:
        break;
      }
      return ATCTL_RET_CMD_JOIN;
    }
  }
  return ATCTL_RET_CMD_OK;
}

/**************************************************************
* @brief  Parse the AT+MSG command received data
* @param  *buf: point to the data
* @param  len: length of the data
* @param  *dt: point to area used to store the parsed data
* @retval LoRa return code
**************************************************************/
static atctl_ret_t atctl_msg(char *buf, int len, atctl_data_t *dt)
{
  int i, j, k, index, ret;
  int slen;
  int x0, x1, x2;

  for (index = 0; index < ATCTL_CMD_MAX_SIZE; index ++)
  {
    if (buf[index] == ':')
    {
      break;
    }
  }

  for (i = 0; i < sizeof(atctl_msg_str) / sizeof(struct atctl_msg_str); i ++)
  {
    slen = strlen(atctl_msg_str[i].info);
    if ( 0 == strncasecmp(atctl_msg_str[i].info, (char *)buf+index+2, slen) )
    {
      dt->msg.sta =  atctl_msg_str[i].type;
      switch (atctl_msg_str[i].type)
      {
        case ATCTL_MSG_ACK_RECEIVED:
          dt->msg.ack = true;
          break;
        case ATCTL_MSG_RX:
          ret = sscanf((char*)buf, "%*s %*s %d", &x0);
          if (ret != 1)
          {
            return ATCTL_RET_CMD_OK;
          }
          dt->msg.port = (uint8_t)x0;
          j = 0;
          while ( ( buf[j] != 0 ) && (buf[j] != '\"') )
          {
            j ++;
          }
          j ++;
          k = 0;
          while (j < len)
          {
            ret = sscanf((char*)buf+j, "%X", &x0);
            if (ret != 1)
            {
              break;
            }
            atctl_dl_buf[k++] = (uint8_t)x0;
            j += 3;
          }
          dt->msg.buf = atctl_dl_buf;
          dt->msg.size = k;
          break;
        case ATCTL_MSG_RXWIN:
          ret = sscanf((char*)buf, "%*s RXWIN%d, %*s %d, SNR %d", &x0, &x1, &x2);
          if (ret != 3)
          {
            return ATCTL_RET_CMD_OK;
          }
          dt->msg.win = (uint8_t)x0;
          dt->msg.rssi = (int16_t)x1;
          dt->msg.snr = (int8_t)x2;
          break;
        default:
          break;
      }
      return ATCTL_RET_CMD_MSG;
    }
  }

  return ATCTL_RET_CMD_OK;
}

/**************************************************************
* @brief  Reset the key variables of the atctl function
* @param  void
* @retval void
**************************************************************/
static void atctl_reset(void)
{
  atctl_sta = ATCTL_RX_HEAD;
  atctl_rx_wr_index = 0;
  atctl_rx_tmp_cnt = 0;
  atctl_rx_tmp_rd_index = atctl_rx_tmp_wr_index;
}

/**************************************************************
* @brief  Store the received data in some place base on the rule for easy parse
* @param  data: one byte received
* @retval void
**************************************************************/
static void atctl_rx_byte(uint8_t data)
{
  switch (atctl_sta)
  {
    case ATCTL_RX_HEAD:
      atctl_rx_buf[0] = data;
      if ( atctl_rx_buf[0] == '+' )
      {
        atctl_rx_wr_index = 1;
        atctl_sta = ATCTL_RX_CMD;
      }
      break;
    case ATCTL_RX_CMD:
      atctl_rx_buf[atctl_rx_wr_index++] = data;
      if (data == '\n')
      {
        atctl_sta = ATCTL_RX_DONE;
        atctl_rx_buf[atctl_rx_wr_index] = '\0';
      } else if ( (data == '\t') || (data == '\r') || (data > 'a' && data <= 'z') || \
                  (data >= ' ' && data <= '~') )
      {

      }
      else
      {
        /** Unknow character */
        atctl_reset();
      }
      if ((atctl_rx_wr_index >= ATCTL_CMD_BUF_SIZE) && (atctl_sta != ATCTL_RX_DONE))
      {
        /** atmunication buffer overflow */
        atctl_reset();
      }
      break;
    case ATCTL_RX_DONE:
    case ATCTL_PARSE_DONE:
      /*save ongoing commands */
      if (atctl_rx_tmp_cnt<ATCTL_CMD_BUF_SIZE)
      {
        atctl_rx_tmp_buf[atctl_rx_tmp_wr_index++] = data;
        if (atctl_rx_tmp_wr_index == ATCTL_CMD_BUF_SIZE)
        {
          atctl_rx_tmp_wr_index = 0;
        }
        atctl_rx_tmp_cnt++;
      }
      break;
  }
}

/**************************************************************
* @brief  Handle the input params
* @retval length of buf
**************************************************************/
static int atctl_buf(char *buf, char *fmt, va_list ap)
{
  int i, d, ret, len;
  char c, *s;
  uint8_t *hbuf;
  double f;

  if (fmt == NULL)
  {
    return 0;
  }
  i = 0;
  while (*fmt)
  {
    if (*fmt == '%')
    {
      fmt++;
      switch (*fmt)
      {
        case 'd':
          d = va_arg(ap, int);
          ret = sprintf(buf+i, "%d", d);
          i+=ret;
          break;
        case 'x':
        case 'X':
          d = va_arg(ap, int);
          ret = sprintf(buf+i, "%X", d);
          i+=ret;
          break;
        case 'h':
          hbuf = va_arg(ap, uint8_t *);
          len = va_arg(ap, int);
          for (d=0; d<len; d++)
          {
            ret = sprintf(buf+i, "%02X", hbuf[d]);
            i+=ret;
          }
          break;
        case 's':
          s = va_arg(ap, char *);
          ret = sprintf(buf+i, "\"%s\"", s);
          i+=ret;
          break;
        case 'c':
          c = (char)va_arg(ap, int);
          ret = sprintf(buf+i, "%c", c);
          i+=ret;
          break;
        case 'f':
          f = va_arg(ap, double);
          ret = sprintf(buf+i, "%.3f", f);
          i+=ret;
          break;
      }
      fmt++;
    }
    else
    {
      buf[i++] = *fmt++;
    }
  }

  buf[i] = '\0';

  return i;
}

/**************************************************************
* @brief  Send the command and receive
* @param  *dt: atctl_data_t
* @retval LoRa return code
**************************************************************/
atctl_ret_t atctl_tx(atctl_data_t *dt, ATCmd_t cmd, char *fmt, ...)
{
  char buf[256];
  uint8_t wakeup_ch[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  va_list ap;
  int i, len;
  atctl_ret_t ret = ATCTL_RET_CMD_ERR;
  HAL_StatusTypeDef HAL_Status;

  for (i=0; i<AT_MAX; i++)
  {
    if (cmd == atctl_cmd_list[i].cmd)
    {
      break;
    }
  }
  if (i == AT_MAX)
  {
    return ATCTL_RET_ERR;
  }

  va_start(ap, fmt);
  len = atctl_buf(buf, fmt, ap);
  va_end(ap);

  atctl_reset();

  if (ATCTL_WAKEUP)
  {
    HAL_UART_Transmit(&huart1, wakeup_ch, strlen((char *)wakeup_ch), 5000);
  }

  memset(LoRa_AT_Cmd_Buff, 0x00, sizeof LoRa_AT_Cmd_Buff);

  if ( (len != 0) && (cmd != AT) )
  {
    len = AT_VPRINTF("AT+%s=%s\r\n", atctl_cmd_list[i].name, buf);
  }
  else
  {
    if (cmd == AT)
    {
      len = AT_VPRINTF("AT\r\n");
    }
    else
    {
      len = AT_VPRINTF("AT+%s\r\n", atctl_cmd_list[i].name);
    }
  }

  HAL_Status = at_cmd_send(len);
  if (HAL_Status != HAL_OK)
  {
    return (ATCTL_RET_ERR); /*problem on UART transmission*/
  }
  if (cmd != AT_RESET)
  {
    ret = at_cmd_receive(cmd, dt);
  }
#if defined CMD_DEBUG
  if (len)
  {
    at_printf_send((uint8_t *)LoRa_AT_Cmd_Buff, len);
  }

  if (response[0] != '\0')
  {
    at_printf_send((uint8_t *)response, strlen(response));
  }
#endif
  return ret;
}

/**************************************************************
* @brief  Parse the data stored in buffer
* @param  *dt: atctl_data_t
* @retval LoRa return code
**************************************************************/
atctl_ret_t atctl_rx(atctl_data_t *dt, int timeout)
{
  atctl_ret_t ret = ATCTL_RET_CMD_ERR;
  int curindex;

  if (timeout == 0)
  {
    if ( (atctl_sta == ATCTL_RX_DONE) || ( (atctl_sta == ATCTL_PARSE_DONE) && (atctl_rx_tmp_cnt>0) ) )
    {
      timeout = ATCTL_RX_TIMEOUT;
    }
    else
    {
      return ATCTL_RET_IDLE;
    }
  }

  curindex = atctl_rx_wr_index;
  while (1)
  {
    if ( (atctl_sta == ATCTL_PARSE_DONE) && (atctl_rx_tmp_cnt>0) )
    {
      __disable_irq();
      atctl_sta = ATCTL_RX_HEAD;
      while ( atctl_rx_tmp_cnt > 0 )
      {
        atctl_rx_byte(atctl_rx_tmp_buf[atctl_rx_tmp_rd_index++]);
        if (atctl_rx_tmp_rd_index == ATCTL_CMD_BUF_SIZE)
        {
          atctl_rx_tmp_rd_index = 0;
        }
        atctl_rx_tmp_cnt--;
        if (atctl_sta == ATCTL_RX_DONE)
        {
          break;
        }
      }
      __enable_irq();
    } else if (atctl_sta == ATCTL_RX_DONE)
    {
      atctl_sta = ATCTL_PARSE_DONE;
      ret = atctl_parse(atctl_rx_buf, atctl_rx_wr_index, dt);
      if (atctl_rx_tmp_cnt == 0)
      {
        atctl_sta = ATCTL_RX_HEAD;
      }
      return ret;
    } else if (atctl_sta == ATCTL_RX_HEAD)
    {
      return ret;
    }
    if ( atctl_rx_wr_index > curindex )
    {
      curindex = atctl_rx_wr_index;
    }
  }
}

/******************************************************************************
* @brief  Configures modem UART interface.
* @param  None
* @retval AT_OK in case of success
* @retval AT_UART_LINK_ERROR in case of failure
*****************************************************************************/
HAL_StatusTypeDef Modem_IO_Init( void )
{
  if ( HW_UART_Modem_Init(BAUD_RATE)== HAL_OK )
  {
    return HAL_OK;
  }
  else
  {
    return HAL_ERROR;
  }
}


/******************************************************************************
* @brief  Deinitialise modem UART interface.
* @param  None
* @retval None
*****************************************************************************/
void Modem_IO_DeInit( void )
{
  HAL_UART_MspDeInit(&huart2);
  HAL_UART_MspDeInit(&huart1);
}

/******************************************************************************
 * @brief  Handle the AT cmd following their Groupp type
 * @param  at_group AT group [control, set , get)
 *         Cmd AT command
 *         pdata pointer to the IN/OUT buffer
 * @retval module status
 *****************************************************************************/
ATEerror_t  Modem_AT_Cmd(ATGroup_t at_group, ATCmd_t Cmd, void *pdata )
{
ATEerror_t Status = ATCTL_RET_ERR;
HAL_StatusTypeDef HAL_Status;
uint16_t Len = 0;
uint8_t wakeup_ch[4] = {0xFF, 0xFF, 0xFF, 0xFF};

  /*reset At_cmd buffer for each transmission*/
  memset(LoRa_AT_Cmd_Buff, 0x00, sizeof LoRa_AT_Cmd_Buff);

  atctl_reset();

  if (ATCTL_WAKEUP)
  {
    HAL_UART_Transmit(&huart1, wakeup_ch, strlen((char *)wakeup_ch), 5000);
  }

  switch (at_group)
  {
    case AT_CTRL:
    {
      Len = at_cmd_format( Cmd, NULL, CTRL_MARKER);
      HAL_Status = at_cmd_send(Len);
      if (HAL_Status != HAL_OK)
      {
        return (ATCTL_RET_ERR); /*problem on UART transmission*/
      }
      Status = at_cmd_receive(Cmd, &dt);
      break;
    }
    case AT_SET:
    {
      Len = at_cmd_format(Cmd, pdata, SET_MARKER);
      HAL_Status = at_cmd_send(Len);
      if (HAL_Status != HAL_OK)
      {
        return (ATCTL_RET_ERR); /*problem on UART transmission*/
      }
      Status = at_cmd_receive(Cmd, &dt);
      break;
    }
    case AT_GET:
    {
      Len = at_cmd_format(Cmd, pdata, GET_MARKER);
      HAL_Status = at_cmd_send(Len);
      if (HAL_Status != HAL_OK)
      {
        return (ATCTL_RET_ERR); /*problem on UART transmission*/
      }
      Status = at_cmd_receive(Cmd, &dt);
      break;
    }
    default:
      DBG_PRINTF("unknow group\n\r");
      break;

  } /*end switch(at_group)*/

#if defined CMD_DEBUG
  if (Len)
  {
    at_printf_send((uint8_t *)LoRa_AT_Cmd_Buff, Len);
  }

  if (response[0] != '\0')
  {
    at_printf_send((uint8_t *)response, strlen(response));
  }
#endif
  return Status;
}


/******************************************************************************
 * @brief  format the cmd in order to be send
 * @param  Cmd AT command
 *         ptr generic pointer to the IN/OUT buffer
 *         Marker to discriminate the Set from the Get
 * @retval length of the formated frame to be send
 *****************************************************************************/
static uint8_t at_cmd_format(ATCmd_t Cmd, void *ptr, Marker_t Marker)
{
  uint16_t len;      /*length of the formated command*/
  uint8_t *PtrValue; /*for IN/OUT buffer*/
  uint32_t value;    /*for 32_02X and 32_D*/
  uint8_t value_8;   /*for 8_D*/

  switch (Cmd)
  {
    case AT:              /*supported*/
    case AT_RESET:        /*supported*/
    case AT_JOIN:         /*supported*/
    case AT_VER:          /*supported*/
    case AT_FDEFAULT:     /*supported*/
    case AT_LEN:          /*supported*/
    case AT_VDD:          /*supported*/
    {
      /*Format = FORMAT_VOID_PARAM;*/
      len = AT_VPRINTF("%s%s%s",AT_HEADER,CmdTab[Cmd],AT_TAIL);
      break;
    }

    case AT_CMSG:
    case AT_SEND:        /*RisingHF equivalent MSG -- supported*/
    case AT_RECV:        /*not supported*/
    {
      /*Format = FORMAT_PLAIN_TEXT;*/
      if (Marker == SET_MARKER)
      {
        len = AT_VPRINTF("%s%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_SET_MARKER,((sSendDataBinary_t *)ptr)->Buffer,AT_TAIL);
      }
      else
      {
        len = AT_VPRINTF("%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_GET_MARKER,AT_TAIL);
      }
      break;
    }
    case AT_CMSGHEX:
    case AT_SENDB:      /*RisingHF equivalent MSGHEX -- supported*/
    case AT_RECVB:      /*not supported*/
    {
      /*Format = FORMAT_BINARY_TEXT; */
      if (Marker == SET_MARKER)
      {
        Offset = AT_VPRINTF("%s%s%s",AT_HEADER,CmdTab[Cmd],AT_SET_MARKER);
        unsigned i;
        for (i = 0; i < ((sSendDataBinary_t *)ptr)->DataSize; i++)
        {
          Offset+=AT_VPRINTF("%02x", ((sSendDataBinary_t *)ptr)->Buffer[i]);
        }
        Offset+=AT_VPRINTF("%s",AT_TAIL);
        len = Offset;
        Offset = 0;
      }
      else
      {
        len = AT_VPRINTF("%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_GET_MARKER,AT_TAIL);
      }
      break;
    }
    case AT_APPKEY:    /*RisingHF equivalent AT+KEY=APPKEY -- supported*/
    case AT_NWKSKEY:   /*RisingHF equivalent AT+KEY=NWKKEY -- supported*/
    case AT_APPSKEY:   /*RisingHF equivalent AT+KEY=APPSKEY -- supported*/
    {
      /*Format = FORMAT_16_02X_PARAM;*/
      PtrValue = (uint8_t*) ptr;
      if (Marker == SET_MARKER)
      {
        len = AT_VPRINTF("%s%s%s%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%s",
                        AT_HEADER,CmdTab[Cmd],AT_COMMA,
                        PtrValue[0], PtrValue[1],
                       PtrValue[2], PtrValue[3],
                        PtrValue[4], PtrValue[5],
                        PtrValue[6], PtrValue[7],
                        PtrValue[8], PtrValue[9],
                        PtrValue[10], PtrValue[11],
                        PtrValue[12], PtrValue[13],
                        PtrValue[14], PtrValue[15], AT_TAIL);
      }
      else
      {
        len = AT_VPRINTF("%s%s%s",AT_HEADER,CmdTab[Cmd],AT_TAIL);
      }
      break;
    }
    case AT_DADDR:       /*supported*/
    case AT_NWKID:       /*not supported*/
    {
      /*Format = FORMAT_32_02X_PARAM;*/
      value =  *(uint32_t*)ptr;
      if (Marker == SET_MARKER)
      {
        len = AT_VPRINTF("%s%s%s%02x%02x%02x%02x%s",AT_HEADER,CmdTab[Cmd],AT_COMMA,
                      (unsigned)((unsigned char *)(&value))[3],
                      (unsigned)((unsigned char *)(&value))[2],
                      (unsigned)((unsigned char *)(&value))[1],
                      (unsigned)((unsigned char *)(&value))[0], AT_TAIL);
      }
      else
      {
        len = AT_VPRINTF("%s%s%s",AT_HEADER,CmdTab[Cmd],AT_TAIL);
      }
      break;
    }
    case AT_APPEUI:     /*supported*/
    case AT_DEUI:       /*supported*/
    {
      /*Format = FORMAT_8_02X_PARAM;*/
      PtrValue = (uint8_t*)ptr;
      if (Marker == SET_MARKER)
      {
        len = AT_VPRINTF("%s%s%s%02x%02x%02x%02x%02x%02x%02x%02x%s",
                        AT_HEADER,CmdTab[Cmd],AT_COMMA,
                        PtrValue[0], PtrValue[1], PtrValue[2],
                        PtrValue[3], PtrValue[4], PtrValue[5],
                        PtrValue[6], PtrValue[7],AT_TAIL);
      }
      else
      {
        len = AT_VPRINTF("%s%s%s",AT_HEADER,CmdTab[Cmd],AT_TAIL);
      }
      break;
    }
    case AT_RX1DL:    /*supported*/
    case AT_RX2DL:    /*supported*/
    case AT_JN1DL:    /*supported*/
    case AT_JN2DL:    /*supported*/
    {
      /*Format = FORMAT_32_D_PARAM;*/
      if (Marker == SET_MARKER)
      {
        value =  *(uint32_t*)ptr;
        len = AT_VPRINTF("%s%s%s%u%s",AT_HEADER,CmdTab[Cmd],AT_COMMA,value, AT_TAIL);
      }
      else
      {
        len = AT_VPRINTF("%s+DELAY%s",AT_HEADER,AT_TAIL);
      }
      break;
    }

    case AT_PNM:         /*supported*/
    case AT_DCS:         /*supported*/
    {
      /*Format = FORMAT_8_D_PARAM;*/
      if (Marker == SET_MARKER)
      {
        value_8 =  *(uint8_t*)ptr;
        if (value_8 == 0)
        {
          len = AT_VPRINTF("%s%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_COMMA,AT_OFF,AT_TAIL);
        } else if (value_8 == 1)
        {
          len = AT_VPRINTF("%s%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_COMMA,AT_ON,AT_TAIL);
        }
      }
      else
      {
        len = AT_VPRINTF("%s%s%s",AT_HEADER,CmdTab[Cmd],AT_TAIL);
      }
      break;
    }

    case AT_WDG:
    case AT_ADR:         /*supported*/
    {
      /*Format = FORMAT_8_D_PARAM;*/
      if (Marker == SET_MARKER)
      {
        value_8 =  *(uint8_t*)ptr;
        if (value_8 == 0)
        {
          len = AT_VPRINTF("%s%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_SET_MARKER,AT_OFF,AT_TAIL);
        } else if (value_8 == 1)
        {
          len = AT_VPRINTF("%s%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_SET_MARKER,AT_ON,AT_TAIL);
        }
      }
      else
      {
        len = AT_VPRINTF("%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_GET_MARKER,AT_TAIL);
      }
      break;
    }

    case AT_BAT:         /*supported*/
    {
      /*Format = FORMAT_8_D_PARAM;*/
      if (Marker == SET_MARKER )
      {
        value = *(uint32_t*)ptr;
        len = AT_VPRINTF("%s%s%s%u%s",AT_HEADER,CmdTab[Cmd],AT_COMMA,value,AT_TAIL);
      }
      else
      {
        len = AT_VPRINTF("%s%s%s",AT_HEADER,CmdTab[Cmd],AT_TAIL);
      }
      break;
    }

    case AT_PORT:
    case AT_DR:          /*supported*/
    case AT_TXP:         /*supported*/
    {
      /*Format = FORMAT_32_D_PARAM;*/
      if (Marker == SET_MARKER)
      {
        value =  *(uint32_t*)ptr;
        len = AT_VPRINTF("%s%s%s%u%s",AT_HEADER,CmdTab[Cmd],AT_SET_MARKER,value, AT_TAIL);
      }
      else
      {
        len = AT_VPRINTF("%s%s%s",AT_HEADER,AT_GET_MARKER,AT_TAIL);
      }
      break;
    }

    case AT_BAND:        /*supported*/
    {
      /*Format = FORMAT_8_D_PARAM;*/
      if (Marker == SET_MARKER)
      {
        value_8 =  *(uint8_t*)ptr;
        len = AT_VPRINTF("%s%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_SET_MARKER,band_str[value_8].info,AT_TAIL);
      }
      else
      {
        len = AT_VPRINTF("%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_GET_MARKER,AT_TAIL);
      }
      break;
    }

    case AT_NJM:         /*supported*/
    {
      /*Format = FORMAT_8_D_PARAM;*/
      if (Marker == SET_MARKER)
      {
        value_8 =  *(uint8_t*)ptr;
        if (value_8 == 0)
        {
          len = AT_VPRINTF("%s%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_SET_MARKER,AT_ABP,AT_TAIL);
        } else if (value_8 == 1)
        {
          len = AT_VPRINTF("%s%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_SET_MARKER,AT_OTAA,AT_TAIL);
        }
      }
      else
      {
        len = AT_VPRINTF("%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_GET_MARKER,AT_TAIL);
      }
      break;
    }

    case AT_CLASS:         /*supported*/
    {
      /*Format = FORMAT_8_D_PARAM;*/
      if (Marker == SET_MARKER)
      {
        value_8 =  *(uint8_t*)ptr;
        if (value_8 == 0)
        {
          len = AT_VPRINTF("%s%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_SET_MARKER,AT_CLASS_A,AT_TAIL);
        } else if (value_8 == 2)
        {
          len = AT_VPRINTF("%s%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_SET_MARKER,AT_CLASS_C,AT_TAIL);
        }
      }
      else
      {
        len = AT_VPRINTF("%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_GET_MARKER,AT_TAIL);
      }
      break;
    }

    case AT_RX2:
    {
      /*Format = FORMAT_8_D_PARAM;*/
      if (Marker == SET_MARKER)
      {
        PtrValue =  (uint8_t*)ptr;
        len = AT_VPRINTF("%s%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_SET_MARKER,PtrValue,AT_TAIL);
      }
      else
      {
        len = AT_VPRINTF("%s%s%s",AT_HEADER,CmdTab[Cmd],AT_TAIL);
      }
      break;
    }

    case AT_SLEEP:
    {
      /*Format = FORMAT_8_D_PARAM;*/
      if (Marker == SET_MARKER)
      {
        value_8 =  *(uint8_t*)ptr;
        if (value_8 == 0)
        {
          len = AT_VPRINTF("%s%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_SET_MARKER,AT_LPAUTOOFF,AT_TAIL);
        } else if (value_8 == 1)
        {
          len = AT_VPRINTF("%s%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_SET_MARKER,AT_LPAUTOON,AT_TAIL);
        }
      }
      else
      {
        len = AT_VPRINTF("%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_GET_MARKER,AT_TAIL);
      }
      break;
    }

    case AT_RX2FQ:       /*not supported*/
    case AT_FCU:         /*not supported*/
    case AT_FCD:         /*not supported*/
    case AT_ATE:         /*not supported*/
    case AT_RX2DR:       /*not supported*/
    case AT_CFM:         /*not supported*/
    case AT_CFS:         /*not supported*/
    case AT_RSSI:        /*not supported by RisingHF FW version*/
    case AT_SNR:         /*not supported by RisingHF FW version*/
    case AT_NJS:         /*not supported by RisingHF FW version*/
    case AT_WDCT:        /*not supported*/
    case AT_PS:          /*not supported*/

    default:
      len = AT_VPRINTF("%s%s%s",AT_HEADER,CmdTab[Cmd],AT_TAIL);
      DBG_PRINTF ("format not yet supported \n\r");
      break;
  } /*end switch(cmd)*/
  return len;
}

/******************************************************************************
* @brief This function sends an AT cmd to the slave device
* @param len: length of the AT cmd to be sent
* @retval HAL return code
******************************************************************************/
static HAL_StatusTypeDef at_cmd_send(uint16_t len)
{
  HAL_StatusTypeDef RetCode;

  /*transmit the command from master to slave*/
  RetCode = HAL_UART_Transmit(&huart1, (uint8_t*)LoRa_AT_Cmd_Buff, len, 5000);
  return ( RetCode);
}

/******************************************************************************
* @brief This function sends an AT cmd to debug in PC
* @param len: length of the AT cmd to be sent
* @retval HAL return code
******************************************************************************/
HAL_StatusTypeDef at_printf_send(uint8_t *buf, uint16_t len)
{
  HAL_StatusTypeDef RetCode;

  /*transmit the command from master to debug*/
  RetCode = HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, 5000);
  return ( RetCode);
}

/******************************************************************************
* @brief This function receives response from the slave device
* @param viod
* @retval LoRa return code
******************************************************************************/
atctl_ret_t at_cmd_receive_evt(void)
{
  uint8_t  ResponseComplete = 0;

  atctl_ret_t RetCode = ATCTL_RET_CMD_OK;
  uint32_t time_ms = 0;
  /*cleanup the response buffer*/
  memset(response, 0x00, sizeof(response));

  record_num = 0;

  /*UART peripheral in reception process for response returned by slave*/
  if (HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer,1) != HAL_OK)
  {
    while (1);
  }


  time_ms = TimerGetCurrentTime();

  while (!ResponseComplete)
  {
    while (HW_UART_Modem_IsNewCharReceived() == RESET)
    {
      if (record_num!=0)
      {
        if (response[record_num-1] != '\n')
        {
          continue;
        }
      }
      if (TimerGetElapsedTime(time_ms) > 300)
      {
        ResponseComplete = 1;
        break;
      }
    }

    if (ResponseComplete)
    {
      break;
    }

    time_ms = TimerGetCurrentTime();

    /*process the response*/
    response[record_num] = HW_UART_Modem_GetNewChar();

    if (response[record_num] == '\0')
    {
      continue;
    }

    /*process the rx data in the atctl_rx_byte*/
    atctl_rx_byte(response[record_num++]);

    HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer,1) ;
  }
  huart1.gState = HAL_UART_STATE_READY;
  huart1.RxState = HAL_UART_STATE_READY;        /*to be checked since was validated with previous */

  return RetCode;
}

/******************************************************************************
* @brief This function receives response from the slave device and parse
* @param Cmd: command type
* @param *dt: atctl_data_t type
* @retval LoRa return code
******************************************************************************/
atctl_ret_t at_cmd_receive(ATCmd_t Cmd, atctl_data_t *dt)
{
  uint8_t  ResponseComplete = 0;
  uint16_t i = 0;
  uint32_t time_ms = 0;

  atctl_ret_t RetCode = ATCTL_RET_IDLE;

  uint32_t response_timeout = 0;
  /*cleanup the response buffer*/
  memset(response, 0x00, sizeof(response));

  /*UART peripheral in reception process for response returned by slave*/
  if (HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer,1) != HAL_OK)
  {
    while (1);
  }


  time_ms = TimerGetCurrentTime();

  while (!ResponseComplete)
  {
    while (HW_UART_Modem_IsNewCharReceived() == RESET)
    {
      if ((Cmd == AT_DR) && (i == 0))
      {
        response_timeout = 3000;
      }
      else
      {
        response_timeout = 300;
      }

      if (TimerGetElapsedTime(time_ms) > response_timeout)
      {
        ResponseComplete = 1;
        break;
      }
    }

    if (ResponseComplete)
    {
      break;
    }

    time_ms = TimerGetCurrentTime();

    /*process the response*/
    response[i] = HW_UART_Modem_GetNewChar();

    if (response[i] == '\0')
    {
      continue;
    }

    /*process the rx data in the atctl_rx_byte*/
    atctl_rx_byte(response[i]);

    i++;

    HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer,1) ;
  }
  huart1.gState = HAL_UART_STATE_READY;
  huart1.RxState = HAL_UART_STATE_READY;        /*to be checked since was validated with previous */


  /* need to parse the rx data, and RetCode indicates the result */
  if (i == 0)
  {
    return RetCode;
  }

  memset(dt, 0, sizeof(atctl_data_t));
  /*handle  and parse the rx data*/
  RetCode = atctl_rx(dt, 300);//300ms

  /*do next base on the parsed data*/
  if (RetCode == ATCTL_RET_CMD_MSG)
  {
    switch ( dt->msg.sta )
    {
      case ATCTL_MSG_START:
        switch (Cmd)
        {
          case AT_CMSG:
          case AT_CMSGHEX:
            /* dump TX echo*/
            atctl_rx(dt, ATCTL_RX_TIMEOUT);
          case AT_SEND:
          case AT_SENDB:
            break;
          default:
            break;
        }
        break;
      case ATCTL_MSG_DONE:
        break;
      default:
        break;        
    }
  } else if (RetCode == ATCTL_RET_CMD_JOIN)
  {
    switch ( dt->join.sta )
    {
      case ATCTL_MSG_START:
        atctl_rx(dt, ATCTL_RX_TIMEOUT);
        break;
      default:
        break;        
    }
  } else if (RetCode == ATCTL_RET_CMD_ID)
  {
    i=0;
    do
    {
      RetCode = atctl_rx(dt, ATCTL_RX_TIMEOUT);
      i++;
    } while ( (RetCode == ATCTL_RET_CMD_ID) && (i<2) );
    RetCode = ATCTL_RET_CMD_ID;
  } else if (RetCode == ATCTL_RET_CMD_DR)
  {
    atctl_rx(dt, ATCTL_RX_TIMEOUT);
  } else if (RetCode == ATCTL_RET_CMD_DELAY)
  {
    i=0;
    do
    {
      RetCode = atctl_rx(dt, ATCTL_RX_TIMEOUT);
      i++;
    } while ( (RetCode == ATCTL_RET_CMD_DELAY) && (i<3) );
    RetCode = ATCTL_RET_CMD_DELAY;
  }

  return ( RetCode);
}


/******************************************************************************
* @brief format the AT frame to be sent to the modem (slave)
* @param pointer to the format string
* @retval len of the string to be sent
******************************************************************************/
uint16_t at_cmd_vprintf(const char *format, ...)
{
  va_list args;
  uint16_t len;

  va_start(args, format);

  len = tiny_vsnprintf_like(LoRa_AT_Cmd_Buff+Offset, sizeof(LoRa_AT_Cmd_Buff)-Offset, format, args);

  va_end(args);

  return len;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
