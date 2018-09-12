 /*******************************************************************************
  * @file    i_nucleo_lrwan1_wm_sg_sm_xx.c
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    01-June-2017
  * @brief   driver I_NUCLEO_LRWAN1 for WM_SG_SM_XX modem board
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
#include "i_nucleo_lrwan1_wm_sg_sm_xx.h"
#include "tiny_sscanf.h"

#include <stdarg.h>
#include "tiny_vsnprintf.h"
#include "debug.h"  





/*!
 * Radio driver structure initialization
 */
const struct Modem_s Modem =
{
  Modem_IO_Init,
  Modem_IO_DeInit,
  Modem_AT_Cmd
  // Lora_Init,             /*there is sense if the lora driver is part of BSP architecture*/           
  // Lora_Join,
  // Lora_SetJoinMode,
  // Lora_GetJoinMode,
  // LoRa_SetKey,
  // LoRa_GetKey,
  // LoRa_SetAppID,
  // LoRa_GetAppID,
  // LoRa_SetDeviceID,
  // LoRa_GetDeviceID,
  // LoRa_SetDeviceAddress,
  // LoRa_GetDeviceAddress,
  // LoRa_SetNetworkID, 
  // LoRa_GetNetworkID,
  // Lora_SetAdaptiveDataRate,
  // Lora_GetAdaptiveDataRate,
  // Lora_SetClass,
  // Lora_GetClass,
  // Lora_SetDutyCycle,
  // Lora_GetDutyCycle,
  // Lora_SetDataRate,
  // Lora_GetDataRate,
  // LoRa_SetFrameCounter,
};



/* External variables --------------------------------------------------------*/
extern ATCmd_t gFlagException;  /*defined in lora_driver.c*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

char LoRa_AT_Cmd_Buff[DATA_TX_MAX_BUFF_SIZE];    /* Buffer used for AT cmd transmission */

static uint16_t Offset = 0;   /*write position needed for send command*/

static uint8_t aRxBuffer[5];  /* Buffer used for Rx input character */   

static char response[DATA_RX_MAX_BUFF_SIZE];  /*has to be the largest of the response*/
                                              /*not only for return code but also for*/
                                              /*return value: exemple KEY*/

/****************************************************************************/
/*here we have to include a list of AT cmd by the way of #include<file>     */
/*this file will be preprocessed for CmdTab and ATE_RetCode definition      */
/****************************************************************************/

#undef    __ATCMD_MODEM_H__    /*to avoid recursive include*/
#define   AT_CMD_STRING  
#define   AT_ERROR_STRING
#undef    AT_CMD_INDEX
#undef    AT_ERROR_INDEX
#include "atcmd_modem.h"   /*to include WM_SG_SM_42 specific string AT cmd definition*/       


/* Private function prototypes -----------------------------------------------*/
/* private functions ------------------------------------------------------- */


static uint8_t at_cmd_format(ATCmd_t Cmd, void *ptr, Marker_t Marker);

static HAL_StatusTypeDef at_cmd_send(uint16_t len);

static ATEerror_t at_cmd_receive(void *pdata);

static ATEerror_t at_cmd_responseAnalysing(const char *ReturnResp);

static ATEerror_t at_cmd_receive_async_event(void);

static ATEerror_t at_cmd_AsyncEventAnalysing(const char *ReturnResp,int8_t *Flag);

static ATEerror_t at_cmd_receive_async_event_downlink_data(void *ptr);


//static void at_cmd_send_noresp(uint16_t len); 

/* Exported functions ------------------------------------------------------- */


/******************************************************************************
 * @brief  Configures modem UART interface.
 * @param  None
 * @retval AT_OK in case of success
 * @retval AT_UART_LINK_ERROR in case of failure
*****************************************************************************/
ATEerror_t Modem_IO_Init( void )
{
  if ( HW_UART_Modem_Init(BAUD_RATE)== HAL_OK )
  {
    return AT_OK;
  }
  else
  {
    return AT_UART_LINK_ERROR;
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
}


/******************************************************************************
 * @brief  Receive data in interrupt mode from modem UART interface.
 * @param  UART handle
 * @retval None
*****************************************************************************/
void Modem_UART_Receive_IT(UART_HandleTypeDef *huart)
{
  /*UART peripheral in reception process for response returned by slave*/  
  if(HAL_UART_Receive_IT(huart, (uint8_t *)aRxBuffer,1) != HAL_OK)
  {
   while (1);
  } 
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
ATEerror_t Status = AT_END_ERROR;
HAL_StatusTypeDef HAL_Status;
uint16_t Len;  

  /*reset At_cmd buffer for each transmission*/
  memset(LoRa_AT_Cmd_Buff, 0x00, sizeof LoRa_AT_Cmd_Buff); 

  switch (at_group)
  {
  case AT_CTRL:
  {
    Len = at_cmd_format( Cmd, NULL, CTRL_MARKER); 
    HAL_Status = at_cmd_send(Len);
    if(HAL_Status != HAL_OK)
    {  
        return (AT_UART_LINK_ERROR); /*problem on UART transmission*/
    }    
    if(Cmd != AT_RESET)
    {
        Status = at_cmd_receive(NULL);   
        Status = AT_OK;
    }    
    break;
  }  
  case AT_SET:
  {
      Len = at_cmd_format(Cmd, pdata, SET_MARKER); 
      HAL_Status = at_cmd_send(Len);
      if(HAL_Status != HAL_OK)
        return (AT_UART_LINK_ERROR); /*problem on UART transmission*/
      Status = at_cmd_receive(NULL);
    break;
  }  
  case AT_GET:
  {
      Len = at_cmd_format(Cmd, pdata, GET_MARKER);       
      HAL_Status = at_cmd_send(Len);
      if(HAL_Status != HAL_OK)
        return (AT_UART_LINK_ERROR); /*problem on UART transmission*/      
      Status = at_cmd_receive(pdata);
    break;
  }  
  case AT_ASYNC_EVENT:
  {
      if ( Cmd == AT_JOIN)
      Status = at_cmd_receive_async_event(); 
      else
      Status = at_cmd_receive_async_event_downlink_data(pdata);  
    break;    
  }  
  case AT_EXCEPT: 
  {
      HAL_Delay(1000); 
      Len = at_cmd_format(Cmd, pdata, SET_MARKER); 
      HAL_Status = at_cmd_send(Len);     
       if(HAL_Status != HAL_OK)
        return (AT_UART_LINK_ERROR); /*problem on UART transmission*/  
       else
        HAL_Delay(1000); 
      return (AT_OK);
  }        
  case AT_EXCEPT_1:
  {
      Len = at_cmd_format(Cmd, NULL, SET_MARKER); 
      HAL_Status = at_cmd_send(Len);     
       if(HAL_Status != HAL_OK)
          return (AT_UART_LINK_ERROR); /*problem on UART transmission*/  
       else
       {
         /*Status = at_cmd_receive(NULL);*/
         return (AT_OK);    
       }   
  }    
  default:  
    DBG_PRINTF("unknow group\n\r");
    break;
    
  } /*end switch(at_group)*/
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
//Fmt_t Format;      /*type of format*/
uint8_t *PtrValue; /*for IN/OUT buffer*/
uint32_t value;    /*for 32_02X and 32_D*/
uint8_t value_8;   /*for 8_D*/
//char value_c;      /*for 8_C*/ 

        
 
  switch (Cmd){
  case AT:              /*supported*/
  case AT_RESET:        /*supported*/
  case AT_SLEEP:        /*supported*/ 
  case AT_FWVERSION:    /*supported*/ 
  { 
    /*Format = FORMAT_VOID_PARAM;*/
    len = AT_VPRINTF("%s%s%s",AT_HEADER,CmdTab[Cmd],AT_TAIL);  
    break;
  }
  case  AT_RECV: 
  case  AT_VER:  
  {
    /*Format = FORMAT_PLAIN_TEXT;*/
    if(Marker == SET_MARKER)
    {  
      len = AT_VPRINTF("%s%s%s%d%s%s\r\n",AT_HEADER,CmdTab[Cmd],AT_SET_MARKER,((sSendDataString_t *)ptr)->Port,
                                                              AT_COLON,((sSendDataString_t *)ptr)->Buffer);  
    }                                                          
    else
    {
      len = AT_VPRINTF("%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_GET_MARKER,AT_TAIL);     
    }
    break;  
  }
  case  AT_SEND:      /*supported - sendB replaced by send - just one send mode on USI*/
  case  AT_RECVB:     /*not supported*/
  {
  /*Format = FORMAT_BINARY_TEXT; */
    if(Marker == SET_MARKER)
    {
      Offset = AT_VPRINTF("%s%s%s%d%s",AT_HEADER,CmdTab[Cmd],AT_SET_MARKER,((sSendDataBinary_t *)ptr)->Port,AT_COMMA); 
      unsigned i;
      for (i = 0; i < ((sSendDataBinary_t *)ptr)->DataSize; i++)
      {
        Offset+=AT_VPRINTF("%02x", ((sSendDataBinary_t *)ptr)->Buffer[i]);
      }
      Offset+=AT_VPRINTF("%s%d%s",AT_COMMA,((sSendDataBinary_t *)ptr)->Ack,AT_TAIL); 
      /*Offset+=AT_VPRINTF("\r\n");*/
      len = Offset;
      Offset = 0;
    }
    else
    {  
      len = AT_VPRINTF("%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_GET_MARKER,AT_TAIL);         
    }
    break; 
  }
  case AT_APPKEY:    /* USI equivalent AK -- supported*/
  case AT_NWKSKEY:   /* USI equivalent NSK -- supported*/
  case AT_APPSKEY:   /* USI equivalent ASK -- supported*/
  {
    /*Format = FORMAT_16_02X_PARAM;*/
    PtrValue = (uint8_t*) ptr;     
    if(Marker == SET_MARKER) 
    {  
      len = AT_VPRINTF("%s%s%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s", 
                      AT_HEADER,CmdTab[Cmd],AT_SET_MARKER,
                      PtrValue[0],AT_SEPARATOR, PtrValue[1],AT_SEPARATOR, 
                      PtrValue[2],AT_SEPARATOR, PtrValue[3],AT_SEPARATOR,
                      PtrValue[4],AT_SEPARATOR, PtrValue[5],AT_SEPARATOR, 
          PtrValue[6],AT_SEPARATOR, PtrValue[7],AT_SEPARATOR,
                      PtrValue[8],AT_SEPARATOR, PtrValue[9],AT_SEPARATOR, 
          PtrValue[10],AT_SEPARATOR, PtrValue[11],AT_SEPARATOR,
                      PtrValue[12],AT_SEPARATOR, PtrValue[13],AT_SEPARATOR, 
          PtrValue[14],AT_SEPARATOR, PtrValue[15], AT_TAIL); 
    }     
    else
    { 
      len = AT_VPRINTF("%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_GET_MARKER,AT_TAIL);    
    }  
    break;
  }
  case AT_DADDR:       /*supported*/
  case AT_NWKID:       /*N/A*/
  {
    /*Format = FORMAT_32_02X_PARAM;*/
    value =  *(uint32_t*)ptr;    
    if(Marker == SET_MARKER)
    {  
      len = AT_VPRINTF("%s%s%s%02x%s%02x%s%02x%s%02x%s",AT_HEADER,CmdTab[Cmd],AT_SET_MARKER,
                    (unsigned)((unsigned char *)(&value))[3], AT_SEPARATOR,
                    (unsigned)((unsigned char *)(&value))[2], AT_SEPARATOR,
                    (unsigned)((unsigned char *)(&value))[1], AT_SEPARATOR,
                    (unsigned)((unsigned char *)(&value))[0], AT_TAIL);
    }                 
    else
    {  
      len = AT_VPRINTF("%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_GET_MARKER,AT_TAIL);   
    }      
    break;
  }
  case AT_APPEUI:     /*supported*/
  case AT_DEUI:       /* USI equivalent EUI - not relevant for SET since burned unique IEEE EUI64 at factory. */
  {
    /*Format = FORMAT_8_02X_PARAM;*/
    PtrValue = (uint8_t*)ptr; 
    if(Marker == SET_MARKER)
    { 
      len = AT_VPRINTF("%s%s%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s%02x%s",
                      AT_HEADER,CmdTab[Cmd],AT_SET_MARKER,
                      PtrValue[0],AT_SEPARATOR, PtrValue[1],AT_SEPARATOR, PtrValue[2],AT_SEPARATOR,
                      PtrValue[3],AT_SEPARATOR, PtrValue[4],AT_SEPARATOR, PtrValue[5],AT_SEPARATOR, 
                      PtrValue[6],AT_SEPARATOR, PtrValue[7],AT_TAIL);   
    }                  
    else
    { 
      len = AT_VPRINTF("%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_GET_MARKER,AT_TAIL);       
    }
    break;
  }
  case  AT_RX2FQ:    /*N/A*/
  case  AT_RX1DL:    /* USI equivalent RX1DT supported*/
  case  AT_RX2DL:    /* USI equivalent RX2DT - supported*/
  case  AT_JN1DL:    /* USI equivalent JRX1DT - supported*/
  case  AT_JN2DL:    /* USI equivalent JRX2DT - supported*/
  case  AT_FCU:      /* USI equivalent -> not supported*/
  case  AT_FCD:      /* USI equivalent -> not supported*/
  {
    /*Format = FORMAT_32_D_PARAM;*/
    if(Marker == SET_MARKER)
    {    
      value =  *(uint32_t*)ptr;    
      len = AT_VPRINTF("%s%s%s%u%s",AT_HEADER,CmdTab[Cmd],AT_SET_MARKER,value, AT_TAIL); 
    }
    else
    {  
      len = AT_VPRINTF("%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_GET_MARKER,AT_TAIL); 
    }  
    break; 
  }
  case  AT_JOIN:        /* supported*/
  case  AT_ATE:         /* supported*/
  case  AT_DR:          /* supported*/
  case  AT_RX2DR:       /* supported*/
  case  AT_BAND:        /* supported*/  
  case  AT_TXP:         /* N/A*/
  case  AT_NJM:         /* N/A*/
  case  AT_PNM:         /* supported - USI equivalent NTYP*/
  case  AT_DCS:         /* supported - USI equivalent DC - Disabling duty cycle for testing only. It should be enabled for shipping*/
  case  AT_ADR:         /* supported*/
  case  AT_CFM:         /* N/A*/
  case  AT_CFS:         /* N/A*/
  case  AT_BAT:         /* supported*/
  case  AT_RSSI:        /* not supported by USI FW version*/
  case  AT_SNR:         /* not supported by USI FW version*/
  case  AT_NJS:         /* N/A*/
  case  AT_CLASS:       /* not supported on V2.5 USI FW version*/
  case  AT_WDCT:        /* supported*/ 
  {
    /*Format = FORMAT_8_D_PARAM;*/
    if(Marker == SET_MARKER)
    {  
      value_8 =  *(uint8_t*)ptr;    
      len = AT_VPRINTF("%s%s%s%d%s",AT_HEADER,CmdTab[Cmd],AT_SET_MARKER,value_8,AT_TAIL);   
    }
    else
    {
      len = AT_VPRINTF("%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_GET_MARKER,AT_TAIL);  
    }   
    break;  
  }
  case AT_PS:
  {
    if(Marker == SET_MARKER)
    {  
      /*value_8 =  *(uint8_t*)ptr;*/
      len = AT_VPRINTF("%s%s%s%d,%d%s",AT_HEADER,CmdTab[Cmd],AT_SET_MARKER,
                      ((sPowerCtrlSet_t *)ptr)->SetType,((sPowerCtrlSet_t *)ptr)->Value,AT_TAIL);     
    }
    else
    {
      len = AT_VPRINTF("%s%s%s%s",AT_HEADER,CmdTab[Cmd],AT_GET_MARKER,AT_TAIL);  
    }   
    break;     
  }
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
  RetCode = HAL_UART_Transmit(&huart2, (uint8_t*)LoRa_AT_Cmd_Buff, len, 5000);
  return ( RetCode);  
}



/******************************************************************************
  * @brief This function receives response from the slave device
  * @param pdata: pointeur to the value returned by the slave
  * @retval return code coming from slave
******************************************************************************/ 
static ATEerror_t at_cmd_receive(void *pdata)
{
uint8_t  ResponseComplete = 0;
int8_t i = 0;
int8_t charnumber = 0;
char *ptrChr;
ATEerror_t RetCode;
uint8_t NoReturnCode =1;   /*to discriminate the Get return code from return value*/
  
  /*cleanup the response buffer*/
  memset(response, 0x00, 16); 

  /*UART peripheral in reception process for response returned by slave*/  
  if(HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer,1) != HAL_OK)
  {
   while (1);
  } 
  
   
  while (!ResponseComplete)
  {  
   
      while (HW_UART_Modem_IsNewCharReceived() == RESET);   
    
      /*process the response*/    
      response[i] = HW_UART_Modem_GetNewChar();
         
      /*wait up to carriage return OR the line feed marker*/
       if (/*(response[i] =='\r') || */(response[i] == '\n'))
       {
        if(pdata == NULL) /*return code following a SET cmd or simple AT cmd*/
        { 
          if (i>1)  /*return code following a SET cmd or simple AT cmd- we skip the first <cr><ln>*/
          {
            i= 0;
            ResponseComplete = 1;
            RetCode = at_cmd_responseAnalysing(&response[2]);  /* to skip the '\0' in position [0]*/
            break;
          }  
        }
        else    /* returned value following a GET cmd */
        {
          if (i!= 0 && NoReturnCode)
          {
            /*first statement to get back the return value*/
            response[i] = '\0';
            if (gFlagException != AT_FWVERSION)         /*see comment in loara_driver.c*/
            {  
              ptrChr = strchr(&response[1],'=');       /*to skip the '\0''\r'*/
              strcpy(pdata,ptrChr+1);
              gFlagException = AT_END_AT;
            }
            else
            { 
              strcpy(pdata,&response[1]);  
            } 
            memset(response, 0x00, 16);
            i= -1;             /*to compensate the next index iteration and restart in [0]*/
            NoReturnCode = 0;  /*return code for the Get cmd*/
          }
          else
          {
            if (i>1)
            { 
              /*second statement to get back the return code*/
              i= 0;
              ResponseComplete = 1;   /*when value + return code have been trapped*/
              RetCode = at_cmd_responseAnalysing(response);
              memset(response, 0x00, 16);
              break;        
            }   
          }
        }  
       }   
       else
       {
        if (i ==  (DATA_RX_MAX_BUFF_SIZE-1)) /* frame overflow */           
         {  
           i = 0; 
           return (AT_TEST_PARAM_OVERFLOW);
         } 
       }
        i++;
      HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer,1) ;
      charnumber++;
  } 
      huart2.gState = HAL_UART_STATE_READY;
      huart2.RxState = HAL_UART_STATE_READY;        /*to be checked since was validated with previous */
      return ( RetCode);                            /*version of HAL .. there was not Rx field state*/
}




/******************************************************************************
  * @brief This function receives asynchronus event from the slave device
  * @param none
  * @retval return code coming from slave
******************************************************************************/ 
static ATEerror_t at_cmd_receive_async_event(void)
{
uint8_t  ResponseComplete = 0;
int8_t i = 0;
int8_t charnumber = 0;
char *ptrChr;
ATEerror_t RetCode;
uint8_t NoReturnCode =1;   /*too discriminate the Get reurn code from return value*/
  
  /*cleanup the response buffer*/
  memset(response, 0x00, 16); 

  /*UART peripheral in reception process for response returned by slave*/  
  if(HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer,1) != HAL_OK)
  {
   while (1);
  } 
  
  while (!ResponseComplete)
  {  
   
    while (HW_UART_Modem_IsNewCharReceived() == RESET);   
    
    /*process the response*/    
    response[i] = HW_UART_Modem_GetNewChar();
            
    /*wait up to carriage return OR the line feed marker*/
    if (/*(response[i] =='\r') || */(response[i] == '\n'))       
    {
      if (i!= 0 && NoReturnCode)      /*trap the asynchronous event*/
      {
        /*first statement to get back the return value*/
        response[i] = '\0';
        ptrChr = strchr(&response[1],'+');       /*to skip the '\0''\r'*/
        RetCode = at_cmd_AsyncEventAnalysing(ptrChr,NULL);
        memset(response, 0x00, 16);
        i= -1;             /*to compensate the next index iteration and restart in [0]*/
        NoReturnCode = 0;  /*return code for the Get cmd*/
        break;
      }
      else
      {
        if (i>1)
        { 
          /*second statement to get back the return code*/
          i= 0;
          ResponseComplete = 1;   /*when value + return code have been trapped*/
          RetCode = at_cmd_responseAnalysing(response);
          memset(response, 0x00, 16);
          break;        
        }   
      }
    }   
    else
    {
      if (i ==  (DATA_RX_MAX_BUFF_SIZE-1)) /* frame overflow */           
      {  
        i = 0; 
        return (AT_TEST_PARAM_OVERFLOW);
      } 
    }
      i++;
      HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer,1) ;
      charnumber++;
  } 
      huart2.gState = HAL_UART_STATE_READY;
      huart2.RxState = HAL_UART_STATE_READY;        /*to be checked since was validated with previous */
      return ( RetCode);                            /*version of HAL .. there was not Rx field state*/
}



/******************************************************************************
  * @brief This function receives asynchronus event from the slave device
  * @param none
  * @retval return code coming from slave
******************************************************************************/ 
static ATEerror_t at_cmd_receive_async_event_downlink_data(void *pdata)
{

int8_t i = 0;
int8_t charnumber = 0;
char *ptrChr;
ATEerror_t RetCode;
uint8_t NoReturnCode =1;   /*too discriminate the Get reurn code from return value*/
int8_t DlinkData_Complete = (0x1U);

  /*cleanup the response buffer*/
  memset(response, 0x00, 16); 

  /*UART peripheral in reception process for response returned by slave*/  
  if(HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer,1) != HAL_OK)
  {
   while (1);
  } 
  
   
  while ( !(DlinkData_Complete & (0x1u <<2)))   /*received sequence not complete*/
  {  
   
    while (HW_UART_Modem_IsNewCharReceived() == RESET);   
    
    /*process the response*/    
    response[i] = HW_UART_Modem_GetNewChar();
            
      /*wait up to carriage return OR the line feed marker*/
       if ((response[i] =='\r') || (response[i] == '\n'))        
       {
          if (i!= 0 && NoReturnCode)      /*trap the asynchronous events associated to network downlink data*/
          {
            /*sequence of events to be trapped: +RXPORT , +PAYLOADSIZE , +RCV*/
            response[i] = '\0';
            ptrChr = strchr(&response[0],'+');   /*Here when we go out from low power mode the prefix is*/
                                                /* '\r' only. We do not skip the '\0''\r' - USI behavior ...*/
            RetCode = at_cmd_AsyncEventAnalysing(ptrChr,&DlinkData_Complete);
            if(RetCode == AT_OK)
            {
                ptrChr = strchr(&response[1],'=');       /*to skip the '\0''\r'*/
                strcpy(pdata,ptrChr+1);
                pdata = (char*)pdata + strlen(ptrChr+1);
                *((char*)pdata) = ',';                  /* introduce separator in order to discriminate port, size and data*/
                pdata = (char*)pdata + 1;
            }   
                
            memset(response, 0x00, 16);
            i= -1;             /*to compensate the next index iteration and restart in [0]*/
//      NoReturnCode = 0;  /*return code for the Get cmd*/
//                  break;
          } 
       }   
       else
       {
          if (i ==  (DATA_RX_MAX_BUFF_SIZE-1)) /* frame overflow */           
          {  
            i = 0; 
            return (AT_TEST_PARAM_OVERFLOW);
          } 
       }
       i++;
       HAL_UART_Receive_IT(&huart2, (uint8_t *)aRxBuffer,1) ;
       charnumber++;
  } 
       huart2.gState = HAL_UART_STATE_READY;
       huart2.RxState = HAL_UART_STATE_READY;        /*to be checked since was validated with previous */
       return ( RetCode);                            /*version of HAL .. there was not Rx field state*/
}

/******************************************************************************
  * @brief This function does analysis of the response received by the device
  * @param response: pointer to the received response
  * @retval ATEerror_t error type 
******************************************************************************/
static ATEerror_t at_cmd_responseAnalysing(const char *ReturnResp)
{
ATEerror_t status;
int i;

    status = AT_END_ERROR;

    for (i = 0; i < AT_END_ERROR; i++)
    {   
      if (strncmp(ReturnResp, ATE_RetCode[i].RetCodeStr, (ATE_RetCode[i].SizeRetCodeStr-1)) == 0)
      {
        /* command has been found found*/
        status = ATE_RetCode[i].RetCode;
        return (status);
      }          
    }
      return (status);
}



/******************************************************************************
  * @brief This function does analysis of the asynchronous event received by the device
  * @param response: pointer to the received response
  * @retval ATEerror_t error type 
******************************************************************************/
static ATEerror_t at_cmd_AsyncEventAnalysing(const char *ReturnResp, int8_t *Flag)
{
ATEerror_t status;

    status = AT_END_ERROR;

    if (strncmp(ReturnResp, "+JoinAccepted\r", sizeof("+JoinAccepted\r")-1) == 0)
    {
      /* event has been identified*/
      status = AT_OK;
      return (status);
    }     
    
    /*following statements for network downlink data analysis*/
    if (strncmp(ReturnResp, "+RXPORT", sizeof("+RXPORT")-1) == 0)
    {
      /* event has been identified*/
      *Flag <<= (0x0U);
      status = AT_OK;
      return (status);
    }
    
    /*following statement for network downlink data*/
    if (strncmp(ReturnResp, "+PAYLOADSIZE", sizeof("+PAYLOADSIZE")-1) == 0)
    {
      /* event has been identified*/
      *Flag <<= (0x1U);
      status = AT_OK;
      return (status);
    }
    
    /*following statement for network downlink data*/
    if (strncmp(ReturnResp, "+RCV", sizeof("+RCV")-1) == 0)
    {
      /* event has been identified*/
      *Flag <<= (0x1U); 
      status = AT_OK;
      return (status);
    }

    return (status);
}
/******************************************************************************
  * @brief This function sends an AT cmd to the slave device
  * @brief It is an AT cmd without response from slave device
  * @param len: length of the AT cmd to be sent
  * @retval void
******************************************************************************/ 
//static void at_cmd_send_noresp(uint16_t len)
//{
//  /*transmit the command from master to slave*/
//  if(HAL_UART_Transmit(&huart2, (uint8_t*)LoRa_AT_Cmd_Buff, len, 5000) != HAL_OK) 
//  {
//   while (1);
//  } 
//}




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
