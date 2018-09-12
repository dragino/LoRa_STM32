 /*******************************************************************************
  * @file    i_nucleo_lrwan1_wm_sg_sm_xx.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I_NUCLEO_LRWAN1_WM_SG_SM_XX_H__
#define __I_NUCLEO_LRWAN1_WM_SG_SM_XX_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

	
/* Exported functions ------------------------------------------------------- */ 


 
/* Exported types ------------------------------------------------------------*/

#define DATA_RX_MAX_BUFF_SIZE    64       /*Max size of the received buffer*/
                                          /*to optimize we can match with device key sizeof*/
 
#define DATA_TX_MAX_BUFF_SIZE    78       /*Max size of the transmit buffer*/
                                          /*it is the worst-case when sending*/
                                          /*a max payload equal to 64 bytes*/

typedef enum ATGroup
{
  AT_CTRL = 0,
  AT_SET,
  AT_GET,
  AT_EXCEPT,
  AT_ASYNC_EVENT,
  AT_EXCEPT_1,
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
#define  AT_CMD_INDEX
#define  AT_CMD_MARKER
#include "atcmd_modem.h"    /*to include WM_SG_SM_42 specific string AT cmd definition*/



/*!
 * \brief Modem driver definition
 */
typedef struct Modem_s
{
    /*!
     * \brief Initializes the IO interface of the Modem
     */
    ATEerror_t    ( *IoInit )( void );
    /*!
     * \brief DeInitializes the IO interface of the Modem
     */
    void    ( *IoDeInit )( void );
    /*!
     * \brief Handler for Modem AT command
     */
    ATEerror_t  ( *ATCmd )( ATGroup_t at_group, ATCmd_t Cmd, void *pdata );

} sModem_t;



/*type definition for SEND command*/
typedef struct sSendDataString
{
    char *Buffer;
    uint8_t Port;    
}sSendDataString_t;          

/*type definition for RECV command*/
typedef struct sReceivedDataString
{
    uint8_t *Buffer;
    uint8_t Port;    
}sReceivedDataString_t;       


/*type definition for SENDB command*/
typedef struct sSendDataBinary
{
    char *Buffer;
    uint8_t DataSize;
    uint8_t Port; 
    uint8_t Ack;
}sSendDataBinary_t;


/*type definition for RECVB command*/
typedef struct sReceivedDataBinary
{
    uint8_t *Buffer;
    uint32_t DataSize;
    uint8_t Port;    
}sReceivedDataBinary_t;


/*type definition for return code analysis*/
typedef  char* ATEerrorStr_t;

typedef struct RetCode_s{
  ATEerrorStr_t RetCodeStr;
  int SizeRetCodeStr;
  ATEerror_t RetCode;
} ATE_RetCode_t;

/*type definition for the MCU power Control setting*/
typedef struct sPowerCtrlSet{
  uint8_t  SetType;
  uint8_t  Value;
  uint8_t  AutoSleepTime;
} sPowerCtrlSet_t;  

/*type definition for AT cmd format identification*/
typedef enum Fmt
{
  FORMAT_VOID_PARAM,
  FORMAT_8_02X_PARAM,
  FORMAT_16_02X_PARAM,
  FORMAT_32_02X_PARAM,
  FORMAT_32_D_PARAM,
  FORMAT_8_D_PARAM,
  FORMAT_8_C_PARAM,
  FORMAT_PLAIN_TEXT,
  FORMAT_BINARY_TEXT  
}  Fmt_t;  



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
 * @retval AT_UART_LINK_ERROR in case of failure
*****************************************************************************/
ATEerror_t Modem_IO_Init( void ) ;

/******************************************************************************
 * @brief  Deinitialise modem UART interface.
 * @param  None
 * @retval None
*****************************************************************************/
void Modem_IO_DeInit( void ) ;

/******************************************************************************
 * @brief  Receive data in interrupt mode from modem UART interface.
 * @param  UART handle
 * @retval None
*****************************************************************************/
void Modem_UART_Receive_IT(UART_HandleTypeDef *huart);

/******************************************************************************
 * @brief  Handle the AT cmd following their Groupp type
 * @param  at_group AT group [control, set , get)
 *         Cmd AT command
 *         pdata pointer to the IN/OUT buffer
 * @retval module status
 *****************************************************************************/
ATEerror_t Modem_AT_Cmd(ATGroup_t at_group, ATCmd_t Cmd, void *pdata );



#ifdef __cplusplus
}
#endif

#endif /* __I_NUCLEO_LRWAN1_WM_SG_SM_XX_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
