/******************************************************************************
 * @file    vcom.h
 * @author  MCD Application Team
 * @version V1.1.4
 * @date    08-January-2018
 * @brief   Header for vcom.c module
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
#ifndef __VCOM_H__
#define __VCOM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "hw_conf.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Character added when a RX error has been detected */
#define AT_ERROR_RX_CHAR 0x01
/* External variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Init the VCOM.
 * @param  None
 * @retval None
 */
void vcom_Init(void);

/**
 * @brief  DeInit the VCOM.
 * @param  None
 * @retval None
 */
void vcom_DeInit(void);

/**
 * @brief  Init the VCOM IOs.
 * @param  None
 * @retval None
 */
void vcom_IoInit(void);

/**
 * @brief  Init the VCOM RX
 * @param  None
 * @retval None
 */
void vcom_ReceiveInit(void);

/**
 * @brief  DeInit the VCOM IOs.
 * @param  None
 * @retval None
 */
void vcom_IoDeInit(void);

/**
 * @brief  Sends string on com port
 * @param  String
 * @retval None
 */
void vcom_Send(const char *format, ...);

/**
 * @brief  Checks if a new character has been received on com port
 * @param  None
 * @retval Returns SET if new character has been received on com port, RESET otherwise
 */
FlagStatus IsNewCharReceived(void);

/**
 * @brief  Gets new received characters on com port
 * @param  None
 * @retval Returns the character
 */
uint8_t GetNewChar(void);

/**
 * @brief  vcom IRQ Handler
 * @param  None
 * @retval None
 */
void vcom_IRQHandler(UART_HandleTypeDef *huart);

/** 
* @brief  Sends circular Buffer on com port in IT mode
* @note   called from low Priority interrupt
* @param  None
* @return None
*/
void vcom_Print( void);

/** 
* @brief  Records string on circular Buffer
* @note   To be called only from critical section from low power section
*         Other wise use vcom_Send
* @param  string
* @return None
*/
void vcom_Send_Lp( char *format, ... );

/**
 * @brief  vcom DMA IRQ Handler
 * @param  None
 * @retval None
 */
void vcom_Dma_IRQHandler( void );

/* Exported macros -----------------------------------------------------------*/
#if 0
#define PRINTF(...)     vcom_Send(__VA_ARGS__)
#define AT_PRINTF(...)     vcom_Send(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


#ifdef __cplusplus
}
#endif

#endif /* __VCOM_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
