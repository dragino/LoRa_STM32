/******************************************************************************
  * @file    sht31.h
  * @author  MCD Application Team
  * @version V1.1.2
  * @date    30-Novermber-2018
  * @brief   contains all hardware driver
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics International N.V. 
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
#ifndef __BH1750_H__
#define __BH1750_H__

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 
/**
 * @brief  initialises the 
 *
 * @note
 * @retval None
 */
#include "hw.h"
 
#define GPIO_PORT_IIC	 GPIOA		
#define IIC_SCL_PIN		 GPIO_PIN_9		
#define IIC_SDA_PIN		 GPIO_PIN_10

#define IIC_SCL_1  HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SCL_PIN, GPIO_PIN_SET)		
#define IIC_SCL_0  HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SCL_PIN, GPIO_PIN_RESET)	
	
#define IIC_SDA_1  HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SDA_PIN, GPIO_PIN_SET)		/* SDA = 1 */
#define IIC_SDA_0  HAL_GPIO_WritePin(GPIO_PORT_IIC, IIC_SDA_PIN, GPIO_PIN_RESET)		/* SDA = 0 */
	
#define IIC_SDA_READ()  HAL_GPIO_ReadPin(GPIO_PORT_IIC, IIC_SDA_PIN)	

float bh1750_read(void);
void I2C_IoInit(void);
void I2C_DoInit(void);
void IIC_Delay(void);
void IIC_Start(void);
void IIC_Stop(void);
uint8_t IIC_WaitAck(void);
void IIC_NAck(void);
void IIC_Ack(void);
void IIC_SendByte(uint8_t _ucByte);
uint8_t IIC_ReadByte(unsigned char ack);
void SDA_OUT(void);
void SDA_IN(void);
uint8_t IIC_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);
uint8_t IIC_Read_Len(uint8_t addr,uint8_t len,uint8_t *buf);
uint8_t IIC_Write_Byte(uint8_t addr,uint8_t data);
uint8_t IIC_Read_Byte(uint8_t addr,uint8_t reg);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_H__ */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
