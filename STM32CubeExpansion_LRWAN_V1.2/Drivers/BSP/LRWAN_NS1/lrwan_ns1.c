/**
 ******************************************************************************
 * @file    lrwan_ns1.c
 * @author  MEMS Application Team
 * @version V1.0.3
 * @date    20-December-2017
 * @brief   This file provides LRWAN_NS1 shield board specific functions
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

#include "lrwan_ns1.h"

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup LRWAN_NS1 LRWAN_NS1
 * @{
 */

/** @addtogroup LRWAN_NS1_IO IO
 * @{
 */

/** @addtogroup LRWAN_NS1_IO_Private_Variables Private variables
 * @{
 */

static uint32_t I2C_EXPBD_Timeout =
 NUCLEO_I2C_EXPBD_TIMEOUT_MAX;    /*<! Value of Timeout when I2C communication fails */
//static I2C_HandleTypeDef I2C_EXPBD_Handle;
I2C_HandleTypeDef I2C_EXPBD_Handle;

/**
 * @}
 */

/* Link function for sensor peripheral */
uint8_t Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );

static void I2C_EXPBD_MspInit( void );
static void I2C_EXPBD_Error( uint8_t Addr );
static uint8_t I2C_EXPBD_ReadData( uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size );
static uint8_t I2C_EXPBD_WriteData( uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size );
static uint8_t I2C_EXPBD_Init( void );

/** @addtogroup LRWAN_NS1_IO_Public_Functions Public functions
 * @{
 */

/**
 * @brief  Configures sensor I2C interface.
 * @param  None
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef Sensor_IO_Init( void )
{

  if ( I2C_EXPBD_Init() )
  {
    return COMPONENT_ERROR;
  }
  else
  {
    return COMPONENT_OK;
  }
}



/**
 * @brief  Configures sensor interrupts interface for LSM6DS0 sensor).
 * @param  None
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef LSM6DS0_Sensor_IO_ITConfig( void )
{

  /*Not implemented yet*/

  return COMPONENT_OK;
}



/**
 * @brief  Configures sensor interrupts interface for LSM6DS3 sensor.
 * @param  None
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef LSM6DS3_Sensor_IO_ITConfig( void )
{

  /* At the moment this feature is only implemented for LSM6DS3 */
  GPIO_InitTypeDef GPIO_InitStructureInt1;
  GPIO_InitTypeDef GPIO_InitStructureInt2;
  /* Enable INT1 GPIO clock */
  LSM6DS3_INT1_GPIO_CLK_ENABLE();


  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructureInt1.Pin = LSM6DS3_INT1_PIN;
  GPIO_InitStructureInt1.Mode = GPIO_MODE_IT_RISING;
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
  GPIO_InitStructureInt1.Speed = GPIO_SPEED_FAST;
#endif

#if (defined (USE_STM32L1XX_NUCLEO))
  GPIO_InitStructureInt1.Speed = GPIO_SPEED_MEDIUM;
#endif
  GPIO_InitStructureInt1.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(LSM6DS3_INT1_GPIO_PORT, &GPIO_InitStructureInt1);

  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(LSM6DS3_INT1_EXTI_IRQn, 0x00, 0x00);
  HAL_NVIC_EnableIRQ(LSM6DS3_INT1_EXTI_IRQn);

  /* Enable INT2 GPIO clock */
  LSM6DS3_INT2_GPIO_CLK_ENABLE();

  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructureInt2.Pin = LSM6DS3_INT2_PIN;
  GPIO_InitStructureInt2.Mode = GPIO_MODE_IT_RISING;
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
  GPIO_InitStructureInt2.Speed = GPIO_SPEED_FAST;
#endif

#if (defined (USE_STM32L1XX_NUCLEO))
  GPIO_InitStructureInt2.Speed = GPIO_SPEED_MEDIUM;
#endif
  GPIO_InitStructureInt2.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(LSM6DS3_INT2_GPIO_PORT, &GPIO_InitStructureInt2);

  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(LSM6DS3_INT2_EXTI_IRQn, 0x00, 0x00);
  HAL_NVIC_EnableIRQ(LSM6DS3_INT2_EXTI_IRQn);

  return COMPONENT_OK;
}



/**
 * @brief  Configures sensor interrupts interface for LPS22HB sensor.
 * @param  None
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef LPS22HB_Sensor_IO_ITConfig( void )
{

#if 0
  /* At the moment this feature is only implemented for LPS22HB */
  GPIO_InitTypeDef GPIO_InitStructureInt1;
  /* Enable INT1 GPIO clock */
  LPS22HB_INT1_GPIO_CLK_ENABLE();

  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructureInt1.Pin = LPS22HB_INT1_PIN;
  GPIO_InitStructureInt1.Mode = GPIO_MODE_IT_RISING;
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
  GPIO_InitStructureInt1.Speed = GPIO_SPEED_FAST;
#endif

#if (defined (USE_STM32L1XX_NUCLEO))
  GPIO_InitStructureInt1.Speed = GPIO_SPEED_MEDIUM;
#endif
  GPIO_InitStructureInt1.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(LPS22HB_INT1_GPIO_PORT, &GPIO_InitStructureInt1);

  /* Enable and set EXTI Interrupt priority */
  HAL_NVIC_SetPriority(LPS22HB_INT1_EXTI_IRQn, 0x00, 0x00);
  HAL_NVIC_EnableIRQ(LPS22HB_INT1_EXTI_IRQn);
#endif

  return COMPONENT_OK;
}

/**
 * @}
 */


/** @addtogroup LRWAN_NS1_IO_Private_Functions Private functions
 * @{
 */



/******************************* Link functions *******************************/


/**
 * @brief  Writes a buffer to the sensor
 * @param  handle instance handle
 * @param  WriteAddr specifies the internal sensor address register to be written to
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToWrite number of bytes to be written
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  /* call I2C_EXPBD Read data bus function */
  if ( I2C_EXPBD_WriteData( ctx->address, WriteAddr, pBuffer, nBytesToWrite ) )
  {
    return 1;
  }
  else
  {
    return 0;
  }
}



/**
 * @brief  Reads a from the sensor to buffer
 * @param  handle instance handle
 * @param  ReadAddr specifies the internal sensor address register to be read from
 * @param  pBuffer pointer to data buffer
 * @param  nBytesToRead number of bytes to be read
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
uint8_t Sensor_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  /* call I2C_EXPBD Read data bus function */
  if ( I2C_EXPBD_ReadData( ctx->address, ReadAddr, pBuffer, nBytesToRead ) )
  {
    return 1;
  }
  else
  {
    return 0;
  }
}



/******************************* I2C Routines *********************************/

/**
 * @brief  Configures I2C interface.
 * @param  None
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
static uint8_t I2C_EXPBD_Init( void )
{
  if(HAL_I2C_GetState( &I2C_EXPBD_Handle) == HAL_I2C_STATE_RESET )
  {

    /* I2C_EXPBD peripheral configuration */

#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
    I2C_EXPBD_Handle.Init.ClockSpeed = NUCLEO_I2C_EXPBD_SPEED;
    I2C_EXPBD_Handle.Init.DutyCycle = I2C_DUTYCYCLE_2;
#endif

#if (defined (USE_STM32L0XX_NUCLEO))
    I2C_EXPBD_Handle.Init.Timing = NUCLEO_I2C_EXPBD_TIMING_400KHZ;    /* 400KHz */
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
    I2C_EXPBD_Handle.Init.Timing = NUCLEO_I2C_EXPBD_TIMING_400KHZ;    /* 400KHz */
#endif

    I2C_EXPBD_Handle.Init.OwnAddress1    = 0x33;
    I2C_EXPBD_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    I2C_EXPBD_Handle.Instance            = NUCLEO_I2C_EXPBD;

    /* Init the I2C */
    I2C_EXPBD_MspInit();
    HAL_I2C_Init( &I2C_EXPBD_Handle );
  }

  if( HAL_I2C_GetState( &I2C_EXPBD_Handle) == HAL_I2C_STATE_READY )
  {
    return 0;
  }
  else
  {
    return 1;
  }
}



/**
 * @brief  Write data to the register of the device through BUS
 * @param  Addr Device address on BUS
 * @param  Reg The target register address to be written
 * @param  pBuffer The data to be written
 * @param  Size Number of bytes to be written
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
static uint8_t I2C_EXPBD_WriteData( uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size )
{

  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write( &I2C_EXPBD_Handle, Addr, ( uint16_t )Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Size, I2C_EXPBD_Timeout );

  /* Check the communication status */
  if( status != HAL_OK )
  {

    /* Execute user timeout callback */
    I2C_EXPBD_Error( Addr );
    return 1;
  }
  else
  {
    return 0;
  }
}



/**
 * @brief  Read a register of the device through BUS
 * @param  Addr Device address on BUS
 * @param  Reg The target register address to read
 * @param  pBuffer The data to be read
 * @param  Size Number of bytes to be read
 * @retval 0 in case of success
 * @retval 1 in case of failure
 */
static uint8_t I2C_EXPBD_ReadData( uint8_t Addr, uint8_t Reg, uint8_t* pBuffer, uint16_t Size )
{

  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Read( &I2C_EXPBD_Handle, Addr, ( uint16_t )Reg, I2C_MEMADD_SIZE_8BIT, pBuffer, Size,
                             I2C_EXPBD_Timeout );

  /* Check the communication status */
  if( status != HAL_OK )
  {

    /* Execute user timeout callback */
    I2C_EXPBD_Error( Addr );
    return 1;
  }
  else
  {
    return 0;
  }
}



/**
 * @brief  Manages error callback by re-initializing I2C
 * @param  Addr I2C Address
 * @retval None
 */
static void I2C_EXPBD_Error( uint8_t Addr )
{

  /* De-initialize the I2C comunication bus */
  HAL_I2C_DeInit( &I2C_EXPBD_Handle );

  /* Re-Initiaize the I2C comunication bus */
  I2C_EXPBD_Init();
}



/**
 * @brief I2C MSP Initialization
 * @param None
 * @retval None
 */

static void I2C_EXPBD_MspInit( void )
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable I2C GPIO clocks */
  NUCLEO_I2C_EXPBD_SCL_SDA_GPIO_CLK_ENABLE();

  /* I2C_EXPBD SCL and SDA pins configuration -------------------------------------*/
  GPIO_InitStruct.Pin        = NUCLEO_I2C_EXPBD_SCL_PIN | NUCLEO_I2C_EXPBD_SDA_PIN;
  GPIO_InitStruct.Mode       = GPIO_MODE_AF_OD;
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
#endif

#if (defined (USE_STM32L1XX_NUCLEO))
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
#endif
  GPIO_InitStruct.Pull       = GPIO_NOPULL;
  GPIO_InitStruct.Alternate  = NUCLEO_I2C_EXPBD_SCL_SDA_AF;

  HAL_GPIO_Init( NUCLEO_I2C_EXPBD_SCL_SDA_GPIO_PORT, &GPIO_InitStruct );

  /* Enable the I2C_EXPBD peripheral clock */
  NUCLEO_I2C_EXPBD_CLK_ENABLE();

  /* Force the I2C peripheral clock reset */
  NUCLEO_I2C_EXPBD_FORCE_RESET();

  /* Release the I2C peripheral clock reset */
  NUCLEO_I2C_EXPBD_RELEASE_RESET();

  /* Enable and set I2C_EXPBD Interrupt to the highest priority */
  HAL_NVIC_SetPriority(NUCLEO_I2C_EXPBD_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(NUCLEO_I2C_EXPBD_EV_IRQn);

#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
  /* Enable and set I2C_EXPBD Interrupt to the highest priority */
  HAL_NVIC_SetPriority(NUCLEO_I2C_EXPBD_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(NUCLEO_I2C_EXPBD_ER_IRQn);
#endif

}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
