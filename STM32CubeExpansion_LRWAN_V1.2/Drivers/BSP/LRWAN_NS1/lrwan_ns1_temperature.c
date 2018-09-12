/**
 ******************************************************************************
 * @file    lrwan_ns1_temperature.c
 * @author  MEMS Application Team
 * @version V1.0.3
 * @date    20-December-2017
 * @brief   This file provides a set of functions needed to manage the temperature sensor
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
#include "lrwan_ns1_temperature.h"



/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup LRWAN_NS1 LRWAN_NS1
 * @{
 */

/** @addtogroup LRWAN_NS1_TEMPERATURE Temperature
 * @{
 */

/** @addtogroup LRWAN_NS1_TEMPERATURE_Private_Variables Private variables
 * @{
 */

static DrvContextTypeDef TEMPERATURE_SensorHandle[ TEMPERATURE_SENSORS_MAX_NUM ];
static TEMPERATURE_Data_t TEMPERATURE_Data[ TEMPERATURE_SENSORS_MAX_NUM ]; // Temperature - all.
static HTS221_T_Data_t HTS221_T_0_Data; // Temperature - sensor 0 HTS221 on board.
static LPS25HB_T_Data_t LPS25HB_T_0_Data; // Temperature - sensor 1 LPS25H/B on board.
static LPS25HB_T_Data_t LPS25HB_T_1_Data; // Temperature - sensor 2 LPS25H/B via DIL24.
static LPS22HB_T_Data_t LPS22HB_T_0_Data; // Temperature - sensor 3 LPS22H/B via DIL24.

/**
 * @}
 */

/** @addtogroup LRWAN_NS1_TEMPERATURE_Private_FunctionPrototypes Private function prototypes
 * @{
 */

static DrvStatusTypeDef BSP_HTS221_TEMPERATURE_Init( void **handle );
static DrvStatusTypeDef BSP_LPS25HB_TEMPERATURE_Init( int id, void **handle );
static DrvStatusTypeDef BSP_LPS22HB_TEMPERATURE_Init( void **handle );

/**
 * @}
 */

/** @addtogroup LRWAN_NS1_TEMPERATURE_Public_Functions Public functions
 * @{
 */

/**
 * @brief Initialize a temperature sensor
 * @param id the temperature sensor identifier
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Init( TEMPERATURE_ID_t id, void **handle )
{

  *handle = NULL;

  switch(id)
  {
    case TEMPERATURE_SENSORS_AUTO:
    default:
    {
      /* Try to init the LPS22HB DIL24 first */
      if( BSP_LPS22HB_TEMPERATURE_Init(handle) == COMPONENT_ERROR )
      {
        /* Try to init the LPS25H/B DIL24 if we do not use the LPS22HB DIL24 */
        if( BSP_LPS25HB_TEMPERATURE_Init(LPS25HB_T_1, handle) == COMPONENT_ERROR )
        {
          /* Try to init the HTS221 on board if we do not use the LPS22HB DIL24 and LPS25H/B DIL24 */
          if( BSP_HTS221_TEMPERATURE_Init(handle) == COMPONENT_ERROR )
          {
            /* Try to init the LPS25H/B on board if we do not use the LPS22HB DIL24 and LPS25H/B DIL24 and the HTS221 on board */
            if(BSP_LPS25HB_TEMPERATURE_Init(LPS25HB_T_0, handle) == COMPONENT_ERROR )
            {
              return COMPONENT_ERROR;
            }
          }
        }
      }
      break;
    }
    case HTS221_T_0:
    {
      if( BSP_HTS221_TEMPERATURE_Init(handle) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    }
    case LPS25HB_T_0:
    {
      if(BSP_LPS25HB_TEMPERATURE_Init(LPS25HB_T_0, handle) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    }
    case LPS25HB_T_1:
    {
      if(BSP_LPS25HB_TEMPERATURE_Init(LPS25HB_T_1, handle) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    }
    case LPS22HB_T_0:
    {
      if(BSP_LPS22HB_TEMPERATURE_Init(handle) == COMPONENT_ERROR )
      {
        return COMPONENT_ERROR;
      }
      break;
    }
  }

  return COMPONENT_OK;
}


/**
 * @brief Initialize HTS221 temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef BSP_HTS221_TEMPERATURE_Init( void **handle )
{
  TEMPERATURE_Drv_t *driver = NULL;
#if 0
  if(TEMPERATURE_SensorHandle[ HTS221_T_0 ].isInitialized == 1)
  {
    /* We have reached the max num of instance for this component */
    return COMPONENT_ERROR;
  }
#endif

  if ( Sensor_IO_Init() == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Setup sensor handle. */
  TEMPERATURE_SensorHandle[ HTS221_T_0 ].who_am_i      = HTS221_WHO_AM_I_VAL;
  TEMPERATURE_SensorHandle[ HTS221_T_0 ].address       = HTS221_ADDRESS_DEFAULT;
  TEMPERATURE_SensorHandle[ HTS221_T_0 ].instance      = HTS221_T_0;
  TEMPERATURE_SensorHandle[ HTS221_T_0 ].isInitialized = 0;
  TEMPERATURE_SensorHandle[ HTS221_T_0 ].isEnabled     = 0;
  TEMPERATURE_SensorHandle[ HTS221_T_0 ].isCombo       = 1;
  TEMPERATURE_SensorHandle[ HTS221_T_0 ].pData         = ( void * )&TEMPERATURE_Data[ HTS221_T_0 ];
  TEMPERATURE_SensorHandle[ HTS221_T_0 ].pVTable       = ( void * )&HTS221_T_Drv;
  TEMPERATURE_SensorHandle[ HTS221_T_0 ].pExtVTable    = 0;

  HTS221_T_0_Data.comboData = &HTS221_Combo_Data[0];
  TEMPERATURE_Data[ HTS221_T_0 ].pComponentData = ( void * )&HTS221_T_0_Data;
  TEMPERATURE_Data[ HTS221_T_0 ].pExtData       = 0;

  *handle = (void *)&TEMPERATURE_SensorHandle[ HTS221_T_0 ];

  driver = ( TEMPERATURE_Drv_t * )((DrvContextTypeDef *)(*handle))->pVTable;

  if ( driver->Init == NULL )
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  if ( driver->Init( (DrvContextTypeDef *)(*handle) ) == COMPONENT_ERROR )
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Initialize LPS25HB temperature sensor
 * @param id the temperature sensor identifier
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef BSP_LPS25HB_TEMPERATURE_Init( int id, void **handle )
{
  TEMPERATURE_Drv_t *driver = NULL;

  if(TEMPERATURE_SensorHandle[ id ].isInitialized == 1)
  {
    /* We have reached the max num of instance for this component */
    return COMPONENT_ERROR;
  }

  if ( Sensor_IO_Init() == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Setup sensor handle. */
  switch(id)
  {
    case LPS25HB_T_0:
      TEMPERATURE_SensorHandle[ LPS25HB_T_0 ].who_am_i      = LPS25HB_WHO_AM_I_VAL;
      TEMPERATURE_SensorHandle[ LPS25HB_T_0 ].address       = LPS25HB_ADDRESS_HIGH;
      TEMPERATURE_SensorHandle[ LPS25HB_T_0 ].instance      = LPS25HB_T_0;
      TEMPERATURE_SensorHandle[ LPS25HB_T_0 ].isInitialized = 0;
      TEMPERATURE_SensorHandle[ LPS25HB_T_0 ].isEnabled     = 0;
      TEMPERATURE_SensorHandle[ LPS25HB_T_0 ].isCombo       = 1;
      TEMPERATURE_SensorHandle[ LPS25HB_T_0 ].pData         = ( void * )&TEMPERATURE_Data[ LPS25HB_T_0 ];
      TEMPERATURE_SensorHandle[ LPS25HB_T_0 ].pVTable       = ( void * )&LPS25HB_T_Drv;
      TEMPERATURE_SensorHandle[ LPS25HB_T_0 ].pExtVTable    = 0;

      LPS25HB_T_0_Data.comboData = &LPS25HB_Combo_Data[0];
      TEMPERATURE_Data[ LPS25HB_T_0 ].pComponentData = ( void * )&LPS25HB_T_0_Data;
      TEMPERATURE_Data[ LPS25HB_T_0 ].pExtData       = 0;

      *handle = (void *)&TEMPERATURE_SensorHandle[ LPS25HB_T_0 ];
      break;
    case LPS25HB_T_1:
      TEMPERATURE_SensorHandle[ LPS25HB_T_1 ].who_am_i      = LPS25HB_WHO_AM_I_VAL;
      TEMPERATURE_SensorHandle[ LPS25HB_T_1 ].address       = LPS25HB_ADDRESS_LOW;
      TEMPERATURE_SensorHandle[ LPS25HB_T_1 ].instance      = LPS25HB_T_1;
      TEMPERATURE_SensorHandle[ LPS25HB_T_1 ].isInitialized = 0;
      TEMPERATURE_SensorHandle[ LPS25HB_T_1 ].isEnabled     = 0;
      TEMPERATURE_SensorHandle[ LPS25HB_T_1 ].isCombo       = 1;
      TEMPERATURE_SensorHandle[ LPS25HB_T_1 ].pData         = ( void * )&TEMPERATURE_Data[ LPS25HB_T_1 ];
      TEMPERATURE_SensorHandle[ LPS25HB_T_1 ].pVTable       = ( void * )&LPS25HB_T_Drv;
      TEMPERATURE_SensorHandle[ LPS25HB_T_1 ].pExtVTable    = 0;

      LPS25HB_T_1_Data.comboData = &LPS25HB_Combo_Data[1];
      TEMPERATURE_Data[ LPS25HB_T_1 ].pComponentData = ( void * )&LPS25HB_T_1_Data;
      TEMPERATURE_Data[ LPS25HB_T_1 ].pExtData       = 0;

      *handle = (void *)&TEMPERATURE_SensorHandle[ LPS25HB_T_1 ];
      break;
  }

  driver = ( TEMPERATURE_Drv_t * )((DrvContextTypeDef *)(*handle))->pVTable;

  if ( driver->Init == NULL )
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  if ( driver->Init( (DrvContextTypeDef *)(*handle) ) == COMPONENT_ERROR )
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Initialize LPS22HB temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
static DrvStatusTypeDef BSP_LPS22HB_TEMPERATURE_Init( void **handle )
{
  TEMPERATURE_Drv_t *driver = NULL;

  if(TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].isInitialized == 1)
  {
    /* We have reached the max num of instance for this component */
    return COMPONENT_ERROR;
  }

  if ( Sensor_IO_Init() == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  /* Setup sensor handle. */
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].who_am_i      = LPS22HB_WHO_AM_I_VAL;
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].address       = LPS22HB_ADDRESS_HIGH;         //rhf LPS22HB_ADDRESS_LOW;
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].instance      = LPS22HB_T_0;
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].isInitialized = 0;
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].isEnabled     = 0;
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].isCombo       = 1;
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].pData         = ( void * )&TEMPERATURE_Data[ LPS22HB_T_0 ];
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].pVTable       = ( void * )&LPS22HB_T_Drv;
  TEMPERATURE_SensorHandle[ LPS22HB_T_0 ].pExtVTable    = ( void * )&LPS22HB_T_ExtDrv;

  LPS22HB_T_0_Data.comboData = &LPS22HB_Combo_Data[0];
  TEMPERATURE_Data[ LPS22HB_T_0 ].pComponentData = ( void * )&LPS22HB_T_0_Data;
  TEMPERATURE_Data[ LPS22HB_T_0 ].pExtData       = 0;

  *handle = (void *)&TEMPERATURE_SensorHandle[ LPS22HB_T_0 ];

  driver = ( TEMPERATURE_Drv_t * )((DrvContextTypeDef *)(*handle))->pVTable;

  if ( driver->Init == NULL )
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  if ( driver->Init( (DrvContextTypeDef *)(*handle) ) == COMPONENT_ERROR )
  {
    memset((*handle), 0, sizeof(DrvContextTypeDef));
    *handle = NULL;
    return COMPONENT_ERROR;
  }

  /* Configure interrupt lines for LPS22HB */
  LPS22HB_Sensor_IO_ITConfig();

  return COMPONENT_OK;
}


/**
 * @brief Deinitialize a temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_DeInit( void **handle )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)(*handle);
  TEMPERATURE_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( TEMPERATURE_Drv_t * )ctx->pVTable;

  if ( driver->DeInit == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->DeInit( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  memset(ctx, 0, sizeof(DrvContextTypeDef));

  *handle = NULL;

  return COMPONENT_OK;
}


/**
 * @brief Enable temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Sensor_Enable( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( TEMPERATURE_Drv_t * )ctx->pVTable;

  if ( driver->Sensor_Enable == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Sensor_Enable( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Disable temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Sensor_Disable( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( TEMPERATURE_Drv_t * )ctx->pVTable;

  if ( driver->Sensor_Disable == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Sensor_Disable( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Check if the temperature sensor is initialized
 * @param handle the device handle
 * @param status the pointer to the initialization status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_IsInitialized( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isInitialized;

  return COMPONENT_OK;
}


/**
 * @brief Check if the temperature sensor is enabled
 * @param handle the device handle
 * @param status the pointer to the enable status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_IsEnabled( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isEnabled;

  return COMPONENT_OK;
}


/**
 * @brief Check if the temperature sensor is combo
 * @param handle the device handle
 * @param status the pointer to the combo status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_IsCombo( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }

  *status = ctx->isCombo;

  return COMPONENT_OK;
}


/**
 * @brief Get the temperature sensor instance
 * @param handle the device handle
 * @param instance the pointer to the device instance
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Get_Instance( void *handle, uint8_t *instance )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( instance == NULL )
  {
    return COMPONENT_ERROR;
  }

  *instance = ctx->instance;

  return COMPONENT_OK;
}



/**
 * @brief Get the WHO_AM_I ID of the temperature sensor
 * @param handle the device handle
 * @param who_am_i pointer to the value of WHO_AM_I register
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Get_WhoAmI( void *handle, uint8_t *who_am_i )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( TEMPERATURE_Drv_t * )ctx->pVTable;

  if ( who_am_i == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_WhoAmI == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_WhoAmI( ctx, who_am_i ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Check the WHO_AM_I ID of the temperature sensor
 * @param handle the device handle
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Check_WhoAmI( void *handle )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( TEMPERATURE_Drv_t * )ctx->pVTable;

  if ( driver->Check_WhoAmI == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Check_WhoAmI( ctx ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Get the temperature value
 * @param handle the device handle
 * @param temperature pointer where the value is written [C]
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Get_Temp( void *handle, float *temperature )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( TEMPERATURE_Drv_t * )ctx->pVTable;

  if ( temperature == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_Temp == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_Temp( ctx, temperature ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Get the temperature sensor output data rate
 * @param handle the device handle
 * @param odr pointer where the output data rate is written
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Get_ODR( void *handle, float *odr )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( TEMPERATURE_Drv_t * )ctx->pVTable;

  if ( odr == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_ODR == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Get_ODR( ctx, odr ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Set the temperature sensor output data rate
 * @param handle the device handle
 * @param odr the functional output data rate to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Set_ODR( void *handle, SensorOdr_t odr )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( TEMPERATURE_Drv_t * )ctx->pVTable;

  if ( driver->Set_ODR == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Set_ODR( ctx, odr ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Set the temperature sensor output data rate
 * @param handle the device handle
 * @param odr the output data rate value to be set
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Set_ODR_Value( void *handle, float odr )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( TEMPERATURE_Drv_t * )ctx->pVTable;

  if ( driver->Set_ODR_Value == NULL )
  {
    return COMPONENT_ERROR;
  }
  if ( driver->Set_ODR_Value( ctx, odr ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Read the data from register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Read_Reg( void *handle, uint8_t reg, uint8_t *data )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( TEMPERATURE_Drv_t * )ctx->pVTable;

  if(data == NULL)
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Read_Reg == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Read_Reg( ctx, reg, data ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Write the data to register
 * @param handle the device handle
 * @param reg register address
 * @param data register data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Write_Reg( void *handle, uint8_t reg, uint8_t data )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( TEMPERATURE_Drv_t * )ctx->pVTable;

  if ( driver->Write_Reg == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Write_Reg( ctx, reg, data ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Get temperature data ready status
 * @param handle the device handle
 * @param status the data ready status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_Get_DRDY_Status( void *handle, uint8_t *status )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
  TEMPERATURE_Drv_t *driver = NULL;

  if(ctx == NULL)
  {
    return COMPONENT_ERROR;
  }

  driver = ( TEMPERATURE_Drv_t * )ctx->pVTable;

  if ( driver->Get_DRDY_Status == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( driver->Get_DRDY_Status( ctx, status ) == COMPONENT_ERROR )
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief Get FIFO THR status (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param *status FIFO THR status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Get_Fth_Status_Ext( void *handle, uint8_t *status )
{
  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if ( ctx->who_am_i == LPS22HB_WHO_AM_I_VAL )
  {
    LPS22HB_T_ExtDrv_t *extDriver = ( LPS22HB_T_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->FIFO_Get_Fth_Status == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Fth_Status( ctx, status );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get FIFO FULL status (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param *status FIFO FULL status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Get_Full_Status_Ext( void *handle, uint8_t *status )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if ( ctx->who_am_i == LPS22HB_WHO_AM_I_VAL )
  {
    LPS22HB_T_ExtDrv_t *extDriver = ( LPS22HB_T_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->FIFO_Get_Full_Status == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Full_Status( ctx, status );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get FIFO OVR status (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param *status FIFO OVR status
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Get_Ovr_Status_Ext( void *handle, uint8_t *status )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( status == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if ( ctx->who_am_i == LPS22HB_WHO_AM_I_VAL )
  {
    LPS22HB_T_ExtDrv_t *extDriver = ( LPS22HB_T_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->FIFO_Get_Ovr_Status == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Ovr_Status( ctx, status );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get FIFO data (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param *pressure pointer to FIFO pressure data
 * @param *temperature pointer to FIFO temperature data
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Get_Data_Ext( void *handle, float *pressure, float *temperature )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( pressure == NULL || temperature == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if ( ctx->who_am_i == LPS22HB_WHO_AM_I_VAL )
  {
    LPS22HB_T_ExtDrv_t *extDriver = ( LPS22HB_T_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->FIFO_Get_Data == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Data( ctx, pressure, temperature );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Get number of unread FIFO samples (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param *nSamples Number of unread FIFO samples
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Get_Num_Of_Samples_Ext( void *handle, uint8_t *nSamples )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( nSamples == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if ( ctx->who_am_i == LPS22HB_WHO_AM_I_VAL )
  {
    LPS22HB_T_ExtDrv_t *extDriver = ( LPS22HB_T_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->FIFO_Get_Num_Of_Samples == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Get_Num_Of_Samples( ctx, nSamples );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set FIFO mode (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param mode FIFO mode
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Set_Mode_Ext( void *handle, uint8_t mode )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if ( ctx->who_am_i == LPS22HB_WHO_AM_I_VAL )
  {
    LPS22HB_T_ExtDrv_t *extDriver = ( LPS22HB_T_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->FIFO_Set_Mode == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Set_Mode( ctx, mode );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set FIFO interrupt (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param interrupt FIFO interrupt
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Set_Interrupt_Ext( void *handle, uint8_t interrupt )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if ( ctx->who_am_i == LPS22HB_WHO_AM_I_VAL )
  {
    LPS22HB_T_ExtDrv_t *extDriver = ( LPS22HB_T_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->FIFO_Set_Interrupt == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Set_Interrupt( ctx, interrupt );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set FIFO interrupt (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param interrupt FIFO interrupt
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Reset_Interrupt_Ext( void *handle, uint8_t interrupt )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if ( ctx->who_am_i == LPS22HB_WHO_AM_I_VAL )
  {
    LPS22HB_T_ExtDrv_t *extDriver = ( LPS22HB_T_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->FIFO_Reset_Interrupt == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Reset_Interrupt( ctx, interrupt );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set FIFO watermark (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param watermark FIFO watermark
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Set_Watermark_Level_Ext( void *handle, uint8_t watermark )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if ( ctx->who_am_i == LPS22HB_WHO_AM_I_VAL )
  {
    LPS22HB_T_ExtDrv_t *extDriver = ( LPS22HB_T_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->FIFO_Set_Watermark_Level == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Set_Watermark_Level( ctx, watermark );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief Set FIFO to stop on FTH (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param status enable or disable stopping on FTH interrupt
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Stop_On_Fth_Ext( void *handle, uint8_t status )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if ( ctx->who_am_i == LPS22HB_WHO_AM_I_VAL )
  {
    LPS22HB_T_ExtDrv_t *extDriver = ( LPS22HB_T_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->FIFO_Stop_On_Fth == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Stop_On_Fth( ctx, status );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
}



/**
 * @brief FIFO usage (available only for LPS22HB sensor)
 * @param handle the device handle
 * @param status enable or disable FIFO
 * @retval COMPONENT_OK in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
DrvStatusTypeDef BSP_TEMPERATURE_FIFO_Usage_Ext( void *handle, uint8_t status )
{

  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;

  if ( ctx == NULL )
  {
    return COMPONENT_ERROR;
  }

  if ( ctx->pExtVTable == NULL )
  {
    return COMPONENT_ERROR;
  }

  /* At the moment this feature is only implemented for LPS22HB */
  if ( ctx->who_am_i == LPS22HB_WHO_AM_I_VAL )
  {
    LPS22HB_T_ExtDrv_t *extDriver = ( LPS22HB_T_ExtDrv_t * )ctx->pExtVTable;

    if ( extDriver->FIFO_Usage == NULL )
    {
      return COMPONENT_ERROR;
    }

    else
    {
      return extDriver->FIFO_Usage( ctx, status );
    }
  }

  else
  {
    return COMPONENT_ERROR;
  }
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
