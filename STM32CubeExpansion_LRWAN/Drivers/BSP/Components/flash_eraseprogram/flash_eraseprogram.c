 /******************************************************************************
  * @file    oil_float.c
  * @author  MCD Application Team
  * @version V1.1.2
  * @date    01-June-2017
  * @brief   manages the sensors on the application
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
  
  /* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "flash_eraseprogram.h"

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t PAGEError = 0;
__IO uint32_t data32 = 0 ;

/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;	

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Erase the user Flash area
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
void EEPROM_program(uint32_t add, uint32_t *data, uint8_t count)
{
	uint32_t Address=0;
	int i=0;
	Address = add;
	
	BACKUP_PRIMASK();
	
	DISABLE_IRQ( );
	
	HAL_FLASHEx_DATAEEPROM_Unlock();
	while (i<count)
  {
		if(HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD,Address,data[i])== HAL_OK)
		{
			Address = Address + 4;
			i++;
		}
		else
		{
			RESTORE_PRIMASK();
      PRINTF("error in EEPROM Write error\r");
			break;
		}
  }
	HAL_FLASHEx_DATAEEPROM_Lock();
	RESTORE_PRIMASK();
}

void  FLASH_erase(uint32_t page_address)
{
	HAL_FLASH_Unlock();
  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = page_address;
  EraseInitStruct.NbPages     = 1;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
  {
    /*
      Error occurred while page erase.
      User can add here some code to deal with this error.
      PAGEError will contain the faulty page and then to know the code error on this page,
      user can call function 'HAL_FLASH_GetError()'
    */
		
    /* indicate error in Erase operation */
    PRINTF("error in Erase operation\n\r");
  }
/* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
}
void  FLASH_program(uint32_t add, uint32_t *data, uint8_t count)
{
	uint32_t Address=0;
	int i=0;
	
	HAL_FLASH_Unlock();

  Address = add;

  while (i<count)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, data[i]) == HAL_OK)
    {
      Address = Address + 4;
			i++;
		}
    else
    {
      /* Error occurred while writing data in Flash memory.*/
      PRINTF("error in Write operation\n\r");
    }
  }

  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
}
void  FLASH_program_on_addr(uint32_t addr,uint32_t data)
{
	uint32_t Address=0;
	
	HAL_FLASH_Unlock();

  Address = addr;

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, data) == HAL_OK)
    {
      
		}
    else
    {
      /* Error occurred while writing data in Flash memory.*/
      PRINTF("error in Write operation\n\r");
    }

  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
}
uint32_t FLASH_read(uint32_t Address)
{
	  data32 = *(__IO uint32_t *)Address;
		return data32;
}

void EEPROM_erase_one_address(uint32_t address)
{
	BACKUP_PRIMASK();
	
	DISABLE_IRQ( );
	
	HAL_FLASHEx_DATAEEPROM_Unlock();
	
	if(HAL_FLASHEx_DATAEEPROM_Erase(address)!=HAL_OK)
	{
		RESTORE_PRIMASK();
    PRINTF("error in EEPROM Erase operation\n\r");
	}
	
	HAL_FLASHEx_DATAEEPROM_Lock();
	
	RESTORE_PRIMASK();
}

void EEPROM_erase_lora_config(void)
{
	uint32_t address;
	
	address=EEPROM_USER_START_ADDR_CONFIG;
	
	BACKUP_PRIMASK();
	
	DISABLE_IRQ( );
	
	HAL_FLASHEx_DATAEEPROM_Unlock();
	while(address<EEPROM_USER_END_ADDR_CONFIG)
	{
		if(HAL_FLASHEx_DATAEEPROM_Erase(address)!=HAL_OK)
		{
			RESTORE_PRIMASK();
			PRINTF("error in EEPROM Erase operation\n\r");
		}
		address = address + 4;
  }
	
	HAL_FLASHEx_DATAEEPROM_Lock();
	
	RESTORE_PRIMASK();
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
