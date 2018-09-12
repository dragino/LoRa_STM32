 /******************************************************************************
  * @file    test_tx.c
  * @author  MCD Application Team
  * @version V1.1.4
  * @date    08-January-2018
  * @brief   manages tx tests
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
  
#include "hw.h"
#include "vcom.h"
#include <stdarg.h>
#include "radio.h"
#include "sx1276.h"
#include "mlm32l07x01.h"
#include "at.h"
#include "test_rf.h"
#include "tiny_sscanf.h"


/* Private typedef -----------------------------------------------------------*/
  typedef struct
  {
    uint32_t freqMHz;
    uint32_t power;
    uint32_t bandwidth;                                      // [0: 125 kHz,
                                                             //  1: 250 kHz,
                                                             //  2: 500 kHz,
                                                             //  3: Reserved]
    uint32_t sf;                                             // [SF7..SF12]
    uint32_t codingRate;                                     // [1: 4/5,
                                                             //  2: 4/6,
                                                             //  3: 4/7,
                                                             //  4: 4/8]
    uint32_t lna;                                     // 0:off 1:On
    uint32_t paBoost;                                     // 0:off 1:On
  } s_loraParameter_t;
  
typedef enum
{
  BW_125kHz=0,
  BW_250kHz=1,
  BW_500kHz=2,
} e_BandWidth_t;

typedef enum
{
  CR_4o5=1,
  CR_4o6=2,
  CR_4o7=3,
  CR_4o8=4,
} e_CodingRates_t;
typedef enum
{
  SF_7  =7,
  SF_8  =8,
  SF_9  =9,
  SF_10 =10,
  SF_11 =11,
  SF_12 =12,
} e_SpreadingFactors_t;

/* Private define ------------------------------------------------------------*/
#define F_868MHz 868

#define P_14dBm 14
  
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#define TX_TEST_TONE    1<<0
#define RX_TEST_RSSI    1<<1
#define TX_TEST_LORA  1<<2
#define RX_TEST_LORA  1<<3
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t TestState =0;

static s_loraParameter_t loraParam= { F_868MHz, P_14dBm, BW_125kHz, SF_12, 4, 0, 0};

/* Private function prototypes -----------------------------------------------*/

static bool is_in_list( uint32_t in, uint32_t* list, uint32_t list_len);
/* Functions Definition ------------------------------------------------------*/

/* receive test functions */

ATEerror_t TST_TxTone(const char *buf, unsigned bufSize)
{
  uint8_t paboost = loraParam.paBoost;
  
  if ( (TestState & TX_TEST_TONE) != TX_TEST_TONE )
  {
    TestState |= TX_TEST_TONE;

    PRINTF("Tx Test\n\r");
    
    SX1276SetModem( MODEM_FSK );
  
    Radio.SetChannel( loraParam.freqMHz * 1000000 );
        
    Radio.Write( REG_FDEVMSB, 0x00 );                           // FdevMsb = 0
    Radio.Write( REG_FDEVLSB, 0x00 );                           // FdevLsb = 0
        
    // SX1276 in continuous mode FSK                
    Radio.Write( REG_PACKETCONFIG2, ( Radio.Read( REG_PACKETCONFIG2 ) & RF_PACKETCONFIG2_DATAMODE_MASK ) );  
  
    switch (loraParam.power)
    {
      case 20:
      {
        Radio.Write( REG_PACONFIG, 0xFF );                             // PA_Boost 17 dBm
        Radio.Write( REG_PADAC, ( Radio.Read( REG_PADAC ) & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON );  // Enable 20dBm boost
        PRINTF("force PA boost Output\n\r");
        break;
      }
      case 17:
      {
        Radio.Write( REG_PACONFIG, 0xFF );                           // PA_Boost 17 dBm
        Radio.Write( REG_PADAC, ( Radio.Read( REG_PADAC ) & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF );  // Disable 20dBm boost
        PRINTF("force PA boost Output\n\r");
        break;
      }
      case 14 :
      {
        if ( paboost == 1 )
        {
          Radio.Write( REG_PACONFIG, 0xFB );                           // PA_Boost 14 dBm
          Radio.Write( REG_PADAC, ( Radio.Read( REG_PADAC ) & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF );  // Disable 20dBm boost  
        }
        else
        {
        //selects RFO
          Radio.Write( REG_PACONFIG, 0x7F );                           // RFO 14 dBm
        }
        break;
      }
      case 10:
      {
        if ( paboost == 1 )
        {
            Radio.Write( REG_PACONFIG, 0xF7 );                        // PA_Boost 10 dBm
            Radio.Write( REG_PADAC, ( Radio.Read( REG_PADAC ) & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF );  // Disable 20dBm boost 
        }
        else
        {
            Radio.Write( REG_PACONFIG, 0x7A );                        // RFO 10 dBm
        }
        break;
      }
      case 7 :
      {
        if ( paboost == 1 )
        {
            Radio.Write( REG_PACONFIG, 0xF5 );                                // PA_Boost 7 dBm
            Radio.Write( REG_PADAC, ( Radio.Read( REG_PADAC ) & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF );  // Disable 20dBm boost 
        }
        else
        {
            Radio.Write( REG_PACONFIG, 0x78 );                                // RFO 7 dBm
        }
        break;
      }
      default:
        break;
    }  
    SX1276SetOpMode( RF_OPMODE_TRANSMITTER );
    return AT_OK;
  }
  else
  {
    return AT_BUSY_ERROR;
  }
}


/* receive test functions */

ATEerror_t TST_RxTone(const char *buf, unsigned bufSize)
{
  /* check that test is not already started*/
  if ( (TestState & RX_TEST_RSSI) != RX_TEST_RSSI )
  {
    TestState |= RX_TEST_RSSI;
    PRINTF("Rx Test\n\r");  
    
     SX1276SetModem( MODEM_FSK );
  
    Radio.SetChannel( loraParam.freqMHz * 1000000 );

    Radio.Write( REG_BITRATEMSB, 0x1A );           // bitrate =  4800 bps
    Radio.Write( REG_BITRATELSB, 0x0B );           // 
                
    Radio.Write( REG_FDEVMSB, 0x00 );              // Frequency deviation = 5 KHz
    Radio.Write( REG_FDEVLSB, 0x52 );              //
  
    if (loraParam.lna ==0 )
    {
      Radio.Write( REG_LR_LNA,0x20); //LNA off
      PRINTF(">>> LNA is OFF\n\r");
    }
    else// if (lnaState==1)
    {
      PRINTF(">>> LNA is ON\n\r");
      Radio.Write( REG_LR_LNA,0x23); //LNA on
    }

  // SX1276 in continuous mode FSK                
    Radio.Write( REG_PACKETCONFIG2, ( Radio.Read( REG_PACKETCONFIG2 ) & RF_PACKETCONFIG2_DATAMODE_MASK ) );  

    Radio.Write( REG_RSSICONFIG, 0x07 );//256 samples average

    SX1276SetOpMode( RF_OPMODE_RECEIVER );

    Radio.Write( REG_RXCONFIG, 0x40  ); //Triggers a manual Restart of the Receiver chain
    
    return AT_OK;
  }
  else
  {
    return AT_BUSY_ERROR;
  }
}




ATEerror_t TST_SET_lora_config(const char *buf, unsigned bufSize)
{
  s_loraParameter_t loraParamTmp;
  uint32_t sfSet[]={7,8,9,10,11,12};
  uint32_t crDenSet[]={5,6,7,8};
  uint32_t crNum;
  
  if (8 != tiny_sscanf(buf, "%d:%d:%d:%d:%d/%d:%d:%d", 
                       &loraParamTmp.freqMHz, 
                       &loraParamTmp.power, 
                       &loraParamTmp.bandwidth, 
                       &loraParamTmp.sf, 
                       &crNum, 
                       &loraParamTmp.codingRate,
                       &loraParamTmp.lna,
                       &loraParamTmp.paBoost) )
  {
    return AT_PARAM_ERROR; 
  }

  
  loraParam.freqMHz =loraParamTmp.freqMHz;
  loraParam.power =loraParamTmp.power;
  /* check that bandwidth is ok*/
  if ( loraParamTmp.bandwidth== 125 )
  {
    loraParam.bandwidth= BW_125kHz;
  }
  else if( loraParamTmp.bandwidth== 250 )
  {
    loraParam.bandwidth= BW_250kHz;
  }
  else if( loraParamTmp.bandwidth== 500 )
  {
    loraParam.bandwidth= BW_500kHz;
  }
  else
  {
    return AT_PARAM_ERROR; 
  }
  /* check that spreading factor is ok*/
  if ( !is_in_list(loraParamTmp.sf, sfSet, sizeof(sfSet) ) )
  {
    return AT_PARAM_ERROR; 
  }
  else
  {
    loraParam.sf =loraParamTmp.sf;
  }
   /* check coding rate numerator is ok*/
  if (crNum != 4)
  {
    return AT_PARAM_ERROR; 
  }
  /* check coding rate denominator is ok*/
  if ( !is_in_list(loraParamTmp.codingRate, crDenSet, sizeof(crDenSet) ) )
  {
    return AT_PARAM_ERROR; 
  }
  else
  {
    loraParam.codingRate =loraParamTmp.codingRate-4;  
  }
  
  loraParam.freqMHz =loraParamTmp.freqMHz;
  loraParam.lna =loraParamTmp.lna;
  loraParam.paBoost = loraParamTmp.paBoost;
  
  return AT_OK;
}

ATEerror_t TST_get_lora_config(const char *buf, unsigned bufSize)
{
  uint32_t bwSet[]={125, 250, 500};
  AT_PRINTF("Freq= %d MHz\r\n", loraParam.freqMHz);
  AT_PRINTF("Power= %d dBm\r\n", loraParam.power);
  AT_PRINTF("Bandwidth= %d kHz\r\n", bwSet[loraParam.bandwidth] );
  AT_PRINTF("SF= %d \r\n", loraParam.sf);
  AT_PRINTF("CR= 4/%d \r\n", loraParam.codingRate+4);
  AT_PRINTF("LNA State =%d  \r\n", loraParam.lna);
  AT_PRINTF("PA Boost State =%d  \r\n", loraParam.paBoost);

  return AT_OK;
}


ATEerror_t TST_stop( void )
{
  
  if ( (TestState & RX_TEST_RSSI) == RX_TEST_RSSI )
  {
    uint8_t rssiReg =  Radio.Read( REG_RSSIVALUE );
    AT_PRINTF("RSSI=%d,%d dBm\n\r", -(rssiReg/2), rssiReg&0x1?5:0);
  }
  
  TestState = 0;
  
  PRINTF("Test Stop\n\r");
  /* Set the radio in standBy*/
  SX1276SetOpMode( RF_OPMODE_SLEEP );
  
  return AT_OK;
}



ATEerror_t TST_TX_LoraStart(const char *buf, unsigned bufSize)
{
  
  uint8_t bufTx[]={0x00, 0x11, 0x22, 0x33, 
                   0x44, 0x55, 0x66, 0x77, 
                   0x88, 0x99, 0xAA, 0xBB, 
                   0xCC, 0xDD, 0xEE, 0xFF};

  if ( (TestState & TX_TEST_LORA) != TX_TEST_LORA )
  {
    TestState |= TX_TEST_LORA;
    
    Radio.SetModem( MODEM_LORA );
  
    Radio.SetChannel( loraParam.freqMHz * 1000000 );
  // test only
    Radio.SetTxConfig( MODEM_LORA, loraParam.power, 0, loraParam.bandwidth,
                                 loraParam.sf, loraParam.sf,
                                 LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                 true, 0, 0, LORA_IQ_INVERSION_ON, 3000 ); 
  
    Radio.Send( bufTx, sizeof(bufTx) );
  
    return AT_OK;
  }
  else
  {
    return AT_BUSY_ERROR;
  }
}


ATEerror_t TST_RX_LoraStart( void )
{
  if ( (TestState & RX_TEST_LORA) != RX_TEST_LORA )
  {
    TestState |= RX_TEST_LORA;
    
    Radio.SetModem( MODEM_LORA );
  
    Radio.SetChannel( loraParam.freqMHz );

    Radio.SetRxConfig( MODEM_LORA, loraParam.bandwidth, loraParam.sf,
                               loraParam.sf, 0, LORA_PREAMBLE_LENGTH,
                               LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                               0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
  
    if (loraParam.lna ==0 )
    {
      Radio.Write( REG_LR_LNA,0x20); //LNA off
      PRINTF(">>> LNA is OFF\n\r");
    }
    else// if (lnaState==1)
    {
      PRINTF(">>> LNA is ON\n\r");
      Radio.Write( REG_LR_LNA,0x23); //LNA on
    }

    Radio.Rx( 0 );
  
    return AT_OK;
  }
  else
  {
    return AT_BUSY_ERROR;
  }
}


static bool is_in_list( uint32_t in, uint32_t* list, uint32_t list_len)
{
  bool status =false;
  for (int i=0; i <list_len; i++)
  {
    if (list[i] == in)
      status =true;
  }
  return status;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

