 /*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: LoRaMac classA device implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Wael Guibene
*/
/******************************************************************************
  * @file    lora-test.c
  * @author  MCD Application Team
  * @version V1.1.4
  * @date    08-January-2018
  * @brief   lora API to drive the lora state Machine
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
#include "LoRaMac.h"
#include "LoRaMacTest.h"
#include "lora.h"
#include "lora-test.h"
#include "timeServer.h"
#include "delay.h"

extern TimerEvent_t TxTimer;

/* Private typedef -----------------------------------------------------------*/
typedef struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    LoraConfirm_t IsTxConfirmed;
    uint8_t DataBufferSize;
    uint8_t DataBuffer[242];
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}ComplianceTest_t;

/* Private define ------------------------------------------------------------*/
#define TEST_TX_DUTYCYCLE 6000
/* Private variables ---------------------------------------------------------*/

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t CertifTxNextPacketTimer;
static ComplianceTest_t certifParam;
static LoraConfirm_t IsTxConfirmed;
static bool AdrEnableInit;

/* Private functions ---------------------------------------------------------*/

static void OnCertifTxNextPacketTimerEvent( void );
static bool certif_tx( void );

/* Exported functions definition---------------------------------------------------------*/
bool certif_running(void)
{
    return certifParam.Running;
}

void certif_DownLinkIncrement( void )
{
    certifParam.DownLinkCounter++;
}

void certif_linkCheck(MlmeConfirm_t *mlmeConfirm)
{
  certifParam.LinkCheck = true;
  certifParam.DemodMargin = mlmeConfirm->DemodMargin;
  certifParam.NbGateways = mlmeConfirm->NbGateways;
}

static bool certif_tx( void )
{
  McpsReq_t mcpsReq;
  LoRaMacTxInfo_t txInfo;
  
  if( certifParam.LinkCheck == true )
  {
    certifParam.LinkCheck = false;
    certifParam.DataBufferSize = 3;
    certifParam.DataBuffer[0] = 5;
    certifParam.DataBuffer[1] = certifParam.DemodMargin;
    certifParam.DataBuffer[2] = certifParam.NbGateways;
    certifParam.State = 1;
  }
  else
  {
    switch( certifParam.State )
    {
    case 4:
      certifParam.State = 1;
      break;
    case 1:
      certifParam.DataBufferSize = 2;
      certifParam.DataBuffer[0] = certifParam.DownLinkCounter >> 8;
      certifParam.DataBuffer[1] = certifParam.DownLinkCounter;
      break;
    }
  }
    
  if( LoRaMacQueryTxPossible( certifParam.DataBufferSize, &txInfo ) != LORAMAC_STATUS_OK )
  {
      // Send empty frame in order to flush MAC commands
      mcpsReq.Type = MCPS_UNCONFIRMED;
      mcpsReq.Req.Unconfirmed.fBuffer = NULL;
      mcpsReq.Req.Unconfirmed.fBufferSize = 0;
      mcpsReq.Req.Unconfirmed.Datarate = DR_0;
  }
  else
  {
      if( IsTxConfirmed == LORAWAN_UNCONFIRMED_MSG )
      {
          mcpsReq.Type = MCPS_UNCONFIRMED;
          mcpsReq.Req.Unconfirmed.fPort = CERTIF_PORT;
          mcpsReq.Req.Unconfirmed.fBufferSize = certifParam.DataBufferSize;
          mcpsReq.Req.Unconfirmed.fBuffer = &(certifParam.DataBuffer);
          mcpsReq.Req.Unconfirmed.Datarate = DR_0;
      }
      else
      {
          mcpsReq.Type = MCPS_CONFIRMED;
          mcpsReq.Req.Confirmed.fPort = CERTIF_PORT;
          mcpsReq.Req.Confirmed.fBufferSize = certifParam.DataBufferSize;
          mcpsReq.Req.Confirmed.fBuffer = &(certifParam.DataBuffer);
          mcpsReq.Req.Confirmed.NbTrials = 8;
          mcpsReq.Req.Confirmed.Datarate = DR_0;
      }
  }

  /*cerification test on-going*/
  TimerStart( &CertifTxNextPacketTimer );
	
  if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
  {
      return false;
  }
    return true;
}

void certif_rx( McpsIndication_t *mcpsIndication, MlmeReqJoin_t* JoinParameters)
{
  if( certifParam.Running == false )
  {
      // Check compliance test enable command (i)
      if( ( mcpsIndication->BufferSize == 4 ) &&
          ( mcpsIndication->Buffer[0] == 0x01 ) &&
          ( mcpsIndication->Buffer[1] == 0x01 ) &&
          ( mcpsIndication->Buffer[2] == 0x01 ) &&
          ( mcpsIndication->Buffer[3] == 0x01 ) )
      {
          MibRequestConfirm_t mibReq;
          IsTxConfirmed = LORAWAN_UNCONFIRMED_MSG;
          certifParam.DataBufferSize = 2;
          certifParam.DownLinkCounter = 0;
          certifParam.LinkCheck = false;
          certifParam.DemodMargin = 0;
          certifParam.NbGateways = 0;
          certifParam.Running = true;
          certifParam.State = 1;

          mibReq.Type = MIB_ADR;

          LoRaMacMibGetRequestConfirm( &mibReq );
          AdrEnableInit=mibReq.Param.AdrEnable;
          
          mibReq.Type = MIB_ADR;
          mibReq.Param.AdrEnable = true;
          LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( REGION_EU868 ) || defined( REGION_RU864 ) || defined( REGION_CN779 ) || defined( REGION_EU433 )
          LoRaMacTestSetDutyCycleOn( false );
  #endif

        TimerStop(&TxTimer); 
        TimerInit( &CertifTxNextPacketTimer, OnCertifTxNextPacketTimerEvent );
        TimerSetValue( &CertifTxNextPacketTimer,  TEST_TX_DUTYCYCLE); 
        TimerStart( &CertifTxNextPacketTimer);
//        /*confirm test mode activation  ASAP XLO?*/
//        certif_tx( );
      }
  }

  else
  {
      certifParam.State = mcpsIndication->Buffer[0];
      switch( certifParam.State )
      {
        case 0: // Check compliance test disable command (ii)
        {
          
          certifParam.DownLinkCounter = 0;
          certifParam.Running = false;
          
          MibRequestConfirm_t mibReq;
          mibReq.Type = MIB_ADR;
          mibReq.Param.AdrEnable = AdrEnableInit;
          LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( REGION_EU868 ) || defined( REGION_RU864 ) || defined( REGION_CN779 ) || defined( REGION_EU433 )
          LoRaMacTestSetDutyCycleOn( true );
  #endif
          
          break;
        }
        case 1: // (iii, iv)
          certifParam.DataBufferSize = 2;
          break;
        case 2: // Enable confirmed messages (v)
          IsTxConfirmed = LORAWAN_CONFIRMED_MSG;
          certifParam.State = 1;
          break;
        case 3:  // Disable confirmed messages (vi)
          IsTxConfirmed = LORAWAN_UNCONFIRMED_MSG;
          certifParam.State = 1;
          break;
        case 4: // (vii)
          certifParam.DataBufferSize = mcpsIndication->BufferSize;

          certifParam.DataBuffer[0] = 4;
          for( uint8_t i = 1; i < certifParam.DataBufferSize; i++ )
          {
              certifParam.DataBuffer[i] = mcpsIndication->Buffer[i] + 1;
          }
          break;
        case 5: // (viii)
        {
          MlmeReq_t mlmeReq;
          mlmeReq.Type = MLME_LINK_CHECK;
          LoRaMacMlmeRequest( &mlmeReq );
          break;
        }      
        case 6: // (ix)
        {										
					TimerStop( &CertifTxNextPacketTimer );
					DelayMs(500);
					
            MlmeReq_t mlmeReq;

            // Disable TestMode and revert back to normal operation

            certifParam.DownLinkCounter = 0;
            certifParam.Running = false;

            MibRequestConfirm_t mibReq;
            mibReq.Type = MIB_ADR;
            mibReq.Param.AdrEnable = AdrEnableInit;
            LoRaMacMibSetRequestConfirm( &mibReq );

            mlmeReq.Type = MLME_JOIN;
            mlmeReq.Req.Join = *JoinParameters;

            LoRaMacMlmeRequest( &mlmeReq );
            break;
        }

        case 7: // (x)
        {
          if( mcpsIndication->BufferSize == 3 )
          {
              MlmeReq_t mlmeReq;
              mlmeReq.Type = MLME_TXCW;
              mlmeReq.Req.TxCw.Timeout = ( uint16_t )( ( mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2] );
              LoRaMacMlmeRequest( &mlmeReq );
          }
          else if( mcpsIndication->BufferSize == 7 )
          {
              MlmeReq_t mlmeReq;
              mlmeReq.Type = MLME_TXCW_1;
              mlmeReq.Req.TxCw.Timeout = ( uint16_t )( ( mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2] );
              mlmeReq.Req.TxCw.Frequency = ( uint32_t )( ( mcpsIndication->Buffer[3] << 16 ) | ( mcpsIndication->Buffer[4] << 8 ) | mcpsIndication->Buffer[5] ) * 100;
              mlmeReq.Req.TxCw.Power = mcpsIndication->Buffer[6];
              LoRaMacMlmeRequest( &mlmeReq );
          }
          certifParam.State = 1;
          break;
        }
        default:                  
          break;
        }
    }
  
    if ( certifParam.Running == false )
    {
//			PPRINTF("cerification test stops\r");
			      /*cerification test stops*/
      TimerStop( &CertifTxNextPacketTimer );
			if(mcpsIndication->Buffer[0]==0x00)
			{
				TimerStart(&TxTimer); 
			}
    }
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnCertifTxNextPacketTimerEvent( void )
{	  
	  PPRINTF("On Certif tx next packet\r");
    certif_tx( );
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

