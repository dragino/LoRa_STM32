/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech
 ___ _____ _   ___ _  _____ ___  ___  ___ ___
/ __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
\__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
|___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
embedded.connectivity.solutions===============

Description: LoRa MAC region US915 implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis ( Semtech ), Gregory Cristian ( Semtech ) and Daniel Jaeckle ( STACKFORCE )
*/
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "radio.h"
#include "timer.h"
#include "LoRaMac.h"

#include "utilities.h"

#include "Region.h"
#include "RegionCommon.h"
#include "RegionUS915.h"
#include "debug.h"

#include "lora.h"
// Definitions
#define CHANNELS_MASK_SIZE              6

// A mask to select only valid 500KHz channels
#define CHANNELS_MASK_500KHZ_MASK       0x00FF

extern LoRaMacParams_t LoRaMacParams;
extern uint8_t payloadlens;
extern bool DR_small;
extern bool debug_flags;
/*!
 * Index of current in use 8 bit group (0: bit 0 - 7, 1: bit 8 - 15, ..., 7: bit 56 - 63)
 */
static uint8_t JoinChannelGroupsCurrentIndex;
		
// Global attributes
/*!
 * LoRaMAC channels
 */
static uint8_t current_channel=0;

static ChannelParams_t Channels[US915_MAX_NB_CHANNELS];

/*!
 * LoRaMac bands
 */
static Band_t Bands[US915_MAX_NB_BANDS] =
{
    US915_BAND0
};

/*!
 * LoRaMac channels mask
 */
static uint16_t ChannelsMask[CHANNELS_MASK_SIZE];

/*!
 * LoRaMac channels remaining
 */
static uint16_t ChannelsMaskRemaining[CHANNELS_MASK_SIZE];

static uint16_t ChannelsJoinAcceptMask[CHANNELS_MASK_SIZE];

/*!
 * LoRaMac channels default mask
 */
static uint16_t ChannelsDefaultMask[CHANNELS_MASK_SIZE];

// Static functions
static int8_t GetNextLowerTxDr( int8_t dr, int8_t minDr )
{
    uint8_t nextLowerDr = 0;

    if( dr == minDr )
    {
        nextLowerDr = minDr;
    }
    else
    {
        nextLowerDr = dr - 1;
    }
    return nextLowerDr;
}

/*!
 * \brief Searches for available 125 kHz channels in the given channel mask.
 *
 * \param [IN] channelMaskRemaining The remaining channel mask.
 *
 * \param [OUT] findAvailableChannelsIndex List containing the indexes of all available 125 kHz channels.
 *
 * \param [OUT] availableChannels Number of available 125 kHz channels.
 *
 * \retval Status
 */
static LoRaMacStatus_t FindAvailable125kHzChannels( uint8_t* findAvailableChannelsIndex, uint16_t channelMaskRemaining, uint8_t* availableChannels )
{
    // Nullpointer check
    if( findAvailableChannelsIndex == NULL || availableChannels == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }

    // Initialize counter
    *availableChannels = 0;
    for( uint8_t i = 0; i < 8; i++ )
    {
        // Find available channels
        if( ( channelMaskRemaining & ( 1 << i ) ) != 0 )
        {
            // Save available channel index
            findAvailableChannelsIndex[*availableChannels] = i;
            // Increment counter of available channels if the current channel is available
            ( *availableChannels )++;
        }
    }

    return LORAMAC_STATUS_OK;
}

/*!
 * \brief Computes the next 125kHz channel used for join requests.
 *
 * \param [OUT] newChannelIndex Index of available channel.
 *
 * \retval Status
 */
static LoRaMacStatus_t ComputeNext125kHzJoinChannel( uint8_t* newChannelIndex )
{
    uint8_t currentChannelsMaskRemainingIndex;
    uint16_t channelMaskRemaining;
    uint8_t findAvailableChannelsIndex[8] = { 0 };
    uint8_t availableChannels = 0;
    uint8_t startIndex = JoinChannelGroupsCurrentIndex;

    // Null pointer check
    if( newChannelIndex == NULL )
    {
        return LORAMAC_STATUS_PARAMETER_INVALID;
    }

    do {
        // Current ChannelMaskRemaining, two groups per channel mask. For example Group 0 and 1 (8 bit) are ChannelMaskRemaining 0 (16 bit), etc.
        currentChannelsMaskRemainingIndex = (uint8_t) startIndex / 2;

        // For even numbers we need the 8 LSBs and for uneven the 8 MSBs
        if( ( startIndex % 2 ) == 0 )
        {
					  if((customize_set8channel_get()==0)&&(startIndex==0))
						{
							channelMaskRemaining = ( ( ChannelsMaskRemaining[currentChannelsMaskRemainingIndex] >> 8 ) & 0x00FF );
						}
						else
						{
							channelMaskRemaining = ( ChannelsMaskRemaining[currentChannelsMaskRemainingIndex] & 0x00FF );
						}
        }
        else
        {
					  if((customize_set8channel_get()==0)&&(startIndex==1))
						{
							channelMaskRemaining = ( ChannelsMaskRemaining[currentChannelsMaskRemainingIndex] & 0x00FF );
						}
						else
						{
							channelMaskRemaining = ( ( ChannelsMaskRemaining[currentChannelsMaskRemainingIndex] >> 8 ) & 0x00FF );	
						}					
        }


        if( FindAvailable125kHzChannels( findAvailableChannelsIndex, channelMaskRemaining, &availableChannels ) == LORAMAC_STATUS_PARAMETER_INVALID )
        {
            return LORAMAC_STATUS_PARAMETER_INVALID;
        }

        if ( availableChannels > 0 )
        {
            uint8_t channels_temp;
					  if(customize_set8channel_get()==0)
						{
							if(startIndex==0)
							{
								channels_temp=1;
							}
							else if(startIndex==1)
							{
								channels_temp=0;
							}
							else
							{
								channels_temp=startIndex;
							}
						}
						else
						{
							channels_temp=startIndex;
						}
            // Choose randomly a free channel 125kHz
            *newChannelIndex = ( channels_temp * 8 ) + findAvailableChannelsIndex[randr( 0, ( availableChannels - 1 ) )];
        }

        // Increment start index
        startIndex++;
        if ( startIndex > 7 )
        {
            startIndex = 0;
        }
    } while( ( availableChannels == 0 ) && ( startIndex != JoinChannelGroupsCurrentIndex ) );

    if ( availableChannels > 0 )
    {
        JoinChannelGroupsCurrentIndex = startIndex;
        return LORAMAC_STATUS_OK;
    }

    return LORAMAC_STATUS_PARAMETER_INVALID;
}

static uint32_t GetBandwidth( uint32_t drIndex )
{
    switch( BandwidthsUS915[drIndex] )
    {
        default:
        case 125000:
            return 0;
        case 250000:
            return 1;
        case 500000:
            return 2;
    }
}

static int8_t LimitTxPower( int8_t txPower, int8_t maxBandTxPower, int8_t datarate, uint16_t* channelsMask )
{
    int8_t txPowerResult = txPower;

    // Limit tx power to the band max
    txPowerResult =  MAX( txPower, maxBandTxPower );

    if( datarate == DR_4 )
    {// Limit tx power to max 26dBm
        txPowerResult = MAX( txPower, TX_POWER_2 );
    }
    else
    {
        if( RegionCommonCountChannels( channelsMask, 0, 4 ) < 50 )
        {// Limit tx power to max 21dBm
            txPowerResult = MAX( txPower, TX_POWER_5 );
        }
    }
    return txPowerResult;
}

static bool VerifyRfFreq( uint32_t freq )
{
    // Check radio driver support
    if( Radio.CheckRfFrequency( freq ) == false )
    {
        return false;
    }

    if( ( freq < 915000000 ) || ( freq > 928000000 ) )
    {
        return false;
    }
    return true;
}

static uint8_t CountNbOfEnabledChannels( uint8_t datarate, uint16_t* channelsMask, ChannelParams_t* channels, Band_t* bands, uint8_t* enabledChannels, uint8_t* delayTx )
{
    uint8_t nbEnabledChannels = 0;
    uint8_t delayTransmission = 0;

    for( uint8_t i = 0, k = 0; i < US915_MAX_NB_CHANNELS; i += 16, k++ )
    {
        for( uint8_t j = 0; j < 16; j++ )
        {
            if( ( channelsMask[k] & ( 1 << j ) ) != 0 )
            {
                if( channels[i + j].Frequency == 0 )
                { // Check if the channel is enabled
                    continue;
                }
                if( RegionCommonValueInRange( datarate, channels[i + j].DrRange.Fields.Min,
                                              channels[i + j].DrRange.Fields.Max ) == false )
                { // Check if the current channel selection supports the given datarate
                    continue;
                }
                if( bands[channels[i + j].Band].TimeOff > 0 )
                { // Check if the band is available for transmission
                    delayTransmission++;
                    continue;
                }
                enabledChannels[nbEnabledChannels++] = i + j;
            }
        }
    }

    *delayTx = delayTransmission;
    return nbEnabledChannels;
}

PhyParam_t RegionUS915GetPhyParam( GetPhyParams_t* getPhy )
{
    PhyParam_t phyParam = { 0 };

    switch( getPhy->Attribute )
    {
        case PHY_MIN_RX_DR:
        {
            phyParam.Value = US915_RX_MIN_DATARATE;
            break;
        }
        case PHY_MIN_TX_DR:
        {
            phyParam.Value = US915_TX_MIN_DATARATE;
            break;
        }
        case PHY_DEF_TX_DR:
        {
            phyParam.Value = US915_DEFAULT_DATARATE;
            break;
        }
        case PHY_NEXT_LOWER_TX_DR:
        {
            phyParam.Value = GetNextLowerTxDr( getPhy->Datarate, US915_TX_MIN_DATARATE );
            break;
        }
        case PHY_DEF_TX_POWER:
        {
            phyParam.Value = US915_DEFAULT_TX_POWER;
            break;
        }
        case PHY_MAX_PAYLOAD:
        {
            phyParam.Value = MaxPayloadOfDatarateUS915[getPhy->Datarate];
            break;
        }
        case PHY_MAX_PAYLOAD_REPEATER:
        {
            phyParam.Value = MaxPayloadOfDatarateRepeaterUS915[getPhy->Datarate];
            break;
        }
        case PHY_DUTY_CYCLE:
        {
            phyParam.Value = US915_DUTY_CYCLE_ENABLED;
            break;
        }
        case PHY_MAX_RX_WINDOW:
        {
            phyParam.Value = US915_MAX_RX_WINDOW;
            break;
        }
        case PHY_RECEIVE_DELAY1:
        {
            phyParam.Value = US915_RECEIVE_DELAY1;
            break;
        }
        case PHY_RECEIVE_DELAY2:
        {
            phyParam.Value = US915_RECEIVE_DELAY2;
            break;
        }
        case PHY_JOIN_ACCEPT_DELAY1:
        {
            phyParam.Value = US915_JOIN_ACCEPT_DELAY1;
            break;
        }
        case PHY_JOIN_ACCEPT_DELAY2:
        {
            phyParam.Value = US915_JOIN_ACCEPT_DELAY2;
            break;
        }
        case PHY_MAX_FCNT_GAP:
        {
            phyParam.Value = US915_MAX_FCNT_GAP;
            break;
        }
        case PHY_ACK_TIMEOUT:
        {
            phyParam.Value = ( US915_ACKTIMEOUT + randr( -US915_ACK_TIMEOUT_RND, US915_ACK_TIMEOUT_RND ) );
            break;
        }
        case PHY_DEF_DR1_OFFSET:
        {
            phyParam.Value = US915_DEFAULT_RX1_DR_OFFSET;
            break;
        }
        case PHY_DEF_RX2_FREQUENCY:
        {
            phyParam.Value = US915_RX_WND_2_FREQ;
            break;
        }
        case PHY_DEF_RX2_DR:
        {
            phyParam.Value = US915_RX_WND_2_DR;
            break;
        }
        case PHY_CHANNELS_MASK:
        {
            phyParam.ChannelsMask = ChannelsMask;
            break;
        }
        case PHY_CHANNELS_DEFAULT_MASK:
        {
            phyParam.ChannelsMask = ChannelsDefaultMask;
            break;
        }
        case PHY_MAX_NB_CHANNELS:
        {
            phyParam.Value = US915_MAX_NB_CHANNELS;
            break;
        }
        case PHY_CHANNELS:
        {
            phyParam.Channels = Channels;
            break;
        }
        case PHY_DEF_UPLINK_DWELL_TIME:
        case PHY_DEF_DOWNLINK_DWELL_TIME:
        {
            phyParam.Value = 0;
            break;
        }
        case PHY_DEF_MAX_EIRP:
        case PHY_DEF_ANTENNA_GAIN:
        {
            phyParam.fValue = 0;
            break;
        }
        case PHY_NB_JOIN_TRIALS:
        case PHY_DEF_NB_JOIN_TRIALS:
        {
            phyParam.Value = 2;
            break;
        }
        default:
        {
            break;
        }
    }

    return phyParam;
}

void RegionUS915SetBandTxDone( SetBandTxDoneParams_t* txDone )
{
    RegionCommonSetBandTxDone( txDone->Joined, &Bands[Channels[txDone->Channel].Band], txDone->LastTxDoneTime );
}

void RegionUS915InitDefaults( InitType_t type )
{
	uint32_t channel_single;
	
	channel_single=customize_freq1_get();
	
    switch( type )
    {
        case INIT_TYPE_INIT:
        {
					  JoinChannelGroupsCurrentIndex=0;				
            // Channels
            // 125 kHz channels
            for( uint8_t i = 0; i < US915_MAX_NB_CHANNELS - 8; i++ )
            {
							if(channel_single==0)
				    	{
                Channels[i].Frequency = 902300000 + i * 200000;
							}
							else
              {
								Channels[i].Frequency =channel_single;
							}
                Channels[i].DrRange.Value = ( DR_3 << 4 ) | DR_0;
                Channels[i].Band = 0;
            }
            // 500 kHz channels
            for( uint8_t i = US915_MAX_NB_CHANNELS - 8; i < US915_MAX_NB_CHANNELS; i++ )
            {
                Channels[i].Frequency = 903000000 + ( i - ( US915_MAX_NB_CHANNELS - 8 ) ) * 1600000;
                Channels[i].DrRange.Value = ( DR_4 << 4 ) | DR_4;
                Channels[i].Band = 0;
            }
						
						// ChannelsMask
            ChannelsDefaultMask[0] = 0x0000;
            ChannelsDefaultMask[1] = 0x0000;
            ChannelsDefaultMask[2] = 0x0000;
            ChannelsDefaultMask[3] = 0x0000;
            ChannelsDefaultMask[4] = 0x0002;
            ChannelsDefaultMask[5] = 0x0000;

						uint8_t num,channel_num,k;
						
						channel_num=customize_set8channel_get();
						
						num=(customize_set8channel_get()%2==1)? 0:8;
						
						if(channel_num>=1&&channel_num<=2)
							k=0;
						else if(channel_num>=3&&channel_num<=4)
							k=1;
						else if(channel_num>=5&&channel_num<=6)
							k=2;
						else if(channel_num>=7&&channel_num<=8)
							k=3;
						else 
							k=5;
						
						ChannelsDefaultMask[k]=0x00FF<<num;						
						ChannelsDefaultMask[4]=1<<(channel_num-1);
						
						if(k==5)
						{
						  ChannelsDefaultMask[0] = 0xFFFF;
              ChannelsDefaultMask[1] = 0xFFFF;
              ChannelsDefaultMask[2] = 0xFFFF;
              ChannelsDefaultMask[3] = 0xFFFF;
              ChannelsDefaultMask[4] = 0x00FF;
              ChannelsDefaultMask[5] = 0x0000;
						}
						
            // Copy channels default mask
            RegionCommonChanMaskCopy( ChannelsMask, ChannelsDefaultMask, 6 );

            // Copy into channels mask remaining
            RegionCommonChanMaskCopy( ChannelsMaskRemaining, ChannelsMask, 6 );
            break;
        }
        case INIT_TYPE_RESTORE:
        {
            // Copy channels default mask
            //RegionCommonChanMaskCopy( ChannelsMask, ChannelsDefaultMask, 6 );

            for( uint8_t i = 0; i < 6; i++ )
            { // Copy-And the channels mask
                ChannelsMaskRemaining[i] &= ChannelsMask[i];
            }
            break;
        }
        case INIT_TYPE_REJOIN:
        {
					uint8_t bandnum=0;
					for(uint8_t i=0;i<=4;i++)
					{
						for(uint8_t j=0;j<16;j++)
						{
							if( ChannelsJoinAcceptMask[i] & (1<<j))
							{
								bandnum++;
							}
						}
					}
//					PRINTF("band num=%d\r",bandnum);
					
					if(bandnum>0)
					{          
            RegionCommonChanMaskCopy( ChannelsMask, ChannelsJoinAcceptMask, 6 );

				  	RegionCommonChanMaskCopy( ChannelsMaskRemaining, ChannelsMask, 6 );
					}
					else
					{
						// Copy channels default mask
            RegionCommonChanMaskCopy( ChannelsMask, ChannelsDefaultMask, 6 );

				  	RegionCommonChanMaskCopy( ChannelsMaskRemaining, ChannelsMask, 6 );
					}
					
//					  for(int i=0;i<6;i++)
//					{
//						PPRINTF("%0x ",ChannelsMask[i]);
//					}
//					PRINTF("\r");
            break;
        }									
        default:
        {
            break;
        }
    }
}

bool RegionUS915Verify( VerifyParams_t* verify, PhyAttribute_t phyAttribute )
{
    switch( phyAttribute )
    {
			  case PHY_FREQUENCY:
        {
            return VerifyRfFreq( verify->Frequency );
        }	
        case PHY_TX_DR:
        {
            return RegionCommonValueInRange( verify->DatarateParams.Datarate, US915_TX_MIN_DATARATE, US915_TX_MAX_DATARATE );
        }
        case PHY_DEF_TX_DR:
        {
            return RegionCommonValueInRange( verify->DatarateParams.Datarate, DR_0, DR_5 );
        }
        case PHY_RX_DR:
        {
            return RegionCommonValueInRange( verify->DatarateParams.Datarate, US915_RX_MIN_DATARATE, US915_RX_MAX_DATARATE );
        }
        case PHY_DEF_TX_POWER:
        case PHY_TX_POWER:
        {
            // Remark: switched min and max!
            return RegionCommonValueInRange( verify->TxPower, US915_MAX_TX_POWER, US915_MIN_TX_POWER );
        }
        case PHY_DUTY_CYCLE:
        {
            return US915_DUTY_CYCLE_ENABLED;
        }
        case PHY_NB_JOIN_TRIALS:
        {
            if( verify->NbJoinTrials < 2 )
            {
                return false;
            }
            break;
        }
        default:
            return false;
    }
    return true;
}

void RegionUS915ApplyCFList( ApplyCFListParams_t* applyCFList )
{
	  // Size of the optional CF list must be 16 byte
    if( applyCFList->Size != 16 )
    {	
			uint16_t channel_mask[6]={0,0,0,0,0,0};
			uint8_t index=0,num=0,k=0;
			
			if(current_channel<64)
			{
				index=current_channel/8+1;
			}
			else if(current_channel>63 && current_channel<72)
			{
				index=current_channel-63;
			}
						
				num=(index%2==1)? 0:8;
				
				if(index>=1&&index<=2)
					k=0;
				else if(index>=3&&index<=4)
					k=1;
				else if(index>=5&&index<=6)
					k=2;
				else if(index>=7&&index<=8)
					k=3;
				else 
					k=5;
								
				channel_mask[k]=0xFF<<num;
				channel_mask[4]=1<<(index-1);
				
				RegionCommonChanMaskCopy( ChannelsJoinAcceptMask, channel_mask, 6 );
				RegionCommonChanMaskCopy( ChannelsMask, ChannelsJoinAcceptMask, 6 );
				
				for(int i=0;i<6;i++)
				{
					ChannelsMaskRemaining[i] &= channel_mask[i];
				}	
			 
        return;
    }

    // Last byte CFListType must be 0x01 to indicate the CFList contains a series of ChMask fields
    if( applyCFList->Payload[15] != 0x01 )
    {
        return;
    }

    // ChMask0 - ChMask4 must be set (every ChMask has 16 bit)
    for( uint8_t chMaskItr = 0, cntPayload = 0; chMaskItr <= 4; chMaskItr++, cntPayload+=2 )
    {
        ChannelsMask[chMaskItr] = (uint16_t) (0x00FF & applyCFList->Payload[cntPayload]);
        ChannelsMask[chMaskItr] |= (uint16_t) (applyCFList->Payload[cntPayload+1] << 8);
        if( chMaskItr == 4 )
        {
            ChannelsMask[chMaskItr] = ChannelsMask[chMaskItr] & CHANNELS_MASK_500KHZ_MASK;
        }
        // Set the channel mask to the remaining
        ChannelsMaskRemaining[chMaskItr] &= ChannelsMask[chMaskItr];
				ChannelsJoinAcceptMask[chMaskItr]= ChannelsMask[chMaskItr];					
    }
}

bool RegionUS915ChanMaskSet( ChanMaskSetParams_t* chanMaskSet )
{
    uint8_t nbChannels = RegionCommonCountChannels( chanMaskSet->ChannelsMaskIn, 0, 4 );

    // Check the number of active channels
    if( ( nbChannels < 2 ) &&
        ( nbChannels > 0 ) )
    {
        return false;
    }

    switch( chanMaskSet->ChannelsMaskType )
    {
        case CHANNELS_MASK:
        {
            RegionCommonChanMaskCopy( ChannelsMask, chanMaskSet->ChannelsMaskIn, 6 );

            for( uint8_t i = 0; i < 6; i++ )
            { // Copy-And the channels mask
                ChannelsMaskRemaining[i] &= ChannelsMask[i];
            }
            break;
        }
        case CHANNELS_DEFAULT_MASK:
        {
            RegionCommonChanMaskCopy( ChannelsDefaultMask, chanMaskSet->ChannelsMaskIn, 6 );
            break;
        }
        default:
            return false;
    }
    return true;
}

bool RegionUS915AdrNext( AdrNextParams_t* adrNext, int8_t* drOut, int8_t* txPowOut, uint32_t* adrAckCounter )
{
    bool adrAckReq = false;
    int8_t datarate = adrNext->Datarate;
    int8_t txPower = adrNext->TxPower;
    GetPhyParams_t getPhy;
    PhyParam_t phyParam;

    // Report back the adr ack counter
    *adrAckCounter = adrNext->AdrAckCounter;

    if( adrNext->AdrEnabled == true )
    {
        if( datarate == US915_TX_MIN_DATARATE )
        {
            *adrAckCounter = 0;
            adrAckReq = false;
        }
        else
        {
            if( adrNext->AdrAckCounter >= US915_ADR_ACK_LIMIT )
            {
                adrAckReq = true;
                txPower = US915_MAX_TX_POWER;
            }
            else
            {
                adrAckReq = false;
            }
            if( adrNext->AdrAckCounter >= ( US915_ADR_ACK_LIMIT + US915_ADR_ACK_DELAY ) )
            {
                if( ( adrNext->AdrAckCounter % US915_ADR_ACK_DELAY ) == 1 )
                {
                    // Decrease the datarate
                    getPhy.Attribute = PHY_NEXT_LOWER_TX_DR;
                    getPhy.Datarate = datarate;
                    getPhy.UplinkDwellTime = adrNext->UplinkDwellTime;
                    phyParam = RegionUS915GetPhyParam( &getPhy );
                    datarate = phyParam.Value;

                    if( datarate == US915_TX_MIN_DATARATE )
                    {
                        // We must set adrAckReq to false as soon as we reach the lowest datarate
                        adrAckReq = false;
                        if( adrNext->UpdateChanMask == true )
                        {
                            // Re-enable default channels
//                            ChannelsMask[0] = 0xFFFF;
//                            ChannelsMask[1] = 0xFFFF;
//                            ChannelsMask[2] = 0xFFFF;
//                            ChannelsMask[3] = 0xFFFF;
//                            ChannelsMask[4] = 0x00FF;
//                            ChannelsMask[5] = 0x0000;
                        }
                    }
                }
            }
        }
    }

    *drOut = datarate;
    *txPowOut = txPower;
    return adrAckReq;
}

void RegionUS915ComputeRxWindowParameters( int8_t datarate, uint8_t minRxSymbols, uint32_t rxError, RxConfigParams_t *rxConfigParams )
{
    double tSymbol = 0.0;
    uint32_t radioWakeUpTime;

    // Get the datarate, perform a boundary check
    rxConfigParams->Datarate = MIN( datarate, US915_RX_MAX_DATARATE );
    rxConfigParams->Bandwidth = GetBandwidth( rxConfigParams->Datarate );

    tSymbol = RegionCommonComputeSymbolTimeLoRa( DataratesUS915[rxConfigParams->Datarate], BandwidthsUS915[rxConfigParams->Datarate] );

    radioWakeUpTime = Radio.GetRadioWakeUpTime( );
    RegionCommonComputeRxWindowParameters( tSymbol, minRxSymbols, rxError, radioWakeUpTime, &rxConfigParams->WindowTimeout, &rxConfigParams->WindowOffset );
}

bool RegionUS915RxConfig( RxConfigParams_t* rxConfig, int8_t* datarate )
{
    int8_t dr = rxConfig->Datarate;
    uint8_t maxPayload = 0;
    int8_t phyDr = 0;
    uint32_t frequency = rxConfig->Frequency;
	  uint16_t rxfreq_a,rxfreq_b;
	
    if( Radio.GetStatus( ) != RF_IDLE )
    {
        return false;
    }

    if( rxConfig->Window == 0 )
    {
        // Apply window 1 frequency
        frequency = US915_FIRST_RX1_CHANNEL + ( rxConfig->Channel % 8 ) * US915_STEPWIDTH_RX1_CHANNEL;
    }

    // Read the physical datarate from the datarates table
    phyDr = DataratesUS915[dr];

    Radio.SetChannel( frequency );

    // Radio configuration
    Radio.SetRxConfig( MODEM_LORA, rxConfig->Bandwidth, phyDr, 1, 0, 8, rxConfig->WindowTimeout, false, 0, false, 0, 0, true, rxConfig->RxContinuous );

    if( rxConfig->RepeaterSupport == true )
    {
        maxPayload = MaxPayloadOfDatarateRepeaterUS915[dr];
    }
    else
    {
        maxPayload = MaxPayloadOfDatarateUS915[dr];
    }
    Radio.SetMaxPayloadLength( MODEM_LORA, maxPayload + LORA_MAC_FRMPAYLOAD_OVERHEAD );

		if(debug_flags==1)
		{	
			TimerTime_t ts = TimerGetCurrentTime(); 
			PPRINTF("[%lu]", ts); 
			PPRINTF( "RX on freq %d Hz at DR %d\r\n", frequency, dr );						
		}
		else
		{
			rxfreq_a=frequency/1000000;
			rxfreq_b=(frequency%1000000)/1000;			
			PPRINTF( "RX on freq %d.%d MHz at DR %d\r\n",rxfreq_a,rxfreq_b,dr );				
		}
		
    *datarate = (uint8_t) dr;
    return true;
}

bool RegionUS915TxConfig( TxConfigParams_t* txConfig, int8_t* txPower, TimerTime_t* txTimeOnAir )
{
    int8_t phyDr = DataratesUS915[txConfig->Datarate];
    int8_t txPowerLimited = LimitTxPower( txConfig->TxPower, Bands[Channels[txConfig->Channel].Band].TxMaxPower, txConfig->Datarate, ChannelsMask );
    uint32_t bandwidth = GetBandwidth( txConfig->Datarate );
    int8_t phyTxPower = 0;
	  uint16_t txfreq_a,txfreq_b;
	
    // Calculate physical TX power
    phyTxPower = RegionCommonComputeTxPower( txPowerLimited, US915_DEFAULT_MAX_ERP, 0 );

    // Setup the radio frequency
    Radio.SetChannel( Channels[txConfig->Channel].Frequency );

    Radio.SetTxConfig( MODEM_LORA, phyTxPower, 0, bandwidth, phyDr, 1, 8, false, true, 0, 0, false, 3000 );
	
	  current_channel=txConfig->Channel;
	
		if(debug_flags==1)
		{		
			TimerTime_t ts = TimerGetCurrentTime(); 
			PPRINTF("[%lu]", ts); 	
			PPRINTF( "TX on freq %d Hz at DR %d\r\n", Channels[txConfig->Channel].Frequency, txConfig->Datarate );			
		}			
		else
		{
			txfreq_a=Channels[txConfig->Channel].Frequency/1000000;
			txfreq_b=(Channels[txConfig->Channel].Frequency%1000000)/1000;
			PPRINTF( "TX on freq %d.%d MHz at DR %d\r\n",txfreq_a,txfreq_b,txConfig->Datarate );						
    }
		
    // Setup maximum payload lenght of the radio driver
    Radio.SetMaxPayloadLength( MODEM_LORA, txConfig->PktLen );
    // Get the time-on-air of the next tx frame
    *txTimeOnAir = Radio.TimeOnAir( MODEM_LORA, txConfig->PktLen );
    *txPower = txPowerLimited;

    return true;
}

uint8_t RegionUS915LinkAdrReq( LinkAdrReqParams_t* linkAdrReq, int8_t* drOut, int8_t* txPowOut, uint8_t* nbRepOut, uint8_t* nbBytesParsed )
{
    uint8_t status = 0x07;
    RegionCommonLinkAdrParams_t linkAdrParams;
    uint8_t nextIndex = 0;
    uint8_t bytesProcessed = 0;
    uint16_t channelsMask[6] = { 0, 0, 0, 0, 0, 0 };
    GetPhyParams_t getPhy;
    PhyParam_t phyParam;
    RegionCommonLinkAdrReqVerifyParams_t linkAdrVerifyParams;

    // Initialize local copy of channels mask
    RegionCommonChanMaskCopy( channelsMask, ChannelsMask, 6 );

    while( bytesProcessed < linkAdrReq->PayloadSize )
    {
        nextIndex = RegionCommonParseLinkAdrReq( &( linkAdrReq->Payload[bytesProcessed] ), &linkAdrParams );

        if( nextIndex == 0 )
            break; // break loop, since no more request has been found

        // Update bytes processed
        bytesProcessed += nextIndex;

        // Revert status, as we only check the last ADR request for the channel mask KO
        status = 0x07;

        if( linkAdrParams.ChMaskCtrl == 6 )
        {
            // Enable all 125 kHz channels
            channelsMask[0] = 0xFFFF;
            channelsMask[1] = 0xFFFF;
            channelsMask[2] = 0xFFFF;
            channelsMask[3] = 0xFFFF;
            // Apply chMask to channels 64 to 71
            channelsMask[4] = linkAdrParams.ChMask & CHANNELS_MASK_500KHZ_MASK;
        }
        else if( linkAdrParams.ChMaskCtrl == 7 )
        {
            // Disable all 125 kHz channels
            channelsMask[0] = 0x0000;
            channelsMask[1] = 0x0000;
            channelsMask[2] = 0x0000;
            channelsMask[3] = 0x0000;
            // Apply chMask to channels 64 to 71
            channelsMask[4] = linkAdrParams.ChMask & CHANNELS_MASK_500KHZ_MASK;
        }
        else if( linkAdrParams.ChMaskCtrl == 5 )
        {
            // Start value for comparision
            uint8_t bitMask = 1;

            // cntChannelMask for channelsMask[0] until channelsMask[3]
            uint8_t cntChannelMask = 0;

            // i will be 1, 2, 3, ..., 7
            for( uint8_t i = 0; i <= 7; i++ )
            {
                // 8 MSBs of ChMask are RFU
                // Checking if the ChMask is set, then true
                if( ( ( linkAdrParams.ChMask & 0x00FF ) & ( bitMask << i ) ) != 0 )
                {
                    if( ( i % 2 ) == 0 )
                    {
                        // Enable a bank of 8 125kHz channels, 8 LSBs
                        channelsMask[cntChannelMask] |= 0x00FF;
                        // Enable the corresponding 500kHz channel
                        channelsMask[4] |= ( bitMask << i );
                    }
                    else
                    {
                        // Enable a bank of 8 125kHz channels, 8 MSBs
                        channelsMask[cntChannelMask] |= 0xFF00;
                        // Enable the corresponding 500kHz channel
                        channelsMask[4] |= ( bitMask << i );
                        // cntChannelMask increment for uneven i
                        cntChannelMask++;
                    }
                }
                // ChMask is not set
                else
                {
                    if( ( i % 2 ) == 0 )
                    {
                        // Disable a bank of 8 125kHz channels, 8 LSBs
                        channelsMask[cntChannelMask] &= 0xFF00;
                        // Disable the corresponding 500kHz channel
                        channelsMask[4] &= ~( bitMask << i );
                    }
                    else
                    {
                        // Enable a bank of 8 125kHz channels, 8 MSBs
                        channelsMask[cntChannelMask] &= 0x00FF;
                        // Disable the corresponding 500kHz channel
                        channelsMask[4] &= ~( bitMask << i );
                        // cntChannelMask increment for uneven i
                        cntChannelMask++;
                    }
                }
            }
        }
        else
        {
            channelsMask[linkAdrParams.ChMaskCtrl] = linkAdrParams.ChMask;
        }
    }

    // FCC 15.247 paragraph F mandates to hop on at least 2 125 kHz channels
    if( ( linkAdrParams.Datarate < DR_4 ) && ( RegionCommonCountChannels( channelsMask, 0, 4 ) < 2 ) )
    {
        status &= 0xFE; // Channel mask KO
    }

    // Get the minimum possible datarate
    getPhy.Attribute = PHY_MIN_TX_DR;
    getPhy.UplinkDwellTime = linkAdrReq->UplinkDwellTime;
    phyParam = RegionUS915GetPhyParam( &getPhy );

    linkAdrVerifyParams.Status = status;
    linkAdrVerifyParams.AdrEnabled = linkAdrReq->AdrEnabled;
    linkAdrVerifyParams.Datarate = linkAdrParams.Datarate;
    linkAdrVerifyParams.TxPower = linkAdrParams.TxPower;
    linkAdrVerifyParams.NbRep = linkAdrParams.NbRep;
    linkAdrVerifyParams.CurrentDatarate = linkAdrReq->CurrentDatarate;
    linkAdrVerifyParams.CurrentTxPower = linkAdrReq->CurrentTxPower;
    linkAdrVerifyParams.CurrentNbRep = linkAdrReq->CurrentNbRep;
    linkAdrVerifyParams.NbChannels = US915_MAX_NB_CHANNELS;
    linkAdrVerifyParams.ChannelsMask = channelsMask;
    linkAdrVerifyParams.MinDatarate = ( int8_t )phyParam.Value;
    linkAdrVerifyParams.MaxDatarate = US915_TX_MAX_DATARATE;
    linkAdrVerifyParams.Channels = Channels;
    linkAdrVerifyParams.MinTxPower = US915_MIN_TX_POWER;
    linkAdrVerifyParams.MaxTxPower = US915_MAX_TX_POWER;

    // Verify the parameters and update, if necessary
    status = RegionCommonLinkAdrReqVerifyParams( &linkAdrVerifyParams, &linkAdrParams.Datarate, &linkAdrParams.TxPower, &linkAdrParams.NbRep );

    // Update channelsMask if everything is correct
    if( status == 0x07 )
    {
				if(customize_set8channel_get()==0)
				{
					// Copy Mask
					RegionCommonChanMaskCopy( ChannelsMask, channelsMask, 6 );

					ChannelsMaskRemaining[0] &= ChannelsMask[0];
					ChannelsMaskRemaining[1] &= ChannelsMask[1];
					ChannelsMaskRemaining[2] &= ChannelsMask[2];
					ChannelsMaskRemaining[3] &= ChannelsMask[3];
					ChannelsMaskRemaining[4] = ChannelsMask[4];
					ChannelsMaskRemaining[5] = ChannelsMask[5];		
				}					
    }

		 if((linkAdrParams.Datarate==0)&&(payloadlens>11)&&(payloadlens<53))
		{
			linkAdrParams.Datarate=1;
			DR_small=1;				
		}	
		else if((linkAdrParams.Datarate==1)&&(payloadlens>=53))
		{
			linkAdrParams.Datarate=2;		
			DR_small=1;			
		}
		
    // Update status variables
    *drOut = linkAdrParams.Datarate;
    *txPowOut = linkAdrParams.TxPower;
    *nbRepOut = linkAdrParams.NbRep;
    *nbBytesParsed = bytesProcessed;
		
    return status;
}

uint8_t RegionUS915RxParamSetupReq( RxParamSetupReqParams_t* rxParamSetupReq )
{
    uint8_t status = 0x07;
    uint32_t freq = rxParamSetupReq->Frequency;

    // Verify radio frequency
    if( ( Radio.CheckRfFrequency( freq ) == false ) ||
        ( freq < US915_FIRST_RX1_CHANNEL ) ||
        ( freq > US915_LAST_RX1_CHANNEL ) ||
        ( ( ( freq - ( uint32_t ) US915_FIRST_RX1_CHANNEL ) % ( uint32_t ) US915_STEPWIDTH_RX1_CHANNEL ) != 0 ) ||
    		( freq < 902300000 ) ||  ( freq > 927500000 ))
    {
        status &= 0xFE; // Channel frequency KO
    }

    // Verify datarate
    if( RegionCommonValueInRange( rxParamSetupReq->Datarate, US915_RX_MIN_DATARATE, US915_RX_MAX_DATARATE ) == false )
    {
        status &= 0xFD; // Datarate KO
    }
    if( ( RegionCommonValueInRange( rxParamSetupReq->Datarate, DR_5, DR_7 ) == true ) ||
        ( rxParamSetupReq->Datarate > DR_13 ) )
    {
        status &= 0xFD; // Datarate KO
    }

    // Verify datarate offset
    if( RegionCommonValueInRange( rxParamSetupReq->DrOffset, US915_MIN_RX1_DR_OFFSET, US915_MAX_RX1_DR_OFFSET ) == false )
    {
        status &= 0xFB; // Rx1DrOffset range KO
    }

    return status;
}

uint8_t RegionUS915NewChannelReq( NewChannelReqParams_t* newChannelReq )
{
    // Datarate and frequency KO
    return 0;
}

int8_t RegionUS915TxParamSetupReq( TxParamSetupReqParams_t* txParamSetupReq )
{
    return -1;
}

uint8_t RegionUS915DlChannelReq( DlChannelReqParams_t* dlChannelReq )
{
    return 0;
}

int8_t RegionUS915AlternateDr( AlternateDrParams_t* alternateDr )
{
    int8_t datarate = 0;

    // Re-enable 500 kHz default channels
//    ChannelsMask[4] = 0x00FF;

//    if( ( alternateDr->NbTrials & 0x01 ) == 0x01 )
	  if( alternateDr->NbTrials % 9 == 0 )	
    {
        datarate = DR_4;
    }
    else
    {
        datarate = DR_0;
    }
    return datarate;
}

void RegionUS915CalcBackOff( CalcBackOffParams_t* calcBackOff )
{
    RegionCommonCalcBackOffParams_t calcBackOffParams;

    calcBackOffParams.Channels = Channels;
    calcBackOffParams.Bands = Bands;
    calcBackOffParams.LastTxIsJoinRequest = calcBackOff->LastTxIsJoinRequest;
    calcBackOffParams.Joined = calcBackOff->Joined;
    calcBackOffParams.DutyCycleEnabled = calcBackOff->DutyCycleEnabled;
    calcBackOffParams.Channel = calcBackOff->Channel;
    calcBackOffParams.ElapsedTime = calcBackOff->ElapsedTime;
    calcBackOffParams.TxTimeOnAir = calcBackOff->TxTimeOnAir;

    RegionCommonCalcBackOff( &calcBackOffParams );
}

bool RegionUS915NextChannel( NextChanParams_t* nextChanParams, uint8_t* channel, TimerTime_t* time, TimerTime_t* aggregatedTimeOff )
{
    uint8_t nbEnabledChannels = 0;
    uint8_t delayTx = 0;
    uint8_t enabledChannels[US915_MAX_NB_CHANNELS] = { 0 };
    TimerTime_t nextTxDelay = 0;
    uint8_t newChannelIndex;
		
    // Count 125kHz channels
    if( RegionCommonCountChannels( ChannelsMaskRemaining, 0, 4 ) == 0 )
    { // Reactivate default channels
        RegionCommonChanMaskCopy( ChannelsMaskRemaining, ChannelsMask, 4  );
			  JoinChannelGroupsCurrentIndex = 0;
    }
    // Check other channels
    if( nextChanParams->Datarate >= DR_4 )
    {
        if( ( ChannelsMaskRemaining[4] & 0x00FF ) == 0 )
        {
            ChannelsMaskRemaining[4] = ChannelsMask[4];
        }
    }

    if( nextChanParams->AggrTimeOff <= TimerGetElapsedTime( nextChanParams->LastAggrTx ) )
    {
        // Reset Aggregated time off
        *aggregatedTimeOff = 0;

        // Update bands Time OFF
        nextTxDelay = RegionCommonUpdateBandTimeOff( nextChanParams->Joined, nextChanParams->DutyCycleEnabled, Bands, US915_MAX_NB_BANDS );

        // Search how many channels are enabled
        nbEnabledChannels = CountNbOfEnabledChannels( nextChanParams->Datarate,
                                                      ChannelsMaskRemaining, Channels,
                                                      Bands, enabledChannels, &delayTx );
    }
    else
    {
        delayTx++;
        nextTxDelay = nextChanParams->AggrTimeOff - TimerGetElapsedTime( nextChanParams->LastAggrTx );
    }

    if( nbEnabledChannels > 0 )
    {
			if(nextChanParams->Joined==true)
			{
        // We found a valid channel
        *channel = enabledChannels[randr( 0, nbEnabledChannels - 1 )];
			}
			else
			{
				// For rapid network acquisition in mixed gateway channel plan environments, the device
				// follow a random channel selection sequence. It probes alternating one out of a
				// group of eight 125 kHz channels followed by probing one 500 kHz channel each pass.
				// Each time a 125 kHz channel will be selected from another group.

				// 125kHz Channels (0 - 63) DR0
				if( nextChanParams->Datarate == DR_0 )
				{
						if( ComputeNext125kHzJoinChannel( &newChannelIndex ) == LORAMAC_STATUS_PARAMETER_INVALID )
						{
								return LORAMAC_STATUS_PARAMETER_INVALID;
						}
						*channel = newChannelIndex;
				}
				// 500kHz Channels (64 - 71) DR4
				else
				{
						// Choose the next available channel
						uint8_t i;
					  if((ChannelsMaskRemaining[4]&0x02)==0x02)
						{
							i=1;
						}
						else
						{
							i=0;
						}
						
						while( ( ( ChannelsMaskRemaining[4] & CHANNELS_MASK_500KHZ_MASK ) & ( 1 << i ) ) == 0 )
						{
								i++;
						}
						*channel = 64 + i;
				}
			}
        // Disable the channel in the mask
        RegionCommonChanDisable( ChannelsMaskRemaining, *channel, US915_MAX_NB_CHANNELS );

        *time = 0;
        return true;
    }
    else
    {
        if( delayTx > 0 )
        {
            // Delay transmission due to AggregatedTimeOff or to a band time off
            *time = nextTxDelay;
            return true;
        }
        // Datarate not supported by any channel
        *time = 0;
        return false;
    }
}

LoRaMacStatus_t RegionUS915ChannelAdd( ChannelAddParams_t* channelAdd )
{
    return LORAMAC_STATUS_PARAMETER_INVALID;
}

bool RegionUS915ChannelsRemove( ChannelRemoveParams_t* channelRemove  )
{
    return LORAMAC_STATUS_PARAMETER_INVALID;
}

void RegionUS915SetContinuousWave( ContinuousWaveParams_t* continuousWave )
{
    int8_t txPowerLimited = LimitTxPower( continuousWave->TxPower, Bands[Channels[continuousWave->Channel].Band].TxMaxPower, continuousWave->Datarate, ChannelsMask );
    int8_t phyTxPower = 0;
    uint32_t frequency = Channels[continuousWave->Channel].Frequency;

    // Calculate physical TX power
    phyTxPower = RegionCommonComputeTxPower( txPowerLimited, US915_DEFAULT_MAX_ERP, 0 );

    Radio.SetTxContinuousWave( frequency, phyTxPower, continuousWave->Timeout );
}

uint8_t RegionUS915ApplyDrOffset( uint8_t downlinkDwellTime, int8_t dr, int8_t drOffset )
{
    int8_t datarate = DatarateOffsetsUS915[dr][drOffset];

    if( datarate < 0 )
    {
        datarate = DR_0;
    }
    return datarate;
}

//void RegionUS915RxBeaconSetup( RxBeaconSetup_t* rxBeaconSetup, uint8_t* outDr )
//{
//    RegionCommonRxBeaconSetupParams_t regionCommonRxBeaconSetup;

//    regionCommonRxBeaconSetup.Datarates = DataratesUS915;
//    regionCommonRxBeaconSetup.Frequency = rxBeaconSetup->Frequency;
//    regionCommonRxBeaconSetup.BeaconSize = US915_BEACON_SIZE;
//    regionCommonRxBeaconSetup.BeaconDatarate = US915_BEACON_CHANNEL_DR;
//    regionCommonRxBeaconSetup.BeaconChannelBW = US915_BEACON_CHANNEL_BW;
//    regionCommonRxBeaconSetup.RxTime = rxBeaconSetup->RxTime;
//    regionCommonRxBeaconSetup.SymbolTimeout = rxBeaconSetup->SymbolTimeout;

//    RegionCommonRxBeaconSetup( &regionCommonRxBeaconSetup );

//    // Store downlink datarate
//    *outDr = US915_BEACON_CHANNEL_DR;
//}
