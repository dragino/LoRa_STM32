 /******************************************************************************
  * @file    npn_output.c
  * @author  MCD Application Team
  * @version V1.1.1
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
#include "ds18b20.h"
#include "delay.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
void DS18B20_delay(uint16_t time)
{
        uint8_t i;

  while(time)
  {    
          for (i = 0; i < 4; i++)
    {
      
    }
    time--;
  }
}

void DS18B20_Mode_IPU(uint8_t num)
{
  GPIO_InitTypeDef GPIO_InitStruct;
	if(num==1)
	{
  DOUT1_CLK_ENABLE();
  GPIO_InitStruct.Pin   = DOUT1_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  HAL_GPIO_Init(DOUT1_PORT, &GPIO_InitStruct); 
	}
  else if(num==2)
	{
  DOUT2_CLK_ENABLE();
  GPIO_InitStruct.Pin   = DOUT2_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  HAL_GPIO_Init(DOUT2_PORT, &GPIO_InitStruct); 		
	}
	else if(num==3)
	{
  DOUT3_CLK_ENABLE();
  GPIO_InitStruct.Pin   = DOUT3_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  HAL_GPIO_Init(DOUT3_PORT, &GPIO_InitStruct); 		
	}
}

void DS18B20_Mode_Out_PP(uint8_t num)
{
  GPIO_InitTypeDef GPIO_InitStruct;
	if(num==1)
	{	
  DOUT1_CLK_ENABLE();
  GPIO_InitStruct.Pin = DOUT1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DOUT1_PORT, &GPIO_InitStruct);   
  }
	else if(num==2)
	{	
  DOUT2_CLK_ENABLE();
  GPIO_InitStruct.Pin = DOUT2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DOUT2_PORT, &GPIO_InitStruct);   
  }
	else if(num==3)
	{	
  DOUT3_CLK_ENABLE();
  GPIO_InitStruct.Pin = DOUT3_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DOUT3_PORT, &GPIO_InitStruct);   
  }	
}

void DS18B20_IoDeInit(uint8_t num)
{
	if(num==1)
	{
	GPIO_InitTypeDef GPIO_InitStruct;
  DOUT1_CLK_ENABLE();

  GPIO_InitStruct.Pin = DOUT1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DOUT1_PORT, &GPIO_InitStruct);
	}
	else if(num==2)
	{
	GPIO_InitTypeDef GPIO_InitStruct;
  DOUT2_CLK_ENABLE();

  GPIO_InitStruct.Pin = DOUT2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DOUT2_PORT, &GPIO_InitStruct);		
	}
	else if(num==3)
	{
	GPIO_InitTypeDef GPIO_InitStruct;
  DOUT3_CLK_ENABLE();

  GPIO_InitStruct.Pin = DOUT3_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DOUT3_PORT, &GPIO_InitStruct);		
	}
}

void DS18B20_Rst(uint8_t num)
{
	      if(num==1)
				{
        DS18B20_Mode_Out_PP(1);
        
        DOUT1_0;
  
        DS18B20_delay(750);
        
        DOUT1_1;
               
        DS18B20_delay(15);
				}
				else if(num==2)
				{
        DS18B20_Mode_Out_PP(2);
        
        DOUT2_0;
  
        DS18B20_delay(750);
        
        DOUT2_1;
               
        DS18B20_delay(15);
				}
				else if(num==3)
				{
        DS18B20_Mode_Out_PP(3);
        
        DOUT3_0;
  
        DS18B20_delay(750);
        
        DOUT3_1;
               
        DS18B20_delay(15);
				}
}

 uint8_t DS18B20_Presence(uint8_t num)
{
        uint8_t pulse_time = 0;
	
        if(num==1)
				{        
        DS18B20_Mode_IPU(1);

        while( DOUT1_READ() && pulse_time<100 )
        {
                pulse_time++;
                DS18B20_delay(1);
        }        

        if( pulse_time >=100 )
				{
                return 1;
				}
        else
                pulse_time = 0;
        
				
        while( !DOUT1_READ() && pulse_time<240 )
        {
                pulse_time++;
                DS18B20_delay(1);
        }        
        if( pulse_time >=240 )
				{
                return 1;
				}
        else			
                return 0;
			  }
        else if(num==2)
				{        
        DS18B20_Mode_IPU(2);

        while( DOUT2_READ() && pulse_time<100 )
        {
                pulse_time++;
                DS18B20_delay(1);
        }        

        if( pulse_time >=100 )
				{
                return 1;
				}
        else
                pulse_time = 0;
        
				
        while( !DOUT2_READ() && pulse_time<240 )
        {
                pulse_time++;
                DS18B20_delay(1);
        }        
        if( pulse_time >=240 )
				{
                return 1;
				}
        else			
                return 0;
			  }
        else if(num==3)
				{        
        DS18B20_Mode_IPU(3);

        while( DOUT3_READ() && pulse_time<100 )
        {
                pulse_time++;
                DS18B20_delay(1);
        }        

        if( pulse_time >=100 )
				{
                return 1;
				}
        else
                pulse_time = 0;
        
				
        while( !DOUT3_READ() && pulse_time<240 )
        {
                pulse_time++;
                DS18B20_delay(1);
        }        
        if( pulse_time >=240 )
				{
                return 1;
				}
        else			
                return 0;
			  }
				
				return 0;
}

uint8_t DS18B20_ReadBit(uint8_t num)
{
        uint8_t dat;
	
	     if(num==1)
		  {               
        DS18B20_Mode_Out_PP(1);

        DOUT1_0;
        DS18B20_delay(10);
        DS18B20_Mode_IPU(1);
        
        if( DOUT1_READ()==SET)
                dat = 1;
        else
                dat = 0;
			}    
	    else if(num==2)
		  {               
        DS18B20_Mode_Out_PP(2);

        DOUT2_0;
        DS18B20_delay(10);
        DS18B20_Mode_IPU(2);
        
        if( DOUT2_READ()==SET)
                dat = 1;
        else
                dat = 0;
			}  
	    else if(num==3)
		  {               
        DS18B20_Mode_Out_PP(3);

        DOUT3_0;
        DS18B20_delay(10);
        DS18B20_Mode_IPU(3);
        
        if( DOUT3_READ()==SET)
                dat = 1;
        else
                dat = 0;
			}  
			
        DS18B20_delay(45);
        
        return dat;
}

uint8_t DS18B20_ReadByte(uint8_t num)
{
        uint8_t i, j, dat = 0;        
        
        for(i=0; i<8; i++) 
        {
                j = DS18B20_ReadBit(num); 	               								
                dat = (dat) | (j<<i);
        }
        
        return dat;
}

void DS18B20_WriteBit(uint8_t dat,uint8_t num)  
{  
	  if(num==1)
		{
    DS18B20_Mode_Out_PP(1);  
    if (dat)  
    {  
        DOUT1_0; 
        DS18B20_delay(8);  
        DOUT1_1;  
        DS18B20_delay(58);  
    }  
    else  
    {  
        DOUT1_0; 
        DS18B20_delay(70);  
        DOUT1_1;  
        DS18B20_delay(2);  
    } 
	 }	
	  else if(num==2)
		{
    DS18B20_Mode_Out_PP(2);  
    if (dat)  
    {  
        DOUT2_0; 
        DS18B20_delay(8);  
        DOUT2_1;  
        DS18B20_delay(58);  
    }  
    else  
    {  
        DOUT2_0; 
        DS18B20_delay(70);  
        DOUT2_1;  
        DS18B20_delay(2);  
    } 
	 }	
	  else if(num==3)
		{
    DS18B20_Mode_Out_PP(3);  
    if (dat)  
    {  
        DOUT3_0; 
        DS18B20_delay(8);  
        DOUT3_1;  
        DS18B20_delay(58);  
    }  
    else  
    {  
        DOUT3_0; 
        DS18B20_delay(70);  
        DOUT3_1;  
        DS18B20_delay(2);  
    } 
	 }				
}

void DS18B20_WriteByte(uint8_t dat,uint8_t num)
{
        uint8_t i, testb;
	      if(num==1)
				{
        DS18B20_Mode_Out_PP(1);
        
        for( i=0; i<8; i++ )
        {
                testb = dat&0x01;
                dat = dat>>1;                
               
                if (testb)
                {                        
                        DOUT1_0;
                        /* 1us < ???? < 15us */
                        DS18B20_delay(8);
                        
                        DOUT1_1;
                        DS18B20_delay(58);
                }                
                else
                {                        
                        DOUT1_0;
                        /* 60us < Tx 0 < 120us */
                        DS18B20_delay(70);
                        
                        DOUT1_1;                
                        /* 1us < Trec(????) < ???*/
                        DS18B20_delay(2);
                }
         }
			   }
	      else if(num==2)
				{
        DS18B20_Mode_Out_PP(2);
        
        for( i=0; i<8; i++ )
        {
                testb = dat&0x01;
                dat = dat>>1;                
               
                if (testb)
                {                        
                        DOUT2_0;
                        /* 1us < ???? < 15us */
                        DS18B20_delay(8);
                        
                        DOUT2_1;
                        DS18B20_delay(58);
                }                
                else
                {                        
                        DOUT2_0;
                        /* 60us < Tx 0 < 120us */
                        DS18B20_delay(70);
                        
                        DOUT2_1;                
                        /* 1us < Trec(????) < ???*/
                        DS18B20_delay(2);
                }
         }
			 }
	      else if(num==3)
				{
        DS18B20_Mode_Out_PP(3);
        
        for( i=0; i<8; i++ )
        {
                testb = dat&0x01;
                dat = dat>>1;                
               
                if (testb)
                {                        
                        DOUT3_0;
                        /* 1us < ???? < 15us */
                        DS18B20_delay(8);
                        
                        DOUT3_1;
                        DS18B20_delay(58);
                }                
                else
                {                        
                        DOUT3_0;
                        /* 60us < Tx 0 < 120us */
                        DS18B20_delay(70);
                        
                        DOUT3_1;                
                        /* 1us < Trec(????) < ???*/
                        DS18B20_delay(2);
                }
         }
			 }
}

void DS18B20_SkipRom(uint8_t num)
{
        DS18B20_Rst(num);                   
        DS18B20_Presence(num);                 
        DS18B20_WriteByte(0XCC,num);       
}

float DS18B20_GetTemp_SkipRom (uint8_t num)
{
        uint8_t tpmsb, tplsb;
        short s_tem;
        float f_tem;
        
        
        DS18B20_SkipRom(num);
        DS18B20_WriteByte(0X44,num);                                
        
        DelayMs(750);
	
        DS18B20_SkipRom (num);
        DS18B20_WriteByte(0XBE,num);                                
        
        tplsb = DS18B20_ReadByte(num);                 
        tpmsb = DS18B20_ReadByte(num); 
        
        if((tpmsb==5)&&(tplsb==80))     //1360= 00000101 01010000,tpmsb=00000101=5,tplsb=01010000=80;  
        {
		    DS18B20_SkipRom(num);
        DS18B20_WriteByte(0X44,num);                                
        
        DelayMs(750);
	
        DS18B20_SkipRom (num);
        DS18B20_WriteByte(0XBE,num); 
					
        tplsb = DS18B20_ReadByte(num);                 
        tpmsb = DS18B20_ReadByte(num);
				}
				
        s_tem = tpmsb<<8;
        s_tem = s_tem | tplsb;
        
        if( s_tem < 0 )                
                f_tem = (~s_tem+1) * -0.0625;        
        else
                f_tem = s_tem * 0.0625;
        
//				PPRINTF("temp is %f\r\n",f_tem);
        return f_tem;         
}
/*******END OF FILE********/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
