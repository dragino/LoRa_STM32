/**
  @page LoRa Readme file
 
  @verbatim
  ******************** (C) COPYRIGHT 2017 STMicroelectronics *******************
  * @file    LoRa/readme.txt 
  * @author  MCD Application Team
  * @version V1.1
  * @date    5-JAN-2018
  * @brief   This application is a simple demo of a LoRa Object connecting to 
  *          a LoRa Network. 
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
   @endverbatim

@par Description

This directory contains a set of source files that implements a simple demo of an end 
device also known as a LoRa Object connecting to a LoRa Network. The LoRa Object can be 
   - LoRa ST 
   - or LSN50
By setting the LoRa Ids in comissioning.h file according to the LoRa Network requirements, 
the end device will send periodically the sensor data to the LoRa network.
  ******************************************************************************



@par Directory contents 


  - LoRa/DRAGINO-LRWAN(AT)/Inc/hw_conf.h                file to manage Cube SW family used
  - LoRa/DRAGINO-LRWAN(AT)/Inc/stm32l0xx_hal_conf.h     Library Configuration file
  - LoRa/DRAGINO-LRWAN(AT)/Inc/stm32l0xx_it.h           Header for stm32l0xx_it.c
  - LoRa/DRAGINO-LRWAN(AT)/Inc/stm32l0xx_hw_conf.h      Header for stm32l0xx_hw_conf.c(contains hardaware configuration Macros and Constants)
  - LoRa/DRAGINO-LRWAN(AT)/Inc/hw_spi.h                 Header for hw_spi.c
  - LoRa/DRAGINO-LRWAN(AT)/Inc/hw_rtc.h                 Header for hw_rtc.c
  - LoRa/DRAGINO-LRWAN(AT)/Inc/hw_gpio.h                Header for hw_gpio.c
  - LoRa/DRAGINO-LRWAN(AT)/Inc/hw.h                     group all hw interface
  - LoRa/DRAGINO-LRWAN(AT)/Inc/vcom.h                   interface to vcom.c 
  - LoRa/DRAGINO-LRWAN(AT)/Inc/Comissioning.h           End device comissioning parameters
  - LoRa/DRAGINO-LRWAN(AT)/Inc/version .h               version file
  - LoRa/DRAGINO-LRWAN(AT)/Inc/at.h                     Header for at.c
  - LoRa/DRAGINO-LRWAN(AT)/Inc/command.h                Header for command.c

  - LoRa/DRAGINO-LRWAN(AT)/Src/main.c                   Main program file
  - LoRa/DRAGINO-LRWAN(AT)/Src/stm32l0xx_it.c           STM32l0xx Interrupt handlers
  - LoRa/DRAGINO-LRWAN(AT)/Src/stm32l0xx_hal_msp.c      STM32l0xx specific hardware HAL code
  - LoRa/DRAGINO-LRWAN(AT)/Src/stm32l0xx_hw.c           STM32l0xx specific hardware driver code
  - LoRa/DRAGINO-LRWAN(AT)/Src/hw_spi.c                 spi driver
  - LoRa/DRAGINO-LRWAN(AT)/Src/hw_rtc.c                 rtc driver
  - LoRa/DRAGINO-LRWAN(AT)/Src/hw_gpio.c                gpio driver
  - LoRa/DRAGINO-LRWAN(AT)/Src/vcom.c                   virtual com port interface on Terminal
 
@par Hardware and Software environment 


  - This example runs on STM32L072CZT6 devices.
    
  - This application has been tested with Dragino:
    LoRa ST
    LSN50
    boards can be easily tailored to any other supported device 
    and development board.

   
  - Connect the board to your PC with a USB-TTL
  -Set Up:


             --------------------------  V    V  --------------------------
             |      LoRa Object       |  |    |  |      LoRa Netork       |
             |                        |  |    |  |                        |
   ComPort<--|                        |--|    |--|                        |-->Web Server
             |                        |          |                        |
             --------------------------          --------------------------

@par How to use it ? 
In order to make the program work, you must do the following :
  - Open your Serial Port Utility 
  - Rebuild all files and load your image into target memory
  - Run the example
  - Serial Port Utility Setting Config = 9600, 8b, 1 stopbit, no parity, no flow control ( in src/vcom.c)
   
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
