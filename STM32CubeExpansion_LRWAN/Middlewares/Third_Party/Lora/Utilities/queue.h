/**
  ******************************************************************************
  * @file    stm_queue.h
  * @author  MCD Application Team
  * @brief   Header for queue.c
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright c 2017 STMicroelectronics International N.V.
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
#ifndef __STM_QUEUE_H
#define __STM_QUEUE_H

/* Includes ------------------------------------------------------------------*/
#include "utilities_conf.h"
/* Exported define -----------------------------------------------------------*/
/* Options flags */
#define CIRCULAR_QUEUE_NO_FLAG 0
#define CIRCULAR_QUEUE_NO_WRAP_FLAG 1
#define CIRCULAR_QUEUE_SPLIT_IF_WRAPPING_FLAG 2


/* Exported types ------------------------------------------------------------*/
typedef struct {
   uint8_t* qBuff;        /* queue buffer, , provided by init fct */
   uint32_t queueMaxSize;   /* size of the queue, provided by init fct (in bytes)*/
   uint16_t elementSize;    /* -1 variable. If variable elemenet size the size is stored in the 4 first of the queue element */
   uint32_t first;          /* position of first element */
   uint32_t last;           /* position of last element */
   uint32_t byteCount;      /* number of bytes in the queue */
   uint32_t elementCount;   /* number of element in the queue */
   uint8_t  optionFlags;     /* option to enable specific features */
} queue_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
  * @brief   Initilaiilze queue strcuture .
  * @note   This function is used to initialize the global queue strcuture.  
  * @param  q: pointer on queue strcture to be initialised 
  * @param  queueBuffer: pointer on Queue Buffer
  * @param  queueSize:  Size of Queue Buffer
  * @param  elementSize: Size of an element in the queue. if =0, the queue will manage variable sizze elements
  * @retval   always 0
  */
int CircularQueue_Init(queue_t *q, uint8_t* queueBuffer, uint32_t queueSize, uint16_t elementSize, uint8_t optionlags);

/**
  * @brief   Add  element to the queue .
  * @note   This function is used to add one or more  element(s) to the Circular Queue .  
  * @param  q: pointer on queue structure   to be handled
  * @param  X; pointer on element(s) to be added 
  * @param  elementSize:  Size of element to be added to the queue. Only used if the queue manage variable size elements
  * @param  nbElements:  number of elements in the in buffer pointed by x
  * //@retval   number of elements in the queue, -1 if the element to be added do not fit in the queue (too big)
  * @retval  pointer on last element just added to the queue, NULL if the element to be added do not fit in the queue (too big)
  */
uint8_t* CircularQueue_Add(queue_t *q, uint8_t* x, uint16_t elementSize, uint32_t nbElements);

/**
  * @brief  Remove element from  the queue.
  * @note   This function is used to remove and element from  the Circular Queue .  
  * @param  q: pointer on queue structure  to be handled
  * @param  elementSize: Pointer to return Size of element to be removed  
  * @retval Pointer on removed element. NULL if queue was empty
  */
uint8_t* CircularQueue_Remove(queue_t *q, uint16_t* elementSize);

/**
  * @brief  "Sense" first element of the queue, without removing it.
  * @note   This function is used to return a pointer on the first element of the queue without removing it.  
  * @param  q: pointer on queue structure  to be handled
  * @param  elementSize:  Pointer to return Size of element to be removed  
  * @retval Pointer on sensed element. NULL if queue was empty
  */
uint8_t* CircularQueue_Sense(queue_t *q, uint16_t* elementSize);

/**
  * @brief   Check if queue is empty.
  * @note    This function is used to to check if the queue is empty.  
  * @param  q: pointer on queue structure  to be handled
  * @retval   TRUE (!0) if the queue is empyu otherwise FALSE (0)
  */
int CircularQueue_Empty(queue_t *q);

/**
  * @brief   Get the number of element in the queue .  
  * @param  q: pointer on queue structure  to be handled
  * @retval   The number of element
  */
int CircularQueue_NbElement(queue_t *q);

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
#endif /* __STM_QUEUE_H */
