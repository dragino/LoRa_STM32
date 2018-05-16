/******************************************************************************
 * @file    tiny_vsnprintf.h
 * @author  MCD Application Team
 * @version V1.1.4
 * @date    08-January-2018
 * @brief   Header for tiny_vsnprintf.c module
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TINY_VSNPRINTF_H__
#define __TINY_VSNPRINTF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Tiny implementation of vsnprintf() like function
 *
 *         It has been adapted so that:
 *         - Tiny implementation, when defining TINY_PRINTF, is available. In such as case,
 *           not all the format are available. Instead, only %02X, %x, %d, %u, %s and %c are available.
 *           %f,, %+, %#, %- and others are excluded
 *         - Provide a snprintf like implementation. The size of the buffer is provided,
 *           and the length of the filled buffer is returned (not including the final '\0' char).
 *         The string may be truncated
 * @param  Pointer to a buffer where the resulting C-string is stored. The buffer should have a size of
 *         at least n characters.
 * @param  Maximum number of bytes to be used in the buffer. The generated string has a length of at
 *         most n-1, leaving space for the additional terminating null character.
 * @param  C string that contains a format string that follows the same specifications as format
 *         in printf (see printf for details).
 * @param  A value identifying a variable arguments list initialized with va_start.
 * @retval The number of written char (note that this is different from vsnprintf()
 */
int tiny_vsnprintf_like(char *buf, const int size, const char *fmt, va_list args);

#ifdef __cplusplus
}
#endif

#endif /* __TINY_VSNPRINTF_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
