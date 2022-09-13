/******************************************************************************
  * @file    lubcos.c
  * @author  Preddata
  * @version V1.1.0
  * @date    2022-02-21
  * @brief   manages the sensors on the application
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics International N.V. 
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
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "float_encode.h"


int calculaPontoFlutuante(double data, int tamanhoExpoente, int tamanhoMantissa, int numeroBits, int maxExp)
{
		double fractpart, intpart;
    
    int sign                  = 0;
    int expoente[tamanhoExpoente];
		// Inicializa array com 0s
    memset( expoente, 0, tamanhoExpoente*sizeof(int) );
    int mantissa[tamanhoMantissa];
    memset( mantissa, 0, tamanhoMantissa*sizeof(int) );
    
    // separa parte inteira e parte fracionaria
    fractpart = modf(data , &intpart);
    if (fractpart < 0)
    {
        fractpart = -1*fractpart;
    }
    if (intpart < 0)
    {
        intpart = -1*intpart;
    }    
    
		
    // se menor que zero, sinal igual a 1
    sign = ( data < 0 ) ? 1 : 0;
 
    // CALCULO DA MANTISSA
    long quociente = (int) intpart;

    int resto = 0;
    int contador = 0;
    
		// parte inteira
    while (quociente != 0)
    {
        resto = ( quociente % 2 );
        quociente = ( quociente / 2 );
        
        if (quociente != 0)
        {
            contador++;
            // rotacao a direita, mantissa e lida ao contrario
            for (int i = tamanhoMantissa-1; i > 0; i--)
            {
                mantissa[i] = mantissa[i-1];
            }
    
            mantissa[0] = resto;
        }
    }
    

    //--- salvando valor para calculo do expoente
    if (contador != 0)
    {
        // Se tem parte inteira
        quociente = contador+maxExp;
    }
    else if (intpart == 1)
    {
        // Se tem parte inteira
        quociente = maxExp;
    }
    else
    {
        // Se nao tem parte inteira
        quociente = 0;
    }
        
    // parte fracionaria
    int contadorNegativo = 0;
    double fraction = fractpart;
    while (fraction != 0 && contador < tamanhoMantissa)
    {
        fraction = fraction*2;
        int parteInteira = (int)fraction;

        // Se houver parte inteira, adiciona no final
        if (intpart != 0)
        {
            mantissa[contador] = parteInteira;
            contador++;
        }
        // Se nao tem parte inteira, faz a rotacao adiciona no inicio
        else
        {
            if (quociente != 0)
            {    
                mantissa[contador] = parteInteira;
                contador++;
            }
            
            // para numeros menor que 1, tem que diminuir o contador para o calculo do expoente
            contadorNegativo--;
            if (parteInteira == 1 && quociente == 0)
            {
                quociente = contadorNegativo+maxExp;
            }
        }
        
        if (fraction >= 1)
        {
            fraction -= 1;
        }
    }
    
    
    // CALULO DO EXPOENTE (ADICIONADO NA ORDEM INVERSA PELO CALCULO)
    contador = tamanhoExpoente-1;
    while (quociente != 0 && contador >= 0)
    {
        resto = ( quociente % 2 );
        quociente = ( quociente / 2 );
        
        expoente[contador] = resto;       
        contador--;
    }
		
    
    // Calculo do valor inteiro (Base 2 para base 10)
    int fullArray[numeroBits];
    memset( fullArray, 0, numeroBits*sizeof(int) );

		// concatena tudo em um vetor (base 2)
    fullArray[0] = sign;
    for (int i=0 ; i < tamanhoExpoente ; i++)
    {
        fullArray[i+1] = expoente[i];
    }
    for (int i=0 ; i < tamanhoMantissa ; i++)
    {
        fullArray[i+tamanhoExpoente+1] = mantissa[i];
    }
    
    // calcula o valor na base 10
    long valorInteiro   = 0;
    int  multiplicador  = 1;
    
    for (int i = numeroBits-1; i >= 0; i--)
    {
        valorInteiro += fullArray[i]*(multiplicador);
        multiplicador *= 2;
    }
    
    return valorInteiro;
}

int calculaPontoFlutuante16Bits(double data)
{
		// Se numero maior que maximo (65504 - 7BFF), retorna FFFF = 65535
		if (data > 65504)
		{
			return 0xFFFF; // FFFF
		}
		
		
		static const int tamanhoExpoente = 5 ;
		static const int tamanhoMantissa = 10;
		static const int maxExp          = 15;
		static const int numeroBits      = 16;

		return calculaPontoFlutuante(data, tamanhoExpoente, tamanhoMantissa, numeroBits, maxExp);
}

int calculaPontoFlutuante32Bits(double data)
{
	
		// Se numero maior que maximo (??? - 0x7F7FFFFF), retorna FFFFFFFF
		if (data >= 0x7F7FFFFF)
		{
			return 0xFFFFFFFF; // FFFF
		}
		
		
		static const int tamanhoExpoente = 8 ;
		static const int tamanhoMantissa = 23;
		static const int numeroBits      = 32;
		static const int maxExp          = 127;

		return calculaPontoFlutuante(data, tamanhoExpoente, tamanhoMantissa, numeroBits, maxExp);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
