/*
******************************************************************************
File:     main.c
Info:     Generated by Atollic TrueSTUDIO(R) 9.0.1   2018-08-05

The MIT License (MIT)
Copyright (c) 2018 STMicroelectronics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

******************************************************************************
*/

/* Includes */
#include "stm32f4xx.h"
#include <stdio.h>
#include <math.h>
#include "myLib.h"
/* Private macro */

typedef enum {ATTESA = 0, SWAP, FFT} states_t;

/* Private variables */

float dataBuffer0_X[n_C], dataBuffer0_Y[n_C], dataBuffer0_Z[n_C];
float dataBuffer1_X[n_C], dataBuffer1_Y[n_C], dataBuffer1_Z[n_C];

float *workBuf_X, *workBuf_Y, *workBuf_Z;
extern float *storeBuf_X, *storeBuf_Y, *storeBuf_Z; //Da condividere con la ISR
extern u16 cont; //Da condividere con la ISR

/* Private function prototypes */
/* Private functions */

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/

states_t statoCorrente = ATTESA;

int main(void)
{
	storeBuf_X = dataBuffer0_X;
	storeBuf_Y = dataBuffer0_Y;
	storeBuf_Z = dataBuffer0_Z;

	workBuf_X = dataBuffer1_X;
	workBuf_Y = dataBuffer1_Y;
	workBuf_Z = dataBuffer1_Z;

	cont = 0;

    myUSART2_Init();
	myMEMSBoard_Init();

  /* Infinite loop */
	while (1)
	{
		switch(statoCorrente){

		case ATTESA:
		{
           if(cont == n_C){
        	   statoCorrente = SWAP;
           }
           break;
		}

		case SWAP:
		{

		    float *temp = workBuf_X;
			workBuf_X = storeBuf_X;
			storeBuf_X = temp;

			temp = workBuf_Y;
			workBuf_Y = storeBuf_X;
			storeBuf_Y = temp;

			temp = workBuf_Z;
			workBuf_Z = storeBuf_Z;
			storeBuf_Z = temp;

			statoCorrente = FFT;
			cont = 0;
			break;
		}
		case FFT: {

			myDelay_ms(100);
			statoCorrente = ATTESA;
			break;
		}

		}
		
		//myDelay_ms(5000);

  }
}



