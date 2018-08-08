/*prova Alfonso 08/08/2018 15:43*/
/*
******************************************************************************
File:     main.c
Info:     Generated by Atollic TrueSTUDIO(R) 9.0.1   2018-08-04

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
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

GPIO_InitTypeDef Pin_PA5;
GPIO_InitTypeDef Pin_PC13;



int main(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    //GPIO_StructInit(&Pin_PA5);
	Pin_PA5.GPIO_Pin  = GPIO_Pin_5;
	Pin_PA5.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOA, &Pin_PA5);


    //GPIO_StructInit(&Pin_PC13);
	Pin_PC13.GPIO_Pin  = GPIO_Pin_13;
	Pin_PC13.GPIO_Mode = GPIO_Mode_IN;
	Pin_PC13.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &Pin_PC13);
   // uint8_t pulsante = 0, accendi = 1, prec_pulsante;


while (1)
  {
	//GPIO_SetBits(GPIOA, GPIO_Pin_5);
	if(GPIO_ReadInputDataBitCOSINONFUNZIONAPIU_NIENTE(GPIOC, GPIO_Pin_13) == 0){
		GPIO_SetBits(GPIOA, GPIO_Pin_5);
}
	else{
		GPIO_ResetBits(GPIOA, GPIO_Pin_5);
	}
	/*pulsante = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);
	if(pulsante == prec_pulsante){return 0;}

	prec_pulsante = pulsante;

	if(pulsante == 0){
		accendi = !accendi;
	}

	if (accendi == 0){
		GPIO_SetBits(GPIOA, GPIO_Pin_5);
	}
	else{
		GPIO_ResetBits(GPIOA, GPIO_Pin_5);
	}*/

	/*if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_5) == 1){
		if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 0){
			GPIO_ResetBits(GPIOA, GPIO_Pin_5);
			pulsante = 0;
		}

	}
	else if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_5) == 0){
		if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == 0){
			GPIO_SetBits(GPIOA, GPIO_Pin_5);
			pulsante = 1;
		}

	}*/




}
}

