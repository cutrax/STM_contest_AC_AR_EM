/*
******************************************************************************
File:     main.c
Info:     Generated by Atollic TrueSTUDIO(R) 9.0.1   2018-08-06

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
#include <stm32f4xx.h>
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
//#include <stdio.h>





volatile uint32_t tick = 0; //Variabile che conta il numero di overflow

void TIM_Config(void);
void PORT_Config(void);

int main(void)
{
	PORT_Config();
	TIM_Config();

	while (1)
	{

	}
}

void TIM_Config(void)
{
	uint16_t PrescalerValue;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); //Inizializzo la struttura
	TIM_TimeBaseStructure.TIM_Period = 62500; //Impostando questo periodo quanto il timer raggiunge l'overflow sono passati 250 ms
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //Configuro il timer

	PrescalerValue = (uint16_t) (SystemCoreClock / 250000) - 1; //Con questo valore di prescaler porto il clock
                                                                      //del contatore a 210 KHz
	TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate); //Imposto il valore del prescaler

	/* TIM Interrupts enable */
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); //Abilito l'interruzione una volta che il contatore va in overflow


	/* Enable the TIM3 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE); //Abilito il contatore una volta che tutto � pronto

}

void PORT_Config(void)
{
	GPIO_InitTypeDef Pin_PA5;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	Pin_PA5.GPIO_Pin  = GPIO_Pin_5;
	Pin_PA5.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOA, &Pin_PA5);
}

/* Con questa funzione mi aspetto che ogni volta che il contatore si aggiorna e riparte da 0
 * (L'interrupt lo rilevo con "TIM_GetITStatus")
 */
void TIM3_IRQHandler(void) ///Mannaggia all'underscore!!!
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) //Se siamo qui per l'evento di UPDATE
	{
		tick++; //Incrementa la time base

		if((tick%4)==0) //Ogni tre secondi toggla
			{
				GPIO_ToggleBits(GPIOA,GPIO_Pin_5);
			}

		TIM_ClearITPendingBit(TIM3,TIM_IT_Update); //Interruzione onorata, puliscila
	}

}




