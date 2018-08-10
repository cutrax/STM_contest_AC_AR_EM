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
#include "myLib.h"
/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/


void USART2_myInit(void);

int main(void)
{
  myUSART2_Init();
  mySWITCH_Init();
  myLED_Init();
  myADC_Init();

  ADC_SoftwareStartConv(ADC1);

  //Adesso funziona, dare un'occhiata ai tempi di conversione per effettuare conversioni multiple

  //ADC_TempSensorVrefintCmd(ENABLE);
 /* ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor  , 1, ADC_SampleTime_15Cycles); //Attivo il canale 16
  ADC_EOCOnEachRegularChannelCmd(ADC1, ENABLE); //Abilito l'end of conversion
  ADC_ContinuousModeCmd(ADC1, ENABLE); //Abilito il modo di conversione continuo
  ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE); //Abilito l'interrupt su EOC
  ADC_TempSensorVrefintCmd(ENABLE); //Abilito il sensore di temperatura
  ADC_Cmd(ADC1, ENABLE); //Abilito l'ADC */


  /* Infinite loop */
  //printf("Hello World!\n");
  while (1)
  {
	  /*if((GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13) == 0))
	  	{
	  		if(USART_GetFlagStatus(USART2,USART_FLAG_RXNE) == SET) //Se ci sono dati in arrivo
	  			{
	  				if(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == SET) //Se � possibile trasmetterli
	  				{
	  					USART_SendData(USART2,USART_ReceiveData(USART2)); //Rimandali
	  				}

	  			}
	  	}
	  else
	  {
		  USART_ClearFlag(USART2,USART_FLAG_RXNE);
	  }
  }*/
//printf("Hello World!\n");

}
 // printf("Hello World!\n");
}

void ADC_IRQHandler(void)
{
	uint16_t conversionValue;
	//float tempValue;

	if((ADC_GetITStatus(ADC1, ADC_IT_EOC)) == SET){
        GPIO_SetBits(GPIOA, GPIO_Pin_5);
        //ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
		conversionValue = ADC_GetConversionValue(ADC1);
		printf("Ho convertito\n");
		//ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
	//	ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
		ADC_SoftwareStartConv(ADC1);
		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);

		}

}



