/*
 * myLib.c
 *
 *  Created on: 07 ago 2018
 *      Author: cicci
 *
 *      In questo file delle funzioni ricorrenti da includere in tutti i nostri progetti
 */

#include "myLib.h"

/*
 * myADC_Init
 * Inizializza e abilita l'ADC.
 * Il micro che usiamo noi (f401) possiede solamente un ADC, che dalle librerie è definito
 * come ADC1.
 */


void myADC_Init(void)
{
ADC_InitTypeDef ADC_InitStructure;
ADC_CommonInitTypeDef ADC_CommonInitStructure;
NVIC_InitTypeDef NVIC_InitStructure;

RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //Abilito il clock per l'ADC

ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
ADC_CommonInit(&ADC_CommonInitStructure);

ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
ADC_InitStructure.ADC_ScanConvMode = DISABLE;
ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
ADC_InitStructure.ADC_NbrOfConversion = 1;
ADC_Init(ADC1, &ADC_InitStructure);

//ADC_ContinuousModeCmd(ADC1, ENABLE); //Abilito il modo di conversione continuo
ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor  , 1, ADC_SampleTime_144Cycles); //Attivo il canale 16
ADC_EOCOnEachRegularChannelCmd(ADC1, ENABLE); //Abilito l'end of conversion


ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE); //Abilito l'interrupt su EOC
//ADC_TempSensorVrefintCmd(ENABLE); //Abilito il sensore di temperatura
  //ADC_Cmd(ADC1, ENABLE); //Abilito l'ADC

  //Abilito le interruzioni provenienti dall'ADC
  NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

ADC_TempSensorVrefintCmd(ENABLE); //Abilito il sensore di temperatura
ADC_Cmd(ADC1, ENABLE); //Abilito l'ADC

}

/*
 * myUSART2_Init
 * Inizializza la USART2 con i relativi pin PA2(TX) e PA3(RX)
 * Inizializza anche il clock a AHB1_GPIOA e APB1_USART2
 */

void myUSART2_Init(void)
{
	USART_InitTypeDef usart2Init;
	GPIO_InitTypeDef pa2Init,pa3Init;


	//INIZIO configurazione della porta A
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

	//INIZIO configurazione di PA2 (AF07 USART2_TX)
	//Inizializzazione della struttura del pin
	GPIO_StructInit(&pa2Init);

	pa2Init.GPIO_Pin = GPIO_Pin_2;
	pa2Init.GPIO_Mode = GPIO_Mode_AF;
	//Push/Pull, NO PU, NO PD

	//Configura la funzione alternativa USART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);

	//Configura il pin
	GPIO_Init(GPIOA,&pa2Init);

	//FINE della configurazione di PA2

	//INIZIO configurazione di PA3 (AF07 USART2_RX)

	//Inizializzazione della struttura del pin
	GPIO_StructInit(&pa3Init);

	pa3Init.GPIO_Pin = GPIO_Pin_3;
	pa3Init.GPIO_Mode = GPIO_Mode_AF;
	//Push/Pull, NO PU, NO PD

	//Configura la funzione alternativa USART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);

	//Configura il pin
	GPIO_Init(GPIOA,&pa3Init);

	//FINE della configurazione di PA3


	//INIZIO configurazione di USART2

	//Abilito il clock alla periferica
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);


	//Inizializza la struttura di configurazione
	USART_StructInit(&usart2Init);
	//usart2Init.USART_BaudRate = 115200;
	//No parity, no flow control, RX_TX, 1 stop bit...

	//Configura la periferica
	USART_Init(USART2,&usart2Init);

	//Abilita la periferica
	USART_Cmd(USART2,ENABLE);

	//FINE della configurazione di USART2
}
/*
 *
 */


/*
 * myLED_Init
 * Inizializza il pin PA5 come output per pilotare il LED
 * Inizializza anche il clock a AHB1_GPIOA
 */
void myLED_Init(void)
{
	GPIO_InitTypeDef Pin_PA5;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_StructInit(&Pin_PA5);
	Pin_PA5.GPIO_Pin  = GPIO_Pin_5;
	Pin_PA5.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOA, &Pin_PA5);
}
/*
 *
 */


/*
 * mySWITCH_Init
 * Inizializza il pin PC13 come input per leggere lo SWITCH
 * Inizializza anche il clock a AHB1_GPIOC
 */
void mySWITCH_Init(void)
{
	GPIO_InitTypeDef Pin_PC13;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_StructInit(&Pin_PC13);
	Pin_PC13.GPIO_Pin  = GPIO_Pin_13;
	GPIO_Init(GPIOC, &Pin_PC13);
}
/*
 * Funzioni di scrittura personalizzate su USART2 da parte delle STDLIB
 * Attenzione: Funzioni BLOCCANTI!
 */
int __io_putchar(int ch)
{
 uint8_t c;
 c = ch & 0x00FF;
 while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)!=SET);
 USART_SendData(USART2,c);
 return ch;
}

int _write(int file,char *ptr, int len)
{
 int DataIdx;
 for(DataIdx= 0; DataIdx< len; DataIdx++)
 {
 __io_putchar(*ptr++);
 }
return len;
}
/*
 *
 */
