/*
 * Accelerometro.c
 *
 *  Created on: 14 set 2018
 *      Author: Emanuele
 */

#include "Accelerometro.h"
#include "myLib.h"

u16 cont; //Dal main
float *storeBuf_X, *storeBuf_Y, *storeBuf_Z; //Dal main

/**myGyrAcc_StructInit: INIZIALIZZAZIONE STRUTTURA
 *
 *
 * -GIROSCOPIO: SPENTO (Powered down);
 *
 * Solo accelerometro acceso;
 *
 * -OUTPUT DATA RATE ACCELEROMETRO: l'ho impostato in modo da abilitare solo l'accelerometro.
 * (istruzioni seguite passo passo dal manuale), cioè ODR = 110 (952 Hz).
 *
 *
 * -SCALA ACCELEROMETRO: (+/-)2g;
 *
 * -BANDA ACCELEROMETRO: 408 Hz;
 *
 *  Bandwidth selection. Default value: 0
(0: bandwidth determined by ODR selection:
- BW = 408 Hz when ODR = 952 Hz, 50 Hz, 10 Hz;
- BW = 211 Hz when ODR = 476 Hz;
- BW = 105 Hz when ODR = 238 Hz;
- BW = 50 Hz when ODR = 119 Hz;
1: bandwidth selected according to BW_XL [1:0] selection);
 *
 * -BANDA FILTRO ANTI-ALIASING: 408Hz;
 */
void myGyrAcc_StructInit(MyGyrAcc_InitTypeDef *MyGyrAcc_InitStruct)
{
	      MyGyrAcc_InitStruct->MyGyrOutput_DataRate = GYR_ODR_1;
	 	  MyGyrAcc_InitStruct->MyGyrFull_Scale = GYR_FULLSCALE_245;
	 	  MyGyrAcc_InitStruct->MyGyrBandwith_Sel = BANDWITH_00;
	 	  MyGyrAcc_InitStruct->MyAccOutput_DataRate = ACC_ODR_7;
	 	  MyGyrAcc_InitStruct->MyAccFull_Scale = ACC_FULLSCALE_10;
	 	  MyGyrAcc_InitStruct->MyAcc_Bandwith_Sel = BW_SCAL_ODR_0;
	 	  MyGyrAcc_InitStruct->My_Acc_AntiAliasingBwSel = BW_XL_00;
}

/**myAccBoard_Init:
 * Attiva solo l'accelerometro.
 *
 */
void myAccBoard_Init(void)
{
	MyGyrAcc_InitTypeDef MyGyrAcc_InitStructure;

	myI2C_Init(); //Bus I2C
	//Inizializzo accelerometro alla frequenza di campionamento di 952 Hz
	//banda filtro anti-aliasing: 408 Hz, giroscopio spento
	myGyrAcc_StructInit(&MyGyrAcc_InitStructure);
	myGyrAcc_Init(&MyGyrAcc_InitStructure);
}

/**myGyrAcc_Init:
 * scrive negli opportuni registri di controllo del chip (in questo caso
 * CTRL_REG1_G, CTRL_REG6_XL), i valori dichiarati nella struttura.
 */

void myGyrAcc_Init(MyGyrAcc_InitTypeDef *MyGyrAcc_InitStruct)
{
	GPIO_InitTypeDef pb5Init;

	EXTI_InitTypeDef EXTIInit;

	NVIC_InitTypeDef NVICInit;

	uint8_t maskReg = 0x00;
	uint8_t maskReg1 = 0x00;
	//myI2C_Init();

	maskReg = (uint8_t) (MyGyrAcc_InitStruct->MyGyrOutput_DataRate | MyGyrAcc_InitStruct->MyGyrFull_Scale | \
	                     MyGyrAcc_InitStruct->MyGyrBandwith_Sel);

	maskReg1 = (uint8_t) (MyGyrAcc_InitStruct->MyAccOutput_DataRate | MyGyrAcc_InitStruct->MyAccFull_Scale | \
			              MyGyrAcc_InitStruct->MyAcc_Bandwith_Sel | MyGyrAcc_InitStruct->My_Acc_AntiAliasingBwSel);

	//Abilita il pin DRDY(CN9->5->PB5) e le relative interruzioni
	GPIO_StructInit(&pb5Init); //Default come input
	pb5Init.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOB,&pb5Init);

	//Configura SYSCFG
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource5);

	//Configura EXTI
	EXTIInit.EXTI_Line = EXTI_Line5;
	EXTIInit.EXTI_LineCmd = ENABLE;
	EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTIInit);

	//Configura NVIC
	NVICInit.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0;
	NVICInit.NVIC_IRQChannelSubPriority = 1;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVICInit);

	myI2C_WriteReg(CHIP_ADDR, CTRL_REG1_G_ADDR , maskReg);
	myI2C_WriteReg(CHIP_ADDR, CTRL_REG6_XL_ADDR, maskReg1);
	//Interruzioni per Acc
	myI2C_WriteReg(CHIP_ADDR,INT_CTRL_ADDR, 0b00000001);

	EXTI_GenerateSWInterrupt(EXTI_Line5);
}

/*
 * Routine di gestione delle interruzioni provenienti dall'accelerometro
 */
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line5) == SET) //Acc Gyr
	{
		//printf("Interrupt %d\n", cont);
		s16 acc[3]; //X [0]->Y [1]->Z [2]

		//Leggi tutti gli assi dell'accelerometro
		//A prescindere che ci sia lo spazio o meno!
		//Il clear dell'interruzione deve sempre avvenire
		myI2C_MultipleReadReg(CHIP_ADDR,0x28,(uint8_t *)&acc,6,1);

		if(cont==n_C)
		{
			//Buffer pieno, stiamo perdendo dati
		}
		else
		{
			//Salvo i campioni che via via si vanno presentando
			storeBuf_X[cont] = ((((float)acc[0])*LINEAR_ACC_SENSE0)/1000)*GRAVITY_ACC;
			storeBuf_Y[cont] = ((((float)acc[1])*LINEAR_ACC_SENSE0)/1000)*GRAVITY_ACC;
			storeBuf_Z[cont] = ((((float)acc[2])*LINEAR_ACC_SENSE0)/1000)*GRAVITY_ACC;
			cont++;
		}

		//GYR_ACC_newValues = SET;

		EXTI_ClearITPendingBit(EXTI_Line5);
	}
}
