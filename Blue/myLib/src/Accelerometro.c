/*
 * Accelerometro.c
 *
 *  Created on: 14 set 2018
 *      Author: Emanuele
 */

#include "Accelerometro.h"
#include "fft.h"
#include "Elaborazioni.h"
#include <stm32f4xx_hal.h>
#include <stdint.h>
uint16_t cont; //Dal main
I2C_HandleTypeDef hi2c1; //Handle per I2C
float *storeBuf_X, *storeBuf_Y, *storeBuf_Z; //Dal main
int16_t prev_acc[3]; //X [0]->Y [1]->Z [2]

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

	prev_acc[0] = 0;
	prev_acc[1] = 0;
	prev_acc[2] = 0;
	uint8_t maskReg = 0x00;
	uint8_t maskReg1 = 0x00;
	//myI2C_Init();

	maskReg = (uint8_t) (MyGyrAcc_InitStruct->MyGyrOutput_DataRate | MyGyrAcc_InitStruct->MyGyrFull_Scale | \
	                     MyGyrAcc_InitStruct->MyGyrBandwith_Sel);

	maskReg1 = (uint8_t) (MyGyrAcc_InitStruct->MyAccOutput_DataRate | MyGyrAcc_InitStruct->MyAccFull_Scale | \
			              MyGyrAcc_InitStruct->MyAcc_Bandwith_Sel | MyGyrAcc_InitStruct->My_Acc_AntiAliasingBwSel);

	HAL_I2C_Mem_Write(&hi2c1,CHIP_ADDR,CTRL_REG1_G_ADDR, 1, &maskReg,I2C_MEMADD_SIZE_8BIT,100);
	HAL_I2C_Mem_Write(&hi2c1,CHIP_ADDR,CTRL_REG6_XL_ADDR, 1, &maskReg1,I2C_MEMADD_SIZE_8BIT,100);
	//Interruzioni per Acc
	uint8_t c = 0x01;
	HAL_I2C_Mem_Write(&hi2c1,CHIP_ADDR,INT_CTRL_ADDR, 1, &c,I2C_MEMADD_SIZE_8BIT,100);

	//Linea5
	__HAL_GPIO_EXTI_GENERATE_SWIT(EXTI_SWIER_SWIER5);
}

/*
 * Routine di gestione delle interruzioni provenienti dall'accelerometro
 */
void myAcc_Handler(void)
{
		//printf("Interrupt %d\n", cont);





		if(cont==n_C)
		{
			//Buffer pieno, stiamo perdendo dati
		}
		else
		{
			//Salvo i campioni che via via si vanno presentando
			storeBuf_X[cont] = ((((float)prev_acc[0])*LINEAR_ACC_SENSE1)/1000)*GRAVITY_ACC;
			storeBuf_Y[cont] = ((((float)prev_acc[1])*LINEAR_ACC_SENSE1)/1000)*GRAVITY_ACC;
			storeBuf_Z[cont] = ((((float)prev_acc[2])*LINEAR_ACC_SENSE1)/1000)*GRAVITY_ACC;
			cont++;
		}

		//Leggi tutti gli assi dell'accelerometro
		//A prescindere che ci sia lo spazio o meno!
		//Il clear dell'interruzione deve sempre avvenire
		//Non-blocking mode!
		HAL_I2C_Mem_Read(&hi2c1,CHIP_ADDR,0x28,I2C_MEMADD_SIZE_8BIT,(uint8_t*)&prev_acc,6,7000);

}
