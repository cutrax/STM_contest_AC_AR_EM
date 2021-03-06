/*
 * myLib.c
 *
 *  Created on: 07 ago 2018
 *      Author: cicci
 *
 *      In questo file delle funzioni ricorrenti da includere in tutti i nostri progetti
 */

#include "myLib.h"
#include "Accelerometro.h"

//Variabili per HumTemp
static float m_T,m_H,q_T,q_H;
static float T,H;
static uint8_t H_T_newValues;

//Variabili per Bar
static float P;
static uint8_t P_newValues;

//Variabili per AccGyr
//static float GYRO_X,GYRO_Y,GYRO_Z;
//static float ACC_X,ACC_Y,ACC_Z;
static uint8_t GYR_ACC_newValues;

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
	usart2Init.USART_BaudRate = 115200;
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
 * INIZIO FUNZIONI I2C
 */
/*
 * myI2C_Init
 * Inizializza la I2C1 con i relativi pin PB8 e PB9 (SCL e SDA)
 * Inizializza anche il clock a AHB1_GPIOB e APB1_I2C1
 */
void myI2C_Init(void)
{
	GPIO_InitTypeDef pb89Init;
	I2C_InitTypeDef i2c1Init;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //Abilita il clock a GPIOB
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE); //Abilita il clock a I2C1

	//Configura PB8 (SCL) e PB9 (SDA)

	/*
	 * Cicla i pin SCL e SDA per correggere un bug sulla I2C (ST)
	 */
	GPIO_StructInit(&pb89Init);
	pb89Init.GPIO_Mode = GPIO_Mode_OUT;
	pb89Init.GPIO_OType = GPIO_OType_OD;
	pb89Init.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	pb89Init.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB,&pb89Init);

	GPIO_WriteBit(GPIOB,GPIO_Pin_8,1); //SCL high
	GPIO_WriteBit(GPIOB,GPIO_Pin_9,1); //SDA high

	while((GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8) == 0) || (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9) == 0));

	GPIO_WriteBit(GPIOB,GPIO_Pin_9,0); //SDA low
	while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9) != 0);

	GPIO_WriteBit(GPIOB,GPIO_Pin_8,0); //SCL low
	while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8) != 0);

	GPIO_WriteBit(GPIOB,GPIO_Pin_8,1); //SCL high
	while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8) != 1);

	GPIO_WriteBit(GPIOB,GPIO_Pin_9,1); //SDA high
	while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9) != 1);

	/*
	 * INIZIO configurazione standard
	 */
	//SCL e SDA devono essere open drain con pull-up, modo alternate
	pb89Init.GPIO_Mode = GPIO_Mode_AF;

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9, GPIO_AF_I2C1);

	GPIO_Init(GPIOB,&pb89Init);

	//I pin sono pronti per I2C

	//Inizializzazione I2C
	I2C_StructInit(&i2c1Init);

	I2C_Init(I2C1,&i2c1Init);

	I2C_Cmd(I2C1, ENABLE);

}
/*
 *
 */
/*
 * myI2C_ReadReg
 * Legge un registro Reg della periferica all'indirizzo BaseAddr
 * E' necessario chiamare prima myI2C_Init
 */

uint8_t myI2C_ReadReg(uint8_t BaseAddr,uint8_t Reg)
{
	uint8_t temp;
	myI2C_MultipleReadReg(BaseAddr,Reg,&temp,1,0);
	return temp;
}

/*
 *
 */

/*
 * myI2C_MultipleReadReg
 * Legge una seguenza di registri (autoInc=1) o un unico registro ripetutamente (autoInc=0)
 * buf � un puntatore al buffer ove riversare i dati
 * cnt � il numero di letture N da effettuare (0 non ha senso)
 * E' necessario chiamare prima myI2C_Init
 */

void myI2C_MultipleReadReg(uint8_t BaseAddr, uint8_t Reg, uint8_t *buf, uint8_t cnt, uint8_t autoInc)
{
	__disable_irq();

	I2C_GenerateSTART(I2C1, ENABLE); //Fai partire lo START
	//Attendi che lo SB vada ad uno, quando il while se ne accorger�, avr� letto ancora
	//una volta lo SR1, e EV5 � soddisfatto
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_SB) != SET);

	//Invia l'indirizzo della periferica in write
	I2C_Send7bitAddress(I2C1,BaseAddr,I2C_Direction_Transmitter);
	//Aspetta che l'indirizzo sia trasmesso  e che la periferica abbia dato ACK
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_ADDR) !=SET);
	//Read SR1 inutile, gi� il while lo ha letto dopo l'evento
	//Leggi SR2
	I2C_ReadRegister(I2C1,I2C_Register_SR2);
	//EV6 onorato

	//Manda come primo byte l'indirizzo del registro
	//MSb settato a 1 se si desidera l'autoincremento
	I2C_SendData(I2C1, Reg | (autoInc << 7));
	//Single Byte transfer, si passa direttamente a EV8_2, assicurandosi che la trasmissione � terminata
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BTF)!=SET);

	//Non generare una STOP
	//Siamo in master mode, lo SB ha valore di ReStart
	//Inizia la serie di transazioni di lettura

	//START
	I2C_GenerateSTART(I2C1, ENABLE);
	//Attendi che lo SB vada ad uno, quando il while se ne accorger�, avr� letto ancora
	//una volta lo SR1, e EV5 � soddisfatto
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_SB) != SET);

	//Invia l'indirizzo della periferica in read
	I2C_Send7bitAddress(I2C1,BaseAddr,I2C_Direction_Receiver);
	//Aspetta che l'indirizzo sia trasmesso  e che la periferica abbia dato ACK
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_ADDR) !=SET);
	//Read SR1 inutile, gi� il while lo ha letto dopo l'evento
	if(cnt==1) //Se � una single read si deve programmare subito un NACK
	{
		I2C_AcknowledgeConfig(I2C1, DISABLE);
	}
	else
	{
		I2C_AcknowledgeConfig(I2C1, ENABLE);
	}
	//Leggi SR2
	I2C_ReadRegister(I2C1,I2C_Register_SR2);
	//EV6 onorato (ADDR cleared)

	//Se � una single read si deve programmare subito lo STOP
	if(cnt==1)
	{
		I2C_GenerateSTOP(I2C1,ENABLE);
	}

	//Qui si entra solo � realmente una multiple read (cnt>1)
	cnt--; //Artificio per gestire nel while tutti i byte tranne l'ultimo
	while(cnt)
	{
		//Attendi di avere dati nel DR da potere prelevare
		while(I2C_GetFlagStatus(I2C1,I2C_FLAG_RXNE) != SET);

		//Dopo questo while l'interfaccia sta leggendo gi� il prossimo byte...
		//Salva il byte, e post-incrementa il puntatore
		*buf++ = I2C_ReceiveData(I2C1);

		//Se abbiamo letto il penultimo byte, � ora di programmare ACK=0 (NACK) e STOP=1
		//infatti l'interfaccia sta leggendo l'ultimo byte
		//Inoltre cnt dopo questo if sar� 0, il while terminer� alla fine di questa iterazione
		if(cnt==1)
		{
			I2C_AcknowledgeConfig(I2C1, DISABLE);
			I2C_GenerateSTOP(I2C1, ENABLE);
		}

		//Un byte in meno da leggere...
		cnt--;

	}

	//Non disperdere l'ultimo byte letto, attendi la sua lettura
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_RXNE) != SET);
	*buf = I2C_ReceiveData(I2C1); //E' gi� incrementato nel while, oppure � una single read

	__enable_irq();
}

/*
 *
 */

/*
 * myI2C_WriteReg
 * Scrive Data su un registro Reg della periferica all'indirizzo BaseAddr
 * E' necessario chiamare prima myI2C_Init
 */

void myI2C_WriteReg(uint8_t BaseAddr,uint8_t Reg, uint8_t Data)
{
	myI2C_MultipleWriteReg(BaseAddr,Reg,&Data,1,0);
}

/*
 *
 */

/*
 * myI2C_MultipleWriteReg
 * Scrive una seguenza di registri (autoInc=1) o un unico registro ripetutamente (autoInc=0)
 * buf � un puntatore al buffer ove leggere i dati
 * cnt � il numero di scritture N da effettuare (0 non ha senso)
 * E' necessario chiamare prima myI2C_Init
 */

void myI2C_MultipleWriteReg(uint8_t BaseAddr, uint8_t Reg, uint8_t *buf, uint8_t cnt, uint8_t autoInc)
{
	__disable_irq();

	//Se cnt � nullo, esci
	if (cnt==0) return;


	I2C_GenerateSTART(I2C1, ENABLE); //Fai partire lo START
	//Attendi che lo SB vada ad uno, quando il while se ne accorger�, avr� letto ancora
	//una volta lo SR1, e EV5 � soddisfatto
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_SB) != SET);

	//Invia l'indirizzo della periferica in write
	I2C_Send7bitAddress(I2C1,BaseAddr,I2C_Direction_Transmitter);
	//Aspetta che l'indirizzo sia trasmesso  e che la periferica abbia dato ACK
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_ADDR) !=SET);
	//Read SR1 inutile, gi� il while lo ha letto dopo l'evento
	//Leggi SR2
	I2C_ReadRegister(I2C1,I2C_Register_SR2);
	//EV6 onorato

	//Manda come primo byte l'indirizzo del registro
	//Se autoInc � presente, setta MSb
	I2C_SendData(I2C1,Reg | (autoInc << 7));
	//EV8_1 onorato


	while(cnt)
	{
		//Aspetto il momento giusto per caricare il data byte in DR (TXE set)
		while(I2C_GetFlagStatus(I2C1,I2C_FLAG_TXE)!=SET);
		//All'uscita del while di sopra l'interfaccia sta scrivendo il byte
		//caricato in DR all'iterazione precedente
		//Ma il DR � pronto per ricevere il dato successivo
		//Post-incrementa il puntatore
		I2C_SendData(I2C1,*buf++);
		//EV8 onorato
		cnt--;
	}

	//All'uscita del while(cnt=0) abbiamo cariato tutti i byte
	//Non stiamo riprogrammando l'interfaccia per un altro byte, dunque � l'ultimo.
	//In queste condizioni BTF indica l'ACK dello slave dell'ultimo byte
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BTF) != SET);
	//Ora si programma lo STOP
	I2C_GenerateSTOP(I2C1,ENABLE);

	__enable_irq();
}

/*
 *
 */

/*
 * FINE FUNZIONI I2C
 */

/*
 * WATCHDOG
 * *************ATTENZIONE**********
 * **UTILIZZARE LA FUNZIONE "IWDG_ReloadCounter()" per far ricominciare il conteggio, altrimenti
 * il watchdog va a zero e il sistema si resetta.**
 * *********************************
 *  */
void myWatchDog_Init(void)
{
	//Abilito l'accesso ai registri per inserire il valore di prescaler e di conteggio da cui partire
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	//Setto il prescaler
	IWDG_SetPrescaler(IWDG_Prescaler_4);
	//Imposto il conteggio iniziale (per 10ms), per info guardare la tabella 62 del manuale a pag 416.
	/*
	 * 0x0FFF sta a 512 ms, 0x000 sta a 0.125ms. Pertanto se vogliamo contare per 10ms:
	 *
	 * 4096:512ms = X:10ms; ->  X= (4096*10)/512 = 80 = 0x0050.
	 */
	IWDG_SetReload(0x0050);
	//Abilito il watchdog
	IWDG_Enable();
}

/*
 * FINE WATCHDOG
 */

/*
 * INIZIO FUNZIONI MEMS IKS01A1
 */

/*
 * myMEMSBoard_Init
 * Inizializza tutta la scheda MEMS IKS01A1
 * Chiama tutte le _Init dei vari sensori e inizializza l' I2C1
 */

void myMEMSBoard_Init(void)
{
myI2C_Init(); //Bus I2C
//	myMag_Init(); //Magnetometro
myBar_Init(); //Barometro
myHumTemp_Init(); //Umidit� e Temperatura
}

/*
 * myBar_Init
 * Inizializza il barometro LPS25HB con:
 * -Refresh 1Hz
 * -Continuous mode
 */

void myBar_Init(void)
{
	GPIO_InitTypeDef pb4Init;

	EXTI_InitTypeDef EXTIInit;

	NVIC_InitTypeDef NVICInit;

	//Configuro PB4 per DRDY
	GPIO_StructInit(&pb4Init);
	pb4Init.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOB,&pb4Init);

	//Configuro SYSCFG
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource4);

	//Configuro ExTI
	EXTIInit.EXTI_Line = EXTI_Line4;
	EXTIInit.EXTI_LineCmd = ENABLE;
	EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTIInit);

	//Configuro NVIC
	NVICInit.NVIC_IRQChannel = EXTI4_IRQn;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0;
	NVICInit.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVICInit);

	//Continuous mode, 1 Hz, PowerUP
	myI2C_WriteReg(0xBA,0x20,0b10010000);

	//Data Ready on DRDY (CTRL_REG4)
	myI2C_WriteReg(0xBA,0x23,0b00000001);

	EXTI_GenerateSWInterrupt(EXTI_Line4);

}

void EXTI4_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line4) == SET)
	{
		s32 var=0;

		//Scarica 3 byte in un colpo solo XL 0x28->L 0x29->H 0x2A
		myI2C_MultipleReadReg(0xBA,0x28,(uint8_t *) &var,3,1);

		P = (float) var / 4096; //hPa
		P_newValues = SET;

		//ISR completata, pulisci il flag
		EXTI_ClearITPendingBit(EXTI_Line4);
	}
}

/*
 *
 */

/*
 * myBar_Get
 * Inizializza il barometro LPS25HB con:
 * -Refresh 1Hz
 * -Continuous mode
 */

float myBar_Get(void)
{
	P_newValues = RESET;
	return P;

}

/*
 *
 */

/*
 * myBar_newData
 * SET se vi sono nuovi dati non ancora letti dall'utente, RESET altrimenti
 */

uint8_t myBar_newData(void)
{
	return P_newValues;
}

/*
 *
 */

/*
 * myHumTemp_Init
 * Inizializza il sensore HTS221 con:
 * -Refresh 1Hz
 * -Continuous mode
 * Costrusice le rette di interpolazione per umidit� e temperatura
 */

void myHumTemp_Init(void)
{
	EXTI_InitTypeDef EXTIInit;

	NVIC_InitTypeDef NVICInit;

	GPIO_InitTypeDef pb10Init;

	s16 T0_OUT,T1_OUT,H0_OUT,H1_OUT;
	uint16_t T0_deg,T1_deg;
	uint8_t H0_r,H1_r;
	uint8_t T1_T0_MSB;
	uint8_t table[16];

	//Scarica la tabella
	myI2C_MultipleReadReg(0xBE,0x30,table,16,1);

	//Retta di interpolazione per la temperatura
	T0_OUT = (s16)table[0xD] << 8 | (s16)table[0xC];
	T1_OUT = (s16)table[0xF] << 8 | (s16)table[0xE];


	T1_T0_MSB = table[5];
	T0_deg = ((uint16_t) (T1_T0_MSB & 0b11) << 8) | (s16)table[0X2];
	T1_deg = ((uint16_t) (T1_T0_MSB & 0b1100) << 6) | (s16)table[0X3];

	m_T = (float) (T1_deg - T0_deg) / (float)(T1_OUT - T0_OUT) /8;
	q_T = (T0_deg - m_T*T0_OUT) /8;

	//Retta di interpolazione per l'umidit� percentuale
	H0_OUT = (s16)table[0x7] << 8 | (s16)table[0x6];
	H1_OUT = (s16)table[0xB] << 8 | (s16)table[0xA];

	H0_r = table[0x0];
	H1_r = table[0x1];

	m_H = (float) (H1_r - H0_r) / (float)(H1_OUT - H0_OUT) /2;
	q_H = (H0_r - m_H*H0_OUT) /2;

	//Calibrazione terminata

	//Abilita il pin DRDY(CN9->7->PB10) e relative le interruzioni
	GPIO_StructInit(&pb10Init); //Default come input
	pb10Init.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOB,&pb10Init);

	//Configura SYSCFG
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource10);

	//Configura EXTI
	EXTIInit.EXTI_Line = EXTI_Line10;
	EXTIInit.EXTI_LineCmd = ENABLE;
	EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTIInit);

	//Configura NVIC
	NVICInit.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0;
	NVICInit.NVIC_IRQChannelSubPriority = 1;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVICInit);

	//Fai partire il sensore

	//Continuous mode, 1Hz
	myI2C_WriteReg(0xBE,0x20,0b10000001);

	//Abilita DRDY
	myI2C_WriteReg(0xBE,0x22,0b00000100);

	EXTI_GenerateSWInterrupt(EXTI_Line10);

}


void EXTI15_10_IRQHandler()
{
	if(EXTI_GetITStatus(EXTI_Line10) == SET) //ISR per HTS221
	{
		s16 var[2]; //Alloca 4 byte, ovvero due word per memorizzare in un colpo solo
		//var[0] HUM REGS 0x28->0x29
		//var[1] TEMP REGS 0x2A->0x2B
		//Leggi entrambe le grandezze in un colpo solo
		myI2C_MultipleReadReg(0xBE,0x28,(uint8_t *) &var,4,1);
		T = (m_T*var[1]) + q_T;
		H = (m_H*var[0]) + q_H;
		H_T_newValues = SET;

		//ISR completata, pulisci il flag
		EXTI_ClearITPendingBit(EXTI_Line10);
	}

}

/*
 *
 */

/*
 * myTemp_Get
 * Ricava la temperatura mediante interpolazione dal sensore
 */

float myHumTemp_Temp_Get(void)
{

	H_T_newValues = RESET;

	return T;
}

/*
 *
 */

/*
 * myHum_Get
 * Ricava l' umidit� percentuale mediante interpolazione dal sensore
 */

float myHumTemp_Hum_Get(void)
{

	H_T_newValues = RESET;

	return H;
}


/*
 * myHumTemp_newData
 * SET se vi sono nuovi dati non ancora letti dall'utente, RESET altrimenti
 */

uint8_t myHumTemp_newData(void)
{
	return H_T_newValues;
}

/*
 * FINE FUNZIONI MEMS IKS01A1
 */

/*myAcc_Get_X, myAcc_Get_Y, myAcc_Get_Z
 * Leggo l'accelerazione rispettivamente lungo
 * gli assi X, Y, e Z.
*/

/*float myAcc_Get_X(void)
{
	GYR_ACC_newValues = RESET;
	return ACC_X;
}

float myAcc_Get_Y(void)
{
	GYR_ACC_newValues = RESET;
	return ACC_Y;
}

float myAcc_Get_Z(void)
{
	GYR_ACC_newValues = RESET;
	return ACC_Z;
}*/

/*
 * myGyr_Get_X, myGyr_Get_Y, myGyr_Get_Z
 * Leggo i valori della velocit� angolare lungo
 * X,Y, e Z.
 */

/*float myGyr_Get_X(void)
{
	GYR_ACC_newValues = RESET;
	return GYRO_X;
}

float myGyr_Get_Y(void)
{
	GYR_ACC_newValues = RESET;
	return GYRO_Y;
}

float myGyr_Get_Z(void)
{
	GYR_ACC_newValues = RESET;
	return GYRO_Z;
}*/

/*
 *
 */

/*
 * myGyrAcc_newData
 * SET se vi sono nuovi dati non ancora letti dall'utente, RESET altrimenti
 */

uint8_t myGyrAcc_newData(void)
{
	return GYR_ACC_newValues;
}

/*
 * INIZIO FUNZIONI VARIE
 */

/*
 * myDelay_ms
 * Blocca il programma per del ms
 */

void myDelay_ms(u32 del)
{
	del *= 11947; //Valore empirico con SYSCLK 84 MHz
	while(del--);
}

/*
 * my2decs
 * Restituisce i primi due decimali di un float
 */

uint8_t my2decs(float var)
{
	return (int) ((var - floorf(var)) * 100);
}

/*
 * myInt
 * Restituisce la parte intera sinistra
 */

uint16_t myInt(float var)
{
	return (int) floorf(var);
}

/*
 * FINE FUNZIONI VARIE
 */

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








