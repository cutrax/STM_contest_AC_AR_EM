/*
 * myLib.c
 *
 *  Created on: 07 ago 2018
 *      Author: cicci
 *
 *      In questo file delle funzioni ricorrenti da includere in tutti i nostri progetti
 */

#include "myLib.h"

//Variabili per HumTemp
static float m_T,m_H,q_T,q_H;


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
	GPIO_StructInit(&pb89Init);

	//SCL e SDA devono essere open drain con pull-up, modo alternate
	pb89Init.GPIO_Mode = GPIO_Mode_AF;

	pb89Init.GPIO_OType = GPIO_OType_OD;

	pb89Init.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;

	pb89Init.GPIO_PuPd = GPIO_PuPd_NOPULL;

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
	I2C_GenerateSTART(I2C1, ENABLE); //Fai partire lo START
	//Attendi che lo SB vada ad uno, quando il while se ne accorgerà, avrà letto ancora
	//una volta lo SR1, e EV5 è soddisfatto
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_SB) != SET);

	//Invia l'indirizzo della periferica in write
	I2C_Send7bitAddress(I2C1,BaseAddr,I2C_Direction_Transmitter);
	//Aspetta che l'indirizzo sia trasmesso  e che la periferica abbia dato ACK
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_ADDR) !=SET);
	//Read SR1 inutile, già il while lo ha letto dopo l'evento
	//Leggi SR2
	I2C_ReadRegister(I2C1,I2C_Register_SR2);
	//EV6 onorato

	//Manda come primo byte l'indirizzo del registro
	I2C_SendData(I2C1,Reg);
	//Single Byte transfer, si passa direttamente a EV8_2, assicurandosi che la trasmissione è terminata
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BTF)!=SET);

	//Non generare una STOP
	//Siamo in master mode, lo SB ha valore di ReStart
	//Inizia la transazione di lettura

	//START
	I2C_GenerateSTART(I2C1, ENABLE);
	//Attendi che lo SB vada ad uno, quando il while se ne accorgerà, avrà letto ancora
	//una volta lo SR1, e EV5 è soddisfatto
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_SB) != SET);

	//Invia l'indirizzo della periferica in read
	I2C_Send7bitAddress(I2C1,BaseAddr,I2C_Direction_Receiver);
	//Aspetta che l'indirizzo sia trasmesso  e che la periferica abbia dato ACK
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_ADDR) !=SET);
	//Read SR1 inutile, già il while lo ha letto dopo l'evento
	//Leggi SR2
	I2C_ReadRegister(I2C1,I2C_Register_SR2);
	//EV6 onorato

	//L'ACK non è mai stato programmato, ottimo, vista la necessità di un NACK
	//I2C_AcknowledgeConfig(I2C1, DISABLE);

	//L'interfaccia sta leggendo il byte...
	//Nel frattempo programma lo STOP alla fine della lettura
	I2C_GenerateSTOP(I2C1, ENABLE);
	//Attendi che il dato sia stato caricato
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_RXNE) != SET);
	//L'interfaccia darà lo STOP programmato
	return I2C_ReceiveData(I2C1);
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
	I2C_GenerateSTART(I2C1, ENABLE); //Fai partire lo START
	//Attendi che lo SB vada ad uno, quando il while se ne accorgerà, avrà letto ancora
	//una volta lo SR1, e EV5 è soddisfatto
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_SB) != SET);

	//Invia l'indirizzo della periferica in write
	I2C_Send7bitAddress(I2C1,BaseAddr,I2C_Direction_Transmitter);
	//Aspetta che l'indirizzo sia trasmesso  e che la periferica abbia dato ACK
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_ADDR) !=SET);
	//Read SR1 inutile, già il while lo ha letto dopo l'evento
	//Leggi SR2
	I2C_ReadRegister(I2C1,I2C_Register_SR2);
	//EV6 onorato

	//Manda come primo byte l'indirizzo del registro
	I2C_SendData(I2C1,Reg);
	//EV8_1 onorato

	//Aspetto il momento giusto per caricare il secondo byte in DR (TXE set)
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_TXE)!=SET);
	I2C_SendData(I2C1,Data);
	//EV8 onorato

	//L'interfaccia, mentre il secondo byte veniva scritto, non è stata programmata
	//per un ulteriore byte, dunque è l'ultimo.
	//In queste condizioni BTF indica l'ACK dello slave dell'ultimo byte
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BTF) != SET);
	//Ora si programma lo STOP
	I2C_GenerateSTOP(I2C1,ENABLE);

}

/*
 *
 */

/*
 * FINE FUNZIONI I2C
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

MyGyrAcc_InitTypeDef MyGyrAcc_InitStructure;

myI2C_Init(); //Bus I2C
//	myMag_Init(); //Magnetometro
myBar_Init(); //Barometro
myHumTemp_Init(); //Umidità e Temperatura

//Inizializzo accelerometro e giroscopio con parametri di default
myGyrAcc_StructInit(&MyGyrAcc_InitStructure);
myGyrAcc_Init(&MyGyrAcc_InitStructure);
}

/*myAccGyr_Init()
 * Abilito accelerometro e giroscopio
 *
 *
 */

/*void myAccGyr_Init(void)
{
	// "Accendo" il giroscopio
	myI2C_WriteReg(0xD6, 0x10, 0x40);
	//"Accendo" l'accelerometro
	myI2C_WriteReg(0xD6, 0x20, 0x00);
	//Abilito le interruzioni quando il dato è pronto
	//myI2C_WriteReg(0xD4, 0x0C, 0x41);
}*/

/*myAcc_Get_X, myAcc_Get_Y, myAcc_Get_Z
 * Leggo l'accelerazione rispettivamente lungo
 * gli assi X, Y, e Z.
*/

int myAcc_Get_X(void)
{
	u16 acc_x = myI2C_ReadReg(0xD6, 0x29) << 8 | myI2C_ReadReg(0xD6, 0x28);
	return acc_x;
}

int myAcc_Get_Y(void)
{
	u16 acc_y = myI2C_ReadReg(0xD6, 0x2B) << 8 | myI2C_ReadReg(0xD6, 0x2A);
	return acc_y;
}

int myAcc_Get_Z(void)
{
	u16 acc_z = myI2C_ReadReg(0xD6, 0x2D) << 8 | myI2C_ReadReg(0xD6, 0x2C);
	return acc_z;
}

/*
 * myGyr_Get_X, myGyr_Get_Y, myGyr_Get_Z
 * Leggo i valori della velocità angolare lungo
 * X,Y, e Z.
 */
//Al momento per semplicità definite come int
int myGyr_Get_X(void)
{
	u16 gyr_x = myI2C_ReadReg(0xD6, 0x19) << 8 | myI2C_ReadReg(0xD6, 0x18);
	return gyr_x;
}

int myGyr_Get_Y(void)
{
	u16 gyr_y = myI2C_ReadReg(0xD6, 0x1B) << 8 | myI2C_ReadReg(0xD6, 0x1A);
	return gyr_y;
}

int myGyr_Get_Z(void)
{
	u16 gyr_z = myI2C_ReadReg(0xD6, 0x1D) << 8 | myI2C_ReadReg(0xD6, 0x1C);
	return gyr_z;
}


/*
 * myBar_Init
 * Inizializza il barometro LPS25HB con:
 * -Refresh 1Hz
 * -Continuous mode
 */

void myBar_Init(void)
{
	//Continuous mode, 1 Hz, PowerUP
	myI2C_WriteReg(0xBA,0x20,0b10010000);

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
	s32 temp = myI2C_ReadReg(0xBA,0x2A) << 16 | myI2C_ReadReg(0xBA,0x29) << 8 | myI2C_ReadReg(0xBA,0x28);
	return (float) temp / 4096;

}

/*
 *
 */

/*
 * myHumTemp_Init
 * Inizializza il sensore HTS221 con:
 * -Refresh 1Hz
 * -Continuous mode
 * Costrusice le rette di interpolazione per umidità e temperatura
 */

void myHumTemp_Init(void)
{
	s16 T0_OUT,T1_OUT,H0_OUT,H1_OUT;
	uint16_t T0_deg,T1_deg;
	uint8_t H0_r,H1_r;
	uint8_t T1_T0_MSB;

	//Continuous mode, 1Hz
	myI2C_WriteReg(0xBE,0x20,0b10000001);

	//Retta di interpolazione per la temperatura
	T0_OUT = (s16)myI2C_ReadReg(0xBE,0x3D) << 8 | (s16)myI2C_ReadReg(0xBE,0x3C);
	T1_OUT = (s16)myI2C_ReadReg(0xBE,0x3F) << 8 | (s16)myI2C_ReadReg(0xBE,0x3E);

	T1_T0_MSB = myI2C_ReadReg(0xBE,0x35);
	T0_deg = ((uint16_t) (T1_T0_MSB & 0b11) << 8) | (s16)myI2C_ReadReg(0xBE,0x32);
	T1_deg = ((uint16_t) (T1_T0_MSB & 0b1100) << 6) | (s16)myI2C_ReadReg(0xBE,0x33);

	m_T = (float) (T1_deg - T0_deg) / (float)(T1_OUT - T0_OUT) /8;
	q_T = (T0_deg - m_T*T0_OUT) /8;

	//Retta di interpolazione per l'umidità percentuale
	H0_OUT = (s16)myI2C_ReadReg(0xBE,0x37) << 8 | (s16)myI2C_ReadReg(0xBE,0x36);
	H1_OUT = (s16)myI2C_ReadReg(0xBE,0x3B) << 8 | (s16)myI2C_ReadReg(0xBE,0x3A);

	H0_r = myI2C_ReadReg(0xBE,0x30);
	H1_r = myI2C_ReadReg(0xBE,0x31);

	m_H = (float) (H1_r - H0_r) / (float)(H1_OUT - H0_OUT) /2;
	q_H = (H0_r - m_H*H0_OUT) /2;
}

/*
 *
 */

/*
 * myTemp_Get
 * Ricava la temperatura mediante interpolazione dal sensore
 */

float myTemp_Get(void)
{
	s16 temp;

	temp = (s16)myI2C_ReadReg(0xBE,0x2B) << 8 | (s16)myI2C_ReadReg(0xBE,0x2A);

	return (m_T*temp + q_T);
}

/*
 *
 */

/*
 * myHum_Get
 * Ricava l' umidità percentuale mediante interpolazione dal sensore
 */

float myHum_Get(void)
{
	s16 temp;

	temp = (s16)myI2C_ReadReg(0xBE,0x29) << 8 | (s16)myI2C_ReadReg(0xBE,0x28);

	return (m_H*temp + q_H);
}

/*
 *
 */

/*
 * FINE FUNZIONI MEMS IKS01A1
 */


/*FUNZIONI MEMS LSM6DS0 ACCELEROMETRO E GIROSCOPIO
 *
 */

/**myGyrAcc_StructInit: INIZIALIZZAZIONE STRUTTURA CON VALORI DI DEFAULT
 * I valori di default (impostati da Emanuele) sono i seguenti:
 *
 * -OUTPUT DATA RATE GIROSCOPIO: 59.5Hz;
 *
 * -SCALA GIROSCOPIO = 245dps;
 *
 * -BANDA: LA BANDA (BITS BW_G[1:0]) Dipende dall'ODR selezionato quando
 * accelerometro e giroscopio sono entrambi abilitati. Avendo scelto ODR pari a
 * 59.5Hz, settando i bit BW_G[1:0] = 00, si ottiene una frequenza di taglio pari a
 * 16Hz. Per maggiori approfondimenti consultare la tabella nel manuale del sensore a pag.40;
 *
 * -OUTPUT DATA RATE ACCELEROMETRO: l'ho impostato in modo da abilitare sia accelerometro che giroscopio
 * (istruzioni seguite passo passo dal manuale), cioè ODR = 000 (Power Down). In questo modo l'accelerometro
 * è abilitato allo stesso ODR del giroscopio;
 *
 * -SCALA ACCELEROMETRO: (+/-)2g;
 *
 * -BANDA ACCELEROMETRO: Bandwidth selection. Default value: 0
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
	      MyGyrAcc_InitStruct->MyGyrOutput_DataRate = GYR_ODR_3;
	 	  MyGyrAcc_InitStruct->MyGyrFull_Scale = GYR_FULLSCALE_245;
	 	  MyGyrAcc_InitStruct->MyGyrBandwith_Sel = BANDWITH_00;
	 	  MyGyrAcc_InitStruct->MyAccOutput_DataRate = ACC_ODR_1;
	 	  MyGyrAcc_InitStruct->MyAccFull_Scale = ACC_FULLSCALE_00;
	 	  MyGyrAcc_InitStruct->MyAcc_Bandwith_Sel = BW_SCAL_ODR_0;
	 	  MyGyrAcc_InitStruct->My_Acc_AntiAliasingBwSel = BW_XL_00;
}

/**myGyrAcc_Init:
 * scrive negli opportuni registri di controllo del chip (in questo caso
 * CTRL_REG1_G, CTRL_REG6_XL), i valori dichiarati nella struttura.
 */
void myGyrAcc_Init(MyGyrAcc_InitTypeDef *MyGyrAcc_InitStruct)
{
	uint8_t maskReg = 0x00;
	uint8_t maskReg1 = 0x00;
	//myI2C_Init();

	maskReg = (uint8_t) (MyGyrAcc_InitStruct->MyGyrOutput_DataRate | MyGyrAcc_InitStruct->MyGyrFull_Scale | \
	                     MyGyrAcc_InitStruct->MyGyrBandwith_Sel);

	maskReg1 = (uint8_t) (MyGyrAcc_InitStruct->MyAccOutput_DataRate | MyGyrAcc_InitStruct->MyAccFull_Scale | \
			              MyGyrAcc_InitStruct->MyAcc_Bandwith_Sel | MyGyrAcc_InitStruct->My_Acc_AntiAliasingBwSel);

	myI2C_WriteReg(CHIP_ADDR, CTRL_REG1_G_ADDR , maskReg);
	myI2C_WriteReg(CHIP_ADDR, CTRL_REG6_XL_ADDR, maskReg1);
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
 *
 */

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
/*
 *
 */
