/*
 * myLib.h
 *
 *  Created on: 07 ago 2018
 *      Author: cicci
 */

#ifndef MYLIB_INC_MYLIB_H_
#define MYLIB_INC_MYLIB_H_

#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include <stdio.h>



//Comunicazione I2C
void myI2C_Init(void); //Inizializza la comunicazione I2C
uint8_t myI2C_ReadReg(uint8_t BaseAddr,uint8_t Reg); //Scrivi un registro I2C
void myI2C_WriteReg(uint8_t BaseAddr, uint8_t Reg, uint8_t Data); //Leggi un registro I2C

//Comunicazione USART2 (USB Virtual COM Port)
void myUSART2_Init(void); //Inizializza la porta seriale di dialogo col pc

void myLED_Init(void); //Inizializza il LED sulla scheda

void mySWITCH_Init(void); //Inizializza il pulsante sulla scheda

void myMEMSBoard_Init(void); //Inizializza la scheda IKS01A1 globalmente

//Sensore accelerometro + giroscopio L6MSDS0
void myAccGyr_Init(void); //Inizializza il sensore
//Da completare

//Sensore magnetometro LIS3MDL
void myMag_Init(void); //Inizializza il sensore
//Da completare

//Sensore di pressione LPS25HB*
void myBar_Init(void); //Inizializza il sensore
float myBar_Get(void); //Leggi la pressione

//Sensore HTS221
void myHumTemp_Init(void); //Inizializza il sensore
float myTemp_Get(void); //Leggi la temperatura
float myHum_Get(void); //Leggi l'umidità relativa

void myDelay_ms(uint32_t del); //Delay approssimativo
#endif /* MYLIB_INC_MYLIB_H_ */
