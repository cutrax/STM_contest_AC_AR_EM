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
#include "stm32f4xx_iwdg.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Accelerometro.h"

//Comunicazione I2C
void myI2C_Init(void); //Inizializza la comunicazione I2C
uint8_t myI2C_ReadReg(uint8_t BaseAddr,uint8_t Reg); //Scrivi un registro I2C
void myI2C_WriteReg(uint8_t BaseAddr, uint8_t Reg, uint8_t Data); //Leggi un registro I2C
void myI2C_MultipleReadReg(uint8_t BaseAddr, uint8_t Reg, uint8_t *buf, uint8_t cnt, uint8_t autoInc);
void myI2C_MultipleWriteReg(uint8_t BaseAddr, uint8_t Reg, uint8_t *buf, uint8_t cnt, uint8_t autoInc);

//Watchdog
void myWatchDog_Init(void);

//Comunicazione USART2 (USB Virtual COM Port)
void myUSART2_Init(void); //Inizializza la porta seriale di dialogo col pc

void myLED_Init(void); //Inizializza il LED sulla scheda

void mySWITCH_Init(void); //Inizializza il pulsante sulla scheda

void myMEMSBoard_Init(void); //Inizializza la scheda IKS01A1 globalmente

//Sensore magnetometro LIS3MDL
void myMag_Init(void); //Inizializza il sensore
//Da completare

//Sensore di pressione LPS25HB*
void myBar_Init(void); //Inizializza il sensore
float myBar_Get(void); //Leggi la pressione
uint8_t myBar_newData(void); //SET se vi sono nuovi dati non ancora letti

//Sensore HTS221
void myHumTemp_Init(void); //Inizializza il sensore
float myHumTemp_Temp_Get(void); //Leggi la temperatura
float myHumTemp_Hum_Get(void); //Leggi l'umidità relativa
uint8_t myHumTemp_newData(void); //SET se vi sono nuovi dati non ancora letti

void myDelay_ms(uint32_t del); //Delay approssimativo
uint16_t myInt(float var); //Parte intera sinistra
uint8_t my2decs(float var); //Due decimali

/*********************************************************************************
 *****************************ELABORAZIONE DEI SEGNALI****************************
 *********************************************************************************
 */

#define d_M                      3     //Dimensione matrici (3 righe, 3 colonne)
#define n_C        1024  //Numero campioni acquisiti(conviene per FFT)
#define Tw			1.0756302521 //Fc=952, Tw per 1024 campioni
//Fattori il cui prodotto è 952
#define n1_C       119
#define n2_C       8

void matriceDiRotazione_Init(float theta, float psi); //Inizializzazione matrice di rotazione
void debugMatrice(float matrix[d_M][d_M]);


#endif /* MYLIB_INC_MYLIB_H_ */
