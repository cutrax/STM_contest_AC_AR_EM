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
#include <math.h>



//Comunicazione I2C
void myI2C_Init(void); //Inizializza la comunicazione I2C
uint8_t myI2C_ReadReg(uint8_t BaseAddr,uint8_t Reg); //Scrivi un registro I2C
void myI2C_WriteReg(uint8_t BaseAddr, uint8_t Reg, uint8_t Data); //Leggi un registro I2C
void myI2C_MultipleReadReg(uint8_t BaseAddr, uint8_t Reg, uint8_t *buf, uint8_t cnt, uint8_t autoInc);
void myI2C_MultipleWriteReg(uint8_t BaseAddr, uint8_t Reg, uint8_t *buf, uint8_t cnt, uint8_t autoInc);

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

/*********************************************************************************
 ******************************ACCELEROMETRO E GIROSCOPIO*************************
 ***************************************LSM6DS0***********************************
 *********************************************************************************
 */

//ACCELERAZIONE DI GRAVITA' (in m/s^2)
#define GRAVITY_ACC                                 (9.8)

/**FATTORE DI MOLTIPLICAZIONE PER OTTENERE L'ACCELERAZIONE IN G
 *
 */
#define LINEAR_ACC_SENSE0                           (0.061)//Full Scale= (+/-)2g
#define LINEAR_ACC_SENSE1                           (0.122)//Full Scale= (+/-)4g
#define LINEAR_ACC_SENSE2                           (0.244)//Full Scale= (+/-)8g
#define LINEAR_ACC_SENSE3                           (0.732)//Full Scale= (+/-)16g

/**FATTORE DI MOLTIPLICAZIONE PER OTTENERE LA VELOCITA' ANGOLARE IN DPS
 *
 */
#define ANGULAR_RATE_SENSE0                        (8.75)//Full Scale= (+/-)245dps
#define ANGULAR_RATE_SENSE1                         (17.50)//Full Scale= (+/-)500dps
#define ANGULAR_RATE_SENSE2                         (70)//Full Scale= 8+/-)2000dps

/********************************************************************************
 ********************************DEFINIZIONE STRUTTURE***************************
 ********************************************************************************
 */

typedef struct
{
	uint8_t MyGyrOutput_DataRate;

	uint8_t MyAccOutput_DataRate;

	uint8_t MyGyrFull_Scale;

	uint8_t MyAccFull_Scale;

	uint8_t MyGyrBandwith_Sel;

	uint8_t MyAcc_Bandwith_Sel;

	uint8_t My_Acc_AntiAliasingBwSel;

}MyGyrAcc_InitTypeDef;

/********************************************************************************
 *********************************MAPPA DEI REGISTRI*****************************
 ********************************************************************************
 */

//INDIRIZZO CHIP LSM6DS0
#define CHIP_ADDR                                   0xD6

//REGISTRI DI CONTROLLO

#define CTRL_REG1_G_ADDR                            0x10
#define CTRL_REG2_G_ADDR                            0x11
#define CTRL_REG3_G_ADDR                            0x12
#define CTRL_REG4_ADDR                              0x1E
#define CTRL_REG5_XL_ADDR                           0x1F
#define CTRL_REG6_XL_ADDR                           0x20
#define CTRL_REG7_XL_ADDR                           0x21
#define CTRL_REG8_ADDR                              0x22
#define CTRL_REG9_ADDR                              0x23
#define CTRL_REG10_ADDR                             0x24

//FINE REGISTRI DI CONTROLLO

/*
 * ACT_THS: ACTIVITY THRESHOLD REGISTER
 * VEDI MANUALE PAG.35
 */

#define ACT_THS_ADDR                                0x04

/*
 * ACT_DUR: INACTIVITY DURATION REGISTER
 * VEDI MANUALE PAG.35
 */

#define ACT_DUR_ADDR                                0x05

/*
 * INT_GEN_CFG_XL: LINEAR ACCELERATION SENSOR INTERRUPT
 * CONFIGURATION REGISTER
 * VEDI MANUALE PAG.35
 */

#define INT_GEN_CFG_XL_ADDR                         0x06

/*
 * INT_GEN_THS_X_XL: LINEAR ACCELERATION SENSOR INTERRUPT
 * THRESHOLD REGISTER
 * VEDI MANUALE PAG.36
 */

#define INT_GEN_THS_X_XL_ADDR                       0x07

/*
 * INT_GEN_THS_Y_XL: LINEAR ACCELERATION SENSOR INTERRUPT
 * THRESHOLD REGISTER
 * VEDI MANUALE PAG.37
 */

#define INT_GEN_THS_Y_XL_ADDR                       0x08

/*
 * INT_GEN_THS_Z_XL: LINEAR ACCELERATION SENSOR INTERRUPT
 * THRESHOLD REGISTER
 * VEDI MANUALE PAG.37
 */

#define INT_GEN_THS_Z_XL_ADDR                       0x09

/*
 * INT_GEN_DUR_XL: LINEAR ACCELERATION SENSOR INTERRUPT
 * DURATION REGISTER
 * VEDI MANUALE PAG.37
 */

#define INT_GEN_DUR_XL_ADDR                         0x0A

/*
 * REFERENCE_G: ANGULAR RATE SENSOR REFERENCE VALUE
 * REGISTER FOR DIGITAL HIGH-PASS FILTER (R/W)
 * VEDI MANUALE PAG.37
 */

#define REFERENCE_G_ADDR                            0x0B

/*
 * INT_CTRL: INT PIN CONTROL REGISTER
 * VEDI MANUALE PAG.38
 */

#define INT_CTRL_ADDR                               0x0C

/*
 * WHO_AM_I: WHO_AM_I REGISTER
 */

#define WHO_AM_I_ADDR                               0x0F

/*
 * ORIENT_CFG_G: ANGULAR RATE SENSOR SIGN AND ORIENTATION
 * REGISTER
 * VEDI MANUALE PAG.42
 */

#define ORIENT_CFG_G_ADDR                           0x13

/*
 * INT_GEN_SRC_G: ANGULAR RATE SENSOR INTERRUPT SOURCE
 * REGISTER
 * VEDI MANUALE PAG.43
 */

#define INT_GEN_SRC_G_ADDR                          0x14

/*
 * OUT_TEMP_L, OUT_TEMP_H: TEMPERATURE DATA OUTPUT
 * REGISTER. L AND H REGISTER TOGETHER EXPRESS A
 * 16-BIT WORD IN TWO'S COMPLEMENT RIGHT-JUSTIFIED
 * VEDI MANUALE PAG.43
 */

#define OUT_TEMP_L_ADDR                             0x15
#define OUT_TEMP_H_ADDR                             0x16

/*
 * STATUS_REG: STATUS REGISTER
 * VEDI MANUALE PAG.44
 */

#define STATUS_REG_ADDR                             0x17

/*
 * OUT_X_G, OUT_Y_G, OUT_Z_G: ANGULAR RATE SENSOR
 * PITCH, ROLL, AND YAW AXIS ANGULAR RATE OUTPUT
 * REGISTER. THE VALUE IS EXPRESSED AS A 16-BIT WORD
 * IN TWO'S COMPLEMENT.
 * VEDI MANUALE PAG.44
 */

#define OUT_X_G_L_ADDR                              0x18
#define OUT_X_G_H_ADDR                              0x19

#define OUT_Y_G_L_ADDR                              0x1A
#define OUT_Y_G_H_ADDR                              0x1B

#define OUT_Z_G_L_ADDR                              0x1C
#define OUT_Z_G_H_ADDR                              0x1D

/*
 * INT_GEN_SRC_XL: LINEAR ACCELERATION SENSOR INTERRUPT
 * SOURCE REGISTER
 * VEDI MANUALE PAG.49
 */

#define INT_GEN_SRC_XL_ADDR                         0x26

/*
 * STATUS_REG1: STATUS REGISTER
 * VEDI MANUALE PAG.50
 */

#define STATUS_REG1_ADDR                            0x27

/*
 * OUT_X_XL, OUT_Y_XL, OUT_Z_XL: LINEAR ACCELERATION
 * SENSOR X,Y, AND Z AXIS OUTPUT REGISTER. THE VALUE
 * IS EXPRESSED AS A 16-BIT WORD IN TWO'S COMPLEMENT.
 * VEDI MANUALE PAG.50
 */

#define OUT_X_XL_L_ADDR                             0x28
#define OUT_X_XL_H_ADDR                             0x29

#define OUT_Y_XL_L_ADDR                             0x2A
#define OUT_Y_XL_H_ADDR                             0x2B

#define OUT_Z_XL_L_ADDR                             0x2C
#define OUT_Z_XL_H_ADDR                             0x2D

/*
 * FIFO_CTRL: FIFO CONTROL REGISTER
 * VEDI MANUALE PAG.51
 */

#define FIFO_CTRL_ADDR                              0x2E

/*
 * FIFO_SRC: FIFO STATUS CONTROL REGISTER
 * VEDI MANUALE PAG.51
 */

#define FIFO_SRC_ADDR                               0x2F

/*
 * INT_GEN_CFG_G: ANGULAR RATE SENSOR INTERRUPT
 * GENERATOR CONFIGURATION REGISTER
 * VEDI MANUALE PAG.52
 */

#define INT_GEN_CFG_G_ADDR                          0x30

/*
 * INT_GEN_THS_X_G, INT_GEN_THS_Y_G, INT_GEN_THS_Z_G:
 * ANGULAR RATE SENSOR INTERRUPT THRESHOLD REGISTERS.
 * THE VALUE IS EXPRESSED AS A 15-BIT WORD IN TWO'S COMPLEMENT.
 * VEDI MANUALE PAG.53
 */

#define INT_GEN_THS_X_G_L_ADDR                      0x31
#define INT_GEN_THS_X_G_H_ADDR                      0x32

#define INT_GEN_THS_Y_G_L_ADDR                      0x33
#define INT_GEN_THS_Y_G_H_ADDR                      0x34

#define INT_GEN_THS_Z_G_L_ADDR                      0x35
#define INT_GEN_THS_Z_G_H_ADDR                      0x36

/*
 * INT_GEN_DUR_G: ANGULAR RATE SENSOR INTERRUPT
 * GENERATOR DURATION REGISTER.
 * VEDI MANUALE PAG.54
 */

#define INT_GEN_DUR_G_ADDR                          0x37

/*
 * FINE MAPPA DEI REGISTRI
 */

/********************************************************************************
 * *******************************GIROSCOPIO*************************************
 * ******************************************************************************
 */

/**Selezione ODR
 *
 */
#define GYR_ODR_1                                       ((uint8_t)0x00)//Power Down
#define GYR_ODR_2                                       ((uint8_t)0x10)//14.9Hz
#define GYR_ODR_3                                       ((uint8_t)0x40)//59.5Hz
#define GYR_ODR_4                                       ((uint8_t)0x60)//119Hz
#define GYR_ODR_5                                       ((uint8_t)0x80)//238Hz
#define GYR_ODR_6                                       ((uint8_t)0xA0)//476Hz
#define GYR_ODR_7                                       ((uint8_t)0xC0)//952Hz
#define GYR_ODR_8                                       ((uint8_t)0xE0)//n.a

/*
 * FINE SELEZIONE ODR
 */

/**Selezione scala giroscopio
 *
 */
#define GYR_FULLSCALE_245                                ((uint8_t)0x00)
#define GYR_FULLSCALE_500                                ((uint8_t)0x01)
#define GYR_FULLSCALE_NA                                 ((uint8_t)0x10)//Not Available
#define GYR_FULLSCALE_2000                               ((uint8_t)0x11)

/*
 * Fine selezione scala giroscopio
 */

/**Selezione Power Mode
 *
 */
#define LOW_POWERMODE_DISABLE                         ((uint8_t)0x00)
#define LOW_POWERMODE_ENABLE                          ((uint8_t)0x80)

/*
 * Fine selzione Power Mode
 */

/**Selezione banda (valore di default:00)
 *
 */
#define BANDWITH_00                                   ((uint8_t)0x00)
#define BANDWITH_01                                   ((uint8_t)0x01)
#define BANDWITH_10                                   ((uint8_t)0x10)
#define BANDWITH_11                                   ((uint8_t)0x11)

/*
 * Fine selezione banda
 */

/*
 * Abilitazione assi giroscopio
 */
#define GYR_AXIS_DISABLE                                  ((uint8_t)0x00)
#define GYR_AXIS_X_ENABLE                                 ((uint8_t)0x08)
#define GYR_AXIS_Y_ENABLE                                 ((uint8_t)0x10)
#define GYR_AXIS_XY_ENABLE                                ((uint8_t)0x18)
#define GYR_AXIS_Z_ENABLE                                 ((uint8_t)0x20)
#define GYR_AXIS_ZX_ENABLE                                ((uint8_t)0x28)
#define GYR_AXIS_ZY_ENABLE                                ((uint8_t)0x30)
#define GYR_AXIS_XYZ_ENABLE                               ((uint8_t)0x38)

/*
 * Fine abilitazione assi giroscopio
 */

/*******************************************************************************
 * ********************************ACCELEROMETRO********************************
 * *****************************************************************************
 */

/*
 * Selezione aggiornamento dati nell'OUT REG ed eventualmente
 * nella FIFO
 */
#define DEC_00                                             ((uint8_t)0x00)
#define DEC_01                                             ((uint8_t)0x40)
#define DEC_10                                             ((uint8_t)0x80)
#define DEC_11                                             ((uint8_t)0xA0)
/*
 * Fine
 */

/*Abilitazione assi accelerometro
 *
 */
#define ACC_AXIS_DISABLE                                  ((uint8_t)0x00)
#define ACC_AXIS_X_ENABLE                                 ((uint8_t)0x08)
#define ACC_AXIS_Y_ENABLE                                 ((uint8_t)0x10)
#define ACC_AXIS_XY_ENABLE                                ((uint8_t)0x18)
#define ACC_AXIS_Z_ENABLE                                 ((uint8_t)0x20)
#define ACC_AXIS_ZX_ENABLE                                ((uint8_t)0x28)
#define ACC_AXIS_ZY_ENABLE                                ((uint8_t)0x30)
#define ACC_AXIS_XYZ_ENABLE                               ((uint8_t)0x38)

/*Fine abilitazione assi accelerometro
*
*/

/**ODR Accelerometro
 * Selezionando ACC_ODR_1 (Power Down), e selezionando
 * qualsiasi valore di GYR_ODR_X tranne che 1, abilito sia
 * il giroscopio che l'accelerometro, entrambi allo stesso ODR, quello cioè
 * imposto dal giroscopio (GYR_ODR_X). Spegnendo invece il giroscopio
 * (GYR_ODR_1) E selezionando qualsiasi valore di ACC_ODR_X tranne che 1
 * abilito SOLAMENTE l'accelerometro, con le seguenti frequenze di funzionamento
 */
#define ACC_ODR_1                                       ((uint8_t)0x00)//Power Down
#define ACC_ODR_2                                       ((uint8_t)0x10)//10Hz
#define ACC_ODR_3                                       ((uint8_t)0x40)//50Hz
#define ACC_ODR_4                                       ((uint8_t)0x60)//119Hz
#define ACC_ODR_5                                       ((uint8_t)0x80)//238Hz
#define ACC_ODR_6                                       ((uint8_t)0xA0)//476Hz
#define ACC_ODR_7                                       ((uint8_t)0xC0)//952Hz
#define ACC_ODR_8                                       ((uint8_t)0xE0)//n.a

/*
 * Fine ODR accelerometro
 */

/*
 * Selezione scala accelerometro
 */
#define ACC_FULLSCALE_00                                ((uint8_t)0x00)//(+/-)2g
#define ACC_FULLSCALE_01                                ((uint8_t)0x08)//(+/-)16g
#define ACC_FULLSCALE_10                                ((uint8_t)0x10)//(+/-)4g
#define ACC_FULLSCALE_11                                ((uint8_t)0x18)//(+/-)8g

/*
 * Fine selezione scala accelerometro
 */

/*Banda accelerometro
 * se è 0, la banda è cosi determinata:
 * -BW = 408Hz quando ODR = 952Hz, 50Hz, 10Hz;
 * -BW = 211Hz quando ODR = 476Hz;
 * -BW = 105Hz quando ODR = 238Hz;
 * -BW = 50Hz quando ODR = 119Hz;
 * se è 1, la banda è selezionata in base a BW_XL_X
 */
#define BW_SCAL_ODR_0                                    ((uint8_t)0x00)
#define BW_SCAL_ODR_1                                    ((uint8_t)0x04)

/*
 * Fine banda accelerometro
 */

/*
 * Selezione banda filtro anti-aliasing
 */
#define BW_XL_00                                         ((uint8_t)0x00)//408Hz
#define BW_XL_01                                         ((uint8_t)0x01)//211Hz
#define BW_XL_10                                         ((uint8_t)0x10)//105Hz
#define BW_XL_11                                         ((uint8_t)0x11)//50Hz

/**FUNZIONI**
 *
 */

void myGyrAcc_StructInit(MyGyrAcc_InitTypeDef *MyGyrAcc_InitStruct); //Inizializza struttura
void myGyrAcc_Init(MyGyrAcc_InitTypeDef *MyGyrAcc_InitStruct); //Inizializza i sensori (in questo caso solo l'accelerometro)

float myAcc_Get_X(void); //Legge l'accelerazione lungo X
float myAcc_Get_Y(void); //Legge l'accelerazione lungo Y
float myAcc_Get_Z(void); //Legge l'accelerazione lungo Z

float myGyr_Get_X(void); //Legge la velocità angolare lungo X
float myGyr_Get_Y(void); //Legge la velocità angolare lungo Y
float myGyr_Get_Z(void); //Legge la velocità angolare lungo Z

uint8_t myGyrAcc_newData(void); //SET se vi sono nuovi dati non ancora letti

/*********************************************************************************
 ******************************FINE ACCELEROMETRO E GIROSCOPIO********************
 ***************************************LSM6DS0***********************************
 *********************************************************************************
 */

void myDelay_ms(uint32_t del); //Delay approssimativo
uint16_t myInt(float var); //Parte intera sinistra
uint8_t my2decs(float var); //Due decimali

/*********************************************************************************
 *****************************ELABORAZIONE DEI SEGNALI****************************
 *********************************************************************************
 */
#define N                      3     //Dimensione matrici (3 righe, 3 colonne)
#define n_C        952  //Numero campioni acquisiti se Fc = 952 Hz, lavorando all'interno
                                    //di una finestra di 1 secondo

void matriceDiRotazione_Init(float theta, float psi); //Inizializzazione matrice di rotazione
void debugMatrice(float matrix[N][N]);


#endif /* MYLIB_INC_MYLIB_H_ */
