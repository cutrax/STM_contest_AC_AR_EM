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
#include "stm32f4xx_adc.h"


void myUSART2_Init(void);
void myLED_Init(void);
void mySWITCH_Init(void);
void myADC_Init(void);
void ADC_IRQnHandler(void);

#endif /* MYLIB_INC_MYLIB_H_ */
