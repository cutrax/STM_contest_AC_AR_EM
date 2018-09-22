/*
 * myLib.h
 *
 *  Created on: 07 ago 2018
 *      Author: cicci
 */

#ifndef MYLIB_INC_MYLIB_H_
#define MYLIB_INC_MYLIB_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Accelerometro.h"



void myDelay_ms(uint32_t del); //Delay approssimativo
uint16_t myInt(float var); //Parte intera sinistra
uint8_t my2decs(float var); //Due decimali

#endif /* MYLIB_INC_MYLIB_H_ */
