/*
 * Elaborazioni.h
 *
 *  Created on: 18 set 2018
 *      Author: Emanuele
 */

#ifndef MYLIB_INC_ELABORAZIONI_H_
#define MYLIB_INC_ELABORAZIONI_H_

#include "fft.h"

#define d_M                      3     //Dimensione matrici (3 righe, 3 colonne)
#define n_C        1024  //Numero campioni acquisiti(conviene per FFT)
#define Tw			1.0756302521 //Fc=952, Tw per 1024 campioni

void matriceDiRotazione_Init(float matrice[d_M][d_M], float cosx, float sinx, float cosy, float siny); //Inizializzazione matrice di rotazione

complex* rotazione_X(float matrice[d_M][d_M], complex* buf_x, complex* buf_y, complex* buf_z, float* workBuffer_X);
complex* rotazione_Y(float matrice[d_M][d_M], complex* buf_x, complex* buf_y, complex* buf_z, float* workBuffer_Y);
complex* rotazione_Z(float matrice[d_M][d_M], complex* buf_x, complex* buf_y, complex* buf_z, float* workBuffer_Z);


#endif /* MYLIB_INC_ELABORAZIONI_H_ */
