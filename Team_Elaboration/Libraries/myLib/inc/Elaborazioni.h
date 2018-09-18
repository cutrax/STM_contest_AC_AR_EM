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
#define Tw		   1.0756302521 //Fc=952, Tw per 1024 campioni
#define n_F        24  //Numero frequenze pesate (0 incluso)

void matriceDiRotazione_Init(float matrice[d_M][d_M], float cosx, float sinx, float cosy, float siny); //Inizializzazione matrice di rotazione

void rotazione_X(float matrice[d_M][d_M], complex* buf_x, complex* buf_y, complex* buf_z, complex* workBuf_X_cplx);
void rotazione_Y(float matrice[d_M][d_M], complex* buf_x, complex* buf_y, complex* buf_z, complex* workBuf_Y_cplx);
void rotazione_Z(float matrice[d_M][d_M], complex* buf_x, complex* buf_y, complex* buf_z, complex* workBuf_Z_cplx);

void pesatura_Z(complex* buf_z,const u16* weight_z );
void pesatura_X(complex* buf_x,const u16* weight_x);
void pesatura_Y(complex* buf_y,const u16* weight_y);


#endif /* MYLIB_INC_ELABORAZIONI_H_ */
