/*
 * fft.h
 *
 *  Created on: 14 set 2018
 *      Author: Emanuele
 */

#ifndef MYLIB_INC_FFT_H_
#define MYLIB_INC_FFT_H_

#include "myLib.h"

#define PI 3.1415926535897932384626434

typedef struct complex_t{
	float re;
	float im;
} complex;

complex conv_from_polar(float r, float radians);
complex complex_add(complex left, complex right);
complex complex_multiply(complex left, complex right);
complex complex_multiply_r_c(float left, complex right);


complex* DFT_naive(complex* x, int N);
complex* FFT_CooleyTukey(float* x, int N, int N1, int N2);

#endif /* MYLIB_INC_FFT_H_ */
