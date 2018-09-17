/*
 * fft.c
 *
 *  Created on: 14 set 2018
 *      Author: Emanuele
 */

#include "fft.h"

complex conv_from_polar(float r, float radians){
	complex result;
	result.re = r * cos(radians);
	result.im = r * sin(radians);
	return result;
}

complex complex_add(complex left, complex right){
	complex result;
	result.re = left.re + right.re;
	result.im = left.im + right.im;
	return result;
}

complex complex_multiply(complex left, complex right){
	complex result;
	result.re = left.re*right.re - left.im*right.im;
	result.im = left.re*right.im + left.im*right.re;
	return result;
}

complex complex_multiply_r_c(float left, complex right)
{
	complex result;
	result.re = left*right.re;
	result.im = left*right.im;
	return result;
}

//CMSIS FFT


complex* DFT_naive(complex* x, int N) {
    complex* X = (complex*) malloc(sizeof(struct complex_t) * N);
    int k, n;
    for(k = 0; k < N; k++) {
        X[k].re = 0.0;
        X[k].im = 0.0;
        for(n = 0; n < N; n++) {
            X[k] = complex_add(X[k], complex_multiply(x[n], conv_from_polar(1, -2*PI*n*k/N)));
        }
    }

    return X;
}

/** Implements the Cooley-Tukey FFT algorithm.
  *
  * @expects: N1*N2 = N
  */
complex* FFT_CooleyTukey(float* input, int N, int N1, int N2) {
    int k1, k2;
    printf("Calcolo la FFT. . .\r\n");
    /* Allocate columnwise matrix */
    complex** columns = (complex**) malloc(sizeof(struct complex_t*) * N1);
    for(k1 = 0; k1 < N1; k1++) {
        columns[k1] = (complex*) malloc(sizeof(struct complex_t) * N2);
    }

    /* Allocate rowwise matrix */
    complex** rows = (complex**) malloc(sizeof(struct complex_t*) * N2);
    for(k2 = 0; k2 < N2; k2++) {
        rows[k2] = (complex*) malloc(sizeof(struct complex_t) * N1);
    }

    /* Reshape input into N1 columns */
    for (k1 = 0; k1 < N1; k1++) {
        for(k2 = 0; k2 < N2; k2++) {
        	//Qui c'è la mia modificaaaa
        	complex temp = {input[N1*k2 + k1],0};
            columns[k1][k2] = temp;
        }
    }

    /* Compute N1 DFTs of length N2 using naive method */
    for (k1 = 0; k1 < N1; k1++) {
        columns[k1] = DFT_naive(columns[k1], N2);
    }

    /* Multiply by the twiddle factors  ( e^(-2*pi*j/N * k1*k2)) and transpose */
    for(k1 = 0; k1 < N1; k1++) {
        for (k2 = 0; k2 < N2; k2++) {
            rows[k2][k1] = complex_multiply(conv_from_polar(1, -2.0*PI*k1*k2/N), columns[k1][k2]);
        }
    }

    /* Compute N2 DFTs of length N1 using naive method */
    for (k2 = 0; k2 < N2; k2++) {
        rows[k2] = DFT_naive(rows[k2], N1);
    }

    /* Flatten into single output */
    complex* output = (complex*) malloc(sizeof(struct complex_t) * N);
    for(k1 = 0; k1 < N1; k1++) {
        for (k2 = 0; k2 < N2; k2++) {
            output[N2*k1 + k2] = rows[k2][k1];
        }
    }

    /* Free all alocated memory except output and input arrays */
    for(k1 = 0; k1 < N1; k1++) {
        free(columns[k1]);
    }
    for(k2 = 0; k2 < N2; k2++) {
        free(rows[k2]);
    }
    free(columns);
    free(rows);
    printf("Fine calcolo\n\n");
    return output;
}
