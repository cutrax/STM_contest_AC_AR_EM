/*
 * Elaborazioni.c
 *
 *  Created on: 18 set 2018
 *      Author: Emanuele
 */

#include "Elaborazioni.h"

complex* rotazione_X(float matrice[d_M][d_M], complex* buf_x, complex* buf_y, complex* buf_z, float* workBuffer_X){

	 complex *workBuf_X_cplx = (complex*) workBuffer_X;


      for(int i=0;i<n_C/2;i++){
    	  workBuf_X_cplx[i] = complex_add(complex_multiply_r_c(matrice[0][0],buf_x[i]),complex_multiply_r_c(matrice[0][1],buf_y[i]));
    	  	   workBuf_X_cplx[i] = complex_add(workBuf_X_cplx[i], complex_multiply_r_c(matrice[0][2], buf_z[i]));

      }

	 return workBuf_X_cplx;
}

complex* rotazione_Y(float matrice[d_M][d_M], complex* buf_x, complex* buf_y, complex* buf_z, float* workBuffer_Y){

	 complex *workBuf_Y_cplx = (complex*) workBuffer_Y;

      for(int i=0; i<n_C/2; i++){
		workBuf_Y_cplx[i] = complex_add(complex_multiply_r_c(matrice[1][0],buf_x[i]),complex_multiply_r_c(matrice[1][1],buf_y[i]));
	    workBuf_Y_cplx[i] = complex_add(workBuf_Y_cplx[i], complex_multiply_r_c(matrice[1][2],buf_z[i]));
      }

	return workBuf_Y_cplx;
}

complex* rotazione_Z(float matrice[d_M][d_M], complex* buf_x, complex* buf_y, complex* buf_z, float* workBuffer_Z){

	complex* workBuf_Z_cplx = (complex*) workBuffer_Z;

	    for(int i=0; i<n_C/2; i++){
		workBuf_Z_cplx[i] = complex_add(complex_multiply_r_c(matrice[2][0],buf_x[i]),complex_multiply_r_c(matrice[2][1],buf_y[i]));
	    workBuf_Z_cplx[i] = complex_add(workBuf_Z_cplx[i], complex_multiply_r_c(matrice[2][2],buf_z[i]));
	    }

	return workBuf_Z_cplx;
}


/*
 * matricediRotazione_Init
 * Inizializzazione matrice di rotazione,
 * cosx = cosb
 * sinx = sinb
 * cosy = cosa
 * siny = sina
 * La matrice contiene già i valori salvati di seno e coseno, così da non calcolarli in continuazione.
 */
void matriceDiRotazione_Init(float matrice[d_M][d_M], float cosx, float sinx, float cosy, float siny){

	/* float matrice[d_M][d_M] = { {cos(psi), 0, sin(psi)},

	                        {sin(theta)*sin(psi), cos(theta), -sin(theta)*cos(psi)},

							{-cos(theta)*sin(psi), sin(theta), cos(theta)*cos(psi)} };
	*/
	matrice[0][0] = cosx; //Elementi prima riga
	matrice[0][1] = 0;
	matrice[0][2] = sinx;

	matrice[1][0] = siny*sinx;          //Elementi seconda riga
	matrice[1][1] = cosy;
	matrice[1][2] = -siny*cosx;

	matrice[2][0] = -cosy*sinx;//Elementi terza riga
	matrice[2][1] = siny;
	matrice[2][2] = cosy*cosx;
}

