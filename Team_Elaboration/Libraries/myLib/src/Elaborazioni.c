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
 * Funzioni di pesatura
 * Parametri:
 *
 * buf_z -> workingBuf_Z_cplx;
 * weight_z -> frequencyWeight_Z;
 *
 * buf_x -> workingBuf_X_cplx;
 * weight_x -> frequencyWeight_XY;
 *
 * buf_y -> workingBuf_Y_cplx;
 * weight_y -> frequencyWeight_XY;
 */

void pesatura_Z(complex* buf_z,const u16* weight_z){

	 buf_z[1].re*= weight_z[0];
	 buf_z[2].re*= weight_z[1];
	 buf_z[3].re*= weight_z[2];
	 buf_z[4].re*= weight_z[3];
	 buf_z[5].re*= weight_z[4];
	 buf_z[6].re*= weight_z[5];
	 buf_z[9].re*= weight_z[6];
	 buf_z[11].re*= weight_z[7];
	 buf_z[13].re*= weight_z[8];
	 buf_z[17].re*= weight_z[9];
	 buf_z[22].re*= weight_z[10];
	 buf_z[27].re*= weight_z[11];
	 buf_z[34].re*= weight_z[12];
	 buf_z[43].re*= weight_z[13];
	 buf_z[54].re*= weight_z[14];
	 buf_z[68].re*= weight_z[15];
	 buf_z[86].re*= weight_z[16];
	 buf_z[108].re*= weight_z[17];
	 buf_z[134].re*= weight_z[18];
	 buf_z[172].re*= weight_z[19];
	 buf_z[215].re*= weight_z[20];
	 buf_z[269].re*= weight_z[21];
	 buf_z[339].re*= weight_z[22];
	 buf_z[430].re*= weight_z[23];
}

void pesatura_X(complex* buf_x,const u16* weight_x ){

	     buf_x[1].re*= weight_x[0];
		 buf_x[2].re*= weight_x[1];
		 buf_x[3].re*= weight_x[2];
		 buf_x[4].re*= weight_x[3];
		 buf_x[5].re*= weight_x[4];
		 buf_x[6].re*= weight_x[5];
		 buf_x[9].re*= weight_x[6];
		 buf_x[11].re*= weight_x[7];
		 buf_x[13].re*= weight_x[8];
		 buf_x[17].re*= weight_x[9];
		 buf_x[22].re*= weight_x[10];
		 buf_x[27].re*= weight_x[11];
		 buf_x[34].re*= weight_x[12];
		 buf_x[43].re*= weight_x[13];
		 buf_x[54].re*= weight_x[14];
		 buf_x[68].re*= weight_x[15];
		 buf_x[86].re*= weight_x[16];
		 buf_x[108].re*= weight_x[17];
		 buf_x[134].re*= weight_x[18];
		 buf_x[172].re*= weight_x[19];
		 buf_x[215].re*= weight_x[20];
		 buf_x[269].re*= weight_x[21];
		 buf_x[339].re*= weight_x[22];
		 buf_x[430].re*= weight_x[23];

}

void pesatura_Y(complex* buf_y,const u16* weight_y){

	     buf_y[1].re*= weight_y[0];
		 buf_y[2].re*= weight_y[1];
		 buf_y[3].re*= weight_y[2];
		 buf_y[4].re*= weight_y[3];
		 buf_y[5].re*= weight_y[4];
		 buf_y[6].re*= weight_y[5];
		 buf_y[9].re*= weight_y[6];
		 buf_y[11].re*= weight_y[7];
		 buf_y[13].re*= weight_y[8];
		 buf_y[17].re*= weight_y[9];
		 buf_y[22].re*= weight_y[10];
		 buf_y[27].re*= weight_y[11];
		 buf_y[34].re*= weight_y[12];
		 buf_y[43].re*= weight_y[13];
		 buf_y[54].re*= weight_y[14];
		 buf_y[68].re*= weight_y[15];
		 buf_y[86].re*= weight_y[16];
		 buf_y[108].re*= weight_y[17];
		 buf_y[134].re*= weight_y[18];
		 buf_y[172].re*= weight_y[19];
		 buf_y[215].re*= weight_y[20];
		 buf_y[269].re*= weight_y[21];
		 buf_y[339].re*= weight_y[22];
		 buf_y[430].re*= weight_y[23];
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

