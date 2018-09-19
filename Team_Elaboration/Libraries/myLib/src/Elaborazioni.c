/*
 * Elaborazioni.c
 *
 *  Created on: 18 set 2018
 *      Author: Emanuele
 */

#include "Elaborazioni.h"

void rotazione_X(float matrice[d_M][d_M], complex* buf_x, complex* buf_y, complex* buf_z, complex* workBuf_X_cplx){



      for(int i=0;i<n_C/2;i++){
    	  workBuf_X_cplx[i] = complex_add(complex_multiply_r_c(matrice[0][0],buf_x[i]),complex_multiply_r_c(matrice[0][1],buf_y[i]));
    	  	   workBuf_X_cplx[i] = complex_add(workBuf_X_cplx[i], complex_multiply_r_c(matrice[0][2], buf_z[i]));

      }
}

void rotazione_Y(float matrice[d_M][d_M], complex* buf_x, complex* buf_y, complex* buf_z, complex* workBuf_Y_cplx){


      for(int i=0; i<n_C/2; i++){
		workBuf_Y_cplx[i] = complex_add(complex_multiply_r_c(matrice[1][0],buf_x[i]),complex_multiply_r_c(matrice[1][1],buf_y[i]));
	    workBuf_Y_cplx[i] = complex_add(workBuf_Y_cplx[i], complex_multiply_r_c(matrice[1][2],buf_z[i]));
      }

}

void rotazione_Z(float matrice[d_M][d_M], complex* buf_x, complex* buf_y, complex* buf_z, complex* workBuf_Z_cplx){


	    for(int i=0; i<n_C/2; i++){
		workBuf_Z_cplx[i] = complex_add(complex_multiply_r_c(matrice[2][0],buf_x[i]),complex_multiply_r_c(matrice[2][1],buf_y[i]));
	    workBuf_Z_cplx[i] = complex_add(workBuf_Z_cplx[i], complex_multiply_r_c(matrice[2][2],buf_z[i]));
	    }

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
	 buf_z[7].re*= weight_z[5];
	 buf_z[8].re*= weight_z[6];
	 buf_z[9].re*= weight_z[6];
	 buf_z[10].re*= weight_z[7];
	 buf_z[11].re*= weight_z[7];
	 buf_z[12].re*= weight_z[7];
	 buf_z[13].re*= weight_z[8];
	 buf_z[14].re*= weight_z[8];
	 buf_z[15].re*= weight_z[8];


	 for(int i=16; i<=n_C/2; i++)
	         {
	        	 if(i<=19)    {
	        		// printf("weight_z[9] = %d", weight_z[9]);
	        		 buf_z[i].re*=weight_z[9];
	        	 }

	        	 if(i<=24)    {buf_z[i].re*=weight_z[10];}
	        	 if(i<=31)    { buf_z[i].re*=weight_z[11];}
	        	 if(i<=35)    { buf_z[i].re*=weight_z[12];}
	        	 if(i<=48)    { buf_z[i].re*=weight_z[13];}
	        	 if(i<=60)    { buf_z[i].re*=weight_z[14];}
	        	 if(i<=77)    { buf_z[i].re*=weight_z[15];}
	        	 if(i<=96)    { buf_z[i].re*=weight_z[16];}
	        	 if(i<=121)   {buf_z[i].re*=weight_z[17];}
	        	 if(i<=153)   {buf_z[i].re*=weight_z[18];}
	        	 if(i<=193)   {buf_z[i].re*=weight_z[19];}
	        	 if(i<=241)   {buf_z[i].re*=weight_z[20];}
	        	 if(i<=304)   {buf_z[i].re*=weight_z[21];}
	        	 if(i<=386)   {buf_z[i].re*=weight_z[22];}
	        	 if(i<=n_C/2) {buf_z[i].re*=weight_z[23];}
	         }

}

void pesatura_X(complex* buf_x,const u16* weight_x ){


	     buf_x[1].re*= weight_x[0];
		 buf_x[2].re*= weight_x[1];
		 buf_x[3].re*= weight_x[2];
		 buf_x[4].re*= weight_x[3];
		 buf_x[5].re*= weight_x[4];
		 buf_x[6].re*= weight_x[5];
		 buf_x[7].re*= weight_x[5];
		 buf_x[8].re*= weight_x[6];
		 buf_x[9].re*= weight_x[6];
		 buf_x[10].re*= weight_x[7];
		 buf_x[11].re*= weight_x[7];
		 buf_x[12].re*= weight_x[7];
		 buf_x[13].re*= weight_x[8];
		 buf_x[14].re*= weight_x[8];
		 buf_x[15].re*= weight_x[8];

		 for(int i=16; i<=n_C/2; i++)
		         {
		        	 if(i<=19)    {buf_x[i].re*=weight_x[9];}
		        	 if(i<=24)    {buf_x[i].re*=weight_x[10];}
		        	 if(i<=31)    { buf_x[i].re*=weight_x[11];}
		        	 if(i<=35)    { buf_x[i].re*=weight_x[12];}
		        	 if(i<=48)    { buf_x[i].re*=weight_x[13];}
		        	 if(i<=60)    { buf_x[i].re*=weight_x[14];}
		        	 if(i<=77)    { buf_x[i].re*=weight_x[15];}
		        	 if(i<=96)    { buf_x[i].re*=weight_x[16];}
		        	 if(i<=121)   {buf_x[i].re*=weight_x[17];}
		        	 if(i<=153)   {buf_x[i].re*=weight_x[18];}
		        	 if(i<=193)   {buf_x[i].re*=weight_x[19];}
		        	 if(i<=241)   {buf_x[i].re*=weight_x[20];}
		        	 if(i<=304)   {buf_x[i].re*=weight_x[21];}
		        	 if(i<=386)   {buf_x[i].re*=weight_x[22];}
		        	 if(i<=n_C/2) {buf_x[i].re*=weight_x[23];}
		         }


}

void pesatura_Y(complex* buf_y,const u16* weight_y){


	     buf_y[1].re*= weight_y[0];
		 buf_y[2].re*= weight_y[1];
		 buf_y[3].re*= weight_y[2];
		 buf_y[4].re*= weight_y[3];
		 buf_y[5].re*= weight_y[4];
		 buf_y[6].re*= weight_y[5];
		 buf_y[7].re*= weight_y[5];
		 buf_y[8].re*= weight_y[6];
		 buf_y[9].re*= weight_y[6];
		 buf_y[10].re*= weight_y[7];
		 buf_y[11].re*= weight_y[7];
		 buf_y[12].re*= weight_y[7];
		 buf_y[13].re*= weight_y[8];
		 buf_y[14].re*= weight_y[8];
		 buf_y[15].re*= weight_y[8];
         for(int i=16; i<=n_C/2; i++)
         {
        	 if(i<=19)    {buf_y[i].re*=weight_y[9];}
        	 if(i<=24)    {buf_y[i].re*=weight_y[10];}
        	 if(i<=31)    { buf_y[i].re*=weight_y[11];}
        	 if(i<=35)    { buf_y[i].re*=weight_y[12];}
        	 if(i<=48)    { buf_y[i].re*=weight_y[13];}
        	 if(i<=60)    { buf_y[i].re*=weight_y[14];}
        	 if(i<=77)    { buf_y[i].re*=weight_y[15];}
        	 if(i<=96)    { buf_y[i].re*=weight_y[16];}
        	 if(i<=121)   {buf_y[i].re*=weight_y[17];}
        	 if(i<=153)   {buf_y[i].re*=weight_y[18];}
        	 if(i<=193)   {buf_y[i].re*=weight_y[19];}
        	 if(i<=241)   {buf_y[i].re*=weight_y[20];}
        	 if(i<=386)   {buf_y[i].re*=weight_y[22];}
        	 if(i<=n_C/2) {buf_y[i].re*=weight_y[23];}
         }
         /*
		 for(i= 16; i<=19; i++) {buf_y[i].re*=weight_y[9];}
		 for(i=20; i<=24; i++)      {buf_y[i].re*=weight_y[10];}
		 for(i=25; i<=31; i++)      { buf_y[i].re*=weight_y[11];}
		 for(i=32; i<=35; i++)      { buf_y[i].re*=weight_y[12];}
		 for(i=36; i<=48; i++)      { buf_y[i].re*=weight_y[13];}
		 for(i=49; i<=60; i++)      { buf_y[i].re*=weight_y[14];}
		 for(i=61; i<=77; i++)      { buf_y[i].re*=weight_y[15];}
		 for(i=78; i<=96; i++)      { buf_y[i].re*=weight_y[16];}
		 for(i=97; i<=121; i++)     {buf_y[i].re*=weight_y[17];}
		 for(i=122; i<=153; i++)    {buf_y[i].re*=weight_y[18];}
		 for(i=154; i<=193; i++)    {buf_y[i].re*=weight_y[19];}
		 for(i=194; i<=241; i++)    {buf_y[i].re*=weight_y[20];}
		 for(i=242; i<=304; i++)    {buf_y[i].re*=weight_y[21];}
		 for(i=305; i<=386; i++)    {buf_y[i].re*=weight_y[22];}
		 for(i=387; i<=n_C/2; i++)  {buf_y[i].re*=weight_y[23];}
		 */
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

float maxRmsValue(float rmsA, float rmsB, float rmsC){

	//se A è maggiore di B
	if(rmsA >= rmsB){
		//se A è maggiore di C
	   if(rmsA >= rmsC){
		   //Ritorna A
			return rmsA;
		}
	   //Se A è maggiore di B ma minore di C
		else{
			//ritorna C
			return rmsC;
		}
	}
	//Se A non è maggiore di B
	else{
		//Se B è maggiore di C
		if(rmsB >= rmsC){
			//Ritorna B
			return rmsB;
		}
		//Se B è minore di C e minore di A
		else{
			//Ritorna C
			return rmsC;
		}
	}
}

