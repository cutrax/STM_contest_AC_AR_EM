/*
 * Elaborazioni.c
 *
 *  Created on: 18 set 2018
 *      Author: Emanuele
 */

#include "Elaborazioni.h"
/*
 * TABELLA DELLE FREQUENZE PESATE
 *
 * Vettori costanti usati:
 * frequencyfrequencyWeight_Z[n_F], frequencyfrequencyWeight_XY[n_F];
 * weight_k: pesatura asse z,
 * weight_d: pesatura assi xy.
 *
 * **************************************
 * F[Hz]       Weight_k        Weight_d *
 *                                      *
 * 1           991             1011     *
 * 2           1012            890      *
 * 3           1022            642      *
 * 4           1024            512      *
 * 5           1013            409      *
 * 6           974             323      *
 * 8           891             253      *
 * 10          776             212      *
 * 12.5        647             161      *
 * 16          512             125      *
 * 20          409             100      *
 * 25          325             80       *
 * 31.5        256             63.2     *
 * 40          199             49.4     *
 * 50          156             38.8     *
 * 63          118             29.5     *
 * 80          84.4            21.1     *
 * 100         56.7            14.1     *
 * 125         34.5            8.63     *
 * 160         18.2            4.55     *
 * 200         9.71            2.43     *
 * 250         5.06            1.26     *
 * 315         2.55            0.64     *
 * 400         1.25            0.31     *
 * **************************************
 * **************************************
 */

static const float frequencyWeight_Z[n_F] = {0.991, 1.012, 1.022, 1.024, 1.013, 0.974, 0.891, 0.776,
		                                  0.647, 0.512, 0.409, 0.325, 0.256, 0.199, 0.156, 0.118, 0.0844,
										  0.0567, 0.0345, 0.0182, 0.00971, 0.00506, 0.00255, 0};//0.00125};

static const float frequencyWeight_XY[n_F] = {1.011, 0.890, 0.642, 0.512, 0.409, 0.323, 0.253, 0.212, 0.161,
		                                  0.125, 0.100, 0.080, 0.0632, 0.0494, 0.0388, 0.0295, 0.0211, 0.0141,
										  0.00863, 0.00455, 0.00243, 0.00126, 0.00064, 0};//0.00031};


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
 * frequencyWeight_Z -> frequencyfrequencyWeight_Z;
 *
 * buf_x -> workingBuf_X_cplx;
 * frequencyWeight_XY -> frequencyfrequencyWeight_XYY;
 *
 * buf_y -> workingBuf_Y_cplx;
 * frequencyWeight_XY -> frequencyfrequencyWeight_XYY;
 */

void pesatura_Z(complex* buf_z){


	 buf_z[1] = complex_multiply_r_c(frequencyWeight_Z[0],buf_z[1]);
	 buf_z[2] = complex_multiply_r_c(frequencyWeight_Z[1],buf_z[2]);
	 buf_z[3] = complex_multiply_r_c(frequencyWeight_Z[2],buf_z[3]);
	 buf_z[4] = complex_multiply_r_c(frequencyWeight_Z[3],buf_z[4]);
	 buf_z[5] = complex_multiply_r_c(frequencyWeight_Z[4],buf_z[5]);
	 buf_z[6] = complex_multiply_r_c(frequencyWeight_Z[5],buf_z[6]);
	 buf_z[7] = complex_multiply_r_c(frequencyWeight_Z[5],buf_z[7]);
	 buf_z[8] = complex_multiply_r_c(frequencyWeight_Z[6],buf_z[8]);
	 buf_z[9] = complex_multiply_r_c(frequencyWeight_Z[6],buf_z[9]);
	 buf_z[10] = complex_multiply_r_c(frequencyWeight_Z[7],buf_z[10]);
	 buf_z[11] = complex_multiply_r_c(frequencyWeight_Z[7],buf_z[11]);
	 buf_z[12] = complex_multiply_r_c(frequencyWeight_Z[7],buf_z[12]);
	 buf_z[13] = complex_multiply_r_c(frequencyWeight_Z[8],buf_z[13]);
	 buf_z[14] = complex_multiply_r_c(frequencyWeight_Z[8],buf_z[14]);
	 buf_z[15] = complex_multiply_r_c(frequencyWeight_Z[8],buf_z[15]);


	 for(int i=16; i<n_C/2; i++)
	         {
	        	 if(i<=19)    {
	        		// printf("complex_multiply_r_c(frequencyWeight_Z[9] = %d", complex_multiply_r_c(frequencyWeight_Z[9]);
	        		 buf_z[i] =complex_multiply_r_c(frequencyWeight_Z[9],buf_z[i]);
	        	 }

	        	 else if(i<=24)    {buf_z[i] =complex_multiply_r_c(frequencyWeight_Z[10],buf_z[i]);}
	        	 else if(i<=31)    { buf_z[i] =complex_multiply_r_c(frequencyWeight_Z[11],buf_z[i]);}
	        	 else if(i<=35)    { buf_z[i] =complex_multiply_r_c(frequencyWeight_Z[12],buf_z[i]);}
	        	 else if(i<=48)    { buf_z[i] =complex_multiply_r_c(frequencyWeight_Z[13],buf_z[i]);}
	        	 else if(i<=60)    { buf_z[i] =complex_multiply_r_c(frequencyWeight_Z[14],buf_z[i]);}
	        	 else if(i<=77)    { buf_z[i] =complex_multiply_r_c(frequencyWeight_Z[15],buf_z[i]);}
	        	 else if(i<=96)    { buf_z[i] =complex_multiply_r_c(frequencyWeight_Z[16],buf_z[i]);}
	        	 else if(i<=121)   {buf_z[i] =complex_multiply_r_c(frequencyWeight_Z[17],buf_z[i]);}
	        	 else if(i<=153)   {buf_z[i] =complex_multiply_r_c(frequencyWeight_Z[18],buf_z[i]);}
	        	 else if(i<=193)   {buf_z[i] =complex_multiply_r_c(frequencyWeight_Z[19],buf_z[i]);}
	        	 else if(i<=241)   {buf_z[i] =complex_multiply_r_c(frequencyWeight_Z[20],buf_z[i]);}
	        	 else if(i<=304)   {buf_z[i] =complex_multiply_r_c(frequencyWeight_Z[21],buf_z[i]);}
	        	 else if(i<=386)   {buf_z[i] =complex_multiply_r_c(frequencyWeight_Z[22],buf_z[i]);}
	        	 else if(i<=n_C/2) {buf_z[i] =complex_multiply_r_c(frequencyWeight_Z[23],buf_z[i]);}
	         }

}

void pesatura_Y(complex* buf_y){


	 buf_y[1] = complex_multiply_r_c(frequencyWeight_XY[0],buf_y[1]);
	 buf_y[2] = complex_multiply_r_c(frequencyWeight_XY[1],buf_y[2]);
	 buf_y[3] = complex_multiply_r_c(frequencyWeight_XY[2],buf_y[3]);
	 buf_y[4] = complex_multiply_r_c(frequencyWeight_XY[3],buf_y[4]);
	 buf_y[5] = complex_multiply_r_c(frequencyWeight_XY[4],buf_y[5]);
	 buf_y[6] = complex_multiply_r_c(frequencyWeight_XY[5],buf_y[6]);
	 buf_y[7] = complex_multiply_r_c(frequencyWeight_XY[5],buf_y[7]);
	 buf_y[8] = complex_multiply_r_c(frequencyWeight_XY[6],buf_y[8]);
	 buf_y[9] = complex_multiply_r_c(frequencyWeight_XY[6],buf_y[9]);
	 buf_y[10] = complex_multiply_r_c(frequencyWeight_XY[7],buf_y[10]);
	 buf_y[11] = complex_multiply_r_c(frequencyWeight_XY[7],buf_y[11]);
	 buf_y[12] = complex_multiply_r_c(frequencyWeight_XY[7],buf_y[12]);
	 buf_y[13] = complex_multiply_r_c(frequencyWeight_XY[8],buf_y[13]);
	 buf_y[14] = complex_multiply_r_c(frequencyWeight_XY[8],buf_y[14]);
	 buf_y[15] = complex_multiply_r_c(frequencyWeight_XY[8],buf_y[15]);


	 for(int i=16; i<n_C/2; i++)
	         {
	        	 if(i<=19)    {
	        		// printf("complex_multiply_r_c(frequencyWeight_XY[9] = %d", complex_multiply_r_c(frequencyWeight_XY[9]);
	        		 buf_y[i] =complex_multiply_r_c(frequencyWeight_XY[9],buf_y[i]);
	        	 }

	        	 else if(i<=24)    {buf_y[i] =complex_multiply_r_c(frequencyWeight_XY[10],buf_y[i]);}
	        	 else if(i<=31)    { buf_y[i] =complex_multiply_r_c(frequencyWeight_XY[11],buf_y[i]);}
	        	 else if(i<=35)    { buf_y[i] =complex_multiply_r_c(frequencyWeight_XY[12],buf_y[i]);}
	        	 else if(i<=48)    { buf_y[i] =complex_multiply_r_c(frequencyWeight_XY[13],buf_y[i]);}
	        	 else if(i<=60)    { buf_y[i] =complex_multiply_r_c(frequencyWeight_XY[14],buf_y[i]);}
	        	 else if(i<=77)    { buf_y[i] =complex_multiply_r_c(frequencyWeight_XY[15],buf_y[i]);}
	        	 else if(i<=96)    { buf_y[i] =complex_multiply_r_c(frequencyWeight_XY[16],buf_y[i]);}
	        	 else if(i<=121)   {buf_y[i] =complex_multiply_r_c(frequencyWeight_XY[17],buf_y[i]);}
	        	 else if(i<=153)   {buf_y[i] =complex_multiply_r_c(frequencyWeight_XY[18],buf_y[i]);}
	        	 else if(i<=193)   {buf_y[i] =complex_multiply_r_c(frequencyWeight_XY[19],buf_y[i]);}
	        	 else if(i<=241)   {buf_y[i] =complex_multiply_r_c(frequencyWeight_XY[20],buf_y[i]);}
	        	 else if(i<=304)   {buf_y[i] =complex_multiply_r_c(frequencyWeight_XY[21],buf_y[i]);}
	        	 else if(i<=386)   {buf_y[i] =complex_multiply_r_c(frequencyWeight_XY[22],buf_y[i]);}
	        	 else if(i<=n_C/2) {buf_y[i] =complex_multiply_r_c(frequencyWeight_XY[23],buf_y[i]);}
	         }

}


void pesatura_X(complex* buf_x){


	 buf_x[1] = complex_multiply_r_c(frequencyWeight_XY[0],buf_x[1]);
	 buf_x[2] = complex_multiply_r_c(frequencyWeight_XY[1],buf_x[2]);
	 buf_x[3] = complex_multiply_r_c(frequencyWeight_XY[2],buf_x[3]);
	 buf_x[4] = complex_multiply_r_c(frequencyWeight_XY[3],buf_x[4]);
	 buf_x[5] = complex_multiply_r_c(frequencyWeight_XY[4],buf_x[5]);
	 buf_x[6] = complex_multiply_r_c(frequencyWeight_XY[5],buf_x[6]);
	 buf_x[7] = complex_multiply_r_c(frequencyWeight_XY[5],buf_x[7]);
	 buf_x[8] = complex_multiply_r_c(frequencyWeight_XY[6],buf_x[8]);
	 buf_x[9] = complex_multiply_r_c(frequencyWeight_XY[6],buf_x[9]);
	 buf_x[10] = complex_multiply_r_c(frequencyWeight_XY[7],buf_x[10]);
	 buf_x[11] = complex_multiply_r_c(frequencyWeight_XY[7],buf_x[11]);
	 buf_x[12] = complex_multiply_r_c(frequencyWeight_XY[7],buf_x[12]);
	 buf_x[13] = complex_multiply_r_c(frequencyWeight_XY[8],buf_x[13]);
	 buf_x[14] = complex_multiply_r_c(frequencyWeight_XY[8],buf_x[14]);
	 buf_x[15] = complex_multiply_r_c(frequencyWeight_XY[8],buf_x[15]);


	 for(int i=16; i<n_C/2; i++)
	         {
	        	 if(i<=19)    {
	        		// printf("complex_multiply_r_c(frequencyWeight_XY[9] = %d", complex_multiply_r_c(frequencyWeight_XY[9]);
	        		 buf_x[i] =complex_multiply_r_c(frequencyWeight_XY[9],buf_x[i]);
	        	 }

	        	 else if(i<=24)    {buf_x[i] =complex_multiply_r_c(frequencyWeight_XY[10],buf_x[i]);}
	        	 else if(i<=31)    { buf_x[i] =complex_multiply_r_c(frequencyWeight_XY[11],buf_x[i]);}
	        	 else if(i<=35)    { buf_x[i] =complex_multiply_r_c(frequencyWeight_XY[12],buf_x[i]);}
	        	 else if(i<=48)    { buf_x[i] =complex_multiply_r_c(frequencyWeight_XY[13],buf_x[i]);}
	        	 else if(i<=60)    { buf_x[i] =complex_multiply_r_c(frequencyWeight_XY[14],buf_x[i]);}
	        	 else if(i<=77)    { buf_x[i] =complex_multiply_r_c(frequencyWeight_XY[15],buf_x[i]);}
	        	 else if(i<=96)    { buf_x[i] =complex_multiply_r_c(frequencyWeight_XY[16],buf_x[i]);}
	        	 else if(i<=121)   {buf_x[i] =complex_multiply_r_c(frequencyWeight_XY[17],buf_x[i]);}
	        	 else if(i<=153)   {buf_x[i] =complex_multiply_r_c(frequencyWeight_XY[18],buf_x[i]);}
	        	 else if(i<=193)   {buf_x[i] =complex_multiply_r_c(frequencyWeight_XY[19],buf_x[i]);}
	        	 else if(i<=241)   {buf_x[i] =complex_multiply_r_c(frequencyWeight_XY[20],buf_x[i]);}
	        	 else if(i<=304)   {buf_x[i] =complex_multiply_r_c(frequencyWeight_XY[21],buf_x[i]);}
	        	 else if(i<=386)   {buf_x[i] =complex_multiply_r_c(frequencyWeight_XY[22],buf_x[i]);}
	        	 else if(i<=n_C/2) {buf_x[i] =complex_multiply_r_c(frequencyWeight_XY[23],buf_x[i]);}
	         }

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

