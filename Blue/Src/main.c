
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "app_bluenrg-ms.h"
#include "stm32f4_nucleo_f401re.h"

/* USER CODE BEGIN Includes */
#include "Accelerometro.h"
#include "fft.h"
#include "Elaborazioni.h"
#include "sensor_service.h"
#include <arm_math.h>
#include <arm_const_structs.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1; //I2C HAL Handle

float dataBuffer0_X[n_C], dataBuffer0_Y[n_C], dataBuffer0_Z[n_C];
float dataBuffer1_X[n_C], dataBuffer1_Y[n_C], dataBuffer1_Z[n_C];
complex fft_X[n_C/2], fft_Y[n_C/2], fft_Z[n_C/2];
float *workBuf_X, *workBuf_Y, *workBuf_Z;
//Beta e alpha angoli di rotazione. Cosb,sinb,cosa,sina, variabili dove salvare i valori calcolati di cos e sin
float beta, alpha, cosb, sinb, cosa, sina;
//Contiene le frequenze a cui corrispondono i singoli campioni
float scalaFrequenze[n_C/2];
extern float *storeBuf_X, *storeBuf_Y, *storeBuf_Z; //Da condividere con la ISR
extern uint16_t cont; //Da condividere con la ISR

arm_rfft_fast_instance_f32 S;  //Struttura di configurazione di RFFT

uint8_t windowCont;
float a8[60];

volatile int connected;

uint32_t savedTick;

float rotMat[3][3];
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	//Inizializzazione dei puntatori ai buffer
	storeBuf_X = dataBuffer0_X;
	storeBuf_Y = dataBuffer0_Y;
	storeBuf_Z = dataBuffer0_Z;

	workBuf_X = dataBuffer1_X;
	workBuf_Y = dataBuffer1_Y;
	workBuf_Z = dataBuffer1_Z;

	//Inizializzazione FFT
	arm_rfft_fast_init_f32(&S,1024);

	//Contatore di riempimento del buffer azzerato
	cont = 0;
	//Contatore di finestre elaborate... e dunque di tempo
	windowCont = 0;

	//Azzero lo stato
	states_t statoCorrente = ATTESA;
	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_X_CUBE_BLE1_Init();
	/* USER CODE BEGIN 2 */
	myAccBoard_Init();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
	 MX_X_CUBE_BLE1_Process();

	 switch(statoCorrente){
		case ATTESA:
		{
			if(cont == n_C){
			   statoCorrente = SWAP;
			   savedTick = HAL_GetTick();
			}
			if(HAL_GetTick() - savedTick > 2000)
			{
				//Spurga il sensore in crash
				myAcc_Handler();
				printf("Sensore sbloccato: %d\r\n",(uint16_t)(HAL_GetTick() - savedTick));
				savedTick = HAL_GetTick();

			}
			break;
		}

		case SWAP:
		{

			//printf("Numero campioni acquisiti: %d\n", cont);
			float *temp = workBuf_X;
			workBuf_X = storeBuf_X;
			storeBuf_X = temp;

			temp = workBuf_Y;
			workBuf_Y = storeBuf_Y;
			storeBuf_Y = temp;

			temp = workBuf_Z;
			workBuf_Z = storeBuf_Z;
			storeBuf_Z = temp;

			statoCorrente = FFT;
			cont = 0;
			BSP_LED_Toggle(LED2);
			break;
		}
		case FFT: {

			//Sezione RFFT (FFT reale)
			//Da un segnale reale produce N/2 campioni complessi nel DF
			//Il prototipo della RFFT intende come buffer di uscita un buffer float
			//organizzato alternando prima la parte reale, e poi quella immaginaria
			//Il casting è obbligatorio, ma nella pratica i dati si troveranno in formato comlesso
		//	printf("FFT\r\n");

			if(windowCont == 60){
				windowCont = 0;

				//Elabora l'a8 totale
				float a8t = 0;
				for(int i=0;i<60;i++)
				{
					a8t += a8[i]*a8[i];
				}
				a8t = sqrtf(a8t);
				printf("a8 per 60 secondi: %f\r\n",a8t);

			}
			else{
				++windowCont;
			}

			arm_rfft_fast_f32(&S,workBuf_X,(float *)fft_X,0);
			arm_rfft_fast_f32(&S,workBuf_Y,(float *)fft_Y,0);
			arm_rfft_fast_f32(&S,workBuf_Z,(float *)fft_Z,0);
		  //  printf("Ho calcolato le FFT. . .\r\n");
		  //  printf("Normalizzo\r\n");
			for(int i=0;i<n_C/2;i++)
			{
				fft_X[i].re /= n_C;
				fft_X[i].im /= n_C;
				fft_Y[i].re /= n_C;
				fft_Y[i].im /= n_C;
				fft_Z[i].re /= n_C;
				fft_Z[i].im /= n_C;
			}

			//Calcolo campioni da inviare all'app tramite BLE
			float fftx1_6=0;float fftx7_12=0;float fftx13_18=0;float fftx19_24=0;float fftx25_30=0;
			float ffty1_6=0;float ffty7_12=0;float ffty13_18=0;float ffty19_24=0;float ffty25_30=0;
			float fftz1_6=0;float fftz7_12=0;float fftz13_18=0;float fftz19_24=0;float fftz25_30=0;


			fft_t i_frequency = {0,0,0,0,0};



			for(int j=1; j<=30; j++)
			{
				if(j<=6) { fftx1_6 += fft_X[j].re*fft_X[j].re + fft_X[j].im*fft_X[j].im;
				           ffty1_6 += fft_Y[j].re*fft_Y[j].re + fft_Y[j].im*fft_Y[j].im;
				           fftz1_6 += fft_Z[j].re*fft_Z[j].re + fft_Z[j].im*fft_Z[j].im;}

				else if(j<=12) { fftx7_12 += fft_X[j].re*fft_X[j].re + fft_X[j].im*fft_X[j].im;
				            ffty7_12 += fft_Y[j].re*fft_Y[j].re + fft_Y[j].im*fft_Y[j].im;
				            fftz7_12 += fft_Z[j].re*fft_Z[j].re + fft_Z[j].im*fft_Z[j].im;}

				else if(j<=18) { fftx13_18 += fft_X[j].re*fft_X[j].re + fft_X[j].im*fft_X[j].im;
				            ffty13_18 += fft_Y[j].re*fft_Y[j].re + fft_Y[j].im*fft_Y[j].im;
				            fftz13_18 += fft_Z[j].re*fft_Z[j].re + fft_Z[j].im*fft_Z[j].im;}

				else if(j<=24) { fftx19_24 += fft_X[j].re*fft_X[j].re + fft_X[j].im*fft_X[j].im;
				            ffty19_24 += fft_Y[j].re*fft_Y[j].re + fft_Y[j].im*fft_Y[j].im;
				            fftz19_24 += fft_Z[j].re*fft_Z[j].re + fft_Z[j].im*fft_Z[j].im;}

				else if(j<=30) { fftx25_30 += fft_X[j].re*fft_X[j].re + fft_X[j].im*fft_X[j].im;
				            ffty25_30 += fft_Y[j].re*fft_Y[j].re + fft_Y[j].im*fft_Y[j].im;
				            fftz25_30 += fft_Z[j].re*fft_Z[j].re + fft_Z[j].im*fft_Z[j].im;}
			}

			fftx1_6 = sqrtf(fftx1_6);
			fftx7_12 = sqrtf(fftx7_12);
			fftx13_18 = sqrtf(fftx13_18);
			fftx19_24 = sqrtf(fftx19_24);
			fftx25_30 = sqrtf(fftx25_30);

			ffty1_6 = sqrtf(ffty1_6);
			ffty7_12 = sqrtf(ffty7_12);
			ffty13_18 = sqrtf(ffty13_18);
			ffty19_24 = sqrtf(ffty19_24);
			ffty25_30 = sqrtf(ffty25_30);

			fftz1_6 = sqrtf(fftz1_6);
			fftz7_12 = sqrtf(fftz7_12);
			fftz13_18 = sqrtf(fftz13_18);
			fftz19_24 = sqrtf(fftz19_24);
			fftz25_30 = sqrtf(fftz25_30);

			//Valori da mandare in pasto al BLE
			i_frequency.FREQUENCY1_6 = sqrtf((fftx1_6*fftx1_6) + (ffty1_6*ffty1_6) + (fftz1_6*fftz1_6));
			i_frequency.FREQUENCY7_12 = sqrtf((fftx7_12*fftx7_12) + (ffty7_12*ffty7_12) + (fftz7_12*fftz7_12));
			i_frequency.FREQUENCY13_18 = sqrtf((fftx13_18*fftx13_18) + (ffty13_18*ffty13_18) + (fftz13_18*fftz13_18));
			i_frequency.FREQUENCY19_24 = sqrtf((fftx19_24*fftx19_24) + (ffty19_24*ffty19_24) + (fftz19_24*fftz19_24));
			i_frequency.FREQUENCY25_30 = sqrtf((fftx25_30*fftx25_30) + (ffty25_30)*(ffty25_30) + (fftz25_30*fftz25_30));



 //			printf("Stampo i valori della DC:\r\n");
 //			printf("X:%f,Y:%f,Z:%f\r\n",fft_X[0].re,fft_Y[0].re,fft_Z[0].re);
		 //   printf("FFT terminata\r\n");
			//Fine FFT

			//Rotazione
		  //  printf("Rotazione\r\n");

			//Calcolo angolo beta
			beta = -atan2f(fft_X[0].re, fft_Z[0].re);
			//Salvo cos e sin di beta
			cosb = cosf(beta);
			 sinb = sinf(beta);

			 //Cacolo angolo alpha
			 alpha = atan2f(fft_Y[0].re, (cosb*fft_Z[0].re)-(sinb*fft_X[0].re));
			 //Salvo cos e sin di alpha
			 cosa = cosf(alpha);
			 sina = sinf(alpha);

			//Inizializzo la matrice di rotazione
			//printf("Matrice\r\n");
			matriceDiRotazione_Init(rotMat, cosb, sinb, cosa, sina);

			//Calcolo il prodotto tra i vettori FFT e la matrice
			//Le destinazioni sono i work_buff, visti stavolta come buffer complex, per risparmiare memoria
			//visto che i campioni nel DT non servono più
			//printf("Prodotto\r\n");


			complex *workingBuf_X_cplx = (complex *) workBuf_X;
			complex *workingBuf_Y_cplx = (complex *) workBuf_Y;
			complex *workingBuf_Z_cplx = (complex *) workBuf_Z;

			rotazione_X(rotMat, fft_X, fft_Y, fft_Z, workingBuf_X_cplx);
			rotazione_Y(rotMat, fft_X, fft_Y, fft_Z, workingBuf_Y_cplx);
			rotazione_Z(rotMat, fft_X, fft_Y, fft_Z, workingBuf_Z_cplx);
			//printf("Fine Rotazione\r\n");

			//printf("X:%f, Y:%f, Z:%f\r\n", workingBuf_X_cplx[0].re, workingBuf_Y_cplx[0].re, workingBuf_Z_cplx[0].re);

			//Azzero la gravità
			workingBuf_Z_cplx[0].re = 0;

			//Scala delle frequenze
			for( int j=0; j<n_C/2; j++){
				scalaFrequenze[j] = j/Tw;
			}


			//Pesature utilizzando le funzioni
			//printf("Inizio pesatura\r\n");

			pesatura_Z(workingBuf_Z_cplx);
			pesatura_X(workingBuf_X_cplx);
			pesatura_Y(workingBuf_Y_cplx);

			//printf("Fine pesatura\r\n");


			//Calcolo degli RMS di ogni asse, a partire dallo spettro
			//Il prodotto scalare di ogni vettore complesso per se stesso e diviso 2(V^2/2 per l'RMS)
			float rmsX=0;
			float rmsY=0;
			float rmsZ=0;

			for(int i=0;i<n_C/2;i++)
			{
				rmsX += workingBuf_X_cplx[i].re*workingBuf_X_cplx[i].re + workingBuf_X_cplx[i].im*workingBuf_X_cplx[i].im;
				rmsY += workingBuf_Y_cplx[i].re*workingBuf_Y_cplx[i].re + workingBuf_Y_cplx[i].im*workingBuf_Y_cplx[i].im;
				rmsZ += workingBuf_Z_cplx[i].re*workingBuf_Z_cplx[i].re + workingBuf_Z_cplx[i].im*workingBuf_Z_cplx[i].im;
			}

			rmsX /= 2;
			rmsY /= 2;
			rmsZ /= 2;

			rmsX = sqrtf(rmsX);
			rmsY = sqrtf(rmsY);
			rmsZ = sqrtf(rmsZ);

			//printf("ax:%f\r\n",rmsX);
			//printf("ay:%f\r\n",rmsY);
			//printf("az:%f\r\n",rmsZ);

			//Moltiplico i valori efficaci su x e y per 1.4
			rmsX*= F;
			rmsY*= F;

			//Calcolo degli a(8)
		   a8[windowCont] = maxRmsValue(rmsX, rmsY, rmsZ)*scala_T;
			//printf("a8[%d] = %f\r\n", windowCont, a8[windowCont]);
			if(connected){
				Acc_Update(a8[windowCont]);
				FFT_Update(&i_frequency);
			}

			//printf("exec_time:%d\r\n",(uint16_t)(HAL_GetTick()- savedTick));
			printf("A8:%f\r\n",a8[windowCont]);
			statoCorrente = ATTESA;
			break;
		}

		}


	}

}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_Pin|RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IRQ_Pin */
  GPIO_InitStruct.Pin = IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Pin RES_Pin */
  GPIO_InitStruct.Pin = CS_Pin|RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ACC_DRDY_Pin */
  GPIO_InitStruct.Pin = ACC_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ACC_DRDY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 2);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 3);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
