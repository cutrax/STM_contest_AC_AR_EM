/*
******************************************************************************
File:     main.c
Info:     Generated by Atollic TrueSTUDIO(R) 9.0.1   2018-08-05

The MIT License (MIT)
Copyright (c) 2018 STMicroelectronics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

******************************************************************************
*/



/* Includes */
#include "stm32f4xx.h"
#include <stdio.h>
#include <math.h>
#include "myLib.h"
/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/

int main(void)
{

	myUSART2_Init();
	myMEMSBoard_Init();


  /* Infinite loop */
	while (1)
	{
		if(myBar_newData())
		{
			printf("Pressione: %d.%d hPa\r\n",myInt(myBar_Get()), my2decs(myBar_Get()));
		}

		if(myHumTemp_newData())
		{
			printf("Temperatura: %d.%d *C; Umidita': %d.%d \r\n\n",myInt(myHumTemp_Temp_Get()), my2decs(myHumTemp_Temp_Get()),myInt(myHumTemp_Hum_Get()), my2decs(myHumTemp_Hum_Get()));
		}
		
		if(myGyrAcc_newData())
		{

			printf("Accelerazione lungo Z: %d.%d m/s^2\r\n", myInt(myAcc_Get_Z()), my2decs(myAcc_Get_Z()));
			printf("Accelerazione lungo X: %d.%d m/s^2\r\n", myInt(myAcc_Get_Y()), my2decs(myAcc_Get_Y()));
			printf("Accelerazione lungo Y: %d.%d m/s^2\r\n\n\n", myInt(myAcc_Get_X()), my2decs(myAcc_Get_X()));

			printf("Velocita' angolare lungo Z: %d.%d dps\r\n", myInt(myGyr_Get_Z()), my2decs(myGyr_Get_Z()));
			printf("Velocita' angolare lungo X: %d.%d dps\r\n", myInt(myGyr_Get_Y()), my2decs(myGyr_Get_Y()));
			printf("velocita' angolare lungo Y: %d.%d dps\r\n\n\n", myInt(myGyr_Get_X()), my2decs(myGyr_Get_X()));

		}
		myDelay_ms(1000);

  }
}



