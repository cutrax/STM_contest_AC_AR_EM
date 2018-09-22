/*
 * myLib.c
 *
 *  Created on: 07 ago 2018
 *      Author: cicci
 *
 *      In questo file delle funzioni ricorrenti da includere in tutti i nostri progetti
 */

#include "myLib.h"
#include "Accelerometro.h"






/*
 * myDelay_ms
 * Blocca il programma per del ms
 */

void myDelay_ms(uint32_t del)
{
	del *= 11947; //Valore empirico con SYSCLK 84 MHz
	while(del--);
}

/*
 * my2decs
 * Restituisce i primi due decimali di un float
 */

uint8_t my2decs(float var)
{
	return (int) ((var - floorf(var)) * 100);
}

/*
 * myInt
 * Restituisce la parte intera sinistra
 */

uint16_t myInt(float var)
{
	return (int) floorf(var);
}

/*
 * FINE FUNZIONI VARIE
 */









