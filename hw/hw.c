/*
 * hw.c
 *
 *  Created on: May 2, 2024
 *      Author: user
 */


#include "hw.h"



void hwInit(void)
{
	bspInit();

	buttonInit();
	ledInit();
	uartInit();
	tmcInit();
}
