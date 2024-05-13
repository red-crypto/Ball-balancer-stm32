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
	cliInit();

	buttonInit();
	//ledInit();
	uartInit();
	tmcInit();
	Touch_Init();
	joyInit();
}
