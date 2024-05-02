/*
 * ap.c
 *
 *  Created on: May 2, 2024
 *      Author: user
 */

#include "ap.h"




void apInit(void)
{
	uartOpen(1, 115200);
	SetMoveTo(108, 0,0,1000);

}
void apMain(void)
{
	while(1)
	{
		ledToggle();
		delay(1000);
		delay_us(1000);
		uartPrintf(1, "test\n");
		//PID(10, 0, 0);
		delay(1000);

	}
}
