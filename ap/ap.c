/*
 * ap.c
 *
 *  Created on: May 2, 2024
 *      Author: user
 */

#include "ap.h"




void apInit(void)
{

	uartOpen(0, 57600);
	uartOpen(1, 57600);
	SetMoveTo(108, 0,0,1000);

	cliOpen(0, 57600);
	cliOpenLog(1, 57600);

}
void apMain(void)
{
	while(1)
	{
//		ledToggle();
//		delay(500);

		//uartPrintf(0, "test 0\n");
		//uartPrintf(1, "test 1\n");
//		if(uartAvailable(1) > 0)
//		{
//			uint8_t rx_data;
//			rx_data = uartRead(1);
//
//			uartPrintf(1, "Rx : 0x%X\n", rx_data);
//		}
		//freeRTOS이용해서 task1:PID task2:cliMain
		cliMain();
		PID(108, 0, 0);


	}
}
