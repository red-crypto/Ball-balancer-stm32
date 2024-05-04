/*
 * ap.c
 *
 *  Created on: May 2, 2024
 *      Author: user
 */

#include "ap.h"



static void threadPID(void const *argument);







void apInit(void)
{

	uartOpen(0, 57600);
	uartOpen(1, 57600);
	SetMoveTo(108, 0,0,1000);

	cliOpen(0, 57600);
	cliOpenLog(1, 57600);


	 osThreadDef(threadPID, threadPID, _HW_DEF_RTOS_THREAD_PRT_PID, 0, _HW_DEF_RTOS_THREAD_MEM_PID);
	 if (osThreadCreate(osThread(threadPID), NULL) != NULL)
	 {
	  uartPrintf(1, "ThreadPID \t\t: OK\r\n");
	 }
	 else
	 {
	  uartPrintf(1, "ThreadPID \t\t: fail\r\n");
	  while(1);
	 }

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

		//PID(108, 0, 0);
		//osDelay(1) or .. osThreadYield
		cliMain();
		delay(1);

	}
}


static void threadPID(void const *argument)
{
	uint32_t pretime;
	pretime = mills();
	UNUSED(argument);

	while(1)
	{
		PID(108, 0, 0);

		if(mills() - pretime > 5000)
		{
		uartPrintf(1, "PID is working on...\r\n");
		pretime = mills();
		}
	}
}
