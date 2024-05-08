/*
 * ap.c
 *
 *  Created on: May 2, 2024
 *      Author: user
 */

#include "ap.h"
#include "mutex.h"


void threadMain(void const *argument);
void threadPID(void const *argument);

uint8_t gdFlag=0;

void apInit(void)
{

	uartOpen(0, 57600);
	uartOpen(1, 57600);
	//Initialize axis
	uartPrintf(1,"Move to initial position\r\n");
	uartPrintf(1,"------------------------\r\n");
	SetMoveTo(108, 0,0,1000);
	uartPrintf(1,"\r\n");

	cliOpen(0, 57600);
	cliOpenLog(1, 57600);

	uartPrintf(1,"Check the threads status\r\n");
	uartPrintf(1,"------------------------\r\n");



	myMutexHandle = osMutexCreate(NULL);

	  osThreadDef(threadMain, threadMain, _HW_DEF_RTOS_THREAD_PRT_MAIN, 0, _HW_DEF_RTOS_THREAD_MEM_MAIN);

	  if (osThreadCreate(osThread(threadMain), NULL) != NULL)
	  {

		  uartPrintf(1, "ThreadMain \t\t: OK\r\n");
		  uartPrintf(1,"\r\n");
	  }
	  else
	  {
		  uartPrintf(1, "ThreadMain \t\t: fail\r\n");
		  uartPrintf(1,"\r\n");
		  while(1);
	  }

	  osThreadDef(threadPID, threadPID, _HW_DEF_RTOS_THREAD_PRT_PID, 0, _HW_DEF_RTOS_THREAD_MEM_PID);
	  if (osThreadCreate(osThread(threadPID), NULL) != NULL)
	  {

	    uartPrintf(1, "ThreadPID \t\t: OK\r\n");
	    uartPrintf(1,"\r\n");
	   }
	   else
	   {
	    uartPrintf(1, "ThreadPID \t\t: fail\r\n");
	    uartPrintf(1,"\r\n");
	    while(1);
	   }



}
void apMain(void)
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

//		if(osMutexWait(myMutexHandle, portMAX_DELAY) == osOK )
//		{
//			uartPrintf(1, "cliMain MutexWait\n");
//		}

		cliMain();


//		if(osMutexRelease(myMutexHandle) == osOK)
//		{
//			uartPrintf(1, "cliMain MutexRelease\n");
//		}
//        osThreadYield();

}

void threadMain(void const *argument)
{

	UNUSED(argument);

	while(1)
	{
		osMutexWait(myMutexHandle, portMAX_DELAY);
		apMain();
		osMutexRelease(myMutexHandle);

	}

}

void threadPID(void const *argument)
{

	uint32_t pretime;
//	pretime = xTaskGetTickCount();
	UNUSED(argument);

	while(1)
	{
		osMutexWait(myMutexHandle, portMAX_DELAY);
		pretime=mills();
		if(gdFlag == 0)
		{
		  PID(108, 0, 0);
		}
		else if(gdFlag == 1)
		{
			uartPrintf(1, "gdFlag : %d\n", gdFlag);
		    cliPrintf("Move to Home\n");
		    while(mills()-pretime<1000)
		    {
			  PID(108, -10, 0);
		    }
			gdFlag=0;
		}
		else if(gdFlag == 2)
		{
			uartPrintf(1, "gdFlag : %d\n", gdFlag);
		    cliPrintf("Move LEFT\n");
		    while(mills()-pretime<1000)
		    {
		      PID(108, -30, 0);
		    }
			gdFlag=0;
		}
		else if(gdFlag == 3)
		{
			uartPrintf(1, "gdFlag : %d\n", gdFlag);
		    cliPrintf("Move to RIGHT\n");
		    while(mills()-pretime<1000)
		    {
		      PID(108, 20, 0);
		    }
			gdFlag=0;
		}
		else if(gdFlag == 4)
		{

			uartPrintf(1, "gdFlag : %d\n", gdFlag);
		    cliPrintf("BUNNY HOP\n");
		    while(mills()-pretime<1000)
		    {
		      PID(88, -10, 0);
		    }
		    while(mills()-pretime<1000)
		    {
		      PID(108, -10, 0);
		    }
			gdFlag=0;
		}
//		if(xTaskGetTickCount() - pretime > 1000)
//		{
//		uartPrintf(1, "PID is working on...\n");
//		pretime = xTaskGetTickCount();
//		}
		osMutexRelease(myMutexHandle);
		osThreadYield();

	}

}



