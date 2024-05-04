/*
 * main.c
 *
 *  Created on: May 2, 2024
 *      Author: user
 */



#include "main.h"



static void threadMain(void const *argument);


int main(void)
{
	hwInit();
	apInit();

	  osThreadDef(threadMain, threadMain, _HW_DEF_RTOS_THREAD_PRT_MAIN, 0, _HW_DEF_RTOS_THREAD_MEM_MAIN);
	  if (osThreadCreate(osThread(threadMain), NULL) != NULL)
	  {
		  uartPrintf(1, "ThreadMain \t\t: OK\r\n");
	  }
	  else
	  {
		  uartPrintf(1, "ThreadMain \t\t: fail\r\n");
		  while(1);
	  }


	  osKernelStart();
	//apMain();


	return 0;
}

static void threadMain(void const *argument)
{
	UNUSED(argument);

	apMain();
}

