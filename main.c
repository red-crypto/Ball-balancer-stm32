/*
 * main.c
 *
 *  Created on: May 2, 2024
 *      Author: user
 */



#include "main.h"



int main(void)
{

	hwInit();
	apInit();

	osKernelStart();
	//apMain();


	return 0;
}


