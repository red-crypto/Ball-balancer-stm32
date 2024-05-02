/*
 * button.c
 *
 *  Created on: May 2, 2024
 *      Author: user
 */



#include "reset.h"



void buttonInit(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};


	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  /*Configure GPIO pin : PtPin */
	  GPIO_InitStruct.Pin = B1_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	  /* EXTI interrupt init*/
	  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
