/*
 * tim.h
 *
 *  Created on: May 2, 2024
 *      Author: user
 */

#ifndef SRC_COMMON_TIM_H_
#define SRC_COMMON_TIM_H_



#include "hw_def.h"



void timInit(void);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle);
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle);



#endif /* SRC_COMMON_TIM_H_ */
