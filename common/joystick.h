/*
 * joystick.h
 *
 *  Created on: May 12, 2024
 *      Author: user
 */

#ifndef SRC_COMMON_JOYSTICK_H_
#define SRC_COMMON_JOYSTICK_H_



#include "hw_def.h"



void joyInit(void);
int  joyRead(ADC_HandleTypeDef* hadc);


#endif /* SRC_COMMON_JOYSTICK_H_ */
