/*
 * ap.h
 *
 *  Created on: May 2, 2024
 *      Author: user
 */

#ifndef SRC_AP_AP_H_
#define SRC_AP_AP_H_



#include "hw.h"

extern uint8_t gdFlag;



void apInit(void);
void apMain(void);


void threadMain(void const *argument);
void threadPID(void const *argument);

#endif /* SRC_AP_AP_H_ */
