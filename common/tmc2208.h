/*
 * tmc2208.h
 *
 *  Created on: May 2, 2024
 *      Author: user
 */

#ifndef SRC_COMMON_TMC2208_H_
#define SRC_COMMON_TMC2208_H_



#include "hw_def.h"
#include "tmc2209.h"



#define INTERNAL_PWM_FREQUENCY_23KHZ 0 // Actual frequency is 23.44 kHz
#define INTERNAL_PWM_FREQUENCY_35KHZ 1 // Actual frequency is 35.15 kHz
#define INTERNAL_PWM_FREQUENCY_46KHZ 2 // Actual frequency is 46.51 kHz
#define INTERNAL_PWM_FREQUENCY_58KHZ 3 // Actual frequency is 58.82 kHz



void tmcInit(void);
void tmc2209_set_hardware_enable_pin(tmc2209_stepper_driver_t *stepper_driver, uint8_t hardware_enable_pin);
void tmc2209_disable(tmc2209_stepper_driver_t *stepper_driver);
void tmc2209_enable(tmc2209_stepper_driver_t *stepper_driver);
void tmc2209_write(tmc2209_stepper_driver_t *stepper_driver, uint8_t register_address, uint32_t data);
uint32_t tmc2209_read(tmc2209_stepper_driver_t *stepper_driver, uint8_t register_address);


#endif /* SRC_COMMON_TMC2208_H_ */
