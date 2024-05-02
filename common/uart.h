/*
 * uart.h
 *
 *  Created on: May 2, 2024
 *      Author: user
 */

#ifndef SRC_COMMON_UART_H_
#define SRC_COMMON_UART_H_



#include "hw_def.h"


bool uartInit(void);
bool uartOpen(uint8_t ch, uint32_t baud);
uint32_t uartAvailable(uint8_t ch);
uint8_t uartRead(uint8_t ch);
uint32_t uartWrite(uint8_t ch, uint32_t *p_data, uint32_t length);
uint32_t uartPrintf(uint8_t ch, char *fmt, ...);
uint32_t uartGetBaud(uint8_t ch);

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle);
void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle);



#endif /* SRC_COMMON_UART_H_ */
