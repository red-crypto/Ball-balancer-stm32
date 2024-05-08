/*
 * bsp.h
 *
 *  Created on: May 2, 2024
 *      Author: user
 */

#ifndef SRC_BSP_BSP_H_
#define SRC_BSP_BSP_H_



#include "def.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"




void bspInit(void);

void     delay(uint32_t ms);
void     delay_us(uint32_t us);
uint32_t mills(void);



void Error_Handler(void);

//default setting defines
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define MOT_EN_Pin GPIO_PIN_4
#define MOT_EN_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define MOTOR_C_STEP_Pin GPIO_PIN_6
#define MOTOR_C_STEP_GPIO_Port GPIOA
#define MOTOR_C_DIR_Pin GPIO_PIN_7
#define MOTOR_C_DIR_GPIO_Port GPIOA
#define MOTOR_B_DIR_Pin GPIO_PIN_8
#define MOTOR_B_DIR_GPIO_Port GPIOC
#define MOTOR_B_STEP_Pin GPIO_PIN_9
#define MOTOR_B_STEP_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define MOTOR_A_DIR_Pin GPIO_PIN_5
#define MOTOR_A_DIR_GPIO_Port GPIOB
#define MOTOR_A_STEP_Pin GPIO_PIN_6
#define MOTOR_A_STEP_GPIO_Port GPIOB

#endif /* SRC_BSP_BSP_H_ */
