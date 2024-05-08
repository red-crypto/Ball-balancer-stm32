/*
 * hw_def.h
 *
 *  Created on: May 2, 2024
 *      Author: user
 */

#ifndef SRC_HW_HW_DEF_H_
#define SRC_HW_HW_DEF_H_



#include "def.h"
#include "bsp.h"



#define _USE_HW_UART
#define      HW_UART_MAX_CH         2

#define _USE_HW_CLI
#define      HW_CLI_CMD_NAME_MAX    16
#define      HW_CLI_CMD_LIST_MAX    16
#define      HW_CLI_LINE_HIS_MAX    4
#define      HW_CLI_LINE_BUF_MAX    32

#define _USE_HW_RTOS
#define _HW_DEF_RTOS_MEM_SIZE(x)         ((x)/4)

#define _HW_DEF_RTOS_THREAD_PRT_MAIN     osPriorityNormal
#define _HW_DEF_RTOS_THREAD_PRT_PID      osPriorityNormal//osPriorityRealtime

#define _HW_DEF_RTOS_THREAD_MEM_MAIN     _HW_DEF_RTOS_MEM_SIZE( 2*1024)
#define _HW_DEF_RTOS_THREAD_MEM_PID      _HW_DEF_RTOS_MEM_SIZE( 2*1024)


#endif /* SRC_HW_HW_DEF_H_ */
