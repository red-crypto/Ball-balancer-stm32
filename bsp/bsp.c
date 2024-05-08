/*
 * bsp.c
 *
 *  Created on: May 2, 2024
 *      Author: user
 */


#include "bsp.h"
#include "tim.h"



extern TIM_HandleTypeDef htim1;


void SystemClock_Config(void);



void bspInit(void)
{
	  HAL_Init();
	  SystemClock_Config();

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  timInit();
	  HAL_TIM_Base_Start(&htim1);

      GPIO_InitTypeDef GPIO_InitStruct = {0};

      /*Configure GPIO pin Output Level */
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_2|MOTOR_B_DIR_Pin|MOTOR_B_STEP_Pin, GPIO_PIN_RESET);

      /*Configure GPIO pin Output Level */
      HAL_GPIO_WritePin(GPIOA, MOT_EN_Pin|LD2_Pin|MOTOR_C_STEP_Pin|MOTOR_C_DIR_Pin, GPIO_PIN_RESET);

      /*Configure GPIO pin Output Level */
      HAL_GPIO_WritePin(GPIOB, MOTOR_A_DIR_Pin|MOTOR_A_STEP_Pin, GPIO_PIN_RESET);

      /*Configure GPIO pin : PtPin */
      GPIO_InitStruct.Pin = B1_Pin;
      GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

      /*Configure GPIO pins : PC1 PC2 PCPin PCPin */
      GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|MOTOR_B_DIR_Pin|MOTOR_B_STEP_Pin;
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

      /*Configure GPIO pins : PAPin PAPin PAPin PAPin */
      GPIO_InitStruct.Pin = MOT_EN_Pin|LD2_Pin|MOTOR_C_STEP_Pin|MOTOR_C_DIR_Pin;
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

      /*Configure GPIO pins : PBPin PBPin */
      GPIO_InitStruct.Pin = MOTOR_A_DIR_Pin|MOTOR_A_STEP_Pin;
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

      /* EXTI interrupt init*/
      HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
      HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

void delay(uint32_t ms)
{

#ifdef _USE_HW_RTOS
	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	{
		osDelay(ms);
	}
	else
	{
		HAL_Delay(ms);
	}
#else
	HAL_Delay(ms);
#endif
}

void delay_us(uint32_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

uint32_t mills(void)
{
	return HAL_GetTick();
}




void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
