/*
 * joystick.c
 *
 *  Created on: May 12, 2024
 *      Author: user
 */


#include "joystick.h"
#include "uart.h"
//조이스틱 dma adc로 입력값 받고 joyX,joyY를 mapping해서 return
//기존 my_touch에서 사용하는 dma와 어떻게 분리해서 값을 받을 것인가?

extern ADC_HandleTypeDef hadc2;


int joyX,joyY = 0;



void joyInit(void)
{

	  ADC_ChannelConfTypeDef sConfig = {0};

	  hadc2.Instance = ADC2;
	  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
	  hadc2.Init.ContinuousConvMode = ENABLE;
	  hadc2.Init.DiscontinuousConvMode = DISABLE;
	  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  hadc2.Init.NbrOfConversion = 2;
	  if (HAL_ADC_Init(&hadc2) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /** Configure Regular Channel
	  */
	  sConfig.Channel = ADC_CHANNEL_1;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Configure Regular Channel
	  */
	  sConfig.Channel = ADC_CHANNEL_0;
	  sConfig.Rank = ADC_REGULAR_RANK_2;
	  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

	if(hadc->Instance == ADC2)
	{
			volatile float tempjoyX=HAL_ADC_GetValue(&hadc2);
			joyX = (int)((tempjoyX / 4095.0)*20);
			joyX = joyX - 10;



			  volatile float tempjoyY=HAL_ADC_GetValue(&hadc2);
			joyY = (int)((tempjoyY / 4095.0)*20);
			joyY = joyY - 10;

		if(joyX != 0 || joyY != 0)
		{
		uartPrintf(1, "joystick X:%d  ",joyX);
		uartPrintf(1, "joystick Y:%d\n",joyY);

		}
		HAL_ADC_Start_IT(&hadc2);
	}

}

int joyRead(ADC_HandleTypeDef* hadc)
{
	int ret=0;
//	HAL_ADC_Start(&hadc2);
//	HAL_ADC_PollForConversion(&hadc2, 1000);
//	joyX= HAL_ADC_GetValue(&hadc2);
//			uartPrintf(1, "joystick X:%d  ",joyX);
//	HAL_ADC_Start(&hadc2);
//	HAL_ADC_PollForConversion(&hadc2, 1000);
//	joyY= HAL_ADC_GetValue(&hadc2);
//			uartPrintf(1, "joystick Y:%d\n",joyY);
	HAL_ADC_Start_IT(&hadc2);


	return ret;

}
