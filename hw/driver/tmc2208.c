/*
 * tmc2208.c
 *
 *  Created on: May 2, 2024
 *      Author: user
 */



#include "tmc2208.h"
#include "tmc2209.h"

tmc2209_stepper_driver_t stepper_driver;
UART_HandleTypeDef huart3;


void tmcInit(void)
{
	//tmc2209 init and uart3 init
	  huart3.Instance = USART3;
	  huart3.Init.BaudRate = 9600;
	  huart3.Init.WordLength = UART_WORDLENGTH_8B;
	  huart3.Init.StopBits = UART_STOPBITS_1;
	  huart3.Init.Parity = UART_PARITY_NONE;
	  huart3.Init.Mode = UART_MODE_TX_RX;
	  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	  if (HAL_UART_Init(&huart3) != HAL_OK)
	  {
	    Error_Handler();
	  }


	  tmc2209_setup(&stepper_driver, 9600, SERIAL_ADDRESS_0);
	  //enable_cool_step(&stepper_driver, 1, 0);
	  tmc2209_enable(&stepper_driver);
	  set_micro_steps_per_step(&stepper_driver, 256);
	  set_pwm_frequency(&stepper_driver, INTERNAL_PWM_FREQUENCY_23KHZ);
	  set_stand_still_mode(&stepper_driver, TMC_STRONG_BRAKING);
	  set_all_current_percent_values(&stepper_driver, 70, 50, 0);
	  enable_automatic_current_scaling(&stepper_driver);
	  enable_stealth_chop(&stepper_driver);
	  set_stealth_chop_duration_threshold(&stepper_driver, 9999999);
	  enable_inverse_motor_direction(&stepper_driver);
	  move_using_step_dir_interface(&stepper_driver);

}

void tmc2209_set_hardware_enable_pin(tmc2209_stepper_driver_t *stepper_driver, uint8_t hardware_enable_pin)
{
  stepper_driver->hardware_enable_pin_ = hardware_enable_pin;

  HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, GPIO_PIN_SET);
}

void tmc2209_enable(tmc2209_stepper_driver_t *stepper_driver)
{
  if (stepper_driver->hardware_enable_pin_ >= 0)
  {
    HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, GPIO_PIN_RESET);
  }
  stepper_driver->toff_                = TOFF_DEFAULT;
  stepper_driver->chopper_config_.toff = TOFF_DEFAULT;
  write_stored_chopper_config(stepper_driver);
}

void tmc2209_disable(tmc2209_stepper_driver_t *stepper_driver)
{
  if (stepper_driver->hardware_enable_pin_ >= 0)
  {
    HAL_GPIO_WritePin(MOT_EN_GPIO_Port, MOT_EN_Pin, GPIO_PIN_SET);
  }
  stepper_driver->chopper_config_.toff = TOFF_DISABLE;
  write_stored_chopper_config(stepper_driver);
}

void tmc2209_write(tmc2209_stepper_driver_t *stepper_driver, uint8_t register_address, uint32_t data)
{
  write_read_reply_datagram_t write_datagram;
  write_datagram.bytes            = 0;
  write_datagram.sync             = SYNC;
  write_datagram.serial_address   = stepper_driver->serial_address_;
  write_datagram.register_address = register_address;
  write_datagram.rw               = RW_WRITE;
  write_datagram.data             = reverse_data(stepper_driver, data);
  write_datagram.crc              = calculate_crc_write(stepper_driver, &write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);

  uint8_t datagram_bytes[8];
  for (int i = 0; i < 8; i++)
  {
    datagram_bytes[i] = (write_datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
  }
  HAL_HalfDuplex_EnableTransmitter(&huart3);
  HAL_UART_Transmit(&huart3, datagram_bytes, WRITE_READ_REPLY_DATAGRAM_SIZE, 0XFFFF);
}

uint32_t tmc2209_read(tmc2209_stepper_driver_t *stepper_driver, uint8_t register_address)
{
  read_request_datagram_t read_request_datagram;
  read_request_datagram.bytes            = 0;
  read_request_datagram.sync             = SYNC;
  read_request_datagram.serial_address   = stepper_driver->serial_address_;
  read_request_datagram.register_address = register_address;
  read_request_datagram.rw               = RW_READ;
  read_request_datagram.crc              = calculate_crc_read(stepper_driver, &read_request_datagram, READ_REQUEST_DATAGRAM_SIZE);

  uint8_t datagram_bytes[WRITE_READ_REPLY_DATAGRAM_SIZE];
  for (int i = 0; i < READ_REQUEST_DATAGRAM_SIZE; ++i)
  {
    datagram_bytes[i] = (read_request_datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
  }
  HAL_HalfDuplex_EnableTransmitter(&huart3);
  HAL_UART_Transmit(&huart3, datagram_bytes, READ_REQUEST_DATAGRAM_SIZE, 0XFFFF);

  uint8_t                     byte       = 0;
  uint8_t                     byte_count = 0;
  write_read_reply_datagram_t read_reply_datagram;
  read_reply_datagram.bytes = 0;

  HAL_HalfDuplex_EnableReceiver(&huart3);
  for (uint8_t i = 0; i < WRITE_READ_REPLY_DATAGRAM_SIZE; ++i)
  {
    HAL_UART_Receive(&huart3, (uint8_t *)&byte, 1, 100);
    datagram_bytes[i] = byte;
  }
  for (uint8_t i = 0; i < WRITE_READ_REPLY_DATAGRAM_SIZE; ++i)
  {
    read_reply_datagram.bytes |= ((uint64_t)datagram_bytes[i] << (byte_count++ * BITS_PER_BYTE));
  }
  uint32_t reversed_data = reverse_data(stepper_driver, read_reply_datagram.data);
  uint8_t  crc           = calculate_crc_write(stepper_driver, &read_reply_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);
  if (crc != read_reply_datagram.crc)
  {
  }
  return reversed_data;
}
