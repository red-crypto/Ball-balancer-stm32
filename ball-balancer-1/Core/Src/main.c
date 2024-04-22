/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "tmc2209.h"
#include "my_touch.h"
//#include "pattern.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
ADC_ChannelConfTypeDef sConfig;
tmc2209_stepper_driver_t stepper_driver;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INTERNAL_PWM_FREQUENCY_23KHZ 0 // Actual frequency is 23.44 kHz
#define INTERNAL_PWM_FREQUENCY_35KHZ 1 // Actual frequency is 35.15 kHz
#define INTERNAL_PWM_FREQUENCY_46KHZ 2 // Actual frequency is 46.51 kHz
#define INTERNAL_PWM_FREQUENCY_58KHZ 3 // Actual frequency is 58.82 kHz
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//PID variables
double kp =0.408, ki = 0.000001, kd =3.2;   //kp =0.4, ki = 0.000001, kd =3.2; for 256,20us  kp =0.408, ki = 0.000002, kd =2.2; for 256,5us                             //PID constants
double error[2] = { 0, 0 }, errorPrev[2], integr[2] = { 0, 0 }, deriv[2] = { 0, 0 };  //PID terms for X and Y directions
float out[2];
static float xx = 0, yy = 0;
int currentPos[3]= { 0 , 0 , 0 }, targetPos[3]= { 0 , 0 , 0 }, direction[3]= { 0 , 0 , 0 },distanceToGo[3] = { 0 , 0 , 0 };
int flag[3]={ 0 , 0 , 0 };
int pos[3]={ 0 , 0 , 0 };
int j=0;
double angToStep = 142.2; //200/360*microsteps , ex)16->8.89 32->17.78 128->71.1 256->142.2
double angOrig = 206.66;
//Calculation Variables
float theta;
float ix=0;
float iy=0;
float Psix;
float Psiy;

float Psiz;

float ux;
float uy;
float uz;
float vx;
float vy;
float vz;

float l1;
float l2;
float b;
float p;
float a1;
float a2;
float a3;

float O7x;
float O7y;

float O4x;
float O5x;
float O6x;
float O4y;
float O5y;
float O6y;
float O4z;
float O5z;
float O6z;
float A1;
float A2;
float A3;
float B1;
float B2;
float B3;
float C1;
float C2;
float C3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void PID(double height,double setPointX, double setPointY);
void SetMoveTo(double hz, double nx, double ny,int timeOut);
void step(int i, int spd, int mst);
float cal_theta(int legg, float O7z, float pax, float pay);
void tmc2209_set_hardware_enable_pin(tmc2209_stepper_driver_t *stepper_driver, uint8_t hardware_enable_pin);

void tmc2209_disable(tmc2209_stepper_driver_t *stepper_driver);
void tmc2209_enable(tmc2209_stepper_driver_t *stepper_driver);
void tmc2209_write(tmc2209_stepper_driver_t *stepper_driver, uint8_t register_address, uint32_t data);
uint32_t tmc2209_read(tmc2209_stepper_driver_t *stepper_driver, uint8_t register_address);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//printf

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
  return ch;
}

//delay_us

void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */



  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	Touch_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

	//initializing tmc2209
	  tmc2209_setup(&stepper_driver, 9600, SERIAL_ADDRESS_0);
	  tmc2209_set_hardware_enable_pin(&stepper_driver, MOT_EN_Pin);
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

	  HAL_TIM_Base_Start(&htim1);
	  HAL_TIM_Base_Start(&htim2);
	  HAL_TIM_Base_Start(&htim3);

	  //Move to initial position
	  SetMoveTo(108, 0,0,1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //PID(108,0,0);

	  //linePattern(-10,60,2000,5);
	  bunnyhop();
	  //squarepattern(2000, 5);
	  //tripattern();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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

/* USER CODE BEGIN 4 */
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

float cal_theta(int leg, float O7z, float pax, float pay){
	   ix=pax;
	   iy=pay;
	   float k1,k2=0;
	   k1=-0.2;// psi angle / width 20 / 155
	   k2=0.133;// psi angle / width 20 / 100
	   //rotate the angle
	   float xxx=0.5*(k2*iy-sqrt(3)*k1*ix);
	   float yyy=0.5*(-k1*ix-sqrt(3)*k2*iy);

	   Psix=xxx*3.14/180;
	   Psiy=-yyy*3.14/180;

	   Psiz=atan2(-sin(Psix)*sin(Psiy),(cos(Psix)+cos(Psiy)));

	   ux=cos(Psiy)*cos(Psiz);
	   uy=cos(Psiz)*sin(Psix)*sin(Psiy)+cos(Psix)*sin(Psiz);
	   uz=sin(Psix)*sin(Psiz)-cos(Psix)*cos(Psiz)*sin(Psiy);

	   vx=-cos(Psiy)*sin(Psiz);
	   vy=cos(Psix)*cos(Psiz) - sin(Psix)*sin(Psiy)*sin(Psiz);
	   vz=sin(Psix)*cos(Psiz) + cos(Psix)*sin(Psiy)*sin(Psiz);

	   l1=44.45; // link1
	   l2=93.2; // link2
	   b=50.8; // base plate length
	   p=79.375; // moving plate length

	   a1=0*3.14/180;
	   a2=120*3.14/180;
	   a3=240*3.14/180;

	   O7x=(p*(ux-vy))/2;
	   O7y=-uy*p;

	   O4x=O7x+p*ux;
	   O5x=O7x-p*ux/2+sqrt(3)*p*vx/2;
	   O6x=O7x-p*ux/2-sqrt(3)*p*vx/2;
	   O4y=O7y+p*uy;
	   O5y=O7y-p*uy/2+sqrt(3)*p*vy/2;
	   O6y=O7y-p*uy/2-sqrt(3)*p*vy/2;
	   O4z=O7z+p*uz;
	   O5z=O7z-p*uz/2+sqrt(3)*p*vz/2;
	   O6z=O7z-p*uz/2-sqrt(3)*p*vz/2;

	  //cos(2x)=cos^2x-sinx;
	  //cos^2x=cos2x+1 /2

	 	  	  //theta for another asw of eqs.
//	  double thetam1=2*atan2((-B1-sqrt(pow(A1,2)+pow(B1,2)-pow(C1,2))),(C1-A1))*180/3.14;
//	  double thetam2=2*atan2((-B2-sqrt(pow(A2,2)+pow(B2,2)-pow(C2,2))),(C2-A2))*180/3.14;
//	  double thetam3=2*atan2((-B3-sqrt(pow(A3,2)+pow(B3,2)-pow(C3,2))),(C3-A3))*180/3.14;

	  switch(leg) {
	  case 0:

		   A1=2*l1*cos(a1)*(b*cos(a1)-O4x);
		   B1=2*l1*O4z*(cos(2*a1)+1)/2;
		   C1=pow(O4x,2)-2*b*O4x*cos(a1)+(cos(2*a1)+1)/2*(pow(b,2)+pow(l1,2)-pow(l2,2)+pow(O4z,2));

		  theta=2*atan2((-B1+sqrt(pow(A1,2)+pow(B1,2)-pow(C1,2))),(C1-A1))*180/3.14;
		  theta=180+theta;
		  break;
	  case 1:
		  A2=2*l1*cos(a2)*(b*cos(a2)-O5x);
		  B2=2*l1*O5z*(cos(2*a2)+1)/2;
		  C2=pow(O5x,2)-2*b*O5x*cos(a2)+(cos(2*a2)+1)/2*(pow(b,2)+pow(l1,2)-pow(l2,2)+pow(O5z,2));
		  theta=2*atan2((-B2+sqrt(pow(A2,2)+pow(B2,2)-pow(C2,2))),(C2-A2))*180/3.14;
		  theta=180+theta;
		  break;
	  case 2:
		  A3=2*l1*cos(a2)*(b*cos(a2)-O6x);
		  B3=2*l1*O6z*(cos(2*a3)+1)/2;
		  C3=pow(O6x,2)-2*b*O6x*cos(a3)+(cos(2*a3)+1)/2*(pow(b,2)+pow(l1,2)-pow(l2,2)+pow(O6z,2));
		  theta=2*atan2((-B3+sqrt(pow(A3,2)+pow(B3,2)-pow(C3,2))),(C3-A3))*180/3.14;
		  theta=180+theta;
		  break;

	  }

	  return theta;


  }

  void step(int i, int spd, int mst){

	  switch(i){
	  case 0:
  			if(direction[i]==0){

				currentPos[i]+=1;

				   HAL_GPIO_WritePin(GPIOB, MOTOR_A_DIR_Pin, 0);
				   // Generate a pulse on the STEP pin
				   HAL_GPIO_WritePin(GPIOB, MOTOR_A_STEP_Pin, 1);
				   delay_us(spd); // 200ns setup time
				   HAL_GPIO_WritePin(GPIOB, MOTOR_A_STEP_Pin, 0);


  			}
  			else{

				currentPos[i]-=1;

				HAL_GPIO_WritePin(GPIOB, MOTOR_A_DIR_Pin, 1);
				   // Generate a pulse on the STEP pin
				HAL_GPIO_WritePin(GPIOB, MOTOR_A_STEP_Pin, 1);
				delay_us(spd);// 200ns setup time
				HAL_GPIO_WritePin(GPIOB, MOTOR_A_STEP_Pin, 0);

  			}
  			break;
	  case 1:
			if(direction[i]==0){

				currentPos[i]+=1;

		  		   HAL_GPIO_WritePin(GPIOA, MOTOR_B_DIR_Pin, 0);
		  		   // Generate a pulse on the STEP pin
		  		   HAL_GPIO_WritePin(GPIOA, MOTOR_B_STEP_Pin, 1);
		  		   delay_us(spd); // 200ns setup time
		  		   HAL_GPIO_WritePin(GPIOA, MOTOR_B_STEP_Pin, 0);


			}
			else{

				currentPos[i]-=1;

	  			   HAL_GPIO_WritePin(GPIOA, MOTOR_B_DIR_Pin, 1);
	  			   // Generate a pulse on the STEP pin
	  			   HAL_GPIO_WritePin(GPIOA, MOTOR_B_STEP_Pin, 1);
	  			   delay_us(spd); // 200ns setup time
	  			   HAL_GPIO_WritePin(GPIOA, MOTOR_B_STEP_Pin, 0);



			}
		  break;
	  case 2:
			if(direction[i]==0){

				currentPos[i]+=1;

		  		   HAL_GPIO_WritePin(GPIOA, MOTOR_C_DIR_Pin, 0);
		  		   // Generate a pulse on the STEP pin
		  		   HAL_GPIO_WritePin(GPIOA, MOTOR_C_STEP_Pin, 1);
		  		   delay_us(spd); // 200ns setup time
		  		   HAL_GPIO_WritePin(GPIOA, MOTOR_C_STEP_Pin, 0);

			}
			else{

				currentPos[i]-=1;

	  			   HAL_GPIO_WritePin(GPIOA, MOTOR_C_DIR_Pin, 1);
	  			   // Generate a pulse on the STEP pin
	  			   HAL_GPIO_WritePin(GPIOA, MOTOR_C_STEP_Pin, 1);
	  			   delay_us(spd); // 200ns setup time
	  			   HAL_GPIO_WritePin(GPIOA, MOTOR_C_STEP_Pin, 0);


			}
		  break;
	  }


  }
  void SetMoveTo(double hz, double nx, double ny,int timeOut){
	  float ahz=(float)hz;
	  float anx=(float)nx;
	  float any=(float)ny;

  		for(int i=0 ; i<3; i++){
  			float pTheta=cal_theta(i, ahz, anx, any);
  			printf("motor shaft angle: %f \r\n",pTheta);
  			pos[i]=round((angOrig - pTheta) * angToStep);

  			//moveTo()
  			targetPos[i] = pos[i];
  		}
  		//run();
  			//runSpeed();
  	  int timeInt=__HAL_TIM_GET_COUNTER(&htim2);//60000->1ms
  			while(__HAL_TIM_GET_COUNTER(&htim2)-timeInt<timeOut){
				//computeNewSpeed()
				distanceToGo[j]=targetPos[j]-currentPos[j];

				if(distanceToGo[j] == 0){
					flag[j]=1;

				}
				else if(distanceToGo[j]>0){
					direction[j]=0;
					flag[j]=0;
				}
				else {
					direction[j]=1;
					flag[j]=0;
				}

  				if(flag[j]!=1){
				step(j,20,256); // default : 20us (datasheet : 20ns)
  				}

				j++;
				if(j>2)j=0;

  			}
  			//reset flags

  			flag[0]=0;
  			flag[1]=0;
  			flag[2]=0;

  }

    void PID(double height,double setPointX, double setPointY){
  	if(TouchRead(&xx, &yy)){
  		//mapping x,y
      if(yy<250)yy=250;
      if(yy>3700)yy=3700;
      if(xx<600)xx=600;
      if(xx>3500)xx=3500;
      xx=(xx-600)/(3500-600)* 100;
      yy=(yy-250)/(3700-250) * 155;
      printf("X: %f Y: %f \r\n",xx-50,yy-77.5);
      for (int i = 0; i < 2; i++) {

      	      errorPrev[i] = error[i];

                                                                      //sets previous error
        error[i] = (i==0)?(xx-50 - setPointX):(yy-77.5 - setPointY);
        	  	  	  	  	  //sets error aka X or Y ball position

        integr[i] += (error[i] + errorPrev[i]);                                                        //calculates the integral of the error (proportional but not equal to the true integral of the error)
        deriv[i] = (error[i] - errorPrev[i]);       //calcuates the derivative of the error (proportional but not equal to the true derivative of the error)
        deriv[i] = isnan(deriv[i]) || isinf(deriv[i]) ? 0 : deriv[i];                                //checks if the derivative is a real number or infinite
        out[i] = kp * error[i] + ki * integr[i] + kd * deriv[i];
        out[i]= (float)out[i];

      }

      SetMoveTo(height, out[0],out[1],20);//height 108

  	}


  }


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
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

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
