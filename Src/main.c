
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "math.h"
#include <string.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
SD_MPU6050 mpu1;

int8_t count = 0;
uint16_t micros = 0;

int16_t ax, ay, az;
int16_t gx, gy, gz;

int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

float dt;
float accel_angle_x,    accel_angle_y,    accel_angle_z;
float filtered_angle_x, filtered_angle_y, filtered_angle_z;

float baseAcX, baseAcY, baseAcZ;
float baseGyX, baseGyY, baseGyZ;

/* PID값 세팅 */
float roll_target_angle = 0.0;
float roll_angle_in;
float roll_rate_in;
float roll_stabilize_kp = 1.0;
float roll_stabilize_ki = 0;
float roll_rate_kp = 0.6;
float roll_rate_ki = 0.3;
float roll_stabilize_iterm;
float roll_rate_iterm;
float roll_output;

float pitch_target_angle = 0.0;
float pitch_angle_in;
float pitch_rate_in;
float pitch_stabilize_kp = 1.0;
float pitch_stabilize_ki = 0;
float pitch_rate_kp = 0.6;
float pitch_rate_ki = 0.3;
float pitch_stabilize_iterm;
float pitch_rate_iterm;
float pitch_output;

float yaw_target_angle = 0.0;
float yaw_angle_in;
float yaw_rate_in;
float yaw_stabilize_kp = 1;
float yaw_stabilize_ki = 0;
float yaw_rate_kp = 1;
float yaw_rate_ki = 0;
float yaw_stabilize_iterm;
float yaw_rate_iterm;
float yaw_output;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_RTC_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM3){
		if(++count == 7){ //5
			count = 0;
			micros++;
		}
	}

}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void readAccelGyro() {
	SD_MPU6050_ReadAccelerometer(&hi2c1,&mpu1);
	AcX = mpu1.Accelerometer_X;
	AcY = mpu1.Accelerometer_Y;
	AcZ = mpu1.Accelerometer_Z;
	SD_MPU6050_ReadGyroscope(&hi2c1,&mpu1);
	GyX = mpu1.Gyroscope_X;
	GyY = mpu1.Gyroscope_Y;
	GyZ = mpu1.Gyroscope_Z;
}

void calibrate_sensors(void) {
	float sumAcX = 0, sumAcY = 0, sumAcZ = 0;
	float sumGyX = 0, sumGyY = 0, sumGyZ = 0;
	int num_readings = 10;

	// Read and average the raw values from the IMU
	for (int i = 0; i < num_readings; i++) {
	  readAccelGyro();

	  sumAcX += AcX;
	  sumAcY += AcY;
	  sumAcZ += AcZ;
	  sumGyX += GyX;
	  sumGyY += GyY;
	  sumGyZ += GyZ;
	  HAL_Delay(100);
	}

	baseAcX = sumAcX / num_readings;
	baseAcY = sumAcY / num_readings;
	baseAcZ = sumAcZ / num_readings;
	baseGyX = sumGyX / num_readings;
	baseGyY = sumGyY / num_readings;
	baseGyZ = sumGyZ / num_readings;
}

void calcurate_DT(void){
	dt = ((float)micros / (1000000.0 / 16.0));
	micros = 0;
}

void calcAccelYPR(void) {
	float accel_x, accel_y, accel_z;
	float accel_xz, accel_yz;
	const float RADIANS_TO_DEGREES = 180 / 3.14159;

	accel_x = AcX - baseAcX;
	accel_y = AcY - baseAcY;
	accel_z = AcZ + (16384 - baseAcZ);

	accel_yz = sqrt(pow(accel_y, 2) + pow(accel_z, 2));
	accel_angle_y = atan(-accel_x / accel_yz) * RADIANS_TO_DEGREES;

	accel_xz = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
	accel_angle_x = atan( accel_y / accel_xz) * RADIANS_TO_DEGREES;

	accel_angle_z = 0;
}

float gyro_x, gyro_y, gyro_z;

void calcGyroYPR(void) {
	const float GYROXYZ_TO_DEGREES_PER_SEC = 131;

	gyro_x = (GyX - baseGyX) / GYROXYZ_TO_DEGREES_PER_SEC;
	gyro_y = (GyY - baseGyY) / GYROXYZ_TO_DEGREES_PER_SEC;
	gyro_z = (GyZ - baseGyZ) / GYROXYZ_TO_DEGREES_PER_SEC;

	//  gyro_angle_x += gyro_x * dt;
	//  gyro_angle_y += gyro_y * dt;
	//  gyro_angle_z += gyro_z * dt;
}

void calcFilteredYPR(void) {
	const float ALPHA = 0.97;
	float tmp_angle_x, tmp_angle_y, tmp_angle_z;

	tmp_angle_x = filtered_angle_x + gyro_x * dt;
	tmp_angle_y = filtered_angle_y + gyro_y * dt;
	tmp_angle_z = filtered_angle_z + gyro_z * dt;

	/* 간단한 선형 필터를 적용하여 값을 안정화합니다. */
	/* Edu
	filtered_angle_x = ALPHA * tmp_angle_x + (1.0-ALPHA) * accel_angle_x;
	filtered_angle_y = 0; // 값을 지정해야 합니다.
	filtered_angle_z = 0; // 값을 지정해야 합니다.
	*/

	/* Sample */
	filtered_angle_x = ALPHA * tmp_angle_x + (1.0 - ALPHA) * accel_angle_x;
	filtered_angle_y = ALPHA * tmp_angle_y + (1.0 - ALPHA) * accel_angle_y;

//	filtered_angle_x = ALPHA * accel_angle_x + (1.0 - ALPHA) * tmp_angle_x;
//	filtered_angle_y = ALPHA * accel_angle_y + (1.0 - ALPHA) * tmp_angle_y;
	filtered_angle_z = tmp_angle_z; // Accel 값으로는 Yaw 변화 감지가 불가능합니다. 지자기계 등 추가필요.
}

float base_roll_target_angle;
float base_pitch_target_angle;
float base_yaw_target_angle;

extern float roll_target_angle;
extern float pitch_target_angle;
extern float yaw_target_angle;

void initYPR(void) {

	for (int i = 0; i < 10; i++) {
		readAccelGyro();
		calcurate_DT();
		calcAccelYPR();
		calcGyroYPR();
		calcFilteredYPR();

		base_roll_target_angle += filtered_angle_y;
		base_pitch_target_angle += filtered_angle_x;
		base_yaw_target_angle += filtered_angle_z;

		HAL_Delay(100);
	}

	base_roll_target_angle /= 10;
	base_pitch_target_angle /= 10;
	base_yaw_target_angle /= 10;

	roll_target_angle = base_roll_target_angle;
	pitch_target_angle = base_pitch_target_angle;
	yaw_target_angle = base_yaw_target_angle;
}

void dualPID(float target_angle, float angle_in, float rate_in, float stabilize_kp, float stabilize_ki, float rate_kp,
             float rate_ki, float *stabilize_iterm, float *rate_iterm, float *output) {
	float angle_error;
	float desired_rate;
	float rate_error;
	float stabilize_pterm, rate_pterm;

	angle_error = target_angle - angle_in;

	stabilize_pterm  = stabilize_kp * angle_error;
	(*stabilize_iterm) += stabilize_ki * angle_error * dt; // dt 를 계산하여야 합니다.

	desired_rate = stabilize_pterm;

	rate_error = desired_rate - rate_in;

	rate_pterm  = rate_kp * rate_error;
	(*rate_iterm) += rate_ki * rate_error * dt;

	(*output) = rate_pterm + (*rate_iterm) + (*stabilize_iterm);
}

void calcYPRtoDualPID() {
	roll_angle_in = filtered_angle_y;
	roll_rate_in = gyro_y;
	dualPID(  roll_target_angle,
	          roll_angle_in,
	          roll_rate_in,
	          roll_stabilize_kp,
	          roll_stabilize_ki,
	          roll_rate_kp,
	          roll_rate_ki,
	          &roll_stabilize_iterm,
	          &roll_rate_iterm,
	          &roll_output);

	pitch_angle_in = filtered_angle_x;
	pitch_rate_in = gyro_x;
	dualPID(pitch_target_angle,
	        pitch_angle_in,
	        pitch_rate_in,
	        pitch_stabilize_kp,
	        pitch_stabilize_ki,
	        pitch_rate_kp,
	        pitch_rate_ki,
	        &pitch_stabilize_iterm,
	        &pitch_rate_iterm,
	        &pitch_output);

	yaw_angle_in = filtered_angle_z;
	yaw_rate_in = gyro_z;
	dualPID(  yaw_target_angle,
	          yaw_angle_in,
	          yaw_rate_in,
	          yaw_stabilize_kp,
	          yaw_stabilize_ki,
	          yaw_rate_kp,
	          yaw_rate_ki,
	          &yaw_stabilize_iterm,
	          &yaw_rate_iterm,
	          &yaw_output);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	SD_MPU6050_Result result;
	uint8_t mpu_ok[15] = {"MPU WORK FINE\n"};
	uint8_t mpu_not[17] = {"MPU NOT WORKING\n"};
	uint8_t data[100] = {0, };
	uint8_t data2[100] = {0, };
	int8_t speed;
	double angle_accel;
	double angle_gyro;
	double angle_gyro_integral = 0;
	float angle_final;
	float alpha = 0.96;


  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  //LEFT WHEEL
  //TIM_CHANNEL_1:SPEED
  //TIM_CHANNEL_3:BREAK
  //RIGHT WHEEL
  //TIM_CHANNEL_2:SPEED
  //TIM_CHANNEL_4:BREAK
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  HAL_TIM_Base_Start_IT(&htim3);

  //LEFT WHEEL
  //GPIO_PIN_14:DIRECTION
  //RIGHT WHEEL
  //GPIO_PIN_15:DIRECTION
//  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 0);
//  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  result = SD_MPU6050_Init(&hi2c1,&mpu1,SD_MPU6050_Device_0,SD_MPU6050_Accelerometer_2G,SD_MPU6050_Gyroscope_250s );
  HAL_Delay(1000);
  if(result == SD_MPU6050_Result_Ok){
	HAL_UART_Transmit_IT(&huart3,mpu_ok,(uint16_t)15);
  }
  else{
	HAL_UART_Transmit_IT(&huart3,mpu_not,(uint16_t)17);
	while(1){
	  result = SD_MPU6050_Init(&hi2c1,&mpu1,SD_MPU6050_Device_0,SD_MPU6050_Accelerometer_2G,SD_MPU6050_Gyroscope_250s );
	  HAL_UART_Transmit_IT(&huart3,mpu_not,(uint16_t)17);
	  HAL_Delay(500);
	  if(result == SD_MPU6050_Result_Ok){
		break;
	  }
	}
  }
  HAL_Delay(500);

  calibrate_sensors();
  initYPR();

  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	calcurate_DT();
	readAccelGyro();
	calcAccelYPR();
	calcGyroYPR();
	calcFilteredYPR();


//	angle_accel = -atan((double)a_y/(double)a_z) * 57.3;
//	//(180/pi=57.3)가속도 센서 각도구하기
//
//	//angle_gyro_integral += (-g_y/16383.0*90.0) * dt;
//	angle_gyro = (g_y - base_y_gyro) / 131;
	//자이로 센서의 값도 -32768에서 32767 값을 가질 수 있다.
	//FS_SEL 레지스터가 =일 경우 -250 에서 +250까지 측정되며 이 값은 -32768에서 32767까지 매핑된다.
	//1초 동안 일정한 속도로 1도 회전하였다고 한다면 250도/s 는 32767 이므로 1도/s는 (32767/250) = 131이 된다.
//
//	angle_final = alpha * angle_gyro + (1.0-alpha) * angle_accel;


	//이동시에 speed핀에 pwm주면서 pitch_target_angle 변수 +=5, -=5
	//현재각도 filtered_angle_x변수 이용

	sprintf(data, "filtered_angle_x : %f\n", filtered_angle_x);
	HAL_UART_Transmit_IT(&huart3,data,(uint16_t)strlen(data));


  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 90;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 90;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC1   ------> ETH_MDC
     PA1   ------> ETH_REF_CLK
     PA2   ------> ETH_MDIO
     PA7   ------> ETH_CRS_DV
     PC4   ------> ETH_RXD0
     PC5   ------> ETH_RXD1
     PB13   ------> ETH_TXD1
     PA8   ------> USB_OTG_FS_SOF
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
     PG11   ------> ETH_TX_EN
     PG13   ------> ETH_TXD0
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
