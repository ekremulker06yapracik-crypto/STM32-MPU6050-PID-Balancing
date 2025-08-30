/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU_V2.1.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Accel_Gyro MPU6050;
Temperature temp;

uint16_t MotorOneValue = 120;
uint16_t MotorTwoValue = 120;

float kp = 1.3;    // P
float ki = 0.0023;  // I
float kd = 0.002;    // D (örnek değerle başlıyoruz)

float setPoint = 0.0;
float error = 0.0;
float integral = 0.0;
float integralIn = 50;
float derivative = 0.0;
float lastError = 0.0;
float output = 0.0;

int16_t motorBaseSpeed = 110; // Dengede çalışılan sabit hız
float pitch = 0.0;

uint32_t TimeLed = 0;

uint16_t Motor1Max = 170;
uint16_t Motor1Min = 106;

uint16_t Motor2Max = 150;
uint16_t Motor2Min = 106;

uint32_t lastTime = 0;
uint32_t loopInterval = 10; // 10 ms

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_Init(&MPU6050, FS_500, AFS_4G);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  /*
   * Motor Calib Ayarları
   */

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 200);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 200);
  HAL_Delay(2000);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 100);
  HAL_Delay(2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (HAL_GetTick() - lastTime >= loopInterval)
	  {
		   MPU6050_Start(&MPU6050, &temp);

		    error = setPoint - MPU6050.Pitch;
		    integral += error;

		    // Integral windup sınırı
		    if (integral > integralIn) integral = integralIn;
		    if (integral < -1 * integralIn) integral = -1 * integralIn;

		    // Derivative hesapla
		    derivative = error - lastError;
		    lastError = error;

		    // PID toplam çıkış
		    output = kp * error + ki * integral + kd * derivative;

		    MotorOneValue = motorBaseSpeed + output;
		    MotorTwoValue = motorBaseSpeed - output;

		    if (MotorOneValue > Motor1Max) MotorOneValue = Motor1Max;
		    if (MotorOneValue < Motor1Min) MotorOneValue = Motor1Min;

		    if (MotorTwoValue > Motor2Max) MotorTwoValue = Motor2Max;
		    if (MotorTwoValue < Motor2Min) MotorTwoValue = Motor2Min;

		    TIM3->CCR1 = MotorOneValue;
		    TIM3->CCR2 = MotorTwoValue;

		    if(uwTick - TimeLed > 500)
		    {
		    	HAL_GPIO_TogglePin(UserLed_GPIO_Port, UserLed_Pin);
		    	TimeLed = uwTick;
		    }

		    lastTime = HAL_GetTick();
	  }

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
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
