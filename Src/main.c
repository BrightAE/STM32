/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "button.h"
#include "encoder.h"
#include "mpu6050.h"
#include "control.h"
#include "outputdata.h"
#include "u8g2.h"
#include "u8x8.h"
#include "oled.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define NORMAL_SPEED						40
#define TURN_BIG_STEP 					150
#define TURN_SMALL_STEP 				-150
#define TURN_SPEED							20
#define TURN_PULSE_LEFT 				2100
#define TURN_PULSE_RIGHT 				2300
#define PULSE_FORWARD_1M				15000
#define PULSE_BACKWARD_1M				19000

unsigned short SoftTimer[5] = {0, 0, 0, 0, 0};

void SoftTimerCountDown(void) {
	char i;
	for (i = 0; i < 5; ++i) {
		if (SoftTimer[i] > 0) SoftTimer[i]--;
	}
}

void ResetSpeed() {
	g_nLeftBias = 0;
	g_nRightBias = 0;
	g_nTargetSpeed = 0;
}

int action = 0;
int g_nTargetPulse = 0;

void SetForward() {
	g_lLeftMotorPulseAction = g_lRightMotorPulseAction = 0;
	g_nTargetPulse = PULSE_FORWARD_1M;
	action = 1;
}

void SetBackForward() {
	g_lLeftMotorPulseAction = g_lRightMotorPulseAction = 0;
	g_nTargetPulse = PULSE_BACKWARD_1M;
	action = 3;
}
void SetLeft() {
	g_lLeftMotorPulseAction = g_lRightMotorPulseAction = 0;
	g_nTargetPulse = TURN_PULSE_LEFT;
	action = 5;
}
void SetRight() {
	g_lLeftMotorPulseAction = g_lRightMotorPulseAction = 0;
	g_nTargetPulse = TURN_PULSE_RIGHT;
	action = 7;
}
void SetAction() {
	action++;
	if (action % 2 == 0) SoftTimer[1] = 3000;
	else if (action == 1) SetForward();
	else if (action == 3) SetBackForward();
	else if (action == 5) SetLeft();
	else if (action == 7) SetRight();
}


void TurnLeft() {
	g_nLeftBias = TURN_BIG_STEP;
	g_nRightBias = TURN_SMALL_STEP;
	g_nTargetSpeed = TURN_SPEED;
}

void TurnRight() {
	g_nLeftBias = TURN_SMALL_STEP;
	g_nRightBias = TURN_BIG_STEP;
	g_nTargetSpeed = TURN_SPEED;
}
void Forward() {
	g_nTargetSpeed = NORMAL_SPEED;
}
void Backward() {
	g_nTargetSpeed = -NORMAL_SPEED;
}

void ExecAction() {
	if (action == 1) Forward();
	else if (action == 3) Backward();
	else if (action == 5) TurnLeft();
	else if (action == 7) TurnRight();
}

int CheckActionFinished() {
	if (action % 2 == 0)
		return SoftTimer[1] == 0;
	
	int temp = 0;
	if (action == 1 || action == 3) {
		temp = g_lLeftMotorPulseAction + g_lRightMotorPulseAction;
		int different = g_lLeftMotorPulseAction - g_lRightMotorPulseAction;
		if (different > 50) {
			g_nRightBias = 10; g_nLeftBias = -10;
		} else if (different < -50) {
			g_nRightBias = -10; g_nLeftBias = 10;
		} else {
			g_nRightBias = 0; g_nLeftBias = 0;
		}
	}
	else temp = g_lLeftMotorPulseAction - g_lRightMotorPulseAction;
	
	if (temp < 0) temp = -temp;
	if (temp >= g_nTargetPulse)
		return 1;
	return 0;
}

void SecTask() {
	if (SoftTimer[0] == 0) {
		SoftTimer[0] = 5;
		
		if (CheckActionFinished()) {
			ResetSpeed();
			SetAction();
		} else ExecAction();
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	u8g2_t u8g2;
	char cStr[9];
	SoftTimer[1] = 3000;
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
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	MPU_Init();
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
	
	u8g2_Setup_ssd1306_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_4wire_sw_spi, u8x8_stm32_gpio_and_delay);
	u8g2_InitDisplay(&u8g2);
	u8g2_SetPowerSave(&u8g2, 0);
	u8g2_SetFont(&u8g2, u8g2_font_6x12_mr);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		SecTask();
		if (SoftTimer[2] == 0) {
			SoftTimer[2] = 20;
			
			u8g2_ClearBuffer(&u8g2);
			u8g2_DrawStr(&u8g2, 0, 30, "Angle:");
			sprintf(cStr, "%5.1f", g_fCarAngle);
			u8g2_DrawStr(&u8g2, 50, 30, cStr);
			
			u8g2_DrawStr(&u8g2, 0, 50, "State:");
			sprintf(cStr, "%d %d", (int)g_lLeftMotorPulseAction, (int)g_lRightMotorPulseAction);
			u8g2_DrawStr(&u8g2, 50, 50, cStr);
			u8g2_SendBuffer(&u8g2);
			ReadDistance();
		}

		
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
