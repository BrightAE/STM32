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
#define TURN_BIG_STEP 		150
#define TURN_SMALL_STEP 	-150
#define TURN_SPEED				-20
#define REC_PULSE_1 				1600
#define REC_PULSE_2 				1450
#define DETECTION_DISTANCE 15

unsigned short SoftTimer[5] = {0, 0, 0, 0, 0};
int CurrentAction = 0; // 0: balance 1: avoiding
int DistancePrev = 0;
int state = 0;
int action = 0;
int hasGap = 0;
int g_nLeftBiasAvoidance = 0, g_nRightBiasAvoidance = 0, g_nTargetSpeedAvoidance = 0;
int g_nLeftBiasTrail = 0, g_nRightBiasTrail = 0, g_nTargetSpeedTrail = 0;
int La = 0, Lb = 0, Ra = 0, Rb = 0;
int g_nTargetPulse = 0;

void SoftTimerCountDown(void) {
	char i;
	for (i = 0; i < 5; ++i) {
		if (SoftTimer[i] > 0) SoftTimer[i]--;
	}
}

void SetRight() {
	g_lLeftMotorPulseAction = g_lRightMotorPulseAction = 0;
	g_nTargetPulse = REC_PULSE_2;
	action = 1;
}

void SetLeft() {
	g_lLeftMotorPulseAction = g_lRightMotorPulseAction = 0;
	g_nTargetPulse = REC_PULSE_1;
	action = 2;
}

void TurnRight() {
	g_nLeftBiasAvoidance = TURN_BIG_STEP;
	g_nRightBiasAvoidance = TURN_SMALL_STEP;
	g_nTargetSpeedAvoidance = TURN_SPEED;
}

void TurnLeft() {
	g_nLeftBiasAvoidance = TURN_SMALL_STEP;
	g_nRightBiasAvoidance = TURN_BIG_STEP;
	g_nTargetSpeedAvoidance = TURN_SPEED;
}

void CheckActionFinished() {
	if (action == 0) return;
	int temp = g_lLeftMotorPulseAction - g_lRightMotorPulseAction;
	if (temp < 0) temp = -temp;
	if (temp >= g_nTargetPulse) {
		action = 0;
		state++;
	}
}

void ExecAction() {
	CheckActionFinished();
	if (action == 0) {
		return;
	}
	if (action == 1) {
		TurnLeft();
		return;
	} else if (action == 2) {
		TurnRight();
		return;
	}
}

void StateControl() {
	if (state == 4) state = 0;
	if (state == 0) {
		if (Distance <= DETECTION_DISTANCE) {
			SetLeft();
		}
	} else if (state == 1) {
		if (Distance <= DETECTION_DISTANCE) {
			SetRight();
		}
	} else if (state == 2) {
		if (Distance <= DETECTION_DISTANCE) {
			SetRight();
		}
	} else if (state == 3) {
		if (Distance <= DETECTION_DISTANCE) {
			SetLeft();
		}
	}
}

void AvoidTask() {
	if (SoftTimer[0] == 0) {
		SoftTimer[0] = 20;
		
		g_nLeftBiasAvoidance = 0;
		g_nRightBiasAvoidance = 0;
		g_nTargetSpeedAvoidance = 14;
		
		
		if (!action) StateControl();
		else ExecAction();
	}
}

void TraceTask() {
	int bigStep = 250, smallStep = 150;
	int FASTSPEED = 14, SLOWSPEED= 14;
	g_nLeftBiasTrail = 0, g_nRightBiasTrail = 0, g_nTargetSpeedTrail = 0;
	if (SoftTimer[1] == 0) {
		SoftTimer[1] = 5;
		La = HAL_GPIO_ReadPin(La_GPIO_Port, La_Pin);
		Lb = HAL_GPIO_ReadPin(Lb_GPIO_Port, Lb_Pin);
		Ra = HAL_GPIO_ReadPin(Ra_GPIO_Port, Ra_Pin);
		Rb = HAL_GPIO_ReadPin(Rb_GPIO_Port, Rb_Pin);
		g_nTargetSpeedTrail = FASTSPEED;
		if ((La + Lb + Ra + Rb) <= 2) {
			if (Lb == 1) g_nRightBiasTrail += bigStep, g_nLeftBiasTrail -= bigStep, g_nTargetSpeedTrail = SLOWSPEED;
			if (La == 1) g_nRightBiasTrail += smallStep, g_nLeftBiasTrail -= smallStep, g_nTargetSpeedTrail = FASTSPEED;
			if (Ra == 1) g_nLeftBiasTrail += smallStep, g_nRightBiasTrail -= smallStep, g_nTargetSpeedTrail = FASTSPEED;
			if (Rb == 1) g_nLeftBiasTrail += bigStep, g_nRightBiasTrail -= bigStep, g_nTargetSpeedTrail = SLOWSPEED;
		}else{
			g_nTargetSpeedTrail = 7;
		}
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
	char cStr[3];
	char cStr2[10];
	
	int dist[4] = { 1000 }, min_dist= 1000 , BIG_TICK_COUNT=0;
	Distance = 1000;
	int stage = 1, flag = 0;
	
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
	u8g2_SetFont(&u8g2, u8g2_font_4x6_mr);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
//		SecTask();
		if (stage == 0) TraceTask();
		else if (stage == 1) AvoidTask();
		
		if (SoftTimer[2] == 0) {
			SoftTimer[2] = 20;
			
			u8g2_ClearBuffer(&u8g2);
			
			u8g2_DrawStr(&u8g2, 0, 20, "Angle:");
			sprintf(cStr, "%5.1f", g_fCarAngle);
			u8g2_DrawStr(&u8g2, 50, 20, cStr);
			
			u8g2_DrawStr(&u8g2, 0, 40, "Distance:");
			sprintf(cStr2, "%d %d", Distance, stage);			
			u8g2_DrawStr(&u8g2, 50, 40, cStr2);
			
			u8g2_DrawStr(&u8g2, 0, 60, "Laser:");
			sprintf(cStr2, "%d %d %d %d", Lb, La, Ra, Rb);
			u8g2_DrawStr(&u8g2, 50, 60, cStr2);
			
			u8g2_SendBuffer(&u8g2);
		}
		
		if (SoftTimer[3] == 0) {
			SoftTimer[3] = 2;
			La = HAL_GPIO_ReadPin(La_GPIO_Port, La_Pin);
			Lb = HAL_GPIO_ReadPin(Lb_GPIO_Port, Lb_Pin);
			Ra = HAL_GPIO_ReadPin(Ra_GPIO_Port, Ra_Pin);
			Rb = HAL_GPIO_ReadPin(Rb_GPIO_Port, Rb_Pin);
			if ((La + Lb + Ra + Rb) == 4 && g_fCarAngle <= 2.0 && !flag) {
				stage++; flag = 1;
				g_nLeftBiasAvoidance = g_nRightBiasAvoidance = g_nTargetSpeedAvoidance = 0;
				g_nLeftBiasTrail = g_nRightBiasTrail = g_nTargetSpeedTrail = 0;
			} else if ((La + Lb + Ra + Rb) != 4)  flag = 0;
			
			BIG_TICK_COUNT++;
			int temp = Distance;
			if(temp < 3) temp = 1000;
			ReadDistance();
		  if (Distance > 1000 || Distance <= 3) {
				Distance = temp;
			}
			//int temp_count=0;
			//for (int i=0;i<4;i++) if( dist[i]-Distance <= 6 && Distance-dist[i]<=6) temp_count++;
			//if(temp_count<=2 && BIG_TICK_COUNT>=5)
			//	Distance = dist[3];
			//else
			//{
				//for (int i = 0; i < 3; ++i)
					//dist[i] = dist[i + 1];
//				dist[3] = Distance;
			//}
			//if(BIG_TICK_COUNT>=5&&temp_count<=2)
				//Distance=temp;
			//for (int i = 0; i < 4; ++i)
				
			
			
		}
		
		g_nLeftBias = g_nLeftBiasAvoidance + g_nLeftBiasTrail;
		g_nRightBias = g_nRightBiasAvoidance + g_nRightBiasTrail;
		g_nTargetSpeed = g_nTargetSpeedAvoidance + g_nTargetSpeedTrail;
		
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
