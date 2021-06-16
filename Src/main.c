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
#define INITIAL_SPEED							6
#define START_STAGE								1
#define UNTRUSTWORTHY_ANGLE 			3.0
#define DEBUG_WANT_STAGE_CHANGE		1
#define DEBUG_JUST_TURN						0
#define DEBUG_WAIT_TIME						0

#define TRACE_TURN_BIG_STEP						180//150
#define TRACE_TURN_SMALL_STEP				  80//120
#define TRACE_TURN_FAST_SPEED					5
#define TRACE_TURN_SLOW_SPEED					5
#define TRACE_JITTER_SPEED						5

#define AVOID_NORMAL_SPEED						18		//23 30
#define AVOID_TURN_BIG_STEP 					150		//120 200
#define AVOID_TURN_SMALL_STEP 				-150	//-120 200
#define AVOID_TURN_SPEED							-15		//-6 -33
#define AVOID_REC_PULSE_LEFT 					-1900	//-1900 -1840
#define AVOID_REC_PULSE_RIGHT 				1900	//1900 1840
#define AVOID_DETECTION_DISTANCE 			20		//20 25

#define RECORD_DIST_NUMBER						3
#define TRUST_RECORD_NUMBER						2
#define BALANCE_SPEED						-3

int DEBUG_SLEEP = 0;



unsigned short SoftTimer[10] = {0, 0, 0, 0, 0};

int stage = START_STAGE, flag = 0;
int La = 0, Lb = 0, Ra = 0, Rb = 0;
int state = 0;
int action = 0;
int lastDist[RECORD_DIST_NUMBER];
int DebugDist[RECORD_DIST_NUMBER];

int g_nLeftBiasAvoidance = 0, g_nRightBiasAvoidance = 0, g_nTargetSpeedAvoidance = 0; long g_nTargetPulse = 0, g_nStartPulse = 0;
int g_nLeftBiasTrail = 0, g_nRightBiasTrail = 0, g_nTargetSpeedTrail = 0, g_nLastActivedLaser = 0  , g_nLastActivedLaserCount=0;
double g_nTurnSpeedPercent;

int g_nWantStraight = 0; long g_nStraightMotorPulseActionDelta = 0;

// try to take 90 turn
long ApproachDegree90Delta(long true_delta){
	int SmallRange = 250, BigRange = 300;
	true_delta%=4000;
	if(true_delta>=-BigRange&&true_delta<=BigRange)return 0;
	if(true_delta>=-SmallRange+1000&&true_delta<=SmallRange+1000)return 1000;
	if(true_delta>=-SmallRange+2000&&true_delta<=SmallRange+2000)return 2000;
	if(true_delta>=-SmallRange+3000&&true_delta<=SmallRange+3000)return 3000;
	if(true_delta>=-SmallRange-3000&&true_delta<=SmallRange-3000)return -3000;
	if(true_delta>=-SmallRange-2000&&true_delta<=SmallRange-2000)return -2000;
	if(true_delta>=-SmallRange-1000&&true_delta<=SmallRange-1000)return -1000;
	if(true_delta>=4000-BigRange||true_delta<=-4000+BigRange)return 0;
	return true_delta;
}
void SetWantStraight(){
	g_nWantStraight = 1 ;
	g_nStraightMotorPulseActionDelta = ApproachDegree90Delta(g_lLeftMotorPulseAction - g_lRightMotorPulseAction);
}
void UnsetWantStraight(){
	g_nWantStraight = 0 ;
}
void SoftTimerCountDown(void) {
	char i;
	for (i = 0; i < 10; ++i) {
		if (SoftTimer[i] > 0) SoftTimer[i]--;
	}
}

void SetLeft() {
	//g_lLeftMotorPulseAction = g_lRightMotorPulseAction = 0;
	g_nStartPulse = g_lLeftMotorPulseAction-g_lRightMotorPulseAction;
	g_nTargetPulse = g_nStartPulse + AVOID_REC_PULSE_LEFT;
	action = 1;
	
	g_nTurnSpeedPercent = 1.0;
	UnsetWantStraight();
}
void TurnLeft() {
	g_nLeftBiasAvoidance = AVOID_TURN_BIG_STEP		*g_nTurnSpeedPercent;
	g_nRightBiasAvoidance = AVOID_TURN_SMALL_STEP	*g_nTurnSpeedPercent;
	g_nTargetSpeedAvoidance = AVOID_TURN_SPEED		*g_nTurnSpeedPercent;
}

void SetRight() {
	g_nStartPulse = g_lLeftMotorPulseAction-g_lRightMotorPulseAction;
	g_nTargetPulse = g_nStartPulse + AVOID_REC_PULSE_RIGHT;
	action = 2;
	
	g_nTurnSpeedPercent = 1.0;
	UnsetWantStraight();
}

void TurnRight() {
	g_nLeftBiasAvoidance = AVOID_TURN_SMALL_STEP	*g_nTurnSpeedPercent;
	g_nRightBiasAvoidance = AVOID_TURN_BIG_STEP		*g_nTurnSpeedPercent;
	g_nTargetSpeedAvoidance = AVOID_TURN_SPEED		*g_nTurnSpeedPercent;
}



void CheckActionFinished() {
	if (action == 0) return;
	long nowDelta = g_lLeftMotorPulseAction - g_lRightMotorPulseAction;
	double percentage = 0.0;
	if(g_nStartPulse<g_nTargetPulse)
		percentage = 1.0f*(nowDelta-g_nStartPulse)/AVOID_REC_PULSE_LEFT;
	if(g_nStartPulse>g_nTargetPulse)
		percentage = 1.0f*(nowDelta-g_nStartPulse)/AVOID_REC_PULSE_RIGHT;
	
	if (percentage>=1.0) {
		action = 0;
		state++;
		SetWantStraight();
		
		//SoftTimer[3]=DEBUG_WAIT_TIME;	DEBUG_SLEEP=1;
		g_nLeftBiasAvoidance = 0, g_nRightBiasAvoidance = 0;
		g_nTargetSpeedAvoidance = 0;
		
		g_nStartPulse = g_nTargetPulse = 0;
		
	}else if(percentage>=0.9){
		g_nTurnSpeedPercent = 0.25;
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
		if (Distance <= AVOID_DETECTION_DISTANCE || DEBUG_JUST_TURN) {
			//SetLeft();
			
			SetRight();
		}
	} else if (state == 1) {
		if (Distance <= AVOID_DETECTION_DISTANCE || DEBUG_JUST_TURN) {
			//SetRight();
			
			SetLeft();
		}
	} else if (state == 2) {
		if (Distance <= AVOID_DETECTION_DISTANCE || DEBUG_JUST_TURN) {
			//SetRight();
			
			SetLeft();
		}
	} else if (state == 3) {
		if (Distance <= AVOID_DETECTION_DISTANCE || DEBUG_JUST_TURN) {
			//SetLeft();
			
			SetRight();
		}
	}
}


// avoid task
// if no action executing, do state control
void AvoidTask() {
	g_nLeftBiasAvoidance = 0, g_nRightBiasAvoidance = 0;
	g_nTargetSpeedAvoidance = AVOID_NORMAL_SPEED;
	
	if (!action) StateControl();
	else ExecAction();
}

// trace task
void TraceTask() {
	g_nLeftBiasTrail = 0, g_nRightBiasTrail = 0;
	g_nTargetSpeedTrail = TRACE_TURN_FAST_SPEED;
	if ((La + Lb + Ra + Rb) == 0 ) {
		if(g_nLastActivedLaser==1)
			g_nRightBiasTrail += TRACE_TURN_BIG_STEP, g_nLeftBiasTrail -= TRACE_TURN_BIG_STEP, g_nTargetSpeedTrail = 0;
		if(g_nLastActivedLaser==2)
			g_nRightBiasTrail += TRACE_TURN_SMALL_STEP, g_nLeftBiasTrail -= TRACE_TURN_SMALL_STEP, g_nTargetSpeedTrail = 0;
		if(g_nLastActivedLaser==3)
			g_nLeftBiasTrail += TRACE_TURN_SMALL_STEP, g_nRightBiasTrail -= TRACE_TURN_SMALL_STEP, g_nTargetSpeedTrail = 0;
		if(g_nLastActivedLaser==4)
			g_nLeftBiasTrail += TRACE_TURN_BIG_STEP, g_nRightBiasTrail -= TRACE_TURN_BIG_STEP, g_nTargetSpeedTrail = 0;
		
		g_nLastActivedLaserCount --;
		if(!g_nLastActivedLaserCount)g_nLastActivedLaser = 0;
	}
	else if ((La + Lb + Ra + Rb) <= 3) {
		if (Lb == 1) g_nRightBiasTrail += TRACE_TURN_BIG_STEP, g_nLeftBiasTrail -= TRACE_TURN_BIG_STEP, g_nTargetSpeedTrail = TRACE_TURN_SLOW_SPEED , 		g_nLastActivedLaser = 1 , g_nLastActivedLaserCount=80;
		if (La == 1) g_nRightBiasTrail += TRACE_TURN_SMALL_STEP, g_nLeftBiasTrail -= TRACE_TURN_SMALL_STEP, g_nTargetSpeedTrail = TRACE_TURN_FAST_SPEED , g_nLastActivedLaser = 2 , g_nLastActivedLaserCount=5;
		if (Ra == 1) g_nLeftBiasTrail += TRACE_TURN_SMALL_STEP, g_nRightBiasTrail -= TRACE_TURN_SMALL_STEP, g_nTargetSpeedTrail = TRACE_TURN_FAST_SPEED , g_nLastActivedLaser = 3 , g_nLastActivedLaserCount=5;
		if (Rb == 1) g_nLeftBiasTrail += TRACE_TURN_BIG_STEP, g_nRightBiasTrail -= TRACE_TURN_BIG_STEP, g_nTargetSpeedTrail = TRACE_TURN_SLOW_SPEED , 		g_nLastActivedLaser = 4 , g_nLastActivedLaserCount=80;
	} else {
		g_nTargetSpeedTrail = TRACE_JITTER_SPEED;
	}
}


void RecordDistance(){
	int temp = lastDist[RECORD_DIST_NUMBER-1];
	if(temp <= 2) temp = 1000;
	ReadDistance();
	if (Distance > 1000 || Distance <= 3) {
		Distance = temp;
	}
	for(int i=0;i<RECORD_DIST_NUMBER-1;i++)
		lastDist[i]=lastDist[i+1];
	lastDist[RECORD_DIST_NUMBER-1]=Distance;									// lastDist: actual records
	if(Distance <= AVOID_DETECTION_DISTANCE){// 
		int TEMP_COUNT=0;
		for(int i=0;i<RECORD_DIST_NUMBER;i++)TEMP_COUNT += (lastDist[i]<=AVOID_DETECTION_DISTANCE);
		if(TEMP_COUNT < TRUST_RECORD_NUMBER)
			Distance = 6666,SoftTimer[4]+=23;									// if only TRUST_RECORD_NUMBER-1 in RECORD_DIST_NUMBER actual records  demand  avoidance, just ignore.
		else{
			for(int i=0;i<RECORD_DIST_NUMBER;i++)DebugDist[i]=lastDist[i]<1000?lastDist[i]:0;
			//SoftTimer[3]=30000;
		}
	}
}

void SecTask() {
	
	g_nLeftBiasAvoidance = g_nRightBiasAvoidance = g_nTargetSpeedAvoidance = 0;
	g_nLeftBiasTrail = g_nRightBiasTrail = g_nTargetSpeedTrail = 0;
	
	if(DEBUG_WANT_STAGE_CHANGE){
		if ((La + Lb + Ra + Rb) == 4 && g_fCarAngle <= UNTRUSTWORTHY_ANGLE && !flag) {
			
			stage++; flag = 1;
			
			if(stage == 1) UnsetWantStraight();
			else if(stage == 2)SetWantStraight();
			
		} else if ((La + Lb + Ra + Rb) != 4)  flag = 0;
	}
	if(stage==1 && Distance <= AVOID_DETECTION_DISTANCE){			// trick
		stage = 2;
	}
	
	
	if (stage == 1) TraceTask();
	else if (stage == 2) AvoidTask();
	
	g_nLeftBias = g_nLeftBiasAvoidance + g_nLeftBiasTrail;
	g_nRightBias = g_nRightBiasAvoidance + g_nRightBiasTrail;
	g_nTargetSpeed = g_nTargetSpeedAvoidance + g_nTargetSpeedTrail;
	
	
	if (stage == 0) g_nTargetSpeed = INITIAL_SPEED;
	
	g_nTargetSpeed += BALANCE_SPEED;
}


void KeepStraight(){
	// keep straight
	if(g_nWantStraight){
		long nowDelta = g_lLeftMotorPulseAction - g_lRightMotorPulseAction;
		if(nowDelta > g_nStraightMotorPulseActionDelta + 100)
			g_nLeftBias -= 25, g_nRightBias += 25;
		else if(nowDelta < g_nStraightMotorPulseActionDelta - 100)
			g_nLeftBias += 25, g_nRightBias -= 25;
		else if(nowDelta > g_nStraightMotorPulseActionDelta + 30)
			g_nLeftBias -= 15, g_nRightBias += 15;
		else if(nowDelta < g_nStraightMotorPulseActionDelta - 30)
			g_nLeftBias += 15, g_nRightBias -= 15;
		else if(nowDelta > g_nStraightMotorPulseActionDelta + 15)
			g_nLeftBias -= 3, g_nRightBias += 3;
		else if(nowDelta < g_nStraightMotorPulseActionDelta - 15)
			g_nLeftBias += 3, g_nRightBias -= 3;
		
		if(stage == 1){			
			if(nowDelta > g_nStraightMotorPulseActionDelta + 100)
				g_nLeftBias -= 24, g_nRightBias += 24;
			else if(nowDelta < g_nStraightMotorPulseActionDelta - 100)
				g_nLeftBias += 24, g_nRightBias -= 24;
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
	char cStr[30];
	char cStr2[30];
	int laserCount = 0;
	
	Distance = 1000;
	
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
	
	SetWantStraight();
	for(int i=0;i<RECORD_DIST_NUMBER;i++)
		lastDist[i]=1000, DebugDist[i]=-1;
	
  while (1)
  {
		if (stage == 3) {
			g_fCarAngleOffset = 2.0;
			g_nTargetSpeed = 40;
			g_nLeftBias = 0;
			g_nRightBias = 0;
			SoftTimer[5] = 3000;
			stage = 4;
		}
		
		if (SoftTimer[5] == 0 && stage == 4) {
			g_nTargetSpeed = 0;
			g_nLeftBias = 0;
			g_nRightBias = 0;
		}
		
		
		if(SoftTimer[4]==0){
			SoftTimer[4]=71;
			RecordDistance();
		}
		if(!DEBUG_SLEEP)
		if(SoftTimer[0] == 0){
			SoftTimer[0] = 5;
			
			laserCount++;
			if (laserCount == 1) {
				laserCount = 0;
				La = HAL_GPIO_ReadPin(La_GPIO_Port, La_Pin);
				Lb = HAL_GPIO_ReadPin(Lb_GPIO_Port, Lb_Pin);
				Ra = HAL_GPIO_ReadPin(Ra_GPIO_Port, Ra_Pin);
				Rb = HAL_GPIO_ReadPin(Rb_GPIO_Port, Rb_Pin);
			}
			
			if (SoftTimer[3] == 0) {
				SecTask();
			}else{
				g_nLeftBias = g_nRightBias = g_nTargetSpeed = 0;
			}
			KeepStraight();
		}
		
		if (SoftTimer[2] == 0) {
			SoftTimer[2] = 20;
			
			u8g2_ClearBuffer(&u8g2);
			
			u8g2_DrawStr(&u8g2, 0, 8, "Angle:");
			sprintf(cStr, "%4.1f", g_fCarAngle);
			u8g2_DrawStr(&u8g2, 50, 8, cStr);
//			u8g2_DrawStr(&u8g2, 0, 8, "DebugDist:");
			//sprintf(cStr, "%d %d %d %d %d %d %d %d", DebugDist[0], DebugDist[1], DebugDist[2],DebugDist[3],DebugDist[4],DebugDist[5],DebugDist[6],DebugDist[7]);
//			sprintf(cStr, "%d %d %d %d", DebugDist[0], DebugDist[1], DebugDist[2],DebugDist[3]);
//			u8g2_DrawStr(&u8g2, 0, 16, cStr);
			
			u8g2_DrawStr(&u8g2, 0, 16, "Laser:");
			sprintf(cStr2, "%d %d %d %d", Lb, La, Ra, Rb);
			u8g2_DrawStr(&u8g2, 50, 16, cStr2);
			
			u8g2_DrawStr(&u8g2, 0, 30, "Dis/Stage:");
			sprintf(cStr2, "%d %d", Distance, stage);			
			u8g2_DrawStr(&u8g2, 50, 30, cStr2);
			
			u8g2_DrawStr(&u8g2, 0, 45, "Straight:");
			sprintf(cStr2, " %d %d", g_nWantStraight, (int)g_nStraightMotorPulseActionDelta);			
			u8g2_DrawStr(&u8g2, 50, 45, cStr2);
			
			u8g2_DrawStr(&u8g2, 0, 60, "Delta/TarV:");
			sprintf(cStr2, "%d %d", (int)(g_lLeftMotorPulseAction - g_lRightMotorPulseAction), g_nTargetSpeed);			
			u8g2_DrawStr(&u8g2, 50, 60, cStr2);
			
			//u8g2_DrawStr(&u8g2, 0, 60, "Laser:");
			//sprintf(cStr2, "%d %d %d %d", Lb, La, Ra, Rb);
			//u8g2_DrawStr(&u8g2, 50, 60, cStr2);
			
			u8g2_SendBuffer(&u8g2);
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
