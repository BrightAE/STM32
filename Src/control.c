#include "control.h"
#include "filter.h"
#include "mpu6050.h"
#include "math.h"
#include "outputdata.h"
#include "tim.h"
#include "main.h"

#define MOTOR_OUT_DEAD_VAL          0
#define MOTOR_OUT_MAX               1000
#define MOTOR_OUT_MIN               -1000

#define CAR_ANGLE_SET								0
#define CAR_ANGLE_SPEED_SET					0

#define CAR_SPEED_SET								0
#define CAR_POSITION_MAX						900
#define CAR_POSITION_MIN						-900
#define SPEED_CONTROL_PERIOD				25

#define CAR_ZERO_ANGLE							g_fCarAngleOffset
float g_fCarAngleOffset = 5.0;

unsigned int g_nMainEventCount;
unsigned int g_nGetPulseCount;
unsigned int g_nSpeedControlCount;

short x_nAcc, y_nAcc, z_nAcc;
short x_nGyro, y_nGyro, z_nGyro;

float x_fAcc, y_fAcc, z_fAcc;
float g_fAccAngle;
float g_fGyroAngleSpeed;
float g_fCarAngle;
float dt = 0.005;

unsigned int g_nLeftMotorPulse, g_nRightMotorPulse;

int nPwmBais;
int nLeftMotorPwm, nRightMotorPwm;
int nLeftMotorErrorPrev, nRightMotorErrorPrev;

float g_fLeftMotorOut, g_fRightMotorOut;
float g_fAngleControlOut;

float g_fCarSpeed;
float g_fCarSpeedPrev;
float g_fCarPosition;
long g_lLeftMotorPulseSigma, g_lRightMotorPulseSigma;
float g_fSpeedControlOut;
float g_fSpeedControlOutNew, g_fSpeedControlOutOld;
int g_nSpeedControlPeriod;

void GetMPUData(void) {
	MPU_Get_Accelerometer(&x_nAcc, &y_nAcc, &z_nAcc);
	MPU_Get_Gyroscope(&x_nGyro, &y_nGyro, &z_nGyro);
}

void AngleCalculate(void) {
	x_fAcc = x_nAcc / 16384.0;
	y_fAcc = y_nAcc / 16384.0;
	z_fAcc = z_nAcc / 16384.0;
	
	g_fAccAngle = atan2(y_fAcc, z_fAcc) * 180.0 / 3.14;
	g_fGyroAngleSpeed = x_nGyro / 16.4;
	g_fCarAngle = ComplementaryFilter(g_fAccAngle, g_fGyroAngleSpeed, dt);
	g_fCarAngle = g_fCarAngle - CAR_ZERO_ANGLE;

	OutData[0] = g_fAccAngle;
	OutData[1] = g_fGyroAngleSpeed;
	OutData[2] = g_fCarAngle;
}

void GetMotorPulse(void) {
	g_nLeftMotorPulse = (short)(__HAL_TIM_GET_COUNTER(&htim2));
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	g_nRightMotorPulse = -(short)(__HAL_TIM_GET_COUNTER(&htim4));
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	
	g_lLeftMotorPulseSigma += g_nLeftMotorPulse;
	g_lRightMotorPulseSigma += g_nRightMotorPulse;
}

int SpeedInnerControl(int nPulse, int nTarget, int nPwm, int nErrorPrev) {
	int nError;
	float fP = 10.0, fI = 0.9;
	
	nError = nPulse - nTarget;
	nPwmBais = fP * (nError - nErrorPrev) + fI * nError;
	nErrorPrev = nError;
	nPwm += nPwmBais;
	
	if (nPwm > MOTOR_OUT_MAX) nPwm = MOTOR_OUT_MAX;
	if (nPwm < MOTOR_OUT_MIN) nPwm = MOTOR_OUT_MIN;
	
	// OutData[0] = (float)nPulse;
	// OutData[1] = (float)nTarget;
	// OutData[2] = (float)nPwm;
	
	return nPwm;
}

void SetMotorVoltageAndDirection(int nLeftMotorPwm, int nRightMotorPwm) {
	if (nLeftMotorPwm < 0) {
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
		nLeftMotorPwm = (-nLeftMotorPwm);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, nLeftMotorPwm);
	} else {
		HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, nLeftMotorPwm);		
	}

	if (nRightMotorPwm < 0) {
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
		nRightMotorPwm = (-nRightMotorPwm);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, nRightMotorPwm);
	} else {
		HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, nRightMotorPwm);		
	}
}

void MotorOutput(void) {
	g_fLeftMotorOut = g_fAngleControlOut - g_fSpeedControlOut;
	g_fRightMotorOut = g_fAngleControlOut - g_fSpeedControlOut;

	if ((int)g_fLeftMotorOut > 0) g_fLeftMotorOut += MOTOR_OUT_DEAD_VAL;
	else if ((int)g_fLeftMotorOut < 0) g_fLeftMotorOut -= MOTOR_OUT_DEAD_VAL;
	if ((int)g_fRightMotorOut > 0) g_fRightMotorOut += MOTOR_OUT_DEAD_VAL;
	else if ((int)g_fRightMotorOut < 0) g_fRightMotorOut -= MOTOR_OUT_DEAD_VAL;

	if ((int)g_fLeftMotorOut > MOTOR_OUT_MAX) g_fLeftMotorOut = MOTOR_OUT_MAX;
	if ((int)g_fLeftMotorOut < MOTOR_OUT_MIN) g_fLeftMotorOut = MOTOR_OUT_MIN;
	if ((int)g_fRightMotorOut > MOTOR_OUT_MAX) g_fRightMotorOut = MOTOR_OUT_MAX;
	if ((int)g_fRightMotorOut < MOTOR_OUT_MIN) g_fRightMotorOut = MOTOR_OUT_MIN;

  SetMotorVoltageAndDirection((int)g_fLeftMotorOut, (int)g_fRightMotorOut);
}

void AngleControl(void) {
    float fP = 65.0;
    float fD = 2.3;
    
    g_fAngleControlOut = (CAR_ANGLE_SET - g_fCarAngle) * fP + (CAR_ANGLE_SPEED_SET - g_fGyroAngleSpeed) * fD;
}

void SpeedControl(void) {
	float fP = 10.25, fI = 0.108;
	float fDelta;
	
	g_fCarSpeed = (g_lLeftMotorPulseSigma + g_lRightMotorPulseSigma) / 2;
	g_lLeftMotorPulseSigma = g_lRightMotorPulseSigma = 0;
	g_fCarSpeed = 0.7 * g_fCarSpeedPrev + 0.3 * g_fCarSpeed;
	g_fCarSpeedPrev = g_fCarSpeed;
	fDelta = CAR_SPEED_SET - g_fCarSpeed;
	g_fCarPosition += fDelta;
	
	if ((int)g_fCarPosition > CAR_POSITION_MAX) g_fCarPosition = CAR_POSITION_MAX;
	if ((int)g_fCarPosition < CAR_POSITION_MIN) g_fCarPosition = CAR_POSITION_MIN;
	
	g_fSpeedControlOutOld = g_fSpeedControlOutNew;
	g_fSpeedControlOutNew = fDelta * fP + g_fCarPosition * fI;
}

void SpeedControlOutput(void) {
	float fValue;
	fValue = g_fSpeedControlOutNew - g_fSpeedControlOutOld;
	g_fSpeedControlOut = fValue * (g_nSpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_fSpeedControlOutOld;
}
