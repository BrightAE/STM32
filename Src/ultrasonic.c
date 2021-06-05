#include "ultrasonic.h"
#include "tim.h"
#include "main.h"

unsigned int TIM1CH4_CAPTURE_STA;
unsigned int TIM1CH4_CAPTURE_VAL;

int Distance = 0;
int UltraError = 0;

void ReadDistance(void) {
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
	
	if (TIM1CH4_CAPTURE_STA & 0x80) {
		Distance = TIM1CH4_CAPTURE_STA & 0x3F;
		Distance *= 65536;
		Distance += TIM1CH4_CAPTURE_VAL;
		Distance = Distance * 170 / 10000;
		TIM1CH4_CAPTURE_STA = 0;
	}
}

void UltraSelfCheck(void) {
	HAL_Delay(1000);
	
	if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11)) {
		HAL_Delay(50);
		if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11)) {
			UltraError = 1;
		}
	}
}

int IsUltraOK(void) {
	return UltraError;
}
