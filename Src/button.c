#include "button.h"
#include "main.h"
#include "stm32f1xx_it.h"

int iButtonCount;
int iButtonFlag;
int g_iButtonState;


void ButtonScan(void) {
	if (HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin) == GPIO_PIN_RESET) {
		iButtonCount++;
		if (iButtonCount >= 30) {
			if (iButtonFlag == 0) {
				g_iButtonState = 1;
				iButtonFlag = 1;
			}
			iButtonCount = 0;
		} else {
			g_iButtonState = 0;
		}
	}
	else {
		iButtonCount = 0;
		iButtonFlag = 0;
		g_iButtonState = 0;
	}
}

