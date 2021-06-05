#include "gpio.h"
#include "oled.h"

#define OLED_RST_Clr() HAL_GPIO_WritePin(GPIOB,OLED_RST_Pin,GPIO_PIN_RESET)
#define OLED_RST_Set() HAL_GPIO_WritePin(GPIOB,OLED_RST_Pin,GPIO_PIN_SET)
#define OLED_DC_Clr() HAL_GPIO_WritePin(GPIOC,OLED_DC_Pin,GPIO_PIN_RESET)
#define OLED_DC_Set() HAL_GPIO_WritePin(GPIOC,OLED_DC_Pin,GPIO_PIN_SET)
#define OLED_SCLK_Clr() HAL_GPIO_WritePin(GPIOB,OLED_SCL_Pin,GPIO_PIN_RESET)
#define OLED_SCLK_Set() HAL_GPIO_WritePin(GPIOB,OLED_SCL_Pin,GPIO_PIN_SET)
#define OLED_SDIN_Clr() HAL_GPIO_WritePin(GPIOB,OLED_SDA_Pin,GPIO_PIN_RESET)
#define OLED_SDIN_Set() HAL_GPIO_WritePin(GPIOB,OLED_SDA_Pin,GPIO_PIN_SET) 

void OLED_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* GPIOB/GPIOC clock enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET);
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	LED_Pin|OLED_RST_Pin|OLED_SDA_Pin|OLED_SCL_Pin, GPIO_PIN_RESET);
	/*Configure GPIO pin : PtPin */
	GPIO_InitStruct.Pin = OLED_DC_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(OLED_DC_GPIO_Port, &GPIO_InitStruct);
	/*Configure GPIO pins : PBPin PBPin PBPin PBPin
	PBPin */
	GPIO_InitStruct.Pin = OLED_RST_Pin|OLED_SDA_Pin|OLED_SCL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); 
}
uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8,
		U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
		U8X8_UNUSED void *arg_ptr) {
	switch (msg) {
		case U8X8_MSG_GPIO_AND_DELAY_INIT:
			OLED_Init(); 
			break;
		
		case U8X8_MSG_GPIO_SPI_DATA:
			if(arg_int)OLED_SDIN_Set();
			else OLED_SDIN_Clr();
			break;
		
			case U8X8_MSG_GPIO_SPI_CLOCK:
			if(arg_int)OLED_SCLK_Set();
			else OLED_SCLK_Clr();
			break; 
			
		case U8X8_MSG_GPIO_CS:
			
		case U8X8_MSG_GPIO_DC:
			if(arg_int)OLED_DC_Set();
			else OLED_DC_Clr();
			break;
		
		case U8X8_MSG_GPIO_RESET:
			if(arg_int)OLED_RST_Set();
			else OLED_RST_Clr();
			break;
		
			case U8X8_MSG_DELAY_100NANO: 
			__NOP(); 
			break;
			
		case U8X8_MSG_DELAY_MILLI:
			HAL_Delay(arg_int);
			break;
		
		default:
			return 0;
	}
	return 1;
}
