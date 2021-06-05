#ifndef __OLED_H
#define __OLED_H

#include "u8g2.h"
#include "u8x8.h"

void OLED_Init(void);
uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8, 
		U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, 
		U8X8_UNUSED void *arg_ptr);


#endif
