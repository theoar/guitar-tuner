#ifndef LEDS_H
#define LEDS_H

#include "stm32f4xx_ll_gpio.h"

enum ELeds
{
	BLUE = LL_GPIO_PIN_15,
	RED = LL_GPIO_PIN_14,
	ORAGNE = LL_GPIO_PIN_13,
	GREEN = LL_GPIO_PIN_12
};


void initLeds(void);
void ledOn(enum ELeds Which);
void ledOff(enum ELeds Which);
void ledToggle(enum ELeds Which);

#endif