#include "leds.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"

void initLeds(void)
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
	LL_GPIO_InitTypeDef GpioConf;
	GpioConf.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GpioConf.Mode = LL_GPIO_MODE_OUTPUT;
	GpioConf.Pin = LL_GPIO_PIN_15 | LL_GPIO_PIN_14 | LL_GPIO_PIN_13 | LL_GPIO_PIN_12;
	LL_GPIO_Init(GPIOD, &GpioConf);	
	
	ledOff(BLUE | GREEN | RED | ORAGNE);
}

void ledOn(enum ELeds Which)
{
	const uint8_t LedCnt = 4;
	int32_t CurrentLedPos = LL_GPIO_PIN_12;
	for(uint8_t X = 0; X<LedCnt; ++X)
	{
		if( (CurrentLedPos & Which) != 0)
			LL_GPIO_SetOutputPin(GPIOD, CurrentLedPos & Which);
		
		CurrentLedPos <<= 1;
	}
}

void ledOff(enum ELeds Which)
{
	const uint8_t LedCnt = 4;
	int32_t CurrentLedPos = LL_GPIO_PIN_12;
	for(uint8_t X = 0; X<LedCnt; ++X)
	{
		if( (CurrentLedPos & Which) != 0)
			LL_GPIO_ResetOutputPin(GPIOD, CurrentLedPos & Which);
		
		CurrentLedPos <<= 1;
	}
}

void ledToggle(enum ELeds Which)
{
	const uint8_t LedCnt = 4;
	int32_t CurrentLedPos = LL_GPIO_PIN_12;
	for(uint8_t X = 0; X<LedCnt; ++X)
	{
		if( (CurrentLedPos & Which) != 0)
			LL_GPIO_TogglePin(GPIOD, CurrentLedPos & Which);
		
		CurrentLedPos <<= 1;
	}
}
