#include "lcd.h"
#include "stm32f4xx_ll_bus.h"

LcdConfigStructure PrivateLcdStructure;

LcdConfigStructure *lcdStructure()
{
	return &PrivateLcdStructure;
}

void lcdSetupDataPin(uint8_t LcdPin, uint16_t GpioPin, GPIO_TypeDef* GpioPort)
{
	PrivateLcdStructure.LcdDxPins[LcdPin] = GpioPin;
	PrivateLcdStructure.LcdDxPorts[LcdPin] = GpioPort;
}

void lcdSetupControlPin(uint8_t LcdPin, uint16_t GpioPin, GPIO_TypeDef* GpioPort)	
{
	PrivateLcdStructure.LcdControlPins[LcdPin] = GpioPin;
	PrivateLcdStructure.LcdControlPorts[LcdPin] = GpioPort;
}

void lcdSetupBusWidth(uint8_t Width)
{
	PrivateLcdStructure.LcdBusWidth = Width;
}
void lcdSetupRwLine(uint8_t RwEnableDisable)
{
	PrivateLcdStructure.LcdRwLine = RwEnableDisable;
}
void lcdSetupWaitMode(uint8_t WaitMode)
{
	PrivateLcdStructure.LcdWaitMode = WaitMode;
}

inline void lcdCommand(uint8_t Cmd)
{
	lcdWrite(Cmd, LCD_COMMAND);
}

inline void lcdClr(void)
{
	lcdWrite(_BV(LCD_CLR), LCD_COMMAND);	
	delayMs(5);
}

inline void lcdPutc(unsigned char C)
{
	lcdWrite(C, LCD_DATA);
}

inline void lcdGoTo(uint8_t X, uint8_t Y)
{
	lcdCommand(_BV(LCD_DDRAM)+(Y+_BV(LCD_CGRAM)*X));
}


void lcdInit(void)
{
	GPIO_TypeDef *Oryginal[5] = { GPIOA, GPIOB, GPIOC, GPIOD, GPIOE };
	const uint32_t Addr[5] = { LL_AHB1_GRP1_PERIPH_GPIOA, LL_AHB1_GRP1_PERIPH_GPIOB, LL_AHB1_GRP1_PERIPH_GPIOC, LL_AHB1_GRP1_PERIPH_GPIOD, LL_AHB1_GRP1_PERIPH_GPIOE};
	
	LL_GPIO_InitTypeDef InitGpio;
	InitGpio.Mode = LL_GPIO_MODE_OUTPUT;
	InitGpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	InitGpio.Pull = LL_GPIO_PULL_NO;
	InitGpio.Speed = LL_GPIO_SPEED_FREQ_LOW;
	
	for(uint8_t X = ((PrivateLcdStructure.LcdBusWidth==8) ? 0 : 4); X<8; X++)

	{
		for(uint8_t Y = 0; Y<5; ++Y)
		{
			if(PrivateLcdStructure.LcdDxPorts[X]==Oryginal[Y])
			{	
				LL_AHB1_GRP1_EnableClock(Addr[Y]);
				InitGpio.Pin  = PrivateLcdStructure.LcdDxPins[X];
				LL_GPIO_Init(PrivateLcdStructure.LcdDxPorts[X], &InitGpio);
				break;
			}				
		}
	}
	
	for(uint8_t X = ((PrivateLcdStructure.LcdRwLine==LCD_RW_ENABLE) ? 0 : 1); X<4; X++)
	{
		for(uint8_t Y = 0; Y<5; ++Y)
		{
			if(PrivateLcdStructure.LcdControlPorts[X]==Oryginal[Y])
			{	
				LL_AHB1_GRP1_EnableClock(Addr[Y]);
				InitGpio.Pin  = PrivateLcdStructure.LcdControlPins[X];
				LL_GPIO_Init(PrivateLcdStructure.LcdControlPorts[X], &InitGpio);
				break;
			}				
		}
	}
	
	//RS = 0, RW = 0 E = 0
	LL_GPIO_ResetOutputPin(PrivateLcdStructure.LcdControlPorts[LCD_E], PrivateLcdStructure.LcdControlPins[LCD_E]);		
	LL_GPIO_ResetOutputPin(PrivateLcdStructure.LcdControlPorts[LCD_RS], PrivateLcdStructure.LcdControlPins[LCD_RS]);	
	if(PrivateLcdStructure.LcdRwLine==LCD_RW_ENABLE)
		LL_GPIO_ResetOutputPin(PrivateLcdStructure.LcdControlPorts[LCD_RW], PrivateLcdStructure.LcdControlPins[LCD_RW]);		
	
	for(uint8_t X = 8; X>((PrivateLcdStructure.LcdBusWidth==LCD_BUS_8) ? 0 : 4); --X)	
		LL_GPIO_SetOutputPin(PrivateLcdStructure.LcdDxPorts[X-1], PrivateLcdStructure.LcdDxPins[X-1]);	

	//Startup Delay
	delayMs(DELAY_RESET);

		//Initialize Display
	LL_GPIO_ResetOutputPin(PrivateLcdStructure.LcdDxPorts[LCD_D7], PrivateLcdStructure.LcdDxPins[LCD_D7]);
	LL_GPIO_ResetOutputPin(PrivateLcdStructure.LcdDxPorts[LCD_D6], PrivateLcdStructure.LcdDxPins[LCD_D6]);

	for(uint8_t X = 0; X<3; ++X)
		lcdClock();
	
	if(PrivateLcdStructure.LcdBusWidth==LCD_BUS_4)
	{
			LL_GPIO_ResetOutputPin(PrivateLcdStructure.LcdDxPorts[LCD_D4], PrivateLcdStructure.LcdDxPins[LCD_D4]);					
			lcdClock();
		
			lcdClock();
			
			LL_GPIO_SetOutputPin(PrivateLcdStructure.LcdDxPorts[LCD_D7], PrivateLcdStructure.LcdDxPins[LCD_D7]);				
			LL_GPIO_ResetOutputPin(PrivateLcdStructure.LcdDxPorts[LCD_D5], PrivateLcdStructure.LcdDxPins[LCD_D5]);	
				
			lcdClock();
	}
	else
	{		
			LL_GPIO_ResetOutputPin(PrivateLcdStructure.LcdDxPorts[LCD_D4], PrivateLcdStructure.LcdDxPins[LCD_D2]);	
			LL_GPIO_ResetOutputPin(PrivateLcdStructure.LcdDxPorts[LCD_D4], PrivateLcdStructure.LcdDxPins[LCD_D1]);	
			LL_GPIO_ResetOutputPin(PrivateLcdStructure.LcdDxPorts[LCD_D4], PrivateLcdStructure.LcdDxPins[LCD_D0]);			
			lcdClock();
	}
	
	lcdCommand(_BV(LCD_DISPLAYMODE));
	lcdClr();	
	lcdCommand(_BV(LCD_ENTRY_MODE) | _BV(LCD_ENTRY_INC));	
	lcdCommand(_BV(LCD_DISPLAYMODE) | _BV(LCD_DISPLAYMODE_ON  ) | _BV(LCD_DISPLAYMODE_CURSOR));		
}


void lcdPuts(const char * String)
{
	while(*String!=0)
	{
		lcdPutc(*String++);		
	}
}

void delayMs(volatile uint32_t Milisec)
{
	double TickPerMsc = SystemCoreClock/20000;
	Milisec =  Milisec * TickPerMsc - 20;
	while(Milisec--);		
}

void delayUs(volatile uint32_t Us)
{
	double TickPerMsc = SystemCoreClock/20000;
	TickPerMsc/=1000;
	
	if((Us * TickPerMsc) - 20 < 0)
		return;
		
	Us =  Us * TickPerMsc - 20;
		while(Us--);		
}

void delayNs(volatile uint32_t Ns)
{
	double TickPerMsc = SystemCoreClock/20000;
	TickPerMsc/=1000000;
	
	if((Ns * TickPerMsc) - 20 < 0)
		return;
	
	Ns =  (Ns * TickPerMsc) - 20;
	
	while(Ns--);		
}

void lcdClock(void)
{
		LL_GPIO_SetOutputPin(PrivateLcdStructure.LcdControlPorts[LCD_E], PrivateLcdStructure.LcdControlPins[LCD_E]);
		delayUs(30);		
		LL_GPIO_ResetOutputPin(PrivateLcdStructure.LcdControlPorts[LCD_E], PrivateLcdStructure.LcdControlPins[LCD_E]);
		delayUs(30);
}

void lcdWrite(uint8_t Data, uint8_t Rs)
{
	LL_GPIO_ResetOutputPin(PrivateLcdStructure.LcdControlPorts[LCD_RW], PrivateLcdStructure.LcdControlPins[LCD_RW]);
	
	if(Rs==LCD_COMMAND)			
		LL_GPIO_ResetOutputPin(PrivateLcdStructure.LcdControlPorts[LCD_RS], PrivateLcdStructure.LcdControlPins[LCD_RS]);
	else //if(Rs==LCD_DATA)
		LL_GPIO_SetOutputPin(PrivateLcdStructure.LcdControlPorts[LCD_RS], PrivateLcdStructure.LcdControlPins[LCD_RS]);
	
	if(PrivateLcdStructure.LcdBusWidth==LCD_BUS_4)
	{
		uint8_t Tmp = Data>>4;
		for(uint8_t Y = 0; Y<2; ++Y)
		{
			for(uint8_t X = 0; X<4; ++X)
			{
				if(Tmp & (1<<X))			
					LL_GPIO_SetOutputPin(PrivateLcdStructure.LcdDxPorts[LCD_D4+X], PrivateLcdStructure.LcdDxPins[LCD_D4+X]);	 
				else
					LL_GPIO_ResetOutputPin(PrivateLcdStructure.LcdDxPorts[LCD_D4+X], PrivateLcdStructure.LcdDxPins[LCD_D4+X]);				
			}			
			lcdClock();
			Tmp = Data & 15;
		}		
	}
	else
	{
		for(uint8_t X = 0; X<8; ++X)
			{
				if(Data & (1<<X))			
					LL_GPIO_SetOutputPin(PrivateLcdStructure.LcdDxPorts[LCD_D0+X], PrivateLcdStructure.LcdDxPins[LCD_D0+X]);	
				else
					LL_GPIO_ResetOutputPin(PrivateLcdStructure.LcdDxPorts[LCD_D0+X], PrivateLcdStructure.LcdDxPins[LCD_D0+X]);				
			}
			
		lcdClock();
	}	
}



