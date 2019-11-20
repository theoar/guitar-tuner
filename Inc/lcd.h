#ifndef LCD_H
#define LCD_H

#include <inttypes.h>
#include "stm32f4xx_ll_gpio.h"

#define LCD_BUS_4 4
#define LCD_BUS_8 8
#define LCD_RW_ENABLE 1
#define LCD_RW_DISABLE 0
#define LCD_WAIT_MODE_FLAG 1
#define LCD_WAIT_MODE_DELAY 0

#define LCD_RW 0
#define LCD_RS 1
#define LCD_E 2

#define LCD_D0 0
#define LCD_D1 1
#define LCD_D2 2
#define LCD_D3 3
#define LCD_D4 4
#define LCD_D5 5
#define LCD_D6 6
#define LCD_D7 7

#define DELAY_RESET 100

#define _BV(bit) (1 << (bit)) 

#define LCD_CLR                 0    // DB0: clear display

#define LCD_HOME                1    // DB1: return to home position

#define LCD_ENTRY_MODE          2    // DB2: set entry mode
#define LCD_ENTRY_INC           1    // DB1: 1=increment, 0=decrement
#define LCD_ENTRY_SHIFT         0    // DB0: 1=display shift on

#define LCD_DISPLAYMODE         3    // DB3: turn lcd/cursor on
#define LCD_DISPLAYMODE_ON      2    // DB2: turn display on
#define LCD_DISPLAYMODE_CURSOR  1    // DB1: turn cursor on
#define LCD_DISPLAYMODE_BLINK   0    // DB0: blinking cursor

#define LCD_MOVE                4    // DB4: move cursor/display
#define LCD_MOVE_DISP           3    // DB3: move display (0-> cursor)
#define LCD_MOVE_RIGHT          2    // DB2: move right (0-> left)

#define LCD_FUNCTION            5    // DB5: function set
#define LCD_FUNCTION_8BIT       4    // DB4: set 8BIT mode (0->4BIT mode)
#define LCD_FUNCTION_2LINES     3    // DB3: two lines (0->one line)
#define LCD_FUNCTION_10DOTS     2    // DB2: 5x10 font (0->5x7 font)

#define LCD_CGRAM               6    // DB6: set CG RAM address
#define LCD_DDRAM               7    // DB7: set DD RAM address

#define LCD_BUSY                7    // DB7: LCD is busy

#define LCD_COMMAND							1
#define LCD_DATA								0

typedef struct{	
	uint16_t LcdDxPins[8];
	GPIO_TypeDef *LcdDxPorts[8];
	
	uint16_t LcdControlPins[3];
	GPIO_TypeDef *LcdControlPorts[3];
	
	uint8_t LcdBusWidth;
	uint8_t LcdRwLine;
	uint8_t LcdWaitMode;
	
} LcdConfigStructure;

extern LcdConfigStructure PrivateLcdStructure;

LcdConfigStructure *lcdStructure();

void lcdSetupDataPin(uint8_t LcdPin, uint16_t GpioPin, GPIO_TypeDef* GpioPort);
void lcdSetupControlPin(uint8_t LcdPin, uint16_t GpioPin, GPIO_TypeDef* GpioPort);
void lcdSetupBusWidth(uint8_t Width);
void lcdSetupRwLine(uint8_t RwEnableDisable);
void lcdSetupWaitMode(uint8_t WaitMode);

void lcdInit(void);

void lcdCommand(uint8_t Cmd);
void lcdClr(void);
void lcdPutc(unsigned char C);
void lcdPuts(const char * String);
void lcdGoTo(uint8_t X, uint8_t Y);

void delayMs(volatile uint32_t Milisec);
void delayUs(volatile uint32_t Us);
void delayNs(volatile uint32_t Ns);

void lcdWrite(uint8_t Data, uint8_t Rs);
void lcdClock(void);


#endif