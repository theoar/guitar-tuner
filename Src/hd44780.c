/*****************************************************************************
 Title  :   HD44780 Library
 Author :   SA Development
 Version:   1.11
 *****************************************************************************/

#include "stm32f4xx_ll_bus.h"
#include "hd44780.h"

#if !defined(LCD_BITS) || (LCD_BITS!=4 && LCD_BITS!=8)
#error LCD_BITS is not defined or not valid.
#endif

#if !defined(WAIT_MODE) || (WAIT_MODE!=0 && WAIT_MODE!=1)
#error WAIT_MODE is not defined or not valid.
#endif

#if !defined(RW_LINE_IMPLEMENTED) || (RW_LINE_IMPLEMENTED!=0 && RW_LINE_IMPLEMENTED!=1)
#error RW_LINE_IMPLEMENTED is not defined or not valid.
#endif

#if (WAIT_MODE==1 && RW_LINE_IMPLEMENTED!=1)
#error WAIT_MODE=1 requires RW_LINE_IMPLEMENTED=1.
#endif

#if !defined(LCD_DISPLAYS) || (LCD_DISPLAYS<1) || (LCD_DISPLAYS>4)
#error LCD_DISPLAYS is not defined or not valid.
#endif


//PORT defines
#define lcd_rs_port_low() LL_GPIO_ResetOutputPin(LCD_RS_PORT,LCD_RS_PIN)
#if RW_LINE_IMPLEMENTED==1
#define lcd_rw_port_low() LL_GPIO_ResetOutputPin(LCD_RW_PORT,LCD_RW_PIN)
#endif
#define lcd_db0_port_low() LL_GPIO_ResetOutputPin(LCD_DB0_PORT,LCD_DB0_PIN)
#define lcd_db1_port_low() LL_GPIO_ResetOutputPin(LCD_DB1_PORT,LCD_DB1_PIN)
#define lcd_db2_port_low() LL_GPIO_ResetOutputPin(LCD_DB2_PORT,LCD_DB2_PIN)
#define lcd_db3_port_low() LL_GPIO_ResetOutputPin(LCD_DB3_PORT,LCD_DB3_PIN)
#define lcd_db4_port_low() LL_GPIO_ResetOutputPin(LCD_DB4_PORT,LCD_DB4_PIN)
#define lcd_db5_port_low() LL_GPIO_ResetOutputPin(LCD_DB5_PORT,LCD_DB5_PIN)
#define lcd_db6_port_low() LL_GPIO_ResetOutputPin(LCD_DB6_PORT,LCD_DB6_PIN)
#define lcd_db7_port_low() LL_GPIO_ResetOutputPin(LCD_DB7_PORT,LCD_DB7_PIN)

#define lcd_rs_port_high() LL_GPIO_SetOutputPin(LCD_RS_PORT,LCD_RS_PIN)
#if RW_LINE_IMPLEMENTED==1
#define lcd_rw_port_high() LL_GPIO_SetOutputPin(LCD_RW_PORT,LCD_RW_PIN)
#endif
#define lcd_db0_port_high() LL_GPIO_SetOutputPin(LCD_DB0_PORT,LCD_DB0_PIN)
#define lcd_db1_port_high() LL_GPIO_SetOutputPin(LCD_DB1_PORT,LCD_DB1_PIN)
#define lcd_db2_port_high() LL_GPIO_SetOutputPin(LCD_DB2_PORT,LCD_DB2_PIN)
#define lcd_db3_port_high() LL_GPIO_SetOutputPin(LCD_DB3_PORT,LCD_DB3_PIN)
#define lcd_db4_port_high() LL_GPIO_SetOutputPin(LCD_DB4_PORT,LCD_DB4_PIN)
#define lcd_db5_port_high() LL_GPIO_SetOutputPin(LCD_DB5_PORT,LCD_DB5_PIN)
#define lcd_db6_port_high() LL_GPIO_SetOutputPin(LCD_DB6_PORT,LCD_DB6_PIN)
#define lcd_db7_port_high() LL_GPIO_SetOutputPin(LCD_DB7_PORT,LCD_DB7_PIN)

#define lcd_rs_port_set(value) if (value) lcd_rs_port_high(); else lcd_rs_port_low();
#if RW_LINE_IMPLEMENTED==1
#define lcd_rw_port_set(value) if (value) lcd_rw_port_high(); else lcd_rw_port_low();
#endif
#define lcd_db0_port_set(value) if (value) lcd_db0_port_high(); else lcd_db0_port_low();
#define lcd_db1_port_set(value) if (value) lcd_db1_port_high(); else lcd_db1_port_low();
#define lcd_db2_port_set(value) if (value) lcd_db2_port_high(); else lcd_db2_port_low();
#define lcd_db3_port_set(value) if (value) lcd_db3_port_high(); else lcd_db3_port_low();
#define lcd_db4_port_set(value) if (value) lcd_db4_port_high(); else lcd_db4_port_low();
#define lcd_db5_port_set(value) if (value) lcd_db5_port_high(); else lcd_db5_port_low();
#define lcd_db6_port_set(value) if (value) lcd_db6_port_high(); else lcd_db6_port_low();
#define lcd_db7_port_set(value) if (value) lcd_db7_port_high(); else lcd_db7_port_low();

//PIN defines
#define lcd_db0_pin_get() (((LL_GPIO_IsInputPinSet(LCD_DB0_PORT,LCD_DB0_PIN))==0)?0:1)
#define lcd_db1_pin_get() (((LL_GPIO_IsInputPinSet(LCD_DB1_PORT,LCD_DB1_PIN))==0)?0:1)
#define lcd_db2_pin_get() (((LL_GPIO_IsInputPinSet(LCD_DB2_PORT,LCD_DB2_PIN))==0)?0:1)
#define lcd_db3_pin_get() (((LL_GPIO_IsInputPinSet(LCD_DB3_PORT,LCD_DB3_PIN))==0)?0:1)
#define lcd_db4_pin_get() (((LL_GPIO_IsInputPinSet(LCD_DB4_PORT,LCD_DB4_PIN))==0)?0:1)
#define lcd_db5_pin_get() (((LL_GPIO_IsInputPinSet(LCD_DB5_PORT,LCD_DB5_PIN))==0)?0:1)
#define lcd_db6_pin_get() (((LL_GPIO_IsInputPinSet(LCD_DB6_PORT,LCD_DB6_PIN))==0)?0:1)
#define lcd_db7_pin_get() (((LL_GPIO_IsInputPinSet(LCD_DB7_PORT,LCD_DB7_PIN))==0)?0:1)

//DDR defines
#define lcd_rs_ddr_low() LL_GPIO_SetPinMode(LCD_RS_PORT, LCD_RS_PIN, LL_GPIO_MODE_INPUT)
#if RW_LINE_IMPLEMENTED==1
#define lcd_rw_ddr_low() LL_GPIO_SetPinMode(CD_RW_PORT, LCD_RW_PIN, LL_GPIO_MODE_INPUT)
#endif
#define lcd_db0_ddr_low() LL_GPIO_SetPinMode(LCD_DB0_PORT,LCD_DB0_PIN, LL_GPIO_MODE_INPUT)
#define lcd_db1_ddr_low() LL_GPIO_SetPinMode(LCD_DB1_PORT,LCD_DB1_PIN, LL_GPIO_MODE_INPUT)
#define lcd_db2_ddr_low() LL_GPIO_SetPinMode(LCD_DB2_PORT,LCD_DB2_PIN, LL_GPIO_MODE_INPUT)
#define lcd_db3_ddr_low() LL_GPIO_SetPinMode(LCD_DB3_PORT,LCD_DB3_PIN, LL_GPIO_MODE_INPUT)
#define lcd_db4_ddr_low() LL_GPIO_SetPinMode(LCD_DB4_PORT,LCD_DB4_PIN, LL_GPIO_MODE_INPUT)
#define lcd_db5_ddr_low() LL_GPIO_SetPinMode(LCD_DB5_PORT,LCD_DB5_PIN, LL_GPIO_MODE_INPUT)
#define lcd_db6_ddr_low() LL_GPIO_SetPinMode(LCD_DB6_PORT,LCD_DB6_PIN, LL_GPIO_MODE_INPUT)
#define lcd_db7_ddr_low() LL_GPIO_SetPinMode(LCD_DB7_PORT,LCD_DB7_PIN, LL_GPIO_MODE_INPUT)

#define lcd_rs_ddr_high() LL_GPIO_SetPinMode(LCD_RS_PORT,LCD_RS_PIN,LL_GPIO_MODE_OUTPUT)
#if RW_LINE_IMPLEMENTED==1
#define lcd_rw_ddr_high() LL_GPIO_SetPinMode(LCD_RW_PORT,LCD_RW_PIN,LL_GPIO_MODE_OUTPUT)
#endif
#define lcd_db0_ddr_high() LL_GPIO_SetPinMode(LCD_DB0_PORT,LCD_DB0_PIN, LL_GPIO_MODE_OUTPUT)
#define lcd_db1_ddr_high() LL_GPIO_SetPinMode(LCD_DB1_PORT,LCD_DB1_PIN, LL_GPIO_MODE_OUTPUT)
#define lcd_db2_ddr_high() LL_GPIO_SetPinMode(LCD_DB2_PORT,LCD_DB2_PIN, LL_GPIO_MODE_OUTPUT)
#define lcd_db3_ddr_high() LL_GPIO_SetPinMode(LCD_DB3_PORT,LCD_DB3_PIN, LL_GPIO_MODE_OUTPUT)
#define lcd_db4_ddr_high() LL_GPIO_SetPinMode(LCD_DB4_PORT,LCD_DB4_PIN, LL_GPIO_MODE_OUTPUT)
#define lcd_db5_ddr_high() LL_GPIO_SetPinMode(LCD_DB5_PORT,LCD_DB5_PIN, LL_GPIO_MODE_OUTPUT)
#define lcd_db6_ddr_high() LL_GPIO_SetPinMode(LCD_DB6_PORT,LCD_DB6_PIN, LL_GPIO_MODE_OUTPUT)
#define lcd_db7_ddr_high() LL_GPIO_SetPinMode(LCD_DB7_PORT,LCD_DB7_PIN, LL_GPIO_MODE_OUTPUT)

#define lcd_rs_ddr_set(value) if (value) lcd_rs_ddr_high(); else lcd_rs_ddr_low();
#if RW_LINE_IMPLEMENTED==1
#define lcd_rw_ddr_set(value) if (value) lcd_rw_ddr_high(); else lcd_rw_ddr_low();
#endif
#define lcd_db0_ddr_set(value) if (value) lcd_db0_ddr_high(); else lcd_db0_ddr_low();
#define lcd_db1_ddr_set(value) if (value) lcd_db1_ddr_high(); else lcd_db1_ddr_low();
#define lcd_db2_ddr_set(value) if (value) lcd_db2_ddr_high(); else lcd_db2_ddr_low();
#define lcd_db3_ddr_set(value) if (value) lcd_db3_ddr_high(); else lcd_db3_ddr_low();
#define lcd_db4_ddr_set(value) if (value) lcd_db4_ddr_high(); else lcd_db4_ddr_low();
#define lcd_db5_ddr_set(value) if (value) lcd_db5_ddr_high(); else lcd_db5_ddr_low();
#define lcd_db6_ddr_set(value) if (value) lcd_db6_ddr_high(); else lcd_db6_ddr_low();
#define lcd_db7_ddr_set(value) if (value) lcd_db7_ddr_high(); else lcd_db7_ddr_low();

#if (WAIT_MODE==1 && RW_LINE_IMPLEMENTED==1)
	static unsigned char PrevCmdInvolvedAddressCounter=0;
#endif

#if (LCD_DISPLAYS>1)
	static unsigned char ActiveDisplay=1;
#endif

	static inline void lcd_e_port_low()
	{
#if (LCD_DISPLAYS>1)
		switch (ActiveDisplay)
		{
			case 2 : LL_GPIO_ResetOutputPin(LCD_E2_PORT,LCD_E2_PIN);
			break;
#if (LCD_DISPLAYS>=3)
			case 3 : LL_GPIO_ResetOutputPin(LCD_E3_PORT,LCD_E3_PIN);
			break;
#endif
#if (LCD_DISPLAYS==4)
			case 4 : LL_GPIO_ResetOutputPin(LCD_E4_PORT,LCD_E4_PIN);
			break;
#endif
			default :
#endif
			LL_GPIO_ResetOutputPin(LCD_E_PORT, LCD_E_PIN);
#if (LCD_DISPLAYS>1)
		}
#endif
	}

	static inline void lcd_e_port_high()
	{
#if (LCD_DISPLAYS>1)
		switch (ActiveDisplay)
		{
			case 2 : LL_GPIO_SetOutputPin(LCD_E2_PORT,LCD_E2_PIN);
			break;
#if (LCD_DISPLAYS>=3)
			case 3 : LL_GPIO_SetOutputPin(LCD_E3_PORT,LCD_E3_PIN);
			break;
#endif
#if (LCD_DISPLAYS==4)
			case 4 : LL_GPIO_SetOutputPin(LCD_E4_PORT,LCD_E4_PIN);
			break;
#endif
			default :
#endif
			LL_GPIO_SetOutputPin(LCD_E_PORT, LCD_E_PIN);
#if (LCD_DISPLAYS>1)
		}
#endif
	}

	static inline void lcd_e_ddr_low()
	{
#if (LCD_DISPLAYS>1)
		switch (ActiveDisplay)
		{
			case 2 : DDR(LCD_E2_PORT)&=~_BV(LCD_E2_PIN);
			break;
#if (LCD_DISPLAYS>=3)
			case 3 : DDR(LCD_E3_PORT)&=~_BV(LCD_E3_PIN);
			break;
#endif
#if (LCD_DISPLAYS==4)
			case 4 : DDR(LCD_E4_PORT)&=~_BV(LCD_E4_PIN);
			break;
#endif
			default :
#endif
			LL_GPIO_ResetOutputPin(LCD_E_PORT, LCD_E_PIN);
#if (LCD_DISPLAYS>1)
		}
#endif
	}

	static inline void lcd_e_ddr_high()
	{
#if (LCD_DISPLAYS>1)
		switch (ActiveDisplay)
		{
			case 2 : DDR(LCD_E2_PORT)|=_BV(LCD_E2_PIN);
			break;
#if (LCD_DISPLAYS>=3)
			case 3 : DDR(LCD_E3_PORT)|=_BV(LCD_E3_PIN);
			break;
#endif
#if (LCD_DISPLAYS==4)
			case 4 : DDR(LCD_E4_PORT)|=_BV(LCD_E4_PIN);
			break;
#endif
			default :
#endif
			LL_GPIO_SetOutputPin(LCD_E_PORT, LCD_E_PIN);
#if (LCD_DISPLAYS>1)
		}
#endif
	}

	/*************************************************************************
	 loops while lcd is busy, returns address counter
	 *************************************************************************/
#if (WAIT_MODE==1 && RW_LINE_IMPLEMENTED==1)
	static uint8_t lcd_read(uint8_t rs);

	static void lcd_waitbusy(void)
	{
		register uint8_t c;
		unsigned int ul1=0;

		while ( ((c=lcd_read(0)) & (1<<LCD_BUSY)) && ul1<((SystemCoreClock/16384>=16)?SystemCoreClock/16384:16)) // Wait Until Busy Flag is Cleared
		ul1++;
	}
#endif

	/*************************************************************************
	 Low-level function to read byte from LCD controller
	 Input:    rs     1: read data
	 0: read busy flag / address counter
	 Returns:  byte read from LCD controller
	 *************************************************************************/
#if RW_LINE_IMPLEMENTED==1
	static uint8_t lcd_read(uint8_t rs)
	{
		uint8_t data;

#if (WAIT_MODE==1 && RW_LINE_IMPLEMENTED==1)
		if (rs)
		lcd_waitbusy();
		if (PrevCmdInvolvedAddressCounter)
		{
			delayUs(5);
			PrevCmdInvolvedAddressCounter=0;
		}
#endif

		if (rs)
		{
			lcd_rs_port_high(); // RS=1: Read Data
#if (WAIT_MODE==1 && RW_LINE_IMPLEMENTED==1)
			PrevCmdInvolvedAddressCounter=1;
#endif
		}
		else lcd_rs_port_low(); // RS=0: Read Busy Flag

		lcd_rw_port_high();// RW=1: Read Mode

#if LCD_BITS==4
		lcd_db7_ddr_low(); // Configure Data Pins as Input
		lcd_db6_ddr_low();
		lcd_db5_ddr_low();
		lcd_db4_ddr_low();

		lcd_e_port_high();// Read High Nibble First
		delayNs(500);

		data=lcd_db4_pin_get() << 4 | lcd_db5_pin_get() << 5 |
		lcd_db6_pin_get() << 6 | lcd_db7_pin_get() << 7;

		lcd_e_port_low();
		delayNs(500);

		lcd_e_port_high();// Read Low Nibble
		delayNs(500);

		data|=lcd_db4_pin_get() << 0 | lcd_db5_pin_get() << 1 |
		lcd_db6_pin_get() << 2 | lcd_db7_pin_get() << 3;

		lcd_e_port_low();

		lcd_db7_ddr_high();// Configure Data Pins as Output
		lcd_db6_ddr_high();
		lcd_db5_ddr_high();
		lcd_db4_ddr_high();

		lcd_db7_port_high();// Pins High (Inactive)
		lcd_db6_port_high();
		lcd_db5_port_high();
		lcd_db4_port_high();
#else //using 8-Bit-Mode
		lcd_db7_ddr_low();// Configure Data Pins as Input
		lcd_db6_ddr_low();
		lcd_db5_ddr_low();
		lcd_db4_ddr_low();
		lcd_db3_ddr_low();
		lcd_db2_ddr_low();
		lcd_db1_ddr_low();
		lcd_db0_ddr_low();

		lcd_e_port_high();
		delayNs(500);

		data=lcd_db7_pin_get() << 7 | lcd_db6_pin_get() << 6 |
		lcd_db5_pin_get() << 5 | lcd_db4_pin_get() << 4 |
		lcd_db3_pin_get() << 3 | lcd_db2_pin_get() << 2 |
		lcd_db1_pin_get() << 1 | lcd_db0_pin_get();

		lcd_e_port_low();

		lcd_db7_ddr_high();// Configure Data Pins as Output
		lcd_db6_ddr_high();
		lcd_db5_ddr_high();
		lcd_db4_ddr_high();
		lcd_db3_ddr_high();
		lcd_db2_ddr_high();
		lcd_db1_ddr_high();
		lcd_db0_ddr_high();

		lcd_db7_port_high();// Pins High (Inactive)
		lcd_db6_port_high();
		lcd_db5_port_high();
		lcd_db4_port_high();
		lcd_db3_port_high();
		lcd_db2_port_high();
		lcd_db1_port_high();
		lcd_db0_port_high();
#endif

		lcd_rw_port_low();

#if (WAIT_MODE==0 || RW_LINE_IMPLEMENTED==0)
		if (rs)
		delayUs(40);
		else delayUs(1);
#endif
		return data;
	}

	uint8_t lcd_getc()
	{
		return lcd_read(1);
	}

#endif

	/*************************************************************************
	 Low-level function to write byte to LCD controller
	 Input:    data   byte to write to LCD
	 rs     1: write data
	 0: write instruction
	 Returns:  none
	 *************************************************************************/
	static void lcd_write(uint8_t data,uint8_t rs)
	{
#if (WAIT_MODE==1 && RW_LINE_IMPLEMENTED==1)
		lcd_waitbusy();
		if (PrevCmdInvolvedAddressCounter)
		{
			delayUs(5);
			PrevCmdInvolvedAddressCounter=0;
		}
#endif

		if (rs)
		{
			lcd_rs_port_high(); // RS=1: Write Character
#if (WAIT_MODE==1 && RW_LINE_IMPLEMENTED==1)
			PrevCmdInvolvedAddressCounter=1;
#endif
		}
		else
		{
			lcd_rs_port_low(); // RS=0: Write Command
#if (WAIT_MODE==1 && RW_LINE_IMPLEMENTED==1)
			PrevCmdInvolvedAddressCounter=0;
#endif
		}

#if LCD_BITS==4
		lcd_db7_port_set(data&_BV(7)); //Output High Nibble
		lcd_db6_port_set(data&_BV(6));
		lcd_db5_port_set(data&_BV(5));
		lcd_db4_port_set(data&_BV(4));

		delayNs(100);
		lcd_e_port_high();

		delayNs(500);
		lcd_e_port_low();

		lcd_db7_port_set(data&_BV(3));//Output High Nibble
		lcd_db6_port_set(data&_BV(2));
		lcd_db5_port_set(data&_BV(1));
		lcd_db4_port_set(data&_BV(0));

		delayNs(100);
		lcd_e_port_high();

		delayNs(500);
		lcd_e_port_low();

		lcd_db7_port_high();// All Data Pins High (Inactive)
		lcd_db6_port_high();
		lcd_db5_port_high();
		lcd_db4_port_high();

#else //using 8-Bit_Mode
		lcd_db7_port_set(data&_BV(7));//Output High Nibble
		lcd_db6_port_set(data&_BV(6));
		lcd_db5_port_set(data&_BV(5));
		lcd_db4_port_set(data&_BV(4));
		lcd_db3_port_set(data&_BV(3));//Output High Nibble
		lcd_db2_port_set(data&_BV(2));
		lcd_db1_port_set(data&_BV(1));
		lcd_db0_port_set(data&_BV(0));

		delayNs(100);
		lcd_e_port_high();
		delayNs(500);
		lcd_e_port_low();

		lcd_db7_port_high();// All Data Pins High (Inactive)
		lcd_db6_port_high();
		lcd_db5_port_high();
		lcd_db4_port_high();
		lcd_db3_port_high();
		lcd_db2_port_high();
		lcd_db1_port_high();
		lcd_db0_port_high();
#endif

#if (WAIT_MODE==0 || RW_LINE_IMPLEMENTED==0)
		if (!rs && data<=((1<<LCD_CLR) | (1<<LCD_HOME))) // Is command clrscr or home?
		delayUs(1640);
		else delayUs(40);
#endif
	}

	/*************************************************************************
	 Send LCD controller instruction command
	 Input:   instruction to send to LCD controller, see HD44780 data sheet
	 Returns: none
	 *************************************************************************/
	void lcd_command(uint8_t cmd)
	{
		lcd_write(cmd,0);
	}

	/*************************************************************************
	 Set cursor to specified position
	 Input:    pos position
	 Returns:  none
	 *************************************************************************/
	void lcd_goto(uint8_t pos)
	{
		lcd_command((1<<LCD_DDRAM)+pos);
	}

	/*************************************************************************
		 Set cursor to specified position
		 Input:    x y
		 Returns:  none
		 *************************************************************************/
	void lcd_goto_XY(uint8_t x, uint8_t y)
	{
		lcd_goto(x+0x40*y);
	}

	/*************************************************************************
	 Clear screen
	 Input:    none
	 Returns:  none
	 *************************************************************************/
	void lcd_clrscr()
	{
		lcd_command(1<<LCD_CLR);
	}

	void lcd_clr_row(uint8_t Row)
	{
		lcd_goto_XY(0, Row);
		for(uint8_t X = 0; X<16; X++)
			lcd_putc(' ');
	}

	/*************************************************************************
	 Return home
	 Input:    none
	 Returns:  none
	 *************************************************************************/
	void lcd_home()
	{
		lcd_command(1<<LCD_HOME);
	}

	void lcd_enable_cursor(void)
	{
		lcd_command(_BV(LCD_DISPLAYMODE) | _BV(LCD_DISPLAYMODE_ON) | _BV(LCD_DISPLAYMODE_BLINK));
	}

	void lcd_disable_cursor(void)
	{
		lcd_command(_BV(LCD_DISPLAYMODE) | _BV(LCD_DISPLAYMODE_ON));
	}

	/*************************************************************************
	 Display character
	 Input:    character to be displayed
	 Returns:  none
	 *************************************************************************/
	void lcd_putc(char c)
	{
		lcd_write(c,1);
	}

	/*************************************************************************
	 Display string
	 Input:    string to be displayed
	 Returns:  none
	 *************************************************************************/
	void lcd_puts(const char *s)
	{
		register char c;

		while ((c=*s++))
		lcd_putc(c);
	}

	/*************************************************************************
	 Initialize display
	 Input:    none
	 Returns:  none
	 *************************************************************************/
	void lcd_init()
	{
		//Set All Pins as Output
		lcd_initGPIO();
		lcd_e_ddr_high();
		lcd_rs_ddr_high();
#if RW_LINE_IMPLEMENTED==1
		lcd_rw_ddr_high();
#endif
		lcd_db7_ddr_high();
		lcd_db6_ddr_high();
		lcd_db5_ddr_high();
		lcd_db4_ddr_high();
#if LCD_BITS==8
		lcd_db3_ddr_high();
		lcd_db2_ddr_high();
		lcd_db1_ddr_high();
		lcd_db0_ddr_high();
#endif

		//Set All Control Lines Low
		lcd_e_port_low();
		lcd_rs_port_low();
#if RW_LINE_IMPLEMENTED==1
		lcd_rw_port_low();
#endif

		//Set All Data Lines High
		lcd_db7_port_high();
		lcd_db6_port_high();
		lcd_db5_port_high();
		lcd_db4_port_high();
#if LCD_BITS==8
		lcd_db3_port_high();
		lcd_db2_port_high();
		lcd_db1_port_high();
		lcd_db0_port_high();
#endif

		//Startup Delay
		delayMs(DELAY_RESET);

		//Initialize Display
		lcd_db7_port_low();
		lcd_db6_port_low();
		delayNs(100);
		lcd_e_port_high();
		delayNs(500);
		lcd_e_port_low();

		delayUs(4100);

		lcd_e_port_high();
		delayNs(500);
		lcd_e_port_low();

		delayUs(100);

		lcd_e_port_high();
		delayNs(500);
		lcd_e_port_low();

		delayUs(40);

		//Init differs between 4-bit and 8-bit from here
#if (LCD_BITS==4)
		lcd_db4_port_low();
		delayNs(100);
		lcd_e_port_high();
		delayNs(500);
		lcd_e_port_low();
		delayUs(40);

		lcd_db4_port_low();
		delayNs(100);
		lcd_e_port_high();
		delayNs(500);
		lcd_e_port_low();
		delayNs(500);

#if (LCD_DISPLAYS==1)
		if (LCD_DISPLAY_LINES>1)
		lcd_db7_port_high();
#else
		unsigned char c;
		switch (ActiveDisplay)
		{
			case 1 : c=LCD_DISPLAY_LINES; break;
			case 2 : c=LCD_DISPLAY2_LINES; break;
#if (LCD_DISPLAYS>=3)
			case 3 : c=LCD_DISPLAY3_LINES; break;
#endif
#if (LCD_DISPLAYS==4)
			case 4 : c=LCD_DISPLAY4_LINES; break;
#endif
		}
		if (c>1)
		lcd_db7_port_high();
#endif

		delayNs(100);
		lcd_e_port_high();
		delayNs(500);
		lcd_e_port_low();
		delayUs(40);
#else
#if (LCD_DISPLAYS==1)
		if (LCD_DISPLAY_LINES<2)
		lcd_db3_port_low();
#else
		unsigned char c;
		switch (ActiveDisplay)
		{
			case 1 : c=LCD_DISPLAY_LINES; break;
			case 2 : c=LCD_DISPLAY2_LINES; break;
#if (LCD_DISPLAYS>=3)
			case 3 : c=LCD_DISPLAY3_LINES; break;
#endif
#if (LCD_DISPLAYS==4)
			case 4 : c=LCD_DISPLAY4_LINES; break;
#endif
		}
		if (c<2)
		lcd_db3_port_low();
#endif

		lcd_db2_port_low();
		delayNs(100);
		lcd_e_port_high();
		delayNs(500);
		lcd_e_port_low();
		delayUs(40);
#endif

		//Display Off
		lcd_command(_BV(LCD_DISPLAYMODE));

		//Display Clear
		lcd_clrscr();

		//Entry Mode Set
		lcd_command(_BV(LCD_ENTRY_MODE) | _BV(LCD_ENTRY_INC));

		//Display On
		lcd_command(_BV(LCD_DISPLAYMODE) | _BV(LCD_DISPLAYMODE_ON  ));
		//lcd_command(_BV(LCD_DISPLAYMODE) | _BV(LCD_DISPLAYMODE_BLINK));
	}

#if (LCD_DISPLAYS>1)
	void lcd_use_display(int ADisplay)
	{
		if (ADisplay>=1 && ADisplay<=LCD_DISPLAYS)
		ActiveDisplay=ADisplay;
	}
#endif
	
void delayMs(volatile uint32_t Milisec)
{
	double TickPerMsc = SystemCoreClock/20000;
	Milisec =  Milisec * TickPerMsc/ - 20;
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

void lcd_initGPIO()
{
	GPIO_TypeDef *Porty[7] ={LCD_DB7_PORT, LCD_DB6_PORT, LCD_DB5_PORT, LCD_DB4_PORT, LCD_RS_PORT, LCD_RW_PORT, LCD_E_PORT};
	uint32_t Piny[7] ={LCD_DB7_PIN, LCD_DB6_PIN, LCD_DB5_PIN, LCD_DB4_PIN, LCD_RS_PIN, LCD_RW_PIN, LCD_E_PIN};
	
	GPIO_TypeDef *Oryginal[5] = { GPIOA, GPIOB, GPIOC, GPIOD, GPIOE };
	uint32_t Addr[5] = { LL_AHB1_GRP1_PERIPH_GPIOA, LL_AHB1_GRP1_PERIPH_GPIOB, LL_AHB1_GRP1_PERIPH_GPIOC, LL_AHB1_GRP1_PERIPH_GPIOD, LL_AHB1_GRP1_PERIPH_GPIOE};
	
	LL_GPIO_InitTypeDef InitGpio;
	InitGpio.Mode = LL_GPIO_MODE_OUTPUT;
	InitGpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	InitGpio.Pull = LL_GPIO_PULL_NO;
	InitGpio.Speed = LL_GPIO_SPEED_FREQ_LOW;

	for(uint8_t X = 0; X<7; ++X)
	{
		for(uint8_t Y = 0; Y<5; ++Y)
		{
			if(Porty[X]==Oryginal[Y])
			{	
				LL_AHB1_GRP1_EnableClock(Addr[Y]);
				InitGpio.Pin  = Piny[X];
				LL_GPIO_Init(Porty[X], &InitGpio);
				break;
			}				
		}
	}
}
