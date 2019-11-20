
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"


//static void MX_RTC_Init(void);

static void initSystem();

#define DESIRED_FREQ LL_I2S_AUDIOFREQ_16K
#define DECIMATOR_FACTOR 64
#define PROBING_FREQ (DESIRED_FREQ*DECIMATOR_FACTOR)
#define DATA_FORMAT LL_I2S_DATAFORMAT_16B

#if(DATA_FORMAT==LL_I2S_DATAFORMAT_16B)
	#define SAMPLE_LEN 16
#endif
#if(DATA_FORMAT==LL_I2S_DATAFORMAT_32B)
		#define SAMPLE_LEN 32
#endif

#define FREQ_SETTING (PROBING_FREQ/(SAMPLE_LEN*2))

#define PROBING_TIME 128

#ifdef PROBES_NUMBER
	#define BSIZE PROBES_NUMBER
#endif

#ifdef PROBING_TIME
	#define BSIZE ( ((PROBING_TIME*PROBING_FREQ)/1000)/SAMPLE_LEN )	
#endif

uint16_t Buffer[BSIZE];
uint16_t Filtered[(BSIZE/DECIMATOR_FACTOR)*2];
float FftOutput[(BSIZE/DECIMATOR_FACTOR)];
volatile float32_t *InputBuff;

PDM_Filter_Handler_t Handler;
PDM_Filter_Config_t Config;

arm_rfft_instance_f32 FftInstance;
arm_cfft_radix4_instance_f32 RadixInstance;

void DMA1_Stream3_IRQHandler(void)
{	
	LL_DMA_ClearFlag_TC3(DMA1);	
	LL_I2S_Disable(SPI2);		
	
	PDM_Filter((void*)Buffer, (void*)Filtered, &Handler);		
	
	for(uint32_t X = (BSIZE/DECIMATOR_FACTOR), Y = 1; 0<X; --X, ++Y)	
	{			
		Filtered[(BSIZE/DECIMATOR_FACTOR)*2-2*Y]=Filtered[X-1];
		Filtered[(BSIZE/DECIMATOR_FACTOR)*2-2*Y+1]=0;			
	}
		
	InputBuff = (float32_t*)Filtered;	
	
	float32_t Bff;
	for(uint32_t X = 0; X<(BSIZE/DECIMATOR_FACTOR); ++X)	
	{		
		Bff = (float32_t)Filtered[X*2];
		memcpy(Filtered+X*2, &Bff, 4);
	}
	
}

void SPI2_IRQHandler(void)
{
	lcdPuts("Run");
	LL_I2S_ClearFlag_OVR(SPI2);
}

void initI2S(void)
{	
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
	
	LL_I2S_Disable(SPI2);
	
	LL_GPIO_InitTypeDef I2SPins;
	I2SPins.Alternate = LL_GPIO_AF_5;
	I2SPins.Mode = LL_GPIO_MODE_ALTERNATE;
	I2SPins.OutputType = LL_GPIO_OUTPUT_PUSHPULL;	
	I2SPins.Pull = LL_GPIO_PULL_NO;
	I2SPins.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;	
	
	I2SPins.Pin = LL_GPIO_PIN_10;
	LL_GPIO_Init(GPIOB, &I2SPins);
	
	I2SPins.Pin = LL_GPIO_PIN_3;	
	LL_GPIO_Init(GPIOC, &I2SPins);	

	LL_I2S_DeInit(SPI2);

	LL_I2S_InitTypeDef I2SDef;
	I2SDef.AudioFreq = FREQ_SETTING;
	I2SDef.ClockPolarity = LL_I2S_POLARITY_HIGH;
	I2SDef.DataFormat = DATA_FORMAT;
	I2SDef.MCLKOutput = LL_I2S_MCLK_OUTPUT_DISABLE;
	I2SDef.Mode = LL_I2S_MODE_MASTER_RX;
	I2SDef.Standard = LL_I2S_STANDARD_MSB;	
	
	NVIC_SetPriority(SPI2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	NVIC_EnableIRQ(SPI2_IRQn);		
	LL_I2S_EnableIT_ERR(SPI2);	
		
	LL_I2S_Init(SPI2, &I2SDef);		
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	
	LL_DMA_InitTypeDef DmaConf;
	LL_DMA_StructInit(&DmaConf);
	DmaConf.Channel=LL_DMA_CHANNEL_0;
	DmaConf.Direction=LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
	DmaConf.FIFOMode=LL_DMA_FIFOMODE_DISABLE;
	DmaConf.MemoryOrM2MDstAddress = (uint32_t)Buffer;
	DmaConf.MemoryOrM2MDstDataSize = LL_DMA_PDATAALIGN_HALFWORD;
	DmaConf.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;	
	DmaConf.MemBurst=LL_DMA_MBURST_INC4;
	DmaConf.FIFOThreshold=LL_DMA_FIFOTHRESHOLD_1_2;
	DmaConf.Mode = LL_DMA_MODE_NORMAL;
	DmaConf.NbData = BSIZE;
	DmaConf.PeriphOrM2MSrcAddress = (uint32_t)&(SPI2->DR);
	DmaConf.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
	DmaConf.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
	DmaConf.PeriphBurst = LL_DMA_PBURST_SINGLE;
	DmaConf.Priority = LL_DMA_PRIORITY_HIGH;	
	DmaConf.FIFOMode = LL_DMA_FIFOMODE_ENABLE;
	
	LL_DMA_Init(DMA1, LL_DMA_STREAM_3, &DmaConf);
	
	NVIC_SetPriority(DMA1_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	NVIC_EnableIRQ(DMA1_Stream3_IRQn);	
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_3);	
	
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_3);	
	LL_I2S_EnableDMAReq_RX(SPI2);				
	
	Handler.bit_order = PDM_FILTER_BIT_ORDER_LSB;
	Handler.endianness = PDM_FILTER_ENDIANNESS_BE;
	Handler.high_pass_tap = 2122358088;
	Handler.out_ptr_channels = 1;
	Handler.in_ptr_channels = 1;
	PDM_Filter_Init(&Handler);
	
	Config.decimation_factor = PDM_FILTER_DEC_FACTOR_64;
	Config.mic_gain = 24;
	Config.output_samples_number = BSIZE/DECIMATOR_FACTOR;
		
	arm_rfft_init_f32(&FftInstance, &RadixInstance, 128, 0,1);
		
	PDM_Filter_setConfig(&Handler, &Config);
	
	LL_I2S_Enable(SPI2);	
}



GyroAxes AxeRead;

uint16_t Data ;
volatile uint32_t Freq;

char DispBuff[32];
int main(void)
{
	initSystem();	
	initLeds();
	initGyroSpi();	
	
	delayMs(500);		
	
	lcdSetupDataPin(LCD_D7,  LL_GPIO_PIN_0, GPIOD);
	lcdSetupDataPin(LCD_D6,  LL_GPIO_PIN_1, GPIOD);	
	lcdSetupDataPin(LCD_D5,  LL_GPIO_PIN_2, GPIOD);	
	lcdSetupDataPin(LCD_D4,  LL_GPIO_PIN_3, GPIOD);		
	lcdSetupControlPin(LCD_E,  LL_GPIO_PIN_6, GPIOD);
	lcdSetupControlPin(LCD_RW,  LL_GPIO_PIN_7, GPIOB);
	lcdSetupControlPin(LCD_RS,  LL_GPIO_PIN_8, GPIOB);
	
	lcdSetupBusWidth(LCD_BUS_4);
	lcdSetupRwLine(LCD_RW_ENABLE);
	lcdSetupWaitMode(LCD_WAIT_MODE_FLAG);		
	
	lcdInit();		
	initGyro(GyroSensitivity_2G, GyroFilter_200Hz);		
	
	initI2S();	

  while (1)
  {
		if((gyroStatus() & GyroStatus_ZYXDA) != 0)
		{
			gyroReadAxes(&AxeRead);
			
			if(AxeRead.X>0)
				ledOn(RED);
			else
				ledOff(RED);
			if(AxeRead.X<0)
				ledOn(GREEN);
			else
				ledOff(GREEN);
			
			if(AxeRead.Y>0)
				ledOn(ORAGNE);
			else
				ledOff(ORAGNE);
			if(AxeRead.Y<0)
				ledOn(BLUE);
			else
				ledOff(BLUE);
		}
  }
	
}
static void initSystem(void)
{  
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);
	CRC->CR = CRC_CR_RESET;
	

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
	
  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  NVIC_SetPriority(MemoryManagement_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* BusFault_IRQn interrupt configuration */
  NVIC_SetPriority(BusFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* UsageFault_IRQn interrupt configuration */
  NVIC_SetPriority(UsageFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* SVCall_IRQn interrupt configuration */
  NVIC_SetPriority(SVCall_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* DebugMonitor_IRQn interrupt configuration */
  NVIC_SetPriority(DebugMonitor_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* PendSV_IRQn interrupt configuration */
  NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_5)  
		Error_Handler();  
  
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
	
  LL_RCC_HSE_Enable();
   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1);
	
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 336, LL_RCC_PLLP_DIV_2);
	LL_RCC_PLLI2S_ConfigDomain_I2S(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLI2SM_DIV_8, 192, LL_RCC_PLLI2SR_DIV_2);		
	
	LL_RCC_PLL_Enable();
   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1); 
	
	LL_RCC_PLLI2S_Enable();
	/* Wait till PLL is ready */
  while(LL_RCC_PLLI2S_IsReady() != 1);  
  
  LL_RCC_LSI_Enable();
   /* Wait till LSI is ready */
  while(LL_RCC_LSI_IsReady() != 1);      
  
  LL_PWR_EnableBkUpAccess();
  LL_RCC_ForceBackupDomainReset();
  LL_RCC_ReleaseBackupDomainReset();
  LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
  LL_RCC_EnableRTC();
	
  
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);	
	    
  LL_Init1msTick(168000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(168000000);	
	LL_SYSTICK_EnableIT();		
	
	LL_RCC_SetI2SClockSource(LL_RCC_I2S1_CLKSOURCE_PLLI2S);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));	
	
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
 /*
static void MX_RTC_Init(void)
{

  LL_RTC_InitTypeDef RTC_InitStruct;
  LL_RTC_TimeTypeDef RTC_TimeStruct;
  LL_RTC_DateTypeDef RTC_DateStruct;

  LL_RCC_EnableRTC();


  RTC_InitStruct.HourFormat = LL_RTC_HOURFORMAT_24HOUR;
  RTC_InitStruct.AsynchPrescaler = 127;
  RTC_InitStruct.SynchPrescaler = 255;
  LL_RTC_Init(RTC, &RTC_InitStruct);

  LL_RTC_SetAsynchPrescaler(RTC, 127);

  LL_RTC_SetSynchPrescaler(RTC, 255);


  if(LL_RTC_BAK_GetRegister(RTC, LL_RTC_BKP_DR0) != 0x32F2){
  
  RTC_TimeStruct.Hours = 0;
  RTC_TimeStruct.Minutes = 0;
  RTC_TimeStruct.Seconds = 0;
  LL_RTC_TIME_Init(RTC, LL_RTC_FORMAT_BCD, &RTC_TimeStruct);

  RTC_DateStruct.WeekDay = LL_RTC_WEEKDAY_MONDAY;
  RTC_DateStruct.Month = LL_RTC_MONTH_JANUARY;
  RTC_DateStruct.Year = 0;
  LL_RTC_DATE_Init(RTC, LL_RTC_FORMAT_BCD, &RTC_DateStruct);

    LL_RTC_BAK_SetRegister(RTC,LL_RTC_BKP_DR0,0x32F2);
  }

}
*/

void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

