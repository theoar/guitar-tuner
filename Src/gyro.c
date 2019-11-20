#include "gyro.h"
#include "leds.h"

#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"


float GyroCurrentSensitivity;

void initGyroSpi(void)
{
	LL_SPI_InitTypeDef SPI_InitStruct;
  LL_GPIO_InitTypeDef GPIO_InitStruct;
	LL_GPIO_InitTypeDef GPIO_NssPin;

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
  
  /**SPI1 GPIO Configuration  
  PA5   ------> SPI1_SCK
  PA7   ------> SPI1_MOSI
  PA6   ------> SPI1_MISO 
  */
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5| LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
	
	/* NSS PIN CONF */
	GPIO_NssPin.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_NssPin.Pin = GYRO_NSS_PIN;
	GPIO_NssPin.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_NssPin.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_NssPin.Pull =  LL_GPIO_PULL_NO;
	GPIO_NssPin.Alternate = LL_GPIO_AF_0;
	LL_GPIO_Init(GYRO_NSS_PORT, &GPIO_NssPin);
	
	LL_GPIO_SetOutputPin(GYRO_NSS_PORT, GYRO_NSS_PIN);		

  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV256;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;	
  LL_SPI_Init(SPI1, &SPI_InitStruct);

	LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);	
	
	LL_SPI_Enable(SPI1);	
}

uint8_t initGyro(GyroSensitivity Sensitivity, GyroFilter Filter)
{	
	uint8_t Tmpreg;
	uint16_t Temp;

	/* Set data */
	Temp = (uint16_t) (LIS3DSH_DATARATE_100 | LIS3DSH_XYZ_ENABLE);
	Temp |= (uint16_t) (LIS3DSH_SERIALINTERFACE_4WIRE | LIS3DSH_SELFTEST_NORMAL);
	
	/* Set sensitivity */
	if (Sensitivity == GyroSensitivity_2G) {
		Temp |= (uint16_t) (LIS3DSH_FULLSCALE_2);
		GyroCurrentSensitivity = LIS3DSH_SENSITIVITY_0_06G;
	} else if (Sensitivity == GyroSensitivity_4G) {
		Temp |= (uint16_t) (LIS3DSH_FULLSCALE_4);
		GyroCurrentSensitivity = LIS3DSH_SENSITIVITY_0_12G;
	} else if (Sensitivity == GyroSensitivity_6G) {
		Temp |= (uint16_t) (LIS3DSH_FULLSCALE_6);
		GyroCurrentSensitivity = LIS3DSH_SENSITIVITY_0_18G;
	} else if (Sensitivity == GyroSensitivity_8G) {
		Temp |= (uint16_t) (LIS3DSH_FULLSCALE_8);
		GyroCurrentSensitivity = LIS3DSH_SENSITIVITY_0_24G;
	} else if (Sensitivity == GyroSensitivity_16G) {
		Temp |= (uint16_t) (LIS3DSH_FULLSCALE_16);
		GyroCurrentSensitivity = LIS3DSH_SENSITIVITY_0_73G;
	} else {
		return 0;
	}
	
	/* Set filter */
	if (Filter == GyroFilter_800Hz) {
		Temp |= (uint16_t) (LIS3DSH_FILTER_BW_800 << 8);
	} else if (Filter == GyroFilter_400Hz) {
		Temp |= (uint16_t) (LIS3DSH_FILTER_BW_400 << 8);
	} else if (Filter == GyroFilter_200Hz) {
		Temp |= (uint16_t) (LIS3DSH_FILTER_BW_200 << 8);
	} else if (Filter == GyroFilter_50Hz) {
		Temp |= (uint16_t) (LIS3DSH_FILTER_BW_50 << 8);
	} else {
		return 0;
	}
	
	/* Configure MEMS: power mode(ODR) and axes enable */
	Tmpreg = (uint8_t) (Temp);

	/* Write value to MEMS CTRL_REG4 register */
	gyroSpiWrite(&Tmpreg, LIS3DSH_CTRL_REG4_ADDR, 1);

	/* Configure MEMS: full scale and self test */
	Tmpreg = (uint8_t) (Temp >> 8);

	/* Write value to MEMS CTRL_REG5 register */
	gyroSpiWrite(&Tmpreg, LIS3DSH_CTRL_REG5_ADDR, 1);
	
	return 1;
	
}


void gyroReadAxes(GyroAxes* DataPtr)
{
	uint8_t Status = 0;	
	
	int8_t Buffer[6];	
	gyroSpiRead((uint8_t*)&Buffer[0], LIS3DSH_OUT_X_L_ADDR, 1);
	gyroSpiRead((uint8_t*)&Buffer[1], LIS3DSH_OUT_X_H_ADDR, 1);
	gyroSpiRead((uint8_t*)&Buffer[2], LIS3DSH_OUT_Y_L_ADDR, 1);
	gyroSpiRead((uint8_t*)&Buffer[3], LIS3DSH_OUT_Y_H_ADDR, 1);
	gyroSpiRead((uint8_t*)&Buffer[4], LIS3DSH_OUT_Z_L_ADDR, 1);
	gyroSpiRead((uint8_t*)&Buffer[5], LIS3DSH_OUT_Z_H_ADDR, 1);
	
	/* Set axes */
	DataPtr->X = (int16_t)((Buffer[1] << 8) + Buffer[0]) * GyroCurrentSensitivity;
	DataPtr->Y = (int16_t)((Buffer[3] << 8) + Buffer[2]) * GyroCurrentSensitivity;
	DataPtr->Z = (int16_t)((Buffer[5] << 8) + Buffer[4]) * GyroCurrentSensitivity;	
	
	DataPtr->X /= 100;
	DataPtr->Y /= 100;
	DataPtr->Z /= 100;	

}

void gyroSpiWrite(uint8_t *Data, uint8_t Addr, uint8_t Count)
{
	LL_GPIO_ResetOutputPin(GYRO_NSS_PORT, GYRO_NSS_PIN);	
	
	while(((SPI1)->SR & (SPI_SR_TXE)) == 0);
	LL_SPI_TransmitData8(SPI1, Addr);		
	while(((SPI1)->SR & (SPI_SR_RXNE)) == 0);
	LL_SPI_ReceiveData8(SPI1);
	
	for(uint8_t X = 0; X<Count; ++X)
	{
		while(((SPI1)->SR & (SPI_SR_TXE)) == 0);
		LL_SPI_TransmitData8(SPI1, Data[X]);		
		while(((SPI1)->SR & (SPI_SR_RXNE)) == 0);
		LL_SPI_ReceiveData8(SPI1);
	}
	
	LL_GPIO_SetOutputPin(GYRO_NSS_PORT, GYRO_NSS_PIN);	
}



void gyroSpiRead(uint8_t *Data, uint8_t Addr, uint8_t Count)
{		
	LL_GPIO_ResetOutputPin(GYRO_NSS_PORT, GYRO_NSS_PIN);
	
	Addr|=0x80;
	
	if(Count>1)
		Addr|=0x40;	

	
	while(((SPI1)->SR & (SPI_SR_TXE)) == 0);
	LL_SPI_TransmitData8(SPI1, Addr);		
	while(((SPI1)->SR & (SPI_SR_RXNE)) == 0);
	LL_SPI_ReceiveData8(SPI1);
	
	for(uint8_t X = 0; X<Count; ++X)
	{
		while(((SPI1)->SR & (SPI_SR_TXE)) == 0);
		LL_SPI_TransmitData8(SPI1, 0x00);
		while(((SPI1)->SR & (SPI_SR_RXNE)) == 0);		
		Data[X] = LL_SPI_ReceiveData8(SPI1);
	}
	
	LL_GPIO_SetOutputPin(GYRO_NSS_PORT, GYRO_NSS_PIN);	
}

uint8_t gyroStatus(void)
{
	uint8_t Status;
	gyroSpiRead(&Status, LIS3DSH_STATUS_REG_ADDR, 1);
	
	return Status;
}

uint8_t gyroWhoAmI(void)
{
	uint8_t Name;
	gyroSpiRead(&Name, LIS3DSH_WHO_AM_I_ADDR, 1);
	
	return Name;
}