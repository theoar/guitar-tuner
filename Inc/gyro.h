#ifndef GYRO_H
#define GYRO_H

#include <inttypes.h>
#include "stm32f4xx_ll_gpio.h"

#define GYRO_NSS_PIN LL_GPIO_PIN_3
#define GYRO_NSS_PORT GPIOE

/* ----------------------------------------- */
/* LIS3DSH registers */
/* ----------------------------------------- */

#define LIS3DSH_ID							0x3F
#define LIS302DL_ID							0x3B

#define LIS3DSH_WHO_AM_I_ADDR				0x0F
#define LIS3DSH_CTRL_REG4_ADDR				0x20
#define LIS3DSH_CTRL_REG1_ADDR				0x21
#define LIS3DSH_CTRL_REG2_ADDR				0x22
#define LIS3DSH_CTRL_REG3_ADDR				0x23
#define LIS3DSH_CTRL_REG5_ADDR				0x24
#define LIS3DSH_CTRL_REG6_ADDR				0x25
#define LIS3DSH_OUT_X_L_ADDR				0x28
#define LIS3DSH_OUT_X_H_ADDR				0x29
#define LIS3DSH_OUT_Y_L_ADDR				0x2A
#define LIS3DSH_OUT_Y_H_ADDR				0x2B
#define LIS3DSH_OUT_Z_L_ADDR				0x2C
#define LIS3DSH_OUT_Z_H_ADDR				0x2D
#define LIS3DSH_STATUS_REG_ADDR			0x27

#define LIS3DSH_SENSITIVITY_0_06G                 0.06  /* 0.06 mg/digit*/
#define LIS3DSH_SENSITIVITY_0_12G       0.12  /* 0.12 mg/digit*/
#define LIS3DSH_SENSITIVITY_0_18G            0.18  /* 0.18 mg/digit*/
#define LIS3DSH_SENSITIVITY_0_24G            0.24  /* 0.24 mg/digit*/
#define LIS3DSH_SENSITIVITY_0_73G            0.73  /* 0.73 mg/digit*/

#define LIS3DSH_DATARATE_100				((uint8_t)0x60)

#define LIS3DSH_FULLSCALE_2					((uint8_t)0x00)  /* 2 g  */
#define LIS3DSH_FULLSCALE_4					((uint8_t)0x08)  /* 4 g  */
#define LIS3DSH_FULLSCALE_6					((uint8_t)0x10)  /* 6 g  */
#define LIS3DSH_FULLSCALE_8					((uint8_t)0x18)  /* 8 g  */
#define LIS3DSH_FULLSCALE_16				((uint8_t)0x20)  /* 16 g */
#define LIS3DSH__FULLSCALE_SELECTION		((uint8_t)0x38)

#define LIS3DSH_FILTER_BW_800				((uint8_t)0x00)  /* 800 Hz */
#define LIS3DSH_FILTER_BW_400				((uint8_t)0x40)//((uint8_t)0x08) /* 400 Hz  */
#define LIS3DSH_FILTER_BW_200				((uint8_t)0x80)//((uint8_t)0x10)  /* 200 Hz */
#define LIS3DSH_FILTER_BW_50				((uint8_t)(0x80 | 0x40))//((uint8_t)0x18)  /* 50 Hz  */
#define LIS3DSH_SELFTEST_NORMAL				((uint8_t)0x00)
#define LIS3DSH_XYZ_ENABLE					((uint8_t)0x07)
#define LIS3DSH_SERIALINTERFACE_4WIRE		((uint8_t)0x00)
#define LIS3DSH_SM_ENABLE					((uint8_t)0x01)
#define LIS3DSH_SM_DISABLE					((uint8_t)0x00)


#define SPI_IS_BUSY(SPIx) (((SPIx)->SR & (SPI_SR_TXE | SPI_SR_RXNE)) == 0 || ((SPIx)->SR & SPI_SR_BSY))
#define SPI_WAIT(SPIx)            while (SPI_IS_BUSY(SPIx))


typedef enum {
	
	GyroSensitivity_2G,
	GyroSensitivity_4G,
	GyroSensitivity_6G,
	GyroSensitivity_8G,
	GyroSensitivity_16G,
	
} GyroSensitivity;;

typedef enum {	
	
	GyroFilter_800Hz,
	GyroFilter_400Hz,
	GyroFilter_200Hz,
	GyroFilter_50Hz,
	
} GyroFilter;

typedef enum{
	GyroStatus_ZYXOR = (1<<7),
	GyroStatus_ZOR = (1<<6),
	GyroStatus_YOR = (1<<5),
	GyroStatus_XOR = (1<<4),
	GyroStatus_ZYXDA = (1<<3),
	GyroStatus_ZDA = (1<<2),
	GyroStatus_YDA = (1<<1),
	GyroStatus_XDA = (1),
} GyroStatus;


typedef struct{
	float X;
	float Y;
	float Z;
} GyroAxes;



uint8_t initGyro(GyroSensitivity Sensitivity, GyroFilter Filter);

void initGyroSpi(void);
void gyroReadAxes(GyroAxes* DataPtr);

void gyroSpiWrite(uint8_t *Data, uint8_t Addr, uint8_t Count);
void gyroSpiRead(uint8_t *Data, uint8_t Addr, uint8_t Count);

uint8_t gyroStatus(void);

uint8_t gyroWhoAmI(void);


#endif