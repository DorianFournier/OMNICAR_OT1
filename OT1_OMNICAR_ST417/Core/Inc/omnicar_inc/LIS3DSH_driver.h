/*
 * LIS3DSH_driver.h
 *
 *  Created on: June 11 2021
 *      Author: TEAM OMNICAR
 */

#ifndef INC_OMNICAR_INC_LIS3DSH_DRIVER_H_
#define INC_OMNICAR_INC_LIS3DSH_DRIVER_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <string.h>

/* LIS3DSH Registers Addresses */
#define LIS3DSH_CTRL_REG4_ADDR      0x20
#define LIS3DSH_CTRL_REG3_ADDR      0x23
#define LIS3DSH_CTRL_REG5_ADDR      0x24

#define LIS3DSH_OUT_X_L_ADDR        0x28
#define LIS3DSH_OUT_X_H_ADDR        0x29
#define LIS3DSH_OUT_Y_L_ADDR        0x2A
#define LIS3DSH_OUT_Y_H_ADDR        0x2B
#define LIS3DSH_OUT_Z_L_ADDR        0x2C
#define LIS3DSH_OUT_Z_H_ADDR        0x2D

/* LIS3DSH Data Rate */
#define LIS3DSH_DATARATE_POWERDOWN        ((uint8_t)0x00) /*   Power Down Mode    */
#define LIS3DSH_DATARATE_3_125            ((uint8_t)0x10) /* 3.125 HZ Normal Mode */
#define LIS3DSH_DATARATE_6_25             ((uint8_t)0x20) /*  6.25 HZ Normal Mode */
#define LIS3DSH_DATARATE_12_5             ((uint8_t)0x30) /*  12.5 HZ Normal Mode */
#define LIS3DSH_DATARATE_25               ((uint8_t)0x40) /*   25 HZ Normal Mode  */
#define LIS3DSH_DATARATE_50               ((uint8_t)0x50) /*   50 HZ Normal Mode  */
#define LIS3DSH_DATARATE_100              ((uint8_t)0x60) /*  100 HZ Normal Mode  */
#define LIS3DSH_DATARATE_400              ((uint8_t)0x70) /*  400 HZ Normal Mode  */
#define LIS3DSH_DATARATE_800              ((uint8_t)0x80) /*  800 HZ Normal Mode  */
#define LIS3DSH_DATARATE_1600             ((uint8_t)0x90) /* 1600 HZ Normal Mode  */

/* LIS3DSH Full Scale */
#define LIS3DSH_FULLSCALE_2               ((uint8_t)0x00) /* 2g  */
#define LIS3DSH_FULLSCALE_4               ((uint8_t)0x08) /* 4g  */
#define LIS3DSH_FULLSCALE_6               ((uint8_t)0x10) /* 6g  */
#define LIS3DSH_FULLSCALE_8               ((uint8_t)0x18) /* 8g  */
#define LIS3DSH_FULLSCALE_16              ((uint8_t)0x20) /* 16g */

/* LIS3DSH Anti-Aliasing Bandwidth */
#define LIS3DSH_FILTER_BW_800             ((uint8_t)0x00) /* 800 Hz  */
#define LIS3DSH_FILTER_BW_400             ((uint8_t)0x80) /* 400 Hz  */
#define LIS3DSH_FILTER_BW_200             ((uint8_t)0x40) /* 200 Hz  */
#define LIS3DSH_FILTER_BW_50              ((uint8_t)0xC0) /* 50 Hz  */

/* LIS3DSH Enabled Axis */
#define LIS3DSH_X_ENABLE                  ((uint8_t)0x01)
#define LIS3DSH_Y_ENABLE                  ((uint8_t)0x02)
#define LIS3DSH_XY_ENABLE				  ((uint8_t)0x03)

/* LIS3DSH Sensitivity Values  */
#define LIS3DSH_SENSITIVITY_0_06G         0.06f           /* 0.06 mg/digit */
#define LIS3DSH_SENSITIVITY_0_12G         0.12f           /* 0.12 mg/digit */
#define LIS3DSH_SENSITIVITY_0_18G         0.18f           /* 0.18 mg/digit */
#define LIS3DSH_SENSITIVITY_0_24G         0.24f           /* 0.24 mg/digit */
#define LIS3DSH_SENSITIVITY_0_73G         0.73f           /* 0.73 mg/digit */

/* LIS3DSH Configuration Struct  */
typedef struct
{
   uint8_t dataRate;
   uint8_t fullScale;
   uint8_t antiAliasingBW;
   uint8_t enabledAxis;
   bool    interruptEnable;
} LIS3DSH_Init_t;							// declaration of init structure

/* LIS3DSH Raw Data */
typedef struct
{
   int16_t x;
   int16_t y;
   int16_t z;
}LIS3DSH_DataRaw_t;

/* LIS3DSH mg Data (scaled data) */
typedef struct
{
	float x;
	float y;
	float z;
}LIS3DSH_DataScaled_t;

/* LIS3DSH Read & Write Reg */
void LIS3DSH_Write_Reg(uint8_t reg, uint8_t *dataW, uint8_t size);
void LIS3DSH_Read_Reg(uint8_t reg, uint8_t *dataR, uint8_t size);

/* LIS3DSH Init */
void LIS3DSH_Init(SPI_HandleTypeDef *accSPI ,LIS3DSH_Init_t *accInitDef);

/* LIS3DSH Get Data */
LIS3DSH_DataRaw_t LIS3DSH_Get_Data_Raw(void);
LIS3DSH_DataScaled_t LIS3DSH_Get_Data_Scaled(void);

/* LIS3DSH Axis Calibration */
void LIS3DSH_X_Calibrate(float x_min, float x_max);
void LIS3DSH_Y_Calibrate(float y_min, float y_max);

#endif /* INC_OMNICAR_INC_LIS3DSH_DRIVER_H_ */
