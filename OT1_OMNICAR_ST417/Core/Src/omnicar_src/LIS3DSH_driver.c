/*
 * LIS3DSH_driver.c
 *
 *  Created on: June 11 2021
 *      Author: TEAM OMNICAR
 */

#include "omnicar_inc/LIS3DSH_driver.h"

/* SPI Chip Select */
#define _LIS3DSH_CS_ENABLE          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
#define _LIS3DSH_CS_DISABLE         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

static SPI_HandleTypeDef accSPI_Handle;

/* Sensitivity Value */
static float lis3dshSensitivity = LIS3DSH_SENSITIVITY_0_06G;

/* Bias variables */
static float __X_Bias = 0.0f;
static float __Y_Bias = 0.0f;

/* Scaling variables */
static float __X_Scale = 1.0f;
static float __Y_Scale = 1.0f;

/* LIS3DSH Write Reg */
void LIS3DSH_Write_Reg(uint8_t reg, uint8_t *dataW, uint8_t size)
{
	uint8_t spiReg = reg;													// Variable that store the first argument of the function
	_LIS3DSH_CS_ENABLE; 													// Activation CS pin
	HAL_SPI_Transmit(&accSPI_Handle, &spiReg, sizeof (spiReg), 10); 		// Transmission of register addresses via the SPI serial port
	HAL_SPI_Transmit(&accSPI_Handle, dataW, size, 10);						// Data transmission via the SPI serial port
	_LIS3DSH_CS_DISABLE;													// Disable CS pin
}

/* LIS3DSH Read Reg */
void LIS3DSH_Read_Reg(uint8_t reg, uint8_t *dataR, uint8_t size)
{
	uint8_t spiBuf[4];
	spiBuf[0] = reg | 0x80;									// Mask read bit on register adresse
	_LIS3DSH_CS_ENABLE;										// Enable CS
	HAL_SPI_Transmit(&accSPI_Handle, spiBuf, 1, 10); 		// Send register adresse
	HAL_SPI_Receive(&accSPI_Handle, spiBuf, size, 10);		// Get Register value
	_LIS3DSH_CS_DISABLE;									// Disable CS

	for(uint8_t i = 0; i < size; i++)						// condition made for "i" from 0 to 5
	{
		dataR[i] = spiBuf[i];								// store the data present in the case i of spiBuf into dataR[i] table
	}

}

void LIS3DSH_Init(SPI_HandleTypeDef *accSPI ,LIS3DSH_Init_t *accInitDef)
{
    uint8_t spiData = 0;

    memcpy(&accSPI_Handle, accSPI, sizeof(*accSPI));				// copy accSPI into &accSPI_Handle

    /* 1. Enable Axis and Output data rate */
    // Set CTRL REG4 settings value
    spiData |= (accInitDef->enabledAxis & 0x07);               		// Enable Axis

    spiData |= (accInitDef->dataRate & 0xF0);                 	 	// Output Data Rate
    //Write to accelerometer
    //LIS3DSH_Write_Reg(LIS3DSH_CTRL_REG4_ADDR, &spiData, 1)		// Function that allows to write the above parameters in the control register N°4
    LIS3DSH_Write_Reg(LIS3DSH_CTRL_REG4_ADDR, &spiData, 1);
    /* 2. Full-Scale selection, Anti Aliasing BW */
    spiData =0; //Sets the spiData variable to 0
    //Setting the values of the control register N°5
    spiData |= (accInitDef->antiAliasingBW & 0xC0);
    spiData |= (accInitDef->fullScale & 0x38);
    //Write to accelerometer
    LIS3DSH_Write_Reg(LIS3DSH_CTRL_REG5_ADDR, &spiData, 1); 		// Function that allows to write the above parameters in the control register N°5

    if(accInitDef->interruptEnable)									// Pointer of structure allowing to look at the parameters of interruptEnable
    {
    	spiData = 0x88; 											// Sets the spiData variable equal to 0x88
    	//Write to Accemerometer
    	LIS3DSH_Write_Reg(LIS3DSH_CTRL_REG3_ADDR,&spiData, 1);		// Function that allows to write the above parameters in the control register N°3
    }

        //Assign sensor sensitivity (based on Full-Scale)
        switch(accInitDef->fullScale)
        {
        case LIS3DSH_FULLSCALE_2:
					lis3dshSensitivity = LIS3DSH_SENSITIVITY_0_06G;
					break;

        case LIS3DSH_FULLSCALE_4:
                	lis3dshSensitivity = LIS3DSH_SENSITIVITY_0_12G;
                	break;

        case LIS3DSH_FULLSCALE_6:
                	lis3dshSensitivity = LIS3DSH_SENSITIVITY_0_18G;
                	break;

        case LIS3DSH_FULLSCALE_8:
                	lis3dshSensitivity = LIS3DSH_SENSITIVITY_0_24G;
                	break;

        case LIS3DSH_FULLSCALE_16:
                	lis3dshSensitivity = LIS3DSH_SENSITIVITY_0_73G;
                	break;
        }
}

/* LIS3DSH Get Raw Data */
LIS3DSH_DataRaw_t LIS3DSH_Get_Data_Raw(void)
{
	uint8_t spiBuf[2]; 													// Table of 2 elements
	LIS3DSH_DataRaw_t tmpDataRaw;

	//Read X data
	LIS3DSH_Read_Reg(LIS3DSH_OUT_X_L_ADDR, spiBuf, 2); 					// Register address, Buffer, number of cells in the array
    tmpDataRaw.x = ((spiBuf[1] << 8) + spiBuf [0]);  					// spibuf[0] => the low byte stored in the first cell of the array. spibuf[1] => high byte stored in the second cell shifted by 8 to make 16 bits

    //Read Y dataF
    LIS3DSH_Read_Reg(LIS3DSH_OUT_Y_L_ADDR, spiBuf, 2);					// Register address, Buffer, number of cells in the array
    tmpDataRaw.y = ((spiBuf[1] << 8) + spiBuf [0]);						// spibuf[0] => the low byte stored in the first cell of the array. spibuf[1] => high byte stored in the second cell shifted by 8 to make 16 bits

    //Read Z data
    LIS3DSH_Read_Reg(LIS3DSH_OUT_Z_L_ADDR, spiBuf, 2);					// Register address, Buffer, number of cells in the array
    tmpDataRaw.z = ((spiBuf[1] << 8) + spiBuf [0]);						// spibuf[0] => the low byte stored in the first cell of the array. spibuf[1] => high byte stored in the second cell shifted by 8 to make 16 bits

    return tmpDataRaw;
}

/* LIS3DSH Get mg data */
LIS3DSH_DataScaled_t LIS3DSH_Get_Data_Scaled(void)
{
	LIS3DSH_DataRaw_t tmpDataRaw = LIS3DSH_Get_Data_Raw();					// Read raw data

	//Scale data and return
    LIS3DSH_DataScaled_t tmpScaledData;										// Initialization of the tmpScaledData structure
    tmpScaledData.x = (tmpDataRaw.x * lis3dshSensitivity * __X_Scale) + 0.0f - __X_Bias;
    tmpScaledData.y = (tmpDataRaw.y * lis3dshSensitivity * __Y_Scale) + 0.0f - __Y_Bias;

    return tmpScaledData;
}

void LIS3DSH_X_Calibrate(float x_min, float x_max)
{
	__X_Scale = (2*1000)/(x_max - x_min);
	__X_Bias = (x_max + x_min)/ 2.0f;
}

void LIS3DSH_Y_Calibrate(float y_min, float y_max)
{
	__Y_Scale = (2*1000)/(y_max - y_min);
	__Y_Bias = (y_max + y_min)/ 2.0f;
}
