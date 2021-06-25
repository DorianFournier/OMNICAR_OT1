/*
 * battery.c
 *
 *  Created on: June 5, 2021
 *      Author: TEAM OMNICAR
 */

#include "omnicar_inc/battery.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>

#define MAX 4096
#define MIN 3413

static uint8_t batValue = 0;
static uint8_t bat = 0;
static float batPercent = 0.0;

uint8_t battery_getBatteryLevel(ADC_HandleTypeDef *hadc)
{
    HAL_ADC_Start(hadc);												// call function to start the ADC
    if(HAL_ADC_PollForConversion(hadc, 1) == HAL_OK)					// check if the conversion of ADC is complete
    {
        batValue = HAL_ADC_GetValue(hadc);								// batValue take the converted value of ADC
    }

    //batPercent = 100 - (((MAX - (float)batValue)/ MAX - MIN)*100);
    batPercent = (((MAX - (float)batValue)/ (MAX - MIN)))*100;			// store the result of the equation in batPercent
    batPercent = 100.0f - batPercent;									// subtract the value of batPercent by 100
    bat = (uint8_t)batPercent;											// cast to integer the batPercent variable (float)

    return bat;															// return value of the bat at main.c file
}
