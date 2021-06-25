/*
 * battery.h
 *
 *  Created on: June 5, 2021
 *      Author: TEAM OMNICAR
 */

#ifndef INC_OMNICAR_INC_BATTERY_H_
#define INC_OMNICAR_INC_BATTERY_H_

#include "stm32f4xx_hal.h"

uint8_t battery_getBatteryLevel(ADC_HandleTypeDef *hadc);		// Function prototype to get the battery level for OT1

#endif /* INC_OMNICAR_INC_BATTERY_H_ */
