/*
 * sensors.h
 *
 *  Created on: Jun 7, 2021
 *      Author: TEAM OMNICAR
 */

#ifndef INC_OMNICAR_INC_SENSORS_H_
#define INC_OMNICAR_INC_SENSORS_H_

#include "main.h"
#include <stdbool.h>

extern uint16_t RAW_DISTANCE;

/*
uint16_t sensors_calculated_distance();
uint16_t sensors_calculated_distance_2();
*/
void sensors_init_pwm(TIM_HandleTypeDef *TIMPORT, uint32_t *channel_array);			// Function prototype for init the sensors PWM

bool sensors_calculated_distance(void);												// Function prototype to calculated distance
bool sensors_calculated_distance_2(void);											// Function prototype to calculated distance with a second ultra-sonic sensor

uint16_t calculated_distance();
uint16_t calculated_distance_2();

void sensors_start_servo(TIM_HandleTypeDef *htim, uint32_t CHANNEL);				// Function prototype to start the servomotor

#endif /* INC_OMNICAR_INC_SENSORS_H_ */
