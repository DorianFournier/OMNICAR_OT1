/*
 * motors.h
 *
 *  Created on: June 11, 2021
 *      Author: TEAM OMNICAR
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "stm32f4xx_hal.h"                   //** Change this according to your STM32 series **//

//1. INIT MOTORS
void init_pwm_motors(TIM_HandleTypeDef *TIMPORT, uint32_t *channel_array);							// function prototype for the initialization of PWM motors

//2. MOTORS POWER MANAGEMENT
void motors_management(TIM_HandleTypeDef *TIMPORT, uint32_t *channel_array, uint8_t *myRxData);		// function prototype for the power management

//3. MOTORS DIRECTION
void motors_direction(GPIO_TypeDef *motorsGPIO_B, GPIO_TypeDef *motorsGPIO_E, uint16_t *GPIO_IN_ARRAY, uint8_t *myRxData);		// function prototype for the motors direction

#endif /* INC_MOTORS_H_ */
