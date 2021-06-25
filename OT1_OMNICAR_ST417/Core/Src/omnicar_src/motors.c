/*
 * motors.c
 *
 *  Created on: June 11, 2021
 *      Author: TEAM OMNICAR
 */

#include <omnicar_inc/motors.h>
#include "stm32f4xx_hal_tim.h"

static TIM_HandleTypeDef *TIM_PORT;						// declaration of static data to store TIM_PORT
static GPIO_TypeDef *MOTORS_GPIO_PORT_E;				// declaration of static data to store GPIO_PORT_E
static GPIO_TypeDef *MOTORS_GPIO_PORT_B;				// declaration of static data to store GPIO_PORT_B

static uint16_t		 IN1_A;								// declaration of static data for IN1_A
static uint16_t		 IN1_B;								// declaration of static data for IN1_B
static uint16_t		 IN2_A;								// declaration of static data for IN2_A
static uint16_t		 IN2_B;								// declaration of static data for IN2_B
static uint16_t		 IN1_C;								// declaration of static data for IN1_C
static uint16_t		 IN1_D;								// declaration of static data for IN1_D
static uint16_t		 IN2_C;								// declaration of static data for IN2_C
static uint16_t		 IN2_D;								// declaration of static data for IN2_D

//1. INIT MOTORS
void init_pwm_motors(TIM_HandleTypeDef *TIMPORT, uint32_t *channel_array)
{
	TIM_PORT = TIMPORT;														// store into TIM_PORT the GPIO_TypeDef pass in argument of the function

	for(uint8_t i = 0; i<5 ; i++)											// condition made for "i" from 0 to 5
	{
		if(i == 2)
		{
			HAL_TIMEx_PWMN_Start(TIM_PORT, channel_array[i]);				// start PWMN Channel 2 (N = Negative)
		}
		else
		{
			HAL_TIM_PWM_Start(TIM_PORT, channel_array[i]);					// start PWM channel 1, 3, 4
		}
	}
}

//2. MOTORS POWER MANAGEMENT
void motors_management(TIM_HandleTypeDef *TIMPORT, uint32_t *channel_array, uint8_t *myRxData)
{
	TIM_PORT = TIMPORT;														// store into TIM_PORT the GPIO_TypeDef pass in argument of the function

	for(uint8_t i = 0; i<5; i++)											// condition made for "i" from 0 to 5
	{
		__HAL_TIM_SET_COMPARE(TIM_PORT, channel_array[i], myRxData[1]);		// compare CCR value for the PWM
	}
}

//3. MOTORS DIRECTION
void motors_direction(GPIO_TypeDef *motorsGPIO_B, GPIO_TypeDef *motorsGPIO_E, uint16_t *GPIO_IN_ARRAY, uint8_t *myRxData)
{
	MOTORS_GPIO_PORT_B = motorsGPIO_B;																		// store into MOTORS_GPIO_PORT_E the GPIO_TypeDef pass in argument of the function
	MOTORS_GPIO_PORT_E = motorsGPIO_E;																		// store into MOTORS_GPIO_PORT_E the GPIO_TypeDef pass in argument of the function
	IN1_A = GPIO_IN_ARRAY[0];																				// store into IN1_A the first case of GPIO_IN_ARRAY
	IN2_A = GPIO_IN_ARRAY[1];																				// store into IN2_A the first case of GPIO_IN_ARRAY
	IN1_B = GPIO_IN_ARRAY[2];																				// store into IN1_B the first case of GPIO_IN_ARRAY
	IN2_B = GPIO_IN_ARRAY[3];																				// store into IN2_B the first case of GPIO_IN_ARRAY
	IN1_C = GPIO_IN_ARRAY[4];																				// store into IN1_C the first case of GPIO_IN_ARRAY
	IN2_C = GPIO_IN_ARRAY[5];																				// store into IN2_C the first case of GPIO_IN_ARRAY
	IN1_D = GPIO_IN_ARRAY[6];																				// store into IN1_D the first case of GPIO_IN_ARRAY
	IN2_D = GPIO_IN_ARRAY[7];																				// store into IN2_D the first case of GPIO_IN_ARRAY

	switch (myRxData[0])
	{
	case 'H' :  HAL_GPIO_WritePin(MOTORS_GPIO_PORT_B, IN1_A | IN1_D , GPIO_PIN_SET);						 // Transition to the high state of the pins : IN1_A | IN1_D
				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_E, IN1_C | IN1_B, GPIO_PIN_SET);							 // Transition to the high state of the pins : IN1_C | IN1_B

				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_E, IN2_C |  IN2_B | IN2_A, GPIO_PIN_RESET);				 // Transition to the low state of the pins :  IN2_C | IN2_B | IN2_A
				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_B, IN2_D , GPIO_PIN_RESET);								 // Transition to the low state of the pins :  IN2_D
				break;

	case 'a' :  HAL_GPIO_WritePin(MOTORS_GPIO_PORT_B, IN1_D | IN1_A, GPIO_PIN_SET);							 // Transition to the high state of the pins : IN1_D | IN1_A

				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_E, IN1_C |  IN2_C | IN2_A | IN1_B |IN2_B, GPIO_PIN_RESET);// Transition to the low state of the pins : IN1_C |  IN2_C | IN2_A | IN1_B |IN2_B
				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_B, IN2_D, GPIO_PIN_RESET);								 // Transition to the low state of the pins : IN2_D
				break;

	case 'D' :  HAL_GPIO_WritePin(MOTORS_GPIO_PORT_E, IN2_B | IN2_C , GPIO_PIN_SET);						 // Transition to the high state of the pins : IN2_B | IN2_C
				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_B, IN1_D | IN1_A, GPIO_PIN_SET);							 // Transition to the high state of the pins : IN1_D | IN1_A

				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_B, IN2_D , GPIO_PIN_RESET);								 // Transition to the low state of the pins : IN2_D
				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_E, IN2_A | IN1_B | IN1_C, GPIO_PIN_RESET);				 // Transition to the low state of the pins : IN2_A | IN1_B | IN1_C
				break;

	case 'c' :  HAL_GPIO_WritePin(MOTORS_GPIO_PORT_E, IN2_A , GPIO_PIN_SET);								 // Transition to the high state of the pins : IN2_A
				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_B, IN2_D , GPIO_PIN_SET);								 // Transition to the high state of the pins : IN2_D

				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_E, IN1_B | IN2_B | IN1_C | IN2_C, GPIO_PIN_RESET);		 // Transition to the low state of the pins : IN1_B | IN2_B | IN1_C | IN2_C
				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_B, IN1_A | IN1_D, GPIO_PIN_RESET);						 // Transition to the low state of the pins : IN1_A | IN1_D
				break;

	case 'B' :  HAL_GPIO_WritePin(MOTORS_GPIO_PORT_B, IN2_D , GPIO_PIN_SET);								  // Transition to the high state of the pins : IN2_D
				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_E, IN2_A | IN2_B | IN2_C, GPIO_PIN_SET);					  // Transition to the high state of the pins :  IN2_A | IN2_B | IN2_C

				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_B, IN1_A | IN1_D, GPIO_PIN_RESET);						  // Transition to the low state of the pins : IN1_A | IN1_D
				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_E, IN1_B | IN1_C  , GPIO_PIN_RESET);						  // Transition to the low state of the pins : IN1_B | IN1_C
				break;

	case 'b' :  HAL_GPIO_WritePin(MOTORS_GPIO_PORT_E, IN2_B | IN2_C, GPIO_PIN_SET);							  // Transition to the high state of the pins : IN2_B | IN2_C

				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_E, IN2_A | IN1_B | IN1_C, GPIO_PIN_RESET);				  // Transition to the low state of the pins :  IN2_A | IN1_B | IN1_C
				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_B, IN1_A |IN1_D | IN2_D, GPIO_PIN_RESET);				  // Transition to the low state of the pins : IN1_A |IN1_D | IN2_D
				break;

	case 'G' :  HAL_GPIO_WritePin(MOTORS_GPIO_PORT_E, IN2_A | IN1_B | IN1_C , GPIO_PIN_SET);				  // Transition to the high state of the pins : IN2_A | IN1_B | IN1_C
				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_B, IN2_D, GPIO_PIN_SET);									  // Transition to the high state of the pins : IN2_D

				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_B, IN1_A | IN1_D , GPIO_PIN_RESET);						  // Transition to the low state of the pins : IN1_A | IN1_D
				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_E, IN2_C | IN2_B, GPIO_PIN_RESET);						  // Transition to the low state of the pins : IN2_C | IN2_B
				break;

	case 'd' :	HAL_GPIO_WritePin(MOTORS_GPIO_PORT_E, IN1_B | IN1_C, GPIO_PIN_SET);							  // Transition to the high state of the pins : IN1_B | IN1_C

				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_E, IN2_C | IN2_A | IN2_B, GPIO_PIN_RESET);				  // Transition to the low state of the pins :IN2_C | IN2_A | IN2_B
				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_B, IN1_A | IN1_D | IN2_D, GPIO_PIN_RESET);				  // Transition to the low state of the pins : IN1_A | IN1_D | IN2_D
				break;

	case 'N' :  HAL_GPIO_WritePin(MOTORS_GPIO_PORT_E, IN2_A | IN1_B | IN2_B | IN1_C | IN2_C, GPIO_PIN_RESET); // Transition to the low state of the pins : IN2_A | IN1_B | IN2_B | IN1_C | IN2_C
				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_B, IN1_A | IN1_D | IN2_D, GPIO_PIN_RESET);				  // Transition to the low state of the pins : IN1_A | IN1_D | IN2_D
				break;

	case 'R' :  HAL_GPIO_WritePin(MOTORS_GPIO_PORT_E, IN1_C | IN2_B , GPIO_PIN_SET);						  // Transition to the high state of the pins :IN1_C | IN2_B
				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_B, IN1_A | IN2_D , GPIO_PIN_SET);						  // Transition to the high state of the pins :IN1_A | IN2_D

				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_E, IN2_A | IN1_B | IN2_C, GPIO_PIN_RESET);				  // Transition to the low state of the pins : IN2_A | IN1_B | IN2_C
				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_B, IN1_D , GPIO_PIN_RESET);								  // Transition to the low state of the pins : IN1_D
				break;

	case 'L' : 	HAL_GPIO_WritePin(MOTORS_GPIO_PORT_E, IN2_A | IN1_B | IN2_C, GPIO_PIN_SET);					  // Transition to the high state of the pins :IN2_A | IN1_B | IN2_C
				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_B, IN1_D , GPIO_PIN_SET);								  // Transition to the high state of the pins : IN1_D

				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_E, IN2_B | IN1_C, GPIO_PIN_RESET);						  // Transition to the low state of the pins :IN2_B | IN1_C
				HAL_GPIO_WritePin(MOTORS_GPIO_PORT_B, IN1_A |IN2_D, GPIO_PIN_RESET);						  // Transition to the low state of the pins :IN1_A |IN2_D
				break;

	default :
				break;
	}

	if(myRxData[1] == 0)
	{
		HAL_GPIO_WritePin(MOTORS_GPIO_PORT_E, IN2_A | IN1_B | IN2_B | IN1_C | IN2_C, GPIO_PIN_RESET);		// Transition to the low state of the pins :IN2_A | IN1_B | IN2_B | IN1_C | IN2_C
		HAL_GPIO_WritePin(MOTORS_GPIO_PORT_B, IN1_A | IN1_D | IN2_D, GPIO_PIN_RESET);						// Transition to the low state of the pins :IN1_A | IN1_D | IN2_D
	}
}

// DUTY CYCLE
/*
 * DutyCycle = (CCR * 100) / ARR    <>   CCR = (ARR * DutyCycle) / 100
 * 25 = (CCR * 100) / 14
 * CCR = 14 * 25 / 100 = 3,5
 */
