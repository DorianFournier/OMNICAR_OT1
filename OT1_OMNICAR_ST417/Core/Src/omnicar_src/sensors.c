/*
 * ultrason.c
 *
 *  Created on: June 11, 2021
 *      Author: TEAM OMNICAR
 */

#include "omnicar_inc/sensors.h"
#include "main.h"

uint16_t IC_Value [ 4 ] = { 0, 0 , 0 }; 			// [0] = 0
uint8_t Is_First_Captured [ 2 ] = { 0, 0};  		// 0 - not captured, 1 - captured
uint16_t temp = 0 ;
uint16_t RAW_DISTANCE = 0;
uint16_t RAW_DISTANCE_2 = 0;

static TIM_HandleTypeDef *TIM_PORT;									// declaration of static TIM_Handle to store the timer port used
static bool state = false;											// declaration of static data servomotor state
static uint8_t dutyCycle = 25;										// declaration of static variable for servomotor
static bool dst = false;											// declaration of static variable for the distance 1
static bool dst_2 = false;											// declaration of static variable for the distance 2

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim8)
{
	if (htim8-> Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		if (Is_First_Captured [ 0 ] == 0 )
		{
			IC_Value [ 0 ] = HAL_TIM_ReadCapturedValue (htim8, TIM_CHANNEL_2);
			Is_First_Captured [ 0 ] = 1 ;
			__HAL_TIM_SET_CAPTUREPOLARITY (htim8, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else  if (Is_First_Captured [ 0 ])
		{
			IC_Value [ 1 ] = HAL_TIM_ReadCapturedValue (htim8, TIM_CHANNEL_2);
			__HAL_TIM_SET_COUNTER (htim8, 0 );
			Is_First_Captured [ 0 ] = 0 ;
			__HAL_TIM_SET_CAPTUREPOLARITY (htim8, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);

			if (IC_Value [ 1 ]> IC_Value [ 0 ])
			{
				temp = IC_Value [ 1 ] -IC_Value [ 0 ];
				RAW_DISTANCE = IC_Value [ 1 ] -IC_Value [ 0 ];   // calculer la différence
			}
		}
	}

	if (htim8-> Channel == HAL_TIM_ACTIVE_CHANNEL_4)
	{
		if (Is_First_Captured [ 1 ] == 0 )
		{
			IC_Value [ 2 ] = HAL_TIM_ReadCapturedValue (htim8, TIM_CHANNEL_4);
			Is_First_Captured [ 1 ] = 1 ;
			__HAL_TIM_SET_CAPTUREPOLARITY (htim8, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else  if (Is_First_Captured [ 1 ])
		{
			IC_Value [ 3 ] = HAL_TIM_ReadCapturedValue (htim8, TIM_CHANNEL_4);
			__HAL_TIM_SET_COUNTER (htim8, 0 );
			Is_First_Captured [ 1 ] = 0 ;
			__HAL_TIM_SET_CAPTUREPOLARITY (htim8, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);

			if (IC_Value [ 3 ]> IC_Value [ 2 ])
			{
				temp = IC_Value [ 3 ] -IC_Value [ 2 ];
				RAW_DISTANCE_2 = IC_Value [ 3 ] -IC_Value [ 2 ];   // calculer la différence
			}
		}
	}
}


void sensors_init_pwm(TIM_HandleTypeDef *TIMPORT, uint32_t *channel_array)
{
	TIM_PORT = TIMPORT;														// Store into TIM_PORT the first argument passed to the function

	for(uint8_t i = 0; i<5 ; i++)											// condition made for "i" from 0 to 4
	{
		if(i%2 == 0)														// if "i" is even (modulo)
		{
			HAL_TIM_IC_Start_IT(TIM_PORT, channel_array[i]);				// Function call to start input capture
		}
		else
		{
			HAL_TIM_PWM_Start(TIM_PORT, channel_array[i]);					// Function call to start the PWM
		}
	}
}

/*
uint16_t calculated_distance ()
{
	const float coef = ((float)17/1000);
	float distance;
	distance = coef*RAW_DISTANCE;
	distance = (uint16_t)distance;
	return distance;
}

uint16_t calculated_distance_2 ()
{
	const float coef = ((float)17/1000);
	float distance_2;
	distance_2 = coef*RAW_DISTANCE_2;
	distance_2 = (uint16_t)distance_2;
	return distance_2;
}
*/

bool sensors_calculated_distance(){
	const float coef = ((float)17/1000);				// declaration of a coefficient
	float distance;
	distance = coef*RAW_DISTANCE;						// distance calculation
	distance = (uint16_t)distance;						// cast distance : float -> integer

	if(distance < 25)								// if distance is smaller than 25
	{
		dst = true;										// dst is true
	}
	else
	{
		dst = false;									// dst is false
	}
	return dst;											// return the state of dst at the principal program
}

bool sensors_calculated_distance_2 (){
	const float coef = ((float)17/1000);				// declaration of a coefficient
	float distance_2;
	distance_2 = coef*RAW_DISTANCE_2;					// distance_2 calculation
	distance_2 = (uint16_t)distance_2;					// cast distance_2 : float -> integer

	if(distance_2 < 25)									// if distance_2 is smaller than 25
	{
		dst_2 = true;									// dst_2 is true
	}
	else
	{
		dst_2 = false;									// dst_2 is false
	}
	return dst_2;										// return the state of dst_2 at the principal program
}


void sensors_start_servo(TIM_HandleTypeDef *htim_servo, uint32_t CHANNEL)
{
    TIM_PORT = htim_servo;								 // Store into TIM_PORT the first argument passed to the function

    if(dutyCycle < 125 && state == false)
    {
        __HAL_TIM_SET_COMPARE(TIM_PORT, CHANNEL, dutyCycle);		// compare CCR value for the PWM
        dutyCycle = dutyCycle + 1;									// increase dutycycle
        HAL_Delay(5);												// Delay of 5 ms
    }
    else
    {
        state = true;
        __HAL_TIM_SET_COMPARE(TIM_PORT, CHANNEL, dutyCycle);		// compare the CCR value for the PWM
        dutyCycle = dutyCycle - 1;									// decrement dutycycle
        HAL_Delay(5);												// delay of 5 ms
        if (dutyCycle == 25)										// if the value of dutycycle is 25
        {
            state = false;											// state pass to false
        }
    }
}
