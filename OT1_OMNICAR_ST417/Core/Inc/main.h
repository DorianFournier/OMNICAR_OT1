/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS_Pin GPIO_PIN_4
#define CS_GPIO_Port GPIOC
#define INT1_DRY_Pin GPIO_PIN_5
#define INT1_DRY_GPIO_Port GPIOC
#define IN1_A_Pin GPIO_PIN_2
#define IN1_A_GPIO_Port GPIOB
#define IN2_A_Pin GPIO_PIN_7
#define IN2_A_GPIO_Port GPIOE
#define IN1_B_Pin GPIO_PIN_8
#define IN1_B_GPIO_Port GPIOE
#define IN2_B_Pin GPIO_PIN_11
#define IN2_B_GPIO_Port GPIOE
#define IN1_C_Pin GPIO_PIN_12
#define IN1_C_GPIO_Port GPIOE
#define IN2_C_Pin GPIO_PIN_15
#define IN2_C_GPIO_Port GPIOE
#define IN1_D_Pin GPIO_PIN_10
#define IN1_D_GPIO_Port GPIOB
#define IN2_D_Pin GPIO_PIN_11
#define IN2_D_GPIO_Port GPIOB
#define CE_Pin GPIO_PIN_12
#define CE_GPIO_Port GPIOB
#define CSN_Pin GPIO_PIN_8
#define CSN_GPIO_Port GPIOD
#define USER_LED_Pin GPIO_PIN_10
#define USER_LED_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
