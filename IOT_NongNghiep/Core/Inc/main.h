/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f1xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

#define Relay_4_Pin GPIO_PIN_4
#define Relay_4_GPIO_Port GPIOA

#define Relay_3_Pin GPIO_PIN_5
#define Relay_3_GPIO_Port GPIOB

#define Relay_2_Pin GPIO_PIN_4
#define Relay_2_GPIO_Port GPIOB

#define Relay_1_Pin GPIO_PIN_3
#define Relay_1_GPIO_Port GPIOB

#define Button_Mode_Pin GPIO_PIN_11
#define Button_Mode_GPIO_Port GPIOB

#define Button_Ok_Pin GPIO_PIN_10
#define Button_Ok_GPIO_Port GPIOB

#define Button_Up_Pin GPIO_PIN_1
#define Button_Up_GPIO_Port GPIOB

#define Button_Down_Pin GPIO_PIN_0
#define Button_Down_GPIO_Port GPIOB

#define Button_Mode HAL_GPIO_ReadPin(Button_Mode_GPIO_Port, Button_Mode_Pin)
#define Button_Ok	   HAL_GPIO_ReadPin(Button_Ok_GPIO_Port, Button_Ok_Pin)
#define Button_Up		HAL_GPIO_ReadPin(Button_Up_GPIO_Port, Button_Up_Pin)
#define Button_Down	HAL_GPIO_ReadPin(Button_Down_GPIO_Port, Button_Down_Pin)

#define Buzzer_Pin GPIO_PIN_11
#define Buzzer_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
