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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EN2_1_Pin GPIO_PIN_0
#define EN2_1_GPIO_Port GPIOA
#define EN2_2_Pin GPIO_PIN_1
#define EN2_2_GPIO_Port GPIOA
#define CFG0_Pin GPIO_PIN_12
#define CFG0_GPIO_Port GPIOB
#define CFG1_Pin GPIO_PIN_13
#define CFG1_GPIO_Port GPIOB
#define CFG2_Pin GPIO_PIN_14
#define CFG2_GPIO_Port GPIOB
#define EN1_1_Pin GPIO_PIN_8
#define EN1_1_GPIO_Port GPIOA
#define EN1_2_Pin GPIO_PIN_9
#define EN1_2_GPIO_Port GPIOA
#define MT1_Pin GPIO_PIN_8
#define MT1_GPIO_Port GPIOB
#define MT2_Pin GPIO_PIN_9
#define MT2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
