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
#include "stm32l4xx_hal.h"

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
void serialPrint(char val[]);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define JOYY_Pin GPIO_PIN_0
#define JOYY_GPIO_Port GPIOA
#define JOYX_Pin GPIO_PIN_1
#define JOYX_GPIO_Port GPIOA
#define C0_Pin GPIO_PIN_3
#define C0_GPIO_Port GPIOA
#define OUTR_Pin GPIO_PIN_4
#define OUTR_GPIO_Port GPIOA
#define OUTL_Pin GPIO_PIN_5
#define OUTL_GPIO_Port GPIOA
#define REN_Pin GPIO_PIN_6
#define REN_GPIO_Port GPIOA
#define C2_Pin GPIO_PIN_7
#define C2_GPIO_Port GPIOA
#define RA0_Pin GPIO_PIN_0
#define RA0_GPIO_Port GPIOB
#define RA1_Pin GPIO_PIN_1
#define RA1_GPIO_Port GPIOB
#define C1_Pin GPIO_PIN_8
#define C1_GPIO_Port GPIOA
#define C3_Pin GPIO_PIN_9
#define C3_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define LED_BUILTIN_Pin GPIO_PIN_3
#define LED_BUILTIN_GPIO_Port GPIOB
#define RA2_Pin GPIO_PIN_4
#define RA2_GPIO_Port GPIOB
#define OUT_Pin GPIO_PIN_5
#define OUT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
