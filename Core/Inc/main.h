/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define NOT_EN_TMC1_Pin GPIO_PIN_13
#define NOT_EN_TMC1_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOC
#define DE_RS_Pin GPIO_PIN_1
#define DE_RS_GPIO_Port GPIOA
#define TX_RS_Pin GPIO_PIN_2
#define TX_RS_GPIO_Port GPIOA
#define RX_RS_Pin GPIO_PIN_3
#define RX_RS_GPIO_Port GPIOA
#define ETH_RESET_Pin GPIO_PIN_4
#define ETH_RESET_GPIO_Port GPIOA
#define SWD_TX_Pin GPIO_PIN_4
#define SWD_TX_GPIO_Port GPIOC
#define SWD_RX_Pin GPIO_PIN_5
#define SWD_RX_GPIO_Port GPIOC
#define DIR_TMC2_Pin GPIO_PIN_1
#define DIR_TMC2_GPIO_Port GPIOB
#define TX_TMC2_Pin GPIO_PIN_10
#define TX_TMC2_GPIO_Port GPIOB
#define RX_TMC2_Pin GPIO_PIN_11
#define RX_TMC2_GPIO_Port GPIOB
#define TMC2_invEN_Pin GPIO_PIN_12
#define TMC2_invEN_GPIO_Port GPIOB
#define ALERT2_Pin GPIO_PIN_7
#define ALERT2_GPIO_Port GPIOC
#define CS2_Pin GPIO_PIN_8
#define CS2_GPIO_Port GPIOC
#define DIR_TMC1_Pin GPIO_PIN_8
#define DIR_TMC1_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define TX_TMC1_Pin GPIO_PIN_10
#define TX_TMC1_GPIO_Port GPIOC
#define RX_TMC1_Pin GPIO_PIN_11
#define RX_TMC1_GPIO_Port GPIOC
#define DIAG_TMC1_Pin GPIO_PIN_12
#define DIAG_TMC1_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define CS1_Pin GPIO_PIN_4
#define CS1_GPIO_Port GPIOB
#define INT_Pin GPIO_PIN_5
#define INT_GPIO_Port GPIOB
#define SS_Pin GPIO_PIN_6
#define SS_GPIO_Port GPIOB
#define PWDN_Pin GPIO_PIN_7
#define PWDN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
