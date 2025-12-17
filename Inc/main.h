/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_gpio.h"

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
#define USR_BUTTON	LL_GPIO_PIN_0
#define BUTTON_Port	GPIOA

#define SS_D0	LL_GPIO_PIN_10	// UP
#define SS_D1	LL_GPIO_PIN_11	// DOWN	
#define SS_D2	LL_GPIO_PIN_12	// LEFT	
#define SS_D3	LL_GPIO_PIN_13	// RIGHT
#define SS_D4	LL_GPIO_PIN_14	// TL
#define SS_S0	LL_GPIO_PIN_15	// TR
#define SS_S1	LL_GPIO_PIN_4	// TH
#define L3BTN	LL_GPIO_PIN_0
#define R3BTN	LL_GPIO_PIN_1
#define LED_BLUE	LL_GPIO_PIN_2

#define SS_Port	GPIOB

#define LED_GREEN	LL_GPIO_PIN_13
#define LED_Port	GPIOC

#define TH_HIGH() LL_GPIO_SetOutputPin(SS_Port, SS_S1)
#define TH_LOW()  LL_GPIO_ResetOutputPin(SS_Port, SS_S1)
#define TR_HIGH() LL_GPIO_SetOutputPin(SS_Port, SS_S0)
#define TR_LOW()  LL_GPIO_ResetOutputPin(SS_Port, SS_S0)
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
