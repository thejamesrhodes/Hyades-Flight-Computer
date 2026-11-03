/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file. Common application defines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics. All rights reserved.
  * Licensed under terms in the LICENSE file; if none, provided AS-IS.
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
#define CLK_IN_Pin GPIO_PIN_0
#define CLK_IN_GPIO_Port GPIOH
#define CLK_OUT_Pin GPIO_PIN_1
#define CLK_OUT_GPIO_Port GPIOH
#define LED_2_Pin GPIO_PIN_1
#define LED_2_GPIO_Port GPIOA
#define PWM_1_Pin GPIO_PIN_5
#define PWM_1_GPIO_Port GPIOA
#define PWM_2_Pin GPIO_PIN_6
#define PWM_2_GPIO_Port GPIOA
#define PWM_3_Pin GPIO_PIN_7
#define PWM_3_GPIO_Port GPIOA
#define PWM_4_Pin GPIO_PIN_0
#define PWM_4_GPIO_Port GPIOB
#define PWM_5_Pin GPIO_PIN_1
#define PWM_5_GPIO_Port GPIOB
#define PWM_6_Pin GPIO_PIN_10
#define PWM_6_GPIO_Port GPIOB
#define SPI_SCK_Pin GPIO_PIN_12
#define SPI_SCK_GPIO_Port GPIOB
#define RX_1_Pin GPIO_PIN_10
#define RX_1_GPIO_Port GPIOA
#define USB_D__Pin GPIO_PIN_11
#define USB_D__GPIO_Port GPIOA
#define USB_D_A12_Pin GPIO_PIN_12
#define USB_D_A12_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define TX_1_Pin GPIO_PIN_15
#define TX_1_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define MISO_Pin GPIO_PIN_4
#define MISO_GPIO_Port GPIOB
#define MOSI_Pin GPIO_PIN_5
#define MOSI_GPIO_Port GPIOB
#define SCL_1_Pin GPIO_PIN_6
#define SCL_1_GPIO_Port GPIOB
#define SDA_1_Pin GPIO_PIN_7
#define SDA_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
