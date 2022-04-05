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
#define D_RSTN_Pin GPIO_PIN_0
#define D_RSTN_GPIO_Port GPIOC
#define D_IRQ_Pin GPIO_PIN_1
#define D_IRQ_GPIO_Port GPIOC
#define D_IRQ_EXTI_IRQn EXTI1_IRQn
#define D_WKUP_Pin GPIO_PIN_2
#define D_WKUP_GPIO_Port GPIOC
#define ESP_WKUP_Pin GPIO_PIN_3
#define ESP_WKUP_GPIO_Port GPIOC
#define D_CS_Pin GPIO_PIN_4
#define D_CS_GPIO_Port GPIOA
#define D_CLK_Pin GPIO_PIN_5
#define D_CLK_GPIO_Port GPIOA
#define D_MISO_Pin GPIO_PIN_6
#define D_MISO_GPIO_Port GPIOA
#define D_MOSI_Pin GPIO_PIN_7
#define D_MOSI_GPIO_Port GPIOA
#define BAT_ADC_Pin GPIO_PIN_0
#define BAT_ADC_GPIO_Port GPIOB
#define VBUS_DET_Pin GPIO_PIN_1
#define VBUS_DET_GPIO_Port GPIOB
#define PG_PWR_Pin GPIO_PIN_9
#define PG_PWR_GPIO_Port GPIOE
#define PG_PWR_EXTI_IRQn EXTI9_5_IRQn
#define CHG_EN_Pin GPIO_PIN_10
#define CHG_EN_GPIO_Port GPIOE
#define CHG_PPR_Pin GPIO_PIN_11
#define CHG_PPR_GPIO_Port GPIOE
#define CHG_PPR_EXTI_IRQn EXTI15_10_IRQn
#define CHG_CH_Pin GPIO_PIN_12
#define CHG_CH_GPIO_Port GPIOE
#define CHG_CH_EXTI_IRQn EXTI15_10_IRQn
#define CHG_FAST_Pin GPIO_PIN_13
#define CHG_FAST_GPIO_Port GPIOE
#define CHG_FAST_EXTI_IRQn EXTI15_10_IRQn
#define ESP_EN_Pin GPIO_PIN_15
#define ESP_EN_GPIO_Port GPIOE
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_13
#define LED_B_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_14
#define LED_G_GPIO_Port GPIOB
#define MSD_DET_Pin GPIO_PIN_10
#define MSD_DET_GPIO_Port GPIOD
#define BUZZER_Pin GPIO_PIN_15
#define BUZZER_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
