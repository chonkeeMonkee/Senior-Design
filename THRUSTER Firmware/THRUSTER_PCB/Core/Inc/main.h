/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/


#define A_SENSE_ADC_Pin        GPIO_PIN_0
#define A_SENSE_ADC_GPIO_Port  GPIOA
#define B_SENSE_ADC_Pin        GPIO_PIN_1
#define B_SENSE_ADC_GPIO_Port  GPIOA
#define SO1_ADC_Pin            GPIO_PIN_2
#define SO1_ADC_GPIO_Port      GPIOA
#define SO2_MCU_ADC_Pin        GPIO_PIN_3
#define SO2_MCU_ADC_GPIO_Port  GPIOA


#define MCU_PWM1_L_Pin         GPIO_PIN_7
#define MCU_PWM1_L_GPIO_Port   GPIOA
#define MCU_PWM2_L_Pin         GPIO_PIN_0
#define MCU_PWM2_L_GPIO_Port   GPIOB
#define MCU_PWM3_L_Pin         GPIO_PIN_1
#define MCU_PWM3_L_GPIO_Port   GPIOB


#define EN_GATE_OUT_Pin        GPIO_PIN_10
#define EN_GATE_OUT_GPIO_Port  GPIOB
#define DC_CAL_Out_Pin         GPIO_PIN_11
#define DC_CAL_Out_GPIO_Port   GPIOB
#define FAULT_Pin              GPIO_PIN_12
#define FAULT_GPIO_Port        GPIOB
#define OCTW_Pin               GPIO_PIN_13
#define OCTW_GPIO_Port         GPIOB


#define MCU_PWM1_H_Pin         GPIO_PIN_8
#define MCU_PWM1_H_GPIO_Port   GPIOA
#define MCU_PWM2_H_Pin         GPIO_PIN_9
#define MCU_PWM2_H_GPIO_Port   GPIOA
#define MCU_PWM3_H_Pin         GPIO_PIN_10
#define MCU_PWM3_H_GPIO_Port   GPIOA


#define LED1_Pin               GPIO_PIN_5
#define LED1_GPIO_Port         GPIOC
#define LED2_Pin               GPIO_PIN_6
#define LED2_GPIO_Port         GPIOC
#define LED3_Pin               GPIO_PIN_7
#define LED3_GPIO_Port         GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
