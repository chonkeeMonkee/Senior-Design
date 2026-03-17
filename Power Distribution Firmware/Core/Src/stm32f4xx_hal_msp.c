/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f4xx_hal_msp.c
  * @brief        MSP Initialization for Servo PCB (STM32F405RGT6)
  *
  * Peripherals configured:
  *   CAN1  — PB8 (RX), PB9 (TX)        AF9
  *   TIM3  — PA6/PA7 (CH1/2), PB0/PB1 (CH3/4)  AF2  (50 Hz servo PWM)
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */
  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init */

  /* USER CODE BEGIN MspInit 1 */
  /* USER CODE END MspInit 1 */
}

/* ============================================================================
   CAN1 MSP
   PB8  ——> CAN1_RX   (AF9)
   PB9  ——> CAN1_TX   (AF9)
   ============================================================================ */
void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (hcan->Instance == CAN1)
  {
    /* USER CODE BEGIN CAN1_MspInit 0 */
    /* USER CODE END CAN1_MspInit 0 */

    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN CAN1_MspInit 1 */
    /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan)
{
  if (hcan->Instance == CAN1)
  {
    /* USER CODE BEGIN CAN1_MspDeInit 0 */
    /* USER CODE END CAN1_MspDeInit 0 */

    __HAL_RCC_CAN1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);

    /* USER CODE BEGIN CAN1_MspDeInit 1 */
    /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* ============================================================================
   TIM3 MSP — 50 Hz PWM for 4× SG90 servos
   Clock enable only (GPIO configured in MspPostInit after channel setup)
   ============================================================================ */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
  if (htim_pwm->Instance == TIM3)
  {
    /* USER CODE BEGIN TIM3_MspInit 0 */
    /* USER CODE END TIM3_MspInit 0 */

    __HAL_RCC_TIM3_CLK_ENABLE();

    /* USER CODE BEGIN TIM3_MspInit 1 */
    /* USER CODE END TIM3_MspInit 1 */
  }
}

/* ============================================================================
   TIM3 MspPostInit — GPIO alternate function setup
   PA6  ——> TIM3_CH1  (AF2)   S1 Bow Port
   PA7  ——> TIM3_CH2  (AF2)   S2 Bow Starboard
   PB0  ——> TIM3_CH3  (AF2)   S3 Stern Port
   PB1  ——> TIM3_CH4  (AF2)   S4 Stern Starboard
   ============================================================================ */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (htim->Instance == TIM3)
  {
    /* USER CODE BEGIN TIM3_MspPostInit 0 */
    /* USER CODE END TIM3_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PA6 (CH1) and PA7 (CH2) */
    GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PB0 (CH3) and PB1 (CH4) */
    GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM3_MspPostInit 1 */
    /* USER CODE END TIM3_MspPostInit 1 */
  }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{
  if (htim_pwm->Instance == TIM3)
  {
    /* USER CODE BEGIN TIM3_MspDeInit 0 */
    /* USER CODE END TIM3_MspDeInit 0 */

    __HAL_RCC_TIM3_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6 | GPIO_PIN_7);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0 | GPIO_PIN_1);

    /* USER CODE BEGIN TIM3_MspDeInit 1 */
    /* USER CODE END TIM3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* ============================================================================
   I2C1 MSP — INA226 power monitor
   PB6  ——> I2C1_SCL  (AF4, open-drain)
   PB7  ——> I2C1_SDA  (AF4, open-drain)
   External 4.7 kΩ pull-up resistors to 3.3 V required on both lines.
   ============================================================================ */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (hi2c->Instance == I2C1)
  {
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB6 = SCL, PB7 = SDA — open-drain, AF4 */
    GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;   /* rely on external pull-ups */
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    __HAL_RCC_I2C1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
  }
}

/* USER CODE END 1 */
