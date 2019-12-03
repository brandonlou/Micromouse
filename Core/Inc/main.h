/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

TIM_HandleTypeDef* Get_HTim2_Ptr(void);
TIM_HandleTypeDef* Get_HTim3_Ptr(void);
TIM_HandleTypeDef* Get_HTim4_Ptr(void);

ADC_HandleTypeDef* Get_HAdc1_Ptr(void);


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PushButton_Pin GPIO_PIN_13
#define PushButton_GPIO_Port GPIOC
#define PushButton_EXTI_IRQn EXTI15_10_IRQn
#define IrLeftR_Pin GPIO_PIN_0
#define IrLeftR_GPIO_Port GPIOC
#define IrFrontLR_Pin GPIO_PIN_1
#define IrFrontLR_GPIO_Port GPIOC
#define IrRightR_Pin GPIO_PIN_0
#define IrRightR_GPIO_Port GPIOA
#define EncoderLF_Pin GPIO_PIN_1
#define EncoderLF_GPIO_Port GPIOA
#define EncoderLF_EXTI_IRQn EXTI1_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define IrFrontRR_Pin GPIO_PIN_4
#define IrFrontRR_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOA
#define MotorLB_Pin GPIO_PIN_6
#define MotorLB_GPIO_Port GPIOA
#define EncoderLB_Pin GPIO_PIN_4
#define EncoderLB_GPIO_Port GPIOC
#define EncoderLB_EXTI_IRQn EXTI4_IRQn
#define IrFrontLE_Pin GPIO_PIN_0
#define IrFrontLE_GPIO_Port GPIOB
#define MotorRB_Pin GPIO_PIN_10
#define MotorRB_GPIO_Port GPIOB
#define MotorRF_Pin GPIO_PIN_7
#define MotorRF_GPIO_Port GPIOC
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define EncoderRB_Pin GPIO_PIN_15
#define EncoderRB_GPIO_Port GPIOA
#define EncoderRB_EXTI_IRQn EXTI15_10_IRQn
#define IrRightE_Pin GPIO_PIN_10
#define IrRightE_GPIO_Port GPIOC
#define IrFrontRE_Pin GPIO_PIN_11
#define IrFrontRE_GPIO_Port GPIOC
#define EncoderRF_Pin GPIO_PIN_3
#define EncoderRF_GPIO_Port GPIOB
#define EncoderRF_EXTI_IRQn EXTI3_IRQn
#define MotorLF_Pin GPIO_PIN_6
#define MotorLF_GPIO_Port GPIOB
#define IrLeftE_Pin GPIO_PIN_7
#define IrLeftE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
