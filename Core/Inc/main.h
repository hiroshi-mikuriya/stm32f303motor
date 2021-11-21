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
#include "stm32f3xx_ll_adc.h"
#include "stm32f3xx_ll_dma.h"
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_system.h"
#include "stm32f3xx_ll_exti.h"
#include "stm32f3xx_ll_cortex.h"
#include "stm32f3xx_ll_utils.h"
#include "stm32f3xx_ll_pwr.h"
#include "stm32f3xx_ll_tim.h"
#include "stm32f3xx_ll_usart.h"
#include "stm32f3xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task_resource.h"
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
#define MOTOR1_ENC_TIM TIM1
#define MOTOR_PWM_TIM TIM3
#define MOTOR2_ENC_TIM TIM2
#define ENC_UPDATE_TIM TIM6
#define KEY_CHECK_TIM TIM7
#define MCO_Pin LL_GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define SWT1_Pin LL_GPIO_PIN_1
#define SWT1_GPIO_Port GPIOF
#define MOTOR2_ENC_A_Pin LL_GPIO_PIN_0
#define MOTOR2_ENC_A_GPIO_Port GPIOA
#define MOTOR2_ENC_B_Pin LL_GPIO_PIN_1
#define MOTOR2_ENC_B_GPIO_Port GPIOA
#define VCP_TX_Pin LL_GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define BTN1_Pin LL_GPIO_PIN_3
#define BTN1_GPIO_Port GPIOA
#define MOTOR1_PWM_Pin LL_GPIO_PIN_6
#define MOTOR1_PWM_GPIO_Port GPIOA
#define MOTOR2_PWM_Pin LL_GPIO_PIN_7
#define MOTOR2_PWM_GPIO_Port GPIOA
#define BTN2_Pin LL_GPIO_PIN_0
#define BTN2_GPIO_Port GPIOB
#define MOTOR1_CTRL1_Pin LL_GPIO_PIN_1
#define MOTOR1_CTRL1_GPIO_Port GPIOB
#define MOTOR1_ENC_A_Pin LL_GPIO_PIN_8
#define MOTOR1_ENC_A_GPIO_Port GPIOA
#define MOTOR1_ENC_B_Pin LL_GPIO_PIN_9
#define MOTOR1_ENC_B_GPIO_Port GPIOA
#define MOTOR1_CTRL2_Pin LL_GPIO_PIN_10
#define MOTOR1_CTRL2_GPIO_Port GPIOA
#define MOTOR2_CTRL1_Pin LL_GPIO_PIN_11
#define MOTOR2_CTRL1_GPIO_Port GPIOA
#define MOTOR2_CTRL2_Pin LL_GPIO_PIN_12
#define MOTOR2_CTRL2_GPIO_Port GPIOA
#define SWDIO_Pin LL_GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin LL_GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin LL_GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define LD3_Pin LL_GPIO_PIN_3
#define LD3_GPIO_Port GPIOB
#define LED1_Pin LL_GPIO_PIN_4
#define LED1_GPIO_Port GPIOB
#define LED2_Pin LL_GPIO_PIN_5
#define LED2_GPIO_Port GPIOB
#define SWT2_Pin LL_GPIO_PIN_7
#define SWT2_GPIO_Port GPIOB
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
