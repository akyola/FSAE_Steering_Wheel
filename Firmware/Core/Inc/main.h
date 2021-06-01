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
#include "stm32f2xx_hal.h"
#include "stm32f2xx_ll_dma.h"
#include "stm32f2xx_ll_rcc.h"
#include "stm32f2xx_ll_bus.h"
#include "stm32f2xx_ll_system.h"
#include "stm32f2xx_ll_exti.h"
#include "stm32f2xx_ll_cortex.h"
#include "stm32f2xx_ll_utils.h"
#include "stm32f2xx_ll_pwr.h"
#include "stm32f2xx_ll_tim.h"
#include "stm32f2xx_ll_gpio.h"

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
#define test_led_Pin GPIO_PIN_2
#define test_led_GPIO_Port GPIOE
#define down_shift_nc_Pin GPIO_PIN_3
#define down_shift_nc_GPIO_Port GPIOE
#define down_shift_no_Pin GPIO_PIN_4
#define down_shift_no_GPIO_Port GPIOE
#define down_shift_no_EXTI_IRQn EXTI4_IRQn
#define neutral_sens_Pin GPIO_PIN_2
#define neutral_sens_GPIO_Port GPIOC
#define OLED1_CS_Pin GPIO_PIN_4
#define OLED1_CS_GPIO_Port GPIOA
#define OLED1_RST_Pin GPIO_PIN_6
#define OLED1_RST_GPIO_Port GPIOA
#define OLED1_DC_Pin GPIO_PIN_4
#define OLED1_DC_GPIO_Port GPIOC
#define down_shift_Pin GPIO_PIN_7
#define down_shift_GPIO_Port GPIOE
#define up_shift_Pin GPIO_PIN_8
#define up_shift_GPIO_Port GPIOE
#define up_shift_no_Pin GPIO_PIN_12
#define up_shift_no_GPIO_Port GPIOE
#define up_shift_no_EXTI_IRQn EXTI15_10_IRQn
#define up_shift_nc_Pin GPIO_PIN_13
#define up_shift_nc_GPIO_Port GPIOE
#define E_Pin GPIO_PIN_8
#define E_GPIO_Port GPIOD
#define D_Pin GPIO_PIN_9
#define D_GPIO_Port GPIOD
#define C_Pin GPIO_PIN_10
#define C_GPIO_Port GPIOD
#define DP_Pin GPIO_PIN_11
#define DP_GPIO_Port GPIOD
#define B_Pin GPIO_PIN_12
#define B_GPIO_Port GPIOD
#define A_Pin GPIO_PIN_13
#define A_GPIO_Port GPIOD
#define F_Pin GPIO_PIN_14
#define F_GPIO_Port GPIOD
#define G_Pin GPIO_PIN_15
#define G_GPIO_Port GPIOD
#define OLED2_DC_Pin GPIO_PIN_15
#define OLED2_DC_GPIO_Port GPIOA
#define OLED2_RST_Pin GPIO_PIN_11
#define OLED2_RST_GPIO_Port GPIOC
#define OELD2_CS_Pin GPIO_PIN_0
#define OELD2_CS_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
