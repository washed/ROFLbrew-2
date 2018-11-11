/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32f7xx.h"
#include "stm32f7xx_ll_system.h"
#include "stm32f7xx_ll_gpio.h"
#include "stm32f7xx_ll_exti.h"
#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_cortex.h"
#include "stm32f7xx_ll_rcc.h"
#include "stm32f7xx_ll_utils.h"
#include "stm32f7xx_ll_pwr.h"
#include "stm32f7xx_ll_dma.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LCD_TE_Pin LL_GPIO_PIN_14
#define LCD_TE_GPIO_Port GPIOB
#define LCD_UPDATE_Pin LL_GPIO_PIN_15
#define LCD_UPDATE_GPIO_Port GPIOB
#define LCD_BCKLIGHT_PWM_Pin LL_GPIO_PIN_12
#define LCD_BCKLIGHT_PWM_GPIO_Port GPIOD
#define LCD_RST_Pin LL_GPIO_PIN_13
#define LCD_RST_GPIO_Port GPIOD
#define MAX31865_CS_Pin LL_GPIO_PIN_15
#define MAX31865_CS_GPIO_Port GPIOA
#define MAX31865_SCK_Pin LL_GPIO_PIN_10
#define MAX31865_SCK_GPIO_Port GPIOC
#define MAX31865_MISO_Pin LL_GPIO_PIN_11
#define MAX31865_MISO_GPIO_Port GPIOC
#define MAX31865_MOSI_Pin LL_GPIO_PIN_12
#define MAX31865_MOSI_GPIO_Port GPIOC
#define MAX31865_DR_Pin LL_GPIO_PIN_2
#define MAX31865_DR_GPIO_Port GPIOD
#define MAX31865_DR_EXTI_IRQn EXTI2_IRQn
#define MAX31865_PWR_Pin LL_GPIO_PIN_3
#define MAX31865_PWR_GPIO_Port GPIOD
#define TOUCH_RST_Pin LL_GPIO_PIN_6
#define TOUCH_RST_GPIO_Port GPIOD
#define TOUCH_WAKE_Pin LL_GPIO_PIN_3
#define TOUCH_WAKE_GPIO_Port GPIOB
#define TOUCH_PEN_Pin LL_GPIO_PIN_4
#define TOUCH_PEN_GPIO_Port GPIOB
#define TOUCH_PEN_EXTI_IRQn EXTI4_IRQn
#define TOUCH_INT_Pin LL_GPIO_PIN_5
#define TOUCH_INT_GPIO_Port GPIOB
#define TOUCH_INT_EXTI_IRQn EXTI9_5_IRQn
#define TOUCH_SCL_Pin LL_GPIO_PIN_6
#define TOUCH_SCL_GPIO_Port GPIOB
#define TOUCH_SDA_Pin LL_GPIO_PIN_7
#define TOUCH_SDA_GPIO_Port GPIOB
#define STV_PWR_Pin LL_GPIO_PIN_8
#define STV_PWR_GPIO_Port GPIOB
#define STV_PWR_MODE_Pin LL_GPIO_PIN_9
#define STV_PWR_MODE_GPIO_Port GPIOB
#define STV_PWR_INC_Pin LL_GPIO_PIN_0
#define STV_PWR_INC_GPIO_Port GPIOE
#define STV_PWR_DEC_Pin LL_GPIO_PIN_1
#define STV_PWR_DEC_GPIO_Port GPIOE

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
