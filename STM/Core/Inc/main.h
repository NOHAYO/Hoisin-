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
#include "stm32l4xx_hal.h"

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
#define PMIC_PFN2_Pin GPIO_PIN_14
#define PMIC_PFN2_GPIO_Port GPIOG
#define NIM_SWDCLK_Pin GPIO_PIN_14
#define NIM_SWDCLK_GPIO_Port GPIOA
#define SHDNL_N_Pin GPIO_PIN_12
#define SHDNL_N_GPIO_Port GPIOC
#define PMIC_MPC_Pin GPIO_PIN_13
#define PMIC_MPC_GPIO_Port GPIOG
#define PMIC_ALRT_N_Pin GPIO_PIN_13
#define PMIC_ALRT_N_GPIO_Port GPIOC
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define NIM_SWDIO_Pin GPIO_PIN_13
#define NIM_SWDIO_GPIO_Port GPIOA
#define PMIC_INT_N_Pin GPIO_PIN_5
#define PMIC_INT_N_GPIO_Port GPIOB
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define NIM_UART_RX_Pin GPIO_PIN_10
#define NIM_UART_RX_GPIO_Port GPIOA
#define NIM_P2_2_Pin GPIO_PIN_9
#define NIM_P2_2_GPIO_Port GPIOC
#define NIM_UART_TX_Pin GPIO_PIN_9
#define NIM_UART_TX_GPIO_Port GPIOA
#define J3_17_Pin GPIO_PIN_8
#define J3_17_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOC
#define LED0_Pin GPIO_PIN_8
#define LED0_GPIO_Port GPIOC
#define PWR_ON_WIFI_Pin GPIO_PIN_2
#define PWR_ON_WIFI_GPIO_Port GPIOC
#define WAKE_UP_WIFI_Pin GPIO_PIN_1
#define WAKE_UP_WIFI_GPIO_Port GPIOC
#define PMIC_Mon_Pin GPIO_PIN_0
#define PMIC_Mon_GPIO_Port GPIOC
#define SPI1_WP_IO2_Pin GPIO_PIN_11
#define SPI1_WP_IO2_GPIO_Port GPIOB
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define SPI1_FRST_IO3_Pin GPIO_PIN_10
#define SPI1_FRST_IO3_GPIO_Port GPIOB
#define A_SPI0_CSACC_N_Pin GPIO_PIN_3
#define A_SPI0_CSACC_N_GPIO_Port GPIOA
#define SPI1_CSW_N_Pin GPIO_PIN_2
#define SPI1_CSW_N_GPIO_Port GPIOB
#define ACC_INT_Pin GPIO_PIN_1
#define ACC_INT_GPIO_Port GPIOB
#define OPT_INT_Pin GPIO_PIN_0
#define OPT_INT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
