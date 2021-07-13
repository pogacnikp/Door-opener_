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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
#include "buzzer_control.h"
#include "lin_communication.h"
#include "motor_control.h"
#include "motor_diag.h"
#include "power_supply_det.h"
#include "switch.h"
#include "flash_fnc.h"
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
/* USER CODE BEGIN Private defines */
/* Main machine states */
#define MS_MAIN_START 	0
#define MS_MAIN_S1 		10
#define MS_MAIN_S2 		20
#define MS_MAIN_S3 		30

/* Locking (Unlocking) machine states */
#define MS_LCK_MOTOR 		0
#define MS_LCK_END_POSIT 	10
#define MS_LCK_IDLE 		20

/* Set door opener to lock/unlock */
#define DO_SET_LOCK 	0
#define DO_SET_UNLOCK 	1

/* Number of ADC channels */
#define ADC_CH_NUM		4

/* Level store address in FLASH
 * Note: Last 4 bytes in FLASH to prevent data overwrite */
#define SWITCH_LEVEL_ADDR 0x08007FFB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
