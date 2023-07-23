/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

#include "stm32g0xx_ll_ucpd.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_dma.h"

#include "stm32g0xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "YSwitchProc.h"
#include "usbpd_trace.h"
#include "stdio.h"
#include "AQM1248_Driver.h"
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

// User
extern uint32_t g_lastTick;
extern volatile uint16_t g_adcData[4]; // ADC0,1,4,5 updated by isr
extern volatile uint8_t g_swState;

// Display
extern struct sPDDisplayInfo g_dispInfo;

// stored PDO/RDO
extern uint8_t  g_receivedPDOSize, g_receivedFirstAPDOIdx; // 0-7
extern USBPD_PDO_TypeDef g_receivedPDO[16];
extern uint8_t g_curRequestPDOIndex; //  (object position of PDO) - 1
extern USBPD_SNKRDO_TypeDef g_curActiveRDO;

// voltage level
extern uint16_t g_fixedVoltageLevel; // mV
extern uint16_t g_ppsVoltageRange[2], g_ppsVoltageLevel; // min, max
extern uint8_t  g_isPPSMode;

extern uint8_t g_srcPDOReceived;

//#define IS_PPS_MODE() (g_receivedPDO[g_curRequestPDOIndex].GenericPDO.PowerObject == USBPD_CORE_PDO_TYPE_APDO)
#define IS_PPS_MODE() (g_isPPSMode)

uint16_t GetVBUSVoltage(); // result is in mV
//void CurRDO_SetVoltage(uint16_t newVoltage);
uint16_t CurRDO_GetVoltage();
uint16_t CurRDO_GetCurrent();


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW1_Pin GPIO_PIN_9
#define SW1_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_14
#define SW2_GPIO_Port GPIOC
#define SW3_Pin GPIO_PIN_15
#define SW3_GPIO_Port GPIOC
#define SW4_Pin GPIO_PIN_0
#define SW4_GPIO_Port GPIOB
#define SW5_Pin GPIO_PIN_1
#define SW5_GPIO_Port GPIOB
#define VBUS_EN_Pin GPIO_PIN_15
#define VBUS_EN_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_8
#define LCD_CS_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOA
#define LCD_RS_Pin GPIO_PIN_6
#define LCD_RS_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_10
#define LED2_GPIO_Port GPIOA
#define EEP_CS_Pin GPIO_PIN_11
#define EEP_CS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

#define LED_DEBUG_MODE 0 // 1 -> LED1,2 blinks to notify it works to user, 0 -> LED1,2 indicates PD status
#define LED_STATE_OFF 0
#define LED_STATE_RED			1
#define LED_STATE_GREEN			2
#define LED_STATE_ORANGE		3
#define LED_STATE_BLINK			8
#define LED_STATE_BLINK_RED		(LED_STATE_BLINK | LED_STATE_RED)
#define LED_STATE_BLINK_GREEN	(LED_STATE_BLINK | LED_STATE_GREEN)
#define LED_STATE_BLINK_ORANGE	(LED_STATE_BLINK | LED_STATE_ORANGE)
extern volatile uint8_t g_ledState;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
