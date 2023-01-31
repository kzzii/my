/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32g4xx_ll_iwdg.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_crc.h"
#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"
#include "stm32g4xx_hal_flash_ramfunc.h"


#include<stdbool.h>

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

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
void SPI3_Rx_Callback(void);
void SPI3_Tx_Callback(void);
void SPI3_TransferError_Callback(void);
void LED_Blinking(uint32_t Period);

///* USER CODE BEGIN EFP */
extern uint8_t aTxBuffer[11];
//extern uint8_t ubNbDataToTransmit;
//extern __IO int ubTransmitIndex;
//
///* Buffer used for reception */
//extern uint8_t aRxBuffer[20];
//extern uint8_t ubNbDataToReceive;
//extern __IO int ubReceiveIndex;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SEL_CS1_Pin LL_GPIO_PIN_0
#define SEL_CS1_GPIO_Port GPIOC
#define SEL_CS2_Pin LL_GPIO_PIN_1
#define SEL_CS2_GPIO_Port GPIOC
#define SEL_CS3_Pin LL_GPIO_PIN_2
#define SEL_CS3_GPIO_Port GPIOC
#define SEL_CS4_Pin LL_GPIO_PIN_3
#define SEL_CS4_GPIO_Port GPIOC
#define mSlot_ID_0_Pin LL_GPIO_PIN_0
#define mSlot_ID_0_GPIO_Port GPIOA
#define mSlot_ID_1_Pin LL_GPIO_PIN_1
#define mSlot_ID_1_GPIO_Port GPIOA
#define mSlot_ID_2_Pin LL_GPIO_PIN_2
#define mSlot_ID_2_GPIO_Port GPIOA
#define SPI1_CS1_Pin LL_GPIO_PIN_3
#define SPI1_CS1_GPIO_Port GPIOA
#define SPI1_CS2_Pin LL_GPIO_PIN_4
#define SPI1_CS2_GPIO_Port GPIOA
#define Cell1_Balancing_Pin LL_GPIO_PIN_4
#define Cell1_Balancing_GPIO_Port GPIOC
#define Cell2_Balancing_Pin LL_GPIO_PIN_5
#define Cell2_Balancing_GPIO_Port GPIOC
#define mLED1_Pin LL_GPIO_PIN_0
#define mLED1_GPIO_Port GPIOB
#define mLED2_Pin LL_GPIO_PIN_1
#define mLED2_GPIO_Port GPIOB
#define LEVEL_SEL_Pin LL_GPIO_PIN_2
#define LEVEL_SEL_GPIO_Port GPIOB
#define SPI2_CS1_Pin LL_GPIO_PIN_11
#define SPI2_CS1_GPIO_Port GPIOB
#define SPI2_CS2_Pin LL_GPIO_PIN_12
#define SPI2_CS2_GPIO_Port GPIOB
#define Cell3_Balancing_Pin LL_GPIO_PIN_6
#define Cell3_Balancing_GPIO_Port GPIOC
#define Cell4_Balancing_Pin LL_GPIO_PIN_7
#define Cell4_Balancing_GPIO_Port GPIOC
#define SPI3_CS_Pin LL_GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
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
#define LED_BLINK_FAST  200
#define LED_BLINK_SLOW  500
#define LED_BLINK_ERROR 1000

#define MAIN_rLED1_Toggle()	LL_GPIO_TogglePin(mLED1_GPIO_Port, mLED1_Pin);
#define MAIN_yLED2_Toggle()	LL_GPIO_TogglePin(mLED2_GPIO_Port, mLED2_Pin);
#define MAIN_rLED1_ON()		LL_GPIO_SetOutputPin(mLED1_GPIO_Port, mLED1_Pin);
#define MAIN_yLED2_ON()		LL_GPIO_SetOutputPin(mLED2_GPIO_Port, mLED2_Pin);
#define MAIN_rLED1_OFF()	LL_GPIO_ResetOutputPin(mLED1_GPIO_Port, mLED1_Pin);
#define MAIN_yLED2_OFF()	LL_GPIO_ResetOutputPin(mLED2_GPIO_Port, mLED2_Pin);

#define BALANCING_CELL1_Toggle()	LL_GPIO_TogglePin(Cell1_Balancing_GPIO_Port, Cell1_Balancing_Pin);
#define BALANCING_CELL2_Toggle()	LL_GPIO_TogglePin(Cell2_Balancing_GPIO_Port, Cell2_Balancing_Pin);
#define BALANCING_CELL3_Toggle()	LL_GPIO_TogglePin(Cell3_Balancing_GPIO_Port, Cell3_Balancing_Pin);
#define BALANCING_CELL4_Toggle()	LL_GPIO_TogglePin(Cell4_Balancing_GPIO_Port, Cell4_Balancing_Pin);

#define BALANCING_CELL1_ON() LL_GPIO_SetOutputPin(Cell1_Balancing_GPIO_Port, Cell1_Balancing_Pin);
#define BALANCING_CELL2_ON() LL_GPIO_SetOutputPin(Cell2_Balancing_GPIO_Port, Cell2_Balancing_Pin);
#define BALANCING_CELL3_ON() LL_GPIO_SetOutputPin(Cell3_Balancing_GPIO_Port, Cell3_Balancing_Pin);
#define BALANCING_CELL4_ON() LL_GPIO_SetOutputPin(Cell4_Balancing_GPIO_Port, Cell4_Balancing_Pin);
#define BALANCING_CELL1_OFF() LL_GPIO_ResetOutputPin(Cell1_Balancing_GPIO_Port, Cell1_Balancing_Pin);
#define BALANCING_CELL2_OFF() LL_GPIO_ResetOutputPin(Cell2_Balancing_GPIO_Port, Cell2_Balancing_Pin);
#define BALANCING_CELL3_OFF() LL_GPIO_ResetOutputPin(Cell3_Balancing_GPIO_Port, Cell3_Balancing_Pin);
#define BALANCING_CELL4_OFF() LL_GPIO_ResetOutputPin(Cell4_Balancing_GPIO_Port, Cell4_Balancing_Pin);

/* ADC,DAC SPI */
#define HW_SPI1_CS1_LOW()	LL_GPIO_ResetOutputPin(SPI1_CS1_GPIO_Port, SPI1_CS1_Pin);
#define HW_SPI1_CS1_HIGH()	LL_GPIO_SetOutputPin(SPI1_CS1_GPIO_Port, SPI1_CS1_Pin);
#define HW_SPI1_CS2_LOW()	LL_GPIO_ResetOutputPin(SPI1_CS2_GPIO_Port, SPI1_CS2_Pin);
#define HW_SPI1_CS2_HIGH()	LL_GPIO_SetOutputPin(SPI1_CS2_GPIO_Port, SPI1_CS2_Pin);

#define HW_SPI2_CS1_LOW()	LL_GPIO_ResetOutputPin(SPI2_CS1_GPIO_Port, SPI2_CS1_Pin);
#define HW_SPI2_CS1_HIGH()	LL_GPIO_SetOutputPin(SPI2_CS1_GPIO_Port, SPI2_CS1_Pin);
#define HW_SPI2_CS2_LOW()	LL_GPIO_ResetOutputPin(SPI2_CS2_GPIO_Port, SPI2_CS2_Pin);
#define HW_SPI2_CS2_HIGH()	LL_GPIO_SetOutputPin(SPI2_CS2_GPIO_Port, SPI2_CS2_Pin);

#define HW_SEL_CS1_EN()		LL_GPIO_SetOutputPin(SEL_CS1_GPIO_Port, SEL_CS1_Pin);
#define HW_SEL_CS2_EN()		LL_GPIO_SetOutputPin(SEL_CS2_GPIO_Port, SEL_CS2_Pin);
#define HW_SEL_CS3_EN()		LL_GPIO_SetOutputPin(SEL_CS3_GPIO_Port, SEL_CS3_Pin);
#define HW_SEL_CS4_EN()		LL_GPIO_SetOutputPin(SEL_CS4_GPIO_Port, SEL_CS4_Pin);

#define HW_SEL_CS1_DIS()	LL_GPIO_ResetOutputPin(SEL_CS1_GPIO_Port, SEL_CS1_Pin);
#define HW_SEL_CS2_DIS()	LL_GPIO_ResetOutputPin(SEL_CS2_GPIO_Port, SEL_CS2_Pin);
#define HW_SEL_CS3_DIS()	LL_GPIO_ResetOutputPin(SEL_CS3_GPIO_Port, SEL_CS3_Pin);
#define HW_SEL_CS4_DIS()	LL_GPIO_ResetOutputPin(SEL_CS4_GPIO_Port, SEL_CS4_Pin);

#define HW_SPI_CELL1_En(){	LL_GPIO_ResetOutputPin(SEL_CS1_GPIO_Port, SEL_CS1_Pin); LL_GPIO_SetOutputPin(SEL_CS2_GPIO_Port, SEL_CS2_Pin);}
#define HW_SPI_CELL2_En(){	LL_GPIO_ResetOutputPin(SEL_CS2_GPIO_Port, SEL_CS2_Pin); LL_GPIO_SetOutputPin(SEL_CS1_GPIO_Port, SEL_CS1_Pin);}
#define HW_SPI_CELL3_En(){	LL_GPIO_ResetOutputPin(SEL_CS3_GPIO_Port, SEL_CS3_Pin); LL_GPIO_SetOutputPin(SEL_CS4_GPIO_Port, SEL_CS4_Pin);}
#define HW_SPI_CELL4_En(){	LL_GPIO_ResetOutputPin(SEL_CS4_GPIO_Port, SEL_CS4_Pin); LL_GPIO_SetOutputPin(SEL_CS3_GPIO_Port, SEL_CS3_Pin);}

extern uint8_t recvData[20];

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
