/**
  ******************************************************************************
  * File Name          : gpio.h
  * Description        : This file contains all the functions prototypes for 
  *                      the gpio  
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __gpio_H
#define __gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stm32l1xx_nucleo.h"

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
typedef enum 
{
  NUCLEO_LED2 = 0,
  BLUE_LED = 1,
  GREEN_LED = 2,
  YELLOW_LED = 3,
  gTAM_PWR = 4,
  gVDD_PWR = 5,
  gRESET_BGM111 = 6,
  gCHARGE_ON = 7,
  gHEAT_ON = 8,
  gI2C_CLK = 9,
  g_PC0 = 10,
  g_PC1 = 11,
  g_PC2 = 12,
  g_PC3 = 13,
  g_PC4 = 14,
  
  BGM_LED = BLUE_LED,
  MICRO_LED = GREEN_LED,
  STATUS_LED = YELLOW_LED,
  NUCLEO_LED_GREEN = LED2
} RoadBrd_Led_TypeDef;
//#define RoadBrd_LEDn                     10
#define RoadBrd_LEDn                     15

#define LED2_PIN                         GPIO_PIN_5
#define LED2_GPIO_PORT                   GPIOA
#define BLUE_PIN                         GPIO_PIN_9
#define BLUE_GPIO_PORT                   GPIOB
#define GREEN_PIN                        GPIO_PIN_12
#define GREEN_GPIO_PORT                  GPIOB
#define YELLOW_PIN                       GPIO_PIN_8
#define YELLOW_GPIO_PORT                 GPIOB
#define TAM_PWR_PIN                      GPIO_PIN_1
#define TAM_PWR_GPIO_PORT                GPIOB
#define VDD_PWR_PIN                      GPIO_PIN_5
#define VDD_PWR_GPIO_PORT                GPIOB
#define RESET_BGM111_PIN                 GPIO_PIN_15
#define RESET_BGM111_GPIO_PORT           GPIOB
#define CHARGE_ON_PIN                    GPIO_PIN_7
#define CHARGE_ON_GPIO_PORT              GPIOA
#define HEAT_ON_PIN                      GPIO_PIN_8
#define HEAT_ON_GPIO_PORT                GPIOA
#define PC0_PIN                          GPIO_PIN_0
#define PC0_GPIO_PORT                    GPIOC
#define PC1_PIN                          GPIO_PIN_1
#define PC1_GPIO_PORT                    GPIOC
#define PC2_PIN                          GPIO_PIN_2
#define PC2_GPIO_PORT                    GPIOC
#define PC3_PIN                          GPIO_PIN_3
#define PC3_GPIO_PORT                    GPIOC
#define PC4_PIN                          GPIO_PIN_4
#define PC4_GPIO_PORT                    GPIOC

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
void RoadBrd_LED_On(RoadBrd_Led_TypeDef Led);
void RoadBrd_LED_Off(RoadBrd_Led_TypeDef Led);
void RoadBrd_LED_Toggle(RoadBrd_Led_TypeDef Led);
void RoadBrd_gpio_On(RoadBrd_Led_TypeDef Port);
void RoadBrd_gpio_Off(RoadBrd_Led_TypeDef Port);
void RoadBrd_gpio_Toggle(RoadBrd_Led_TypeDef Port);
#ifdef WM
void WM_GPIO_Init(void);
#endif
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
