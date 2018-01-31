/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
/* USER CODE BEGIN 0 */
const uint32_t RoadBrd_LED_PORT[RoadBrd_LEDn] = {(uint32_t)LED2_GPIO_PORT, 
                                                (uint32_t)BLUE_GPIO_PORT, 
                                                (uint32_t)GREEN_GPIO_PORT, 
                                                (uint32_t)YELLOW_GPIO_PORT,
                                                (uint32_t)TAM_PWR_GPIO_PORT,
                                                (uint32_t)VDD_PWR_GPIO_PORT,
                                                (uint32_t)RESET_BGM111_GPIO_PORT,
                                                (uint32_t)CHARGE_ON_GPIO_PORT,
                                                (uint32_t)HEAT_ON_GPIO_PORT,
                                                (uint32_t)GPIOB,
                                                (uint32_t)PC0_GPIO_PORT,
                                                (uint32_t)PC1_GPIO_PORT,
                                                (uint32_t)PC2_GPIO_PORT,
                                                (uint32_t)PC3_GPIO_PORT,
                                                (uint32_t)PC4_GPIO_PORT};

const uint16_t RoadBrd_LED_PIN[RoadBrd_LEDn] = {LED2_PIN, 
                                                BLUE_PIN, 
                                                GREEN_PIN, 
                                                YELLOW_PIN,
                                                TAM_PWR_PIN,
                                                VDD_PWR_PIN,
                                                RESET_BGM111_PIN,
                                                CHARGE_ON_PIN,
                                                HEAT_ON_PIN,
                                                I2C_SCL_Pin,
                                                PC0_PIN,
                                                PC1_PIN,
                                                PC2_PIN,
                                                PC3_PIN,
                                                PC4_PIN};

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

#ifdef WM
//****************************************************************************************
// WEATHERMESH User Code starts here. Custom GPIO Initialization
// Configure PC0, PC1, PC2, PC3, and PC4 as Input or Output here.
// CURRENT CONFIGURATION
//      PC0: Input, Analog, No Pullup
//      PC1: Input, Analog, No Pullup
//      PC2: Input, Analog, No Pullup
//      PC3: Input, Analog, No Pullup
//      PC4: Output, Speed:Low, Push-Pull, No Pullup, Output=LOW
//
/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
void WM_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  //GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PC4_PIN, GPIO_PIN_RESET);

}

// END OF ALL User Code for WM Here.
//*****************************************************************************************     
#endif
/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 
                           PC4 PC5 PC6 PC7 
                           PC8 PC9 PC10 PC11 
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA6 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin */
  GPIO_InitStruct.Pin = LD2_Pin|CHARGE_ON_Pin|HEAT_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB2 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  //GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin 
                           PBPin */
  GPIO_InitStruct.Pin = TAM_PWR_Pin|LED_MICRO_Pin|RESET_BGM111_Pin|LED_STATUS_Pin 
                          |LED_BGM111_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|CHARGE_ON_Pin|HEAT_ON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TAM_PWR_Pin|LED_MICRO_Pin|RESET_BGM111_Pin|LED_STATUS_Pin 
                          |LED_BGM111_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 2 */
/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *     @arg LED2
  * @retval None
  */
void RoadBrd_LED_On(RoadBrd_Led_TypeDef Led)
{
  GPIO_TypeDef* temp_GPIOx;
  temp_GPIOx = (GPIO_TypeDef*)RoadBrd_LED_PORT[Led];
  HAL_GPIO_WritePin(temp_GPIOx, RoadBrd_LED_PIN[Led], GPIO_PIN_SET); 
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg LED2
  * @retval None
  */
void RoadBrd_LED_Off(RoadBrd_Led_TypeDef Led)
{
  GPIO_TypeDef* temp_GPIOx;
  temp_GPIOx = (GPIO_TypeDef*)RoadBrd_LED_PORT[Led];
  HAL_GPIO_WritePin(temp_GPIOx, RoadBrd_LED_PIN[Led], GPIO_PIN_RESET); 
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *            @arg  LED2
  * @retval None
  */
void RoadBrd_LED_Toggle(RoadBrd_Led_TypeDef Led)
{
  GPIO_TypeDef* temp_GPIOx;
  temp_GPIOx = (GPIO_TypeDef*)RoadBrd_LED_PORT[Led];
  HAL_GPIO_TogglePin(temp_GPIOx, RoadBrd_LED_PIN[Led]);
}

/**
  * @brief  Turns selected gpio On.
  * @param  Led: Specifies the gpio to be set on. 
  *   This parameter can be one of following parameters:
  * @retval None
  */
void RoadBrd_gpio_On(RoadBrd_Led_TypeDef Port)
{
  GPIO_TypeDef* temp_GPIOx;
  temp_GPIOx = (GPIO_TypeDef*)RoadBrd_LED_PORT[Port];
  HAL_GPIO_WritePin(temp_GPIOx, RoadBrd_LED_PIN[Port], GPIO_PIN_SET); 
}

/**
  * @brief  Turns selected gpio Off.
  * @param  Led: Specifies the gpio to be set off. 
  *   This parameter can be one of following parameters:
  * @retval None
  */
void RoadBrd_gpio_Off(RoadBrd_Led_TypeDef Port)
{
  GPIO_TypeDef* temp_GPIOx;
  temp_GPIOx = (GPIO_TypeDef*)RoadBrd_LED_PORT[Port];
  HAL_GPIO_WritePin(temp_GPIOx, RoadBrd_LED_PIN[Port], GPIO_PIN_RESET); 
}

/**
  * @brief  Toggles the selected gpio.
  * @param  Led: Specifies the gpio to be toggled. 
  *   This parameter can be one of following parameters:
  * @retval None
  */
void RoadBrd_gpio_Toggle(RoadBrd_Led_TypeDef Port)
{
  GPIO_TypeDef* temp_GPIOx;
  temp_GPIOx = (GPIO_TypeDef*)RoadBrd_LED_PORT[Port];
  HAL_GPIO_TogglePin(temp_GPIOx, RoadBrd_LED_PIN[Port]);
}


/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
