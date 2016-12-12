/**
  ******************************************************************************
  * File Name          : I2C.h
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
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
#ifndef __i2c_H
#define __i2c_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stm32l1xx_nucleo.h"
#include "stm32l1xx_hal_i2c.h"
   
/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN Private defines */
#define ONE_SECOND_DELAY        200             // 200 5msec ticks.
#define FIVE_SECOND_DELAY       1000            // 1000 5msec ticks.

/* Size of Transmission buffer */
#define TXBUFFERSIZE                      80
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE
#define NULL_SIZE                         0
//#define I2C_MAX_TRIES                     100
#define I2C_MAX_TRIES                     1

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

/* USER CODE END Private defines */

extern void Error_Handler(void);

void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */
void MX_I2C1_Reset(void);
HAL_StatusTypeDef RoadBrd_I2C_Master_Transmit_CMDData_IT(uint16_t DevAddress, uint8_t Command, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef RoadBrd_I2C_Master_Transmit_CMDData(uint16_t DevAddress, uint8_t Command, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef RoadBrd_I2C_Master_Transmit_IT(uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef RoadBrd_I2C_Master_Transmit(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef RoadBrd_I2C_Master_Receive_IT(uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef RoadBrd_I2C_Master_CmdReceive(uint16_t DevAddress, uint8_t Command, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef RoadBrd_I2C_Master_Receive(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_I2C_StateTypeDef RoadBrd_I2C_GetState( void );
HAL_StatusTypeDef RoadBrd_WaitForState( uint16_t WaitCnt );
void RoadBrd_Delay( __IO uint32_t Delay );
uint32_t RoadBrd_I2C_GetError( void );
HAL_StatusTypeDef I2C_WaitBusyFlag(void);
HAL_StatusTypeDef RoadBrd_TestI2C( void );

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ i2c_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
