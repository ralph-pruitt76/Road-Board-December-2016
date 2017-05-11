/**
  ******************************************************************************
  * File Name          : WWDG.c
  * Description        : This file provides code for the configuration
  *                      of the WWDG instances.
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
#include "wwdg.h"
#include "gpio.h"
#include "stdbool.h"
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

WWDG_HandleTypeDef hwwdg;

// wwdg Save Frame
struct
{
  bool  event;
} static wwdg_Save;

// wwdg Save Frame
struct
{
  bool  event;
} wwdg_Save2  @ 0x08070000;

/* WWDG init function */
void MX_WWDG_Init(void)
{

  /*##-2- Configure the WWDG peripheral ######################################*/
  /* WWDG clock counter = (PCLK1 (32MHz)/4096)/8) = 976.6 Hz (1.02ms) 
     WWDG Window value = 80 means that the WWDG counter should be refreshed only 
     when the counter is below 80 (and greater than 64/0x40) otherwise a reset will 
     be generated. 
     WWDG Counter value = 127, WWDG timeout = ~1024 us * 64 = 65.57 ms */
  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
  hwwdg.Init.Window = ROADBRD_HIGHLMIT;               // Set High end of 102.4 ms. 
                                                      // Max_Data = 0x80
  hwwdg.Init.Counter = ROADBRD_TIMEOUT;               // Set Timer at 130.048 ms.
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    Error_Handler();
  }
  // Clear Save Frame.
  wwdg_Save.event = false;
}

void HAL_WWDG_MspInit(WWDG_HandleTypeDef* wwdgHandle)
{

  if(wwdgHandle->Instance==WWDG)
  {
  /* USER CODE BEGIN WWDG_MspInit 0 */

  /* USER CODE END WWDG_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_WWDG_CLK_ENABLE();
  /* USER CODE BEGIN WWDG_MspInit 1 */

  /* USER CODE END WWDG_MspInit 1 */
  }
}

void HAL_WWDG_MspDeInit(WWDG_HandleTypeDef* wwdgHandle)
{

  if(wwdgHandle->Instance==WWDG)
  {
  /* USER CODE BEGIN WWDG_MspDeInit 0 */

  /* USER CODE END WWDG_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_WWDG_CLK_DISABLE();
  }
  /* USER CODE BEGIN WWDG_MspDeInit 1 */

  /* USER CODE END WWDG_MspDeInit 1 */
} 

/* USER CODE BEGIN 1 */
/**
  * @brief  Start WWDG Timer
  * @param  none
  * @retval HAL_StatusTypeDef:     HAL_OK:       Error Code logged
  *                                HAL_ERROR:    Error Log full or Bad Code
  */
HAL_StatusTypeDef RoadBrd_WWDG_Start( void )
{
  //return HAL_WWDG_Start(&hwwdg);
  return HAL_WWDG_Start_IT(&hwwdg);
}

/**
  * @brief  Refresh WWDG Timer
  * @param  none
  * @retval HAL_StatusTypeDef:     HAL_OK:       Error Code logged
  *                                HAL_ERROR:    Error Log full or Bad Code
  */
HAL_StatusTypeDef RoadBrd_WWDG_Refresh( void )
{
  uint32_t rfrsh_cnt;
  
  // Let's Only Refresh in Window...
  rfrsh_cnt = RoadBrd_WWDG_GetRefreshCnt();
  if ( rfrsh_cnt < ROADBRD_HIGHLMIT)
    return HAL_WWDG_Refresh(&hwwdg, ROADBRD_TIMEOUT);
  else
    return HAL_OK;
}

/**
  * @brief  Refresh WWDG Timer
  * @param  none
  * @retval HAL_StatusTypeDef:     HAL_OK:       Error Code logged
  *                                HAL_ERROR:    Error Log full or Bad Code
  */
uint32_t RoadBrd_WWDG_GetRefreshCnt( void )
{
  // Return the 7-Bit Counter Value.
  return(READ_REG(hwwdg.Instance->CR) & 0x7f);                 // hwwdg->Instance->CR
}

    
void HAL_WWDG_WakeupCallback(WWDG_HandleTypeDef* hwwdg)
{
  //uint8_t tempBffr2[20];

  static bool OnceFlg = false;
  
  if ( OnceFlg )
  {
    wwdg_Save.event = true;   // We have had a wwdg Event...Mark that it did occur.
//    if (wwdg_Save2.event)
//      OnceFlg = false;
  }
  else
  {
    OnceFlg = true;
  }
}

/**
* @brief This function handles wwdt global interrupt.
*/
void WWDG_IRQHandler(void)
{
  HAL_WWDG_IRQHandler(&hwwdg);
}

bool RoadBrd_WWDG_TstEvent( void )
{
  return wwdg_Save.event;
}
/* USER CODE END 1 */


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
