/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
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
#include "tim.h"
#include "wwdg.h"

/* USER CODE BEGIN 0 */
__IO ITStatus Timr2Ready = RESET;
__IO ITStatus Timr3Ready = RESET;
static uint32_t SnsrTickCnt = PROCESS_SNSR_TIME;
static uint32_t RdSndTickCnt = PROCESS_RD_SND_TIME;
static uint32_t ledOffCnt = PROCESS_LEDOFF_TIME;

/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* TIM2 init function */
void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}
/* TIM3 init function */
void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
/**
  * @brief  Timer Period Elipsed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Handle appropriate TIMER by setting the key flag.*/
   if(htim->Instance==TIM2)
  {
	Timr2Ready = SET;
  }
  else if(htim->Instance==TIM3)
  {
    // Handle LED Off Count;
    ledOffCnt--;
     if ( ledOffCnt == 0 )
    {
      // Reset Timer;
      ledOffCnt = PROCESS_LEDOFF_TIME;
      RoadBrd_gpio_Off( STATUS_LED );
      RoadBrd_gpio_Off( BGM_LED );
    }
    Timr3Ready = SET;
  }
  else if(htim->Instance==TIM4)
  {
	HAL_IncTick();
  }
}

/**
  * @brief  Clear Timer 2 Flag
  * @param  None
  * @retval None
  */
void Clear_Timer2_Flg( void )
{
  Timr2Ready = RESET;
}

/**
  * @brief  Clear Timer 3 Flag
  * @param  None
  * @retval None
  */
void Clear_Timer3_Flg( void )
{
  Timr3Ready = RESET;
}

/**
  * @brief  Test Timer 2 Flag
  * @param  None
  * @retval bool: true(1)       Timr2Ready SET
  *               false(0)      TImr2Ready RESET
  */
bool Test_Timer2( void )
{
  if (Timr2Ready == SET)
    return true;
  else
    return false;
}

/**
  * @brief  Test Timer 3 Flag
  * @param  None
  * @retval bool: true(1)       Timr3Ready SET
  *               false(0)      TImr3Ready RESET
  */
bool Test_Timer3( void )
{
  if (Timr3Ready == SET)
    return true;
  else
    return false;
}

/**
  * @brief  Processes Timer Flag status and performs requested operations.
  * @param  None
  * @note   This code is the heart beat for handling all sensor stimulus and updating 
  *         the BLE data according to the stimulus
  * @retval None
  */
HAL_StatusTypeDef Proc_Timer2( void )
{
  HAL_StatusTypeDef Status;
  
  Status = HAL_OK;
  if ( Test_Timer2() )
  {
    // Clear Timer 2 flag for next interupt.
    Clear_Timer2_Flg();
    // Handle Road Board Stimulus.
    RdSndTickCnt--;
    if ( RdSndTickCnt == 0 )
    {
      // Reset Timer;
      //RdSndTickCnt = PROCESS_RD_SND_TIME;
      RdSndTickCnt = RoadBrd_Get_RdSndTickCnt();
      // TBD...Processing code for Road Sound goes here.
      // This will be a tasks start here in the future. For now, Will execute the FFT here(Set Time window at least 300msec out of sync to 1 sec timer and at least 5 Seconds.
      Process_RdSound();
      // This code checks to determine if we have an active connection.
      Test_Connection();
    }
    // Handle Sensor Stimulus Count;
    SnsrTickCnt--;
    if ( SnsrTickCnt == 0 )
    {
      // Reset Timer;
      //SnsrTickCnt = PROCESS_SNSR_TIME;
      SnsrTickCnt = RoadBrd_Get_SnsrTickCnt();
      // Enable processing sensors here....
      SetDataReady();
    }
  }
  return Status;
}

/**
  * @brief  Start Timer 2 in Interrupt mode.
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_TIM_StartTimer2( void )
{
  SnsrTickCnt = RoadBrd_Get_SnsrTickCnt();
  RdSndTickCnt = RoadBrd_Get_RdSndTickCnt();
  if (HAL_TIM_Base_Start_IT( &htim2 ) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Start Timer 3 in Interrupt mode.
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_TIM_StartTimer3( void )
{
  if (HAL_TIM_Base_Start_IT( &htim3 ) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Update Key Tick Counts.
  * @param  uint32_t PassedRdSndTickCnt
  * @param  uint32_t PassedSnsrTickCnt
  * @retval None
  */
void Set_TickCounts( uint32_t PassedRdSndTickCnt, uint32_t PassedSnsrTickCnt )
{
  RdSndTickCnt = PassedRdSndTickCnt;
  SnsrTickCnt = PassedSnsrTickCnt;
}

/**
  * @brief  Update RdSndTickCnt.
  * @param  uint32_t PassedRdSndTickCnt
  * @retval None
  */
void Set_RdSndTickCnt( uint32_t PassedRdSndTickCnt )
{
  RdSndTickCnt = PassedRdSndTickCnt;
}

/**
  * @brief  Update SnsrTickCnt.
  * @param  uint32_t PassedSnsrTickCnt
  * @retval None
  */
void Set_SnsrTickCnt( uint32_t PassedSnsrTickCnt )
{
  SnsrTickCnt = PassedSnsrTickCnt;
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
