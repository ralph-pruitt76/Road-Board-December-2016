/**
  ******************************************************************************
  * File Name          : I2C.c
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

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "wwdg.h"

#include "gpio.h"
    
/* USER CODE BEGIN 0 */
#include "stm32l1xx_hal.h"
#include "ErrorCodes.h"
#include "app_data.h"
#include "usart.h"

#define I2C_TIMEOUT_FLAG          ((uint32_t)35)      /* 35 ms */

static HAL_StatusTypeDef RdBrdI2C_WaitOnFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Flag, FlagStatus Status, uint32_t Timeout);

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 20000;
  //hi2c1.Init.ClockSpeed = 10000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    GPIO_InitStruct.Pin = I2C_SCL_Pin|I2C_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(GPIOB, I2C_SCL_Pin|I2C_SDA_Pin);

  }
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
} 

/* USER CODE BEGIN 1 */
/* I2C1 Reset Function */
void MX_I2C1_Reset(void)
{
  // Disabe I2C first.
  __HAL_I2C_DISABLE(&hi2c1);
  
  // Soft Reset I2C Now.
  SET_BIT(hi2c1.Instance->CR1, I2C_CR1_SWRST);
  
  // Hold Reset for 5 msec.
  RoadBrd_Delay( 5 );
   
  // Clear Reset State.
  CLEAR_BIT(hi2c1.Instance->CR1, I2C_CR1_SWRST);
  
  // Wait for 5 msec.
  RoadBrd_Delay( 5 );

  // Call Init Function.
  MX_I2C1_Init();
}

/**
  * @brief  HAL_StatusTypeDef RoadBrd_I2C_Master_Transmit_CMDData_IT(uint16_t DevAddress, uint8_t Command, uint8_t *pData, uint16_t Size)
  * @param  DevAddress Target device address
  * @param  Command 8 bit Command to be tasked as the first byte to the I2C Channel.
  * @param  pData Pointer to data buffer                
  * @param  Size Amount of data to be sent  TXBUFFERSIZE=80 Bytes
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_I2C_Master_Transmit_CMDData_IT(uint16_t DevAddress, uint8_t Command, uint8_t *pData, uint16_t Size)
{ 
  int x;
  uint8_t TempBuffer[TXBUFFERSIZE+2];
  
  // Test o see if data is legal.
  if (Size <= TXBUFFERSIZE)
  {
    // Now build Temp Buffer for tasking.
    TempBuffer[0] = Command;
    if( Size>0 )
    {
      for(x=0; x<Size; x++)
        TempBuffer[x+1] = pData[x];
      TempBuffer[Size+1] = 0x00;
      Size++;
    }
    else
    {
      TempBuffer[1] = 0x00;
      Size = 1;
    }
    return RoadBrd_I2C_Master_Transmit_IT(DevAddress, TempBuffer, Size);
  }
  else
    return HAL_ERROR;
}

/**
  * @brief  HAL_StatusTypeDef RoadBrd_I2C_Master_Transmit_CMDData(uint16_t DevAddress, uint8_t Command, uint8_t *pData, uint16_t Size, uint32_t Timeout)
  * @param  DevAddress Target device address
  * @param  Command 8 bit Command to be tasked as the first byte to the I2C Channel.
  * @param  pData Pointer to data buffer                
  * @param  Size Amount of data to be sent  TXBUFFERSIZE=80 Bytes
  * @param  Timeout NUmber of 1ms ticks to wait.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_I2C_Master_Transmit_CMDData(uint16_t DevAddress, uint8_t Command, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{ 
  int x;
  uint8_t TempBuffer[TXBUFFERSIZE+2];
  
  // Test o see if data is legal.
  if (Size <= TXBUFFERSIZE)
  {
    // Now build Temp Buffer for tasking.
    TempBuffer[0] = Command;
    if( Size>0 )
    {
      for(x=0; x<Size; x++)
        TempBuffer[x+1] = pData[x];
      TempBuffer[Size+1] = 0x00;
      Size++;
    }
    else
    {
      TempBuffer[1] = 0x00;
      Size = 1;
    }
    return RoadBrd_I2C_Master_Transmit(DevAddress, TempBuffer, Size, Timeout);
  }
  else
    return HAL_ERROR;
}

/**
  * @brief  HAL_StatusTypeDef RoadBrd_I2C_Master_Transmit_IT(uint16_t DevAddress, uint8_t *pData, uint16_t Size)
  * @param  DevAddress Target device address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_I2C_Master_Transmit_IT(uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{ 
    HAL_StatusTypeDef Status;

    // Turn On STATUS_LED LED.
#ifndef NUCLEO
#ifndef LED_OFF
RoadBrd_gpio_On( STATUS_LED );
#endif
#endif
    Status = HAL_I2C_Master_Transmit_IT(&hi2c1, (uint16_t)DevAddress, (uint8_t*)pData, Size);
    // Turn Off STATUS_LED LED.
#ifndef NUCLEO
#ifndef LED_OFF
    RoadBrd_gpio_Off( STATUS_LED );
#endif
#endif
    
    return Status;
}

/**
  * @brief  HAL_StatusTypeDef RoadBrd_I2C_Master_Transmit(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
  * @param  DevAddress Target device address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout NUmber of 1ms ticks to wait.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_I2C_Master_Transmit(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{ 
    HAL_StatusTypeDef Status;

    // Turn On STATUS_LED LED.
#ifndef NUCLEO
#ifndef LED_OFF
    RoadBrd_gpio_On( STATUS_LED );
#endif
#endif
    Status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DevAddress, (uint8_t*)pData, Size, Timeout);
    // Turn Off STATUS_LED LED.
#ifndef NUCLEO
#ifndef LED_OFF
    RoadBrd_gpio_Off( STATUS_LED );
#endif
#endif
    
    return Status;
}

/**
  * @brief  HAL_StatusTypeDef RoadBrd_I2C_Master_CmdReceive(uint16_t DevAddress, uint8_t Command, uint8_t *pData, uint16_t Size, uint32_t Timeout): 
  *             Receive in master mode an amount of data in blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be received.
  * @param  Timeout NUmber of 1ms ticks to wait.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_I2C_Master_CmdReceive(uint16_t DevAddress, uint8_t Command, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  HAL_StatusTypeDef Status;
  uint8_t TempBuffer[2];

  // Now build Temp Buffer for tasking.
  TempBuffer[0] = Command;
  TempBuffer[1] = 0x00;
  
  // Turn On STATUS_LED LED.
#ifndef NUCLEO
#ifndef LED_OFF
  RoadBrd_gpio_On( STATUS_LED );
#endif
#endif
  Status = RoadBrd_I2C_Master_Transmit(DevAddress, TempBuffer, 1, Timeout);
  if(Status != HAL_OK)
    return Status;
  
  Status = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)DevAddress, (uint8_t *)pData, Size, Timeout);
  // Turn Off STATUS_LED LED.
#ifndef NUCLEO
#ifndef LED_OFF
  //RoadBrd_gpio_Off( STATUS_LED );
#endif
#endif
    
  return Status;
}

/**
  * @brief  HAL_StatusTypeDef RoadBrd_I2C_Master_Receive(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout): 
  *             Receive in master mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be received.
  * @param  Timeout NUmber of 1ms ticks to wait.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_I2C_Master_Receive(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    HAL_StatusTypeDef Status;

    // Turn On STATUS_LED LED.
#ifndef NUCLEO
#ifndef LED_OFF
    RoadBrd_gpio_On( STATUS_LED );
#endif
#endif
    Status = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)DevAddress, (uint8_t *)pData, Size, Timeout);
    // Turn Off STATUS_LED LED.
#ifndef NUCLEO
#ifndef LED_OFF
    RoadBrd_gpio_Off( STATUS_LED );
#endif
#endif
    
    return Status;
}

/**
  * @brief  HAL_StatusTypeDef RoadBrd_I2C_Master_Receive_IT(uint16_t DevAddress, uint8_t *pData, uint16_t Size): 
  *             Receive in master mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be received.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_I2C_Master_Receive_IT(uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
    HAL_StatusTypeDef Status;
    
    // Turn On STATUS_LED LED.
#ifndef NUCLEO
#ifndef LED_OFF
    RoadBrd_gpio_On( STATUS_LED );
#endif
#endif
    Status = HAL_I2C_Master_Receive_IT(&hi2c1, (uint16_t)DevAddress, (uint8_t *)pData, Size);
    // Turn Off STATUS_LED LED.
#ifndef NUCLEO
#ifndef LED_OFF
    RoadBrd_gpio_Off( STATUS_LED );
#endif
#endif
    
    return Status;
}

/**
* @brief  Return the I2C error code.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *              the configuration information for the specified I2C.
  * @retval I2C Error Code
*/
uint32_t RoadBrd_I2C_GetError( void )
{
  return HAL_I2C_GetError(&hi2c1);
}


/**
  * @brief  HAL_I2C_StateTypeDef RoadBrd_I2C_GetState( void ): Return the I2C handle state.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL state   :   HAL_I2C_STATE_RESET             = 0x00,   Peripheral is not yet Initialized         
  *                         HAL_I2C_STATE_READY             = 0x20,   Peripheral Initialized and ready for use
  *                         HAL_I2C_STATE_BUSY              = 0x24,   An internal process is ongoing             
  *                         HAL_I2C_STATE_BUSY_TX           = 0x21,   Data Transmission process is ongoing  
  *                         HAL_I2C_STATE_BUSY_RX           = 0x22,   Data Reception process is ongoing        
  *                         HAL_I2C_STATE_TIMEOUT           = 0xA0,   Timeout state                            
  *                         HAL_I2C_STATE_ERROR             = 0xE0    Error                                    
  */
HAL_I2C_StateTypeDef RoadBrd_I2C_GetState( void )
{
  return HAL_I2C_GetState(&hi2c1);
}

/**
  * @brief  HAL_I2C_StateTypeDef RoadBrd_WaitForState( uint16_t WaitCnt ): Wait for state to change
  * @param  uint16_t WaitCnt: Count of 5msec ticks to wait before timing out.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_WaitForState( uint16_t WaitCnt )
{
  uint16_t x;
  
  // Now wait for transaction to complete.
  for( x=0; x<=WaitCnt; x++)
  {
    // Need to Service Watch Dog or we die here....
    RoadBrd_WWDG_Refresh();     // Refresh WatchDog
    // Test to see if event finished.
    if (RoadBrd_I2C_GetState() == HAL_I2C_STATE_READY)
      break;
    // Wait 5msec.
    RoadBrd_Delay(5);
  }
  // Test for timeout.
  if( x == WaitCnt)
    return HAL_TIMEOUT;
  else
    return HAL_OK;
}

/**
  * @brief  void RoadBrd_Delay( __IO uint32_t Delay ): Implement Delay by calling HAL_Delay.
  *             NOTE: This currently uses HAL_Delay. This may se a tasking delay in the future.
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
void RoadBrd_Delay( __IO uint32_t Delay )
{
  HAL_Delay( Delay );
}

HAL_StatusTypeDef I2C_WaitBusyFlag( void )
{
  return(RdBrdI2C_WaitOnFlagUntilTimeout(&hi2c1, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_FLAG));
}

/**
  * @brief  This function handles I2C Communication Timeout.
  * @param  hi2c pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @param  Flag specifies the I2C flag to check.
  * @param  Status The new Flag status (SET or RESET).
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
static HAL_StatusTypeDef RdBrdI2C_WaitOnFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Flag, FlagStatus Status, uint32_t Timeout)
{
  uint32_t tickstart = 0;

  /* Get tick */
  tickstart = HAL_GetTick();

  /* Wait until flag is set */
  if(Status == RESET)
  {
    while(__HAL_I2C_GET_FLAG(hi2c, Flag) == RESET)
    {
      /* Check for the Timeout */
      if(Timeout != HAL_MAX_DELAY)
      {
        if((Timeout == 0)||((HAL_GetTick() - tickstart ) > Timeout))
        {
          hi2c->State= HAL_I2C_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hi2c);

          return HAL_TIMEOUT;
        }
      }
    }
  }
  else
  {
    while(__HAL_I2C_GET_FLAG(hi2c, Flag) != RESET)
    {
      /* Check for the Timeout */
      if(Timeout != HAL_MAX_DELAY)
      {
        if((Timeout == 0)||((HAL_GetTick() - tickstart ) > Timeout))
        {
          hi2c->State= HAL_I2C_STATE_READY;

          /* Process Unlocked */
          __HAL_UNLOCK(hi2c);

          return HAL_TIMEOUT;
        }
      }
    }
  }
  return HAL_OK;
}


/**
  * @brief  Attempts to Clear I2C channel Data Pin by pulsing SDA Line 9 times.
  * @param None.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to I2C success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  */
HAL_StatusTypeDef RoadBrd_I2CRepair( void )
{
  GPIO_InitTypeDef GPIO_InitStructure;
  HAL_StatusTypeDef Status;
  int x, loop_cnt;
  char tempBffr2[8];
  
  Status = HAL_OK;

  GPIO_InitStructure.Pin = I2C_SDA_Pin;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Wait for Hardware to stabilize....10ms
  RoadBrd_Delay( 10 );
  
  GPIO_InitStructure.Pin = I2C_SCL_Pin;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Wait for Hardware to stabilize....100ms
  RoadBrd_Delay( 100 );
  for (loop_cnt=0; loop_cnt<I2C_LOOPCNT; loop_cnt++)
  {
    // Print Loop Count
    sprintf( (char *)tempBffr2, "**%02d.", loop_cnt);
    RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);
    RoadBrd_WWDG_Refresh();     // Refresh WatchDog
    
    // Now...Pulse SDA Pin I2C_CLKRPRCNT Times.
    for (x=0; x<I2C_CLKRPRCNT; x++)
    {
      RoadBrd_gpio_Off( gI2C_CLK );
      RoadBrd_Delay( 10 );               // Wait 10ms;
      RoadBrd_gpio_On(gI2C_CLK);   // Set Clock High.
      RoadBrd_Delay( 10 );               // Wait 10ms;
    } // EndFor (x=0; x<I2C_CLKRPRCNT; x++)
    
    // Finally Reset Clock LOW.
    RoadBrd_gpio_Off(gI2C_CLK);  // Set Clock Low.
    
    // Wait for Hardware to stabilize....100ms
    RoadBrd_Delay( 100 );
    
    // Test Data Pin State.
    if ( HAL_GPIO_ReadPin( GPIOB, I2C_SDA_Pin) == GPIO_PIN_RESET)
    {
      Status = HAL_ERROR;
    }
    else
    {
      // If High, Then we have been succesful. Time to Indicate Repaired and Init I2C BUS.
      //SkPck_ErrCdLogErrCd( REPAIR_I2C, MODULE_i2c );
      // Enable all I2C Sensors.
      //Set_DriverStates( I2C_STATE, DRIVER_ON );
      //Set_DriverStates( IMU_STATE_TASK, DRIVER_ON );
      //Set_DriverStates( IRRADIANCE_MNTR_TASK, DRIVER_ON );
      //Set_DriverStates( PRESSURE_MNTR_TASK, DRIVER_ON );
      // Now Reinit I2C Bus.
      //I2C_LowLevel_Init();
      return HAL_OK;
    }
    RoadBrd_Delay( 100 );
  } // EndFor (loop_cnt=0; loop_cnt<I2C_LOOPCNT; loop_cnt++)
  return Status;
}

/**
  * @brief  Tests I2C channel and sets error codes if failed.
  * @param None.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_TestI2C( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;
  HAL_StatusTypeDef Status;
  
  GPIO_InitStruct.Pin = I2C_SCL_Pin|I2C_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  // Wait for Hardware to stabilize....10ms
  RoadBrd_Delay( 10 );
  
  // Test I2C Clock
  if ( HAL_GPIO_ReadPin( GPIOB, I2C_SCL_Pin) == GPIO_PIN_RESET )
  {
    // If low....Then SCLK has failed...Record Error.
    RdBrd_ErrCdLogErrCd( ERROR_I2C_SCLK, MODULE_i2c );
    Set_DriverStates( I2C_STATE, DRIVER_OFF );
    Status = HAL_ERROR;
    
    // Time to test SDAT
    if ( HAL_GPIO_ReadPin( GPIOB, I2C_SDA_Pin) == GPIO_PIN_RESET)
    {
      // If low....Then SDAT has failed...Record Error.
      RdBrd_ErrCdLogErrCd( ERROR_I2C_SDAT, MODULE_i2c );
    }
  }
  else
  {
    // Passed, Time to test SDAT
    if ( HAL_GPIO_ReadPin( GPIOB, I2C_SDA_Pin) == GPIO_PIN_RESET)
    {
      // If low....Then SDAT has failed...Record Error.
      RdBrd_ErrCdLogErrCd( ERROR_I2C_SDAT, MODULE_i2c );
      Set_DriverStates( I2C_STATE, DRIVER_OFF );
      Status = HAL_ERROR;
    }
    else
    {
      Set_DriverStates( I2C_STATE, DRIVER_ON );
      Status = HAL_OK;
    }
  }
  
  return Status;
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
