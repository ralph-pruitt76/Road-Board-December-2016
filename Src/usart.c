/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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
#include "usart.h"
#include <string.h>
#include "parser.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
/* Buffer used for transmission */
uint8_t aTxBuffer[TXBUFFERSIZE];

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];

__IO ITStatus Uart2Ready = RESET;
__IO ITStatus Uart3Ready = RESET;

static uint8_t          *save_Ptr;
static uint16_t         Save_Size;
static uint16_t         tmpSize;
static uint8_t          *tmpPdata;
static uint8_t          tmpData[2];
/* USER CODE END 0 */

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

#ifndef PATCH_UART
  static uint8_t bgm1data[40];
  static bool bffr1Flag = false;
  static int saveLen = 3;
#endif

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;  // 9600Baud
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;  // 9600Baud
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
#ifdef TEST2
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
#else
  #ifdef PATCH_UART
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  #else
    huart3.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  #endif
//  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
#endif
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();
  
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    PB13     ------> USART3_CTS
    PB14     ------> USART3_RTS 
    */
    GPIO_InitStruct.Pin = TX_BGM111_Pin|RX_BGM111_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CTS_BGM111_Pin|RTS_BGM111_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOA, USART_TX_Pin|USART_RX_Pin);

  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();
  
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    PB13     ------> USART3_CTS
    PB14     ------> USART3_RTS 
    */
    HAL_GPIO_DeInit(GPIOB, TX_BGM111_Pin|RX_BGM111_Pin|CTS_BGM111_Pin|RTS_BGM111_Pin);

  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
/**
  * @brief  RoadBrd_UART_Transmit
  * @param  RoadBrd_uart_TypeDef Port: USART Port, uint8_t *pData: Pointer to Data buffer to transmit. 
  * @note   This routine uses the HAL USART routines to send the reqiuested buffer to the requested channel
  *         in a blocking mode. 
  *         It does wait for completion.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_UART_Transmit(RoadBrd_uart_TypeDef Port, uint8_t *pData)
{
  HAL_StatusTypeDef Status;
  uint16_t Size;
  
  Size = strlen((char *)pData);  
  // Test parameters before starting process
  if (Size > TXBUFFERSIZE)
    return HAL_ERROR;
  
  // Test Port to determine which uart to use for this transfer.
  if (Port == USART_2)
  {
    // Is UART Busy right now?
    if (Uart2Ready != RESET)
      return HAL_BUSY;
    else
    {
      Status = HAL_UART_Transmit(&huart2, (uint8_t*)pData, Size, HAL_MAX_DELAY);
      if(Status != HAL_OK)
      {
        return Status;
      }
      else
      {
         return Status;
      }
    }
  }
  else if (Port == USART_3)
  {
    // Is UART Busy right now?
    if (Uart3Ready != RESET)
      return HAL_BUSY;
    else
    {
      Status = HAL_UART_Transmit(&huart3, (uint8_t*)pData, Size, HAL_MAX_DELAY);
      if(Status != HAL_OK)
      {
        return Status;
      }
      else
      {
         return Status;
      }
    }
  }
  else 
    return HAL_ERROR;
}

/**
  * @brief  RoadBrd_UART_Transmit_ITSZ
  * @param  RoadBrd_uart_TypeDef Port: USART Port, uint8_t *pData: Pointer to Data buffer to transmit. 
  * @note   This routine uses the HAL USART routines to send the reqiuested buffer to the requested channel. 
  *         It does not wait for completion and must be tested for completion.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_UART_Transmit_ITSZ(RoadBrd_uart_TypeDef Port, uint8_t *pData, uint16_t Size)
{
  HAL_StatusTypeDef Status;
  
  //Size = strlen((char *)pData);  
  // Test parameters before starting process
  if (Size > TXBUFFERSIZE)
    return HAL_ERROR;
  
  // Test Port to determine which uart to use for this transfer.
  if (Port == USART_2)
  {
    // Is UART Busy right now?
    if (Uart2Ready != RESET)
      return HAL_BUSY;
    else
    {
      Status = HAL_UART_Transmit_IT(&huart2, (uint8_t*)pData, Size);
      if(Status != HAL_OK)
      {
        return Status;
      }
      else
      {
         return Status;
      }
    }
  }
  else if (Port == USART_3)
  {
    // Is UART Busy right now?
    if (Uart3Ready != RESET)
      return HAL_BUSY;
    else
    {
      Status = HAL_UART_Transmit_IT(&huart3, (uint8_t*)pData, Size);
      if(Status != HAL_OK)
      {
        return Status;
      }
      else
      {
         return Status;
      }
    }
  }
  else 
    return HAL_ERROR;
}

/**
  * @brief  RoadBrd_UART_Transmit_IT
  * @param  RoadBrd_uart_TypeDef Port: USART Port, uint8_t *pData: Pointer to Data buffer to transmit. 
  * @note   This routine uses the HAL USART routines to send the reqiuested buffer to the requested channel. 
  *         It does not wait for completion and must be tested for completion.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_UART_Transmit_IT(RoadBrd_uart_TypeDef Port, uint8_t *pData)
{
  HAL_StatusTypeDef Status;
  uint16_t Size;
  
  Size = strlen((char *)pData);  
  // Test parameters before starting process
  if (Size > TXBUFFERSIZE)
    return HAL_ERROR;
  
  // Test Port to determine which uart to use for this transfer.
  if (Port == USART_2)
  {
    // Is UART Busy right now?
    if (Uart2Ready != RESET)
      return HAL_BUSY;
    else
    {
      Status = HAL_UART_Transmit_IT(&huart2, (uint8_t*)pData, Size);
      if(Status != HAL_OK)
      {
        return Status;
      }
      else
      {
         return Status;
      }
    }
  }
  else if (Port == USART_3)
  {
    // Is UART Busy right now?
    if (Uart3Ready != RESET)
      return HAL_BUSY;
    else
    {
      Status = HAL_UART_Transmit_IT(&huart3, (uint8_t*)pData, Size);
      if(Status != HAL_OK)
      {
        return Status;
      }
      else
      {
         return Status;
      }
    }
  }
  else 
    return HAL_ERROR;
}

#ifndef PATCH_UART
/**
  * @brief  RoadBrd_UART_Receive_ITBG
  * @param  RoadBrd_uart_TypeDef Port: USART Port, uint16_t Size: Number of  bytes to wait for completion. 
  * @note   This routine uses the HAL USART routines to receive the requested number of characters to the reqiuested buffer on the requested channel. 
  *         It does not wait for completion and must be tested for completion.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_UART_Receive_ITBG(RoadBrd_uart_TypeDef Port, uint16_t Size)
{
  HAL_StatusTypeDef Status;
  
  // Test parameters before starting process
  if (Size > RXBUFFERSIZE)
    return HAL_ERROR;
  
  // Test Port to determine which uart to use for this transfer.
  if (Port == USART_2)
  {
    // Is UART Busy right now?
    if (Uart2Ready != RESET)
      return HAL_BUSY;
    else
    {
      Status = HAL_UART_Receive_IT(&huart2, (uint8_t*)bgm1data, Size);
      if(Status != HAL_OK)
      {
        return Status;
      }
      else
      {
         return Status;
      }
    }
  }
  else if (Port == USART_3)
  {
    // Is UART Busy right now?
    if (Uart3Ready != RESET)
      return HAL_BUSY;
    else
    {
      Status = HAL_UART_Receive_IT(&huart3, (uint8_t*)bgm1data, Size);
      if(Status != HAL_OK)
      {
        return Status;
      }
      else
      {
         return Status;
      }
    }
  }
  else 
    return HAL_ERROR;
}

/**
  * @brief  Returns the pointer to the bgm1data buffer.
  * @param  None
  * @retval uint8_t*:     address of bgm1data buffer
  */
uint8_t *RoadBrd_GetBGPtr( void )
{
  return &bgm1data[0];
}

/**
  * @brief  Sets the Buffer Flag for BGM Traffic.
  * @param  None
  * @retval None.
  */
void RoadBrd_SetBffrFlg( void )
{
  bffr1Flag = true;
}

/**
  * @brief  Clears the Buffer Flag for BGM Traffic.
  * @param  None
  * @retval None.
  */
void RoadBrd_ClrBffrFlg( void )
{
  bffr1Flag = false;
}

/**
  * @brief  Returns the Buffer Flag for BGM Traffic.
  * @param  None
  * @retval uint8_t*:     address of bgm1data buffer
  */
bool RoadBrd_GetBffrFlg( void )
{
  return bffr1Flag;
}

/**
  * @brief  Return saveLen Variable
  * @param  NONE. 
  * @note   This is needed to allow the next IT transfer to usart. 
* @retval int: saveLen
  */
int RoadBrd_getSaveLen( void )
{
  return saveLen;
}
#endif

/**
  * @brief  RoadBrd_UART_Receive_IT
  * @param  RoadBrd_uart_TypeDef Port: USART Port, uint8_t *pData: Pointer to Data buffer to transmit, uint16_t Size: Number of  bytes to wait for completion. 
  * @note   This routine uses the HAL USART routines to receive the requested number of characters to the reqiuested buffer on the requested channel. 
  *         It does not wait for completion and must be tested for completion.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_UART_Receive_IT(RoadBrd_uart_TypeDef Port, uint8_t *pData, uint16_t Size)
{
  HAL_StatusTypeDef Status;
  
  // Test parameters before starting process
  if (Size > RXBUFFERSIZE)
    return HAL_ERROR;
  
  // Test Port to determine which uart to use for this transfer.
  if (Port == USART_2)
  {
    // Is UART Busy right now?
    if (Uart2Ready != RESET)
      return HAL_BUSY;
    else
    {
      // Initialize all Key Vars from Passes Parameters
      save_Ptr = pData;
      Save_Size = Size;
      tmpSize = Save_Size;
      tmpPdata = save_Ptr;
      
      // Task Usart for First Transfer.
      Status = HAL_UART_Receive_IT(&huart2, (uint8_t*)tmpData, 1);
      if(Status != HAL_OK)
      {
        return Status;
      }
      else
      {
         return Status;
      }
      /* OLD............................................
      Status = HAL_UART_Receive_IT(&huart2, (uint8_t*)pData, Size);
      if(Status != HAL_OK)
      {
        return Status;
      }
      else
      {
         return Status;
      }
      */
    }
  }
  else if (Port == USART_3)
  {
    // Is UART Busy right now?
    if (Uart3Ready != RESET)
      return HAL_BUSY;
    else
    {
      // Initialize all Key Vars from Passes Parameters
      save_Ptr = pData;
      Save_Size = Size;
      tmpSize = Save_Size;
      tmpPdata = save_Ptr;
      
      // Task Usart for First Transfer.
      Status = HAL_UART_Receive_IT(&huart3, (uint8_t*)tmpData, 1);
      if(Status != HAL_OK)
      {
        return Status;
      }
      else
      {
         return Status;
      }
      /* OLD............................................
      Status = HAL_UART_Receive_IT(&huart3, (uint8_t*)pData, Size);
      if(Status != HAL_OK)
      {
        return Status;
      }
      else
      {
         return Status;
      }
    */
    }
  }
  else 
    return HAL_ERROR;
}

/**
  * @brief  RoadBrd_UART_Receive
  * @param  RoadBrd_uart_TypeDef Port: USART Port, uint8_t *pData: Pointer to Data buffer to transmit, uint16_t Size: Number of  bytes to wait for completion. 
  * @note   This routine uses the HAL USART routines to receive the requested number of characters to the reqiuested buffer on the requested channel. 
  *         It DOES wait for completion OR a termination character of 0x0a or 0x0d.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_UART_Receive(RoadBrd_uart_TypeDef Port, uint8_t *pData, uint16_t Size)
{
  HAL_StatusTypeDef Status;
  uint16_t tmpSize;
  uint8_t tmpData[2];
  
  
  // Test parameters before starting process
  if (Size > RXBUFFERSIZE)
    return HAL_ERROR;
  
  // Test Port to determine which uart to use for this transfer.
  if (Port == USART_2)
  {
    // Is UART Busy right now?
    if (Uart2Ready != RESET)
      return HAL_BUSY;
    else
    {
      tmpSize = Size;
      while ( tmpSize>0 )
      {
        Status = HAL_UART_Receive(&huart2, (uint8_t*)tmpData, 1, HAL_MAX_DELAY);
        if(Status == HAL_OK)
        {
          // Watch for termination characters.
          if((tmpData[0]==0x0a) || (tmpData[0]==0x0d))
          {
            *pData = 0x00;
            // Yes..We are done.
            return Status;
          }
          else
          {
            // Move new character into passed buffer.
            *pData = tmpData[0];
            tmpSize--;                          // Decrement Count
            pData++;                            // Move pointer to next buffer location.
          }
        }
      }
      return Status;
    }
  }
  else if (Port == USART_3)
  {
    // Is UART Busy right now?
    if (Uart3Ready != RESET)
      return HAL_BUSY;
    else
    {
      tmpSize = Size;
      while ( tmpSize>0 )
      {
        Status = HAL_UART_Receive(&huart3, (uint8_t*)tmpData, 1, HAL_MAX_DELAY);
        if(Status == HAL_OK)
        {
          // Watch for termination characters.
          if((tmpData[0]==0x0a) || (tmpData[0]==0x0d))
          {
            *pData = 0x00;
            // Yes..We are done.
            return Status;
          }
          else
          {
            // Move new character into passed buffer.
            *pData = tmpData[0];
            tmpSize--;                          // Decrement Count
            pData++;                            // Move pointer to next buffer location.
          }
        }
      }
      return Status;
    }
  }
  else 
    return HAL_ERROR;
}

/**
  * @brief  RoadBrd_Uart_Status
  * @param  RoadBrd_uart_TypeDef Port: USART Port.
  * @note   This routine returns the current state of the UART passed.
  * @retval ITStatus:     RESET:       Last transaction complete
  *                       SET:         UART Channel busy.
  */
ITStatus RoadBrd_Uart_Status(RoadBrd_uart_TypeDef Port)
{
  // Test Port to determine which uart to use for this transfer.
  if ( RoadBrd_Uart_GetState(Port) != HAL_UART_STATE_READY)
    return RESET;
  else
    return SET;
}
  
/**
  * @brief  RoadBrd_Uart_GetState
  * @param  RoadBrd_uart_TypeDef Port: USART Port.
  * @note   This routine returns the current state of the UART passed.
  * @retval ITStatus:     RESET:       Last transaction complete
  *   HAL_UART_STATE_RESET             = 0x00,    < Peripheral is not initialized                      
  *   HAL_UART_STATE_READY             = 0x01,    < Peripheral Initialized and ready for use           
  *   HAL_UART_STATE_BUSY              = 0x02,    < an internal process is ongoing                     
  *   HAL_UART_STATE_BUSY_TX           = 0x12,    < Data Transmission process is ongoing               
  *   HAL_UART_STATE_BUSY_RX           = 0x22,    < Data Reception process is ongoing                  
  *   HAL_UART_STATE_BUSY_TX_RX        = 0x32,    < Data Transmission and Reception process is ongoing 
  *   HAL_UART_STATE_TIMEOUT           = 0x03,    < Timeout state                                      
  *   HAL_UART_STATE_ERROR             = 0x04     < Error                                              
  */
HAL_UART_StateTypeDef RoadBrd_Uart_GetState(RoadBrd_uart_TypeDef Port)
{
  // Test Port to determine which uart to use for this transfer.
  if (Port == USART_2)
    return huart2.State;
  else if (Port == USART_3)
    return huart3.State;
  else
    //return huart3.State;
    return HAL_UART_STATE_ERROR;
}
  
/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of IT Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  // Set transmission flag: trasfer complete for correct USART
  //RoadBrd_gpio_On( BLUE_LED );
  if(UartHandle->Instance==USART2)
  {
    Uart2Ready = SET;
  }
  else if(UartHandle->Instance==USART3)
  {
    Uart3Ready = SET;
  }
}

/**
  * @brief  Clear Usart Ready Flag
  * @param  NONE. 
  * @note   This is needed to allow the next IT transfer to usart. 
  * @retval None
  */
void clrUsartState( RoadBrd_uart_TypeDef Port )
{
  // Test Port to determine which uart to use for this transfer.
  if (Port == USART_2)
  {
    Uart2Ready = RESET;
  }
  else if (Port == USART_3)
  {
    Uart3Ready = RESET;
  }
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* uartHandle)
{
#ifndef PATCH_UART
  int x;
  static uint8_t *temp_ptr;
  static int len;
  int newLen;
#endif

  /* Set transmission flag: trasfer complete for correct USART*/
   if(uartHandle->Instance==USART2)
  {
//***** START HERE
    // Test Bypass Flag...If Set, we ae in special monitor mode.
    if (Tst_Bypass())
    {
      tmpPdata[0] = tmpData[0];
      tmpPdata[1] = 0x00;
      // Set Complete Flag.
      Uart2Ready = SET;        
    } // EndIf (Tst_Bypass())
    else {
      // Watch for termination characters.
      if((tmpData[0]==0x0a) || (tmpData[0]==0x0d) || (tmpSize<=0) )
      {
        *tmpPdata = 0x00;
        // Yes..We are done.
        // Set Complete Flag.
        Uart2Ready = SET;        
      } // EndIf ((tmpData[0]==0x0a) || (tmpData[0]==0x0d) || (tmpSize<=0) )
      // Is this a BackSpace Character?
      else if(tmpData[0]==0x08)
      {
        // YES...Are there any chars to undo?
        if (tmpSize < Save_Size)
        {
          // YES...Undo Previous Character.
          tmpSize++;                          // Decrement Count
          tmpPdata--;                            // Move pointer to Previous buffer location.
          //Time to retask for next Character.
          HAL_UART_Receive_IT(&huart2, (uint8_t*)tmpData, 1);
        } //endif (tmpSize < RECEIVE_SZ)
      } //endif (tmpData[0]==0x08)
      else
      {
        *tmpPdata = tmpData[0];
        tmpSize--;                          // Decrement Count
        tmpPdata++;                            // Move pointer to next buffer location.
        //Time to retask for next Character.
        HAL_UART_Receive_IT(&huart2, (uint8_t*)tmpData, 1);
      }
    } // EndElse (Tst_Bypass())
  }
//***** END HERE
  else if(uartHandle->Instance==USART3)
  {
#ifndef TEST2
#ifndef PATCH_UART
#ifndef LED_OFF
    //RoadBrd_gpio_On( BGM_LED );
#endif
      //if (bffrFlag)
    if (RoadBrd_GetBffrFlg())
    {
      len = 3;
      // Process BGM Byte received.
      temp_ptr = RoadBrd_GetBGPtr();
      newLen = temp_ptr[1]+1;
      for (x=0; x<len; x++)
      {
        RoadBrd_ProcessBGMChar(*temp_ptr);
        temp_ptr++;
        //Status = RoadBrd_ProcessBGMChar(bgmdata[x]);
      }
      len = newLen;
      saveLen = len;
    }
    else
    {
      // Process BGM Byte received.
      temp_ptr = RoadBrd_GetBGPtr();
      for (x=0; x<len; x++)
      {
        RoadBrd_ProcessBGMChar(*temp_ptr);
        temp_ptr++;
        //Status = RoadBrd_ProcessBGMChar(bgmdata[x]);
       }
      len = 3;
      saveLen = len;
      //Status = RoadBrd_ProcessBGMChar(bgmdata[0]);
    }
#endif
#endif
    Uart3Ready = SET;
  }
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef* uartHandle)
{
    //Error_Handler();
}

#ifndef PATCH_UART
/**
  * @brief  Enable UART3 for TX.
  * @param  none
  * @retval None
  */
void HAL_UART_EnableBGM_TXE( void )
{
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_TXE);
}

/**
  * @brief  Enable UART3 for RX.
  * @param  none
  * @retval None
  */
void HAL_UART_EnableBGM_RX( void )
{
  /* Enable the UART Parity Error Interrupt */
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_PE);

  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_ERR);

  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
}

#endif


/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
