/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
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
#include "adc.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */
/* Variable to report ADC sequencer status */
uint8_t         ubSequenceCompleted = RESET;     /* Set when all ranks of the sequence have been converted */
uint8_t         adcActive = RESET;               /* If this flag is set, we currently have an active ADC event. */
static RoadBrd_adc_TypeDef ADC_Channel = DEFAULT_AUDIO; /* Set selection to default Adudio Channel 0. */
static RoadBrd_adc_TypeDef Old_ADC_Channel = DEFAULT_AUDIO; /* Set selection to default Adudio Channel 0. */

/* Lookup for ADC Channel Type. */
uint32_t RoadBrd_ADC_PORT[RoadBrd_ADCn] = {ADCx_CHANNEL_Audio, 
                                           ADCx_CHANNEL_3, 
                                           ADCx_CHANNEL_4};

/* USER CODE END 0 */

ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;
ADC_ChannelConfTypeDef sConfig;
  

/**
  * @brief  Configure the global features of the ADC (Clock, Resolution, Data 
  * Alignment and number of conversion. Configure for the selected ADC regular 
  * channel its corresponding rank in the sequencer and its sample time.
  *     ClockPrescaler = ADC_CLOCK_ASYNC_DIV1   ((uint32_t)0x00000000) 
  *                            Select ADC clock source (asynchronous clock derived from HSI RC oscillator) and clock prescaler.
  *                            This parameter can be a value of @ref ADC_ClockPrescaler
  *                            Note: In case of usage of channels on injected group, ADC frequency should be lower than AHB clock frequency /4 for resolution 12 or 10 bits, 
  *                            AHB clock frequency /3 for resolution 8 bits, AHB clock frequency /2 for resolution 6 bits.
  *                            Note: HSI RC oscillator must be preliminarily enabled at RCC top level.
  *     Resolution = ADC_RESOLUTION_12B         ((uint32_t)0x00000000)          !<  ADC 12-bit resolution 
  *     ChannelsBank = ADC_CHANNELS_BANK_A    
  *     NbrOfConversion = 1     Specifies the number of ranks that will be converted within the regular group sequencer.
  *                             To use regular group sequencer and convert several ranks, parameter 'ScanConvMode' must be enabled.
  *                             This parameter must be a number between Min_Data = 1 and Max_Data = 28.
  *     Channel = ADC_CHANNEL_0
  *                             Specifies the channel to configure into ADC regular group.
  *                                        This parameter can be a value of @ref ADC_channels
  *                                        Note: Depending on devices, some channels may not be available on package pins. Refer to device datasheet for channels availability.
  *                                              Maximum number of channels by device category (without taking in account each device package constraints): 
  *                                              STM32L1 category 1, 2: 24 channels on external pins + 3 channels on internal measurement paths (VrefInt, Temp sensor, Vcomp): Channel 0 to channel 26.
  *                                              STM32L1 category 3:    25 channels on external pins + 3 channels on internal measurement paths (VrefInt, Temp sensor, Vcomp): Channel 0 to channel 26, 1 additional channel in bank B. Note: OPAMP1 and OPAMP2 are connected internally but not increasing internal channels number: they are sharing ADC input with external channels ADC_IN3 and ADC_IN8.
  *                                              STM32L1 category 4, 5: 40 channels on external pins + 3 channels on internal measurement paths (VrefInt, Temp sensor, Vcomp): Channel 0 to channel 31, 11 additional channels in bank B. Note: OPAMP1 and OPAMP2 are connected internally but not increasing internal channels number: they are sharing ADC input with external channels ADC_IN3 and ADC_IN8.
  *                                        Note: In case of peripherals OPAMPx not used: 3 channels (3, 8, 13) can be configured as direct channels (fast channels). Refer to macro ' __HAL_ADC_CHANNEL_SPEED_FAST() '.
  *                                        Note: In case of peripheral OPAMP3 and ADC channel OPAMP3 used (OPAMP3 available on STM32L1 devices Cat.4 only): the analog switch COMP1_SW1 must be closed. Refer to macro: ' __HAL_OPAMP_OPAMP3OUT_CONNECT_ADC_COMP1() '.
  *     Rank = 1
  *     SamplingTime = ADC_SAMPLETIME_4CYCLES
  *     
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void MX_ADC_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    /**ADC GPIO Configuration    
    PA0-WKUP1     ------> ADC_IN0
    PA4     ------> ADC_IN4 
    */
    GPIO_InitStruct.Pin = AUD_Pin|AUDA4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral DMA init*/
  
    hdma_adc.Instance = DMA1_Channel1;
    hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_adc.Init.Mode = DMA_NORMAL;
    hdma_adc.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    if (HAL_DMA_Init(&hdma_adc) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC GPIO Configuration    
    PA0-WKUP1     ------> ADC_IN0
    PA4     ------> ADC_IN4 
    */
    HAL_GPIO_DeInit(GPIOA, AUD_Pin|AUDA4_Pin);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  }
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
} 

/* USER CODE BEGIN 1 */
/**
  * @brief  ADC configuration
  * @param  None
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef ADC_Config(void)
{
  /* Configuration of AdcHandle init structure: ADC parameters and regular group */
  hadc.Instance = ADCx;
  
  HAL_StatusTypeDef Status;
  
  Status = HAL_ADC_DeInit(&hadc);
  if (Status != HAL_OK)
  {
    /* ADC initialization error */
    return Status;
  }

  hadc.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV4;
//  hadc.Init.Resolution            = ADC_RESOLUTION_12B;  ADC_RESOLUTION_8B
  hadc.Init.Resolution            = ADC_RESOLUTION_8B;
  hadc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
//  hadc.Init.ScanConvMode          = ADC_SCAN_ENABLE;               /* Sequencer enabled (ADC conversion on several channels, successively, following settings below) */
  hadc.Init.ScanConvMode          = ADC_SCAN_DISABLE;                /* Sequencer disabled (ADC conversion on only one channel. RP 7/16/16 */
  hadc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait      = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff  = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank          = ADC_CHANNELS_BANK_A;
//  hadc.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 rank converted at each conversion trig, and because discontinuous mode is enabled */
  hadc.Init.ContinuousConvMode    = ENABLE;                       /* Continuous mode enabled to have only 1 channel converted continuously. RP 7/16/16 */
//  hadc.Init.NbrOfConversion       = 3;                             /* Sequencer of regular group will convert the 3 first ranks: rank1, rank2, rank3 */
  hadc.Init.NbrOfConversion       = 1;                             /* Sequencer of regular group will convert the first rank: rank1 */
//  hadc.Init.DiscontinuousConvMode = ENABLE;                        /* Sequencer of regular group will convert the sequence in several sub-divided sequences */
  hadc.Init.DiscontinuousConvMode = DISABLE;                        /* Will be using Continuous mode on only one channel. RP 7/16/16 */
//  hadc.Init.NbrOfDiscConversion   = 1;                             /* Sequencer of regular group will convert ranks one by one, at each conversion trig */
  hadc.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Trig of conversion start done manually by software, without external event */
  hadc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;                        /* ADC-DMA continuous requests to match with DMA configured in circular mode */
  
  Status = HAL_ADC_Init(&hadc);
  if (Status != HAL_OK)
  {
    /* ADC initialization error */
        return Status;
  }
  
  /* Configuration of channel on ADCx regular group on sequencer rank 1 */
  /* Note: Considering IT occurring after each ADC conversion (IT by DMA end  */
  /*       of transfer), select sampling time and ADC clock with sufficient   */
  /*       duration to not create an overhead situation in IRQHandler.        */
  /* Note: Set long sampling time due to internal channels (VrefInt,          */
  /*       temperature sensor) constraints.                                   */
  /*       For example, sampling time of temperature sensor must be higher    */
  /*       than 4us. Refer to device datasheet for min/typ/max values.        */
  sConfig.Channel      = RoadBrd_ADC_PORT[ADC_Channel];
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_96CYCLES;
  
  Status = HAL_ADC_ConfigChannel(&hadc, &sConfig);
  if (Status != HAL_OK)
  {
    /* Channel Configuration Error */
    return Status;
  }
  
  /* Configuration of channel on ADCx regular group on sequencer rank 2 */
  /* Replicate previous rank settings, change only channel and rank */
  //sConfig.Channel      = ADC_CHANNEL_VREFINT;
  //sConfig.Rank         = ADC_REGULAR_RANK_2;

  //if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  //{
  //  /* Channel Configuration Error */
  //  Error_Handler();
  //}
  
  /* Configuration of channel on ADCx regular group on sequencer rank 3 */
  /* Replicate previous rank settings, change only channel and rank */
  //sConfig.Channel      = ADC_CHANNEL_TEMPSENSOR;
  //sConfig.Rank         = ADC_REGULAR_RANK_3;

  //if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  //{
  //  /* Channel Configuration Error */
  //  Error_Handler();
  //}
  return HAL_OK;
}

/**
  * @brief  Enables ADC, starts conversion of regular group.
  *         Interruptions enabled in this function: None.
  * @param  hadc: ADC handle
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_ADC_Start(uint32_t* pData, uint32_t Length)
{
  HAL_StatusTypeDef Status;
  
  // 0. Test Flg to determine if we are currently busy.
  if (adcActive == RESET)
  {
    // 1. Stop DMA if started.
    Status = HAL_ADC_Stop_DMA( &hadc );
    if (Status == HAL_OK)
    {
      // 2. Re-start DMA with passed parameters.
      Status = HAL_ADC_Start_DMA(&hadc, pData, Length );
      if (Status == HAL_OK)
      {
        // 3. Reset tracking flag so that we can continue operation.
        adcActive = SET;                        // We are now busy with ADC event.
        ubSequenceCompleted = RESET;            // Ready to start event.
        // 4. Start ADC operation.
        Status = HAL_ADC_Start(&hadc);
        if (Status == HAL_OK)
        {
          adcActive = SET;
          return HAL_OK;              // ADC Tasked with no errors.
        }
        else
        {
          adcActive = RESET;          // Event never started. Reset.
          return Status;
        }
      } //Endif (Status == HAL_OK)
      else
        return Status;
    } //Endif (Status == HAL_OK)
    else
      return Status;
  } //Endif (adcActive == RESET)
  else
    return HAL_BUSY;            // ADC Busy.
}

/**
  * @brief  Returns status of the ADC Channel that is selected.
  * @param  none.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_BUSY:     UART is busy.
  */
HAL_StatusTypeDef RoadBrd_ADC_Status( void )
{
  if (adcActive == SET)
    return HAL_BUSY;
  else
    return HAL_OK;
}

/**
  * @brief  Sets the new Channel and initializes the Channel
  * @param  RoadBrd_adc_TypeDef NewChannel: New Channel to be used for sampling.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_ADC_SetChannel( RoadBrd_adc_TypeDef NewChannel )
{
  HAL_StatusTypeDef Status;
  
  if (NewChannel != Old_ADC_Channel)
  {
    // Set Channel Variable.
    ADC_Channel = NewChannel;
    // Now, attempt to reinitialize with new settings.
    Status = ADC_Config();
    if (Status != HAL_OK)
    {
      // Reset ADC_Channel to original value.
      ADC_Channel = Old_ADC_Channel;

      /* ADC initialization error */
      return Status;
    }
    // OK. Update Old_ADC_Channel.
    Old_ADC_Channel = ADC_Channel;
    // Reset flags.
    ubSequenceCompleted = RESET;     /* Set when all ranks of the sequence have been converted */
    adcActive = RESET;               /* If this flag is set, we currently have an active ADC event. */
    return HAL_OK;
  }
  else
    return HAL_OK;
}

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : ADC handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
  /* Report to main program that ADC sequencer has reached its end */
  ubSequenceCompleted = SET;
  adcActive = RESET;
}

/**
  * @brief  Conversion DMA half-transfer callback in non blocking mode 
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{

}

/**
  * @brief  ADC error callback in non blocking mode
  *        (ADC conversion with interruption or transfer by DMA)
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  /* In case of ADC error, call main error handler */
  Error_Handler();
}


/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
