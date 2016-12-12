/**
  ******************************************************************************
  * File Name          : barometer.c
  * Description        : This file provides code for the control of the baromoeter
  *                      hardware based on the LPS22HB chip.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 WeatherCloud
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of WeatherCloud nor the names of its contributors
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
#include "barometer.h"

/* barometer init function */
/**
  * @brief  RoadBrd_Init_Barometer( void )
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_Init_Barometer( void )
{
//  uint8_t tempBuffer[2];
//  HAL_StatusTypeDef Status;
  PRESSURE_StatusTypeDef Status2;

  PRESSURE_InitTypeDef LPS25HB_InitStructure;
  
  /* Set up the pressure/temperature sensor init structure */
  LPS25HB_InitStructure.OutputDataRate = LPS25HB_ODR_7Hz;               /*!< Output Data Rate: P - 7Hz, T - 7Hz */
  LPS25HB_InitStructure.PressureResolution = LPS25HB_P_RES_AVG_32;      // LPS25HB_Pressure_Resolution_Selection 32Bit
  LPS25HB_InitStructure.TemperatureResolution = LPS25HB_T_RES_AVG_32;   // LPS25HB_Temperature_Resolution_Selection 32Bit
  LPS25HB_InitStructure.DiffEnable = LPS25HB_DIFF_DISABLE;              /*!< interrupt circuit enabled */
  LPS25HB_InitStructure.BlockDataUpdate = LPS25HB_BDU_CONT;             /*!< continuous update */
  LPS25HB_InitStructure.SPIMode = LPS25HB_SPI_SIM_4W;                   /*!< 4-wire interface */
  
  /* Initialize the pressure/temperature sensor */
  Status2 = LPS25HB_Init(&LPS25HB_InitStructure);
  switch(Status2)
  {
    case PRESSURE_OK:
      return HAL_OK;
    case PRESSURE_ERROR:
      return HAL_ERROR;
    case PRESSURE_TIMEOUT:
      return HAL_TIMEOUT;
    case PRESSURE_NOT_IMPLEMENTED:
      return HAL_ERROR;
    default:
      return HAL_ERROR;
  }
}
/* Misc barometer functions */
/**
  * @brief  RoadBrd_Enable_Barometer( void )
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_Enable_Barometer( void )
{
  uint8_t tempBuffer[2];
  HAL_StatusTypeDef Status;

  // CTRL_REG1 =	1000.0100 > 0x86
  tempBuffer[0] = 0x80;
  tempBuffer[1] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)BARO_ADDR, (uint8_t)CTRL_REG1, tempBuffer, (uint16_t)1);
  // 2. If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
  }// EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Transmit_CMDData_IT
  return Status;
#else
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)BARO_ADDR, (uint8_t)CTRL_REG1, tempBuffer, (uint16_t)1, I2C_TIMEOUT);
  return Status;
#endif
}

/**
  * @brief  RoadBrd_Disable_Barometer( void )
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_Disable_Barometer( void )
{
  uint8_t tempBuffer[2];
  HAL_StatusTypeDef Status;

  // CTRL_REG1 =	1000.0100 > 0x06
  tempBuffer[0] = 0x00;
  tempBuffer[1] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)BARO_ADDR, (uint8_t)CTRL_REG1, tempBuffer, (uint16_t)1);
  // 2. If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
  }// EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Transmit_CMDData_IT
  return Status;
#else
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)BARO_ADDR, (uint8_t)CTRL_REG1, tempBuffer, (uint16_t)1, I2C_TIMEOUT);
  return Status;
#endif
}

/**
  * @brief  RoadBrd_StartSample_Barometer( void )
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_StartSample_Barometer( void )
{
  uint8_t tempBuffer[2];
  HAL_StatusTypeDef Status;

  // CTRL_REG2 =	0001.0010 > 0x12
  tempBuffer[0] = 0x01;
  tempBuffer[1] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)BARO_ADDR, (uint8_t)CTRL_REG2, tempBuffer, (uint16_t)1);
  // 2. If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
  }// EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Transmit_CMDData_IT
  return Status;
#else
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)BARO_ADDR, (uint8_t)CTRL_REG2, tempBuffer, (uint16_t)1, I2C_TIMEOUT);
  return Status;
#endif
}

/**
  * @brief  RoadBrd_StartSample_BarometerWait( void )
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_StartSample_BarometerWait( void )
{
  uint8_t tempBuffer[2];
  HAL_StatusTypeDef Status;

  // CTRL_REG2 =	0001.0010 > 0x12
  tempBuffer[0] = 0x01;
  tempBuffer[1] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)BARO_ADDR, (uint8_t)CTRL_REG2, tempBuffer, (uint16_t)1);
  // 2. If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
  }// EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Transmit_CMDData_IT
  else
    return Status;
#else
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)BARO_ADDR, (uint8_t)CTRL_REG2, tempBuffer, (uint16_t)1, I2C_TIMEOUT);
  if (Status != HAL_OK)                 // If Error, return Status     
    return Status;
#endif
  //  =Let's wait a little time to allow cmd to complete and clear.
  RoadBrd_Delay(WAIT_PRESSEVNT);

  // OK...NOW Wait for Data to be ready. Will set timeout at five seconds.
  Status = RoadBrd_WaitForPressure( (uint16_t)FIVE_SECOND_DELAY );
  
  return Status;
}

/**
  * @brief  RoadBrd_Barometer_Status( uint8_t *pStatus )
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_Barometer_Status( PRStatPtr SPtr )
{
  uint8_t tempBuffer[2];
  HAL_StatusTypeDef Status;

 // Test STATUS as follows:
  //    Bit 0: T_DA			0: new data for temperature is not yet available
  //    				1: new data for temperature is available
  //    Bit 1: P_DA			0: new data for pressure is not yet available
  //    				1: new data for pressure is available
  //    Bit 4: T_OR			0: no overrun has occurred    
  //    				1: a new data for temperature has overwritten the previous one
  //    Bit 5: P_OR			0: no overrun has occurred
  //    				1: new data for pressure has overwritten the previous one
  // Setup to Read WHOAMI Status from device.
  tempBuffer[0] = 0x00;
  tempBuffer[1] = 0x00;

#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)BARO_ADDR, (uint8_t)STATUS, tempBuffer, (uint16_t)NULL_SIZE);
  // 2. If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
    else
    {
      Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)BARO_ADDR, tempBuffer, (uint16_t)1);
      if (Status == HAL_OK)
      {
        // Now wait for completion of Receive.
        Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
        if (Status != HAL_OK)
          return Status;
      } // EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Receive_IT
      else
        return Status;
    } // EndElse (Status != HAL_OK) RoadBrd_WaitForState
  }// EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Transmit_CMDData_IT
  else
    return Status;
#else
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)BARO_ADDR, (uint8_t)STATUS, tempBuffer, (uint16_t)NULL_SIZE, I2C_TIMEOUT);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    Status =  RoadBrd_I2C_Master_Receive((uint16_t)BARO_ADDR, tempBuffer, (uint16_t)1, I2C_TIMEOUT);
  }
  else
    return Status;
#endif
  // Build Status NOW
  // Build Raw Response
  sprintf( (char *)SPtr->Raw, "%02x", tempBuffer[0]);
  strcat( (char *)SPtr->Raw, "Rw" );
  
  // Pass Status Response.
  SPtr->Status = tempBuffer[0];

  return Status;
}

/**
  * @brief  HAL_I2C_StateTypeDef RoadBrd_WaitForPressure( uint16_t WaitCnt ): Wait for state to change
  * @param  uint16_t WaitCnt: Count of 5msec ticks to wait before timing out.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_WaitForPressure( uint16_t WaitCnt )
{
  HAL_StatusTypeDef Status;
  uint16_t x;
  PRStatus PRMeasure;
  
  // Now wait for transaction to complete.
  for( x=0; x<=WaitCnt; x++)
  {
    // Get new Status from Pressure Transducer.
    Status = RoadBrd_Barometer_Status(&PRMeasure);
    if(Status != HAL_OK)
      return Status;
    
    // Test to see if event finished.
    if (((PRMeasure.Status&TDA_TEST) > 0) &&
        ((PRMeasure.Status&PDA_TEST) > 0))
      break;
    // Wait 5msec.
    RoadBrd_Delay(WAIT_PRESSURE);
  }
  // Test for timeout.
  if( x == WaitCnt)
    return HAL_TIMEOUT;
  // Test for Over run on Sensors.
  if (((PRMeasure.Status&TOR_TEST) > 0) ||
      ((PRMeasure.Status&POR_TEST) > 0))
    return HAL_ERROR;
  else
    return HAL_OK;
}

/**
  * @brief  RoadBrd_Baro_ReadPressure( uint8_t *pData  ): This routine reads the Pressure and returns as a string of 3 bytes 
  * as follows.....PRESS_OUT_XL...PRESS_OUT_L...PRESS_OUT_H
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_Baro_ReadPressure( PRPrsPtr PRPtr )
//HAL_StatusTypeDef RoadBrd_Baro_ReadPressure( uint8_t *pData )
{
  HAL_StatusTypeDef Status;
  int32_t legacyValue;
  float PressureResult, value, fracvalue;
  double temp2;
  int temp3;
  
  Status = HAL_OK;
  
  if (LPS25HB_GetPressure(&value) == PRESSURE_OK)
  {
    PressureResult = value;

    // Now Build Legacy Format.
    legacyValue = (uint32_t)(value * 2.5);
    legacyValue = (legacyValue << 8);
    fracvalue = modf(value, &temp2);
    temp3 = (int)(fracvalue * 256);
    temp3 = temp3 & 0xF0;
    legacyValue = legacyValue + temp3;
  }
  else
    return HAL_ERROR;

  // NOW Build Result
  // Build Raw Result.
  sprintf( (char *)PRPtr->Raw, "%08xRw", legacyValue);
  PRPtr->RawC = legacyValue;
  // Build Clean Result.
  sprintf( (char *)PRPtr->Pressure,"%6.3fPa",PressureResult);

  return Status;
}

/**
  * @brief  RoadBrd_Baro_ReadPressureAscii( uint8_t *pData  )
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */

/**
  * @brief  RoadBrd_Baro_ReadTemp( uint8_t *pData  ) TEMP_OUT_L...TEMP_OUT_H
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_Baro_ReadTemp( TempPtr TmpPtr )
{
  int16_t calcValue;
  float Temp_C, Temp_F;
  char tempBffr2[5];

#if 0
  // 1. Read Read TEMP_OUT_L
  uint8_t tempBuffer[4];
  tempBuffer[0] = 0x00;
  tempBuffer[1] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)BARO_ADDR, (uint8_t)TEMP_OUT_L, tempBuffer, (uint16_t)NULL_SIZE);
  // 2. If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
    else
    {
      Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)BARO_ADDR, &pData[0], (uint16_t)1);
      if (Status == HAL_OK)
      {
        // Now wait for completion of Receive.
        Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
        if (Status != HAL_OK)
          return Status;
      } // EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Receive_IT
      else
        return Status;
    } // EndElse (Status != HAL_OK) RoadBrd_WaitForState
  }// EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Transmit_CMDData_IT
  else
    return Status;
#else
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)BARO_ADDR, (uint8_t)TEMP_OUT_L, tempBuffer, (uint16_t)NULL_SIZE, I2C_TIMEOUT);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    Status =  RoadBrd_I2C_Master_Receive((uint16_t)BARO_ADDR, &pData[0], (uint16_t)1, I2C_TIMEOUT);
    if(Status != HAL_OK)
      return Status;
  }
  else
    return Status;
#endif
  // 2. Read Read TEMP_OUT_H
  tempBuffer[0] = 0x00;
  tempBuffer[1] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)BARO_ADDR, (uint8_t)TEMP_OUT_H, tempBuffer, (uint16_t)NULL_SIZE);
  // 2. If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
    else
    {
      Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)BARO_ADDR, &pData[1], (uint16_t)1);
      if (Status == HAL_OK)
      {
        // Now wait for completion of Receive.
        Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
        if (Status != HAL_OK)
          return Status;
      } // EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Receive_IT
      else
        return Status;
    } // EndElse (Status != HAL_OK) RoadBrd_WaitForState
  }// EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Transmit_CMDData_IT
  return Status;
#else
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)BARO_ADDR, (uint8_t)TEMP_OUT_H, tempBuffer, (uint16_t)NULL_SIZE, I2C_TIMEOUT);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    Status =  RoadBrd_I2C_Master_Receive((uint16_t)BARO_ADDR, &pData[1], (uint16_t)1, I2C_TIMEOUT);
  }
  return Status;
#endif
#endif
  if (LPS25HB_I2C_GetRawTemperature(&calcValue) != PRESSURE_OK)
    return HAL_ERROR;
  
  if (LPS25HB_GetTemperature(&Temp_C) == PRESSURE_OK)
  {
    // TIme to build all needed outputs.
    Temp_F = (Temp_C * 1.8) + 32;
    TmpPtr->RawC = calcValue;
    sprintf( (char *)TmpPtr->TempC, "%3.1fC", Temp_C );
    sprintf( (char *)TmpPtr->TempF, "%3.1fF", Temp_F );
    // NOW, Build Raw Data String..
    sprintf( (char *)TmpPtr->Raw, "%02x", ((calcValue & 0xff00)>>8));
    sprintf( (char *)tempBffr2, "%02x", (calcValue & 0x00ff));
    strcat( (char *)TmpPtr->Raw, (char *)tempBffr2 );
    strcat( (char *)TmpPtr->Raw, "Rw" );

  }
  else
    return HAL_ERROR;

  return HAL_OK;
}

/**
  * @brief  RoadBrd_TestandRead_Barometer( void )
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_TestandRead_Barometer( void )
{
  uint8_t tempBuffer[TXBUFFERSIZE];
  
  HAL_StatusTypeDef Status;
  
  // Setup to Read WHOAMI Status from device.
  tempBuffer[0] = 0x00;
  tempBuffer[1] = 0x00;
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)BARO_ADDR, (uint8_t)WHO_AM_I, tempBuffer, (uint16_t)NULL_SIZE, I2C_TIMEOUT);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    Status =  RoadBrd_I2C_Master_Receive((uint16_t)BARO_ADDR, tempBuffer, (uint16_t)1, I2C_TIMEOUT);
  }
  
  // Now test result and see if it compares.
  if(tempBuffer[0] == (uint8_t)WHOAMI_RSLT)
    return HAL_OK;
  else
    return HAL_ERROR;
}
/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
