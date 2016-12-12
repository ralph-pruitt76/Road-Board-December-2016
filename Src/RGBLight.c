/**
  ******************************************************************************
  * File Name          : RGBLight.c
  * Description        : This file provides code for the control of the RGB Light
  *                      sensor hardware based on the ISL91250 chip.
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
#include "RGBLight.h"

  /**
  * @brief  This function initializes the RGB sensor Hardware. This is the 
  *         default init. Assume Default Parms and write them.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_RGBInit( void )
{
  HAL_StatusTypeDef Status;
  int num_bytes;
  uint8_t i2cData[4];  

  // This is the default init. Assume Default Parms and write them.
  // Load Config Register with Config Settings
  num_bytes = 3;    // No Data to Pass.
  i2cData[0] = (uint8_t)RGB_CNFG1_DEFAULT;
  i2cData[1] = (uint8_t)RGB_CNFG2_DEFAULT;
  i2cData[2] = (uint8_t)RGB_CNFG3_DEFAULT;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)RGB_SNSR, (uint8_t)RGB_SNSR_CNFG1, i2cData, (uint16_t)num_bytes);
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)RGB_SNSR, (uint8_t)RGB_SNSR_CNFG1, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
#endif
  return Status;
}

  /**
  * @brief  This function initializes the RGB sensor Hardware. This is the 
  *         Full  init. Assume Default Parms and write them.
  * @param  RGBInitPtr LPtr: Ptr to structure of initialization parameters.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_RGBFullInit( RGBInitPtr LPtr )
{
  HAL_StatusTypeDef Status;
  int num_bytes;
  uint8_t i2cData[4];  

  // This is the default init. Assume Default Parms and write them.
  // Load Config Register with Config Settings
  num_bytes = 3;    // No Data to Pass.
  i2cData[0] = LPtr->config1;
  i2cData[1] = LPtr->config2;
  i2cData[2] = LPtr->config3;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)RGB_SNSR, (uint8_t)RGB_SNSR_CNFG1, i2cData, (uint16_t)num_bytes);
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)RGB_SNSR, (uint8_t)RGB_SNSR_CNFG1, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
#endif
  return Status;
}

  /**
  * @brief  This function resets the RGB sensor Hardware. 
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_RGBReset( void )
{
  HAL_StatusTypeDef Status;
  int num_bytes;
  uint8_t i2cData[4];  

  // 1. Time to send Command and collect status.
  num_bytes = 1;    // No Data to Pass.
  i2cData[0] = (uint8_t)RGB_RESET_CODE;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)RGB_SNSR, (uint8_t)RGB_SNSR_IDRST, i2cData, (uint16_t)num_bytes);
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)RGB_SNSR, (uint8_t)RGB_SNSR_IDRST, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
#endif

  return Status;
}

/**
  * @brief  This function reads the Part ID.
  * @param  RGBIdentPtr id: Ptr to structure to receive the final values of the Part ID.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_RGBReadID( RGBIdentPtr id )
{
  HAL_StatusTypeDef Status;
  int num_bytes;
  uint8_t i2cData[4];  

  // 1. Time to send Command and collect status.
  num_bytes = 0;    // No Data to Pass.
  i2cData[0] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)RGB_SNSR, (uint8_t)RGB_SNSR_IDRST, i2cData, (uint16_t)num_bytes);
  // 2. If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
    else
    {
      Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)RGB_SNSR, i2cData, (uint16_t)1);
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)RGB_SNSR, (uint8_t)RGB_SNSR_IDRST, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
  // 2. If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    Status =  RoadBrd_I2C_Master_Receive((uint16_t)RGB_SNSR, i2cData, (uint16_t)1, I2C_TIMEOUT);
  }
  else
    return Status;
#endif
  if (Status == HAL_OK)
  {
    // Build Status
    id->id = i2cData[0];
    sprintf( (char *)id->Raw, "%02x", i2cData[0]);
    strcat( (char *)id->Raw, "Rw" );
  }
  return Status;
}

/**
  * @brief  This function reads the Part Status.
  * @param  RGBIdentPtr id: Ptr to structure to receive the final values of the Part Status.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_RGBReadStatus( RGBStatusPtr Stat )
{
  HAL_StatusTypeDef Status;
  int num_bytes;
  uint8_t i2cData[4];  

  // 1. Time to send Command and collect status.
  num_bytes = 0;    // No Data to Pass.
  i2cData[0] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)RGB_SNSR, (uint8_t)RGB_SNSR_STATUS, i2cData, (uint16_t)num_bytes);
  // 2. If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
    else
    {
      Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)RGB_SNSR, i2cData, (uint16_t)1);
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)RGB_SNSR, (uint8_t)RGB_SNSR_IDRST, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
  // 2. If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    Status =  RoadBrd_I2C_Master_Receive((uint16_t)RGB_SNSR, i2cData, (uint16_t)1, I2C_TIMEOUT);
  }
  else
    return Status;
#endif
  if (Status == HAL_OK)
  {
    // Build Status
    Stat->status = i2cData[0];
    sprintf( (char *)Stat->Raw, "%02x", i2cData[0]);
    strcat( (char *)Stat->Raw, "Rw" );
  }
  return Status;
}

/**
  * @brief  This function reads the Light Values to the passed Light Structure.
  * @param  RGBLghtPtr LPtr: Ptr to structure to receive the final values of the Light Values.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_RGBReadValues( RGBLghtPtr LPtr )
{
  HAL_StatusTypeDef Status;
  int num_bytes;
  uint8_t i2cData[6];  
  char tempBffr2[5];
  uint16_t CalcValue;
  int x;

  num_bytes = 0;    // No Data to Pass.
  i2cData[0] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)RGB_SNSR, (uint8_t)RGB_SNSR_GREENL, i2cData, (uint16_t)num_bytes);
  // 2. If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
    else
    {
      Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)RGB_SNSR, i2cData, (uint16_t)6);
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)RGB_SNSR, (uint8_t)RGB_SNSR_GREENL, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
  // 2. If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    Status =  RoadBrd_I2C_Master_Receive((uint16_t)RGB_SNSR, i2cData, (uint16_t)6, I2C_TIMEOUT);
  }
  else
    return Status;
#endif
  if (Status == HAL_OK)
  {
    // NOW, Build Data String..
    sprintf( (char *)LPtr->Raw, "%02x", i2cData[0]);
    for(x=1; x<6; x++)
    {
      sprintf( (char *)tempBffr2, "%02x", i2cData[x]);
      strcat( (char *)LPtr->Raw, (char *)tempBffr2 );
    }
    strcat( (char *)LPtr->Raw, "Rw" );
    strcat( (char *)tempBffr2, "\r\n" );
  }
  
  // Now calculate Each RGB Value.
  CalcValue = i2cData[0] + (i2cData[1] * 256);
  sprintf( (char *)LPtr->Red, "%5dlx", CalcValue);
  CalcValue = i2cData[2] + (i2cData[3] * 256);
  sprintf( (char *)LPtr->Green, "%5dlx", CalcValue);
  CalcValue = i2cData[4] + (i2cData[5] * 256);
  sprintf( (char *)LPtr->Blue, "%5dlx", CalcValue);
  return Status;
}

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
