/**
  ******************************************************************************
  * File Name          : VMonitor.c
  * Description        : This file provides code for the control of the Power
  *                      monitor hardware based on the ISL28022 chip.
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
#include "VMonitor.h"
#include "Calibration.h"


/* barometer init function */
/**
  * @brief  This function initiaizes the Baromoeter Sensor
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_Init_VMonitor( void )
{
  HAL_StatusTypeDef Status;
  int Loop_cnt;
  int num_bytes;
  uint8_t i2cData[4];  
  Loop_cnt = 0;

  while( Loop_cnt < I2C_MAX_TRIES)
  {
    num_bytes = 0;    // No Data to Pass.
#ifdef REV_L
    Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)VOLTAGE_MNTR, (uint8_t)VOLTAGE_MNTR_CALIB, i2cData, (uint16_t)num_bytes);
    RoadBrd_Delay( 50 );
    // 2. If Status was good, Time to get response.
    if (Status == HAL_OK)
    {
      // Now wait for completion of XMIT.
      Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
      if (Status == HAL_OK)
      {
        Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)VOLTAGE_MNTR, i2cData, (uint16_t)2);
        RoadBrd_Delay( 50 );
        if (Status == HAL_OK)
        {
          // Now wait for completion of Receive.
          Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
          if (Status == HAL_OK)
            break;
        } // EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Receive_IT
      } // EndElse (Status != HAL_OK) RoadBrd_WaitForState
    }// EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Transmit_CMDData_IT
#else
    Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)VOLTAGE_MNTR, (uint8_t)VOLTAGE_MNTR_CALIB, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
    // Wait 100msec.
    RoadBrd_Delay( 50 );
  
    // 2. If Status was good, Time to get response.
    if (Status == HAL_OK)
    {
      Status =  RoadBrd_I2C_Master_Receive((uint16_t)VOLTAGE_MNTR, i2cData, (uint16_t)2, I2C_TIMEOUT);
    }
    if (Status == HAL_OK)
      break;
    // Wait 100msec.
    RoadBrd_Delay( 50 );
#endif
    Loop_cnt++;
  }
  // Test Loop_cnt. If greater or equal, we never were able to get a valid value.
  if(Loop_cnt >= I2C_MAX_TRIES)
    return Status;
                    
  // Now Test Results from read..
  if((i2cData[0]==((uint8_t)(CALIBRATION_VALUE/256))) && 
     (i2cData[1]==((uint8_t)CALIBRATION_VALUE & 0xff)))
  {
    // No Need to write New Value. It is good.)
    return HAL_OK;
  }
  else
  {
    // Wait 50msec.
    RoadBrd_Delay( 50 );
    // Load Config Register with Config Settings
    num_bytes = 2;    // No Data to Pass.
    i2cData[0] = (uint8_t)(CALIBRATION_VALUE/256);
    i2cData[1] = (uint8_t)CALIBRATION_VALUE & 0xff;
    Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)VOLTAGE_MNTR, (uint8_t)VOLTAGE_MNTR_CALIB, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
  }
  return Status;
}

  /**
  * @brief  This function reads the Scaled Shunt Voltage to the passed Voltage Structure.
  * @param  VoltagePtr VPtr: Ptr to structure to receive the final values of the SHunt Voltage.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_VMonitor_RdShntVltg_Scaled( VoltagePtr VPtr )
{
  HAL_StatusTypeDef Status;
  int Loop_cnt;
  int num_bytes;
  uint8_t i2cData[4];  
  Loop_cnt = 0;
  char tempBffr2[5];
  float Shunt_V;

  Loop_cnt = 0;
  while( Loop_cnt < I2C_MAX_TRIES)
  {
    num_bytes = 0;    // No Data to Pass.
#ifdef REV_L
    Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)VOLTAGE_MNTR, (uint8_t)VOLTAGE_MNTR_SHNTV, i2cData, (uint16_t)num_bytes);
    // 2. If Status was good, Time to get response.
    if (Status == HAL_OK)
    {
      // Now wait for completion of XMIT.
      Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
      if (Status == HAL_OK)
      {
        Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)VOLTAGE_MNTR, i2cData, (uint16_t)2);
        if (Status == HAL_OK)
        {
        // Now wait for completion of Receive.
          Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
          if (Status == HAL_OK)
            break;
        } // EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Receive_IT
      } // EndElse (Status != HAL_OK) RoadBrd_WaitForState
    }// EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Transmit_CMDData_IT
#else
    Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)VOLTAGE_MNTR, (uint8_t)VOLTAGE_MNTR_SHNTV, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
    // 2. If Status was good, Time to get response.
    if (Status == HAL_OK)
    {
      Status =  RoadBrd_I2C_Master_Receive((uint16_t)VOLTAGE_MNTR, i2cData, (uint16_t)2, I2C_TIMEOUT);
    }
    if (Status == HAL_OK)
      break;
#endif
    Loop_cnt++;
  }
  // Test Loop_cnt. If greater or equal, we never were able to get a valid value.
  if(Loop_cnt >= I2C_MAX_TRIES)
    return Status;
                      
  if (Status == HAL_OK)
  {
    // NOW, Build Data String..
    sprintf( (char *)VPtr->Raw, "%02x", i2cData[0]);
    sprintf( (char *)tempBffr2, "%02x", i2cData[1]);
    strcat( (char *)VPtr->Raw, (char *)tempBffr2 );
    //strcat( (char *)VPtr->Raw, "Rw" );
  }
  else
    return Status;
  // Now calculate Shunt Voltage.
  Shunt_V = (i2cData[0]*256 + i2cData[1]) * SHNT_VLTG_TICK * MV_SCALE_ADJUST;
  Shunt_V = RoadBrd_CAL_ScaleValue( CAL_SHNT_VLTG, Shunt_V);
  sprintf( (char *)VPtr->Voltage, "%2.3f", Shunt_V );
  return Status;
}

  /**
  * @brief  This function reads the Shunt Voltage to the passed Voltage Structure.
  * @param  VoltagePtr VPtr: Ptr to structure to receive the final values of the SHunt Voltage.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_VMonitor_RdShntVltg( VoltagePtr VPtr )
{
  HAL_StatusTypeDef Status;
  int Loop_cnt;
  int num_bytes;
  uint8_t i2cData[4];  
  Loop_cnt = 0;
  char tempBffr2[5];
  float Shunt_V;

  Loop_cnt = 0;
  while( Loop_cnt < I2C_MAX_TRIES)
  {
    num_bytes = 0;    // No Data to Pass.
#ifdef REV_L
    Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)VOLTAGE_MNTR, (uint8_t)VOLTAGE_MNTR_SHNTV, i2cData, (uint16_t)num_bytes);
    // 2. If Status was good, Time to get response.
    if (Status == HAL_OK)
    {
      // Now wait for completion of XMIT.
      Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
      if (Status == HAL_OK)
      {
        Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)VOLTAGE_MNTR, i2cData, (uint16_t)2);
        if (Status == HAL_OK)
        {
        // Now wait for completion of Receive.
          Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
          if (Status == HAL_OK)
            break;
        } // EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Receive_IT
      } // EndElse (Status != HAL_OK) RoadBrd_WaitForState
    }// EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Transmit_CMDData_IT
#else
    Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)VOLTAGE_MNTR, (uint8_t)VOLTAGE_MNTR_SHNTV, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
    // 2. If Status was good, Time to get response.
    if (Status == HAL_OK)
    {
      Status =  RoadBrd_I2C_Master_Receive((uint16_t)VOLTAGE_MNTR, i2cData, (uint16_t)2, I2C_TIMEOUT);
    }
    if (Status == HAL_OK)
      break;
#endif
    Loop_cnt++;
  }
  // Test Loop_cnt. If greater or equal, we never were able to get a valid value.
  if(Loop_cnt >= I2C_MAX_TRIES)
    return Status;
                      
  if (Status == HAL_OK)
  {
    // NOW, Build Data String..
    sprintf( (char *)VPtr->Raw, "%02x", i2cData[0]);
    sprintf( (char *)tempBffr2, "%02x", i2cData[1]);
    strcat( (char *)VPtr->Raw, (char *)tempBffr2 );
    //strcat( (char *)VPtr->Raw, "Rw" );
  }
  else
    return Status;
  // Now calculate Shunt Voltage.
  Shunt_V = (i2cData[0]*256 + i2cData[1]) * SHNT_VLTG_TICK * MV_SCALE_ADJUST;
  sprintf( (char *)VPtr->Voltage, "%2.3f", Shunt_V );
  return Status;
}

  /**
  * @brief  This function reads the operagtional current to the passed Current Structure.
  * @param  CurrentPtr CPtr: Ptr to structure to receive the final values of the Operational Current.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_VMonitor_RdCurrent_Scaled( CurrentPtr CPtr )
{
  HAL_StatusTypeDef Status;
  int Loop_cnt;
  int num_bytes;
  uint8_t i2cData[4];  
  Loop_cnt = 0;
  char tempBffr2[5];
  float Crrnt;

  Loop_cnt = 0;
  while( Loop_cnt < I2C_MAX_TRIES)
  {
    num_bytes = 0;    // No Data to Pass.
#ifdef REV_L
    Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)VOLTAGE_MNTR, (uint8_t)VOLTAGE_MNTR_CRRNT, i2cData, (uint16_t)num_bytes);
    // 2. If Status was good, Time to get response.
    if (Status == HAL_OK)
    {
      // Now wait for completion of XMIT.
      Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
      if (Status == HAL_OK)
      {
        Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)VOLTAGE_MNTR, i2cData, (uint16_t)2);
        if (Status == HAL_OK)
        {
          // Now wait for completion of Receive.
          Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
          if (Status == HAL_OK)
            break;
        } // EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Receive_IT
      } // EndElse (Status != HAL_OK) RoadBrd_WaitForState
    }// EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Transmit_CMDData_IT
#else
    Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)VOLTAGE_MNTR, (uint8_t)VOLTAGE_MNTR_CRRNT, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
    // 2. If Status was good, Time to get response.
    if (Status == HAL_OK)
    {
      Status =  RoadBrd_I2C_Master_Receive((uint16_t)VOLTAGE_MNTR, i2cData, (uint16_t)2, I2C_TIMEOUT);
    }
    if (Status == HAL_OK)
      break;
#endif
    Loop_cnt++;
  }
  // Test Loop_cnt. If greater or equal, we never were able to get a valid value.
  if(Loop_cnt >= I2C_MAX_TRIES)
    return Status;
                      
  if (Status == HAL_OK)
  {
    // NOW, Build Data String..
    sprintf( (char *)CPtr->Raw, "%02x", i2cData[0]);
    sprintf( (char *)tempBffr2, "%02x", i2cData[1]);
    strcat( (char *)CPtr->Raw, (char *)tempBffr2 );
    //strcat( (char *)CPtr->Raw, "Rw" );
  }
  else
    return Status;
  // Now calculate Current.
  Crrnt = (i2cData[0]*256 + i2cData[1]) * CURRENT_TICK * MA_SCALE_ADJUST;
  Crrnt = RoadBrd_CAL_ScaleValue( CAL_CURRENT, Crrnt);
  sprintf( (char *)CPtr->Current, "%4.1f", Crrnt );
  return Status;
}

  /**
  * @brief  This function reads the operagtional current to the passed Current Structure.
  * @param  CurrentPtr CPtr: Ptr to structure to receive the final values of the Operational Current.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_VMonitor_RdCurrent( CurrentPtr CPtr )
{
  HAL_StatusTypeDef Status;
  int Loop_cnt;
  int num_bytes;
  uint8_t i2cData[4];  
  Loop_cnt = 0;
  char tempBffr2[5];
  float Crrnt;

  Loop_cnt = 0;
  while( Loop_cnt < I2C_MAX_TRIES)
  {
    num_bytes = 0;    // No Data to Pass.
#ifdef REV_L
    Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)VOLTAGE_MNTR, (uint8_t)VOLTAGE_MNTR_CRRNT, i2cData, (uint16_t)num_bytes);
    // 2. If Status was good, Time to get response.
    if (Status == HAL_OK)
    {
      // Now wait for completion of XMIT.
      Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
      if (Status == HAL_OK)
      {
        Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)VOLTAGE_MNTR, i2cData, (uint16_t)2);
        if (Status == HAL_OK)
        {
          // Now wait for completion of Receive.
          Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
          if (Status == HAL_OK)
            break;
        } // EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Receive_IT
      } // EndElse (Status != HAL_OK) RoadBrd_WaitForState
    }// EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Transmit_CMDData_IT
#else
    Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)VOLTAGE_MNTR, (uint8_t)VOLTAGE_MNTR_CRRNT, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
    // 2. If Status was good, Time to get response.
    if (Status == HAL_OK)
    {
      Status =  RoadBrd_I2C_Master_Receive((uint16_t)VOLTAGE_MNTR, i2cData, (uint16_t)2, I2C_TIMEOUT);
    }
    if (Status == HAL_OK)
      break;
#endif
    Loop_cnt++;
  }
  // Test Loop_cnt. If greater or equal, we never were able to get a valid value.
  if(Loop_cnt >= I2C_MAX_TRIES)
    return Status;
                      
  if (Status == HAL_OK)
  {
    // NOW, Build Data String..
    sprintf( (char *)CPtr->Raw, "%02x", i2cData[0]);
    sprintf( (char *)tempBffr2, "%02x", i2cData[1]);
    strcat( (char *)CPtr->Raw, (char *)tempBffr2 );
    //strcat( (char *)CPtr->Raw, "Rw" );
  }
  else
    return Status;
  // Now calculate Current.
  Crrnt = (i2cData[0]*256 + i2cData[1]) * CURRENT_TICK * MA_SCALE_ADJUST;
  sprintf( (char *)CPtr->Current, "%4.1f", Crrnt );
  return Status;
}

  /**
  * @brief  This function reads the operagtional Power to the passed Power Structure.
  * @param  PowerPtr PPtr: Ptr to structure to receive the final values of the Operational Power.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_VMonitor_RdPower_Scaled( PowerPtr PPtr )
{
  HAL_StatusTypeDef Status;
  int Loop_cnt;
  int num_bytes;
  uint8_t i2cData[4];  
  Loop_cnt = 0;
  char tempBffr2[5];
  float Power;

  Loop_cnt = 0;
  while( Loop_cnt < I2C_MAX_TRIES)
  {
    num_bytes = 0;    // No Data to Pass.
#ifdef REV_L
    Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)VOLTAGE_MNTR, (uint8_t)VOLTAGE_MNTR_POWER, i2cData, (uint16_t)num_bytes);
    // 2. If Status was good, Time to get response.
    if (Status == HAL_OK)
    {
      // Now wait for completion of XMIT.
      Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
      if (Status == HAL_OK)
      {
        Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)VOLTAGE_MNTR, i2cData, (uint16_t)2);
        if (Status == HAL_OK)
        {
          // Now wait for completion of Receive.
          Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
          if (Status == HAL_OK)
            break;
        } // EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Receive_IT
      } // EndElse (Status != HAL_OK) RoadBrd_WaitForState
    }// EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Transmit_CMDData_IT
#else
    Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)VOLTAGE_MNTR, (uint8_t)VOLTAGE_MNTR_POWER, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
    // 2. If Status was good, Time to get response.
    if (Status == HAL_OK)
    {
      Status =  RoadBrd_I2C_Master_Receive((uint16_t)VOLTAGE_MNTR, i2cData, (uint16_t)2, I2C_TIMEOUT);
    }
    if (Status == HAL_OK)
      break;
#endif
    Loop_cnt++;
  } // EndWhile
  // Test Loop_cnt. If greater or equal, we never were able to get a valid value.
  if(Loop_cnt >= I2C_MAX_TRIES)
    return Status;
                      
  if (Status == HAL_OK)
  {
    // NOW, Build Data String..
    sprintf( (char *)PPtr->Raw, "%02x", i2cData[0]);
    sprintf( (char *)tempBffr2, "%02x", i2cData[1]);
    strcat( (char *)PPtr->Raw, (char *)tempBffr2 );
    //strcat( (char *)PPtr->Raw, "Rw" );
  }
  else
    return Status;
  // Now calculate Power.
  Power = (i2cData[0]*256 + i2cData[1]) * POWER_TICK * POWER_ADJUST * MW_SCALE_ADJUST;
  Power = RoadBrd_CAL_ScaleValue( CAL_POWER, Power);
  sprintf( (char *)PPtr->Power, "%4.1f", Power );
  return Status;
}

  /**
  * @brief  This function reads the operagtional Power to the passed Power Structure.
  * @param  PowerPtr PPtr: Ptr to structure to receive the final values of the Operational Power.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_VMonitor_RdPower( PowerPtr PPtr )
{
  HAL_StatusTypeDef Status;
  int Loop_cnt;
  int num_bytes;
  uint8_t i2cData[4];  
  Loop_cnt = 0;
  char tempBffr2[5];
  float Power;

  Loop_cnt = 0;
  while( Loop_cnt < I2C_MAX_TRIES)
  {
    num_bytes = 0;    // No Data to Pass.
#ifdef REV_L
    Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)VOLTAGE_MNTR, (uint8_t)VOLTAGE_MNTR_POWER, i2cData, (uint16_t)num_bytes);
    // 2. If Status was good, Time to get response.
    if (Status == HAL_OK)
    {
      // Now wait for completion of XMIT.
      Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
      if (Status == HAL_OK)
      {
        Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)VOLTAGE_MNTR, i2cData, (uint16_t)2);
        if (Status == HAL_OK)
        {
          // Now wait for completion of Receive.
          Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
          if (Status == HAL_OK)
            break;
        } // EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Receive_IT
      } // EndElse (Status != HAL_OK) RoadBrd_WaitForState
    }// EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Transmit_CMDData_IT
#else
    Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)VOLTAGE_MNTR, (uint8_t)VOLTAGE_MNTR_POWER, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
    // 2. If Status was good, Time to get response.
    if (Status == HAL_OK)
    {
      Status =  RoadBrd_I2C_Master_Receive((uint16_t)VOLTAGE_MNTR, i2cData, (uint16_t)2, I2C_TIMEOUT);
    }
    if (Status == HAL_OK)
      break;
#endif
    Loop_cnt++;
  } // EndWhile
  // Test Loop_cnt. If greater or equal, we never were able to get a valid value.
  if(Loop_cnt >= I2C_MAX_TRIES)
    return Status;
                      
  if (Status == HAL_OK)
  {
    // NOW, Build Data String..
    sprintf( (char *)PPtr->Raw, "%02x", i2cData[0]);
    sprintf( (char *)tempBffr2, "%02x", i2cData[1]);
    strcat( (char *)PPtr->Raw, (char *)tempBffr2 );
    //strcat( (char *)PPtr->Raw, "Rw" );
  }
  else
    return Status;
  // Now calculate Power.
  Power = (i2cData[0]*256 + i2cData[1]) * POWER_TICK * POWER_ADJUST * MW_SCALE_ADJUST;
  sprintf( (char *)PPtr->Power, "%4.1f", Power );
  return Status;
}

  /**
  * @brief  This function reads the operagtional Bus Voltage to the passed Voltage Structure.
  * @param  VoltagePtr VPtr: Ptr to structure to receive the final values of the Bus Voltage.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_VMonitor_RdVoltage_Scaled( VoltagePtr VPtr )
{
  HAL_StatusTypeDef Status;
  int Loop_cnt;
  int num_bytes;
  uint8_t i2cData[4];  
  Loop_cnt = 0;
  char tempBffr2[5];
  float Bus_V;
  
  Loop_cnt = 0;
  while( Loop_cnt < I2C_MAX_TRIES)
  {
    num_bytes = 0;    // No Data to Pass.
#ifdef REV_L
    Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)VOLTAGE_MNTR, (uint8_t)VOLTAGE_MNTR_BUSV, i2cData, (uint16_t)num_bytes);
    // 2. If Status was good, Time to get response.
    if (Status == HAL_OK)
    {
      // Now wait for completion of XMIT.
      Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
      if (Status == HAL_OK)
      {
        Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)VOLTAGE_MNTR, i2cData, (uint16_t)2);
        if (Status == HAL_OK)
        {
          // Now wait for completion of Receive.
          Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
          if (Status == HAL_OK)
            break;
        } // EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Receive_IT
      } // EndElse (Status != HAL_OK) RoadBrd_WaitForState
    }// EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Transmit_CMDData_IT
#else
    Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)VOLTAGE_MNTR, (uint8_t)VOLTAGE_MNTR_BUSV, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
    // 2. If Status was good, Time to get response.
    if (Status == HAL_OK)
    {
      Status =  RoadBrd_I2C_Master_Receive((uint16_t)VOLTAGE_MNTR, i2cData, (uint16_t)2, I2C_TIMEOUT);
    }
    if (Status == HAL_OK)
      break;
#endif
    Loop_cnt++;
  }
  // Test Loop_cnt. If greater or equal, we never were able to get a valid value.
  if(Loop_cnt >= I2C_MAX_TRIES)
    return Status;
                      
  if (Status == HAL_OK)
  {
    // NOW, Build Data String..
    sprintf( (char *)VPtr->Raw, "%02x", i2cData[0]);
    sprintf( (char *)tempBffr2, "%02x", i2cData[1]);
    strcat( (char *)VPtr->Raw, (char *)tempBffr2 );
    //strcat( (char *)VPtr->Raw, "Rw" );
    strcat( (char *)tempBffr2, "\r\n" );
  }
  else
    return Status;

  // Now calculate Bus Voltage.
  Bus_V = ((i2cData[0]*256 + i2cData[1])/4) * VOLT_TICK;
  Bus_V = RoadBrd_CAL_ScaleValue( CAL_VOLTAGE, Bus_V);
  sprintf( (char *)VPtr->Voltage, "%4.1f", Bus_V );
  return Status;
}

  /**
  * @brief  This function reads the operagtional Bus Voltage to the passed Voltage Structure.
  * @param  VoltagePtr VPtr: Ptr to structure to receive the final values of the Bus Voltage.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_VMonitor_RdVoltage( VoltagePtr VPtr )
{
  HAL_StatusTypeDef Status;
  int Loop_cnt;
  int num_bytes;
  uint8_t i2cData[4];  
  Loop_cnt = 0;
  char tempBffr2[5];
  float Bus_V;
  
  Loop_cnt = 0;
  while( Loop_cnt < I2C_MAX_TRIES)
  {
    num_bytes = 0;    // No Data to Pass.
#ifdef REV_L
    Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)VOLTAGE_MNTR, (uint8_t)VOLTAGE_MNTR_BUSV, i2cData, (uint16_t)num_bytes);
    // 2. If Status was good, Time to get response.
    if (Status == HAL_OK)
    {
      // Now wait for completion of XMIT.
      Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
      if (Status == HAL_OK)
      {
        Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)VOLTAGE_MNTR, i2cData, (uint16_t)2);
        if (Status == HAL_OK)
        {
          // Now wait for completion of Receive.
          Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
          if (Status == HAL_OK)
            break;
        } // EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Receive_IT
      } // EndElse (Status != HAL_OK) RoadBrd_WaitForState
    }// EndIf (Status == HAL_OK) RoadBrd_I2C_Master_Transmit_CMDData_IT
#else
    Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)VOLTAGE_MNTR, (uint8_t)VOLTAGE_MNTR_BUSV, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
    // 2. If Status was good, Time to get response.
    if (Status == HAL_OK)
    {
      Status =  RoadBrd_I2C_Master_Receive((uint16_t)VOLTAGE_MNTR, i2cData, (uint16_t)2, I2C_TIMEOUT);
    }
    if (Status == HAL_OK)
      break;
#endif
    Loop_cnt++;
  }
  // Test Loop_cnt. If greater or equal, we never were able to get a valid value.
  if(Loop_cnt >= I2C_MAX_TRIES)
    return Status;
                      
  if (Status == HAL_OK)
  {
    // NOW, Build Data String..
    sprintf( (char *)VPtr->Raw, "%02x", i2cData[0]);
    sprintf( (char *)tempBffr2, "%02x", i2cData[1]);
    strcat( (char *)VPtr->Raw, (char *)tempBffr2 );
    //strcat( (char *)VPtr->Raw, "Rw" );
    strcat( (char *)tempBffr2, "\r\n" );
  }
  else
    return Status;

  // Now calculate Bus Voltage.
  Bus_V = ((i2cData[0]*256 + i2cData[1])/4) * VOLT_TICK;
  sprintf( (char *)VPtr->Voltage, "%4.1f", Bus_V );
  return Status;
}

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
