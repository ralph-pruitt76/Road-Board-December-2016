/**
  ******************************************************************************
  * File Name          : GridEye.c
  * Description        : This file provides code for the control of the Panasonic
  *                      IR Grid Eye Sensor based on the AMG8851 chip.
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
#include "GridEye.h"
#include "Calibration.h"
#include <stdio.h>
  /**
  * @brief  This function initializes the Excelitas Cool Eye sensor Hardware. Parameters are as follows:
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  */
HAL_StatusTypeDef RoadBrd_CoolEyeInit( void )
{
  HAL_StatusTypeDef Status;
  int num_bytes;
  uint8_t i2cData[4];  

  // The following only performs a simple access to the Cool Eye to test if it is present.
  num_bytes = 0;    // No Data to Pass.
  i2cData[0] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)COOLEYE_SNSR, (uint8_t)COOLEYE_SNSR_AMBIENT, i2cData, (uint16_t)num_bytes);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
    else
    {
      Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)COOLEYE_SNSR, i2cData, (uint16_t)2);
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
#else
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)COOLEYE_SNSR, (uint8_t)COOLEYE_SNSR_AMBIENT, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    Status =  RoadBrd_I2C_Master_Receive((uint16_t)COOLEYE_SNSR, i2cData, (uint16_t)2, I2C_TIMEOUT);
  }
#endif

  return Status;
} 

/**
  * @brief  This function reads the Thermal Temp Values to the passed GridEye Structure.
  * @param  GridEPtr GPtr: Ptr to structure to receive the final values of the Thermal Values.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_CoolEye_ReadValues_Scaled( GridEPtr GPtr )
{
  HAL_StatusTypeDef Status;
  int num_bytes;
  uint8_t i2cData[4];  
  char tempBffr2[5];
  int x;
  float Temp_C, Temp_F;
  uint16_t tempC;
  int nativeInt;
  uint16_t smallInt;

  
  //***** 1. Read Ambient Values and calculate and populate structure.
  num_bytes = 0;    // No Data to Pass.
  i2cData[0] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)COOLEYE_SNSR, (uint8_t)COOLEYE_SNSR_AMBIENT, i2cData, (uint16_t)num_bytes);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
    else
    {
      Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)COOLEYE_SNSR, i2cData, (uint16_t)2);
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)COOLEYE_SNSR, (uint8_t)COOLEYE_SNSR_AMBIENT, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    Status =  RoadBrd_I2C_Master_Receive((uint16_t)COOLEYE_SNSR, i2cData, (uint16_t)2, I2C_TIMEOUT);
  }
  else
    return Status;
#endif
  if (Status == HAL_OK)
  {
#ifdef TESTTEMP
    i2cData[0] = 0x80;
    i2cData[1] = 0xff;
#endif
    // NOW, Build Data String..
    smallInt = ((i2cData[1]*256) + i2cData[0]);
    //**Need to first Convert 2s Compliment 11 bit to int.
    // Is this negative?
    if (smallInt & (1 << 15))
      // Yes...Extend Sign Bit
      nativeInt = (int)(smallInt | ~((1 << 15) - 1));
    else
    // No...Nothing to Do.
      nativeInt = smallInt;

    Temp_C = nativeInt/10;
    Temp_C = RoadBrd_CAL_ScaleValue( CAL_THERM_C, Temp_C);
    tempC = (uint16_t)(round(Temp_C * 10));
    Temp_F = (Temp_C * 1.8) + 32;
    GPtr->Thermistor.RawC = tempC;
    sprintf( (char *)GPtr->Thermistor.TempC, "%3.1fC", Temp_C );
    sprintf( (char *)GPtr->Thermistor.TempF, "%3.1fF", Temp_F );
    // NOW, Build Raw Data String..
    sprintf( (char *)GPtr->Thermistor.Raw, "%02x", ((tempC & 0xff00)>>8));
    sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
    strcat( (char *)GPtr->Thermistor.Raw, (char *)tempBffr2 );
    strcat( (char *)GPtr->Thermistor.Raw, "Rw" );
  }
  else
    return Status;
  
  //***** 2. Read all 8 Values of temperature data. 
  for( x=0; x<8; x++)
  {
    num_bytes = 0;    // No Data to Pass.
    i2cData[0] = 0x00;
#ifdef REV_L
    Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)COOLEYE_SNSR, (uint8_t)(COOLEYE_SNSR_T01 + x), i2cData, (uint16_t)num_bytes);
    // If Status was good, Time to get response.
    if (Status == HAL_OK)
    {
      // Now wait for completion of XMIT.
      Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
      if (Status != HAL_OK)
        return Status;
      else
      {
        Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)COOLEYE_SNSR, i2cData, (uint16_t)2);
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
    Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)COOLEYE_SNSR, (uint8_t)(COOLEYE_SNSR_T01 + x), i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
    // If Status was good, Time to get response.
    if (Status == HAL_OK)
    {
      Status =  RoadBrd_I2C_Master_Receive((uint16_t)COOLEYE_SNSR, i2cData, (uint16_t)2, I2C_TIMEOUT);
    }
    else
      return Status;
#endif
    if (Status == HAL_OK)
    {
      // NOW, Build Data String..
 #ifdef TESTTEMP
      i2cData[0] = 0x80;
      i2cData[1] = 0xff;
#endif
      smallInt = ((i2cData[1]*256) + i2cData[0]);
      //**Need to first Convert 2s Compliment 11 bit to int.
      // Is this negative?
      if (smallInt & (1 << 15))
        // Yes...Extend Sign Bit
        nativeInt = (int)(smallInt | ~((1 << 15) - 1));
      else
        // No...Nothing to Do.
        nativeInt = smallInt;
      
      Temp_C = nativeInt/10;
      tempC = (uint16_t)(round(Temp_C * 10));
//      Temp_F = (Temp_C * 1.8) + 32;
      switch(x)
      {
        case 0:
          Temp_C = RoadBrd_CAL_ScaleValue( CAL_ROADT_1C, Temp_C);
          Temp_F = (Temp_C * 1.8) + 32;
          GPtr->GridEye1.RawC = tempC;
          sprintf( (char *)GPtr->GridEye1.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye1.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye1.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye1.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye1.Raw, "Rw" );
          break;
        case 1:
          Temp_C = RoadBrd_CAL_ScaleValue( CAL_ROADT_2C, Temp_C);
          Temp_F = (Temp_C * 1.8) + 32;
          GPtr->GridEye2.RawC = tempC;
          sprintf( (char *)GPtr->GridEye2.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye2.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye2.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye2.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye2.Raw, "Rw" );
          break;
        case 2:
          Temp_C = RoadBrd_CAL_ScaleValue( CAL_ROADT_3C, Temp_C);
          Temp_F = (Temp_C * 1.8) + 32;
          GPtr->GridEye3.RawC = tempC;
          sprintf( (char *)GPtr->GridEye3.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye3.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye3.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye3.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye3.Raw, "Rw" );
          break;
        case 3:
          Temp_C = RoadBrd_CAL_ScaleValue( CAL_ROADT_4C, Temp_C);
          Temp_F = (Temp_C * 1.8) + 32;
          GPtr->GridEye4.RawC = tempC;
          sprintf( (char *)GPtr->GridEye4.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye4.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye4.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye4.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye4.Raw, "Rw" );
          break;
        case 4:
          Temp_C = RoadBrd_CAL_ScaleValue( CAL_ROADT_5C, Temp_C);
          Temp_F = (Temp_C * 1.8) + 32;
          GPtr->GridEye5.RawC = tempC;
          sprintf( (char *)GPtr->GridEye5.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye5.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye5.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye5.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye5.Raw, "Rw" );
          break;
        case 5:
          Temp_C = RoadBrd_CAL_ScaleValue( CAL_ROADT_6C, Temp_C);
          Temp_F = (Temp_C * 1.8) + 32;
          GPtr->GridEye6.RawC = tempC;
          sprintf( (char *)GPtr->GridEye6.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye6.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye6.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye6.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye6.Raw, "Rw" );
          break;
        case 6:
          Temp_C = RoadBrd_CAL_ScaleValue( CAL_ROADT_7C, Temp_C);
          Temp_F = (Temp_C * 1.8) + 32;
          GPtr->GridEye7.RawC = tempC;
          sprintf( (char *)GPtr->GridEye7.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye7.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye7.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye7.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye7.Raw, "Rw" );
          break;
        case 7:
          Temp_C = RoadBrd_CAL_ScaleValue( CAL_ROADT_8C, Temp_C);
          Temp_F = (Temp_C * 1.8) + 32;
          GPtr->GridEye8.RawC = tempC;
          sprintf( (char *)GPtr->GridEye8.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye8.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye8.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye8.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye8.Raw, "Rw" );
          break;
      } // EndSwitch(x)
    } // Endif(Status == HAL_OK)
    else
      return Status;
  } // EndIf(Status == HAL_OK)
  return Status;
}

/**
  * @brief  This function reads the Thermal Temp Values to the passed GridEye Structure.
  * @param  GridEPtr GPtr: Ptr to structure to receive the final values of the Thermal Values.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_CoolEye_ReadValues( GridEPtr GPtr )
{
  HAL_StatusTypeDef Status;
  int num_bytes;
  uint8_t i2cData[4];  
  char tempBffr2[5];
  int x;
  float Temp_C, Temp_F;
  uint16_t tempC;
  int nativeInt;
  uint16_t smallInt;

  
  //***** 1. Read Ambient Values and calculate and populate structure.
  num_bytes = 0;    // No Data to Pass.
  i2cData[0] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)COOLEYE_SNSR, (uint8_t)COOLEYE_SNSR_AMBIENT, i2cData, (uint16_t)num_bytes);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
    else
    {
      Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)COOLEYE_SNSR, i2cData, (uint16_t)2);
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)COOLEYE_SNSR, (uint8_t)COOLEYE_SNSR_AMBIENT, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    Status =  RoadBrd_I2C_Master_Receive((uint16_t)COOLEYE_SNSR, i2cData, (uint16_t)2, I2C_TIMEOUT);
  }
  else
    return Status;
#endif
  if (Status == HAL_OK)
  {
#ifdef TESTTEMP
    i2cData[0] = 0x80;
    i2cData[1] = 0xff;
#endif
    // NOW, Build Data String..
    smallInt = ((i2cData[1]*256) + i2cData[0]);
    //**Need to first Convert 2s Compliment 11 bit to int.
    // Is this negative?
    if (smallInt & (1 << 15))
      // Yes...Extend Sign Bit
      nativeInt = (int)(smallInt | ~((1 << 15) - 1));
    else
    // No...Nothing to Do.
      nativeInt = smallInt;

    Temp_C = nativeInt/10;
    tempC = (uint16_t)(round(Temp_C * 10));
    Temp_F = (Temp_C * 1.8) + 32;
    GPtr->Thermistor.RawC = tempC;
    sprintf( (char *)GPtr->Thermistor.TempC, "%3.1fC", Temp_C );
    sprintf( (char *)GPtr->Thermistor.TempF, "%3.1fF", Temp_F );
    // NOW, Build Raw Data String..
    sprintf( (char *)GPtr->Thermistor.Raw, "%02x", ((tempC & 0xff00)>>8));
    sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
    strcat( (char *)GPtr->Thermistor.Raw, (char *)tempBffr2 );
    strcat( (char *)GPtr->Thermistor.Raw, "Rw" );
  }
  else
    return Status;
  
  //***** 2. Read all 8 Values of temperature data. 
  for( x=0; x<8; x++)
  {
    num_bytes = 0;    // No Data to Pass.
    i2cData[0] = 0x00;
#ifdef REV_L
    Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)COOLEYE_SNSR, (uint8_t)(COOLEYE_SNSR_T01 + x), i2cData, (uint16_t)num_bytes);
    // If Status was good, Time to get response.
    if (Status == HAL_OK)
    {
      // Now wait for completion of XMIT.
      Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
      if (Status != HAL_OK)
        return Status;
      else
      {
        Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)COOLEYE_SNSR, i2cData, (uint16_t)2);
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
    Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)COOLEYE_SNSR, (uint8_t)(COOLEYE_SNSR_T01 + x), i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
    // If Status was good, Time to get response.
    if (Status == HAL_OK)
    {
      Status =  RoadBrd_I2C_Master_Receive((uint16_t)COOLEYE_SNSR, i2cData, (uint16_t)2, I2C_TIMEOUT);
    }
    else
      return Status;
#endif
    if (Status == HAL_OK)
    {
      // NOW, Build Data String..
 #ifdef TESTTEMP
      i2cData[0] = 0x80;
      i2cData[1] = 0xff;
#endif
      smallInt = ((i2cData[1]*256) + i2cData[0]);
      //**Need to first Convert 2s Compliment 11 bit to int.
      // Is this negative?
      if (smallInt & (1 << 15))
        // Yes...Extend Sign Bit
        nativeInt = (int)(smallInt | ~((1 << 15) - 1));
      else
      // No...Nothing to Do.
        nativeInt = smallInt;

      Temp_C = nativeInt/10;
      tempC = (uint16_t)(round(Temp_C * 10));
      Temp_F = (Temp_C * 1.8) + 32;
      switch(x)
      {
        case 0:
          GPtr->GridEye1.RawC = tempC;
          sprintf( (char *)GPtr->GridEye1.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye1.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye1.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye1.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye1.Raw, "Rw" );
          break;
        case 1:
          GPtr->GridEye2.RawC = tempC;
          sprintf( (char *)GPtr->GridEye2.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye2.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye2.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye2.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye2.Raw, "Rw" );
          break;
        case 2:
          GPtr->GridEye3.RawC = tempC;
          sprintf( (char *)GPtr->GridEye3.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye3.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye3.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye3.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye3.Raw, "Rw" );
          break;
        case 3:
          GPtr->GridEye4.RawC = tempC;
          sprintf( (char *)GPtr->GridEye4.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye4.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye4.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye4.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye4.Raw, "Rw" );
          break;
        case 4:
          GPtr->GridEye5.RawC = tempC;
          sprintf( (char *)GPtr->GridEye5.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye5.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye5.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye5.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye5.Raw, "Rw" );
          break;
        case 5:
          GPtr->GridEye6.RawC = tempC;
          sprintf( (char *)GPtr->GridEye6.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye6.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye6.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye6.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye6.Raw, "Rw" );
          break;
        case 6:
          GPtr->GridEye7.RawC = tempC;
          sprintf( (char *)GPtr->GridEye7.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye7.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye7.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye7.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye7.Raw, "Rw" );
          break;
        case 7:
          GPtr->GridEye8.RawC = tempC;
          sprintf( (char *)GPtr->GridEye8.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye8.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye8.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye8.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye8.Raw, "Rw" );
          break;
      } // EndSwitch(x)
    } // Endif(Status == HAL_OK)
    else
      return Status;
  } // EndIf(Status == HAL_OK)
  return Status;
}

  /**
  * @brief  This function initializes the Panasonic Grid Eye sensor Hardware. Parameters are as follows:
  *                     GRIDEYE_SNSR_PCTL = GRIDEYE_PCTL_STNDBY_60      Stand-by Mode (60sec intermittence)
  *                     GRIDEYE_SNSR_FPSC = GRIDEYE_FPSC_1FPS           1 Frame per second
  *                     GRIDEYE_SNSR_AVE = GRIDEYE_AVE_MAMOD            Twice moving average Output Mode
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_GridEyeInit( void )
{
  HAL_StatusTypeDef Status;
  int num_bytes;
  uint8_t i2cData[4];  
  Status = HAL_OK;
  //************* 1. GRIDEYE_SNSR_PCTL = GRIDEYE_PCTL_STNDBY_60...Stand-by Mode (60sec intermittence)
  num_bytes = 1;    // No Data to Pass.
//  i2cData[0] = (uint8_t)GRIDEYE_PCTL_STNDBY_60;
  i2cData[0] = (uint8_t)GRIDEYE_PCTL_NORMAL;
  i2cData[1] = (uint8_t)0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)GRIDEYE_SNSR, (uint8_t)GRIDEYE_SNSR_PCTL, i2cData, (uint16_t)num_bytes);
  // If Status was good, Time to get response.
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)GRIDEYE_SNSR, (uint8_t)GRIDEYE_SNSR_PCTL, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
#endif
  
  //************* 2. GRIDEYE_SNSR_FPSC = GRIDEYE_FPSC_1FPS...1 Frame per second
  num_bytes = 1;    // No Data to Pass.
  i2cData[0] = (uint8_t)GRIDEYE_FPSC_1FPS;
  i2cData[1] = (uint8_t)0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)GRIDEYE_SNSR, (uint8_t)GRIDEYE_SNSR_FPSC, i2cData, (uint16_t)num_bytes);
  // If Status was good, Time to get response.
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)GRIDEYE_SNSR, (uint8_t)GRIDEYE_SNSR_FPSC, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
#endif
  
  //************* 3. GRIDEYE_SNSR_AVE = GRIDEYE_AVE_MAMOD...Twice moving average Output Mode
  num_bytes = 1;    // No Data to Pass.
  i2cData[0] = (uint8_t)GRIDEYE_AVE_MAMOD;
  i2cData[1] = (uint8_t)0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)GRIDEYE_SNSR, (uint8_t)GRIDEYE_SNSR_AVE, i2cData, (uint16_t)num_bytes);
  // If Status was good, Time to get response.
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)GRIDEYE_SNSR, (uint8_t)GRIDEYE_SNSR_AVE, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
#endif
  
  return Status;
} 

  /**
  * @brief  This function resets the GridEye sensor Hardware. 
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_GridEyeReset( void )
{
  HAL_StatusTypeDef Status;
  int num_bytes;
  uint8_t i2cData[4];  

  //******* 1. GRIDEYE_SNSR_RST = GRIDEYE_RST_INIT...Performs flag reset and sets initial values.
  num_bytes = 1;    // No Data to Pass.
  i2cData[0] = (uint8_t)GRIDEYE_RST_INIT;
  i2cData[1] = (uint8_t)0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)GRIDEYE_SNSR, (uint8_t)GRIDEYE_SNSR_RST, i2cData, (uint16_t)num_bytes);
  // If Status was good, Time to get response.
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)GRIDEYE_SNSR, (uint8_t)GRIDEYE_SNSR_RST, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
#endif

  //******* 2. Wait 100msec for Reset.
  RoadBrd_Delay( 1000 );

  //******* 3. GRIDEYE_SNSR_RST = GRIDEYE_RST_NONE...Back to Normal Mode
  num_bytes = 1;    // No Data to Pass.
  i2cData[0] = (uint8_t)GRIDEYE_RST_NONE;
  i2cData[1] = (uint8_t)0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)GRIDEYE_SNSR, (uint8_t)GRIDEYE_SNSR_RST, i2cData, (uint16_t)num_bytes);
  // If Status was good, Time to get response.
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)GRIDEYE_SNSR, (uint8_t)GRIDEYE_SNSR_RST, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
#endif

  return Status;
}

/**
  * @brief  This function reads the Thermal Temp Values to the passed GridEye Structure.
  * @param  GridEPtr GPtr: Ptr to structure to receive the final values of the Thermal Values.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_GridEye_ReadValues_Scaled( GridEPtr GPtr )
{
  HAL_StatusTypeDef Status;
  int num_bytes;
  uint8_t i2cData[20];  
  char tempBffr2[5];
  
  int x;
  float Temp_C, Temp_F;
  uint16_t tempC;
  int nativeInt;
  uint16_t smallInt;

  
  //***** 1. Read Thermistor Values and calculate and populate structure.
  num_bytes = 0;    // No Data to Pass.
  i2cData[0] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)GRIDEYE_SNSR, (uint8_t)GRIDEYE_SNSR_TTHL, i2cData, (uint16_t)num_bytes);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
    else
    {
      Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)GRIDEYE_SNSR, i2cData, (uint16_t)2);
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)GRIDEYE_SNSR, (uint8_t)GRIDEYE_SNSR_TTHL, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    Status =  RoadBrd_I2C_Master_Receive((uint16_t)GRIDEYE_SNSR, i2cData, (uint16_t)2, I2C_TIMEOUT);
  }
  else
    return Status;
#endif
  if (Status == HAL_OK)
  {
    // NOW, Build Data String..
#ifdef TESTTEMP
    i2cData[0] = 0x80;
    i2cData[1] = 0xff;
#endif
    smallInt = ((i2cData[1]*256) + i2cData[0]);
    //**Need to first Convert 2s Compliment 11 bit to int.
    // Is this negative?
    if (smallInt & (1 << 11))
      // Yes...Extend Sign Bit
      nativeInt = (int)(smallInt | ~((1 << 11) - 1));
    else
      // No...Nothing to Do.
      nativeInt = smallInt;
    Temp_C = nativeInt * 0.0625;
    if (Temp_C > 0x400)
      Temp_C = -Temp_C;

    Temp_C = RoadBrd_CAL_ScaleValue( CAL_THERM_C, Temp_C);

    tempC = (uint16_t)(round(Temp_C * 10));
    Temp_F = (Temp_C * 1.8) + 32;
    GPtr->Thermistor.RawC = tempC;
    
    sprintf( (char *)GPtr->Thermistor.TempC, "%3.1fC", Temp_C );
    sprintf( (char *)GPtr->Thermistor.TempF, "%3.1fF", Temp_F );
    // NOW, Build Raw Data String..
    sprintf( (char *)GPtr->Thermistor.Raw, "%02x", ((tempC & 0xff00)>>8));
    sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
    strcat( (char *)GPtr->Thermistor.Raw, (char *)tempBffr2 );
    strcat( (char *)GPtr->Thermistor.Raw, "Rw" );
  }
  
  //***** 2. Read all 16 Values of temperature data for Cells 33-40. 
  num_bytes = 0;    // No Data to Pass.
  i2cData[0] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)GRIDEYE_SNSR, (uint8_t)GRIDEYE_SNSR_T33L, i2cData, (uint16_t)num_bytes);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
    else
    {
      Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)GRIDEYE_SNSR, i2cData, (uint16_t)16);
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)GRIDEYE_SNSR, (uint8_t)GRIDEYE_SNSR_T33L, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    Status =  RoadBrd_I2C_Master_Receive((uint16_t)GRIDEYE_SNSR, i2cData, (uint16_t)16, I2C_TIMEOUT);
  }
  else
    return Status;
#endif
  if (Status == HAL_OK)
  {
#ifdef TESTTEMP
    i2cData[0] = 0x80;
    i2cData[1] = 0xff;
    i2cData[2] = 0x80;
    i2cData[3] = 0xff;
    i2cData[4] = 0x80;
    i2cData[5] = 0xff;
    i2cData[6] = 0x80;
    i2cData[7] = 0xff;
    i2cData[8] = 0x80;
    i2cData[9] = 0xff;
    i2cData[10] = 0x80;
    i2cData[11] = 0xff;
    i2cData[12] = 0x80;
    i2cData[13] = 0xff;
    i2cData[14] = 0x80;
    i2cData[15] = 0xff;
#endif
    // NOW, Build Data String..
    for( x=0; x<8; x++)
    {
      smallInt = (i2cData[(x*2)+1]*256) + i2cData[(x*2)];
      //**Need to first Convert 2s Compliment 11 bit to int.
      // Is this negative?
      if (smallInt & (1 << 11))
        // Yes...Extend Sign Bit
        nativeInt = (int)(smallInt | ~((1 << 11) - 1));
      else
        // No...Nothing to Do.
      nativeInt = smallInt;
      Temp_C = nativeInt * 0.25;
      //**Need to first Convert 2s Compliment 11 bit to int.
      // Is this negative?
      if (smallInt & (1 << 10))
        // Yes...Extend Sign Bit
        nativeInt = (int)(smallInt | ~((1 << 10) - 1));
      else
        // No...Nothing to Do.
        nativeInt = smallInt;
      tempC = (uint16_t)(round(Temp_C * 10));
//      Temp_F = (Temp_C * 1.8) + 32;
      switch(x)
      {
        case 0:
          Temp_C = RoadBrd_CAL_ScaleValue( CAL_ROADT_1C, Temp_C);
          Temp_F = (Temp_C * 1.8) + 32;
          GPtr->GridEye1.RawC = tempC;
          sprintf( (char *)GPtr->GridEye1.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye1.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye1.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye1.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye1.Raw, "Rw" );
          break;
        case 1:
          Temp_C = RoadBrd_CAL_ScaleValue( CAL_ROADT_2C, Temp_C);
          Temp_F = (Temp_C * 1.8) + 32;
          GPtr->GridEye2.RawC = tempC;
          sprintf( (char *)GPtr->GridEye2.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye2.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye2.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye2.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye2.Raw, "Rw" );
          break;
        case 2:
          Temp_C = RoadBrd_CAL_ScaleValue( CAL_ROADT_3C, Temp_C);
          Temp_F = (Temp_C * 1.8) + 32;
          GPtr->GridEye3.RawC = tempC;
          sprintf( (char *)GPtr->GridEye3.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye3.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye3.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye3.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye3.Raw, "Rw" );
          break;
        case 3:
          Temp_C = RoadBrd_CAL_ScaleValue( CAL_ROADT_4C, Temp_C);
          Temp_F = (Temp_C * 1.8) + 32;
          GPtr->GridEye4.RawC = tempC;
          sprintf( (char *)GPtr->GridEye4.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye4.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye4.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye4.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye4.Raw, "Rw" );
          break;
        case 4:
          Temp_C = RoadBrd_CAL_ScaleValue( CAL_ROADT_5C, Temp_C);
          Temp_F = (Temp_C * 1.8) + 32;
          GPtr->GridEye5.RawC = tempC;
          sprintf( (char *)GPtr->GridEye5.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye5.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye5.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye5.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye5.Raw, "Rw" );
          break;
        case 5:
          Temp_C = RoadBrd_CAL_ScaleValue( CAL_ROADT_6C, Temp_C);
          Temp_F = (Temp_C * 1.8) + 32;
          GPtr->GridEye6.RawC = tempC;
          sprintf( (char *)GPtr->GridEye6.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye6.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye6.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye6.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye6.Raw, "Rw" );
          break;
        case 6:
          Temp_C = RoadBrd_CAL_ScaleValue( CAL_ROADT_7C, Temp_C);
          Temp_F = (Temp_C * 1.8) + 32;
          GPtr->GridEye7.RawC = tempC;
          sprintf( (char *)GPtr->GridEye7.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye7.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye7.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye7.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye7.Raw, "Rw" );
          break;
        case 7:
          Temp_C = RoadBrd_CAL_ScaleValue( CAL_ROADT_8C, Temp_C);
          Temp_F = (Temp_C * 1.8) + 32;
          GPtr->GridEye8.RawC = tempC;
          sprintf( (char *)GPtr->GridEye8.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye8.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye8.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye8.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye8.Raw, "Rw" );
          break;
      } // EndSwitch(x)
    } // EdnFor( x=0; x<8; x++)
  } // EndIf(Status == HAL_OK)
  return Status;
}

/**
  * @brief  This function reads the Thermal Temp Values to the passed GridEye Structure.
  * @param  GridEPtr GPtr: Ptr to structure to receive the final values of the Thermal Values.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_GridEye_ReadValues( GridEPtr GPtr )
{
  HAL_StatusTypeDef Status;
  int num_bytes;
  uint8_t i2cData[20];  
  char tempBffr2[5];
  
  int x;
  float Temp_C, Temp_F;
  uint16_t tempC;
  int nativeInt;
  uint16_t smallInt;

  
  //***** 1. Read Thermistor Values and calculate and populate structure.
  num_bytes = 0;    // No Data to Pass.
  i2cData[0] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)GRIDEYE_SNSR, (uint8_t)GRIDEYE_SNSR_TTHL, i2cData, (uint16_t)num_bytes);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
    else
    {
      Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)GRIDEYE_SNSR, i2cData, (uint16_t)2);
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)GRIDEYE_SNSR, (uint8_t)GRIDEYE_SNSR_TTHL, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    Status =  RoadBrd_I2C_Master_Receive((uint16_t)GRIDEYE_SNSR, i2cData, (uint16_t)2, I2C_TIMEOUT);
  }
  else
    return Status;
#endif
  if (Status == HAL_OK)
  {
    // NOW, Build Data String..
#ifdef TESTTEMP
    i2cData[0] = 0x80;
    i2cData[1] = 0xff;
#endif
    smallInt = ((i2cData[1]*256) + i2cData[0]);
    //**Need to first Convert 2s Compliment 11 bit to int.
    // Is this negative?
    if (smallInt & (1 << 11))
      // Yes...Extend Sign Bit
      nativeInt = (int)(smallInt | ~((1 << 11) - 1));
    else
      // No...Nothing to Do.
      nativeInt = smallInt;
    Temp_C = nativeInt * 0.0625;
    if (Temp_C > 0x400)
      Temp_C = -Temp_C;
    tempC = (uint16_t)(round(Temp_C * 10));
    Temp_F = (Temp_C * 1.8) + 32;
    GPtr->Thermistor.RawC = tempC;
    
    sprintf( (char *)GPtr->Thermistor.TempC, "%3.1fC", Temp_C );
    sprintf( (char *)GPtr->Thermistor.TempF, "%3.1fF", Temp_F );
    // NOW, Build Raw Data String..
    sprintf( (char *)GPtr->Thermistor.Raw, "%02x", ((tempC & 0xff00)>>8));
    sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
    strcat( (char *)GPtr->Thermistor.Raw, (char *)tempBffr2 );
    strcat( (char *)GPtr->Thermistor.Raw, "Rw" );
  }
  
  //***** 2. Read all 16 Values of temperature data for Cells 33-40. 
  num_bytes = 0;    // No Data to Pass.
  i2cData[0] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)GRIDEYE_SNSR, (uint8_t)GRIDEYE_SNSR_T33L, i2cData, (uint16_t)num_bytes);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
    else
    {
      Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)GRIDEYE_SNSR, i2cData, (uint16_t)16);
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)GRIDEYE_SNSR, (uint8_t)GRIDEYE_SNSR_T33L, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    Status =  RoadBrd_I2C_Master_Receive((uint16_t)GRIDEYE_SNSR, i2cData, (uint16_t)16, I2C_TIMEOUT);
  }
  else
    return Status;
#endif
  if (Status == HAL_OK)
  {
#ifdef TESTTEMP
    i2cData[0] = 0x80;
    i2cData[1] = 0xff;
    i2cData[2] = 0x80;
    i2cData[3] = 0xff;
    i2cData[4] = 0x80;
    i2cData[5] = 0xff;
    i2cData[6] = 0x80;
    i2cData[7] = 0xff;
    i2cData[8] = 0x80;
    i2cData[9] = 0xff;
    i2cData[10] = 0x80;
    i2cData[11] = 0xff;
    i2cData[12] = 0x80;
    i2cData[13] = 0xff;
    i2cData[14] = 0x80;
    i2cData[15] = 0xff;
#endif
    // NOW, Build Data String..
    for( x=0; x<8; x++)
    {
      smallInt = (i2cData[(x*2)+1]*256) + i2cData[(x*2)];
      //**Need to first Convert 2s Compliment 11 bit to int.
      // Is this negative?
      if (smallInt & (1 << 11))
        // Yes...Extend Sign Bit
        nativeInt = (int)(smallInt | ~((1 << 11) - 1));
      else
        // No...Nothing to Do.
      nativeInt = smallInt;
      Temp_C = nativeInt * 0.25;
      //**Need to first Convert 2s Compliment 11 bit to int.
      // Is this negative?
      if (smallInt & (1 << 10))
        // Yes...Extend Sign Bit
        nativeInt = (int)(smallInt | ~((1 << 10) - 1));
      else
        // No...Nothing to Do.
        nativeInt = smallInt;
      tempC = (uint16_t)(round(Temp_C * 10));
      Temp_F = (Temp_C * 1.8) + 32;
      switch(x)
      {
        case 0:
          GPtr->GridEye1.RawC = tempC;
          sprintf( (char *)GPtr->GridEye1.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye1.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye1.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye1.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye1.Raw, "Rw" );
          break;
        case 1:
          GPtr->GridEye2.RawC = tempC;
          sprintf( (char *)GPtr->GridEye2.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye2.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye2.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye2.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye2.Raw, "Rw" );
          break;
        case 2:
          GPtr->GridEye3.RawC = tempC;
          sprintf( (char *)GPtr->GridEye3.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye3.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye3.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye3.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye3.Raw, "Rw" );
          break;
        case 3:
          GPtr->GridEye4.RawC = tempC;
          sprintf( (char *)GPtr->GridEye4.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye4.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye4.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye4.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye4.Raw, "Rw" );
          break;
        case 4:
          GPtr->GridEye5.RawC = tempC;
          sprintf( (char *)GPtr->GridEye5.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye5.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye5.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye5.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye5.Raw, "Rw" );
          break;
        case 5:
          GPtr->GridEye6.RawC = tempC;
          sprintf( (char *)GPtr->GridEye6.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye6.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye6.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye6.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye6.Raw, "Rw" );
          break;
        case 6:
          GPtr->GridEye7.RawC = tempC;
          sprintf( (char *)GPtr->GridEye7.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye7.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye7.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye7.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye7.Raw, "Rw" );
          break;
        case 7:
          GPtr->GridEye8.RawC = tempC;
          sprintf( (char *)GPtr->GridEye8.TempC, "%3.1fC", Temp_C );
          sprintf( (char *)GPtr->GridEye8.TempF, "%3.1fF", Temp_F );
          // NOW, Build Raw Data String..
          sprintf( (char *)GPtr->GridEye8.Raw, "%02x", ((tempC & 0xff00)>>8));
          sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
          strcat( (char *)GPtr->GridEye8.Raw, (char *)tempBffr2 );
          strcat( (char *)GPtr->GridEye8.Raw, "Rw" );
          break;
      } // EndSwitch(x)
    } // EdnFor( x=0; x<8; x++)
  } // EndIf(Status == HAL_OK)
  return Status;
}

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
