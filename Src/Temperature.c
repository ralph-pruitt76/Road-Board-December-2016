/**
  ******************************************************************************
  * File Name          : Temperature.c
  * Description        : This file provides code for the control of the Temperature
  *                      sense hardware based on the PCT2075 chip.
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
#include "Temperature.h"
#include "Calibration.h"

  /**
  * @brief  This function reads the Shunt Voltage to the passed Voltage Structure.
  * @param  VoltagePtr VPtr: Ptr to structure to receive the final values of the SHunt Voltage.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_ReadTemp_Scaled( TempPtr TPtr )
{
  HAL_StatusTypeDef Status;
  int num_bytes;
  uint8_t i2cData[4];  
  char tempBffr2[5];
  float Temp_C, Temp_F;
  uint16_t tempC;
  int nativeInt;
  uint16_t smallInt;


  num_bytes = 0;    // No Data to Pass.
  i2cData[0] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)TEMP_SNSR, (uint8_t)TEMP_SNSR_READ, i2cData, (uint16_t)num_bytes);
  // 2. If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
    else
    {
      Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)TEMP_SNSR, i2cData, (uint16_t)2);
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)TEMP_SNSR, (uint8_t)TEMP_SNSR_READ, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
  // 2. If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    Status =  RoadBrd_I2C_Master_Receive((uint16_t)TEMP_SNSR, i2cData, (uint16_t)2, I2C_TIMEOUT);
  }
  else
    return Status;
#endif
  if (Status == HAL_OK)
  {
    // Now calculate Celcius and Farenheit Temp.
#ifdef TESTTEMP
    i2cData[0] = 0xff;
    i2cData[1] = 0x00;
#endif
    smallInt = ((i2cData[0]*256) + i2cData[1]) >> 5;
    //**Need to first Convert 2s Compliment 11 bit to int.
    // Is this negative?
    if (smallInt & (1 << 10))
      // Yes...Extend Sign Bit
      nativeInt = (int)(smallInt | ~((1 << 10) - 1));
    else
      // No...Nothing to Do.
      nativeInt = smallInt;
    
    Temp_C = nativeInt * 0.125;

    tempC = (uint16_t)(round(Temp_C * 10));
    Temp_F = (Temp_C * 1.8) + 32;
    Temp_C = RoadBrd_CAL_ScaleValue( CAL_TEMPC, Temp_C);
    Temp_F = RoadBrd_CAL_ScaleValue( CAL_TEMPF, Temp_F);

    TPtr->RawC = tempC;
    sprintf( (char *)TPtr->TempC, "%03.1f", Temp_C );
    sprintf( (char *)TPtr->TempF, "%03.1f", Temp_F );
    // NOW, Build Raw Data String..
    sprintf( (char *)TPtr->Raw, "%02x", ((tempC & 0xff00)>>8));
    sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
    strcat( (char *)TPtr->Raw, (char *)tempBffr2 );
    //strcat( (char *)TPtr->Raw, "Rw" );
   }
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
HAL_StatusTypeDef RoadBrd_ReadTemp( TempPtr TPtr )
{
  HAL_StatusTypeDef Status;
  int num_bytes;
  uint8_t i2cData[4];  
  char tempBffr2[5];
  float Temp_C, Temp_F;
  uint16_t tempC;
  int nativeInt;
  uint16_t smallInt;


  num_bytes = 0;    // No Data to Pass.
  i2cData[0] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)TEMP_SNSR, (uint8_t)TEMP_SNSR_READ, i2cData, (uint16_t)num_bytes);
  // 2. If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
    else
    {
      Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)TEMP_SNSR, i2cData, (uint16_t)2);
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)TEMP_SNSR, (uint8_t)TEMP_SNSR_READ, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
  // 2. If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    Status =  RoadBrd_I2C_Master_Receive((uint16_t)TEMP_SNSR, i2cData, (uint16_t)2, I2C_TIMEOUT);
  }
  else
    return Status;
#endif
  if (Status == HAL_OK)
  {
    // Now calculate Celcius and Farenheit Temp.
#ifdef TESTTEMP
    i2cData[0] = 0xff;
    i2cData[1] = 0x00;
#endif
    smallInt = ((i2cData[0]*256) + i2cData[1]) >> 5;
    //**Need to first Convert 2s Compliment 11 bit to int.
    // Is this negative?
    if (smallInt & (1 << 10))
      // Yes...Extend Sign Bit
      nativeInt = (int)(smallInt | ~((1 << 10) - 1));
    else
      // No...Nothing to Do.
      nativeInt = smallInt;
    
    Temp_C = (float)nativeInt * 0.125;
    tempC = (uint16_t)(round(Temp_C * 10));
    Temp_F = (Temp_C * 1.8) + 32;
    TPtr->RawC = tempC;
    sprintf( (char *)TPtr->TempC, "%03.1f", Temp_C );
    sprintf( (char *)TPtr->TempF, "%03.1f", Temp_F );
    // NOW, Build Raw Data String..
    sprintf( (char *)TPtr->Raw, "%02x", ((tempC & 0xff00)>>8));
    sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
    strcat( (char *)TPtr->Raw, (char *)tempBffr2 );
    //strcat( (char *)TPtr->Raw, "Rw" );
   }
  return Status;
}

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
