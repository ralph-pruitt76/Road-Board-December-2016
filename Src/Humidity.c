/**
  ******************************************************************************
  * File Name          : Humidity.c
  * Description        : This file provides code for the control of the Humidity
  *                      sensor hardware based on the HTS221TR chip.
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
#include "Humidity.h"

  /**
  * @brief  This function initializes the Humidity sensor Hardware. 
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_HumidityInit( void )
{
  HAL_StatusTypeDef Status;
  int num_bytes;
  uint8_t i2cData[4];  

  // Load Humidity and temperature average configuration..
  num_bytes = 1;    // No Data to Pass.
  i2cData[0] = (uint8_t)(HMDTY_SNSR_AVGHx2_5 + HMDTY_SNSR_AVGTx2_0);
  i2cData[1] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)HMDTY_SNSR, (uint8_t)HMDTY_SNSR_AV_CONF, i2cData, (uint16_t)num_bytes);
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)HMDTY_SNSR, (uint8_t)HMDTY_SNSR_AV_CONF, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
#endif
  if (Status != HAL_OK)
    return Status;
  
  // Load Control register 1, 2 and3.
  num_bytes = 3;    // No Data to Pass.
  i2cData[0] = (uint8_t)(HMDTY_SNSR_PD_ON + HMDTY_SNSR_BDU_CONT + HMDTY_SNSR_ODR_1Hz);
  i2cData[1] = (uint8_t)(HMDTY_SNSR_BOOT_NRML + HMDTY_SNSR_HEATER_OFF);
  i2cData[2] = (uint8_t)(HMDTY_SNSR_DRDYHL_ACTVH + HMDTY_SNSR_PPOD_PSHPLL + HMDTY_SNSR_DRDY_NO);
  i2cData[3] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)HMDTY_SNSR, (uint8_t)(HMDTY_SNSR_CTRL_REG1 | 0x80), i2cData, (uint16_t)num_bytes);
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)HMDTY_SNSR, (uint8_t)(HMDTY_SNSR_CTRL_REG1 | 0x80), i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
#endif
  return Status;
}

/**
  * @brief  This function reads HTS221 Humidity output registers, and calculates humidity. It
  * then loads these values to the passed Humidity Structure.
  * @param  HumidtyPtr HPtr: Ptr to structure to receive the final values of Humidity.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_Humidity_ReadHumidity( HumidtyPtr HPtr )
{
  HAL_StatusTypeDef Status;
  int num_bytes;
  uint8_t i2cData[4];  
  int16_t H0_T0_out, H1_T0_out, H_T_out;
  int16_t H0_rh, H1_rh, tmp3;
  char tempBffr2[5];
  float tmp, tmp2;
  
  Status = HAL_OK;
  
  /* 1. Read H0_rH and H1_rH coefficients*/
  num_bytes = 0;    // No Data to Pass.
  i2cData[0] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)HMDTY_SNSR, (uint8_t)(HMDTY_SNSR_H0rH_x2 | 0x80), i2cData, (uint16_t)num_bytes);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
    else
    {
      Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)HMDTY_SNSR, i2cData, (uint16_t)2);
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)HMDTY_SNSR, (uint8_t)(HMDTY_SNSR_H0rH_x2 | 0x80), i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    Status =  RoadBrd_I2C_Master_Receive((uint16_t)HMDTY_SNSR, i2cData, (uint16_t)2, I2C_TIMEOUT);
    if (Status != HAL_OK)
      return Status;
  }
  else
    return Status;
#endif
  
  // 1a. Now extract Coefficients.
  H0_rh = i2cData[0]>>1;
  H1_rh = i2cData[1]>>1;
  
  /*2. Read H0_T0_OUT */
  num_bytes = 0;    // No Data to Pass.
  i2cData[0] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)HMDTY_SNSR, (uint8_t)(HMDTY_SNSR_H0_T0_OUT | 0x80), i2cData, (uint16_t)num_bytes);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
    else
    {
      Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)HMDTY_SNSR, i2cData, (uint16_t)2);
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)HMDTY_SNSR, (uint8_t)(HMDTY_SNSR_H0_T0_OUT | 0x80), i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    Status =  RoadBrd_I2C_Master_Receive((uint16_t)HMDTY_SNSR, i2cData, (uint16_t)2, I2C_TIMEOUT);
    if (Status != HAL_OK)
      return Status;
  }
  else
    return Status;
#endif
  H0_T0_out = (((uint16_t)i2cData[1])<<8) | (uint16_t)i2cData[0];
  
  /*3. Read H1_T0_OUT */
  num_bytes = 0;    // No Data to Pass.
  i2cData[0] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)HMDTY_SNSR, (uint8_t)(HMDTY_SNSR_H1_T0_OUT | 0x80), i2cData, (uint16_t)num_bytes);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
    else
    {
      Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)HMDTY_SNSR, i2cData, (uint16_t)2);
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)HMDTY_SNSR, (uint8_t)(HMDTY_SNSR_H1_T0_OUT | 0x80), i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    Status =  RoadBrd_I2C_Master_Receive((uint16_t)HMDTY_SNSR, i2cData, (uint16_t)2, I2C_TIMEOUT);
    if (Status != HAL_OK)
      return Status;
  }
  else
    return Status;
#endif
  H1_T0_out = (((uint16_t)i2cData[1])<<8) | (uint16_t)i2cData[0];
  
  /*4. Read H_T_OUT */
  num_bytes = 0;    // No Data to Pass.
  i2cData[0] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)HMDTY_SNSR, (uint8_t)(HMDTY_SNSR_HUMIDITY_OUT_L | 0x80), i2cData, (uint16_t)num_bytes);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
    else
    {
      Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)HMDTY_SNSR, i2cData, (uint16_t)2);
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)HMDTY_SNSR, (uint8_t)(HMDTY_SNSR_HUMIDITY_OUT_L | 0x80), i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    Status =  RoadBrd_I2C_Master_Receive((uint16_t)HMDTY_SNSR, i2cData, (uint16_t)2, I2C_TIMEOUT);
    if (Status != HAL_OK)
      return Status;
  }
  else
    return Status;
#endif
  H_T_out = (((uint16_t)i2cData[1])<<8) | (uint16_t)i2cData[0];

  /*5. Compute the RH [%] value by linea r interpolation */
  tmp = ((float)(H_T_out - H0_T0_out)) * ((float)(H1_rh - H0_rh)*10);
  tmp = tmp/(float)(H1_T0_out - H0_T0_out) + (float)(H0_rh*10);

  /* Saturation cond ition*/
  if(tmp>1000 ) 
    tmp = 1000;
  
  // Now Build results.
  HPtr->HRawC = (uint16_t)(tmp);
  sprintf( (char *)HPtr->Humidity, "%2.1fPr", (float)(tmp/10) );
  // NOW, Build Raw Data String..
  sprintf( (char *)HPtr->HRaw, "%02x", ((HPtr->HRawC & 0xff00)>>8));
  sprintf( (char *)tempBffr2, "%02x", (HPtr->HRawC & 0x00ff));
  strcat( (char *)HPtr->HRaw, (char *)tempBffr2 );
  strcat( (char *)HPtr->HRaw, "Rw" );
  // Calculate Legacy Value.
  tmp2 = (tmp/10) + HUMIDITY_SCALE2;
  tmp2 = tmp2 * HUMIDITY_SCALE1;
  tmp3 = (uint16_t)tmp2;
  HPtr->HRawC = tmp3;

  return Status;
}

/**
  * @brief  This function reads HTS221 Humidity output registers, and calculates temperature. It
  * then loads these values to the passed temperature Structure.
  * @param  TempPtr TPtr: Ptr to structure to receive the final values of Temperature.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_Humidity_ReadTemperature( TempPtr TPtr )
{
  HAL_StatusTypeDef Status;
  int num_bytes;
  uint8_t i2cData[5];  
  int16_t T0_out, T1_out, T_out, T0_degC_x8_u16, T1_degC_x8_u16;
  int16_t T0_degC, T1_degC;
  char tempBffr2[5];
  uint8_t buffer[4], tmp;
  uint32_t tmp32;
  float Temp_C, Temp_F;
  uint16_t tempC;
  
  Status = HAL_OK;
  
  /*1. Read from 0x32 & 0x33 registers the value of coefficients T0_d egC_x8 and T1_de gC_x8*/
  num_bytes = 0;    // No Data to Pass. | 0x80)
  i2cData[0] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)HMDTY_SNSR, (uint8_t)(HMDTY_SNSR_T0_degC_x8 | 0x80), i2cData, (uint16_t)num_bytes);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
    else
    {
      Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)HMDTY_SNSR, buffer, (uint16_t)2);
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)HMDTY_SNSR, (uint8_t)(HMDTY_SNSR_T0_degC_x8 | 0x80), i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    Status =  RoadBrd_I2C_Master_Receive((uint16_t)HMDTY_SNSR, buffer, (uint16_t)2, I2C_TIMEOUT);
    if (Status != HAL_OK)
      return Status;
  }
  else
    return Status;
#endif

  /*2. Read from 0x35 register the value of the MSB bits of T1_deg C and T0_deg C */
  num_bytes = 0;    // No Data to Pass.
  i2cData[0] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)HMDTY_SNSR, (uint8_t)(HMDTY_SNSR_T1_T0_msb | 0x80), i2cData, (uint16_t)num_bytes);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
    else
    {
      Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)HMDTY_SNSR, &tmp, (uint16_t)1);
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)HMDTY_SNSR, (uint8_t)(HMDTY_SNSR_T1_T0_msb | 0x80), i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    Status =  RoadBrd_I2C_Master_Receive((uint16_t)HMDTY_SNSR, &tmp, (uint16_t)1, I2C_TIMEOUT);
    if (Status != HAL_OK)
      return Status;
  }
  else
    return Status;
#endif
  /*2a. Calculate the T0_deg C and T1_degC values*/
  T0_degC_x8_u16 = (((uint16_t)(tmp & 0x03)) << 8) | ((uint16_t)buffer[0]);
  T1_degC_x8_u16 = (((uint16_t)(tmp & 0x0C)) << 6) | ((uint16_t)buffer[1]);
  T0_degC = T0_degC_x8_u16>>3;
  T1_degC = T1_degC_x8_u16>>3;

  /*3. Read from 0x3C & 0x3D registers the value of T0_OUT*/
  /*4. Read from 0x3E & 0x3F registers the value of T1_OUT*/
  num_bytes = 0;    // No Data to Pass.
  i2cData[0] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)HMDTY_SNSR, (uint8_t)(HMDTY_SNSR_T0_OUT | 0x80), i2cData, (uint16_t)num_bytes);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
    else
    {
      Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)HMDTY_SNSR, buffer, (uint16_t)4);
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)HMDTY_SNSR, (uint8_t)(HMDTY_SNSR_T0_OUT | 0x80), i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    Status =  RoadBrd_I2C_Master_Receive((uint16_t)HMDTY_SNSR, buffer, (uint16_t)4, I2C_TIMEOUT);
    if (Status != HAL_OK)
      return Status;
  }
  else
    return Status;
#endif
  T0_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];
  T1_out = (((uint16_t)buffer[3])<<8) | (uint16_t)buffer[2];

  /* 5.Read from 0x2A & 0x2B registers the value T_OUT (ADC_OUT).*/
  num_bytes = 0;    // No Data to Pass.
  i2cData[0] = 0x00;
#ifdef REV_L
  Status = RoadBrd_I2C_Master_Transmit_CMDData_IT((uint16_t)HMDTY_SNSR, (uint8_t)(HMDTY_SNSR_TEMP_OUT_L | 0x80), i2cData, (uint16_t)num_bytes);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    // Now wait for completion of XMIT.
    Status = RoadBrd_WaitForState( (uint16_t)( I2C_TIMEOUT/5 ));
    if (Status != HAL_OK)
      return Status;
    else
    {
      Status =  RoadBrd_I2C_Master_Receive_IT((uint16_t)HMDTY_SNSR, buffer, (uint16_t)2);
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
  Status = RoadBrd_I2C_Master_Transmit_CMDData((uint16_t)HMDTY_SNSR, (uint8_t)(HMDTY_SNSR_TEMP_OUT_L | 0x80), i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
  // If Status was good, Time to get response.
  if (Status == HAL_OK)
  {
    Status =  RoadBrd_I2C_Master_Receive((uint16_t)HMDTY_SNSR, buffer, (uint16_t)2, I2C_TIMEOUT);
    if (Status != HAL_OK)
      return Status;
  }
  else
    return Status;
#endif
  T_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];

  /* 6. Compute the Temperature value by line r interpolation*/
  tmp32 = ((uint32_t)(T_out - T0_out)) * ((uint32_t)(T1_degC - T0_degC)*10);
  tmp32 = tmp32 /(T1_out - T0_out) + T0_degC*10;
  tempC = (uint16_t)tmp32;

  // Now Build results.
  Temp_C = tmp32;
  Temp_C = Temp_C/10;
  Temp_F = (Temp_C * 1.8) + 32;
  
  TPtr->RawC = tempC;
  sprintf( (char *)TPtr->TempC, "%3.1fC", Temp_C );
  sprintf( (char *)TPtr->TempF, "%3.1fF", Temp_F );
  // NOW, Build Raw Data String..
  sprintf( (char *)TPtr->Raw, "%02x", ((tempC & 0xff00)>>8));
  sprintf( (char *)tempBffr2, "%02x", (tempC & 0x00ff));
  strcat( (char *)TPtr->Raw, (char *)tempBffr2 );
  strcat( (char *)TPtr->Raw, "Rw" );

  return Status;
}


  /**
  * @brief  This function resets the RGB sensor Hardware. 
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
/*HAL_StatusTypeDef RoadBrd_RGBReset( void )
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
}*/

/**
  * @brief  This function reads the Part ID.
  * @param  RGBIdentPtr id: Ptr to structure to receive the final values of the Part ID.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
/*HAL_StatusTypeDef RoadBrd_RGBReadID( RGBIdentPtr id )
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
}*/

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
