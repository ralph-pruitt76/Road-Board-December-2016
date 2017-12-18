/**
  ******************************************************************************
  * File Name          : s_record.c
  * Description        : This file provides code for the parsing of S-Record data.
  * error buffer.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 WeatherCloud
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
#include "s_record.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"
#include <stdio.h>
#include <string.h>


/**
  * @brief  This function Parses the passed s-record file.
  * @param  char *tempBffr      Pointer to passed Chars to be translated
  * @param  SrecElmentPtr SrecPtr    Pointer to Structure to receive results of Translation
  * @retval HAL_StatusTypeDef:     HAL_OK:       No Errors
  *                                HAL_ERROR:    Error Found during initialization.
  */
HAL_StatusTypeDef Parse_srecord( char *tempBffr, SrecElmentPtr SrecPtr )
{
  uint8_t ByteBffr[BYTE_BFFR_SIZE];     // Bffr to Convert Bytes.
  uint8_t Calc_Checksum;                // Used to Verify Checksum;
  int x;
  
  // Set Passed SRec Error Code to null....
  SrecPtr->Srec_Err = NO_ERROR;
  SrecPtr->Address = 0;
  
  // Clear Out Passed Buffer before populating.
  for (x=0; x<SREC_SIZE; x++)
    SrecPtr->Data[x] = 0x00;
  // Validate if this is an S-Record.
  if (tempBffr[0] != 'S')
  {
    SrecPtr->Srec_Err = ILLEGAL_SREC;
    return HAL_ERROR;
  }
  
  // Get Byte Count.
  if (Get_Bytes( 1, &tempBffr[2], ByteBffr ) != HAL_OK)
  {
    // Illegal Byte Count Value.
    SrecPtr->Srec_Err = ILLEGAL_BYTE_CNT;
    return HAL_ERROR;
  }
  // Now Save Byte Count.
  SrecPtr->ByteCount = *ByteBffr;
  // Now get all remaining bytes in Record.
  if (Get_Bytes( SrecPtr->ByteCount+1, &tempBffr[2], ByteBffr ) != HAL_OK)
  {
    // Illegal Byte Count Value.
    SrecPtr->Srec_Err = ILLEGAL_BYTE_DATA;
    return HAL_ERROR;
  }
  // Place ChkSum into Structure.
  SrecPtr->Checksum = ByteBffr[SrecPtr->ByteCount];
  // Now Calculate CheckSum.
  Calc_Checksum = Calc_ChkSum( (SrecPtr->ByteCount), ByteBffr );
  if (Calc_Checksum != SrecPtr->Checksum)
  {
    // Illegal Byte Count Value.
    SrecPtr->Srec_Err = BAD_CHECKSUM;
    return HAL_ERROR;
  }
  
  // OK...Now Determine what Kind of S-Record and process it.
  switch (tempBffr[1])
  {
    case '0':
      // S0: Header: vendor specific ASCII text represented as a series of hex digit pairs
      SrecPtr->RecordType = S0_HEADER;
      break;
    case '1':
      // S1: Data: data that starts at the 16-bit address field.
      SrecPtr->RecordType = S1_DATA;
      // Now Get Address Field...
      SrecPtr->Address = (uint32_t)(ByteBffr[1]<<8) + ByteBffr[2];
      // Populate Data Field
      for (x=3; x<SrecPtr->ByteCount; x++)
        SrecPtr->Data[x-3] = ByteBffr[x];
      break;
    case '2':
      // S2: Data: data that starts at a 24-bit address.
      SrecPtr->RecordType = S2_DATA;
      // Now Get Address Field...
      SrecPtr->Address = (uint32_t)(ByteBffr[1]<<16) + (uint32_t)(ByteBffr[2]<<8) + ByteBffr[3];
      // Populate Data Field
      for (x=4; x<SrecPtr->ByteCount; x++)
        SrecPtr->Data[x-4] = ByteBffr[x];
      break;
    case '3':
      // S3: Data: data that starts at a 32-bit address.
      SrecPtr->RecordType = S3_DATA;
      // Now Get Address Field...
      SrecPtr->Address = (uint32_t)(ByteBffr[1]<<24) + (uint32_t)(ByteBffr[2]<<16) + (uint32_t)(ByteBffr[3]<<8) + ByteBffr[4];
      // Populate Data Field
      for (x=5; x<SrecPtr->ByteCount; x++)
        SrecPtr->Data[x-5] = ByteBffr[x];
      break;
    case '4':
      // S4: Reserved: ERROR.
      SrecPtr->Srec_Err = RESERVED_RECORD;
      return HAL_ERROR;
      break;
    case '5':
      // S5: Count: 16-bit count of S1 / S2 / S3 records.
      SrecPtr->RecordType = S5_COUNT;
      // Now Get 16-Bit Count Field...
      SrecPtr->ByteCount = (uint32_t)(ByteBffr[1]<<8) + ByteBffr[2];
      break;
    case '6':
      // S6: Count: 24-bit count of S1 / S2 / S3 records.
      SrecPtr->RecordType = S6_COUNT;
      // Now Get 24-Bit Count Field...
      SrecPtr->ByteCount = (uint32_t)(ByteBffr[1]<<16) + (uint32_t)(ByteBffr[2]<<8) + ByteBffr[3];
      break;
    case '7':
      // S7: Start Address: starting execution location at a 32-bit address.
      SrecPtr->RecordType = S7_START;
      // Now Get Address Field...
      SrecPtr->Address = (uint32_t)(ByteBffr[1]<<24) + (uint32_t)(ByteBffr[2]<<16) + (uint32_t)(ByteBffr[3]<<8) + ByteBffr[4];
      break;
    case '8':
      // S8: Start Address: starting execution location at a 24-bit address.
      SrecPtr->RecordType = S8_START;
      // Now Get Address Field...
      SrecPtr->Address = (uint32_t)(ByteBffr[1]<<16) + (uint32_t)(ByteBffr[2]<<8) + ByteBffr[3];
      break;
    case '9':
      // S9: Start Address: starting execution location at a 16-bit address.
      SrecPtr->RecordType = S9_START;
      // Now Get Address Field...
      SrecPtr->Address = (uint32_t)(ByteBffr[1]<<8) + ByteBffr[2];
      break;
    default:
      // ERROR
      SrecPtr->Srec_Err = ILLEGAL_RECORD;
      return HAL_ERROR;
      break;
  } //EndSwitch (tempBffr[1])

  // Done Return HAL_OK.
  return HAL_OK;
}

/**
  * @brief  This function converts the passed bffr to Bytes.
  * @param  uint8_t size        Number of Bytes to translate
  * @param  char *tempBffr      Pointer to passed Chars to be translated
  * @param  uint8_t *bytePtr    Resulting bytes converted from Characters passed.
  * @retval HAL_StatusTypeDef:     HAL_OK:       No Errors
  *                                HAL_ERROR:    Error Found during initialization.
  */
HAL_StatusTypeDef Get_Bytes( uint8_t size, char *tempBffr, uint8_t *bytePtr )
{
  uint8_t temp_Size;
  uint8_t temp_Value;
  uint8_t temp_Value2;
  
  for (temp_Size=0; temp_Size<size; temp_Size++)
  {
    // Get First Char in pointed and verify if Number 0-9 or a-f or A-F.
    if (Get_Nibble( tempBffr, &temp_Value ) != HAL_OK)
      return HAL_ERROR;
    tempBffr++;
    
    // OK. We Now Have a good value. Convert to MSB Value.
    temp_Value = temp_Value << 4;
    // Get Next Nibble
    if (Get_Nibble( tempBffr, &temp_Value2 ) != HAL_OK)
      return HAL_ERROR;
    tempBffr++;

    // Now Add the new Value into result
    temp_Value += temp_Value2;
    // Store Result.
    *bytePtr = temp_Value;
    bytePtr++;
  } //Enfor (temp_Size=0; temp_Size<size; temp_Size++)
  
  // Done. Return HAL_OK.
  return HAL_OK;
}

/**
  * @brief  This function converts the passed Char to Byte.
  * @param  char *tempBffr      Pointer to passed Char to be translated
  * @param  uint8_t *Result     Resulting byte converted from Character passed.
  * @retval HAL_StatusTypeDef:     HAL_OK:       No Errors
  *                                HAL_ERROR:    Error Found during initialization.
  */
HAL_StatusTypeDef Get_Nibble( char *tempBffr, uint8_t *Result )
{
  // Get Char in pointed and verify if Number 0-9 or a-f or A-F.
  if ( (*tempBffr>= '0') && (*tempBffr<= '9') )
  {
    *Result = (uint8_t)(*tempBffr) - (uint8_t)('0');
  }
  else if ( (*tempBffr>= 'a') && (*tempBffr<= 'f') )
  {
    *Result = ((uint8_t)(*tempBffr) - (uint8_t)('a')) + 10;
  }
  else if ( (*tempBffr>= 'A') && (*tempBffr<= 'F') )
  {
    *Result = ((uint8_t)(*tempBffr) - (uint8_t)('A')) + 10;
  }
  else
    return HAL_ERROR;
  return HAL_OK;
}

/**
  * @brief  This function calculates the Checksum of the passed data.
  * @param  uint8_t size        Number of Bytes to translate
  * @param  uint8_t *bytePtr    Bffr of Bytes to Calculate Checksum.
  * @retval uint8_t:     Calculated Checksum
  */
uint8_t Calc_ChkSum( uint8_t size, uint8_t *bytePtr )
{
  uint16_t temp_value;
  uint8_t final_value;
  int x;
  
  // Clear temp_value
  temp_value = 0;
  
  // Now Calculate Checksum.
  for (x=0; x<size; x++)
  {
    temp_value += *bytePtr++;
  }
  
  // Now Convert to final_value
  final_value = (uint8_t)(temp_value & 0xff);
  
  // Get ones compliment
  final_value ^=0xff;

  return final_value;
}

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
