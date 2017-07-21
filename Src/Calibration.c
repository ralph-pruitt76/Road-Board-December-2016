/**
  ******************************************************************************
  * File Name          : Calibration.c
  * Description        : This file provides code for the implementation of the 
  * Calibration code used to calibrate the Sensors and measurements.
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
#include "Calibration.h"
#include "Flash.h"
#include "wwdg.h"

/* Variables and buffer definitions */

// Frame Structure Define
// Will base new Flash Structure to be a modulo 256 to align to a second Flash Page to ensure it works with the previous structure.
// The previous structure is wwdg_Frames and must be an even Page length.
Calibration_Frames Calibration_HardFrames  @ (BASE_FLASH_ADDRESS + ((sizeof(wwdg_Frames) - (sizeof(wwdg_Frames)%256)) + 256));
static Calibration_Frames Cal_Save_Frames;

// Constant Strings Definition
const char * const SensorStrings[] = { "Shnt_Vltg	",          // CODE 000: CAL_SHNT_VLTG/ Calibration Shunt Voltage String
                                    "Current	",                  // CODE 001: CAL_CURRENT/ Calibration Current String
                                    "Power		",          // CODE 002: CAL_POWER/ Calibration Power String
                                    "Voltage	",                  // CODE 003: CAL_VOLTAGE/ Calibration Voltage String
                                    "TempC		",          // CODE 004: CAL_TEMPC/ Calibration Temperature C String
                                    "TempF		",          // CODE 005: CAL_TEMPF/ Calibration Temperature F String
                                    "Pressure	",                  // CODE 006: CAL_PRESSURE/ Calibration Pressure String
                                    "Humidity	",                  // CODE 007: CAL_HUMIDITY/ Calibration Humidity String
                                    "Hum_TempC	",                  // CODE 008: CAL_HUM_TEMPC/ Calibration Humidity Temperature C String
                                    "Hum_TempF	",                  // CODE 009: CAL_HUM_TEMPF/ Calibration Humidity Temperature F String
                                    "RGB_Red	",                  // CODE 010: CAL_RGB_RED/ Calibration RGB Sensor Red String
                                    "RGB_Green	",                  // CODE 011: CAL_RGB_GREEN/ Calibration RGB Sensor Green String
                                    "RGB_Blue	",                  // CODE 012: CAL_RGB_BLUE/ Calibration RGB Sensor Blue String
                                    "Therm_C	",                  // CODE 013: CAL_THERM_C/ Calibration Thermistor String
                                    "RoadT_1C	",                  // CODE 014: CAL_ROADT_1C/ Calibration Road Temperature Sensor 1 String
                                    "RoadT_2C	",                  // CODE 015: CAL_ROADT_1C/ Calibration Road Temperature Sensor 2 String
                                    "RoadT_3C	",                  // CODE 016: CAL_ROADT_1C/ Calibration Road Temperature Sensor 3 String
                                    "RoadT_4C	",                  // CODE 017: CAL_ROADT_1C/ Calibration Road Temperature Sensor 4 String
                                    "RoadT_5C	",                  // CODE 018: CAL_ROADT_1C/ Calibration Road Temperature Sensor 5 String
                                    "RoadT_6C	",                  // CODE 019: CAL_ROADT_1C/ Calibration Road Temperature Sensor 6 String
                                    "RoadT_7C	",                  // CODE 020: CAL_ROADT_1C/ Calibration Road Temperature Sensor 7 String
                                    "RoadT_8C	",                  // CODE 021: CAL_ROADT_1C/ Calibration Road Temperature Sensor 8 String
                                    "       ",                      // CODE 022: NULL...
                                    "       ",                      // CODE 023: NULL...
                                    "       " };                    // CODE 024: NULL...

/**
* @brief  This function verifies the WWDG Flash Frame Structure.
* @param  none
* @retval bool:     true:       Valid Frames
*                   false:      Frame Bad.
*/
bool RoadBrd_CAL_VerifyFrame( void )
{
  HAL_StatusTypeDef Status;
  
  Status = HAL_OK;
  // Read Frame from Flash.
  Status = RoadBrd_FlashRead(  (uint32_t)&Calibration_HardFrames, (uint32_t *)&Cal_Save_Frames, sizeof(Cal_Save_Frames));
  // Compare SYnc Workd and return status.
  if (Status != HAL_OK)
    return false;
  else
  {
    if (Cal_Save_Frames.checksum == CALIBRATION_CHKSUM)
      return true;
    else
      return false;
  }
}

/**
* @brief  This function initializes the Calibration Structure.
* @param  none
* @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
*                                HAL_ERROR:    Error found in Tasking or data passed.
*                                HAL_BUSY:     Flash is busy.
*                                HAL_TIMEOUT:  Flash timed out.
*/
HAL_StatusTypeDef RoadBrd_CAL_InitializeFrmFlash( void )
{
  HAL_StatusTypeDef Status;
  int x;
  
  Status = HAL_OK;
  // Initialize Key Structures of Frame
  Cal_Save_Frames.checksum = CALIBRATION_CHKSUM;
  strcpy( (char *)Cal_Save_Frames.TimeString, "-------EMPTY-------");
  for (x=0; x<CALIBRATION_DATA_SIZE; x++)
  {
    Cal_Save_Frames.Cal_Entry[x].offset = 0.0;
    Cal_Save_Frames.Cal_Entry[x].slope = 1.0;
  }
  // Write Structure to Flash Memory.
  Status = RoadBrd_FlashWrite( 0x00, 
                               FLASH_TYPEERASE_PAGES, 
                               (uint32_t)&Calibration_HardFrames, 
                               (uint32_t *)&Cal_Save_Frames, 
                               sizeof(Cal_Save_Frames));
  return Status;
}

  /**
  * @brief  This function Reads the Calibration Structure from Flash..
  * @param  none
  * @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     Flash is busy.
  *                                HAL_TIMEOUT:  Flash timed out.
  */
HAL_StatusTypeDef RoadBrd_CAL_ReadFrmFlash( void )
{
  HAL_StatusTypeDef Status;
  
  Status = HAL_OK;
  // Read Structure from Flash Memory.
  Status = RoadBrd_FlashRead(  (uint32_t)&Calibration_HardFrames, 
                               (uint32_t *)&Cal_Save_Frames, 
                               sizeof(Cal_Save_Frames));
  return Status;
}

  /**
  * @brief  This function writes the Calibration Structure to Flash..
  * @param  none
  * @retval HAL_StatusTypeDef:     HAL_OK:       Flash Operation success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     Flash is busy.
  *                                HAL_TIMEOUT:  Flash timed out.
  */
HAL_StatusTypeDef RoadBrd_CAL_WriteFrmFlash( void )
{
  HAL_StatusTypeDef Status;
  
  Status = HAL_OK;
  // Write Structure to Flash Memory.
  Status = RoadBrd_FlashWrite( 0x00, 
                               FLASH_TYPEERASE_PAGES, 
                               (uint32_t)&Calibration_HardFrames, 
                               (uint32_t *)&Cal_Save_Frames, 
                               sizeof(Cal_Save_Frames));
  return Status;
}

/**
  * @brief  This function initializes the Calibration Structure.
  * @param  Cal_Characteristic Cal_Item: Indexed Calibration Item.
  * @param  float Offset:                Offset to be set into indexed Cal Item.
  * @param  float Slope:                 Slope to be set into indexed Cal Item.
  * @param  Cal_Characteristic Cal_Item: Indexed Calibration Item.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Operation success.
  *                                HAL_ERROR:    found in Tasking or data passed.
  */
float RoadBrd_CAL_ScaleValue( Cal_Characteristic Cal_Item, float Old_value)
{
  return ( (RoadBrd_CAL_GetSlope(Cal_Item) * Old_value) + RoadBrd_CAL_GetOffset(Cal_Item));
}

/**
  * @brief  This function initializes the Calibration Structure.
  * @param  Cal_Characteristic Cal_Item: Indexed Calibration Item.
  * @param  float Offset:                Offset to be set into indexed Cal Item.
  * @param  float Slope:                 Slope to be set into indexed Cal Item.
  * @param  Cal_Characteristic Cal_Item: Indexed Calibration Item.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Operation success.
  *                                HAL_ERROR:    found in Tasking or data passed.
  */
HAL_StatusTypeDef RoadBrd_CAL_Set_CalItem( Cal_Characteristic Cal_Item,
                                           float Offset, 
                                           float Slope)
{
  if (Cal_Item < CAL_LAST_VALUE)
  {
    HAL_StatusTypeDef Status;

    Cal_Save_Frames.Cal_Entry[Cal_Item].offset = Offset;
    Cal_Save_Frames.Cal_Entry[Cal_Item].slope = Slope;
    // OK...Time to update Flash.
    Status = RoadBrd_CAL_WriteFrmFlash();
    return Status;
  }
  return HAL_ERROR;
}

/**
  * @brief  This function Sets the Time string.
  * @param  uint8_t *time_stringP: Pointer to Time string to set.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Operation success.
  *                                HAL_ERROR:    found in Tasking or data passed.
  */
HAL_StatusTypeDef RoadBrd_CAL_Set_TimeString( uint8_t *time_stringP )
{
  strcpy( (char *)Cal_Save_Frames.TimeString, (char *)time_stringP);
  return HAL_OK;
}

/**
  * @brief  This function returns the indexed Calibration Item Offset.
  * @param  Cal_Characteristic Cal_Item: Indexed Calibration Item.
  * @retval float:     Indexed Item Stored Offset.
  */
float RoadBrd_CAL_GetOffset( Cal_Characteristic Cal_Item )
{
  return ( Cal_Save_Frames.Cal_Entry[Cal_Item].offset );
}

/**
* @brief  This function returns the indexed Calibration Item slope.
* @param  Cal_Characteristic Cal_Item: Indexed Calibration Item.
* @retval float:     Indexed Item Stored slope.
*/
float RoadBrd_CAL_GetSlope( Cal_Characteristic Cal_Item )
{
  return ( Cal_Save_Frames.Cal_Entry[Cal_Item].slope );
}

/**
* @brief  This function returns a Pointer to a String for the indicated Characteristic.
* @param  Cal_Characteristic Cal_Item: Indexed Calibration Item.
* @retval char *:     Points to a constant string for the indicated characteristic.
*/
char *RdBrd_CAL_GetStr( Cal_Characteristic StringCds )
{
  return ((char *)SensorStrings[StringCds]);
}

/**
  * @brief  This function Sets the Time string.
  * @param  None
  * @retval uint8_t *time_stringP: Pointer to returned Time String.
  */
uint8_t *RoadBrd_CAL_GetTimeString( void )
{
  return Cal_Save_Frames.TimeString;
}



/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/

