/**
  ******************************************************************************
  * File Name          : Rd_Sound.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __Calibration_H
#define __Calibration_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"
#include <Math.h>
#include <string.h>
#include "stdbool.h"

// Enums
typedef enum Cal_Char
{
  CAL_SHNT_VLTG     = 0, 
  CAL_CURRENT       = 1,
  CAL_POWER         = 2,
  CAL_VOLTAGE       = 3,
  CAL_TEMPC         = 4,
  CAL_TEMPF         = 5,
  CAL_PRESSURE      = 6,
  CAL_HUMIDITY      = 7,
  CAL_HUM_TEMPC     = 8,
  CAL_HUM_TEMPF     = 9,
  CAL_RGB_RED       = 10,
  CAL_RGB_GREEN     = 11,
  CAL_RGB_BLUE      = 12,
  CAL_THERM_C       = 13,
  CAL_ROADT_1C      = 14,
  CAL_ROADT_2C      = 15,
  CAL_ROADT_3C      = 16,
  CAL_ROADT_4C      = 17,
  CAL_ROADT_5C      = 18,
  CAL_ROADT_6C      = 19,
  CAL_ROADT_7C      = 20,
  CAL_ROADT_8C      = 21,
  CAL_LAST_VALUE    = 22,
} Cal_Characteristic;

/* Defines */
#define CALIBRATION_CHKSUM      0x5a5a5a5a      // Code to determine if Calibration has been Initialized
#define CALIBRATION_DATA_SIZE   CAL_LAST_VALUE  // Size of Calibration Structure.
#define CAL_TIMESTRING_SIZE     20              // Size of Calibration Time String.

// Structure
typedef struct Cal_Vals
{
  float         slope;
  float         offset;
} Cal_Values;
typedef struct Cal_Vals *Cal_ValsPtr;

typedef struct Calibration_Frmes
{
  uint32_t      checksum;
  Cal_Values    Cal_Entry[CALIBRATION_DATA_SIZE];
  uint8_t       TimeString[CAL_TIMESTRING_SIZE];
} Calibration_Frames;
typedef struct Calibration_Frmes *Calibration_FrmesPtr;

/* Prototypes */
bool RoadBrd_CAL_VerifyFrame( void );
HAL_StatusTypeDef RoadBrd_CAL_InitializeFrmFlash( void );
HAL_StatusTypeDef RoadBrd_CAL_ReadFrmFlash( void );
HAL_StatusTypeDef RoadBrd_CAL_WriteFrmFlash( void );
float RoadBrd_CAL_GetOffset( Cal_Characteristic Cal_Item );
float RoadBrd_CAL_GetSlope( Cal_Characteristic Cal_Item );
HAL_StatusTypeDef RoadBrd_CAL_Set_CalItem( Cal_Characteristic Cal_Item,
                                           float Offset, 
                                           float Slope);
HAL_StatusTypeDef RoadBrd_CAL_Set_TimeString( uint8_t *time_stringP );
char *RdBrd_CAL_GetStr( Cal_Characteristic StringCds );
float RoadBrd_CAL_ScaleValue( Cal_Characteristic Cal_Item, float Old_value);
uint8_t *RoadBrd_CAL_GetTimeString( void );

#ifdef __cplusplus
}
#endif
#endif /*__Rd_Sound_H */

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
