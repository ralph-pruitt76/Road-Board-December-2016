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

/* Variables and buffer definitions */
// Frame Structure Define
//wwdg_Frames wwdg_HardFrames  @ 0x08070000;
Calibration_Frames Calibration_HardFrames  @ 0x08071000;
static Calibration_Frames Cal_Save_Frames;



//static uint8_t fr[FFT_BUFFER_SIZE];
//static uint8_t fi[FFT_BUFFER_SIZE];
//static uint8_t fs[FFT_BUFFER_SIZE];

  /**
  * @brief  This function clears both real and imaginary buffers.
  * @param  none.
  * @retval none.
  */
/*void RoadBrdSnd_ClearBffrs( void )
{
  int x;
  
  for (x=0; x<FFT_BUFFER_SIZE; x++)
  {
    fr[x] = 0;
    fi[x] = 0;
  }
}*/

  /**
  * @brief  This function clears the real buffer.
  * @param  none.
  * @retval none.
  */
/*void RoadBrdSnd_ClearRealBffr( void )
{
  int x;
  
  for (x=0; x<FFT_BUFFER_SIZE; x++)
  {
    fr[x] = 0;
  }
}*/
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
HAL_StatusTypeDef RoadBrd_CAL_Set_CalItem( Cal_Characteristic Cal_Item,
                                           float Offset, 
                                           float Slope)
{
  if (Cal_Item < CAL_LAST_VALUE)
  {
    Cal_Save_Frames.Cal_Entry[Cal_Item].offset = Offset;
    Cal_Save_Frames.Cal_Entry[Cal_Item].slope = Slope;
    return HAL_OK;
  }
  return HAL_ERROR;
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

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/

