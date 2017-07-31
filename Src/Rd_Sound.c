/**
  ******************************************************************************
  * File Name          : Rd_Sound.c
  * Description        : This file provides code for the implementation of the 
  * Sound processing code used to determine the sound characteristics of the road.
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
#include "Rd_Sound.h"


/* Variables and buffer definitions */
static int8_t fr[FFT_BUFFER_SIZE];
static int8_t fi[FFT_BUFFER_SIZE];
static int8_t fs[FFT_BUFFER_SIZE];

  /**
  * @brief  This function clears both real and imaginary buffers.
  * @param  none.
  * @retval none.
  */
void RoadBrdSnd_ClearBffrs( void )
{
  int x;
  
  for (x=0; x<FFT_BUFFER_SIZE; x++)
  {
    fr[x] = 0;
    fi[x] = 0;
  }
}

  /**
  * @brief  This function clears the real buffer.
  * @param  none.
  * @retval none.
  */
void RoadBrdSnd_ClearRealBffr( void )
{
  int x;
  
  for (x=0; x<FFT_BUFFER_SIZE; x++)
  {
    fr[x] = 0;
  }
}

  /**
  * @brief  This function clears the imaginary buffer.
  * @param  none.
  * @retval none.
  */
void RoadBrdSnd_ClearImgnryBffr( void )
{
  int x;
  
  for (x=0; x<FFT_BUFFER_SIZE; x++)
  {
    fi[x] = 0;
  }
}

  /**
  * @brief  This function return the pointer to the imaginary buffer.
  * @param  none.
  * @retval none.
  */
int8_t* RoadBrdSnd_getImgnryBffr( void )
{
  return fi;
}

   /**
  * @brief  This function return the pointer to the Real buffer.
  * @param  none.
  * @retval none.
  */
int8_t* RoadBrdSnd_getRealBffr( void )
{
  return fr;
}

   /**
  * @brief  This function return the pointer to the Save buffer.
  * @param  none.
  * @retval none.
  */
int8_t* RoadBrdSnd_getSaveBffr( void )
{
  return fs;
}

 /**
  * @brief  This function reads FFT_BUFFER_SIZE(128) sampes into the passed buffer pointer.
  * @param  uint8_t* pData: Ptr to the buffer to receive the sampled data.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Operation completed with no errors.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     ADC is busy.
  *                                HAL_TIMEOUT:  ADC timed out.
  */
HAL_StatusTypeDef RoadBrdSnd_SampleSound( int8_t* pData )
{
  HAL_StatusTypeDef Status;
  int x;
  uint8_t* tempPtr;

  // Start ADC to sample Sound Data and fill buffer pointed to bt pData.
  Status = RoadBrd_ADC_Start((uint32_t*)pData, (uint32_t)FFT_BUFFER_SIZE);
  if(Status != HAL_OK)
    return Status;
  
  // Wait for Operation to complete.
  Status = RoadBrd_ADC_Status();
  for( x=0; x<MAX_TIMEOUT; x++)
  {
    Status = RoadBrd_ADC_Status();
    if(Status == HAL_OK)
      break;
    // If Tasking code...Add wait loop
    RoadBrd_Delay(5);
  }
  
  // Did we timeout?
  if(x >= MAX_TIMEOUT)
    return HAL_TIMEOUT;
  else
  {
    // OK...Adjust the Data for FFT Calculaztion
    tempPtr = (uint8_t *)pData;
    for( x=0; x<FFT_BUFFER_SIZE; x++)
    {
      *tempPtr = *tempPtr-128;
      tempPtr++;
    }
    return Status;
  }
}

 /**
  * @brief  This function reads Road Sound and processes it to build FFT Bins.
  * @param  None.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Operation completed with no errors.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     ADC is busy.
  *                                HAL_TIMEOUT:  ADC timed out.
  */
HAL_StatusTypeDef RoadBrdSnd_ProcessSound( void )
{
  HAL_StatusTypeDef Status;
  float temp_val, temp_real, temp_img;
  int x;
  
  // Sample 128 Sampled from ADC
  Status = RoadBrdSnd_SampleSound( fr );
  if (Status != HAL_OK)
    return Status;
  //TEST CODE....3.704Khz Waveform.
  /*uint8_t fTst[FFT_BUFFER_SIZE] = {  0x34, 0x00, 0x00, 0x00, 0x00, 0x72, 0xe2, 0xe1, 0xe0, 0xdf, 0x6d, 0x00, 0x00, 0x00, 0x00, 0x3b,
                                     0xe2, 0xe1, 0xe1, 0xe0, 0xa3, 0x00, 0x00, 0x00, 0x00, 0x07, 0xb8, 0xe2, 0xe0, 0xe0, 0xd2, 0x26,
                                     0x00, 0x00, 0x00, 0x00, 0x7f, 0xe1, 0xe1, 0xe0, 0xe0, 0x61, 0x00, 0x00, 0x00, 0x00, 0x48, 0xe2,
                                     0xe2, 0xe0, 0xe0, 0x96, 0x00, 0x00, 0x00, 0x00, 0x14, 0xc6, 0xe1, 0xe1, 0xe0, 0xc7, 0x17, 0x00,
                                     0x00, 0x00, 0x00, 0x8d, 0xe2, 0xe1, 0xdf, 0xdf, 0x53, 0x00, 0x00, 0x00, 0x00, 0x55, 0xe2, 0xe1,
                                     0xe0, 0xe0, 0x89, 0x00, 0x00, 0x00, 0x00, 0x20, 0xd4, 0xe1, 0xe1, 0xdf, 0xbc, 0x07, 0x00, 0x00,
                                     0x00, 0x00, 0x9c, 0xe2, 0xe1, 0xe0, 0xe0, 0x44, 0x00, 0x00, 0x00, 0x00, 0x63, 0xe2, 0xe1, 0xe1,
                                     0xdf, 0x7d, 0x00, 0x00, 0x00, 0x00, 0x2c, 0xde, 0xe1, 0xe1, 0xe0, 0xb0, 0x00, 0x00, 0x00, 0x00 };
  
  for (x=0; x<FFT_BUFFER_SIZE; x++)
    fr[x] = fTst[x];*/
  // Save results for possible recall.
  for (x=0; x<FFT_BUFFER_SIZE; x++)
    fs[x] = fr[x];
  
  // Clear Imaginary buffer im[128].
  RoadBrdSnd_ClearImgnryBffr();
  
  // Perform Forward FFT as follows: fix_fft(re,im,7,0).
  x = fix_fft(fr, fi, 7, 0);
  if (x == -1)
    return HAL_ERROR;
  
  // For each item of re in sample size 64
  //    re[i] = sqrt(re[i] * re[i] + im[i] * im[i])
  for (x=0; x<(FFT_BUFFER_SIZE/2); x++)
  {
    temp_real = fr[x];
    temp_img = fi[x];
    temp_val = sqrt((temp_real*temp_real) + (temp_img*temp_img));
//    temp_val = sqrt((fr[x]*fr[x]) + (fi[x]*fi[x]));
    fr[x] = ((uint16_t)temp_val & 0xff);
  }
  
  return Status;
}

  /**
  * @brief  This function Returns Bins 0-15 in the passed structure pointer.
  * @param  BinSPtr BinPtr: Ptr to structure to receive the Bins 0-15 Data.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrdSnd_DumpBin0( BinSPtr BinPtr )
{
  uint8_t tempstr[4];
  HAL_StatusTypeDef Status;
  int x;
  
  Status = HAL_OK;
  
  // Clear pass string fisrt.
  sprintf( (char *)BinPtr->dumpStr, "");
  
  for(x=0; x<16; x++)
  {
    sprintf( (char *)tempstr, "%02x", fr[x]);
    strcat ((char *)BinPtr->dumpStr, (char *)tempstr);
  }
  //strcat((char *)BinPtr->dumpStr, "Bn");
  
  return Status;
}

  /**
  * @brief  This function Returns Bins 16-31 in the passed structure pointer.
  * @param  BinSPtr BinPtr: Ptr to structure to receive the Bins 0-15 Data.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrdSnd_DumpBin16( BinSPtr BinPtr )
{
  uint8_t tempstr[4];
  HAL_StatusTypeDef Status;
  int x;
  
  Status = HAL_OK;
  
  // Clear pass string fisrt.
  sprintf( (char *)BinPtr->dumpStr, "");
  
  for(x=0; x<16; x++)
  {
    sprintf( (char *)tempstr, "%02x", fr[x+16]);
    strcat ((char *)BinPtr->dumpStr, (char *)tempstr);
  }
  //strcat((char *)BinPtr->dumpStr, "Bn");
  
  return Status;
}

  /**
  * @brief  This function Returns Bins 32-47 in the passed structure pointer.
  * @param  BinSPtr BinPtr: Ptr to structure to receive the Bins 0-15 Data.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrdSnd_DumpBin32( BinSPtr BinPtr )
{
  uint8_t tempstr[4];
  HAL_StatusTypeDef Status;
  int x;
  
  Status = HAL_OK;
  
  // Clear pass string fisrt.
  sprintf( (char *)BinPtr->dumpStr, "");
  
  for(x=0; x<16; x++)
  {
    sprintf( (char *)tempstr, "%02x", fr[x+32]);
    strcat ((char *)BinPtr->dumpStr, (char *)tempstr);
  }
  //strcat((char *)BinPtr->dumpStr, "Bn");
  
  return Status;
}

  /**
  * @brief  This function Returns Bins 48-63 in the passed structure pointer.
  * @param  BinSPtr BinPtr: Ptr to structure to receive the Bins 0-15 Data.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrdSnd_DumpBin48( BinSPtr BinPtr )
{
  uint8_t tempstr[4];
  HAL_StatusTypeDef Status;
  int x;
  
  Status = HAL_OK;
  
  // Clear pass string fisrt.
  sprintf( (char *)BinPtr->dumpStr, "");
  
  for(x=0; x<16; x++)
  {
    sprintf( (char *)tempstr, "%02x", fr[x+48]);
    strcat ((char *)BinPtr->dumpStr, (char *)tempstr);
  }
  //strcat((char *)BinPtr->dumpStr, "Bn");
  
  return Status;
}

  /**
  * @brief  This function clears all buffers
  * @param  None
  * @retval None
  */
void RoadBrdSnd_ClrBffrs( void )
{
  int x;
  
  for (x=0; x<FFT_BUFFER_SIZE; x++)
  {
    fr[x] = 0;
    fi[x] = 0;
    fs[x] = 0;
  }
}

  

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/

