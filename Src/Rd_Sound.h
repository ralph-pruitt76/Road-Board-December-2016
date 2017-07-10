/**
  ******************************************************************************
  * File Name          : Rd_Sound.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __Rd_Sound_H
#define __Rd_Sound_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"
#include "adc.h"
#include "fix_fft.h"
#include "i2c.h"
#include <Math.h>
#include <string.h>

/* Defines */
#define FFT_BUFFER_SIZE         128     // Size of FFT Buffer to be used for conversion.
#define MAX_TIMEOUT             200     // Timeout is set for 1 Second.

// Structure
typedef struct BinS {
   uint8_t dumpStr[35];
 } BinString;
typedef struct BinS *BinSPtr;

/* Prototypes */
void RoadBrdSnd_ClearImgnryBffr( void );
void RoadBrdSnd_ClearRealBffr( void );
void RoadBrdSnd_ClearBffrs( void );
int8_t* RoadBrdSnd_getImgnryBffr( void );
int8_t* RoadBrdSnd_getRealBffr( void );
int8_t* RoadBrdSnd_getSaveBffr( void );
HAL_StatusTypeDef RoadBrdSnd_SampleSound( int8_t* pData );
HAL_StatusTypeDef RoadBrdSnd_ProcessSound( void );
HAL_StatusTypeDef RoadBrdSnd_DumpBin0( BinSPtr BinPtr );
HAL_StatusTypeDef RoadBrdSnd_DumpBin16( BinSPtr BinPtr );
HAL_StatusTypeDef RoadBrdSnd_DumpBin32( BinSPtr BinPtr );
HAL_StatusTypeDef RoadBrdSnd_DumpBin48( BinSPtr BinPtr );
void RoadBrdSnd_ClrBffrs( void );
   
#ifdef __cplusplus
}
#endif
#endif /*__Rd_Sound_H */

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
