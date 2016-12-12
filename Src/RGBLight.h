/**
  ******************************************************************************
  * File Name          : RGBLight.h
  * Description        : This file provides code for the control of the RGB Light
  *                      sensor hardware based on the ISL91250 chip.
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
#ifndef __RGBLight_H
#define __RGBLight_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"
#include "i2c.h"
#include "parser.h"

/* Defines */
// I2C Addresses
#define RGB_SNSR        0x88           // RGB Color Light Sensor U15(ISL91250).  Addr: 0x88
   
// I2C Commands
#define RGB_SNSR_IDRST          0x00            // COLOR_IDRST			0x00		(Page 9)Device Register (Address: 0x00)
#define RGB_SNSR_CNFG1          0x01            // COLOR_CNFG1			0x01		(Page 9)Configuation-1 Register (Address: 0x01)
#define RGB_SNSR_CNFG2          0x02            // COLOR_CNFG2			0x02
#define RGB_SNSR_CNFG3          0x03            // COLOR_CNFG3			0x03
#define RGB_SNSR_LOWTHRSH_L     0x04            // COLOR_LOWTHRSH_L	        0x04
#define RGB_SNSR_LOWTHRSH_H     0x05            // COLOR_LOWTHRSH_H	        0x05
#define RGB_SNSR_HIGHTHRSH_L    0x06            // COLOR_HIGHTHRSH_L	        0x06
#define RGB_SNSR_HIGHTHRSH_H    0x07            // COLOR_HIGHTHRSH_H	        0x07
#define RGB_SNSR_STATUS         0x08            // COLOR_STATUS		        0x08
#define RGB_SNSR_GREENL         0x09            // COLOR_GREENL		        0x09
#define RGB_SNSR_GREENH         0x0A            // COLOR_GREENH		        0x0A
#define RGB_SNSR_REDL           0x0B            // COLOR_REDL			0x0B
#define RGB_SNSR_REDH           0x0C            // COLOR_REDH			0x0C
#define RGB_SNSR_BLUEL          0x0D            // COLOR_BLUEL			0x0D
#define RGB_SNSR_BLUEH          0x0E            // COLOR_BLUEH			0x0E
   
   
#define RGB_CNFG1_DEFAULT       0x1D            // OP_MODE: 101: GREEN/RED/BLUE
                                                // DS_RANGE: 1: 10,000 lux
                                                // ADC_RSL: 1:12 Bits
                                                // SYNC: 0: ADC start at I2C write 0x01
#define RGB_CNFG2_DEFAULT       0x00            // CMP_ADJST: 0
                                                // CMP_OFFST: 0
#define RGB_CNFG3_DEFAULT       0x00            // INT_ASSGN: 00: No Interrupt
                                                // INT_PERSIST: 00:	One Integrastion Cycle
                                                // CNVRSN_INT: 0:	Disable
#define RGB_RESET_CODE          0x46            // Code to reset RGB Sensor

// Structure
typedef struct RGBInit {
   uint8_t config1;
   uint8_t config2;
   uint8_t config3;
 } RGBInitialize;
typedef struct RGBInit *RGBInitPtr;
  
typedef struct RGBLght {
   uint8_t Red[8];
   uint8_t Green[8];
   uint8_t Blue[8];
   uint8_t Raw[15];
 } RGBLight;
typedef struct RGBLght *RGBLghtPtr;

typedef struct RGBid {
   uint8_t id;
   uint8_t Raw[5];
 } RGBIdent;
typedef struct RGBid *RGBIdentPtr;

typedef struct RGBStat {
   uint8_t status;
   uint8_t Raw[5];
 } RGBStatus;
typedef struct RGBStat *RGBStatusPtr;

/* Prototypes */
HAL_StatusTypeDef RoadBrd_RGBInit( void );
HAL_StatusTypeDef RoadBrd_RGBFullInit( RGBInitPtr LPtr );
HAL_StatusTypeDef RoadBrd_RGBReset( void );
HAL_StatusTypeDef RoadBrd_RGBReadID( RGBIdentPtr id );
HAL_StatusTypeDef RoadBrd_RGBReadStatus( RGBStatusPtr Status );
HAL_StatusTypeDef RoadBrd_RGBReadValues( RGBLghtPtr LPtr );

#ifdef __cplusplus
}
#endif
#endif /*__ adc_H */

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
