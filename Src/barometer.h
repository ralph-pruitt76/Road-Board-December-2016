/**
  ******************************************************************************
  * File Name          : baromoeter.h
  * Description        : This file provides code for the control of the baromoeter
  *                      hardware based on the LPS22HB chip.
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
#ifndef __baromoeter_H
#define __baromoeter_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"
#include "i2c.h"
#include "parser.h"
#include "lps25hb.h"
#include "Temperature.h"
   
/* Defines */
#define WHOAMI_RSLT             0xBD            // Result returned from WhoAmI register.
#define BARO_ADDR               0xBA
#define TDA_TEST               0x01
//    Bit 0: T_DA			0: new data for temperature is not yet available
//    				1: new data for temperature is available
#define PDA_TEST               0x02
//    Bit 1: P_DA			0: new data for pressure is not yet available
//    				1: new data for pressure is available
#define TOR_TEST               0x10
//    Bit 4: T_OR			0: no overrun has occurred    
//    				1: a new data for temperature has overwritten the previous one
#define POR_TEST               0x20
//    Bit 5: P_OR			0: no overrun has occurred
#define SCALE_FACTOR            4096    // See Section 4.4 in LPS22HB Guide
#define BARO_WAITCNT            100     // Wait 500 ms

#define WAIT_PRESSEVNT          300      // Wait 300 msec after event before testing for Status.
#define WAIT_PRESSURE           5        // Wait 5 msec from each check of status.
#define HW_RESET_TIME           5000      // Wait 500 msec After HW reset before continuing.
   


#define LPS25HB

#ifdef LPS22HB  
// Barometer Registers Mapped
#define INTERRUPT_CFG           0x0b            // 9.1 Interrupt mode for pressure acquisition configuration.
#define THS_P_L                 0x0c            // 9.2 User defined threshold value for pressure interrupt event (Least significant bits).
#define THS_P_H                 0x0d            // 9.3 User defined threshold value for pressure interrupt event (Most significant bits).
#define WHO_AM_I                0x0f            // 9.4 Device Who am I ...0xd1
#define CTRL_REG1               0x10            // 9.5 Control register 1
#define CTRL_REG2               0x11            // 9.6 Control register 2
#define CTRL_REG3               0x12            // 9.7 Control register 3 - INT_DRDY pin control register
#define FIFO_CTRL               0x14            // 9.8 FIFO control register
#define RPDS_L                  0x18            // 9.12 Pressure offset (LSB data)
#define RPDS_H                  0x19            // 9.13 Pressure offset (MSB data)
#define RES_CONF                0x1a            // 9.14 Low-power mode configuration
#define INT_SOURCE              0x25            // 9.15 Interrupt source
#define FIFO_STATUS             0x26            // 9.16 FIFO status
#define STATUS                  0x27            // 9.17 Status register
#define PRESS_OUT_XL            0x28            // 9.18 Pressure output value (LSB)
#define PRESS_OUT_L             0x29            // 9.19 Pressure output value (mid part)
#define PRESS_OUT_H             0x2a            // 9.20 Pressure output value (MSB)
#define TEMP_OUT_L              0x2b            // 9.21 Temperature output value (LSB)
#define TEMP_OUT_H              0x2c            // 9.22 Temperature output value (MSB)
#define FPFP_RES                0x33            // 9.23 Low-pass filter reset register.
#endif

#ifdef LPS25HB
// Barometer Registers Mapped
#define REF_P_XL                0x08            // 8.1 Reference pressure (LSB data)
#define REF_P_L                 0x09            // 8.2 Reference pressure (middle part)
#define REF_P_H                 0x0a            // 8.3 Reference pressure (MSB part)
#define WHO_AM_I                0x0f            // 8.4 Device Who am I ...0xd1
#define RES_CONF                0x10            // 8.5 Pressure and temperature resolution
#define CTRL_REG1               0x20            // 8.6 Control register 1
#define CTRL_REG2               0x21            // 8.7 Control register 2
#define CTRL_REG3               0x22            // 8.8 Control register 3 - Interrupt control
#define CTRL_REG4               0x23            // 8.9 Control register 4 - Interrupt configuration
#define INTERRUPT_CFG           0x24            // 8.10 Interrupt configuration.
#define INT_SOURCE              0x25            // 8.11 Interrupt source
#define STATUS                  0x27            // 8.12 Status register
#define PRESS_OUT_XL            0x28            // 8.13 Pressure output value (LSB)
#define PRESS_OUT_L             0x29            // 8.14 Pressure output value (mid part)
#define PRESS_OUT_H             0x2a            // 8.15 Pressure output value (MSB)
#define TEMP_OUT_L              0x2b            // 8.16 Temperature output value (LSB)
#define TEMP_OUT_H              0x2c            // 8.17 Temperature output value (MSB)
#define FIFO_CTRL               0x2e            // 8.18 FIFO control register
#define FIFO_STATUS             0x2f            // 8.19 FIFO status
#define THS_P_L                 0x30            // 8.20 User defined threshold value for pressure interrupt event (Least significant bits).
#define THS_P_H                 0x31            // 8.21 User defined threshold value for pressure interrupt event (Most significant bits).
#define RPDS_L                  0x39            // 8.22 Pressure offset (LSB data)
#define RPDS_H                  0x3a            // 8.23 Pressure offset (MSB data)
#define FPFP_RES                0xff            // NOT USED
#endif
   
// Structure
typedef struct PRStat {
  uint8_t Status;
  uint8_t Raw[5];
 } PRStatus;
typedef struct PRStat *PRStatPtr;

typedef struct PRPrs {
  uint8_t Pressure[12];
  uint8_t Raw[10];
  uint32_t RawC;
 } PRPressure;
typedef struct PRPrs *PRPrsPtr;

/* Prototypes */
HAL_StatusTypeDef RoadBrd_Init_Barometer( void );
HAL_StatusTypeDef RoadBrd_Enable_Barometer( void );
HAL_StatusTypeDef RoadBrd_Disable_Barometer( void );
HAL_StatusTypeDef RoadBrd_StartSample_Barometer( void );
HAL_StatusTypeDef RoadBrd_StartSample_BarometerWait( void );
HAL_StatusTypeDef RoadBrd_Barometer_Status( PRStatPtr SPtr );
HAL_StatusTypeDef RoadBrd_Baro_ReadPressure( PRPrsPtr PRPtr );
HAL_StatusTypeDef RoadBrd_Baro_ReadPressure_Scaled( PRPrsPtr PRPtr );
HAL_StatusTypeDef RoadBrd_Baro_ReadTemp( TempPtr TmpPtr );
HAL_StatusTypeDef RoadBrd_WaitForPressure( uint16_t WaitCnt );
HAL_StatusTypeDef RoadBrd_TestandRead_Barometer( void );

#ifdef __cplusplus
}
#endif
#endif /*__ adc_H */

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
