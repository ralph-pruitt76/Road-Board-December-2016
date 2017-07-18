/**
  ******************************************************************************
  * File Name          : VMonitor.h
  * Description        : This file provides code for the control of the Power
  *                      monitor hardware based on the ISL28022 chip.
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
#ifndef __VMonitor_H
#define __VMonitor_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"
#include "i2c.h"
#include "parser.h"

/* Defines */
// I2C Addresses
#define VOLTAGE_MNTR    0x82           // Power Monitor U1(ISL28022).  Addr: 0x82
   
// I2C Commands
#define VOLTAGE_MNTR_CNFIG      0x00            // Configuration Register
#define VOLTAGE_MNTR_SHNTV      0x01            // Shunt Voltage Register
#define VOLTAGE_MNTR_BUSV       0x02            // Bus Voltage Register
#define VOLTAGE_MNTR_POWER      0x03            // Power Register
#define VOLTAGE_MNTR_CRRNT      0x04            // Current Register
#define VOLTAGE_MNTR_CALIB      0x05            // Calibration Register
   
#define CALIBRATION_VALUE       0x1062          // Calibration Value for Road Board. See Notes
#define SHNT_VLTG_TICK          1e-5            // Shunt Voltage Tick 10uV
#define CURRENT_TICK            244.141e-6      // Current Tick = 244.141uA
#define POWER_TICK              976.56e-9       // Power Tick = 976.56nW
#define VOLT_TICK               4e-3            // Bus Voltage Tick = 4mV
#define POWER_ADJUST            10000           // Power Adjust = 5000 * 2
#define MV_SCALE_ADJUST         1000            // Millivolt Scale Adjust           
#define MA_SCALE_ADJUST         1000            // Milliamp Scale Adjust           
#define MW_SCALE_ADJUST         1000            // MilliWatt Scale Adjust   

// Structure
typedef struct Vltg {
   uint8_t Voltage[8];
   uint8_t Raw[7];
 } Voltage;
typedef struct Vltg *VoltagePtr;

typedef struct Crnt {
   uint8_t Current[9];
   uint8_t Raw[7];
 } Current;
typedef struct Crnt *CurrentPtr;
   
typedef struct Pwr {
   uint8_t Power[9];
   uint8_t Raw[7];
 } Power;
typedef struct Pwr *PowerPtr;
   
/* Prototypes */
HAL_StatusTypeDef RoadBrd_Init_VMonitor( void );
HAL_StatusTypeDef RoadBrd_VMonitor_RdShntVltg( VoltagePtr VPtr );
HAL_StatusTypeDef RoadBrd_VMonitor_RdShntVltg_Scaled( VoltagePtr VPtr );
HAL_StatusTypeDef RoadBrd_VMonitor_RdCurrent( CurrentPtr CPtr );
HAL_StatusTypeDef RoadBrd_VMonitor_RdCurrent_Scaled( CurrentPtr CPtr );
HAL_StatusTypeDef RoadBrd_VMonitor_RdPower( PowerPtr PPtr );
HAL_StatusTypeDef RoadBrd_VMonitor_RdPower_Scaled( PowerPtr PPtr );
HAL_StatusTypeDef RoadBrd_VMonitor_RdVoltage( VoltagePtr VPtr );
HAL_StatusTypeDef RoadBrd_VMonitor_RdVoltage_Scaled( VoltagePtr VPtr );
   
#ifdef __cplusplus
}
#endif
#endif /*__ adc_H */

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
