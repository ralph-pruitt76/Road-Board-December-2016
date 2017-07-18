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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __Humidity_H
#define __Humidity_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"
#include "i2c.h"
#include "Temperature.h"
#include "parser.h"

/* Defines */
#define HUMIDITY_SCALE1          524           // Humidity Scale for Legacy design.
#define HUMIDITY_SCALE2          6             // Humidity Scale for Legacy design.
   
// I2C Addresses
#define HMDTY_SNSR              0xBE           // Humidity Sensor U9(HTS221TR).  Addr: 0xBE
   
// I2C Commands
#define HMDTY_SNSR_WHO_AM_I             0x0F            // (Page 18)7.1 Device identification
#define HMDTY_SNSR_AV_CONF              0x10            // (Page 18)7.2 Humidity and temperature resolution mode
#define HMDTY_SNSR_CTRL_REG1            0x20            // (Page 19)7.3 Control register 1
#define HMDTY_SNSR_CTRL_REG2            0x21            // (Page 21)7.4 Control register 2
#define HMDTY_SNSR_CTRL_REG3            0x22            // (Page 22)7.5 Control register 3
#define HMDTY_SNSR_STATUS_REG           0x27            // (Page 22)7.6 Status register
#define HMDTY_SNSR_HUMIDITY_OUT_L       0x28            // (Page 23)7.7 Relative humidity data (LSB)
#define HMDTY_SNSR_HUMIDITY_OUT_H       0x29            // (Page 23)7.8 Relative humidity data (MSB)
#define HMDTY_SNSR_TEMP_OUT_L           0x2A            // (Page 23)7.9 Temperature data (LSB)
#define HMDTY_SNSR_TEMP_OUT_H           0x2B            // (Page 23)7.10 Temperature data (MSB)
#define HMDTY_SNSR_CALIB_TABLE          0x30            // (Page 25)8 Humidity and temperature data conversion
                                                        //      The Registers in 30h..3Fh address range contain 
                                                        //      calibration coefficients. Every sensor module has 
                                                        //      individual coefficients. Before the first calculation 
                                                        //      of temperature and humidity, the master reads out the 
                                                        //      calibration coefficients.
                                                        //      (NO NEED TO MODIFY THESE VALUES. SET AT FACTORY.)
                                                        //      (USED TO CALCULATE HUMIDITY AND TEMPERATURE.) 
#define HMDTY_SNSR_H0rH_x2              0x30            // (Page 25)8 H0rH_x2 Coefficient
#define HMDTY_SNSR_H1rH_x2              0x31            // (Page 25)8 H1rH_x2 Coefficient
#define HMDTY_SNSR_T0_degC_x8           0x32            // (Page 25)8 T0_degC_x8 Coefficient
#define HMDTY_SNSR_T1_degC_x8           0x33            // (Page 25)8 T1_degC_x8 Coefficient
#define HMDTY_SNSR_T1_T0_msb            0x35            // (Page 25)8 T1_T0_msb Coefficient
#define HMDTY_SNSR_H0_T0_OUT            0x36            // (Page 25)8 H0_T0_OUT Coefficient
#define HMDTY_SNSR_H1_T0_OUT            0x3A            // (Page 25)8 H1_T0_OUT Coefficient
#define HMDTY_SNSR_T0_OUT               0x3C            // (Page 25)8 T0_OUT Coefficient
#define HMDTY_SNSR_T1_OUT               0x3E            // (Page 25)8 T1_OUT Coefficient
 
   
// Table 16. Humidity and temperature average configuration(Page 18)
#define HMDTY_SNSR_AVGHx2_0             0x00    // NrAvg:4      Noise:0.4%      Crrnt at 1Hz: 0.8uA
#define HMDTY_SNSR_AVGHx2_1             0x01    // NrAvg:8      Noise:0.3%      Crrnt at 1Hz: 1.05uA
#define HMDTY_SNSR_AVGHx2_2             0x02    // NrAvg:16     Noise:0.2%      Crrnt at 1Hz: 1.4uA
#define HMDTY_SNSR_AVGHx2_3             0x03    // NrAvg:32     Noise:0.15%     Crrnt at 1Hz: 2.1uA
#define HMDTY_SNSR_AVGHx2_4             0x04    // NrAvg:64     Noise:0.1%      Crrnt at 1Hz: 3.43uA
#define HMDTY_SNSR_AVGHx2_5             0x05    // NrAvg:128    Noise:0.07%     Crrnt at 1Hz: 6.15uA
#define HMDTY_SNSR_AVGHx2_6             0x06    // NrAvg:256    Noise:0.05%     Crrnt at 1Hz: 11.6uA
#define HMDTY_SNSR_AVGHx2_7             0x07    // NrAvg:512    Noise:0.03%     Crrnt at 1Hz: 22.5uA

#define HMDTY_SNSR_AVGTx2_0             0x00    // NrAvg:2      Noise:0.08DegC  Crrnt at 1Hz: 0.8uA
#define HMDTY_SNSR_AVGTx2_1             0x08    // NrAvg:4      Noise:0.05DegC  Crrnt at 1Hz: 1.05uA
#define HMDTY_SNSR_AVGTx2_2             0x10    // NrAvg:8      Noise:0.04DegC  Crrnt at 1Hz: 1.4uA
#define HMDTY_SNSR_AVGTx2_3             0x18    // NrAvg:16     Noise:0.03DegC  Crrnt at 1Hz: 2.1uA
#define HMDTY_SNSR_AVGTx2_4             0x20    // NrAvg:32     Noise:0.02DegC  Crrnt at 1Hz: 3.43uA
#define HMDTY_SNSR_AVGTx2_5             0x28    // NrAvg:64     Noise:0.015DegC    Crrnt at 1Hz: 6.15uA
#define HMDTY_SNSR_AVGTx2_6             0x30    // NrAvg:128    Noise:0.01DegC     Crrnt at 1Hz: 11.6uA
#define HMDTY_SNSR_AVGTx2_7             0x38    // NrAvg:256    Noise:0.007DegC    Crrnt at 1Hz: 22.5uA

#define HMDTY_SNSR_PD_OFF               0x00    // PD: power down control  0: power-down mode
#define HMDTY_SNSR_PD_ON                0x80    // PD: power down control  1: active mode

#define HMDTY_SNSR_BDU_CONT             0x00    // BDU: block data update  0: continuous update
#define HMDTY_SNSR_BDU_BLK              0x04    // BDU: block data update  1: output registers not updated until MSB and LSB reading

// Table 17. Output data rate configuration
#define HMDTY_SNSR_ODR_OneShot          0x00    // ODR: output data rate selection  00: One Shot
#define HMDTY_SNSR_ODR_1Hz              0x01    // ODR: output data rate selection  01: 1 Hz
#define HMDTY_SNSR_ODR_7Hz              0x02    // ODR: output data rate selection  10: 7 Hz
#define HMDTY_SNSR_ODR_12_5Hz           0x03    // ODR: output data rate selection  22: 12.5 Hz

#define HMDTY_SNSR_BOOT_NRML            0x00    // BOOT: Reboot memory content  0: normal mode
#define HMDTY_SNSR_BOOT_REBOOT          0x80    // BOOT: Reboot memory content  1: reboot memory content

#define HMDTY_SNSR_HEATER_OFF           0x00    // Heater:  0: heater disable
#define HMDTY_SNSR_HEATER_ON            0x02    // Heater:  1: heater enable

#define HMDTY_SNSR_OneSht_START         0x01    // One shot enable:   1: start for a new dataset

#define HMDTY_SNSR_DRDYHL_ACTVH         0x00    // DRDY_H_L: Data Ready output signal active high, low   0: active high -default
#define HMDTY_SNSR_DRDYHL_ACTVL         0x80    // DRDY_H_L: Data Ready output signal active high, low   1: active low
   
#define HMDTY_SNSR_PPOD_PSHPLL          0x00    // PP_OD: Push-pull / Open Drain selection on pin 3 (DRDY)   0: push-pull - default
#define HMDTY_SNSR_PPOD_OPNDRN          0x40    // PP_OD: Push-pull / Open Drain selection on pin 3 (DRDY)   1: open drain
   
#define HMDTY_SNSR_DRDY_NO              0x00    // DRDY_EN: Data Ready enable   0: Data Ready disabled - default
#define HMDTY_SNSR_DRDY_EN              0x04    // DRDY_EN: Data Ready enable   1: Data Ready signal available on pin 3

// Structure
typedef struct Humidty {
   uint8_t Humidity[8];
   uint8_t HRaw[7];
   uint16_t HRawC;
} Humidity;
typedef struct Humidty *HumidtyPtr;

/* Prototypes */
HAL_StatusTypeDef RoadBrd_HumidityInit( void );
HAL_StatusTypeDef RoadBrd_Humidity_ReadHumidity( HumidtyPtr HPtr );
HAL_StatusTypeDef RoadBrd_Humidity_ReadHumidity_Scaled( HumidtyPtr HPtr );
HAL_StatusTypeDef RoadBrd_Humidity_ReadTemperature( TempPtr TPtr );
HAL_StatusTypeDef RoadBrd_Humidity_ReadTemperature_Scaled( TempPtr TPtr );

#ifdef __cplusplus
}
#endif
#endif /*__ adc_H */

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
