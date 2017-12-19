/**
  ******************************************************************************
  * File Name          : BootMonitor.h
  * Description        : This file provides code that parses the passed string and 
  *                      performs the requested operation. It is the Stand Alone
  *                      Boot Monitor Code Module.
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BootMonitor_H
#define __BootMonitor_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"
#include "stm32l1xx_nucleo.h"
#include "i2c.h"
#include <string.h>
#include "usart.h"
#include "gpio.h"
#include "Temperature.h"
#include "parser.h"
//#include "main.h"
//#include "mbed.h"

/* Defines */
//#define I2C_TIMEOUT     500            // Set as 500msec
//#define SCALE_DELAY     10             // Scale Delay.
//#define BUFFER_SIZE     40             // Maximum Buffer Size

/* Prototypes */
HAL_StatusTypeDef Parse_BootString(char *tempBffr, bool BLE_Flag);
bool Tst_Boot_Bypass( void);
void Set_Boot_Bypass( void );

#ifdef __cplusplus
}
#endif
#endif /*__ adc_H */

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/