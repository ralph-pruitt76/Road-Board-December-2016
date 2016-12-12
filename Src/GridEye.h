/**
  ******************************************************************************
  * File Name          : GridEye.h
  * Description        : This file provides code for the control of the Panasonic
  *                      IR Grid Eye Sensor based on the AMG8851 chip.
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
#ifndef __GridEye_H
#define __GridEye_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"
#include "i2c.h"
#include "Temperature.h"
//#include "parser.h"

/* Defines */
// I2C Addresses
#define GRIDEYE_SNSR            0xD0           // Grid Eye Sensor U16(AMG8851).  Addr: 0xD0
#define COOLEYE_SNSR            0x14           // Cool Eye Sensor U14(TPiL 8T 2246 L3.9 OAA 060).  Addr: 0x14
   
// I2C Commands
#define COOLEYE_SNSR_TEMPBASE   0x12            // Temperature Registers		0x12		(Page 9)Register for reading only to indicate temperature data per 1 pixel.
#define COOLEYE_SNSR_AMBIENT    0x12
#define COOLEYE_SNSR_T01        0x13
#define COOLEYE_SNSR_T02        0x14
#define COOLEYE_SNSR_T03        0x15
#define COOLEYE_SNSR_T04        0x16
#define COOLEYE_SNSR_T05        0x17
#define COOLEYE_SNSR_T06        0x18
#define COOLEYE_SNSR_T07        0x19
#define COOLEYE_SNSR_T08        0x1A
  
#define GRIDEYE_SNSR_PCTL       0x00            // Power Control Register		0x00		(Page 9)Register for setting operating mode of device.
#define GRIDEYE_SNSR_RST        0x01            // Reset Register		        0x01		(Page 10)Register only for writing to reset Hardware.
#define GRIDEYE_SNSR_FPSC       0x02            // Frame Rate Register		        0x02		(Page 10)Register for setting Frame Rate.
#define GRIDEYE_SNSR_INTC       0x03            // Interrupt Control Register		0x03		(Page 10)Register for setting Interrupt Function.
#define GRIDEYE_SNSR_STAT       0x04            // Status Register		        0x04		(Page 11)Register for only reading to indicate Overflow Flag and Interrupt Flag.
#define GRIDEYE_SNSR_SCLR       0x05            // Status Clear Register		0x05		(Page 11)Register for only writing to clear Overflow Flag and Interrupt Flag.
#define GRIDEYE_SNSR_AVE        0x07            // Average Register		        0x07		(Page 12)Register for setting moving average Output Mode.
#define GRIDEYE_SNSR_INTHL      0x08            // Interrupt Level Register Upper LSB	0x08		(Page 12)Register for setting upper limit LSB.
#define GRIDEYE_SNSR_INTHH      0x09            // Interrupt Level Register Upper MSB	0x09		(Page 12)Register for setting upper limit MSB.
#define GRIDEYE_SNSR_INTLL      0x0A            // Interrupt Level Register Lower LSB	0x0A		(Page 12)Register for setting lower limit LSB.
#define GRIDEYE_SNSR_INTLH      0x0B            // Interrupt Level Register Lower MSB	0x0B		(Page 12)Register for setting lower limit MSB.
#define GRIDEYE_SNSR_IHYSL      0x0C            // Interrupt Level Reg Hysteresis LSB	0x0C		(Page 12)Register for setting Hysteresis LSB.
#define GRIDEYE_SNSR_IHYSH      0x0D            // Interrupt Level Reg Hysteresis MSB	0x0D		(Page 12)Register for setting Hysteresis MSB.
#define GRIDEYE_SNSR_TTHL       0x0E            // Thermistor Register LSB		0x0E		(Page 13)Thermistor Temperature Register LSB.
#define GRIDEYE_SNSR_TTHH       0x0F            // Thermistor Register MSB		0x0F		(Page 13)Thermistor Temperature Register MSB.
#define GRIDEYE_SNSR_INT0       0x10            // Interrupt Table Register 0		0x10		(Page 13)Register for reading pixels 1-8 over threshold.
#define GRIDEYE_SNSR_INT1       0x11            // Interrupt Table Register 1		0x11		(Page 13)Register for reading pixels 9-16 over threshold.
#define GRIDEYE_SNSR_INT2       0x12            // Interrupt Table Register 2		0x12		(Page 13)Register for reading pixels 17-24 over threshold.
#define GRIDEYE_SNSR_INT3       0x13            // Interrupt Table Register 3		0x13		(Page 13)Register for reading pixels 25-32 over threshold.
#define GRIDEYE_SNSR_INT4       0x14            // Interrupt Table Register 4		0x14		(Page 13)Register for reading pixels 33-40 over threshold.
#define GRIDEYE_SNSR_INT5       0x15            // Interrupt Table Register 5		0x15		(Page 13)Register for reading pixels 41-48 over threshold.
#define GRIDEYE_SNSR_INT6       0x16            // Interrupt Table Register 6		0x16		(Page 13)Register for reading pixels 49-56 over threshold.
#define GRIDEYE_SNSR_INT7       0x17            // Interrupt Table Register 7		0x17		(Page 13)Register for reading pixels 57-64 over threshold.
#define GRIDEYE_SNSR_TEMPBASE   0x80            // Temperature Registers		0x80		(Page 14)Register for reading only to indicate temperature data per 1 pixel.
#define GRIDEYE_SNSR_T01L       0x80
#define GRIDEYE_SNSR_T01H       0x81
#define GRIDEYE_SNSR_T02L       0x82
#define GRIDEYE_SNSR_T02H       0x83
#define GRIDEYE_SNSR_T03L       0x84
#define GRIDEYE_SNSR_T03H       0x85
#define GRIDEYE_SNSR_T04L       0x86
#define GRIDEYE_SNSR_T04H       0x87
#define GRIDEYE_SNSR_T05L       0x88
#define GRIDEYE_SNSR_T05H       0x89
#define GRIDEYE_SNSR_T06L       0x8A
#define GRIDEYE_SNSR_T06H       0x8B
#define GRIDEYE_SNSR_T07L       0x8C
#define GRIDEYE_SNSR_T07H       0x8D
#define GRIDEYE_SNSR_T08L       0x8E
#define GRIDEYE_SNSR_T08H       0x8F
#define GRIDEYE_SNSR_T09L       0x90
#define GRIDEYE_SNSR_T09H       0x91
#define GRIDEYE_SNSR_T10L       0x92
#define GRIDEYE_SNSR_T10H       0x93
#define GRIDEYE_SNSR_T11L       0x94
#define GRIDEYE_SNSR_T11H       0x95
#define GRIDEYE_SNSR_T12L       0x96
#define GRIDEYE_SNSR_T12H       0x97
#define GRIDEYE_SNSR_T13L       0x98
#define GRIDEYE_SNSR_T13H       0x99
#define GRIDEYE_SNSR_T14L       0x9A
#define GRIDEYE_SNSR_T14H       0x9B
#define GRIDEYE_SNSR_T15L       0x9C
#define GRIDEYE_SNSR_T15H       0x9D
#define GRIDEYE_SNSR_T16L       0x9E
#define GRIDEYE_SNSR_T16H       0x9F
#define GRIDEYE_SNSR_T17L       0xA0
#define GRIDEYE_SNSR_T17H       0xA1
#define GRIDEYE_SNSR_T18L       0xA2
#define GRIDEYE_SNSR_T18H       0xA3
#define GRIDEYE_SNSR_T19L       0xA4
#define GRIDEYE_SNSR_T19H       0xA5
#define GRIDEYE_SNSR_T20L       0xA6
#define GRIDEYE_SNSR_T20H       0xA7
#define GRIDEYE_SNSR_T21L       0xA8
#define GRIDEYE_SNSR_T21H       0xA9
#define GRIDEYE_SNSR_T22L       0xAA
#define GRIDEYE_SNSR_T22H       0xAB
#define GRIDEYE_SNSR_T23L       0xAC
#define GRIDEYE_SNSR_T23H       0xAD
#define GRIDEYE_SNSR_T24L       0xAE
#define GRIDEYE_SNSR_T24H       0xAF
#define GRIDEYE_SNSR_T25L       0xB0
#define GRIDEYE_SNSR_T25H       0xB1
#define GRIDEYE_SNSR_T26L       0xB2
#define GRIDEYE_SNSR_T26H       0xB3
#define GRIDEYE_SNSR_T27L       0xB4
#define GRIDEYE_SNSR_T27H       0xB5
#define GRIDEYE_SNSR_T28L       0xB6
#define GRIDEYE_SNSR_T28H       0xB7
#define GRIDEYE_SNSR_T29L       0xB8
#define GRIDEYE_SNSR_T29H       0xB9
#define GRIDEYE_SNSR_T30L       0xBA
#define GRIDEYE_SNSR_T30H       0xBB
#define GRIDEYE_SNSR_T31L       0xBC
#define GRIDEYE_SNSR_T31H       0xBD
#define GRIDEYE_SNSR_T32L       0xBE
#define GRIDEYE_SNSR_T32H       0xBF
#define GRIDEYE_SNSR_T33L       0xC0
#define GRIDEYE_SNSR_T33H       0xC1
#define GRIDEYE_SNSR_T34L       0xC2
#define GRIDEYE_SNSR_T34H       0xC3
#define GRIDEYE_SNSR_T35L       0xC4
#define GRIDEYE_SNSR_T35H       0xC5
#define GRIDEYE_SNSR_T36L       0xC6
#define GRIDEYE_SNSR_T36H       0xC7
#define GRIDEYE_SNSR_T37L       0xC8
#define GRIDEYE_SNSR_T37H       0xC9
#define GRIDEYE_SNSR_T38L       0xCA
#define GRIDEYE_SNSR_T38H       0xCB
#define GRIDEYE_SNSR_T39L       0xCC
#define GRIDEYE_SNSR_T39H       0xCD
#define GRIDEYE_SNSR_T40L       0xCE
#define GRIDEYE_SNSR_T40H       0xCF
#define GRIDEYE_SNSR_T41L       0xD0
#define GRIDEYE_SNSR_T41H       0xD1
#define GRIDEYE_SNSR_T42L       0xD2
#define GRIDEYE_SNSR_T42H       0xD3
#define GRIDEYE_SNSR_T43L       0xD4
#define GRIDEYE_SNSR_T43H       0xD5
#define GRIDEYE_SNSR_T44L       0xD6
#define GRIDEYE_SNSR_T44H       0xD7
#define GRIDEYE_SNSR_T45L       0xD8
#define GRIDEYE_SNSR_T45H       0xD9
#define GRIDEYE_SNSR_T46L       0xDA
#define GRIDEYE_SNSR_T46H       0xDB
#define GRIDEYE_SNSR_T47L       0xDC
#define GRIDEYE_SNSR_T47H       0xDD
#define GRIDEYE_SNSR_T48L       0xDE
#define GRIDEYE_SNSR_T48H       0xDF
#define GRIDEYE_SNSR_T49L       0xE0
#define GRIDEYE_SNSR_T49H       0xE1
#define GRIDEYE_SNSR_T50L       0xE2
#define GRIDEYE_SNSR_T50H       0xE3
#define GRIDEYE_SNSR_T51L       0xE4
#define GRIDEYE_SNSR_T51H       0xE5
#define GRIDEYE_SNSR_T52L       0xE6
#define GRIDEYE_SNSR_T52H       0xE7
#define GRIDEYE_SNSR_T53L       0xE8
#define GRIDEYE_SNSR_T53H       0xE9
#define GRIDEYE_SNSR_T54L       0xEA
#define GRIDEYE_SNSR_T54H       0xEB
#define GRIDEYE_SNSR_T55L       0xEC
#define GRIDEYE_SNSR_T55H       0xED
#define GRIDEYE_SNSR_T56L       0xEE
#define GRIDEYE_SNSR_T56H       0xEF
#define GRIDEYE_SNSR_T57L       0xF0
#define GRIDEYE_SNSR_T57H       0xF1
#define GRIDEYE_SNSR_T58L       0xF2
#define GRIDEYE_SNSR_T58H       0xF3
#define GRIDEYE_SNSR_T59L       0xF4
#define GRIDEYE_SNSR_T59H       0xF5
#define GRIDEYE_SNSR_T60L       0xF6
#define GRIDEYE_SNSR_T60H       0xF7
#define GRIDEYE_SNSR_T61L       0xF8
#define GRIDEYE_SNSR_T61H       0xF9
#define GRIDEYE_SNSR_T62L       0xFA
#define GRIDEYE_SNSR_T62H       0xFB
#define GRIDEYE_SNSR_T63L       0xFC
#define GRIDEYE_SNSR_T63H       0xFD
#define GRIDEYE_SNSR_T64L       0xFE
#define GRIDEYE_SNSR_T64H       0xFF

// Commands
#define GRIDEYE_PCTL_NORMAL     0x00            // Normal Mode
#define GRIDEYE_PCTL_SLEEP      0x01            // Sleep Mode
#define GRIDEYE_PCTL_STNDBY_60  0x20            // Stand-by Mode (60sec intermittence)
#define GRIDEYE_PCTL_STNDBY_10  0x21            // Stand-by Mode (10sec intermittence)

#define GRIDEYE_FPSC_1FPS       0x01            // 1 Frame per second
#define GRIDEYE_FPSC_10FPS      0x00            // 10 Frames per second
   
#define GRIDEYE_AVE_MAMOD       0x20            // Twice moving average Output Mode
#define GRIDEYE_AVE_NORMAL      0x00            // Normal no average Output Mode

#define GRIDEYE_RST_FLAG        0x30            // Clears all Flags...Status, Ints, and Table
#define GRIDEYE_RST_INIT        0x3F            // Performs flag reset and sets initial values.
#define GRIDEYE_RST_NONE        0x00            // No Action
// Structure

typedef struct GridE {
   Temperature Thermistor;
   Temperature GridEye1;
   Temperature GridEye2;
   Temperature GridEye3;
   Temperature GridEye4;
   Temperature GridEye5;
   Temperature GridEye6;
   Temperature GridEye7;
   Temperature GridEye8;
} GridEye;
typedef struct GridE *GridEPtr;

/* Prototypes */
HAL_StatusTypeDef RoadBrd_GridEyeInit( void );
HAL_StatusTypeDef RoadBrd_GridEyeReset( void );
HAL_StatusTypeDef RoadBrd_GridEye_ReadValues( GridEPtr GPtr );
HAL_StatusTypeDef RoadBrd_CoolEyeInit( void );
HAL_StatusTypeDef RoadBrd_CoolEye_ReadValues( GridEPtr GPtr );

#ifdef __cplusplus
}
#endif
#endif /*__ adc_H */

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
