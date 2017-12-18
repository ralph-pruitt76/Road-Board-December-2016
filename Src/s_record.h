/**
  ******************************************************************************
  * File Name          : s_record.h
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __srecord_H
#define __srecord_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"
#include <stdbool.h>
 
/* Definition s_record.h */
#define SREC_SIZE        70                     // Buffer Size for S-Records
#define BYTE_BFFR_SIZE   SREC_SIZE/2 + 10       // Enough to Store all Data.

// Enums
typedef enum 
{
  S0_HEADER = 0,
  S1_DATA = 1,
  S2_DATA = 2,
  S3_DATA = 3,
  S4_RESERVED = 4,
  S5_COUNT = 5,
  S6_COUNT = 6,
  S7_START = 7,
  S8_START = 8,
  S9_START = 9
} SrecType;

typedef enum 
{
  NO_ERROR = 0,                 // 00. No Error
  ILLEGAL_SREC = 1,             // 01. S-Record did not start with 'S'.
  ILLEGAL_RECORD = 2,           // 02. Illegal Record Type.
  RESERVED_RECORD = 3,          // 03. Record Type = 4....Reserved.
  ILLEGAL_BYTE_CNT = 4,         // 04. Illegal Byte Count.
  ILLEGAL_BYTE_DATA = 5,        // 05. Illegal Byte Data.
  BAD_CHECKSUM = 6              // 06. Checksum did not match data.
} SrecError;

// Structure
typedef struct SrecElment {
   SrecType 	RecordType;			// 16 Bit Code representing Error Code
   SrecError    Srec_Err;                       // S-Record Error Code. See Above Enum
   uint32_t     ByteCount;			// 32 Bit Byte Count
   uint32_t	Address;		        // Address of S-Record
   uint8_t      Checksum;
   uint8_t      Data[SREC_SIZE];                // Unsigned Bytes of parsed S-Record Data.
} SrecElement;
typedef struct SrecElment *SrecElmentPtr;

/* Prototypes */
HAL_StatusTypeDef Parse_srecord( char *tempBffr, SrecElmentPtr SrecPtr );
HAL_StatusTypeDef Get_Bytes( uint8_t size, char *tempBffr, uint8_t *bytePtr );
HAL_StatusTypeDef Get_Nibble( char *tempBffr, uint8_t *Result );
uint8_t Calc_ChkSum( uint8_t size, uint8_t *bytePtr );

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
