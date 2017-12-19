/**
  ******************************************************************************
  * File Name          : BootMonitor.c
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

/* Includes ------------------------------------------------------------------*/
#include "BootMonitor.h"
#include "ErrorCodes.h"
#include <ctype.h>
#include <stdlib.h>
#include "wwdg.h"
#include "Calibration.h"
#include "s_record.h"

static bool Boot_Bypass = false;

/**
  * @brief  This function tests the passed string to make sure it is Hex format.
  * @param  char *ptr: Ptr to string to be tested. NULL terminated
  * @retval int: 1: String is HEX.
  *              0:   String is not HEX.
  */
/*int isHexNum(char *ptr)
{
  int Size, x, test1;
  
  Size = strlen(ptr);
  test1 = 0;
  for (x=0; x<Size; x++)
  {
    if (((ptr[x] <= '9') && (ptr[x] >= '0')) ||
        ((ptr[x] <= 'F') && (ptr[x] >= 'A')))
      test1 = 0;
    else
      test1 = 1;
    if (test1 == 1)
      return 0;
  }
  return 1;
} */

/**
  * @brief  This function converts the passed Hex String to an Integer value.
  * @param  char *ptr: Ptr to string to be converted. NULL terminated
  * @retval int: -1: Error in String
  *              Value converted returned.
  */
/*int hatoi( char *ptr )
{
  int Size, x;
  int Value = 0;
  int FinalValue = 0;
  
  Size = strlen(ptr);
  for (x=0; x<Size; x++)
  {
    if ((ptr[x] <= '9') && (ptr[x] >= '0'))
      Value = ptr[x] - '0';
    else if((ptr[x] <= 'F') && (ptr[x] >= 'A'))
      Value = ptr[x] - 'A' + 10;
    else
      return -1;
    FinalValue = FinalValue*16 + Value;
  }
  return FinalValue;
}*/
/**
  * @brief  This routine parses the passed string and performs the passed operation
  * @param  *tempBffr: String to be parsed.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef Parse_BootString(char *tempBffr, bool BLE_Flag)
{
  #define RECEIVE_SZ      30
  uint16_t DriverStatus;
  int8_t tempBffr2[120];
  int8_t tempBffr3[10];
  int8_t s_cmd[5];
  int8_t s_recrd[120];
  HAL_StatusTypeDef Status;
  uint8_t Size;
  int Numbr_Rcrds;
  int x, y;
  Temperature TMeasure, TMeasureScaled;
  SrecElement Srec_Elem;
  uint32_t Err_code;
  
  Size = strlen((char *)tempBffr);
  Status = HAL_OK;
  
    // Test esc. If set, then exit mode.
    if (tempBffr[0] == 0x1B)
    {
      Boot_Bypass = false;
      strcpy( (char *)tempBffr2, "\r\n\r\n T........TERMINATING BOOT-MONITOR MODE.........\r\n\r\n> ");
#ifdef NUCLEO
      Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
      if (Status != HAL_OK)
        return Status;
    } //EndIf (tempBffr[0] == 0x1B)
    else
    {
      // Normal Mode
    
            // We have a good Tasking String. Time to determine action.
            switch( tempBffr[0] )
            {
//**************************************************************************************************
            case 'E':
              // Read Temp and Pressure. 
              // Is this a BLE Operation?
              if ( BLE_Flag )
              {
                // Yes...Build and Send BLE Response NOW.
                strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
              }
              
              strcpy( (char *)tempBffr2, "TBD: Read Temp and Pressure NOT IMPLEMENTED\r\n");
              break;
//**************************************************************************************************
            case 'F':
              // NO ACTION. 
              // Is this a BLE Operation?
              if ( BLE_Flag )
              {
                // Yes...Build and Send BLE Response NOW.
                strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
              }
              
              strcpy( (char *)tempBffr2, "NO Action...(0x00).\r\n");
              break;
//**************************************************************************************************
            case 'G':
              // Read Temperature sensor and return results....Temperature Sensor U10(PCT2075GVJ).  Addr: 0x94
              Status = RoadBrd_ReadTemp( &TMeasure );
              if (Status == HAL_OK)
                Status = RoadBrd_ReadTemp_Scaled( &TMeasureScaled );
              if (Status == HAL_OK)
              {
                // Send string to UART..
                strcpy( (char *)tempBffr2, "TEMP SENSOR...\r\n");
#ifdef NUCLEO
                Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                if (Status != HAL_OK)
                  return Status;
                // NOW, Build Data String..
                sprintf( (char *)tempBffr2, "     TEMP DATA: " );
                strcat( (char *)tempBffr2, (char *)TMeasure.Raw );
                strcat( (char *)tempBffr2, "\r\n" );
              }
              else
                break;
#ifdef NUCLEO
              Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
              Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
              if (Status != HAL_OK)
                return Status;
              // NOW, Build Data String..
              sprintf( (char *)tempBffr2, "     TEMP DATA(Decimal): %d\r\n", TMeasure.RawC );
#ifdef NUCLEO
              Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
              Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
              if (Status != HAL_OK)
                return Status;
              // Is this a BLE Operation?
              if ( BLE_Flag )
              {
                // Yes...Build and Send BLE Response NOW.
                sprintf( (char *)tempBffr2, "<STATUS>TEMPC:%s/%s//TEMPF:%s/%s</STATUS>", 
                        (char *)TMeasure.TempC, 
                        (char *)TMeasureScaled.TempC,
                        (char *)TMeasure.TempF, 
                        (char *)TMeasureScaled.TempF);
                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
              }
              
              // Now calculate Celcius and Farenheit Temp.
              sprintf( (char *)tempBffr2, "     TempC: %s/%s     TempF: %s/%s\r\n", 
                      (char *)TMeasure.TempC, 
                      (char *)TMeasureScaled.TempC,
                      (char *)TMeasure.TempF, 
                      (char *)TMeasureScaled.TempF);
              break;
//**************************************************************************************************
            case 'P':
              // POWER SYSTEM. 
              switch( tempBffr[1] )
              {
//++++++++++++++++++++++++++++++++++++++++++  5V Power Supply Commands.
                case 'U':
                  // Turn on 5V Power Supply.
                  RoadBrd_gpio_On( gTAM_PWR );
                  // Is this a BLE Operation?
                  if ( BLE_Flag )
                  {
                    // Yes...Build and Send BLE Response NOW.
                    strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                  }
                  
                  strcpy( (char *)tempBffr2, "5V Power Plane Powered UP.\r\n");
                  break;
                case 'D':
                  // Turn off 5V Power Supply.
                  RoadBrd_gpio_Off( gTAM_PWR );
                  strcpy( (char *)tempBffr2, "5V Power Plane Powered DOWN.\r\n");
                  break;
                default:
                  strcpy( (char *)tempBffr2, "ERROR: Illegal 5V Power Plane Command.\r\n");
                  break;
              }
              break;
//**************************************************************************************************
            case 'T':
              // TEST CMDS. 
              // Test Size to make sure we have enough Characters for this operation
              if (Size <= 1)
              {
                // Is this a BLE Operation?
                if ( BLE_Flag )
                {
                  // Yes...Build and Send BLE Response NOW.
                  strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                }
                
                strcpy( (char *)tempBffr2, "T ERROR: Not a legal command.\r\n");
              }
              else
              {
                switch( tempBffr[1] )
                {
//++++++++++++++++++++++++++++++++++++++++++  Dump Driver State.
                  case 'D':
                    // Read Driver Status
                    DriverStatus = Get_DriverStatus();
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      sprintf( (char *)tempBffr2, "<STATUS>ST_DRIVER:%04x</STATUS>", DriverStatus );
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    
                    sprintf( (char *)tempBffr2, "Driver Status: %04x\r\n", DriverStatus );
                    break;
//++++++++++++++++++++++++++++++++++++++++++  RESET Micro.
                  case 'R':
                    // RESET
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>ST_RESET_ACK</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                      HAL_Delay(100);           // Wait 100ms
                    }
                    HAL_NVIC_SystemReset();
                    sprintf( (char *)tempBffr2, "RESET CALLED BUT NO RESPONSE!!\r\n" );
                    break;
//++++++++++++++++++++++++++++++++++++++++++  Reset Flash Frame Variables(Factory).
                  case 'F':
                    // Reset Flash Frame Variables.
                    RoadBrd_WWDG_InitializeFrmFlash();
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      sprintf( (char *)tempBffr2, "<STATUS>TF_ACK</STATUS>" );
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    
                    sprintf( (char *)tempBffr2, "Flash Frame Variables Reset to Factory Values.\r\n" );
                    break;
//++++++++++++++++++++++++++++++++++++++++++  Lock Code to allow stable Program of BLE Module.
                  case 'L':
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      sprintf( (char *)tempBffr2, "<STATUS>TL_ERROR</STATUS>" );
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    else
                    {
                      sprintf( (char *)tempBffr2, "Code Locked for Programming!\r\n\r\n" );
#ifdef NUCLEO
                      Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                      if (Status != HAL_OK)
                        return Status;
                      sprintf( (char *)tempBffr2, "   HARD RESET NEEDED TO EXIT MODE\r\n" );
#ifdef NUCLEO
                      Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                      if (Status != HAL_OK)
                        return Status;
                      // Start Hard Loop
                      for (;;)
                      {
                      }
                    }
                    break;

//++++++++++++++++++++++++++++++++++++++++++  S-Record Test Monitor.
                  case 'S':
                    // S-Record Test Monitor.
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                      HAL_Delay(100);           // Wait 100ms
                    }
                    // Parse Data and extract S-Record.
                    Numbr_Rcrds = sscanf (tempBffr, "%s %s", s_cmd, s_recrd);
//                    if (sscanf (tempBffr, "%s %s", s_cmd, s_recrd) == 2)
                    if (Numbr_Rcrds == 2)
                    {
                      sprintf( (char *)tempBffr2, "TS: %s\r\n", s_recrd );
#ifdef NUCLEO
                      Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                      if (Status != HAL_OK)
                        return Status;
                      // Pass to S-Record Parser..
                      Status = Parse_srecord( (char *)s_recrd, &Srec_Elem );
                      if (Status != HAL_OK)
                      {
                        // Report Failed Parse Operation
                        switch(Srec_Elem.Srec_Err)
                        {
                          case NO_ERROR:
                            sprintf( (char *)tempBffr2, "NO ERROR FOUND.\r\n" );
                            break;
                          case ILLEGAL_SREC:
                            sprintf( (char *)tempBffr2, "ERROR: ILLEGAL S-Record.\r\n" );
                            break;
                          case ILLEGAL_RECORD:
                            sprintf( (char *)tempBffr2, "ERROR: ILLEGAL Record.\r\n" );
                            break;
                          case RESERVED_RECORD:
                            sprintf( (char *)tempBffr2, "ERROR: Reserved S-Record Type.\r\n" );
                            break;
                          case ILLEGAL_BYTE_CNT:
                            sprintf( (char *)tempBffr2, "ERROR: Illegal Byte Count.\r\n" );
                            break;
                          case ILLEGAL_BYTE_DATA:
                            sprintf( (char *)tempBffr2, "ERROR: Reserved S-Record Type.\r\n" );
                            break;
                          case BAD_CHECKSUM:
                            sprintf( (char *)tempBffr2, "ERROR: Illegal Byte Data.\r\n" );
                            break;
                          default:
                            sprintf( (char *)tempBffr2, "ERROR: Checksum did not match data.\r\n" );
                            break;
                        } // EndSwitch (Srec_Elem.Srec_Err)
                      } // EndIf (Status != HAL_OK)..Parse_srecord
                      else
                      {
                        // Display Type of S-Record
                        switch(Srec_Elem.RecordType)
                        {
                          case S0_HEADER:
                            sprintf( (char *)tempBffr2, "Record Type: S0_HEADER: Vendor specific ASCII text.\r\n" );
                            break;
                          case S1_DATA:
                            sprintf( (char *)tempBffr2, "Record Type: S1_DATA: Data that starts at the 16-bit address field.\r\n" );
                            break;
                          case S2_DATA:
                            sprintf( (char *)tempBffr2, "Record Type: S2_DATA: Data that starts at the 24-bit address field.\r\n" );
                            break;
                          case S3_DATA:
                            sprintf( (char *)tempBffr2, "Record Type: S3_DATA: Data that starts at the 32-bit address field.\r\n" );
                            break;
                          case S4_RESERVED:
                            sprintf( (char *)tempBffr2, "Record Type: S4_RESERVED\r\n" );
                            break;
                          case S5_COUNT:
                            sprintf( (char *)tempBffr2, "Record Type: S5_COUNT: 16-bit count of S1 / S2 / S3 records.\r\n" );
                            break;
                          case S6_COUNT:
                            sprintf( (char *)tempBffr2, "Record Type: S6_COUNT: 24-bit count of S1 / S2 / S3 records.\r\n" );
                            break;
                          case S7_START:
                            sprintf( (char *)tempBffr2, "Record Type: S7_START: Starting execution location at a 32-bit address.\r\n" );
                            break;
                          case S8_START:
                            sprintf( (char *)tempBffr2, "Record Type: S8_START: Starting execution location at a 24-bit address.\r\n" );
                            break;
                          case S9_START:
                            sprintf( (char *)tempBffr2, "Record Type: S9_START: Starting execution location at a 16-bit address.\r\n" );
                            break;
                          default:
                            sprintf( (char *)tempBffr2, "Record Type: UNKNOWN.\r\n" );
                            break;
                        } // EndSwitch (Srec_Elem.RecordType)
#ifdef NUCLEO
                        Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                        Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                        if (Status != HAL_OK)
                          return Status;
                        // Display Byte Count Field.
                        sprintf( (char *)tempBffr2, "Byte Count: %08x\r\n", Srec_Elem.ByteCount );
#ifdef NUCLEO
                        Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                        Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                        if (Status != HAL_OK)
                          return Status;
                        // Display Address Field.
                        sprintf( (char *)tempBffr2, "Address: %08x\r\n", Srec_Elem.Address );
#ifdef NUCLEO
                        Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                        Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                        // If S-Record Type is type 0 then display Buffer as String terminated by nulls.
                        if (Srec_Elem.RecordType == S0_HEADER)
                        {
                          sprintf( (char *)tempBffr2, "Header: %s\r\n", (char *)Srec_Elem.Data);
#ifdef NUCLEO
                          Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                          Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                        }
                        else
                        {
                          // Dump Buffer of S-Record.
                          y=0;
                          sprintf( (char *)tempBffr2, "" );
                          for (x=0; x<BYTE_BFFR_SIZE; x++)
                          {
                            sprintf( (char *)tempBffr3, "%02x ", Srec_Elem.Data[x]);
                            strcat( (char *)tempBffr2, (char *)tempBffr3 );
                            y++;
                            if (y>=16)
                            {
                              strcat( (char *)tempBffr2, "\r\n" );
                              y=0;
#ifdef NUCLEO
                              Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                              Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                              if (Status != HAL_OK)
                                return Status;
                              sprintf( (char *)tempBffr2, "" );
                            }
                          }
                        }
                        // Finally Display Checksum.
                        sprintf( (char *)tempBffr2, "Checksum: %02x\r\n", Srec_Elem.Checksum );
                      }// EndElse (Status != HAL_OK)..Parse_srecord
                    } // EndIf (Numbr_Rcrds == 2)
                    else
                    {
                      strcpy( (char *)tempBffr2, "TS SYNTAX ERROR: Too many parameters.\r\n");
                    } // EndElse (Numbr_Rcrds == 2)
                    break;
//**************************************************************************************************
                  default:
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    
                    // ERROR if we get here.. 
                    strcpy( (char *)tempBffr2, "ERROR: Not a legal command.\r\n");
                    break;
                 
                } //EndSwitch ( tempBffr[1] )
              } //EndElse (Size <= 1)
              break;
//++++++++++++++++++++++++++++++++++++++++++  Unknown Command.
            default:
              // Is this a BLE Operation?
              if ( BLE_Flag )
              {
                // Yes...Build and Send BLE Response NOW.
                strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
              }
              
              // ERROR if we get here.. 
              strcpy( (char *)tempBffr2, "ERROR: Not a legal command.\r\n");
              break;
            } // EndSwitch
            

            // Test last I2C Status to determine next msg.
            switch( Status )
            {
              case HAL_OK:
                break;
              case HAL_ERROR:
                // Determine what kind of error.
                Err_code = RoadBrd_I2C_GetError();
                switch( Err_code )
                {
                  case HAL_I2C_ERROR_BERR:
                    strcpy( (char *)tempBffr2, "I2C ERROR: BERR: Bus Error reported.\r\n");
                    break;
                  case HAL_I2C_ERROR_ARLO:
                    strcpy( (char *)tempBffr2, "I2C ERROR: ARLO: Arbitration lost reported.\r\n");
                    break;
                  case HAL_I2C_ERROR_AF:
                    strcpy( (char *)tempBffr2, "I2C ERROR: AF: Acknowledge failure reported.\r\n");
                    break;
                  case HAL_I2C_ERROR_OVR:
                    strcpy( (char *)tempBffr2, "I2C ERROR: OVR: Overrun/Underrun reported.\r\n");
                    break;
                  case HAL_I2C_ERROR_DMA:
                    strcpy( (char *)tempBffr2, "I2C ERROR: DMA: DMA Error reported.\r\n");
                    break;
                  case HAL_I2C_ERROR_TIMEOUT:
                    strcpy( (char *)tempBffr2, "I2C ERROR: TIMEOUT: Timeout Error reported.\r\n");
                    break;
                  default:
                    strcpy( (char *)tempBffr2, "I2C ERROR: Unknown Error reported.\r\n");
                    break;
                 
                }
                // Re-Initialize I2C....It has been corrupted.
                MX_I2C1_Reset();
                break;
              case HAL_BUSY:
                strcpy( (char *)tempBffr2, "I2C BUSY: I2C reported BUSY error.\r\n");
                // Re-Initialize I2C....It has been corrupted.
                MX_I2C1_Reset();
                break;
              case HAL_TIMEOUT:
                strcpy( (char *)tempBffr2, "I2C TIMEOUT: I2C reported TIMEOUT.\r\n");
                // Re-Initialize I2C....It has been corrupted.
                MX_I2C1_Reset();
                break;
              default:  
                strcpy( (char *)tempBffr2, "I2C ERROR: I2C reported unnown error.\r\n");
                // Re-Initialize I2C....It has been corrupted.
                MX_I2C1_Reset();
                break;
            }
            // Send string to UART..
#ifdef NUCLEO
            Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);
#else
            Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
           if (Status != HAL_OK)
              return Status;
            // Send Prompt to UART..
            strcpy( (char *)tempBffr2, "\r\n\r\n> ");
#ifdef NUCLEO
            Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);
#else
            Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
            if (Status != HAL_OK)
              return Status;
    } // EndElse (tempBffr[0] == 0x1B)
  return Status;
}

bool Tst_Boot_Bypass( void)
{
  return Boot_Bypass;
}

void Set_Boot_Bypass( void )
{
  Boot_Bypass =  true;
}
/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
