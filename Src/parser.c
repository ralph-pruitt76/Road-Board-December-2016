/**
  ******************************************************************************
  * File Name          : parser.c
  * Description        : This file provides code that parses the passed string and 
  *                      performs the requested operation. It then returns a string
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

/* Includes ------------------------------------------------------------------*/
#include "parser.h"
#include "ErrorCodes.h"
#include <ctype.h>
#include <stdlib.h>
#include "wwdg.h"
#include "Calibration.h"
#include "s_record.h"
#include "BootMonitor.h"

// Enums
typedef enum 
{
  NOT_INIT = 0,
  AVAILABLE = 1,
  BUSY = 2
} ParseTskFlg;

static bool Bypass = false;

// Parser Structure for tasks.
struct
{
  char          tempBuffer[BUFFER_SIZE];
  ParseTskFlg   ParseFlg;
} static ParseString;

// BGM Structure for tasks.
struct
{
  char          tempBuffer[BUFFER_SIZE];
  ParseTskFlg   ParseFlg;
} static BGMString;

/* Parser functions */

/**
  * @brief  This routine initializes the Parse Task Structure and the BGM Task Structure.
  * @param  *tempBffr: String to be parsed.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  */
HAL_StatusTypeDef RoadBrd_ParserInit( void )
{
  ParseString.ParseFlg = AVAILABLE;
  BGMString.ParseFlg = AVAILABLE;
  return HAL_OK;
}

/**
  * @brief  This routine handles the operation of setting up a BGM Event.
  * @param  *tempBffr: String to be parsed.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_BGMTsk(char *tempBffr)
{
  int x;
  
  // Test ParseFlg.
  if (BGMString.ParseFlg == BUSY)
    return HAL_BUSY;
  else if (BGMString.ParseFlg == NOT_INIT)
    return HAL_ERROR;
  // Next Lets make sure passed string is not too big.
  if (strlen(tempBffr) >= BUFFER_SIZE)
    return HAL_ERROR;
  // Clear Buffer before copying new string.
  for (x=0; x<BUFFER_SIZE; x++)
    BGMString.tempBuffer[x] = 0x00;
  // Copy String into Structure and set as busy.
  strcpy( BGMString.tempBuffer, tempBffr);
  BGMString.ParseFlg = BUSY;
  return HAL_OK;
}

/**
  * @brief  This routine handles the operation of setting up a Parse Event.
  * @param  *tempBffr: String to be parsed.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_ParserTsk(char *tempBffr)
{
  int x;
  
  // Test ParseFlg.
  if (ParseString.ParseFlg == BUSY)
    return HAL_BUSY;
  else if (ParseString.ParseFlg == NOT_INIT)
    return HAL_ERROR;
  // Next Lets make sure passed string is not too big.
  if (strlen(tempBffr) >= BUFFER_SIZE)
    return HAL_ERROR;
  // Clear Buffer before copying new string.
  for (x=0; x<BUFFER_SIZE; x++)
    ParseString.tempBuffer[x] = 0x00;
  // Copy String into Structure and set as busy.
  strcpy( ParseString.tempBuffer, tempBffr);
  ParseString.ParseFlg = BUSY;
  return HAL_OK;
}

/**
  * @brief  This routine handles the operation of processing a Parse Event.
  * @param  *tempBffr: String to be parsed.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_ProcessParserTsk( void )
{
  HAL_StatusTypeDef Status;
  uint8_t tempBffr2[80];

  // Test BGMString ParseFlg and process.
  if (BGMString.ParseFlg == BUSY)
  {
    sprintf( (char *)tempBffr2, "<SENT:%s>\r\n\r\n", BGMString.tempBuffer);

    Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);
    if (Status != HAL_OK)
      return Status;
    BGM111_Transmit((uint32_t)(strlen(BGMString.tempBuffer)), (uint8_t *)BGMString.tempBuffer);
    BGMString.ParseFlg = AVAILABLE;
  }
  
  // Test ParseFlg and process.
  if (ParseString.ParseFlg == BUSY)
  {
    // Next We need to see if OTA Parser is active...
    // Test Boot Monitor Flag...If Set, we ae in special Boot monitor mode.
    if (Tst_Boot_Bypass())
    {
      // Yes...Task to Boot Monitor.
      Status = Parse_BootString(ParseString.tempBuffer, true);
    }
    else
    {
      // Else...Normal Monitor Tasking.
      Status = RoadBrd_ParseString(ParseString.tempBuffer, true);
    }
    ParseString.ParseFlg = AVAILABLE;
    return Status;
  }
  else
    return HAL_OK;
}


/**
  * @brief  This routine parses the passed string and performs the passed operation
  * @param  *tempBffr: String to be parsed.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_ParseString(char *tempBffr, bool BLE_Flag)
{
#ifdef TEST2
  #define RECEIVE_SZ      5
  uint8_t Size;
  uint8_t i2cData[80];
  uint8_t tempBffr3[10];
  char tempBffr2[5];
  int Loop_cnt;
  HAL_StatusTypeDef Status, Save_Status;
  int Address;
  int num_bytes;
  int num_bytes_received;
  int Error, x;
  char tempstr[20];
  char* tempPstr;
  int new_value, flag;
  float Temp_C, Temp_F, Shunt_V, Bus_V, Crrnt, Power;
  uint32_t Err_code;
#else
  #ifdef TEST
    #define RECEIVE_SZ      5
  #else
    #define RECEIVE_SZ      30
    uint16_t DriverStatus;
    int8_t tempBffr2[120];
    int8_t tempBffr3[10];
    int8_t s_cmd[5];
    int8_t s_recrd[120];
    int8_t* BufferPntr;
    HAL_StatusTypeDef Status, Save_Status;
    uint8_t Size;
    int Address;
    int num_bytes;
    int Numbr_Rcrds;
    int num_bytes_received;
    uint8_t i2cData[80];
    int Error, x, y;
    Voltage VMeasure, VMeasureScaled;
    Current CMeasure, CMeasureScaled;
    Power PMeasure, PMeasureScaled;
    Temperature TMeasure, TMeasureScaled;
    Humidity HMeasure, HMeasureScaled;
    RGBInitialize RGBMeasure;
    RGBIdent IDMeasure;
    RGBStatus RGBSMeasure;
    RGBLight RGBValues;
    SrecElement Srec_Elem;
    char uuid[10];
    float Scale, Offset;
//    PRStatus PRMeasure;
    PRPressure PRPMeasure, PRPMeasureScaled;
    BinString RSFFTBins;
    GridEye     GridMeasure, GridMeasureScaled;
    uint32_t Err_code;
    uint8_t op_mode, ds_range, adc_rsl, sync, cmp_adjst, cmp_offst, int_assgn, int_persist, cnvrsn_int;
    int new_value, flag;
    char* tempPstr;
    char tempstr[20];
  #endif
#endif

    Size = strlen((char *)tempBffr);
    Status = HAL_OK;
    
    // Test Bypass. If set, then we are in streaming mode.
    if ( Bypass )
    {
      if (tempBffr[0] == 0x1B)
      {
        Bypass = false;
        strcpy( (char *)tempBffr2, "\r\n\r\n T........TERMINATING MONITOR MODE.........\r\n\r\n> ");
#ifdef NUCLEO
        Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
        Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
        if (Status != HAL_OK)
          return Status;
      }
      else
      {
        // Transmit Character to BGM111.
        BGM111_Transmit(1, (uint8_t *)tempBffr);
      }
    }// EndIf ( Bypass )
    else
    {
      // Normal Mode
    
            // We have a good Tasking String. Time to determine action.
            switch( tempBffr[0] )
            {
//**************************************************************************************************
            case '0':
              // Enable Road sound and fill buffer. 
              Status = RoadBrdSnd_ProcessSound();
              if (Status == HAL_OK)
              {
                // Is this a BLE Operation?
                if ( BLE_Flag )
                {
                  // Yes...Build and Send BLE Response NOW.
                  strcpy( (char *)tempBffr2, "<STATUS>CMD_RDSNDBFFR_FILL</STATUS>");
                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                }
                strcpy( (char *)tempBffr2, "Road Sound: Road Sound loaded, processed, and sent to FFT Bins.\r\n");
              }
              break;
//**************************************************************************************************
            case '1':
              // Rd_Sound FFT Measurements 0-15. 
              Status = RoadBrdSnd_DumpBin0( &RSFFTBins );
              strcpy( (char *)tempBffr2, "Road Sound: FFT Measurements 0-15.\r\n");
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
                sprintf( (char *)tempBffr2, "<STATUS>ST_FFT0-15:%s</STATUS>", RSFFTBins.dumpStr );
                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
              }
              // NOW, Build Data String..
              sprintf( (char *)tempBffr2, "     FFT Measurements 0-15: " );
              strcat( (char *)tempBffr2, (char *)RSFFTBins.dumpStr );
              strcat( (char *)tempBffr2, "\r\n" );
             break;
//**************************************************************************************************
            case '2':
              // Rd_Sound FFT Measurements 16-31. 
              Status = RoadBrdSnd_DumpBin16( &RSFFTBins );
              strcpy( (char *)tempBffr2, "Road Sound: FFT Measurements 16-31.\r\n");
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
                sprintf( (char *)tempBffr2, "<STATUS>ST_FFT16-31:%s</STATUS>", RSFFTBins.dumpStr );
                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
              }
              // NOW, Build Data String..
              sprintf( (char *)tempBffr2, "     FFT Measurements 16-31: " );
              strcat( (char *)tempBffr2, (char *)RSFFTBins.dumpStr );
              strcat( (char *)tempBffr2, "\r\n" );
              break;
//**************************************************************************************************
            case '3':
              // Rd_Sound FFT Measurements 32-47. 
              Status = RoadBrdSnd_DumpBin32( &RSFFTBins );
              strcpy( (char *)tempBffr2, "Road Sound: FFT Measurements 32-47.\r\n");
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
                sprintf( (char *)tempBffr2, "<STATUS>ST_FFT32-47:%s</STATUS>", RSFFTBins.dumpStr );
                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
              }
              // NOW, Build Data String..
              sprintf( (char *)tempBffr2, "     FFT Measurements 32-47: " );
              strcat( (char *)tempBffr2, (char *)RSFFTBins.dumpStr );
              strcat( (char *)tempBffr2, "\r\n" );
              break;
//**************************************************************************************************
            case '4':
              // Rd_Sound FFT Measurements 48-63. 
              Status = RoadBrdSnd_DumpBin48( &RSFFTBins );
              strcpy( (char *)tempBffr2, "Road Sound: FFT Measurements 48-63.\r\n");
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
                sprintf( (char *)tempBffr2, "<STATUS>ST_FFT48-63:%s</STATUS>", RSFFTBins.dumpStr );
                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
              }
              // NOW, Build Data String..
              sprintf( (char *)tempBffr2, "     FFT Measurements 48-63: " );
              strcat( (char *)tempBffr2, (char *)RSFFTBins.dumpStr );
              strcat( (char *)tempBffr2, "\r\n" );
             break;
//**************************************************************************************************
            case '5':
              // dump results of save Buffer. BufferPntr
              BufferPntr = RoadBrdSnd_getSaveBffr();
              strcpy( (char *)tempBffr2, "Road Sound: Results of save Buffer.\r\n");
              Status = RoadBrdSnd_DumpBin0( &RSFFTBins );
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
                strcpy( (char *)tempBffr2, "<STATUS>ST_FFTBFFR_DUMP:");
                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
              }
              // NOW, Build Data String..
              y=0;
              sprintf( (char *)tempBffr2, "" );
              for (x=0; x<FFT_BUFFER_SIZE; x++)
              {
//                sprintf( (char *)tempBffr3, "%02x ", BufferPntr[x]);
                sprintf( (char *)tempBffr3, "%04d ", BufferPntr[x]);
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
                  if ( BLE_Flag )
                  {
                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                  }
                  sprintf( (char *)tempBffr2, "" );
                }
              }
              if ( BLE_Flag )
              {
                strcpy( (char *)tempBffr2, "</STATUS>");
                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
              }
              sprintf( (char *)tempBffr2, "     ---COMPLETE---" );
              break;
//**************************************************************************************************
            case '6':
              // Clear all buffers. 
              Status = HAL_OK;
              RoadBrdSnd_ClrBffrs();
              // Is this a BLE Operation?
              if ( BLE_Flag )
              {
                // Yes...Build and Send BLE Response NOW.
                strcpy( (char *)tempBffr2, "<STATUS>ST_FFTCLR</STATUS>");
                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
              }
              strcpy( (char *)tempBffr2, "Road Sound: All buffers cleared.\r\n");;
              break;
//**************************************************************************************************
            case 'A':
              // Barometer. 
              if (Size == 1)
              {
//------------------ A Command...Read Pressure...PRESS_OUT_XL...PRESS_OUT_L...PRESS_OUT_H     
              }
              else
              {
                switch( tempBffr[1] )
                {
//------------------ AI Command...Initialize Barometer Sensor     
                  case 'I':
                    // Clean out old pressure.
//                    Status = RoadBrd_Baro_ReadPressure( &PRPMeasure );
                    // Clean out old Temp data.
//                    Status = RoadBrd_Baro_ReadTemp( i2cData );
                    //************ Wait 50msec.
//                    RoadBrd_Delay(500);  
         
                    Status = RoadBrd_Init_Barometer();
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    if (Status == HAL_OK)
                    {
                        strcpy( (char *)tempBffr2, "Initialize Barometer Sensor Passed.\r\n");;
                    }
                    else
                    {
                        strcpy( (char *)tempBffr2, "Initialize Barometer Sensor FAILED.\r\n");
                        Save_Status = Status;
#ifdef NUCLEO
                        Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                        Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                        if (Status != HAL_OK)
                          return Status;
                        else
                          Status = Save_Status;
                    }
                    break;
//------------------ A0 Command...Enable Barometer     
#if 0                  
                case '0':
                    Status = RoadBrd_Enable_Barometer();
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    if (Status == HAL_OK)
                    {
                        strcpy( (char *)tempBffr2, "Barometer Sensor: Enable Barometer Passed.\r\n");;
                    }
                    else
                    {
                        strcpy( (char *)tempBffr2, "Barometer Sensor: Enable Barometer FAILED.\r\n");;
                    }
                    break;
//------------------ A1 Command...Disable Barometer     
                  case '1':
                    Status = RoadBrd_Disable_Barometer();
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    if (Status == HAL_OK)
                    {
                        strcpy( (char *)tempBffr2, "Barometer Sensor: Disable Barometer Passed.\r\n");;
                    }
                    else
                    {
                        strcpy( (char *)tempBffr2, "Barometer Sensor: Disable Barometer FAILED.\r\n");;
                        Save_Status = Status;
#ifdef NUCLEO
                        Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                        Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                        if (Status != HAL_OK)
                          return Status;
                        else
                          Status = Save_Status;
                    }
                    break;
//------------------ A2 Command...Start the Barometer but do not wait.     
                  case '2':
                    Status = RoadBrd_StartSample_Barometer();
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    if (Status == HAL_OK)
                    {
                        strcpy( (char *)tempBffr2, "Start the Barometer but do not wait Passed.\r\n");;
                    }
                    else
                    {
                        strcpy( (char *)tempBffr2, "Start the Barometer but do not wait FAILED.\r\n");;
                        Save_Status = Status;
#ifdef NUCLEO
                        Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                        Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                        if (Status != HAL_OK)
                          return Status;
                        else
                          Status = Save_Status;
                    }
                    break;
//------------------ A3 Command...Start the Barometer and wait for response.    
                  case '3':
                    Status = RoadBrd_StartSample_BarometerWait();
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    if (Status == HAL_OK)
                    {
                        strcpy( (char *)tempBffr2, "Start the Barometer and wait for response Passed.\r\n");;
                    }
                    else
                    {
                        strcpy( (char *)tempBffr2, "Start the Barometer and wait for response FAILED.\r\n");;
                        Save_Status = Status;
#ifdef NUCLEO
                        Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                        Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                        if (Status != HAL_OK)
                          return Status;
                        else
                          Status = Save_Status;
                    }
                    break;
//------------------ A4 Command...Return Status    
                  case '4':
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    Status = RoadBrd_Barometer_Status( &PRMeasure );
                    if (Status == HAL_OK)
                    {
                      sprintf( (char *)tempBffr2, "Barometer Sensor Status: %02x / ", PRMeasure.Status );
                      strcat( (char *)tempBffr2, (char *)PRMeasure.Raw );
                      strcat( (char *)tempBffr2, "\r\n" );
                    }
                    break;
//------------------ A5 Command...Wait for Pressure to be complete. 
                  case '5':
                    Status = RoadBrd_WaitForPressure( (uint16_t)BARO_WAITCNT );
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    if (Status == HAL_OK)
                    {
                        strcpy( (char *)tempBffr2, "Wait for Pressure to be complete Passed.\r\n");;
                    }
                    else
                    {
                        strcpy( (char *)tempBffr2, "Wait for Pressure to be complete FAILED.\r\n");;
                        Save_Status = Status;
#ifdef NUCLEO
                        Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                        Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                        if (Status != HAL_OK)
                          return Status;
                        else
                          Status = Save_Status;
                    }
                    break;
#endif
//------------------ A6 Command...Read Pressure...PRESS_OUT_XL...PRESS_OUT_L...PRESS_OUT_H     
                  case '6':
                    //Status = RoadBrd_Barometer_Status( &PRMeasure );
                    Status = RoadBrd_Baro_ReadPressure( &PRPMeasure );
                    if (Status == HAL_OK)
                      Status = RoadBrd_Baro_ReadPressure_Scaled( &PRPMeasureScaled );
                    if (Status == HAL_OK)
                    {
                      strcpy( (char *)tempBffr2, "A6 Command...Read Pressure Passed.\r\n");;
#ifdef NUCLEO
                      Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                      if (Status != HAL_OK)
                        return Status;
                      // NOW, Build Data String..
                      sprintf( (char *)tempBffr2, " PRESSURE(PRESS_OUT_H...PRESS_OUT_L...PRESS_OUT_XL)\r\n" );
#ifdef NUCLEO
                      Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                      if (Status != HAL_OK)
                        return Status;
                     
                      // Now show Decimal value of items.
                      sprintf( (char *)tempBffr2, " PRESSURE: %s/%s\r\n", (char *)PRPMeasure.Pressure, (char *)PRPMeasureScaled.Pressure );
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
                        sprintf( (char *)tempBffr2, "<STATUS>PRESSURE: %s/%s</STATUS>", (char *)PRPMeasure.Pressure, (char *)PRPMeasureScaled.Pressure );
                        BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                      }
                      // Now show hex value of items.
                      sprintf( (char *)tempBffr2, " PRESSURE:  " );
                      strcat( (char *)tempBffr2, (char *)PRPMeasure.Raw );
                      sprintf( (char *)tempBffr3, "  -  %08x\r\n", PRPMeasure.RawC);
                      strcat( (char *)tempBffr2, (char *)tempBffr3 );
                    }
                    else
                    {
                        strcpy( (char *)tempBffr2, "A6 Command...Read Pressure FAILED.\r\n");;
                        Save_Status = Status;
#ifdef NUCLEO
                        Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                        Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                        if (Status != HAL_OK)
                          return Status;
                        else
                          Status = Save_Status;
                    }
                    break;

//------------------ A8 Command...Read Temperature....TEMP_OUT_L...TEMP_OUT_H    
                  case '8':
                    Status = RoadBrd_Baro_ReadTemp( &TMeasure );
                    if (Status == HAL_OK)
                    {
                      // Send string to UART..
                      strcpy( (char *)tempBffr2, "A8 Command...Read Temperature Passed.\r\n");
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
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    // Now calculate Celcius and Farenheit Temp.
                    sprintf( (char *)tempBffr2, "     TempC: " );
                    strcat( (char *)tempBffr2, (char *)TMeasure.TempC );
                    strcat( (char *)tempBffr2, "     TempF: " );
                    strcat( (char *)tempBffr2, (char *)TMeasure.TempF );
                    strcat( (char *)tempBffr2, "\r\n" );
/*                    if (Status == HAL_OK)
                    {
                      strcpy( (char *)tempBffr2, "A8 Command...Read Temperature Passed.\r\n");;
#ifdef NUCLEO
                      Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                      if (Status != HAL_OK)
                        return Status;
                      // NOW, Build Data String..
                      sprintf( (char *)tempBffr2, " Temperature(TEMP_OUT_L...TEMP_OUT_H)\r\n" );
#ifdef NUCLEO
                      Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                      if (Status != HAL_OK)
                        return Status;
                      sprintf( (char *)tempBffr2, " Temperature:  %02x   %02x\r\n", i2cData[0], i2cData[1] );
                    }
                    else
                    {
                        strcpy( (char *)tempBffr2, "A8 Command...Read Temperature FAILED.\r\n");;
                        Save_Status = Status;
#ifdef NUCLEO
                        Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                        Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                        if (Status != HAL_OK)
                          return Status;
                        else
                          Status = Save_Status;
                    }*/
                    break;
//------------------ A9 Command...Test and Verify WHO_AM_I     
                  case '9':
                    Status = RoadBrd_TestandRead_Barometer();
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    if (Status == HAL_OK)
                    {
                        strcpy( (char *)tempBffr2, "Barometer Sensor: WHO_AM_I Passed.\r\n");
                    }
                    else
                    {
                        strcpy( (char *)tempBffr2, "Barometer Sensor: WHO_AM_I FAILED.\r\n");
                        Status = HAL_OK;
                    }
                    break;
                  default:
                    strcpy( (char *)tempBffr2, "ERROR: Not a legal command.\r\n");
                    break;
                } //EndSwitch
              } //EndElse (Size == 1)
              break;
//**************************************************************************************************
            case 'B':
              // Read Cool Eye. 
//++++++++++++++++++++++++++++++++++++++++++  Cool Eye/Grid Eye Monitor Commands.
              if (Size == 1)
              {
//------------------ B Command: Read Cool Eye/Grid Eye Values      
                // Read Cool Eye/Grid Eye Values.....
                if ( Get_DriverStates( GRIDEYE_MNTR_TASK ))
                {
                  Status = RoadBrd_GridEye_ReadValues( &GridMeasure );
                  if (Status == HAL_OK)
                    Status = RoadBrd_GridEye_ReadValues_Scaled( &GridMeasureScaled );
                }
                else if ( Get_DriverStates( COOLEYE_MNTR_TASK ))
                {
                  Status = RoadBrd_CoolEye_ReadValues( &GridMeasure );
                  if (Status == HAL_OK)
                    Status = RoadBrd_CoolEye_ReadValues_Scaled( &GridMeasureScaled );
                }
                else
                  Status = HAL_ERROR;
                
                // Is this a BLE Operation?
                if ( BLE_Flag )
                {
                  // Yes...Build and Send BLE Response NOW.
                  strcpy( (char *)tempBffr2, "<STATUS>ST_THERMAL_DUMP:");
                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                }
                if (Status == HAL_OK)
                {
                  // Send string to UART..
                  strcpy( (char *)tempBffr2, "Voltage Monitor/Shunt Voltage...\r\n");
#ifdef NUCLEO
                  Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                  Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                  if (Status != HAL_OK)
                    return Status;
                  // NOW, Build Data String..
                  for (x=0; x<9; x++)
                  {
                    // Build String
                    switch(x)
                    {
                      case 0: //Thermistor Values
                        sprintf( (char *)tempBffr2, "     Thermistor DATA: %s   %s/%s   %s/%s   %d\r\n", GridMeasure.Thermistor.Raw,
                                                                                               GridMeasure.Thermistor.TempC,GridMeasureScaled.Thermistor.TempC,
                                                                                               GridMeasure.Thermistor.TempF,GridMeasureScaled.Thermistor.TempF,
                                                                                               GridMeasure.Thermistor.RawC );
//                        sprintf( (char *)tempBffr2, "     Thermistor DATA: %s   %s   %s   %d\r\n", GridMeasure.Thermistor.Raw,
//                                                                                               GridMeasure.Thermistor.TempC,
//                                                                                               GridMeasure.Thermistor.TempF,
//                                                                                               GridMeasure.Thermistor.RawC );
                        break;
                      case 1: //GridEye1 Values
                        sprintf( (char *)tempBffr2, "     Grid 33 DATA: %s   %s/%s   %s/%s   %d\r\n", GridMeasure.GridEye1.Raw,
                                                                                               GridMeasure.GridEye1.TempC,GridMeasureScaled.GridEye1.TempC,
                                                                                               GridMeasure.GridEye1.TempF,GridMeasureScaled.GridEye1.TempF,
                                                                                               GridMeasure.GridEye1.RawC );
//                        sprintf( (char *)tempBffr2, "     Grid 33 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye1.Raw,
//                                                                                               GridMeasure.GridEye1.TempC,
//                                                                                               GridMeasure.GridEye1.TempF,
//                                                                                               GridMeasure.GridEye1.RawC );
                        break;
                      case 2: //GridEye2 Values
                        sprintf( (char *)tempBffr2, "     Grid 33 DATA: %s   %s/%s   %s/%s   %d\r\n", GridMeasure.GridEye2.Raw,
                                                                                               GridMeasure.GridEye2.TempC,GridMeasureScaled.GridEye2.TempC,
                                                                                               GridMeasure.GridEye2.TempF,GridMeasureScaled.GridEye2.TempF,
                                                                                               GridMeasure.GridEye2.RawC );
//                        sprintf( (char *)tempBffr2, "     Grid 34 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye2.Raw,
//                                                                                               GridMeasure.GridEye2.TempC,
//                                                                                               GridMeasure.GridEye2.TempF,
//                                                                                               GridMeasure.GridEye2.RawC );
                        break;
                      case 3: //GridEye3 Values
                        sprintf( (char *)tempBffr2, "     Grid 33 DATA: %s   %s/%s   %s/%s   %d\r\n", GridMeasure.GridEye3.Raw,
                                                                                               GridMeasure.GridEye3.TempC,GridMeasureScaled.GridEye3.TempC,
                                                                                               GridMeasure.GridEye3.TempF,GridMeasureScaled.GridEye3.TempF,
                                                                                               GridMeasure.GridEye3.RawC );
//                        sprintf( (char *)tempBffr2, "     Grid 35 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye3.Raw,
//                                                                                               GridMeasure.GridEye3.TempC,
//                                                                                               GridMeasure.GridEye3.TempF,
//                                                                                               GridMeasure.GridEye3.RawC );
                        break;
                      case 4: //GridEye4 Values
                        sprintf( (char *)tempBffr2, "     Grid 33 DATA: %s   %s/%s   %s/%s   %d\r\n", GridMeasure.GridEye4.Raw,
                                                                                               GridMeasure.GridEye4.TempC,GridMeasureScaled.GridEye4.TempC,
                                                                                               GridMeasure.GridEye4.TempF,GridMeasureScaled.GridEye4.TempF,
                                                                                               GridMeasure.GridEye4.RawC );
//                        sprintf( (char *)tempBffr2, "     Grid 36 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye4.Raw,
//                                                                                               GridMeasure.GridEye4.TempC,
//                                                                                               GridMeasure.GridEye4.TempF,
//                                                                                               GridMeasure.GridEye4.RawC );
                        break;
                      case 5: //GridEye5 Values
                        sprintf( (char *)tempBffr2, "     Grid 33 DATA: %s   %s/%s   %s/%s   %d\r\n", GridMeasure.GridEye5.Raw,
                                                                                               GridMeasure.GridEye5.TempC,GridMeasureScaled.GridEye5.TempC,
                                                                                               GridMeasure.GridEye5.TempF,GridMeasureScaled.GridEye5.TempF,
                                                                                               GridMeasure.GridEye5.RawC );
//                        sprintf( (char *)tempBffr2, "     Grid 37 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye5.Raw,
//                                                                                               GridMeasure.GridEye5.TempC,
//                                                                                               GridMeasure.GridEye5.TempF,
//                                                                                               GridMeasure.GridEye5.RawC );
                        break;
                      case 6: //GridEye6 Values
                        sprintf( (char *)tempBffr2, "     Grid 33 DATA: %s   %s/%s   %s/%s   %d\r\n", GridMeasure.GridEye6.Raw,
                                                                                               GridMeasure.GridEye6.TempC,GridMeasureScaled.GridEye6.TempC,
                                                                                               GridMeasure.GridEye6.TempF,GridMeasureScaled.GridEye6.TempF,
                                                                                               GridMeasure.GridEye6.RawC );
//                        sprintf( (char *)tempBffr2, "     Grid 38 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye6.Raw,
//                                                                                               GridMeasure.GridEye6.TempC,
//                                                                                               GridMeasure.GridEye6.TempF,
//                                                                                               GridMeasure.GridEye6.RawC );
                        break;
                      case 7: //GridEye7 Values
                        sprintf( (char *)tempBffr2, "     Grid 33 DATA: %s   %s/%s   %s/%s   %d\r\n", GridMeasure.GridEye7.Raw,
                                                                                               GridMeasure.GridEye7.TempC,GridMeasureScaled.GridEye7.TempC,
                                                                                               GridMeasure.GridEye7.TempF,GridMeasureScaled.GridEye7.TempF,
                                                                                               GridMeasure.GridEye7.RawC );
//                        sprintf( (char *)tempBffr2, "     Grid 39 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye7.Raw,
//                                                                                               GridMeasure.GridEye7.TempC,
//                                                                                               GridMeasure.GridEye7.TempF,
//                                                                                               GridMeasure.GridEye7.RawC );
                        break;
                      case 8: //GridEye8 Values
                        sprintf( (char *)tempBffr2, "     Grid 33 DATA: %s   %s/%s   %s/%s   %d\r\n", GridMeasure.GridEye8.Raw,
                                                                                               GridMeasure.GridEye8.TempC,GridMeasureScaled.GridEye8.TempC,
                                                                                               GridMeasure.GridEye8.TempF,GridMeasureScaled.GridEye8.TempF,
                                                                                               GridMeasure.GridEye8.RawC );
//                        sprintf( (char *)tempBffr2, "     Grid 40 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye8.Raw,
//                                                                                               GridMeasure.GridEye8.TempC,
//                                                                                               GridMeasure.GridEye8.TempF,
//                                                                                               GridMeasure.GridEye8.RawC );
                        break;
                    } // EndSwitch(x)
                    if ( BLE_Flag )
                    {
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    // Now Print String.
 #ifdef NUCLEO
                    Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                    Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                    if (Status != HAL_OK)
                      return Status;
                  } // EndFor(x=0; x<9; x++)
                  sprintf( (char *)tempBffr2, "     SHNT VLTG DATA: " );
                  strcat( (char *)tempBffr2, (char *)VMeasure.Raw );
                  strcat( (char *)tempBffr2, "\r\n" );
                } //Endif(Status == HAL_OK)
                else
                  break;
                if ( BLE_Flag )
                {
                  strcpy( (char *)tempBffr2, "</STATUS>");
                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                }
                sprintf( (char *)tempBffr2, "     COMPLETE.\r\n" );
              }
              else
              {
                switch( tempBffr[1] )
                {
//------------------ BI Command: Initialize Cool Eye/Grid Eye Sensor
                  case 'I':
                    // Initialize Cool Eye/Grid Eye Sensor.
                    Status = RoadBrd_GridEyeInit();
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    if (Status == HAL_OK)
                    {
                      strcpy( (char *)tempBffr2, "Cool Eye/Grid Eye Sensor: Initialization Complete.\r\n");;
                    }
                    break;
//------------------ B0 Command...Read Cool Eye/Grid Eye Values.....
                  case '0':
                    // Read Cool Eye/Grid Eye Values.....
                    if ( Get_DriverStates( GRIDEYE_MNTR_TASK ))
                    {
                      Status = RoadBrd_GridEye_ReadValues( &GridMeasure );
                      if (Status == HAL_OK)
                        Status = RoadBrd_GridEye_ReadValues_Scaled( &GridMeasureScaled );
                    }
                    else if ( Get_DriverStates( COOLEYE_MNTR_TASK ))
                    {
                      Status = RoadBrd_CoolEye_ReadValues( &GridMeasure );
                      if (Status == HAL_OK)
                        Status = RoadBrd_CoolEye_ReadValues_Scaled( &GridMeasureScaled );
                    }
                    else
                      Status = HAL_ERROR;

                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }

                    if (Status == HAL_OK)
                    {
                      // Send string to UART..
                      strcpy( (char *)tempBffr2, "Voltage Monitor/Shunt Voltage...\r\n");
#ifdef NUCLEO
                      Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                      if (Status != HAL_OK)
                        return Status;
                      // NOW, Build Data String..
                      for (x=0; x<9; x++)
                      {
                        // Build String
                        switch(x)
                        {
                        case 0: //Thermistor Values
                          sprintf( (char *)tempBffr2, "     Thermistor DATA: %s   %s/%s   %s/%s   %d\r\n", GridMeasure.Thermistor.Raw,
                                  GridMeasure.Thermistor.TempC,GridMeasureScaled.Thermistor.TempC,
                                  GridMeasure.Thermistor.TempF,GridMeasureScaled.Thermistor.TempF,
                                  GridMeasure.Thermistor.RawC );
                          //                        sprintf( (char *)tempBffr2, "     Thermistor DATA: %s   %s   %s   %d\r\n", GridMeasure.Thermistor.Raw,
                          //                                                                                               GridMeasure.Thermistor.TempC,
                          //                                                                                               GridMeasure.Thermistor.TempF,
                          //                                                                                               GridMeasure.Thermistor.RawC );
                          break;
                        case 1: //GridEye1 Values
                          sprintf( (char *)tempBffr2, "     Grid 33 DATA: %s   %s/%s   %s/%s   %d\r\n", GridMeasure.GridEye1.Raw,
                                  GridMeasure.GridEye1.TempC,GridMeasureScaled.GridEye1.TempC,
                                  GridMeasure.GridEye1.TempF,GridMeasureScaled.GridEye1.TempF,
                                  GridMeasure.GridEye1.RawC );
                          //                        sprintf( (char *)tempBffr2, "     Grid 33 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye1.Raw,
                          //                                                                                               GridMeasure.GridEye1.TempC,
                          //                                                                                               GridMeasure.GridEye1.TempF,
                          //                                                                                               GridMeasure.GridEye1.RawC );
                          break;
                        case 2: //GridEye2 Values
                          sprintf( (char *)tempBffr2, "     Grid 33 DATA: %s   %s/%s   %s/%s   %d\r\n", GridMeasure.GridEye2.Raw,
                                  GridMeasure.GridEye2.TempC,GridMeasureScaled.GridEye2.TempC,
                                  GridMeasure.GridEye2.TempF,GridMeasureScaled.GridEye2.TempF,
                                  GridMeasure.GridEye2.RawC );
                          //                        sprintf( (char *)tempBffr2, "     Grid 34 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye2.Raw,
                          //                                                                                               GridMeasure.GridEye2.TempC,
                          //                                                                                               GridMeasure.GridEye2.TempF,
                          //                                                                                               GridMeasure.GridEye2.RawC );
                          break;
                        case 3: //GridEye3 Values
                          sprintf( (char *)tempBffr2, "     Grid 33 DATA: %s   %s/%s   %s/%s   %d\r\n", GridMeasure.GridEye3.Raw,
                                  GridMeasure.GridEye3.TempC,GridMeasureScaled.GridEye3.TempC,
                                  GridMeasure.GridEye3.TempF,GridMeasureScaled.GridEye3.TempF,
                                  GridMeasure.GridEye3.RawC );
                          //                        sprintf( (char *)tempBffr2, "     Grid 35 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye3.Raw,
                          //                                                                                               GridMeasure.GridEye3.TempC,
                          //                                                                                               GridMeasure.GridEye3.TempF,
                          //                                                                                               GridMeasure.GridEye3.RawC );
                          break;
                        case 4: //GridEye4 Values
                          sprintf( (char *)tempBffr2, "     Grid 33 DATA: %s   %s/%s   %s/%s   %d\r\n", GridMeasure.GridEye4.Raw,
                                  GridMeasure.GridEye4.TempC,GridMeasureScaled.GridEye4.TempC,
                                  GridMeasure.GridEye4.TempF,GridMeasureScaled.GridEye4.TempF,
                                  GridMeasure.GridEye4.RawC );
                          //                        sprintf( (char *)tempBffr2, "     Grid 36 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye4.Raw,
                          //                                                                                               GridMeasure.GridEye4.TempC,
                          //                                                                                               GridMeasure.GridEye4.TempF,
                          //                                                                                               GridMeasure.GridEye4.RawC );
                          break;
                        case 5: //GridEye5 Values
                          sprintf( (char *)tempBffr2, "     Grid 33 DATA: %s   %s/%s   %s/%s   %d\r\n", GridMeasure.GridEye5.Raw,
                                  GridMeasure.GridEye5.TempC,GridMeasureScaled.GridEye5.TempC,
                                  GridMeasure.GridEye5.TempF,GridMeasureScaled.GridEye5.TempF,
                                  GridMeasure.GridEye5.RawC );
                          //                        sprintf( (char *)tempBffr2, "     Grid 37 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye5.Raw,
                          //                                                                                               GridMeasure.GridEye5.TempC,
                          //                                                                                               GridMeasure.GridEye5.TempF,
                          //                                                                                               GridMeasure.GridEye5.RawC );
                          break;
                        case 6: //GridEye6 Values
                          sprintf( (char *)tempBffr2, "     Grid 33 DATA: %s   %s/%s   %s/%s   %d\r\n", GridMeasure.GridEye6.Raw,
                                  GridMeasure.GridEye6.TempC,GridMeasureScaled.GridEye6.TempC,
                                  GridMeasure.GridEye6.TempF,GridMeasureScaled.GridEye6.TempF,
                                  GridMeasure.GridEye6.RawC );
                          //                        sprintf( (char *)tempBffr2, "     Grid 38 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye6.Raw,
                          //                                                                                               GridMeasure.GridEye6.TempC,
                          //                                                                                               GridMeasure.GridEye6.TempF,
                          //                                                                                               GridMeasure.GridEye6.RawC );
                          break;
                        case 7: //GridEye7 Values
                          sprintf( (char *)tempBffr2, "     Grid 33 DATA: %s   %s/%s   %s/%s   %d\r\n", GridMeasure.GridEye7.Raw,
                                  GridMeasure.GridEye7.TempC,GridMeasureScaled.GridEye7.TempC,
                                  GridMeasure.GridEye7.TempF,GridMeasureScaled.GridEye7.TempF,
                                  GridMeasure.GridEye7.RawC );
                          //                        sprintf( (char *)tempBffr2, "     Grid 39 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye7.Raw,
                          //                                                                                               GridMeasure.GridEye7.TempC,
                          //                                                                                               GridMeasure.GridEye7.TempF,
                          //                                                                                               GridMeasure.GridEye7.RawC );
                          break;
                        case 8: //GridEye8 Values
                          sprintf( (char *)tempBffr2, "     Grid 33 DATA: %s   %s/%s   %s/%s   %d\r\n", GridMeasure.GridEye8.Raw,
                                  GridMeasure.GridEye8.TempC,GridMeasureScaled.GridEye8.TempC,
                                  GridMeasure.GridEye8.TempF,GridMeasureScaled.GridEye8.TempF,
                                  GridMeasure.GridEye8.RawC );
                          //                        sprintf( (char *)tempBffr2, "     Grid 40 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye8.Raw,
                          //                                                                                               GridMeasure.GridEye8.TempC,
                          //                                                                                               GridMeasure.GridEye8.TempF,
                          //                                                                                               GridMeasure.GridEye8.RawC );
                          break;
                        } // EndSwitch(x)
                        // Now Print String.
 #ifdef NUCLEO
                        Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                        Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                        if (Status != HAL_OK)
                          return Status;
                     } // EndFor(x=0; x<9; x++)
                      sprintf( (char *)tempBffr2, "     SHNT VLTG DATA: " );
                      strcat( (char *)tempBffr2, (char *)VMeasure.Raw );
                      strcat( (char *)tempBffr2, "\r\n" );
                    } //Endif(Status == HAL_OK)
                    else
                      break;
                    sprintf( (char *)tempBffr2, "     COMPLETE.\r\n" );
                    break;
//------------------ B1 Command...Reset Cool Eye/Grid Eye Sensor..... 
                  case '1':
                    // Reset Cool Eye/Grid Eye Sensor.
                    Status = RoadBrd_GridEyeReset();
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }

                    if (Status == HAL_OK)
                    {
                      strcpy( (char *)tempBffr2, "Cool Eye/Grid Eye Sensor: Reset Complete.\r\n");;
                    }
                    break;
                } //EndSwitch
              } //EndElse (Size == 1)
              break;
//**************************************************************************************************
            case 'C':
              // Read Voltage. 
//++++++++++++++++++++++++++++++++++++++++++  Voltage Monitor Commands.
              if (Size == 1)
              {
//------------------ C Command       
                // Read Bus Voltage and return as 2 Byte Field.
                Status = RoadBrd_VMonitor_RdVoltage( &VMeasure );
                if (Status == HAL_OK)
                  Status = RoadBrd_VMonitor_RdVoltage_Scaled( &VMeasureScaled );
                if (Status == HAL_OK)
                {
                  // Send string to UART..
                  strcpy( (char *)tempBffr2, "Voltage Monitor/Read Bus Voltage...\r\n");
#ifdef NUCLEO
                  Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                  Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                  if (Status != HAL_OK)
                  return Status;
                  // NOW, Build Data String..
                  sprintf( (char *)tempBffr2, "     BUS VOLTAGE DATA: " );
                  strcat( (char *)tempBffr2, (char *)VMeasure.Raw );
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
                // Is this a BLE Operation?
                if ( BLE_Flag )
                {
                  // Yes...Build and Send BLE Response NOW.
                  sprintf( (char *)tempBffr2, "<STATUS>BUS_VLTG:%s/%s</STATUS>", (char *)VMeasure.Voltage, (char *)VMeasureScaled.Voltage );
                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                }
                // Now calculate Bus Voltage.
                sprintf( (char *)tempBffr2, "     Bus Voltage: %s/%s\r\n", (char *)VMeasure.Voltage, (char *)VMeasureScaled.Voltage );
              }
              else
              {
                switch( tempBffr[1] )
                {
//------------------ CI Command       
                  case 'I':
                    // Initialize and load Calibration Register.
                    // Read Calibration Value first.
                    Status = RoadBrd_Init_VMonitor();
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    if (Status == HAL_OK)
                    {
                      strcpy( (char *)tempBffr2, "Voltage Monitor: Initialization Complete.\r\n");;
                    }
                    break;
//------------------ C0 Command...Read Shunt Voltage and return results.....
                  case '0':
                    // Read Shunt Voltage and return results.....
                    Status = RoadBrd_VMonitor_RdShntVltg( &VMeasure );
                    if (Status == HAL_OK)
                      Status = RoadBrd_VMonitor_RdShntVltg_Scaled( &VMeasureScaled );
                    // Is this a BLE Operation?
                    if (Status == HAL_OK)
                    {
                      // Send string to UART..
                      strcpy( (char *)tempBffr2, "Voltage Monitor/Shunt Voltage...\r\n");
#ifdef NUCLEO
                      Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                      if (Status != HAL_OK)
                        return Status;
                      // NOW, Build Data String..
                      sprintf( (char *)tempBffr2, "     SHNT VLTG DATA: " );
                      strcat( (char *)tempBffr2, (char *)VMeasure.Raw );
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
                    // Now calculate Shunt Voltage.
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      sprintf( (char *)tempBffr2, "<STATUS>SHNT_VLTG:%s/%s</STATUS>", (char *)VMeasure.Voltage, (char *)VMeasureScaled.Voltage );
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    sprintf( (char *)tempBffr2, "     Shunt Voltage: %s/%s\r\n", (char *)VMeasure.Voltage, (char *)VMeasureScaled.Voltage );
                    break;
//------------------ C1 Command...Read Current and return results..... 
                  case '1':
                    // Read Current and return results.....
                    Status = RoadBrd_VMonitor_RdCurrent( &CMeasure );
                    if (Status == HAL_OK)
                      Status = RoadBrd_VMonitor_RdCurrent_Scaled( &CMeasureScaled );
                    // Is this a BLE Operation?
                    if (Status == HAL_OK)
                    {
                      // Send string to UART..
                      strcpy( (char *)tempBffr2, "Voltage Monitor/Read Current...\r\n");
#ifdef NUCLEO
                      Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                      if (Status != HAL_OK)
                        return Status;
                      // NOW, Build Data String..
                      sprintf( (char *)tempBffr2, "     CURRENT DATA: " );
                      strcat( (char *)tempBffr2, (char *)CMeasure.Raw );
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
                    // Now calculate Current.
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      sprintf( (char *)tempBffr2, "<STATUS>SHNT_CRNT:%s/%s</STATUS>", (char *)CMeasure.Current, (char *)CMeasureScaled.Current );
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    sprintf( (char *)tempBffr2, "     Current: %s/%s\r\n", (char *)CMeasure.Current, (char *)CMeasureScaled.Current );
                    break;
//------------------ C2 Command...Read Power and return results.....     
                  case '2':
                    // Read Power and return results.....
                    Status = RoadBrd_VMonitor_RdPower( &PMeasure );
                    if (Status == HAL_OK)
                      Status = RoadBrd_VMonitor_RdPower_Scaled( &PMeasureScaled );
                    // Is this a BLE Operation?
                    if (Status == HAL_OK)
                    {
                      // Send string to UART..
                      strcpy( (char *)tempBffr2, "Voltage Monitor/Read Power...\r\n");
#ifdef NUCLEO
                      Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                      if (Status != HAL_OK)
                        return Status;
                      // NOW, Build Data String..
                      sprintf( (char *)tempBffr2, "     POWER DATA: " );
                      strcat( (char *)tempBffr2, (char *)PMeasure.Raw );
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
                    // Now calculate Power.
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      sprintf( (char *)tempBffr2, "<STATUS>POWER:%s/%s</STATUS>", (char *)PMeasure.Power, (char *)PMeasureScaled.Power );
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    sprintf( (char *)tempBffr2, "     Power: %s/%s\r\n", (char *)PMeasure.Power, (char *)PMeasureScaled.Power );
                    break;
//------------------ C3 Command...Read Bus Voltage and return results.....
                  case '3':
                    // Read Bus Voltage and return results.....
                    Status = RoadBrd_VMonitor_RdVoltage( &VMeasure );
                    if (Status == HAL_OK)
                      Status = RoadBrd_VMonitor_RdVoltage_Scaled( &VMeasureScaled );
                      
                    if (Status == HAL_OK)
                    {
                      // Send string to UART..
                      strcpy( (char *)tempBffr2, "Voltage Monitor/Read Bus Voltage...\r\n");
#ifdef NUCLEO
                      Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                      if (Status != HAL_OK)
                        return Status;
                      // NOW, Build Data String..
                      sprintf( (char *)tempBffr2, "     BUS VOLTAGE DATA: " );
                      strcat( (char *)tempBffr2, (char *)VMeasure.Raw );
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
                    // Now calculate Bus Voltage.
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      sprintf( (char *)tempBffr2, "<STATUS>BUS_VLTG:%s/%s</STATUS>", (char *)VMeasure.Voltage, (char *)VMeasureScaled.Voltage );
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    sprintf( (char *)tempBffr2, "     Bus Voltage: %s/%s\r\n", (char *)VMeasure.Voltage, (char *)VMeasureScaled.Voltage );
                    break;
                  default:
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_C_SYNTAX</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    strcpy( (char *)tempBffr2, "ERROR: Not a legal command.\r\n");
                    break;
                } //EndSwitch
              } //EndElse (Size == 1)
              break;
//**************************************************************************************************
            case 'D':
              // Read Humidity. 
              if (Size == 1)
              {
//------------------ D Command: Read Humidity Values      
                // Read Humidity Sensor sensor and return Humidity results....
                Status = RoadBrd_Humidity_ReadHumidity( &HMeasure );
                if (Status == HAL_OK)
                  Status = RoadBrd_Humidity_ReadHumidity_Scaled( &HMeasureScaled );
                // Is this a BLE Operation?
                if (Status == HAL_OK)
                {
                  // Send string to UART..
                  strcpy( (char *)tempBffr2, "Humidity SENSOR...\r\n");
#ifdef NUCLEO
                  Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                  Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                  if (Status != HAL_OK)
                    return Status;
                  // NOW, Build Data String..
                  sprintf( (char *)tempBffr2, "     Humidity DATA: " );
                  strcat( (char *)tempBffr2, (char *)HMeasure.HRaw );
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
                sprintf( (char *)tempBffr2, "     Humidity DATA(Decimal): %d\r\n", HMeasure.HRawC );
#ifdef NUCLEO
                Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                if (Status != HAL_OK)
                  return Status;
                // Now calculate Celcius and Farenheit Temp.
                if ( BLE_Flag )
                {
                  // Yes...Build and Send BLE Response NOW.
                  sprintf( (char *)tempBffr2, "<STATUS>HUMIDITY:%s/%s</STATUS>", (char *)HMeasure.Humidity, (char *)HMeasureScaled.Humidity );
                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                }
                sprintf( (char *)tempBffr2, "     Humidity: %s/%s\r\n", (char *)HMeasure.Humidity, (char *)HMeasureScaled.Humidity );
              }
              else
              {
                switch( tempBffr[1] )
                {
//------------------ DI Command: Initialize Humidity Sensor
                  case 'I':
                    // Initialize Humidity Sensor.
                    Status = RoadBrd_HumidityInit();
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }

                    if (Status == HAL_OK)
                    {
                      strcpy( (char *)tempBffr2, "Humidity Sensor: Initialization Complete.\r\n");;
                    }
                    break;
//------------------ D0 Command...Read Humidity Values.....
                  case '0':
                    // Read Humidity Sensor sensor and return Humidity results....
                    Status = RoadBrd_Humidity_ReadHumidity( &HMeasure );
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }

                    if (Status == HAL_OK)
                    {
                      // Send string to UART..
                      strcpy( (char *)tempBffr2, "Humidity SENSOR...\r\n");
#ifdef NUCLEO
                      Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                      if (Status != HAL_OK)
                        return Status;
                      // NOW, Build Data String..
                      sprintf( (char *)tempBffr2, "     Humidity DATA: " );
                      strcat( (char *)tempBffr2, (char *)HMeasure.HRaw );
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
                    sprintf( (char *)tempBffr2, "     Humidity DATA(Decimal): %d\r\n", HMeasure.HRawC );
#ifdef NUCLEO
                    Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                    Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                    if (Status != HAL_OK)
                      return Status;
                    // Now calculate Humidity.
                    sprintf( (char *)tempBffr2, "     Humidity: " );
                    strcat( (char *)tempBffr2, (char *)HMeasure.Humidity );
                    strcat( (char *)tempBffr2, "\r\n" );
                    break;
//------------------ D1 Command...Read Temperature Values..... 
                  case '1':
                    // Read Humidity Sensor sensor and return Temperature results....
                    Status = RoadBrd_Humidity_ReadTemperature( &TMeasure );
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }

                    if (Status == HAL_OK)
                    {
                      // Send string to UART..
                      strcpy( (char *)tempBffr2, "Humidity SENSOR...\r\n");
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
                    // Now calculate Celcius and Farenheit Temp.
                    sprintf( (char *)tempBffr2, "     TempC: " );
                    strcat( (char *)tempBffr2, (char *)TMeasure.TempC );
                    strcat( (char *)tempBffr2, "     TempF: " );
                    strcat( (char *)tempBffr2, (char *)TMeasure.TempF );
                    strcat( (char *)tempBffr2, "\r\n" );
                    break;
                } //EndSwitch
              } //EndElse (Size == 1)
              break;
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
            case 'H':
              // RGB Color Light Sensor U15(ISL91250).  Addr: 0x88
              if (Size == 1)
              {
//------------------ H Command...Read RGB Values and Return as 3 (2 Byte Fields)....REDmsb,REDlsb,GREENmsb,GREENlsb,BLUEmsb,BLUElsb.....     
                // 1. Time to send Command and collect status.
                Status = RoadBrd_RGBReadValues( &RGBValues );
                if (Status == HAL_OK)
                {
                  // Send string to UART..
                  strcpy( (char *)tempBffr2, "RGB Color Light Sensor...\r\n");
#ifdef NUCLEO
                  Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                  Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                  if (Status != HAL_OK)
                    return Status;
                  // NOW, Build Data String..
                  sprintf( (char *)tempBffr2, " RGB(REDmsb,REDlsb,GREENmsb,GREENlsb,BLUEmsb,BLUElsb)\r\n" );
#ifdef NUCLEO
                  Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                  Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                  if (Status != HAL_OK)
                    return Status;
                  strcpy( (char *)tempBffr2, "       DATA: ");
                  strcat( (char *)tempBffr2, (char *)RGBValues.Raw );
                  strcat( (char *)tempBffr2, "\r\n" );
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
                    sprintf( (char *)tempBffr2, "<STATUS>RED:%s/GREEN:%s/BLUE:%s</STATUS>", 
                            (char *)RGBValues.Red, 
                            (char *)RGBValues.Green,
                            (char *)RGBValues.Blue);
                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                  }
                  
                  // Now DisplayEach Value Calculated.
                  strcpy( (char *)tempBffr2, "    Red: ");
                  strcat( (char *)tempBffr2, (char *)RGBValues.Red );
                  strcat( (char *)tempBffr2, "    Green: ");
                  strcat( (char *)tempBffr2, (char *)RGBValues.Green );
                  strcat( (char *)tempBffr2, "    Blue: ");
                  strcat( (char *)tempBffr2, (char *)RGBValues.Blue );
                  strcat( (char *)tempBffr2, "\r\n" );
                }
                else
                  break;
              }
              else
              {
                switch( tempBffr[1] )
                {
//------------------ HI Command...Initialize RGB Color Light Sensor.....     
                  case 'I':
                    if (Size == 2)
                    {
                      // This is the default init. Assume Default Parms and write them.
                      Status = RoadBrd_RGBInit();

                      // Is this a BLE Operation?
                      if ( BLE_Flag )
                      {
                        // Yes...Build and Send BLE Response NOW.
                        strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                        BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                      }
                      
                      if (Status == HAL_OK)
                      {
                        strcpy( (char *)tempBffr2, "RGB Color Light Sensor: Initialization Complete with DEFAULT Values.\r\n");;
                      }
                      
                    }
                    else
                    {
                      // Is this a BLE Operation?
                      if ( BLE_Flag )
                      {
                        // Yes...Build and Send BLE Response NOW.
                        strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                        BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                      }
                      
                      // This is the Parameter init. Will have to verify all parameters first.
                      if ( Size != 29 )
                      {
                        strcpy( (char *)tempBffr2, "RGB Color Light Sensor: SYNTAX Error. Parameters are not correct.\r\n");;
                      }
                      else
                      {
                        // Step 1. Validate format.
                        if( (tempBffr[2]!=':') ||
                            (tempBffr[5]!='.') || 
                            (tempBffr[8]!='.') || 
                            (tempBffr[11]!='.') || 
                            (tempBffr[14]!='.') || 
                            (tempBffr[17]!='.') || 
                            (tempBffr[20]!='.') || 
                            (tempBffr[23]!='.') || 
                            (tempBffr[26]!='.') )
                        {
                          strcpy( (char *)tempBffr2, "RGB Color Light Sensor SYNTAX ERROR: Not correct format. Punctuation!\r\n");
                        }
                        else
                        {
                          // Syntax correct. Time to grab parameters.
                          Error = 0;
                          for (x=0; x<9; x++)
                          {
                            tempBffr3[0] = tempBffr[3+x*3];
                            tempBffr3[1] = tempBffr[4+x*3];
                            tempBffr3[2] = 0x00;
                            if (isHexNum( (char *)tempBffr3 ) == 0)
                            {
                              strcpy( (char *)tempBffr2, "RGB Color Light Sensor SYNTAX ERROR:Parameters not HEX Value.\r\n");
                              Error = 1;
                              break;
                            }
                            else
                            {
                              i2cData[x] =  hatoi( (char *)tempBffr3 );
                            } //EndElse (isHexNum( (char *)tempBffr3 ) == 0)
                          } //EndFor (x=0; x<num_bytes; x++)
                          // Format and parameters now in i2cData array. Pull them out and validate them.
                          // OP_MODE Verify.
                          if(i2cData[0]>7)
                          {
                            strcpy( (char *)tempBffr2, "RGB Color Light Sensor SYNTAX ERROR: BAD PARAM: OP_MODE.\r\n");
                            break;
                          }
                          else
                            op_mode = (i2cData[0] & 0x07) * 1;
                          // DS_RANGE Verify.
                          if(i2cData[1]>1)
                          {
                            strcpy( (char *)tempBffr2, "RGB Color Light Sensor SYNTAX ERROR: BAD PARAM: DS_RANGE.\r\n");
                            break;
                          }
                          else
                            ds_range = (i2cData[1] & 0x01) * 8;
                          // ADC_RSL Verify.
                          if(i2cData[2]>1)
                          {
                            strcpy( (char *)tempBffr2, "RGB Color Light Sensor SYNTAX ERROR: BAD PARAM: ADC_RSL.\r\n");
                            break;
                          }
                          else
                            adc_rsl = (i2cData[2] & 0x01) * 16;
                          // SYNC Verify.
                          if(i2cData[3]>1)
                          {
                            strcpy( (char *)tempBffr2, "RGB Color Light Sensor SYNTAX ERROR: BAD PARAM: SYNC.\r\n");
                            break;
                          }
                          else
                            sync = (i2cData[3] & 0x01) * 32;
                          // CMP_ADJST Verify.
                          if(i2cData[4]>63)
                          {
                            strcpy( (char *)tempBffr2, "RGB Color Light Sensor SYNTAX ERROR: BAD PARAM: CMP_ADJST.\r\n");
                            break;
                          }
                          else
                            cmp_adjst = (i2cData[4] & 0x3f) * 1;
                          // CMP_OFFST Verify.
                          if(i2cData[5]>1)
                          {
                            strcpy( (char *)tempBffr2, "RGB Color Light Sensor SYNTAX ERROR: BAD PARAM: CMP_OFFST.\r\n");
                            break;
                          }
                          else
                            cmp_offst = (i2cData[5] & 0x01) * 128;
                          // INT_ASSGN Verify.
                          if(i2cData[6]>3)
                          {
                            strcpy( (char *)tempBffr2, "RGB Color Light Sensor SYNTAX ERROR: BAD PARAM: INT_ASSGN.\r\n");
                            break;
                          }
                          else
                            int_assgn = (i2cData[6] & 0x03) * 1;
                          // INT_PERSIST Verify.
                          if(i2cData[7]>3)
                          {
                            strcpy( (char *)tempBffr2, "RGB Color Light Sensor SYNTAX ERROR: BAD PARAM: INT_PERSIST.\r\n");
                            break;
                          }
                          else
                            int_persist = (i2cData[7] & 0x03) * 4;
                          // CNVRSN_INT Verify.
                          if(i2cData[8]>1)
                          {
                            strcpy( (char *)tempBffr2, "RGB Color Light Sensor SYNTAX ERROR: BAD PARAM: CNVRSN_INT.\r\n");
                            break;
                          }
                          else
                            cnvrsn_int = (i2cData[8] & 0x01) * 16;
                          // OK, all parameters have been verified. Time to build final params.
                          RGBMeasure.config1 = op_mode + ds_range + adc_rsl + sync;
                          RGBMeasure.config2 = cmp_adjst + cmp_offst;
                          RGBMeasure.config3 = int_assgn + int_persist + cnvrsn_int;
                          // Load Config Register with Config Settings
                          Status = RoadBrd_RGBFullInit( &RGBMeasure );

                          if (Status == HAL_OK)
                          {
                            strcpy( (char *)tempBffr2, "RGB Color Light Sensor: Initialization Complete with USER Values.\r\n");;
                          }
                        } //ElseIf Validate format.
                      } //ElseIf ( Size != 29 )
                    } //ElseIf (Size == 2)
                    break;
//------------------ H0 Command...Read RGB Values and Return as 3 (2 Byte Fields)....REDmsb,REDlsb,GREENmsb,GREENlsb,BLUEmsb,BLUElsb.....     
                  case '0':
                    // 1. Time to send Command and collect status.
                    Status = RoadBrd_RGBReadValues( &RGBValues );
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    
                    if (Status == HAL_OK)
                    {
                      // Send string to UART..
                      strcpy( (char *)tempBffr2, "RGB Color Light Sensor...\r\n");
#ifdef NUCLEO
                      Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                      if (Status != HAL_OK)
                        return Status;
                      // NOW, Build Data String..
                      sprintf( (char *)tempBffr2, " RGB(REDmsb,REDlsb,GREENmsb,GREENlsb,BLUEmsb,BLUElsb)\r\n" );
#ifdef NUCLEO
                      Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                      if (Status != HAL_OK)
                        return Status;
                      strcpy( (char *)tempBffr2, "       DATA: ");
                      strcat( (char *)tempBffr2, (char *)RGBValues.Raw );
                      strcat( (char *)tempBffr2, "\r\n" );
#ifdef NUCLEO
                      Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                      if (Status != HAL_OK)
                        return Status;
                      // Now DisplayEach Value Calculated.
                      strcpy( (char *)tempBffr2, "    Red: ");
                      strcat( (char *)tempBffr2, (char *)RGBValues.Red );
                      strcat( (char *)tempBffr2, "    Green: ");
                      strcat( (char *)tempBffr2, (char *)RGBValues.Green );
                      strcat( (char *)tempBffr2, "    Blue: ");
                      strcat( (char *)tempBffr2, (char *)RGBValues.Blue );
                      strcat( (char *)tempBffr2, "\r\n" );
                    }
                    else
                      break;
                    break;
//------------------ H1 Command...Read Status.....     
                  case '1':
                    // 1. Time to send Command and collect status.  RGBSMeasure
                    Status = RoadBrd_RGBReadStatus( &RGBSMeasure );

                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    
                    if (Status == HAL_OK)
                    {
                      // Build Status
                      strcpy( (char *)tempBffr2, "RGB Color Light Sensor Status: ");
                      sprintf( (char *)tempBffr3, "%02x / ", RGBSMeasure.status);
                      strcat( (char *)tempBffr2, (char *)tempBffr3 );
                      strcat( (char *)tempBffr2, (char *)RGBSMeasure.Raw );
                      strcat( (char *)tempBffr2, "\r\n" );
                    }
                    else
                      break;
                    break;
//------------------ H2 Command...Reset Hardware......     
                  case '2':
                    Status = RoadBrd_RGBReset();
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    
                    if (Status == HAL_OK)
                    {
                      // Build Status
                      strcpy( (char *)tempBffr2, "RGB Color Light Sensor: RESET CMD Sent Succesful.\r\n" );
                    }
                    else
                      break;
                    break;
//------------------ H3 Command...Read ID.....     
                  case '3':
                    // 1. Time to send Command and collect status.  IDMeasure
                    Status = RoadBrd_RGBReadID( &IDMeasure );
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    
                    if (Status == HAL_OK)
                    {
                      // Build Status
                      strcpy( (char *)tempBffr2, "RGB Color Light Sensor ID Code: ");
                      sprintf( (char *)tempBffr3, "%02x / ", IDMeasure.id);
                      strcat( (char *)tempBffr2, (char *)tempBffr3 );
                      strcat( (char *)tempBffr2, (char *)IDMeasure.Raw );
                      strcat( (char *)tempBffr2, "\r\n" );
                   }
                    else
                      break;
                    break;
                  default:
                    strcpy( (char *)tempBffr2, "ERROR: Not a legal command.\r\n");
                    break;
                } //EndSwitch
              } //EndElse (Size == 1)
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
//++++++++++++++++++++++++++++++++++++++++++  I2C Commands.
                  case 'I':
                    // I2C Commands.
                    // Test Size to make sure we have enough Characters for this operation
                    if (Size < 9)
                    {
                      // Is this a BLE Operation?
                      if ( BLE_Flag )
                      {
                        // Yes...Build and Send BLE Response NOW.
                        strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                        BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                      }
                      
                      strcpy( (char *)tempBffr2, "TI SYNTAX ERROR: Not correct format.\r\n");
                    }
                    else
                    {
                      switch( tempBffr[2] )
                      {
//------------------
                        case 'S':
                          //I2C Send Command.
                          // Step 1. Validate format.
                          if( (tempBffr[3]!=':') ||
                              (tempBffr[6]!='.') )
                          {
                            // Is this a BLE Operation?
                            if ( BLE_Flag )
                            {
                              // Yes...Build and Send BLE Response NOW.
                              strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                              BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                            }
                           
                            strcpy( (char *)tempBffr2, "TIS SYNTAX ERROR: Not correct format.\r\n");
                          }
                          else
                          {
                            // Is this a BLE Operation?
                            if ( BLE_Flag )
                            {
                              // Yes...Build and Send BLE Response NOW.
                              strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                              BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                            }
                            
                            // 2. Grab Address and validate a legal number
                            tempBffr3[0] = tempBffr[4];
                            tempBffr3[1] = tempBffr[5];
                            tempBffr3[2] = 0x00;
                            if (isHexNum( (char *)tempBffr3 ) == 0)
                              strcpy( (char *)tempBffr2, "TIS SYNTAX ERROR: Address not HEX Value.\r\n");
                            else
                            {
                              // Legal Address. Save it as value
                              Address = hatoi( (char *)tempBffr3 );
                              // 3. Now get the number of bytes of data from field.
//                              sprintf( (char *)tempBffr2, "TIS: Good Address: %x.\r\n", Address);
                              tempBffr3[0] = tempBffr[7];
                              tempBffr3[1] = tempBffr[8];
                              tempBffr3[2] = 0x00;
                              if (isHexNum( (char *)tempBffr3 ) == 0)
                                strcpy( (char *)tempBffr2, "TIS SYNTAX ERROR: Number of Bytes not HEX Value.\r\n");
                              else
                              {
                                // Legal NUMBER BYTES. Save it as value
                                num_bytes = hatoi( (char *)tempBffr3 );
                                // 4. Test num_bytes. If Zero, We are done
                                if (num_bytes == 0)
                                {
                                  sprintf( (char *)tempBffr2, "TIS: GOOD CMD: %x.\r\n", Address);
                                }
                                else
                                {
                                  // 5. Time to get all the data.
                                  Error = 0;
                                  for (x=0; x<num_bytes; x++)
                                  {
                                    tempBffr3[0] = tempBffr[10+x*3];
                                    tempBffr3[1] = tempBffr[11+x*3];
                                    tempBffr3[2] = 0x00;
                                    if (isHexNum( (char *)tempBffr3 ) == 0)
                                    {
                                      strcpy( (char *)tempBffr2, "TIS SYNTAX ERROR: Address not HEX Value.\r\n");
                                      Error = 1;
                                      break;
                                    }
                                    else
                                    {
                                      i2cData[x] =  hatoi( (char *)tempBffr3 );
                                    } //EndElse (isHexNum( (char *)tempBffr3 ) == 0)
                                  } //EndFor (x=0; x<num_bytes; x++)
                                } //EndElse (num_bytes == 0)
                                if (Error==0)
                                {
                                  sprintf( (char *)tempBffr2, "TIS: GOOD CMD: %x.", Address);
                                }
                                for(x=0; x<num_bytes; x++)
                                {
                                  sprintf( (char *)tempBffr3, "%x.", i2cData[x]);
                                  strcat( (char *)tempBffr2, (char *)tempBffr3 );
                                }
                                strcat( (char *)tempBffr2, "\r\n" );
                                // 6. Time to send Command and collect status.
                                Status =  RoadBrd_I2C_Master_Transmit((uint16_t)Address, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
                                
                              } //EndElse (isHexNum( (char *)tempBffr3 ) == 0)...NUMBER BYTES
                              
                            } //EndElse (isHexNum( (char *)tempBffr3 ) == 0)...Address
                            
                          } //EndElse ( (tempBffr[2]!=':') || (tempBffr[5]!='.') )
                      
                          break;
//------------------
                        case 'R':
                          //I2C Receive Command
                          // Step 1. Validate format.
                          if( (tempBffr[3]!=':') ||
                              (tempBffr[6]!='.') ||
                              (tempBffr[9]!='.')  )
                          {
                            // Is this a BLE Operation?
                            if ( BLE_Flag )
                            {
                              // Yes...Build and Send BLE Response NOW.
                              strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                              BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                            }
                            
                            strcpy( (char *)tempBffr2, "TIR SYNTAX ERROR: Not correct format.\r\n");
                          }
                          else
                          {
                            // Is this a BLE Operation?
                            if ( BLE_Flag )
                            {
                              // Yes...Build and Send BLE Response NOW.
                              strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                              BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                            }
                            
                            // 2. Grab Address and validate a legal number
                            tempBffr3[0] = tempBffr[4];
                            tempBffr3[1] = tempBffr[5];
                            tempBffr3[2] = 0x00;
                            if (isHexNum( (char *)tempBffr3 ) == 0)
                              strcpy( (char *)tempBffr2, "TIR SYNTAX ERROR: Address not HEX Value.\r\n");
                            else
                            {
                              // Legal Address. Save it as value
                              Address = hatoi( (char *)tempBffr3 );
                              // 3. Now get the number of bytes of data from field.
                              tempBffr3[0] = tempBffr[7];
                              tempBffr3[1] = tempBffr[8];
                              tempBffr3[2] = 0x00;
                              if (isHexNum( (char *)tempBffr3 ) == 0)
                                strcpy( (char *)tempBffr2, "TIR SYNTAX ERROR: Number of Bytes not HEX Value.\r\n");
                              else
                              {
                                // Legal NUMBER BYTES. Save it as value
                                num_bytes = hatoi( (char *)tempBffr3 );
                                
                                // 3a. Now get the number of bytes Received of data from field.
                                tempBffr3[0] = tempBffr[10];
                                tempBffr3[1] = tempBffr[11];
                                tempBffr3[2] = 0x00;
                                if (isHexNum( (char *)tempBffr3 ) == 0)
                                  strcpy( (char *)tempBffr2, "TIR SYNTAX ERROR: Number of Bytes RECEIVED not HEX Value.\r\n");
                                else
                                {
                                  // Legal NUMBER BYTES. Save it as value
                                  num_bytes_received = hatoi( (char *)tempBffr3 );
                                  // 4. Test num_bytes. If Zero, We are done
                                  if (num_bytes == 0)
                                  {
                                    sprintf( (char *)tempBffr2, "TIR: GOOD CMD: %x.\r\n", Address);
                                  }
                                  else
                                  {
                                    // 5. Time to get all the data.
                                    Error = 0;
                                    for (x=0; x<num_bytes; x++)
                                    {
                                      tempBffr3[0] = tempBffr[13+x*3];
                                      tempBffr3[1] = tempBffr[14+x*3];
                                      tempBffr3[2] = 0x00;
                                      if (isHexNum( (char *)tempBffr3 ) == 0)
                                      {
                                        strcpy( (char *)tempBffr2, "TIR SYNTAX ERROR: Data not HEX Value.\r\n");
                                        Error = 1;
                                        break;
                                      }
                                      else
                                      {
                                        i2cData[x] =  hatoi( (char *)tempBffr3 );
                                      } //EndElse (isHexNum( (char *)tempBffr3 ) == 0)
                                    } //EndFor (x=0; x<num_bytes; x++)
                                  } //EndElse (num_bytes == 0)
                                  if (Error==0)
                                  {
                                    sprintf( (char *)tempBffr2, "TIR: GOOD CMD: %x.", Address);
                                  }
                                  for(x=0; x<num_bytes; x++)
                                  {
                                    sprintf( (char *)tempBffr3, "%x.", i2cData[x]);
                                    strcat( (char *)tempBffr2, (char *)tempBffr3 );
                                  }
                                  strcat( (char *)tempBffr2, "\r\n" );
                                  // 6. Time to send Command and collect status.
                                  Status =  RoadBrd_I2C_Master_Transmit((uint16_t)Address, i2cData, (uint16_t)num_bytes, I2C_TIMEOUT);
                                  // 6a. Wait for Command to complete(100ms).
 
                                  // 7. If Status was good, Time to get response.
                                  if (Status == HAL_OK)
                                  {
                                    Status =  RoadBrd_I2C_Master_Receive((uint16_t)Address, i2cData, (uint16_t)num_bytes_received, I2C_TIMEOUT);
                                  }
                                  else
                                    break;
                                  // 7a. Wait for Command to complete(100ms).
                                  if (Status == HAL_OK)
                                  {
                                    Status = RoadBrd_WaitForState( 20 );
                                  }

                                  // 8. IfGood report, Need to Output Data.
                                  if (Status == HAL_OK)
                                  {
                                    // Send string to UART..
#ifdef NUCLEO
                                    Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                                    Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                                    if (Status != HAL_OK)
                                      return Status;
                                    // NOW, Build Data String..
                                    sprintf( (char *)tempBffr2, "     DATA: " );
                                    for(x=0; x<num_bytes_received; x++)
                                    {
                                      sprintf( (char *)tempBffr3, "%x.", i2cData[x]);
                                      strcat( (char *)tempBffr2, (char *)tempBffr3 );
                                    }
                                    strcat( (char *)tempBffr2, "\r\n" );
                                  }
                                }
                              } //EndElse (isHexNum( (char *)tempBffr3 ) == 0)...NUMBER BYTES
                              
                            } //EndElse (isHexNum( (char *)tempBffr3 ) == 0)...Address
                            
                          } //EndElse ( (tempBffr[2]!=':') || (tempBffr[5]!='.') )
                      
                          break;
//------------------
                        case 'Q':
                          //I2C Receive Command
                          // Step 1. Validate format.
                          if( (tempBffr[3]!=':') ||
                              (tempBffr[6]!='.')  )
                          {
                            // Is this a BLE Operation?
                            if ( BLE_Flag )
                            {
                              // Yes...Build and Send BLE Response NOW.
                              strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                              BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                            }
                            
                            strcpy( (char *)tempBffr2, "TIQ SYNTAX ERROR: Not correct format.\r\n");
                          }
                          else
                          {
                            // Is this a BLE Operation?
                            if ( BLE_Flag )
                            {
                              // Yes...Build and Send BLE Response NOW.
                              strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                              BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                            }
                            
                            // 2. Grab Address and validate a legal number
                            tempBffr3[0] = tempBffr[4];
                            tempBffr3[1] = tempBffr[5];
                            tempBffr3[2] = 0x00;
                            if (isHexNum( (char *)tempBffr3 ) == 0)
                              strcpy( (char *)tempBffr2, "TIQ SYNTAX ERROR: Address not HEX Value.\r\n");
                            else
                            {
                              // Legal Address. Save it as value
                              Address = hatoi( (char *)tempBffr3 );
                              // 3. Now get the number of bytes Received of data from field.
                              tempBffr3[0] = tempBffr[7];
                              tempBffr3[1] = tempBffr[8];
                              tempBffr3[2] = 0x00;
                              if (isHexNum( (char *)tempBffr3 ) == 0)
                                strcpy( (char *)tempBffr2, "TIQ SYNTAX ERROR: Number of Bytes not HEX Value.\r\n");
                              else
                              {
                                  // Legal NUMBER BYTES. Save it as value
                                  num_bytes_received = hatoi( (char *)tempBffr3 );
                                  // 4. Test num_bytes. If Zero, We are done
                                  sprintf( (char *)tempBffr2, "TIR: GOOD CMD: %x.\r\n", Address);
                                  // 7. If Status was good, Time to get response.
                                  Status =  RoadBrd_I2C_Master_Receive((uint16_t)Address, i2cData, (uint16_t)num_bytes_received, I2C_TIMEOUT);
                                  // 7a. Wait for Command to complete(100ms).
                                  if (Status == HAL_OK)
                                  {
                                    Status = RoadBrd_WaitForState( 20 );
                                  }
                                  else
                                    break;
                                  // 8. IfGood report, Need to Output Data.
                                  if (Status == HAL_OK)
                                  {
                                    // Send string to UART..
#ifdef NUCLEO
                                    Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                                    Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                                    if (Status != HAL_OK)
                                      return Status;
                                    // NOW, Build Data String..
                                    sprintf( (char *)tempBffr2, "     DATA: " );
                                    for(x=0; x<num_bytes_received; x++)
                                    {
                                      sprintf( (char *)tempBffr3, "%x.", i2cData[x]);
                                      strcat( (char *)tempBffr2, (char *)tempBffr3 );
                                    }
                                    strcat( (char *)tempBffr2, "\r\n" );
                                }
                              } //EndElse (isHexNum( (char *)tempBffr3 ) == 0)...NUMBER BYTES
                              
                            } //EndElse (isHexNum( (char *)tempBffr3 ) == 0)...Address
                            
                          } //EndElse ( (tempBffr[2]!=':') || (tempBffr[5]!='.') )
                      
                          break;
                        default:
                          strcpy( (char *)tempBffr2, "ERROR: Not a legal command.\r\n");
                          break;
                      } //EndSwitch ( tempBffr[2] )
                    } //EndElse (Size < 9)
                    break;
//++++++++++++++++++++++++++++++++++++++++++  Calibration Commands.
                  case 'C':
                    Status = HAL_OK;
                    if (Size == 2)
                    {
                      //------------------ TC Command: Dump Calibration Settings.      
                      // Read Cool Eye/Grid Eye Values.....
                      if ( Get_DriverStates( GRIDEYE_MNTR_TASK ))
                      {
                        Status = RoadBrd_GridEye_ReadValues_Scaled( &GridMeasure );
                      }
                      else if ( Get_DriverStates( COOLEYE_MNTR_TASK ))
                      {
                        Status = RoadBrd_CoolEye_ReadValues_Scaled( &GridMeasure );
                      }
                      else
                        Status = HAL_ERROR;
                      
                      // Is this a BLE Operation?
                      if (Status == HAL_OK)
                      {
                        // OK Next Sensor.
                        // Read Temperature sensor and return results....Temperature Sensor U10(PCT2075GVJ).  Addr: 0x94
                        Status = RoadBrd_ReadTemp_Scaled( &TMeasure );
                        if (Status == HAL_OK)
                        {
                          // OK Next Sensor.
                          // Read Humidity Sensor sensor and return Humidity results....
                          Status = RoadBrd_Humidity_ReadHumidity_Scaled( &HMeasure );
                          if (Status == HAL_OK)
                          {
                            // OK Next Sensor.
                            //Status = RoadBrd_Barometer_Status( &PRMeasure );
                            Status = RoadBrd_Baro_ReadPressure_Scaled( &PRPMeasure );
                            if (Status == HAL_OK)
                            {
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "<STATUS>%s//%s//%s//%s//%s//%s//%s//%s//%s//%s//%s//%s</STATUS>", (char *)GridMeasure.GridEye1.TempC,
                                        (char *)GridMeasure.GridEye2.TempC,
                                        (char *)GridMeasure.GridEye3.TempC,
                                        (char *)GridMeasure.GridEye4.TempC,
                                        (char *)GridMeasure.GridEye5.TempC,
                                        (char *)GridMeasure.GridEye6.TempC,
                                        (char *)GridMeasure.GridEye7.TempC,
                                        (char *)GridMeasure.GridEye8.TempC,
                                        (char *)GridMeasure.Thermistor.TempC,
                                        (char *)TMeasure.TempC,
                                        (char *)HMeasure.Humidity,
                                        (char *)PRPMeasure.Pressure);
                                strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              
                              //sprintf( (char *)tempBffr2, "Driver Status: %04x\r\n", DriverStatus );
                              sprintf( (char *)tempBffr2, "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\r\n", (char *)GridMeasure.GridEye1.TempC,
                                      (char *)GridMeasure.GridEye2.TempC,
                                      (char *)GridMeasure.GridEye3.TempC,
                                      (char *)GridMeasure.GridEye4.TempC,
                                      (char *)GridMeasure.GridEye5.TempC,
                                      (char *)GridMeasure.GridEye6.TempC,
                                      (char *)GridMeasure.GridEye7.TempC,
                                      (char *)GridMeasure.GridEye8.TempC,
                                      (char *)GridMeasure.Thermistor.TempC,
                                      (char *)TMeasure.TempC,
                                      (char *)HMeasure.Humidity,
                                      (char *)PRPMeasure.Pressure);
                              // Send string to UART..
#ifdef NUCLEO
                              Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                              Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                              if (Status != HAL_OK)
                                return Status;
                              // NOW, Build Data String..
                              sprintf( (char *)tempBffr2, "COMPLETE" );
                            } // Endif (Status == HAL_OK) RoadBrd_Baro_ReadPressure
                            else
                            {
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                strcpy( (char *)tempBffr2, "<STATUS>CMD_TC_HWERR_PRESSUE</STATUS>");
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              
                              sprintf( (char *)tempBffr2, "Pressure TASKING ERROR!" );
                            } // EndElse (Status == HAL_OK) RoadBrd_Baro_ReadPressure
                          } // Endif (Status == HAL_OK) RoadBrd_Humidity_ReadHumidity
                          else
                          {
                            if ( BLE_Flag )
                            {
                              // Yes...Build and Send BLE Response NOW.
                              strcpy( (char *)tempBffr2, "<STATUS>CMD_TC_HWERR_HUMIDITY</STATUS>");
                              BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                            }
                            
                            sprintf( (char *)tempBffr2, "Humidity TASKING ERROR!" );
                          } // EndElse (Status == HAL_OK) RoadBrd_Humidity_ReadHumidity
                        } // Endif (Status == HAL_OK) RoadBrd_ReadTemp
                        else
                        {
                          if ( BLE_Flag )
                          {
                            // Yes...Build and Send BLE Response NOW.
                            strcpy( (char *)tempBffr2, "<STATUS>CMD_TC_HWERR_TEMPERATURE</STATUS>");
                            BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                          }
                          
                          sprintf( (char *)tempBffr2, "AMBIENT TEMPERATURE TASKING ERROR!" );
                        } // EndElse (Status == HAL_OK) RoadBrd_ReadTemp
                      } // Endif (Status == HAL_OK) RoadBrd_CoolEye_ReadValues
                      else
                      {
                        if ( BLE_Flag )
                        {
                          // Yes...Build and Send BLE Response NOW.
                          strcpy( (char *)tempBffr2, "<STATUS>CMD_TC_HWERR_IRTEMP</STATUS>");
                          BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                        }
                        
                        sprintf( (char *)tempBffr2, "GRID EYE/COOL EYE TASKING ERROR!" );
                      } // EndElse (Status == HAL_OK) RoadBrd_CoolEye_ReadValues
                    }
                    else
                    {
                      switch( tempBffr[2] )
                      {
                        //------------------ TCS Command: Calibration Set Command
                      case 'S':
                        // Step 1. Validate format.
                        if(tempBffr[3]!=':')
                        {
                          // Is this a BLE Operation?
                          if ( BLE_Flag )
                          {
                            // Yes...Build and Send BLE Response NOW.
                            strcpy( (char *)tempBffr2, "<STATUS>CMD_TCS_SYNTAX</STATUS>");
                            BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                          }
                          
                          strcpy( (char *)tempBffr2, "TCS SYNTAX ERROR: Not correct format.\r\n");
                        } // Endif (tempBffr[3]!=':')
                        else
                        {
                          // 2. Verify if remaining string is digits
                          if (Size <= 4)
                          {
                            // Is this a BLE Operation?
                            if ( BLE_Flag )
                            {
                              // Yes...Build and Send BLE Response NOW.
                              strcpy( (char *)tempBffr2, "<STATUS>CMD_TCS_BADPARAM</STATUS>");
                              BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                            }
                            strcpy( (char *)tempBffr2, "TCS SYNTAX ERROR: Bad Parameter.\r\n");
                          } // EndIf (Size > 4)
                          else
                          {
                            // 3. Grab remaining string and convert to integer.
                            tempPstr = &tempBffr[4];
                            strcpy(tempstr, tempPstr);
                            // Time to parse and test remaining string
                            Scale = 0.0;
                            Offset = 0.0;
                            if (sscanf (tempstr, "%s %f %f", uuid, &Scale, &Offset) == 3)
                            {
                              sprintf( (char *)tempBffr2, "Parms: %s, %f, %f.\r\n", uuid, Scale, Offset );
                              // OK, We have 3 good parameters... NOW Need to determine if UUID is good.
                              if (strncmp((char *)uuid,"0002",4) == 0) // Shnt_Vltg
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_SHNT_VLTG, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_SHNT_VLTG</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "Shnt_Vltg Set COMPLETE.\r\n" );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_SHNT_VLTG_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              else if (strncmp((char *)uuid,"0004",4) == 0) // Current
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_CURRENT, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_CURRENT</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "Current Set COMPLETE." );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_CURRENT_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              else if (strncmp((char *)uuid,"0006",4) == 0) // Power
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_POWER, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_POWER</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "Power Set COMPLETE." );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_POWER_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              else if (strncmp((char *)uuid,"0008",4) == 0) // Voltage
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_VOLTAGE, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_VOLTAGE</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "Voltage Set COMPLETE." );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_VOLTAGE_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              else if (strncmp((char *)uuid,"000A",4) == 0) // TempC
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_TEMPC, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_TEMPC</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TempC Set COMPLETE." );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_TEMPC_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              else if (strncmp((char *)uuid,"000B",4) == 0) // TempF
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_TEMPF, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_TEMPF</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TempF Set COMPLETE." );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_TEMPF_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              else if (strncmp((char *)uuid,"0011",4) == 0) // Pressure
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_PRESSURE, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_PRESSURE</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "Pressure Set COMPLETE." );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_PRESSURE_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              else if (strncmp((char *)uuid,"0030",4) == 0) // Humidity
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_HUMIDITY, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_HUMIDITY</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "Humidity Set COMPLETE." );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_HUMIDITY_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              else if (strncmp((char *)uuid,"0032",4) == 0) // Hum_TempC
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_HUM_TEMPC, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_HUM_TEMPC</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "Hum_TempC Set COMPLETE." );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_HUM_TEMPC_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              else if (strncmp((char *)uuid,"0033",4) == 0) // Hum_TempF
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_HUM_TEMPF, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_HUM_TEMPF</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "Hum_TempF Set COMPLETE." );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_HUM_TEMPF_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              else if (strncmp((char *)uuid,"000D",4) == 0) // RGB_Red
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_RGB_RED, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_RGB_RED</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "RGB_Red Set COMPLETE." );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_RGB_RED_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              else if (strncmp((char *)uuid,"000E",4) == 0) // RGB_Green
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_RGB_GREEN, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_RGB_GREEN</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "RGB_Green Set COMPLETE." );
                                }
                                else
                                {
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_RGB_GREEN_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              else if (strncmp((char *)uuid,"000F",4) == 0) // RGB_Blue
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_RGB_BLUE, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_RGB_BLUE</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "RGB_Blue Set COMPLETE." );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_RGB_BLUE_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              else if (strncmp((char *)uuid,"0017",4) == 0) // Therm_C
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_THERM_C, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_THERM_C</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "Therm_C Set COMPLETE." );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_THERM_C_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              else if (strncmp((char *)uuid,"0019",4) == 0) // RoadT_1C
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_ROADT_1C, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_ROADT_1C</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "RoadT_1C Set COMPLETE." );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_ROADT_1C_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              else if (strncmp((char *)uuid,"001B",4) == 0) // RoadT_2C
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_ROADT_2C, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_ROADT_2C</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "RoadT_2C Set COMPLETE." );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_ROADT_2C_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              else if (strncmp((char *)uuid,"001D",4) == 0) // RoadT_3C
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_ROADT_3C, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_ROADT_3C</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "RoadT_3C Set COMPLETE." );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_ROADT_3C_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              else if (strncmp((char *)uuid,"001F",4) == 0) // RoadT_4C
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_ROADT_4C, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_ROADT_4C</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "RoadT_4C Set COMPLETE." );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_ROADT_4C_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              else if (strncmp((char *)uuid,"0021",4) == 0) // RoadT_5C
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_ROADT_5C, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_ROADT_5C</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "RoadT_5C Set COMPLETE." );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_ROADT_5C_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              else if (strncmp((char *)uuid,"0023",4) == 0) // RoadT_6C
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_ROADT_6C, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_ROADT_6C</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "RoadT_6C Set COMPLETE." );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_ROADT_6C_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              else if (strncmp((char *)uuid,"0025",4) == 0) // RoadT_7C
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_ROADT_7C, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_ROADT_7C</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "RoadT_7C Set COMPLETE." );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_ROADT_7C_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              else if (strncmp((char *)uuid,"0027",4) == 0) // RoadT_8C
                              {
                                Status = RoadBrd_CAL_Set_CalItem( CAL_ROADT_8C, Offset, Scale);
                                if (Status == HAL_OK)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_ROADT_8C</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "RoadT_8C Set COMPLETE." );
                                }
                                else
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TCS_ROADT_8C_ERR</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  sprintf( (char *)tempBffr2, "TCS SET ERROR: FAILED.\r\n" );
                                }
                              }
                              else
                              {
                                // Is this a BLE Operation?
                                if ( BLE_Flag )
                                {
                                  // Yes...Build and Send BLE Response NOW.
                                  strcpy( (char *)tempBffr2, "<STATUS>CMD_TCS_BADUUID</STATUS>");
                                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                }
                                sprintf( (char *)tempBffr2, "TCS SYNTAX ERROR: Bad UUID.\r\n" );
                              }
                              Status = HAL_OK;
                            }
                            else
                            {
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                strcpy( (char *)tempBffr2, "<STATUS>CMD_TCS_BADPARAM</STATUS>");
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              
                              strcpy( (char *)tempBffr2, "TCS SYNTAX ERROR: Wrong Number of Parameters.\r\n");
                            }
                          } // EndElse (flag == 0)
                        } // EndElse (tempBffr[3]!=':')
                        break;
                        //------------------ TCR Command: Calibration Read Command
                      case 'R':
                        // Build Read Calibration Dump Part I....
                        // Is this a BLE Operation?
                        if ( BLE_Flag )
                        {
                          // Yes...Build and Send BLE Response NOW.
                          sprintf( (char *)tempBffr2, "<STATUS>TCR:%s\\",  RoadBrd_CAL_GetTimeString());
                          BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                        }
                        
                        // Send string to UART..
                        sprintf( (char *)tempBffr2, "CALIBRATION DATA\r\nDate: %s\r\n",  RoadBrd_CAL_GetTimeString());
#ifdef NUCLEO
                        Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                        Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                        if (Status != HAL_OK)
                            return Status;
                        // Build Read Calibration Dump Part II....
                        // Send string to UART..
                        sprintf( (char *)tempBffr2, "Name		        UUID		Slope		Offset\r\n" );
#ifdef NUCLEO
                        Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                        Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                        if (Status != HAL_OK)
                            return Status;
                        // NOW, Build Data String..
                        for (x=0; x<CAL_LAST_VALUE; x++)
                        {
                          // Build String
                          switch(x)
                          {
                            case CAL_SHNT_VLTG: //CAL_SHNT_VLTG Values
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:0002//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	0002		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_CURRENT: //CAL_CURRENT Values
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:0004//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	        0004		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_POWER: //CAL_POWER Values
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:0006//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	0006		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_VOLTAGE: //CAL_VOLTAGE Values
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:0008//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	        0008		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_TEMPC: //CAL_TEMPC Values
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:000A//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	000A		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_TEMPF: //CAL_TEMPF Values
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:000B//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	000B		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_PRESSURE: //CAL_PRESSURE Values
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:0011//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	0011		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_HUMIDITY: //CAL_HUMIDITY Values
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:0030//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	0030		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_HUM_TEMPC: //CAL_HUM_TEMPC Values
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:0032//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	0032		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_HUM_TEMPF: //CAL_HUM_TEMPF Values
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:0033//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	0033		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_RGB_RED: //CAL_RGB_RED Values
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:000D//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	        000D		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_RGB_GREEN: //CAL_RGB_GREEN Values
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:000E//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	000E		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_RGB_BLUE: //CAL_RGB_BLUE Values
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:000F//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	000F		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_THERM_C: //CAL_THERM_C Values
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:0017//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	        0017		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_ROADT_1C: //CAL_ROADT_1C Values
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:0019//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	0019		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_ROADT_2C: //CAL_ROADT_2C Values
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:001B//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	001B		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_ROADT_3C: //CAL_ROADT_3C Values
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:001D//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	001D		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_ROADT_4C: //CAL_ROADT_4C Values
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:001F//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	001F		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_ROADT_5C: //CAL_ROADT_5C Values
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:0021//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	0021		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_ROADT_6C: //CAL_ROADT_6C Values
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:0023//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	0023		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_ROADT_7C: //CAL_ROADT_7C Values
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:0025//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	0025		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                            case CAL_ROADT_8C: //CAL_ROADT_8C Values
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "%s:0027//%1.4f//%2.3f//", 
                                        (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                        RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "%s	0027		%1.4f		%2.3f\r\n", 
                                      (char *)RdBrd_CAL_GetStr( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetSlope( (Cal_Characteristic)x ),
                                      RoadBrd_CAL_GetOffset( (Cal_Characteristic)x ) );
                              break;
                          } // EndSwitch(x)
                          // Now Print String.
#ifdef NUCLEO
                          Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                          Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                          if (Status != HAL_OK)
                            return Status;
                        } // EndFor(x=0; x<CAL_LAST_VALUE; x++)
                        // Is this a BLE Operation?
                        if ( BLE_Flag )
                        {
                          // Yes...Build and Send BLE Response NOW.
                          strcpy( (char *)tempBffr2, "</STATUS>");
                          BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                        }
                        sprintf( (char *)tempBffr2, "\r\n     COMPLETE.\r\n" );
                        break;
                        //------------------ TCT Command: Calibration Set Time Command
                      case 'T':
                        // Step 1. Validate format.
                        if(tempBffr[3]!=':')
                        {
                          // Is this a BLE Operation?
                          if ( BLE_Flag )
                          {
                            // Yes...Build and Send BLE Response NOW.
                            strcpy( (char *)tempBffr2, "<STATUS>CMD_TCT_SYNTAX</STATUS>");
                            BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                          }
                          
                          strcpy( (char *)tempBffr2, "TCT SYNTAX ERROR: Not correct format.\r\n");
                        } // Endif (tempBffr[3]!=':')
                        else
                        {
                          // 2. Verify if remaining string is digits
                          if (Size <= 4)
                          {
                            // Is this a BLE Operation?
                            if ( BLE_Flag )
                            {
                              // Yes...Build and Send BLE Response NOW.
                              strcpy( (char *)tempBffr2, "<STATUS>CMD_TCT_BADPARAM</STATUS>");
                              BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                            }
                            strcpy( (char *)tempBffr2, "TCT SYNTAX ERROR: Bad Parameter.\r\n");
                          } // EndIf (Size > 4)
                          else
                          {
                            // 3. Grab remaining string and Save it.
                            tempPstr = &tempBffr[4];
                            strcpy(tempstr, tempPstr);
                            // NOW...Save it.
                            Status = RoadBrd_CAL_Set_TimeString( (uint8_t *)tempPstr );
                            if (Status != HAL_OK)
                            {
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                strcpy( (char *)tempBffr2, "<STATUS>CMD_TCT_ERR</STATUS>");
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              return Status;
                            }
                            else
                            {
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                strcpy( (char *)tempBffr2, "<STATUS>ST_TCT_ACK</STATUS>");
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                            }
                            sprintf( (char *)tempBffr2, "\r\n     COMPLETE.\r\n" );
                          } // EndElse (Size > 4)
                        } // EndElse (tempBffr[3]!=':')
                        break;
                        //------------------ TCI Command: Calibration Initialize Cal Table(Reset)
                      case 'I':
                        Status = RoadBrd_CAL_InitializeFrmFlash();
                        if (Status != HAL_OK)
                        {
                          // Is this a BLE Operation?
                          if ( BLE_Flag )
                          {
                            // Yes...Build and Send BLE Response NOW.
                            strcpy( (char *)tempBffr2, "<STATUS>CMD_TCI_ERR</STATUS>");
                            BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                          }
                          return Status;
                        }
                        else
                        {
                          // Is this a BLE Operation?
                          if ( BLE_Flag )
                          {
                            // Yes...Build and Send BLE Response NOW.
                            strcpy( (char *)tempBffr2, "<STATUS>ST_TCI_ACK</STATUS>");
                            BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                          }
                        }
                        sprintf( (char *)tempBffr2, "\r\n     COMPLETE.\r\n" );
                        break;
                      } //EndSwitch
                    } //EndElse (Size == 2)
                    break;
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
//++++++++++++++++++++++++++++++++++++++++++  Key Flash Variable Commands.
                  case 'K':
                    // Key Flash Variable Commands.
                    // Test Size to make sure we have enough Characters for this operation
                    Status = HAL_OK;
                    if (Size < 4)
                    {
                      // Is this a BLE Operation?
                      if ( BLE_Flag )
                      {
                        // Yes...Build and Send BLE Response NOW.
                        strcpy( (char *)tempBffr2, "<STATUS>CMD_TK_SYNTAX</STATUS>");
                        BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                      }
                      strcpy( (char *)tempBffr2, "TK SYNTAX ERROR: Not correct format.\r\n");
                    }
                    else
                    {
                      switch( tempBffr[2] )
                      {
//------------------
                        case 'S':
                          //Key Flash Variable Set Command.
                          switch( tempBffr[3] )
                          {
//------------------
                            case 'R':
                              //Key Flash Variable Set Road Sound Sample Rate Command.
                              // Step 1. Validate format.
                              if(tempBffr[4]!=':')
                              {
                                // Is this a BLE Operation?
                                if ( BLE_Flag )
                                {
                                  // Yes...Build and Send BLE Response NOW.
                                  strcpy( (char *)tempBffr2, "<STATUS>CMD_TKSR_SYNTAX</STATUS>");
                                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                }
                                strcpy( (char *)tempBffr2, "TKSR SYNTAX ERROR: Not correct format.\r\n");
                              } // Endif (tempBffr[4]!=':')
                              else
                              {
                                // 2. Verify if remaining string is digits
                                if (Size > 5)
                                {
                                  flag = 1;
                                  for (x=5; x< Size; x++)
                                  {
                                    if (isdigit(tempBffr[x]) == 0)
                                      flag = 0;
                                  }
                                } // EndIf (Size > 5)
                                else
                                  flag = 0;
                                if (flag == 0)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>CMD_TKSR_BADPARAM</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  strcpy( (char *)tempBffr2, "TKSR SYNTAX ERROR: Bad Parameter.\r\n");
                                }
                                else
                                {
                                  // 3. Grab remaining string and convert to integer.
                                  tempPstr = &tempBffr[5];
                                  strcpy(tempstr, tempPstr);
                                  new_value = atoi( tempstr );
                                  if((new_value > 9999) ||
                                     (new_value < 0))
                                  {
                                    // Is this a BLE Operation?
                                    if ( BLE_Flag )
                                    {
                                      // Yes...Build and Send BLE Response NOW.
                                      strcpy( (char *)tempBffr2, "<STATUS>CMD_TKSR_BADPARAM</STATUS>");
                                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                    }
                                    strcpy( (char *)tempBffr2, "TKSR SYNTAX ERROR: Bad Parameter.\r\n");
                                  }
                                  else
                                  {
                                    // Time to set new Road Sound Sample Rate.
                                    RoadBrd_Set_RdSndTickCnt( new_value );
                                    // NOW, Build Data String..
                                    // Is this a BLE Operation?
                                    if ( BLE_Flag )
                                    {
                                      // Yes...Build and Send BLE Response NOW.
                                      strcpy( (char *)tempBffr2, "<STATUS>ST_TKSR_ACK</STATUS>");
                                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                    }
                                    sprintf( (char *)tempBffr2, "COMPLETE" );
                                  } // EndElse ((new_value > 9999) || (new_value < 0))
                                } // EndElse (flag == 0)
                              } // EndElse (tempBffr[4]!=':')
                              break;
//------------------
                            case 'S':
                              //Key Flash Variable Set Sensor Sample Rate Command.
                              // Step 1. Validate format.
                              if(tempBffr[4]!=':')
                              {
                                // Is this a BLE Operation?
                                if ( BLE_Flag )
                                {
                                  // Yes...Build and Send BLE Response NOW.
                                  strcpy( (char *)tempBffr2, "<STATUS>CMD_TKSS_SYNTAX</STATUS>");
                                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                }
                                strcpy( (char *)tempBffr2, "TKSS SYNTAX ERROR: Not correct format.\r\n");
                              } // Endif (tempBffr[4]!=':')
                              else
                              {
                                // 2. Verify if remaining string is digits
                                if (Size > 5)
                                {
                                  flag = 1;
                                  for (x=5; x< Size; x++)
                                  {
                                    if (isdigit(tempBffr[x]) == 0)
                                      flag = 0;
                                  }
                                } // EndIf (Size > 5)
                                else
                                  flag = 0;
                                if (flag == 0)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>CMD_TKSS_BADPARAM</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  strcpy( (char *)tempBffr2, "TKSS SYNTAX ERROR: Bad Parameter.\r\n");
                                }
                                else
                                {
                                  // 3. Grab remaining string and convert to integer.
                                  tempPstr = &tempBffr[5];
                                  strcpy(tempstr, tempPstr);
                                  new_value = atoi( tempstr );
                                  if((new_value > 9999) ||
                                     (new_value < 0))
                                  {
                                    // Is this a BLE Operation?
                                    if ( BLE_Flag )
                                    {
                                      // Yes...Build and Send BLE Response NOW.
                                      strcpy( (char *)tempBffr2, "<STATUS>CMD_TKSS_BADPARAM</STATUS>");
                                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                    }
                                    strcpy( (char *)tempBffr2, "TKSS SYNTAX ERROR: Bad Parameter.\r\n");
                                  }
                                  else
                                  {
                                    // Time to set new Road Sound Sample Rate.
                                    RoadBrd_Set_SnsrTickCnt( new_value );
                                    // Is this a BLE Operation?
                                    if ( BLE_Flag )
                                    {
                                      // Yes...Build and Send BLE Response NOW.
                                      strcpy( (char *)tempBffr2, "<STATUS>ST_TKSS_ACK</STATUS>");
                                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                    }
                                    // NOW, Build Data String..
                                    sprintf( (char *)tempBffr2, "COMPLETE" );
                                  } // EndElse ((new_value > 9999) || (new_value < 0))
                                } // EndElse (flag == 0)
                              } // EndElse (tempBffr[4]!=':')
                              break;
//------------------
                            case 'T':
                              //Key Flash Variable Set TACK Limit(Multiple of Road Sound Throttles).
                              // Step 1. Validate format.
                              if(tempBffr[4]!=':')
                              {
                                // Is this a BLE Operation?
                                if ( BLE_Flag )
                                {
                                  // Yes...Build and Send BLE Response NOW.
                                  strcpy( (char *)tempBffr2, "<STATUS>CMD_TKST_SYNTAX</STATUS>");
                                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                }
                                strcpy( (char *)tempBffr2, "TKST SYNTAX ERROR: Not correct format.\r\n");
                              } // Endif (tempBffr[4]!=':')
                              else
                              {
                                // 2. Verify if remaining string is digits
                                if (Size > 5)
                                {
                                  flag = 1;
                                  for (x=5; x< Size; x++)
                                  {
                                    if (isdigit(tempBffr[x]) == 0)
                                      flag = 0;
                                  }
                                } // EndIf (Size > 5)
                                else
                                  flag = 0;
                                if (flag == 0)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>CMD_TKST_BADPARAM</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  strcpy( (char *)tempBffr2, "TKST SYNTAX ERROR: Bad Parameter.\r\n");
                                }
                                else
                                {
                                  // 3. Grab remaining string and convert to integer.
                                  tempPstr = &tempBffr[5];
                                  strcpy(tempstr, tempPstr);
                                  new_value = atoi( tempstr );
                                  if((new_value > 9999) ||
                                     (new_value < 0))
                                  {
                                    // Is this a BLE Operation?
                                    if ( BLE_Flag )
                                    {
                                      // Yes...Build and Send BLE Response NOW.
                                      strcpy( (char *)tempBffr2, "<STATUS>CMD_TKST_BADPARAM</STATUS>");
                                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                    }
                                    strcpy( (char *)tempBffr2, "TKSS SYNTAX ERROR: Bad Parameter.\r\n");
                                  }
                                  else
                                  {
                                    // Time to set new TACK Limit.
                                    RoadBrd_Set_TackLimit( new_value );
                                    // Is this a BLE Operation?
                                    if ( BLE_Flag )
                                    {
                                      // Yes...Build and Send BLE Response NOW.
                                      strcpy( (char *)tempBffr2, "<STATUS>ST_TKST_ACK</STATUS>");
                                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                    }
                                    // NOW, Build Data String..
                                    sprintf( (char *)tempBffr2, "COMPLETE" );
                                  } // EndElse ((new_value > 9999) || (new_value < 0))
                                } // EndElse (flag == 0)
                              } // EndElse (tempBffr[4]!=':')
                              break;
//------------------
                            case 'B':
                              //Key Flash Variable Set Boot Delay(Seconds).
                              // Step 1. Validate format.
                              if(tempBffr[4]!=':')
                              {
                                // Is this a BLE Operation?
                                if ( BLE_Flag )
                                {
                                  // Yes...Build and Send BLE Response NOW.
                                  strcpy( (char *)tempBffr2, "<STATUS>CMD_TKSB_SYNTAX</STATUS>");
                                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                }
                                strcpy( (char *)tempBffr2, "TKSB SYNTAX ERROR: Not correct format.\r\n");
                              } // Endif (tempBffr[4]!=':')
                              else
                              {
                                // 2. Verify if remaining string is digits
                                if (Size > 5)
                                {
                                  flag = 1;
                                  for (x=5; x< Size; x++)
                                  {
                                    if (isdigit(tempBffr[x]) == 0)
                                      flag = 0;
                                  }
                                } // EndIf (Size > 5)
                                else
                                  flag = 0;
                                if (flag == 0)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>CMD_TKSB_BADPARAM</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  strcpy( (char *)tempBffr2, "TKSB SYNTAX ERROR: Bad Parameter.\r\n");
                                }
                                else
                                {
                                  // 3. Grab remaining string and convert to integer.
                                  tempPstr = &tempBffr[5];
                                  strcpy(tempstr, tempPstr);
                                  new_value = atoi( tempstr );
                                  if((new_value > 999) ||
                                     (new_value < 0))
                                  {
                                    // Is this a BLE Operation?
                                    if ( BLE_Flag )
                                    {
                                      // Yes...Build and Send BLE Response NOW.
                                      strcpy( (char *)tempBffr2, "<STATUS>CMD_TKSB_BADPARAM</STATUS>");
                                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                    }
                                    strcpy( (char *)tempBffr2, "TKSB SYNTAX ERROR: Bad Parameter.\r\n");
                                  }
                                  else
                                  {
                                    // Time to set new Boot Delay.
                                    RoadBrd_Set_BootDelay( new_value );
                                    // Is this a BLE Operation?
                                    if ( BLE_Flag )
                                    {
                                      // Yes...Build and Send BLE Response NOW.
                                      strcpy( (char *)tempBffr2, "<STATUS>ST_TKSB_ACK</STATUS>");
                                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                    }
                                    // NOW, Build Data String..
                                    sprintf( (char *)tempBffr2, "COMPLETE" );
                                  } // EndElse ((new_value > 999) || (new_value < 0))
                                } // EndElse (flag == 0)
                              } // EndElse (tempBffr[4]!=':')
                              break;
//------------------
                            case 'V':
                              //Key Flash Variable Set Version String.
                              // Step 1. Validate format.
                              if(tempBffr[4]!=':')
                              {
                                // Is this a BLE Operation?
                                if ( BLE_Flag )
                                {
                                  // Yes...Build and Send BLE Response NOW.
                                  strcpy( (char *)tempBffr2, "<STATUS>CMD_TKSV_SYNTAX</STATUS>");
                                  BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                }
                                strcpy( (char *)tempBffr2, "TKSV SYNTAX ERROR: Not correct format.\r\n");
                              } // Endif (tempBffr[4]!=':')
                              else
                              {
                                // 2. Verify if remaining string is digits
                                if (Size > 5)
                                {
                                  flag = 1;
                                } // EndIf (Size > 5)
                                else
                                  flag = 0;
                                if (flag == 0)
                                {
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>CMD_TKSV_BADPARAM</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  strcpy( (char *)tempBffr2, "TKSV SYNTAX ERROR: Bad Parameter.\r\n");
                                }
                                else
                                {
                                  // 3. Grab remaining string and convert to integer.
                                  tempPstr = &tempBffr[5];
                                  // Time to set new Version String.
                                  RoadBrd_Set_VersionString( tempPstr );
                                  // Is this a BLE Operation?
                                  if ( BLE_Flag )
                                  {
                                    // Yes...Build and Send BLE Response NOW.
                                    strcpy( (char *)tempBffr2, "<STATUS>ST_TKSV_ACK</STATUS>");
                                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                                  }
                                  // NOW, Build Data String..
                                  sprintf( (char *)tempBffr2, "COMPLETE" );
                                } // EndElse (flag == 0)
                              } // EndElse (tempBffr[4]!=':')
                              break;
                            default:
                              strcpy( (char *)tempBffr2, "TKS ERROR: Not a legal command.\r\n");
                              break;
                          } // EndSwitch ( tempBffr[3] )
                          break;
//------------------
                        case 'R':
                          //Key Flash Variable Read Command
                          switch( tempBffr[3] )
                          {
//------------------
                            case 'R':
                              //Key Flash Variable Read Road Sound Sample Rate Command.
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "<STATUS>ST_TKRR:%3.1f</STATUS>", ((float)RoadBrd_Get_RdSndTickCnt()/10));
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "RdSnd Sample Rate:  %3.1f Seconds.\r\n", ((float)RoadBrd_Get_RdSndTickCnt()/10));
                              break;
//------------------
                            case 'S':
                              //Key Flash Variable Read Sensor Sample Rate Command.
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "<STATUS>ST_TKRS:%3.1f</STATUS>", ((float)RoadBrd_Get_SnsrTickCnt()/10));
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "Sensor Sample Rate: %3.1f Seconds.\r\n", ((float)RoadBrd_Get_SnsrTickCnt()/10));
                              break;
//------------------
                            case 'T':
                              //Key Flash Variable Read TACK Limit(Multiple of Road Sound Throttles).
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "<STATUS>ST_TKRT:%d</STATUS>", RoadBrd_Get_TackLimit());
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "TACK Limit: %d.\r\n", RoadBrd_Get_TackLimit());
                              break;
//------------------
                            case 'B':
                              //Key Flash Variable Read Boot Delay.(Seconds).
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "<STATUS>ST_TKRB:%d</STATUS>", RoadBrd_Get_BootDelay());
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "Boot Delay: %d Seconds.\r\n", RoadBrd_Get_BootDelay());
                              break;
//------------------
                            case 'V':
                              //Key Flash Variable Read Version String.
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                sprintf( (char *)tempBffr2, "<STATUS>ST_TKRB:%s</STATUS>", RoadBrd_Get_VersionString());
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              sprintf( (char *)tempBffr2, "Version String: %s\r\n", RoadBrd_Get_VersionString());
                              break;
                            default:
                              // Is this a BLE Operation?
                              if ( BLE_Flag )
                              {
                                // Yes...Build and Send BLE Response NOW.
                                strcpy( (char *)tempBffr2, "<STATUS>CMD_TKR_SYNTAX</STATUS>");
                                BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                              }
                              strcpy( (char *)tempBffr2, "TKR ERROR: Not a legal command.\r\n");
                              break;
                          } // EndSwitch ( tempBffr[3] )
                          break;
                          // Is this a BLE Operation?
                        default:
                          if ( BLE_Flag )
                          {
                            // Yes...Build and Send BLE Response NOW.
                            strcpy( (char *)tempBffr2, "<STATUS>CMD_TK_SYNTAX</STATUS>");
                            BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                          }
                          strcpy( (char *)tempBffr2, "ERROR: Not a legal command.\r\n");
                          break;
                      } //EndSwitch ( tempBffr[2] )
                    } //EndElse (Size < 3)
                    break;
//++++++++++++++++++++++++++++++++++++++++++  Units Enable/Disable Commands.
                  case 'U':
                    // Key Flash Variable Commands.
                    // Test Size to make sure we have enough Characters for this operation
                    Status = HAL_OK;
                    if (Size < 3)
                    {
                      // Is this a BLE Operation?
                      if ( BLE_Flag )
                      {
                        // Yes...Build and Send BLE Response NOW.
                        strcpy( (char *)tempBffr2, "<STATUS>CMD_TU_SYNTAX</STATUS>");
                        BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                      }
                      strcpy( (char *)tempBffr2, "TU SYNTAX ERROR: Not correct format.\r\n");
                    }
                    else
                    {
                      switch( tempBffr[2] )
                      {
//------------------
                        case 'E':
                          //Units Enable Command.
                          // Is this a BLE Operation?
                          if ( BLE_Flag )
                          {
                            // Yes...Build and Send BLE Response NOW.
                            strcpy( (char *)tempBffr2, "<STATUS>ST_TUE_ACK</STATUS>");
                            BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                          }
                          sprintf( (char *)tempBffr2, "Units XML State CHANGED: ENABLED\r\n\r\n> ");
                          Status = RoadBrd_Set_UnitsFlag( true );
                          break;
//------------------
                        case 'D':
                          //Units Disable Command
                          // Is this a BLE Operation?
                          if ( BLE_Flag )
                          {
                            // Yes...Build and Send BLE Response NOW.
                            strcpy( (char *)tempBffr2, "<STATUS>ST_TUD_ACK</STATUS>");
                            BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                          }
                          sprintf( (char *)tempBffr2, "Units XML State CHANGED: DISABLED\r\n\r\n> ");
                          Status = RoadBrd_Set_UnitsFlag( false );
                          break;
                        default:
                          // Is this a BLE Operation?
                          if ( BLE_Flag )
                          {
                            // Yes...Build and Send BLE Response NOW.
                            strcpy( (char *)tempBffr2, "<STATUS>CMD_TU_SYNTAX</STATUS>");
                            BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                          }
                          strcpy( (char *)tempBffr2, "TU SYNTAX ERROR: Not a legal command.\r\n");
                          break;
                      } //EndSwitch ( tempBffr[2] )
                    } //EndElse (Size < 3)
                    break;
//++++++++++++++++++++++++++++++++++++++++++  Special Monitor Mode to intercept traffic from UART and pass to BGM111
                  case 'M':
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    
                    strcpy( (char *)tempBffr2, "BGM111 MONITOR MODE.\r\n\r\n");
#ifdef NUCLEO
                    Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                    Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                    if (Status != HAL_OK)
                      return Status;
                    strcpy( (char *)tempBffr2, "Use <ESC> to exit mode.\r\n\r\n");
                    // Set Bypass Flag
                    Bypass = true;
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

//++++++++++++++++++++++++++++++++++++++++++  Special Monitor Mode to intercept traffic from UART and pass to BGM111
                  case 'B':
                    // Is this a BLE Operation?
                    if ( BLE_Flag )
                    {
                      // Yes...Build and Send BLE Response NOW.
                      strcpy( (char *)tempBffr2, "<STATUS>ST_OTAMD_ACK</STATUS>");
                      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                    }
                    
                    strcpy( (char *)tempBffr2, "Boot Loader MONITOR MODE.\r\n\r\n");
#ifdef NUCLEO
                    Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                    Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                    if (Status != HAL_OK)
                      return Status;
                    strcpy( (char *)tempBffr2, "Use <ESC> followed by <CR> to exit mode.\r\n\r\n");
                    // Set Bypass Flag
                    Set_Boot_Bypass();
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
#if 0
//**************************************************************************************************
            case 'S':
              // Sleep Mode. 
              switch( tempBffr[1] )
              {
//++++++++++++++++++++++++++++++++++++++++++  5V Power Supply Commands.
                case 'S':
                  // Is this a BLE Operation?
                  if ( BLE_Flag )
                  {
                    // Yes...Build and Send BLE Response NOW.
                    strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                  }
                  
                  strcpy( (char *)tempBffr2, "Micro in Sleep Mode NOW.\r\n");
#ifdef NUCLEO
                  Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                  Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                  if (Status != HAL_OK)
                    return Status;
                  // Sleep Micro NOW!
                  //sleep();
                  break;
                case 'D':
                  // Is this a BLE Operation?
                  if ( BLE_Flag )
                  {
                    // Yes...Build and Send BLE Response NOW.
                    strcpy( (char *)tempBffr2, "<STATUS>CMD_NOSUPPORT</STATUS>");
                    BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), (uint8_t *)tempBffr2);
                  }
                  
                  // Turn off 5V Power Supply.
                  strcpy( (char *)tempBffr2, "Micro in Deep Sleep Mode NOW.\r\n");
#ifdef NUCLEO
                  Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                  Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                  if (Status != HAL_OK)
                    return Status;
                  // Sleep Micro NOW!
                  //deepsleep();
                  break;
              }
              break;
#endif
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
    } // EndElse ( Bypass )
  return Status;
}

/**
  * @brief  This function tests the passed string to make sure it is Hex format.
  * @param  char *ptr: Ptr to string to be tested. NULL terminated
  * @retval int: 1: String is HEX.
  *              0:   String is not HEX.
  */
int isHexNum(char *ptr)
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
}

/**
  * @brief  This function converts the passed Hex String to an Integer value.
  * @param  char *ptr: Ptr to string to be converted. NULL terminated
  * @retval int: -1: Error in String
  *              Value converted returned.
  */
int hatoi( char *ptr )
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
}


bool Tst_Bypass( void)
{
  return Bypass;
}

/*void sleep(void) {
//    TimMasterHandle.Instance = TIM5;
 
    // Disable HAL tick interrupt
//    __HAL_TIM_DISABLE_IT(&TimMasterHandle, TIM_IT_CC2);
 
    // Request to enter SLEEP mode
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
 
    // Enable HAL tick interrupt
//    __HAL_TIM_ENABLE_IT(&TimMasterHandle, TIM_IT_CC2);
}*/
 
/*void deepsleep(void) {
    // Request to enter STOP mode with regulator in low power mode
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
 
    // After wake-up from STOP reconfigure the PLL
    //SetSysClock();
    SystemClock_Config();
}*/

/************************ (C) COPYRIGHT WeatherCloud *****END OF FILE****/
