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

/* Parser function */

/**
  * @brief  This routine parses the passed string and performs the passed operation
  * @param  *tempBffr: String to be parsed.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_ParseString(char *tempBffr)
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
  float Temp_C, Temp_F, Shunt_V, Bus_V, Crrnt, Power;
  uint32_t Err_code;
#else
  #ifdef TEST
    #define RECEIVE_SZ      5
  #else
    #define RECEIVE_SZ      30
    uint16_t DriverStatus;
    uint8_t tempBffr2[120];
    uint8_t tempBffr3[10];
    uint8_t* BufferPntr;
    HAL_StatusTypeDef Status, Save_Status;
    uint8_t Size;
    int Address;
    int num_bytes;
    int num_bytes_received;
    uint8_t i2cData[80];
    int Error, x, y;
    Voltage VMeasure;
    Current CMeasure;
    Power PMeasure;
    Temperature TMeasure;
    Humidity HMeasure;
    RGBInitialize RGBMeasure;
    RGBIdent IDMeasure;
    RGBStatus RGBSMeasure;
    RGBLight RGBValues;
//    PRStatus PRMeasure;
    PRPressure PRPMeasure;
    BinString RSFFTBins;
    GridEye     GridMeasure;
    uint32_t Err_code;
    uint8_t op_mode, ds_range, adc_rsl, sync, cmp_adjst, cmp_offst, int_assgn, int_persist, cnvrsn_int;
  #endif
#endif

    Size = strlen((char *)tempBffr);
    
            // We have a good Tasking String. Time to determine action.
            switch( tempBffr[0] )
            {
//**************************************************************************************************
            case '0':
              // Enable Road sound and fill buffer. 
              Status = RoadBrdSnd_ProcessSound();
              if (Status == HAL_OK)
              {
                strcpy( (char *)tempBffr2, "Road Sound: Road Sound loaded, processed, and sent to FFT Bins.\r\n");;
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
              // NOW, Build Data String..
              y=0;
              sprintf( (char *)tempBffr2, "" );
              for (x=0; x<FFT_BUFFER_SIZE; x++)
              {
                sprintf( (char *)tempBffr3, "%02x ", BufferPntr[x]);
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
              sprintf( (char *)tempBffr2, "     ---COMPLETE---" );
              break;
//**************************************************************************************************
            case '6':
              // Clear all buffers. 
              Status = HAL_OK;
              RoadBrdSnd_ClrBffrs();
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
                      sprintf( (char *)tempBffr2, " PRESSURE: " );
                      strcat( (char *)tempBffr2, (char *)PRPMeasure.Pressure );
                      strcat( (char *)tempBffr2, "\r\n" );
#ifdef NUCLEO
                      Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);                   
#else
                      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
                      if (Status != HAL_OK)
                        return Status;
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
                }
                else if ( Get_DriverStates( COOLEYE_MNTR_TASK ))
                {
                  Status = RoadBrd_CoolEye_ReadValues( &GridMeasure );
                }
                else
                  Status = HAL_ERROR;
                
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
                        sprintf( (char *)tempBffr2, "     Thermistor DATA: %s   %s   %s   %d\r\n", GridMeasure.Thermistor.Raw,
                                                                                               GridMeasure.Thermistor.TempC,
                                                                                               GridMeasure.Thermistor.TempF,
                                                                                               GridMeasure.Thermistor.RawC );
                        break;
                      case 1: //GridEye1 Values
                        sprintf( (char *)tempBffr2, "     Grid 33 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye1.Raw,
                                                                                               GridMeasure.GridEye1.TempC,
                                                                                               GridMeasure.GridEye1.TempF,
                                                                                               GridMeasure.GridEye1.RawC );
                        break;
                      case 2: //GridEye2 Values
                        sprintf( (char *)tempBffr2, "     Grid 34 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye2.Raw,
                                                                                               GridMeasure.GridEye2.TempC,
                                                                                               GridMeasure.GridEye2.TempF,
                                                                                               GridMeasure.GridEye2.RawC );
                        break;
                      case 3: //GridEye3 Values
                        sprintf( (char *)tempBffr2, "     Grid 35 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye3.Raw,
                                                                                               GridMeasure.GridEye3.TempC,
                                                                                               GridMeasure.GridEye3.TempF,
                                                                                               GridMeasure.GridEye3.RawC );
                        break;
                      case 4: //GridEye4 Values
                        sprintf( (char *)tempBffr2, "     Grid 36 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye4.Raw,
                                                                                               GridMeasure.GridEye4.TempC,
                                                                                               GridMeasure.GridEye4.TempF,
                                                                                               GridMeasure.GridEye4.RawC );
                        break;
                      case 5: //GridEye5 Values
                        sprintf( (char *)tempBffr2, "     Grid 37 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye5.Raw,
                                                                                               GridMeasure.GridEye5.TempC,
                                                                                               GridMeasure.GridEye5.TempF,
                                                                                               GridMeasure.GridEye5.RawC );
                        break;
                      case 6: //GridEye6 Values
                        sprintf( (char *)tempBffr2, "     Grid 38 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye6.Raw,
                                                                                               GridMeasure.GridEye6.TempC,
                                                                                               GridMeasure.GridEye6.TempF,
                                                                                               GridMeasure.GridEye6.RawC );
                        break;
                      case 7: //GridEye7 Values
                        sprintf( (char *)tempBffr2, "     Grid 39 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye7.Raw,
                                                                                               GridMeasure.GridEye7.TempC,
                                                                                               GridMeasure.GridEye7.TempF,
                                                                                               GridMeasure.GridEye7.RawC );
                        break;
                      case 8: //GridEye8 Values
                        sprintf( (char *)tempBffr2, "     Grid 40 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye8.Raw,
                                                                                               GridMeasure.GridEye8.TempC,
                                                                                               GridMeasure.GridEye8.TempF,
                                                                                               GridMeasure.GridEye8.RawC );
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
              }
              else
              {
                switch( tempBffr[1] )
                {
//------------------ BI Command: Initialize Cool Eye/Grid Eye Sensor
                  case 'I':
                    // Initialize Cool Eye/Grid Eye Sensor.
                    Status = RoadBrd_GridEyeInit();
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
                    }
                    else if ( Get_DriverStates( COOLEYE_MNTR_TASK ))
                    {
                      Status = RoadBrd_CoolEye_ReadValues( &GridMeasure );
                    }
                    else
                      Status = HAL_ERROR;

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
                          case 0: //Thermistor Value
                            sprintf( (char *)tempBffr2, "     Thermistor DATA: %s   %s   %s   %d\r\n", GridMeasure.Thermistor.Raw,
                                                                                                   GridMeasure.Thermistor.TempC,
                                                                                                   GridMeasure.Thermistor.TempF,
                                                                                                   GridMeasure.Thermistor.RawC );
                            break;
                          case 1: //GridEye1 Values
                            sprintf( (char *)tempBffr2, "     Grid 33 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye1.Raw,
                                                                                               GridMeasure.GridEye1.TempC,
                                                                                               GridMeasure.GridEye1.TempF,
                                                                                               GridMeasure.GridEye1.RawC );
                            break;
                          case 2: //GridEye2 Values
                            sprintf( (char *)tempBffr2, "     Grid 34 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye2.Raw,
                                                                                               GridMeasure.GridEye2.TempC,
                                                                                               GridMeasure.GridEye2.TempF,
                                                                                               GridMeasure.GridEye2.RawC );
                            break;
                          case 3: //GridEye3 Values
                            sprintf( (char *)tempBffr2, "     Grid 35 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye3.Raw,
                                                                                               GridMeasure.GridEye3.TempC,
                                                                                               GridMeasure.GridEye3.TempF,
                                                                                               GridMeasure.GridEye3.RawC );
                            break;
                          case 4: //GridEye4 Values
                            sprintf( (char *)tempBffr2, "     Grid 36 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye4.Raw,
                                                                                               GridMeasure.GridEye4.TempC,
                                                                                               GridMeasure.GridEye4.TempF,
                                                                                               GridMeasure.GridEye4.RawC );
                            break;
                          case 5: //GridEye5 Values
                            sprintf( (char *)tempBffr2, "     Grid 37 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye5.Raw,
                                                                                               GridMeasure.GridEye5.TempC,
                                                                                               GridMeasure.GridEye5.TempF,
                                                                                               GridMeasure.GridEye5.RawC );
                            break;
                          case 6: //GridEye6 Values
                            sprintf( (char *)tempBffr2, "     Grid 38 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye6.Raw,
                                                                                               GridMeasure.GridEye6.TempC,
                                                                                               GridMeasure.GridEye6.TempF,
                                                                                               GridMeasure.GridEye6.RawC );
                            break;
                          case 7: //GridEye7 Values
                            sprintf( (char *)tempBffr2, "     Grid 39 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye7.Raw,
                                                                                               GridMeasure.GridEye7.TempC,
                                                                                               GridMeasure.GridEye7.TempF,
                                                                                               GridMeasure.GridEye7.RawC );
                            break;
                          case 8: //GridEye8 Values
                            sprintf( (char *)tempBffr2, "     Grid 40 DATA: %s   %s   %s   %d\r\n", GridMeasure.GridEye8.Raw,
                                                                                               GridMeasure.GridEye8.TempC,
                                                                                               GridMeasure.GridEye8.TempF,
                                                                                               GridMeasure.GridEye8.RawC );
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
                sprintf( (char *)tempBffr2, "     Bus Voltage: " );
                strcat( (char *)tempBffr2, (char *)VMeasure.Voltage );
                strcat( (char *)tempBffr2, "\r\n" );
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
                    sprintf( (char *)tempBffr2, "     Shunt Voltage: " );
                    strcat( (char *)tempBffr2, (char *)VMeasure.Voltage );
                    strcat( (char *)tempBffr2, "\r\n" );
                    break;
//------------------ C1 Command...Read Current and return results..... 
                  case '1':
                    // Read Current and return results.....
                    Status = RoadBrd_VMonitor_RdCurrent( &CMeasure );
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
                    sprintf( (char *)tempBffr2, "     Current: " );
                    strcat( (char *)tempBffr2, (char *)CMeasure.Current );
                    strcat( (char *)tempBffr2, "\r\n" );
                    break;
//------------------ C2 Command...Read Power and return results.....     
                  case '2':
                    // Read Power and return results.....
                    Status = RoadBrd_VMonitor_RdPower( &PMeasure );
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
                    sprintf( (char *)tempBffr2, "     Power: " );
                    strcat( (char *)tempBffr2, (char *)PMeasure.Power );
                    strcat( (char *)tempBffr2, "\r\n" );
                    break;
//------------------ C3 Command...Read Bus Voltage and return results.....
                  case '3':
                    // Read Bus Voltage and return results.....
                    Status = RoadBrd_VMonitor_RdVoltage( &VMeasure );
                      
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
                    sprintf( (char *)tempBffr2, "     Bus Voltage: " );
                    strcat( (char *)tempBffr2, (char *)VMeasure.Voltage );
                    strcat( (char *)tempBffr2, "\r\n" );
                    break;
                  default:
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
                sprintf( (char *)tempBffr2, "     Humidity: " );
                strcat( (char *)tempBffr2, (char *)HMeasure.Humidity );
                strcat( (char *)tempBffr2, "\r\n" );
              }
              else
              {
                switch( tempBffr[1] )
                {
//------------------ DI Command: Initialize Humidity Sensor
                  case 'I':
                    // Initialize Humidity Sensor.
                    Status = RoadBrd_HumidityInit();
                    if (Status == HAL_OK)
                    {
                      strcpy( (char *)tempBffr2, "Humidity Sensor: Initialization Complete.\r\n");;
                    }
                    break;
//------------------ D0 Command...Read Humidity Values.....
                  case '0':
                    // Read Humidity Sensor sensor and return Humidity results....
                    Status = RoadBrd_Humidity_ReadHumidity( &HMeasure );
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
              strcpy( (char *)tempBffr2, "TBD: Read Temp and Pressure NOT IMPLEMENTED\r\n");
              break;
//**************************************************************************************************
            case 'F':
              // NO ACTION. 
              strcpy( (char *)tempBffr2, "NO Action...(0x00).\r\n");
              break;
//**************************************************************************************************
            case 'G':
              // Read Temperature sensor and return results....Temperature Sensor U10(PCT2075GVJ).  Addr: 0x94
              Status = RoadBrd_ReadTemp( &TMeasure );
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
              // Now calculate Celcius and Farenheit Temp.
              sprintf( (char *)tempBffr2, "     TempC: " );
              strcat( (char *)tempBffr2, (char *)TMeasure.TempC );
              strcat( (char *)tempBffr2, "     TempF: " );
              strcat( (char *)tempBffr2, (char *)TMeasure.TempF );
              strcat( (char *)tempBffr2, "\r\n" );
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

                      if (Status == HAL_OK)
                      {
                        strcpy( (char *)tempBffr2, "RGB Color Light Sensor: Initialization Complete with DEFAULT Values.\r\n");;
                      }
                      
                    }
                    else
                    {
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
                strcpy( (char *)tempBffr2, "T ERROR: Not a legal command.\r\n");
              else
              {
                switch( tempBffr[1] )
                {
//++++++++++++++++++++++++++++++++++++++++++  I2C Commands.
                  case 'I':
                    // I2C Commands.
                    // Test Size to make sure we have enough Characters for this operation
                    if (Size < 9)
                    strcpy( (char *)tempBffr2, "TI SYNTAX ERROR: Not correct format.\r\n");
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
                            strcpy( (char *)tempBffr2, "TIS SYNTAX ERROR: Not correct format.\r\n");
                          }
                          else
                          {
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
                            strcpy( (char *)tempBffr2, "TIR SYNTAX ERROR: Not correct format.\r\n");
                          }
                          else
                          {
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
                            strcpy( (char *)tempBffr2, "TIQ SYNTAX ERROR: Not correct format.\r\n");
                          }
                          else
                          {
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
//++++++++++++++++++++++++++++++++++++++++++  Dump Calibration Settings.
                  case 'C':
                    // Read Cool Eye/Grid Eye Values.....
                    if ( Get_DriverStates( GRIDEYE_MNTR_TASK ))
                    {
                      Status = RoadBrd_GridEye_ReadValues( &GridMeasure );
                    }
                    else if ( Get_DriverStates( COOLEYE_MNTR_TASK ))
                    {
                      Status = RoadBrd_CoolEye_ReadValues( &GridMeasure );
                    }
                    else
                      Status = HAL_ERROR;
                
                    if (Status == HAL_OK)
                    {
                      // OK Next Sensor.
                      // Read Temperature sensor and return results....Temperature Sensor U10(PCT2075GVJ).  Addr: 0x94
                      Status = RoadBrd_ReadTemp( &TMeasure );
                      if (Status == HAL_OK)
                      {
                        // OK Next Sensor.
                        // Read Humidity Sensor sensor and return Humidity results....
                        Status = RoadBrd_Humidity_ReadHumidity( &HMeasure );
                        if (Status == HAL_OK)
                        {
                          // OK Next Sensor.
                          //Status = RoadBrd_Barometer_Status( &PRMeasure );
                          Status = RoadBrd_Baro_ReadPressure( &PRPMeasure );
                          if (Status == HAL_OK)
                          {
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
                            sprintf( (char *)tempBffr2, "Pressure TASKING ERROR!" );
                          } // EndElse (Status == HAL_OK) RoadBrd_Baro_ReadPressure
                        } // Endif (Status == HAL_OK) RoadBrd_Humidity_ReadHumidity
                        else
                        {
                          sprintf( (char *)tempBffr2, "Humidity TASKING ERROR!" );
                        } // EndElse (Status == HAL_OK) RoadBrd_Humidity_ReadHumidity
                      } // Endif (Status == HAL_OK) RoadBrd_ReadTemp
                      else
                      {
                        sprintf( (char *)tempBffr2, "AMBIENT TEMPERATURE TASKING ERROR!" );
                      } // EndElse (Status == HAL_OK) RoadBrd_ReadTemp
                    } // Endif (Status == HAL_OK) RoadBrd_CoolEye_ReadValues
                    else
                    {
                       sprintf( (char *)tempBffr2, "GRID EYE/COOL EYE TASKING ERROR!" );
                    } // EndElse (Status == HAL_OK) RoadBrd_CoolEye_ReadValues

                    break;
//++++++++++++++++++++++++++++++++++++++++++  Dump Driver State.
                  case 'D':
                    // Read Driver Status
                    DriverStatus = Get_DriverStatus();
                    sprintf( (char *)tempBffr2, "Driver Status: %04x\r\n", DriverStatus );
                    break;
//++++++++++++++++++++++++++++++++++++++++++  RESET Micro.
                  case 'R':
                    // Read Driver Status
                    RdBrd_BlinkErrCd( ERROR_I2CBUSY );
                    HAL_NVIC_SystemReset();
                    sprintf( (char *)tempBffr2, "RESET CALLED BUT NO RESPONSE!!\r\n" );
                    break;
//**************************************************************************************************
                  default:
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
            default:
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
