/* Application data manager implementation
 * Takes care of scheduling sample times and formatting the sensor data
 * to be sent as BLE characteristics to the app. */

#include "app_data.h"
#include "stm32l1xx_hal.h"
#include "VMonitor.h"
#include "Temperature.h"
#include "Humidity.h"
#include "RGBLight.h"
#include "barometer.h"
#include "GridEye.h"
#include <stdio.h>
#include <string.h>
#include "ErrorCodes.h"
#include "wwdg.h"
#include "tim.h"
#include <time.h>

/* Characteristic handles */
/*
#ifdef LEGACY_PATCH
#define gattdb_xgatt_temp0                      8
#define gattdb_xgatt_temp1                     12
#define gattdb_xgatt_temp2                     16
#define gattdb_xgatt_temp3                     20
#define gattdb_xgatt_temp4                     24
#define gattdb_xgatt_temp5                     28
#define gattdb_xgatt_temp6                     32
#define gattdb_xgatt_temp7                     36
#define gattdb_xgatt_temp8                     40
#define gattdb_xgatt_fuel                      44
#define gattdb_xgatt_humidity                  48
#define gattdb_xgatt_barometer                 52
#define gattdb_xgatt_lightning                 56
#define gattdb_xgatt_rev                       60
#define gattdb_xgatt_fft1                      64
#define gattdb_xgatt_fft2                      68
#define gattdb_xgatt_fft3                      72
#define gattdb_xgatt_fft4                      76
#else
#define gattdb_ShntVltg                         8
#define gattdb_Current                         12
#define gattdb_Power                           16
#define gattdb_Voltage                         20
#define gattdb_TemperatureC                    24
#define gattdb_TemperatureF                    28
#define gattdb_Pressure                        32
#define gattdb_PrTemperatureC                  36
#define gattdb_PrTemperatureF                  40
#define gattdb_Humidity                        44
#define gattdb_HmdtyTempC                      48
#define gattdb_HmdtyTempF                      52
#define gattdb_RGBLightRd                      57
#define gattdb_RGBLightGrn                     61
#define gattdb_RGBLightBlu                     65
#define gattdb_RdSound0                        70
#define gattdb_RdSound16                       74
#define gattdb_RdSound32                       78
#define gattdb_RdSound48                       82
#define gattdb_ThermistorC                     87
#define gattdb_Thermal_1RwC                    91
#define gattdb_Thermal_2RwC                    95
#define gattdb_Thermal_3RwC                    99
#define gattdb_Thermal_4RwC                   103
#define gattdb_Thermal_5RwC                   107
#define gattdb_Thermal_6RwC                   111
#define gattdb_Thermal_7RwC                   115
#define gattdb_Thermal_8RwC                   119
#define gattdb_xgatt_temp0                    124
#define gattdb_xgatt_temp1                    128
#define gattdb_xgatt_temp2                    132
#define gattdb_xgatt_temp3                    136
#define gattdb_xgatt_temp4                    140
#define gattdb_xgatt_temp5                    144
#define gattdb_xgatt_temp6                    148
#define gattdb_xgatt_temp7                    152
#define gattdb_xgatt_temp8                    156
#define gattdb_xgatt_fuel                     160
#define gattdb_xgatt_humidity                 164
#define gattdb_xgatt_barometer                168
#define gattdb_xgatt_lightning                172
#define gattdb_xgatt_rev                      176
#define gattdb_xgatt_fft1                     180
#define gattdb_xgatt_fft2                     184
#define gattdb_xgatt_fft3                     188
#define gattdb_xgatt_fft4                     192
#define gattdb_AnlErrCnt                      197
#define gattdb_AnlErrCd                       201
#define gattdb_AnlDevCd                       205
#define gattdb_AnlTickCnt                     209
#define gattdb_AnlHrtBt                       213
#define gattdb_AnlHrtBt2                      217
#endif */
#define gattdb_xgatt_spp_data                   13


// Global Vars to app_data.
static uint16_t connection_cnt = 0;
static uint16_t HeartBeat_Cnt = 0;

// Driver list Structure
struct
{
  bool  VoltageMonitor;
  bool  Temperature;
  bool  Irradiance;
  bool  Pressure;
  bool  Humidity;
  bool  GridEye;
  bool  CoolEye;
  bool  I2CState;
  bool  FrameState;
  bool  CalibrationState;
} static driver_list;

// Analytics Structure
struct
{
  bool  HrtBeat_Flg;
  uint16_t HrtBeat_Cnt;
  uint16_t FrmRpt_Cnt;
  uint8_t CMD_Md_Cnt;
  uint8_t Frame_Ratio;
} static analytics;

/* App data measurment structure */

typedef struct
{
  volatile bool         reading_scheduled;
  volatile bool         RdSnd_Scheduled;
  bool          Legacy_OneTime;
  uint8_t       task_item;
  // Voltage Monitor Data
  Voltage       ShntVltg;
  Current       Current;
  Power         Power;
  Voltage       Voltage;
  // Temperature Data
  Temperature   Temp;
  // Irradiance Data
  RGBLight      RGBValues;
  // Pressure Data.
  PRPressure    Pressure;
  Temperature   PrTemp;
  Humidity      Humidity;
  Temperature   HmTemp;
  // FFT Bin Data
  BinString     FFTBin0;
  BinString     FFTBin16;
  BinString     FFTBin32;
  BinString     FFTBin48;
  GridEye       GridValues;
} dataTmplate;

dataTmplate TmpData;
dataTmplate data;

//* Initialize all sensors */

void InitSensors(void)
{
  data.Legacy_OneTime = true;                   // Clear Legacy One time flag so that we can set key characteristics...once.
  ClrDataStructure();                           // Clear Backup data structure.
  data.task_item = VOLTAGE_MNTR_TASK;           // Initialize the task_item to first item.
  analytics.HrtBeat_Flg = false;                // Set flasg to clear before using it.
  analytics.HrtBeat_Cnt = 0;                    // Clear count before using it.
  analytics.FrmRpt_Cnt = 0;                     // Clear Frame Repeat Count.
  analytics.CMD_Md_Cnt = 0;                     // Clear CMD_Md_Cnt.
  analytics.Frame_Ratio = 0;                    // Clear Frame_Ratio. Measures the ratio of Sensor Frames to Road Sound Frames.
}

void ClrDataStructure(void)
{
  data.Legacy_OneTime = true;                   // Clear Legacy One time flag so that we can set key characteristics...once.
  strcpy( (char *)data.ShntVltg.Raw, "------" );
  strcpy( (char *)data.Current.Raw, "------" );
  strcpy( (char *)data.Power.Raw, "------" );
  strcpy( (char *)data.Voltage.Raw, "------" );
  strcpy( (char *)data.Temp.Raw, "------" );
  strcpy( (char *)data.RGBValues.Raw, "--------------" );
  strcpy( (char *)data.Pressure.Raw, "---------" );
  strcpy( (char *)data.PrTemp.Raw, "------" );
  strcpy( (char *)data.Humidity.HRaw, "------" );
  strcpy( (char *)data.HmTemp.Raw, "------" );
  strcpy( (char *)data.GridValues.Thermistor.Raw, "------" );
  strcpy( (char *)data.GridValues.GridEye1.Raw, "------" );
  strcpy( (char *)data.GridValues.GridEye2.Raw, "------" );
  strcpy( (char *)data.GridValues.GridEye3.Raw, "------" );
  strcpy( (char *)data.GridValues.GridEye4.Raw, "------" );
  strcpy( (char *)data.GridValues.GridEye5.Raw, "------" );
  strcpy( (char *)data.GridValues.GridEye6.Raw, "------" );
  strcpy( (char *)data.GridValues.GridEye7.Raw, "------" );
  strcpy( (char *)data.GridValues.GridEye8.Raw, "------" );
}

/**
  * @brief  Set the reading_scheduled flag to enable Sensor processing.
  * @param  None
  * @retval None
  */
void SetDataReady( void )
{
  /* Schedule a sensor reading */
  data.reading_scheduled = true;
}

/**
  * @brief  Set the RdSnd_Scheduled flag to enable Sensor processing.
  * @param  None
  * @retval None
  */
void SetRdSndReady( void )
{
  /* Schedule a sensor reading */
  data.RdSnd_Scheduled = true;
}

/**
  * @brief  Clear the reading_scheduled flag to enable Sensor processing.
  * @param  None
  * @retval None
  */
void ClrDataReady( void )
{
  /* Schedule a sensor reading */
  data.reading_scheduled = false;
}

/**
  * @brief  Clear the RdSnd_Scheduled flag to enable Sensor processing.
  * @param  None
  * @retval None
  */
void ClrRdSndReady( void )
{
  /* Schedule a sensor reading */
  data.RdSnd_Scheduled = false;
}

/**
  * @brief  Return the state of the Data Ready Flag..
  * @param  None
  * @retval bool data.reading_scheduled
  */
bool TstDataReady( void )
{
  /* Schedule a sensor reading */
  return data.reading_scheduled;
}

/**
  * @brief  Return the state of the RdSnd_Scheduled Flag.
  * @param  None
  * @retval bool data.reading_scheduled
  */
bool TstRdSndReady( void )
{
  /* Schedule a sensor reading */
  return data.RdSnd_Scheduled;
}

/* Process sensor state machine */

HAL_StatusTypeDef  ProcessSensorState(void)
{
#define TMPBUFFER_LNGTH 80
  HAL_StatusTypeDef Status;
  uint8_t tmpBuffer[TMPBUFFER_LNGTH];
  Temperature   Temp;
  int x;
  
  Status = HAL_OK;

  /* Is a reading scheduled? */
  if (data.reading_scheduled)
  {
//************************NEW STYLE TASKING STARTS HERE************************
    // Display State and current Frame Ratio.
    sprintf( (char *)tmpBuffer, "[TI:%d/FR:%d]", 
            data.task_item,
            analytics.Frame_Ratio );
    RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
    // Clear Frame Ratio.
    analytics.Frame_Ratio = 0;
    
    switch(data.task_item)
    {
      case VOLTAGE_MNTR_TASK:
        if ( Get_DriverStates( VOLTAGE_MNTR_TASK ))
        {
          /* Read the Voltage Monitor Data. */
          Status = RoadBrd_VMonitor_RdShntVltg_Scaled( &TmpData.ShntVltg );
          if (Status != HAL_OK)
            break;
          Status = RoadBrd_VMonitor_RdCurrent_Scaled( &TmpData.Current );
          if (Status != HAL_OK)
            break;
          Status = RoadBrd_VMonitor_RdPower_Scaled( &TmpData.Power );
          if (Status != HAL_OK)
            break;
          Status = RoadBrd_VMonitor_RdVoltage_Scaled( &TmpData.Voltage );
          if (Status != HAL_OK)
            break;
        }
        break;
      case TEMPERATURE_MNTR_TASK:
        if ( Get_DriverStates( TEMPERATURE_MNTR_TASK ))
        {
          /* Read the Temperature Data. */
          Status = RoadBrd_ReadTemp_Scaled( &TmpData.Temp );
        }
        break;
      case IRRADIANCE_MNTR_TASK:
        if ( Get_DriverStates( IRRADIANCE_MNTR_TASK ))
        {
          /* Read the Irradiance Data. */
          Status = RoadBrd_RGBReadValues( &TmpData.RGBValues );
        }
        break;
      case PRESSURE_MNTR_TASK:
        if ( Get_DriverStates( PRESSURE_MNTR_TASK ))
        {
          /* Read the pressure and temperature */
          Status = RoadBrd_Baro_ReadPressure_Scaled( &TmpData.Pressure );
          if (Status != HAL_OK)
            break;
          Status = RoadBrd_Baro_ReadTemp( &TmpData.PrTemp );
        }
        break;
      case HUMIDITY_MNTR_TASK:
        if ( Get_DriverStates( HUMIDITY_MNTR_TASK ))
        {
          /* Read the Humidity and temperature */
          Status = RoadBrd_Humidity_ReadHumidity_Scaled( &TmpData.Humidity );
          if (Status != HAL_OK)
            break;
          Status = RoadBrd_Humidity_ReadTemperature_Scaled( &TmpData.HmTemp );
        }
        break;
      case GRIDEYE_MNTR_TASK:
        if ( Get_DriverStates( GRIDEYE_MNTR_TASK ))
        {
          Status = RoadBrd_GridEye_ReadValues_Scaled( &TmpData.GridValues );
        }
        else if ( Get_DriverStates( COOLEYE_MNTR_TASK ))
        {
          Status = RoadBrd_CoolEye_ReadValues_Scaled( &TmpData.GridValues );
        }
        break;
    } // EndSwitch (data.task_item)
    // Update Count
    data.task_item++;
    if (data.task_item >= TASK_LENGTH)
    {
      // OK...Time to Send Data to BLE Interface...Once for every time ALL sensors sampled.
      // This is to ensure the repeat of the full frame of data at least FRM_REPEAT_CNT Times.
//************************Build Frame and Send to BGM111************************
      // Send <FRM> String.
      sprintf( (char *)tmpBuffer, "<FRM>");
      RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
      BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
#ifndef LEGACY_PATCH
      if(strcmp( (char *)TmpData.ShntVltg.Raw, (char *)data.ShntVltg.Raw) != 0 )
      {
        RoadBrd_gpio_On( BGM_LED );
        // Update Information in data structure
        strcpy( (char *)data.ShntVltg.Raw, (char *)TmpData.ShntVltg.Raw );
        strcpy( (char *)data.ShntVltg.Voltage, (char *)TmpData.ShntVltg.Voltage );
        // Update BLE Characteristics
        /* Send the ShntVltgRw to the BLE module */
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U0002 Units= %cmV%c>%s</U0002>", '"', '"', (uint8_t *)data.ShntVltg.Voltage);
        else
          sprintf( (char *)tmpBuffer, "<U0002>%s</U0002>", (uint8_t *)data.ShntVltg.Voltage);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        //      BGM111_WriteCharacteristic(gattdb_ShntVltgRw,
        //                                 strlen((char *)data.ShntVltg.Raw), (uint8_t *)data.ShntVltg.Raw);
        /* Send the ShntVltg to the BLE module */
        //**HERE      BGM111_WriteCharacteristic(gattdb_ShntVltg,
        //                                 strlen((char *)data.ShntVltg.Voltage), (uint8_t *)data.ShntVltg.Voltage);
      }
      //..Current
      if(strcmp( (char *)TmpData.Current.Raw, (char *)data.Current.Raw) != 0 )
      {
        RoadBrd_gpio_On( BGM_LED );
        // Update Information in data structure
        strcpy( (char *)data.Current.Raw, (char *)TmpData.Current.Raw );
        strcpy( (char *)data.Current.Current, (char *)TmpData.Current.Current );
        // Update BLE Characteristics
        /* Send the CurrentRw to the BLE module */
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U0004 Units=%cmA%c>%s</U0004>", '"', '"', (uint8_t *)data.Current.Current);
        else
          sprintf( (char *)tmpBuffer, "<U0004>%s</U0004>", (uint8_t *)data.Current.Current);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        //      BGM111_WriteCharacteristic(gattdb_CurrentRw,
        //                                 strlen((char *)data.Current.Raw), (uint8_t *)data.Current.Raw);
        /* Send the Current to the BLE module */
        //**HERE      BGM111_WriteCharacteristic(gattdb_Current,
        //                                 strlen((char *)data.Current.Current), (uint8_t *)data.Current.Current);
      }
      //..Power
      if(strcmp( (char *)TmpData.Power.Raw, (char *)data.Power.Raw) != 0 )
      {
        RoadBrd_gpio_On( BGM_LED );
        // Update Information in data structure
        strcpy( (char *)data.Power.Raw, (char *)TmpData.Power.Raw );
        strcpy( (char *)data.Power.Power, (char *)TmpData.Power.Power );
        // Update BLE Characteristics
        /* Send the PowerRw to the BLE module */
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U0006 Units=%cmW%c>%s</U0006>", '"', '"', (uint8_t *)data.Power.Power);
        else
          sprintf( (char *)tmpBuffer, "<U0006>%s</U0006>", (uint8_t *)data.Power.Power);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        //      BGM111_WriteCharacteristic(gattdb_PowerRw,
        //                                 strlen((char *)data.Power.Raw), (uint8_t *)data.Power.Raw);
        /* Send the Power to the BLE module */
        //**HERE      BGM111_WriteCharacteristic(gattdb_Power,
        //                                 strlen((char *)data.Power.Power), (uint8_t *)data.Power.Power);
      }
      //..Voltage
      if(strcmp( (char *)TmpData.Voltage.Raw, (char *)data.Voltage.Raw) != 0 )
      {
        RoadBrd_gpio_On( BGM_LED );
        // Update Information in data structure
        strcpy( (char *)data.Voltage.Raw, (char *)TmpData.Voltage.Raw );
        strcpy( (char *)data.Voltage.Voltage, (char *)TmpData.Voltage.Voltage );
        // Update BLE Characteristics
        /* Send the VoltageRw to the BLE module */
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U0008 Units=%cV%c>%s</U0008>", '"', '"', (uint8_t *)data.Voltage.Voltage);
        else
          sprintf( (char *)tmpBuffer, "<U0008>%s</U0008>", (uint8_t *)data.Voltage.Voltage);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        //      BGM111_WriteCharacteristic(gattdb_VoltageRw,
        //                                 strlen((char *)data.Voltage.Raw), (uint8_t *)data.Voltage.Raw);
        /* Send the Voltage to the BLE module */
        //**HERE      BGM111_WriteCharacteristic(gattdb_Voltage,
        //                                 strlen((char *)data.Voltage.Voltage), (uint8_t *)data.Voltage.Voltage);
      }
#endif
      // Test Code to generate an event for Testing Tasked Error Msgs...Uncomment to Test..
      //RdBrd_ErrCdLogErrCd( REPAIR_I2C, MODULE_AppData );
      //..Temperature
      if(strcmp( (char *)TmpData.Temp.Raw, (char *)data.Temp.Raw) != 0 )
      {
        RoadBrd_gpio_On( BGM_LED );
        // Update Information in data structure
        strcpy( (char *)data.Temp.Raw, (char *)TmpData.Temp.Raw );
        strcpy( (char *)data.Temp.TempC, (char *)TmpData.Temp.TempC );
        strcpy( (char *)data.Temp.TempF, (char *)TmpData.Temp.TempF );
        data.Temp.RawC = TmpData.Temp.RawC;
        // Update BLE Characteristics
        /* Send the TemperatureRw to the BLE module */
#ifndef LEGACY_PATCH
        //      BGM111_WriteCharacteristic(gattdb_TemperatureRw,
        //                                 strlen((char *)data.Temp.Raw), (uint8_t *)data.Temp.Raw);
        /* Send the TemperatureC to the BLE module */
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U000A Units=%cC%c>%s</U000A>", '"', '"', (uint8_t *)data.Temp.TempC);
        else
          sprintf( (char *)tmpBuffer, "<U000A>%s</U000A>", (uint8_t *)data.Temp.TempC);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        //**HERE      BGM111_WriteCharacteristic(gattdb_TemperatureC,
        //                                 strlen((char *)data.Temp.TempC), (uint8_t *)data.Temp.TempC);
        /* Send the TemperatureF to the BLE module */
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U000B Units=%cF%c>%s</U000B>", '"', '"', (uint8_t *)data.Temp.TempF);
        else
          sprintf( (char *)tmpBuffer, "<U000B>%s</U000B>", (uint8_t *)data.Temp.TempF);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        //**HERE      BGM111_WriteCharacteristic(gattdb_TemperatureF,
        //                                 strlen((char *)data.Temp.TempF), (uint8_t *)data.Temp.TempF);
#endif
        // Update Legacy Temperature Characteristic...
        //      tmpBuffer[0] = (uint8_t)('T');
        //      tmpBuffer[1] = (uint8_t)(0x00);
        //      tmpBuffer[2] = (uint8_t)(data.Temp.RawC & 0x00ff);
        //      tmpBuffer[3] = (uint8_t)((data.Temp.RawC & 0xff00) >> 8);
        /* Send the Temperature RawC to the BLE module */
        //**HERE      BGM111_WriteCharacteristic(gattdb_xgatt_temp0,
        //                                 0x04, (uint8_t *)tmpBuffer);
      }
      
      //..Irradiance
#ifndef LEGACY_PATCH
      if(strcmp( (char *)TmpData.RGBValues.Raw, (char *)data.RGBValues.Raw) != 0 )
      {
        RoadBrd_gpio_On( BGM_LED );
        // Update Information in data structure
        strcpy( (char *)data.RGBValues.Raw, (char *)TmpData.RGBValues.Raw );
        strcpy( (char *)data.RGBValues.Red, (char *)TmpData.RGBValues.Red );
        strcpy( (char *)data.RGBValues.Green, (char *)TmpData.RGBValues.Green );
        strcpy( (char *)data.RGBValues.Blue, (char *)TmpData.RGBValues.Blue );
        // Update BLE Characteristics
        /* Send the RGBLightRw to the BLE module */
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U000C Units=%cRw%c>%s</U000C>", '"', '"', (uint8_t *)data.RGBValues.Raw);
        else
          sprintf( (char *)tmpBuffer, "<U000C>%s</U000C>", (uint8_t *)data.RGBValues.Raw);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        //      BGM111_WriteCharacteristic(gattdb_RGBLightRw,
        //                                 strlen((char *)data.RGBValues.Raw), (uint8_t *)data.RGBValues.Raw);
        /* Send the RGBLightRd to the BLE module */
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U000D Units=%clx%c>%s</U000D>", '"', '"', (uint8_t *)data.RGBValues.Red);
        else
          sprintf( (char *)tmpBuffer, "<U000D>%s</U000D>", (uint8_t *)data.RGBValues.Red);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        //**HERE      BGM111_WriteCharacteristic(gattdb_RGBLightRd,
        //                                 strlen((char *)data.RGBValues.Red), (uint8_t *)data.RGBValues.Red);
        /* Send the RGBLightGrn to the BLE module */
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U000E Units=%clx%c>%s</U000E>", '"', '"', (uint8_t *)data.RGBValues.Green);
        else
          sprintf( (char *)tmpBuffer, "<U000E>%s</U000E>", (uint8_t *)data.RGBValues.Green);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        //**HERE      BGM111_WriteCharacteristic(gattdb_RGBLightGrn,
        //                                 strlen((char *)data.RGBValues.Green), (uint8_t *)data.RGBValues.Green);
        /* Send the RGBLightBlu to the BLE module */
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U000F Units=%clx%c>%s</U000F>", '"', '"', (uint8_t *)data.RGBValues.Blue);
        else
          sprintf( (char *)tmpBuffer, "<U000F>%s</U000F>", (uint8_t *)data.RGBValues.Blue);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        //**HERE      BGM111_WriteCharacteristic(gattdb_RGBLightBlu,
        //                                 strlen((char *)data.RGBValues.Blue), (uint8_t *)data.RGBValues.Blue);
      }
      //..Pressure
      if(strcmp( (char *)TmpData.Pressure.Raw, (char *)data.Pressure.Raw) != 0 )
      {
        RoadBrd_gpio_On( BGM_LED );
        // Update Information inPressure data structure
        strcpy( (char *)data.Pressure.Raw, (char *)TmpData.Pressure.Raw );
        strcpy( (char *)data.Pressure.Pressure, (char *)TmpData.Pressure.Pressure );
        data.Pressure.RawC = TmpData.Pressure.RawC;
        // Update BLE Characteristics
        /* Send the PressureRw to the BLE module */
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<UACAC Units=%chP%c>%s</UACAC>", '"', '"', (uint8_t *)data.Pressure.Pressure);
        else
          sprintf( (char *)tmpBuffer, "<UACAC>%s</UACAC>", (uint8_t *)data.Pressure.Pressure);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        //      BGM111_WriteCharacteristic(gattdb_PressureRw,
        //                                 strlen((char *)data.Pressure.Raw), (uint8_t *)data.Pressure.Raw);
        /* Send the Pressure to the BLE module */
        //**HERE      BGM111_WriteCharacteristic(gattdb_Pressure,
        //                                 strlen((char *)data.Pressure.Pressure), (uint8_t *)data.Pressure.Pressure);
        // Update Legacy Pressure Characteristic..
        //      tmpBuffer[0] = (uint8_t)('P');
        //      tmpBuffer[3] = (uint8_t)(data.Pressure.RawC & 0x00ff);
        //      tmpBuffer[2] = (uint8_t)((data.Pressure.RawC & 0xff00) >> 8);
        //      tmpBuffer[1] = (uint8_t)((data.Pressure.RawC & 0xff0000) >> 16);
        /* Send the Pressure RawC to the BLE module */
        //**HERE      BGM111_WriteCharacteristic(gattdb_xgatt_barometer,
        //                                 0x04, (uint8_t *)tmpBuffer);
      }
      /*    if(strcmp( (char *)TmpData.PrTemp.Raw, (char *)data.PrTemp.Raw) != 0 )
      {
      RoadBrd_gpio_On( BGM_LED );
      // Update Information in data structure
      strcpy( (char *)data.PrTemp.Raw, (char *)TmpData.PrTemp.Raw );
      strcpy( (char *)data.PrTemp.TempC, (char *)TmpData.PrTemp.TempC );
      strcpy( (char *)data.PrTemp.TempF, (char *)TmpData.PrTemp.TempF );
      // Update BLE Characteristics*/
      /* Send the PrTemp to the BLE module */
      /*      if (RoadBrd_Get_UnitsFlag())
      sprintf( (char *)tmpBuffer, "<UACAC Units=\”hP\”>%s</UACAC>", (uint8_t *)data.Pressure.Pressure);
      else
      sprintf( (char *)tmpBuffer, "<UACAC>%s</UACAC>", (uint8_t *)data.Pressure.Pressure);
      RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
      BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
      //      BGM111_WriteCharacteristic(gattdb_PrTemperatureRw,
      //                                 strlen((char *)data.PrTemp.Raw), (uint8_t *)data.PrTemp.Raw); */
      /* Send the PrTemp to the BLE module */
      //**HERE      BGM111_WriteCharacteristic(gattdb_PrTemperatureC,
      //                                 strlen((char *)data.PrTemp.TempC), (uint8_t *)data.PrTemp.TempC);
      /* Send the PrTemp to the BLE module */
      //**HERE      BGM111_WriteCharacteristic(gattdb_PrTemperatureF,
      //                                 strlen((char *)data.PrTemp.TempF), (uint8_t *)data.PrTemp.TempF);
      //    } 
      //..Humidity
      if(strcmp( (char *)TmpData.Humidity.HRaw, (char *)data.Humidity.HRaw) != 0 )
      {
        RoadBrd_gpio_On( BGM_LED );
        // Update Information in Humidity data structure
        strcpy( (char *)data.Humidity.HRaw, (char *)TmpData.Humidity.HRaw );
        strcpy( (char *)data.Humidity.Humidity, (char *)TmpData.Humidity.Humidity );
        data.Humidity.HRawC = TmpData.Humidity.HRawC;
        // Update BLE Characteristics
        /* Send the data to the BLE module */
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<UABAB Units=%cPr%c>%s</UABAB>", '"', '"', (uint8_t *)data.Humidity.Humidity);
        else
          sprintf( (char *)tmpBuffer, "<UABAB>%s</UABAB>", (uint8_t *)data.Humidity.Humidity);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        //      BGM111_WriteCharacteristic(gattdb_HumidityRw,
        //                                 strlen((char *)data.Humidity.HRaw), (uint8_t *)data.Humidity.HRaw);
        /* Send the data to the BLE module */
        //**HERE      BGM111_WriteCharacteristic(gattdb_Humidity,
        //                                 strlen((char *)data.Humidity.Humidity), (uint8_t *)data.Humidity.Humidity);
        // Update Legacy Humidity Characteristic..
        //      tmpBuffer[0] = (uint8_t)('H');
        //      tmpBuffer[2] = (uint8_t)(data.Humidity.HRawC & 0x00ff);
        //      tmpBuffer[1] = (uint8_t)((data.Humidity.HRawC & 0xff00) >> 8);
        //      tmpBuffer[3] = (uint8_t)0x00;
        /* Send the Temperature RawC to the BLE module */
        //**HERE      BGM111_WriteCharacteristic(gattdb_xgatt_humidity,
        //                                 0x03, (uint8_t *)tmpBuffer);
      }
      if(strcmp( (char *)TmpData.HmTemp.Raw, (char *)data.HmTemp.Raw) != 0 )
      {
        RoadBrd_gpio_On( BGM_LED );
        // Update Information in data structure
        strcpy( (char *)data.HmTemp.Raw, (char *)TmpData.HmTemp.Raw );
        strcpy( (char *)data.HmTemp.TempC, (char *)TmpData.HmTemp.TempC );
        strcpy( (char *)data.HmTemp.TempF, (char *)TmpData.HmTemp.TempF );
        data.HmTemp.RawC = TmpData.HmTemp.RawC;
        // Update BLE Characteristics
        /* Send the HmTemp to the BLE module */
        //      BGM111_WriteCharacteristic(gattdb_HmdtyTempRw,
        //                                 strlen((char *)data.HmTemp.Raw), (uint8_t *)data.HmTemp.Raw);
        /* Send the HmTemp to the BLE module */
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U0032 Units=%cC%c>%s</U0032>", '"', '"', (uint8_t *)data.HmTemp.TempC);
        else
          sprintf( (char *)tmpBuffer, "<U0032>%s</U0032>", (uint8_t *)data.HmTemp.TempC);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        //**HERE      BGM111_WriteCharacteristic(gattdb_HmdtyTempC,
        //                                 strlen((char *)data.HmTemp.TempC), (uint8_t *)data.HmTemp.TempC);
        /* Send the HmTemp to the BLE module */
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U0033 Units=%cF%c>%s</U0033>", '"', '"', (uint8_t *)data.HmTemp.TempF);
        else
          sprintf( (char *)tmpBuffer, "<U0033>%s</U0033>", (uint8_t *)data.HmTemp.TempF);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        //**HERE      BGM111_WriteCharacteristic(gattdb_HmdtyTempF,
        //                                 strlen((char *)data.HmTemp.TempF), (uint8_t *)data.HmTemp.TempF);
      } 
      //..GridEye...Thermistor
#ifndef ALWAYS_SEND
      if(strcmp( (char *)TmpData.GridValues.Thermistor.Raw, (char *)data.GridValues.Thermistor.Raw) != 0 )
      {
#endif
        RoadBrd_gpio_On( BGM_LED );
        // Update Information in data structure
        strcpy( (char *)data.GridValues.Thermistor.Raw, (char *)TmpData.GridValues.Thermistor.Raw );
        strcpy( (char *)data.GridValues.Thermistor.TempC, (char *)TmpData.GridValues.Thermistor.TempC );
        strcpy( (char *)data.GridValues.Thermistor.TempF, (char *)TmpData.GridValues.Thermistor.TempF );
        // Update BLE Characteristics
        //      BGM111_WriteCharacteristic(gattdb_ThermistorRw,
        //                                 strlen((char *)data.GridValues.Thermistor.Raw), (uint8_t *)data.GridValues.Thermistor.Raw);
        /* Send the Thermistor to the BLE module */
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U0017 Units=%cC%c>%s</U0017>", '"', '"', (uint8_t *)data.GridValues.Thermistor.TempC);
        else
          sprintf( (char *)tmpBuffer, "<U0017>%s</U0017>", (uint8_t *)data.GridValues.Thermistor.TempC);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U0117 Units=%cF%c>%s</U0117>", '"', '"', (uint8_t *)data.GridValues.Thermistor.TempF);
        else
          sprintf( (char *)tmpBuffer, "<U0117>%s</U0117>", (uint8_t *)data.GridValues.Thermistor.TempF);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        //**HERE      BGM111_WriteCharacteristic(gattdb_ThermistorC,
        //                                 strlen((char *)data.GridValues.Thermistor.TempC), (uint8_t *)data.GridValues.Thermistor.TempC);
#ifndef ALWAYS_SEND
      }
#endif
#endif
      //..GridEye...Grid 1
#ifndef ALWAYS_SEND
      if(strcmp( (char *)TmpData.GridValues.GridEye1.Raw, (char *)data.GridValues.GridEye1.Raw) != 0 )
      {
#endif
        RoadBrd_gpio_On( BGM_LED );
        // Update Information in data structure
        strcpy( (char *)data.GridValues.GridEye1.Raw, (char *)TmpData.GridValues.GridEye1.Raw );
        strcpy( (char *)data.GridValues.GridEye1.TempC, (char *)TmpData.GridValues.GridEye1.TempC );
        strcpy( (char *)data.GridValues.GridEye1.TempF, (char *)TmpData.GridValues.GridEye1.TempF );
        data.GridValues.GridEye1.RawC = TmpData.GridValues.GridEye1.RawC;
        // Update BLE Characteristics
        /* Send the Grid 1 to the BLE module */
#ifndef LEGACY_PATCH
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U0019 Units=%cC%c>%s</U0019>", '"', '"', (uint8_t *)data.GridValues.GridEye1.TempC);
        else
          sprintf( (char *)tmpBuffer, "<U0019>%s</U0019>", (uint8_t *)data.GridValues.GridEye1.TempC);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U0119 Units=%cF%c>%s</U0119>", '"', '"', (uint8_t *)data.GridValues.GridEye1.TempF);
        else
          sprintf( (char *)tmpBuffer, "<U0119>%s</U0119>", (uint8_t *)data.GridValues.GridEye1.TempF);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        //      BGM111_WriteCharacteristic(gattdb_Thermal_1Rw,
        //                                 strlen((char *)data.GridValues.GridEye1.Raw), (uint8_t *)data.GridValues.GridEye1.Raw);
        /* Send the Pressure to the BLE module */
        //**HERE      BGM111_WriteCharacteristic(gattdb_Thermal_1RwC,
        //                                 strlen((char *)data.GridValues.GridEye1.TempC), (uint8_t *)data.GridValues.GridEye1.TempC);
#endif
        // Update Legacy Temperature Characteristic..
        //      tmpBuffer[0] = (uint8_t)('T');
        //      tmpBuffer[1] = (uint8_t)(0x01);
        //      tmpBuffer[2] = (uint8_t)(data.GridValues.GridEye1.RawC & 0x00ff);
        //      tmpBuffer[3] = (uint8_t)((data.GridValues.GridEye1.RawC & 0xff00) >> 8);
        /* Send the Temperature RawC to the BLE module */
        //**HERE      BGM111_WriteCharacteristic(gattdb_xgatt_temp1,
        //                                 0x04, (uint8_t *)tmpBuffer);
#ifndef ALWAYS_SEND
      }
#endif
      //.GridEye..Grid 2
      if(strcmp( (char *)TmpData.GridValues.GridEye2.Raw, (char *)data.GridValues.GridEye2.Raw) != 0 )
      {
        RoadBrd_gpio_On( BGM_LED );
        // Update Information in data structure
        strcpy( (char *)data.GridValues.GridEye2.Raw, (char *)TmpData.GridValues.GridEye2.Raw );
        strcpy( (char *)data.GridValues.GridEye2.TempC, (char *)TmpData.GridValues.GridEye2.TempC );
        strcpy( (char *)data.GridValues.GridEye2.TempF, (char *)TmpData.GridValues.GridEye2.TempF );
        data.GridValues.GridEye2.RawC = TmpData.GridValues.GridEye2.RawC;
        // Update BLE Characteristics
        /* Send the Grid 2 to the BLE module */
#ifndef LEGACY_PATCH
        //      BGM111_WriteCharacteristic(gattdb_Thermal_2Rw,
        //                                 strlen((char *)data.GridValues.GridEye2.Raw), (uint8_t *)data.GridValues.GridEye2.Raw);
        /* Send the Pressure to the BLE module */
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U001B Units=%cC%c>%s</U001B>", '"', '"', (uint8_t *)data.GridValues.GridEye2.TempC);
        else
          sprintf( (char *)tmpBuffer, "<U001B>%s</U001B>", (uint8_t *)data.GridValues.GridEye2.TempC);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U011B Units=%cF%c>%s</U011B>", '"', '"', (uint8_t *)data.GridValues.GridEye2.TempF);
        else
          sprintf( (char *)tmpBuffer, "<U011B>%s</U011B>", (uint8_t *)data.GridValues.GridEye2.TempF);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        //**HERE      BGM111_WriteCharacteristic(gattdb_Thermal_2RwC,
        //                                 strlen((char *)data.GridValues.GridEye2.TempC), (uint8_t *)data.GridValues.GridEye2.TempC);
#endif
        // Update Legacy Temperature Characteristic..
        //      tmpBuffer[0] = (uint8_t)('T');
        //      tmpBuffer[1] = (uint8_t)(0x02);
        //      tmpBuffer[2] = (uint8_t)(data.GridValues.GridEye2.RawC & 0x00ff);
        //      tmpBuffer[3] = (uint8_t)((data.GridValues.GridEye2.RawC & 0xff00) >> 8);
        /* Send the Temperature RawC to the BLE module */
        //**HERE      BGM111_WriteCharacteristic(gattdb_xgatt_temp2,
        //                                 0x04, (uint8_t *)tmpBuffer);
      }
      //.GridEye..Grid 3
      if(strcmp( (char *)TmpData.GridValues.GridEye3.Raw, (char *)data.GridValues.GridEye3.Raw) != 0 )
      {
        RoadBrd_gpio_On( BGM_LED );
        // Update Information in data structure
        strcpy( (char *)data.GridValues.GridEye3.Raw, (char *)TmpData.GridValues.GridEye3.Raw );
        strcpy( (char *)data.GridValues.GridEye3.TempC, (char *)TmpData.GridValues.GridEye3.TempC );
        strcpy( (char *)data.GridValues.GridEye3.TempF, (char *)TmpData.GridValues.GridEye3.TempF );
        data.GridValues.GridEye3.RawC = TmpData.GridValues.GridEye3.RawC;
        // Update BLE Characteristics
        /* Send the Grid 3 to the BLE module */
#ifndef LEGACY_PATCH
        //      BGM111_WriteCharacteristic(gattdb_Thermal_3Rw,
        //                                 strlen((char *)data.GridValues.GridEye3.Raw), (uint8_t *)data.GridValues.GridEye3.Raw);
        /* Send the Pressure to the BLE module */
        //**HERE      BGM111_WriteCharacteristic(gattdb_Thermal_3RwC,
        //                                 strlen((char *)data.GridValues.GridEye3.TempC), (uint8_t *)data.GridValues.GridEye3.TempC);
#endif
        // Update Legacy Temperature Characteristic..
        //      tmpBuffer[0] = (uint8_t)('T');
        //      tmpBuffer[1] = (uint8_t)(0x04);
        //      tmpBuffer[2] = (uint8_t)(data.GridValues.GridEye3.RawC & 0x00ff);
        //      tmpBuffer[3] = (uint8_t)((data.GridValues.GridEye3.RawC & 0xff00) >> 8);
        /* Send the Temperature RawC to the BLE module */
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U001D Units=%cC%c>%s</U001D>", '"', '"', (uint8_t *)data.GridValues.GridEye3.TempC);
        else
          sprintf( (char *)tmpBuffer, "<U001D>%s</U001D>", (uint8_t *)data.GridValues.GridEye3.TempC);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U011D Units=%cF%c>%s</U011D>", '"', '"', (uint8_t *)data.GridValues.GridEye3.TempF);
        else
          sprintf( (char *)tmpBuffer, "<U011D>%s</U011D>", (uint8_t *)data.GridValues.GridEye3.TempF);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        //**HERE      BGM111_WriteCharacteristic(gattdb_xgatt_temp3,
        //                                 0x04, (uint8_t *)tmpBuffer);
      }
      //.GridEye..Grid 4
#ifndef ALWAYS_SEND
      if(strcmp( (char *)TmpData.GridValues.GridEye4.Raw, (char *)data.GridValues.GridEye4.Raw) != 0 )
      {
#endif
        RoadBrd_gpio_On( BGM_LED );
        // Update Information in data structure
        strcpy( (char *)data.GridValues.GridEye4.Raw, (char *)TmpData.GridValues.GridEye4.Raw );
        strcpy( (char *)data.GridValues.GridEye4.TempC, (char *)TmpData.GridValues.GridEye4.TempC );
        strcpy( (char *)data.GridValues.GridEye4.TempF, (char *)TmpData.GridValues.GridEye4.TempF );
        data.GridValues.GridEye4.RawC = TmpData.GridValues.GridEye4.RawC;
        // Update BLE Characteristics
        /* Send the Grid 4 to the BLE module */
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U001F Units=%cC%c>%s</U001F>", '"', '"', (uint8_t *)data.GridValues.GridEye4.TempC);
        else
          sprintf( (char *)tmpBuffer, "<U001F>%s</U001F>", (uint8_t *)data.GridValues.GridEye4.TempC);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U011F Units=%cF%c>%s</U011F>", '"', '"', (uint8_t *)data.GridValues.GridEye4.TempF);
        else
          sprintf( (char *)tmpBuffer, "<U011F>%s</U011F>", (uint8_t *)data.GridValues.GridEye4.TempF);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
#ifndef LEGACY_PATCH
        //      BGM111_WriteCharacteristic(gattdb_Thermal_4Rw,
        //                                 strlen((char *)data.GridValues.GridEye4.Raw), (uint8_t *)data.GridValues.GridEye4.Raw);
        /* Send the Pressure to the BLE module */
        //**HERE      BGM111_WriteCharacteristic(gattdb_Thermal_4RwC,
        //                                 strlen((char *)data.GridValues.GridEye4.TempC), (uint8_t *)data.GridValues.GridEye4.TempC);
#endif
        // Update Legacy Temperature Characteristic..
        //      tmpBuffer[0] = (uint8_t)('T');
        //      tmpBuffer[1] = (uint8_t)(0x04);
        //      tmpBuffer[2] = (uint8_t)(data.GridValues.GridEye4.RawC & 0x00ff);
        //      tmpBuffer[3] = (uint8_t)((data.GridValues.GridEye4.RawC & 0xff00) >> 8);
        /* Send the Temperature RawC to the BLE module */
        //**HERE      BGM111_WriteCharacteristic(gattdb_xgatt_temp4,
        //                                 0x04, (uint8_t *)tmpBuffer);
#ifndef ALWAYS_SEND
      }
      //.GridEye..Grid 5
#endif
      if(strcmp( (char *)TmpData.GridValues.GridEye5.Raw, (char *)data.GridValues.GridEye5.Raw) != 0 )
      {
        RoadBrd_gpio_On( BGM_LED );
        // Update Information in data structure
        strcpy( (char *)data.GridValues.GridEye5.Raw, (char *)TmpData.GridValues.GridEye5.Raw );
        strcpy( (char *)data.GridValues.GridEye5.TempC, (char *)TmpData.GridValues.GridEye5.TempC );
        strcpy( (char *)data.GridValues.GridEye5.TempF, (char *)TmpData.GridValues.GridEye5.TempF );
        data.GridValues.GridEye5.RawC = TmpData.GridValues.GridEye5.RawC;
        // Update BLE Characteristics
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U0021 Units=%cC%c>%s</U0021>", '"', '"', (uint8_t *)data.GridValues.GridEye5.TempC);
        else
          sprintf( (char *)tmpBuffer, "<U0021>%s</U0021>", (uint8_t *)data.GridValues.GridEye5.TempC);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U0121 Units=%cF%c>%s</U0121>", '"', '"', (uint8_t *)data.GridValues.GridEye5.TempF);
        else
          sprintf( (char *)tmpBuffer, "<U0121>%s</U0121>", (uint8_t *)data.GridValues.GridEye5.TempF);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
#ifndef LEGACY_PATCH
        //      BGM111_WriteCharacteristic(gattdb_Thermal_5Rw,
        //                                 strlen((char *)data.GridValues.GridEye5.Raw), (uint8_t *)data.GridValues.GridEye5.Raw);
        /* Send the Pressure to the BLE module */
        //**HERE      BGM111_WriteCharacteristic(gattdb_Thermal_5RwC,
        //                                 strlen((char *)data.GridValues.GridEye5.TempC), (uint8_t *)data.GridValues.GridEye5.TempC);
#endif
        // Update Legacy Temperature Characteristic..
        //      tmpBuffer[0] = (uint8_t)('T');
        //      tmpBuffer[1] = (uint8_t)(0x05);
        //      tmpBuffer[2] = (uint8_t)(data.GridValues.GridEye5.RawC & 0x00ff);
        //      tmpBuffer[3] = (uint8_t)((data.GridValues.GridEye5.RawC & 0xff00) >> 8);
        /* Send the Temperature RawC to the BLE module */
        //**HERE      BGM111_WriteCharacteristic(gattdb_xgatt_temp5,
        //                                 0x04, (uint8_t *)tmpBuffer);
      }
      //.GridEye..Grid 6
      if(strcmp( (char *)TmpData.GridValues.GridEye6.Raw, (char *)data.GridValues.GridEye6.Raw) != 0 )
      {
        RoadBrd_gpio_On( BGM_LED );
        // Update Information in data structure
        strcpy( (char *)data.GridValues.GridEye6.Raw, (char *)TmpData.GridValues.GridEye6.Raw );
        strcpy( (char *)data.GridValues.GridEye6.TempC, (char *)TmpData.GridValues.GridEye6.TempC );
        strcpy( (char *)data.GridValues.GridEye6.TempF, (char *)TmpData.GridValues.GridEye6.TempF );
        data.GridValues.GridEye6.RawC = TmpData.GridValues.GridEye6.RawC;
        // Update BLE Characteristics
        /* Send the Grid 6 to the BLE module */
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U0023 Units=%cC%c>%s</U0023>", '"', '"', (uint8_t *)data.GridValues.GridEye6.TempC);
        else
          sprintf( (char *)tmpBuffer, "<U0023>%s</U0023>", (uint8_t *)data.GridValues.GridEye6.TempC);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U0123 Units=%cF%c>%s</U0123>", '"', '"', (uint8_t *)data.GridValues.GridEye6.TempF);
        else
          sprintf( (char *)tmpBuffer, "<U0123>%s</U0123>", (uint8_t *)data.GridValues.GridEye6.TempF);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
#ifndef LEGACY_PATCH
        //      BGM111_WriteCharacteristic(gattdb_Thermal_6Rw,
        //                                 strlen((char *)data.GridValues.GridEye6.Raw), (uint8_t *)data.GridValues.GridEye6.Raw);
        /* Send the Pressure to the BLE module */
        //**HERE      BGM111_WriteCharacteristic(gattdb_Thermal_6RwC,
        //                                 strlen((char *)data.GridValues.GridEye6.TempC), (uint8_t *)data.GridValues.GridEye6.TempC);
#endif
        // Update Legacy Temperature Characteristic..
        //      tmpBuffer[0] = (uint8_t)('T');
        //      tmpBuffer[1] = (uint8_t)(0x06);
        //      tmpBuffer[2] = (uint8_t)(data.GridValues.GridEye6.RawC & 0x00ff);
        //      tmpBuffer[3] = (uint8_t)((data.GridValues.GridEye6.RawC & 0xff00) >> 8);
        /* Send the Temperature RawC to the BLE module */
        //**HERE      BGM111_WriteCharacteristic(gattdb_xgatt_temp6,
        //                                 0x04, (uint8_t *)tmpBuffer);
      }
      //.GridEye..Grid 7
#ifndef ALWAYS_SEND
      if(strcmp( (char *)TmpData.GridValues.GridEye7.Raw, (char *)data.GridValues.GridEye7.Raw) != 0 )
      {
#endif
        RoadBrd_gpio_On( BGM_LED );
        // Update Information in data structure
        strcpy( (char *)data.GridValues.GridEye7.Raw, (char *)TmpData.GridValues.GridEye7.Raw );
        strcpy( (char *)data.GridValues.GridEye7.TempC, (char *)TmpData.GridValues.GridEye7.TempC );
        strcpy( (char *)data.GridValues.GridEye7.TempF, (char *)TmpData.GridValues.GridEye7.TempF );
        data.GridValues.GridEye7.RawC = TmpData.GridValues.GridEye7.RawC;
        // Update BLE Characteristics
        /* Send the Grid 7 to the BLE module */
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U0025 Units=%cC%c>%s</U0025>", '"', '"', (uint8_t *)data.GridValues.GridEye7.TempC);
        else
          sprintf( (char *)tmpBuffer, "<U0025>%s</U0025>", (uint8_t *)data.GridValues.GridEye7.TempC);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U0125 Units=%cF%c>%s</U0125>", '"', '"', (uint8_t *)data.GridValues.GridEye7.TempF);
        else
          sprintf( (char *)tmpBuffer, "<U0125>%s</U0125>", (uint8_t *)data.GridValues.GridEye7.TempF);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
#ifndef LEGACY_PATCH
        //      BGM111_WriteCharacteristic(gattdb_Thermal_7Rw,
        //                                 strlen((char *)data.GridValues.GridEye7.Raw), (uint8_t *)data.GridValues.GridEye7.Raw);
        /* Send the Pressure to the BLE module */
        //**HERE      BGM111_WriteCharacteristic(gattdb_Thermal_7RwC,
        //                                 strlen((char *)data.GridValues.GridEye7.TempC), (uint8_t *)data.GridValues.GridEye7.TempC);
#endif
        // Update Legacy Temperature Characteristic..
        //      tmpBuffer[0] = (uint8_t)('T');
        //      tmpBuffer[1] = (uint8_t)(0x07);
        //      tmpBuffer[2] = (uint8_t)(data.GridValues.GridEye7.RawC & 0x00ff);
        //      tmpBuffer[3] = (uint8_t)((data.GridValues.GridEye7.RawC & 0xff00) >> 8);
        /* Send the Temperature RawC to the BLE module */
        //**HERE      BGM111_WriteCharacteristic(gattdb_xgatt_temp7,
        //                                 0x04, (uint8_t *)tmpBuffer);
#ifndef ALWAYS_SEND
      }
      //.GridEye..Grid 8
#endif
      if(strcmp( (char *)TmpData.GridValues.GridEye8.Raw, (char *)data.GridValues.GridEye8.Raw) != 0 )
      {
        RoadBrd_gpio_On( BGM_LED );
        // Update Information in data structure
        strcpy( (char *)data.GridValues.GridEye8.Raw, (char *)TmpData.GridValues.GridEye8.Raw );
        strcpy( (char *)data.GridValues.GridEye8.TempC, (char *)TmpData.GridValues.GridEye8.TempC );
        strcpy( (char *)data.GridValues.GridEye8.TempF, (char *)TmpData.GridValues.GridEye8.TempF );
        data.GridValues.GridEye8.RawC = TmpData.GridValues.GridEye8.RawC;
        // Update BLE Characteristics
        /* Send the Grid 8 to the BLE module */
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U0027 Units=%cC%c>%s</U0027>", '"', '"', (uint8_t *)data.GridValues.GridEye8.TempC);
        else
          sprintf( (char *)tmpBuffer, "<U0027>%s</U0027>", (uint8_t *)data.GridValues.GridEye8.TempC);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        if (RoadBrd_Get_UnitsFlag())
          sprintf( (char *)tmpBuffer, "<U0127 Units=%cF%c>%s</U0127>", '"', '"', (uint8_t *)data.GridValues.GridEye8.TempF);
        else
          sprintf( (char *)tmpBuffer, "<U0127>%s</U0127>", (uint8_t *)data.GridValues.GridEye8.TempF);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
#ifndef LEGACY_PATCH
        //      BGM111_WriteCharacteristic(gattdb_Thermal_8Rw,
        //                                 strlen((char *)data.GridValues.GridEye8.Raw), (uint8_t *)data.GridValues.GridEye8.Raw);
        /* Send the Pressure to the BLE module */
        //**HERE      BGM111_WriteCharacteristic(gattdb_Thermal_8RwC,
        //                                 strlen((char *)data.GridValues.GridEye8.TempC), (uint8_t *)data.GridValues.GridEye8.TempC);
#endif
        // Update Legacy Temperature Characteristic..
        //      tmpBuffer[0] = (uint8_t)('T');
        //      tmpBuffer[1] = (uint8_t)(0x08);
        //      tmpBuffer[2] = (uint8_t)(data.GridValues.GridEye8.RawC & 0x00ff);
        //      tmpBuffer[3] = (uint8_t)((data.GridValues.GridEye8.RawC & 0xff00) >> 8);
        /* Send the Temperature RawC to the BLE module */
        //**HERE      BGM111_WriteCharacteristic(gattdb_xgatt_temp8,
        //                                 0x04, (uint8_t *)tmpBuffer);
      }
      // Test Legacy flag to perform one time operations
      if( data.Legacy_OneTime )
      {
        // Clear Flag...We are done.
        data.Legacy_OneTime = false;
        // Clean out Buffer.
        for (x=0; x<TMPBUFFER_LNGTH; x++)
          tmpBuffer[x] = 0x00;
        // Update BLE Characteristics
        sprintf( (char *)tmpBuffer, "<UAEAE>B:%s", get_BGMBanner());
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        // Clean out Buffer.
        for (x=0; x<TMPBUFFER_LNGTH; x++)
          tmpBuffer[x] = 0x00;
        sprintf( (char *)tmpBuffer, " M:%s</UAEAE>", (uint8_t *)LEGACY_BANNER);
        RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
        BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
        //**HERE      BGM111_WriteCharacteristic(gattdb_xgatt_rev,
        //                                 strlen((char *)LEGACY_BANNER), (uint8_t *)LEGACY_BANNER);
      }
      // Clean out Buffer.
      for (x=0; x<TMPBUFFER_LNGTH; x++)
        tmpBuffer[x] = 0x00;
      // Send </FRM> String.
      sprintf( (char *)tmpBuffer, "</FRM>");
      RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
      BGM111_Transmit((uint32_t)(strlen((char *)tmpBuffer)), tmpBuffer);
      // Clean out Buffer.
      for (x=0; x<TMPBUFFER_LNGTH; x++)
        tmpBuffer[x] = 0x00;
      sprintf( (char *)tmpBuffer, "\r\n\r\n" );
      RoadBrd_UART_Transmit(MONITOR_UART, tmpBuffer);
      if (data.task_item == VOLTAGE_MNTR_TASK)
      {
      }
      //************************END Build Frame and Send to BGM111************************

      // Test Whether we need to reload all settings.
      if (analytics.FrmRpt_Cnt < FRM_REPEAT_CNT)
      {
        analytics.FrmRpt_Cnt++;
        ClrDataStructure();                           // Clear Backup data structure.
        
        // Test for failed I2C Bus and attempt to repair...If Drivers are ALL NULL....I2C has failed.
        if ( (Get_DriverStatus() & 0x00FF) == 0x0000 )
        {
          if ((RoadBrd_I2CRepair()) == HAL_OK)
          {
            RdBrd_ErrCdLogErrCd( REPAIR_I2C, MODULE_AppData );
            // Enable I2C_STATE.
            Set_DriverStates( I2C_STATE, DRIVER_ON );
            // Now Reinit I2C Bus.
            MX_I2C1_Init();

            // Re-Init All I2C Drivers again.
            
            Status = RoadBrd_RGBInit();
            if (Status == HAL_OK)
            {
              Set_DriverStates( IRRADIANCE_MNTR_TASK, DRIVER_ON );
            }
            else
            {
              RdBrd_ErrCdLogErrCd( ERROR_RGB_INIT, MODULE_main );
              Set_DriverStates( IRRADIANCE_MNTR_TASK, DRIVER_OFF );
            }
            
            // Initialize Voltage Monitor Hardware
            Status = RoadBrd_Init_VMonitor();
            if (Status == HAL_OK)
            {
              Set_DriverStates( VOLTAGE_MNTR_TASK, DRIVER_ON );
            }
            else
            {
              RdBrd_ErrCdLogErrCd( ERROR_VMNTR_INIT, MODULE_main );
              Set_DriverStates( VOLTAGE_MNTR_TASK, DRIVER_OFF );
            }
            
            // Initialize Grid Eye Hardware
            Status = RoadBrd_GridEyeInit();
            if (Status == HAL_OK)
            {
              Set_DriverStates( GRIDEYE_MNTR_TASK, DRIVER_ON );
            }
            else
            {
              RdBrd_ErrCdLogErrCd( ERROR_GDEYE_INIT, MODULE_main );
              Set_DriverStates( GRIDEYE_MNTR_TASK, DRIVER_OFF );
              // OK, We have no Grid Eye...Do we have a Cool Eye?
              Status = RoadBrd_CoolEyeInit();
              if (Status == HAL_OK)
              {
                Set_DriverStates( COOLEYE_MNTR_TASK, DRIVER_ON );
              }
              else
              {
                RdBrd_ErrCdLogErrCd( ERROR_CLEYE_INIT, MODULE_main );
                Set_DriverStates( COOLEYE_MNTR_TASK, DRIVER_OFF );
              }
            }
            // Initialize Pressure Sensor Hardware
            Status = RoadBrd_Init_Barometer();
            if (Status == HAL_OK)
            {
              Set_DriverStates( PRESSURE_MNTR_TASK, DRIVER_ON );
            }
            else
            {
              RdBrd_ErrCdLogErrCd( ERROR_PRESSURE_INIT, MODULE_main );
              Set_DriverStates( PRESSURE_MNTR_TASK, DRIVER_OFF );
            }
            
            // Initialize Humidity Sensor Hardware
            Status = RoadBrd_HumidityInit();
            if (Status == HAL_OK)
            {
              Set_DriverStates( HUMIDITY_MNTR_TASK, DRIVER_ON );
            }
            else
            {
              RdBrd_ErrCdLogErrCd( ERROR_HUMIDITY_INIT, MODULE_main );
              Set_DriverStates( HUMIDITY_MNTR_TASK, DRIVER_OFF );
            }
            
            // Test Temperature Sensor Hardware
            Status = RoadBrd_ReadTemp( &Temp );
            if (Status == HAL_OK)
            {
              Set_DriverStates( TEMPERATURE_MNTR_TASK, DRIVER_ON );
            }
            else
            {
              RdBrd_ErrCdLogErrCd( ERROR_TEMP_INIT, MODULE_main );
              Set_DriverStates( TEMPERATURE_MNTR_TASK, DRIVER_OFF );
            }
          } // EndIf ((SkyPack_I2CRepair()) == HAL_OK)
          else
          {
            RdBrd_ErrCdLogErrCd( ERROR_I2CBUSY, MODULE_AppData );
          } // EndElse ((SkyPack_I2CRepair()) == HAL_OK)
        } // EndIf ( !(Get_DriverStates( I2C_STATE )) )
        
      } //EndIf (analytics.FrmRpt_Cnt < FRM_REPEAT_CNT)
      // Reset Count
      data.task_item = VOLTAGE_MNTR_TASK;
      /* Clear the scheduling flag */
      data.reading_scheduled = false;
    } // EndIf (data.task_item >= TASK_LENGTH)
    if (Status != HAL_OK)
      return Status;
//************************NEW STYLE TASKING ENDS HERE************************
    // Service Watchdog
    RoadBrd_WWDG_Refresh();     // Refresh WatchDog
    //************ NOW Compare the data strings and determine if characteristics need to be sent.
    //..ShntVltg
//#if 0

    
  } // EndIf (data.reading_scheduled)

  return Status;
}

  /**
  * @brief  This function Processes the Road SOund interface and builds the appropriate characteristics.
  * @param  None
  * @retval None
  */
void Process_RdSound( void )
{
//  uint8_t tempStr[13];
  char tempstr[30];
  uint8_t tempBffr2[80];
  //static uint8_t analytics.CMD_Md_Cnt = 0;

  /* Is a RdSnd scheduled? */
  if (data.RdSnd_Scheduled)
  {

/*    // Is CMD_Mode active?
    if ((BGM111_Ready()) &&
        (BGM111_Connected()) &&
          (BGM111_CMD_Mode()) )
    {
      // 1. Increment CMD_Mode Count
      analytics.CMD_Md_Cnt++;
      sprintf( (char *)tempBffr2, "<<%d", analytics.CMD_Md_Cnt );
      RoadBrd_UART_Transmit(MONITOR_UART, tempBffr2);
      
      // Test Cnt against Limit.
      if ( analytics.CMD_Md_Cnt >= RoadBrd_Get_BootDelay())
      {
        RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)">>\r\n");
        // Clear Count for Next Event.
        analytics.CMD_Md_Cnt = 0;
        // Send String to Server to indicate new CMD Mode.
        sprintf( (char *)tempBffr2, "<STATUS>DATA_ASYNC</STATUS>" );
        RoadBrd_UART_Transmit(MONITOR_UART, tempBffr2);
        BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), tempBffr2);
        // Clear CMD_Mode.
        BGM111_SetCMD_Mode( false );
        // Set Data_Connection Mode.
        BGM111_SetDataConnected( true );
        // Change RD_Sound Timer to correct value for Data Mode.
        // First Reload FLASH Frames
        RoadBrd_WWDG_VerifyFrame();
        // NOW...Reload Active Timer.
        Set_RdSndTickCnt( RoadBrd_Get_RdSndTickCnt() );
      } // EndIf ( CMD_Md_Cnt >= CMD_MODE_LMT)
    } // EndIf ((BGM111_Ready()) && (BGM111_Connected()) && (BGM111_CMD_Mode()) ) */
    
    if ((BGM111_Ready()) &&
        (BGM111_Connected()) &&
          (BGM111_DataConnected()) &&
            (BGM111_SyncModeTestNoInc()) )
    {
      // Increment Frame Ratio
      analytics.Frame_Ratio++;
      // Now Test to determine if we have passed a threshold
      if (analytics.Frame_Ratio > RATIO_THRESHOLD)
      {
        sprintf( (char *)tempBffr2, "<ERROR: FRAME RATIO EXCEEEDED: %d>", analytics.Frame_Ratio);
        RoadBrd_UART_Transmit(MONITOR_UART, tempBffr2);
        BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), tempBffr2);
      }
      
      // Send <FRM> String.
      sprintf( (char *)tempBffr2, "<FRM>");
      RoadBrd_UART_Transmit(MONITOR_UART, tempBffr2);
      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), tempBffr2);
      
      RoadBrd_gpio_On( MICRO_LED );
      // 1. Build and Process Road Sound
      RoadBrdSnd_ProcessSound();
      
      // 2. Load FFT Data into local Data.
      RoadBrdSnd_DumpBin0( &data.FFTBin0 );
      RoadBrdSnd_DumpBin16( &data.FFTBin16 );
      RoadBrdSnd_DumpBin32( &data.FFTBin32 );
      RoadBrdSnd_DumpBin48( &data.FFTBin48 );
      
      // 3. Update BLE Characteristics with new data.
      /* Send the FFTBin0 to the BLE module */
      RoadBrd_gpio_On( BGM_LED );
      
#ifndef LEGACY_PATCH
      if (RoadBrd_Get_UnitsFlag())
        sprintf( (char *)tempBffr2, "<U0012 Units=%cBn%c>%s</U0012>", '"', '"', (uint8_t *)data.FFTBin0.dumpStr);
      else
        sprintf( (char *)tempBffr2, "<U0012>%s</U0012>", (uint8_t *)data.FFTBin0.dumpStr);
      RoadBrd_UART_Transmit(MONITOR_UART, tempBffr2);
      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), tempBffr2);
      //  BGM111_WriteCharacteristic(gattdb_RdSound0,
      //                             strlen((char *)data.FFTBin0.dumpStr), (uint8_t *)data.FFTBin0.dumpStr);
#endif
      /* Send the FFTBin16 to the BLE module */
      RoadBrd_gpio_On( BGM_LED );
#ifndef LEGACY_PATCH
      if (RoadBrd_Get_UnitsFlag())
        sprintf( (char *)tempBffr2, "<U0013 Units=%cBn%c>%s</U0013>", '"', '"', (uint8_t *)data.FFTBin16.dumpStr);
      else
        sprintf( (char *)tempBffr2, "<U0013>%s</U0013>", (uint8_t *)data.FFTBin16.dumpStr);
      RoadBrd_UART_Transmit(MONITOR_UART, tempBffr2);
      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), tempBffr2);
      //  BGM111_WriteCharacteristic(gattdb_RdSound16,
      //                             strlen((char *)data.FFTBin16.dumpStr), (uint8_t *)data.FFTBin16.dumpStr);
#endif
      /* Send the FFTBin32 to the BLE module */
      RoadBrd_gpio_On( BGM_LED );
#ifndef LEGACY_PATCH
      if (RoadBrd_Get_UnitsFlag())
        sprintf( (char *)tempBffr2, "<U0014 Units=%cB%c”>%s</U0014>", '"', '"', (uint8_t *)data.FFTBin32.dumpStr);
      else
        sprintf( (char *)tempBffr2, "<U0014>%s</U0014>", (uint8_t *)data.FFTBin32.dumpStr);
      RoadBrd_UART_Transmit(MONITOR_UART, tempBffr2);
      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), tempBffr2);
      //  BGM111_WriteCharacteristic(gattdb_RdSound32,
      //                             strlen((char *)data.FFTBin32.dumpStr), (uint8_t *)data.FFTBin32.dumpStr);
#endif
      /* Send the FFTBin48 to the BLE module */
      RoadBrd_gpio_On( BGM_LED );
#ifndef LEGACY_PATCH
      if (RoadBrd_Get_UnitsFlag())
        sprintf( (char *)tempBffr2, "<U0015 Units=%cBn%c>%s</U0015>", '"', '"', (uint8_t *)data.FFTBin48.dumpStr);
      else
        sprintf( (char *)tempBffr2, "<U0015>%s</U0015>", (uint8_t *)data.FFTBin48.dumpStr);
      RoadBrd_UART_Transmit(MONITOR_UART, tempBffr2);
      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), tempBffr2);
      //  BGM111_WriteCharacteristic(gattdb_RdSound48,
      //                             strlen((char *)data.FFTBin48.dumpStr), (uint8_t *)data.FFTBin48.dumpStr);
      //sprintf( (char *)tempBffr2, "<TICK>RP/%08x/%04x/%04x</TICK>", HAL_GetTick(), HeartBeat_Cnt, connection_cnt);
      // Build Tick String
      sprintf( tempstr, "%s", getTickString());
      // Save Tick String....
      RoadBrd_WWDG_SetTickString( tempstr );
      sprintf( (char *)tempBffr2, "<TICK>RP/%s</TICK>", tempstr);
      RoadBrd_UART_Transmit(MONITOR_UART, tempBffr2);
      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), tempBffr2);
      // Set Sync Flag for Frame.
      BGM111_cntrlSetSyncFlg( SYNC_WAIT );
      // Test TACK State
      if (BGM111_GetTackState() == TACK_ARMED)
      {
        BGM111_SetTackState(TACK_ARMED2);
        RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)"<ble.TackArmed = TACK_ARMED2>");
      }
      else if (BGM111_GetTackState() == TACK_ARMED2)
      {
        BGM111_SetTackState(TACK_ASYNC);
        RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)"<ble.TackArmed = TACK_ASYNC>");
        sprintf( (char *)tempBffr2, "<STATUS>DATA_ASYNC</STATUS>" );
        RoadBrd_UART_Transmit(MONITOR_UART, tempBffr2);
        BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), tempBffr2);
      }
      // Service Watchdog
      RoadBrd_WWDG_Refresh();     // Refresh WatchDog
      // Send </FRM> String.
      sprintf( (char *)tempBffr2, "</FRM>");
      RoadBrd_UART_Transmit(MONITOR_UART, tempBffr2);
      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), tempBffr2);
      sprintf( (char *)tempBffr2, "\r\n\r\n" );
      //BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), tempBffr2);
      RoadBrd_UART_Transmit(MONITOR_UART, tempBffr2);
    } // Endif (BGM111_Ready()
    // Test Analytics flag and determine if we need to update that characteristic
    //  if (!(Tst_HeartBeat()))
    if (analytics.HrtBeat_Cnt < FRM_REPEAT_CNT)
    {
      ClrDataStructure();                           // Clear Backup data structure.
    }
    if (analytics.HrtBeat_Cnt++ >= ANALYTICS_MAXCNT)
    {
      analytics.FrmRpt_Cnt = 0;                     // Clear Frame Repeat Count.
      analytics.HrtBeat_Cnt = 0;
      ClrDataStructure();                           // Clear Backup data structure.
      //    sprintf( (char *)tempStr, "%010dHB", analytics.HrtBeat_Cnt++);
      //    BGM111_WriteCharacteristic(gattdb_AnlHrtBt,
      //                             strlen((char *)tempStr), (uint8_t *)tempStr);
    }
#endif
    RoadBrd_gpio_Off( MICRO_LED );
    // Last....Report Perioic Status of time/Hrtbt_Cnt/Cnct_Cnt
    // ALSO Last....Report Perioic Status of Voltage/Current/Power
    //sprintf( (char *)tempBffr2, " <%s/%s/%s> ", data.Voltage.Voltage, data.Current.Current, data.Power.Power);
    //RoadBrd_UART_Transmit(MONITOR_UART, tempBffr2);
    
    data.RdSnd_Scheduled = false;
  } // EndIf (data.RdSnd_Scheduled)
}


  /**
  * @brief  This function Processes the CMD Mode Interface on a Road Sound Event.
  * @param  None
  * @retval None
  */
void Process_CmdMode( void )
{
//  uint8_t tempStr[13];
  uint8_t tempBffr2[80];
  //static uint8_t analytics.CMD_Md_Cnt = 0;

  /* Is a RdSnd scheduled? */
  if (data.RdSnd_Scheduled)
  {

    // Is CMD_Mode active?
    if ((BGM111_Ready()) &&
        (BGM111_Connected()) &&
          (BGM111_CMD_Mode()) )
    {
      // 1. Increment CMD_Mode Count
      analytics.CMD_Md_Cnt++;
      sprintf( (char *)tempBffr2, "<<%d", analytics.CMD_Md_Cnt );
      RoadBrd_UART_Transmit(MONITOR_UART, tempBffr2);
      
      // Test Cnt against Limit.
      if ( analytics.CMD_Md_Cnt >= RoadBrd_Get_BootDelay())
      {
        RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)">>\r\n");
        // Clear Count for Next Event.
        analytics.CMD_Md_Cnt = 0;
        // Send String to Server to indicate new CMD Mode.
        sprintf( (char *)tempBffr2, "<STATUS>DATA_ASYNC</STATUS>" );
        RoadBrd_UART_Transmit(MONITOR_UART, tempBffr2);
        BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), tempBffr2);
        // Clear CMD_Mode.
        BGM111_SetCMD_Mode( false );
        // Set Data_Connection Mode.
        BGM111_SetDataConnected( true );
        // Change RD_Sound Timer to correct value for Data Mode.
        // First Reload FLASH Frames
        RoadBrd_WWDG_VerifyFrame();
        // NOW...Reload Active Timer.
        Set_RdSndTickCnt( RoadBrd_Get_RdSndTickCnt() );
      } // EndIf ( CMD_Md_Cnt >= CMD_MODE_LMT)
      data.RdSnd_Scheduled = false;
    } // EndIf ((BGM111_Ready()) && (BGM111_Connected()) && (BGM111_CMD_Mode()) )
        
  } // EndIf (data.RdSnd_Scheduled)
}


/**
  * @brief  This function streams the passed string to the App via characteristics..
  * @param  uint8_t *pData
  * @retval None
  */
void SendApp_String( uint8_t *pData )
{
  uint8_t tempBffr3[25];
  uint8_t *tempPtr;
  //int tempval;

  if (BGM111_Ready())
  {
    strncpy( (char *)tempBffr3, (char *)pData, 20);
    //BGM111_WriteCharacteristic(gattdb_AnlHrtBt,
    //                          strlen((char *)tempBffr3), (uint8_t *)tempBffr3);
    //tempval = strlen((char *)pData);
    //if (tempval > 20)
    if (strlen((char *)pData) > 20)
      tempPtr = &pData[20];
    else
      tempPtr = &pData[0];
    strncpy( (char *)tempBffr3, (char *)tempPtr, 20);
    //BGM111_WriteCharacteristic(gattdb_AnlHrtBt2,
    //                          strlen((char *)tempBffr3), (uint8_t *)tempBffr3);
  }
}

/**
  * @brief  This function Tests for an active connection.
  * @param  None
  * @retval None
  */
void Test_Connection( void )
{
//  uint8_t tempBffr2[40];
  
  // Test HAL Count
/*  if (HAL_GetTick() >= HAL_TIMEOUT_CNT)
  {
    // Has been 90 Seconds....Time to reset Code.
    RdBrd_ErrCdLogErrCd( ERROR_BGM_CNNCT, MODULE_bgm111 );
    Clr_HrtBeat_Cnt();
    RdBrd_BlinkErrCd( ERROR_BGM_CNNCT );
    //RoadBrd_Delay( 1000 );
    HAL_NVIC_SystemReset();
  }*/
  
  // Test Connection
  if ( BGM111_Connected() )
  {
    // Yes...Clear count
   connection_cnt = 0;
    // Test Heart Beat. Has it been cleared?
    if (Tst_HeartBeat())
    {
      // No We need to watch this closely.
      // Test Heart Beat Count and determine if time to reset.
      HeartBeat_Cnt++;
      // Time to Build Status?
//      if ( (HeartBeat_Cnt % 5) == 0 )
//      {
//        sprintf( (char *)tempBffr2, " \r\n<HB:%04x/%08x> ", HeartBeat_Cnt, HAL_GetTick());
//        RoadBrd_UART_Transmit(MONITOR_UART, tempBffr2);
//        SendApp_String( tempBffr2 );
//      }
      // Test Heart Beat Count. If expired, reset.
//      if (HeartBeat_Cnt > HEARTBEAT_CNT)
//      {
        // Has been 30 Seconds....Time to reset Code.
//        RdBrd_ErrCdLogErrCd( ERROR_BGM_HRTBT, MODULE_bgm111 );
//        HeartBeat_Cnt = 0;
//        Clr_HrtBeat_Cnt();
        //RoadBrd_Delay( 1000 );
//        RdBrd_BlinkErrCd( ERROR_BGM_HRTBT );
//        HAL_NVIC_SystemReset();
//      }
    } // EndIf (Tst_HeartBeat())
    else
    {
      // OK. Clear Count.
      HeartBeat_Cnt = 0;
      // Set Heart Beat Flag for next Sequence.
      Set_HeartBeat();
    }
  } // EndIf ( BGM111_Connected() )
  else
  {
    // No..
    connection_cnt++;
    // Test Connection Count. If expired, reset.
//    if ( (connection_cnt % 5) == 0 )
//    {
//      sprintf( (char *)tempBffr2, " \r\n<CNCT:%04x/%08x> ", connection_cnt, HAL_GetTick());
//      RoadBrd_UART_Transmit(MONITOR_UART, tempBffr2);
//    }
//    if (connection_cnt > CONNECTION_CNT)
//    {
      // Has been 90 Seconds....Time to reset Code.
//      RdBrd_ErrCdLogErrCd( ERROR_BGM_CNNCT, MODULE_bgm111 );
//      Clr_HrtBeat_Cnt();
//      RdBrd_BlinkErrCd( ERROR_BGM_CNNCT );
      //RoadBrd_Delay( 1000 );
//      HAL_NVIC_SystemReset();
//    }
  } // EndElse ( BGM111_Connected() )
}

  /**
  * @brief  This function resets the driver State List to all off.
  * @param  None
  * @retval None
  */
void Reset_DriverStates( void )
{
  driver_list.VoltageMonitor = DRIVER_OFF;
  driver_list.Temperature = DRIVER_OFF;
  driver_list.Humidity = DRIVER_OFF;
  driver_list.Pressure = DRIVER_OFF;
  driver_list.Irradiance = DRIVER_OFF;
  driver_list.GridEye = DRIVER_OFF;
  driver_list.CoolEye = DRIVER_OFF;
  driver_list.I2CState = DRIVER_OFF;
}

  /**
  * @brief  This function updates the Driver list based on parameter passed.
  * @param  task_defs Task: Driver State to be modified, bool New State for Driver
  * @retval None
  */
void Set_DriverStates( task_defs Task, bool State )
{
  switch(Task)
  {
  case VOLTAGE_MNTR_TASK:
    driver_list.VoltageMonitor = State;
    break;
  case TEMPERATURE_MNTR_TASK:
    driver_list.Temperature = State;
    break;
  case IRRADIANCE_MNTR_TASK:
    driver_list.Irradiance = State;
    break;
  case PRESSURE_MNTR_TASK:
    driver_list.Pressure = State;
    break;
  case HUMIDITY_MNTR_TASK:
    driver_list.Humidity = State;
    break;
  case GRIDEYE_MNTR_TASK:
    driver_list.GridEye = State;
    break;
  case COOLEYE_MNTR_TASK:
    driver_list.CoolEye = State;
    break;
  case I2C_STATE:
    driver_list.I2CState = State;
    break;
  case FRAME_TASK:
    driver_list.FrameState = State;
    break;
  case CAL_TASK:
    driver_list.CalibrationState = State;
    break;
  default:
    break;
  }
}

  /**
  * @brief  This function returns the status based on parameter passed.
  * @param  task_defs Task: Driver State to be modified
  * @retval bool State for Driver
  */
bool Get_DriverStates( task_defs Task )
{
  switch(Task)
  {
  case VOLTAGE_MNTR_TASK:
    return driver_list.VoltageMonitor;
    break;
  case TEMPERATURE_MNTR_TASK:
    return driver_list.Temperature;
    break;
  case IRRADIANCE_MNTR_TASK:
    return driver_list.Irradiance;
    break;
  case PRESSURE_MNTR_TASK:
    return driver_list.Pressure;
    break;
  case HUMIDITY_MNTR_TASK:
    return driver_list.Humidity;
    break;
  case GRIDEYE_MNTR_TASK:
    return driver_list.GridEye;
    break;
  case COOLEYE_MNTR_TASK:
    return driver_list.CoolEye;
    break;
  case I2C_STATE:
    return driver_list.I2CState;
    break;
  case FRAME_TASK:
    return driver_list.FrameState;
    break;
  case CAL_TASK:
    return driver_list.CalibrationState;
    break;
  default:
    return DRIVER_OFF;
    break;
  }
}

  /**
  * @brief  This function returns the status of all Drivers.
  * @param  task_defs Task: Driver State to be modified
  * @retval bool State for Driver
  */
uint16_t Get_DriverStatus( void )
{
  uint16_t Status = 0x00;
  
  if ( Get_DriverStates( VOLTAGE_MNTR_TASK ) )
    Status += 0x0001;
  if ( Get_DriverStates( TEMPERATURE_MNTR_TASK ) )
    Status += 0x0002;
  if ( Get_DriverStates( IRRADIANCE_MNTR_TASK ) )
    Status += 0x0004;
  if ( Get_DriverStates( PRESSURE_MNTR_TASK ) )
    Status += 0x0008;
  if ( Get_DriverStates( HUMIDITY_MNTR_TASK ) )
    Status += 0x0010;
  if ( Get_DriverStates( GRIDEYE_MNTR_TASK ) )
    Status += 0x0020;
  if ( Get_DriverStates( COOLEYE_MNTR_TASK ) )
    Status += 0x0040;
  if ( Get_DriverStates( I2C_STATE ) )
    Status += 0x0080;
  if ( Get_DriverStates( FRAME_TASK ) )
    Status += 0x0100;
  if ( Get_DriverStates( CAL_TASK ) )
    Status += 0x0200;
  return Status;
}

  /**
  * @brief  This function Clears the Heart Beat Count.
  * @param  None
  * @retval bool: Status of Heart Beat Flag
  */
void Clr_HrtBeat_Cnt( void )
{
  analytics.HrtBeat_Cnt = 0;
}

  /**
  * @brief  This function returns the status of the Heart Beat Flag.
  * @param  None
  * @retval bool: Status of Heart Beat Flag
  */
bool Tst_HeartBeat( void )
{
  return analytics.HrtBeat_Flg;
}

  /**
  * @brief  This function sets the Heart Beat Flag.
  * @param  None
  * @retval None
  */
void Set_HeartBeat( void )
{
  analytics.HrtBeat_Flg = true;
}

  /**
  * @brief  This function clears the Heart Beat Flag.
  * @param  None
  * @retval None
  */
void Clr_HeartBeat( void )
{
  analytics.HrtBeat_Flg = false;
}

  /**
  * @brief  This function clears the CMD_Md_Cnt.
  * @param  None
  * @retval None
  */
void Clr_CMD_Md_Cnt( void )
{
  analytics.CMD_Md_Cnt = 0;                     // Clear CMD_Md_Cnt.
}

  /**
  * @brief  This function clears the CMD_Md_Cnt.
  * @param  None
  * @retval None
  */
void Clr_Frame_Ratio( void )
{
  analytics.Frame_Ratio = 0;                     // Clear Frame_Ratio.
}

  /**
  * @brief  This function clears Frame Repeat Count.
  * @param  None
  * @retval None
  */
void ClrAnalyticsRepeat( void )
{
      analytics.FrmRpt_Cnt = 0;                     // Clear Frame Repeat Count.
}

  /**
  * @brief  This function returns a formatted System String.
  * @param  None
  * @retval char *: Pointer to System Time String.
  */
char *getTickString( void )
{
  static char tempstr[15];
  int   hours, minutes, seconds, milliseconds;
  uint32_t TickCnt;
  
  TickCnt = HAL_GetTick();
  
  seconds = (int)floor((float)TickCnt/1000);
  milliseconds = TickCnt % 1000;
  
  minutes = (int)floor((float)seconds/60);
  seconds = seconds % 60;
  
  hours = (int)floor((float)minutes/60);
  minutes = minutes % 60;
  
  sprintf( tempstr, "%02d:%02d:%02d.%03d", hours, minutes, seconds, milliseconds);
  return tempstr;
}

