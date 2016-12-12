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

/* Characteristic handles */
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
#define gattdb_ShntVltgRw                       8
#define gattdb_ShntVltg                        12
#define gattdb_CurrentRw                       16
#define gattdb_Current                         20
#define gattdb_PowerRw                         24
#define gattdb_Power                           28
#define gattdb_VoltageRw                       32
#define gattdb_Voltage                         36
#define gattdb_TemperatureRw                   40
#define gattdb_TemperatureC                    44
#define gattdb_TemperatureF                    48
#define gattdb_PressureRw                      52
#define gattdb_Pressure                        56
#define gattdb_PrTemperatureRw                 60
#define gattdb_PrTemperatureC                  64
#define gattdb_PrTemperatureF                  68
#define gattdb_HumidityRw                      72
#define gattdb_Humidity                        76
#define gattdb_HmdtyTempRw                     80
#define gattdb_HmdtyTempC                      84
#define gattdb_HmdtyTempF                      88
#define gattdb_RGBLightRw                      93
#define gattdb_RGBLightRd                      97
#define gattdb_RGBLightGrn                    101
#define gattdb_RGBLightBlu                    105
#define gattdb_RdSound0                       110
#define gattdb_RdSound16                      114
#define gattdb_RdSound32                      118
#define gattdb_RdSound48                      122
#define gattdb_ThermistorRw                   127
#define gattdb_ThermistorC                    131
#define gattdb_Thermal_1Rw                    135
#define gattdb_Thermal_1RwC                   139
#define gattdb_Thermal_2Rw                    143
#define gattdb_Thermal_2RwC                   147
#define gattdb_Thermal_3Rw                    151
#define gattdb_Thermal_3RwC                   155
#define gattdb_Thermal_4Rw                    159
#define gattdb_Thermal_4RwC                   163
#define gattdb_Thermal_5Rw                    167
#define gattdb_Thermal_5RwC                   171
#define gattdb_Thermal_6Rw                    175
#define gattdb_Thermal_6RwC                   179
#define gattdb_Thermal_7Rw                    183
#define gattdb_Thermal_7RwC                   187
#define gattdb_Thermal_8Rw                    191
#define gattdb_Thermal_8RwC                   195
#define gattdb_xgatt_temp0                    200
#define gattdb_xgatt_temp1                    204
#define gattdb_xgatt_temp2                    208
#define gattdb_xgatt_temp3                    212
#define gattdb_xgatt_temp4                    216
#define gattdb_xgatt_temp5                    220
#define gattdb_xgatt_temp6                    224
#define gattdb_xgatt_temp7                    228
#define gattdb_xgatt_temp8                    232
#define gattdb_xgatt_fuel                     236
#define gattdb_xgatt_humidity                 240
#define gattdb_xgatt_barometer                244
#define gattdb_xgatt_lightning                248
#define gattdb_xgatt_rev                      252
#define gattdb_xgatt_fft1                     256
#define gattdb_xgatt_fft2                     260
#define gattdb_xgatt_fft3                     264
#define gattdb_xgatt_fft4                     268
#endif

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
} static driver_list;

/* App data measurment structure */

typedef struct
{
  volatile bool         reading_scheduled;
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
  data.task_item = VOLTAGE_MNTR_TASK;           // Initialize the task_item to first item.
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

/* Sample timer interrupt handler */

void SAMPLE_TIM_IRQHandler(void)
{
  /* Clear the interrupt flag */
  //TIM_ClearITPendingBit(SAMPLE_TIM, TIM_IT_Update);
  /* Schedule a sensor reading */
  data.reading_scheduled = true;
}

/* Process sensor state machine */

HAL_StatusTypeDef  ProcessSensorState(void)
{
  HAL_StatusTypeDef Status;
  uint8_t tmpBuffer[10];
  
  Status = HAL_OK;

  /* Is a reading scheduled? */
  if (data.reading_scheduled)
  {
#if 0
//************************OLD STYLE TASKING STARTS HERE************************
// OLD STYLE CODE. Does not allow flexible tasking. Possible hole in design.
    /* Clear the scheduling flag */
    data.reading_scheduled = false;
    
    /* Read the Voltage Monitor Data. */
    Status = RoadBrd_VMonitor_RdShntVltg( &TmpData.ShntVltg );
    if (Status != HAL_OK)
      return Status;
    Status = RoadBrd_VMonitor_RdCurrent( &TmpData.Current );
    if (Status != HAL_OK)
      return Status;
    Status = RoadBrd_VMonitor_RdPower( &TmpData.Power );
    if (Status != HAL_OK)
      return Status;
    Status = RoadBrd_VMonitor_RdVoltage( &TmpData.Voltage );
   if (Status != HAL_OK)
      return Status;
    /* Read the Temperature Data. */
    Status = RoadBrd_ReadTemp( &TmpData.Temp );
    if (Status != HAL_OK)
      return Status;

    /* Read the Irradiance Data. */
    Status = RoadBrd_RGBReadValues( &TmpData.RGBValues );
    if (Status != HAL_OK)
      return Status;

    /* Read the pressure and temperature */
    Status = RoadBrd_Baro_ReadPressure( &TmpData.Pressure );
    if (Status != HAL_OK)
      return Status;
    Status = RoadBrd_Baro_ReadTemp( &TmpData.PrTemp );
    if (Status != HAL_OK)
      return Status;
    
    /* Read the Humidity and temperature */
    Status = RoadBrd_Humidity_ReadHumidity( &TmpData.Humidity );
    if (Status != HAL_OK)
      return Status;
    Status = RoadBrd_Humidity_ReadTemperature( &TmpData.HmTemp );
    if (Status != HAL_OK)
      return Status;
    
    /* Read Grid Eye Values */
    Status = RoadBrd_GridEye_ReadValues( &TmpData.GridValues );
    if (Status != HAL_OK)
      return Status;
//************************OLD STYLE TASKING ENDS HERE************************
#else
//************************NEW STYLE TASKING STARTS HERE************************
    switch(data.task_item)
    {
      case VOLTAGE_MNTR_TASK:
        if ( Get_DriverStates( VOLTAGE_MNTR_TASK ))
        {
          /* Read the Voltage Monitor Data. */
          Status = RoadBrd_VMonitor_RdShntVltg( &TmpData.ShntVltg );
          if (Status != HAL_OK)
            break;
          Status = RoadBrd_VMonitor_RdCurrent( &TmpData.Current );
          if (Status != HAL_OK)
            break;
          Status = RoadBrd_VMonitor_RdPower( &TmpData.Power );
          if (Status != HAL_OK)
            break;
          Status = RoadBrd_VMonitor_RdVoltage( &TmpData.Voltage );
          if (Status != HAL_OK)
            break;
        }
        break;
      case TEMPERATURE_MNTR_TASK:
        if ( Get_DriverStates( TEMPERATURE_MNTR_TASK ))
        {
          /* Read the Temperature Data. */
          Status = RoadBrd_ReadTemp( &TmpData.Temp );
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
          Status = RoadBrd_Baro_ReadPressure( &TmpData.Pressure );
          if (Status != HAL_OK)
            break;
          Status = RoadBrd_Baro_ReadTemp( &TmpData.PrTemp );
        }
        break;
      case HUMIDITY_MNTR_TASK:
        if ( Get_DriverStates( HUMIDITY_MNTR_TASK ))
        {
          /* Read the Humidity and temperature */
          Status = RoadBrd_Humidity_ReadHumidity( &TmpData.Humidity );
          if (Status != HAL_OK)
            break;
          Status = RoadBrd_Humidity_ReadTemperature( &TmpData.HmTemp );
        }
        break;
      case GRIDEYE_MNTR_TASK:
        if ( Get_DriverStates( GRIDEYE_MNTR_TASK ))
        {
          Status = RoadBrd_GridEye_ReadValues( &TmpData.GridValues );
        }
        else if ( Get_DriverStates( COOLEYE_MNTR_TASK ))
        {
          Status = RoadBrd_CoolEye_ReadValues( &TmpData.GridValues );
        }
        break;
    }
    // Update Count
    data.task_item++;
    if (data.task_item >= TASK_LENGTH)
    {
      // Reset Count
      data.task_item = VOLTAGE_MNTR_TASK;
      /* Clear the scheduling flag */
      data.reading_scheduled = false;
    }
    if (Status != HAL_OK)
      return Status;
//************************NEW STYLE TASKING ENDS HERE************************
#endif
    //************ NOW Compare the data strings and determine if characteristics need to be sent.
    //..ShntVltg
#ifndef LEGACY_PATCH
    if(strcmp( (char *)TmpData.ShntVltg.Raw, (char *)data.ShntVltg.Raw) != 0 )
    {
      RoadBrd_gpio_On( BGM_LED );
      // Update Information in data structure
      strcpy( (char *)data.ShntVltg.Raw, (char *)TmpData.ShntVltg.Raw );
      strcpy( (char *)data.ShntVltg.Voltage, (char *)TmpData.ShntVltg.Voltage );
      // Update BLE Characteristics
      /* Send the ShntVltgRw to the BLE module */
      BGM111_WriteCharacteristic(gattdb_ShntVltgRw,
                                 strlen((char *)data.ShntVltg.Raw), (uint8_t *)data.ShntVltg.Raw);
      /* Send the ShntVltg to the BLE module */
      BGM111_WriteCharacteristic(gattdb_ShntVltg,
                                 strlen((char *)data.ShntVltg.Voltage), (uint8_t *)data.ShntVltg.Voltage);
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
      BGM111_WriteCharacteristic(gattdb_CurrentRw,
                                 strlen((char *)data.Current.Raw), (uint8_t *)data.Current.Raw);
      /* Send the Current to the BLE module */
      BGM111_WriteCharacteristic(gattdb_Current,
                                 strlen((char *)data.Current.Current), (uint8_t *)data.Current.Current);
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
      BGM111_WriteCharacteristic(gattdb_PowerRw,
                                 strlen((char *)data.Power.Raw), (uint8_t *)data.Power.Raw);
      /* Send the Power to the BLE module */
      BGM111_WriteCharacteristic(gattdb_Power,
                                 strlen((char *)data.Power.Power), (uint8_t *)data.Power.Power);
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
      BGM111_WriteCharacteristic(gattdb_VoltageRw,
                                 strlen((char *)data.Voltage.Raw), (uint8_t *)data.Voltage.Raw);
      /* Send the Voltage to the BLE module */
      BGM111_WriteCharacteristic(gattdb_Voltage,
                                 strlen((char *)data.Voltage.Voltage), (uint8_t *)data.Voltage.Voltage);
    }
#endif
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
      BGM111_WriteCharacteristic(gattdb_TemperatureRw,
                                 strlen((char *)data.Temp.Raw), (uint8_t *)data.Temp.Raw);
      /* Send the TemperatureC to the BLE module */
      BGM111_WriteCharacteristic(gattdb_TemperatureC,
                                 strlen((char *)data.Temp.TempC), (uint8_t *)data.Temp.TempC);
      /* Send the TemperatureF to the BLE module */
      BGM111_WriteCharacteristic(gattdb_TemperatureF,
                                 strlen((char *)data.Temp.TempF), (uint8_t *)data.Temp.TempF);
#endif
      // Update Legacy Temperature Characteristic...
      tmpBuffer[0] = (uint8_t)('T');
      tmpBuffer[1] = (uint8_t)(0x00);
      tmpBuffer[2] = (uint8_t)(data.Temp.RawC & 0x00ff);
      tmpBuffer[3] = (uint8_t)((data.Temp.RawC & 0xff00) >> 8);
      /* Send the Temperature RawC to the BLE module */
      BGM111_WriteCharacteristic(gattdb_xgatt_temp0,
                                 0x04, (uint8_t *)tmpBuffer);
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
      BGM111_WriteCharacteristic(gattdb_RGBLightRw,
                                 strlen((char *)data.RGBValues.Raw), (uint8_t *)data.RGBValues.Raw);
      /* Send the RGBLightRd to the BLE module */
      BGM111_WriteCharacteristic(gattdb_RGBLightRd,
                                 strlen((char *)data.RGBValues.Red), (uint8_t *)data.RGBValues.Red);
      /* Send the RGBLightGrn to the BLE module */
      BGM111_WriteCharacteristic(gattdb_RGBLightGrn,
                                 strlen((char *)data.RGBValues.Green), (uint8_t *)data.RGBValues.Green);
      /* Send the RGBLightBlu to the BLE module */
      BGM111_WriteCharacteristic(gattdb_RGBLightBlu,
                                 strlen((char *)data.RGBValues.Blue), (uint8_t *)data.RGBValues.Blue);
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
      BGM111_WriteCharacteristic(gattdb_PressureRw,
                                 strlen((char *)data.Pressure.Raw), (uint8_t *)data.Pressure.Raw);
      /* Send the Pressure to the BLE module */
      BGM111_WriteCharacteristic(gattdb_Pressure,
                                 strlen((char *)data.Pressure.Pressure), (uint8_t *)data.Pressure.Pressure);
      // Update Legacy Pressure Characteristic..
      tmpBuffer[0] = (uint8_t)('P');
      tmpBuffer[3] = (uint8_t)(data.Pressure.RawC & 0x00ff);
      tmpBuffer[2] = (uint8_t)((data.Pressure.RawC & 0xff00) >> 8);
      tmpBuffer[1] = (uint8_t)((data.Pressure.RawC & 0xff0000) >> 16);
      /* Send the Pressure RawC to the BLE module */
      BGM111_WriteCharacteristic(gattdb_xgatt_barometer,
                                 0x04, (uint8_t *)tmpBuffer);
    }
    if(strcmp( (char *)TmpData.PrTemp.Raw, (char *)data.PrTemp.Raw) != 0 )
    {
      RoadBrd_gpio_On( BGM_LED );
      // Update Information in data structure
      strcpy( (char *)data.PrTemp.Raw, (char *)TmpData.PrTemp.Raw );
      strcpy( (char *)data.PrTemp.TempC, (char *)TmpData.PrTemp.TempC );
      strcpy( (char *)data.PrTemp.TempF, (char *)TmpData.PrTemp.TempF );
      // Update BLE Characteristics
      /* Send the PrTemp to the BLE module */
      BGM111_WriteCharacteristic(gattdb_PrTemperatureRw,
                                 strlen((char *)data.PrTemp.Raw), (uint8_t *)data.PrTemp.Raw);
      /* Send the PrTemp to the BLE module */
      BGM111_WriteCharacteristic(gattdb_PrTemperatureC,
                                 strlen((char *)data.PrTemp.TempC), (uint8_t *)data.PrTemp.TempC);
      /* Send the PrTemp to the BLE module */
      BGM111_WriteCharacteristic(gattdb_PrTemperatureF,
                                 strlen((char *)data.PrTemp.TempF), (uint8_t *)data.PrTemp.TempF);
    } 
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
      BGM111_WriteCharacteristic(gattdb_HumidityRw,
                                 strlen((char *)data.Humidity.HRaw), (uint8_t *)data.Humidity.HRaw);
      /* Send the data to the BLE module */
      BGM111_WriteCharacteristic(gattdb_Humidity,
                                 strlen((char *)data.Humidity.Humidity), (uint8_t *)data.Humidity.Humidity);
      // Update Legacy Humidity Characteristic..
      tmpBuffer[0] = (uint8_t)('H');
      tmpBuffer[2] = (uint8_t)(data.Humidity.HRawC & 0x00ff);
      tmpBuffer[1] = (uint8_t)((data.Humidity.HRawC & 0xff00) >> 8);
      tmpBuffer[3] = (uint8_t)0x00;
      /* Send the Temperature RawC to the BLE module */
      BGM111_WriteCharacteristic(gattdb_xgatt_humidity,
                                 0x03, (uint8_t *)tmpBuffer);
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
      BGM111_WriteCharacteristic(gattdb_HmdtyTempRw,
                                 strlen((char *)data.HmTemp.Raw), (uint8_t *)data.HmTemp.Raw);
      /* Send the HmTemp to the BLE module */
      BGM111_WriteCharacteristic(gattdb_HmdtyTempC,
                                 strlen((char *)data.HmTemp.TempC), (uint8_t *)data.HmTemp.TempC);
      /* Send the HmTemp to the BLE module */
      BGM111_WriteCharacteristic(gattdb_HmdtyTempF,
                                 strlen((char *)data.HmTemp.TempF), (uint8_t *)data.HmTemp.TempF);
    } 
    //..GridEye...Thermistor
    if(strcmp( (char *)TmpData.GridValues.Thermistor.Raw, (char *)data.GridValues.Thermistor.Raw) != 0 )
    {
      RoadBrd_gpio_On( BGM_LED );
      // Update Information in data structure
      strcpy( (char *)data.GridValues.Thermistor.Raw, (char *)TmpData.GridValues.Thermistor.Raw );
      strcpy( (char *)data.GridValues.Thermistor.TempC, (char *)TmpData.GridValues.Thermistor.TempC );
      // Update BLE Characteristics
      /* Send the PressureRw to the BLE module */
      BGM111_WriteCharacteristic(gattdb_ThermistorRw,
                                 strlen((char *)data.GridValues.Thermistor.Raw), (uint8_t *)data.GridValues.Thermistor.Raw);
      /* Send the Pressure to the BLE module */
      BGM111_WriteCharacteristic(gattdb_ThermistorC,
                                 strlen((char *)data.GridValues.Thermistor.TempC), (uint8_t *)data.GridValues.Thermistor.TempC);
    }
#endif
    //..GridEye...Grid 1
    if(strcmp( (char *)TmpData.GridValues.GridEye1.Raw, (char *)data.GridValues.GridEye1.Raw) != 0 )
    {
      RoadBrd_gpio_On( BGM_LED );
      // Update Information in data structure
      strcpy( (char *)data.GridValues.GridEye1.Raw, (char *)TmpData.GridValues.GridEye1.Raw );
      strcpy( (char *)data.GridValues.GridEye1.TempC, (char *)TmpData.GridValues.GridEye1.TempC );
      data.GridValues.GridEye1.RawC = TmpData.GridValues.GridEye1.RawC;
      // Update BLE Characteristics
      /* Send the PressureRw to the BLE module */
#ifndef LEGACY_PATCH
      BGM111_WriteCharacteristic(gattdb_Thermal_1Rw,
                                 strlen((char *)data.GridValues.GridEye1.Raw), (uint8_t *)data.GridValues.GridEye1.Raw);
      /* Send the Pressure to the BLE module */
      BGM111_WriteCharacteristic(gattdb_Thermal_1RwC,
                                 strlen((char *)data.GridValues.GridEye1.TempC), (uint8_t *)data.GridValues.GridEye1.TempC);
#endif
      // Update Legacy Temperature Characteristic..
      tmpBuffer[0] = (uint8_t)('T');
      tmpBuffer[1] = (uint8_t)(0x01);
      tmpBuffer[2] = (uint8_t)(data.GridValues.GridEye1.RawC & 0x00ff);
      tmpBuffer[3] = (uint8_t)((data.GridValues.GridEye1.RawC & 0xff00) >> 8);
      /* Send the Temperature RawC to the BLE module */
      BGM111_WriteCharacteristic(gattdb_xgatt_temp1,
                                 0x04, (uint8_t *)tmpBuffer);
    }
    //.GridEye..Grid 2
    if(strcmp( (char *)TmpData.GridValues.GridEye2.Raw, (char *)data.GridValues.GridEye2.Raw) != 0 )
    {
      RoadBrd_gpio_On( BGM_LED );
      // Update Information in data structure
      strcpy( (char *)data.GridValues.GridEye2.Raw, (char *)TmpData.GridValues.GridEye2.Raw );
      strcpy( (char *)data.GridValues.GridEye2.TempC, (char *)TmpData.GridValues.GridEye2.TempC );
      data.GridValues.GridEye2.RawC = TmpData.GridValues.GridEye2.RawC;
      // Update BLE Characteristics
      /* Send the PressureRw to the BLE module */
#ifndef LEGACY_PATCH
      BGM111_WriteCharacteristic(gattdb_Thermal_2Rw,
                                 strlen((char *)data.GridValues.GridEye2.Raw), (uint8_t *)data.GridValues.GridEye2.Raw);
      /* Send the Pressure to the BLE module */
      BGM111_WriteCharacteristic(gattdb_Thermal_2RwC,
                                 strlen((char *)data.GridValues.GridEye2.TempC), (uint8_t *)data.GridValues.GridEye2.TempC);
#endif
      // Update Legacy Temperature Characteristic..
      tmpBuffer[0] = (uint8_t)('T');
      tmpBuffer[1] = (uint8_t)(0x02);
      tmpBuffer[2] = (uint8_t)(data.GridValues.GridEye2.RawC & 0x00ff);
      tmpBuffer[3] = (uint8_t)((data.GridValues.GridEye2.RawC & 0xff00) >> 8);
      /* Send the Temperature RawC to the BLE module */
      BGM111_WriteCharacteristic(gattdb_xgatt_temp2,
                                 0x04, (uint8_t *)tmpBuffer);
    }
    //.GridEye..Grid 3
    if(strcmp( (char *)TmpData.GridValues.GridEye3.Raw, (char *)data.GridValues.GridEye3.Raw) != 0 )
    {
      RoadBrd_gpio_On( BGM_LED );
      // Update Information in data structure
      strcpy( (char *)data.GridValues.GridEye3.Raw, (char *)TmpData.GridValues.GridEye3.Raw );
      strcpy( (char *)data.GridValues.GridEye3.TempC, (char *)TmpData.GridValues.GridEye3.TempC );
      data.GridValues.GridEye3.RawC = TmpData.GridValues.GridEye3.RawC;
      // Update BLE Characteristics
      /* Send the PressureRw to the BLE module */
#ifndef LEGACY_PATCH
      BGM111_WriteCharacteristic(gattdb_Thermal_3Rw,
                                 strlen((char *)data.GridValues.GridEye3.Raw), (uint8_t *)data.GridValues.GridEye3.Raw);
      /* Send the Pressure to the BLE module */
      BGM111_WriteCharacteristic(gattdb_Thermal_3RwC,
                                 strlen((char *)data.GridValues.GridEye3.TempC), (uint8_t *)data.GridValues.GridEye3.TempC);
#endif
      // Update Legacy Temperature Characteristic..
      tmpBuffer[0] = (uint8_t)('T');
      tmpBuffer[1] = (uint8_t)(0x04);
      tmpBuffer[2] = (uint8_t)(data.GridValues.GridEye3.RawC & 0x00ff);
      tmpBuffer[3] = (uint8_t)((data.GridValues.GridEye3.RawC & 0xff00) >> 8);
      /* Send the Temperature RawC to the BLE module */
      BGM111_WriteCharacteristic(gattdb_xgatt_temp3,
                                 0x04, (uint8_t *)tmpBuffer);
    }
    //.GridEye..Grid 4
    if(strcmp( (char *)TmpData.GridValues.GridEye4.Raw, (char *)data.GridValues.GridEye4.Raw) != 0 )
    {
      RoadBrd_gpio_On( BGM_LED );
      // Update Information in data structure
      strcpy( (char *)data.GridValues.GridEye4.Raw, (char *)TmpData.GridValues.GridEye4.Raw );
      strcpy( (char *)data.GridValues.GridEye4.TempC, (char *)TmpData.GridValues.GridEye4.TempC );
      data.GridValues.GridEye4.RawC = TmpData.GridValues.GridEye4.RawC;
      // Update BLE Characteristics
      /* Send the PressureRw to the BLE module */
#ifndef LEGACY_PATCH
      BGM111_WriteCharacteristic(gattdb_Thermal_4Rw,
                                 strlen((char *)data.GridValues.GridEye4.Raw), (uint8_t *)data.GridValues.GridEye4.Raw);
      /* Send the Pressure to the BLE module */
      BGM111_WriteCharacteristic(gattdb_Thermal_4RwC,
                                 strlen((char *)data.GridValues.GridEye4.TempC), (uint8_t *)data.GridValues.GridEye4.TempC);
#endif
      // Update Legacy Temperature Characteristic..
      tmpBuffer[0] = (uint8_t)('T');
      tmpBuffer[1] = (uint8_t)(0x04);
      tmpBuffer[2] = (uint8_t)(data.GridValues.GridEye4.RawC & 0x00ff);
      tmpBuffer[3] = (uint8_t)((data.GridValues.GridEye4.RawC & 0xff00) >> 8);
      /* Send the Temperature RawC to the BLE module */
      BGM111_WriteCharacteristic(gattdb_xgatt_temp4,
                                 0x04, (uint8_t *)tmpBuffer);
    }
    //.GridEye..Grid 5
    if(strcmp( (char *)TmpData.GridValues.GridEye5.Raw, (char *)data.GridValues.GridEye5.Raw) != 0 )
    {
      RoadBrd_gpio_On( BGM_LED );
      // Update Information in data structure
      strcpy( (char *)data.GridValues.GridEye5.Raw, (char *)TmpData.GridValues.GridEye5.Raw );
      strcpy( (char *)data.GridValues.GridEye5.TempC, (char *)TmpData.GridValues.GridEye5.TempC );
      data.GridValues.GridEye5.RawC = TmpData.GridValues.GridEye5.RawC;
      // Update BLE Characteristics
      /* Send the PressureRw to the BLE module */
#ifndef LEGACY_PATCH
      BGM111_WriteCharacteristic(gattdb_Thermal_5Rw,
                                 strlen((char *)data.GridValues.GridEye5.Raw), (uint8_t *)data.GridValues.GridEye5.Raw);
      /* Send the Pressure to the BLE module */
      BGM111_WriteCharacteristic(gattdb_Thermal_5RwC,
                                 strlen((char *)data.GridValues.GridEye5.TempC), (uint8_t *)data.GridValues.GridEye5.TempC);
#endif
      // Update Legacy Temperature Characteristic..
      tmpBuffer[0] = (uint8_t)('T');
      tmpBuffer[1] = (uint8_t)(0x05);
      tmpBuffer[2] = (uint8_t)(data.GridValues.GridEye5.RawC & 0x00ff);
      tmpBuffer[3] = (uint8_t)((data.GridValues.GridEye5.RawC & 0xff00) >> 8);
      /* Send the Temperature RawC to the BLE module */
      BGM111_WriteCharacteristic(gattdb_xgatt_temp5,
                                 0x04, (uint8_t *)tmpBuffer);
    }
    //.GridEye..Grid 6
    if(strcmp( (char *)TmpData.GridValues.GridEye6.Raw, (char *)data.GridValues.GridEye6.Raw) != 0 )
    {
      RoadBrd_gpio_On( BGM_LED );
      // Update Information in data structure
      strcpy( (char *)data.GridValues.GridEye6.Raw, (char *)TmpData.GridValues.GridEye6.Raw );
      strcpy( (char *)data.GridValues.GridEye6.TempC, (char *)TmpData.GridValues.GridEye6.TempC );
      data.GridValues.GridEye6.RawC = TmpData.GridValues.GridEye6.RawC;
      // Update BLE Characteristics
      /* Send the PressureRw to the BLE module */
#ifndef LEGACY_PATCH
      BGM111_WriteCharacteristic(gattdb_Thermal_6Rw,
                                 strlen((char *)data.GridValues.GridEye6.Raw), (uint8_t *)data.GridValues.GridEye6.Raw);
      /* Send the Pressure to the BLE module */
      BGM111_WriteCharacteristic(gattdb_Thermal_6RwC,
                                 strlen((char *)data.GridValues.GridEye6.TempC), (uint8_t *)data.GridValues.GridEye6.TempC);
#endif
      // Update Legacy Temperature Characteristic..
      tmpBuffer[0] = (uint8_t)('T');
      tmpBuffer[1] = (uint8_t)(0x06);
      tmpBuffer[2] = (uint8_t)(data.GridValues.GridEye6.RawC & 0x00ff);
      tmpBuffer[3] = (uint8_t)((data.GridValues.GridEye6.RawC & 0xff00) >> 8);
      /* Send the Temperature RawC to the BLE module */
      BGM111_WriteCharacteristic(gattdb_xgatt_temp6,
                                 0x04, (uint8_t *)tmpBuffer);
    }
    //.GridEye..Grid 7
    if(strcmp( (char *)TmpData.GridValues.GridEye7.Raw, (char *)data.GridValues.GridEye7.Raw) != 0 )
    {
      RoadBrd_gpio_On( BGM_LED );
      // Update Information in data structure
      strcpy( (char *)data.GridValues.GridEye7.Raw, (char *)TmpData.GridValues.GridEye7.Raw );
      strcpy( (char *)data.GridValues.GridEye7.TempC, (char *)TmpData.GridValues.GridEye7.TempC );
      data.GridValues.GridEye7.RawC = TmpData.GridValues.GridEye7.RawC;
      // Update BLE Characteristics
      /* Send the PressureRw to the BLE module */
#ifndef LEGACY_PATCH
      BGM111_WriteCharacteristic(gattdb_Thermal_7Rw,
                                 strlen((char *)data.GridValues.GridEye7.Raw), (uint8_t *)data.GridValues.GridEye7.Raw);
      /* Send the Pressure to the BLE module */
      BGM111_WriteCharacteristic(gattdb_Thermal_7RwC,
                                 strlen((char *)data.GridValues.GridEye7.TempC), (uint8_t *)data.GridValues.GridEye7.TempC);
#endif
      // Update Legacy Temperature Characteristic..
      tmpBuffer[0] = (uint8_t)('T');
      tmpBuffer[1] = (uint8_t)(0x07);
      tmpBuffer[2] = (uint8_t)(data.GridValues.GridEye7.RawC & 0x00ff);
      tmpBuffer[3] = (uint8_t)((data.GridValues.GridEye7.RawC & 0xff00) >> 8);
      /* Send the Temperature RawC to the BLE module */
      BGM111_WriteCharacteristic(gattdb_xgatt_temp7,
                                 0x04, (uint8_t *)tmpBuffer);
    }
    //.GridEye..Grid 8
    if(strcmp( (char *)TmpData.GridValues.GridEye8.Raw, (char *)data.GridValues.GridEye8.Raw) != 0 )
    {
      RoadBrd_gpio_On( BGM_LED );
      // Update Information in data structure
      strcpy( (char *)data.GridValues.GridEye8.Raw, (char *)TmpData.GridValues.GridEye8.Raw );
      strcpy( (char *)data.GridValues.GridEye8.TempC, (char *)TmpData.GridValues.GridEye8.TempC );
      data.GridValues.GridEye8.RawC = TmpData.GridValues.GridEye8.RawC;
      // Update BLE Characteristics
      /* Send the PressureRw to the BLE module */
#ifndef LEGACY_PATCH
      BGM111_WriteCharacteristic(gattdb_Thermal_8Rw,
                                 strlen((char *)data.GridValues.GridEye8.Raw), (uint8_t *)data.GridValues.GridEye8.Raw);
      /* Send the Pressure to the BLE module */
      BGM111_WriteCharacteristic(gattdb_Thermal_8RwC,
                                 strlen((char *)data.GridValues.GridEye8.TempC), (uint8_t *)data.GridValues.GridEye8.TempC);
#endif
      // Update Legacy Temperature Characteristic..
      tmpBuffer[0] = (uint8_t)('T');
      tmpBuffer[1] = (uint8_t)(0x08);
      tmpBuffer[2] = (uint8_t)(data.GridValues.GridEye8.RawC & 0x00ff);
      tmpBuffer[3] = (uint8_t)((data.GridValues.GridEye8.RawC & 0xff00) >> 8);
      /* Send the Temperature RawC to the BLE module */
      BGM111_WriteCharacteristic(gattdb_xgatt_temp8,
                                 0x04, (uint8_t *)tmpBuffer);
    }
    // Test Legacy flag to perform one time operations
    if( data.Legacy_OneTime )
    {
      // Clear Flag...We are done.
      data.Legacy_OneTime = false;
      // Update BLE Characteristics
      BGM111_WriteCharacteristic(gattdb_xgatt_rev,
                                 strlen((char *)LEGACY_BANNER), (uint8_t *)LEGACY_BANNER);
    }
 }

  return Status;
}

  /**
  * @brief  This function Processes the Road SOund interface and builds the appropriate characteristics.
  * @param  None
  * @retval None
  */
void Process_RdSound( void )
{
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
  BGM111_WriteCharacteristic(gattdb_RdSound0,
                             strlen((char *)data.FFTBin0.dumpStr), (uint8_t *)data.FFTBin0.dumpStr);
#endif
  /* Send the FFTBin16 to the BLE module */
  RoadBrd_gpio_On( BGM_LED );
#ifndef LEGACY_PATCH
  BGM111_WriteCharacteristic(gattdb_RdSound16,
                             strlen((char *)data.FFTBin16.dumpStr), (uint8_t *)data.FFTBin16.dumpStr);
#endif
  /* Send the FFTBin32 to the BLE module */
  RoadBrd_gpio_On( BGM_LED );
#ifndef LEGACY_PATCH
  BGM111_WriteCharacteristic(gattdb_RdSound32,
                             strlen((char *)data.FFTBin32.dumpStr), (uint8_t *)data.FFTBin32.dumpStr);
#endif
  /* Send the FFTBin48 to the BLE module */
  RoadBrd_gpio_On( BGM_LED );
#ifndef LEGACY_PATCH
  BGM111_WriteCharacteristic(gattdb_RdSound48,
                             strlen((char *)data.FFTBin48.dumpStr), (uint8_t *)data.FFTBin48.dumpStr);
#endif
  RoadBrd_gpio_Off( MICRO_LED );
 }


  /**
  * @brief  This function Tests for an active connection.
  * @param  None
  * @retval None
  */
void Test_Connection( void )
{
  static uint16_t connection_cnt = 0;
  
  // Test Connection
  if ( BGM111_Connected() )
  {
    // Yes...Clear count
    connection_cnt = 0;
    
  }
  else
  {
    // No...Increment Count...
    connection_cnt++;
    
    if (connection_cnt > 18)
    {
      // Has been 90 Seconds....Time to reset Code.
      RdBrd_ErrCdLogErrCd( ERROR_BGM_CNNCT, MODULE_bgm111 );
      //RoadBrd_Delay( 1000 );
      HAL_NVIC_SystemReset();
    }
  }
  
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
  return Status;
}
