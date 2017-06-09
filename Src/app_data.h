/* Application data manager header file
 * Takes care of scheduling sample times and formatting the sensor data
 * to be sent as BLE characteristics to the app. */

#ifndef APP_DATA_H_
#define APP_DATA_H_

#include "stm32l1xx.h"
#include "bgm111.h"
#include "main.h"

/* Sample timer definitions */

#define SAMPLE_TIM              TIM6
#define SAMPLE_TIM_RCC          RCC_APB1Periph_TIM6
#define SAMPLE_TIM_IRQn         TIM6_IRQn
#define SAMPLE_TIM_IRQHandler   TIM6_IRQHandler

#define PROCESS_SNSR_TIME       100      // Process all sensors every Second(100 * 100ms tick)...10.0 Seconds
#define ANALYTICS_MAXCNT        14        // 180 Seconds
#define PROCESS_RD_SND_TIME     123      // Process all Road Sound every Second(123 * 100ms tick)...12.3 Seconds
#define PROCESS_LEDOFF_TIME     1       // Process and turn off all active LEDs. Controls the Blink rate of the LEDs(1 * 50ms tick)
#ifdef LONG_DELAY
//  #define CONNECTION_CNT          180      // 15 Minutes.
//  #define HEARTBEAT_CNT           60       // 5 Minutes
//  #define CONNECTION_CNT          36       // 3 Minutes.
//  #define HEARTBEAT_CNT           36       // 3 Minutes
  #define CONNECTION_CNT          3600       // 300 Minutes.
  #define HEARTBEAT_CNT           3600       // 300 Minutes
#else
  #define CONNECTION_CNT          18      // 90 Seconds.
  #define HEARTBEAT_CNT           6       // 30 Seconds
#endif
#define HAL_TIMEOUT_CNT           240000    // 4m Minutes in 10 ns Ticks/ 0003.A980
                                            // 10 Seconds is 10000
                                            // 4 Minutes = 240 Seconds = 240000ms

typedef enum 
{
  VOLTAGE_MNTR_TASK = 0,
  TEMPERATURE_MNTR_TASK = 1,
  IRRADIANCE_MNTR_TASK = 2,
  PRESSURE_MNTR_TASK = 3,
  HUMIDITY_MNTR_TASK = 4,
  GRIDEYE_MNTR_TASK = 5,
  COOLEYE_MNTR_TASK = 6,
  I2C_STATE = 7,
  FRAME_TASK = 8,
  TASK_LENGTH = 9
} task_defs;

typedef enum 
{
  DRIVER_OFF = 0,
  DRIVER_ON = 1
} driver_state;

/* Initialize all sensors */

void InitSensors(void);
void SetDataReady( void );
   
/* Process sensor state machine */

HAL_StatusTypeDef  ProcessSensorState(void);
void Process_RdSound( void );

void Reset_DriverStates( void );
void Set_DriverStates( task_defs Task, bool State );
bool Get_DriverStates( task_defs Task );
uint16_t Get_DriverStatus( void );
void Test_Connection( void );
bool Tst_HeartBeat( void );
void Set_HeartBeat( void );
void Clr_HeartBeat( void );
void Clr_HrtBeat_Cnt( void );
void SendApp_String( uint8_t *pData );
void ClrDataStructure(void);

#endif