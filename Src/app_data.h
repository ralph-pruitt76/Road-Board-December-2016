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
#define PROCESS_SNSR_TIME       10      // Process all sensors every Second(10 * 100ms tick)
#define PROCESS_RD_SND_TIME     53      // Process all Road Sound every Second(53 * 100ms tick)...5.3 Seconds
#define PROCESS_LEDOFF_TIME     1       // Process and turn off all active LEDs. Controls the Blink rate of the LEDs(1 * 50ms tick)

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
  TASK_LENGTH = 8
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

#endif