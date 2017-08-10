/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
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
#include "stm32l1xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "wwdg.h"
#include "Flash.h"

/* USER CODE BEGIN Includes */
#include "stm32l1xx_nucleo.h"
#ifdef REV_L
  #include "bgm111.h"
  #include "app_data.h"
#endif
#include "gpio.h"
#include "main.h"
#include "parser.h"
#include "stdbool.h"
#include "ErrorCodes.h"
    
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
int main(void)
{
  /* USER CODE BEGIN 1 */
#ifdef TEST2
  #define RECEIVE_SZ      5
  uint8_t tempBffr[RECEIVE_SZ];
  char tempBffr2[5];
  HAL_StatusTypeDef Status;
#else
  #ifdef TEST
    #define RECEIVE_SZ      5
  #else
//HERE
    #define RECEIVE_SZ      30
    uint8_t tempBffr[RECEIVE_SZ];
    uint8_t tempBffr2[80];
    HAL_StatusTypeDef Status;
  #endif
#endif

#ifdef REV_L
  uint16_t tmpSize = RECEIVE_SZ;
  uint8_t tmpData[2];
//  uint8_t tmpData[RECEIVE_SZ];
  Temperature   Temp;
#ifndef PATCH_UART
  bool firstTime = true;
#endif
  uint8_t *pData = tempBffr;
#endif

//ITStatus PStatus;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

#ifdef REV_L
  // Turn Off Power Supplies
  //RoadBrd_gpio_Off(gTAM_PWR);    // Turn Off 5V Power
  //RoadBrd_gpio_Off(gVDD_PWR);    // Turn Off 3.3V Power
#endif

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  ADC_Config();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
#ifdef REV_L
  // Wait for power to stabilize off...200msec
  //RoadBrd_Delay( 200 );
  // Turn on Power Supplies.
  RoadBrd_gpio_On(gTAM_PWR);    // Turn on 5V Power
  RoadBrd_gpio_On(gVDD_PWR);    // Turn on 3.3V Power
  // Wait for power to stabilize...200msec
  RoadBrd_Delay( 200 );
  // Reset all Drivers to Off before starting init process.
  Reset_DriverStates();

  // Enable Interrupts
  //---UART
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_NVIC_EnableIRQ(USART3_IRQn);

  // Test I2C Channel and see if we even have a working I2C.
  RoadBrd_TestI2C();
  
  // Test I2C Status and Task init I2C if Active driver.
  if ( Get_DriverStates( I2C_STATE ) )
  {
    // Now..Initialize I2C and test Drivers.
    MX_I2C1_Init();
  }
  
  //---I2C1
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  //---TIM
#ifndef PATCH_UART
  HAL_TIM_StartTimer2();
  HAL_TIM_StartTimer3();
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
#endif
  // Test I2C State.

  // Test I2C Status and Task init I2C if Active driver.
  if ( Get_DriverStates( I2C_STATE ) )
  {
    // Wait 35  ms. For Busy flag to drop.
    if(I2C_WaitBusyFlag() != HAL_OK)
    {
      RdBrd_ErrCdLogErrCd( ERROR_I2CBUSY, MODULE_main );
    }
  }

  //BGM Initialization
#ifndef TEST2
#ifndef PATCH_UART
  BGM111_Init();
#else
  RoadBrd_gpio_Off(gRESET_BGM111);
#endif
#endif
  
  // Test I2C Status and Task init I2C if Active driver.
  if ( Get_DriverStates( I2C_STATE ) )
  {
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
  } //EndIf ( Get_DriverStates( I2C_STATE )
  
  // Initialize key app vars.
  InitSensors();
  
  //**
  //**
  //** Initialize all Flash Structures.
  //**
  //**
  //*******1. Initializ WWDG Flash Structure
  // 1a. Is WWDG Flash Frame Initialized?
  if (RoadBrd_WWDG_VerifyFrame())
  {
    //Yes....Set FRAME_TASK Bit in Driver State Variable.
    Set_DriverStates( FRAME_TASK, DRIVER_ON );
  } // EndIf (RoadBrd_WWDG_VerifyFrame())
  else
  {
    //No....1b. Attempt to Initialize WWDG Flash Frame.
    if (RoadBrd_WWDG_InitializeFrmFlash() != HAL_OK)
    {
      //FAILED....Indicate Error Code and Fail Driver State.
      RdBrd_ErrCdLogErrCd( ERROR_FRAME_INIT, MODULE_main );
      Set_DriverStates( FRAME_TASK, DRIVER_OFF );
    }
    else
    {
      //SUCCESS....Set FRAME_TASK Bit in Driver State Variable.
      Set_DriverStates( FRAME_TASK, DRIVER_ON );
    }
  } // EndElse (RoadBrd_WWDG_VerifyFrame())
  
  //*******2. Initializ Calibration Flash Structure
  // 2a. Is Calibration Flash Frame Initialized?
  if (RoadBrd_CAL_VerifyFrame())
  {
    //Yes....Set CAL_TASK Bit in Driver State Variable.
    Set_DriverStates( CAL_TASK, DRIVER_ON );
  } // EndIf (RoadBrd_WWDG_VerifyFrame())
  else
  {
    //No....2b. Attempt to Initialize Structure Flash Structure.
    if (RoadBrd_CAL_InitializeFrmFlash() != HAL_OK)
    {
      //FAILED....Indicate Error Code and Fail Driver State.
      RdBrd_ErrCdLogErrCd( ERROR_CAL_INIT, MODULE_main );
      Set_DriverStates( CAL_TASK, DRIVER_OFF );
    }
    else
    {
      //SUCCESS....Set FRAME_TASK Bit in Driver State Variable.
      Set_DriverStates( CAL_TASK, DRIVER_ON );
    }
  } // EndElse (RoadBrd_WWDG_VerifyFrame())
  
  
// Initialize Key Vars once Flash has been validated.
  // Initialize Key Timer Sampling Vars.
  Set_TickCounts( RoadBrd_Get_RdSndTickCnt(), RoadBrd_Get_SnsrTickCnt() );
    
  // Time to start WWDG..
  HAL_NVIC_EnableIRQ(WWDG_IRQn);
  MX_WWDG_Init();
  RoadBrd_WWDG_Start();

#endif

#ifdef TASKING
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#else
  
  #ifdef TEST
      // Turn On Bluetooth Interface for Debug.
      RoadBrd_gpio_On( gRESET_BGM111 );
  #else
//    #ifndef TEST2
      #ifdef ASCII
        // Send Opening Banner for Monitor Code.
        #ifdef NUCLEO
//        strcpy( (char *)tempBffr2, "*********************  WEATHERCLOUD *********************\r\n\r\n");
          strcpy( (char *)tempBffr2, "\r\n\r\n");
          // Send string to UART..
#ifdef REV_L
          Status = RoadBrd_UART_Transmit_IT(NUCLEO_USART, (uint8_t *)tempBffr2);
          // Wait for msg to be completed.
          while (RoadBrd_Uart_Status(NUCLEO_USART) != SET)
          {
            RoadBrd_WWDG_Refresh();     // Refresh WatchDog
          }
          // Clear State for Next Transfer.
          clrUsartState( NUCLEO_USART );
#else
          Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);
          RoadBrd_WWDG_Refresh();     // Refresh WatchDog
#endif
          if (Status != HAL_OK)
            Error_Handler();
          sprintf( (char *)tempBffr2, "     Road Board Monitor %s Hardware Version %s \r\n", VERSION_NUM, BRD_REV);
          //strcpy( (char *)tempBffr2, "     Road Board Monitor Rev K Hardware Version 1.0 \r\n");
          // Send string to UART..
#ifdef REV_L
          Status = RoadBrd_UART_Transmit_IT(NUCLEO_USART, (uint8_t *)tempBffr2);
          // Wait for msg to be completed.
          while (RoadBrd_Uart_Status(NUCLEO_USART) != SET)
          {
          }
          // Clear State for Next Transfer.
          clrUsartState( NUCLEO_USART );
#else
          Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);
#endif
          if (Status != HAL_OK)
            Error_Handler();
          sprintf( (char *)tempBffr2, "                  Copyright %s. \r\n\r\n", REL_DATE);
          //strcpy( (char *)tempBffr2, "                  Copyright August 9, 2016. \r\n\r\n\r\n> ");
          // Send string to UART..
          // *******Time to Build Extra Information...
#ifdef REV_L
          Status = RoadBrd_UART_Transmit_IT(NUCLEO_USART, (uint8_t *)tempBffr2);
          // Wait for msg to be completed.
          while (RoadBrd_Uart_Status(NUCLEO_USART) != SET)
          {
          }
          // Clear State for Next Transfer.
          clrUsartState( NUCLEO_USART );
#else
          Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);
#endif
          if (Status != HAL_OK)
            Error_Handler();
          sprintf( (char *)tempBffr2, "RdSnd Sample Rate:  %3.1f Seconds.\r\n", ((float)RoadBrd_Get_RdSndTickCnt()/10));
#ifdef REV_L
          Status = RoadBrd_UART_Transmit_IT(NUCLEO_USART, (uint8_t *)tempBffr2);
          // Wait for msg to be completed.
          while (RoadBrd_Uart_Status(NUCLEO_USART) != SET)
          {
          }
          // Clear State for Next Transfer.
          clrUsartState( NUCLEO_USART );
#else
          Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);
#endif
          if (Status != HAL_OK)
            Error_Handler();
          sprintf( (char *)tempBffr2, "Sensor Sample Rate: %3.1f Seconds.\r\n\r\n> ", ((float)RoadBrd_Get_SnsrTickCnt()/10));
#ifdef REV_L
          Status = RoadBrd_UART_Transmit_IT(NUCLEO_USART, (uint8_t *)tempBffr2);
          // Wait for msg to be completed.
          while (RoadBrd_Uart_Status(NUCLEO_USART) != SET)
          {
          }
          // Clear State for Next Transfer.
          clrUsartState( NUCLEO_USART );
#else
          Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);
#endif
          if (Status != HAL_OK)
            Error_Handler();
        #else
          strcpy( (char *)tempBffr2, "*********************  WEATHERCLOUD *********************\r\n\r\n");
          // Send string to UART..
#ifdef REV_L
          Status = RoadBrd_UART_Transmit_IT(MONITOR_UART, (uint8_t *)tempBffr2);
          // Wait for msg to be completed.
          while (RoadBrd_Uart_Status(MONITOR_UART) != SET)
          {
            RoadBrd_WWDG_Refresh();     // Refresh WatchDog
          }
          // Clear State for Next Transfer.
          clrUsartState( MONITOR_UART );
#else
          Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);
          RoadBrd_WWDG_Refresh();     // Refresh WatchDog
#endif
          if (Status != HAL_OK)
            Error_Handler();
          sprintf( (char *)tempBffr2, "     Road Board Monitor %s Hardware Version %s \r\n", VERSION_NUM, BRD_REV);
          //strcpy( (char *)tempBffr2, "     Road Board Monitor Rev K Hardware Version 1.0 \r\n");
          // Send string to UART..
#ifdef REV_L
          Status = RoadBrd_UART_Transmit_IT(MONITOR_UART, (uint8_t *)tempBffr2);
          // Wait for msg to be completed.
          while (RoadBrd_Uart_Status(MONITOR_UART) != SET)
          {
            RoadBrd_WWDG_Refresh();     // Refresh WatchDog
          }
          // Clear State for Next Transfer.
          clrUsartState( MONITOR_UART );
#else
          Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);
          RoadBrd_WWDG_Refresh();     // Refresh WatchDog
#endif
          if (Status != HAL_OK)
            Error_Handler();
          sprintf( (char *)tempBffr2, "                  Copyright %s. \r\n\r\n", REL_DATE);
          //strcpy( (char *)tempBffr2, "                  Copyright August 9, 2016. \r\n\r\n\r\n> ");
          // Send string to UART..
#ifdef REV_L
          Status = RoadBrd_UART_Transmit_IT(MONITOR_UART, (uint8_t *)tempBffr2);
          // Wait for msg to be completed.
          while (RoadBrd_Uart_Status(MONITOR_UART) != SET)
          {
            RoadBrd_WWDG_Refresh();     // Refresh WatchDog
          }
          // Clear State for Next Transfer.
          clrUsartState( MONITOR_UART );
#else
          Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);
          RoadBrd_WWDG_Refresh();     // Refresh WatchDog
#endif
          if (Status != HAL_OK)
            Error_Handler();
          // *******Time to Build Extra Information...
          sprintf( (char *)tempBffr2, "RdSnd Sample Rate:  %3.1f Seconds.\r\n", ((float)RoadBrd_Get_RdSndTickCnt()/10));
#ifdef REV_L
          Status = RoadBrd_UART_Transmit_IT(NUCLEO_USART, (uint8_t *)tempBffr2);
          // Wait for msg to be completed.
          while (RoadBrd_Uart_Status(NUCLEO_USART) != SET)
          {
          }
          // Clear State for Next Transfer.
          clrUsartState( NUCLEO_USART );
#else
          Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);
#endif
          if (Status != HAL_OK)
            Error_Handler();
          sprintf( (char *)tempBffr2, "Sensor Sample Rate: %3.1f Seconds.\r\n", ((float)RoadBrd_Get_SnsrTickCnt()/10));
#ifdef REV_L
          Status = RoadBrd_UART_Transmit_IT(NUCLEO_USART, (uint8_t *)tempBffr2);
          // Wait for msg to be completed.
          while (RoadBrd_Uart_Status(NUCLEO_USART) != SET)
          {
          }
          // Clear State for Next Transfer.
          clrUsartState( NUCLEO_USART );
#else
          Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);
#endif
          if (Status != HAL_OK)
            Error_Handler();
          // Now Display the Units Enabled State.
          if (RoadBrd_Get_UnitsFlag())
          {
            sprintf( (char *)tempBffr2, "Units XML State: ENABLED\r\n\r\n> ");
          }
          else
          {
            sprintf( (char *)tempBffr2, "Units XML State: DISABLED\r\n\r\n> ");
          }
#ifdef REV_L
          Status = RoadBrd_UART_Transmit_IT(NUCLEO_USART, (uint8_t *)tempBffr2);
          // Wait for msg to be completed.
          while (RoadBrd_Uart_Status(NUCLEO_USART) != SET)
          {
          }
          // Clear State for Next Transfer.
          clrUsartState( NUCLEO_USART );
#else
          Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);
#endif
          if (Status != HAL_OK)
            Error_Handler();
       #endif
//      #endif
    #endif
  #endif
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /****************************************************************************
    * TEST CODE....Blinks LEDs for both Road Board and Nucleo Board.
    **************************************************************************/
  #ifdef TEST
    #ifdef NUCLEO
      // Turn On NUCLEO_LED_GREEN LED.
      RoadBrd_gpio_On( NUCLEO_LED_GREEN );
      // Wait 500msec.
      RoadBrd_Delay( 500 );
      // Turn Off NUCLEO_LED_GREEN LED.
      RoadBrd_gpio_Off( NUCLEO_LED_GREEN );
    #else
      // Turn On Blue LED and turn off Yellow LED.
      RoadBrd_gpio_On( BLUE_LED );
      RoadBrd_gpio_Off( YELLOW_LED );
      // Wait 500msec.
      RoadBrd_Delay( 500 );
      // Turn On Green LED and turn off Blue LED.
      RoadBrd_gpio_On( GREEN_LED );
      RoadBrd_gpio_Off( BLUE_LED );
      // Wait 500msec.
      RoadBrd_Delay( 500 );
      // Turn On Yellow LED and turn off Green LED.
      RoadBrd_gpio_On( YELLOW_LED );
      RoadBrd_gpio_Off( GREEN_LED );
    #endif
    // Wait 500msec.
    RoadBrd_Delay( 500 );
  //******************Endif TEST
  #else
  //******************Else TEST

  /****************************************************************************
   * TEST2 CODE....Reads COM port and translates back to sender. Simple COMM test.
   **************************************************************************/
    #ifdef TEST2
      #ifdef NUCLEO
        // Turn Off NUCLEO_LED_GREEN LED.
        RoadBrd_gpio_Off( NUCLEO_LED_GREEN );
        // Wait on a character from Nucleo COM Port.
#ifdef REV_L
        Status = RoadBrd_UART_Receive_IT(NUCLEO_USART, tempBffr, 1);
        // Wait for msg to be completed.
        while (RoadBrd_Uart_Status(NUCLEO_USART) != SET)
        {
        }
        // Clear State for Next Transfer.
        clrUsartState( NUCLEO_USART );
#else
        Status = RoadBrd_UART_Receive(NUCLEO_USART, tempBffr, 1);
#endif
        // Turn On NUCLEO_LED_GREEN LED.
        RoadBrd_gpio_On( NUCLEO_LED_GREEN );
        switch(  Status)
        {
          case HAL_OK:
            // We have a good character. Time to build a response string.
            tempBffr2[0] = '(';
            tempBffr2[1] = tempBffr[0];
            tempBffr2[2] = ')';
            tempBffr2[3] = 0x00;
            // Send string to UART..
#ifdef REV_L
            Status = RoadBrd_UART_Transmit_IT(NUCLEO_USART, (uint8_t *)tempBffr2);
            // Wait for msg to be completed.
            while (RoadBrd_Uart_Status(NUCLEO_USART) != SET)
            {
            }
            // Clear State for Next Transfer.
            clrUsartState( NUCLEO_USART );
#else
            Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);
#endif
            if (Status != HAL_OK)
              Error_Handler();
            /*Status = RoadBrd_UART_Transmit_IT(NUCLEO_USART, (uint8_t *)tempBffr2);
            if (Status != HAL_OK)
              Error_Handler();
            // Now wait for completion of Transmit
            PStatus = RESET;
            while (PStatus == RESET)
            {
              PStatus = RoadBrd_Uart_Status(NUCLEO_USART);
            } */
            break;
          case HAL_ERROR:
            // ERROR. We are done.
            Error_Handler();
            break;
          case HAL_BUSY:
            // ERROR. We are done.
            Error_Handler();
            break;
          case HAL_TIMEOUT:
            // Nothing to do. Try again.
            break;
          default:
            // ERROR. We are done.
            Error_Handler();
            break;
          
        }
      // Wait 500msec.
      RoadBrd_Delay( 50 );
      #else
        // Turn Off BGM_LED LED.
        RoadBrd_gpio_Off( BGM_LED );
        // Wait on a character from Nucleo COM Port.
 #ifdef REV_L
        Status = RoadBrd_UART_Receive_IT(MONITOR_UART, tempBffr, 1);
        // Wait for msg to be completed.
        while (RoadBrd_Uart_Status(MONITOR_UART) != SET)
        {
        }
        // Clear State for Next Transfer.
        clrUsartState( MONITOR_UART );
#else
        Status = RoadBrd_UART_Receive(MONITOR_UART, tempBffr, 1);
#endif
        // Turn On BGM_LED LED.
        RoadBrd_gpio_On( BGM_LED );
        switch(  Status)
        {
          case HAL_OK:
            // We have a good character. Time to build a response string.
            tempBffr2[0] = '(';
            tempBffr2[1] = tempBffr[0];
            tempBffr2[2] = ')';
            tempBffr2[3] = 0x00;
            // Send string to UART..
#ifdef REV_L
            Status = RoadBrd_UART_Transmit_IT(MONITOR_UART, (uint8_t *)tempBffr2);
            // Wait for msg to be completed.
            while (RoadBrd_Uart_Status(MONITOR_UART) != SET)
            {
            }
            // Clear State for Next Transfer.
            clrUsartState( MONITOR_UART );
#else
            Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);
#endif
            if (Status != HAL_OK)
              Error_Handler();
            /*Status = RoadBrd_UART_Transmit_IT(NUCLEO_USART, (uint8_t *)tempBffr2);
            if (Status != HAL_OK)
              Error_Handler();
            // Now wait for completion of Transmit
            PStatus = RESET;
            while (PStatus == RESET)
            {
              PStatus = RoadBrd_Uart_Status(NUCLEO_USART);
            } */
            break;
          case HAL_ERROR:
            // ERROR. We are done.
            Error_Handler();
            break;
          case HAL_BUSY:
            // ERROR. We are done.
            Error_Handler();
            break;
          case HAL_TIMEOUT:
            // Nothing to do. Try again.
            break;
          default:
            // ERROR. We are done.
            Error_Handler();
            break;
          
        }
      // Wait 500msec.
      RoadBrd_Delay( 50 );
      #endif
    //******************Endif TEST2
    #else
    //******************Else TEST2
  /*****************************************************************************
   *    NORMAL FLOW HERE
   ****************************************************************************/
//         RoadBrd_gpio_Off( NUCLEO_LED_GREEN );
//         Status = RoadBrd_UART_Receive(NUCLEO_USART, tempBffr, 1);
      #ifdef NUCLEO
      
      //************************* NUCLEO VERSION *************************************   
         // Turn Off NUCLEO_LED_GREEN LED.
        RoadBrd_gpio_Off( NUCLEO_LED_GREEN );
        // Wait on a character from Nucleo COM Port.
#ifdef REV_L
        tmpSize = Size;
        pData = tempBffr;
        while ( tmpSize>0 )
        {
          Status = RoadBrd_UART_Receive_IT(NUCLEO_USART, tmpData, 1);
          // Wait for msg to be completed.
          while (RoadBrd_Uart_Status(NUCLEO_USART) != SET)
          {
          }
          // Clear State for Next Transfer.
          clrUsartState( NUCLEO_USART );
          if(Status == HAL_OK)
          {
            // Watch for termination characters.
            if((tmpData[0]==0x0a) || (tmpData[0]==0x0d))
            {
              *pData = 0x00;
              // Yes..We are done.
              return Status;
            }
            else
            {
              // Move new character into passed buffer.
              *pData = tmpData[0];
              tmpSize--;                          // Decrement Count
              pData++;                            // Move pointer to next buffer location.
            }
          }
          else
            Error_Handler();
        } // EndWhile ( tmpSize>0 )
#else
        Status = RoadBrd_UART_Receive(NUCLEO_USART, tempBffr, RECEIVE_SZ);
#endif
        // Turn On NUCLEO_LED_GREEN LED.
#ifndef LED_OFF
        RoadBrd_gpio_On( NUCLEO_LED_GREEN );
#endif
#ifdef ASCII
        // Send <CR><LF> to UART..
        strcpy( (char *)tempBffr2, "\r\n");
#ifdef REV_L
            Status = RoadBrd_UART_Transmit_IT(NUCLEO_USART, (uint8_t *)tempBffr2);
            // Wait for msg to be completed.
            while (RoadBrd_Uart_Status(NUCLEO_USART) != SET)
            {
            }
            // Clear State for Next Transfer.
            clrUsartState( NUCLEO_USART );
#else
        Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);
#endif
        if (Status != HAL_OK)
          Error_Handler();
#endif
        switch(  Status )
        {
          case HAL_OK:
            Status = RoadBrd_ParseString((char *)tempBffr);
            // We have a good Tasking String. Time to determine action.
            if (Status != HAL_OK)
              Error_Handler();
            
            break;
          case HAL_ERROR:
            // ERROR. We are done.
            Error_Handler();
            break;
          case HAL_BUSY:
            // ERROR. We are done.
            Error_Handler();
            break;
          case HAL_TIMEOUT:
            // Nothing to do. Try again.
            break;
          default:
            // ERROR. We are done.
            Error_Handler();
            break;
        }
      // Wait 500msec.
      RoadBrd_Delay( 50 );
      //************************* END NUCLEO VERSION *************************************   
      
      #else

      //************************* ROAD BRD VERSION *************************************   
        // Wait on a character from Nucleo COM Port.
//********************PATCH_UART ACTIVE*******************************************
#ifdef PATCH_UART
#ifdef REV_L
        // Turn Off MICRO_LED LED.
        RoadBrd_gpio_Off( MICRO_LED );
        tmpSize = RECEIVE_SZ;
        pData = tempBffr;
        while ( tmpSize>0 )
        {
          Status = RoadBrd_UART_Receive_IT(MONITOR_UART, tmpData, 1);
          // Wait for msg to be completed.
          while (RoadBrd_Uart_Status(MONITOR_UART) != SET)
          {
           }
          // Clear State for Next Transfer.
          clrUsartState( MONITOR_UART );
          if(Status == HAL_OK)
          {
            // Watch for termination characters.
            if((tmpData[0]==0x0a) || (tmpData[0]==0x0d))
            {
              *pData = 0x00;
              // Yes..We are done.
              break;
            }
            else
            {
              // Move new character into passed buffer.
              *pData = tmpData[0];
              tmpSize--;                          // Decrement Count
              pData++;                            // Move pointer to next buffer location.
            }
          }
          else
            Error_Handler();
        } // EndWhile ( tmpSize>0 )
#ifndef LED_OFF
        RoadBrd_gpio_On( MICRO_LED );
#endif
        // Send <CR><LF> to UART..
        strcpy( (char *)tempBffr2, "\r\n");
        Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);
        if (Status != HAL_OK)
          Error_Handler();
        switch(  Status )
        {
          case HAL_OK:
            // We have a good Tasking String. Time to determine action.
            Status = RoadBrd_ParseString((char *)tempBffr);
            // We have a good Tasking String. Time to determine action.
            if (Status != HAL_OK)
              Error_Handler();
            break;
          case HAL_ERROR:
            // ERROR. We are done.
            Error_Handler();
            break;
          case HAL_BUSY:
            // ERROR. We are done.
            Error_Handler();
            break;
          case HAL_TIMEOUT:
            // Nothing to do. Try again.
            break;
          default:
            // ERROR. We are done.
            Error_Handler();
            break;
          
        } // EndSwitch (  Status )
#else
        // Turn Off BGM_LED LED.
        RoadBrd_gpio_Off( MICRO_LED );
        Status = RoadBrd_UART_Receive(MONITOR_UART, tempBffr, RECEIVE_SZ);
        // Process Buffer Now.
        // Turn On BGM_LED LED.
#ifndef LED_OFF
        RoadBrd_gpio_On( MICRO_LED );
#endif
        // Send <CR><LF> to UART..
        strcpy( (char *)tempBffr2, "\r\n");
#ifdef REV_L
            Status = RoadBrd_UART_Transmit_IT(MONITOR_UART, (uint8_t *)tempBffr2);
            // Wait for msg to be completed.
            while (RoadBrd_Uart_Status(MONITOR_UART) != SET)
            {
            }
            // Clear State for Next Transfer.
            clrUsartState( MONITOR_UART );
#else
        Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);
#endif
        if (Status != HAL_OK)
          Error_Handler();
        switch(  Status )
        {
          case HAL_OK:
            // We have a good Tasking String. Time to determine action.
            Status = RoadBrd_ParseString((char *)tempBffr);
            // We have a good Tasking String. Time to determine action.
            if (Status != HAL_OK)
              Error_Handler();
            break;
          case HAL_ERROR:
            // ERROR. We are done.
            Error_Handler();
            break;
          case HAL_BUSY:
            // ERROR. We are done.
            Error_Handler();
            break;
          case HAL_TIMEOUT:
            // Nothing to do. Try again.
            break;
          default:
            // ERROR. We are done.
            Error_Handler();
            break;
          
        } // EndSwitch (  Status )
#endif
//********************END PATCH_UART ACTIVE*******************************************
#else
//********************PATCH_UART NOT ACTIVE*******************************************
#ifdef REV_L
          /* Process BLE input */
          BGM111_ProcessInput();
        
          // Process Timer Stimulus for Timer 2.
          Status = Proc_Timer2();
          if(Status != HAL_OK)
          {
            strcpy( (char *)tempBffr2, "BGM111_ProcessInput ERROR: Sensors reported Background error.\r\n");
#ifdef NUCLEO
            Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);
#else
            Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
            if (Status != HAL_OK)
              Error_Handler();
          }
          /* Process the sensor state machine if the BLE module is ready */
            if ((BGM111_Ready()) &&
                (BGM111_Connected()) &&
                (BGM111_DataConnected()) &&
                (BGM111_SyncModeTest()) )
          {
            ProcessSensorState();
          }
          /* Sleep when we have nothing to process */
          //PWR_EnterSleepMode(PWR_Regulator_ON, PWR_SLEEPEntry_WFI);
          // Only Process the first Time
          if (firstTime)
          {
            firstTime = false;
            Status = RoadBrd_UART_Receive_IT(MONITOR_UART, tmpData, 1);
            // Enable BGM Serial Traffic.
            HAL_UART_EnableBGM_RX();
            RoadBrd_SetBffrFlg();
            // Clear bffrFlag..Only Processing one character.
//            bffrFlag = false;
            // Turn Off BGM_LED and MICRO_LED.
            RoadBrd_gpio_Off( MICRO_LED );
          }
          // Service Watchdog
          RoadBrd_WWDG_Refresh();     // Refresh WatchDog
          // Wait for msg to be completed.
          while (RoadBrd_Uart_Status(MONITOR_UART) != SET)
          {
            // Process Timer Stimulus for Timer 2.
            Status = Proc_Timer2();
            if(Status != HAL_OK)
            {
              strcpy( (char *)tempBffr2, "BGM111_ProcessInput ERROR: Sensors reported Background error.\r\n");
#ifdef NUCLEO
              Status = RoadBrd_UART_Transmit(NUCLEO_USART, (uint8_t *)tempBffr2);
#else
              Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);                   
#endif
              if (Status != HAL_OK)
                Error_Handler();
            }
            /* Process the sensor state machine if the BLE module is ready */
            if ((BGM111_Ready()) &&
                (BGM111_Connected()) &&
                (BGM111_DataConnected()) &&
                (BGM111_SyncModeTest()) )
            {
              // Service Watchdog
              RoadBrd_WWDG_Refresh();     // Refresh WatchDog
              ProcessSensorState();
            }
            // Test to see if we have any BGM Traffic to process.
            //BGM111_ProcessInput();
          }
          // Test if BGM or Monitor Character received.
          if (RoadBrd_Uart_Status(MONITOR_UART) == SET)
          {
            // Clear State for Next Transfer.
            clrUsartState( MONITOR_UART );
            if(Status == HAL_OK)
            {
              // Test Bypass Flag...If Set, we ae in special monitor mode.
              if (Tst_Bypass())
              {
                pData[0] = tmpData[0];
                pData[1] = 0x00;
                // Clear State for Next Transfer.
                clrUsartState( MONITOR_UART );
                if (Status != HAL_OK)
                  Error_Handler();
                if(Status == HAL_OK)
                {
                  // We have a good Tasking String. Time to determine action.
                  // Turn On BGM_LED LED.
  #ifndef LED_OFF
                  RoadBrd_gpio_On( GREEN_LED );
  #endif
                  Status = RoadBrd_ParseString((char *)tempBffr);
                  // We have a good Tasking String. Time to determine action.
                  if (Status != HAL_OK)
                    Error_Handler();
                }
                else
                  Error_Handler();
                tmpSize = RECEIVE_SZ;
                pData = tempBffr;
              } // EndIf (Tst_Bypass())
              else {
                // Watch for termination characters.
                if((tmpData[0]==0x0a) || (tmpData[0]==0x0d) || (tmpSize<=0) )
                {
                  *pData = 0x00;
                  // Yes..We are done.
                  // Process Buffer NOW.
                  // Send <CR><LF> to UART..
                  strcpy( (char *)tempBffr2, "\r\n");
                  Status = RoadBrd_UART_Transmit_IT(MONITOR_UART, (uint8_t *)tempBffr2);
                  // Wait for msg to be completed.
                  while (RoadBrd_Uart_Status(MONITOR_UART) != SET)
                  {
                  }
                  // Clear State for Next Transfer.
                  clrUsartState( MONITOR_UART );
                  if (Status != HAL_OK)
                    Error_Handler();
                  if(Status == HAL_OK)
                  {
                    // We have a good Tasking String. Time to determine action.
                    // Turn On BGM_LED LED.
  #ifndef LED_OFF
                    RoadBrd_gpio_On( GREEN_LED );
  #endif
                    Status = RoadBrd_ParseString((char *)tempBffr);
                    // We have a good Tasking String. Time to determine action.
                    if (Status != HAL_OK)
                      Error_Handler();
                  }
                  else
                    Error_Handler();
                  tmpSize = RECEIVE_SZ;
                  pData = tempBffr;
                } // EndIf ((tmpData[0]==0x0a) || (tmpData[0]==0x0d) || (tmpSize<=0) )
                else
                {
                  *pData = tmpData[0];
                  tmpSize--;                          // Decrement Count
                  pData++;                            // Move pointer to next buffer location.
                }
              } // EndElse (Tst_Bypass())
            }
            else
              Error_Handler();
          Status = RoadBrd_UART_Receive_IT(MONITOR_UART, tmpData, 1);
          // Turn Off MICRO_LED.
          RoadBrd_gpio_Off( MICRO_LED );
        } // EndIf (RoadBrd_Uart_Status(MONITOR_UART) == SET)

#else
        // Turn Off BGM_LED LED.
        RoadBrd_gpio_Off( MICRO_LED );
        Status = RoadBrd_UART_Receive(MONITOR_UART, tempBffr, RECEIVE_SZ);
        // Turn On BGM_LED LED.
#ifndef LED_OFF
        RoadBrd_gpio_On( MICRO_LED );
#endif
        // Send <CR><LF> to UART..
        strcpy( (char *)tempBffr2, "\r\n");
        Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);
        if (Status != HAL_OK)
          Error_Handler();
        switch(  Status )
        {
          case HAL_OK:
            // We have a good Tasking String. Time to determine action.
            Status = RoadBrd_ParseString((char *)tempBffr);
            // We have a good Tasking String. Time to determine action.
            if (Status != HAL_OK)
              Error_Handler();
            break;
          case HAL_ERROR:
            // ERROR. We are done.
            Error_Handler();
            break;
          case HAL_BUSY:
            // ERROR. We are done.
            Error_Handler();
            break;
          case HAL_TIMEOUT:
            // Nothing to do. Try again.
            break;
          default:
            // ERROR. We are done.
            Error_Handler();
            break;
          
        } // EndSwitch (  Status )
#endif
//********************END PATCH_UART NOT ACTIVE*******************************************
#endif
      // Wait 50msec.
      RoadBrd_Delay( 50 );
      //************************* END Road BRD VERSION *************************************   
      #endif
  /*****************************************************************************
   *    END NORMAL FLOW HERE
   ****************************************************************************/
    //******************EndElse TEST2
    #endif

  //******************EndElse TEST
  #endif
  } // EndWhile (1)
//********************ENDIF TASKING
#endif
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  // Turn LED2 on
#ifdef NUCLEO
  BSP_LED_On(LED2);
#else
  RoadBrd_LED_On(BLUE_LED);
  RoadBrd_LED_On(GREEN_LED);
  RoadBrd_LED_On(YELLOW_LED);
#endif
  while(1) 
  {
    // Error if LED2 is slowly blinking (1 sec. period)
#ifdef NUCLEO
    BSP_LED_Toggle(LED2); 
#else
    RoadBrd_LED_Toggle(BLUE_LED); 
    RoadBrd_LED_Toggle(GREEN_LED); 
    RoadBrd_LED_Toggle(YELLOW_LED); 
#endif
    HAL_Delay(50); 
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
