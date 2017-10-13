/***********************************************************************************************//**
 * \file   main.c
 * \brief  Silicon Labs Thermometer Example Application
 *
 * This Thermometer and OTA example allows the user to measure temperature
 * using the temperature sensor on the WSTK. The values can be read with the
 * Health Thermometer reader on the Blue Gecko smartphone app.
 ***************************************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silicon Labs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

/* Board Headers */
#include "boards.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "aat.h"
#include "infrastructure.h"

/* GATT database */
#include "gatt_db.h"

/* EM library (EMlib) */
#include "em_system.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#ifdef FEATURE_BOARD_DETECTED
#include "bspconfig.h"
#include "pti.h"
#else
#error This sample app only works with a Silicon Labs Board
#endif

#ifdef FEATURE_IOEXPANDER
#include "bsp.h"
#include "bsp_stk_ioexp.h"
#endif /* FEATURE_IOEXPANDER */

/* Device initialization header */
#include "InitDevice.h"

/* Temperature sensor and I2c*/
#include "i2cspmconfig.h"
#include "i2cspm.h"
#include "si7013.h"
#include "tempsens.h"
#include "main.h"

#ifdef FEATURE_SPI_FLASH
#include "em_usart.h"
#include "mx25flash_spi.h"
#endif /* FEATURE_SPI_FLASH */

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

/* Gecko configuration parameters (see gecko_configuration.h) */
#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

#ifdef FEATURE_PTI_SUPPORT
static const RADIO_PTIInit_t ptiInit = RADIO_PTI_INIT;
#endif

static const gecko_configuration_t config = {
  .config_flags = 0,
//  .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
  .sleep.flags = 0,
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap),
  .bluetooth.sleep_clock_accuracy = 100, // ppm
  .gattdb = &bg_gattdb_data,
  .ota.flags = 0,
  .ota.device_name_len = 3,
  .ota.device_name_ptr = "OTA",
  #ifdef FEATURE_PTI_SUPPORT
  .pti = &ptiInit,
  #endif
};

void UART_Init(int enable_vcom)
{
	/* Enable VCOM */
	GPIO_PinModeSet(BSP_BCC_ENABLE_PORT, BSP_BCC_ENABLE_PIN, gpioModePushPull, enable_vcom);

}
/* Flag for indicating DFU Reset must be performed */
uint8_t boot_to_dfu = 0;

/**
 * @brief  Main function
 */
int main(void)
{
	int ticker;
	char string[STR_LEN];
	int strlen;

	int connected;
	int conn_handle;

	int res;

	int n;

	int temp;

	char tx_fifo[BUFFSIZE];
	int tx_wr;						// Write Index
	int	tx_rd;						// Read Index
	int	tx_num;						// Number of bytes in Buffer

	int max_len;
	int tx_size;

	int unrecoverable_error;

	/* Initialize peripherals */
	enter_DefaultMode_from_RESET();

	/* Initialize stack */
	gecko_init(&config);


  while (1) {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;

    /* Check for stack event. */
    evt = gecko_wait_event();

    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header)) {
      /* This boot event is generated when the system boots up after reset.
       * Here the system is set to start advertising immediately after boot procedure. */
      case gecko_evt_system_boot_id:
    	gecko_cmd_endpoint_send(UART_EP, 12, (uint8 *)"SPP server\r\n");
        /* Set advertising parameters. 100ms advertisement interval. All channels used.
         * The first two parameters are minimum and maximum advertising interval, both in
         * units of (milliseconds * 1.6). The third parameter '7' sets advertising on all channels. */
        gecko_cmd_le_gap_set_adv_parameters(160, 160, 7);

        /* Start general advertising and enable connections. */
        gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);
        break;

      /* This event is generated when a connected client has either
       * 1) changed a Characteristic Client Configuration, meaning that they have enabled
       * or disabled Notifications or Indications, or
       * 2) sent a confirmation upon a successful reception of the indication. */
//      case gecko_evt_gatt_server_characteristic_status_id:
        /* Check that the characteristic in question is temperature - its ID is defined
         * in gatt.xml as "temp_measurement". Also check that status_flags = 1, meaning that
         * the characteristic client configuration was changed (notifications or indications
         * enabled or disabled). */
//        if ((evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_temp_measurement)
//            && (evt->data.evt_gatt_server_characteristic_status.status_flags == 0x01)) {
//          if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == 0x02) {
            /* Indications have been turned ON - start the repeating timer. The 1st parameter '32768'
             * tells the timer to run for 1 second (32.768 kHz oscillator), the 2nd parameter is
             * the timer handle and the 3rd parameter '0' tells the timer to repeat continuously until
             * stopped manually.*/
//            gecko_cmd_hardware_set_soft_timer(32768, 0, 0);
//          } else if (evt->data.evt_gatt_server_characteristic_status.client_config_flags == 0x00) {
//            /* Indications have been turned OFF - stop the timer. */
//            gecko_cmd_hardware_set_soft_timer(0, 0, 0);
//          }
//        }
//        break;

      /* This event is generated when the software timer has ticked. In this example the temperature
       * is read after every 1 second and then the indication of that is sent to the listening client. */
//      case gecko_evt_hardware_soft_timer_id:
//        /* Measure the temperature as defined in the function temperatureMeasure() */
//        temperatureMeasure();
//        break;

      case gecko_evt_le_connection_closed_id:
        /* Check if need to boot to dfu mode */
        if (boot_to_dfu) {
          /* Enter to DFU OTA mode */
          gecko_cmd_system_reset(2);
        } else {
          /* Stop timer in case client disconnected before indications were turned off */
          gecko_cmd_hardware_set_soft_timer(0, 0, 0);
          /* Restart advertising after client has disconnected */
          gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);
        }
        break;

      /* Events related to OTA upgrading
         ----------------------------------------------------------------------------- */

      /* Checks if the user-type OTA Control Characteristic was written.
       * If written, boots the device into Device Firmware Upgrade (DFU) mode. */
      case gecko_evt_gatt_server_user_write_request_id:
        if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
          /* Set flag to enter to OTA mode */
          boot_to_dfu = 1;
          /* Send response to Write Request */
          gecko_cmd_gatt_server_send_user_write_response(
            evt->data.evt_gatt_server_user_write_request.connection,
            gattdb_ota_control,
            bg_err_success);

          /* Close connection to enter to DFU OTA mode */
          gecko_cmd_endpoint_close(evt->data.evt_gatt_server_user_write_request.connection);
        }
        break;

      default:
        break;
    }
  }
}

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
