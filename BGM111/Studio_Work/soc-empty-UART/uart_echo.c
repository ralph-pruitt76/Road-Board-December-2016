/*
 * uart_echo.c
 *
 *  Created on: 2016.04.20.
 *      Author: baadamff
 */

/* Board support header */
#include "bsp.h"

/* emlib & emdrv */
#include "em_int.h"
#include "em_emu.h"
#include "em_gpio.h"

#include "uartdrv.h"
#include "uart_echo.h"



#define NUM_TX_BUFFERS          8

/***************************************************************************************************
 Local Variables
 **************************************************************************************************/
static UARTDRV_HandleData_t uartHandle0; /* UART driver handle */
static UARTDRV_Handle_t  testHandle0 = &uartHandle0;
static uint8_t rxByte;

/***************************************************************************************************
 Static Function Declarations
 **************************************************************************************************/
static void UART_tx_callback(UARTDRV_Handle_t handle, Ecode_t transferStatus, uint8_t *data,
                             UARTDRV_Count_t transferCount);
static void UART_rx_callback(UARTDRV_Handle_t handle, Ecode_t transferStatus, uint8_t *data,
                             UARTDRV_Count_t transferCount);

/***************************************************************************************************
 Static Function Definitions
 **************************************************************************************************/
static void UART_tx_callback(UARTDRV_Handle_t handle, Ecode_t transferStatus, uint8_t *data,
                             UARTDRV_Count_t transferCount)
{
  static uint8_t txCnt = 0;

  if (transferStatus == ECODE_EMDRV_UARTDRV_OK)
  {
	  txCnt++;
  }
}

/***********************************************************************************************//**
 *  \brief      This function called when the UART RX operation completed
 *  \param[in]  -
 *  \return     -
 **************************************************************************************************/
static void UART_rx_callback(UARTDRV_Handle_t handle, Ecode_t transferStatus, uint8_t *data,
                             UARTDRV_Count_t transferCount)
{
  static  uint8_t rxCnt = 0;
  static  uint8_t txByte = 0;

  if(transferStatus == ECODE_EMDRV_UARTDRV_OK)
  {
	  txByte = *data;
	  //UARTDRV_Transmit(testHandle0, "ACK\r\n", 5 , UART_tx_callback);
	  UARTDRV_Transmit(testHandle0, &txByte, 1 , UART_tx_callback);

	  rxCnt++;
  }

  /* RX the next byte */
  UARTDRV_Receive(testHandle0, &rxByte, 1, UART_rx_callback);
}

/***************************************************************************************************
 Public Function Definitions
 **************************************************************************************************/
void UART_Init(void)
{
	/* Enable VCOM */
	GPIO_PinModeSet(BSP_BCC_ENABLE_PORT, BSP_BCC_ENABLE_PIN, gpioModePushPull, 1);
	GPIO_PinOutSet(BSP_BCC_ENABLE_PORT, BSP_BCC_ENABLE_PIN);

	/* uart init */
	UARTDRV_Init_t initData;

	DEFINE_BUF_QUEUE(EMDRV_UARTDRV_MAX_CONCURRENT_RX_BUFS, rxBufferQueueI0);
	DEFINE_BUF_QUEUE(EMDRV_UARTDRV_MAX_CONCURRENT_TX_BUFS, txBufferQueueI0);


	/* UART init */
	initData.port                 = USART0;
	initData.baudRate             = 115200;
	initData.portLocationTx       = USART_ROUTELOC0_RXLOC_LOC0;
	initData.portLocationRx       = USART_ROUTELOC0_RXLOC_LOC0;
	initData.stopBits             = (USART_Stopbits_TypeDef) USART_FRAME_STOPBITS_ONE;
	initData.parity               = (USART_Parity_TypeDef) USART_FRAME_PARITY_NONE;
	initData.oversampling         = (USART_OVS_TypeDef) USART_CTRL_OVS_X16;
	initData.mvdis                = false;
	initData.fcType               = uartdrvFlowControlNone;
	initData.rxQueue              = (UARTDRV_Buffer_FifoQueue_t *)&rxBufferQueueI0;
	initData.txQueue              = (UARTDRV_Buffer_FifoQueue_t *)&txBufferQueueI0;

	UARTDRV_Init(testHandle0, &initData);
	UARTDRV_Transmit(testHandle0, "OK\r\n", 4, UART_tx_callback);
	UARTDRV_Receive(testHandle0, &rxByte, 1, UART_rx_callback);

}


