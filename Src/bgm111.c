/* SiLabs BGM111 module access implementation using BGLib */

#include "bgm111.h"
#include <string.h>
#include "gecko_bglib.h"
#include "stm32l1xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "ErrorCodes.h"
#include "i2c.h"
#include "wwdg.h"
#include "tim.h"
#include "Parser.h"

/* BGLib instantiation */

BGLIB_DEFINE();

/* Define buffer size for BLE communication */

// #define BG_DATA_LENGTH          70   // Code fills this buffer size full on the RX side...11/18/16 RP
#define BG_DATA_LENGTH          200   

/* Define number of free bytes in buffer when we need to process a packet
 * even if that means we need to wait in a loop */

#define BG_DATA_LOW_WATERMARK   5

/* BG reception states */

enum BgRxState
{
  BGRX_SYNC,
  BGRX_HDR,
  BGRX_DATA
};


/* MSG LOGGER */
struct
{
  uint32_t msgBuff[BG_DATA_LENGTH];
  uint16_t tx_wr;
} static blemsgs;

/* BLE communication structure */
struct
{
  uint8_t tx_buf[BG_DATA_LENGTH];
  uint8_t tx_wr;
  volatile uint8_t tx_rd;
  uint8_t rx_buf[BG_DATA_LENGTH];
  volatile uint8_t rx_wr;
  uint8_t rx_rd;
  volatile enum BgRxState rx_state;
  volatile bool req_exec;
  bool booted;
  bool connection;
  bool data_Connection;
  bool CMD_Mode;
  uint8_t TackArmed;
  uint8_t TackCnt;
  uint8_t SyncFlag;
  struct gecko_cmd_packet *evt;
} static ble;


/* BGM111 module low level init */

void BGM111_LowLevel_Init(void)
{
  blemsgs.tx_wr = 0;
  ble.booted =  false;
  ble.connection = false;
  ble.data_Connection = false;
  ble.CMD_Mode = false;
  ble.TackArmed = TACK_OFF;
  ble.TackCnt = 0;
  ble.SyncFlag = SYNC_PROC;
}

/* Next buffer index based on current index and buffer size */

//#pragma inline=forced
uint8_t NextBufIdx(uint8_t idx)
{
  idx++;
  return idx < BG_DATA_LENGTH ? idx : 0;
}

/* Report if the buffer is full based on its indexes */

//#pragma inline=forced
bool IsBufFull(uint8_t wr_idx, uint8_t rd_idx)
{
  return NextBufIdx(wr_idx) == rd_idx;
}

/* Get the used space in the buffer based on its indexes */

//#pragma inline=forced
uint8_t BufUsed(uint8_t wr_idx, uint8_t rd_idx)
{
  int size = (int)wr_idx - (int)rd_idx;
  if (size < 0)
  {
    size = BG_DATA_LENGTH + size;
  }
  return size;
}

/* Get the free space in the buffer based on its indexes */

//#pragma inline=forced
uint8_t BufFree(uint8_t wr_idx, uint8_t rd_idx)
{
  return (BG_DATA_LENGTH - 1) - BufUsed(wr_idx, rd_idx);
}

/* Receive data from the BGM111 module */

int BGM111_Receive(uint32_t len, uint8_t *data)
{
  
  /* Wait until we received the requested number of data bytes */
  while (len)
  {
    /* Is there a byte in the receive buffer? */
    if (ble.rx_rd != ble.rx_wr)
    {
      /* Disable interrupt */
//      NVIC_DisableIRQ(BGM111_UART_IRQn);
      /* Clear the request to execute the BLE stack now we're doing it */
      ble.req_exec = false;
      /* Write to the output buffer */
      *data++ = ble.rx_buf[ble.rx_rd];
      /* Next byte in the receive buffer */
      ble.rx_rd = NextBufIdx(ble.rx_rd);
      /* Enable interrupt */
//      NVIC_EnableIRQ(BGM111_UART_IRQn);

      /* One byte less to wait for */
      len--;
    }
    else
    {
      // Time to prime the pump again....
      HAL_UART_EnableBGM_RX();
      break;
    }
  }
  return 0;
}

/* Check whether there is input data from the BGM111 module */

int BGM111_Peek(void)
{
  return ble.req_exec;
}

/* Initialize the BGM111 module and BGLib */

void BGM111_Init(void)
{
  /* Perform low level init to initialize the UART for use with the 
   * BGM111 module */
  BGM111_LowLevel_Init();
  /* Pull the BGM111 reset pin low */
  RoadBrd_gpio_Off( gRESET_BGM111 );
  /* Initialize BGLib with our transmit, receive and peek routines */
  BGLIB_INITIALIZE_NONBLOCK(BGM111_Transmit, BGM111_Receive, BGM111_Peek);
  /* Release the BGM111 reset pin */
  RoadBrd_gpio_On( gRESET_BGM111 );
}

/* Process any input from the BLE module */

void BGM111_ProcessInput(void)
{
  /* Event structure */
  uint32_t temp1;
  uint8_t tempBffr2[20];
  bool Boot_evt = false;
  struct gecko_msg_le_gap_set_mode_rsp_t *Result_Ptr;
  /* Check whether there is an event to service */
  if (!ble.evt)
  {
    ble.evt = gecko_peek_event();
  }
  if (ble.evt)
  {
    // Log Event
    blemsgs.msgBuff[blemsgs.tx_wr] = BGLIB_MSG_ID(ble.evt->header);
    blemsgs.tx_wr = NextBufIdx(blemsgs.tx_wr);
    
    /* Service based on event header message ID */
    temp1 = BGLIB_MSG_ID(ble.evt->header);
    switch(temp1)
    //switch (BGLIB_MSG_ID(ble.evt->header))
    {
      /* System boot handler */
      case gecko_evt_system_boot_id:
        RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)"<BGM_BOOT>");
        Boot_evt = true;
        /* Flag that the BLE module has booted */
        ble.booted = true;
        /* Fallthrough intentional */
      /* Connection closed handler */
      case gecko_evt_le_connection_closed_id:
        // Wait for power to stabilize...200msec
//        RoadBrd_Delay( 100 );
//        gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);
//        RoadBrd_Delay( 100 );
//        gecko_cmd_le_gap_set_mode(le_gap_non_discoverable, le_gap_non_connectable);
//        RoadBrd_Delay( 100 );
        /* Set GAP mode: discoverable and connectable */
        ble.connection = false;
//        Result_Ptr = gecko_cmd_le_gap_set_mode(le_gap_limited_discoverable, le_gap_undirected_connectable);
        Result_Ptr = gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable);
        if ( Result_Ptr->result == 0)
//        if (gecko_cmd_le_gap_set_mode(le_gap_general_discoverable, le_gap_undirected_connectable)->result == 0)
//        if (gecko_cmd_le_gap_set_mode(le_gap_limited_discoverable, le_gap_undirected_connectable)->result < bg_errspc_bg)
        {
          // Don't reset first time. This is legitimate
          // The following is a simple patch...Best way right now to recover is to force HARD Reset....
          if (Boot_evt == false)
          {
            RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)"<BGM_CNCTCLOSE>");
            RdBrd_ErrCdLogErrCd( ERROR_BGM_CNNCT, MODULE_bgm111 );
            Clr_HrtBeat_Cnt();
            RdBrd_BlinkErrCd( ERROR_BGM_CNNCT );
            //RoadBrd_Delay( 1000 );
            HAL_NVIC_SystemReset();
          }
          /* We succeeded, don't handle this event again */
          ble.evt = NULL;
        }
        break;
      //case 0x000800A0:
      case gecko_evt_le_connection_opened_id:
        /* Open Event...Set Active Connection Flag */
        /* Don't handle this event again */
        RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)"<BGM_CNCTOPEN>");
        ble.connection = true;
        ble.evt = NULL;
        break;
      // gecko_evt_gatt_server_user_write_request_id
      case 0x020A0020:
        // Clear Heart Beat... We have detected it.
        RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)"-*-");
        Clr_HeartBeat();
        ble.evt = NULL;
        break;
      //case 0x030A0000:
      case gecko_evt_gatt_server_characteristic_status_id:
        // Clear Heart Beat... We have detected it.
        RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)"-O-");
        Clr_HeartBeat();
        ble.evt = NULL;
        break;
//      case 0x080000A0:
//      case 0x020B00A0:
//      case 0x000000A0:
      case 0x200000A0:
      case 0x050A0020:
//      case 0x200000A0:
//      case gecko_cmd_gatt_server_send_characteristic_notification_id: */
      /* Dummy catchall */
        /* Don't handle this event again */
        ble.evt = NULL;
        break;
      case gecko_rsp_gatt_read_characteristic_value_by_uuid_id:
      case 0x020800A0:
      case 0x000900A0:
        RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)"<rsp_read_ch_value_by_uuid_id>");
        if (ble.evt->data.evt_gatt_server_attribute_value.value.len != 0)
        {
          sprintf( (char *)tempBffr2, "<STR:%s>", ble.evt->data.evt_gatt_server_attribute_value.value.data);
          RoadBrd_UART_Transmit(MONITOR_UART, tempBffr2);
        }
        ble.evt = NULL;
        break;
      case gecko_evt_endpoint_status_id:
        RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)"<evt_endpoint_status_id>");
        ble.evt = NULL;
        break;
      /* Dummy catchall */
      default:
        sprintf( (char *)tempBffr2, "<UNKN:%08x>", temp1);
        RoadBrd_UART_Transmit(MONITOR_UART, tempBffr2);
        /* Don't handle this event again */
        ble.evt = NULL;
        break;
    };
    // Test RX Buffer and Set req_exec flag.
//    if (BufUsed(ble.rx_wr, ble.rx_rd) != 0)
//    {
//      /* Indicate we need to execute the BLE stack to process 
//       * the received packet */
//      ble.req_exec = true;
//    }
  } // EndIf (ble.evt)
}

/* BLE write characteristic */

void BGM111_WriteCharacteristic(uint16_t handle, uint8_t len, uint8_t *data)
{
  /* Write the attribute */
  gecko_cmd_gatt_server_write_attribute_value(handle, 0, len, data);
  /* Also trigger notify if enabled */
  gecko_cmd_gatt_server_send_characteristic_notification(0xFF,
            handle, len, data);
}

/**
  * @brief  Check whether the BLE module has booted and is ready for a command.
  * @retval bool:         true(1):        BGAPI IS ready.
  *                       false(0):         BGAPI NOT ready.
  */
bool BGM111_Ready(void)
{
  return ble.booted;
}

/**
  * @brief  Check whether the BLE module is connected.
  * @retval bool:         true(1):        Connection is Active.
  *                       false(0):       NO Connection.
  */
bool BGM111_Connected(void)
{
  return ble.connection;
}

/**
  * @brief  Check whether the BLE module is DATA connected.
  * @retval bool:         true(1):        DATA Connection is Active.
  *                       false(0):       NO DATA Connection.
  */
bool BGM111_DataConnected(void)
{
  return ble.data_Connection;
}

/**
  * @brief  Check whether the BLE module is in CMD Mode.
  * @retval bool:         true(1):        CMD Mode is Active.
  *                       false(0):       CMD Mode is  not Active.
  */
bool BGM111_CMD_Mode(void)
{
  return ble.CMD_Mode;
}

/**
  * @brief  Set the BLE module is DATA connected.
  * @param bool:         true(1):        DATA Connection is Active.
  *                      false(0):       NO DATA Connection.
  */
void BGM111_SetDataConnected(bool NewMode)
{
  ble.data_Connection = NewMode;
}

/**
  * @brief  Set new BLE CMD Mode.
  * @param bool:         true(1):        CMD Mode is Active.
  *                      false(0):       CMD Mode is  not Active.
  */
void BGM111_SetCMD_Mode(bool NewMode)
{
  ble.CMD_Mode = NewMode;
}

/**
  * @brief  Check whether the BLE module is in Sync Mode and Waiting for ACK.
  * @retval bool:         true(1):        Sync Ready for Processing next Frame.
  *                       false(0):       Sync needs to wait.
  */
bool BGM111_SyncModeTest(void)
{
  uint8_t tempBffr2[20];

  // Is Sync Mode armed? Yes.. Then Need to test SyncFlag
  if (ble.TackArmed == TACK_SYNC)
  {
    // If SyncFlag is SYNC_PROC, then allow Frame send.
    if (ble.SyncFlag == SYNC_PROC)
      return true;
    // NO, then Incrment count, We are one step closer to Reset Code.
    else
    {
      // Test to see if we have had a timing tick yet...
      if ( TstDataReady() )
      {
        // Clear Flag for next Tick Event.
        ClrDataReady();
        // Increment Cnt and Report...
        ble.TackCnt++;
        sprintf( (char *)tempBffr2, "<TACK Strike:%d/%d>", ble.TackCnt, RoadBrd_Get_TackLimit() );
        RoadBrd_UART_Transmit(MONITOR_UART, tempBffr2);

        if (ble.TackCnt > RoadBrd_Get_TackLimit())
        {
          // Time to process error and reset code....NO Choice.
          RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)"<BGMSYNC_CNCTCLOSE>");
          RdBrd_ErrCdLogErrCd( ERROR_BGM_SYNCCNCT, MODULE_bgm111 );
          Clr_HrtBeat_Cnt();
          RdBrd_BlinkErrCd( ERROR_BGM_SYNCCNCT );
          //RoadBrd_Delay( 1000 );
          HAL_NVIC_SystemReset();
        } // EndIf (ble.TackCnt >TACK_LIMIT)
      } // EndIf ( TstDataReady() )
      return false;
    } // EndElse (ble.SyncFlag == SYNC_PROC)
  } // EndIf (ble.TackArmed == TACK_SYNC)
  // No....Then we can continue process. Return true.
  else
    return true;
}

/**
  * @brief  Check whether the BLE module is in Sync Mode and Waiting for ACK.
  * @retval bool:         true(1):        Sync Ready for Processing next Frame.
  *                       false(0):       Sync needs to wait.
  */
bool BGM111_SyncModeTestNoInc(void)
{
  // Is Sync Mode armed? Yes.. Then Need to test SyncFlag
  if (ble.TackArmed == TACK_SYNC)
  {
    // If SyncFlag is SYNC_PROC, then allow Frame send.
    if (ble.SyncFlag == SYNC_PROC)
      return true;
    else
      return false;
  } // EndIf (ble.TackArmed == TACK_SYNC)
  // No....Then we can continue process. Return true.
  else
    return true;
}

/**
  * @brief  Set new Value for Sync Flag.
  * @param uint8_t:       SYNC_WAIT(0):       Force Wait on all tests to send additional data
  *                       SYNC_PROC(1):       Allow sending of additional frame of data.
  * @retval None
  */
void BGM111_SetSyncFlg(uint8_t NewFlag)
{
  ble.SyncFlag = NewFlag;
}

/**
  * @brief  Set new Value for Sync Flag if in SYNC Mode.
  * @param uint8_t:       SYNC_WAIT(0):       Force Wait on all tests to send additional data
  *                       SYNC_PROC(1):       Allow sending of additional frame of data.
  * @retval None
  */
void BGM111_cntrlSetSyncFlg(uint8_t NewFlag)
{
  // Is Sync Mode armed? Yes.. Then Need to test SyncFlag
  if (ble.TackArmed == TACK_SYNC)
  {
    ble.SyncFlag = NewFlag;
  }
}

/**
  * @brief  Return Tack State.
  * @retval uint8_t:      TACK_OFF      = 0        Power Up Initialized Value.
  *                       TACK_ARMED    = 1        Set when Connection has been detected.
  *                       TACK_ARMED2   = 2        Set when First Tick Generated assuming TACK_ARMED State detected.
  *                       TACK_SYNC     = 3        Set when first TACK detected while a TACK_ARMED2 State detected.
  *                       TACK_ASYNC    = 4        Set when next TICK detected while TACK_ARMED2 State active.
  */
uint8_t BGM111_GetTackState(void)
{
  return ble.TackArmed;
}

/**
  * @brief  Set Tack State.
  * @param uint8_t:       TACK_OFF      = 0        Power Up Initialized Value.
  *                       TACK_ARMED    = 1        Set when Connection has been detected.
  *                       TACK_ARMED2   = 2        Set when First Tick Generated assuming TACK_ARMED State detected.
  *                       TACK_SYNC     = 3        Set when first TACK detected while a TACK_ARMED2 State detected.
  *                       TACK_ASYNC    = 4        Set when next TICK detected while TACK_ARMED2 State active.
  * @retval None
  */
void BGM111_SetTackState(uint8_t NewValue)
{
  ble.TackArmed = NewValue;
}

/**
  * @brief  This interrupt handler is called to handle the Usart3 interruptes.
  *         from the BGM111
  * @param  None
  * @retval None
  */
void BGM111_UART_IRQHandler(UART_HandleTypeDef *huart)
{
  /* Transmit register empty? */
  //if (USART_GetITStatus(BGM111_UART, USART_IT_TXE) == SET)
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_TXE) == SET)
  {
    /* Are we still sending data? */
    if (ble.tx_rd != ble.tx_wr)
    {
      /* Send a byte */
      USART_SendData(huart, ble.tx_buf[ble.tx_rd]);
      /* Bump the index */
      ble.tx_rd = NextBufIdx(ble.tx_rd);
    }
    else
    {
      /* Turn off the transmit interrupt */
      __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);
    }
  }
  
  /* Was there an error? */
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE | 
      HAL_UART_ERROR_PE | UART_FLAG_NE | UART_FLAG_FE) == SET)
  {
    /* Reset the receive state */
    ble.rx_state = BGRX_SYNC;
    /* Clear the error by reading the data register */
    USART_ReceiveData(huart);
    /* We're done */
    return;
  }

  /* Was a new byte received? */
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) == SET)
  {
    /* Get the byte (this also clears the flag) */
    uint8_t c = USART_ReceiveData(huart);
    /* Process the received character... */
    RoadBrd_ProcessBGMChar(c);
  }
}

/* Transmit data to the BGM111 module */
/**
  * @brief  Returns the most recent received data by the USARTx peripheral.
  * @param  USARTx: Select the USART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @retval The received data.
  */
void BGM111_Transmit(uint32_t len, uint8_t *data)
{
  uint32_t tempmsg;
  uint32_t timeout_Cnt;
  
  // Log MSG into buffer
  tempmsg = 0xff000000 | len;
  blemsgs.msgBuff[blemsgs.tx_wr] = tempmsg;
  blemsgs.tx_wr = NextBufIdx(blemsgs.tx_wr);
  tempmsg = (uint32_t)(data[0]<<24) + (uint32_t)(data[1]<<16) + (uint32_t)(data[2]<<8) + (uint32_t)(data[3]);
  blemsgs.msgBuff[blemsgs.tx_wr] = tempmsg;
  blemsgs.tx_wr = NextBufIdx(blemsgs.tx_wr);
  tempmsg = (uint32_t)(data[4]<<24) + (uint32_t)(data[5]<<16) + (uint32_t)(data[6]<<8) + (uint32_t)(data[7]);
  blemsgs.msgBuff[blemsgs.tx_wr] = tempmsg;
  blemsgs.tx_wr = NextBufIdx(blemsgs.tx_wr);
  
  /* Add bytes when there is space, wait if necessary */
  // OK...But we Can't wait forever...Will place a timeout Count.
  timeout_Cnt = TX_TIMEOUT_CNT;
  
  while (len) {
    /* Get the next write index */
    uint8_t nextidx = NextBufIdx(ble.tx_wr);
    /* We're not hitting the read index? (There is space?) */
    if (nextidx != ble.tx_rd)
    {
      /* Put the data in the buffer */
      ble.tx_buf[ble.tx_wr] = *data;

      /* Disable interrupt */
      HAL_NVIC_DisableIRQ(USART3_IRQn);
      /* Trigger an interrupt if we're starting with an empty buffer */
      if (ble.tx_wr == ble.tx_rd)
      {
        HAL_UART_EnableBGM_TXE();
      }
      /* Increment the write index */
      ble.tx_wr = nextidx;
      /* Enable interrupt */
      HAL_NVIC_EnableIRQ(USART3_IRQn);

      /* Update the incoming data length and pointer */
      len--;
      data++;
      //Found Space...Reset Count.
      timeout_Cnt = TX_TIMEOUT_CNT;
    }
    else
    {
      //Decrement Count
      if ( timeout_Cnt-- == 0)
      {
        // If this happens...NOT BOOTED..Clear ths flag or infinite loop.
        ble.booted = false;
        // We have detected a ERROR_TXBGMBUF_FULL error on BGM111...Log it!
        RdBrd_ErrCdLogErrCd( ERROR_TXBGMBUF_FULL, MODULE_bgm111 );
        RdBrd_BlinkErrCd( ERROR_TXBGMBUF_FULL );
        HAL_NVIC_SystemReset();
      }
    }
  }
}

/**
  * @brief  Transmits single data through the USARTx peripheral.
  * @param  USARTx: Select the USART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  Data: the data to transmit.
  * @retval None.
  */
void USART_SendData(UART_HandleTypeDef *huart, uint16_t Data)
{
  /* Transmit Data */
  huart->Instance->DR = (Data & (uint16_t)0x01FF);
}

/**
  * @brief  Returns the most recent received data by the USARTx peripheral.
  * @param  USARTx: Select the USART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @retval The received data.
  */
uint16_t USART_ReceiveData(UART_HandleTypeDef *huart)
{
  /* Receive Data */
  return (uint16_t)(huart->Instance->DR & (uint16_t)0x01FF);
}

/**
  * @brief  This routine Process the passed character as part of the process of building status
  *         from the BGM111
  * @param  uint8_t c: Character to process.
  * @retval HAL_StatusTypeDef:     HAL_OK:       Tasking of block of data to UART success.
  *                                HAL_ERROR:    Error found in Tasking or data passed.
  *                                HAL_BUSY:     UART is busy.
  *                                HAL_TIMEOUT:  UART timed out.
  */
HAL_StatusTypeDef RoadBrd_ProcessBGMChar(uint8_t c)
{
//  static uint8_t header_cnt, payload_cnt, payload_len;
  HAL_StatusTypeDef Status;
  static uint8_t tempBffr2[60];
  char* tempPstr;
  char tempstr[60];
#ifndef XML_SHRT  
  char tempstr2[60];
#endif
//  int x;

  uint8_t tempBffr3[60];
  static uint8_t in_ptr = 0;
  
  // First pull new character into buffer.
  tempBffr2[in_ptr++] = c;
  // Now, Did we get a termination character?
  if( (c == 0x0a)  ||
      (c == '?') )
  {
    // Reset Ptr.
    in_ptr = 0;

    // Yes...We will now test contents of buffer. And then reset ptr back to 0.
    sprintf( (char *)tempBffr3, "\r\n<<FULL STRING>>: %s \r\n", tempBffr2);
    Status = RoadBrd_UART_Transmit(MONITOR_UART, tempBffr3);
    if (Status != HAL_OK)
      return Status;
    // Test Strings for Key items.
    // Boot String?
    if (strncmp((char *)tempBffr2,"Boot",4) == 0)
    {
      // Yes....Set Boot Flag.
      ble.booted = true;
      // If we are booted....Then lets arm TACK Test Code.
      ble.data_Connection = false;
      ble.CMD_Mode = false;
      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)"<ble.booted> ");
    }
    // Connection String?
    else if (strncmp((char *)tempBffr2,"Connected",9) == 0)
    {
      // Yes....Set Boot Flag.
      ble.connection = true;
      ble.TackArmed = TACK_ARMED;
      ble.TackCnt = 0;
      RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)"<ble.TackArmed = TACK_ARMED>");
      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)"<ble.connection> ");

    }
    // Disconnection String?
    else if (strncmp((char *)tempBffr2,"Disconnected",12) == 0)
    {
      // Yes....Clear Flags.
      ble.connection = false;
      ble.data_Connection = false;
      ble.CMD_Mode = false;
      ble.TackArmed = TACK_ARMED;
      ble.TackCnt = 0;
      RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)"<ble.TackArmed = TACK_ARMED>");
      ClrDataStructure();                           // Clear Backup data structure.
      ClrAnalyticsRepeat();                          // Clear Frame Repeat Count.
      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)"<DISCONNECTED> ");
    }
    // Data String?
    else if (strncmp((char *)tempBffr2,"DATA",4) == 0)
    {
      // 1. Send String to Server to indicate new CMD Mode.
      sprintf( (char *)tempBffr2, "<STATUS>CMD</STATUS>" );
      Status = RoadBrd_UART_Transmit(MONITOR_UART, tempBffr2);
      if (Status != HAL_OK)
        return Status;
      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), tempBffr2);
      
      // 2. Set the Timer to the RD_Sound at 1 Second Increments.
      RoadBrd_Set_TmpRdSndTickCnt( CMD_TIME );                  // One Second Ticks.
      
      // 3. Set CMD_Mode Active.
      BGM111_SetCMD_Mode( true );
      
      // Final Status.
      //ble.data_Connection = true;
      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)"<ble.data_Connection> ");
    }
    // TACK String?
    else if (strncmp((char *)tempBffr2,"<TACK>",6) == 0)
    {
      // CMD_Mode active?
      if ( BGM111_CMD_Mode() )
      {
        // Send String to Server.
        sprintf( (char *)tempBffr3, "<STATUS>DATA_SYNC</STATUS>" );
        Status = RoadBrd_UART_Transmit(MONITOR_UART, tempBffr3);
        if (Status != HAL_OK)
          return Status;
        BGM111_Transmit((uint32_t)(strlen((char *)tempBffr3)), tempBffr3);
        // Clear CMD_Mode.
        BGM111_SetCMD_Mode( false );
        // Set Data_Connection Mode.
        BGM111_SetDataConnected( true );
        // Change RD_Sound Timer to correct value for Data Mode.
        // First Reload FLASH Frames
        RoadBrd_WWDG_VerifyFrame();
        // NOW...Reload Active Timer.
        Set_RdSndTickCnt( RoadBrd_Get_RdSndTickCnt() );
        // Now Set Correct SYNC Mode.
        ble.TackArmed = TACK_SYNC;
        ble.TackCnt = 0;
        Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)"<ble.TackArmed=TACK_SYNC>");
      }
      else if (ble.TackArmed == TACK_ARMED2)
      {
        ble.TackArmed = TACK_SYNC;
        ble.TackCnt = 0;
        sprintf( (char *)tempBffr3, "<STATUS>DATA_SYNC</STATUS>" );
        Status = RoadBrd_UART_Transmit(MONITOR_UART, tempBffr3);
        if (Status != HAL_OK)
          return Status;
        BGM111_Transmit((uint32_t)(strlen((char *)tempBffr3)), tempBffr3);
        Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)"<ble.TackArmed=TACK_SYNC>");
      }
      else if (ble.TackArmed == TACK_SYNC)
      {
        ble.TackCnt = 0;
        Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)"<TACK Received.>");
        // Set Sync Flag for Frame.
          BGM111_SetSyncFlg( SYNC_PROC );
      }
      // OK...Now we need to process what is in the TACK Section.
#ifdef XML_SHRT      
      //********************************************************************************
      //*
      //*  Abbreviate XML Code Processing here for Platinum Initial Release
      //*
      //********************************************************************************
      // A. Strip off Opening <TACK>.
      tempPstr = (char *)&tempBffr2[6];
      strcpy(tempstr, tempPstr);
      //************************ SEQUENCE OF TEST TO TYPE OF OPERATION
      // B. Time to Test String
      // Is this a <CMD>/Monitor Command
      tempPstr = strstr( tempstr, "</TACK>" );
      if (tempPstr  != NULL)
      {
        // Now NULL Out where Tag is at.
        *tempPstr = NULL;
        // Is this a Monitor Command?
        if (strlen(tempstr) > 0)
        {
          // Finally, Send string to Parser.
          Status = RoadBrd_ParserTsk(tempstr);
          if (Status == HAL_BUSY)
          {
            sprintf( (char *)tempBffr3, "<STATUS>CMD_BUSY</STATUS>" );
            Status = RoadBrd_UART_Transmit(MONITOR_UART, tempBffr3);
            BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), tempBffr2);
          }
          else if (Status != HAL_OK)
          {
            sprintf( (char *)tempBffr3, "<STATUS>CMD_ERROR</STATUS>" );
            Status = RoadBrd_UART_Transmit(MONITOR_UART, tempBffr3);
            BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), tempBffr2);
          }
        }
      }
      // Generate an ACK Report.
      sprintf( (char *)tempBffr2, "<STATUS>ST_ACK:%s</STATUS>", RoadBrd_WWDG_GetTickString() );
      Status = RoadBrd_UART_Transmit(MONITOR_UART, tempBffr2);
      if (Status != HAL_OK)
        return Status;
      BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), tempBffr2);
      //********************************************************************************
      //*  END OF CUSTOM SECTION...Abbreviate XML Code Processing here for Platinum Initial Release
      //********************************************************************************
#else      
      //********************************************************************************
      //*
      //*  Normal XML Code Processing here for Platinum Release
      //*
      //********************************************************************************
      // A. Strip off Opening <TACK>.
      tempPstr = (char *)&tempBffr2[6];
      strcpy(tempstr, tempPstr);
      // A1. Find if we have a <DATE> Tag.
      tempPstr = strstr( tempstr, "<DATE>" );
      if (tempPstr != NULL)
      {
        // Found DATE TAG....Set Focus to DATE Tag.
        strcpy(tempstr, tempPstr);
        // Now Strip out DATE Tag and Get Date.
        tempPstr = (char *)&tempstr[6];
        strcpy(tempstr2, tempPstr);
        strcpy(tempstr, tempPstr);
        // Find </DATE>
        tempPstr = strstr( tempstr2, "</DATE>" );
        if (tempPstr != NULL)
        {
          // Found Second String.....NULL It so that we can get Date
          //x = (int)tempPstr;
          //tempstr[x] = NULL;
          *tempPstr = NULL;
          //Now save Date String.
          RoadBrd_WWDG_SetDateString(tempstr2);
        }
        // Let's Advance past this date string.
        tempPstr = strstr( tempstr, "</DATE>" );
        strcpy(tempstr, tempPstr);
        // Now Strip out /DATE Tag and Get Date.
        tempPstr = (char *)&tempstr[7];
        strcpy(tempstr, tempPstr);
      } 
      //************************ SEQUENCE OF TEST TO TYPE OF OPERATION
      // B. Time to Test String
      // Is this a <CMD>/Monitor Command
      tempPstr = strstr( tempstr, "<CMD>" );
      if (tempPstr  != NULL)
      {
        // Yes This is a Monitor CMD... Parse out Key CMD and pass to parser.
        strcpy((char *)tempBffr2, tempPstr);
        tempPstr = (char *)&tempBffr2[5];
        strcpy(tempstr, tempPstr);
        // Finally, Send string to Parser.
        Status = RoadBrd_ParserTsk(tempstr);
        if (Status == HAL_BUSY)
        {
          sprintf( (char *)tempBffr3, "<STATUS>CMD_BUSY</STATUS>" );
          Status = RoadBrd_UART_Transmit(MONITOR_UART, tempBffr3);
        }
        else if (Status != HAL_OK)
        {
          sprintf( (char *)tempBffr3, "<STATUS>CMD_ERROR</STATUS>" );
          Status = RoadBrd_UART_Transmit(MONITOR_UART, tempBffr3);
        }
      }
      else
      {
        // Generate an ACK Report.
        sprintf( (char *)tempBffr2, "<STATUS>ST_ACK:%s</STATUS>", RoadBrd_WWDG_GetDateString() );
        Status = RoadBrd_UART_Transmit(MONITOR_UART, tempBffr2);
        if (Status != HAL_OK)
          return Status;
        BGM111_Transmit((uint32_t)(strlen((char *)tempBffr2)), tempBffr2);
      }
      //********************************************************************************
      //*  END OF CUSTOM SECTION...Normal XML Code Processing here for Platinum Release
      //********************************************************************************
#endif      
    } // EndIf (strncmp((char *)tempBffr2,"<TACK>",6) == 0)
    // NOW TEST ERROR CONDITIONS!!!
    // OVERFLOW?
    else if (strncmp((char *)tempBffr2,"OVERFLOW",8) == 0)
    {
      // Yes....Report Error and Reset.
      // We have detected a ERROR_TXBGMBUF_FULL error on BGM111...Log it!
      RdBrd_ErrCdLogErrCd( ERROR_BGM_OVERFLOW, MODULE_bgm111 );
      RdBrd_BlinkErrCd( ERROR_BGM_OVERFLOW );
      HAL_NVIC_SystemReset();
    }
    else
    {
      Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)"<UNKNOWN STATUS> ");
    }
  }
  //tempBffr2[0] = c;
  //tempBffr2[1] = 0x00;
  //Status = RoadBrd_UART_Transmit(MONITOR_UART, (uint8_t *)tempBffr2);
  return Status;
#if 0
    Status = HAL_OK;
    // Test Buffer. If we are full, An error has occured. Must at least log that.
    if (IsBufFull(ble.rx_wr, ble.rx_rd))
    {
      RdBrd_ErrCdLogErrCd( ERROR_BGMBUF_FULL, MODULE_bgm111 );
      Clr_HrtBeat_Cnt();
      RdBrd_BlinkErrCd( ERROR_BGMBUF_FULL );
      //RoadBrd_Delay( 1000 );
      HAL_NVIC_SystemReset();
      return HAL_ERROR;
    }
    /* Execution based on state */
    switch (ble.rx_state)
    {
      /* Waiting for a valid start of header */
      default:
      case BGRX_SYNC:
        /* Valid start of header? (response or event) */
        if ((c & 0xF8) ==
            ((uint8_t)gecko_dev_type_gecko | (uint8_t)gecko_msg_type_rsp) ||
            (c & 0xF8) ==
            ((uint8_t)gecko_dev_type_gecko | (uint8_t)gecko_msg_type_evt))
        {
          /* Receiving header */
          ble.rx_state = BGRX_HDR;
          /* Initialize header byte counter */
          header_cnt = 0;
        }
        else
        {
          /* Stay in sync state until we receive a valid start of header */
          // We have detected a SYNC error on BGM111...Log it!
          RdBrd_ErrCdLogErrCd( ERROR_BGMSYNC, MODULE_bgm111 );
          Clr_HrtBeat_Cnt();
          if (ble.booted)
            RdBrd_BlinkErrCd( ERROR_BGMSYNC );
          //RoadBrd_Delay( 1000 );
          HAL_NVIC_SystemReset();
          break;
        }
        /* Fallthrough intentional */
      /* Receiving header */
      case BGRX_HDR:
        if (IsBufFull(ble.rx_wr, ble.rx_rd))
        {
          // Oops...Detected a fatal error...RESET!!!
          // We have detected a SYNC error on BGM111...Log it!
          RdBrd_ErrCdLogErrCd( ERROR_BGMSYNC, MODULE_bgm111 );
          /* Indicate we need to execute the BLE stack, it's the
           * only way to get more space in the buffer */
          ble.req_exec = true;
          /* We're back to synchronizing */
          ble.rx_state = BGRX_SYNC;
        }
        else
        {
          /* Save the received byte */
          /* Save the received byte */
          ble.rx_buf[ble.rx_wr] = c;
          /* Increment the index and header byte counter */
          ble.rx_wr = NextBufIdx(ble.rx_wr);
          header_cnt++;
          /* If this is the second header byte, we can grab the payload
           * length.  We ignore the first byte, since the spec says that
           * due to memory limitations in the modules, the packet is
           * never more than 64 bytes. */
          if (header_cnt == 2)
          {
            /* Get the payload length */
            payload_len = c;
            /* If we have a payload bigger than 60 bytes, something's wrong */
            if (payload_len > 60)
            {
              // We have detected a SYNC error on BGM111...Log it!
              RdBrd_ErrCdLogErrCd( ERROR_BGMSYNC, MODULE_bgm111 );
              /* Reset receive state to synchronizing */
              ble.rx_state = BGRX_SYNC;
              /* Indicate we need to execute the BLE stack to free space */
              ble.req_exec = true;
              Status = HAL_ERROR;
            }
          } // EndIf (header_cnt == 2)
          /* Are we done with the header? */
          if (header_cnt >= BGLIB_MSG_HEADER_LEN)
          {
            /* Is there no payload? */
            if (payload_len == 0)
            {
              /* Reset receive state to synchronizing */
              ble.rx_state = BGRX_SYNC;
              /* Indicate we need to execute the BLE stack to process 
               * the received packet */
              ble.req_exec = true;
            }
            else
            {
              /* Start receiving payload data */
              ble.rx_state = BGRX_DATA;
              /* Initialize the payload counter */
              payload_cnt = 0;
            }
          } // EndIf (header_cnt >= BGLIB_MSG_HEADER_LEN)
        } // EndElse (IsBufFull(ble.rx_wr, ble.rx_rd))
        break;
      /* Receiving data */
      case BGRX_DATA:
        /* Did we receive a byte, but the buffer is full? */
        if (IsBufFull(ble.rx_wr, ble.rx_rd))
        {
          /* Indicate we need to execute the BLE stack, it's the
           * only way to get more space in the buffer */
          ble.req_exec = true;
          /* We're back to synchronizing */
          ble.rx_state = BGRX_SYNC;
        }
        else
        {
          /* Store the byte */
          ble.rx_buf[ble.rx_wr] = c;
          /* Increment the index and payload byte counter */
          ble.rx_wr = NextBufIdx(ble.rx_wr);
          payload_cnt++;
          /* Is this the end of the packet? */
          if (payload_cnt >= payload_len)
          {
            /* Reset receive state to synchronizing */
            ble.rx_state = BGRX_SYNC;
            /* Indicate we need to execute the BLE stack to process 
             * the received packet */
            ble.req_exec = true;
          }
          /* Is the buffer almost full? */
          if (BufFree(ble.rx_wr, ble.rx_rd) <= BG_DATA_LOW_WATERMARK)
          {
            /* Indicate we need to execute the BLE stack so it can
             * start reading data from the buffer */
            ble.req_exec = true;
          }
        } // EndElse (IsBufFull(ble.rx_wr, ble.rx_rd))
        break;
    } // EndSwitch (ble.rx_state)
    return Status;
#endif
}

/**
  * @brief  This routine returns the status of the ble.req_exec flag
  *         from the BGM111
  * @param  None
  * @retval bool:     true(1):     Processing of packet about to start
  *                   false(0):      No processing pending.
  */
bool RoadBrd_tstReqexec( void )
{
  return ble.req_exec;
}


