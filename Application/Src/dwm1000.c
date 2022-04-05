 /**
  ******************************************************************************
  * @file    DWM1000.c
  * @author  IBronx MDE team
  * @brief   Application program to control DWM1000 which is is an
  *          IEEE802.15.4-2011 UWB compliant wireless transceiver module
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 IBronx.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by IBronx under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "dwm1000.h"
#include "errorcode.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "cmsis_os.h"
#include "SEGGER_RTT.h"
#include "dws1000_control.h"
#include "flash_control.h"
#include "distance_process.h"
#include "led_control.h"
#include "logger.h"
#include "main_task.h"
#include "app_config.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Private typedef -----------------------------------------------------------*/
typedef signed long long int64;
typedef unsigned long long uint64;

// Default communication configuration.
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PRF_16M,     /* Pulse repetition frequency. */
    DWT_PLEN_256,    /* Preamble length. Used in TX only. */
    DWT_PAC16,        /* Preamble acquisition chunk size. Used in RX only. */
    4,               /* TX preamble code. Used in TX only. */
    4,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (257 + 8 - 16) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

static uint8 dw_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0, 0};
static uint8 dw_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x02, 0, 0, 0, 0};
static uint8 dw_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 dw_report_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0, 0, 0, 0, 0};

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint64 dw_poll_rx_ts;
static uint64 dw_resp_tx_ts;
static uint64 dw_final_rx_ts;
static uint64 dw_poll_tx_ts;
static uint64 dw_resp_rx_ts;
static uint64 dw_final_tx_ts;
static uint8_t dw_frameNum = 0;
static uint8_t dw_rx_msg[DW_FRAME_LEN_MAX];
static double dw_tof;
static double dw_distance;
static volatile uint8_t dw_wait_reset_done;
uint8_t dw_devId[FLASH_DEVICE_ID_SIZE];
static uint16_t dw_networkId;
static uint16_t lp_osc_cal;

dw_DeviceTypeDef dw_device;
dw_DataTypeDef dw_eventlog[DISTANCE_MAX_SUPPORT_DEVICE];
uint32_t dw_devicelist[DISTANCE_MAX_SUPPORT_DEVICE];
uint8_t dw_evtlogcnt;
uint8_t dw_devicelistcnt;
uint8_t dw_currentPollRetry;
bool bEnableReceptionMode;

extern osMessageQueueId_t osMsgQ_distance;
extern osEventFlagsId_t osFlags_Distance;
extern osEventFlagsId_t osFlags_Main;

extern TIM_HandleTypeDef htim14;

/* function prototypes -------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void dw_rx_ok_cb(const dwt_cb_data_t *cb_data);
static void dw_rx_to_cb(const dwt_cb_data_t *cb_data);
static void dw_rx_err_cb(const dwt_cb_data_t *cb_data);

/**
  * @brief  Initialize the DWM1000 module after success configure SPI
  * @param  None
  * @retval rc:  return PER_NO_ERROR if no error, otherwise return rc
  */
uint32_t DWM1000_DeviceInit(void)
{
  // issue a wake-up in case DW1000 is asleep.
  dwm1000_WakeUp();

  // reset and initialize DWM1000.
  dwm1000_Reset();

  // for initialization, DW1000 clocks must be temporarily set to crystal speed.
  dwm1000_SetSlowClockRate();
  if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
  {
    logger_LogError("[DWM1000] - Failed to initialize", NULL);
    return PER_ERROR_DW1000_INIT;
  }

  // measure low power oscillator frequency
  lp_osc_cal = dwt_calibratesleepcnt();

  // after initialization, SPI rate can be increased for optimum performance.
  dwm1000_SetFastClockRate();

  // configure DW1000
  dwt_configure(&config);

  // configure sleep and wake-up parameters
  dwt_configuresleep(DWT_PRESRV_SLEEP /*| DWT_LOADOPSET */| DWT_CONFIG, DWT_WAKE_CS | DWT_SLP_EN);

  // enable the interrupt
  dwm1000_RegisterInterruptCallback(&dwt_isr);

  // register the interrupt callback function
  dwt_setcallbacks(NULL, &dw_rx_ok_cb, &dw_rx_to_cb, &dw_rx_err_cb);

  // enable wanted interrupts (RX good frames, RX timeouts and RX errors)
  dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);

  // apply default antenna delay value.
  dwt_setrxantennadelay(DW_RX_ANT_DLY);
  dwt_settxantennadelay(DW_TX_ANT_DLY);

  // read network & device Id from flash memory
  DWM1000_SetMessageDevId();
  DWM1000_SetMessageNetId();

  // initialize parameters
  dw_frameNum = 0;
  DWM1000_ResetDeviceFilterList();

  return PER_NO_ERROR;
}

/**
  * @brief  Callback to process RX OK events
  * @param  cb_data:  callback data
  * @retval None
  */
static void dw_rx_ok_cb(const dwt_cb_data_t *cb_data)
{
  SEGGER_SYSVIEW_RecordEnterISR();

  // clear local RX buffer to avoid having leftovers from previous receptions
  memset(dw_rx_msg, 0, sizeof(dw_rx_msg));

  // a frame has been received, read it into the local buffer
  uint32 frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
  if (frame_len < DW_FRAME_LEN_MAX)
  {
    dwt_readrxdata(dw_rx_msg, frame_len, 0);

    // manual filtering if network Id not the same
    if (dw_networkId != (dw_rx_msg[4] << 8) + dw_rx_msg[3])
    {
      SEGGER_SYSVIEW_PrintfHost("[Distance] - Manual filter network, %d", dw_networkId);
      SEGGER_SYSVIEW_RecordExitISR();
      return;
    }

    // manual filtering if get distance before from same device id, refresh by every 1 seconds
    uint32_t deviceId = (dw_rx_msg[8] << 24) +  + (dw_rx_msg[7] << 16) +
        (dw_rx_msg[6] << 8) + dw_rx_msg[5];

    for (int idx =0; idx < dw_devicelistcnt; idx++)
    {
      if (deviceId == dw_devicelist[idx])
      {
        SEGGER_SYSVIEW_PrintfHost("[Distance] - Manual device filter, %d", deviceId);
        SEGGER_SYSVIEW_RecordExitISR();
        return;
      }
    }

    if (dw_device == DEVICE_TAG)
    {
      if (dw_rx_msg[10] == 0x10)
      {
        SEGGER_SYSVIEW_PrintfHost("[Distance] - TAG send Final Message to %d", deviceId);
        DWM1000_TagSendFinalMsg();
      }
      if (dw_rx_msg[10] == 0x12)
      {
        uint16_t distance = (dw_rx_msg[13] << 8) +  dw_rx_msg[12];
        SEGGER_SYSVIEW_PrintfHost("[Distance] - TAG receive distance %dcm from %d", distance, deviceId);
        DWM1000_TagReceiveReportMsg();
      }
    }
    else
    {
      if (dw_rx_msg[10] == 0x21)
      {
        SEGGER_SYSVIEW_PrintfHost("[Distance] - ANCHOR send Response Message, %d", deviceId);
        DWM1000_AnchorSendResponseMsg();
      }
      if (dw_rx_msg[10] == 0x23)
      {
        SEGGER_SYSVIEW_PrintfHost("[Distance] - ANCHOR send Optional Response Message from %d", deviceId);
        DWM1000_AnchorSendOptionResponseMsg();
      }
    }
  }

  SEGGER_SYSVIEW_RecordExitISR();
}

/**
  * @brief  Callback to process RX timeout events
  * @param  cb_data:  callback data
  * @retval None
  */
static void dw_rx_to_cb(const dwt_cb_data_t *cb_data)
{
  SEGGER_SYSVIEW_RecordEnterISR();

 // if (dw_device == DEVICE_ANCHOR)
    SEGGER_SYSVIEW_Error("[Distance] - Interrupt Rx timeout events");

  // enable reception mode if send poll message timeout
  if (bEnableReceptionMode)
    DWM1000_StartListenMsg();

  SEGGER_SYSVIEW_RecordExitISR();
}

/**
  * @brief  Callback to process RX error events
  * @param  cb_data:  callback data
  * @retval None
  */
static void dw_rx_err_cb(const dwt_cb_data_t *cb_data)
{
  SEGGER_SYSVIEW_RecordEnterISR();

  SEGGER_SYSVIEW_Error("[Distance] - Interrupt Rx error events");

  if (bEnableReceptionMode)
    DWM1000_StartListenMsg();

  SEGGER_SYSVIEW_RecordExitISR();
}

/**
  * @brief  Set the device Id to all the messages
  * @param  None
  * @retval None
  */
void DWM1000_SetMessageDevId(void)
{
  uint8_t devId[FLASH_DEVICE_ID_SIZE];
  flash_read_DeviceId(devId);

  memcpy(&dw_poll_msg[5], &devId[0], sizeof(uint8_t)*FLASH_DEVICE_ID_SIZE);
  memcpy(&dw_resp_msg[5], &devId[0], sizeof(uint8_t)*FLASH_DEVICE_ID_SIZE);
  memcpy(&dw_final_msg[5], &devId[0], sizeof(uint8_t)*FLASH_DEVICE_ID_SIZE);
  memcpy(&dw_report_msg[5], &devId[0], sizeof(uint8_t)*FLASH_DEVICE_ID_SIZE);
  memcpy(&dw_devId[0], &devId[0], sizeof(uint8_t)*FLASH_DEVICE_ID_SIZE);
}

/**
  * @brief  Set the network Id to all the messages
  * @param  None
  * @retval None
  */
void DWM1000_SetMessageNetId(void)
{
  uint8_t netId[2];
  flash_read_NetworkId(netId);

  dw_poll_msg[3] = netId[0];
  dw_poll_msg[4] = netId[1];

  dw_resp_msg[3] = netId[0];
  dw_resp_msg[4] = netId[1];

  dw_final_msg[3] = netId[0];
  dw_final_msg[4] = netId[1];

  dw_report_msg[3] = netId[0];
  dw_report_msg[4] = netId[1];

  dw_networkId = (netId[1] << 8) + netId[0];
}

/**
  * @brief  Get device Id
  * @param  None
  * @retval dw_deviceId:  device Id
  */
uint16_t DWM1000_GetNetworkId(void)
{
  return dw_networkId;
}

/**
  * @brief  Start to broadcast poll message and set a timer to
  *         retransmit poll message automatically
  * @param  None
  * @retval None
  */
void DWM1000_StartBroadcastMsg(void)
{
  HAL_TIM_Base_Start_IT(&htim14);
}

/**
  * @brief  Always listen poll message from Tx, Rx will turn on immediately.
  * @param  None
  * @retval None
  */
void DWM1000_StartListenMsg(void)
{
  SEGGER_SYSVIEW_Print("[Distance] - TAG switch into Listen Mode");

  dw_device = DEVICE_ANCHOR;

  // disable the active transmitter or receiver
  dwt_forcetrxoff();

  // receiver soft reset
  dwt_rxreset();

  // configure lower power preamble-hunt mode
  dwt_setsniffmode(1, 1, 16);

  dwt_setpreambledetecttimeout(0);
  dwt_setrxtimeout(0);
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

/**
  * @brief  Tag start first Two-Way Range Exchange by send poll message
  * @param  bReceptionMode:  Enable ReceptionMode after poll message
  * @retval None
  */
void DWM1000_TagSendPollMessage(bool bReceptionMode)
{
  SEGGER_SYSVIEW_Print("[Distance] - TAG send POLL message");

  dw_device = DEVICE_TAG;
  bEnableReceptionMode = bReceptionMode;

  // disable the active transmitter or receiver and put the device in IDLE mode
  dwt_forcetrxoff();

  // fill sequence number into blink message
  dw_poll_msg[2] = dw_frameNum;

  // write frame data to DWM1000 and prepare transmission.
  dwt_writetxdata(sizeof(dw_poll_msg), dw_poll_msg, 0);
  dwt_writetxfctrl(sizeof(dw_poll_msg), 0, 0);

  // set delay to turn reception on after transmission of the frame
  dwt_setrxaftertxdelay(DW_POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setpreambledetecttimeout(DW_PRE_TIMEOUT);
  int ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
  if (ret != DWT_SUCCESS)
  {
    SEGGER_SYSVIEW_Error("[Distance] - TAG failed to send POLL message");
  }
  // Increment the blink frame sequence number (modulo 256).
  dw_frameNum++;
}

/**
  * @brief  Anchor send Response message to Tag
  * @param  None
  * @retval None
  */
void DWM1000_AnchorSendResponseMsg(void)
{
  // retrieve poll reception timestamp.
  dw_poll_rx_ts = get_rx_timestamp_u64();

  // write and send the response message.
  dw_resp_msg[2] = dw_frameNum;
  dwt_writetxdata(sizeof(dw_resp_msg), dw_resp_msg, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(sizeof(dw_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

  // sets the delay in turning the receiver on after a frame transmission has completed
  dwt_setrxaftertxdelay(DW_RESP_TX_TO_FINAL_RX_DLY_UUS);

  int ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
  if (ret != DWT_SUCCESS)
  {
    SEGGER_SYSVIEW_Print("[Distance] - ANCHOR failed to send Response Message");
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
  }

  dw_frameNum++;
}

/**
  * @brief  Tag send Final message to Anchor, embedded three timestamps - poll_tx, response_rx & final_tx
  * @param  None
  * @retval None
  */
void DWM1000_TagSendFinalMsg(void)
{
  uint32 final_tx_time;

  // retrieve poll transmission and response reception timestamp
  dw_poll_tx_ts = get_tx_timestamp_u64();
  dw_resp_rx_ts = get_rx_timestamp_u64();

  // compute final message transmission time.
  final_tx_time =  dwt_readsystimestamphi32()  + 0x17cdc * 1.5;
  dwt_setdelayedtrxtime(final_tx_time);

  // sets the delay in turning the receiver on after a frame transmission has completed
   dwt_setrxaftertxdelay(DW_RESP_TX_TO_FINAL_RX_DLY_UUS);

  // final TX timestamp is the transmission time we programmed plus the TX antenna delay.
  dw_final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + DW_TX_ANT_DLY;

  // write all timestamps in the final message.
  final_msg_set_ts(&dw_final_msg[DW_FINAL_MSG_POLL_TX_TS_IDX], dw_poll_tx_ts);
  final_msg_set_ts(&dw_final_msg[DW_FINAL_MSG_RESP_RX_TS_IDX], dw_resp_rx_ts);
  final_msg_set_ts(&dw_final_msg[DW_FINAL_MSG_FINAL_TX_TS_IDX], dw_final_tx_ts);

  // write and send final message.
  dw_final_msg[2] = dw_frameNum;
  dwt_writetxdata(sizeof(dw_final_msg), dw_final_msg, 0);
  dwt_writetxfctrl(sizeof(dw_final_msg), 0, 1);

  int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
  if (ret != DWT_SUCCESS)
  {
    SEGGER_SYSVIEW_Print("[Distance] - TAG failed to send Final Message");
  }

  dw_frameNum++;
}

/**
  * @brief  Anchor receive the Final message from Tag
  *         calculate the distance by using Two-Way Ranging method.
  * @param  None
  * @retval None
  */
void DWM1000_AnchorSendOptionResponseMsg(void)
{
  uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
  uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
  double Ra, Rb, Da, Db;
  int64 tof_dtu;

  // retrieve response transmission and final reception timestamps.
  dw_resp_tx_ts = get_tx_timestamp_u64();
  dw_final_rx_ts = get_rx_timestamp_u64();

  // get timestamps embedded in the final message.
  final_msg_get_ts(&dw_rx_msg[DW_FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
  final_msg_get_ts(&dw_rx_msg[DW_FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
  final_msg_get_ts(&dw_rx_msg[DW_FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

  // compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped
  poll_rx_ts_32 = (uint32_t)dw_poll_rx_ts;
  resp_tx_ts_32 = (uint32_t)dw_resp_tx_ts;
  final_rx_ts_32 = (uint32_t)dw_final_rx_ts;
  Ra = (double)(resp_rx_ts - poll_tx_ts);
  Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
  Da = (double)(final_tx_ts - resp_rx_ts);
  Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
  tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

  dw_tof = tof_dtu * DWT_TIME_UNITS;
  dw_distance = dw_tof * SPEED_OF_LIGHT;

  // range bias correction
  double biasAdjust_distance = dw_distance - dwt_getrangebias(config.chan, (float)dw_distance, config.prf);
  double distance_in_cm = biasAdjust_distance * 100;
  if (distance_in_cm < 0)
    distance_in_cm = 0;

  // store distance data into queue message
  DWM1000_StoreDistanceData(distance_in_cm);

  // fill sequence number into message
  dw_report_msg[2] = dw_frameNum;

  // fill TWR distance result into message
  dw_report_msg[12] = (uint16_t)distance_in_cm & 0xFF;
  dw_report_msg[13] = ((uint16_t)distance_in_cm >> 8) & 0xFF;

  // write frame data to DWM1000 and prepare transmission.
  dwt_writetxdata(sizeof(dw_report_msg), dw_report_msg, 0);
  dwt_writetxfctrl(sizeof(dw_report_msg), 0, 1);

  // sets the delay in turning the receiver on after a frame transmission has completed
  dwt_setrxaftertxdelay(DW_RESP_TX_TO_FINAL_RX_DLY_UUS);

  int ret = dwt_starttx(DWT_START_TX_IMMEDIATE);
  if (ret != DWT_SUCCESS)
  {
    SEGGER_SYSVIEW_Print("[Distance] - ANCHOR failed to Received Final Message");
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
  }

  dw_frameNum++;

  // wait for transmission complete
  while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
  { };

  // restart the listen mode
  DWM1000_StartListenMsg();
}

/**
  * @brief  Tag receive the distance measurement result from Anchor
  * @param  None
  * @retval None
  */
void DWM1000_TagReceiveReportMsg(void)
{
  uint16_t distance_value = (dw_rx_msg[13] << 8) +  dw_rx_msg[12];

  DWM1000_StoreDistanceData((double)distance_value);

  if (bEnableReceptionMode)
    DWM1000_StartListenMsg();
}

/**
  * @brief  Store distance data into Queue message
  * @param  distance:  Distance data in cm
  * @retval None
  */
void DWM1000_StoreDistanceData(double distance)
{
  uint32_t tickCount = HAL_GetTick();

  uint32_t deviceId = (dw_rx_msg[8] << 24) +  + (dw_rx_msg[7] << 16) + (dw_rx_msg[6] << 8) + dw_rx_msg[5];

  dw_eventlog[dw_evtlogcnt].deviceId = deviceId;
  dw_eventlog[dw_evtlogcnt].distance = distance;
  dw_eventlog[dw_evtlogcnt].timestamp = tickCount;
  dw_evtlogcnt++;
  if (dw_evtlogcnt == DISTANCE_MAX_SUPPORT_DEVICE)
    dw_evtlogcnt = 0;

  dw_devicelist[dw_devicelistcnt++] = deviceId;
  if (dw_devicelistcnt == DISTANCE_MAX_SUPPORT_DEVICE)
    dw_devicelistcnt = 0;
}

/**
  * @brief  Get the TX time-stamp in a 64-bit variable.
  *         Note: his function assumes that length of time-stamps is 40 bits, for both TX and RX!
  *
  * @param  None
  * @retval ts:  64-bit value of the read time-stamp
  */
uint64 get_tx_timestamp_u64(void)
{
  uint8_t ts_tab[5];
  uint64 ts = 0;

  dwt_readtxtimestamp(ts_tab);
  for (int i = 4; i >= 0; i--)
  {
      ts <<= 8;
      ts |= ts_tab[i];
  }
  return ts;
}

/**
  * @brief  Get the RX time-stamp in a 64-bit variable.
  *         Note: his function assumes that length of time-stamps is 40 bits, for both TX and RX!
  *
  * @param  None
  * @retval ts:  64-bit value of the read time-stamp
  */
uint64 get_rx_timestamp_u64(void)
{
  uint8_t ts_tab[5];
  uint64 ts = 0;

  dwt_readrxtimestamp(ts_tab);
  for (int i = 4; i >= 0; i--)
  {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}

/**
  * @brief  Read a given timestamp value from the final message. In the timestamp
  *         fields of the final message, the least significant byte is at the lower address.
  *
  * @param  ts_field:  pointer on the first byte of the timestamp field to read
  * @param  ts         timestamp value
  * @retval None.
  */
void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts)
{
  *ts = 0;
  for (int i = 0; i < DW_FINAL_MSG_TS_LEN; i++)
  {
    *ts += ts_field[i] << (i * 8);
  }
}

/**
  * @brief  Fill a given timestamp field in the final message with the given value.
  *         In the timestamp fields of the final message, the least significant byte
  *         is at the lower address.
  *
  * @param  ts_field:  pointer on the first byte of the timestamp field to fill
  * @param  ts         timestamp value
  * @retval None.
  */
void final_msg_set_ts(uint8_t *ts_field, uint64 ts)
{
  int i;
  for (i = 0; i < DW_FINAL_MSG_TS_LEN; i++)
  {
    ts_field[i] = (uint8_t) ts;
    ts >>= 8;
  }
}

/**
  * @brief  Reset the device filter list, it use for prevent duplicate
  *         message from same device within certain timeframe
  * @param  None
  * @retval None
  */
void DWM1000_ResetDeviceFilterList(void)
{
  memset(dw_devicelist, 0, sizeof(dw_devicelist));
  dw_devicelistcnt = 0;
}

/**
  * @brief  Reset the event log before first poll message execute
  * @param  None
  * @retval None
  */
void DWM1000_ResetEventLog(void)
{
  memset(dw_eventlog, 0, sizeof(dw_eventlog));
  dw_evtlogcnt = 0;
}

/**
  * @brief  DWM1000 module enter deep-sleep mode
  * @param  None
  * @retval None
  */
void DWM1000_EnterDeepSleep(void)
{
  SEGGER_SYSVIEW_Warn("[Distance] - Enter deep-sleep mode");

  HAL_TIM_Base_Stop_IT(&htim14);
  __HAL_RCC_TIM14_CLK_DISABLE();
  HAL_NVIC_DisableIRQ(TIM8_TRG_COM_TIM14_IRQn);
  dwt_entersleep();
}

 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
