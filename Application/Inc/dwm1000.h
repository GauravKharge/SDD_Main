/**
  ******************************************************************************
  * @file    dwm1000.h
  * @author  IBronx MDE team
  * @brief   Application program to control DWM1000 which is is an
  *          IEEE802.15.4-2011 UWB compliant wireless transceiver module
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

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __DWM1000_H_
#define __DWM1000_H_

 /* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include <stdbool.h>

 /* Exported types ------------------------------------------------------------*/

#define DW_TX_ANT_DLY                   16505   // Default antenna delay values for 64 MHz PRF.
#define DW_RX_ANT_DLY                   16505   // Default antenna delay values for 64 MHz PRF.
#define DW_PRE_TIMEOUT                  16      // Preamble timeout, in multiple of PAC size.
#define DW_RESP_TX_TO_FINAL_RX_DLY_UUS  50      // delay from the end of the frame transmission to the enable of the receiver
#define DW_POLL_TX_TO_RESP_RX_DLY_UUS   50      // delay from the end of the frame transmission to the enable of the receiver

#define DW_FRAME_LEN_MAX                127     // Buffer to store received frame
#define DW_FINAL_MSG_POLL_TX_TS_IDX     11
#define DW_FINAL_MSG_RESP_RX_TS_IDX     15
#define DW_FINAL_MSG_FINAL_TX_TS_IDX    19
#define DW_FINAL_MSG_TS_LEN             4

#define SPEED_OF_LIGHT                  299702547
#define DW_DATA_SIZE                    16

typedef void (*dw_isr_t)(void);
dw_isr_t dwm1000_exti_cb;

typedef enum
{
  DEVICE_TAG = 0,
  DEVICE_ANCHOR,
}dw_DeviceTypeDef;

typedef struct {
  uint32_t deviceId;
  uint32_t timestamp;
  double distance;
}dw_DataTypeDef;

 /* Exported constants --------------------------------------------------------*/
 /* Exported macro ------------------------------------------------------------*/
 /* Exported functions ------------------------------------------------------- */
uint32_t DWM1000_DeviceInit(void);
uint16_t DWM1000_GetNetworkId(void);
void DWM1000_SetMessageDevId(void);
void DWM1000_SetMessageNetId(void);
void DWM1000_StartBroadcastMsg(void);
void DWM1000_StartListenMsg(void);
void DWM1000_AnchorSendResponseMsg(void);
void DWM1000_AnchorSendOptionResponseMsg(void);
void DWM1000_TagSendPollMessage(bool bReceptionMode);
void DWM1000_TagSendFinalMsg(void);
void DWM1000_TagReceivedAnswerMsg(void);
void DWM1000_TagReceiveReportMsg(void);
void DWM1000_DistanceMeasureHandling(double dist_value);
void DWM1000_ResetDeviceFilterList(void);
void DWM1000_ResetEventLog(void);
void DWM1000_StoreDistanceData(double distance);
void DWM1000_EnterDeepSleep(void);
void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts);
void final_msg_set_ts(uint8_t *ts_field, uint64_t ts);
uint64_t get_tx_timestamp_u64(void);
uint64_t get_rx_timestamp_u64(void);

#ifdef __cplusplus
}
#endif

#endif /* __DWM1000_H_ */


 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
