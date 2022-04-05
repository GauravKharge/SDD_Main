/**
  ******************************************************************************
  * @file    distance_process.h
  * @author  IBronx MDE team
  * @brief   Distance process header file
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
#ifndef __DISTANCE_PROCESS_H_
#define __DISTANCE_PROCESS_H_

#ifdef __cplusplus
 extern "C" {
#endif

 /* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include <stdbool.h>
 /* Exported types ------------------------------------------------------------*/
#define DISTANCE_MAX_RANGE_CM               2000
#define DISTANCE_RANGE_CM                   200
#define DISTANCE_RESET_EVT_MS               2000
#define DISTANCE_TRIGGER_TIME_MS            1000
#define DISTANCE_MAX_TRIGGER_TIME_MS        10000
#define DISTANCE_LED_TIME_S                 10
#define DISTANCE_MAX_LED_TIME_S             60
#define DISTANCE_EVENT_MAX                  DISTANCE_MAX_SUPPORT_DEVICE
#define DISTANCE_DATA_MAX                   DISTANCE_MAX_SUPPORT_DEVICE

#define DISTANCE_LED_CRITICAL_FLAG          0x00000001U
#define DISTANCE_LED_MEDIUM_FLAG            0x00000002U
#define DISTANCE_LED_LOW_FLAG               0x00000004U
#define DISTANCE_BUZZER_FLAG                0x00000008U

 typedef enum
 {
   STATUS_DIST_NORMAL = 0,
   STATUS_DIST_NEAR,
   STATUS_DIST_CLOSE,
 }dist_DeviceStatus;

 typedef struct {
   uint32_t deviceId;
   uint32_t first_timestamp;
   uint32_t last_timestamp;
   uint16_t distance;
   dist_DeviceStatus status;
   uint16_t count;
 }distance_Event;

 typedef struct {
   RTC_TimeTypeDef sTime;
   RTC_DateTypeDef sDate;
   uint32_t timestamp;
   uint32_t deviceId;
   uint16_t distance;
   uint16_t batterylevel;
   uint16_t networkId;
   uint32_t countPerSecond;
   bool bSaveLog;
   char locationId[15];
 }distance_Data;
 /* Exported constants --------------------------------------------------------*/
 /* Exported macro ------------------------------------------------------------*/
 /* Exported functions ------------------------------------------------------- */

 void distance_process_Init(void);
 void distance_process_StartOperation(void);
 void distance_process_StopOperation(void);
 void distance_process_Data(uint32_t deviceId, uint32_t timestamp, double distance);
 void distance_process_UnderNormalContact(uint16_t idx, uint16_t distance, uint32_t timestamp);
 void distance_process_UnderCloseContact(uint16_t idx, uint16_t distance, uint32_t timestamp);
 void distance_process_CheckStatus(void);
 void distance_process_RGBLED(void);

 uint8_t distance_event_GetSameIndex(uint32_t device_id);
 void distance_event_CreateNewEntry(uint32_t deviceId, uint16_t distance, uint32_t timestamp);

 void distance_data_IncreaseCntPerSec(uint32_t deviceId,  uint32_t timeStamp);
 void distance_data_StoreLog(uint32_t deviceId, uint16_t distance, uint32_t timestamp);
 uint16_t distance_data_GetDataIndex(void);
 uint16_t distance_data_GetRemainingCount(void);
 uint16_t distance_GetRangeThreshold(void);
 uint16_t distance_GetEventTriggerTime(void);
 uint16_t distance_GetLEDBlinktime(void);
 void distance_data_SetStoreFlag(uint32_t deviceId);

#ifdef __cplusplus
}
#endif

#endif /* __DISTANCE_PROCESS_H_ */


 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
