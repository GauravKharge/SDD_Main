/**
  ******************************************************************************
  * @file    distance_process.c
  * @author  IBronx MDE team
  * @brief   Peripheral driver for DWM1000 module control
  *          This file provides firmware functions to manage the distance events
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
#include "distance_process.h"
#include "dws1000_control.h"
#include "dwm1000.h"
#include "cmsis_os.h"
#include "led_control.h"
#include "app_config.h"
#include "deca_device_api.h"
#include "common.h"
#include "flash_control.h"
#include "logger.h"
#include "main_task.h"

#include <string.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
distance_Event distance_evt[DISTANCE_EVENT_MAX];
distance_Data distance_data[DISTANCE_DATA_MAX];

static volatile uint8_t distance_data_Idx;
static volatile uint8_t distance_event_Idx;
static volatile uint16_t distance_range_cm;

volatile uint16_t count = 0;

extern RTC_HandleTypeDef hrtc;
extern osEventFlagsId_t osFlags_Distance;
extern osSemaphoreId_t osSmp_distance_count;
extern osEventFlagsId_t osFlags_Main;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/**
* @brief  Distance events process Init
* @param  None
* @retval None
*/
void distance_process_Init(void)
{
  distance_data_Idx = 0;
  distance_event_Idx = 0;
  memset(distance_evt, 0, sizeof(distance_Event)*DISTANCE_EVENT_MAX);
  memset(distance_data, 0, sizeof(distance_Data)*DISTANCE_DATA_MAX);

  dwm1000_Init();
}

/**
* @brief  Trigger DWM1000 to start distance measurement
* @param  None
* @retval None
*/
void distance_process_StartOperation(void)
{
  DWM1000_DeviceInit();
}

/**
* @brief  Stop the DWM1000 distance measurement operation
* @param  None
* @retval None
*/
void distance_process_StopOperation(void)
{
  DWM1000_EnterDeepSleep();
}

/**
* @brief  Process distance data and store the result into distance_data buffer
* @param  deviceId:   Device Id
* @param  timestamp:  TimeStamp
* @param  distance:   Distance
* @retval None
*/
void distance_process_Data(uint32_t deviceId, uint32_t timestamp, double distance)
{
  uint8_t idx = distance_event_GetSameIndex(deviceId);

  distance_range_cm = distance_GetRangeThreshold();
  //SEGGER_SYSVIEW_PrintfHost("[Distance] -  Distance process data with DeviceId %d from EventId %d", deviceId, idx);

  if (idx < DISTANCE_EVENT_MAX)
  {
    switch(distance_evt[idx].status)
    {
      case STATUS_DIST_CLOSE:
      case STATUS_DIST_NEAR:
        distance_process_UnderCloseContact(idx, (uint16_t)distance, timestamp);
        break;
      case STATUS_DIST_NORMAL:
        distance_process_UnderNormalContact(idx, (uint16_t)distance, timestamp);
        break;
    }
  }
  else
  {
    // create new event record
    distance_event_CreateNewEntry(deviceId, (uint16_t)distance, timestamp);
  }
}

/**
* @brief  Do the status update with or without distance measurement process,
*         drop/remove the devices when no signal receive for certain amount of time
* @param  None
* @retval None
*/
void distance_process_CheckStatus(void)
{
  uint32_t tickCount = HAL_GetTick();

  for (int i = 0; i < distance_event_Idx; i++)
  {
   if (distance_evt[i].deviceId != 0)
   {
     if (distance_evt[i].status == STATUS_DIST_CLOSE)
     {
       if (osEventFlagsGet(osFlags_Main) & MAIN_MCU_STANDBY_FLAG)
       {
         // save the record into sd card before enter deep-sleep mode
         distance_data_SetStoreFlag(distance_evt[i].deviceId);
       }
       else
       {
         // save the record into sd card & change led display color
         int total_time = tickCount - distance_evt[i].last_timestamp;
         if (total_time >= DISTANCE_RESET_EVT_MS)
         {
           distance_evt[i].status = STATUS_DIST_NORMAL;
           distance_data_SetStoreFlag(distance_evt[i].deviceId);
         }
       }
     }
     else if (distance_evt[i].status == STATUS_DIST_NEAR)
     {
       // change led display color
       int total_time = tickCount - distance_evt[i].last_timestamp;
       if (total_time >= DISTANCE_RESET_EVT_MS)
       {
         distance_evt[i].status = STATUS_DIST_NORMAL;
       }
     }
   }
  }
}

/**
  * @brief  Display the RGB LED based on distance status results
  * @param  None
  * @retval None
  */
void distance_process_RGBLED(void)
{
  uint32_t flags = (osEventFlagsGet(osFlags_Distance) &
      (DISTANCE_LED_CRITICAL_FLAG | DISTANCE_LED_MEDIUM_FLAG | DISTANCE_LED_LOW_FLAG));

  osEventFlagsClear(osFlags_Distance, DISTANCE_LED_CRITICAL_FLAG | DISTANCE_LED_MEDIUM_FLAG |
      DISTANCE_LED_LOW_FLAG);

  uint8_t critical_count = 0;
  for (int i = 0; i < distance_event_Idx; i++)
  {
    if (distance_evt[i].status == STATUS_DIST_CLOSE)
    {
      osEventFlagsSet(osFlags_Distance, DISTANCE_LED_CRITICAL_FLAG);
      critical_count++;
    }
    else if (distance_evt[i].status == STATUS_DIST_NEAR &&
        (!(osEventFlagsGet(osFlags_Distance) & DISTANCE_LED_CRITICAL_FLAG)))
    {
      osEventFlagsSet(osFlags_Distance, DISTANCE_LED_MEDIUM_FLAG);
    }
    else if ((distance_evt[i].status == STATUS_DIST_NORMAL)               &&
        (!(osEventFlagsGet(osFlags_Distance) & DISTANCE_LED_CRITICAL_FLAG)) &&
        (!(osEventFlagsGet(osFlags_Distance) & DISTANCE_LED_MEDIUM_FLAG)))
    {
      osEventFlagsSet(osFlags_Distance, DISTANCE_LED_LOW_FLAG);
    }
  }

  uint32_t currentflags = (osEventFlagsGet(osFlags_Distance) &
      (DISTANCE_LED_CRITICAL_FLAG | DISTANCE_LED_MEDIUM_FLAG | DISTANCE_LED_LOW_FLAG));

  // if remain the same LED then exit it.
  if (currentflags == flags)
  {
	count++;
    if (osEventFlagsGet(osFlags_Distance) & DISTANCE_LED_CRITICAL_FLAG)
    {
      rgbled_DistanceMeasurement(critical_count, DISTANCE_LED_CRITICAL_FLAG);
    }
    else if (osEventFlagsGet(osFlags_Distance) & DISTANCE_LED_MEDIUM_FLAG)
    {
      rgbled_DistanceMeasurement(critical_count, DISTANCE_LED_MEDIUM_FLAG);
    }
  }
  else
  {
    if (osEventFlagsGet(osFlags_Distance) & DISTANCE_LED_CRITICAL_FLAG)
    {
      rgbled_DistanceMeasurement(critical_count, DISTANCE_LED_CRITICAL_FLAG);
    }
    else if (osEventFlagsGet(osFlags_Distance) & DISTANCE_LED_MEDIUM_FLAG)
    {
      rgbled_DistanceMeasurement(critical_count, DISTANCE_LED_MEDIUM_FLAG);
    }
    else if (osEventFlagsGet(osFlags_Distance) & DISTANCE_LED_LOW_FLAG)
    {
      rgbled_DistanceMeasurement(critical_count, DISTANCE_LED_LOW_FLAG);
    }
  }
}

/**
* @brief Do the distance measurement task when range is under close contact
* @param  idx:        Distance Event Log index index
* @param  distance:   Distance
* @param  timestamp:  Timestamp
* @retval None
*/
void distance_process_UnderCloseContact(uint16_t idx, uint16_t distance, uint32_t timestamp)
{
  if (distance_evt[idx].status == STATUS_DIST_CLOSE)
  {
    if (distance >= distance_range_cm)
    {
      int total_timestamp = timestamp - distance_evt[idx].last_timestamp;
      if (total_timestamp >= DISTANCE_RESET_EVT_MS)
      {
        distance_process_UnderNormalContact(idx, distance, timestamp);
        distance_data_SetStoreFlag(distance_evt[idx].deviceId);
      }
    }
    else
    {
      distance_evt[idx].last_timestamp = timestamp;
      distance_data_IncreaseCntPerSec(distance_evt[idx].deviceId, distance_evt[idx].last_timestamp);

      // store the distance data before MCU enter deep-sleep mode
      if (osEventFlagsGet(osFlags_Main) & MAIN_MCU_STANDBY_FLAG)
      {
        logger_LogInfo("[Distance] - Store distance data before enter deep-sleep mode", LOGGER_NULL_STRING);
        distance_data_SetStoreFlag(distance_evt[idx].deviceId);
      }
    }
  }
  else
  {
    if (distance < distance_range_cm)
    {
      if (distance_evt[idx].count >= (distance_GetEventTriggerTime() / 1000))
      {
        distance_evt[idx].last_timestamp = timestamp;
        distance_evt[idx].distance /= distance_evt[idx].count;
        distance_evt[idx].status = STATUS_DIST_CLOSE;

        distance_data_StoreLog(distance_evt[idx].deviceId, distance_evt[idx].distance,
            distance_evt[idx].last_timestamp);
      }
      else
      {
        distance_evt[idx].last_timestamp = timestamp;
        distance_evt[idx].distance += distance;
        distance_evt[idx].count++;
        distance_evt[idx].status = STATUS_DIST_NEAR;
      }
    }
    else
    {
      distance_process_UnderNormalContact(idx, distance, timestamp);
    }
  }
}

/**
* @brief  Do the distance measurement task when range is under normal contact
* @param  idx:        Distance Event Log index index
* @param  distance:   Distance
* @param  timestamp:  Timestamp
* @retval None
*/
void distance_process_UnderNormalContact(uint16_t idx, uint16_t distance, uint32_t timestamp)
{
  if (distance < distance_range_cm)
  {
    distance_evt[idx].first_timestamp = timestamp;
    distance_evt[idx].last_timestamp = timestamp;
    distance_evt[idx].distance = distance;
    distance_evt[idx].count = 1;
    distance_evt[idx].status = STATUS_DIST_NEAR;
  }
  else
  {
    distance_evt[idx].last_timestamp = timestamp;
    distance_evt[idx].distance = distance;
    distance_evt[idx].status = STATUS_DIST_NORMAL;
    distance_evt[idx].count = 1;
  }
}

/**
  * @brief  Get the index Id from distance event log based on device Id
  * @param  device_id:  Device Id
  * @retval id:         Distance event log Index
  */
uint8_t distance_event_GetSameIndex(uint32_t device_id)
{
  uint8_t id = DISTANCE_EVENT_MAX;

  for (int i = 0; i < DISTANCE_EVENT_MAX; i++)
  {
    if (distance_evt[i].deviceId == device_id)
    {
      return i;
    }
  }

  return id;
}

/**
  * @brief  Create a new entry to fill up the distance info
  * @param  deviceId:   Devie Id
  * @param  distance:   Distance value
  * @param  timestamp:  TimeStamp info
  * @retval None
  */
void distance_event_CreateNewEntry(uint32_t deviceId, uint16_t distance, uint32_t timestamp)
{
  if (distance_event_Idx == (DISTANCE_EVENT_MAX - 1))
    distance_event_Idx = 0;

  uint16_t i = distance_event_Idx;

  if (i < DISTANCE_EVENT_MAX)
  {
    distance_evt[i].deviceId = deviceId;
    distance_evt[i].first_timestamp = timestamp;
    distance_evt[i].last_timestamp = timestamp;
    distance_evt[i].distance = distance;
    distance_evt[i].count = 1;

    if (distance < distance_range_cm)
      distance_evt[i].status = STATUS_DIST_NEAR;
    else
      distance_evt[i].status = STATUS_DIST_NORMAL;

    SEGGER_SYSVIEW_PrintfHost("[Distance] -  Create new event record with DeviceId %d, EventId %d", deviceId, i);

    distance_event_Idx++;
  }
  else
  {
    logger_LogError("[Distance] - Distance events is full", NULL);
  }
}

/**
  * @brief  Increase the total countPerSecond when the device still within
  *         two meters zone and timeStamp delta is not more than 2s.
  * @param  deviceId:   Device Id
  * @param  timeStamp:  TimeStamp
  * @retval None
  */
void distance_data_IncreaseCntPerSec(uint32_t deviceId,  uint32_t timeStamp)
{
  for(int idx = 0; idx < distance_data_Idx; idx++)
  {
    if (distance_data[idx].deviceId == deviceId)
    {
      int timeStamp_delta = timeStamp - distance_data[idx].timestamp;
      if (timeStamp_delta <= DISTANCE_RESET_EVT_MS && timeStamp_delta >= 0)
      {
        distance_data[idx].countPerSecond++;
        distance_data[idx].timestamp = timeStamp;
      }
    }
  }
}

/**
  * @brief  Store the Distance data sample into distance_data buffer
  * @param  deviceId:   Device Id
  * @param  distance:   Distance value
  * @param  timestamp:  Last detection timeStamp
  * @retval None
  */
void distance_data_StoreLog(uint32_t deviceId, uint16_t distance, uint32_t timestamp)
{
  if (distance_data_Idx == (DISTANCE_DATA_MAX - 1))
    distance_data_Idx = 0;

  SEGGER_SYSVIEW_PrintfHost("[Distance] - Logged distance data, %d, %dcm, %d", deviceId, distance, timestamp);

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  distance_data[distance_data_Idx].deviceId = deviceId;
  distance_data[distance_data_Idx].distance = distance;
  distance_data[distance_data_Idx].batterylevel = GetBatteryLevelInPct();
  distance_data[distance_data_Idx].sDate = sDate;
  distance_data[distance_data_Idx].sTime = sTime;
  distance_data[distance_data_Idx].countPerSecond = 5;
  distance_data[distance_data_Idx].timestamp = timestamp;
  distance_data[distance_data_Idx].networkId = DWM1000_GetNetworkId();
  strcpy(distance_data[distance_data_Idx].locationId, "my-01-1000");
  distance_data[distance_data_Idx].bSaveLog = false;

  distance_data_Idx++;
}

/**
  * @brief  Get distance data count Index
  * @param  None
  * @retval distance_data_Idx:  Number of distance data count
  */
void distance_data_SetStoreFlag(uint32_t deviceId)
{
  for(int idx = 0; idx < distance_data_Idx; idx++)
  {
    if (distance_data[idx].deviceId == deviceId)
    {
      distance_data[idx].bSaveLog = true;

      // increase semaphore by one
      osSemaphoreRelease(osSmp_distance_count);
    }
  }
}


/**
  * @brief  Get distance data count Index
  * @param  None
  * @retval distance_data_Idx:  Number of distance data count
  */
uint16_t distance_data_GetDataIndex(void)
{
  return distance_data_Idx;
}

/**
  * @brief  Get the remaining distance data count after upload or save into SD CARD
  *         Note: When distance data uploaded to cloud or saved into SD CARD, it will be
  *         delete the entry
  * @param  None
  * @retval count:  Number of distance data count
  */
uint16_t distance_data_GetRemainingCount(void)
{
  // calculate how many distance events left
  uint16_t count = 0;
  for(int idx = 0; idx < distance_data_Idx; idx++)
  {
    // skip if the distance events is empty
    if (distance_data[idx].deviceId == 0)
      continue;

    count++;
  }

  return count;
}

/**
  * @brief  Get the distance range threshold from flash memory, units in centimeters
  * @param  None
  * @retval distance_range_cm:  distance range threshold
  */
uint16_t distance_GetRangeThreshold(void)
{
  uint8_t pData[2];
  flash_read_DistanceThreshold(pData);

  uint16_t distance_range_cm = pData[0] + (pData[1] << 8);
  if (distance_range_cm == 0 || distance_range_cm >= DISTANCE_MAX_RANGE_CM)
    distance_range_cm = DISTANCE_RANGE_CM;

  return distance_range_cm;
}

/**
  * @brief  Get the distance close contact event trigger time from flash memory,
  *         units in milliseconds
  * @param  None
  * @retval triggerEventTime:  distance event trigger time
  */
uint16_t distance_GetEventTriggerTime(void)
{
  uint8_t pData[2];
  flash_read_DistanceTriggerTime(pData);

  uint16_t triggerEventTime = pData[0] + (pData[1] << 8);
  if (triggerEventTime < DISTANCE_TRIGGER_TIME_MS || triggerEventTime >= DISTANCE_MAX_TRIGGER_TIME_MS)
    triggerEventTime = DISTANCE_TRIGGER_TIME_MS;

  return triggerEventTime;
}

/**
  * @brief  Get the LED blinking time when no close contact event happened from flash memory,
  *         units in seconds
  * @param  None
  * @retval triggerEventTime:  distance event trigger time
  */
uint16_t distance_GetLEDBlinktime(void)
{
  uint8_t pData[2];
  flash_read_LEDBlinkTime(pData);

  uint16_t ledBlinktime = pData[0] + (pData[1] << 8);
  if (ledBlinktime == 0 || ledBlinktime >= DISTANCE_MAX_LED_TIME_S)
    ledBlinktime = DISTANCE_LED_TIME_S;

  return ledBlinktime;
}


/************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
