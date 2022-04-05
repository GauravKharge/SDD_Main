/**
  ******************************************************************************
  * @file    distance_task.c
  * @author  IBronx MDE team
  * @brief   Distance Task program
  *          This file provides distance task functions to execute all the
  *          distance measurement via DWM1000 module
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
#include "distance_task.h"
#include "cmsis_os.h"
#include "dwm1000.h"
#include "distance_process.h"
#include "main_task.h"
#include "logger.h"
#include "app_config.h"


#include <stdio.h>

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
DistanceTask_State distanceTaskState;

extern osSemaphoreId_t osSmp_distance_signal;
extern osSemaphoreId_t osSmp_distance_count;
extern osMessageQueueId_t osMsgQ_distance;
extern osEventFlagsId_t osFlags_Main;
extern osEventFlagsId_t osFlags_Distance;

uint8_t distance_msg_cnt = 0;
extern uint8_t dw_evtlogcnt;
extern dw_DataTypeDef dw_eventlog[DISTANCE_MAX_SUPPORT_DEVICE];
/* Private function prototypes -----------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/**
* @brief Function implementing the DistanceTask thread.
* @param argument: Not used
* @retval None
*/
void DistanceTaskFunc(void *argument)
{
  uint32_t tick = osKernelGetTickCount();
  distanceTaskState = STATE_DISTANCE_INIT;
  for(;;)
  {
    tick += TASK_DISTANCE_DELAY_MS;
    distance_StateOperation();
  }
  osThreadTerminate(NULL);
}

/**
  * @brief  Running distance measurement task operation based on current state
  * @param  None
  * @retval None
  */
void distance_StateOperation(void)
{
  switch(distanceTaskState)
  {
    case STATE_DISTANCE_INIT:
      distance_task_Init();
      break;
    case STATE_DISTANCE_IDLE:
      if (osSemaphoreAcquire(osSmp_distance_signal, 50U) == osOK)
      {
        logger_LogInfo("[Distance] - Task is ready to RUN", LOGGER_NULL_STRING);
        distanceTaskState = STATE_DISTANCE_START;
      }
      break;
    case STATE_DISTANCE_START:
      distance_task_StartOperation();
      break;
    case STATE_DISTANCE_STOP:
      distance_task_StopOperation();
      osDelay(500);
      break;
    case STATE_DISTANCE_PROCESS:
      distance_task_ProcessData();
      break;
    case STATE_DISTANCE_STANDBY:
      distance_task_StandbyOperation();
      osDelay(1000);
      break;
  }
}

/**
  * @brief  Run the distance task Init State
  * @param  None
  * @retval None
  */
void distance_task_Init(void)
{
  distance_process_Init();
  distanceTaskState = STATE_DISTANCE_IDLE;
}

/**
  * @brief  Run the distance task Start Operation State
  * @param  None
  * @retval None
  */
void distance_task_StartOperation(void)
{
  logger_LogInfo("[Distance] - Start Distance Measurement Operation", LOGGER_NULL_STRING);
  distance_process_StartOperation();
  DWM1000_StartBroadcastMsg();
  distance_msg_cnt = 0;
  distanceTaskState = STATE_DISTANCE_PROCESS;
}

/**
  * @brief  Run the distance task Stop Operation State
  * @param  None
  * @retval None
  */
void distance_task_StopOperation(void)
{
  logger_LogInfo("[Distance] - Stop Distance Measurement Operation", LOGGER_NULL_STRING);

  // trigger event flags to main task for enter standby mode
  osEventFlagsSet(osFlags_Main, MAIN_DWM1000_STANDBY_FLAG);
  distanceTaskState = STATE_DISTANCE_STANDBY;
}

/**
  * @brief  Wait for MCU enter standby mode
  * @param  None
  * @retval None
  */
void distance_task_StandbyOperation(void)
{
  // wait until MCU wake up
  if (!(osEventFlagsGet(osFlags_Main) & MAIN_MCU_STANDBY_FLAG))
  {
    logger_LogInfo("[Distance] - Trigger DWM1000 module wake up from standby mode", LOGGER_NULL_STRING);
    distanceTaskState = STATE_DISTANCE_START;
  }
}

/**
  * @brief  Run the distance task process data State
  * @param  None
  * @retval None
  */
void distance_task_ProcessData(void)
{
  uint32_t flags = osEventFlagsWait(osFlags_Main, MAIN_DISTANCE_DATA_FLAG | MAIN_MCU_STANDBY_FLAG,
      osFlagsWaitAny | osFlagsNoClear, osWaitForever);

  if (flags > 0)
  {

    if ((osEventFlagsGet(osFlags_Main) & MAIN_DISTANCE_DATA_FLAG) ||
        (osEventFlagsGet(osFlags_Main) & MAIN_MCU_STANDBY_FLAG))
    {

      DWM1000_ResetEventLog();

      // process distance event first, then transmit the first poll message
      DWM1000_TagSendPollMessage(true);

      // check the device status
      distance_process_CheckStatus();

      // display LED
      distance_process_RGBLED();

      // save distance data into sd card
      if (osSemaphoreGetCount(osSmp_distance_count) > 0)
        filesystem_SaveDistanceData();

      osEventFlagsClear(osFlags_Main, MAIN_DISTANCE_DATA_FLAG);
    }

    if (osEventFlagsGet(osFlags_Main) & MAIN_MCU_STANDBY_FLAG)
    {
      logger_LogInfo("[Distance] - Trigger DWM1000 module enter DeepSleep mode", LOGGER_NULL_STRING);
      distanceTaskState = STATE_DISTANCE_STOP;
      distance_process_StopOperation();
    }
  }
}

/**
  * @brief  Change the current distance task state,
  *         so that it can proceed to next state
  * @param  state:  Distance current task state
  * @retval None
  */
void distance_ChangeCurrentState(DistanceTask_State state)
{
  distanceTaskState = state;
}

/**
  * @brief  Get the current distance task state
  * @param  None
  * @retval State:  Distance current task state
  */
DistanceTask_State distance_GetCurrentState(void)
{
  return distanceTaskState;
}

/**
  * @brief  Timer callback handler to configure DWM1000 transmit/Receive message
  *
  * @param  None
  * @retval None
  */
void distance_TimerCallbackHandler(void)
{
	SEGGER_SYSVIEW_Print("[Distance] - Starting TWR Distance Measurement nnnn -------------------------------");
  SEGGER_SYSVIEW_RecordEnterISR();

  if (distance_msg_cnt == 10)
    distance_msg_cnt = 0;

  switch(distance_msg_cnt)
  {
    case 0:
      SEGGER_SYSVIEW_Print("[Distance] - Starting TWR Distance Measurement -------------------------------");

      for (int i = 0; i < dw_evtlogcnt; i++)
      {
        SEGGER_SYSVIEW_PrintfHost("[Distance] -  Event Log for every seconds, %d, %dcm",
            dw_eventlog[i].deviceId, (int)dw_eventlog[i].distance);
        distance_process_Data(dw_eventlog[i].deviceId, dw_eventlog[i].timestamp, dw_eventlog[i].distance);
      }
      DWM1000_ResetDeviceFilterList();
      osEventFlagsSet(osFlags_Main, MAIN_DISTANCE_DATA_FLAG);
      break;
    case 1:
      break;
    case 2:
      DWM1000_TagSendPollMessage(true);
      break;
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
      DWM1000_TagSendPollMessage(false);
      break;
  }

  distance_msg_cnt++;

  SEGGER_SYSVIEW_RecordExitISR();
}


 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
