/**
  ******************************************************************************
  * @file    filesystem_task.c
  * @author  IBronx MDE team
  * @brief   Filesystem Task program
  *          This file provides filesystem task functions to execute all the
  *          IO Read/Write operation to SD CARD module
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
#include "filesystem_task.h"
#include "cmsis_os.h"
#include "filesystem_control.h"
#include "app_config.h"
#include "logger.h"
#include "main_task.h"
#include "wifi_task.h"

#include <stdio.h>
#include <stdlib.h>

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
fatfsTask_State fatfsTaskState;

extern osSemaphoreId_t osSmp_fatfs_signal;
extern osSemaphoreId_t osSmp_wifi_signal;
extern osEventFlagsId_t osFlags_Main;
extern osEventFlagsId_t osFlags_Wifi;
/* Private function prototypes -----------------------------------------------*/
/* function prototypes -------------------------------------------------------*/


/**
* @brief Function implementing the FatfsTask thread.
* @param argument: Not used
* @retval None
*/
void fatfsTaskFunc(void *argument)
{
  fatfsTaskState = STATE_FATFS_INIT;

  for(;;)
  {
    fatfs_StateOperation();
  }
  osThreadTerminate(NULL);

}

/**
  * @brief  Running SDCard fatfs task operation based on current state
  * @param  None
  * @retval None
  */
void fatfs_StateOperation(void)
{
  switch(fatfsTaskState)
  {
    case STATE_FATFS_INIT:
      if (osSemaphoreAcquire(osSmp_fatfs_signal, 50U) == osOK)
      {
        logger_LogInfo("[FileSystem] - Task is ready to RUN", LOGGER_NULL_STRING);
        fatfsTaskState = STATE_FATFS_MOUNT;
      }
      break;
    case STATE_FATFS_IDLE:
      fatfs_task_Idle();
      osDelay(1000);
      break;
    case STATE_FATFS_MOUNT:
      fatfs_task_Mount();
      break;
    case STATE_FATFS_RUNNING:
      fatfs_task_running();
      break;
    case STATE_FATFS_REMOVE_FILE:
      fatfs_task_RemoveFiles();
      break;
    case STATE_FATFS_GET_DISTANCE_DATA:
      fatfs_task_GetDistanceData();
      break;
    case STATE_FATFS_SD_UNPLUGGED:
      osDelay(1000);
      break;
  }
}

/**
  * @brief  Run the fatfs task mount SDCARD State
  * @param  None
  * @retval None
  */
void fatfs_task_Mount(void)
{
  logger_LogInfo("[FileSystem] - Prepare to mount SDCARD", LOGGER_NULL_STRING);
  filesytem_MountSDCARD();
  fatfsTaskState = STATE_FATFS_REMOVE_FILE;
}

/**
  * @brief  Run the fatfs task store distance data into SDCARD State
  * @param  None
  * @retval None
  */
void fatfs_task_running(void)
{
  filesystem_ProcessDistanceData();
}

/**
  * @brief  Run the fatfs task remove older files from SDCARD
  * @param  None
  * @retval None
  */
void fatfs_task_RemoveFiles(void)
{
  logger_LogInfo("[FileSystem] - SDCARD is ready to do Housekeeping", LOGGER_NULL_STRING);
  filesystem_RemoveOldFiles(FATFS_NUM_DAYS_TO_DELETE);
  fatfsTaskState = STATE_FATFS_GET_DISTANCE_DATA;
}

/**
  * @brief  Get total distance data from SDCARD
  * @param  None
  * @retval None
  */
void fatfs_task_GetDistanceData(void)
{
  char buffer2[20];
  filesystem_CalculateDistanceData();
  uint32_t total_distance_cnt = filesystem_GetDistanceCount();
  if (total_distance_cnt >= DISTANCE_DATA_WIFI_UPLOAD_THRESHOLD)
  {
    osEventFlagsSet(osFlags_Wifi, WIFI_UPLOAD_DATA_SDCARD);
    logger_LogInfo("[FileSystem] - Trigger WIFI to upload data from SDCARD with data count", itoa(total_distance_cnt, buffer2, 10));
  }
  else
  {
    logger_LogInfo("[FileSystem] - Total distance data count", itoa(total_distance_cnt, buffer2, 10));
  }

  fatfsTaskState = STATE_FATFS_RUNNING;
}

/**
  * @brief  Change the current fatfs task state to wake up from standby mode
  * @param  None
  * @retval None
  */
void fatfs_task_Idle(void)
{
  // wait until MCU wake up
  if (!(osEventFlagsGet(osFlags_Main) & MAIN_MCU_STANDBY_FLAG))
  {
    logger_LogInfo("[FileSystem] - Trigger FileSystem module wake up from standby mode", LOGGER_NULL_STRING);
    fatfsTaskState = STATE_FATFS_RUNNING;
  }
}

/**
  * @brief  Change the current fatfs task state,
  *         so that it can proceed to next state
  * @param  state:  fatfs current task state
  * @retval None
  */
void fatfs_ChangeCurrentState(fatfsTask_State state)
{
  fatfsTaskState = state;
}

/**
  * @brief  Get the current fatfs task state
  * @param  None
  * @retval State:  fatfs current task state
  */
fatfsTask_State fatfs_GetCurrentState(void)
{
  return fatfsTaskState;
}


 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
