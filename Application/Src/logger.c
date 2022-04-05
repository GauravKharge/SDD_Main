/**
  ******************************************************************************
  * @file    logger.c
  * @author  IBronx MDE team
  * @brief   logger header file to record operation events
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
#include "logger.h"
#include "fatfs.h"
#include "errorcode.h"
#include "flash_control.h"

#include <stdio.h>
#include <string.h>
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t loggerFileName;
char logger_filepath[LOGGER_PATH_LEN];
char logger_line_buf[LOGGER_STR_LEN];
char logger_string_buf[LOGGER_STR_LEN];

extern osEventFlagsId_t osFlags_Fatfs;
extern RTC_HandleTypeDef hrtc;
/* Private function prototypes -----------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/**
* @brief  Log the Info Event into log file
* @param  sMsg       Event log message
* @param  sArg       Input argument
  @retval None
*/
void logger_LogInfo(const char* sMsg, const char* sArg)
{
  memset(logger_string_buf, '\0', sizeof(logger_string_buf));
  if (strlen(sArg) > 0)
    sprintf(logger_string_buf, "%s: %s", sMsg, sArg);
  else
    sprintf(logger_string_buf, "%s", sMsg);

  logger_SaveLogEvents(LOGGER_TYPE_INFO, logger_string_buf);
  SEGGER_SYSVIEW_Print(logger_string_buf);
}

/**
* @brief  Log the Warn Event into log file
* @param  sMsg       Event log message
* @param  sArg       Input argument
  @retval None
*/
void logger_LogWarn(const char* sMsg, const char* sArg)
{
  memset(logger_string_buf, '\0', sizeof(logger_string_buf));
  if (strlen(sArg) > 0)
    sprintf(logger_string_buf, "%s: %s", sMsg, sArg);
  else
    sprintf(logger_string_buf, "%s", sMsg);

  logger_SaveLogEvents(LOGGER_TYPE_WARN, logger_string_buf);
  SEGGER_SYSVIEW_Warn(logger_string_buf);
}

/**
* @brief  Log the Error Event into log file
* @param  sMsg       Event log message
* @param  sArg       Input argument
  @retval None
*/
void logger_LogError(const char* sMsg, const char* sArg)
{
  memset(logger_string_buf, '\0', sizeof(logger_string_buf));
  if (strlen(sArg) > 0)
    sprintf(logger_string_buf, "%s: %s", sMsg, sArg);
  else
    sprintf(logger_string_buf, "%s", sMsg);

  logger_SaveLogEvents(LOGGER_TYPE_ERROR, logger_string_buf);
  SEGGER_SYSVIEW_Error(logger_string_buf);
}

/**
* @brief  Save log events into log file
* @param  sLogState  Event log state [Info, Warn, Error]
* @param  sMsg       Event log message
  @retval rc:        If pass then return PER_NO_ERROR, otherwise error code
*/
uint32_t logger_SaveLogEvents(const char* sLogState, const char* sMsg)
{
  uint32_t rc = PER_ERROR_SDCARD_FAILED_WRITE;

  RTC_DateTypeDef sDate = {0};
  RTC_TimeTypeDef sTime = {0};

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  memset(logger_line_buf, 0, sizeof(logger_line_buf));
  int size = sprintf(logger_line_buf, "%02d-%02d-%02d %02d:%02d:%02d %6s - %s.\n",
      sDate.Year + 2000, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds,
      sLogState, sMsg);

  //SEGGER_SYSVIEW_Print(logger_line_buf);

  // make sure SD CARD is presented & mounted before
  if ((osEventFlagsGet(osFlags_Fatfs) & FS_SD_PRESENT_FLAG) &&
      (osEventFlagsGet(osFlags_Fatfs) & FS_MOUNT_SDCARD_FLAG) &&
      (osEventFlagsGet(osFlags_Fatfs) & FS_CREATE_FOLDERS_FLAG))
  {
    // skip when log file still open by another process
    if (osEventFlagsGet(osFlags_Fatfs) & FS_OPEN_LOG_FILE_FLAG)
      return rc;

    sprintf(logger_filepath, "%s/%02d%02d%02d.LOG", LOGGER_LOG_DIR, sDate.Year + 2000, sDate.Month, sDate.Date);

    // create a file with read write access and open it
    if(f_open(&SDFile3, logger_filepath, FA_OPEN_APPEND | FA_WRITE) == FR_OK)
    {
      osEventFlagsSet(osFlags_Fatfs, FS_OPEN_LOG_FILE_FLAG);

      // write distance data into files
      uint32_t bw;
      FRESULT res = f_write(&SDFile3, logger_line_buf, size, (void *)&bw);
      if((bw > 0) && (res == FR_OK))
      {
        rc = PER_NO_ERROR;
      }
    }

    f_close(&SDFile3);
    osEventFlagsClear(osFlags_Fatfs, FS_OPEN_LOG_FILE_FLAG);
  }

  return rc;
}


/************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
