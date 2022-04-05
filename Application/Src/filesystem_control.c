/**
  ******************************************************************************
  * @file    filesystem_control.c
  * @author  IBronx MDE team
  * @brief   Peripheral driver for filesystem read/write control
  *          This file provides firmware utility functions to support filesystem
  *          module functions
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
#include "filesystem_control.h"
#include "wifi_control.h"
#include "filesystem_task.h"
#include "fatfs.h"
#include "errorcode.h"
#include "distance_process.h"
#include "flash_control.h"
#include "dwm1000.h"
#include "app_config.h"
#include "main_task.h"
#include "logger.h"
#include "common.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
char fs_filepath[LOGGER_PATH_LEN];
char fs_filepath_temp[LOGGER_PATH_LEN];
char fs_line_buf[LOGGER_STR_LEN];
char fs_line_temp[LOGGER_STR_LEN];
char fs_distance_data[LOGGER_STR_LEN];

extern distance_Data distance_data[DISTANCE_DATA_MAX];
extern RTC_HandleTypeDef hrtc;
extern osEventFlagsId_t osFlags_Fatfs;
extern osEventFlagsId_t osFlags_Main;
extern osSemaphoreId_t osSmp_distance_count;
extern uint8_t dw_devId[FLASH_DEVICE_ID_SIZE];
extern uint8_t esp_mac_addr[ESP_MAC_ADDR_SIZE];

uint32_t total_distance_count;
/* Private function prototypes -----------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/**
  * @brief  Mount the SDCARD drive into file system
  * @param  None
  * @retval None
  */
void filesytem_MountSDCARD(void)
{
  FRESULT res;
  if (BSP_SD_IsDetected() == SD_PRESENT)
  {
    osEventFlagsSet(osFlags_Fatfs, FS_SD_PRESENT_FLAG);
    if ((res = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1)) == FR_OK)
    {
      osEventFlagsSet(osFlags_Fatfs, FS_MOUNT_SDCARD_FLAG);
      logger_LogInfo("[FileSystem] - Mount SDCARD Success", LOGGER_NULL_STRING);
      if (filesystem_CreateSubFolders() == PER_NO_ERROR)
      {
        osEventFlagsSet(osFlags_Fatfs, FS_CREATE_FOLDERS_FLAG);
        logger_LogInfo("[FileSystem] - Start Recording log from here", LOGGER_NULL_STRING);
      }
      else
      {
        osEventFlagsClear(osFlags_Fatfs, FS_CREATE_FOLDERS_FLAG);
      }
    }
    else
    {
      osEventFlagsClear(osFlags_Fatfs, FS_MOUNT_SDCARD_FLAG);
      logger_LogError("[FileSystem] - Failed to mount SDCARD", LOGGER_NULL_STRING);
    }
  }
  else
  {
    osEventFlagsClear(osFlags_Fatfs, FS_SD_PRESENT_FLAG);
    logger_LogError("[FileSystem] - SDCARD is not present", LOGGER_NULL_STRING);
  }
}

/**
  * @brief  Create sub folders for store log files & distance data files
  * @param  None
  * @retval rc:  return PER_NO_ERROR if no error, otherwise return rc
  */
uint32_t filesystem_CreateSubFolders(void)
{
  // create device logs file folder
  FRESULT res = f_mkdir(LOGGER_LOG_DIR);
  if (res == FR_OK || FR_EXIST)
  {
    // create distance data folder
    res = f_mkdir(FS_DATA_DIR);
    if (res == FR_OK || FR_EXIST)
    {
      // create distance data temporary folder
      res = f_mkdir(FS_TEMP_DIR);
      if (res == FR_OK || FR_EXIST)
      {
        return PER_NO_ERROR;
      }
    }
  }

  return PER_ERROR_SDCARD_CREATE_DIRECTORY;
}

/**
  * @brief  Store the distance data into files
  * @param  None
  * @retval rc:  return PER_NO_ERROR if no error, otherwise return rc
  */
uint32_t filesystem_ProcessDistanceData(void)
{
  uint32_t flags = osEventFlagsWait(osFlags_Main, MAIN_MCU_STANDBY_FLAG,
      osFlagsWaitAny | osFlagsNoClear, osWaitForever);

  if (flags > 0)
  {
    if (osEventFlagsGet(osFlags_Main) & MAIN_MCU_STANDBY_FLAG)
    {
      fatfs_ChangeCurrentState(STATE_FATFS_IDLE);
      osEventFlagsSet(osFlags_Main, MAIN_FATFS_STANDBY_FLAG);
    }
  }

  return PER_NO_ERROR;
}

/**
  * @brief  Store the distance data into files
  * @param  None
  * @retval rc:        return PER_NO_ERROR if no error, otherwise return rc
  */
uint32_t filesystem_SaveDistanceData(void)
{
  uint32_t rc = PER_NO_ERROR;

  if (osEventFlagsGet(osFlags_Fatfs) & (FS_MOUNT_SDCARD_FLAG | FS_CREATE_FOLDERS_FLAG))
  {
    // skip it if file is opening by another process.
    if (osEventFlagsGet(osFlags_Fatfs) & FS_OPEN_DATA_FILE_FLAG)
    {
      logger_LogError("[FileSystem] - Failed to write distance data into sd card", LOGGER_NULL_STRING);
      return PER_ERROR_SDCARD_FAILED_WRITE;
    }

    RTC_DateTypeDef sDate = {0};
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
    memset(fs_filepath, 0, sizeof(fs_filepath));
    sprintf(fs_filepath, "%s/%02d%02d%02d.TXT", FS_DATA_DIR, sDate.Year + 2000, sDate.Month, sDate.Date);

    // create a file with read write access and open it
    if(f_open(&SDFile, fs_filepath, FA_OPEN_APPEND | FA_WRITE) == FR_OK)
    {
      osEventFlagsSet(osFlags_Fatfs, FS_OPEN_DATA_FILE_FLAG);

      for(int idx = 0; idx < DISTANCE_DATA_MAX; idx++)
      {
        // skip if the distance data is empty or not yet settle down
        if ((distance_data[idx].deviceId == 0) ||
            (distance_data[idx].bSaveLog == false))
          continue;

        memset(fs_line_buf, 0, sizeof(fs_line_buf));

        // get distance status
        char distStatus = distance_data[idx].distance <= distance_GetRangeThreshold() ? 'C' : 'N';

        // read the company Id
        uint8_t pCompanyId[2];
        flash_read_CompanyId(pCompanyId);
        uint16_t companyId = pCompanyId[0] + (pCompanyId[1] << 8);

        // take last 4 digits of MAC Address - example: 7C:9E:BD:ED:79:A1 = 0x79A1
        uint16_t gatewayId = esp_mac_addr[0] + (esp_mac_addr[1] << 8);
        uint32_t sddId = (dw_devId[3] << 24) +  + (dw_devId[2] << 16) + (dw_devId[1] << 8) + dw_devId[0];

        int size = sprintf(fs_line_buf, "0,%li,%i,%i,%c,%i,%li,%ld,%i,%02d-%02d-%02d,%02d,%02d,%02d\n",
            sddId, distance_data[idx].batterylevel, distance_data[idx].distance, distStatus, gatewayId,
            distance_data[idx].deviceId, distance_data[idx].countPerSecond, companyId,
            distance_data[idx].sDate.Year + 2000, distance_data[idx].sDate.Month, distance_data[idx].sDate.Date,
            distance_data[idx].sTime.Hours, distance_data[idx].sTime.Minutes, distance_data[idx].sTime.Seconds);

        logger_LogInfo(fs_line_buf, LOGGER_NULL_STRING);

        // write distance data into files
        uint32_t bw;
        FRESULT res = f_write(&SDFile, fs_line_buf, size, (void *)&bw);
        if((bw > 0) && (res == FR_OK))
        {
          logger_LogInfo("[FileSystem] - Store Distance Data into file", fs_filepath);

          // erase the distance data
          memset(&distance_data[idx], 0, sizeof(distance_Data));
          rc = PER_NO_ERROR;

          // reduce a semaphore once written distance data into sd card
          osSemaphoreAcquire(osSmp_distance_count, 100U);

          total_distance_count++;
        }
      }
    }

    f_close(&SDFile);
    osEventFlagsClear(osFlags_Fatfs, FS_OPEN_DATA_FILE_FLAG);
  }

  return rc;
}

/**
  * @brief  Get total distance data count
  * @param  None
  * @retval total_distance_count
  */
uint32_t filesystem_GetDistanceCount(void)
{
  return total_distance_count;
}

/**
  * @brief  Get all the distance data count from SDCARD
  * @param  None
  * @retval rc:        return PER_NO_ERROR if no error, otherwise return rc
  */
uint32_t filesystem_CalculateDistanceData(void)
{
  total_distance_count = 0;

  if (osEventFlagsGet(osFlags_Fatfs) & (FS_MOUNT_SDCARD_FLAG | FS_CREATE_FOLDERS_FLAG))
  {
    // skip it if file is opening by another process.
    if (osEventFlagsGet(osFlags_Fatfs) & FS_OPEN_DATA_FILE_FLAG)
    {
      logger_LogError("[FileSystem] - Failed to get data count when file is opened", LOGGER_NULL_STRING);
      return PER_ERROR_FATFS_DUPLICATE_FILE_OPEN;
    }

    DIR dir;
    FRESULT res = f_opendir(&dir, FS_DATA_DIR);
    if (res == FR_OK)
    {
      for(;;)
      {
        FILINFO fno;
        res = f_readdir(&dir, &fno);

        // break on error or end of directory
        if (res != FR_OK || fno.fname[0] == 0)
          break;

        memset(fs_filepath, 0, sizeof(fs_filepath));
        strcpy(fs_filepath, FS_DATA_DIR);
        strcat(fs_filepath, "/");
        strcat(fs_filepath, fno.fname);

        uint32_t count = filesystem_GetDataCntSingleFile(fs_filepath);
        total_distance_count += count;
      }
    }
  }

  return PER_NO_ERROR;
}

/**
  * @brief  Get single file distance data count from SDCARD
  * @param  filepath:  File path
  * @retval rc:        return PER_NO_ERROR if no error, otherwise return rc
  */
uint32_t filesystem_GetDataCntSingleFile(char* filepath)
{
  uint32_t datacnt = 0;

  if(f_open(&SDFile, filepath, FA_READ) == FR_OK)
  {
    osEventFlagsSet(osFlags_Fatfs, FS_OPEN_DATA_FILE_FLAG);

    // count the number of distance info available in the file
    char ws[] = " \n\t\v\b\r\f\a";
    while (f_gets(fs_line_buf, LOGGER_STR_LEN*sizeof(char), &SDFile))
    {
      if (strspn(fs_line_buf, ws) == strlen(fs_line_buf))
        continue;

      datacnt++;
    }

    f_close(&SDFile);
    osEventFlagsClear(osFlags_Fatfs, FS_OPEN_DATA_FILE_FLAG);
  }
  else
  {
    logger_LogError("[FileSystem] - Error opening file", filepath);
  }

  f_close(&SDFile);

  return datacnt;
}

/**
  * @brief  Remove all files from a directory
  * @param  dirpath:  Directory path
  * @retval rc:  return PER_NO_ERROR if no error, otherwise return rc
  */
uint32_t filesystem_RemoveFiles(char* dirpath)
{
  uint32_t rc = PER_NO_ERROR;

  if (osEventFlagsGet(osFlags_Fatfs) & (FS_MOUNT_SDCARD_FLAG | FS_CREATE_FOLDERS_FLAG))
  {
    // skip it if file is opening by another process.
    if (osEventFlagsGet(osFlags_Fatfs) & FS_OPEN_DATA_FILE_FLAG)
      return PER_ERROR_FATFS_DELETE_FILES;

    DIR dir;
    FRESULT res = f_opendir(&dir, dirpath);
    if (res == FR_OK)
    {
      for(;;)
      {
        FILINFO fno;
        res = f_readdir(&dir, &fno);

        // break on error or end of directory
        if (res != FR_OK || fno.fname[0] == 0)
          break;

        memset(fs_filepath, 0, sizeof(fs_filepath));
        strcpy(fs_filepath, FS_DATA_DIR);
        strcat(fs_filepath, "/");
        strcat(fs_filepath, fno.fname);

        if (f_unlink(fs_filepath) != FR_OK)
        {
          logger_LogError("[FileSystem] - Error to delete file", fs_filepath);
          rc = PER_ERROR_FATFS_DELETE_FILES;
          break;
        }
        else
          logger_LogInfo("[FileSystem] - Deleted the file", fs_filepath);
      }
    }
    else
      rc = PER_ERROR_FATFS_DELETE_FILES;
  }

  return rc;
}

/**
  * @brief  Remove all the outdated files based on total given days
  * @param  numOfdays:  Number of days to delete the file
  * @retval rc:  return PER_NO_ERROR if no error, otherwise return rc
  */
uint32_t filesystem_RemoveOldFiles(uint32_t numOfdays)
{
  if (osEventFlagsGet(osFlags_Fatfs) & (FS_MOUNT_SDCARD_FLAG | FS_CREATE_FOLDERS_FLAG))
  {
    // skip it if file is opening by another process.
    if (osEventFlagsGet(osFlags_Fatfs) & FS_OPEN_DATA_FILE_FLAG)
      return PER_ERROR_FATFS_DELETE_FILES;

    DIR dir;
    FRESULT res = f_opendir(&dir, FS_DATA_DIR);
    if (res == FR_OK)
    {
      for(;;)
      {
        FILINFO fno;
        res = f_readdir(&dir, &fno);

        // break on error or end of directory
        if (res != FR_OK || fno.fname[0] == 0)
          break;

        memset(fs_filepath, 0, sizeof(fs_filepath));
        strcpy(fs_filepath, FS_DATA_DIR);
        strcat(fs_filepath, "/");
        strcat(fs_filepath, fno.fname);

        if (strlen(fno.fname) == 12)
        {
          char *pYear = pvPortMalloc(sizeof(char)*5);
          char *pMonth = pvPortMalloc(sizeof(char)*3);
          char *pDay = pvPortMalloc(sizeof(char)*3);
          memcpy(pYear, &fno.fname[0], 4);
          pYear[4] = 0;
          memcpy(pMonth, &fno.fname[4], 2);
          pMonth[2] = 0;
          memcpy(pDay, &fno.fname[6], 2);
          pDay[2] = 0;

          int year = atoi(pYear);
          int month = atoi(pMonth);
          int days = atoi(pDay);

          int numOfDays = filesystem_CalculateNumOfDays(year, month, days);
          if (numOfDays > 0 && numOfDays >= 60)
          {
            // delete the file after upload to cloud
            if (f_unlink(fs_filepath) != FR_OK)
              logger_LogError("[FileSystem] - Error to delete file", fs_filepath);
            else
              logger_LogInfo("[FileSystem] - Deleted the file", fs_filepath);
          }
          vPortFree(pYear);
          vPortFree(pMonth);
          vPortFree(pDay);
        }
      }
    }
  }
  return PER_NO_ERROR;
}

/**
  * @brief  Remove all the files under Temporary folder
  * @param  None
  * @retval rc:  return PER_NO_ERROR if no error, otherwise return rc
  */
uint32_t filesystem_RemoveTemporaryFiles(void)
{
  if (osEventFlagsGet(osFlags_Fatfs) & (FS_MOUNT_SDCARD_FLAG | FS_CREATE_FOLDERS_FLAG))
  {
    // skip it if file is opening by another process.
    if (osEventFlagsGet(osFlags_Fatfs) & FS_OPEN_TEMP_FILE_FLAG)
    {
      logger_LogError("[FileSystem] - Failed to remove temporary files when file is opened", LOGGER_NULL_STRING);
      return PER_ERROR_FATFS_DELETE_FILES;
    }

    DIR dir;
    FRESULT res = f_opendir(&dir, FS_TEMP_DIR);
    if (res == FR_OK)
    {
      logger_LogInfo("[FileSystem] - Remove all the files under temporary folder", FS_TEMP_DIR);

      for(;;)
      {
        FILINFO fno;
        res = f_readdir(&dir, &fno);

        // break on error or end of directory
        if (res != FR_OK || fno.fname[0] == 0)
          break;

        filesystem_CreateTemporaryFilePath(fno.fname);

        if (f_unlink(fs_filepath_temp) != FR_OK)
          logger_LogError("[FileSystem] - Error to delete file", fs_filepath_temp);
        else
          logger_LogInfo("[FileSystem] - Deleted the file", fno.fname);
      }
    }
  }

  return PER_NO_ERROR;
}

/**
  * @brief  Upload distance data from SDCARD to cloud,
  *         Once success to upload all the data, it will delete the file.
  *         Otherwise, it will store the record in xx.txt.
  * @param  None
  * @retval rc:  return PER_NO_ERROR if no error, otherwise return rc
  */
uint32_t filesystem_UploadDistanceData(void)
{
  uint32_t rc = PER_NO_ERROR;

  if (osEventFlagsGet(osFlags_Fatfs) & (FS_MOUNT_SDCARD_FLAG | FS_CREATE_FOLDERS_FLAG))
  {
    // skip it if file is opening by another process.
    if (osEventFlagsGet(osFlags_Fatfs) & FS_OPEN_DATA_FILE_FLAG)
    {
      logger_LogError("[FileSystem] - Failed to Upload distance data when file is opened", LOGGER_NULL_STRING);
      return PER_ERROR_FATFS_DUPLICATE_FILE_OPEN;
    }

    filesystem_RemoveTemporaryFiles();

    DIR dir;
    FRESULT res = f_opendir(&dir, FS_DATA_DIR);
    if (res == FR_OK)
    {
      for(;;)
      {
        FILINFO fno;
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == 0)   // break on error or end of directory
          break;

        logger_LogInfo("[FileSystem] - Upload distance data from file", fno.fname);

        filesystem_CreateFilePath(fno.fname);
        filesystem_CreateTemporaryFilePath(fno.fname);

        char ws[] = " \n\t\v\b\r\f\a";
        uint32_t upload_failed_cnt = 0;
        uint32_t upload_success_cnt = 0;
        uint32_t data_count = 0;
        uint32_t total_distance_cnt = filesystem_GetDataCntSingleFile(fs_filepath);

        // skip if no data available
        if (total_distance_cnt == 0)
          continue;

        if(f_open(&SDFile, fs_filepath, FA_READ) == FR_OK)
        {
          osEventFlagsSet(osFlags_Fatfs, FS_OPEN_DATA_FILE_FLAG);

          // create temporary file
          FRESULT res_temp = f_open(&SDFile2, fs_filepath_temp, FA_WRITE | FA_CREATE_ALWAYS);
          if (res_temp != FR_OK)
            logger_LogError("[FileSystem] - Unable to Open Temporary file", fs_filepath_temp);

          while (f_gets(fs_line_buf, LOGGER_STR_LEN*sizeof(char), &SDFile))
          {
            memcpy(fs_line_temp, fs_line_buf, sizeof(fs_line_temp));

            if (strspn(fs_line_buf, ws) == strlen(fs_line_buf))
              continue;

            // build the data string for upload API
            filesystem_BuildUploadDataString();
            //SEGGER_SYSVIEW_Print(fs_distance_data);

            // upload distance data to cloud, if failed more than limit then only copy distance data
            // to temporary file
            uint32_t rc = PER_NO_ERROR;
            if (upload_failed_cnt < FS_UPLOAD_DATA_FAIL_LIMIT)
            {
              rc = esp32_httpclient_PostRequest("0", DISTANCE_AWS_POST_API, "1", fs_distance_data,
                  ESP_CMD_LONG_DELAY_MS);
              data_count++;

              char buffer[20];
              if (rc != PER_NO_ERROR)
              {
                upload_failed_cnt++;
                logger_LogWarn("[FileSystem] - Failed to upload data", itoa(data_count, buffer, 10));
              }
              else
              {
                upload_success_cnt++;
                logger_LogInfo("[FileSystem] - Success to upload data", itoa(data_count, buffer, 10));

                if (total_distance_count > 0)
                  total_distance_count--;
              }
            }

            // store distance data into temporary file if failed to upload
            if (rc != PER_NO_ERROR)
            {
              UINT bw;
              if (res_temp == FR_OK)
              {
                res_temp = f_write(&SDFile2, fs_line_temp, strlen(fs_line_temp), &bw);
                if((bw > 0) && (res == FR_OK))
                {
                  logger_LogInfo("[FileSystem] - Store data into temporary file", LOGGER_NULL_STRING);
                }
              }
            }
          }

          // close the file and clear the open file flag
          f_close(&SDFile);
          f_close(&SDFile2);
          osEventFlagsClear(osFlags_Fatfs, FS_OPEN_DATA_FILE_FLAG);

          // check whether need to replace original file or not
          if ((upload_failed_cnt > 0) && (upload_success_cnt > 0))
          {
            // copy the temporary file to replace original file,
            if (f_unlink(fs_filepath) == FR_OK)
            {
              logger_LogInfo("[FileSystem] - Copy the temporary file to replace original file", LOGGER_NULL_STRING);
              f_rename(fs_filepath_temp, fs_filepath);
            }
          }
          else
          {
            // delete the temporary file
            if (f_unlink(fs_filepath_temp) == FR_OK)
            {
              logger_LogInfo("[FileSystem] - Delete the temporary file", fs_filepath_temp);
            }
          }

          // check whether upload all the data or not, if yes then delete the original file
          if (upload_success_cnt == total_distance_cnt)
          {
            // delete the temporary file
            if (f_unlink(fs_filepath) == FR_OK)
            {
              logger_LogInfo("[FileSystem] - Delete the original file when uploaded all data", fs_filepath);
            }
          }
        }
        else
        {
          logger_LogError("[FileSystem] - Error opening file", fs_filepath);
        }
      }
    }
    else
      rc = PER_ERROR_FATFS_UPLOAD_DATA;
  }

  return rc;
}

/**
  * @brief  Check the file creation date compare with current date, return number of days
  * @param  year:        Year
  * @param  month:       Month
  * @param  day:         Days
  * @retval days_delta:  Number of days delta
  */
int filesystem_CalculateNumOfDays(int year, int month, int day)
{
  RTC_DateTypeDef sDate = {0};
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  int days_delta = 0;

  // when RTC timestamp is not updated, do not delete the file.
  if ((sDate.Year + 2000) < year)
  {
   return 0;
  }

  int file_days = calculateCalendarDays(year, month, day);
  if ((sDate.Year + 2000) == year)
  {
   int current_days = calculateCalendarDays(sDate.Year + 2000, sDate.Month, sDate.Date);
   days_delta = current_days - file_days;
  }
  else
  {
   // when current timestamp more than file timestamp
   int year_delta = (sDate.Year + 2000) - year;
   if (year_delta > 0)
   {
     int current_days = calculateCalendarDays(sDate.Year + 2000, sDate.Month, sDate.Date);
     current_days += 365*year_delta;
     days_delta = current_days - file_days;
   }
  }

  return days_delta;
}

/**
  * @brief  Build the upload data string which use to upload API
  * @param  None
  * @retval None
  */
void filesystem_BuildUploadDataString(void)
{
  memset(fs_distance_data, 0, sizeof(fs_distance_data));

  // get the battery level
  uint16_t batterylevel = GetBatteryLevelInPct();
  uint16_t GatewayId = esp_mac_addr[0] + (esp_mac_addr[1] << 8);


  char string[20];
  int index = 0;
  char delim[] = ",";
  char *ptr = strtok(fs_line_buf, delim);
  while(ptr != NULL)
  {
    ptr = strtok(NULL, delim);
    switch(index)
    {
     case 0:
       strcpy(fs_distance_data, "SDDId=");
       strcat(fs_distance_data, ptr);
       break;
     case 1:
       strcat(fs_distance_data, "&BatteryLevel=");
       memset(string, 0, sizeof(string));
       strcat(fs_distance_data, itoa(batterylevel, string, 10));
       break;
     case 2:
       strcat(fs_distance_data, "&DistanceValue=");
       strcat(fs_distance_data, ptr);
       break;
     case 3:
       strcat(fs_distance_data, "&DistanceStatus=");
       strcat(fs_distance_data, ptr);
       break;
     case 4:
       strcat(fs_distance_data, "&GatewayID=");
       int value = atoi(ptr);
       if (value == 0)
       {
         memset(string, 0, sizeof(string));
         strcat(fs_distance_data, itoa(GatewayId, string, 10));
       }
       else
         strcat(fs_distance_data, ptr);
       break;
     case 5:
       strcat(fs_distance_data, "&ContactSDD=");
       strcat(fs_distance_data, ptr);
       break;
     case 6:
       strcat(fs_distance_data, "&Duration=");
       strcat(fs_distance_data, ptr);
       break;
     case 7:
       strcat(fs_distance_data, "&CompanyId=");
       strcat(fs_distance_data, ptr);
       break;
     case 8:
       strcat(fs_distance_data, "&Date=");
       strcat(fs_distance_data, ptr);
       break;
     case 9:
       strcat(fs_distance_data, "&Hour=");
       strcat(fs_distance_data, ptr);
       break;
     case 10:
       strcat(fs_distance_data, "&Minute=");
       strcat(fs_distance_data, ptr);
       break;
     case 11:
       strcat(fs_distance_data, "&Seconds=");
       strcat(fs_distance_data, ptr);
       break;
    }
    index++;
  }
}

/**
  * @brief  Create the file path
  * @param  filename:  File name
  * @retval None
  */
void filesystem_CreateFilePath(char* filename)
{
  memset(fs_filepath, 0, sizeof(fs_filepath));
  strcpy(fs_filepath, FS_DATA_DIR);
  strcat(fs_filepath, "/");
  strcat(fs_filepath, filename);
}

/**
  * @brief  Create the temporary file path
  * @param  filename:  File name
  * @retval None
  */
void filesystem_CreateTemporaryFilePath(char* filename)
{
  memset(fs_filepath_temp, 0, sizeof(fs_filepath_temp));
  strcpy(fs_filepath_temp, FS_TEMP_DIR);
  strcat(fs_filepath_temp, "/");
  strcat(fs_filepath_temp, filename);
}

/**
  * @brief  Calculate total days based on calendar date
  * @param  year:   Year
  * @param  month:  Month
  * @param  day:    Days
  * @retval totalDays:  Total days
  */
uint32_t calculateCalendarDays(int year, int month, int day)
{
  int MonthDay[12] = {31,28,31,30,31,30,31,31,30,31,30,31};

  if (((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0))
    MonthDay[1] = 29;

  uint32_t totaldays = 0;
  for (int mIdx = 0; mIdx < month; mIdx++)
  {
    totaldays += MonthDay[mIdx];
  }
  totaldays += day;

  return totaldays;
}



/************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
