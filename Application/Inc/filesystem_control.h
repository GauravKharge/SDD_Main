/**
  ******************************************************************************
  * @file    filesystem_control.h
  * @author  IBronx MDE team
  * @brief   FileSystem controls header file
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
#ifndef __FILESYSTEM_CONTROL_H_
#define __FILESYSTEM_CONTROL_H_

#ifdef __cplusplus
 extern "C" {
#endif

 /* Includes ------------------------------------------------------------------*/
 #include "stm32f4xx_hal.h"

  /* Exported types ------------------------------------------------------------*/
#define LOGGER_LOG_DIR            "0:/LOG"
#define FS_DATA_DIR               "0:/DATA"
#define FS_TEMP_DIR               "0:/TEMP"
#define LOGGER_STR_LEN            300
#define LOGGER_PATH_LEN           30
#define FS_UPLOAD_STATUS_LEN      30
#define FS_UPLOAD_DATA_FAIL_LIMIT 20

#define FS_SD_PRESENT_FLAG        0x00000001U
#define FS_MOUNT_SDCARD_FLAG      0x00000002U
#define FS_CREATE_FOLDERS_FLAG    0x00000004U
#define FS_OPEN_DATA_FILE_FLAG    0x00000008U
#define FS_OPEN_TEMP_FILE_FLAG    0x00000010U
#define FS_OPEN_LOG_FILE_FLAG     0x00000020U
#define FS_STORE_DISTANCE_FLAG    0x00000040U



  /* Exported constants --------------------------------------------------------*/
  /* Exported macro ------------------------------------------------------------*/
  /* Exported functions ------------------------------------------------------- */
 void filesytem_MountSDCARD(void);
 uint32_t filesystem_CreateSubFolders(void);
 uint32_t filesystem_ProcessDistanceData(void);
 uint32_t filesystem_SaveDistanceData(void);
 uint32_t filesystem_GetDistanceCount(void);
 uint32_t filesystem_CalculateDistanceData(void);
 int filesystem_CalculateNumOfDays(int year, int month, int day);
 uint32_t calculateCalendarDays(int year, int month, int day);
 uint32_t filesystem_RemoveOldFiles(uint32_t numOfdays);
 uint32_t filesystem_GetDataCntSingleFile(char* filepath);
 uint32_t filesystem_UploadDistanceData(void);
 void filesystem_BuildUploadDataString(void);
 void filesystem_CreateFilePath(char* filename);
 void filesystem_CreateTemporaryFilePath(char* filename);
 uint32_t filesystem_RemoveTemporaryFiles(void);
 uint32_t filesystem_RemoveFiles(char* dirpath);

#ifdef __cplusplus
}
#endif

#endif /* __FILESYSTEM_CONTROL_H_ */


 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
