/**
  ******************************************************************************
  * @file    app_config.h
  * @author  IBronx MDE team
  * @brief   Application configuration header file
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

#ifndef __APP_CONFIG_H_
#define __APP_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

 /* Includes ------------------------------------------------------------------*/
 /* Exported types ------------------------------------------------------------*/
#define DISTANCE_DATA_WIFI_UPLOAD_THRESHOLD         1
#define DISTANCE_AWS_POST_API                       "http://ibronx.online:3000/contactEvent"

#define FATFS_NUM_DAYS_TO_DELETE                    60
#define TASK_MAIN_EXECUTION_SEC                     10
#define TASK_LOG_BATTERY_LEVEL_SEC                  60
#define DISTANCE_MAX_SUPPORT_DEVICE                 50

#define FIRMWARE_MAJOR_VERSION                      1
#define FIRMWARE_MINOR_VERSION                      0
#define FIRMWARE_PATCH_VERSION                      13

//Firmware History
//1.0.10 - Firmware Bug Fix for LED OFF time in HeartBeat Message - 03/07/2022 by Gaurav

 /* Exported constants --------------------------------------------------------*/
 /* Exported macro ------------------------------------------------------------*/
 /* Exported functions ------------------------------------------------------- */


#ifdef __cplusplus
}
#endif

#endif /* __APP_CONFIG_H_ */

 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
