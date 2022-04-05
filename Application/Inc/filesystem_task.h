/**
  ******************************************************************************
  * @file    filesystem_task.h
  * @author  IBronx MDE team
  * @brief   Filesystem task header file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FILESYSTEM_TASK_H_
#define __FILESYSTEM_TASK_H_

#ifdef __cplusplus
 extern "C" {
#endif

 /* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

 /* Exported types ------------------------------------------------------------*/
 typedef enum
 {
   STATE_FATFS_INIT = 0,
   STATE_FATFS_IDLE,
   STATE_FATFS_MOUNT,
   STATE_FATFS_RUNNING,
   STATE_FATFS_REMOVE_FILE,
   STATE_FATFS_GET_DISTANCE_DATA,
   STATE_FATFS_SD_UNPLUGGED,
 }fatfsTask_State;

 /* Exported constants --------------------------------------------------------*/
 /* Exported macro ------------------------------------------------------------*/
 /* Exported functions ------------------------------------------------------- */
 void fatfsTaskFunc(void *argument);
 void fatfs_StateOperation(void);
 void fatfs_ChangeCurrentState(fatfsTask_State state);
 fatfsTask_State fatfs_GetCurrentState(void);

 void fatfs_task_Mount(void);
 void fatfs_task_running(void);
 void fatfs_task_RemoveFiles(void);
 void fatfs_task_Idle(void);
 void fatfs_task_GetDistanceData(void);

#ifdef __cplusplus
}
#endif

#endif /* __FILESYSTEM_TASK_H_ */


 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
