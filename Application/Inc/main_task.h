/**
  ******************************************************************************
  * @file    main_task.h
  * @author  IBronx MDE team
  * @brief   Main application task header file
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
#ifndef __MAIN_TASK_H_
#define __MAIN_TASK_H_

#ifdef __cplusplus
 extern "C" {
#endif

 /* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

 /* Exported types ------------------------------------------------------------*/
#define TASK_MAIN_DELAY_MS          1000

#define MAIN_MCU_STANDBY_FLAG       0x00000001U
#define MAIN_WIFI_STANDBY_FLAG      0x00000002U
#define MAIN_DWM1000_STANDBY_FLAG   0x00000004U
#define MAIN_FATFS_STANDBY_FLAG     0x00000008U
#define MAIN_USBD_CDC_READY_FLAG    0x00000010U
#define MAIN_BATTERY_LOW_FLAG       0x00000020U
#define MAIN_BATTERY_CHARGE_FLAG    0x00000040U
#define MAIN_BATTERY_FULL_FLAG      0x00000080U
#define MAIN_DISTANCE_DATA_FLAG     0x00000100U

 typedef enum
 {
   STATE_MAIN_INIT = 0,
   STATE_MAIN_PREPARATION,
   STATE_MAIN_START_WIFI,
   STATE_MAIN_START_DISTANCE,
   STATE_MAIN_RUNNING,
   STATE_MAIN_STOP_OPERATIONS,
   STATE_MAIN_STANDBY,
   STATE_MAIN_LOW_BATTERY,
   STATE_MAIN_START_IDLE,
 }MainTask_State;

 /* Exported constants --------------------------------------------------------*/
 /* Exported macro ------------------------------------------------------------*/
 /* Exported functions ------------------------------------------------------- */
 void MainTaskFunc(void *argument);

 void main_task_Init(void);
 void main_task_Preparation(void);
 void main_task_StartFATFSOperation(void);
 void main_task_StartDistanceOperation(void);
 void main_task_Running(void);
 void main_task_StopOperations(void);
 void main_task_EnterStandbyMode(void);
 void main_BatteryMonitorTask(uint32_t count_per_seconds);
 void main_task_LowBatteryMode(void);
 void WakeUp_EXTIHandler(uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_TASK_H_ */


 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
