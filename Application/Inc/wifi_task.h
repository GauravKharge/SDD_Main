/**
  ******************************************************************************
  * @file    wifi_task.h
  * @author  IBronx MDE team
  * @brief   Wifi task header file
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
#ifndef __WIFI_TASK_H_
#define __WIFI_TASK_H_

#ifdef __cplusplus
 extern "C" {
#endif

 /* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

 /* Exported types ------------------------------------------------------------*/

#define WIFI_INIT_FLAG                0x00000001U
#define WIFI_CONNECT_FLAG             0x00000002U
#define WIFI_UPDATE_RTC_FLAG          0x00000004U
#define WIFI_DEEPSLEEP_FLAG           0x00000008U
#define WIFI_DEEPSLEEP_AGAIN_FLAG     0x00000010U
#define WIFI_WAKEUP_FLAG              0x00000020U
#define WIFI_UPLOAD_DATA_SDCARD       0x00000040U
#define WIFI_UPLOAD_DATA              0x00000080U
#define WIFI_WAKEUP_HARD_RESET        0x00000100U

#define WIFI_DEEP_SLEEP_INTERVAL_MS   30000

 typedef enum
 {
   STATE_WIFI_INIT = 0,
   STATE_WIFI_IDLE,
   STATE_WIFI_HARDWARE_RESET,
   STATE_WIFI_DEVICE_INIT,
   STATE_WIFI_DEVICE_START,
   STATE_WIFI_CONNECT,
   STATE_WIFI_DISCONNECT,
   STATE_WIFI_RUNNING,
   STATE_WIFI_DEEPSLEEP,
   STATE_WIFI_DEEPSLEEP_IDLE,

 }wifiTask_State;

 /* Exported constants --------------------------------------------------------*/
 /* Exported macro ------------------------------------------------------------*/
 /* Exported functions ------------------------------------------------------- */
 void WiFiTaskFunc(void *argument);
 void wifi_StateOperation(void);

 void wifi_ChangeCurrentState(wifiTask_State state);
 wifiTask_State wifi_GetCurrentState(void);

 void wifi_task_Init(void);
 void wifi_task_DeviceInit(void);
 void wifi_task_HardwareReset(void);
 void wifi_task_ConnectAccessPoint(void);
 void wifi_task_StartOperation(void);
 void wifi_task_EnterDeepSleep(void);
 void wifi_task_DeepSleepIdle(void);

#ifdef __cplusplus
}
#endif

#endif /* __WIFI_TASK_H_ */


 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
