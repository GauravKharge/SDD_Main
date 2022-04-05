/**
  ******************************************************************************
  * @file    distance_task.h
  * @author  IBronx MDE team
  * @brief   Distance measurement task header file
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
#ifndef __DISTANCE_TASK_H_
#define __DISTANCE_TASK_H_

#ifdef __cplusplus
 extern "C" {
#endif

 /* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

 /* Exported types ------------------------------------------------------------*/
#define TASK_DISTANCE_DELAY_MS           2000U

 typedef enum
 {
   STATE_DISTANCE_INIT = 0,
   STATE_DISTANCE_IDLE,
   STATE_DISTANCE_START,
   STATE_DISTANCE_STOP,
   STATE_DISTANCE_PROCESS,
   STATE_DISTANCE_STANDBY,
 }DistanceTask_State;

 /* Exported constants --------------------------------------------------------*/
 /* Exported macro ------------------------------------------------------------*/
 /* Exported functions ------------------------------------------------------- */
 void DistanceTaskFunc(void *argument);
 void distance_StateOperation(void);

 void distance_task_Init(void);
 void distance_task_StartOperation(void);
 void distance_task_StopOperation(void);
 void distance_task_ProcessData(void);
 void distance_task_StandbyOperation(void);
 void distance_ChangeCurrentState(DistanceTask_State state);
 DistanceTask_State distance_GetCurrentState(void);
 void distance_TimerCallbackHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __DISTANCE_TASK_H_ */


 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
