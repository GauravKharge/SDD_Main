/**
  ******************************************************************************
  * @file    buzzer_task.h
  * @author  IBronx MDE team
  * @brief   Buzzer task header file
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
#ifndef __BUZZER_TASK_H_
#define __BUZZER_TASK_H_

#ifdef __cplusplus
 extern "C" {
#endif

 /* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

 /* Exported types ------------------------------------------------------------*/
 typedef enum
 {
   STATE_BUZZER_INIT = 0,
   STATE_BUZZER_ON,
   STATE_BUZZER_OFF,
 }BuzzerTask_State;

#define BUZZER_START_FLAG              0x00000001U
#define BUZZER_PULSE_NON_STOP_FLAG     0x00000002U


 /* Exported constants --------------------------------------------------------*/
 /* Exported macro ------------------------------------------------------------*/
 /* Exported functions ------------------------------------------------------- */
 void BuzzerTaskFunc(void *argument);
 void buzzer_task_Init(void);
 void buzzer_task_PulseOn(void);
 void buzzer_task_PulseOff(void);
 void buzzer_WaitForDelay(uint32_t delay_ms);

 void buzzer_StateOperation(void);
 void buzzer_ChangeCurrentState(BuzzerTask_State state);
 BuzzerTask_State buzzer_GetCurrentState(void);
 void buzzer_SetConfiguration(uint32_t pulse_count, uint32_t pulse_on_ms, uint32_t pulse_off_ms);

#endif /* __BUZZER_TASK_H_ */


 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
