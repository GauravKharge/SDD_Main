/**
  ******************************************************************************
  * @file    buzzer_task.c
  * @author  IBronx MDE team
  * @brief   Buzzer Task program
  *          This file provides buzzer task functions to execute all the
  *          buzzer operations
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
#include "buzzer_task.h"
#include "cmsis_os.h"
#include "buzzer_control.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
BuzzerTask_State buzzerTaskState;

static volatile uint32_t buzzer_on_ms = 0;
static volatile uint32_t buzzer_off_ms = 0;
static volatile uint32_t buzzer_pulse_target_cnt = 0;
static volatile uint32_t buzzer_pulse_cnt = 0;

extern osEventFlagsId_t osFlags_Buzzer;

/* function prototypes -------------------------------------------------------*/
/**
* @brief Function implementing the BuzzerTask thread.
* @param argument: Not used
* @retval None
*/
void BuzzerTaskFunc(void *argument)
{
  buzzerTaskState = STATE_BUZZER_INIT;

  for(;;)
  {
    buzzer_StateOperation();
  }
  osThreadTerminate(NULL);
}

/**
  * @brief  Running status led task operation based on current state
  * @param  None
  * @retval None
  */
void buzzer_StateOperation(void)
{
  switch(buzzerTaskState)
  {
    case STATE_BUZZER_INIT:
      buzzer_task_Init();
      break;
    case STATE_BUZZER_ON:
      buzzer_task_PulseOn();
      break;
    case STATE_BUZZER_OFF:
      buzzer_task_PulseOff();
      break;
  }
}


/**
  * @brief  Buzzer task init state
  * @param  None
  * @retval None
  */
void buzzer_task_Init(void)
{
  uint32_t flags = osEventFlagsWait(osFlags_Buzzer, BUZZER_START_FLAG,
      osFlagsWaitAny | osFlagsNoClear, osWaitForever);

  if (flags > 0)
  {
    if (osEventFlagsGet(osFlags_Buzzer) & BUZZER_START_FLAG)
    {
      buzzer_pulse_cnt = 0;
      buzzerTaskState = STATE_BUZZER_ON;
      osEventFlagsClear(osFlags_Buzzer, BUZZER_START_FLAG);
    }
  }
}

/**
  * @brief  Buzzer task to execute buzzer on state
  * @param  None
  * @retval None
  */
void buzzer_task_PulseOn(void)
{
  buzzer_TurnOn();
  buzzerTaskState = STATE_BUZZER_OFF;
  osDelay(buzzer_on_ms);
}

/**
  * @brief  Buzzer task to execute buzzer off state
  * @param  None
  * @retval None
  */
void buzzer_task_PulseOff(void)
{
  buzzer_TurnOff();
  buzzer_pulse_cnt++;

  if (!(osEventFlagsGet(osFlags_Buzzer) & BUZZER_PULSE_NON_STOP_FLAG))
  {
    if (buzzer_pulse_cnt < buzzer_pulse_target_cnt)
    {
      buzzerTaskState = STATE_BUZZER_ON;
      buzzer_WaitForDelay(buzzer_off_ms);
    }
    else
    {
      buzzerTaskState = STATE_BUZZER_INIT;
      osDelay(300);
    }
  }
  else
  {
    buzzerTaskState = STATE_BUZZER_ON;
    buzzer_WaitForDelay(buzzer_off_ms);
  }
}

/**
  * @brief  wait for certain amount of time, if start flag is trigger then
  *         proceed to next state immediately
  * @param  delay_ms:  Total delay time in milliseconds
  * @retval None
  */
void buzzer_WaitForDelay(uint32_t delay_ms)
{
  uint32_t time_ms = 0;

  while( time_ms < delay_ms)
  {
    osDelay(100);
    time_ms += 100;

    if (osEventFlagsGet(osFlags_Buzzer) & BUZZER_START_FLAG)
    {
      buzzerTaskState = STATE_BUZZER_INIT;
      break;
    }
  }
}

/**
  * @brief  Change the current buzzer task state,
  *         so that it can proceed to next state
  * @param  state:  Buzzer task state
  * @retval None
  */
void buzzer_ChangeCurrentState(BuzzerTask_State state)
{
  buzzerTaskState = state;
}

/**
  * @brief  Get the current buzzer task state
  * @param  None
  * @retval State:  Buzzer current task state
  */
BuzzerTask_State buzzer_GetCurrentState(void)
{
  return buzzerTaskState;
}

/**
  * @brief  Change buzzer configuration
  * @param  pulse_count:   Number of buzzer pulse count
  * @param  pulse_on_ms:   Buzzer On interval in millisecond
  * @param  pulse_off_ms:  Buzzer Off interval in millisecond
  * @retval None
  */
void buzzer_SetConfiguration(uint32_t pulse_count, uint32_t pulse_on_ms, uint32_t pulse_off_ms)
{
  if (pulse_count == BUZZER_PULSE_NON_STOP)
    osEventFlagsSet(osFlags_Buzzer, BUZZER_PULSE_NON_STOP_FLAG);
  else
    osEventFlagsClear(osFlags_Buzzer, BUZZER_PULSE_NON_STOP_FLAG);

  buzzer_pulse_target_cnt = pulse_count;
  buzzer_on_ms = pulse_on_ms;
  buzzer_off_ms = pulse_off_ms;
}


 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
