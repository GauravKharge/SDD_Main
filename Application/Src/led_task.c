/**
  ******************************************************************************
  * @file    led_task.c
  * @author  IBronx MDE team
  * @brief   LED Task program
  *          This file provides LED task functions to display the status LED
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
#include "led_task.h"
#include "cmsis_os.h"
#include "led_control.h"
#include "buzzer_control.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
LEDTask_State ledTaskState;

//static volatile uint32_t led_blink_delay = 0;
static volatile uint32_t led_blink_on_ms = 0;
static volatile uint32_t led_blink_off_ms = 0;
static volatile uint32_t led_blink_target_cnt = 0;
static volatile uint32_t led_blink_cnt = 0;
static volatile uint32_t led_buzzer_pulse_cnt = 0;

extern osEventFlagsId_t osFlags_LED;
/* function prototypes -------------------------------------------------------*/

/**
* @brief Function implementing the StatusLED thread.
* @param argument: Not used
* @retval None
*/
void StatusLEDFunc(void *argument)
{
  ledTaskState = STATE_LED_INIT;

  for(;;)
  {
    statusLED_StateOperation();
  }
  osThreadTerminate(NULL);
}

/**
  * @brief  Running status led task operation based on current state
  * @param  None
  * @retval None
  */
void statusLED_StateOperation(void)
{
  switch(ledTaskState)
  {
    case STATE_LED_INIT:
      statusLED_Init();
      break;
    case STATE_LED_BLINK_ON:
      statusLED_Blink_On();
      break;
    case STATE_LED_BLINK_OFF:
      statusLED_Blink_Off();
      break;
  }
}

/**
  * @brief  Change the status led to init state
  * @param  None
  * @retval None
  */
void statusLED_Init(void)
{
  uint32_t flags = osEventFlagsWait(osFlags_LED, LED_START_FLAG,
      osFlagsWaitAny | osFlagsNoClear, osWaitForever);

  if (flags > 0)
  {
    if (osEventFlagsGet(osFlags_LED) & LED_START_FLAG)
    {
      led_blink_cnt = 0;
      ledTaskState = STATE_LED_BLINK_ON;
      osEventFlagsClear(osFlags_LED, LED_START_FLAG);
    }
  }
}

/**
  * @brief  Change the status led to blink on state
  * @param  None
  * @retval None
  */
void statusLED_Blink_On(void)
{
  rgbled_TurnOnLED();
  ledTaskState = STATE_LED_BLINK_OFF;

  if (osEventFlagsGet(osFlags_LED) & LED_ENABLE_BUZZER_FLAG)
  {
    led_buzzer_pulse_cnt++;

    if (led_buzzer_pulse_cnt > BUZZER_TRIGGER_TIME_IN_SEC)
    {
      buzzer_ExecBuzzer(1, 200, 200);
      led_buzzer_pulse_cnt = 0;
    }
  }
  else if(osEventFlagsGet(osFlags_LED) & LED_LOW_BATT_BUZZER_FLAG)
  {
	  buzzer_ExecBuzzer(1, 200, 200);
  }
  else
    led_buzzer_pulse_cnt = 0;

  osDelay(led_blink_on_ms);
}

/**
  * @brief  Change the status led to blink off state
  * @param  None
  * @retval None
  */
void statusLED_Blink_Off(void)
{
  rgbled_TurnOffLED();
  led_blink_cnt++;

  if (!(osEventFlagsGet(osFlags_LED) & LED_BLINK_NON_STOP_FLAG))
  {
    if (led_blink_cnt < led_blink_target_cnt)
    {
      ledTaskState = STATE_LED_BLINK_ON;
      statusLED_WaitForDelay(led_blink_off_ms);
    }
    else
    {
      ledTaskState = STATE_LED_INIT;
      osDelay(300);
    }
  }
  else
  {
    ledTaskState = STATE_LED_BLINK_ON;
    statusLED_WaitForDelay(led_blink_off_ms);
  }
}

/**
  * @brief  wait for certain amount of time, if start flag is trigger then
  *         proceed to next state immediately
  * @param  delay_ms:  Total delay time in milliseconds
  * @retval None
  */
void statusLED_WaitForDelay(uint32_t delay_ms)
{
  uint32_t time_ms = 0;

  while( time_ms < delay_ms)
  {
    osDelay(100);
    time_ms += 100;

    if (osEventFlagsGet(osFlags_LED) & LED_START_FLAG)
    {
      ledTaskState = STATE_LED_INIT;
      break;
    }
  }
}

/**
  * @brief  Change the current status led task state,
  *         so that it can proceed to next state
  * @param  state:  Status led current task state
  * @retval None
  */
void statusLED_ChangeCurrentState(LEDTask_State state)
{
  ledTaskState = state;
}

/**
  * @brief  Get the current status led task state
  * @param  None
  * @retval State:  status led current task state
  */
LEDTask_State statusLED_GetCurrentState(void)
{
  return ledTaskState;
}

/**
  * @brief  Configure status led blink settings
  * @param  count:     Blink LED count
  * @param  delay_ms:  LED on/off interval time in ms
  * @retval None
  */
void statusLED_ConfigureBlink(uint32_t count, uint32_t led_on_ms, uint32_t led_off_ms)
{
  if (count == LED_NON_STOP)
    osEventFlagsSet(osFlags_LED, LED_BLINK_NON_STOP_FLAG);
  else
    osEventFlagsClear(osFlags_LED, LED_BLINK_NON_STOP_FLAG);

  led_blink_target_cnt = count;
  led_blink_on_ms = led_on_ms;
  led_blink_off_ms = led_off_ms;
}


 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
