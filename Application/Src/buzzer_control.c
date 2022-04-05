/**
  ******************************************************************************
  * @file    buzzer_control.c
  * @author  IBronx MDE team
  * @brief   Peripheral driver for buzzer control
  *          This file provides firmware utility functions to support Buzzer
  *          on/off functions
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
#include "buzzer_control.h"
#include "buzzer_task.h"
#include "cmsis_os.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern TIM_HandleTypeDef htim2;
extern osEventFlagsId_t osFlags_Buzzer;
/* Private function prototypes -----------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/**
* @brief  Enable the buzzer when device initialize
* @param  None
* @retval None
*/
void buzzer_InitDevice(void)
{
  buzzer_ChangePWMFrequency(4000);
  buzzer_ExecBuzzer(3, 200, 200);
//
//  	//Update Buzzer Count from Flash
//    uint8_t pData[2];
//    flash_read_DistLEDBlinkTime(pData);
//
  	  BUZZER_TRIGGER_TIME_IN_SEC = 0;
}

/**
  * @brief  Trigger Buzzer task to ready turn buzzer
  * @param  pulse_count:   Number of buzzer pulse count
  * @param  pulse_on_ms:   Buzzer On interval in millisecond
  * @param  pulse_off_ms:  Buzzer Off interval in millisecond
  * @retval None
  */
void buzzer_ExecBuzzer(uint32_t pulse_count, uint32_t pulse_on_ms, uint32_t pulse_off_ms)
{
  buzzer_ChangeCurrentState(STATE_BUZZER_INIT);
  buzzer_SetConfiguration(pulse_count, pulse_on_ms, pulse_off_ms);
  osEventFlagsSet(osFlags_Buzzer, BUZZER_START_FLAG);
}

/**
  * @brief  Turn on buzzer
  * @param  None
  * @retval None
  */
void buzzer_TurnOn(void)
{
  HAL_TIM_PWM_Start(&htim2, BUZZER_TIM_CHANNEL);
}

/**
  * @brief  Turn off buzzer
  * @param  None
  * @retval None
  */
void buzzer_TurnOff(void)
{
  HAL_TIM_PWM_Stop(&htim2, BUZZER_TIM_CHANNEL);
}

/**
  * @brief  Change the buzzer PWM frequnecy on runtime
  * @param  h_idx:      The index of the buzzer handle
  * @param  frequency:  Target buzzer frequency
  * @retval None
  */
void buzzer_ChangePWMFrequency(uint32_t frequency)
{
  uint32_t period_cnt = buzzer_GetTIMClkFreq(htim2.Instance) / frequency;
  period_cnt = period_cnt /  BUZZER_TIM_PRESCALER;
  uint32_t pulse_cnt = period_cnt >> 1;

  // Update both TIM Autoreload and Capture Compare Register on runtime
  __HAL_TIM_SET_AUTORELOAD(&htim2, period_cnt);
  __HAL_TIM_SET_COMPARE(&htim2, BUZZER_TIM_CHANNEL, pulse_cnt);
}

/**
  * @brief  Get TIMER Clock Frequency
  * @param  tim:      TIMER instance
  * @retval ClkFreq:  TIMER Clock Frequency
  */
uint32_t buzzer_GetTIMClkFreq(TIM_TypeDef * tim)
{
  // APB1 TIMER Clock
  if (tim == TIM2  ||
      tim == TIM3  ||
      tim == TIM4  ||
      tim == TIM5  ||
      tim == TIM6  ||
      tim == TIM7  ||
      tim == TIM12 ||
      tim == TIM13 ||
      tim == TIM14 )
  {
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0)
      return HAL_RCC_GetPCLK1Freq() << 1;
    else
      return HAL_RCC_GetPCLK1Freq();
  }

  // APB2 TIMER Clock
  if (tim == TIM1  ||
      tim == TIM8  ||
      tim == TIM9  ||
      tim == TIM10 ||
      tim == TIM11)
  {
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0)
      return HAL_RCC_GetPCLK2Freq() << 1;
    else
      return HAL_RCC_GetPCLK2Freq();
  }

  // prevent divide by zero if none of above selection is valid
  return 1;
}



/************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
