/**
  ******************************************************************************
  * @file    led_control.c
  * @author  IBronx MDE team
  * @brief   Peripheral driver for ws28xx RGB LED control
  *          This file provides firmware utility functions to support WS28xx RGB
  *          LED functions
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
#include "led_control.h"
#include "main.h"
#include "led_task.h"
#include "main_task.h"
#include "cmsis_os.h"
#include "distance_process.h"
#include "flash_control.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint16_t LED_GPIO_Pin;

extern osEventFlagsId_t osFlags_LED;

/* Private function prototypes -----------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/**
* @brief  Initialize Red, Green, Blue LED
* @param  None
* @retval None
*/
void rgbled_Init(void)
{
  // turn off all LEDs
  HAL_GPIO_WritePin(GPIOB, LED_RED_PIN|LED_BLUE_PIN|LED_GREEN_PIN, LED_OFF);
}

/**
  * @brief  Display RGB LED when Initialize the device
  * @param  None
  * @retval None
  */
void rgbled_InitDevice(void)
{
  rgbled_ExecBlink(3, 200, 200, LED_DEVICE_INIT);

  rgbled_ClearFlags(LED_STANDBY_MODE_FLAG | LED_DISTANCE_FLAG |
      LED_BATTERY_FLAG | LED_HEARTBEAT_FLAG | LED_ENABLE_BUZZER_FLAG | LED_LOW_BATT_BUZZER_FLAG);
}

/**
  * @brief  Display RGB LED when enter standby mode
  * @param  None
  * @retval None
  */
void rgbled_StandbyMode(void)
{
  osEventFlagsSet(osFlags_LED, LED_STANDBY_MODE_FLAG);
  rgbled_ExecBlink(LED_NON_STOP, 200, 200, LED_STANDBY_MODE);

  rgbled_ClearFlags(LED_DISTANCE_FLAG | LED_BATTERY_FLAG  | LED_HEARTBEAT_FLAG | LED_ENABLE_BUZZER_FLAG | LED_LOW_BATT_BUZZER_FLAG);
}

/**
  * @brief  Display RGB LED for distance measurement results
  * @param  device_cnt  Total detected devices count
  * @param  flags       Distance measurement flags
  * @retval None
  */
void rgbled_DistanceMeasurement(uint32_t device_cnt, uint32_t flags)
{

//  uint8_t pData[2];
//  flash_read_DistLEDBlinkTime(pData);
//
//  uint16_t distLedBlink = pData[0] + (pData[1] << 8);
  osEventFlagsSet(osFlags_LED, LED_DISTANCE_FLAG);
  osEventFlagsClear(osFlags_LED, LED_HEARTBEAT_FLAG | LED_BATTERY_FLAG);

  if (flags == DISTANCE_LED_CRITICAL_FLAG)
  {
	  uint8_t pData[2];
	  flash_read_BuzzState(pData);

	  uint16_t buzzState = pData[0] + (pData[1] << 8);
	  if(buzzState == 1)
	  {
		  osEventFlagsSet(osFlags_LED, LED_ENABLE_BUZZER_FLAG);
	  }
    switch(device_cnt)
    {
      case 0:
        osEventFlagsClear(osFlags_LED, LED_DISTANCE_FLAG | LED_ENABLE_BUZZER_FLAG);
        rgbled_Heartbeat();
        break;
      case 1:
	    SEGGER_SYSVIEW_Print("[Distance] - LED BLINK");
        rgbled_ExecBlink(LED_NON_STOP, 500, 500, LED_CLOSE_DEVICE_1);
        break;
      case 2:
        rgbled_ExecBlink(LED_NON_STOP, 500, 500, LED_CLOSE_DEVICE_2);
        break;
      case 3:
        rgbled_ExecBlink(LED_NON_STOP, 500, 500, LED_CLOSE_DEVICE_3);
        break;
      case 4:
        rgbled_ExecBlink(LED_NON_STOP, 500, 500, LED_CLOSE_DEVICE_4);
        break;
      case 5:
        rgbled_ExecBlink(LED_NON_STOP, 500, 500, LED_CLOSE_DEVICE_5);
        break;
    }
  }
  else if (flags == DISTANCE_LED_MEDIUM_FLAG)
  {
    rgbled_ExecBlink(LED_NON_STOP, 500, 500, LED_RED_PIN | LED_GREEN_PIN);
    osEventFlagsClear(osFlags_LED, LED_ENABLE_BUZZER_FLAG);
  }
  else if (flags == DISTANCE_LED_LOW_FLAG)
  {
    osEventFlagsClear(osFlags_LED, LED_DISTANCE_FLAG | LED_ENABLE_BUZZER_FLAG);
    rgbled_Heartbeat();
  }
}

/**
  * @brief  Display RGB LED for battery level status
  * @param  flags  Battery charging status flag
  * @retval None
  */
void rgbled_BatteryLevelStatus(uint32_t flags)
{
	uint8_t pData[2];
    flash_read_ChgLEDTime(pData);
    uint16_t ChgLedFreq = pData[0] + (pData[1] << 8);

    if(ChgLedFreq >= LED_MAX_CHARGEBEAT_OFF_MS)
    {
    	ChgLedFreq = LED_MAX_CHARGEBEAT_OFF_MS;
    }

    ChgLedFreq = (ChgLedFreq * 1000) - 200;

    uint8_t bData[2];
	flash_read_BattLowFreq(bData);
	uint16_t battLowFreq = bData[0] + (bData[1] << 8);


  osEventFlagsSet(osFlags_LED, LED_BATTERY_FLAG);
  osEventFlagsClear(osFlags_LED, LED_HEARTBEAT_FLAG | LED_DISTANCE_FLAG | LED_ENABLE_BUZZER_FLAG | LED_LOW_BATT_BUZZER_FLAG);

  if (flags == MAIN_BATTERY_CHARGE_FLAG)
  {
    rgbled_ExecBlink(LED_NON_STOP, 200, (uint32_t)ChgLedFreq, LED_BATTERY_CHARGING);
  }
  else if (flags == MAIN_BATTERY_LOW_FLAG)
  {
    rgbled_ExecBlink(LED_NON_STOP, 200, (uint32_t)((battLowFreq*1000)-200), LED_LOW_BATTERY);
    //osEventFlagsSet(osFlags_LED, LED_ENABLE_BUZZER_FLAG);
    osEventFlagsSet(osFlags_LED, LED_LOW_BATT_BUZZER_FLAG);
  }
  else if (flags == MAIN_BATTERY_FULL_FLAG)
  {
    rgbled_ExecBlink(LED_NON_STOP, 200, (uint32_t)ChgLedFreq, LED_BATTERY_FULL_CHARGE);
  }
}

/**
  * @brief  Display RGB LED for hearbeat message
  * @param  None
  * @retval None
  */
void rgbled_Heartbeat(void)
{
  uint8_t pData[2];
  flash_read_LEDBlinkTime(pData);

  uint16_t heartbeat_off_ms = pData[0] + (pData[1] << 8);
  if (heartbeat_off_ms == 0 || heartbeat_off_ms >= LED_MAX_HEARTBEAT_OFF_MS)
  {
	  heartbeat_off_ms = 2500;
	  rgbled_ExecBlink(LED_NON_STOP, 500, heartbeat_off_ms, LED_HEARTBEAT_MODE);
  }
  else
  {
	  rgbled_ExecBlink(LED_NON_STOP, 500, heartbeat_off_ms*1000, LED_HEARTBEAT_MODE);
  }
    //heartbeat_off_ms = 2500;

  //rgbled_ExecBlink(LED_NON_STOP, 500, 2500, LED_HEARTBEAT_MODE);
  osEventFlagsSet(osFlags_LED, LED_HEARTBEAT_FLAG);
}

/**
  * @brief  Clear the LED flags
  * @param  None
  * @retval None
  */
void rgbled_ClearFlags(uint32_t flags)
{
  if ((flags & LED_BATTERY_FLAG) == LED_BATTERY_FLAG)
    if (osEventFlagsGet(osFlags_LED) & LED_BATTERY_FLAG)
      osEventFlagsClear(osFlags_LED, LED_BATTERY_FLAG);

  if ((flags & LED_DISTANCE_FLAG) == LED_DISTANCE_FLAG)
    if (osEventFlagsGet(osFlags_LED) & LED_DISTANCE_FLAG)
      osEventFlagsClear(osFlags_LED, LED_DISTANCE_FLAG);

  if ((flags & LED_STANDBY_MODE_FLAG) == LED_STANDBY_MODE_FLAG)
    if (osEventFlagsGet(osFlags_LED) & LED_STANDBY_MODE_FLAG)
      osEventFlagsClear(osFlags_LED, LED_STANDBY_MODE_FLAG);

  if ((flags & LED_HEARTBEAT_FLAG) == LED_HEARTBEAT_FLAG)
    if (osEventFlagsGet(osFlags_LED) & LED_HEARTBEAT_FLAG)
      osEventFlagsClear(osFlags_LED, LED_HEARTBEAT_FLAG);

  if ((flags & LED_ENABLE_BUZZER_FLAG) == LED_ENABLE_BUZZER_FLAG)
    if (osEventFlagsGet(osFlags_LED) & LED_ENABLE_BUZZER_FLAG)
      osEventFlagsClear(osFlags_LED, LED_ENABLE_BUZZER_FLAG);

  if ((flags & LED_LOW_BATT_BUZZER_FLAG) == LED_LOW_BATT_BUZZER_FLAG)
      if (osEventFlagsGet(osFlags_LED) & LED_LOW_BATT_BUZZER_FLAG)
        osEventFlagsClear(osFlags_LED, LED_LOW_BATT_BUZZER_FLAG);
}

/**
  * @brief  Execute RGB LED blinking
  * @param  blink_count:  Number of led blink count
  * @param  led_on_ms:    LED on time in millisecond
  * @param  led_off_ms:   LED off time in millisecond
  * @param  led_color:    GPIO pin number link to RGB color
  * @retval None
  */
void rgbled_ExecBlink(uint32_t blink_count, uint32_t led_on_ms, uint32_t led_off_ms, uint16_t led_color)
{
  LED_GPIO_Pin = led_color;
  statusLED_ConfigureBlink(blink_count, led_on_ms, led_off_ms);
  osEventFlagsSet(osFlags_LED, LED_START_FLAG);
}

/**
  * @brief  Turn on either Red, Green or Blue LED based on LED_Pin
  * @param  None
  * @retval None
  */
void rgbled_TurnOnLED(void)
{
  HAL_GPIO_WritePin(GPIOB, LED_GPIO_Pin, LED_ON);
}

/**
  * @brief  Turn off Red, Green, Blue LED
  * @param  None
  * @retval None
  */
void rgbled_TurnOffLED(void)
{
  HAL_GPIO_WritePin(GPIOB, LED_RED_PIN|LED_BLUE_PIN|LED_GREEN_PIN, LED_OFF);
}


/************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
