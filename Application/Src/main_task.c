/**
  ******************************************************************************
  * @file    main_task.c
  * @author  IBronx MDE team
  * @brief   Main Task program
  *          This file provides Main task functions to execute all the
  *          main program operations
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
#include "main_task.h"
#include "cmsis_os.h"
#include "wifi_task.h"
#include "distance_task.h"
#include "buzzer_task.h"
#include "led_task.h"
#include "filesystem_task.h"
#include "led_control.h"
#include "buzzer_control.h"
#include "flash_control.h"
#include "filesystem_control.h"
#include "dwm1000.h"
#include "distance_process.h"
#include "usb_device.h"
#include "common.h"
#include "app_config.h"
#include "logger.h"

#include<stdbool.h>

/* Private define ------------------------------------------------------------*/

/* Definitions for DistanceTask */
osThreadId_t DistanceTaskHandle;
const osThreadAttr_t DistanceTask_attributes = {
  .name = "DistanceTask",
  .stack_size = 640 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal3,
};
/* Definitions for StatusLED */
osThreadId_t StatusLEDHandle;
const osThreadAttr_t StatusLED_attributes = {
  .name = "StatusLED",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal2,
};
/* Definitions for BuzzerTask */
osThreadId_t BuzzerTaskHandle;
const osThreadAttr_t BuzzerTask_attributes = {
  .name = "BuzzerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* Definitions for WiFiTask */
osThreadId_t WiFiTaskHandle;
const osThreadAttr_t WiFiTask_attributes = {
  .name = "WiFiTask",
  .stack_size = 640 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for FatfsTask */
osThreadId_t FatfsTaskHandle;
const osThreadAttr_t FatfsTask_attributes = {
  .name = "FatfsTask",
  .stack_size = 1280 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
MainTask_State mainState;
uint32_t count_per_sec;

osSemaphoreId_t osSmp_distance_signal;
osSemaphoreId_t osSmp_wifi_signal;
osSemaphoreId_t osSmp_fatfs_signal;
osSemaphoreId_t osSmp_distance_count;

osEventFlagsId_t osFlags_Distance;
osEventFlagsId_t osFlags_Fatfs;
osEventFlagsId_t osFlags_Wifi;
osEventFlagsId_t osFlags_LED;
osEventFlagsId_t osFlags_Buzzer;
osEventFlagsId_t osFlags_Main;

extern RTC_HandleTypeDef hrtc;

/* function prototypes -------------------------------------------------------*/

/**
  * @brief  Function implementing the MainTask thread.
  * @param  argument: Not used
  * @retval None
  */
void MainTaskFunc(void *argument)
{
  uint32_t tick = osKernelGetTickCount();
  count_per_sec = 0;
  for(;;)
  {
    tick += TASK_MAIN_DELAY_MS;
    count_per_sec++;
    switch(mainState)
    {
      case STATE_MAIN_INIT:
        main_task_Init();
        break;
      case STATE_MAIN_PREPARATION:
        main_task_Preparation();
        break;
      case STATE_MAIN_START_WIFI:
        main_task_StartFATFSOperation();
        break;
      case STATE_MAIN_START_DISTANCE:
        main_task_StartDistanceOperation();
        break;
      case STATE_MAIN_RUNNING:
        main_task_Running();
        break;
      case STATE_MAIN_STOP_OPERATIONS:
        main_task_StopOperations();
        break;
      case STATE_MAIN_STANDBY:
        main_task_EnterStandbyMode();
        break;
      case STATE_MAIN_LOW_BATTERY:
        main_task_LowBatteryMode();
        break;
      case STATE_MAIN_START_IDLE:
        break;
    }
    osDelayUntil(tick);
  }

  // delete the main thread, in case accidentally break the loop
  osThreadTerminate(NULL);
}

void HAL_PWR_PVDCallback(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}

/**
  * @brief  Execute main task initialization
  * @param  None
  * @retval None
  */
void main_task_Init(void)
{
  SEGGER_SYSVIEW_Print("[MAIN] - Task Initialization");
  taskENTER_CRITICAL();

  // semaphore creation
  osSmp_distance_signal = osSemaphoreNew(1, 0, NULL);
  osSmp_fatfs_signal = osSemaphoreNew(1, 0, NULL);
  osSmp_distance_count = osSemaphoreNew(DISTANCE_DATA_MAX, 0, NULL);
  osSmp_wifi_signal = osSemaphoreNew(1, 0, NULL);

  // event flags creation
  osFlags_Distance = osEventFlagsNew(NULL);
  osFlags_Fatfs = osEventFlagsNew(NULL);
  osFlags_Wifi = osEventFlagsNew(NULL);
  osFlags_LED = osEventFlagsNew(NULL);
  osFlags_Buzzer = osEventFlagsNew(NULL);
  osFlags_Main = osEventFlagsNew(NULL);

  // tasks creation
  DistanceTaskHandle = osThreadNew(DistanceTaskFunc, NULL, &DistanceTask_attributes);
  StatusLEDHandle = osThreadNew(StatusLEDFunc, NULL, &StatusLED_attributes);
  BuzzerTaskHandle = osThreadNew(BuzzerTaskFunc, NULL, &BuzzerTask_attributes);
  WiFiTaskHandle = osThreadNew(WiFiTaskFunc, NULL, &WiFiTask_attributes);
  FatfsTaskHandle = osThreadNew(fatfsTaskFunc, NULL, &FatfsTask_attributes);

  taskEXIT_CRITICAL();

  mainState = STATE_MAIN_PREPARATION;
}

/**
  * @brief  Execute main task preparation
  * @param  None
  * @retval None
  */
void main_task_Preparation(void)
{
  SEGGER_SYSVIEW_Print("[MAIN] - Program Initialization");

  // check if the standby flag is set
  if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
  {
    // clear the standby flag
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
    SEGGER_SYSVIEW_Print(" Program wakeup from the Standby Mode!!");

    // disable the WAKEUP Pin, i.e. PA0
    HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
  }

  SEGGER_SYSVIEW_Print("[Main] - Trigger FileSystem SDCARD Operation");
  osSemaphoreRelease(osSmp_fatfs_signal);

  MX_USB_DEVICE_Init();
  rgbled_InitDevice();
  buzzer_InitDevice();

  // battery level measurement
  StartBatteryVoltAdcConversion();

  logger_LogInfo("Social Distance Device Program Start from here", LOGGER_NULL_STRING);

  // update the firmware version
  uint8_t pVersion[FLASH_FIRMWARE_VERSION];
  flash_read_FirmwareVersion(pVersion);
  if ((pVersion[0] != FIRMWARE_MAJOR_VERSION) ||
      (pVersion[1] != FIRMWARE_MINOR_VERSION) ||
      (pVersion[2] != FIRMWARE_PATCH_VERSION))
  {
    flash_write_FirmwareVerWithBackUp(FIRMWARE_MAJOR_VERSION,
        FIRMWARE_MINOR_VERSION, FIRMWARE_PATCH_VERSION);
  }

  uint8_t pData[2];
  flash_read_DistLEDBlinkTime(pData);
  uint16_t buzz = pData[0] + (pData[1] << 8);
  BUZZER_TRIGGER_TIME_IN_SEC = buzz;
  mainState = STATE_MAIN_START_DISTANCE;
  osDelay(100);
}

/**
  * @brief  Execute main task start WiFi operation
  * @param  None
  * @retval None
  */
void main_task_StartFATFSOperation(void)
{
  logger_LogInfo("[Main] - Trigger WIFI Operation", LOGGER_NULL_STRING);
  osSemaphoreRelease(osSmp_wifi_signal);
  mainState = STATE_MAIN_RUNNING;
}

/**
  * @brief  Execute main task start distance measurement operation
  * @param  None
  * @retval None
  */
void main_task_StartDistanceOperation(void)
{
  logger_LogInfo("[Main] - Trigger Distance Measurement Operation", LOGGER_NULL_STRING);
  osSemaphoreRelease(osSmp_distance_signal);
  mainState = STATE_MAIN_START_WIFI;
}

/**
  * @brief  Execute main task main operation
  * @param  None
  * @retval None
  */
void main_task_Running(void)
{
  // when wakeup pin is reset, meaning card is not present
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
  {
    logger_LogInfo("[Main] - MCU Standby Mode Trigger to stop all operations", LOGGER_NULL_STRING);
    osEventFlagsSet(osFlags_Main, MAIN_MCU_STANDBY_FLAG);

    // hard reset to wake up device then execute mcu standby mode
    osEventFlagsSet(osFlags_Wifi, WIFI_WAKEUP_HARD_RESET);

    mainState = STATE_MAIN_STOP_OPERATIONS;

    return;
  }

  // monitor battery level & battery charge status
  main_BatteryMonitorTask(count_per_sec);

  if ((count_per_sec % TASK_MAIN_EXECUTION_SEC) == 0)
  {
    if (osEventFlagsGet(osFlags_Fatfs) & (FS_MOUNT_SDCARD_FLAG | FS_CREATE_FOLDERS_FLAG))
    {
      char buffer2[20];
      uint32_t total_distance_cnt = filesystem_GetDistanceCount();
      if (!(osEventFlagsGet(osFlags_Wifi) & WIFI_UPLOAD_DATA_SDCARD))
      {
        if (total_distance_cnt >= DISTANCE_DATA_WIFI_UPLOAD_THRESHOLD)
        {
          osEventFlagsSet(osFlags_Wifi, WIFI_UPLOAD_DATA_SDCARD);
          if (!(osEventFlagsGet(osFlags_Wifi) & WIFI_WAKEUP_FLAG))
          {
            logger_LogInfo("[Main] - Trigger WIFI to wake up from deep-sleep", LOGGER_NULL_STRING);
            osEventFlagsSet(osFlags_Wifi, WIFI_WAKEUP_FLAG);
          }

          logger_LogInfo("[Main] - Trigger WIFI to upload data from SDCARD with data count", itoa(total_distance_cnt, buffer2, 10));
        }
      }
      else
      {
        if (!(osEventFlagsGet(osFlags_Wifi) & WIFI_WAKEUP_FLAG))
        {
          logger_LogInfo("[Main] - Trigger WIFI to wake up from deep-sleep", LOGGER_NULL_STRING);
          osEventFlagsSet(osFlags_Wifi, WIFI_WAKEUP_FLAG);
        }

        logger_LogInfo("[Main] - Trigger WIFI to upload data from SDCARD with data count", itoa(total_distance_cnt, buffer2, 10));
      }
    }

    if (!(osEventFlagsGet(osFlags_Wifi) & WIFI_UPDATE_RTC_FLAG))
    {
      if (!(osEventFlagsGet(osFlags_Wifi) & WIFI_WAKEUP_FLAG))
      {
        logger_LogInfo("[Main] - Trigger WIFI to wake up from deep-sleep for query TimeStamp", LOGGER_NULL_STRING);
        osEventFlagsSet(osFlags_Wifi, WIFI_WAKEUP_FLAG);
      }
    }
  }
}

/**
  * @brief  Execute main task stop all operations
  * @param  None
  * @retval None
  */
void main_task_StopOperations(void)
{
  // Set DWM1000 enter deep-sleep mode
  // Set ESP32 WIFI enter deep-sleep mode
  // Store the distance data into SDCARD
  // Save RTC into flash

  if (osEventFlagsGet(osFlags_Wifi) & WIFI_UPDATE_RTC_FLAG)
    WriteCurrentRTCToFlash();

  if (osEventFlagsGet(osFlags_Main) & MAIN_BATTERY_LOW_FLAG)
  {
    mainState = STATE_MAIN_LOW_BATTERY;
    rgbled_BatteryLevelStatus(MAIN_BATTERY_LOW_FLAG);
  }
  else
  {
    mainState = STATE_MAIN_STANDBY;
    rgbled_StandbyMode();
  }
}

/**
  * @brief  Execute main task enter standby mode
  * @param  None
  * @retval None
  */
void main_task_EnterStandbyMode(void)
{
  uint32_t flags = osEventFlagsWait(osFlags_Main, MAIN_DWM1000_STANDBY_FLAG | MAIN_WIFI_STANDBY_FLAG |
      MAIN_FATFS_STANDBY_FLAG, osFlagsWaitAll | osFlagsNoClear, osWaitForever);

  if (flags > 0)
  {
    logger_LogInfo("[Main] - MCU Enter the Standby Mode!!", LOGGER_NULL_STRING);

    HAL_Delay(50);

    // enter the standby mode
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
    HAL_PWR_EnterSTANDBYMode();

    mainState = STATE_MAIN_START_IDLE;
  }
}

/**
  * @brief  Execute main task enter low battery mode, stop distance operations
  * @param  None
  * @retval None
  */
void main_task_LowBatteryMode(void)
{
  uint32_t flags = osEventFlagsWait(osFlags_Main, MAIN_DWM1000_STANDBY_FLAG | MAIN_WIFI_STANDBY_FLAG |
      MAIN_FATFS_STANDBY_FLAG, osFlagsWaitAll | osFlagsNoClear, 0);

  if (flags > 0)
  {
    if ((count_per_sec % TASK_LOG_BATTERY_LEVEL_SEC) == 0)
    {
      // monitor battery level
      StartBatteryVoltAdcConversion();

      char buffer[100];
      uint16_t bat_volt = GetBatteryLevelInVolt();
      uint16_t bat_pct = GetBatteryLevelInPct();
      sprintf(buffer, "[Main] - Low battery Mode, battery: %dmV, %d%%", bat_volt , bat_pct);
      logger_LogInfo(buffer, LOGGER_NULL_STRING);

      if (GetBatteryLevelInVolt() >= BATTERY_MID_VOLT)
      {
        logger_LogInfo("[Main] - Resume operation when battery level resume back to normal level", LOGGER_NULL_STRING);
        osEventFlagsClear(osFlags_Main, MAIN_BATTERY_LOW_FLAG);

        // wake up the wifi device and clear the mcu standby flag to execute normal wifi operation
        osEventFlagsClear(osFlags_Main, MAIN_MCU_STANDBY_FLAG);
        osEventFlagsSet(osFlags_Wifi, WIFI_WAKEUP_HARD_RESET);

        mainState = STATE_MAIN_RUNNING;
      }
    }
  }

  // when wakeup pin is reset, meaning card is not present
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
   {
    logger_LogWarn("[Main] - Ready to Enter MCU standbyMode", NULL);
    osEventFlagsClear(osFlags_Main, MAIN_BATTERY_LOW_FLAG);
    osEventFlagsSet(osFlags_Main, MAIN_MCU_STANDBY_FLAG);

    mainState = STATE_MAIN_STANDBY;
   }
}

/**
  * @brief  Monitor battery level, terminate the operations when enter low battery mode
  * @param  count_per_seconds:  Each count represent 1 seconds
  * @retval None
  */
void main_BatteryMonitorTask(uint32_t count_per_seconds)
{
  char buffer[100];

  // measure battery voltage
  StartBatteryVoltAdcConversion();
  uint16_t bat_volt = GetBatteryLevelInVolt();
  uint16_t bat_pct = GetBatteryLevelInPct();


  // log the battery level for every 60 seconds
  if ((count_per_seconds % TASK_LOG_BATTERY_LEVEL_SEC) == 0)
  {
    sprintf(buffer, "[Main] - Log the battery level: %dmV, %d%%", bat_volt , bat_pct);
    logger_LogInfo(buffer, LOGGER_NULL_STRING);
  }

  // when usb plug in
  if (HAL_GPIO_ReadPin(VBUS_DET_GPIO_Port, VBUS_DET_Pin) == GPIO_PIN_SET)
  {
    if (!(osEventFlagsGet(osFlags_Main) & MAIN_BATTERY_CHARGE_FLAG))
    {
      sprintf(buffer, "[Main] - EXTI Battery Charging ON when battery: %dmV, %d%%", bat_volt , bat_pct);
      logger_LogInfo(buffer, LOGGER_NULL_STRING);
      rgbled_BatteryLevelStatus(MAIN_BATTERY_CHARGE_FLAG);

      osEventFlagsSet(osFlags_Main, MAIN_BATTERY_CHARGE_FLAG);
      osEventFlagsClear(osFlags_Main, MAIN_BATTERY_LOW_FLAG | MAIN_BATTERY_FULL_FLAG);
    }
    else
    {
      if (!(osEventFlagsGet(osFlags_LED) & LED_DISTANCE_FLAG))
      {
        // when battery charge until full
        if (GetBatteryLevelInVolt() >= BATTERY_HIGH_VOLT)
        {
          if (!(osEventFlagsGet(osFlags_Main) & MAIN_BATTERY_FULL_FLAG))
          {
            sprintf(buffer, "[Main] - Battery charged Full, battery: %dmV, %d%%", bat_volt , bat_pct);
            logger_LogInfo(buffer, LOGGER_NULL_STRING);
            rgbled_BatteryLevelStatus(MAIN_BATTERY_FULL_FLAG);

            osEventFlagsSet(osFlags_Main, MAIN_BATTERY_FULL_FLAG);
          }

        }
        else
        {
          if (!(osEventFlagsGet(osFlags_LED) & LED_BATTERY_FLAG))
            rgbled_BatteryLevelStatus(MAIN_BATTERY_CHARGE_FLAG);

          osEventFlagsClear(osFlags_Main, MAIN_BATTERY_FULL_FLAG);
        }
      }
    }
  }
  else
  {
    // when usb unplug
    if (osEventFlagsGet(osFlags_Main) & MAIN_BATTERY_CHARGE_FLAG)
    {
      sprintf(buffer, "[Main] - EXTI Battery Charging OFF when battery: %dmV, %d%%", bat_volt , bat_pct);
      logger_LogInfo(buffer, LOGGER_NULL_STRING);

      rgbled_ClearFlags(LED_BATTERY_FLAG);

      if (!(osEventFlagsGet(osFlags_LED) & LED_DISTANCE_FLAG))
        rgbled_Heartbeat();

      osEventFlagsClear(osFlags_Main, MAIN_BATTERY_CHARGE_FLAG);
    }
    else
    {
      if (bat_volt <= BATTERY_LOW_VOLT)
      {
        sprintf(buffer, "[Main] - Low Battery is detected when battery: %dmV, %d%%", bat_volt , bat_pct);
        logger_LogInfo(buffer, LOGGER_NULL_STRING);

        if (!(osEventFlagsGet(osFlags_LED) & LED_DISTANCE_FLAG))
          rgbled_Heartbeat();

        osEventFlagsSet(osFlags_Main, MAIN_BATTERY_LOW_FLAG);
        osEventFlagsClear(osFlags_Main, MAIN_BATTERY_FULL_FLAG);
        mainState = STATE_MAIN_STOP_OPERATIONS;
      }
      else
      {
        if (!(osEventFlagsGet(osFlags_LED) & LED_DISTANCE_FLAG))
        {
          if (!(osEventFlagsGet(osFlags_LED) & LED_HEARTBEAT_FLAG))
          {
            rgbled_Heartbeat();
          }
        }
      }
    }
  }
}

/**
  * @brief  MCU Wakeup Pin EXTI handler
  * @param  None
  * @retval None
  */
void WakeUp_EXTIHandler(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0)
  {
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
    {
      SEGGER_SYSVIEW_Print("[Main] - WAKEUP Pin RESET");
    }
    else
    {
      SEGGER_SYSVIEW_Print("[Main] - WAKEUP Pin SET");
    }
  }
}


 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
