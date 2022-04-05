/**
  ******************************************************************************
  * @file    wifi_task.c
  * @author  IBronx MDE team
  * @brief   WIFI Task program
  *          This file provides wifi task functions to execute all the
  *          WIFI related operations
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
#include "wifi_task.h"
#include "cmsis_os.h"
#include "wifi_control.h"
#include "errorcode.h"
#include "flash_control.h"
#include "filesystem_control.h"
#include "app_config.h"
#include "main_task.h"
#include "logger.h"

#include <stdio.h>

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
wifiTask_State wifiTaskState;

extern osEventFlagsId_t osFlags_Wifi;
extern osEventFlagsId_t osFlags_Main;
extern osSemaphoreId_t osSmp_wifi_signal;
/* Private function prototypes -----------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/**
* @brief Function implementing the WiFiTask thread.
* @param argument: Not used
* @retval None
*/
void WiFiTaskFunc(void *argument)
{
  wifiTaskState = STATE_WIFI_INIT;

  for(;;)
  {
    wifi_StateOperation();
  }
  osThreadTerminate(NULL);
}

/**
  * @brief  Running ESP32 WIFI task operation based on current state
  * @param  None
  * @retval None
  */
void wifi_StateOperation(void)
{
  switch(wifiTaskState)
  {
    case STATE_WIFI_INIT:
      if (osSemaphoreAcquire(osSmp_wifi_signal, 50U) == osOK)
      {
        logger_LogInfo("[WIFI] - Task is ready to RUN", LOGGER_NULL_STRING);
        wifiTaskState = STATE_WIFI_HARDWARE_RESET;
      }
      break;
    case STATE_WIFI_IDLE:
      osDelay(550);
      break;
    case STATE_WIFI_HARDWARE_RESET:
      wifi_task_HardwareReset();
      osDelay(2000);
      break;
    case STATE_WIFI_DEVICE_INIT:
      wifi_task_Init();
      break;
    case STATE_WIFI_DEVICE_START:
      wifi_task_DeviceInit();
      break;
    case STATE_WIFI_CONNECT:
      wifi_task_ConnectAccessPoint();
      break;
    case STATE_WIFI_DISCONNECT:
      osDelay(500);
      break;
    case STATE_WIFI_RUNNING:
      wifi_task_StartOperation();
      break;
    case STATE_WIFI_DEEPSLEEP:
      wifi_task_EnterDeepSleep();
      break;
    case STATE_WIFI_DEEPSLEEP_IDLE:
      wifi_task_DeepSleepIdle();
      break;
  }
}

/**
  * @brief  Run the ESP32 WIFI task Init State
  * @param  None
  * @retval None
  */
void wifi_task_Init(void)
{
  esp32_Init();
  wifiTaskState = STATE_WIFI_DEVICE_START;
  osDelay(200);
}

/**
  * @brief  Run the ESP32 WIFI Hardware Reset State
  * @param  None
  * @retval None
  */
void wifi_task_HardwareReset(void)
{
  logger_LogInfo("[WIFI] - Hardware Reset for ESP32", LOGGER_NULL_STRING);
  HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, ESP_HARDWARE_RESET_ON);
  HAL_Delay(500);
  HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, ESP_HARDWARE_RESET_OFF);

  // reset wifi events flags
  osEventFlagsClear(osFlags_Wifi, WIFI_INIT_FLAG);
  osEventFlagsClear(osFlags_Wifi, WIFI_CONNECT_FLAG);
  osEventFlagsClear(osFlags_Wifi, WIFI_DEEPSLEEP_FLAG);
  osEventFlagsClear(osFlags_Wifi, WIFI_WAKEUP_FLAG);
  osEventFlagsClear(osFlags_Wifi, WIFI_WAKEUP_HARD_RESET);

  wifiTaskState = STATE_WIFI_DEVICE_INIT;
}

/**
  * @brief  Run the ESP32 WIFI task device Init
  * @param  None
  * @retval None
  */
void wifi_task_DeviceInit(void)
{
  uint32_t rc = PER_NO_ERROR;

  // check MCU standby flag before go to wifi connect state
  if (osEventFlagsGet(osFlags_Main) & MAIN_MCU_STANDBY_FLAG)
  {
    wifiTaskState = STATE_WIFI_DEEPSLEEP;
    return;
  }
  else
  {
    logger_LogInfo("[WIFI] - Initialize the ESP32 module", LOGGER_NULL_STRING);
  }

  if ((rc = esp32_atc_Reset())== PER_NO_ERROR)
  {
    if ((rc = esp32_atc_EnableEcho(false))== PER_NO_ERROR)
    {
      if ((rc = esp32_atc_Test())== PER_NO_ERROR)
      {
        if ((rc = esp32_atc_AutoConnectAP(false))== PER_NO_ERROR)
        {
          char *mode = "1";
          if ((rc = esp32_wifi_ConfigureMode(mode)) == PER_NO_ERROR)
          {
            osEventFlagsSet(osFlags_Wifi, WIFI_INIT_FLAG);

            // check MCU standby flag before go to wifi connect state
            if (osEventFlagsGet(osFlags_Main) & MAIN_MCU_STANDBY_FLAG)
            {
              wifiTaskState = STATE_WIFI_DEEPSLEEP;
              return;
            }
            else
              wifiTaskState = STATE_WIFI_CONNECT;
            return;
          }
        }
      }
    }
  }

  wifiTaskState = STATE_WIFI_IDLE;
  logger_LogError("[WIFI] - Failed to Initialize ESP32 module", LOGGER_NULL_STRING);
}

/**
  * @brief  Run the ESP32 WIFI task connect to access point
  * @param  None
  * @retval None
  */
void wifi_task_ConnectAccessPoint(void)
{
  uint32_t flags = osEventFlagsWait(osFlags_Wifi, WIFI_INIT_FLAG,
      osFlagsWaitAny | osFlagsNoClear, osWaitForever);

  if (flags > 0)
  {
    logger_LogInfo("[WIFI] - Preparing to connect AccessPoint", LOGGER_NULL_STRING);

    char ssid[ESP_WIFI_SSID_SIZE];
    char password[ESP_WIFI_SSID_SIZE];
    flash_read_WifiCredentials((uint8_t*)ssid, (uint8_t*)password);

    // look for AccessPoint
    //esp32_wifi_ScanAP(ssid);
    esp32_wifi_GetMacAddr(ssid);

    osDelay(2000);
    // connect to access point
    esp32_wifi_ConnectAP(ssid, password);

    if (osEventFlagsGet(osFlags_Wifi) & WIFI_CONNECT_FLAG)
    {
      logger_LogInfo("[WIFI] - Success to connect AccessPoint", LOGGER_NULL_STRING);
      wifiTaskState = STATE_WIFI_RUNNING;
      osDelay(200);
    }
    else
    {
      logger_LogWarn("[WIFI] - Failed to connect AccessPoint", LOGGER_NULL_STRING);
      wifiTaskState = STATE_WIFI_DEEPSLEEP;
    }
  }
}

/**
  * @brief  Start the ESP32 WIFI task Operation
  * @param  None
  * @retval None
  */
void wifi_task_StartOperation(void)
{
  uint32_t flags = osEventFlagsWait(osFlags_Wifi, WIFI_CONNECT_FLAG,
      osFlagsWaitAny | osFlagsNoClear, osWaitForever);

  if (flags > 0)
  {
    if (!(osEventFlagsGet(osFlags_Wifi) & WIFI_UPDATE_RTC_FLAG))
    {
      logger_LogInfo("[WIFI] - Ready to query TimeStamp when WIFI is connected", LOGGER_NULL_STRING);
      uint32_t rc = esp32_httpclient_GetTimeStamp("Asia/Kuala_Lumpur");
      if (rc == PER_NO_ERROR)
        logger_LogInfo("[WIFI] - Success to query TimeStamp", LOGGER_NULL_STRING);
      else
        logger_LogError("[WIFI] - Failed to query TimeStamp", LOGGER_NULL_STRING);
    }

    if (osEventFlagsGet(osFlags_Wifi) & WIFI_UPLOAD_DATA_SDCARD)
    {
      logger_LogInfo("[WIFI] - Ready to upload data when WIFI is connected", LOGGER_NULL_STRING);
      if (filesystem_UploadDistanceData() == PER_NO_ERROR)
        osEventFlagsClear(osFlags_Wifi, WIFI_UPLOAD_DATA_SDCARD);
    }
    else
    {
      if (osEventFlagsGet(osFlags_Wifi) & WIFI_UPLOAD_DATA)
      {

        // TODO: Upload distance data
        osEventFlagsClear(osFlags_Wifi, WIFI_UPLOAD_DATA);
      }
    }
  }

  // enter deep-sleep mode
  wifiTaskState = STATE_WIFI_DEEPSLEEP;
}

/**
  * @brief  Start the ESP32 WIFI task deep-sleep mode
  * @param  None
  * @retval None
  */
void wifi_task_EnterDeepSleep(void)
{
  // read the wifi sleep interval from flash
  uint8_t pData[2];
  flash_read_WiFiSleepInterval(pData);

  uint16_t deep_sleep_interval = pData[0] + (pData[1] << 8);
  if (deep_sleep_interval == 0 || deep_sleep_interval == 0xFFFF)
  {
    deep_sleep_interval = WIFI_DEEP_SLEEP_INTERVAL_MS;
  }

  if (osEventFlagsGet(osFlags_Main) & MAIN_MCU_STANDBY_FLAG)
    esp32_atc_DeepSleep(1728000000); // deep-sleep 20 days
  else
    esp32_atc_DeepSleep(deep_sleep_interval);

  // check whether deep-sleep command is successful or not.
  if (osEventFlagsGet(osFlags_Wifi) & WIFI_DEEPSLEEP_FLAG)
  {
    if (osEventFlagsGet(osFlags_Main) & MAIN_MCU_STANDBY_FLAG)
    {
      logger_LogInfo("[WIFI] - Enter long deep-sleep mode", LOGGER_NULL_STRING);
      osEventFlagsSet(osFlags_Main, MAIN_WIFI_STANDBY_FLAG);
    }
    else
    {
      char buf[100];
      sprintf(buf, "[WIFI] - Enter deep-sleep mode, %dms", deep_sleep_interval);
      logger_LogInfo(buf, LOGGER_NULL_STRING);
    }

    wifiTaskState = STATE_WIFI_DEEPSLEEP_IDLE;
  }
  else
  {
    logger_LogError("[WIFI] - Failed to enter deep-sleep mode", LOGGER_NULL_STRING);
    wifiTaskState = STATE_WIFI_HARDWARE_RESET;
  }
}

/**
  * @brief  Running the ESP32 WIFI task deep-sleep mode
  * @param  None
  * @retval None
  */
void wifi_task_DeepSleepIdle(void)
{
  uint32_t flags = osEventFlagsWait(osFlags_Wifi, WIFI_CONNECT_FLAG |
      WIFI_DEEPSLEEP_AGAIN_FLAG | WIFI_WAKEUP_HARD_RESET, osFlagsWaitAny | osFlagsNoClear, osWaitForever);

  if (flags > 0)
  {
    if (osEventFlagsGet(osFlags_Wifi) & WIFI_DEEPSLEEP_AGAIN_FLAG)
    {
      logger_LogInfo("[WIFI] - Enter deep-sleep mode again", LOGGER_NULL_STRING);
      wifi_ChangeCurrentState(STATE_WIFI_DEEPSLEEP);
      osEventFlagsClear(osFlags_Wifi, WIFI_DEEPSLEEP_AGAIN_FLAG);
    }

    if (osEventFlagsGet(osFlags_Wifi) & WIFI_CONNECT_FLAG)
    {
      logger_LogInfo("[WIFI] - Wake up and then connect to WIFI", LOGGER_NULL_STRING);
      wifi_ChangeCurrentState(STATE_WIFI_CONNECT);
      osEventFlagsClear(osFlags_Wifi, WIFI_CONNECT_FLAG);
    }

    // wake up the device by execute hard reset
    if (osEventFlagsGet(osFlags_Wifi) & WIFI_WAKEUP_HARD_RESET)
    {
      logger_LogInfo("[WIFI] - Wake up the device by execute hard reset", LOGGER_NULL_STRING);
      wifi_ChangeCurrentState(STATE_WIFI_HARDWARE_RESET);
      osEventFlagsClear(osFlags_Wifi, WIFI_WAKEUP_HARD_RESET);
    }
  }
}

/**
  * @brief  Change the current WIFI task state,
  *         so that it can proceed to next state
  * @param  state:  WIFI current task state
  * @retval None
  */
void wifi_ChangeCurrentState(wifiTask_State state)
{
  wifiTaskState = state;
}

/**
  * @brief  Get the current WIFI task state
  * @param  None
  * @retval State:  fatfs current task state
  */
wifiTask_State wifi_GetCurrentState(void)
{
  return wifiTaskState;
}


 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
