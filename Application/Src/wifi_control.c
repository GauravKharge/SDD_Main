/**
  ******************************************************************************
  * @file    wifi_control.c
  * @author  IBronx MDE team
  * @brief   Peripheral driver for ESP32 WIFI Control
  *          This file provides firmware utility functions to support ESP32 WIFI
  *          module functions
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
#include "wifi_control.h"
#include "cmsis_os.h"
#include "errorcode.h"
#include "wifi_task.h"
#include "flash_control.h"
#include "utils.h"
#include "logger.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t rx_buffer[USART_RX_BUF_LEN];
char esp_http_resp[ESP_AT_RESPONSE_SIZE];
char esp_wifi_ssid[ESP_WIFI_SSID_SIZE];
char esp_query_ssid[ESP_WIFI_SSID_SIZE];

uint8_t esp_mac_addr[ESP_MAC_ADDR_SIZE];
uint8_t esp_ip_addr[ESP_IP_ADDR_SIZE];

esp_AccessPoint_t esp_ap[ESP_TOTAL_ACCESSPOINTS];
esp_AccessPoint_Str_t esp_ap_str[ESP_TOTAL_ACCESSPOINTS];

static volatile esp_CmdTypeDef esp_at_cmd;
static volatile esp_CmdResults esp_cmd_results = 0;

extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

extern osEventFlagsId_t osFlags_Wifi;
/* Private function prototypes -----------------------------------------------*/
/* function prototypes -------------------------------------------------------*/


/**
* @brief  Initialize ESP32 WIFI module
* @param  None
* @retval None
*/
void esp32_Init(void)
{
  // Enable USART Idle Interrupt
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

  // Enable DMA to receive data from USART Rx
  HAL_UART_Receive_DMA(&huart1, (uint8_t*)rx_buffer, USART_RX_BUF_LEN);
}

/**
  * @brief  USART Rx callback Handler function
  * @param  pData:  Pointer memory Rx buffer
  * @param  len:    Number of memory Rx buffer
  * @retval None
  */
void esp32_usart_RxCallback(uint8_t* pData, uint16_t len)
{
  SEGGER_SYSVIEW_RecordEnterISR();

  switch(esp_at_cmd)
  {
    case ESP_AT_WIFI_RESET:
      esp32_command_ParseReset((char*)pData);
      break;
    case ESP_AT_WIFI_SCAN_AP:
      esp32_command_ParseListAP((char*)pData);
      break;
    case ESP_AT_WIFI_ENABLE:
      esp32_command_ParseConnectAP((char*)pData);
      break;
    case ESP_AT_WIFI_DEEPSLEEP:
      esp32_command_ParseDeepSleep((char*)pData);
      break;
    case ESP_AT_WIFI:
    case ESP_AT_WIFI_TIMESTAMP:
    case ESP_AT_WIFI_UPLOAD_DATA:
      SEGGER_SYSVIEW_Print((char*)pData);
      esp32_command_ParseResults((char*)pData);
      break;
    case ESP_AT_TCPIP:
      if (strstr((char*)pData, AT_OK_STRING) != NULL ||
          strstr((char*)pData, AT_TCP_SEND_STRING) != NULL)
      {
        esp_cmd_results = STATUS_AT_PASS;
        SEGGER_SYSVIEW_Print("[ESP32] - TCP/IP OK");
      }
      break;
  }

  SEGGER_SYSVIEW_RecordExitISR();
}


/**
  * @brief  Transmit AT command send via USART/UART module
  * @param  pcmd:        Pointer of command memory buffer
  * @param  cmdLen:      Command memory buffer length
  * @param  timeout_ms:  Command execution timeout in ms, if 0 then no waiting
  * @param  cmd_type:    AT command type
  * @retval rc:  If pass then return PER_NO_ERROR, otherwise error code
  */
uint32_t esp32_command_TransmitAT(char *pcmd, uint16_t cmdLen, uint32_t timeout_ms, esp_CmdTypeDef cmd_type)
{
  esp_at_cmd = cmd_type;
  esp_cmd_results = STATUS_AT_UNKNOWN;

  // enable IT and transmit esp at command via USART/UART module
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

  HAL_UART_Transmit(&huart1, (uint8_t *)pcmd, strlen(pcmd), 1000);

  // waiting for receive the response message from esp32 module
  if (timeout_ms > 0)
  {
    uint32_t starttime = HAL_GetTick();
    while(esp_cmd_results == STATUS_AT_UNKNOWN)
    {
      osDelay(100);
      if ((HAL_GetTick() -  starttime) >= timeout_ms)
      {
        esp_cmd_results = STATUS_AT_TIMEOUT;
        break;
      }
    }

    if (esp_cmd_results == STATUS_AT_PASS)
      return PER_NO_ERROR;
    else if (esp_cmd_results == STATUS_AT_TIMEOUT)
    {
      char buf[20];
      logger_LogError("[ESP32] - AT Command wait timeout", itoa(timeout_ms, buf, 10));
      return PER_ERROR_ESP32_ATC_TIMEOUT;
    }
    else
    {
      logger_LogError("[ESP32] - AT COMMAND ERROR", pcmd);
      return PER_ERROR_ESP32_ATC_ERROR;
    }
  }

  return PER_NO_ERROR;
}

/**
  * @brief  USART IRQ Handler Rx Idle interrupt callback function
  *         Note: Require copy this function to USARTx_IRQHandler() in file stm32f4xx_it.c
  * @param  huart:  USART instance
  * @retval None
  */
void USART_IRQHandler(UART_HandleTypeDef *huart)
{
  if (huart->Instance == huart1.Instance)
  {
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
    {
      __HAL_UART_CLEAR_IDLEFLAG(&huart1);

      HAL_UART_DMAStop(&huart1);

      uint16_t rx_cnt = USART_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

      // execute the callback function
      esp32_usart_RxCallback(rx_buffer, rx_cnt);

      // reset the memory buffer to prepare next receive
      memset(rx_buffer, 0, sizeof(rx_buffer));

      HAL_UART_Receive_DMA(&huart1, (uint8_t*)rx_buffer, USART_RX_BUF_LEN);
    }
  }
}

// ------------------------ ESP32 HTTPCLIENT -------------------------------------

/**
  * @brief  Send HTTP Client Get Request
  * @param  contentType:   Data type of HTTP client request
  *                          0 - application/x-www-form-urlencoded
  *                          1 - application/json
  *                          2 - multipart/form-data
  *                          3 - text/xml
  * @param  url:            HTTP URL
  * @param  transportType:  HTTP Client transport type. Default: 1
  *                          1 - HTTP_TRANSPORT_OVER_TCP
  *                          2 - HTTP_TRANSPORT_OVER_SSL
  * @param  timeout_ms:     Command execution timeout in ms, if 0 then no waiting
  * @retval rc:  If pass then return PER_NO_ERROR, otherwise error code
  */
uint32_t esp32_httpclient_GetRequest(char *contentType, char *url , char* transportType, uint32_t timeout_ms)
{
  memset(esp_http_resp, 0, sizeof(esp_http_resp));

  char *cmdstring = "AT+HTTPCLIENT=";
  char *optstring = "2,";
  char *preUrlstr = ",\"";
  char *postUrlstr = "\",,,";
  char *eofString = "\r\n";

  char *txString = pvPortMalloc(strlen(cmdstring) + strlen(optstring) + strlen(contentType)
      + strlen(preUrlstr) +strlen(url) + + strlen(postUrlstr) + strlen(transportType) + strlen(eofString) + 1);
  if(txString != NULL)
  {
    strcpy(txString, cmdstring);
    strcat(txString, optstring);
    strcat(txString, contentType);
    strcat(txString, preUrlstr);
    strcat(txString, url);
    strcat(txString, postUrlstr);
    strcat(txString, transportType);
    strcat(txString, eofString);
  }

  uint32_t rc =  esp32_command_TransmitAT(txString, strlen(txString), timeout_ms, ESP_AT_WIFI_TIMESTAMP);

  vPortFree(txString);

  return rc;
}

/**
  * @brief  Send HTTP Client Post Request
  * @param  contentType:   Data type of HTTP client request
  *                          0 - application/x-www-form-urlencoded
  *                          1 - application/json
  *                          2 - multipart/form-data
  *                          3 - text/xml
  * @param  url:            HTTP URL
  * @param  transportType:  HTTP Client transport type. Default: 1
  *                          1 - HTTP_TRANSPORT_OVER_TCP
  *                          2 - HTTP_TRANSPORT_OVER_SSL
  * @param  data:           HTTP Post data that will send to HTTP server
  * @param  timeout_ms:     Command execution timeout in ms, if 0 then no waiting
  * @retval rc:  If pass then return PER_NO_ERROR, otherwise error code
  */
uint32_t esp32_httpclient_PostRequest(char *contentType, char *url , char* transportType, char* data, uint32_t timeout_ms)
{
  memset(esp_http_resp, 0, sizeof(esp_http_resp));

  char *cmdstring = "AT+HTTPCLIENT=";
  char *optstring = "3,";
  char *preUrlstr = ",\"";
  char *postUrlstr = "\",,,";
  char *preDatastr = ",\"";
  char *postDatastr = "\"";
  char *eofString = "\r\n";

  char *txString = pvPortMalloc(strlen(cmdstring) + strlen(optstring) + strlen(contentType)
      + strlen(preUrlstr) +strlen(url) + + strlen(postUrlstr) + strlen(transportType)
      + strlen(preDatastr) + strlen(data) + + strlen(postDatastr) + strlen(eofString) + 1);
  if(txString != NULL)
  {
    strcpy(txString, cmdstring);
    strcat(txString, optstring);
    strcat(txString, contentType);
    strcat(txString, preUrlstr);
    strcat(txString, url);
    strcat(txString, postUrlstr);
    strcat(txString, transportType);
    strcat(txString, preDatastr);
    strcat(txString, data);
    strcat(txString, postDatastr);
    strcat(txString, eofString);
  }
  //SEGGER_SYSVIEW_Print(txString);

  uint32_t rc =  esp32_command_TransmitAT(txString, strlen(txString), timeout_ms, ESP_AT_WIFI_UPLOAD_DATA);

  vPortFree(txString);

  return rc;
}

/**
  * @brief  Consume "World Time API" to get the time stamp based on input location argument
  * @param  location:   location string
  * @retval rc:  If pass then return PER_NO_ERROR, otherwise error code
  */
uint32_t esp32_httpclient_GetTimeStamp(const char* location)
{
  char *url = "http://worldtimeapi.org/api/timezone/";
  char *fullUrl = pvPortMalloc(strlen(url) + strlen(location));
  strcpy(fullUrl, url);
  strcat(fullUrl, location);
  uint32_t rc = esp32_httpclient_GetRequest("1", fullUrl, "1", ESP_CMD_LONG_DELAY_MS);
  vPortFree(fullUrl);

  return rc;
}

// ------------------------ ESP32 AT COMMANDS PARSER-----------------------------

/**
  * @brief  Parse esp32 command response from UART module
  * @param  pData:  UART Rx message pointer
  * @retval None
  */
void esp32_command_ParseResults(const char* pData)
{
  if (strstr((char*)pData, AT_WIFI_DISCONNECTED_STRING) != NULL)
  {
    SEGGER_SYSVIEW_Warn("[ESP32] - WIFI DISCONNECTED");
    osEventFlagsClear(osFlags_Wifi, WIFI_CONNECT_FLAG);
  }
  else if (strstr((char*)pData, AT_WIFI_CONNECTED_STRING) != NULL)
  {
    SEGGER_SYSVIEW_Print("[ESP32] - WIFI CONNECTED");
    osEventFlagsSet(osFlags_Wifi, WIFI_CONNECT_FLAG);

  }
  else if (strstr(pData, AT_OK_STRING) != NULL)
  {
    esp_cmd_results = STATUS_AT_PASS;

    if (esp_at_cmd == ESP_AT_WIFI_TIMESTAMP)
      esp32_command_ParseTimeStamp((char*)pData);
  }
  else if (strstr(pData, AT_ERROR_STRING) != NULL)
  {
    esp_cmd_results = STATUS_AT_ERROR;
  }
  else if (strstr(pData, "STATUS:") != NULL)
  {
    if (strstr(pData, "2") != NULL)
      esp_cmd_results = STATUS_AT_PASS;
    else
      esp_cmd_results = STATUS_AT_ERROR;
  }
}

/**
  * @brief  Parse esp32 command response from UART module when reset
  *         command executed
  * @param  pData:  UART Rx message pointer
  * @retval None
  */
void esp32_command_ParseReset(const char* pData)
{
  if (strstr((char*)pData, AT_WIFI_READY_STRING) != NULL)
  {
    SEGGER_SYSVIEW_Warn("[ESP32] - WIFI RESET");
    esp_cmd_results = STATUS_AT_PASS;
  }
}

/**
  * @brief  Parse esp32 command response from UART module when scan all AccessPoint
  *         command executed
  * @param  pData:  UART Rx message pointer
  * @retval None
  */
void esp32_command_ParseListAP(const char* pData)
{
  if (strstr((char*)pData, AT_WIFI_SCAN_AP_STRING) != NULL)
  {
    uint8_t idx_AP = 0;
    char delim[] = "+";
    char *ptr = strtok((char *)pData, delim);
    while(ptr != NULL)
    {
      if (strstr((char*)ptr, esp_query_ssid) != NULL)
      //if (strstr((char*)ptr, "IBRONX") != NULL)
      {
        SEGGER_SYSVIEW_Print(ptr);
        memcpy(esp_ap_str[idx_AP].parse_str, ptr, strlen(ptr));
        idx_AP++;
      }

      ptr = strtok(NULL, delim);
    }

    // skip it if none of APs are belong to Ibronx gateway
    if (idx_AP == 0)
      return;

    // parse the message
    memset(esp_ap, 0, sizeof(esp_ap));
    for (uint8_t idx = 0; idx < idx_AP; idx++)
      esp32_command_ParseCWLAP(idx);

    // choose the nearest Access Point
    uint8_t idx_rssi = 0;
    int rssi_value = 0;

    for (uint8_t idx = 0; idx < idx_AP; idx++)
    {
      if (idx == 0)
      {
        idx_rssi = idx;
        rssi_value = esp_ap[idx].rssi;
      }
      else
      {
        if (esp_ap[idx].rssi > rssi_value)
        {
          idx_rssi = idx;
          rssi_value = esp_ap[idx].rssi;
        }
      }
    }

    // update the SSID based on RSSI value
    memcpy(esp_wifi_ssid, esp_ap[idx_rssi].ssid, sizeof(esp_wifi_ssid));
    memcpy(esp_mac_addr, esp_ap[idx_rssi].mac_addr, sizeof(esp_mac_addr));

    esp_cmd_results = STATUS_AT_PASS;
  }
  else if (strstr(pData, AT_OK_STRING) != NULL)
  {
    esp_cmd_results = STATUS_AT_PASS;
  }
}

/**
  * @brief  Parse esp32 CWLAP AT command return string
  * @param  indexOfAP:  Index of CWLAP string memory array
  * @retval None
  */
void esp32_command_ParseCWLAP(uint8_t indexOfAP)
{
  char string[100];
  memcpy(string, esp_ap_str[indexOfAP].parse_str, strlen(esp_ap_str[indexOfAP].parse_str));

  // Example: CWLAP:(3,"IBRONX_2G",-70,"b0:7f:b9:bd:2e:60",1,-1,-1,4,4,7,0)
  // remove CWLAP: from string
  utils_RemoveSubstrFromString(string, "CWLAP:");

  char pAddr[25];
  char delim[] = ",";
  char *ptr = strtok((char*)string, delim);
  uint8_t index = 0;
  while(ptr != NULL)
  {
    index++;
    switch (index)
    {
      case 1:
        utils_RemoveCharFromString(ptr, '(');
        esp_ap[indexOfAP].encryption = atoi((ptr));
        break;
      case 2:
        utils_RemoveCharFromString(ptr, '\"');
        memcpy(esp_ap[indexOfAP].ssid, ptr, strlen(ptr));
        break;
      case 3:
        esp_ap[indexOfAP].rssi = atoi(ptr);
         break;
      case 4:
        memcpy(pAddr, ptr, strlen(ptr));
        utils_RemoveCharFromString(pAddr, '\"');

        // extract AP MAC Address
        uint8_t mac_index = 5;
        char * token = strtok(pAddr, ":");
        while( token != NULL ) {
          esp_ap[indexOfAP].mac_addr[mac_index] = (int)strtol(token, NULL, 16);
          token = strtok(NULL, ":");
          mac_index--;
        }
        break;
      default:
        break;
    }

    ptr = strtok(NULL, delim);
  }
}

/**
  * @brief  Parse esp32 command response from UART module when HTTP Client Request
  *         command executed
  * @param  pData:  UART Rx message pointer
  * @retval None
  */
void esp32_command_ParseHTTPClientRequest(const char* pData)
{
  if (strstr(pData, AT_OK_STRING) != NULL)
  {
    esp_cmd_results = STATUS_AT_PASS;
  }
  else if (strstr(pData, AT_ERROR_STRING) != NULL)
  {
    esp_cmd_results = STATUS_AT_ERROR;
  }
}

/**
  * @brief  Parse esp32 command response from UART module when connect to AcessPoint
  *         command executed
  * @param  pData:  UART Rx message pointer
  * @retval None
  */
void esp32_command_ParseConnectAP(const char* pData)
{
  if (strstr((char*)pData, AT_WIFI_CONNECTED_STRING) != NULL)
  {
    SEGGER_SYSVIEW_Print("[ESP32] - WIFI CONNECTED");
    osEventFlagsSet(osFlags_Wifi, WIFI_CONNECT_FLAG);

    esp_cmd_results = STATUS_AT_PASS;
  }
  else if (strstr(pData, AT_ERROR_STRING) != NULL)
  {
    SEGGER_SYSVIEW_Warn("[ESP32] - WIFI DISCONNECTED");
    osEventFlagsClear(osFlags_Wifi, WIFI_CONNECT_FLAG);
    esp_cmd_results = STATUS_AT_PASS;
  }
}

/**
  * @brief  Parse esp32 command response from UART module when deep sleep mode
  *         command executed
  * @param  pData:  UART Rx message pointer
  * @retval None
  */
void esp32_command_ParseDeepSleep(const char* pData)
{
  if (strstr((char*)pData, AT_WIFI_READY_STRING) != NULL)
  {
    SEGGER_SYSVIEW_Warn("[ESP32] - WIFI WAKE UP FROM SLEEP");

    osEventFlagsClear(osFlags_Wifi, WIFI_DEEPSLEEP_FLAG);

    // determine whether continue to sleep or wake-up
    if (osEventFlagsGet(osFlags_Wifi) & WIFI_WAKEUP_FLAG)
    {
      SEGGER_SYSVIEW_Warn("[ESP32] - WakeUp flag is up");
      osEventFlagsSet(osFlags_Wifi, WIFI_CONNECT_FLAG);
      osEventFlagsClear(osFlags_Wifi, WIFI_WAKEUP_FLAG);
    }
    else
    {
      SEGGER_SYSVIEW_Print("[ESP32] - Enter Sleep again");
      osEventFlagsSet(osFlags_Wifi, WIFI_DEEPSLEEP_AGAIN_FLAG);
    }
  }
  else if (strstr(pData, AT_OK_STRING) != NULL)
  {
    esp_cmd_results = STATUS_AT_PASS;
    osEventFlagsSet(osFlags_Wifi, WIFI_DEEPSLEEP_FLAG);
    osEventFlagsClear(osFlags_Wifi, WIFI_CONNECT_FLAG);
  }
}

/**
  * @brief  Consume World Time API to retrieve Current TimeStamp
  *         using API: http://worldtimeapi.org/
  * @param  pData:  Pointer of receive memory buffer
  * @retval None
  */
void esp32_command_ParseTimeStamp(const char* pData)
{
  bool bCompleteMsg = true;
  bool bUpdatedRTC = false;

  // if no 'OK', message is not complete yet
  if (strstr((char*)pData, "OK") == NULL)
    bCompleteMsg = false;

  // remove the prefix '+HTTPCLIENT:275,'
  char *ptr = strchr((char *)pData, ',') + 1;
  strcat(esp_http_resp, ptr);

  if (bCompleteMsg == true)
  {
    char delim[] = ",";
    char *ptr = strtok(esp_http_resp, delim);
    while(ptr != NULL)
    {
      if (strstr((char*)ptr, "datetime") != NULL)
      {
        // example: {"datetime":"2021-03-09T08:59:07.368105+08:00"}
        char *p = strstr(ptr, ":");
        int year = atoi(p+2);
        char *p1 = strstr(ptr, "-");
        int month = atoi(p1+1);
        char *p2 = strstr(p1+1, "-");
        int day = atoi(p2+1);

        char *p3 = strstr(ptr, "T");
        int hour = atoi(p3+1);
        char *p4 = strstr(p3, ":");
        int min = atoi(p4+1);
        char *p5 = strstr(p4+1, ":");
        int seconds = atoi(p5+1);

        // Initialize RTC and set the Time and Date
        RTC_DateTypeDef sDate = {0};
        RTC_TimeTypeDef sTime = {0};

        sDate.WeekDay = RTC_WEEKDAY_MONDAY;
        sDate.Month = (uint8_t)month;
        sDate.Date = (uint8_t)day;
        sDate.Year = (uint8_t)(year - 2000); // only allow 0 to 99 years
        sTime.Hours = (uint8_t)hour;
        sTime.Minutes = (uint8_t)min;
        sTime.Seconds = (uint8_t)seconds;
        sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
        sTime.StoreOperation = RTC_STOREOPERATION_RESET;
        if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) == HAL_OK)
        {
          if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) == HAL_OK)
          {
            SEGGER_SYSVIEW_Print("[ESP32] - Write updated RTC to flash");
            bUpdatedRTC = true;
            flash_write_RTCWithBackUp(sDate, sTime);
          }
        }
      }

      if (strstr((char*)ptr, "day_of_week") != NULL)
      {
        // example: {"day_of_week":1} Monday
        char *p6 = strstr(ptr, ":");
        int weekday = atoi(p6+1);

        RTC_DateTypeDef sDate = {0};
        HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
        sDate.WeekDay = (uint8_t)weekday;
        HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
        break;
      }
      ptr = strtok(NULL, delim);
    }

    if (bUpdatedRTC == true)
    {
      SEGGER_SYSVIEW_Print("[ESP32] - Synchronize MCU RTC");
      osEventFlagsSet(osFlags_Wifi, WIFI_UPDATE_RTC_FLAG);
    }
  }
}

// ------------------------ ESP32 BASIC AT COMMANDS -------------------------------

/**
  * @brief  Restart ESP32 module
  * @param  None
  * @retval rc:  If pass then return PER_NO_ERROR, otherwise error code
  */
uint32_t esp32_atc_Reset(void)
{
  char *txString = "AT+RST\r\n";
  return esp32_command_TransmitAT(txString, strlen(txString), ESP_CMD_LONG_DELAY_MS, ESP_AT_WIFI_RESET);
}

/**
  * @brief  Tests AT StartUp command
  * @param  None
  * @retval rc:  If pass then return PER_NO_ERROR, otherwise error code
  */
uint32_t esp32_atc_Test(void)
{
  char *txString = "AT\r\n";
  return esp32_command_TransmitAT(txString, strlen(txString), ESP_CMD_NORMAL_DELAY_MS, ESP_AT_WIFI);
}

/**
  * @brief  Configure echoing of AT commands
  * @param  bEnable:  Enable echo if true otherwise false
  * @retval rc:  If pass then return PER_NO_ERROR, otherwise error code
  */
uint32_t esp32_atc_EnableEcho(bool bEnable)
{
  char *txString;
  if (bEnable)
    txString = "ATE1\r\n";
  else
    txString = "ATE0\r\n";

  return esp32_command_TransmitAT(txString, strlen(txString), ESP_CMD_NORMAL_DELAY_MS, ESP_AT_WIFI);
}

/**
  * @brief  Restore ESP32 the default factory settings
  * @param  None
  * @retval rc:  If pass then return PER_NO_ERROR, otherwise error code
  */
uint32_t esp32_atc_Restore(void)
{
  char *txString = "AT+RESTORE\r\n";
  return esp32_command_TransmitAT(txString, strlen(txString), ESP_CMD_NORMAL_DELAY_MS, ESP_AT_WIFI);
}

/**
  * @brief  Auto connects to AP
  * @param  bEnable:  Enable Auto connect if true otherwise false
  * @retval rc:       If pass then return PER_NO_ERROR, otherwise error code
  */
uint32_t esp32_atc_AutoConnectAP(bool bEnable)
{
  char *txString;
  if (bEnable)
    txString = "AT+CWAUTOCONN=1\r\n";
  else
    txString = "AT+CWAUTOCONN=0\r\n";

  return esp32_command_TransmitAT(txString, strlen(txString), ESP_CMD_NORMAL_DELAY_MS, ESP_AT_WIFI);
}

/**
  * @brief  Enter Deep-slepp Mode, wake up after reach the duration
  * @param  time_ms:  Duration in milliseconds to wake up device
  * @retval rc:       If pass then return PER_NO_ERROR, otherwise error code
  */
uint32_t esp32_atc_DeepSleep(uint32_t time_ms)
{
  char cmd[50];
  sprintf((char*)cmd,"AT+GSLP=%lu\r\n", time_ms);
  return esp32_command_TransmitAT((char*)cmd, strlen((char*)cmd), ESP_CMD_NORMAL_DELAY_MS, ESP_AT_WIFI_DEEPSLEEP);
}

// ------------------------ ESP32 WIFI AT COMMANDS ---------------------------------

/**
  * @brief  Configure ESP32 module WIFI Mode
  * @param  mode:  WIFI Mode
  *                  "1" - Station
  *                  "2" - SoftAP
  *                  "3" - Station + SoftAP
  * @retval rc:    If pass then return PER_NO_ERROR, otherwise error code
  */
uint32_t esp32_wifi_ConfigureMode(char* mode)
{
  char *cmdstring = "AT+CWMODE=";
  char *eofString = "\r\n";
  char *txString = pvPortMalloc(strlen(cmdstring)+strlen(mode)+strlen(eofString)+1);
  if(txString != NULL)
  {
    strcpy(txString, cmdstring);
    strcat(txString, mode);
    strcat(txString, eofString);
  }

  uint32_t rc = esp32_command_TransmitAT(txString, strlen(txString), ESP_CMD_NORMAL_DELAY_MS, ESP_AT_WIFI);

  vPortFree(txString);

  return rc;
}

/**
  * @brief  Set ESP32 module connect to an AP
  * @param  ssid:      WIFI Network Name
  * @param  password:  WIFI password
  * @retval rc:        If pass then return PER_NO_ERROR, otherwise error code
  */
uint32_t esp32_wifi_ConnectAP(char *ssid, char *password)
{
  char *cmdstring = "AT+CWJAP=\"";
  char *midstring = "\",\"";
  char *eofString = "\",,,,,,8\r\n";
  char *txString = pvPortMalloc(strlen(cmdstring) + strlen(esp_wifi_ssid) +
      strlen(midstring) + strlen(password) + strlen(eofString) + 1);
  if(txString != NULL)
  {
    strcpy(txString, cmdstring);
    strcat(txString, esp_wifi_ssid);
    strcat(txString, midstring);
    strcat(txString, password);
    strcat(txString, eofString);
  }

  uint32_t rc = esp32_command_TransmitAT(txString, strlen(txString), ESP_CMD_AP_SCAN_DELAY_MS, ESP_AT_WIFI_ENABLE);

  vPortFree(txString);

  return rc;
}

/**
  * @brief  Set ESP32 module disconnect from an AP
  * @param  None
  * @retval rc:  If pass then return PER_NO_ERROR, otherwise error code
  */
uint32_t esp32_wifi_DisconnectAP(void)
{
  char *txString = "AT+CWQAP\r\n";
  return esp32_command_TransmitAT(txString, strlen(txString), ESP_CMD_NORMAL_DELAY_MS, ESP_AT_WIFI);
}

/**
  * @brief  Scan the Available AP and choose the best RSSI
  * @param  ssid:  WIFI SSID
  * @retval rc:    If pass then return PER_NO_ERROR, otherwise error code
  */
uint32_t esp32_wifi_ScanAP(char *ssid)
{
  char *txString = "AT+CWLAP\r\n";

  // store the wifi credential into memory array
  strcpy(esp_query_ssid, ssid);
  strcpy(esp_wifi_ssid, ssid);

  return esp32_command_TransmitAT(txString, strlen(txString), ESP_CMD_LONG_DELAY_MS, ESP_AT_WIFI_SCAN_AP);
}

/**
  * @brief  Get MAC Address based on SSID
  * @param  ssid:  WIFI SSID
  * @retval rc:    If pass then return PER_NO_ERROR, otherwise error code
  */
uint32_t esp32_wifi_GetMacAddr(char* ssid)
{
  char *cmdstring = "AT+CWLAP=\"";
  char *eofString = "\"\r\n";
  char *txString = pvPortMalloc(strlen(cmdstring) + strlen(ssid) + strlen(eofString));
  if(txString != NULL)
  {
    strcpy(txString, cmdstring);
    strcat(txString, ssid);
    strcat(txString, eofString);
  }

  // store the wifi credential into memory array
  strcpy(esp_query_ssid, ssid);
  strcpy(esp_wifi_ssid, ssid);

  uint32_t rc = esp32_command_TransmitAT(txString, strlen(txString), ESP_CMD_LONG_DELAY_MS, ESP_AT_WIFI_SCAN_AP);
  vPortFree(txString);

  return rc;
}

// ------------------------ ESP32 TCP/IP AT COMMANDS -------------------------------

/**
  * @brief  Set ESP32 module disconnect from an AP
  * @param  None
  * @retval rc:  If pass then return PER_NO_ERROR, otherwise error code
  */
uint32_t esp32_tcpip_GetLocalIpAddress(void)
{
  char *txString = "AT+CIFSR\r\n";
  return esp32_command_TransmitAT(txString, strlen(txString), ESP_CMD_NORMAL_DELAY_MS, ESP_AT_WIFI);
}

/**
  * @brief  Set ESP32 module disconnect from an AP
  * @param  None
  * @retval rc:  If pass then return PER_NO_ERROR, otherwise error code
  */
uint32_t esp32_tcpip_GetConnectionStatus(void)
{
  char *txString = "AT+CIPSTATUS\r\n";
  return esp32_command_TransmitAT(txString, strlen(txString), ESP_CMD_NORMAL_DELAY_MS, ESP_AT_WIFI);
}

/**
  * @brief  Establishes Internet protocol TCP/UDL/SSL connection
  * @param  protocol:  Internet Protocol connection
  *                      "TCP" - Transmission Control Protocol
  *                      "UDP" - User Datagram Protocol
  *                      "SSL" - Secure Sockets Layer
  * @param  ipAddr:    Internet IP Address
  * @param  port:      Port Number
  * @retval rc:        If pass then return PER_NO_ERROR, otherwise error code
  */
uint32_t esp32_tcpip_EstablishProtocolConnection(char *protocol, char *ipAddr, char *port)
{
  char *cmdstring = "AT+CIPSTART=\"";
  char *midstring = "\",\"";
  char *midstring2 = "\",";
  char *eofString = "\r\n";
  char *txString = pvPortMalloc(strlen(cmdstring) + strlen(protocol) + strlen(midstring) +
      strlen(ipAddr) + strlen(midstring2) + strlen(port) + strlen(eofString) + 1);
  if(txString != NULL)
  {
    strcpy(txString, cmdstring);
    strcat(txString, protocol);
    strcat(txString, midstring);
    strcat(txString, ipAddr);
    strcat(txString, midstring2);
    strcat(txString, port);
    strcat(txString, eofString);
  }

  uint32_t rc = esp32_command_TransmitAT(txString, strlen(txString), ESP_CMD_NORMAL_DELAY_MS, ESP_AT_TCPIP);
  vPortFree(txString);

  return rc;

}

/**
  * @brief  Establishes Internet protocol TCP/UDL/SSL connection
  * @param  protocol:  Internet Protocol connection
  *                      "TCP" - Transmission Control Protocol
  *                      "UDP" - User Datagram Protocol
  *                      "SSL" - Secure Sockets Layer
  * @param  ipAddr:    Internet IP Address
  * @param  port:      Port Number
  * @retval rc:        If pass then return PER_NO_ERROR, otherwise error code
  */
uint32_t esp32_tcpip_SendData(char *msg, char *msgLen)
{
  char *cmdstring = "AT+CIPSEND=";
  char *eofString = "\r\n";
  char *txString = pvPortMalloc(strlen(cmdstring)+strlen(msgLen)+strlen(eofString)+1);
  if(txString != NULL)
  {
    strcpy(txString, cmdstring);
    strcat(txString, msgLen);
    strcat(txString, eofString);
  }

  uint32_t rc = esp32_command_TransmitAT(txString, strlen(txString), ESP_CMD_NORMAL_DELAY_MS, ESP_AT_TCPIP);
  vPortFree(txString);
  if (rc == PER_NO_ERROR)
  {
    SEGGER_SYSVIEW_Print("[ESP32] - >...");
    uint8_t data[48];
    memset(data, 0, sizeof(data));
    data[0] = 0xa3;
    data[47] = 0;
    rc = esp32_command_TransmitAT((char*)data, sizeof(data), ESP_CMD_NORMAL_DELAY_MS, ESP_AT_TCPIP);
  }

  return rc;
}


/************************ (C) COPYRIGHT IBronx *****************END OF FILE****/

