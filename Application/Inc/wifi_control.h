/**
  ******************************************************************************
  * @file    wifi_control.h
  * @author  IBronx MDE team
  * @brief   Peripheral driver header file for ESP32 WIFI Control
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
#ifndef __WIFI_CONTROL_H_
#define __WIFI_CONTROL_H_

#ifdef __cplusplus
 extern "C" {
#endif

 /* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

#include <stdbool.h>

 /* Exported types ------------------------------------------------------------*/
#define USART_RX_BUF_LEN                  512
#define AT_OK_STRING                      "OK"
#define AT_ERROR_STRING                   "ERROR"
#define AT_WIFI_READY_STRING              "ready"
#define AT_BUSY_STRING                    "busy p..."

#define AT_WIFI_CONNECTED_STRING          "WIFI GOT IP"
#define AT_WIFI_DISCONNECTED_STRING       "WIFI DISCONNECT"
#define AT_WIFI_SCAN_AP_STRING            "+CWLAP"
#define AT_HTTPCLIENT_RESPONSE_STRING     "+HTTPCLIENT:"
#define AT_TCP_ESTABLISHED_STRING         "ALREADY CONNECTED"
#define AT_TCP_SEND_STRING                ">"
#define AT_TCP_SEND_SUCCESS_STRING        "SEND OK"
#define AT_TCP_RECEIVED_DATA              "+IPD"

#define ESP_WIFI_SCAN_RETRY               3

#define ESP_AT_RESPONSE_SIZE              1024
#define ESP_WIFI_SSID_SIZE                20
#define ESP_WIFI_PASSWORD_SIZE            20
#define ESP_MAC_ADDR_SIZE                 6
#define ESP_IP_ADDR_SIZE                  16
#define ESP_CMD_NO_DELAY_MS               0
#define ESP_CMD_SHORT_DELAY_MS            200
#define ESP_CMD_NORMAL_DELAY_MS           1000
#define ESP_CMD_LONG_DELAY_MS             5000
#define ESP_CMD_AP_SCAN_DELAY_MS          15000

#define ESP_HARDWARE_RESET_ON             GPIO_PIN_SET
#define ESP_HARDWARE_RESET_OFF            GPIO_PIN_RESET

#define ESP_TOTAL_ACCESSPOINTS            10

 typedef enum
 {
   ESP_AT_WIFI_RESET = 1,
   ESP_AT_WIFI_SCAN_AP,
   ESP_AT_WIFI_ENABLE,
   ESP_AT_WIFI_DEEPSLEEP,
   ESP_AT_WIFI,
   ESP_AT_WIFI_TIMESTAMP,
   ESP_AT_WIFI_UPLOAD_DATA,
   ESP_AT_TCPIP,
 }esp_CmdTypeDef;

 typedef enum {
   STATUS_AT_UNKNOWN = 0,
   STATUS_AT_PASS,
   STATUS_AT_ERROR,
   STATUS_AT_BUSY,
   STATUS_AT_TIMEOUT,
   STATUS_AT_IO_ERROR,
 }esp_CmdResults;

 typedef enum {
   ESP_OPEN = 0,
   ESP_WEP,
   ESP_WPA_PSK,
   ESP_WPA2_PSK,
   ESP_WPA_WPA2_PSK,
   ESP_WPA2_ENTERPRISE,
   ESP_WPA3_PSK,
   ESP_WPA2_WPA3_PSK,
   ESP_WAPI_PSK,
}esp_encryption_methods;

typedef struct {
  char parse_str[100];
}esp_AccessPoint_Str_t;

 typedef struct {
   uint8_t encryption;
   char ssid[ESP_WIFI_SSID_SIZE];
   int rssi;
   uint8_t mac_addr[ESP_MAC_ADDR_SIZE];
 }esp_AccessPoint_t;

 /* Exported constants --------------------------------------------------------*/
 /* Exported macro ------------------------------------------------------------*/
 /* Exported functions ------------------------------------------------------- */

 void esp32_Init(void);
 void esp32_usart_RxCallback(uint8_t* pData, uint16_t len);
 void USART_IRQHandler(UART_HandleTypeDef *huart);

 void esp32_command_Response(uint8_t* pData, uint8_t len);
 void esp32_command_ParseResults(const char* pData);
 void esp32_command_ParseReset(const char* pData);
 void esp32_command_ParseListAP(const char* pData);
 void esp32_command_ParseJSON(const char* pData);
 void esp32_command_ParseTimeStamp(const char* pData);
 void esp32_command_ParseConnectAP(const char* pData);
 void esp32_command_ParseDeepSleep(const char* pData);
 void esp32_command_ParseCWLAP(uint8_t indexOfAP);
 uint32_t esp32_command_TransmitAT(char *pcmd, uint16_t cmdLen,
     uint32_t timeout_ms, esp_CmdTypeDef cmd_type);

 uint32_t esp32_atc_Reset(void);
 uint32_t esp32_atc_Test(void);
 uint32_t esp32_atc_EnableEcho(bool bEnable);
 uint32_t esp32_atc_Restore(void);
 uint32_t esp32_atc_AutoConnectAP(bool bEnable);
 uint32_t esp32_atc_DeepSleep(uint32_t time_ms);

 uint32_t esp32_wifi_ConnectAP(char* ssid, char*ps);
 uint32_t esp32_wifi_ConfigureMode(char* mode);
 uint32_t esp32_wifi_DisconnectAP(void);
 uint32_t esp32_wifi_ScanAP(char *ssid);
 uint32_t esp32_wifi_GetMacAddr(char* ssid);

 uint32_t esp32_tcpip_GetLocalIpAddress(void);
 uint32_t esp32_tcpip_GetConnectionStatus(void);
 uint32_t esp32_tcpip_SendData(char *msg, char *msgLen);
 uint32_t esp32_tcpip_EstablishProtocolConnection(char *protocol, char *ipAddr, char *port);
 uint32_t esp32_tcpip_SendData(char *msg, char *msgLen);
 uint32_t esp32_tcpip_GetTimestamp(void);

 uint32_t esp32_httpclient_GetRequest(char *contentType, char *url , char* transportType,
     uint32_t timeout_ms);
 uint32_t esp32_httpclient_PostRequest(char *contentType, char *url , char* transportType,
     char* data, uint32_t timeout_ms);
 uint32_t esp32_httpclient_GetTimeStamp(const char* locationStr);

#ifdef __cplusplus
}
#endif

#endif /* __WIFI_CONTROL_H_ */


 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
