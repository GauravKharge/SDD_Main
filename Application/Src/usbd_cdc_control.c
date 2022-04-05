/**
  ******************************************************************************
  * @file    usbd_cdc_control.c
  * @author  IBronx MDE team
  * @brief   USB device communications device class controls
  *          This file provides firmware utility functions to support USB device
  *          communication device class controls module functions
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
#include "usbd_cdc_control.h"
#include "usbd_cdc_if.h"
#include "flash_control.h"
#include "filesystem_control.h"
#include "errorcode.h"
#include "common.h"

#include <string.h>
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t usbd_cdc_rx_buf[USBD_CDC_RX_LEN];
uint8_t usbd_cdc_params1[USBD_CDC_PARAMS_LEN];
uint8_t usbd_cdc_params2[USBD_CDC_PARAMS_LEN];
uint8_t usbd_cdc_params3[USBD_CDC_PARAMS_LEN];
uint8_t usbd_cdc_params4[USBD_CDC_PARAMS_LEN];

uint8_t usbd_cdc_return_buf[USBD_CDC_RETURN_LEN];
uint8_t usbd_cdc_data1[USBD_CDC_PARAMS_LEN];
uint8_t usbd_cdc_data2[USBD_CDC_PARAMS_LEN];
uint8_t usbd_cdc_data3[USBD_CDC_PARAMS_LEN];
uint8_t usbd_cdc_data4[USBD_CDC_PARAMS_LEN];

usbd_optionsTypeDef usbd_options;
usbd_cmdTypeDef usbd_command;
bool bFirmwareUpdateFlag;
uint32_t firmwareUpdateFlashIdx;

/* Private function prototypes -----------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

void usbd_cdc_RxCallbackHandler(uint8_t* Buf, uint16_t Len)
{
  memset(usbd_cdc_rx_buf, '\0', USBD_CDC_RX_LEN);
  memset(usbd_cdc_params1, '\0', USBD_CDC_PARAMS_LEN);
  memset(usbd_cdc_params2, '\0', USBD_CDC_PARAMS_LEN);
  memset(usbd_cdc_params3, '\0', USBD_CDC_PARAMS_LEN);
  memset(usbd_cdc_params4, '\0', USBD_CDC_PARAMS_LEN);

  memcpy(usbd_cdc_rx_buf, Buf, Len);

  uint16_t battlen = 0;
  uint16_t battdata = 0;

  // check firmware update flags
  if (bFirmwareUpdateFlag)
  {
    if (strcmp((char*)usbd_cdc_rx_buf, USBD_CMD_FW_UPDATE_STOP_STR) == 0)
    {
      bFirmwareUpdateFlag = false;
      return;
    }
    else
      usbd_cdc_Firmware_Update(Len);

    return;
  }

  usbd_options = USBD_OP_UNKNOWN;
  usbd_command = USBD_CMD_UNKNOWN;
  int index = 0;
  char delim[] = " ";
  char *ptr = strtok((char*)usbd_cdc_rx_buf, delim);
  while(ptr != NULL)
  {
    switch(index)
    {
      case 0:
        if (strcmp(ptr, USBD_CMD_FLASH_WRITE_STR) == 0)
        {
          usbd_command = USBD_CMD_FLASH_WRITE;
        }
        else if (strcmp(ptr, USBD_CMD_FLASH_READ_STR) == 0)
        {
          usbd_command = USBD_CMD_FLASH_READ;
        }
        else if (strcmp(ptr, USBD_CMD_FW_UPDATE_START_STR) == 0)
        {
          usbd_command = USBD_CMD_FIRMWARE_UPDATE;
          bFirmwareUpdateFlag = true;
          firmwareUpdateFlashIdx = 0;
        }
        else if (strcmp(ptr, USBD_CMD_FATFS_STR) == 0)
        {
          usbd_command = USBD_CMD_FATFS;
        }
        else if (strcmp(ptr, USBD_CMD_SYSTEM_RST_STR) == 0)
		{
		  HAL_NVIC_SystemReset();
		}
        else if (strcmp(ptr, USBD_BATT_VOLT_STR) == 0)
		{
          battdata = GetBatteryLevelInVolt();
          battlen = sprintf((char*)usbd_cdc_return_buf, "%d", battdata);
		  CDC_Transmit_FS(usbd_cdc_return_buf, battlen);
		}
        else if (strcmp(ptr, USBD_BATT_PCT_STR) == 0)
		{
		  battdata = GetBatteryLevelInPct();
		  battlen = sprintf((char*)usbd_cdc_return_buf, "%d", battdata);
		  CDC_Transmit_FS(usbd_cdc_return_buf, battlen);
		}
        break;
      case 1:
        if (strcmp(ptr, USBD_OP_DEVICE_ID_STR) == 0)
          usbd_options = USBD_OP_DEVICE_ID;
        else if (strcmp(ptr, USBD_OP_NETWORK_ID_STR) == 0)
          usbd_options = USBD_OP_NETWORK_ID;
        else if (strcmp(ptr, USBD_OP_WIFI_CREDENTIALS_STR) == 0)
          usbd_options = USBD_OP_WIFI;
        else if (strcmp(ptr, USBD_OP_HAWARE_VER_STR) == 0)
          usbd_options = USBD_OP_HW_VER;
        else if (strcmp(ptr, USBD_OP_FIRMWARE_VER_STR) == 0)
          usbd_options = USBD_OP_FW_VER;
        else if (strcmp(ptr, USBD_OP_LED_COLOR_STR) == 0)
          usbd_options = USBD_OP_LED_COLOR;
        else if (strcmp(ptr, USBD_OP_LED_BLINK_TIME_STR) == 0)
          usbd_options = USBD_OP_LED_BLINK_TIME;
        else if (strcmp(ptr, USBD_OP_DISTANCE_THRESHOLD_STR) == 0)
          usbd_options = USBD_OP_DISTANCE_THRESHOLD;
        else if (strcmp(ptr, USBD_OP_DISTANCE_TIME_STR) == 0)
          usbd_options = USBD_OP_DISTANCE_TIME;
        else if (strcmp(ptr, USBD_OP_WIFI_SLEEP_TIME_STR) == 0)
          usbd_options = USBD_OP_WIFI_SLEEP_TIME;
        else if (strcmp(ptr, USBD_OP_OTA_FLAG_STR) == 0)
          usbd_options = USBD_OP_OTA_FLAG;
        else if (strcmp(ptr, USBD_OP_ERASE_FLASH_STR) == 0)
          usbd_options = USBD_OP_ERASE_FLASH;
        else if (strcmp(ptr, USBD_OP_ERASE_FLASH_FW_STR) == 0)
          usbd_options = USBD_OP_ERASE_FLASH_FW;
        else if (strcmp(ptr, USBD_OP_FLASH_COM_ID_STR) == 0)
          usbd_options = USBD_OP_COMPANY_ID;
        else if (strcmp(ptr, USBD_OP_FATFS_DELETE_STR) == 0)
          usbd_options = USBD_OP_FATFS_DELETE;
        else if (strcmp(ptr, USBD_OP_DIST_LED_BLINK_TIME_STR) == 0)
          usbd_options = USBD_OP_DIST_LED_BLINK_TIME;
        else if (strcmp(ptr, USBD_OP_BUZZER_STATE_STR) == 0)
		  usbd_options = USBD_OP_BUZZER_STATE;
        else if (strcmp(ptr, USBD_OP_CHG_LED_FREQ_STR) == 0)
		  usbd_options = USBD_OP_CHG_LED_FREQ;
        else if (strcmp(ptr, USBD_OP_CHG_BATT_LOW_STR) == 0)
		  usbd_options = USBD_OP_CHG_BATT_LOW;
        break;

      case 2:
        memcpy(usbd_cdc_params1, ptr, strlen(ptr));
        break;
      case 3:
        memcpy(usbd_cdc_params2, ptr, strlen(ptr));
        break;
      case 4:
        memcpy(usbd_cdc_params3, ptr, strlen(ptr));
        break;
      case 5:
        memcpy(usbd_cdc_params4, ptr, strlen(ptr));
        break;
    }
    index++;
    ptr = strtok(NULL, delim);
  }

  // execute usbd_cdc command
  if (usbd_command == USBD_CMD_FLASH_WRITE)
    usbd_cdc_FlashWrite(usbd_options);
  else if (usbd_command == USBD_CMD_FLASH_READ)
    usbd_cdc_FlashRead(usbd_options);
  else if (usbd_command == USBD_CMD_FATFS)
    usbd_cdc_FileSystem(usbd_options);
}

/**
  * @brief  Execute flash write function
  * @param  None
  * @retval None
  */
void usbd_cdc_FlashWrite(usbd_optionsTypeDef options)
{
  uint8_t pData[8];
  memset(pData, 0, sizeof(pData));
  switch(options)
  {
    case USBD_OP_UNKNOWN:
      break;
    case USBD_OP_DEVICE_ID:
      usbd_cdc_parse_DeviceIdString(pData);
      flash_write_DeviceIdBackUp(pData);
      break;
    case USBD_OP_NETWORK_ID:
      usbd_cdc_parse_16bitData(pData);
      flash_write_NetworkIdBackUp(pData[0] | (pData[1] << 8));
      break;
    case USBD_OP_WIFI:
      // params1: SSID, params2: passwords
      flash_write_WifiCredentialWithBackUp((char*)usbd_cdc_params1, (char*)usbd_cdc_params2);
      break;
    case USBD_OP_HW_VER:
      usbd_cdc_parse_HardwareString(pData);
      flash_write_HardwareVerWithBackUp(pData[0], pData[1], pData[2]);
      break;
    case USBD_OP_FW_VER:
      usbd_cdc_parse_HardwareString(pData);
      flash_write_FirmwareVerWithBackUp(pData[0], pData[1], pData[2]);
      break;
    case USBD_OP_LED_COLOR:
      usbd_cdc_parse_16bitData(pData);
      flash_write_LEDColorWithBackUp(pData[0] | (pData[1] << 8));
      break;
    case USBD_OP_LED_BLINK_TIME:
      usbd_cdc_parse_16bitData(pData);
      flash_write_LEDBlinkWithBackUp(pData[0] | (pData[1] << 8));
      break;
    case USBD_OP_DISTANCE_THRESHOLD:
      usbd_cdc_parse_16bitData(pData);
      flash_write_DistanceThresholdWithBackUp(pData[0] | (pData[1] << 8));
      break;
    case USBD_OP_DISTANCE_TIME:
      usbd_cdc_parse_16bitData(pData);
      flash_write_DistanceToggleTimeWithBackUp(pData[0] | (pData[1] << 8));
      break;
    case USBD_OP_WIFI_SLEEP_TIME:
      usbd_cdc_parse_16bitData(pData);
      flash_write_WiFiSleepIntervalWithBackUp(pData[0] | (pData[1] << 8));
      break;
    case USBD_OP_OTA_FLAG:
      flash_write_UpdateFirmwareFlags();
      break;
    case USBD_OP_ERASE_FLASH:
      flash_erase_BackUp();
      break;
    case USBD_OP_ERASE_FLASH_FW:
      flash_erase_OTAFirmware();
      break;
    case USBD_OP_COMPANY_ID:
      usbd_cdc_parse_16bitData(pData);
      flash_write_CompanyIDWithBackUp(pData[0] | (pData[1] << 8));
      break;
    case USBD_OP_DIST_LED_BLINK_TIME:
          usbd_cdc_parse_16bitData(pData);
          flash_write_DistLEDBlinkWithBackUp(pData[0] | (pData[1] << 8));
          break;
    case USBD_OP_BUZZER_STATE:
              usbd_cdc_parse_16bitData(pData);
              flash_write_BuzzStateWithBackUp(pData[0] | (pData[1] << 8));
              break;
    case USBD_OP_CHG_LED_FREQ:
			  usbd_cdc_parse_16bitData(pData);
			  flash_write_ChgLEDWithBackUp(pData[0] | (pData[1] << 8));
			  break;
    case USBD_OP_CHG_BATT_LOW:
			  usbd_cdc_parse_16bitData(pData);
			  flash_write_BattLowWithBackUp(pData[0] | (pData[1] << 8));
			  break;
    case USBD_OP_FATFS_DELETE:
      break;
  }
}

/**
  * @brief  Execute flash read function
  * @param  None
  * @retval None
  */
void usbd_cdc_FlashRead(usbd_optionsTypeDef options)
{
  memset(usbd_cdc_data1, '\0', USBD_CDC_PARAMS_LEN);
  memset(usbd_cdc_data2, '\0', USBD_CDC_PARAMS_LEN);
  memset(usbd_cdc_data3, '\0', USBD_CDC_PARAMS_LEN);
  memset(usbd_cdc_data4, '\0', USBD_CDC_PARAMS_LEN);
  memset(usbd_cdc_return_buf, '\0', USBD_CDC_RETURN_LEN);

  uint16_t len = 0;
  uint16_t data = 0;
  switch(options)
  {
    case USBD_OP_UNKNOWN:
      break;
    case USBD_OP_DEVICE_ID:
      flash_read_DeviceId(usbd_cdc_data1);
      uint32_t devId = usbd_cdc_data1[0] + (usbd_cdc_data1[1] << 8) +
          (usbd_cdc_data1[2] << 16) + (usbd_cdc_data1[3] << 24);
      len = sprintf((char*)usbd_cdc_return_buf, "%ld", devId);
      break;
    case USBD_OP_NETWORK_ID:
      flash_read_NetworkId(usbd_cdc_data1);
      data = usbd_cdc_data1[0] + (usbd_cdc_data1[1] << 8);
      len = sprintf((char*)usbd_cdc_return_buf, "%d", data);
      break;
    case USBD_OP_WIFI:
      flash_read_WifiCredentials(usbd_cdc_data1, usbd_cdc_data2);
      len = sprintf((char*)usbd_cdc_return_buf, "%s,%s", usbd_cdc_data1, usbd_cdc_data2);
      break;
    case USBD_OP_HW_VER:
      flash_read_HardwareVersion(usbd_cdc_data1);
      len = sprintf((char*)usbd_cdc_return_buf, "%d.%d.%d", usbd_cdc_data1[0], usbd_cdc_data1[1], usbd_cdc_data1[2]);
      break;
    case USBD_OP_FW_VER:
      flash_read_FirmwareVersion(usbd_cdc_data1);
      len = sprintf((char*)usbd_cdc_return_buf, "%d.%d.%d", usbd_cdc_data1[0], usbd_cdc_data1[1], usbd_cdc_data1[2]);
      break;
    case USBD_OP_LED_COLOR:
      flash_read_LEDColor(usbd_cdc_data1);
      data = usbd_cdc_data1[0] + (usbd_cdc_data1[1] << 8);
      len = sprintf((char*)usbd_cdc_return_buf, "%d", data);
      break;
    case USBD_OP_LED_BLINK_TIME:
      flash_read_LEDBlinkTime(usbd_cdc_data1);
      data = usbd_cdc_data1[0] + (usbd_cdc_data1[1] << 8);
      len = sprintf((char*)usbd_cdc_return_buf, "%d", data);
      break;
    case USBD_OP_DISTANCE_THRESHOLD:
      flash_read_DistanceThreshold(usbd_cdc_data1);
      data = usbd_cdc_data1[0] + (usbd_cdc_data1[1] << 8);
      len = sprintf((char*)usbd_cdc_return_buf, "%d", data);
      break;
    case USBD_OP_DISTANCE_TIME:
      flash_read_DistanceTriggerTime(usbd_cdc_data1);
      data = usbd_cdc_data1[0] + (usbd_cdc_data1[1] << 8);
      len = sprintf((char*)usbd_cdc_return_buf, "%d", data);
      break;
    case USBD_OP_WIFI_SLEEP_TIME:
      flash_read_WiFiSleepInterval(usbd_cdc_data1);
      data = usbd_cdc_data1[0] + (usbd_cdc_data1[1] << 8);
      len = sprintf((char*)usbd_cdc_return_buf, "%d", data);
      break;
    case USBD_OP_OTA_FLAG:
      flash_read_FirmwareUpdateFlag(usbd_cdc_data1);
      len = sprintf((char*)usbd_cdc_return_buf, "0x%x%x%x%x", usbd_cdc_data1[0], usbd_cdc_data1[1], usbd_cdc_data1[2], usbd_cdc_data1[3]);
      break;
    case USBD_OP_ERASE_FLASH:
      break;
    case USBD_OP_ERASE_FLASH_FW:
      break;
    case USBD_OP_COMPANY_ID:
      flash_read_CompanyId(usbd_cdc_data1);
      data = usbd_cdc_data1[0] + (usbd_cdc_data1[1] << 8);
      len = sprintf((char*)usbd_cdc_return_buf, "%d", data);
      break;
    case USBD_OP_DIST_LED_BLINK_TIME:
          flash_read_DistLEDBlinkTime(usbd_cdc_data1);
          data = usbd_cdc_data1[0] + (usbd_cdc_data1[1] << 8);
          len = sprintf((char*)usbd_cdc_return_buf, "%d", data);
          break;
    case USBD_OP_BUZZER_STATE:
              flash_read_BuzzState(usbd_cdc_data1);
              data = usbd_cdc_data1[0] + (usbd_cdc_data1[1] << 8);
              len = sprintf((char*)usbd_cdc_return_buf, "%d", data);
              break;
    case USBD_OP_CHG_LED_FREQ:
			  flash_read_ChgLEDTime(usbd_cdc_data1);
			  data = usbd_cdc_data1[0] + (usbd_cdc_data1[1] << 8);
			  len = sprintf((char*)usbd_cdc_return_buf, "%d", data);
			  break;
    case USBD_OP_CHG_BATT_LOW:
			  flash_read_BattLowFreq(usbd_cdc_data1);
			  data = usbd_cdc_data1[0] + (usbd_cdc_data1[1] << 8);
			  len = sprintf((char*)usbd_cdc_return_buf, "%d", data);
			  break;
    case USBD_OP_FATFS_DELETE:
      break;
  }

  CDC_Transmit_FS(usbd_cdc_return_buf, len);
}


/**
  * @brief  Execute filesystem function from SD CARD
  * @param  None
  * @retval None
  */
void usbd_cdc_FileSystem(uint32_t options)
{
  memset(usbd_cdc_return_buf, '\0', USBD_CDC_RETURN_LEN);

  uint16_t len = 0;

  if (options == USBD_OP_FATFS_DELETE)
  {
    if (filesystem_RemoveFiles(FS_DATA_DIR) != PER_NO_ERROR)
      len = sprintf((char*)usbd_cdc_return_buf, "%s", "Unable to delete Files" );
    else
      len = sprintf((char*)usbd_cdc_return_buf, "%s", "Files are deleted");
  }

  CDC_Transmit_FS(usbd_cdc_return_buf, len);
}

/**
  * @brief  Execute Firmware update function
  * @param  length:  Total length of bytes
  * @retval None
  */
void usbd_cdc_Firmware_Update(uint16_t length)
{
  memset(usbd_cdc_return_buf, '\0', USBD_CDC_RETURN_LEN);

  flash_write_FirmwareUpdate(usbd_cdc_rx_buf, length, firmwareUpdateFlashIdx);
  firmwareUpdateFlashIdx += length;

  uint16_t len = sprintf((char*)usbd_cdc_return_buf, "OK-%d", length);
  CDC_Transmit_FS(usbd_cdc_return_buf, len);
}

/**
  * @brief  Parse device Id string into bytes array, i.e. 0x2713 => 0x13, 0x27
  * @param  None
  * @retval None
  */
void usbd_cdc_parse_DeviceIdString(uint8_t* pData)
{
  uint32_t deviceId = atoi((char*)usbd_cdc_params1);
  pData[0] = deviceId & 0xFF;
  pData[1] = (deviceId >> 8) & 0xFF;
  pData[2] = (deviceId >> 16) & 0xFF;
  pData[3] = (deviceId >> 24) & 0xFF;
}

/**
  * @brief  Parse Hardware string into bytes array, i.e. 1.0.12 => 1, 0, 12
  * @param  None
  * @retval None
  */
void usbd_cdc_parse_HardwareString(uint8_t* pData)
{
  int index = 0;
  char delim[] = ".";
  char *ptr = strtok((char*)usbd_cdc_params1, delim);
  while(ptr != NULL)
  {
    switch(index)
    {
      case 0:
        pData[0] = atoi(ptr);
        break;
      case 1:
        pData[1] = atoi(ptr);
        break;
      case 2:
        pData[2] = atoi(ptr);
        break;
    }
    index++;
    ptr = strtok(NULL, delim);
  }
}

/**
  * @brief  Parse usbd_cdc params with 16 bits length
  * @param  pData:  Pointer of data buffer
  * @retval None
  */
void usbd_cdc_parse_16bitData(uint8_t* pData)
{
  uint16_t data = atoi((char*)usbd_cdc_params1);
  pData[0] = data & 0xFF;
  pData[1] = (data >> 8) & 0xFF;
}



/************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
