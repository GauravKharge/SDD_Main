/**
  ******************************************************************************
  * @file    flash_control.c
  * @author  IBronx MDE team
  * @brief   Peripheral driver for flash read/write control
  *          This file provides firmware utility functions to support flash read/
  *          write control functions
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
#include "flash_control.h"

#include "string.h"
/* Private typedef -----------------------------------------------------------*/
uint8_t flash_memory[FLASH_TOTAL_SIZE];

extern RTC_HandleTypeDef hrtc;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* function prototypes -------------------------------------------------------*/


/**
  * @brief  Store the Device Id into flash with back-up previous data
  *         Back-up --> Erase --> Write to flash
  * @param  deviceId:   Social distance device Id
  * @retval None
  */
void flash_write_DeviceIdBackUp(uint8_t* pDevId)
{
  flash_ReadByte(FLASH_ADDR_SECTOR , flash_memory, FLASH_TOTAL_SIZE);

  memcpy(&flash_memory[FLASH_ADDR_IDX_DEVID], pDevId, sizeof(uint8_t)*FLASH_DEVICE_ID_SIZE);

  flash_WriteByte(FLASH_ADDR_SECTOR, flash_memory, FLASH_TOTAL_SIZE, true);
}

/**
  * @brief  Store the Network Id into flash with back-up previous data
  *         Back-up --> Erase --> Write to flash
  * @param  networkId:  DWM1000 Message Network Id
  * @retval None
  */
void flash_write_NetworkIdBackUp(uint16_t networkId)
{
  flash_ReadByte(FLASH_ADDR_SECTOR , flash_memory, FLASH_TOTAL_SIZE);

  flash_memory[FLASH_ADDR_IDX_NETID] = (uint8_t)(networkId & 0xFF);
  flash_memory[FLASH_ADDR_IDX_NETID + 1] = (uint8_t)((networkId >> 8) & 0xFF);

  flash_WriteByte(FLASH_ADDR_SECTOR, flash_memory, FLASH_TOTAL_SIZE, true);
}

/**
  * @brief  Store the RTC into flash with back-up previous data
  *         Back-up --> Erase --> Write to flash
  * @param  sDate:  RTC Date
  * @param  sTime:  RTC Time
  * @retval None
  */
void flash_write_RTCWithBackUp(RTC_DateTypeDef sDate, RTC_TimeTypeDef sTime)
{
  flash_ReadByte(FLASH_ADDR_SECTOR , flash_memory, FLASH_TOTAL_SIZE);

  flash_memory[FLASH_ADDR_IDX_DATE] = sDate.Year;
  flash_memory[FLASH_ADDR_IDX_DATE + 1] = sDate.Month;
  flash_memory[FLASH_ADDR_IDX_DATE + 2] = sDate.Date;
  flash_memory[FLASH_ADDR_IDX_DATE + 3] = sDate.WeekDay;

  flash_memory[FLASH_ADDR_IDX_TIME] = sTime.Hours;
  flash_memory[FLASH_ADDR_IDX_TIME + 1] = sTime.Minutes;
  flash_memory[FLASH_ADDR_IDX_TIME + 2] = sTime.Seconds;

  flash_WriteByte(FLASH_ADDR_SECTOR, flash_memory, FLASH_TOTAL_SIZE, true);
}

/**
  * @brief  Store the wifi credential info into flash with back-up previous data
  *         Back-up --> Erase --> Write to flash
  * @param  ssid:      WiFi SSID string, total string size is 20 chars
  * @param  password:  WiFi Password string, total string size is 20 chars
  * @retval None
  */
void flash_write_WifiCredentialWithBackUp(char* ssid, char* password)
{
  flash_ReadByte(FLASH_ADDR_SECTOR , flash_memory, FLASH_TOTAL_SIZE);

  for(int idx = 0; idx < FLASH_WIFI_SSID_SIZE; idx++)
  {
    if (idx <= strlen(ssid))
      flash_memory[FLASH_ADDR_IDX_SSID + idx] = ssid[idx];
  }

  for(int kdx = 0; kdx < FLASH_WIFI_PASSWORD_SIZE; kdx++)
  {
    if (kdx <= strlen(password))
      flash_memory[FLASH_ADDR_IDX_PS + kdx] = password[kdx];
  }

  flash_WriteByte(FLASH_ADDR_SECTOR, flash_memory, FLASH_TOTAL_SIZE, true);
}

/**
  * @brief  Store the Hardware version into flash with back-up previous data
  *         Back-up --> Erase --> Write to flash
  * @param  majorVer:  Major version
  * @param  minorVer:  Minor version
  * @param  patchVer:  Patch version
  * @retval None
  */
void flash_write_HardwareVerWithBackUp(uint8_t majorVer, uint8_t minorVer, uint8_t patchVer)
{
  flash_ReadByte(FLASH_ADDR_SECTOR , flash_memory, FLASH_TOTAL_SIZE);
  flash_memory[FLASH_ADDR_IDX_HW_VER] = majorVer;
  flash_memory[FLASH_ADDR_IDX_HW_VER + 1] = minorVer;
  flash_memory[FLASH_ADDR_IDX_HW_VER + 2] = patchVer;

  flash_WriteByte(FLASH_ADDR_SECTOR, flash_memory, FLASH_TOTAL_SIZE, true);
}

/**
  * @brief  Store the Firmware version into flash with back-up previous data
  *         Back-up --> Erase --> Write to flash
  * @param  majorVer:  Major version
  * @param  minorVer:  Minor version
  * @param  patchVer:  Patch version
  * @retval None
  */
void flash_write_FirmwareVerWithBackUp(uint8_t majorVer, uint8_t minorVer, uint8_t patchVer)
{
  flash_ReadByte(FLASH_ADDR_SECTOR , flash_memory, FLASH_TOTAL_SIZE);
  flash_memory[FLASH_ADDR_IDX_FM_VER] = majorVer;
  flash_memory[FLASH_ADDR_IDX_FM_VER + 1] = minorVer;
  flash_memory[FLASH_ADDR_IDX_FM_VER + 2] = patchVer;

  flash_WriteByte(FLASH_ADDR_SECTOR, flash_memory, FLASH_TOTAL_SIZE, true);
}

/**
  * @brief  Store the LED Color Index into flash with back-up previous data
  *         Back-up --> Erase --> Write to flash
  * @param  led_color:  LED color Index
  * @retval None
  */
void flash_write_LEDColorWithBackUp(uint16_t led_color)
{
  flash_ReadByte(FLASH_ADDR_SECTOR , flash_memory, FLASH_TOTAL_SIZE);
  flash_memory[FLASH_ADDR_IDX_LED_COLOR] = (uint8_t)(led_color & 0xFF);
  flash_memory[FLASH_ADDR_IDX_LED_COLOR + 1] = (uint8_t)((led_color >> 8) & 0xFF);

  flash_WriteByte(FLASH_ADDR_SECTOR, flash_memory, FLASH_TOTAL_SIZE, true);
}

/**
  * @brief  Store the LED display time when no event happen into flash with back-up previous data
  *         Back-up --> Erase --> Write to flash
  * @param  led_blink_sec:  LED blink time in seconds
  * @retval None
  */
void flash_write_LEDBlinkWithBackUp(uint16_t led_blink_sec)
{
  flash_ReadByte(FLASH_ADDR_SECTOR , flash_memory, FLASH_TOTAL_SIZE);
  flash_memory[FLASH_ADDR_IDX_LED_BLINK] = (uint8_t)(led_blink_sec & 0xFF);
  flash_memory[FLASH_ADDR_IDX_LED_BLINK + 1] = (uint8_t)((led_blink_sec >> 8) & 0xFF);

  flash_WriteByte(FLASH_ADDR_SECTOR, flash_memory, FLASH_TOTAL_SIZE, true);
}

/**
  * @brief  Store the distance threshold to trigger close contact into flash with back-up previous data
  *         Back-up --> Erase --> Write to flash
  * @param  distance_threshold_cm:  Distance threshold in cm
  * @retval None
  */
void flash_write_DistanceThresholdWithBackUp(uint16_t distance_threshold_cm)
{
  flash_ReadByte(FLASH_ADDR_SECTOR , flash_memory, FLASH_TOTAL_SIZE);
  flash_memory[FLASH_ADDR_IDX_DIST_THRES] = (uint8_t)(distance_threshold_cm & 0xFF);
  flash_memory[FLASH_ADDR_IDX_DIST_THRES + 1] = (uint8_t)((distance_threshold_cm >> 8) & 0xFF);

  flash_WriteByte(FLASH_ADDR_SECTOR, flash_memory, FLASH_TOTAL_SIZE, true);
}

/**
  * @brief  Store the distance toggle close contact interval time into flash with back-up previous data
  *         Back-up --> Erase --> Write to flash
  * @param  distance_toggle_sec:  Distance toggle close contact event interval time in seconds
  * @retval None
  */
void flash_write_DistanceToggleTimeWithBackUp(uint16_t distance_toggle_sec)
{
  flash_ReadByte(FLASH_ADDR_SECTOR , flash_memory, FLASH_TOTAL_SIZE);
  flash_memory[FLASH_ADDR_IDX_DIST_TOGGLE] = (uint8_t)(distance_toggle_sec & 0xFF);
  flash_memory[FLASH_ADDR_IDX_DIST_TOGGLE + 1] = (uint8_t)((distance_toggle_sec >> 8) & 0xFF);

  flash_WriteByte(FLASH_ADDR_SECTOR, flash_memory, FLASH_TOTAL_SIZE, true);
}

/**
  * @brief  Store the WiFi deep-sleep interval time into flash with back-up previous data
  *         Back-up --> Erase --> Write to flash
  * @param  wifi_sleep_sec:  WiFi deep-sleep interval time in seconds
  * @retval None
  */
void flash_write_WiFiSleepIntervalWithBackUp(uint16_t wifi_sleep_sec)
{
  flash_ReadByte(FLASH_ADDR_SECTOR , flash_memory, FLASH_TOTAL_SIZE);
  flash_memory[FLASH_ADDR_IDX_WIFI_SLEEP] = (uint8_t)(wifi_sleep_sec & 0xFF);
  flash_memory[FLASH_ADDR_IDX_WIFI_SLEEP + 1] = (uint8_t)((wifi_sleep_sec >> 8) & 0xFF);

  flash_WriteByte(FLASH_ADDR_SECTOR, flash_memory, FLASH_TOTAL_SIZE, true);
}

/**
  * @brief  Store the company id into flash with back-up previous data
  *         Back-up --> Erase --> Write to flash
  * @param  companyId:  Company ID
  * @retval None
  */
void flash_write_CompanyIDWithBackUp(uint16_t companyId)
{
  flash_ReadByte(FLASH_ADDR_SECTOR , flash_memory, FLASH_TOTAL_SIZE);
  flash_memory[FLASH_ADDR_IDX_COM_ID] = (uint8_t)(companyId & 0xFF);
  flash_memory[FLASH_ADDR_IDX_COM_ID + 1] = (uint8_t)((companyId >> 8) & 0xFF);

  flash_WriteByte(FLASH_ADDR_SECTOR, flash_memory, FLASH_TOTAL_SIZE, true);
}

/**
  * @brief  Update the OTA firmware update flags as 0xAAAAAAAA, once bootloader read the flags,
  *         it will copy the APP2 to APP1 and then jump to APP1 to start the program
  * @param  None
  * @retval None
  */
void flash_write_UpdateFirmwareFlags(void)
{
  uint8_t ota_flags[4];
  memset(ota_flags, 0xAA, sizeof(uint8_t)*4);

  flash_WriteByte((FLASH_ADDR_OTA_ADDRESS - 4) , ota_flags, 4, false);
}

/**
  * @brief  Firmware update to flash
  * @param  pData:         Memory point buffer
  * @param  length:        Total memory buffer length
  * @param  flashAddrIdx:  Flash Address Index
  * @retval None
  */
void flash_write_FirmwareUpdate(uint8_t* pData, uint16_t length, uint32_t flashAddrIdx)
{
  flash_WriteByte(FLASH_ADDR_FW_UPDATE +  flashAddrIdx, pData, length, false);
}



/**
  * @brief  Store the LED display time when event happens into flash with back-up previous data
  *         Back-up --> Erase --> Write to flash
  * @param  dist_led_blink_sec: Distance LED blink time in seconds
  * @retval None
  */
void flash_write_DistLEDBlinkWithBackUp(uint16_t dist_led_blink_sec)
{
  flash_ReadByte(FLASH_ADDR_SECTOR , flash_memory, FLASH_TOTAL_SIZE);
  flash_memory[FLASH_ADDR_IDX_DIST_LED_BLINK] = (uint8_t)(dist_led_blink_sec & 0xFF);
  flash_memory[FLASH_ADDR_IDX_DIST_LED_BLINK + 1] = (uint8_t)((dist_led_blink_sec >> 8) & 0xFF);

  flash_WriteByte(FLASH_ADDR_SECTOR, flash_memory, FLASH_TOTAL_SIZE, true);
}

/**
  * @brief  Store the Buzzer State Active/DeActive when event happens into flash with back-up previous data
  *         Back-up --> Erase --> Write to flash
  * @param  buzz_state: Buzzer  State
  * @retval None
  */
void flash_write_BuzzStateWithBackUp(uint16_t buzz_state)
{
  flash_ReadByte(FLASH_ADDR_SECTOR , flash_memory, FLASH_TOTAL_SIZE);
  flash_memory[FLASH_ADDR_IDX_BUZZ_STATE] = (uint8_t)(buzz_state & 0xFF);
  flash_memory[FLASH_ADDR_IDX_BUZZ_STATE + 1] = (uint8_t)((buzz_state >> 8) & 0xFF);

  flash_WriteByte(FLASH_ADDR_SECTOR, flash_memory, FLASH_TOTAL_SIZE, true);
}

/**
  * @brief  Store the Charge LED Frequency during charging into flash with back-up previous data
  *         Back-up --> Erase --> Write to flash
  * @param  chg_led_time: charging LED Frequency in Seconds
  * @retval None
  */
void flash_write_ChgLEDWithBackUp(uint16_t chg_led_time)
{
  flash_ReadByte(FLASH_ADDR_SECTOR , flash_memory, FLASH_TOTAL_SIZE);
  flash_memory[FLASH_ADDR_IDX_CHG_TIME] = (uint8_t)(chg_led_time & 0xFF);
  flash_memory[FLASH_ADDR_IDX_CHG_TIME + 1] = (uint8_t)((chg_led_time >> 8) & 0xFF);

  flash_WriteByte(FLASH_ADDR_SECTOR, flash_memory, FLASH_TOTAL_SIZE, true);
}

/**
  * @brief  Store the Low Batt Blinking Freq into flash with back-up previous data
  *         Back-up --> Erase --> Write to flash
  * @param  batt_low_freq: Low Battery LED Frequency in Seconds
  * @retval None
  */
void flash_write_BattLowWithBackUp(uint16_t batt_low_freq)
{
  flash_ReadByte(FLASH_ADDR_SECTOR , flash_memory, FLASH_TOTAL_SIZE);
  flash_memory[FLASH_ADDR_IDX_BATTLOW_FREQ] = (uint8_t)(batt_low_freq & 0xFF);
  flash_memory[FLASH_ADDR_IDX_BATTLOW_FREQ + 1] = (uint8_t)((batt_low_freq >> 8) & 0xFF);

  flash_WriteByte(FLASH_ADDR_SECTOR, flash_memory, FLASH_TOTAL_SIZE, true);
}

/**
  * @brief  Read the device id from flash memory
  * @param  devId:  Flash memory buffer pointer for device id
  * @retval None
  */
void flash_read_DeviceId(uint8_t* devId)
{
  flash_ReadByte(FLASH_ADDR_SECTOR + FLASH_ADDR_IDX_DEVID, devId, FLASH_DEVICE_ID_SIZE);
}

/**
  * @brief  Read the network id from flash memory
  * @param  devId:  Flash memory buffer pointer for network id
  * @retval None
  */
void flash_read_NetworkId(uint8_t* netId)
{
  flash_ReadByte(FLASH_ADDR_SECTOR + FLASH_ADDR_IDX_NETID, netId, FLASH_NETWORK_ID_SIZE);
}

/**
  * @brief  Read the RTC from flash memory
  * @param  pDate:  Pointer of RTC Date memory buffer
  * @param  pTime:  Pointer of RTC Time memory buffer
  * @retval None
  */
void flash_read_RTC(uint8_t *pDate, uint8_t* pTime)
{
  flash_ReadByte(FLASH_ADDR_SECTOR + FLASH_ADDR_IDX_DATE, pDate, FLASH_DATE_SIZE);
  flash_ReadByte(FLASH_ADDR_SECTOR + FLASH_ADDR_IDX_TIME, pTime, FLASH_TIME_SIZE);
}

/**
  * @brief  Read the AccessPoint SSID from flash
  * @param  pSSId:  Pointer of SSID memory buffer
  * @retval None
  */
void flash_read_WifiCredentials(uint8_t* pSSId, uint8_t* pPassword)
{
  flash_ReadByte(FLASH_ADDR_SECTOR + FLASH_ADDR_IDX_SSID, pSSId, FLASH_WIFI_SSID_SIZE);
  flash_ReadByte(FLASH_ADDR_SECTOR + FLASH_ADDR_IDX_PS, pPassword, FLASH_WIFI_PASSWORD_SIZE);
}

/**
  * @brief  Get flash sector number based on address
  * @param  addrs:      Flash memory address
  * @retval sector Id:  Flash memory sector Id
  */
void flash_read_HardwareVersion(uint8_t* pVersion)
{
  flash_ReadByte(FLASH_ADDR_SECTOR + FLASH_ADDR_IDX_HW_VER, pVersion, FLASH_HARDWARE_VERSION);
}

/**
  * @brief  Read the Firmware version from flash
  * @param  pVersion:  Pointer of Firmware Version, e.g. 1.00.00
  * @retval None
  */
void flash_read_FirmwareVersion(uint8_t* pVersion)
{
  flash_ReadByte(FLASH_ADDR_SECTOR + FLASH_ADDR_IDX_FM_VER, pVersion, FLASH_FIRMWARE_VERSION);
}

/**
  * @brief  Read the LED color from flash
  * @param  pIndex:  Pointer of LED color index, e.g. Index = 1 ==> Red Color
  * @retval None
  */
void flash_read_LEDColor(uint8_t* pColor)
{
  flash_ReadByte(FLASH_ADDR_SECTOR + FLASH_ADDR_IDX_LED_COLOR, pColor, FLASH_LED_COLOR);
}

/**
  * @brief  Read the LED blink interval time when no event happen from flash
  * @param  pInterval:  Pointer of time interval in seconds
  * @retval None
  */
void flash_read_LEDBlinkTime(uint8_t* pInterval)
{
  flash_ReadByte(FLASH_ADDR_SECTOR + FLASH_ADDR_IDX_LED_BLINK, pInterval, FLASH_LED_BLINK_SEC);
}


/**
  * @brief  Read the distance threshold in cm from flash
  * @param  pThreshold:  Pointer of distance threshold in cm
  * @retval None
  */
void flash_read_DistanceThreshold(uint8_t* pThreshold)
{
  flash_ReadByte(FLASH_ADDR_SECTOR + FLASH_ADDR_IDX_DIST_THRES, pThreshold, FLASH_DISTANCE_THRESHOLD_CM);
}

/**
  * @brief  Read the distance close contact trigger event time in milliseconds from flash
  * @param  pInterval:  Pointer of trigger event interval in milliseconds
  * @retval None
  */
void flash_read_DistanceTriggerTime(uint8_t* pInterval)
{
  flash_ReadByte(FLASH_ADDR_SECTOR + FLASH_ADDR_IDX_DIST_TOGGLE, pInterval, FLASH_DISTANCE_TOGGLE_SEC);
}

/**
  * @brief  Read the WiFi deep-sleep interval time in seconds from flash
  * @param  pInterval:  Pointer of WiFi deep-sleep interval in milliseconds
  * @retval None
  */
void flash_read_WiFiSleepInterval(uint8_t* pInterval)
{
  flash_ReadByte(FLASH_ADDR_SECTOR + FLASH_ADDR_IDX_WIFI_SLEEP, pInterval, FLASH_WIFI_SLEEP_SEC);
}

/**
  * @brief  Read the company Id from flash
  * @param  pInterval:  Pointer of company Id
  * @retval None
  */
void flash_read_CompanyId(uint8_t* pCompanyId)
{
  flash_ReadByte(FLASH_ADDR_SECTOR + FLASH_ADDR_IDX_COM_ID, pCompanyId, FLASH_COMPANYID_SIZE);
}

/**
  * @brief  Read the firmware update flag which use for OTA firmware update
  * @param  pFlags:  Pointer of firmware update flag
  * @retval None
  */
void flash_read_FirmwareUpdateFlag(uint8_t* pFlags)
{
  flash_ReadByte((FLASH_ADDR_OTA_ADDRESS - 4), pFlags, 4);
}


/**
  * @brief  Read the Distance LED blink interval time when events happen from flash
  * @param  pDistInterval:  Pointer of time interval in seconds
  * @retval None
  */
void flash_read_DistLEDBlinkTime(uint8_t* pDistInterval)
{
  flash_ReadByte(FLASH_ADDR_SECTOR + FLASH_ADDR_IDX_DIST_LED_BLINK, pDistInterval, FLASH_DISTLED_BLINK_SEC);
}

/**
  * @brief  Read the Buzzer State when events happen from flash
  * @param  pDistInterval:  Pointer of time interval in seconds
  * @retval None
  */
void flash_read_BuzzState(uint8_t* pBuzzState)
{
  flash_ReadByte(FLASH_ADDR_SECTOR + FLASH_ADDR_IDX_BUZZ_STATE, pBuzzState, FLASH_BUZZER_STATE);
}

/**
  * @brief  Read the Buzzer State when events happen from flash
  * @param  pDistInterval:  Pointer of time interval in seconds
  * @retval None
  */
void flash_read_ChgLEDTime(uint8_t* pChgTime)
{
  flash_ReadByte(FLASH_ADDR_SECTOR + FLASH_ADDR_IDX_CHG_TIME, pChgTime, FLASH_CHGLED_TIME_SEC);
}

/**
  * @brief  Read the Buzzer State when events happen from flash
  * @param  pDistInterval:  Pointer of time interval in seconds
  * @retval None
  */
void flash_read_BattLowFreq(uint8_t* pBattLFreq)
{
  flash_ReadByte(FLASH_ADDR_SECTOR + FLASH_ADDR_IDX_BATTLOW_FREQ, pBattLFreq, FLASH_LOW_BATT_FREQ);
}

/**
  * @brief  Erase the OTA flash area
  * @param  None
  * @retval None
  */
void flash_erase_OTAFirmware(void)
{
  flash_EraseSector(flash_GetSectorId(ADDR_FLASH_SECTOR_6));

  // do wait last operation
  FLASH_WaitForLastOperation(FLASH_WAITETIME);

  flash_EraseSector(flash_GetSectorId(ADDR_FLASH_SECTOR_7));
}

/**
  * @brief  Erase the Data backup flash area
  * @param  None
  * @retval None
  */
void flash_erase_BackUp(void)
{
  flash_EraseSector(flash_GetSectorId(ADDR_FLASH_SECTOR_3));
}

/**
  * @brief  Get flash sector number based on address
  * @param  addrs:      Flash memory address
  * @retval sector Id:  Flash memory sector Id
  */
uint8_t flash_GetSectorId(uint32_t addrs)
{
  if (addrs < ADDR_FLASH_SECTOR_1)
    return FLASH_SECTOR_0;
  else if (addrs < ADDR_FLASH_SECTOR_2)
    return FLASH_SECTOR_1;
  else if (addrs < ADDR_FLASH_SECTOR_3)
    return FLASH_SECTOR_2;
  else if (addrs < ADDR_FLASH_SECTOR_4)
    return FLASH_SECTOR_3;
  else if (addrs < ADDR_FLASH_SECTOR_5)
    return FLASH_SECTOR_4;
  else if (addrs < ADDR_FLASH_SECTOR_6)
    return FLASH_SECTOR_5;
  else if (addrs < ADDR_FLASH_SECTOR_7)
    return FLASH_SECTOR_6;
  else
    return FLASH_SECTOR_7;
}

/**
  * @brief  Execute flash erase operation for the whole sector based on flash_sectorNum
  * @param  None
  * @retval None
  */
void flash_EraseSector(uint8_t sectorNum)
{
  HAL_FLASH_Unlock();
  FLASH_Erase_Sector(sectorNum, FLASH_VOLTAGE_RANGE_3);
  HAL_FLASH_Lock();
}

/**
  * @brief  Execute flash Write operation, flash size is DATA_TYPE_8
  * @param  flashAddr:     Flash memory address
  * @param  wBuf:          Write byte buffer
  * @param  size:          Write byte size
  * @param  bSectorErase:  Erase whole sector if set true, otherwise set false
  * @retval None
  */
void flash_WriteByte(uint32_t flashAddress, void *wBuf, uint32_t size, bool bSectorErase)
{
  uint8_t sectorNum = flash_GetSectorId(flashAddress);
  uint32_t flashAddr = flashAddress;

  // do wait last operation to solve first try write failed issue
  FLASH_WaitForLastOperation(FLASH_WAITETIME);

  // erase the whole sector before write based on flash_sectorNum
  if (bSectorErase)
    flash_EraseSector(sectorNum);

  HAL_FLASH_Unlock();

  // do wait last operation to solve first try write failed issue
  FLASH_WaitForLastOperation(FLASH_WAITETIME);

  for (uint32_t i = 0; i < size; i++)
  {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, flashAddr , ((uint8_t *)wBuf)[i]);
    flashAddr++;
  }

  HAL_FLASH_Lock();
}

/**
  * @brief  Execute flash Read operation, flash size is DATA_TYPE_8
  * @param  flashAddr:     Flash memory address
  * @param  rBuf:          Read byte buffer
  * @param  size:          Read byte size
  * @retval None
  */
void flash_ReadByte(uint32_t flashAddress, void *rBuf, uint32_t size)
{
  uint32_t flashAddr = flashAddress;

  for (uint32_t i = 0; i < size; i++)
  {
    *((uint8_t *)rBuf + i) = *(uint8_t *)flashAddr;
    flashAddr++;
  }
}


/************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
