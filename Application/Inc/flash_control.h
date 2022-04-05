/**
  ******************************************************************************
  * @file    flash_control.h
  * @author  IBronx MDE team
  * @brief   Peripheral driver header file for flash read/write control
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

#ifndef __FLASH_CONTROL_H_
#define __FLASH_CONTROL_H_

#ifdef __cplusplus
 extern "C" {
#endif

 /* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include <stdbool.h>

 /* Exported types ------------------------------------------------------------*/
#define STM32_FLASH_BASE            0x08000000
#define FLASH_WAITETIME             50000

//FLASH sector address
#define ADDR_FLASH_SECTOR_0         ((uint32_t)0x08000000)   // sector 0, 16kb
#define ADDR_FLASH_SECTOR_1         ((uint32_t)0x08004000)   // sector 1, 16kb
#define ADDR_FLASH_SECTOR_2         ((uint32_t)0x08008000)   // sector 2, 16kb
#define ADDR_FLASH_SECTOR_3         ((uint32_t)0x0800C000)   // sector 3, 16kb
#define ADDR_FLASH_SECTOR_4         ((uint32_t)0x08010000)   // sector 4, 64kb
#define ADDR_FLASH_SECTOR_5         ((uint32_t)0x08020000)   // sector 5, 128kb
#define ADDR_FLASH_SECTOR_6         ((uint32_t)0x08040000)   // sector 6, 128kb
#define ADDR_FLASH_SECTOR_7         ((uint32_t)0x08060000)   // sector 7, 128kb
#define ADDR_FLASH_SECTOR_8         ((uint32_t)0x08080000)   // sector 8, 128kb
#define ADDR_FLASH_SECTOR_9         ((uint32_t)0x080A0000)   // sector 9, 128kb
#define ADDR_FLASH_SECTOR_10        ((uint32_t)0x080C0000)   // sector 10, 128kb
#define ADDR_FLASH_SECTOR_11        ((uint32_t)0x080E0000)   // sector 11, 128kb


#define FLASH_ADDR_SECTOR           	ADDR_FLASH_SECTOR_3
#define FLASH_ADDR_FW_UPDATE        	ADDR_FLASH_SECTOR_6
#define FLASH_ADDR_OTA_ADDRESS      	FLASH_ADDR_FW_UPDATE + 0x30000U
#define FLASH_ADDR_IDX_DEVID       		0
#define FLASH_ADDR_IDX_NETID        	FLASH_ADDR_IDX_DEVID + FLASH_DEVICE_ID_SIZE
#define FLASH_ADDR_IDX_DATE         	FLASH_ADDR_IDX_NETID + FLASH_NETWORK_ID_SIZE
#define FLASH_ADDR_IDX_TIME         	FLASH_ADDR_IDX_DATE + FLASH_DATE_SIZE
#define FLASH_ADDR_IDX_SSID         	FLASH_ADDR_IDX_TIME + FLASH_TIME_SIZE
#define FLASH_ADDR_IDX_PS           	FLASH_ADDR_IDX_SSID + FLASH_WIFI_SSID_SIZE
#define FLASH_ADDR_IDX_LED_COLOR    	FLASH_ADDR_IDX_PS + FLASH_WIFI_PASSWORD_SIZE
#define FLASH_ADDR_IDX_LED_BLINK    	FLASH_ADDR_IDX_LED_COLOR + FLASH_LED_COLOR
#define FLASH_ADDR_IDX_DIST_THRES   	FLASH_ADDR_IDX_LED_BLINK + FLASH_LED_BLINK_SEC
#define FLASH_ADDR_IDX_WIFI_SLEEP   	FLASH_ADDR_IDX_DIST_THRES + FLASH_DISTANCE_THRESHOLD_CM
#define FLASH_ADDR_IDX_DIST_TOGGLE  	FLASH_ADDR_IDX_WIFI_SLEEP + FLASH_WIFI_SLEEP_SEC
#define FLASH_ADDR_IDX_HW_VER       	FLASH_ADDR_IDX_DIST_TOGGLE + FLASH_DISTANCE_TOGGLE_SEC
#define FLASH_ADDR_IDX_FM_VER       	FLASH_ADDR_IDX_HW_VER + FLASH_HARDWARE_VERSION
#define FLASH_ADDR_IDX_COM_ID       	FLASH_ADDR_IDX_FM_VER + FLASH_FIRMWARE_VERSION
#define FLASH_ADDR_IDX_DIST_LED_BLINK	FLASH_ADDR_IDX_COM_ID + FLASH_COMPANYID_SIZE
#define FLASH_ADDR_IDX_BUZZ_STATE		FLASH_ADDR_IDX_DIST_LED_BLINK + FLASH_DISTLED_BLINK_SEC
#define FLASH_ADDR_IDX_CHG_TIME			FLASH_ADDR_IDX_BUZZ_STATE + FLASH_BUZZER_STATE
#define FLASH_ADDR_IDX_BATTLOW_FREQ		FLASH_ADDR_IDX_CHG_TIME + FLASH_CHGLED_TIME_SEC

#define FLASH_DEVICE_ID_SIZE        5
#define FLASH_NETWORK_ID_SIZE       2
#define FLASH_DATE_SIZE             4
#define FLASH_TIME_SIZE             3
#define FLASH_WIFI_SSID_SIZE        20//ESP_WIFI_SSID_SIZE
#define FLASH_WIFI_PASSWORD_SIZE    20//ESP_WIFI_PASSWORD_SIZE
#define FLASH_LED_COLOR             2
#define FLASH_LED_BLINK_SEC         2
#define FLASH_DISTANCE_THRESHOLD_CM 2
#define FLASH_WIFI_SLEEP_SEC        2
#define FLASH_DISTANCE_TOGGLE_SEC   2
#define FLASH_HARDWARE_VERSION      3
#define FLASH_FIRMWARE_VERSION      3
#define FLASH_COMPANYID_SIZE        2
#define FLASH_DISTLED_BLINK_SEC		2//Blinking Interval during Close Contact Size
#define FLASH_BUZZER_STATE			2//Buzzer State to Activate or Deactivate Buzzer during Close Contact
#define FLASH_CHGLED_TIME_SEC		2//Charging LED Frequency
#define FLASH_LOW_BATT_FREQ			2//Frequency for when Battery is Low

#define FLASH_TOTAL_SIZE            FLASH_DEVICE_ID_SIZE + FLASH_NETWORK_ID_SIZE + \
                                     FLASH_DATE_SIZE + FLASH_TIME_SIZE + \
                                     FLASH_WIFI_SSID_SIZE + FLASH_WIFI_PASSWORD_SIZE + \
                                     FLASH_LED_COLOR + FLASH_LED_BLINK_SEC + \
                                     FLASH_DISTANCE_THRESHOLD_CM + FLASH_WIFI_SLEEP_SEC + \
                                     FLASH_DISTANCE_TOGGLE_SEC + FLASH_HARDWARE_VERSION + \
                                     FLASH_FIRMWARE_VERSION + FLASH_COMPANYID_SIZE + FLASH_DISTLED_BLINK_SEC + \
									 FLASH_BUZZER_STATE + FLASH_CHGLED_TIME_SEC + FLASH_LOW_BATT_FREQ

 /* Exported constants --------------------------------------------------------*/
 /* Exported macro ------------------------------------------------------------*/
 /* Exported functions ------------------------------------------------------- */

 void flash_read_DeviceId(uint8_t* devId);
 void flash_read_NetworkId(uint8_t* netId);
 void flash_read_RTC(uint8_t *pDate, uint8_t* pTime);
 void flash_read_WifiCredentials(uint8_t* pSSId, uint8_t* pPassword);
 void flash_read_HardwareVersion(uint8_t* pVersion);
 void flash_read_FirmwareVersion(uint8_t* pVersion);
 void flash_read_LEDColor(uint8_t* pColor);
 void flash_read_LEDBlinkTime(uint8_t* pInterval);
 void flash_read_DistanceThreshold(uint8_t* pThreshold);
 void flash_read_DistanceTriggerTime(uint8_t* pInterval);
 void flash_read_WiFiSleepInterval(uint8_t* pInterval);
 void flash_read_FirmwareUpdateFlag(uint8_t* pFlags);
 void flash_read_CompanyId(uint8_t* pCompanyId);
 void flash_read_DistLEDBlinkTime(uint8_t* pDistInterval);
 void flash_read_BuzzState(uint8_t* pBuzzState);
 void flash_read_ChgLEDTime(uint8_t* pChgTime);
 void flash_read_BattLowFreq(uint8_t* pBattLFreq);

 void flash_write_DeviceIdBackUp(uint8_t* pDevId);
 void flash_write_NetworkIdBackUp(uint16_t networkId);
 void flash_write_RTCWithBackUp(RTC_DateTypeDef sDate, RTC_TimeTypeDef sTime);
 void flash_write_WifiCredentialWithBackUp(char* ssid, char* password);
 void flash_write_HardwareVerWithBackUp(uint8_t majorVer, uint8_t minorVer, uint8_t patchVer);
 void flash_write_FirmwareVerWithBackUp(uint8_t majorVer, uint8_t minorVer, uint8_t patchVer);
 void flash_write_LEDColorWithBackUp(uint16_t led_color);
 void flash_write_LEDBlinkWithBackUp(uint16_t led_blink_sec);
 void flash_write_DistanceThresholdWithBackUp(uint16_t distance_threshold_cm);
 void flash_write_DistanceToggleTimeWithBackUp(uint16_t distance_toggle_sec);
 void flash_write_WiFiSleepIntervalWithBackUp(uint16_t wifi_sleep_sec);
 void flash_write_UpdateFirmwareFlags(void);
 void flash_write_FirmwareUpdate(uint8_t* pData, uint16_t length, uint32_t flashAddrIdx);
 void flash_erase_OTAFirmware(void);
 void flash_erase_BackUp(void);
 void flash_write_CompanyIDWithBackUp(uint16_t companyId);
 void flash_write_DistLEDBlinkWithBackUp(uint16_t dist_led_blink_sec);
 void flash_write_BuzzStateWithBackUp(uint16_t buzz_state);
 void flash_write_ChgLEDWithBackUp(uint16_t chg_led_time);
 void flash_write_BattLowWithBackUp(uint16_t batt_low_freq);

 uint8_t flash_GetSectorId(uint32_t addrs);
 void flash_EraseSector(uint8_t sectorNum);
 void flash_ReadByte(uint32_t flashAddress, void *rBuf, uint32_t size);
 void flash_WriteByte(uint32_t flashAddr, void *wBuf, uint32_t size, bool bSectorErase);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_CONTROL_H_ */

 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
