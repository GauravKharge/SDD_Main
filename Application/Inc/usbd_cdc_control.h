/**
  ******************************************************************************
  * @file    usbd_cdc_control.h
  * @author  IBronx MDE team
  * @brief   USB device communications device class controls header file
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

#ifndef __USBD_CDC_CONTROL_H_
#define __USBD_CDC_CONTROL_H_

#ifdef __cplusplus
 extern "C" {
#endif

 /* Includes ------------------------------------------------------------------*/
 #include "stm32f4xx_hal.h"

 /* Exported types ------------------------------------------------------------*/
#define USBD_CDC_RX_LEN               1024
#define USBD_CDC_RETURN_LEN           300
#define USBD_CDC_PARAMS_LEN           32

#define USBD_CMD_FLASH_WRITE_STR      "FlashWrite"
#define USBD_CMD_FLASH_READ_STR       "FlashRead"
#define USBD_CMD_FW_UPDATE_START_STR  "FirmwareUpdateStart"
#define USBD_CMD_FW_UPDATE_STOP_STR   "FirmwareUpdateStop"
#define USBD_CMD_FATFS_STR            "FileSystem"
#define USBD_CMD_SYSTEM_RST_STR		  "SystemReset"
#define USBD_BATT_VOLT_STR  			"SystemBattVolt"
#define USBD_BATT_PCT_STR				"SystemBattPCT"


#define USBD_OP_DEVICE_ID_STR          	"-i"
#define USBD_OP_NETWORK_ID_STR         	"-n"
#define USBD_OP_WIFI_CREDENTIALS_STR   	"-w"
#define USBD_OP_HAWARE_VER_STR         	"-h"
#define USBD_OP_FIRMWARE_VER_STR       	"-f"
#define USBD_OP_LED_COLOR_STR          	"-c"
#define USBD_OP_LED_BLINK_TIME_STR     	"-b"
#define USBD_OP_DISTANCE_THRESHOLD_STR 	"-d"
#define USBD_OP_DISTANCE_TIME_STR      	"-t"
#define USBD_OP_WIFI_SLEEP_TIME_STR    	"-s"
#define USBD_OP_OTA_FLAG_STR           	"-o"
#define USBD_OP_ERASE_FLASH_STR        	"-e"
#define USBD_OP_ERASE_FLASH_FW_STR     	"-z"
#define USBD_OP_FLASH_COM_ID_STR       	"-a"
#define USBD_OP_FATFS_DELETE_STR       	"-x"
#define USBD_OP_DIST_LED_BLINK_TIME_STR	"-l"
#define USBD_OP_BUZZER_STATE_STR		"-v"
#define USBD_OP_CHG_LED_FREQ_STR		"-p"
#define USBD_OP_CHG_BATT_LOW_STR		"-j"




 typedef enum
 {
   USBD_CMD_UNKNOWN = 0,
   USBD_CMD_FLASH_WRITE,
   USBD_CMD_FLASH_READ,
   USBD_CMD_FIRMWARE_UPDATE,
   USBD_CMD_FATFS,
 }usbd_cmdTypeDef;

 typedef enum
 {
   USBD_OP_UNKNOWN = 0,
   USBD_OP_DEVICE_ID,
   USBD_OP_NETWORK_ID,
   USBD_OP_WIFI,
   USBD_OP_HW_VER,
   USBD_OP_FW_VER,
   USBD_OP_LED_COLOR,
   USBD_OP_LED_BLINK_TIME,
   USBD_OP_DISTANCE_THRESHOLD,
   USBD_OP_DISTANCE_TIME,
   USBD_OP_WIFI_SLEEP_TIME,
   USBD_OP_OTA_FLAG,
   USBD_OP_ERASE_FLASH,
   USBD_OP_ERASE_FLASH_FW,
   USBD_OP_COMPANY_ID,
   USBD_OP_FATFS_DELETE,
   USBD_OP_DIST_LED_BLINK_TIME,
   USBD_OP_BUZZER_STATE,
   USBD_OP_CHG_LED_FREQ,
   USBD_OP_CHG_BATT_LOW,
 }usbd_optionsTypeDef;

 /* Exported constants --------------------------------------------------------*/
 /* Exported macro ------------------------------------------------------------*/
 /* Exported functions ------------------------------------------------------- */
 void usbd_cdc_RxCallbackHandler(uint8_t* Buf, uint16_t Len);
 void usbd_cdc_FlashWrite(usbd_optionsTypeDef options);
 void usbd_cdc_FlashRead(usbd_optionsTypeDef options);
 void usbd_cdc_FileSystem(uint32_t options);
 void usbd_cdc_parse_DeviceIdString(uint8_t* pData);
 void usbd_cdc_parse_HardwareString(uint8_t* pData);
 void usbd_cdc_parse_16bitData(uint8_t* pData);
 void usbd_cdc_Firmware_Update(uint16_t length);

#ifdef __cplusplus
}
#endif


#endif /* __USBD_CDC_CONTROL_H_ */


 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
