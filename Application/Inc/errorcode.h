/**
  ******************************************************************************
  * @file    errorcode.h
  * @author  IBronx MDE team
  * @brief   Peripheral error codes header file
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
#ifndef __ERRORCODE_H_
#define __ERRORCODE_H_

#ifdef __cplusplus
 extern "C" {
#endif

 /* Includes ------------------------------------------------------------------*/
 /* Exported types ------------------------------------------------------------*/
#define PER_ERROR_BASE_NUM      (0x0)       ///< Global error base
#define PER_ERROR_APP_NUM       (0x1000)
//#define PER_ERROR_SDM_BASE_NUM  (0x1000)    ///< SDM error base
//#define PER_ERROR_SOC_BASE_NUM  (0x2000)    ///< SoC error base
//#define PER_ERROR_STK_BASE_NUM  (0x3000)    ///< STK error base


#define PER_NO_ERROR                          (PER_ERROR_BASE_NUM + 0)  ///< Successful command
#define PER_ERROR_INIT                        (PER_ERROR_BASE_NUM + 1)  ///< Failed during initializations
#define PER_ERROR_TIM_BASE_INIT               (PER_ERROR_BASE_NUM + 2)  ///< Failed to initialize TIMER peripheral module
#define PER_ERROR_TIM_BASE_PWM_INIT           (PER_ERROR_BASE_NUM + 3)  ///< Failed to initialize TIMER PWM peripheral module
#define PER_ERROR_TIM_CONFIG_CLK              (PER_ERROR_BASE_NUM + 4)  ///< Failed to configure TIMER clock source
#define PER_ERROR_TIM_OC_CONFIG_CHANNEL       (PER_ERROR_BASE_NUM + 5)  ///< Failed to configure TIMER Output compare channel
#define PER_ERROR_TIM_CONFIG_BREAK_DEAD_TIME  (PER_ERROR_BASE_NUM + 6)  ///< Failed to configure TIMER break feature, dead time and so on
#define PER_ERROR_TIM_ENCODER_INIT            (PER_ERROR_BASE_NUM + 7)  ///< Failed to initialize TIMER Encoder peripheral module
#define PER_ERROR_TIM_CONFIG_SYNC             (PER_ERROR_BASE_NUM + 8)  ///< Failed to configure TIMER Synchronization
#define PER_ERROR_PWM_CONFIG_CHANNEL          (PER_ERROR_BASE_NUM + 9)  ///< Failed to configure PWM channel
#define PER_ERROR_ADC_INIT                    (PER_ERROR_BASE_NUM + 10) ///< Failed to initialize ADC peripheral module
#define PER_ERROR_ADC_CONFIG_CHANNEL          (PER_ERROR_BASE_NUM + 11) ///< Failed to configured ADC channel
#define PER_ERROR_ADC_CHANNEL_NOT_SUPPORTED   (PER_ERROR_BASE_NUM + 12) ///< Not supported ADC channel
#define PER_ERROR_DMA_INIT                    (PER_ERROR_BASE_NUM + 13) ///< Failed to initialize DMA peripheral module
#define PER_ERROR_USART_INIT                  (PER_ERROR_BASE_NUM + 14) ///< Failed to initialize USART peripheral module
#define PER_ERROR_CAN_INIT                    (PER_ERROR_BASE_NUM + 15) ///< Failed to initialize CAN peripheral module
#define PER_ERROR_CAN_CONFIG_FILTER           (PER_ERROR_BASE_NUM + 16) ///< Failed to configure CAN reception filters
#define PER_ERROR_CAN_START_PERIPHERAL        (PER_ERROR_BASE_NUM + 17) ///< Failed to start CAN peripheral
#define PER_ERROR_CAN_ENABLE_INTERRUPT        (PER_ERROR_BASE_NUM + 18) ///< Failed to enable CAN Rx/Tx interrupt
#define PER_ERROR_CAN_SEND_MESSAGE            (PER_ERROR_BASE_NUM + 19) ///< Failed to send CAN message
#define PER_ERROR_CAN_RECEIVE_MESSAGE         (PER_ERROR_BASE_NUM + 20) ///< Failed to receive CAN message
#define PER_ERROR_I2C_INIT                    (PER_ERROR_BASE_NUM + 21) ///< Failed to initialize I2C peripheral module
#define PER_ERROR_I2C_TRANSMIT_COMMAND        (PER_ERROR_BASE_NUM + 22) ///< Failed to transmit I2C command
#define PER_ERROR_I2C_RECEIVE_DATA            (PER_ERROR_BASE_NUM + 23) ///< Failed to receive I2C data
#define PER_ERROR_SPI_INIT                    (PER_ERROR_BASE_NUM + 24) ///< Failed to initialize SPI peripheral module
#define PER_ERROR_TIMER_NOT_AVAILABLE         (PER_ERROR_BASE_NUM + 25) ///< Software timer is not available
#define PER_ERROR_RTC_INIT                    (PER_ERROR_BASE_NUM + 26) ///< Failed to initialize RTC peripheral module
#define PER_ERROR_RTC_SET_DATE                (PER_ERROR_BASE_NUM + 27) ///< Failed to set RTC Date
#define PER_ERROR_RTC_SET_TIME                (PER_ERROR_BASE_NUM + 28) ///< Failed to set RTC Time

#define PER_ERROR_DW1000_INIT                 (PER_ERROR_APP_NUM + 0)   ///< DWS1000 Module failed to initializations
#define PER_ERROR_DW1000_SEND_MESSAGE         (PER_ERROR_APP_NUM + 1)   ///< DWS1000 Module failed to transmit message
#define PER_ERROR_SDCARD_FAILED_WRITE         (PER_ERROR_APP_NUM + 2)   ///< SDCARD Module failed to write file
#define PER_ERROR_SDCARD_FAILED_READ          (PER_ERROR_APP_NUM + 3)   ///< SDCARD Module failed to read file
#define PER_ERROR_SDCARD_FAILED_MOUNT         (PER_ERROR_APP_NUM + 4)   ///< SDCARD Module failed to mount the drive
#define PER_ERROR_SDCARD_CREATE_DIRECTORY     (PER_ERROR_APP_NUM + 5)   ///< SDCARD Module failed to create directory
#define PER_ERROR_ESP32_UART_INIT             (PER_ERROR_APP_NUM + 6)   ///< ESP32 UART Module failed to initializations
#define PER_ERROR_ESP32_ATC_ERROR             (PER_ERROR_APP_NUM + 7)   ///< ESP32 Module AT Command error
#define PER_ERROR_ESP32_ATC_SEND_FAIL         (PER_ERROR_APP_NUM + 8)   ///< ESP32 Module send data failed to the protocol stack
#define PER_ERROR_ESP32_WIFI_FAIL_CONNECT     (PER_ERROR_APP_NUM + 9)   ///< ESP32 Module WI-FI failed to connect access point
#define PER_ERROR_ESP32_ATC_TIMEOUT           (PER_ERROR_APP_NUM + 10)   ///< ESP32 Module AT Command timeout error
#define PER_ERROR_ESP32_ATC_BUSY              (PER_ERROR_APP_NUM + 11)   ///< ESP32 Module busy and cannot accept the command
#define PER_ERROR_FATFS_UPLOAD_DATA           (PER_ERROR_APP_NUM + 12)   ///< FATFS failed to upload distance data
#define PER_ERROR_FATFS_DELETE_FILES          (PER_ERROR_APP_NUM + 13)   ///< FATFS failed to delete files
#define PER_ERROR_FATFS_DUPLICATE_FILE_OPEN   (PER_ERROR_APP_NUM + 14)   ///< FATFS failed to open duplicate file

 //#define PER_ERROR_INTERNAL                    (PER_ERROR_BASE_NUM + 3)  ///< Internal Error
//#define PER_ERROR_NO_MEM                      (PER_ERROR_BASE_NUM + 4)  ///< No Memory for operation
//#define PER_ERROR_NOT_FOUND                   (PER_ERROR_BASE_NUM + 5)  ///< Not found
//#define PER_ERROR_NOT_SUPPORTED               (PER_ERROR_BASE_NUM + 6)  ///< Not supported
//#define PER_ERROR_INVALID_PARAM               (PER_ERROR_BASE_NUM + 7)  ///< Invalid Parameter
//#define PER_ERROR_INVALID_STATE               (PER_ERROR_BASE_NUM + 8)  ///< Invalid state, operation disallowed in this state
//#define PER_ERROR_INVALID_LENGTH              (PER_ERROR_BASE_NUM + 9)  ///< Invalid Length
//#define PER_ERROR_INVALID_FLAGS               (PER_ERROR_BASE_NUM + 10) ///< Invalid Flags
//#define PER_ERROR_INVALID_DATA                (PER_ERROR_BASE_NUM + 11) ///< Invalid Data
//#define PER_ERROR_DATA_SIZE                   (PER_ERROR_BASE_NUM + 12) ///< Invalid Data size
//#define PER_ERROR_TIMEOUT                     (PER_ERROR_BASE_NUM + 13) ///< Operation timed out
//#define PER_ERROR_NULL                        (PER_ERROR_BASE_NUM + 14) ///< Null Pointer
//#define PER_ERROR_FORBIDDEN                   (PER_ERROR_BASE_NUM + 15) ///< Forbidden Operation
//#define PER_ERROR_INVALID_ADDR                (PER_ERROR_BASE_NUM + 16) ///< Bad Memory Address
//#define PER_ERROR_BUSY                        (PER_ERROR_BASE_NUM + 17) ///< Busy
//#define PER_ERROR_CONN_COUNT                  (PER_ERROR_BASE_NUM + 18) ///< Maximum connection count exceeded.
//#define PER_ERROR_RESOURCES                   (PER_ERROR_BASE_NUM + 19) ///< Not enough resources for operation


 /* Exported constants --------------------------------------------------------*/
 /* Exported macro ------------------------------------------------------------*/
 /* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* __ERRORCODE_H_ */


 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
