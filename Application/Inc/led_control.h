/**
  ******************************************************************************
  * @file    led_control.h
  * @author  IBronx MDE team
  * @brief   Peripheral driver header file for WS28xx RGB LED control
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
#ifndef __LED_CONTROL_H_
#define __LED_CONTROL_H_

#ifdef __cplusplus
 extern "C" {
#endif

 /* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

 /* Exported types ------------------------------------------------------------*/

#define LED_NON_STOP                  0xFFFFFFFF
#define LED_MAX_HEARTBEAT_OFF_MS      10U
#define LED_MAX_CHARGEBEAT_OFF_MS	  10U

#define LED_RED_PIN                   LED_R_Pin
#define LED_RED_PORT                  LED_R_GPIO_Port
#define LED_BLUE_PIN                  LED_B_Pin
#define LED_BLUEPORT                  LED_B_GPIO_Port
#define LED_GREEN_PIN                 LED_G_Pin
#define LED_GREEN_PORT                LED_G_GPIO_Port

#define LED_ON                        GPIO_PIN_RESET
#define LED_OFF                       GPIO_PIN_SET
#define LED_DEVICE_INIT               LED_GREEN_PIN
#define LED_STANDBY_MODE              LED_RED_PIN | LED_BLUE_PIN
#define LED_HEARTBEAT_MODE            LED_GREEN_PIN
#define LED_LOW_BATTERY               LED_BLUE_PIN
#define LED_BATTERY_CHARGING          LED_BLUE_PIN
#define LED_BATTERY_FULL_CHARGE       LED_GREEN_PIN
#define LED_DISABLE                   0

#define LED_CLOSE_DEVICE_1            LED_RED_PIN
#define LED_CLOSE_DEVICE_2            LED_BLUE_PIN
#define LED_CLOSE_DEVICE_3            LED_RED_PIN | LED_BLUE_PIN                   // pink
#define LED_CLOSE_DEVICE_4            LED_BLUE_PIN | LED_GREEN_PIN                 // light blue
#define LED_CLOSE_DEVICE_5            LED_RED_PIN | LED_BLUE_PIN | LED_GREEN_PIN   // white






 /* Exported constants --------------------------------------------------------*/
 /* Exported macro ------------------------------------------------------------*/
 /* Exported functions ------------------------------------------------------- */

 void rgbled_Init(void);
 void rgbled_InitDevice(void);
 void rgbled_StandbyMode(void);
 void rgbled_BatteryLevelStatus(uint32_t batteryFlags);
 void rgbled_DistanceMeasurement(uint32_t device_cnt, uint32_t flags);
 void rgbled_ClearFlags(uint32_t flags);
 void rgbled_Heartbeat(void);
 void rgbled_TurnOnLED(void);
 void rgbled_TurnOffLED(void);
 void rgbled_ExecBlink(uint32_t blink_count, uint32_t led_on_ms, uint32_t led_off_ms, uint16_t led_color);

#ifdef __cplusplus
}
#endif

#endif /* __LED_CONTROL_H_ */


 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
