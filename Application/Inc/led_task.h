/**
  ******************************************************************************
  * @file    led_task.h
  * @author  IBronx MDE team
  * @brief   LED display task header file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LED_TASK_H_
#define __LED_TASK_H_

#ifdef __cplusplus
 extern "C" {
#endif

 /* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

 /* Exported types ------------------------------------------------------------*/

 typedef enum
 {
   STATE_LED_INIT = 0,
   STATE_LED_BLINK_ON,
   STATE_LED_BLINK_OFF,
 }LEDTask_State;


#define LED_START_FLAG              0x00000001U
#define LED_BLINK_NON_STOP_FLAG     0x00000002U
#define LED_STANDBY_MODE_FLAG       0x00000004U
#define LED_ENABLE_BUZZER_FLAG      0x00000008U

#define LED_DISTANCE_FLAG           0x00000010U
#define LED_BATTERY_FLAG            0x00000020U
#define LED_HEARTBEAT_FLAG          0x00000040U
#define LED_LOW_BATT_BUZZER_FLAG	0x00000080U

 /* Exported constants --------------------------------------------------------*/
 /* Exported macro ------------------------------------------------------------*/
 /* Exported functions ------------------------------------------------------- */
 void StatusLEDFunc(void *argument);
 void statusLED_StateOperation(void);
 void statusLED_Init(void);
 void statusLED_Blink_On(void);
 void statusLED_Blink_Off(void);
 void statusLED_ConfigureBlink(uint32_t count, uint32_t led_on_ms, uint32_t led_off_ms);
 LEDTask_State statusLED_GetCurrentState(void);
 void statusLED_ChangeCurrentState(LEDTask_State state);
 void statusLED_WaitForDelay(uint32_t delay_ms);


#ifdef __cplusplus
}
#endif

#endif /* __LED_TASK_H_ */


 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
