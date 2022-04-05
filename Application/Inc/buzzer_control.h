/**
  ******************************************************************************
  * @file    buzzer_control.h
  * @author  IBronx MDE team
  * @brief   Peripheral driver header file for buzzer control
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

#ifndef __BUZZER_CONTROL_H_
#define __BUZZER_CONTROL_H_

#ifdef __cplusplus
 extern "C" {
#endif

 /* Includes ------------------------------------------------------------------*/
 #include "stm32f4xx_hal.h"
 #include "main.h"

  /* Exported types ------------------------------------------------------------*/
#define BUZZER_TIM_PRESCALER          8
#define BUZZER_TIM_CHANNEL            TIM_CHANNEL_1
#define BUZZER_PULSE_NON_STOP         0xFFFFFFFF
//#define BUZZER_TRIGGER_TIME_IN_SEC    5
uint16_t BUZZER_TRIGGER_TIME_IN_SEC;


  /* Exported constants --------------------------------------------------------*/
  /* Exported macro ------------------------------------------------------------*/
  /* Exported functions ------------------------------------------------------- */
 void buzzer_InitDevice(void);
 void buzzer_ExecBuzzer(uint32_t pulse_count, uint32_t pulse_on_ms, uint32_t pulse_off_ms);
 void buzzer_TurnOn(void);
 void buzzer_TurnOff(void);

 void buzzer_ChangePWMFrequency(uint32_t frequency);
 uint32_t buzzer_GetTIMClkFreq(TIM_TypeDef * tim);


#ifdef __cplusplus
}
#endif

#endif /* __BUZZER_CONTROL_H_ */


 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
