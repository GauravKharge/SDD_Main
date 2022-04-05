/**
  ******************************************************************************
  * @file    common.h
  * @author  IBronx MDE team
  * @brief   Provide common functions to support all the peripherals module
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
#ifndef __COMMON_H_
#define __COMMON_H_

#ifdef __cplusplus
 extern "C" {
#endif

 /* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

 /* Exported types ------------------------------------------------------------*/
#define BATTERY_MAX_VOLT            4.185
#define BATTERY_MIN_VOLT            3.0
#define BATTERY_NORMINAL_VOLT       3.7
#define BATTERY_ADC_READ_COUNT      10
#define BATTERY_VOLT_SCALE          1000
#define BATTERY_LOW_VOLT            3.25 * BATTERY_VOLT_SCALE
#define BATTERY_MID_VOLT            BATTERY_NORMINAL_VOLT * BATTERY_VOLT_SCALE
#define BATTERY_HIGH_VOLT           4.175 * BATTERY_VOLT_SCALE
#define BATTERY_ADC_OFFSET          0.015  //mV

 /* Exported constants --------------------------------------------------------*/
 /* Exported macro ------------------------------------------------------------*/
 /* Exported functions ------------------------------------------------------- */
 void StartBatteryVoltAdcConversion(void);
 uint16_t GetBatteryLevelInPct(void);
 uint16_t GetBatteryLevelInVolt(void);
 void UpdateRTCFromFlash(void);
 void WriteCurrentRTCToFlash(void);

#ifdef __cplusplus
}
#endif

#endif /* __COMMON_H_ */


 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
