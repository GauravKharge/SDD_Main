/**
  ******************************************************************************
  * @file    common.c
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

/* Includes ------------------------------------------------------------------*/
#include "common.h"
#include "flash_control.h"
#include "cmsis_os.h"
#include "string.h"
#include "stdio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t vBat_pct;
static double vBat_volt;

extern ADC_HandleTypeDef hadc1;
extern RTC_HandleTypeDef hrtc;
extern uint8_t flash_memory[FLASH_TOTAL_SIZE];

/* Private function prototypes -----------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/**
  * @brief  Start Battery voltage ADC conversion
  * @param  None
  * @retval None
  */
void StartBatteryVoltAdcConversion(void)
{
  uint32_t raw_value = 0;
  for (int i = 0; i < BATTERY_ADC_READ_COUNT; i++)
  {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    raw_value += HAL_ADC_GetValue(&hadc1);
    osDelay(30);
  }
  raw_value = raw_value / BATTERY_ADC_READ_COUNT;

  vBat_volt = (((raw_value * 3.3) * 2) / 4096) + BATTERY_ADC_OFFSET;
  double percentage = ((vBat_volt - BATTERY_MIN_VOLT) / (BATTERY_MAX_VOLT - BATTERY_MIN_VOLT)) * 100;
  percentage = percentage < 100 ? percentage : 100;
  percentage = percentage < 0 ? 0 : percentage;

  vBat_pct = (uint8_t)percentage;
}

/**
  * @brief  Get Battery level in percentage
  * @param  None
  * @retval None
  */
uint16_t GetBatteryLevelInPct(void)
{
  return vBat_pct;
}

/**
  * @brief  Get Battery level in volt
  * @param  None
  * @retval None
  */
uint16_t GetBatteryLevelInVolt(void)
{
  return (uint16_t)(vBat_volt * 1000);
}

/**
  * @brief  Read the RTC from flash and then update the RTC
  * @param  None
  * @retval None
  */
void UpdateRTCFromFlash(void)
{
  uint8_t pDate[FLASH_DATE_SIZE];
  uint8_t pTime[FLASH_TIME_SIZE];
  flash_read_RTC(pDate, pTime);

  RTC_DateTypeDef sDate = {0};
  RTC_TimeTypeDef sTime = {0};

  sDate.Year = pDate[0];
  sDate.Month = pDate[1];
  sDate.Date = pDate[2];
  sDate.WeekDay = pDate[3];
  sTime.Hours = pTime[0];
  sTime.Minutes = pTime[1];
  sTime.Seconds = pTime[2];

  // double check the Date & TimeStamp from flash
  if (sDate.Year + 2000 <  2021)
  {
    return;
  }

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) == HAL_OK)
  {
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) == HAL_OK)
    {
      char buffer[50];
      sprintf(buffer, "%s %04d-%02d-%02d %02d:%02d:%02d", "[MAIN] - RTC from flash ",
          sDate.Year + 2000, sDate.Month, sDate.Date, sTime.Hours,sTime.Minutes, sTime.Seconds);
      SEGGER_SYSVIEW_Print(buffer);
    }
  }
}

/**
  * @brief  Write current RTC to flash memory
  * @param  None
  * @retval None
  */
void WriteCurrentRTCToFlash(void)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  flash_write_RTCWithBackUp(sDate, sTime);
}


 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
