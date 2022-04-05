/**
  ******************************************************************************
  * @file    dws1000_control.h
  * @author  IBronx MDE team
  * @brief   Peripheral driver header file for dwm1000 module control
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
#ifndef __DWS1000_CONTROL_H_
#define __DWS1000_CONTROL_H_

#ifdef __cplusplus
 extern "C" {
#endif

 /* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

#include <stdbool.h>

 /* Exported types ------------------------------------------------------------*/
#define DW_RST_PIN                      D_RSTN_Pin
#define DW_RST_PORT                     D_RSTN_GPIO_Port
#define DW_IRQ_PIN                      D_IRQ_Pin
#define DW_IRQ_PORT                     D_IRQ_GPIO_Port
#define DW_SPI_CS_PIN                   D_CS_Pin
#define DW_SPI_CS_PORT                  D_CS_GPIO_Port

#define DW_EXTI_IRQn                    EXTI1_IRQn
#define DW_EXTI_IRQn_RST                EXTI0_IRQn

 typedef void (*dw_isr_t)(void);
 dw_isr_t dwm1000_exti_cb;

 /* Exported constants --------------------------------------------------------*/
 /* Exported macro ------------------------------------------------------------*/
 /* Exported functions ------------------------------------------------------- */

 uint32_t dwm1000_Init(void);
 void dwm1000_Reset(void);
 void dwm1000_SetupResetOrIRQ(bool bEnable);
 void dwm1000_WakeUp(void);
 bool dwm1000_SetMutexOn(void);
 void dwm1000_SetMutexOff(void);
 void dwm1000_SetSlowClockRate(void);
 void dwm1000_SetFastClockRate(void);
 void dwm1000_EXTIHandler(uint16_t GPIO_Pin);
 void dwm1000_RegisterInterruptCallback(dw_isr_t exti_cb);
 uint32_t SPI_GetBaudRatePrescaler(SPI_TypeDef* spi, uint32_t baudrate);
 uint32_t SPI_GetClkFreq(SPI_TypeDef* spi);
 void SPI_SetChipSelect(bool bEnable);
 int dwm1000_WriteToSPI(uint16_t hLen, const uint8_t *pheadBuf, uint32_t bLen,
     const uint8_t *pbodyBuf);
 int dwm1000_ReadFromSPI(uint16_t hLen, const uint8_t *pheadBuf, uint32_t rLen,
     uint8_t *preadBuf);


#ifdef __cplusplus
}
#endif

#endif /* __DWS1000_CONTROL_H_ */


 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
