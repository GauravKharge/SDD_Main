/**
  ******************************************************************************
  * @file    dws1000_control.c
  * @author  IBronx MDE team
  * @brief   Peripheral driver for DWM1000 module control
  *          This file provides firmware utility functions to support DWM1000
  *          module functions
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
#include "dws1000_control.h"
#include "dwm1000.h"
#include "errorcode.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
dw_isr_t dwm1000_exti_cb = NULL;
volatile uint8_t dw_wait_reset_done;

extern SPI_HandleTypeDef hspi1;

/* Private function prototypes -----------------------------------------------*/
/* function prototypes -------------------------------------------------------*/


/**
  * @brief  DWM1000 module initialization
  * @param  None
  * @retval rc:  return PER_NO_ERROR if no error, otherwise return rc
  */
uint32_t dwm1000_Init(void)
{
  // enble spi1
  __HAL_SPI_ENABLE(&hspi1);

  // disable chip select on dwm1000
  SPI_SetChipSelect(false);

  return PER_NO_ERROR;
}

/**
  * @brief  DWM1000 reset function which consist of two functions
  *          - used as general output
  *          - reset the digital part of DWM1000 by driving this pin low
  *         Note: RSTn pin should not be driven high externally.
  * @param  None
  * @retval None
 */
void dwm1000_Reset(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = DW_RST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DW_RST_PORT, &GPIO_InitStruct);

  //drive the RSTn pin low
  HAL_GPIO_WritePin(DW_RST_PORT, DW_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(1);

  //put the pin back to output open-drain (not active)
  dwm1000_SetupResetOrIRQ(false);
  HAL_Delay(2);
}

/**
  * @brief  Configure Reset pin either Interrupt or output open collector mode
  *         Note: When set as ouput, set DWM1000 as inactive mode
  * @param  bEnable:  Enable interrupt when true otherwise set it as output
  * @retval None
 */
void dwm1000_SetupResetOrIRQ(bool bEnable)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (bEnable)
  {
    GPIO_InitStruct.Pin = DW_RST_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DW_RST_PORT, &GPIO_InitStruct);

    // Enable Reset pin as interrupt
    HAL_NVIC_SetPriority(DW_EXTI_IRQn_RST, 6, 0);
    HAL_NVIC_EnableIRQ(DW_EXTI_IRQn_RST);
  }
  else
  {
    // Disable interrupt for Reset pin
    HAL_NVIC_DisableIRQ(DW_EXTI_IRQn_RST);

    //put the pin back to tri-state ... as
    //output open-drain (not active)
    GPIO_InitStruct.Pin = DW_RST_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DW_RST_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(DW_RST_PORT, DW_RST_PIN, GPIO_PIN_SET);
  }
}

/**
  * @brief  Wake up DWM1000 from sleep/deep sleep mode
  *         Note: require to wait for 7ms to stabilize Crystal XTAL
  * @param  None
  * @retval None
 */
void dwm1000_WakeUp(void)
{
  SPI_SetChipSelect(true);
  HAL_Delay(1);
  SPI_SetChipSelect(false);

  // wait 7ms for DWM1000 XTAL to stabilize
  HAL_Delay(7);
}

/**
  * @brief  Disable interrupt at the start of a critical section, for example spi write/erase
  * @param  None
  * @retval return true when disable interrupt
 */
bool dwm1000_SetMutexOn(void)
{
  // disable interrupt
  NVIC_DisableIRQ(DW_EXTI_IRQn);

  return true;
}

/**
  * @brief  Enable interrupt at the end of a critical section
  * @param  None
  * @retval return true when disable interrupt
 */
void dwm1000_SetMutexOff(void)
{
  // enable interrupt
  HAL_NVIC_SetPriority(DW_EXTI_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DW_EXTI_IRQn);
}

/**
  * @brief  Set SPI Write/Read clock rate to 2.25Mhz
  *         Note: SCK clock is derive from SPI clock with a prescaler
  *         which might have some offset.
  * @param  None
  * @retval None
 */
void dwm1000_SetSlowClockRate(void)
{
  hspi1.Init.BaudRatePrescaler = SPI_GetBaudRatePrescaler(SPI1, 2250000);
  HAL_SPI_Init(&hspi1);
}

/**
  * @brief  Set SPI Write/Read clock rate to 18Mhz
  *         Note: SCK clock is derive from SPI clock with a prescaler
  *         which might have some offset.
  * @param  None
  * @retval None
 */
void dwm1000_SetFastClockRate(void)
{
  hspi1.Init.BaudRatePrescaler = SPI_GetBaudRatePrescaler(SPI1, 18000000);
  HAL_SPI_Init(&hspi1);
}

/**
  * @brief  Enable IRQ Interrupt and register a callback function
  * @param  exti_cb:  EXTI IRQ Callback function
  * @retval None
 */
void dwm1000_RegisterInterruptCallback(dw_isr_t exti_cb)
{
  dwm1000_exti_cb = exti_cb;

  // enable interrupt
  HAL_NVIC_SetPriority(DW_EXTI_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DW_EXTI_IRQn);
}

/**
  * @brief  EXTI GPIO IRQ Interrupt Handler
  * @param  GPIO_Pin:  GPIO Pin number
  * @retval None
 */
__INLINE void dwm1000_EXTIHandler(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == DW_RST_PIN)
  {
    dw_wait_reset_done = 1;
  }

  if (GPIO_Pin == DW_IRQ_PIN)
  {
    //while ((HAL_GPIO_ReadPin(DWM1000_IRQ_PORT, DWM1000_IRQ_PIN) != 0))
    {
      dwm1000_exti_cb();
    } // while DWM1000 IRQ line active
  }
}

/**
  * @brief  DWM1000 module Write SPI function which take two separate byte buffers to
  *         write header & body data.
  *         Rewrite Low level abstract function - writespi()  *
  * @param  hLen:      header data size
  * @param  pheadBuf:  pointer of header data buffer
  * @param  bLen:      body data size
  * @param  pbodyBuf:  pointer of body data buffer
  * @retval return 0 for success
 */
#pragma GCC optimize ("O3")
int dwm1000_WriteToSPI(uint16_t hLen, const uint8_t *pheadBuf, uint32_t bLen, const uint8_t *pbodyBuf)
{
  bool bMutex = dwm1000_SetMutexOn();

  while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

  SPI_SetChipSelect(true);

  // send header data in polling mode
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&pheadBuf[0], hLen, HAL_MAX_DELAY);

  // send body data in polling mode
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&pbodyBuf[0], bLen, HAL_MAX_DELAY);

  SPI_SetChipSelect(false);

  if (bMutex)
    dwm1000_SetMutexOff();

  return 0;
}

/**
  * @brief  DWM1000 module Read SPI function which take two separate byte buffers to
  *         write header & read data. read data will store into preadBuf pointer.
  *         Rewrite Low level abstract function - readfromspi()  *
  * @param  hLen:      header data size
  * @param  pheadBuf:  pointer of header data buffer
  * @param  bLen:      body data size
  * @param  preadBuf:  pointer of read data buffer
  * @retval return 0 for success
 */
#pragma GCC optimize ("O3")
int dwm1000_ReadFromSPI(uint16_t hLen, const uint8_t *pheadBuf, uint32_t rLen, uint8_t *preadBuf)
{
  bool bMutex = dwm1000_SetMutexOn();

  /* Blocking: Check whether previous transfer has been finished */
  while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

  SPI_SetChipSelect(true);

  /* Send header */
  for(int i = 0; i< hLen; i++)
  {
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&pheadBuf[i], 1, HAL_MAX_DELAY); //No timeout
  }

  /* for the data buffer use LL functions directly as the HAL SPI read function
   * has issue reading single bytes */
  while(rLen-- > 0)
  {
    /* Wait until TXE flag is set to send data */
    while(__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) == RESET)
    {
    }

    hspi1.Instance->DR = 0; /* set output to 0 (MOSI), this is necessary for
    e.g. when waking up DW1000 from DEEPSLEEP via dwt_spicswakeup() function.
    */

    /* Wait until RXNE flag is set to read data */
    while(__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE) == RESET)
    {
    }

    (*preadBuf++) = hspi1.Instance->DR;  //copy data read form (MISO)
  }

  SPI_SetChipSelect(false);

  if (bMutex)
    dwm1000_SetMutexOff();

  return 0;
}


/**
  * @brief  Get the Baud Rate prescaler value which will be used to configure
  *         the transmit and receive SCK clock. It can be derive from SPI clock frequency
  * @param  spi:       SPI instance
  * @param  baudrate:  Target Baud Rate that used for transmit and receive SCK clock
  * @retval None
  */
uint32_t SPI_GetBaudRatePrescaler(SPI_TypeDef* spi, uint32_t baudrate)
{
  uint32_t spi_freq = SPI_GetClkFreq(spi);
  uint32_t current_freq = spi_freq;

  // calculate prescaler based on spi clock frequency
  uint8_t scaler = 0;
  while(current_freq > baudrate)
  {
    scaler++;
    current_freq = spi_freq >> scaler;

    if (scaler >= 8)
      break;
  }

  uint32_t baudratescaler = SPI_BAUDRATEPRESCALER_2;
  switch (scaler)
  {
    case 1:
      baudratescaler = SPI_BAUDRATEPRESCALER_2;
      break;
    case 2:
      baudratescaler = SPI_BAUDRATEPRESCALER_4;
      break;
    case 3:
      baudratescaler = SPI_BAUDRATEPRESCALER_8;
      break;
    case 4:
      baudratescaler = SPI_BAUDRATEPRESCALER_16;
      break;
    case 5:
      baudratescaler = SPI_BAUDRATEPRESCALER_32;
      break;
    case 6:
      baudratescaler = SPI_BAUDRATEPRESCALER_64;
      break;
    case 7:
      baudratescaler = SPI_BAUDRATEPRESCALER_128;
      break;
    case 8:
      baudratescaler = SPI_BAUDRATEPRESCALER_256;
      break;
    default:
      baudratescaler = SPI_BAUDRATEPRESCALER_256;
      break;
  }

  return baudratescaler;
}

/**
  * @brief  Get SPI Clock Frequency based on selected SPI instance
  * @param  spi:      SPI instance
  * @retval ClkFreq:  SPI Clock Frequency
  */
uint32_t SPI_GetClkFreq(SPI_TypeDef* spi)
{
  // prevent divide by zero if none of selection is valid
  uint32_t freq = 1;

  if (spi == SPI1)
    freq = HAL_RCC_GetPCLK2Freq();

  if (spi == SPI2 || spi == SPI3)
    freq = HAL_RCC_GetPCLK1Freq();

  return freq;
}

/**
  * @brief  SPI chip select for slave device
  * @param  spiId:     SPI handle Id
  * @param  slave_id:  Slave device Id
  * @param  bEnable:   Enable when set true, otherwise false
  * @retval None
  */
void SPI_SetChipSelect(bool bEnable)
{
  if (bEnable == true)
  {
    // set CS pin as reset
    DW_SPI_CS_PORT->BSRR = DW_SPI_CS_PIN << 16U;
  }
  else
  {
    // set CS pin as set
    DW_SPI_CS_PORT->BSRR = DW_SPI_CS_PIN;
  }
}



/************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
