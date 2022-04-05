/**
  ******************************************************************************
  * @file    utils.h
  * @author  IBronx MDE team
  * @brief   Provide general utility functions
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
#ifndef __UTILS_H_
#define __UTILS_H_

#ifdef __cplusplus
 extern "C" {
#endif

 /* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

 /* Exported types ------------------------------------------------------------*/
 /* Exported constants --------------------------------------------------------*/
 /* Exported macro ------------------------------------------------------------*/
 /* Exported functions ------------------------------------------------------- */
 void utils_RemoveCharFromString(char *pStr, char c);
 void utils_RemoveSubstrFromString (char *pStr, char *pSubStr);

#ifdef __cplusplus
}
#endif

#endif /* __UTILS_H_ */

 /************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
