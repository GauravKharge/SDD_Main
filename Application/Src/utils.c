/**
  ******************************************************************************
  * @file    utils.c
  * @author  IBronx MDE team
  * @brief   Provide general utility function
  *          This file provides firmware utility functions to support all modules
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
#include "utils.h"

#include <stdio.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

/**
  * @brief  Utility function to remove char from string
  * @param  pStr  Pointer of string
  * @param  c     Character to be remove from string
  * @retval None
  */
void utils_RemoveCharFromString(char *pStr, char c)
{
  if (pStr == NULL)
    return;

  char *pDest = pStr;

  while(*pStr)
  {
    if (*pStr != c)
      *pDest++ = *pStr;

    *pStr++;
  }
  *pDest = '\0';
}

/**
  * @brief  Utility function to remove sub string from a string
  * @param  pStr  Pointer of string
  * @param  c     Character to be remove from string
  * @retval None
  */
void utils_RemoveSubstrFromString (char *pStr, char *pSubStr)
{
  char *match;
  int len = strlen(pSubStr);

  while ((match = strstr(pStr, pSubStr)))
  {
    *match = '\0';
    strcat(pStr, match+len);
  }
}



/************************ (C) COPYRIGHT IBronx *****************END OF FILE****/
