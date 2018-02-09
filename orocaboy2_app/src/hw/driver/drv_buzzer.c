/*
 * drv_buzzer.c
 *
 *  Created on: Feb 08, 2018
 *      Author: opus
 */



#include <stdarg.h>
#include <stdbool.h>

#include "drv_buzzer.h"

#include "hw.h"


//-- Internal Variables
//




//-- External Variables
//


//-- Internal Functions
//


//-- External Functions
//




bool drvBuzzerInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;


  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStruct.Pin = GPIO_PIN_15;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



  return true;
}

void drvBuzzerSetPin(bool enable)
{
  if (enable == true )
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  }
}

