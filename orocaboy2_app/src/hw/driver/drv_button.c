/*
 * drv_button.c
 *
 *  Created on: Feb 08, 2018
 *      Author: opus
 */

#include "drv_button.h"

#include "hw.h"

//-- Internal Variables
//



//-- External Variables
//


//-- Internal Functions
//


//-- External Functions
//


typedef struct {
  GPIO_TypeDef *port;
  uint32_t      pin;
} drv_button_t;

drv_button_t drv_button[DRV_BUTTON_MAX_CH] = {
    {GPIOA, GPIO_PIN_0},
};


bool drvButtonInit(void)
{
  uint32_t i;
  GPIO_InitTypeDef  GPIO_InitStruct;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  for(i = 0; i < DRV_BUTTON_MAX_CH; i++)
  {
    GPIO_InitStruct.Pin = drv_button[i].pin;
    HAL_GPIO_Init(drv_button[i].port, &GPIO_InitStruct);
  }

  return true;
}

uint8_t drvButtonGetState(uint8_t ch)
{
  uint8_t ret = 0;

  if(HAL_GPIO_ReadPin(drv_button[ch].port, drv_button[ch].pin) == GPIO_PIN_RESET)
  {
    ret = 1;
  }
  else
  {
    ret = 0;
  }

  return ret;
}

