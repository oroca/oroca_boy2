/*
 * drv_led.c
 *
 *  Created on: Feb 08, 2018
 *      Author: opus
 */



#include <stdarg.h>
#include <stdbool.h>

#include "drv_led.h"

#include "hw.h"




//-- Internal Variables
//




//-- External Variables
//


//-- Internal Functions
//


//-- External Functions
//




typedef struct
{
  GPIO_TypeDef       *port;
  uint16_t            pin;
}drv_led_gpio_t;


typedef struct
{
  GPIO_PinState       pin_state;
  drv_led_gpio_t      gpio;
}drv_led_t;

drv_led_t drv_led_tbl[DRV_LED_MAX_CH];

bool drvLedInit(void)
{
  uint32_t i;
  GPIO_InitTypeDef  GPIO_InitStruct;

  drv_led_tbl[0].gpio.port = GPIOG;
  drv_led_tbl[0].gpio.pin  = GPIO_PIN_6;    // LED1
  drv_led_tbl[1].gpio.port = GPIOD;
  drv_led_tbl[1].gpio.pin  = GPIO_PIN_4;    // LED2
  drv_led_tbl[2].gpio.port = GPIOD;
  drv_led_tbl[2].gpio.pin  = GPIO_PIN_5;    // LED3
  drv_led_tbl[3].gpio.port = GPIOK;
  drv_led_tbl[3].gpio.pin  = GPIO_PIN_3;    // LED4

  drv_led_tbl[3].gpio.port = GPIOD;
  drv_led_tbl[3].gpio.pin  = GPIO_PIN_3;    // LED7


  for (i=0; i<DRV_LED_MAX_CH; i++)
  {
    if (drv_led_tbl[i].gpio.port != NULL)
    {
      GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
      GPIO_InitStruct.Pull  = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      GPIO_InitStruct.Pin   = drv_led_tbl[i].gpio.pin;
      HAL_GPIO_Init(drv_led_tbl[i].gpio.port, &GPIO_InitStruct);

      HAL_GPIO_WritePin(drv_led_tbl[i].gpio.port, drv_led_tbl[i].gpio.pin, GPIO_PIN_SET);
    }
  }

  return true;
}

bool drvLedGetState(uint8_t ch)
{
  GPIO_PinState pin_state = GPIO_PIN_RESET;
  bool ret = false;


  if (ch >= DRV_LED_MAX_CH) return false;

  pin_state = HAL_GPIO_ReadPin(drv_led_tbl[ch].gpio.port, drv_led_tbl[ch].gpio.pin);
  if (pin_state == GPIO_PIN_RESET) ret = true;

  return ret;
}

void drvLedSetState(uint8_t ch, bool led_state)
{
  GPIO_PinState pin_state;

  if (ch >= DRV_LED_MAX_CH) return;


  if (led_state == true )
  {
    pin_state = GPIO_PIN_RESET;
  }
  else
  {
    pin_state = GPIO_PIN_SET;
  }

  HAL_GPIO_WritePin(drv_led_tbl[ch].gpio.port, drv_led_tbl[ch].gpio.pin, pin_state);
}

