/*
 * drv_gpio.h
 *
 *  Created on: 2018. 1. 25.
 *      Author: Baram
 */

#ifndef DRV_GPIO_H_
#define DRV_GPIO_H_



#ifdef __cplusplus
 extern "C" {
#endif

#include "hw_def.h"

#define GPIO_INPUT            _DEF_INPUT
#define GPIO_INPUT_PULLUP     _DEF_INPUT_PULLUP
#define GPIO_INPUT_PULLDOWN   _DEF_INPUT_PULLDOWN
#define GPIO_OUTPUT           _DEF_OUTPUT
#define GPIO_OUTPUT_PULLUP    _DEF_OUTPUT_PULLUP
#define GPIO_OUTPUT_PULLDOWN  _DEF_OUTPUT_PULLDOWN

void drvGpioInit(void);
void drvGpioPinMode(uint8_t channel, uint8_t mode);
void drvGpioPinWrite(uint8_t channel, uint8_t bit);
int drvGpioPinRead(uint8_t channel);


#ifdef __cplusplus
 }
#endif

#endif /* DRV_GPIO_H_ */
