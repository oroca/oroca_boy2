/*
 * drv_led.h
 *
 *  Created on: Feb 08, 2018
 *      Author: opus
 */

#ifndef DRV_LED_H_
#define DRV_LED_H_



#ifdef __cplusplus
 extern "C" {
#endif


#include "hw_def.h"


#define DRV_LED_MAX_CH   _HW_DEF_LED_CH_MAX

bool drvLedInit(void);

bool drvLedGetState(uint8_t ch);
void drvLedSetState(uint8_t ch, bool led_state);



#ifdef __cplusplus
}
#endif

#endif /* DRV_LED_H_ */
