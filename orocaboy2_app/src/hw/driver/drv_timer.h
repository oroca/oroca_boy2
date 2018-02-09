/*
 * drv_timer.h
 *
 *  Created on: Feb 08, 2018
 *      Author: opus
 */

#ifndef DRV_TIMER_H_
#define DRV_TIMER_H_



#ifdef __cplusplus
 extern "C" {
#endif

#include "hw_def.h"


#define DRV_TIMER_CH_MAX   _HW_DEF_TIMER_CH_MAX

bool drvTimerInit(void);


void drvTimerStop(uint8_t channel);
void drvTimerSetPeriod(uint8_t channel, uint32_t period_data);
void drvTimerAttachInterrupt(uint8_t channel, voidFuncPtr handler);
void drvTimerDetachInterrupt(uint8_t channel);
void drvTimerStart(uint8_t channel);


#ifdef __cplusplus
}
#endif

#endif /* DRV_TIMER_H_ */
