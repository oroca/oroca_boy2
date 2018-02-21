/*
 * drv_exti.h
 *
 *  Created on: 2017. 11. 13.
 *      Author: opus
 */

#ifndef DRV_EXTI_H_
#define DRV_EXTI_H_


#include "hw_def.h"

void drvExtiInit(void);

bool drvExtiAttachInterrupt(uint8_t ch, uint32_t mode, void (*func)(void *), void *arg);
void drvExtiDetachInterrupt(uint8_t ch);


#endif /* DRV_EXTI_H_ */
