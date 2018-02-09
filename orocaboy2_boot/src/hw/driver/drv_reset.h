/*
 *  drv_reset.h
 *
 *  Created on: Feb 08, 2018
 *      Author: opus
 */

#ifndef DRV_RESET_H
#define DRV_RESET_H


#ifdef __cplusplus
 extern "C" {
#endif

#include "hw_def.h"


bool drvResetInit(void);
uint8_t drvResetGetStatus(void);
void drvResetRunWdgReset(void);
void drvResetRunSoftReset(void);
void drvResetClearFlag(void);


#ifdef __cplusplus
}
#endif


#endif /* DRV_RESET_H */
