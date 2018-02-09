/*
 * drv_buzzer.h
 *
 *  Created on: Feb 08, 2018
 *      Author: opus
 */

#ifndef DRV_BUZZER_H_
#define DRV_BUZZER_H_



#ifdef __cplusplus
 extern "C" {
#endif

#include "hw_def.h"

bool drvBuzzerInit(void);
void drvBuzzerSetPin(bool enable);


#ifdef __cplusplus
}
#endif

#endif /* DRV_BUZZER_H_ */
