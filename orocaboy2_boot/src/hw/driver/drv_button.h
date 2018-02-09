/*
 * drv_button.h
 *
 *  Created on: Feb 08, 2018
 *      Author: opus
 */

#ifndef DRV_BUTTON_H_
#define DRV_BUTTON_H_



#ifdef __cplusplus
 extern "C" {
#endif

#include "hw_def.h"

#define DRV_BUTTON_MAX_CH        _HW_DEF_BUTTON_CH_MAX

bool     drvButtonInit(void);
uint8_t  drvButtonGetState(uint8_t ch);


#ifdef __cplusplus
}
#endif



#endif /* DRV_BUTTON_H_ */
