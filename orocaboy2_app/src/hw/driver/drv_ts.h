/*
 * drv_ts.h
 *
 *  Created on: Feb 13, 2018
 *      Author: opus
 */

#ifndef DRV_TS_H_
#define DRV_TS_H_


#include "hw_def.h"



err_code_t drvTsInit(uint16_t ts_SizeX, uint16_t ts_SizeY);
err_code_t drvTsUpdateTouchData(void);
err_code_t drvTsResetTouchData(void);

uint8_t drvTsIsDetected(void);
uint16_t drvTsGetXAxis(uint8_t detect_num);
uint16_t drvTsGetYAxis(uint8_t detect_num);
uint8_t drvTsGetWeight(uint8_t detect_num);
touch_event_t drvTsGetEventId(uint8_t detect_num);
uint8_t drvTsGetArea(uint8_t detect_num);
touch_gesture_t drvTsGetGestureId(void);
err_code_t drvTsExtiConfig(void);

#endif /* DRV_TS_H_ */
