/*
 * drv_dac.h
 *
 *  Created on: 2017. 8. 10.
 *      Author: baram
 */

#ifndef DRV_DAC_H_
#define DRV_DAC_H_



#ifdef __cplusplus
 extern "C" {
#endif

#include "hw_def.h"


#define DAC_CH_MAX    1




bool drvDacInit();
bool drvDacDeInit();

bool drvDacSetup(uint32_t hz);
bool drvDacStart(uint8_t ch);
void drvDacStop(uint8_t ch);
uint32_t drvDacAvailable(uint8_t ch);
void drvDacWrite(uint8_t ch, uint16_t data);




#ifdef __cplusplus
}
#endif



#endif
