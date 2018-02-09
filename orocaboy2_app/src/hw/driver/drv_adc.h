/*
 * drv_adc.h
 *
 *  Created on: 2017. 8. 10.
 *      Author: baram
 */

#ifndef DRV_ADC_H_
#define DRV_ADC_H_



#ifdef __cplusplus
 extern "C" {
#endif

#include "hw_def.h"


#define DRV_ADC_MAX_CH    _HW_DEF_ADC_CH_MAX



bool     drvAdcInit(void);

uint16_t drvAdcRead(uint8_t ch);
uint16_t drvAdcRead8(uint8_t ch);
uint16_t drvAdcRead10(uint8_t ch);
uint16_t drvAdcRead12(uint8_t ch);
uint16_t drvAdcRead16(uint8_t ch);
uint16_t drvAdcReadVoltage(uint8_t ch);
uint16_t drvAdcConvVoltage(uint8_t ch, uint32_t adc_value);

uint8_t  drvAdcGetRes(uint8_t ch);


#ifdef __cplusplus
}
#endif



#endif
