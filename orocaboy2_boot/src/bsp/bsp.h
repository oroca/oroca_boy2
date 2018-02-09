/*
 * bsp.h
 *
 *  Created on: Feb 08, 2018
 *      Author: opus
 */

#ifndef BSP_H_
#define BSP_H_


#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include "hw_def.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"

#define _BSP_DEF_BOARD_NAME "OROCABOY2 revA"

void bspInit();
void bspDeinit(void);
void bspInitUSB(void);
void bspJumpToAddress(uint32_t address);


#endif /* BSP_H_ */
