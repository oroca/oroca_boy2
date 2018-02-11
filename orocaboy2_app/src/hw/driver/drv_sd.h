/*
 * drv_sd.h
 *
 *  Created on: 2018. 2. 11.
 *      Author: Baram
 */

#ifndef DRV_SD_H_
#define DRV_SD_H_



#ifdef __cplusplus
 extern "C" {
#endif


#include "hw_def.h"




bool drvSdInit(void);
bool drvSdDeInit(void);

bool drvSdReadBlocks(uint32_t block_addr, uint8_t *p_data, uint32_t num_of_blocks, uint32_t timeout_ms);
bool drvSdWriteBlocks(uint32_t block_addr, uint8_t *p_data, uint32_t num_of_blocks, uint32_t timeout_ms);
bool drvSdEraseBlocks(uint32_t start_addr, uint32_t end_addr);
bool drvSdIsBusy(void);
bool drvSdIsDetected(void);
bool drvSdGetInfo(void *p_info);



#ifdef __cplusplus
}
#endif

#endif /* DRV_SD_H_ */
