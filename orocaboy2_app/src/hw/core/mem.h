/*
 * mem.h
 *
 *  Created on: Feb 08, 2018
 *      Author: baram
 */

#ifndef MEM_H_
#define MEM_H_



#ifdef __cplusplus
 extern "C" {
#endif


#include "hw_def.h"
#include "bsp.h"




void  memInit(uint32_t addr, uint32_t length);
void *memMalloc(size_t size);
void  memFree(void *ptr);


#ifdef __cplusplus
}
#endif

#endif /* MEM_H_ */
