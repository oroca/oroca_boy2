/*
 * gameloader.h
 *
 *  Created on: 2017. 11. 23.
 *      Author: opus
 */

#ifndef GAMELOADER_H
#define GAMELOADER_H


#ifdef __cplusplus
 extern "C" {
#endif

#include "loadercmd.h"


#define _GAMELOADER_VER_STR               "OROCABOY2 GAMELOADER V180219R1"
#define _GAMELOADER_VER_NUM               {1, 0, 0}

void gameloaderInit(void);
void gameloaderProcess(void);


#ifdef __cplusplus
 }
#endif


#endif /* GAMELOADER_H */
