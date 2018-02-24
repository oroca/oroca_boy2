/*
 *  loadercmd.h
 *
 *  Created on: Feb 08, 2018
 *      Author: opus
 */

#ifndef LOADERCMD_H
#define LOADERCMD_H


#ifdef __cplusplus
 extern "C" {
#endif

#include "ap_def.h"


#include "hw_def.h"
#include "ap.h"

#define GAME_TAG_TYPE_A 0

typedef struct
{
  uint32_t type;
  uint32_t address;
  uint32_t length;
  uint16_t crc;
} game_tag_type_a_t;


void loaderCmdInit(void);
void loaderCmdProcess(cmd_t *p_cmd);
err_code_t checkGame(uint32_t type, uint32_t address);



#ifdef __cplusplus
}
#endif


#endif /* LOADERCMD_H */
