/*
 *  loader.h
 *
 *  Created on: 2016. 5. 14.
 *      Author: Baram
 */

#ifndef LOADER_H
#define LOADER_H


#ifdef __cplusplus
extern "C" {
#endif


#include "def.h"


#define USB_AUTO_LOADER_CMD  "GAME OROCABOY2"
#define USB_AUTO_LOADER_BAUD 2400


bool loaderInit(uint8_t channel, char *port_name, uint32_t baud);
bool loaderReset(uint8_t channel, char *port_name, uint32_t baud);

err_code_t loaderCmdAddTagToBIN(uint32_t fw_addr, char *src_filename, char *dst_filename);
err_code_t loaderCmdReadVersion(uint8_t *p_version);
err_code_t loaderCmdReadPacketDataSizeMax(uint16_t *p_size);
err_code_t loaderCmdReadBoardName(uint8_t *p_str);
err_code_t loaderCmdFlashErase(uint32_t addr, uint32_t length);
err_code_t loaderCmdFlashWrite(uint32_t addr, uint8_t *p_data, uint32_t length);
err_code_t loaderCmdFlashVerfy(uint32_t addr, uint32_t length, uint16_t tx_crc, uint16_t *rx_crc);


#ifdef __cplusplus
}
#endif


#endif
