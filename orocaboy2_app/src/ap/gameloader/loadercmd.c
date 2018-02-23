/*
 *  loadercmd.c
 *
 *  Created on: Feb 08, 2018
 *      Author: opus
 */

#include "loadercmd.h"


#include <string.h>
#include "hw.h"
#include "util.h"


#define LOADER_ERR_INVAILD_CMD            0xF0F0

#define LOADER_CMD_READ_VERSION           0x80
#define LOADER_CMD_PACKET_DATA_SIZE_MAX   0x81
#define LOADER_CMD_READ_BOARD_NAME        0x82
#define LOADER_CMD_FLASH_ERASE            0x83
#define LOADER_CMD_FLASH_WRITE            0x84
#define LOADER_CMD_FLASH_READ             0x85
#define LOADER_CMD_FLASH_VERIFY           0x86
#define LOADER_CMD_JUMP                   0x87




static void loaderCmdReadVersion(cmd_t *p_cmd);
static void loaderCmdPacketDataSizeMax(cmd_t *p_cmd);
static void loaderCmdReadBoardName(cmd_t *p_cmd);
static void loaderCmdFlashErase(cmd_t *p_cmd);
static void loaderCmdFlashWrite(cmd_t *p_cmd);
static void loaderCmdFlashRead(cmd_t *p_cmd);
static void loaderCmdJumpToAddress(cmd_t *p_cmd);
static void loaderCmdFlashVerify(cmd_t *p_cmd);

void loaderCmdInit(void)
{
}

err_code_t checkGame(uint32_t type, uint32_t address)
{
  game_tag_type_a_t *p_tag;
  uint8_t *p_data;
  uint32_t i;
  uint16_t crc = 0;

  /* Check Address */
  if(address < _HW_DEF_FLASH_ADDR_GAME_START)
    return ERR_FLASH_INVALID_ADDR;

  if(type != *(uint32_t*) address)
    return ERR_FLASH_INVALID_TAG;

  /* Check Type & CRC16 */
  if(type == GAME_TAG_TYPE_A)
  {
    p_tag = (game_tag_type_a_t*) (address);
    p_data = (uint8_t*) p_tag->address;

    if(p_data < (uint8_t*)_HW_DEF_FLASH_ADDR_GAME_START)
    {
      return ERR_FLASH_INVALID_TAG;
    }

    for (i = 0; i < p_tag->length; i++)
    {
      utilUpdateCrc(&crc, p_data[i]);
    }

    if(crc != p_tag->crc)
    {
      return ERR_FLASH_CRC;
    }
  }
  else
  {
    return ERR_FLASH_INVALID_TAG;
  }

  return OK;
}

void loaderCmdReadVersion(cmd_t *p_cmd)
{
  uint8_t data[8] = {0, };
  uint32_t i = 0;

  data[i++] = 0;
  data[i++] = 0;

  cmdSendResp(p_cmd, OK, data, i);
}

void loaderCmdPacketDataSizeMax(cmd_t *p_cmd)
{
  uint8_t data[8] = {0, };
  uint16_t t_size = (uint16_t) CMD_MAX_DATA_LENGTH;
  uint32_t i = 0;

  data[i++] = (uint8_t) t_size;
  data[i++] = (uint8_t) (t_size >> 8);

  cmdSendResp(p_cmd, OK, data, i);
}

void loaderCmdReadBoardName(cmd_t *p_cmd)
{
  cmdSendResp(p_cmd, OK, (uint8_t*)_BSP_DEF_BOARD_NAME, strlen((char *)_BSP_DEF_BOARD_NAME)+1);
}

void loaderCmdFlashErase(cmd_t *p_cmd)
{
  err_code_t err_code = OK;
  uint32_t addr_begin;
  uint32_t addr_end;
  uint32_t length;

  addr_begin = utilConvert8ToU32(&p_cmd->rx_packet.data[0]);
  length     = utilConvert8ToU32(&p_cmd->rx_packet.data[4]);
  addr_end   = addr_begin + length - 1;

  if ((addr_begin >= _HW_DEF_FLASH_ADDR_GAME_START)
      && (addr_end < _HW_DEF_FLASH_ADDR_GAME_END))
  {
    if (addr_begin % 4 == 0)
    {
      err_code = flashErase(addr_begin, length);
    }
    else
    {
      err_code = ERR_FLASH_ADDR_ALIGN;
    }
  }
  else
  {
    err_code = ERR_FLASH_INVALID_ADDR;
  }

  cmdSendResp(p_cmd, err_code, NULL, 0);

}

void loaderCmdFlashWrite(cmd_t *p_cmd)
{
  err_code_t err_code = OK;
  uint32_t addr_begin;
  uint32_t addr_end;
  uint32_t length;
  uint8_t *p_flash, *p_data;

  addr_begin = utilConvert8ToU32(&p_cmd->rx_packet.data[0]);
  length     = utilConvert8ToU32(&p_cmd->rx_packet.data[4]);
  addr_end   = addr_begin + length - 1;

  if ((addr_begin >= _HW_DEF_FLASH_ADDR_GAME_START)
      && (addr_end < _HW_DEF_FLASH_ADDR_GAME_END))
  {
    if (addr_begin % 4 == 0)
    {
      err_code = flashWrite(addr_begin, &p_cmd->rx_packet.data[8], length);
    }
    else
    {
      err_code = ERR_FLASH_ADDR_ALIGN;
    }
  }
  else
  {
    err_code = ERR_FLASH_INVALID_ADDR;
  }

  if(err_code == OK)
  {
    p_flash = (uint8_t*) addr_begin;
    p_data  = &p_cmd->rx_packet.data[8];
    for(uint32_t i=0; i<length; i++)
    {
      if(p_flash[i] != p_data[i])
      {
        err_code = ERR_FLASH_WRITE;
        break;
      }
    }
  }

  cmdSendResp(p_cmd, err_code, NULL, 0);
}

void loaderCmdFlashRead(cmd_t *p_cmd)
{
  err_code_t err_code = OK;
  uint32_t addr_begin;
  uint32_t length;
  uint32_t i;
  uint8_t *p_data;

  addr_begin = utilConvert8ToU32(&p_cmd->rx_packet.data[0]);
  length     = utilConvert8ToU32(&p_cmd->rx_packet.data[4]);

  if (length > sizeof(p_cmd->tx_packet.data))
  {
    err_code = ERR_INVALID_LENGTH;
  }
  else
  {
    p_data = (uint8_t *) addr_begin;
    for (i = 0; i < length; i++)
    {
      p_cmd->tx_packet.data[i] = p_data[i];
    }
  }

  if (err_code == OK)
  {
    cmdSendResp(p_cmd, err_code, p_cmd->tx_packet.data, length);
  }
  else
  {
    cmdSendResp(p_cmd, err_code, NULL, 0);
  }
}

void loaderCmdFlashVerify(cmd_t *p_cmd)
{
  err_code_t err_code = OK;
  uint32_t addr_begin;
  uint32_t length;
  uint16_t rx_crc;
  uint16_t tx_crc = 0;
  uint8_t *p_data;
  uint32_t i;

  addr_begin = utilConvert8ToU32(&p_cmd->rx_packet.data[0]);
  length     = utilConvert8ToU32(&p_cmd->rx_packet.data[4]);
  rx_crc     = utilConvert8ToU16(&p_cmd->rx_packet.data[8]);
  p_data     = (uint8_t *) addr_begin;

  for (i = 0; i < length; i++)
  {
    utilUpdateCrc(&tx_crc, p_data[i]);
  }

  if(tx_crc != rx_crc)
  {
    err_code = ERR_FLASH_CRC;
  }

  cmdSendResp(p_cmd, err_code, (uint8_t *)&tx_crc, 2);
}


void loaderCmdJumpToAddress(cmd_t *p_cmd)
{
#if 0
  uint32_t address = utilConvert8ToU32(&p_cmd->rx_packet.data[0]);

  err_code_t err = checkGame(FW_TAG_TYPE_A, address);

  cmdSendResp(p_cmd, err, NULL, 0);
  if(err == OK)
  {
    game_tag_type_a_t *p_tag = (game_tag_type_a_t*) (address);
    delay(100);
    bspJumpToAddress(p_tag->address);
  }
#endif
}


void loaderCmdProcess(cmd_t *p_cmd)
{
  switch (p_cmd->rx_packet.cmd)
  {
    case LOADER_CMD_READ_VERSION:
      loaderCmdReadVersion(p_cmd);
      break;

    case LOADER_CMD_PACKET_DATA_SIZE_MAX:
      loaderCmdPacketDataSizeMax(p_cmd);
      break;

    case LOADER_CMD_READ_BOARD_NAME:
      loaderCmdReadBoardName(p_cmd);
      break;

    case LOADER_CMD_FLASH_ERASE:
      loaderCmdFlashErase(p_cmd);
      break;

    case LOADER_CMD_FLASH_WRITE:
      loaderCmdFlashWrite(p_cmd);
      break;

    case LOADER_CMD_FLASH_READ:
      loaderCmdFlashRead(p_cmd);
      break;

    case LOADER_CMD_FLASH_VERIFY:
      loaderCmdFlashVerify(p_cmd);
      break;

    case LOADER_CMD_JUMP:
      loaderCmdJumpToAddress(p_cmd);
      break;

    default:
      cmdSendResp(p_cmd, LOADER_ERR_INVAILD_CMD, NULL, 0);
      break;
  }
}

