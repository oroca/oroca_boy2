/*
 * gameloader.c
 *
 *  Created on: 2017. 11. 23.
 *      Author: opus
 */

#include "gameloader.h"
#include "loadercmd.h"
#include "ap.h"
#include "hw.h"

//-- Internal Variables
//
#define GAME_STATE_LED _DEF_LED3


typedef struct {
    cmd_t   *cmd;
    uint8_t  uart_ch;
    uint32_t baud;
} loader_t;

cmd_t cmd_loader_ch1;

loader_t boot_tbl[] =
{
    {&cmd_loader_ch1, _DEF_UART1, 57600},
    {NULL, 0, 0}
};


static void gameloaderSetup(void);


void gameloaderInit(void)
{
  gameloaderSetup();
}


void gameloaderProcess(void)
{
  uint8_t loader_ch = 0;

  while(boot_tbl[loader_ch].cmd != NULL)
  {
    if(cmdReceivePacket(boot_tbl[loader_ch].cmd))
    {
      if(boot_tbl[loader_ch].cmd->packet_err == OK)
      {
        loaderCmdProcess(boot_tbl[loader_ch].cmd);
      }
      else
      {
        cmdFlush(boot_tbl[loader_ch].cmd);
        cmdSendResp(boot_tbl[loader_ch].cmd, boot_tbl[loader_ch].cmd->packet_err, NULL, OK);
      }
    }
    loader_ch++;
  }
}

static void gameloaderSetup(void)
{
  uint8_t loader_ch = 0;

  while(boot_tbl[loader_ch].cmd != NULL)
  {
    cmdInit(boot_tbl[loader_ch].cmd);
    cmdBegin(boot_tbl[loader_ch].cmd, boot_tbl[loader_ch].uart_ch, boot_tbl[loader_ch].baud);
    loader_ch++;
  }
}
