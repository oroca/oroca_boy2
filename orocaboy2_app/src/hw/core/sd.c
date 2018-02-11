/*
 * led.c
 *
 *  Created on: 2017. 2. 13.
 *      Author: baram
 */





#include <sd.h>

#ifdef _USE_HW_SD
#include <stdbool.h>
#include "driver/drv_sd.h"
#include "hw.h"




//-- Internal Variables
//
static bool is_init = false;


//-- External Variables
//


//-- Internal Functions
//
#ifdef _USE_HW_CMDIF_SD
void sdCmdifInit(void);
int  sdCmdif(int argc, char **argv);
#endif



//-- External Functions
//




bool sdInit(void)
{
  is_init = drvSdInit();



#ifdef _USE_HW_CMDIF_SD
  static bool is_cmd_init = false;

  if (is_cmd_init == false)
  {
    sdCmdifInit();
    is_cmd_init = true;
  }
#endif

  return is_init;
}




bool sdReadBlocks(uint32_t block_addr, uint8_t *p_data, uint32_t num_of_blocks, uint32_t timeout_ms)
{
  return drvSdReadBlocks(block_addr, p_data, num_of_blocks, timeout_ms);
}

bool sdWriteBlocks(uint32_t block_addr, uint8_t *p_data, uint32_t num_of_blocks, uint32_t timeout_ms)
{
  return drvSdWriteBlocks(block_addr, p_data, num_of_blocks, timeout_ms);
}

bool sdEraseBlocks(uint32_t start_addr, uint32_t end_addr)
{
  return drvSdEraseBlocks(start_addr, end_addr);
}

bool sdIsBusy(void)
{
  return drvSdIsBusy();
}

bool sdIsDetected(void)
{
  return drvSdIsDetected();
}

bool sdGetInfo(sd_info_t *p_info)
{
  return drvSdGetInfo((void *)p_info);
}




#ifdef _USE_HW_CMDIF_SD
void sdCmdifInit(void)
{
  if (cmdifIsInit() == false)
  {
    cmdifInit();
  }
  cmdifAdd("sd", sdCmdif);
}

int sdCmdif(int argc, char **argv)
{
  bool ret = true;
  sd_info_t sd_info;


  if (argc == 2 && strcmp("info", argv[1]) == 0)
  {
    cmdifPrintf("sd init      : %d\n", is_init);
    cmdifPrintf("sd connected : %d\n", sdIsDetected());

    if (is_init == true)
    {
      if (sdGetInfo(&sd_info) == true)
      {
        cmdifPrintf("  card_type            : %d\n", sd_info.card_type);
        cmdifPrintf("  card_version         : %d\n", sd_info.card_version);
        cmdifPrintf("  card_class           : %d\n", sd_info.card_class);
        cmdifPrintf("  rel_card_Add         : %d\n", sd_info.rel_card_Add);
        cmdifPrintf("  block_numbers        : %d\n", sd_info.block_numbers);
        cmdifPrintf("  block_size           : %d\n", sd_info.block_size);
        cmdifPrintf("  log_block_numbers    : %d\n", sd_info.log_block_numbers);
        cmdifPrintf("  log_block_size       : %d\n", sd_info.log_block_size);
        cmdifPrintf("  card_size            : %d MB, %d.%d GB\n", sd_info.card_size, sd_info.card_size/1024, ((sd_info.card_size * 10)/1024) % 10);
      }
    }
  }
  else
  {
    ret = false;
  }

  if (ret == false)
  {
    cmdifPrintf( "sd info \n");
  }

  return 0;
}
#endif /* _USE_HW_CMDIF_SD */

#endif /* _USE_HW_SD */
