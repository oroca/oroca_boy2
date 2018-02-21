/*
 *  download.c
 *
 *  Created on: 2016. 7. 13.
 *      Author: Baram
 */


#include "loader.h"
#include "cmd.h"
#include "util.h"
#include "uart.h"

#define FLASH_WRITABLE_DEFAULT_SIZE_MAX (512*1024)


static FILE      *fw_fp;
static uint32_t   fw_fpsize;

int download(int argc, char *argv[])
{
  UNUSED(argc);

  bool ret;
  uint32_t i;
  uint32_t time_pre;
  err_code_t errcode;

  uint16_t tx_crc = 0;
  uint16_t rx_crc = 0;

  uint16_t packet_data_size_max = CMD_MAX_DATA_LENGTH;
  uint16_t flash_tx_block_size_max = packet_data_size_max - 8; // 8 : address(4), address length(4)

  char * portname     = (char *)argv[ _DEF_ARG_PORT ];
  uint32_t baud       = strtoul((const char * ) argv[ _DEF_ARG_BAUD ], (char **)NULL, (int) 0 );

  bool en_verify      = (bool)strtoul((const char * ) argv[ _DEF_ARG_VERI ], (char **)NULL, (int) 0 );
  
  uint32_t fw_addr    = strtoul((const char * ) argv[ _DEF_ARG_ADDR ], (char **)NULL, 16 );

  char *src_filename  = (char *)argv[ _DEF_ARG_FILE ];
  char dst_filename[strlen(src_filename) + 6];


  printf("\r\n@ Make binary (Add Tag)...\r\n");
  strcpy(dst_filename, src_filename);
  strcat(dst_filename, ".cmfw");

  if(loaderCmdAddTagToBIN(fw_addr, src_filename, dst_filename) != OK)
  {
    fprintf( stderr, "  Add tag info to binary Fail! \n");
    exit( 1 );
  }


  printf("\r\n@ File check...\r\n");
  if( ( fw_fp = fopen( dst_filename, "rb" ) ) == NULL )
  {
    fprintf( stderr, "  Unable to open %s\n", dst_filename );
    exit( 1 );
  }
  else
  {
    fseek( fw_fp, 0, SEEK_END );
    fw_fpsize = ftell( fw_fp );
    fseek( fw_fp, 0, SEEK_SET );

    printf("  file name \t: %s \n", dst_filename);
    printf("  file size \t: %d KB (0x%X)\n", fw_fpsize/1024, fw_fpsize);
  }


  printf("\r\n@ Loader init...\r\n");
  printf("  port name \t: %s \n", portname);
  printf("  port baud \t: %d \n", baud);

  uartInit();

  ret = loaderInit(_DEF_UART1, portname, baud);
  if (ret == true)
  {
    printf("  loader init \t: OK\n");
  }
  else
  {
    printf("  loader init \t: Fail\n");
    return 1;
  }


  while(1)
  {
    printf("\r\n@ Target info...\r\n");
    //-- 버전 확인
    //
    uint8_t version_num[2];

    errcode = loaderCmdReadVersion(version_num);
    if (errcode == OK)
    {
      printf("  loader ver \t: %d\n", (int)version_num[0]);
      printf("  fw   ver \t: %d\n", (int)version_num[1]);
    }
    else
    {
      if(errcode == ERRCODE_TIMEOUT)
      {
        printf("  loaderCmdReadVersion fail : TIMEOUT \n");
      }
      else
      {
        printf("  loaderCmdReadVersion fail : errcode %d\n", errcode);
      }
      break;
    }


    //-- 패킷 최대 사이즈 확인
    //
    errcode = loaderCmdReadPacketDataSizeMax(&packet_data_size_max);
    if (errcode == OK)
    {
      if(packet_data_size_max > CMD_MAX_DATA_LENGTH)
      {
        packet_data_size_max = CMD_MAX_DATA_LENGTH;
      }

      flash_tx_block_size_max  = packet_data_size_max - 8;
      flash_tx_block_size_max -= (flash_tx_block_size_max % 128);
      printf("  Data Size Max : %d\n", (int)packet_data_size_max);
      printf("  Flash Tx Max  : %d\n", (int)flash_tx_block_size_max);
    }
    else
    {
      if(errcode == ERRCODE_TIMEOUT)
      {
        printf("  loaderCmdReadVersion fail : TIMEOUT \n");
      }
      else
      {
        printf("  loaderCmdReadVersion fail : errcode %d\n", errcode);
      }
      break;
    }

    //-- 보드 이름 확인
    //
    uint8_t  board_str[16];

    errcode = loaderCmdReadBoardName(board_str);
    if (errcode == OK)
    {
      printf("  board name \t: %s\n", board_str);
    }
    else
    {
      if(errcode == ERRCODE_TIMEOUT)
      {
        printf("  loaderCmdReadBoardName fail : TIMEOUT \n");
      }
      else
      {
        printf("  loaderCmdReadBoardName fail : errcode %d\n", errcode);
      }
      break;
    }


    //-- Flash Erase
    //
    printf("\r\n@ Erase fw...\n");
    printf("  address   \t: 0x%08X\n", fw_addr);
    printf("  size      \t: %d KB (0x%X)\n", fw_fpsize/1024, fw_fpsize);

    for (i=0; i<1; i++)
    {
      time_pre = millis();
      errcode = loaderCmdFlashErase(fw_addr, fw_fpsize);
      if(errcode == OK)
      {
        break;
      }
    }

    if (errcode == OK)
    {
      if (i > 0)
      {
        printf("  erase fw ret \t: OK (%d ms), retry %d\n", millis()-time_pre, i);
      }
      else
      {
        printf("  erase fw ret \t: OK (%d ms)\n", millis()-time_pre);
      }
    }
    else
    {
      if(errcode == ERRCODE_TIMEOUT)
      {
        printf("  loaderCmdFlashErase fail : TIMEOUT \n");
      }
      else
      {
        printf("  loaderCmdFlashErase fail : errcode %d\n", errcode);
      }
      break;
    }


    //-- Flash Write
    //
    printf("\r\n@ Write game...\n");
    printf("  address   \t: 0x%08X\n", fw_addr);
    printf("  size      \t: %d KB (0x%X)\n", fw_fpsize/1024, fw_fpsize);

    uint32_t addr_cnt = 0;
    uint32_t percent = 0;
    size_t readbytes = 0;
    bool flash_write_done = false;
    uint8_t block_buf[flash_tx_block_size_max];

    time_pre = millis();

    while(1)
    {
      if( !feof( fw_fp ) )
      {
        readbytes = fread( block_buf, 1, flash_tx_block_size_max, fw_fp );
      }
      else
      {
        break;
      }

      if( readbytes == 0 )
      {
        break;
      }

      for (i=0; i<3; i++)
      {
        errcode = loaderCmdFlashWrite(fw_addr+addr_cnt, block_buf, readbytes);
        if( errcode == OK )
        {
          percent = (addr_cnt + readbytes) * 100/fw_fpsize;
          printf("  flash fw \t: %d %%\r", percent);
          break;
        }
        else
        {
          printf("  flash fw retry 0x%X 0x%X %d\n", errcode, fw_addr+addr_cnt, (int)readbytes);
        }
      }
      if( errcode != OK )
      {
        printf("  flash fw fail addr cnt : %d (%d) \n", addr_cnt, (int)readbytes);
        break;
      }

      if(en_verify)
      {
        for (i=0; i<readbytes; i++)
        {
          utilUpdateCrc(&tx_crc, block_buf[i]);
        }
      }

      addr_cnt += readbytes;

      if (addr_cnt == fw_fpsize)
      {
        flash_write_done = true;
        break;
      }
    }

    if( (errcode != OK) || (flash_write_done == false) )
    {
      if(errcode == ERRCODE_TIMEOUT)
      {
        printf("\r\n  flash fw fail : TIMEOUT \n");
      }
      else
      {
        printf("\r\n  flash fw fail : errcode %d\r\n", errcode);
      }
      break;
    }
    else
    {
      printf("\r\n  flash fw ret  : OK (%d ms) \r\n", millis()-time_pre);

    }



    //-- Verify Fw
    //
    if(en_verify)
    {
      printf("\r\n@ Verify game...\n");
      printf("  address   \t: 0x%08X\n", fw_addr);
      printf("  size      \t: %d KB (0x%X)\n", fw_fpsize/1024, fw_fpsize);

      time_pre = millis();
      errcode = loaderCmdFlashVerfy(fw_addr, fw_fpsize, tx_crc, &rx_crc);

      printf("  tx crc    \t: %d (0x%04X)\n", tx_crc, tx_crc);
      printf("  rx crc    \t: %d (0x%04X)\n", rx_crc, rx_crc);

      if (errcode == OK)
      {
        printf("  verify game ret : OK (%d ms)\n", millis()-time_pre);
      }
      else
      {
        errcode = loaderCmdFlashVerfy(fw_addr, fw_fpsize, tx_crc, &rx_crc);

        printf("  retry     \t\n");
        printf("  tx crc    \t: %d (0x%04X)\n", tx_crc, tx_crc);
        printf("  rx crc    \t: %d (0x%04X)\n", rx_crc, rx_crc);

        if(errcode == OK)
        {

        }
        else if(errcode == ERRCODE_TIMEOUT)
        {
          printf("  verify game fail: TIMEOUT \n");
        }
        else
        {
          printf("  verify game fail: errcode %d\n", errcode);
        }
        break;
      }
    }



    if( errcode != OK || flash_write_done == false )
    {
      printf("\r\n@ Download \t: Fail\r\n");
      return -2;
    }
    else
    {
      printf("\r\n@ download \t: OK\r\n");
    }

    break;
  }

  fclose( fw_fp );
  uartClose(_DEF_UART1);

  return 0;
}


int downloadAddTagFile(int argc, char *argv[])
{
  uint32_t fw_addr  = strtoul((const char * ) argv[ _TAG_ARG_FW_ADDR ], (char **)NULL, 16 );
  char * src_filename = (char *)argv[ _TAG_ARG_SRC_FILE ];
  char * dst_filename = (char *)argv[ _TAG_ARG_DST_FILE ];

  UNUSED(argc);

  if(loaderCmdAddTagToBIN(fw_addr, src_filename, dst_filename) != OK)
  {
    fprintf( stderr, "  Add tag info to binary Fail! \n");
    exit( 1 );
  }

  return 0;
}
