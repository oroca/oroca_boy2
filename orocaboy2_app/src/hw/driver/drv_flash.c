/*
 *  drv_flash.c
 *
 *  Created on: Feb 08, 2018
 *      Author: opus
 */




#include "drv_flash.h"

#include "hw.h"

typedef struct
{
  uint32_t addr;
  uint32_t length;
} flash_sector_attr_t;




uint32_t            flash_sector_total = FLASH_SECTOR_TOTAL;
flash_sector_attr_t   flash_sector_attr[FLASH_SECTOR_TOTAL];



static err_code_t drvFlashEraseSector(uint32_t sector);







bool drvFlashInit(void)
{
  flash_sector_attr[0].addr   = 0x08000000;
  flash_sector_attr[0].length = 16*1024;

  flash_sector_attr[1].addr   = 0x08004000;
  flash_sector_attr[1].length = 16*1024;

  flash_sector_attr[2].addr   = 0x08008000;
  flash_sector_attr[2].length = 16*1024;

  flash_sector_attr[3].addr   = 0x0800C000;
  flash_sector_attr[3].length = 16*1024;

  flash_sector_attr[4].addr   = 0x08010000;
  flash_sector_attr[4].length = 64*1024;

  flash_sector_attr[5].addr   = 0x08020000;
  flash_sector_attr[5].length = 128*1024;

  flash_sector_attr[6].addr   = 0x08040000;
  flash_sector_attr[6].length = 128*1024;

  flash_sector_attr[7].addr   = 0x08060000;
  flash_sector_attr[7].length = 128*1024;

  flash_sector_attr[8].addr   = 0x08080000;
  flash_sector_attr[8].length = 128*1024;

  flash_sector_attr[9].addr   = 0x080A0000;
  flash_sector_attr[9].length = 128*1024;

  flash_sector_attr[10].addr   = 0x080C0000;
  flash_sector_attr[10].length = 128*1024;

  flash_sector_attr[11].addr   = 0x080E0000;
  flash_sector_attr[11].length = 128*1024;

  flash_sector_attr[12].addr   = 0x08100000;
  flash_sector_attr[12].length = 16*1024;

  flash_sector_attr[13].addr   = 0x08104000;
  flash_sector_attr[13].length = 16*1024;

  flash_sector_attr[14].addr   = 0x08108000;
  flash_sector_attr[14].length = 16*1024;

  flash_sector_attr[15].addr   = 0x0810C000;
  flash_sector_attr[15].length = 16*1024;

  flash_sector_attr[16].addr   = 0x08110000;
  flash_sector_attr[16].length = 64*1024;

  flash_sector_attr[17].addr   = 0x08120000;
  flash_sector_attr[17].length = 128*1024;

  flash_sector_attr[18].addr   = 0x08140000;
  flash_sector_attr[18].length = 128*1024;

  flash_sector_attr[19].addr   = 0x08160000;
  flash_sector_attr[19].length = 128*1024;

  flash_sector_attr[20].addr   = 0x08180000;
  flash_sector_attr[20].length = 128*1024;

  flash_sector_attr[21].addr   = 0x081A0000;
  flash_sector_attr[21].length = 128*1024;

  flash_sector_attr[22].addr   = 0x081C0000;
  flash_sector_attr[22].length = 128*1024;

  flash_sector_attr[23].addr   = 0x081E0000;
  flash_sector_attr[23].length = 128*1024;


  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

  return true;
}

err_code_t drvFlashWrite(uint32_t addr, uint8_t *p_data, uint32_t length)
{
  err_code_t err_code = OK;
  HAL_StatusTypeDef HAL_FLASHStatus = HAL_OK;
  uint32_t StartAddress = addr;
  uint32_t WriteSize;
  uint32_t WriteData;
  uint32_t i;
  uint32_t DataIndex;


  WriteSize = length / 4; // 32Bit

  if( (length%4) > 0 ) WriteSize++;

  DataIndex = 0;
  HAL_FLASH_Unlock();
  for( i=0; i<WriteSize; i++ )
  {
    WriteData  = p_data[ DataIndex++ ] << 0;
    WriteData |= p_data[ DataIndex++ ] << 8;
    WriteData |= p_data[ DataIndex++ ] << 16;
    WriteData |= p_data[ DataIndex++ ] << 24;

    HAL_FLASHStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartAddress+i*4, (uint64_t)WriteData);

    if( HAL_FLASHStatus != HAL_OK )
    {
        err_code = ERR_FLASH_WRITE;
      break;
    }
  }
  HAL_FLASH_Lock();

  return err_code;
}


err_code_t drvFlashRead(uint32_t addr, uint8_t *p_data, uint32_t length)
{
  err_code_t err_code = OK;
  uint32_t Dataindex;
  uint32_t addr_cnt;


  Dataindex = 0;
  for (addr_cnt=0;addr_cnt<length;addr_cnt++)
  {
    p_data[Dataindex++] = *(volatile uint8_t*)(addr+addr_cnt);
  }

  return err_code;
}

err_code_t drvFlashErase(uint32_t addr, uint32_t length)
{
  err_code_t err_code = OK;
  uint32_t addr_begin;
  uint32_t addr_end;
  uint32_t target_addr_begin;
  uint32_t target_addr_end;

  uint32_t i;


  target_addr_begin = addr;
  target_addr_end   = addr + length - 1;


  for (i=0; i<flash_sector_total; i++)
  {
    addr_begin = flash_sector_attr[i].addr;
    addr_end   = flash_sector_attr[i].addr + flash_sector_attr[i].length - 1;

    if ((addr_begin >= target_addr_begin) && (addr_begin <= target_addr_end))
    {
      err_code = drvFlashEraseSector(i);
    }
    else if((addr_end >= target_addr_begin) && (addr_end <= target_addr_end) )
    {
      err_code = drvFlashEraseSector(i);
    }
    else if((addr_begin >= target_addr_begin) && (addr_end <= target_addr_end) )
    {
      err_code = drvFlashEraseSector(i);
    }
    else if((addr_begin <= target_addr_begin) && (addr_end >= target_addr_end) )
    {
      err_code = drvFlashEraseSector(i);
    }

  }


  return err_code;
}

err_code_t drvFlashEraseSector(uint32_t sector)
{
  err_code_t err_code = OK;
  HAL_StatusTypeDef HAL_FLASHStatus = HAL_OK;
  FLASH_EraseInitTypeDef pEraseInit;
  uint32_t SectorError;


  pEraseInit.TypeErase  = FLASH_TYPEERASE_SECTORS;
  pEraseInit.Sector     = sector;
  pEraseInit.NbSectors  = 1;
  pEraseInit.VoltageRange = VOLTAGE_RANGE_3;


  HAL_FLASH_Unlock();


  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

  HAL_FLASHStatus = HAL_FLASHEx_Erase(&pEraseInit, &SectorError);
  if(HAL_FLASHStatus != HAL_OK)
  {
    err_code = ERR_FLASH_ERASE;
  }

  HAL_FLASH_Lock();

  return err_code;
}

err_code_t drvFlashEraseSectors(uint32_t start_sector, uint32_t sector_cnt )
{

  err_code_t err_code = OK;
  HAL_StatusTypeDef HAL_FLASHStatus = HAL_OK;
  FLASH_EraseInitTypeDef pEraseInit;
  uint32_t SectorError;


  pEraseInit.TypeErase  = FLASH_TYPEERASE_SECTORS;
  pEraseInit.Sector     = start_sector;
  pEraseInit.NbSectors  = sector_cnt;
  pEraseInit.VoltageRange = VOLTAGE_RANGE_3;


  HAL_FLASH_Unlock();

  HAL_FLASHStatus = HAL_FLASHEx_Erase(&pEraseInit, &SectorError);
  if(HAL_FLASHStatus != HAL_OK)
  {
    err_code = ERR_FLASH_ERASE;
  }

  HAL_FLASH_Lock();

  return err_code;
}

