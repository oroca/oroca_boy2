/*
 * fatfs.c
 *
 *  Created on: 2017. 2. 13.
 *      Author: baram
 */





#include <sd.h>

#ifdef _USE_HW_FATFS
#include <stdbool.h>
#include "hw.h"
#include "lib/FatFs/src/ff_gen_drv.h"
#include "driver/sd_diskio.h"


//-- Internal Variables
//
static bool is_init = false;

FATFS SDFatFs;    /* File system object for SD disk logical drive */
char  SDPath[4];  /* SD disk logical drive path */


//-- External Variables
//


//-- Internal Functions
//
#ifdef _USE_HW_CMDIF_SD
void fatfsCmdifInit(void);
int  fatfsCmdif(int argc, char **argv);
#endif

static void fatfsPrintErr(FRESULT res);
static FRESULT fatfsScanFiles(char* path);



//-- External Functions
//




bool fatfsInit(void)
{
  is_init = false;



  if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
  {
    if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) == FR_OK)
    {
      is_init = true;
    }
  }

#ifdef _USE_HW_CMDIF_FATFS
  fatfsCmdifInit();
#endif

  return is_init;
}


void fatfsPrintErr(FRESULT res)
{
  switch (res)
  {
    case FR_DISK_ERR:
      cmdifPrintf("FR_DISK_ERR\n");
      break;

    case FR_INT_ERR:
      cmdifPrintf("FR_INT_ERR\n");
      break;

    case FR_NOT_READY:
      cmdifPrintf("FR_NOT_READY\n");
      break;

    case FR_NO_FILE:
      cmdifPrintf("FR_NO_FILE\n");
      break;

    case FR_NO_PATH:
      cmdifPrintf("FR_NO_PATH\n");
      break;

    case FR_INVALID_NAME:
      cmdifPrintf("FR_INVALID_NAME\n");
      break;

    case FR_DENIED:
      cmdifPrintf("FR_DENIED\n");
      break;

    case FR_EXIST:
      cmdifPrintf("FR_EXIST\n");
      break;

    case FR_INVALID_OBJECT:
      cmdifPrintf("FR_INVALID_OBJECT\n");
      break;

    case FR_WRITE_PROTECTED:
      cmdifPrintf("FR_WRITE_PROTECTED\n");
      break;

    case FR_INVALID_DRIVE:
      cmdifPrintf("FR_INVALID_DRIVE\n");
      break;

    case FR_NOT_ENABLED:
      cmdifPrintf("FR_NOT_ENABLED\n");
      break;

    case FR_NO_FILESYSTEM:
      cmdifPrintf("FR_NO_FILESYSTEM\n");
      break;

    case FR_MKFS_ABORTED:
      cmdifPrintf("FR_MKFS_ABORTED\n");
      break;

    case FR_TIMEOUT:
      cmdifPrintf("FR_TIMEOUT\n");
      break;

    case FR_LOCKED:
      cmdifPrintf("FR_LOCKED\n");
      break;

    case FR_NOT_ENOUGH_CORE:
      cmdifPrintf("FR_NOT_ENOUGH_CORE\n");
      break;

    case FR_TOO_MANY_OPEN_FILES:
      cmdifPrintf("FR_NOT_ENOUGH_CORE\n");
      break;

    case FR_INVALID_PARAMETER:
      cmdifPrintf("FR_INVALID_PARAMETER\n");
      break;
  }
}

FRESULT fatfsScanFiles(char* path)
{
  FRESULT res;
  DIR dir;
  UINT i;
  static FILINFO fno;


  res = f_opendir(&dir, path);                       /* Open the directory */
  if (res == FR_OK)
  {
    for (;;)
    {
      res = f_readdir(&dir, &fno);                   /* Read a directory item */
      if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
      if (fno.fattrib & AM_DIR)
      {                    /* It is a directory */
        i = strlen(path);
        sprintf(&path[i], "/%s", fno.fname);
        res = fatfsScanFiles(path);                    /* Enter the directory */
        if (res != FR_OK) break;
        path[i] = 0;
      }
      else
      {                                       /* It is a file. */
        cmdifPrintf(" %s/%s \t%d bytes\n", path, fno.fname, fno.fsize);
      }
    }
    f_closedir(&dir);
  }

  return res;
}



#ifdef _USE_HW_CMDIF_FATFS
void fatfsCmdifInit(void)
{
  if (cmdifIsInit() == false)
  {
    cmdifInit();
  }
  cmdifAdd("fatfs", fatfsCmdif);
}

int fatfsCmdif(int argc, char **argv)
{
  bool ret = true;
  sd_info_t sd_info;
  uint8_t buf[255];

  if (argc == 2 && strcmp("info", argv[1]) == 0)
  {
    cmdifPrintf("fatfs init      : %d\n", is_init);

    if (is_init == true)
    {
      FATFS *fs;
      DWORD fre_clust, fre_sect, tot_sect;
      FRESULT res;

      /* Get volume information and free clusters of drive 1 */
      res = f_getfree("", &fre_clust, &fs);
      if (res == FR_OK)
      {
        /* Get total sectors and free sectors */
        tot_sect = (fs->n_fatent - 2) * fs->csize;
        fre_sect = fre_clust * fs->csize;

        /* Print the free space (assuming 512 bytes/sector) */
        cmdifPrintf("%10lu KiB total drive space.\n%10lu KiB available.\n", tot_sect / 2, fre_sect / 2);
      }
      else
      {
        cmdifPrintf(" err : ");
        fatfsPrintErr(res);
      }
    }
  }
  else if (argc == 2 && strcmp("dir", argv[1]) == 0)
  {
    FRESULT res;

    res = fatfsScanFiles("/sd");

    if (res != FR_OK)
    {
      cmdifPrintf(" err : ");
      fatfsPrintErr(res);
    }
  }
  else
  {
    ret = false;
  }

  if (ret == false)
  {
    cmdifPrintf( "fatfs info \n");
    cmdifPrintf( "fatfs dir \n");
  }

  return 0;
}
#endif /* _USE_HW_CMDIF_FATFS */

#endif /* _USE_HW_FATFS */
