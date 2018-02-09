/*
 * vcp.c
 *
 *  Created on: Feb 08, 2018
 *      Author: opus
 */



#include <stdarg.h>
#include <stdbool.h>

#include "hw.h"
#include "vcp.h"
#include "printf.h"





//-- Internal Variables
//


//-- External Variables
//


//-- Internal Functions
//



//-- External Functions
//





int __io_putchar(int ch)
{
  vcpPutch(ch);

  return 1;
}

int __io_getchar(void)
{
  return vcpGetch();
}



