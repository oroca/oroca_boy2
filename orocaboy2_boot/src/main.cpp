/*
 * main.cpp
 *
 *  Created on: 2017. 2. 13.
 *      Author: baram
 */



#include "main.h"





//-- Internal Variables
//



//-- External Variables
//


//-- Internal Functions
//
void mainInit(void);

//-- External Functions
//





int main(void)
{
  mainInit();

  apMain();

  return 0;
}

void mainInit(void)
{
  bspInit();
  hwInit();
  apInit();
}

