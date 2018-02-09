/*
 * ap.cpp
 *
 *  Created on: 2017. 2. 13.
 *      Author: baram
 */

#include "ap.h"
#include "hw.h"
#include "ap_def.h"
//#include "test_def.h"



//-- External Variables
//

//-- Internal Functions

//-- External Functions
extern void swtimerISR(void);

void apInit(void)
{
  timerSetPeriod(_DEF_TIMER2, 1000);
  timerAttachInterrupt(_DEF_TIMER2, swtimerISR);
  timerStart(_DEF_TIMER2);

  bootInit();
}

void apMain(void)
{
  while(1)
  {
    bootProcess();
  }
}

