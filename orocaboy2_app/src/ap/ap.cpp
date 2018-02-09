/*
 * ap.cpp
 *
 *  Created on: 2017. 2. 13.
 *      Author: baram
 */

#include "ap.h"
#include "hw.h"
#include "ap_def.h"
#include "rtos.h"

const volatile __attribute__((section(".version_str"))) uint8_t fw_version_str[256] = _DEF_APP_VER_STR;
const volatile __attribute__((section(".version_num"))) uint8_t fw_version_num[256] = _DEF_APP_VER_NUM;



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
}

void apMain(void)
{
  uint32_t pre_time;


  pre_time = millis();
  while(1)
  {
    cmdifMain();


    if (millis()-pre_time >= 500)
    {
      pre_time = millis();
      ledToggle(_DEF_LED1);
    }
  }
}




