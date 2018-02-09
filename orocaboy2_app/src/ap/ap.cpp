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
  while(1)
  {
    ledToggle(_DEF_LED1);
    delay(500);
  }
}



extern "C" void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
  ( void ) pcTaskName;
  ( void ) pxTask;


  cmdifPrintf("stack over : %s\n", pcTaskName);
  for( ;; )
  {
    ledOn(0);
    ledOff(1);
    delay(100);
    ledOn(1);
    ledOff(0);
    delay(100);
  }
}
