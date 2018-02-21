/*
 * main.cpp
 *
 *  Created on: 2017. 2. 13.
 *      Author: baram
 */



#include "main.h"
#include "rtos.h"





//-- Internal Variables
//



//-- External Variables
//


//-- Internal Functions
//
static void mainInit(void);
static void threadMain(void const *argument);

//-- External Functions
//





int main(void)
{
  mainInit();


  osThreadDef(threadMain, threadMain, osPriorityNormal, 0, _HW_DEF_RTOS_MEM_SIZE(12*1024));
  osThreadCreate(osThread(threadMain), NULL);

  osKernelStart();


  return 0;
}

static void mainInit(void)
{
  bspInit();
  hwInit();

  /* TODO:FreeRTOS에서 newlib-nano 사용시 strtok 함수 사용시 하드폴트 발생
      그래서 임시로 newlib 사용으로 변경 하였음. 아래 링크 찹조하여 바꿀 필요 있음.
     https://mcuoneclipse.com/2017/07/02/using-freertos-with-newlib-and-newlib-nano/
     */
  //cmdifBegin(_DEF_UART1, 57600);
}

static void threadMain(void const *argument)
{
  UNUSED(argument);

  apInit();

  apMain();
}

