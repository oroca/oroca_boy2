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

  // TODO: FreeRTOS���� newlib-nano ���� strtok �Լ� ���� �ϵ���Ʈ �߻�
  //       �׷��� �ӽ÷� newlib ������� ���� �Ͽ���. �Ʒ� ��ũ �����Ͽ� �ٲ� �ʿ� ����.
  //       https://mcuoneclipse.com/2017/07/02/using-freertos-with-newlib-and-newlib-nano/
  cmdifBegin(_DEF_UART1, 57600);
}

static void threadMain(void const *argument)
{
  UNUSED(argument);

  apInit();

  apMain();
}

