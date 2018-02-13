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
#include "image/oroca_logo.h"

const volatile __attribute__((section(".version_str"))) uint8_t fw_version_str[256] = _DEF_APP_VER_STR;
const volatile __attribute__((section(".version_num"))) uint8_t fw_version_num[256] = _DEF_APP_VER_NUM;



//-- External Variables
//

//-- Internal Functions
void drawLogo(void);


//-- External Functions
extern void swtimerISR(void);


void apInit(void)
{
  timerSetPeriod(_DEF_TIMER2, 1000);
  timerAttachInterrupt(_DEF_TIMER2, swtimerISR);
  timerStart(_DEF_TIMER2);

  drawLogo();
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

void drawLogo(void)
{
  uint32_t x_offset;
  uint32_t y_offset;
  uint32_t step;


  lcdSelectLayer(_DEF_LCD_LAYER2);

  x_offset = (lcdGetXSize() - LOGO_WIDTH ) / 2;
  y_offset = (lcdGetYSize() - LOGO_HEIGHT) / 2;


  for (step = 0; step <= 200; step += 8)
  {
    lcdClear(0x0000);
    for(uint16_t x = 0; x < LOGO_WIDTH; x++)
    {
      for(uint16_t y = 0; y < LOGO_HEIGHT; y++)
      {
        lcdDrawPixel(x + x_offset, y + y_offset + 199-step, (uint32_t)oroca_img[y*LOGO_WIDTH + x]);
      }
    }
    lcdCopyLayer(_DEF_LCD_LAYER2, _DEF_LCD_LAYER1);
  }

}


