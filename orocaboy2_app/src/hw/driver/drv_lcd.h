/**
  ******************************************************************************
  * @file    stm32469i_discovery_lcd.h
  * @author  MCD Application Team
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32469i_discovery_lcd.c driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */


/*
 * drv_lcd.h
 *
 *  Created on: Feb 10, 2018
 *      Author: opus
 */

#ifndef DRV_LCD_H_
#define DRV_LCD_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "hw.h"

#if 0
/**
  * @brief  LCD color definitions values
  * in ARGB8888 format.
  */

#define LCD_COLOR_BLUE          ((uint32_t) 0xFF0000FF)
#define LCD_COLOR_GREEN         ((uint32_t) 0xFF00FF00)
#define LCD_COLOR_RED           ((uint32_t) 0xFFFF0000)
#define LCD_COLOR_CYAN          ((uint32_t) 0xFF00FFFF)
#define LCD_COLOR_MAGENTA       ((uint32_t) 0xFFFF00FF)
#define LCD_COLOR_YELLOW        ((uint32_t) 0xFFFFFF00)
#define LCD_COLOR_LIGHTBLUE     ((uint32_t) 0xFF8080FF)
#define LCD_COLOR_LIGHTGREEN    ((uint32_t) 0xFF80FF80)
#define LCD_COLOR_LIGHTRED      ((uint32_t) 0xFFFF8080)
#define LCD_COLOR_LIGHTCYAN     ((uint32_t) 0xFF80FFFF)
#define LCD_COLOR_LIGHTMAGENTA  ((uint32_t) 0xFFFF80FF)
#define LCD_COLOR_LIGHTYELLOW   ((uint32_t) 0xFFFFFF80)
#define LCD_COLOR_DARKBLUE      ((uint32_t) 0xFF000080)
#define LCD_COLOR_DARKGREEN     ((uint32_t) 0xFF008000)
#define LCD_COLOR_DARKRED       ((uint32_t) 0xFF800000)
#define LCD_COLOR_DARKCYAN      ((uint32_t) 0xFF008080)
#define LCD_COLOR_DARKMAGENTA   ((uint32_t) 0xFF800080)
#define LCD_COLOR_DARKYELLOW    ((uint32_t) 0xFF808000)
#define LCD_COLOR_WHITE         ((uint32_t) 0xFFFFFFFF)
#define LCD_COLOR_LIGHTGRAY     ((uint32_t) 0xFFD3D3D3)
#define LCD_COLOR_GRAY          ((uint32_t) 0xFF808080)
#define LCD_COLOR_DARKGRAY      ((uint32_t) 0xFF404040)
#define LCD_COLOR_BLACK         ((uint32_t) 0xFF000000)
#define LCD_COLOR_BROWN         ((uint32_t) 0xFFA52A2A)
#define LCD_COLOR_ORANGE        ((uint32_t) 0xFFFFA500)
#define LCD_COLOR_TRANSPARENT   ((uint32_t) 0xFF000000)

#else
#define LCD_COLOR_BLUE          ((uint32_t) 0x4439)
#define LCD_COLOR_GREEN         ((uint32_t) 0x044A)
#define LCD_COLOR_RED           ((uint32_t) 0xD8E4)
#define LCD_COLOR_CYAN          ((uint32_t) 0xFF00FFFF)
#define LCD_COLOR_MAGENTA       ((uint32_t) 0xFFFF00FF)
#define LCD_COLOR_YELLOW        ((uint32_t) 0xFFFFFF00)
#define LCD_COLOR_LIGHTBLUE     ((uint32_t) 0xFF8080FF)
#define LCD_COLOR_LIGHTGREEN    ((uint32_t) 0xFF80FF80)
#define LCD_COLOR_LIGHTRED      ((uint32_t) 0xFFFF8080)
#define LCD_COLOR_LIGHTCYAN     ((uint32_t) 0xFF80FFFF)
#define LCD_COLOR_LIGHTMAGENTA  ((uint32_t) 0xFFFF80FF)
#define LCD_COLOR_LIGHTYELLOW   ((uint32_t) 0xFFFFFF80)
#define LCD_COLOR_DARKBLUE      ((uint32_t) 0xFF000080)
#define LCD_COLOR_DARKGREEN     ((uint32_t) 0xFF008000)
#define LCD_COLOR_DARKRED       ((uint32_t) 0xFF800000)
#define LCD_COLOR_DARKCYAN      ((uint32_t) 0xFF008080)
#define LCD_COLOR_DARKMAGENTA   ((uint32_t) 0xFF800080)
#define LCD_COLOR_DARKYELLOW    ((uint32_t) 0xFF808000)
#define LCD_COLOR_WHITE         ((uint32_t) 0xFFFFFFFF)
#define LCD_COLOR_LIGHTGRAY     ((uint32_t) 0xFFD3D3D3)
#define LCD_COLOR_GRAY          ((uint32_t) 0xFF808080)
#define LCD_COLOR_DARKGRAY      ((uint32_t) 0xFF404040)
#define LCD_COLOR_BLACK         ((uint32_t) 0xFF000000)
#define LCD_COLOR_BROWN         ((uint32_t) 0xCC68)
#define LCD_COLOR_ORANGE        ((uint32_t) 0xFFFFA500)
#define LCD_COLOR_TRANSPARENT   ((uint32_t) 0xFF000000)


#endif

err_code_t drvLcdInit(uint8_t orientation);
void       drvLcdReset(void);
err_code_t drvLcdInitLayer(uint16_t layer_idx, uint32_t fb_addr);

err_code_t drvLcdSelectLayer(uint32_t layer_idx);

uint32_t   drvLcdReadPixel(uint16_t x_pos, uint16_t y_pos);
void       drvLcdDrawPixel(uint16_t x_pos, uint16_t y_pos, uint32_t rgb_code);
void       drvLcdClear(uint32_t rgb_code);
void       drvLcdCopyLayer(uint32_t src_index, uint32_t dst_index);

void drvLcdFillRect(uint16_t x_pos, uint16_t y_pos, uint16_t width, uint16_t height, uint16_t rgb_code);

bool       drvLcdDrawAvailable(void);
void       drvLcdOnDoubleBuffering(bool enable);

err_code_t drvLcdSetLayerAddr(uint32_t layer_idx, uint32_t addr);
err_code_t drvLcdSetLayerWindow(uint16_t layer_idx, uint16_t x_pos, uint16_t y_pos, uint16_t width, uint16_t height);
err_code_t drvLcdSetLayerVisible(uint32_t layer_idx, uint8_t state);

err_code_t drvLcdSetTransparency(uint32_t layer_idx, uint8_t transparency);

void       drvLcdDisplayOff(void);
void       drvLcdDisplayOn(void);

uint32_t   drvLcdGetXSize(void);
uint32_t   drvLcdGetYSize(void);
void       drvLcdSetXSize(uint32_t image_width_pixels);
void       drvLcdSetYSize(uint32_t image_height_pixels);


#ifdef __cplusplus
}
#endif



#endif /* DRV_LCD_H_ */
