
/**
  ******************************************************************************
  * @file    stm32469i_discovery_lcd.c
  * @author  MCD Application Team
  * @brief   This file includes the driver for Liquid Crystal Display (LCD) module
  *          mounted on STM32469I-Discovery evaluation board.
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

/* File Info: ------------------------------------------------------------------
                                   User NOTES
1. How To use this driver:
--------------------------
   - This driver is used to drive directly in video mode a LCD TFT using the DSI interface.
     The following IPs are implied : DSI Host IP block working
     in conjunction to the LTDC controller.
   - This driver is linked by construction to LCD KoD mounted on board MB1166.

2. Driver description:
---------------------
  + Initialization steps:
     o Initialize the LCD using the drvLcdInit() function.
     o Select the LCD layer to be used using the drvLcdSelectLayer() function.
     o Enable the LCD display using the BSP_LCD_DisplayOn() function.

  + Options
     o Configure and enable the color keying functionality using the
       drvLcdSetColorKeying() function.
     o Modify in the fly the transparency and/or the frame buffer address
       using the following functions:
       - drvLcdSetTransparency()
       - drvLcdSetLayerAddr()

  + Display on LCD
     o Clear the whole LCD using BSP_LCD_Clear() function or only one specified string
       line using the BSP_LCD_ClearStringLine() function.
     o Display a character on the specified line and column using the drvLcdDisplayChar()
       function or a complete string line using the drvLcdDisplayStringAtLine() function.
     o Display a string line on the specified position (x,y in pixel) and align mode
       using the drvLcdDisplayStringAtLine() function.
     o Draw and fill a basic shapes (dot, line, rectangle, circle, ellipse, .. bitmap)
       on LCD using the available set of functions.

------------------------------------------------------------------------------*/

/*
 * drv_lcd.c
 *
 *  Created on: Feb 10, 2018
 *      Author: opus
 */


/* Includes ------------------------------------------------------------------*/
#include "drv_lcd.h"
#include "drv_sdram.h"
#include "lib/otm8009a/otm8009a.h"




#define XSIZE_PHYS      800
#define YSIZE_PHYS      480

#define ZONES           4
#define VSYNC           1
#define VBP             1
#define VFP             1
#define VACT            YSIZE_PHYS
#define HSYNC           1
#define HBP             1
#define HFP             1
#define HACT            XSIZE_PHYS/ZONES      /* !!!! SCREEN DIVIDED INTO 4 AREAS !!!! */


#define LCD_LAYER_FRAME_BUFFER  ((uint32_t)_HW_DEF_LCD_ADDR_LAYER1_START)

#define LCD_FRAME_BUFFER_MAX    2



typedef struct
{
  int32_t      address;
  __IO int32_t pending_buffer;
  __IO int32_t drawing_buffer;
  int32_t      buffer_index;
  int32_t      width;
  int32_t      height;
  int32_t      bytes_per_pixel;
  int32_t      length;
  uint16_t     *p_buffer[LCD_FRAME_BUFFER_MAX];
} lcd_layer_t;


typedef struct
{
  LTDC_HandleTypeDef    hltdc;
  DSI_HandleTypeDef     hdsi;
} lcd_hw_t;


static lcd_layer_t lcd_layer;
static lcd_hw_t    lcd_hw;


static uint8_t pPage[]=
  {0x00, 0x00, 0x01, 0xDF}; /*   0 -> 479 */


#if 0
static uint8_t pCols[ZONES][4] =
{
  {0x00, 0x00, 0x00, 0xC7}, /*   0 -> 199 */
  {0x00, 0xC8, 0x01, 0x8F}, /* 200 -> 399 */
  {0x01, 0x90, 0x02, 0x57}, /* 400 -> 599 */
  {0x02, 0x58, 0x03, 0x1F}, /* 600 -> 799 */
};

static uint32_t cols_addr[4] =
    {
        0 * HACT * 2,
        1 * HACT * 2,
        2 * HACT * 2,
        3 * HACT * 2
    };

#else
static uint8_t pCols[ZONES][4] =
{
  {0x02, 0x58, 0x03, 0x1F}, /* 600 -> 799 */
  {0x01, 0x90, 0x02, 0x57}, /* 400 -> 599 */
  {0x00, 0xC8, 0x01, 0x8F}, /* 200 -> 399 */
  {0x00, 0x00, 0x00, 0xC7}, /*   0 -> 199 */
};

static uint32_t cols_addr[4] =
    {
        3 * HACT * 2,
        2 * HACT * 2,
        1 * HACT * 2,
        0 * HACT * 2
    };
#endif

volatile int32_t lcd_active_region   = 1;
volatile int32_t lcd_refershing      = 0;
volatile bool    lcd_request_draw    = false;


static void drvLcdHwInit(void);
static void drvLcdHwReset(void);
static void drvLcdMspInit(void);
static void drvLtdcInit(void);
static void drvLcdLayerInit(uint32_t LayerIndex, uint32_t address);
static void drvLcdReqTear(void);
static void drvLcdFillBuffer(void * pDst, uint32_t xSize, uint32_t ySize, uint32_t OffLine, uint32_t ColorIndex);


err_code_t drvLcdInit(void)
{
  err_code_t ret = OK;


  lcd_layer.address         =  LCD_LAYER_FRAME_BUFFER;
  lcd_layer.buffer_index    =  0;
  lcd_layer.pending_buffer  =  -1;
  lcd_layer.drawing_buffer  =  LCD_LAYER_FRAME_BUFFER;
  lcd_layer.bytes_per_pixel =  2;  // RGB565

  lcd_layer.width  = XSIZE_PHYS;
  lcd_layer.height = YSIZE_PHYS;

  lcd_layer.length = lcd_layer.width * lcd_layer.height * lcd_layer.bytes_per_pixel;

  lcd_layer.p_buffer[0] = (uint16_t *)lcd_layer.address;
  lcd_layer.p_buffer[1] = (uint16_t *)(lcd_layer.address + lcd_layer.length);


  drvLcdHwInit();
  drvLcdLayerInit(0, LCD_LAYER_FRAME_BUFFER);

  HAL_DSI_LongWrite(&lcd_hw.hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 4, OTM8009A_CMD_CASET, pCols[0]);
  HAL_DSI_LongWrite(&lcd_hw.hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 4, OTM8009A_CMD_PASET, pPage);

  HAL_LTDC_SetPitch(&lcd_hw.hltdc, XSIZE_PHYS, 0);
  HAL_Delay(20);


  drvLcdReqTear();

  /* Send Display off DCS Command to display */
  HAL_DSI_ShortWrite(&(lcd_hw.hdsi),
                     0,
                     DSI_DCS_SHORT_PKT_WRITE_P1,
                     OTM8009A_CMD_DISPON,
                     0x00);
  return ret;
}

void drvLcdDrawPixel(uint16_t x_pos, uint16_t y_pos, uint32_t rgb_code)
{
  lcd_layer.p_buffer[lcd_layer.buffer_index][y_pos * XSIZE_PHYS + x_pos] = rgb_code;
}

void drvLcdClear(uint32_t rgb_code)
{
  drvLcdFillBuffer((uint32_t *)lcd_layer.p_buffer[lcd_layer.buffer_index], XSIZE_PHYS, YSIZE_PHYS, 0, rgb_code);
}

bool drvLcdDrawAvailable(void)
{
  return !lcd_request_draw;
}

void drvLcdRequestDraw(void)
{
  lcd_request_draw = true;
}

uint32_t drvLcdGetWidth(void)
{
  return XSIZE_PHYS;
}

uint32_t drvLcdGetHeight(void)
{
  return YSIZE_PHYS;
}


void drvLcdCopyLineBuffer(uint16_t x_pos, uint16_t y_pos, uint8_t *p_data, uint32_t length)
{
  uint32_t PixelFormat;
  uint32_t dst_addr;

  dst_addr = (uint32_t)lcd_layer.p_buffer[lcd_layer.buffer_index] + 2*y_pos*drvLcdGetWidth() + x_pos;


  PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  DMA2D->CR      = 0x00000000UL | (1 << 9) | (0x2 << 16);

  /* Set up pointers */
  DMA2D->FGMAR   = (uint32_t)p_data;
  DMA2D->OMAR    = (uint32_t)dst_addr;
  DMA2D->BGMAR   = 0;
  DMA2D->FGOR    = 0;
  DMA2D->OOR     = 0;
  DMA2D->BGOR    = 0;

  /* Set up pixel format */
  DMA2D->FGPFCCR = LTDC_PIXEL_FORMAT_RGB565;
  DMA2D->BGPFCCR = PixelFormat;
  DMA2D->OPFCCR  = PixelFormat;

  /*  Set up size */
  DMA2D->NLR     = (uint32_t)(length << 16) | (uint32_t)1;


  DMA2D->CR     |= DMA2D_CR_START;

  /* Wait until transfer is done */
  while (DMA2D->CR & DMA2D_CR_START)
  {
  }
}

void drvLcdHwInit(void)
{
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
  GPIO_InitTypeDef          GPIO_Init_Structure;
  static DSI_PHY_TimerTypeDef PhyTimings;
  static DSI_CmdCfgTypeDef         CmdCfg;
  static DSI_LPCmdTypeDef          LPCmd;
  static DSI_PLLInitTypeDef        PLLInit;


  /* Toggle Hardware Reset of the DSI LCD using
   * its XRES signal (active low) */
  drvLcdHwReset();

  /* Call first MSP Initialize only in case of first initialization
   * This will set IP blocks LTDC, DSI and DMA2D
   * - out of reset
   * - clocked
   * - NVIC IRQ related to IP blocks enabled
   */
  drvLcdMspInit();

  /* LCD clock configuration */
  /* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
  /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 384 Mhz */
  /* PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 384 MHz / 7 = 54.857 MHz */
  /* LTDC clock frequency = PLLLCDCLK / LTDC_PLLSAI_DIVR_2 = 54.857 MHz / 2 = 27.429 MHz */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 7;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  /* Base address of DSI Host/Wrapper registers to be set before calling De-Init */
  lcd_hw.hdsi.Instance = DSI;

  HAL_DSI_DeInit(&(lcd_hw.hdsi));

#if defined(USE_STM32469I_DISCO_REVA)
  PLLInit.PLLNDIV  = 100;
  PLLInit.PLLIDF   = DSI_PLL_IN_DIV5;
#else
  PLLInit.PLLNDIV  = 125;
  PLLInit.PLLIDF   = DSI_PLL_IN_DIV2;
#endif /* USE_STM32469I_DISCO_REVA */
  PLLInit.PLLODF  = DSI_PLL_OUT_DIV1;

  lcd_hw.hdsi.Init.NumberOfLanes = DSI_TWO_DATA_LANES;
  lcd_hw.hdsi.Init.TXEscapeCkdiv = 0x4;
  HAL_DSI_Init(&(lcd_hw.hdsi), &(PLLInit));

  /* Configure the DSI for Command mode */
  CmdCfg.VirtualChannelID      = 0;
  CmdCfg.HSPolarity            = DSI_HSYNC_ACTIVE_HIGH;
  CmdCfg.VSPolarity            = DSI_VSYNC_ACTIVE_HIGH;
  CmdCfg.DEPolarity            = DSI_DATA_ENABLE_ACTIVE_HIGH;
  CmdCfg.ColorCoding           = DSI_RGB565;
  CmdCfg.CommandSize           = HACT;
  CmdCfg.TearingEffectSource   = DSI_TE_EXTERNAL;
  CmdCfg.TearingEffectPolarity = DSI_TE_RISING_EDGE;
  CmdCfg.VSyncPol              = DSI_VSYNC_FALLING;
  CmdCfg.AutomaticRefresh      = DSI_AR_DISABLE;
  CmdCfg.TEAcknowledgeRequest  = DSI_TE_ACKNOWLEDGE_ENABLE;
  HAL_DSI_ConfigAdaptedCommandMode(&lcd_hw.hdsi, &CmdCfg);

  LPCmd.LPGenShortWriteNoP    = DSI_LP_GSW0P_ENABLE;
  LPCmd.LPGenShortWriteOneP   = DSI_LP_GSW1P_ENABLE;
  LPCmd.LPGenShortWriteTwoP   = DSI_LP_GSW2P_ENABLE;
  LPCmd.LPGenShortReadNoP     = DSI_LP_GSR0P_ENABLE;
  LPCmd.LPGenShortReadOneP    = DSI_LP_GSR1P_ENABLE;
  LPCmd.LPGenShortReadTwoP    = DSI_LP_GSR2P_ENABLE;
  LPCmd.LPGenLongWrite        = DSI_LP_GLW_ENABLE;
  LPCmd.LPDcsShortWriteNoP    = DSI_LP_DSW0P_ENABLE;
  LPCmd.LPDcsShortWriteOneP   = DSI_LP_DSW1P_ENABLE;
  LPCmd.LPDcsShortReadNoP     = DSI_LP_DSR0P_ENABLE;
  LPCmd.LPDcsLongWrite        = DSI_LP_DLW_ENABLE;
  HAL_DSI_ConfigCommand(&lcd_hw.hdsi, &LPCmd);

  /* Configure DSI PHY HS2LP and LP2HS timings */
  PhyTimings.ClockLaneHS2LPTime = 35;
  PhyTimings.ClockLaneLP2HSTime = 35;
  PhyTimings.DataLaneHS2LPTime = 35;
  PhyTimings.DataLaneLP2HSTime = 35;
  PhyTimings.DataLaneMaxReadTime = 0;
  PhyTimings.StopWaitTime = 10;
  HAL_DSI_ConfigPhyTimer(&lcd_hw.hdsi, &PhyTimings);

  /* Initialize LTDC */
  drvLtdcInit();

  /* Start DSI */
  HAL_DSI_Start(&(lcd_hw.hdsi));

  /* Initialize the OTM8009A LCD Display IC Driver (KoD LCD IC Driver)
   *  depending on configuration set in 'hdsivideo_handle'.
   */

  /* Send Display off DCS Command to display */
  HAL_DSI_ShortWrite(&(lcd_hw.hdsi),
                     0,
                     DSI_DCS_SHORT_PKT_WRITE_P1,
                     OTM8009A_CMD_DISPOFF,
                     0x00);

  OTM8009A_Init(OTM8009A_FORMAT_RBG565, OTM8009A_ORIENTATION_LANDSCAPE);

  LPCmd.LPGenShortWriteNoP    = DSI_LP_GSW0P_DISABLE;
  LPCmd.LPGenShortWriteOneP   = DSI_LP_GSW1P_DISABLE;
  LPCmd.LPGenShortWriteTwoP   = DSI_LP_GSW2P_DISABLE;
  LPCmd.LPGenShortReadNoP     = DSI_LP_GSR0P_DISABLE;
  LPCmd.LPGenShortReadOneP    = DSI_LP_GSR1P_DISABLE;
  LPCmd.LPGenShortReadTwoP    = DSI_LP_GSR2P_DISABLE;
  LPCmd.LPGenLongWrite        = DSI_LP_GLW_DISABLE;
  LPCmd.LPDcsShortWriteNoP    = DSI_LP_DSW0P_DISABLE;
  LPCmd.LPDcsShortWriteOneP   = DSI_LP_DSW1P_DISABLE;
  LPCmd.LPDcsShortReadNoP     = DSI_LP_DSR0P_DISABLE;
  LPCmd.LPDcsLongWrite        = DSI_LP_DLW_DISABLE;
  HAL_DSI_ConfigCommand(&lcd_hw.hdsi, &LPCmd);

  HAL_DSI_ConfigFlowControl(&lcd_hw.hdsi, DSI_FLOW_CONTROL_BTA);

  /* Enable GPIOJ clock */
  __HAL_RCC_GPIOJ_CLK_ENABLE();

  /* Configure DSI_TE pin from MB1166 : Tearing effect on separated GPIO from KoD LCD */
  /* that is mapped on GPIOJ2 as alternate DSI function (DSI_TE)                      */
  /* This pin is used only when the LCD and DSI link is configured in command mode    */
  /* Not used in DSI Video mode.                                                      */
  GPIO_Init_Structure.Pin       = GPIO_PIN_2;
  GPIO_Init_Structure.Mode      = GPIO_MODE_AF_PP;
  GPIO_Init_Structure.Pull      = GPIO_NOPULL;
  GPIO_Init_Structure.Speed     = GPIO_SPEED_HIGH;
  GPIO_Init_Structure.Alternate = GPIO_AF13_DSI;
  HAL_GPIO_Init(GPIOJ, &GPIO_Init_Structure);

  drvLcdFillBuffer((uint32_t *)LCD_LAYER_FRAME_BUFFER, XSIZE_PHYS, YSIZE_PHYS, 0, 0x0000);

  /* Refresh the display */
  HAL_DSI_Refresh(&lcd_hw.hdsi);
}

void drvLcdHwReset(void)
{
#if !defined(USE_STM32469I_DISCO_REVA)
  GPIO_InitTypeDef  gpio_init_structure;

  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* Configure the GPIO on PH7 */
  gpio_init_structure.Pin   = GPIO_PIN_7;
  gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Pull  = GPIO_PULLUP;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOH, &gpio_init_structure);

  /* Activate XRES active low */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_Delay(20); /* wait 20 ms */

  /* Desactivate XRES */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(10); /* wait 10 ms */
#endif  /* (USE_STM32469I_DISCO_REVA) */
}

void drvLcdMspInit(void)
{
  /** @brief Enable the LTDC clock */
  __HAL_RCC_LTDC_CLK_ENABLE();

  /** @brief Toggle Sw reset of LTDC IP */
  __HAL_RCC_LTDC_FORCE_RESET();
  __HAL_RCC_LTDC_RELEASE_RESET();

  /** @brief Enable the DMA2D clock */
  __HAL_RCC_DMA2D_CLK_ENABLE();

  /** @brief Toggle Sw reset of DMA2D IP */
  __HAL_RCC_DMA2D_FORCE_RESET();
  __HAL_RCC_DMA2D_RELEASE_RESET();

  /** @brief Enable DSI Host and wrapper clocks */
  __HAL_RCC_DSI_CLK_ENABLE();

  /** @brief Soft Reset the DSI Host and wrapper */
  __HAL_RCC_DSI_FORCE_RESET();
  __HAL_RCC_DSI_RELEASE_RESET();

  /** @brief NVIC configuration for DSI interrupt that is now enabled */
  HAL_NVIC_SetPriority(DSI_IRQn, 0xF, 0);
  HAL_NVIC_EnableIRQ(DSI_IRQn);
}

void drvLtdcInit(void)
{
  /* DeInit */
  HAL_LTDC_DeInit(&lcd_hw.hltdc);

  /* LTDC Config */
  /* Timing and polarity */
  lcd_hw.hltdc.Init.HorizontalSync = HSYNC;
  lcd_hw.hltdc.Init.VerticalSync = VSYNC;
  lcd_hw.hltdc.Init.AccumulatedHBP = HSYNC+HBP;
  lcd_hw.hltdc.Init.AccumulatedVBP = VSYNC+VBP;
  lcd_hw.hltdc.Init.AccumulatedActiveH = VSYNC+VBP+VACT;
  lcd_hw.hltdc.Init.AccumulatedActiveW = HSYNC+HBP+HACT;
  lcd_hw.hltdc.Init.TotalHeigh = VSYNC+VBP+VACT+VFP;
  lcd_hw.hltdc.Init.TotalWidth = HSYNC+HBP+HACT+HFP;

  /* background value */
  lcd_hw.hltdc.Init.Backcolor.Blue = 0;
  lcd_hw.hltdc.Init.Backcolor.Green = 0;
  lcd_hw.hltdc.Init.Backcolor.Red = 0;

  /* Polarity */
  lcd_hw.hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  lcd_hw.hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  lcd_hw.hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  lcd_hw.hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  lcd_hw.hltdc.Instance = LTDC;

  HAL_LTDC_Init(&lcd_hw.hltdc);
}

void drvLcdLayerInit(uint32_t LayerIndex, uint32_t address)
{
  LTDC_LayerCfgTypeDef  Layercfg;

  /* Layer Init */
  Layercfg.WindowX0 = 0;
  Layercfg.WindowX1 = HACT;
  Layercfg.WindowY0 = 0;
  Layercfg.WindowY1 = YSIZE_PHYS;
  Layercfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  Layercfg.FBStartAdress = address;
  Layercfg.Alpha = 255;
  Layercfg.Alpha0 = 0;
  Layercfg.Backcolor.Blue = 0;
  Layercfg.Backcolor.Green = 0;
  Layercfg.Backcolor.Red = 0;
  Layercfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  Layercfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  Layercfg.ImageWidth = HACT;
  Layercfg.ImageHeight = YSIZE_PHYS;

  HAL_LTDC_ConfigLayer(&lcd_hw.hltdc, &Layercfg, LayerIndex);
}

void drvLcdReqTear(void)
{
  static uint8_t ScanLineParams[2];
#if (ZONES == 4 )
  uint16_t scanline = 283;
#elif (ZONES == 2 )
  uint16_t scanline = 200;
#endif
  ScanLineParams[0] = scanline >> 8;
  ScanLineParams[1] = scanline & 0x00FF;

  HAL_DSI_LongWrite(&lcd_hw.hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 2, 0x44, ScanLineParams);
  /* set_tear_on */
  HAL_DSI_ShortWrite(&lcd_hw.hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, OTM8009A_CMD_TEEON, 0x00);
}



void drvLcdFillBuffer(void * pDst, uint32_t xSize, uint32_t ySize, uint32_t OffLine, uint32_t ColorIndex)
{
  /* Set up mode */
  DMA2D->CR      = 0x00030000UL | (1 << 9);
  DMA2D->OCOLR   = ColorIndex;

  /* Set up pointers */
  DMA2D->OMAR    = (uint32_t)pDst;

  /* Set up offsets */
  DMA2D->OOR     = OffLine;

  /* Set up pixel format */
  DMA2D->OPFCCR  = LTDC_PIXEL_FORMAT_RGB565;

  /*  Set up size */
  DMA2D->NLR     = (uint32_t)(xSize << 16) | (uint32_t)ySize;

  DMA2D->CR     |= DMA2D_CR_START;

  /* Wait until transfer is done */
  while (DMA2D->CR & DMA2D_CR_START)
  {
  }
}

void DSI_IO_WriteCmd(uint32_t NbrParams, uint8_t *pParams)
{
  if(NbrParams <= 1)
  {
    HAL_DSI_ShortWrite(&lcd_hw.hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, pParams[0], pParams[1]);
  }
  else
  {
    HAL_DSI_LongWrite(&lcd_hw.hdsi,  0, DSI_DCS_LONG_PKT_WRITE, NbrParams, pParams[NbrParams], pParams);
  }
}

void drvLcdSetUpdateRegion(int idx)
{
  HAL_DSI_LongWrite(&lcd_hw.hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 4, OTM8009A_CMD_CASET, pCols[idx]);
}

/**
  * @brief  Tearing Effect DSI callback.
  * @param  hdsi: pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval None
  */
void HAL_DSI_TearingEffectCallback(DSI_HandleTypeDef *hdsi)
{
  if(!lcd_refershing)
  {
    lcd_refershing = 1;
    lcd_active_region = 1;


    if (lcd_request_draw == true)
    {

      lcd_layer.drawing_buffer = (int32_t)lcd_layer.p_buffer[lcd_layer.buffer_index];
      lcd_layer.buffer_index ^= 1;

      lcd_request_draw = false;
    }


    __HAL_DSI_WRAPPER_DISABLE(hdsi);
      LTDC_LAYER(&lcd_hw.hltdc, 0)->CFBAR = lcd_layer.drawing_buffer + cols_addr[0];
    __HAL_LTDC_RELOAD_IMMEDIATE_CONFIG(&lcd_hw.hltdc);
    __HAL_DSI_WRAPPER_ENABLE(hdsi);
    drvLcdSetUpdateRegion(0);

    HAL_DSI_Refresh(hdsi);
  }

  return;
}

/**
  * @brief  End of Refresh DSI callback.
  * @param  hdsi: pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval None
  */
void HAL_DSI_EndOfRefreshCallback(DSI_HandleTypeDef *hdsi)
{

  if(lcd_active_region < ZONES )
  {
    lcd_refershing = 1;
    /* Disable DSI Wrapper */
    __HAL_DSI_WRAPPER_DISABLE(hdsi);
    LTDC_LAYER(&lcd_hw.hltdc, 0)->CFBAR  = lcd_layer.drawing_buffer + cols_addr[lcd_active_region];
    __HAL_LTDC_RELOAD_IMMEDIATE_CONFIG(&lcd_hw.hltdc);
    __HAL_DSI_WRAPPER_ENABLE(hdsi);
    drvLcdSetUpdateRegion(lcd_active_region++);
    /* Refresh the right part of the display */
    HAL_DSI_Refresh(hdsi);
  }
  else
  {
    lcd_refershing = 0;
  }

  return;
}


/**
  * @brief  This function handles LTDC interrupt request.
  * @param  None
  * @retval None
  */
void LTDC_IRQHandler(void)
{
  HAL_LTDC_IRQHandler(&lcd_hw.hltdc);
}

void LTDC_ER_IRQHandler(void)
{
  /* Check the interrupt and clear flag */
  HAL_LTDC_IRQHandler(&lcd_hw.hltdc);

}
/**
  * @brief  This function handles DSI Handler.
  * @param  None
  * @retval None
  */
void DSI_IRQHandler(void)
{
  HAL_DSI_IRQHandler(&lcd_hw.hdsi);
}
