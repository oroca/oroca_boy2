
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


#define LCD_DSI_PIXEL_DATA_FMT_RBG888  DSI_RGB888 /*!< DSI packet pixel format chosen is RGB888 : 24 bpp */
#define LCD_DSI_PIXEL_DATA_FMT_RBG565  DSI_RGB565 /*!< DSI packet pixel format chosen is RGB565 : 16 bpp */

#define LTDC_DEFAULT_ACTIVE_LAYER         _DEF_LCD_LAYER1

#define LCD_OTM8009A_ID        ((uint32_t) 0)

#define VSA   OTM8009A_480X800_VSYNC /*!< Vertical start active time in units of lines */
#define VBP   OTM8009A_480X800_VBP /*!< Vertical Back Porch time in units of lines */
#define VFP   OTM8009A_480X800_VFP /*!< Vertical Front Porch time in units of lines */
#define VACT  OTM8009A_800X480_HEIGHT /*!< Vertical Active time in units of lines = imageSize Y in pixels to display */
#define HSA   OTM8009A_480X800_HSYNC /*!< Horizontal start active time in units of lcdClk */
#define HBP   OTM8009A_480X800_HBP /*!< Horizontal Back Porch time in units of lcdClk */
#define HFP   OTM8009A_480X800_HFP /*!< Horizontal Front Porch time in units of lcdClk */
#define HACT  OTM8009A_800X480_WIDTH /*!< Horizontal Active time in units of lcdClk = imageSize X in pixels to display */

uint32_t lcd_x_size = HACT;
uint32_t lcd_y_size = VACT;
static uint32_t  active_layer_idx = _DEF_LCD_LAYER1;

DSI_HandleTypeDef hdsi;
LTDC_HandleTypeDef  hltdc;
DMA2D_HandleTypeDef hdma2d;
static DSI_VidCfgTypeDef hdis_video_conf;

static err_code_t fillBuffer(uint32_t layer_idx, void *p_dst, uint32_t x_size, uint32_t y_size, uint32_t line_offset, uint32_t color_idx);

static void drvLcdMspDeinit(void);
static void drvLcdMspInit(void);

/**
  * @brief  Initializes the DSI LCD.
  * The ititialization is done as below:
  *     - DSI PLL ititialization
  *     - DSI ititialization
  *     - LTDC ititialization
  *     - OTM8009A LCD Display IC Driver ititialization
  * @retval LCD state
  */
err_code_t drvLcdInit(uint8_t orientation)
{
  DSI_PLLInitTypeDef dsiPllInit;
  DSI_PHY_TimerTypeDef  PhyTimings;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
  uint32_t LcdClock  = 27429; /*!< LcdClk = 27429 kHz */
  uint32_t otm8009a_lcd_orient;

  uint32_t laneByteClk_kHz = 0;

  /* Toggle Hardware Reset of the DSI LCD using
  * its XRES signal (active low) */
  drvLcdReset();

  /* Call first MSP Initialize only in case of first initialization
  * This will set IP blocks LTDC, DSI and DMA2D
  * - out of reset
  * - clocked
  * - NVIC IRQ related to IP blocks enabled
  */
  drvLcdMspInit();

/*************************DSI Initialization***********************************/

  /* Base address of DSI Host/Wrapper registers to be set before calling De-Init */
  hdsi.Instance = DSI;

  HAL_DSI_DeInit(&(hdsi));

  dsiPllInit.PLLNDIV  = 125;
  dsiPllInit.PLLIDF   = DSI_PLL_IN_DIV2;
  dsiPllInit.PLLODF   = DSI_PLL_OUT_DIV1;

  laneByteClk_kHz = 62500; /* 500 MHz / 8 = 62.5 MHz = 62500 kHz */
  /* Set number of Lanes */
  hdsi.Init.NumberOfLanes = DSI_TWO_DATA_LANES;
  /* TXEscapeCkdiv = f(LaneByteClk)/15.62 = 4 */
  hdsi.Init.TXEscapeCkdiv = laneByteClk_kHz/15620;
  HAL_DSI_Init(&(hdsi), &(dsiPllInit));

  /* Timing parameters for all Video modes
  * Set Timing parameters of LTDC depending on its chosen orientation
  */
  if(orientation == _DEF_PORTRAIT)
  {
    lcd_x_size = OTM8009A_480X800_WIDTH;  /* 480 */
    lcd_y_size = OTM8009A_480X800_HEIGHT; /* 800 */
  }
  else //_DEF_LADSCAPE
  {
    lcd_x_size = OTM8009A_800X480_WIDTH;  /* 800 */
    lcd_y_size = OTM8009A_800X480_HEIGHT; /* 480 */
  }

  hdis_video_conf.VirtualChannelID = LCD_OTM8009A_ID;
  hdis_video_conf.ColorCoding = LCD_DSI_PIXEL_DATA_FMT_RBG565;
  hdis_video_conf.VSPolarity = DSI_VSYNC_ACTIVE_HIGH;
  hdis_video_conf.HSPolarity = DSI_HSYNC_ACTIVE_HIGH;
  hdis_video_conf.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;
  hdis_video_conf.Mode = DSI_VID_MODE_BURST; /* align Video burst ie : one LgP per line */
  hdis_video_conf.NullPacketSize = 0xFFF;
  hdis_video_conf.NumberOfChunks = 0;
  hdis_video_conf.PacketSize                = HACT; /* Value depending on display orientation choice portrait/landscape */
  hdis_video_conf.HorizontalSyncActive      = (HSA * laneByteClk_kHz) / LcdClock;
  hdis_video_conf.HorizontalBackPorch       = (HBP * laneByteClk_kHz) / LcdClock;
  hdis_video_conf.HorizontalLine            = ((HACT + HSA + HBP + HFP) * laneByteClk_kHz) / LcdClock; /* Value depending on display orientation choice portrait/landscape */
  hdis_video_conf.VerticalSyncActive        = VSA;
  hdis_video_conf.VerticalBackPorch         = VBP;
  hdis_video_conf.VerticalFrontPorch        = VFP;
  hdis_video_conf.VerticalActive            = VACT; /* Value depending on display orientation choice portrait/landscape */

  /* Enable or disable sending LP command while streaming is active in video mode */
  hdis_video_conf.LPCommandEnable = DSI_LP_COMMAND_ENABLE; /* Enable sending commands in mode LP (Low Power) */

  /* Largest packet size possible to transmit in LP mode in VSA, VBP, VFP regions */
  /* Only useful when sending LP packets is allowed while streaming is active in video mode */
  hdis_video_conf.LPLargestPacketSize = 16;

  /* Largest packet size possible to transmit in LP mode in HFP region during VACT period */
  /* Only useful when sending LP packets is allowed while streaming is active in video mode */
  hdis_video_conf.LPVACTLargestPacketSize = 0;


  /* Specify for each region of the video frame, if the transmission of command in LP mode is allowed in this region */
  /* while streaming is active in video mode                                                                         */
  hdis_video_conf.LPHorizontalFrontPorchEnable = DSI_LP_HFP_ENABLE;   /* Allow sending LP commands during HFP period */
  hdis_video_conf.LPHorizontalBackPorchEnable  = DSI_LP_HBP_ENABLE;   /* Allow sending LP commands during HBP period */
  hdis_video_conf.LPVerticalActiveEnable = DSI_LP_VACT_ENABLE;  /* Allow sending LP commands during VACT period */
  hdis_video_conf.LPVerticalFrontPorchEnable = DSI_LP_VFP_ENABLE;   /* Allow sending LP commands during VFP period */
  hdis_video_conf.LPVerticalBackPorchEnable = DSI_LP_VBP_ENABLE;   /* Allow sending LP commands during VBP period */
  hdis_video_conf.LPVerticalSyncActiveEnable = DSI_LP_VSYNC_ENABLE; /* Allow sending LP commands during VSync = VSA period */

  /* Configure DSI Video mode timings with settings set above */
  HAL_DSI_ConfigVideoMode(&(hdsi), &(hdis_video_conf));

  /* Configure DSI PHY HS2LP and LP2HS timings */
  PhyTimings.ClockLaneHS2LPTime = 35;
  PhyTimings.ClockLaneLP2HSTime = 35;
  PhyTimings.DataLaneHS2LPTime = 35;
  PhyTimings.DataLaneLP2HSTime = 35;
  PhyTimings.DataLaneMaxReadTime = 0;
  PhyTimings.StopWaitTime = 10;
  HAL_DSI_ConfigPhyTimer(&hdsi, &PhyTimings);

/*************************End DSI Initialization*******************************/


/************************LTDC Initialization***********************************/

  /* Timing Configuration */
  hltdc.Init.HorizontalSync = (HSA - 1);
  hltdc.Init.AccumulatedHBP = (HSA + HBP - 1);
  hltdc.Init.AccumulatedActiveW = (lcd_x_size + HSA + HBP - 1);
  hltdc.Init.TotalWidth = (lcd_x_size + HSA + HBP + HFP - 1);

  /* Initialize the LCD pixel width and pixel height */
  hltdc.LayerCfg->ImageWidth  = lcd_x_size;
  hltdc.LayerCfg->ImageHeight = lcd_y_size;


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

  /* Background value */
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Instance = LTDC;

  /* Get LTDC Configuration from DSI Configuration */
  HAL_LTDCEx_StructInitFromVideoConfig(&(hltdc), &(hdis_video_conf));

  /* Initialize the LTDC */
  HAL_LTDC_Init(&hltdc);

  /* Enable the DSI host and wrapper after the LTDC initialization
     To avoid any synchronization issue, the DSI shall be started after enabling the LTDC */
  HAL_DSI_Start(&(hdsi));

#if !defined(DATA_IN_ExtSDRAM)
  /* Initialize the SDRAM */
//  drvSdramInit();
#endif /* DATA_IN_ExtSDRAM */


/************************End LTDC Initialization*******************************/

  switch(orientation)
  {
    case _DEF_PORTRAIT :
      otm8009a_lcd_orient = OTM8009A_ORIENTATION_PORTRAIT;
      break;
    case _DEF_LADSCAPE :
    default :
      otm8009a_lcd_orient = OTM8009A_ORIENTATION_LANDSCAPE;
      break;
  }

/***********************OTM8009A Initialization********************************/

  /* Initialize the OTM8009A LCD Display IC Driver (KoD LCD IC Driver)
  *  depending on configuration set in 'hdis_video_conf'.
  */
  OTM8009A_Init(OTM8009A_FORMAT_RBG565, otm8009a_lcd_orient);

/***********************End OTM8009A Initialization****************************/

  return OK;
}

/**
  * @brief  Draws a pixel on LCD.
  * @param  x_pos: X position
  * @param  y_pos: Y position
  * @param  rgb_code: Pixel color in ARGB mode (8-8-8-8)
  */
void drvLcdDrawPixel(uint16_t x_pos, uint16_t y_pos, uint32_t rgb_code)
{
  /* Write data value to all SDRAM memory */
  *(__IO uint32_t*) (hltdc.LayerCfg[active_layer_idx].FBStartAdress + (2*(y_pos*drvLcdGetXSize() + x_pos))) = rgb_code;
}

/**
  * @brief  Reads an LCD pixel.
  * @param  x_pos: X position
  * @param  y_pos: Y position
  * @retval RGB pixel color
  */
uint32_t drvLcdReadPixel(uint16_t x_pos, uint16_t y_pos)
{
  uint32_t ret = 0;

  if(hltdc.LayerCfg[active_layer_idx].PixelFormat == LTDC_PIXEL_FORMAT_ARGB8888)
  {
    /* Read data value from SDRAM memory */
    ret = *(__IO uint32_t*) (hltdc.LayerCfg[active_layer_idx].FBStartAdress + (4*(y_pos*drvLcdGetXSize() + x_pos)));
  }
  else if(hltdc.LayerCfg[active_layer_idx].PixelFormat == LTDC_PIXEL_FORMAT_RGB565)
  {
    /* Read data value from SDRAM memory */
    ret = *(__IO uint16_t*) (hltdc.LayerCfg[active_layer_idx].FBStartAdress + (2*(y_pos*drvLcdGetXSize() + x_pos)));
  }
  else
  {
    /* Read data value from SDRAM memory */
    ret = *(__IO uint8_t*) (hltdc.LayerCfg[active_layer_idx].FBStartAdress + (2*(y_pos*drvLcdGetXSize() + x_pos)));
  }

  return ret;
}

/**
  * @brief  Clears the whole currently active layer of LTDC.
  * @param  color: color of the background
  */
void drvLcdClear(uint32_t rgb_code)
{
  /* Clear the LCD */
  fillBuffer(active_layer_idx, (uint32_t *)(hltdc.LayerCfg[active_layer_idx].FBStartAdress), drvLcdGetXSize(), drvLcdGetYSize(), 0, rgb_code);
}

/**
  * @brief  BSP LCD Reset
  *         Hw reset the LCD DSI activating its XRES signal (active low for some time)
  *         and desactivating it later.
  *         This signal is only cabled on Discovery Rev B and beyond.
  */
void drvLcdReset(void)
{
/* EVAL Rev B and beyond : reset the LCD by activation of XRES (active low) connected to PH7 */
  GPIO_InitTypeDef  GPIO_InitStruct;

  __HAL_RCC_GPIOH_CLK_ENABLE();

    /* Configure the GPIO on PH7 */
  GPIO_InitStruct.Pin   = GPIO_PIN_7;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* Activate XRES active low */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, GPIO_PIN_RESET);

  HAL_Delay(20); /* wait 20 ms */

  /* Desactivate XRES */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, GPIO_PIN_SET);

  /* Wait for 10ms after releasing XRES before sending commands */
  HAL_Delay(10);
}

/**
  * @brief  Selects the LCD Layer.
  * @param  layer_idx: Layer foreground or background
  */
err_code_t drvLcdSelectLayer(uint32_t layer_idx)
{
  if(layer_idx >= MAX_LAYER)
  {
    return ERR_LCD_INVAILD_LAYER;
  }
  active_layer_idx = layer_idx;

  return OK;
}

/**
  * @brief  Sets an LCD Layer visible
  * @param  layer_idx: Visible Layer
  * @param  state: New state of the specified layer
  *          This parameter can be one of the following values:
  *            @arg  ENABLE
  *            @arg  DISABLE
  */
err_code_t drvLcdSetLayerVisible(uint32_t layer_idx, uint8_t state)
{
  if(layer_idx >= MAX_LAYER)
  {
    return ERR_LCD_INVAILD_LAYER;
  }

  if(state == _DEF_ENABLE)
  {
    __HAL_LTDC_LAYER_ENABLE(&(hltdc), layer_idx);
  }
  else
  {
    __HAL_LTDC_LAYER_DISABLE(&(hltdc), layer_idx);
  }
  __HAL_LTDC_RELOAD_IMMEDIATE_CONFIG(&(hltdc));

  return OK;
}

/**
  * @brief  Configures the transparency.
  * @param  layer_idx: Layer foreground or background.
  * @param  transparency: transparency
  *           This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF
  */
err_code_t drvLcdSetTransparency(uint32_t layer_idx, uint8_t transparency)
{
  if(layer_idx >= MAX_LAYER)
  {
    return ERR_LCD_INVAILD_LAYER;
  }

  HAL_LTDC_SetAlpha(&(hltdc), transparency, layer_idx);

  return OK;
}

/**
  * @brief  Sets an LCD layer frame buffer address.
  * @param  layer_idx: Layer foreground or background
  * @param  addr: New LCD frame buffer value
  */
err_code_t drvLcdSetLayerAddr(uint32_t layer_idx, uint32_t addr)
{
  if(layer_idx >= MAX_LAYER)
  {
    return ERR_LCD_INVAILD_LAYER;
  }

  HAL_LTDC_SetAddress(&(hltdc), addr, layer_idx);

  return OK;
}

/**
  * @brief  Sets display window.
  * @param  layer_idx: Layer index
  * @param  x_pos: LCD X position
  * @param  y_pos: LCD Y position
  * @param  width: LCD window width
  * @param  height: LCD window height
  */
err_code_t drvLcdSetLayerWindow(uint16_t layer_idx, uint16_t x_pos, uint16_t y_pos, uint16_t width, uint16_t height)
{
  if(layer_idx >= MAX_LAYER)
  {
    return ERR_LCD_INVAILD_LAYER;
  }

  /* Reconfigure the layer size */
  HAL_LTDC_SetWindowSize(&(hltdc), width, height, layer_idx);

  /* Reconfigure the layer position */
  HAL_LTDC_SetWindowPosition(&(hltdc), x_pos, y_pos, layer_idx);

  return OK;
}



/**
  * @brief  Switch back on the display if was switched off by previous call of drvLcdDisplayOff().
  *         Exit DSI ULPM mode if was allowed and configured in Dsi Configuration.
  */
void drvLcdDisplayOn(void)
{
  /* Send Display on DCS command to display */
  HAL_DSI_ShortWrite(&(hdsi),
                     hdis_video_conf.VirtualChannelID,
                     DSI_DCS_SHORT_PKT_WRITE_P1,
                     OTM8009A_CMD_DISPON,
                     0x00);

}

/**
  * @brief  Switch Off the display.
  *         Enter DSI ULPM mode if was allowed and configured in Dsi Configuration.
  */
void drvLcdDisplayOff(void)
{
  /* Send Display off DCS Command to display */
  HAL_DSI_ShortWrite(&(hdsi),
                     hdis_video_conf.VirtualChannelID,
                     DSI_DCS_SHORT_PKT_WRITE_P1,
                     OTM8009A_CMD_DISPOFF,
                     0x00);

}


/**
  * @brief  Gets the LCD X size.
  * @retval Used LCD X size
  */
uint32_t drvLcdGetXSize(void)
{
  return (lcd_x_size);
}

/**
  * @brief  Gets the LCD Y size.
  * @retval Used LCD Y size
  */
uint32_t drvLcdGetYSize(void)
{
  return (lcd_y_size);
}

/**
  * @brief  Set the LCD X size.
  * @param  image_width_pixels : uint32_t image width in pixels unit
  */
void drvLcdSetXSize(uint32_t image_width_pixels)
{
  hltdc.LayerCfg[active_layer_idx].ImageWidth = image_width_pixels;
}

/**
  * @brief  Set the LCD Y size.
  * @param  image_height_pixels : uint32_t image height in lines unit
  */
void drvLcdSetYSize(uint32_t image_height_pixels)
{
  hltdc.LayerCfg[active_layer_idx].ImageHeight = image_height_pixels;
}

/**
  * @brief  Initializes the LCD layers.
  * @param  layer_idx: Layer foreground or background
  * @param  fb_addr: Layer frame buffer
  */
err_code_t drvLcdInitLayer(uint16_t layer_idx, uint32_t fb_addr)
{
  LTDC_LayerCfgTypeDef Layercfg;

  if(layer_idx >= MAX_LAYER)
  {
    return ERR_LCD_INVAILD_LAYER;
  }

  /* Layer Init */
  Layercfg.WindowX0 = 0;
  Layercfg.WindowX1 = drvLcdGetXSize();
  Layercfg.WindowY0 = 0;
  Layercfg.WindowY1 = drvLcdGetYSize();
  Layercfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  Layercfg.FBStartAdress = fb_addr;
  Layercfg.Alpha = 255;
  Layercfg.Alpha0 = 0;
  Layercfg.Backcolor.Blue = 0;
  Layercfg.Backcolor.Green = 0;
  Layercfg.Backcolor.Red = 0;
  Layercfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  Layercfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  Layercfg.ImageWidth = drvLcdGetXSize();
  Layercfg.ImageHeight = drvLcdGetYSize();

  HAL_LTDC_ConfigLayer(&hltdc, &Layercfg, layer_idx);

  return OK;
}


/**
  * @brief  Initialize the BSP LCD Msp.
  * Application can surcharge if needed this function implementation
  */
static void drvLcdMspInit(void)
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

  /** @brief NVIC configuration for LTDC interrupt that is now enabled */
  HAL_NVIC_SetPriority(LTDC_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(LTDC_IRQn);

  /** @brief NVIC configuration for DMA2D interrupt that is now enabled */
  HAL_NVIC_SetPriority(DMA2D_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA2D_IRQn);

  /** @brief NVIC configuration for DSI interrupt that is now enabled */
  HAL_NVIC_SetPriority(DSI_IRQn, 0xF, 0);
  HAL_NVIC_EnableIRQ(DSI_IRQn);
}

/**
  * @brief  De-Initializes the BSP LCD Msp
  * Application can surcharge if needed this function implementation.
  */
static void drvLcdMspDeinit(void)
{
  /** @brief Disable IRQ of LTDC IP */
  HAL_NVIC_DisableIRQ(LTDC_IRQn);

  /** @brief Disable IRQ of DMA2D IP */
  HAL_NVIC_DisableIRQ(DMA2D_IRQn);

  /** @brief Disable IRQ of DSI IP */
  HAL_NVIC_DisableIRQ(DSI_IRQn);

  /** @brief Force and let in reset state LTDC, DMA2D and DSI Host + Wrapper IPs */
  __HAL_RCC_LTDC_FORCE_RESET();
  __HAL_RCC_DMA2D_FORCE_RESET();
  __HAL_RCC_DSI_FORCE_RESET();

  /** @brief Disable the LTDC, DMA2D and DSI Host and Wrapper clocks */
  __HAL_RCC_LTDC_CLK_DISABLE();
  __HAL_RCC_DMA2D_CLK_DISABLE();
  __HAL_RCC_DSI_CLK_DISABLE();
}




/**
  * @brief  Handles DMA2D interrupt request.
  * @note : Can be surcharged by application code implementation of the function.
  */
void DMA2D_IRQHandler(void)
{
  HAL_DMA2D_IRQHandler(&hdma2d);
}

/**
  * @brief  Handles DSI interrupt request.
  * @note : Can be surcharged by application code implementation of the function.
  */
void DSI_IRQHandler(void)
{
  HAL_DSI_IRQHandler(&(hdsi));
}

/**
  * @brief  Handles LTDC interrupt request.
  * @note : Can be surcharged by application code implementation of the function.
  */
void LTDC_IRQHandler(void)
{
  HAL_LTDC_IRQHandler(&(hltdc));
}

/**
  * @brief  This function handles LTDC Error interrupt Handler.
  * @note : Can be surcharged by application code implementation of the function.
  */

void LTDC_ER_IRQHandler(void)
{
  HAL_LTDC_IRQHandler(&(hltdc));
}



/**
  * @brief  Fills a buffer.
  * @param  layer_idx: Layer index
  * @param  p_dst: Pointer to destination buffer
  * @param  x_size: Buffer width
  * @param  y_size: Buffer height
  * @param  line_offset: Offset
  * @param  color_idx: color index
  */
static err_code_t fillBuffer(uint32_t layer_idx, void *p_dst, uint32_t x_size, uint32_t y_size, uint32_t line_offset, uint32_t color_idx)
{
  if(layer_idx >= MAX_LAYER)
  {
    return ERR_LCD_INVAILD_LAYER;
  }

  /* Register to memory mode with ARGB8888 as color align */
  hdma2d.Init.Mode         = DMA2D_R2M;
  //hdma2d.Init.ColorMode    = DMA2D_ARGB8888;
  hdma2d.Init.ColorMode    = DMA2D_RGB565;

  hdma2d.Init.OutputOffset = line_offset;

  hdma2d.Instance = DMA2D;

  /* DMA2D Initialization */
  if(HAL_DMA2D_Init(&hdma2d) == HAL_OK)
  {
    if(HAL_DMA2D_ConfigLayer(&hdma2d, layer_idx) == HAL_OK)
    {
      if (HAL_DMA2D_Start(&hdma2d, color_idx, (uint32_t)p_dst, x_size, y_size) == HAL_OK)
      {
        /* Polling For DMA transfer */
        HAL_DMA2D_PollForTransfer(&hdma2d, 10);
      }
    }
  }

  return OK;
}

void drvLcdCopyPicture(uint32_t *p_src, uint32_t *p_dst)
{
  uint32_t destination = (uint32_t)p_dst;
  uint32_t source      = (uint32_t)p_src;

/*##-1- Configure the DMA2D Mode, Color Mode and output offset #############*/
  hdma2d.Init.Mode         = DMA2D_M2M;
  hdma2d.Init.ColorMode    = DMA2D_ARGB8888;
  hdma2d.Init.OutputOffset = 0;

  /*##-2- DMA2D Callbacks Configuration ######################################*/
  hdma2d.XferCpltCallback  = NULL;

  /*##-3- Foreground Configuration ###########################################*/
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0xFF;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].InputOffset = 0;

  hdma2d.Instance  = DMA2D;

  /* DMA2D Initialization */
  if(HAL_DMA2D_Init(&hdma2d) == HAL_OK)
  {
    if(HAL_DMA2D_ConfigLayer(&hdma2d, 1) == HAL_OK)
    {
      if (HAL_DMA2D_Start(&hdma2d, source, destination, 480, 800) == HAL_OK)
      {
        /* Polling For DMA transfer */
        HAL_DMA2D_PollForTransfer(&hdma2d, 100);
      }
    }
  }
}

void drvLcdCopyLayer(uint32_t src_index, uint32_t dst_index)
{
  uint32_t destination = (uint32_t)hltdc.LayerCfg[dst_index].FBStartAdress;
  uint32_t source      = (uint32_t)hltdc.LayerCfg[src_index].FBStartAdress;

/*##-1- Configure the DMA2D Mode, Color Mode and output offset #############*/
  hdma2d.Init.Mode         = DMA2D_M2M;
  hdma2d.Init.ColorMode    = DMA2D_RGB565;
  hdma2d.Init.OutputOffset = 0;

  /*##-2- DMA2D Callbacks Configuration ######################################*/
  hdma2d.XferCpltCallback  = NULL;

  /*##-3- Foreground Configuration ###########################################*/
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0xFF;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
  hdma2d.LayerCfg[1].InputOffset = 0;

  hdma2d.Instance  = DMA2D;

  /* DMA2D Initialization */
  if(HAL_DMA2D_Init(&hdma2d) == HAL_OK)
  {
    if(HAL_DMA2D_ConfigLayer(&hdma2d, 1) == HAL_OK)
    {
      if (HAL_DMA2D_Start(&hdma2d, source, destination, lcdGetXSize(), lcdGetYSize()) == HAL_OK)
      {
        /* Polling For DMA transfer */
        HAL_DMA2D_PollForTransfer(&hdma2d, 100);
      }
    }
  }
}

//TODO : Transfer complete callback
void HAL_DMA2D_CLUTLoadingCpltCallback(DMA2D_HandleTypeDef *hdma2d)
{
  UNUSED(hdma2d);
}


/**************************** LINK OTM8009A (Display driver) ******************/
/**
  * @brief  DCS or Generic short/long write command
  * @param  NbrParams: Number of parameters. It indicates the write command mode:
  *                 If inferior to 2, a long write command is performed else short.
  * @param  pParams: Pointer to parameter values table.
  * @retval HAL status
  */
void DSI_IO_WriteCmd(uint32_t NbrParams, uint8_t *pParams)
{
  if(NbrParams <= 1)
  {
   HAL_DSI_ShortWrite(&hdsi, LCD_OTM8009A_ID, DSI_DCS_SHORT_PKT_WRITE_P1, pParams[0], pParams[1]);
  }
  else
  {
   HAL_DSI_LongWrite(&hdsi,  LCD_OTM8009A_ID, DSI_DCS_LONG_PKT_WRITE, NbrParams, pParams[NbrParams], pParams);
  }
}

/**
  * @brief  OTM8009A delay
  * @param  Delay: Delay in ms
  */
void OTM8009A_IO_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
