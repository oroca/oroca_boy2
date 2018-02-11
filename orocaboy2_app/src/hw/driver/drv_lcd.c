
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
#include "lib/fonts/fonts.h"
#include "lib/otm8009a/otm8009a.h"

#define ABS(X)                 ((X) > 0 ? (X) : -(X))
#define POLY_X(Z)              ((int32_t)((points + (Z))->X))
#define POLY_Y(Z)              ((int32_t)((points + (Z))->Y))

#define LCD_DEFAULT_FONT       _DEF_FONT24

#define LCD_DSI_PIXEL_DATA_FMT_RBG888  DSI_RGB888 /*!< DSI packet pixel format chosen is RGB888 : 24 bpp */
#define LCD_DSI_PIXEL_DATA_FMT_RBG565  DSI_RGB565 /*!< DSI packet pixel format chosen is RGB565 : 16 bpp */

#define LCD_FB_START_ADDRESS              ((uint32_t)0xC0000000)
#define LTDC_NB_OF_LAYERS                 ((uint32_t) 2)
#define LTDC_MAX_LAYER_NUMBER             ((uint32_t) 2)
#define LTDC_ACTIVE_LAYER_BACKGROUND      ((uint32_t) 0)
#define LTDC_ACTIVE_LAYER_FOREGROUND      ((uint32_t) 1)
#define LTDC_DEFAULT_ACTIVE_LAYER         LTDC_ACTIVE_LAYER_FOREGROUND

#define LCD_OTM8009A_ID                   ((uint32_t) 0)


/**
* @brief  LCD Drawing main properties
*/
typedef struct
{
  uint32_t TextColor; /*!< Specifies the color of text */
  uint32_t BackColor; /*!< Specifies the background color below the text */
  sFONT    *pFont;    /*!< Specifies the font used for the text */

} LCD_DrawPropTypeDef;


uint32_t lcd_x_size = OTM8009A_800X480_WIDTH;
uint32_t lcd_y_size = OTM8009A_800X480_HEIGHT;
static uint32_t  ActiveLayer = LTDC_ACTIVE_LAYER_BACKGROUND;


DSI_HandleTypeDef hdsi;
LTDC_HandleTypeDef  hltdc;
DMA2D_HandleTypeDef hdma2d;
static DSI_VidCfgTypeDef hdis_video_conf;
static LCD_DrawPropTypeDef DrawProp[LTDC_MAX_LAYER_NUMBER];


static void drawChar(uint16_t x_pos, uint16_t y_pos, const uint8_t *c);
static void fillTriangle(uint16_t x1, uint16_t x2, uint16_t x3, uint16_t y1, uint16_t y2, uint16_t y3);
static void fillBuffer(uint32_t layer_idx, void *pDst, uint32_t x_size, uint32_t y_size, uint32_t line_offset, uint32_t color_idx);
static void convertLineToARGB8888(void * pSrc, void *pDst, uint32_t x_size, uint32_t color_mode);

void     drvLcdMspDeinit(void);
void     drvLcdMspInit(void);

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
  static RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
  uint32_t LcdClock  = 27429; /*!< LcdClk = 27429 kHz */
  uint32_t otm8009a_lcd_orient;

  uint32_t laneByteClk_kHz = 0;
  uint32_t                   VSA; /*!< Vertical start active time in units of lines */
  uint32_t                   VBP; /*!< Vertical Back Porch time in units of lines */
  uint32_t                   VFP; /*!< Vertical Front Porch time in units of lines */
  uint32_t                   VACT; /*!< Vertical Active time in units of lines = imageSize Y in pixels to display */
  uint32_t                   HSA; /*!< Horizontal start active time in units of lcdClk */
  uint32_t                   HBP; /*!< Horizontal Back Porch time in units of lcdClk */
  uint32_t                   HFP; /*!< Horizontal Front Porch time in units of lcdClk */
  uint32_t                   HACT; /*!< Horizontal Active time in units of lcdClk = imageSize X in pixels to display */


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

#if !defined(USE_STM32469I_DISCO_REVA)
  dsiPllInit.PLLNDIV  = 125;
  dsiPllInit.PLLIDF   = DSI_PLL_IN_DIV2;
  dsiPllInit.PLLODF   = DSI_PLL_OUT_DIV1;
#else
  dsiPllInit.PLLNDIV  = 100;
  dsiPllInit.PLLIDF   = DSI_PLL_IN_DIV5;
  dsiPllInit.PLLODF   = DSI_PLL_OUT_DIV1;
#endif
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

  HACT = lcd_x_size;
  VACT = lcd_y_size;

  /* The following values are same for portrait and landscape orientations */
  VSA  = OTM8009A_480X800_VSYNC;
  VBP  = OTM8009A_480X800_VBP;
  VFP  = OTM8009A_480X800_VFP;
  HSA  = OTM8009A_480X800_HSYNC;
  HBP  = OTM8009A_480X800_HBP;
  HFP  = OTM8009A_480X800_HFP;


  hdis_video_conf.VirtualChannelID = LCD_OTM8009A_ID;
  hdis_video_conf.ColorCoding = LCD_DSI_PIXEL_DATA_FMT_RBG888;
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
  drvSdramInit();
#endif /* DATA_IN_ExtSDRAM */

  /* Initialize the font */
  drvLcdSetFont(LCD_DEFAULT_FONT);

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
  OTM8009A_Init(OTM8009A_FORMAT_RGB888, otm8009a_lcd_orient);

/***********************End OTM8009A Initialization****************************/

  return OK;
}

/**
  * @brief  BSP LCD Reset
  *         Hw reset the LCD DSI activating its XRES signal (active low for some time)
  *         and desactivating it later.
  *         This signal is only cabled on Discovery Rev B and beyond.
  */
void drvLcdReset(void)
{
#if !defined(USE_STM32469I_DISCO_REVA)
/* EVAL Rev B and beyond : reset the LCD by activation of XRES (active low) connected to PH7 */
  GPIO_InitTypeDef  gpio_init_structure;

  __HAL_RCC_GPIOH_CLK_ENABLE();

    /* Configure the GPIO on PH7 */
    gpio_init_structure.Pin   = GPIO_PIN_7;
    gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio_init_structure.Pull  = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;

    HAL_GPIO_Init(GPIOH, &gpio_init_structure);

    /* Activate XRES active low */
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, GPIO_PIN_RESET);

    HAL_Delay(20); /* wait 20 ms */

    /* Desactivate XRES */
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, GPIO_PIN_SET);

    /* Wait for 10ms after releasing XRES before sending commands */
    HAL_Delay(10);
#else

#endif /* USE_STM32469I_DISCO_REVA == 0 */
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
  hltdc.LayerCfg[ActiveLayer].ImageWidth = image_width_pixels;
}

/**
  * @brief  Set the LCD Y size.
  * @param  image_height_pixels : uint32_t image height in lines unit
  */
void drvLcdSetYSize(uint32_t image_height_pixels)
{
  hltdc.LayerCfg[ActiveLayer].ImageHeight = image_height_pixels;
}


/**
  * @brief  Initializes the LCD layers.
  * @param  layer_idx: Layer foreground or background
  * @param  fb_addr: Layer frame buffer
  */
void drvLcdInitLayerDefault(uint16_t layer_idx, uint32_t fb_addr)
{
  LTDC_LayerCfgTypeDef Layercfg;

  /* Layer Init */
  Layercfg.WindowX0 = 0;
  Layercfg.WindowX1 = drvLcdGetXSize();
  Layercfg.WindowY0 = 0;
  Layercfg.WindowY1 = drvLcdGetYSize();
  Layercfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
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

  DrawProp[layer_idx].BackColor = LCD_COLOR_WHITE;
  DrawProp[layer_idx].pFont     = &Font24;
  DrawProp[layer_idx].TextColor = LCD_COLOR_BLACK;
}


/**
  * @brief  Selects the LCD Layer.
  * @param  layer_idx: Layer foreground or background
  */
void drvLcdSelectLayer(uint32_t layer_idx)
{
  ActiveLayer = layer_idx;
}

/**
  * @brief  Sets an LCD Layer visible
  * @param  layer_idx: Visible Layer
  * @param  state: New state of the specified layer
  *          This parameter can be one of the following values:
  *            @arg  ENABLE
  *            @arg  DISABLE
  */
void drvLcdSetLayerVisible(uint32_t layer_idx, uint8_t state)
{
  if(state == _DEF_ENABLE)
  {
    __HAL_LTDC_LAYER_ENABLE(&(hltdc), layer_idx);
  }
  else
  {
    __HAL_LTDC_LAYER_DISABLE(&(hltdc), layer_idx);
  }
  __HAL_LTDC_RELOAD_IMMEDIATE_CONFIG(&(hltdc));

}

/**
  * @brief  Configures the transparency.
  * @param  layer_idx: Layer foreground or background.
  * @param  transparency: transparency
  *           This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF
  */
void drvLcdSetTransparency(uint32_t layer_idx, uint8_t transparency)
{

  HAL_LTDC_SetAlpha(&(hltdc), transparency, layer_idx);

}

/**
  * @brief  Sets an LCD layer frame buffer address.
  * @param  layer_idx: Layer foreground or background
  * @param  addr: New LCD frame buffer value
  */
void drvLcdSetLayerAddr(uint32_t layer_idx, uint32_t addr)
{

  HAL_LTDC_SetAddress(&(hltdc), addr, layer_idx);

}

/**
  * @brief  Sets display window.
  * @param  layer_idx: Layer index
  * @param  x_pos: LCD X position
  * @param  y_pos: LCD Y position
  * @param  width: LCD window width
  * @param  height: LCD window height
  */
void drvLcdSetLayerWindow(uint16_t layer_idx, uint16_t x_pos, uint16_t y_pos, uint16_t width, uint16_t height)
{
  /* Reconfigure the layer size */
  HAL_LTDC_SetWindowSize(&(hltdc), width, height, layer_idx);

  /* Reconfigure the layer position */
  HAL_LTDC_SetWindowPosition(&(hltdc), x_pos, y_pos, layer_idx);

}

/**
  * @brief  Configures and sets the color keying.
  * @param  layer_idx: Layer foreground or background
  * @param  rgb_value: color reference
  */
void drvLcdSetColorKeying(uint32_t layer_idx, uint32_t rgb_value)
{
  /* Configure and Enable the color Keying for LCD Layer */
  HAL_LTDC_ConfigColorKeying(&(hltdc), rgb_value, layer_idx);
  HAL_LTDC_EnableColorKeying(&(hltdc), layer_idx);
}

/**
  * @brief  Disables the color keying.
  * @param  layer_idx: Layer foreground or background
  */
void drvLcdResetColorKeying(uint32_t layer_idx)
{
  /* Disable the color Keying for LCD Layer */
  HAL_LTDC_DisableColorKeying(&(hltdc), layer_idx);
}

/**
  * @brief  Sets the LCD text color.
  * @param  color: p_text color code ARGB(8-8-8-8)
  */
void drvLcdSetTextColor(uint32_t color)
{
  DrawProp[ActiveLayer].TextColor = color;
}

/**
  * @brief  Gets the LCD text color.
  * @retval Used text color.
  */
uint32_t drvLcdGetTextColor(void)
{
  return DrawProp[ActiveLayer].TextColor;
}

/**
  * @brief  Sets the LCD background color.
  * @param  color: Layer background color code ARGB(8-8-8-8)
  */
void drvLcdSetBackColor(uint32_t color)
{
  DrawProp[ActiveLayer].BackColor = color;
}

/**
  * @brief  Gets the LCD background color.
  * @retval Used background color
  */
uint32_t drvLcdGetBackColor(void)
{
  return DrawProp[ActiveLayer].BackColor;
}

/**
  * @brief  Sets the LCD text font.
  * @param  fonts: Layer font to be used
  */
void drvLcdSetFont(uint8_t font_type)
{
  sFONT *p_font;

  switch(font_type)
  {
    case _DEF_FONT8 :
      p_font = &Font8;
      break;
    case _DEF_FONT12 :
      p_font = &Font12;
      break;
    case _DEF_FONT16 :
      p_font = &Font16;
      break;
    case _DEF_FONT20 :
      p_font = &Font20;
      break;
    case _DEF_FONT24 :
    default :
      p_font = &Font24;
      break;
  }

  DrawProp[ActiveLayer].pFont = p_font;
}

/**
  * @brief  Gets the LCD text font.
  * @retval Used layer font
  */
sFONT* drvLcdGetFont(void)
{
  return DrawProp[ActiveLayer].pFont;
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

  if(hltdc.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_ARGB8888)
  {
    /* Read data value from SDRAM memory */
    ret = *(__IO uint32_t*) (hltdc.LayerCfg[ActiveLayer].FBStartAdress + (4*(y_pos*drvLcdGetXSize() + x_pos)));
  }
  else if(hltdc.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_RGB888)
  {
    /* Read data value from SDRAM memory */
    ret = (*(__IO uint32_t*) (hltdc.LayerCfg[ActiveLayer].FBStartAdress + (4*(y_pos*drvLcdGetXSize() + x_pos))) & 0x00FFFFFF);
  }
  else if((hltdc.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_RGB565) || \
          (hltdc.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_ARGB4444) || \
          (hltdc.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_AL88))
  {
    /* Read data value from SDRAM memory */
    ret = *(__IO uint16_t*) (hltdc.LayerCfg[ActiveLayer].FBStartAdress + (2*(y_pos*drvLcdGetXSize() + x_pos)));
  }
  else
  {
    /* Read data value from SDRAM memory */
    ret = *(__IO uint8_t*) (hltdc.LayerCfg[ActiveLayer].FBStartAdress + (2*(y_pos*drvLcdGetXSize() + x_pos)));
  }

  return ret;
}

/**
  * @brief  Clears the whole currently active layer of LTDC.
  * @param  color: color of the background
  */
void drvLcdClear(uint32_t color)
{
  /* Clear the LCD */
  fillBuffer(ActiveLayer, (uint32_t *)(hltdc.LayerCfg[ActiveLayer].FBStartAdress), drvLcdGetXSize(), drvLcdGetYSize(), 0, color);
}

/**
  * @brief  Clears the selected line in currently active layer.
  * @param  line: line to be cleared
  */
void drvLcdClearStringLine(uint32_t line)
{
  uint32_t color_backup = DrawProp[ActiveLayer].TextColor;
  DrawProp[ActiveLayer].TextColor = DrawProp[ActiveLayer].BackColor;

  /* Draw rectangle with background color */
  drvLcdFillRect(0, (line * DrawProp[ActiveLayer].pFont->Height), drvLcdGetXSize(), DrawProp[ActiveLayer].pFont->Height);

  DrawProp[ActiveLayer].TextColor = color_backup;
  drvLcdSetTextColor(DrawProp[ActiveLayer].TextColor);
}

/**
  * @brief  Displays one character in currently active layer.
  * @param  x_pos: Start column address
  * @param  y_pos: line where to display the character shape.
  * @param  ascii: Character ascii code
  *           This parameter must be a number between Min_Data = 0x20 and Max_Data = 0x7E
  */
void drvLcdDisplayChar(uint16_t x_pos, uint16_t y_pos, uint8_t ascii)
{
  drawChar(x_pos, y_pos, &DrawProp[ActiveLayer].pFont->table[(ascii-' ') *\
    DrawProp[ActiveLayer].pFont->Height * ((DrawProp[ActiveLayer].pFont->Width + 7) / 8)]);
}

/**
  * @brief  Displays characters in currently active layer.
  * @param  x_pos: X position (in pixel)
  * @param  y_pos: Y position (in pixel)
  * @param  p_text: Pointer to string to display on LCD
  * @param  align: Display mode
  *          This parameter can be one of the following values:
  *            @arg  CENTER_MODE
  *            @arg  RIGHT_MODE
  *            @arg  _DEF_LEFT
  */
void drvLcdDisplayStringAt(uint16_t x_pos, uint16_t y_pos, uint8_t *p_text, uint8_t align)
{
  uint16_t refcolumn = 1, i = 0;
  uint32_t size = 0, xsize = 0;
  uint8_t  *ptr = p_text;

  /* Get the text size */
  while (*ptr++) size ++ ;

  /* Characters number per line */
  xsize = (drvLcdGetXSize()/DrawProp[ActiveLayer].pFont->Width);

  switch (align)
  {
  case _DEF_CENTER:
    {
      refcolumn = x_pos + ((xsize - size)* DrawProp[ActiveLayer].pFont->Width) / 2;
      break;
    }
  case _DEF_LEFT:
    {
      refcolumn = x_pos;
      break;
    }
  case _DEF_RIGHT:
    {
      refcolumn = - x_pos + ((xsize - size)*DrawProp[ActiveLayer].pFont->Width);
      break;
    }
  default:
    {
      refcolumn = x_pos;
      break;
    }
  }

  /* Check that the Start column is located in the screen */
  if ((refcolumn < 1) || (refcolumn >= 0x8000))
  {
    refcolumn = 1;
  }

  /* Send the string character by character on LCD */
  while ((*p_text != 0) & (((drvLcdGetXSize() - (i*DrawProp[ActiveLayer].pFont->Width)) & 0xFFFF) >= DrawProp[ActiveLayer].pFont->Width))
  {
    /* Display one character on LCD */
    drvLcdDisplayChar(refcolumn, y_pos, *p_text);
    /* Decrement the column position by 16 */
    refcolumn += DrawProp[ActiveLayer].pFont->Width;

    /* Point on the next character */
    p_text++;
    i++;
  }

}

/**
  * @brief  Displays a maximum of 60 characters on the LCD.
  * @param  line: line where to display the character shape
  * @param  ptr: Pointer to string to display on LCD
  */
void drvLcdDisplayStringAtLine(uint16_t line, uint8_t *ptr)
{
  drvLcdDisplayStringAt(0, LINE(line), ptr, _DEF_LEFT);
}

/**
  * @brief  Draws an horizontal line in currently active layer.
  * @param  x_pos: X position
  * @param  y_pos: Y position
  * @param  length: line length
  */
void drvLcdDrawHLine(uint16_t x_pos, uint16_t y_pos, uint16_t length)
{
  uint32_t  Xaddress = 0;

  /* Get the line address */
  Xaddress = (hltdc.LayerCfg[ActiveLayer].FBStartAdress) + 4*(drvLcdGetXSize()*y_pos + x_pos);

  /* Write line */
  fillBuffer(ActiveLayer, (uint32_t *)Xaddress, length, 1, 0, DrawProp[ActiveLayer].TextColor);
}

/**
  * @brief  Draws a vertical line in currently active layer.
  * @param  x_pos: X position
  * @param  y_pos: Y position
  * @param  length: line length
  */
void drvLcdDrawVLine(uint16_t x_pos, uint16_t y_pos, uint16_t length)
{
  uint32_t  Xaddress = 0;

  /* Get the line address */
  Xaddress = (hltdc.LayerCfg[ActiveLayer].FBStartAdress) + 4*(drvLcdGetXSize()*y_pos + x_pos);

  /* Write line */
  fillBuffer(ActiveLayer, (uint32_t *)Xaddress, 1, length, (drvLcdGetXSize() - 1), DrawProp[ActiveLayer].TextColor);
}

/**
  * @brief  Draws an uni-line (between two points) in currently active layer.
  * @param  x1: Point 1 X position
  * @param  y1: Point 1 Y position
  * @param  x2: Point 2 X position
  * @param  y2: Point 2 Y position
  */
void drvLcdDrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
  curpixel = 0;

  deltax = ABS(x2 - x1);        /* The difference between the x's */
  deltay = ABS(y2 - y1);        /* The difference between the y's */
  x = x1;                       /* Start x off at the first pixel */
  y = y1;                       /* Start y off at the first pixel */

  if (x2 >= x1)                 /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }

  if (y2 >= y1)                 /* The y-values are increasing */
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                          /* The y-values are decreasing */
  {
    yinc1 = -1;
    yinc2 = -1;
  }

  if (deltax >= deltay)         /* There is at least one x-value for every y-value */
  {
    xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
    yinc2 = 0;                  /* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;         /* There are more x-values than y-values */
  }
  else                          /* There is at least one y-value for every x-value */
  {
    xinc2 = 0;                  /* Don't change the x for every iteration */
    yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;         /* There are more y-values than x-values */
  }

  for (curpixel = 0; curpixel <= numpixels; curpixel++)
  {
    drvLcdDrawPixel(x, y, DrawProp[ActiveLayer].TextColor);   /* Draw the current pixel */
    num += numadd;                            /* Increase the numerator by the top of the fraction */
    if (num >= den)                           /* Check if numerator >= denominator */
    {
      num -= den;                             /* Calculate the new numerator value */
      x += xinc1;                             /* Change the x as appropriate */
      y += yinc1;                             /* Change the y as appropriate */
    }
    x += xinc2;                               /* Change the x as appropriate */
    y += yinc2;                               /* Change the y as appropriate */
  }
}

/**
  * @brief  Draws a rectangle in currently active layer.
  * @param  x_pos: X position
  * @param  y_pos: Y position
  * @param  width: Rectangle width
  * @param  height: Rectangle height
  */
void drvLcdDrawRect(uint16_t x_pos, uint16_t y_pos, uint16_t width, uint16_t height)
{
  /* Draw horizontal lines */
  drvLcdDrawHLine(x_pos, y_pos, width);
  drvLcdDrawHLine(x_pos, (y_pos+ height), width);

  /* Draw vertical lines */
  drvLcdDrawVLine(x_pos, y_pos, height);
  drvLcdDrawVLine((x_pos + width), y_pos, height);
}

/**
  * @brief  Draws a circle in currently active layer.
  * @param  x_pos: X position
  * @param  y_pos: Y position
  * @param  radius: Circle radius
  */
void drvLcdDrawCircle(uint16_t x_pos, uint16_t y_pos, uint16_t radius)
{
  int32_t   D;    /* Decision Variable */
  uint32_t  CurX; /* Current X Value */
  uint32_t  CurY; /* Current Y Value */

  D = 3 - (radius << 1);
  CurX = 0;
  CurY = radius;

  while (CurX <= CurY)
  {
    drvLcdDrawPixel((x_pos + CurX), (y_pos - CurY), DrawProp[ActiveLayer].TextColor);

    drvLcdDrawPixel((x_pos - CurX), (y_pos - CurY), DrawProp[ActiveLayer].TextColor);

    drvLcdDrawPixel((x_pos + CurY), (y_pos - CurX), DrawProp[ActiveLayer].TextColor);

    drvLcdDrawPixel((x_pos - CurY), (y_pos - CurX), DrawProp[ActiveLayer].TextColor);

    drvLcdDrawPixel((x_pos + CurX), (y_pos + CurY), DrawProp[ActiveLayer].TextColor);

    drvLcdDrawPixel((x_pos - CurX), (y_pos + CurY), DrawProp[ActiveLayer].TextColor);

    drvLcdDrawPixel((x_pos + CurY), (y_pos + CurX), DrawProp[ActiveLayer].TextColor);

    drvLcdDrawPixel((x_pos - CurY), (y_pos + CurX), DrawProp[ActiveLayer].TextColor);

    if (D < 0)
    {
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }
}

/**
  * @brief  Draws an poly-line (between many points) in currently active layer.
  * @param  points: Pointer to the points array
  * @param  point_cnt: Number of points
  */
void drvLcdDrawPolygon(p_pixel points, uint16_t point_cnt)
{
  int16_t X = 0, Y = 0;

  if(point_cnt < 2)
  {
    return;
  }

  drvLcdDrawLine(points->X, points->Y, (points+point_cnt-1)->X, (points+point_cnt-1)->Y);

  while(--point_cnt)
  {
    X = points->X;
    Y = points->Y;
    points++;
    drvLcdDrawLine(X, Y, points->X, points->Y);
  }
}

/**
  * @brief  Draws an ellipse on LCD in currently active layer.
  * @param  x_pos: X position
  * @param  y_pos: Y position
  * @param  x_rad: Ellipse X radius
  * @param  y_rad: Ellipse Y radius
  */
void drvLcdDrawEllipse(int32_t x_pos, int32_t y_pos, int32_t x_rad, int32_t y_rad)
{
  int32_t x = 0, y = -y_rad, err = 2-2*x_rad, e2;
  float K = 0, rad1 = 0, rad2 = 0;

  rad1 = x_rad;
  rad2 = y_rad;

  K = (float)(rad2/rad1);

  do {
    drvLcdDrawPixel((x_pos-(uint16_t)(x/K)), (y_pos+y), DrawProp[ActiveLayer].TextColor);
    drvLcdDrawPixel((x_pos+(uint16_t)(x/K)), (y_pos+y), DrawProp[ActiveLayer].TextColor);
    drvLcdDrawPixel((x_pos+(uint16_t)(x/K)), (y_pos-y), DrawProp[ActiveLayer].TextColor);
    drvLcdDrawPixel((x_pos-(uint16_t)(x/K)), (y_pos-y), DrawProp[ActiveLayer].TextColor);

    e2 = err;
    if (e2 <= x) {
      err += ++x*2+1;
      if (-y == x && e2 <= y) e2 = 0;
    }
    if (e2 > y) err += ++y*2+1;
  }
  while (y <= 0);
}

/**
  * @brief  Draws a bitmap picture loaded in the internal Flash (32 bpp) in currently active layer.
  * @param  x_pos: Bmp X position in the LCD
  * @param  y_pos: Bmp Y position in the LCD
  * @param  pbmp: Pointer to Bmp picture address in the internal Flash
  */
void drvLcdDrawBitmap(uint32_t x_pos, uint32_t y_pos, uint8_t *pbmp)
{
  uint32_t index = 0, width = 0, height = 0, bit_pixel = 0;
  uint32_t addr;
  uint32_t InputColorMode = 0;

  /* Get bitmap data address offset */
  index = pbmp[10] + (pbmp[11] << 8) + (pbmp[12] << 16)  + (pbmp[13] << 24);

  /* Read bitmap width */
  width = pbmp[18] + (pbmp[19] << 8) + (pbmp[20] << 16)  + (pbmp[21] << 24);

  /* Read bitmap height */
  height = pbmp[22] + (pbmp[23] << 8) + (pbmp[24] << 16)  + (pbmp[25] << 24);

  /* Read bit/pixel */
  bit_pixel = pbmp[28] + (pbmp[29] << 8);

  /* Set the address */
  addr = hltdc.LayerCfg[ActiveLayer].FBStartAdress + (((drvLcdGetXSize()*y_pos) + x_pos)*(4));

  /* Get the layer pixel format */
  if ((bit_pixel/8) == 4)
  {
    InputColorMode = CM_ARGB8888;
  }
  else if ((bit_pixel/8) == 2)
  {
    InputColorMode = CM_RGB565;
  }
  else
  {
    InputColorMode = CM_RGB888;
  }

  /* Bypass the bitmap header */
  pbmp += (index + (width * (height - 1) * (bit_pixel/8)));

  /* Convert picture to ARGB8888 pixel format */
  for(index=0; index < height; index++)
  {
    /* Pixel format conversion */
    convertLineToARGB8888((uint32_t *)pbmp, (uint32_t *)addr, width, InputColorMode);

    /* Increment the source and destination buffers */
    addr+=  (drvLcdGetXSize()*4);
    pbmp -= width*(bit_pixel/8);
  }
}

/**
  * @brief  Draws a full rectangle in currently active layer.
  * @param  x_pos: X position
  * @param  y_pos: Y position
  * @param  width: Rectangle width
  * @param  height: Rectangle height
  */
void drvLcdFillRect(uint16_t x_pos, uint16_t y_pos, uint16_t width, uint16_t height)
{
  uint32_t  Xaddress = 0;

  /* Set the text color */
  drvLcdSetTextColor(DrawProp[ActiveLayer].TextColor);

  /* Get the rectangle start address */
  Xaddress = (hltdc.LayerCfg[ActiveLayer].FBStartAdress) + 4*(drvLcdGetXSize()*y_pos + x_pos);

  /* Fill the rectangle */
  fillBuffer(ActiveLayer, (uint32_t *)Xaddress, width, height, (drvLcdGetXSize() - width), DrawProp[ActiveLayer].TextColor);
}

/**
  * @brief  Draws a full circle in currently active layer.
  * @param  x_pos: X position
  * @param  y_pos: Y position
  * @param  radius: Circle radius
  */
void drvLcdFillCircle(uint16_t x_pos, uint16_t y_pos, uint16_t radius)
{
  int32_t  D;     /* Decision Variable */
  uint32_t  CurX; /* Current X Value */
  uint32_t  CurY; /* Current Y Value */

  D = 3 - (radius << 1);

  CurX = 0;
  CurY = radius;

  drvLcdSetTextColor(DrawProp[ActiveLayer].TextColor);

  while (CurX <= CurY)
  {
    if(CurY > 0)
    {
      drvLcdDrawHLine(x_pos - CurY, y_pos + CurX, 2*CurY);
      drvLcdDrawHLine(x_pos - CurY, y_pos - CurX, 2*CurY);
    }

    if(CurX > 0)
    {
      drvLcdDrawHLine(x_pos - CurX, y_pos - CurY, 2*CurX);
      drvLcdDrawHLine(x_pos - CurX, y_pos + CurY, 2*CurX);
    }
    if (D < 0)
    {
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }

  drvLcdSetTextColor(DrawProp[ActiveLayer].TextColor);
  drvLcdDrawCircle(x_pos, y_pos, radius);
}

/**
  * @brief  Draws a full poly-line (between many points) in currently active layer.
  * @param  points: Pointer to the points array
  * @param  point_cnt: Number of points
  */
void drvLcdFillPolygon(p_pixel points, uint16_t point_cnt)
{
  int16_t X = 0, Y = 0, X2 = 0, Y2 = 0, X_center = 0, Y_center = 0, X_first = 0, Y_first = 0, pixelX = 0, pixelY = 0, counter = 0;
  uint16_t  IMAGE_LEFT = 0, IMAGE_RIGHT = 0, IMAGE_TOP = 0, IMAGE_BOTTOM = 0;

  IMAGE_LEFT = IMAGE_RIGHT = points->X;
  IMAGE_TOP= IMAGE_BOTTOM = points->Y;

  for(counter = 1; counter < point_cnt; counter++)
  {
    pixelX = POLY_X(counter);
    if(pixelX < IMAGE_LEFT)
    {
      IMAGE_LEFT = pixelX;
    }
    if(pixelX > IMAGE_RIGHT)
    {
      IMAGE_RIGHT = pixelX;
    }

    pixelY = POLY_Y(counter);
    if(pixelY < IMAGE_TOP)
    {
      IMAGE_TOP = pixelY;
    }
    if(pixelY > IMAGE_BOTTOM)
    {
      IMAGE_BOTTOM = pixelY;
    }
  }

  if(point_cnt < 2)
  {
    return;
  }

  X_center = (IMAGE_LEFT + IMAGE_RIGHT)/2;
  Y_center = (IMAGE_BOTTOM + IMAGE_TOP)/2;

  X_first = points->X;
  Y_first = points->Y;

  while(--point_cnt)
  {
    X = points->X;
    Y = points->Y;
    points++;
    X2 = points->X;
    Y2 = points->Y;

    fillTriangle(X, X2, X_center, Y, Y2, Y_center);
    fillTriangle(X, X_center, X2, Y, Y_center, Y2);
    fillTriangle(X_center, X2, X, Y_center, Y2, Y);
  }

  fillTriangle(X_first, X2, X_center, Y_first, Y2, Y_center);
  fillTriangle(X_first, X_center, X2, Y_first, Y_center, Y2);
  fillTriangle(X_center, X2, X_first, Y_center, Y2, Y_first);
}

/**
  * @brief  Draws a full ellipse in currently active layer.
  * @param  x_pos: X position
  * @param  y_pos: Y position
  * @param  x_rad: Ellipse X radius
  * @param  y_rad: Ellipse Y radius
  */
void drvLcdFillEllipse(int32_t x_pos, int32_t y_pos, int32_t x_rad, int32_t y_rad)
{
  int32_t x = 0, y = -y_rad, err = 2-2*x_rad, e2;
  float K = 0, rad1 = 0, rad2 = 0;

  rad1 = x_rad;
  rad2 = y_rad;

  K = (float)(rad2/rad1);

  do
  {
    drvLcdDrawHLine((x_pos-(uint16_t)(x/K)), (y_pos+y), (2*(uint16_t)(x/K) + 1));
    drvLcdDrawHLine((x_pos-(uint16_t)(x/K)), (y_pos-y), (2*(uint16_t)(x/K) + 1));

    e2 = err;
    if (e2 <= x)
    {
      err += ++x*2+1;
      if (-y == x && e2 <= y) e2 = 0;
    }
    if (e2 > y) err += ++y*2+1;
  }
  while (y <= 0);
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

/*******************************************************************************
                       LTDC, DMA2D and DSI BSP Routines
*******************************************************************************/
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
  * @brief  De-Initializes the BSP LCD Msp
  * Application can surcharge if needed this function implementation.
  */
__weak void drvLcdMspDeinit(void)
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
  * @brief  Initialize the BSP LCD Msp.
  * Application can surcharge if needed this function implementation
  */
__weak void drvLcdMspInit(void)
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
  HAL_NVIC_SetPriority(DSI_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DSI_IRQn);
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
  * @brief  Draws a pixel on LCD.
  * @param  x_pos: X position
  * @param  y_pos: Y position
  * @param  rgb_code: Pixel color in ARGB mode (8-8-8-8)
  */
void drvLcdDrawPixel(uint16_t x_pos, uint16_t y_pos, uint32_t rgb_code)
{
  /* Write data value to all SDRAM memory */
  *(__IO uint32_t*) (hltdc.LayerCfg[ActiveLayer].FBStartAdress + (4*(y_pos*drvLcdGetXSize() + x_pos))) = rgb_code;
}


/**
  * @brief  Draws a character on LCD.
  * @param  x_pos: line where to display the character shape
  * @param  y_pos: Start column address
  * @param  c: Pointer to the character data
  */
static void drawChar(uint16_t x_pos, uint16_t y_pos, const uint8_t *c)
{
  uint32_t i = 0, j = 0;
  uint16_t height, width;
  uint8_t  offset;
  uint8_t  *pchar;
  uint32_t line;

  height = DrawProp[ActiveLayer].pFont->Height;
  width  = DrawProp[ActiveLayer].pFont->Width;

  offset =  8 *((width + 7)/8) -  width ;

  for(i = 0; i < height; i++)
  {
    pchar = ((uint8_t *)c + (width + 7)/8 * i);

    switch(((width + 7)/8))
    {

    case 1:
      line =  pchar[0];
      break;

    case 2:
      line =  (pchar[0]<< 8) | pchar[1];
      break;

    case 3:
    default:
      line =  (pchar[0]<< 16) | (pchar[1]<< 8) | pchar[2];
      break;
    }

    for (j = 0; j < width; j++)
    {
      if(line & (1 << (width- j + offset- 1)))
      {
        drvLcdDrawPixel((x_pos + j), y_pos, DrawProp[ActiveLayer].TextColor);
      }
      else
      {
        drvLcdDrawPixel((x_pos + j), y_pos, DrawProp[ActiveLayer].BackColor);
      }
    }
    y_pos++;
  }
}

/**
  * @brief  Fills a triangle (between 3 points).
  * @param  x1: Point 1 X position
  * @param  y1: Point 1 Y position
  * @param  x2: Point 2 X position
  * @param  y2: Point 2 Y position
  * @param  x3: Point 3 X position
  * @param  y3: Point 3 Y position
  */
static void fillTriangle(uint16_t x1, uint16_t x2, uint16_t x3, uint16_t y1, uint16_t y2, uint16_t y3)
{
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
  curpixel = 0;

  deltax = ABS(x2 - x1);        /* The difference between the x's */
  deltay = ABS(y2 - y1);        /* The difference between the y's */
  x = x1;                       /* Start x off at the first pixel */
  y = y1;                       /* Start y off at the first pixel */

  if (x2 >= x1)                 /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }

  if (y2 >= y1)                 /* The y-values are increasing */
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                          /* The y-values are decreasing */
  {
    yinc1 = -1;
    yinc2 = -1;
  }

  if (deltax >= deltay)         /* There is at least one x-value for every y-value */
  {
    xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
    yinc2 = 0;                  /* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;         /* There are more x-values than y-values */
  }
  else                          /* There is at least one y-value for every x-value */
  {
    xinc2 = 0;                  /* Don't change the x for every iteration */
    yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;         /* There are more y-values than x-values */
  }

  for (curpixel = 0; curpixel <= numpixels; curpixel++)
  {
    drvLcdDrawLine(x, y, x3, y3);

    num += numadd;              /* Increase the numerator by the top of the fraction */
    if (num >= den)             /* Check if numerator >= denominator */
    {
      num -= den;               /* Calculate the new numerator value */
      x += xinc1;               /* Change the x as appropriate */
      y += yinc1;               /* Change the y as appropriate */
    }
    x += xinc2;                 /* Change the x as appropriate */
    y += yinc2;                 /* Change the y as appropriate */
  }
}

/**
  * @brief  Fills a buffer.
  * @param  layer_idx: Layer index
  * @param  pDst: Pointer to destination buffer
  * @param  x_size: Buffer width
  * @param  y_size: Buffer height
  * @param  line_offset: Offset
  * @param  color_idx: color index
  */
static void fillBuffer(uint32_t layer_idx, void *pDst, uint32_t x_size, uint32_t y_size, uint32_t line_offset, uint32_t color_idx)
{
  /* Register to memory mode with ARGB8888 as color align */
  hdma2d.Init.Mode         = DMA2D_R2M;
  hdma2d.Init.ColorMode    = DMA2D_ARGB8888;
  hdma2d.Init.OutputOffset = line_offset;

  hdma2d.Instance = DMA2D;

  /* DMA2D Initialization */
  if(HAL_DMA2D_Init(&hdma2d) == HAL_OK)
  {
    if(HAL_DMA2D_ConfigLayer(&hdma2d, layer_idx) == HAL_OK)
    {
      if (HAL_DMA2D_Start(&hdma2d, color_idx, (uint32_t)pDst, x_size, y_size) == HAL_OK)
      {
        /* Polling For DMA transfer */
        HAL_DMA2D_PollForTransfer(&hdma2d, 10);
      }
    }
  }
}

/**
  * @brief  Converts a line to an ARGB8888 pixel format.
  * @param  pSrc: Pointer to source buffer
  * @param  pDst: Output color
  * @param  x_size: Buffer width
  * @param  color_mode: Input color mode
  */
static void convertLineToARGB8888(void *pSrc, void *pDst, uint32_t x_size, uint32_t color_mode)
{
  /* Configure the DMA2D align, color align and output offset */
  hdma2d.Init.Mode         = DMA2D_M2M_PFC;
  hdma2d.Init.ColorMode    = DMA2D_ARGB8888;
  hdma2d.Init.OutputOffset = 0;

  /* Foreground Configuration */
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0xFF;
  hdma2d.LayerCfg[1].InputColorMode = color_mode;
  hdma2d.LayerCfg[1].InputOffset = 0;

  hdma2d.Instance = DMA2D;

  /* DMA2D Initialization */
  if(HAL_DMA2D_Init(&hdma2d) == HAL_OK)
  {
    if(HAL_DMA2D_ConfigLayer(&hdma2d, 1) == HAL_OK)
    {
      if (HAL_DMA2D_Start(&hdma2d, (uint32_t)pSrc, (uint32_t)pDst, x_size, 1) == HAL_OK)
      {
        /* Polling For DMA transfer */
        HAL_DMA2D_PollForTransfer(&hdma2d, 10);
      }
    }
  }
}


/**************************** LINK OTM8009A (Display driver) ******************/
/**
  * @brief  OTM8009A delay
  * @param  Delay: Delay in ms
  */
void OTM8009A_IO_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
