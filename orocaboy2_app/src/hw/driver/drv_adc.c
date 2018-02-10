/*
 * drv_adc.c
 *
 *  Created on: 2017. 8. 10.
 *      Author: baram
 */

#include "hw.h"
#include "drv_adc.h"


ADC_HandleTypeDef    hADC1;





typedef struct
{
  GPIO_TypeDef       *port;
  uint16_t            pin;
} drv_adc_gpio_t;


typedef struct
{
  drv_adc_gpio_t      gpio;
  ADC_HandleTypeDef  *p_adc_handle;
  ADC_ChannelConfTypeDef adc_config;
  uint32_t               adc_channel;
} drv_adc_t;



//-- Internal Variables
//
drv_adc_t drv_adc_tbl[DRV_ADC_MAX_CH];
uint32_t  drv_adc_value[DRV_ADC_MAX_CH];

//-- External Variables
//


//-- Internal Functions
//


//-- External Functions
//





bool drvAdcInit(void)
{
  uint32_t i;
  GPIO_InitTypeDef  GPIO_InitStruct;


  drv_adc_tbl[0].gpio.port    = GPIOA;
  drv_adc_tbl[0].gpio.pin     = GPIO_PIN_6;
  drv_adc_tbl[0].p_adc_handle = &hADC1;
  drv_adc_tbl[0].adc_channel  = ADC_CHANNEL_6;

  drv_adc_tbl[1].gpio.port    = GPIOA;
  drv_adc_tbl[1].gpio.pin     = GPIO_PIN_7;
  drv_adc_tbl[1].p_adc_handle = &hADC1;
  drv_adc_tbl[1].adc_channel  = ADC_CHANNEL_7;

  drv_adc_tbl[2].gpio.port    = GPIOC;
  drv_adc_tbl[2].gpio.pin     = GPIO_PIN_0;
  drv_adc_tbl[2].p_adc_handle = &hADC1;
  drv_adc_tbl[2].adc_channel  = ADC_CHANNEL_10;

  drv_adc_tbl[3].gpio.port    = GPIOC;
  drv_adc_tbl[3].gpio.pin     = GPIO_PIN_1;
  drv_adc_tbl[3].p_adc_handle = &hADC1;
  drv_adc_tbl[3].adc_channel  = ADC_CHANNEL_11;

  drv_adc_tbl[4].gpio.port    = GPIOC;
  drv_adc_tbl[4].gpio.pin     = GPIO_PIN_2;
  drv_adc_tbl[4].p_adc_handle = &hADC1;
  drv_adc_tbl[4].adc_channel  = ADC_CHANNEL_12;

  drv_adc_tbl[5].gpio.port    = GPIOA;
  drv_adc_tbl[5].gpio.pin     = GPIO_PIN_5;
  drv_adc_tbl[5].p_adc_handle = &hADC1;
  drv_adc_tbl[5].adc_channel  = ADC_CHANNEL_5;

  drv_adc_tbl[6].gpio.port    = GPIOC;
  drv_adc_tbl[6].gpio.pin     = GPIO_PIN_3;
  drv_adc_tbl[6].p_adc_handle = &hADC1;
  drv_adc_tbl[6].adc_channel  = ADC_CHANNEL_13;


  // INFO: DMA를 이용하여 여러채널 사용시 반드시 ScanConvMode를 활성화 할것(메뉴얼 참조)
  //
  hADC1.Instance                   = ADC1;
  hADC1.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;
  hADC1.Init.Resolution            = ADC_RESOLUTION_12B;
  hADC1.Init.ScanConvMode          = ENABLE;
  hADC1.Init.ContinuousConvMode    = DISABLE;
  hADC1.Init.DiscontinuousConvMode = ENABLE;
  hADC1.Init.NbrOfDiscConversion   = 1;
  hADC1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hADC1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T5_CC3;
  hADC1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hADC1.Init.NbrOfConversion       = DRV_ADC_MAX_CH;
  hADC1.Init.DMAContinuousRequests = ENABLE;
  hADC1.Init.EOCSelection          = DISABLE;

  if(HAL_ADC_Init(&hADC1) != HAL_OK)
  {
    return false;
  }


  for (i=0; i<DRV_ADC_MAX_CH; i++)
  {
    if (drv_adc_tbl[i].gpio.port != NULL)
    {
      GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
      GPIO_InitStruct.Pull  = GPIO_NOPULL;
      GPIO_InitStruct.Pin   = drv_adc_tbl[i].gpio.pin;
      HAL_GPIO_Init(drv_adc_tbl[i].gpio.port, &GPIO_InitStruct);


      drv_adc_tbl[i].adc_config.Channel      = drv_adc_tbl[i].adc_channel;
      drv_adc_tbl[i].adc_config.Rank         = 1 + i;
      drv_adc_tbl[i].adc_config.SamplingTime = ADC_SAMPLETIME_56CYCLES;
      drv_adc_tbl[i].adc_config.Offset       = 0;

      HAL_ADC_ConfigChannel(drv_adc_tbl[i].p_adc_handle, &drv_adc_tbl[i].adc_config);
    }
  }

  if(HAL_ADC_Start_DMA(&hADC1, drv_adc_value, DRV_ADC_MAX_CH) != HAL_OK)
  {
  }
  return true;
}

uint16_t drvAdcRead(uint8_t ch)
{
  if (ch >= DRV_ADC_MAX_CH) return 0;



  return drv_adc_value[ch];
}

uint16_t drvAdcRead8(uint8_t ch)
{
  uint16_t ret = 0;


  ret = drvAdcRead(ch) >> 4;

  return ret;
}

uint16_t drvAdcRead10(uint8_t ch)
{
  uint16_t ret = 0;


  ret = drvAdcRead(ch) >> 2;

  return ret;
}

uint16_t drvAdcRead12(uint8_t ch)
{
  uint16_t ret = 0;

  ret = drvAdcRead(ch);

  return ret;
}

uint16_t drvAdcRead16(uint8_t ch)
{
  uint16_t ret = 0;


  ret = drvAdcRead(ch) << 4;

  return ret;
}

uint16_t drvAdcReadVoltage(uint8_t ch)
{
  uint16_t ret;


  ret = drvAdcConvVoltage(ch, drvAdcRead(ch));

  return ret;
}

uint16_t drvAdcConvVoltage(uint8_t ch, uint32_t adc_value)
{
  uint16_t ret = 0;


  switch(ch)
  {
    case _DEF_ADC1:
    case _DEF_ADC2:
    case _DEF_ADC3:
    case _DEF_ADC4:
    case _DEF_ADC5:
    case _DEF_ADC7:
      ret = (uint16_t)((adc_value * 330 * 151 / 100) / 4095);
      break;

    case _DEF_ADC6:
      ret = (uint16_t)((adc_value * 330 * 253 / 33) / 4095);
      break;
  }

  return ret;
}

uint8_t  drvAdcGetRes(uint8_t ch)
{
  return 12;
}

#if 0
void DMA2_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hADC1.DMA_Handle);
}
#endif

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
  ledOff(0);
  ledOn(0);
}


void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  static DMA_HandleTypeDef  hdma_adc;

  if (hadc->Instance == ADC1)
  {
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /*##-3- Configure the DMA streams ##########################################*/
    /* Set the parameters to be configured */
    hdma_adc.Instance = DMA2_Stream0;

    hdma_adc.Init.Channel             = DMA_CHANNEL_0;
    hdma_adc.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma_adc.Init.Mode                = DMA_CIRCULAR;
    hdma_adc.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_adc.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_adc.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_HALFFULL;
    hdma_adc.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_adc.Init.PeriphBurst         = DMA_PBURST_SINGLE;

    HAL_DMA_Init(&hdma_adc);

    /* Associate the initialized DMA handle to the the ADC handle */
    __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc);

    /*##-4- Configure the NVIC for DMA #########################################*/
    /* NVIC configuration for DMA transfer complete interrupt */
    //HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 6, 0);
    //HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  }

  if (hadc->Instance == ADC2)
  {
    __HAL_RCC_ADC2_CLK_ENABLE();
  }

}


void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
}

