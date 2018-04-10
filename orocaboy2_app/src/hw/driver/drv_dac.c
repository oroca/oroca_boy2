/*
 * drv_dac.c
 *
 *  Created on: 2017. 8. 10.
 *      Author: baram
 */

#include "hw.h"
#include "drv_dac.h"






#define DAC_BUFFER_MAX      (1024*2)




DAC_HandleTypeDef       DacHandle;
TIM_HandleTypeDef       htim;


typedef struct
{
  DAC_ChannelConfTypeDef  sConfig;
  uint32_t                channel;
  uint8_t                 resolution;
  uint8_t                 buffer[DAC_BUFFER_MAX];
} dac_t;



static ring_buf_t tx_buf;
static uint32_t   dac_hz = 0;

static dac_t dac_tbl[DAC_CH_MAX];


//-- External Variables
//


//-- Internal Functions
//


//-- External Functions
//





bool drvDacInit(void)
{
  return true;
}

bool drvDacDeInit(void)
{
  return true;
}

bool drvDacSetup(uint32_t hz)
{
  dac_hz = hz;

  return true;
}

bool drvDacStart(uint8_t ch)
{
  DacHandle.Instance = DAC1;

  HAL_DAC_Init(&DacHandle);

  dac_tbl[0].channel    = DAC_CHANNEL_1;
  dac_tbl[0].resolution = 8;


  dac_tbl[0].sConfig.DAC_Trigger      = DAC_TRIGGER_NONE;
  dac_tbl[0].sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

  HAL_DAC_ConfigChannel(&DacHandle, &dac_tbl[0].sConfig, dac_tbl[0].channel);

  drvDacWrite(0, 0);

  HAL_DAC_Start(&DacHandle, dac_tbl[0].channel);

  return true;
}

void drvDacStop(uint8_t ch)
{
  HAL_DAC_DeInit(&DacHandle);
}

uint32_t drvDacAvailable(uint8_t ch)
{
  uint32_t length = 0;

  return length;
}

void drvDacWrite(uint8_t ch, uint16_t data)
{
  HAL_DAC_SetValue(&DacHandle, dac_tbl[0].channel, DAC_ALIGN_8B_R, data);
}




volatile uint32_t dac_isr_count = 0;

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
  dac_isr_count++;
}

void HAL_DAC_MspInit(DAC_HandleTypeDef *hdac)
{
  GPIO_InitTypeDef          GPIO_InitStruct;
  static DMA_HandleTypeDef  hdma_dac1;


  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO clock ****************************************/
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /* DAC Periph clock enable */
  __HAL_RCC_DAC_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* DAC Channel1 GPIO pin configuration */
  GPIO_InitStruct.Pin   = GPIO_PIN_4;
  GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief  DeInitializes the DAC MSP.
  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @retval None
  */
void HAL_DAC_MspDeInit(DAC_HandleTypeDef *hdac)
{
  __HAL_RCC_DAC_FORCE_RESET();
  __HAL_RCC_DAC_RELEASE_RESET();

  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);
}


