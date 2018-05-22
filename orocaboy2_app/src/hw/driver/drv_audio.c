/*
 * drv_audio.c
 *
 *  Created on: Feb 21, 2018
 *      Author: opus
 */


#include "drv_audio.h"

#include "hw.h"
#include "cs43l22/cs43l22.h"


#define AUDIO_I2C_ADDRESS                ((uint16_t)0x94)


/** @defgroup BSP_Audio_Out_Option BSP AUDIO OUT option
  * @{
  */
#define BSP_AUDIO_OUT_CIRCULARMODE      ((uint32_t)0x00000001) /* BUFFER CIRCULAR MODE */
#define BSP_AUDIO_OUT_NORMALMODE        ((uint32_t)0x00000002) /* BUFFER NORMAL MODE   */
#define BSP_AUDIO_OUT_STEREOMODE        ((uint32_t)0x00000004) /* STEREO MODE          */
#define BSP_AUDIO_OUT_MONOMODE          ((uint32_t)0x00000008) /* MONO MODE            */


/** @defgroup CODEC_AudioFrame_SLOT_TDMMode  STM32469I Discovery Audio Slot TDM mode
  * @brief In W8994 codec the Audio frame contains 4 slots : TDM Mode
  * TDM format :
  * +------------------|------------------|--------------------|-------------------+
  * | CODEC_SLOT0 Left | CODEC_SLOT1 Left | CODEC_SLOT0 Right  | CODEC_SLOT1 Right |
  * +------------------------------------------------------------------------------+
  * @{
  */
/* To have 2 separate audio stream in Both headphone and speaker the 4 slot must be activated */
#define CODEC_AUDIOFRAME_SLOT_0123                   SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1 | SAI_SLOTACTIVE_2 | SAI_SLOTACTIVE_3
/* To have an audio stream in headphone only SAI Slot 0 and Slot 2 must be activated */
#define CODEC_AUDIOFRAME_SLOT_02                     SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_2
/* To have an audio stream in speaker only SAI Slot 1 and Slot 3 must be activated */
#define CODEC_AUDIOFRAME_SLOT_13                     SAI_SLOTACTIVE_1 | SAI_SLOTACTIVE_3

#define AUDIODATA_SIZE        2   /* 16-bits audio data size */

#define DEFAULT_OUTPUT_DEVICE   OUTPUT_DEVICE_AUTO
#define DEFAULT_OUTPUT_VOLUME   70


AUDIO_DrvTypeDef          *audio_drv;
SAI_HandleTypeDef         haudio_out_sai;
I2S_HandleTypeDef         haudio_in_i2s;
TIM_HandleTypeDef         haudio_tim;


static void drvAudioOutClockConfig(SAI_HandleTypeDef *hsai, uint32_t audio_freq, void *Params);
static void TIMx_IC_MspInit(TIM_HandleTypeDef *htim);
static void TIMx_IC_MspDeInit(TIM_HandleTypeDef *htim);
static void TIMx_Init(void);
static void TIMx_DeInit(void);


err_code_t drvAudioOutInit(uint32_t audio_freq)
{
  err_code_t ret = OK;
  uint16_t output_device = DEFAULT_OUTPUT_DEVICE;
  uint8_t volume = DEFAULT_OUTPUT_VOLUME;

  __HAL_SAI_DISABLE(&haudio_out_sai);

  HAL_SAI_DeInit(&haudio_out_sai);

  haudio_out_sai.Instance = SAI1_Block_A;

  drvAudioOutClockConfig(&haudio_out_sai, audio_freq, NULL);

  haudio_out_sai.Init.AudioFrequency = audio_freq;
  haudio_out_sai.Init.ClockSource = SAI_CLKSOURCE_PLLI2S;
  haudio_out_sai.Init.AudioMode = SAI_MODEMASTER_TX;
  haudio_out_sai.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  haudio_out_sai.Init.Protocol = SAI_FREE_PROTOCOL;
  haudio_out_sai.Init.DataSize = SAI_DATASIZE_8; // SAI_DATASIZE_16
  haudio_out_sai.Init.FirstBit = SAI_FIRSTBIT_MSB;
  haudio_out_sai.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  haudio_out_sai.Init.Synchro = SAI_ASYNCHRONOUS;
  haudio_out_sai.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;
  haudio_out_sai.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
  haudio_out_sai.Init.MonoStereoMode = SAI_MONOMODE;

  haudio_out_sai.FrameInit.FrameLength = 32; // 64
  haudio_out_sai.FrameInit.ActiveFrameLength = 16; // 32
  haudio_out_sai.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  haudio_out_sai.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  haudio_out_sai.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;

  haudio_out_sai.SlotInit.FirstBitOffset = 1;
  haudio_out_sai.SlotInit.SlotSize = SAI_SLOTSIZE_16B; // SAI_SLOTSIZE_DATASIZE
  haudio_out_sai.SlotInit.SlotNumber = 2;
  haudio_out_sai.SlotInit.SlotActive = SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1; // CODEC_AUDIOFRAME_SLOT_0123

  if (HAL_SAI_Init(&haudio_out_sai) != HAL_OK)
  {
    ret = ERR_AUDIO;
  }

  __HAL_SAI_ENABLE(&haudio_out_sai);


  haudio_in_i2s.Instance = SPI3;
  haudio_in_i2s.Init.Mode = I2S_MODE_MASTER_TX;
  haudio_in_i2s.Init.Standard = I2S_STANDARD_MSB;
  haudio_in_i2s.Init.DataFormat = I2S_DATAFORMAT_16B;
  haudio_in_i2s.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE; // Enables MCLK output
  haudio_in_i2s.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  haudio_in_i2s.Init.CPOL = I2S_CPOL_LOW;
  haudio_in_i2s.Init.ClockSource = I2S_CLOCK_PLL;
  haudio_in_i2s.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&haudio_in_i2s) != HAL_OK)
  {
    ret = ERR_AUDIO;
  }

  if(ret == OK)
  {
    if (cs43l22_drv.ReadID(AUDIO_I2C_ADDRESS) == CS43L22_ID)
    {
      audio_drv = &cs43l22_drv;
    }
    else
    {
      ret = ERR_AUDIO;
    }
  }

  if(ret == OK)
  {
    if (audio_drv->Init(AUDIO_I2C_ADDRESS, output_device, volume, audio_freq) != OK)
    {
      ret = ERR_AUDIO;
    }
  }

  drvAudioSelectOutDev(_DEF_AUDIO_HEADPHONE);

  return ret;
}

void    drvAudioOutDeInit(void)
{
  HAL_SAI_DeInit(&haudio_out_sai);

  memset(&audio_drv, 0, sizeof(audio_drv));
}

err_code_t drvAudioOutPlay(uint16_t* p_buf, uint32_t size)
{
  err_code_t ret = OK;

  if(audio_drv->Play(AUDIO_I2C_ADDRESS, p_buf, size) != 0)
  {
    ret = ERR_AUDIO;
  }

  if(ret == OK)
  {
    if (HAL_SAI_Transmit_DMA(&haudio_out_sai, (uint8_t*)p_buf, size/AUDIODATA_SIZE) != HAL_OK)
    {
      ret = ERR_AUDIO;
    }
  }

  return ret;
}

void drvAudioSelectOutDev(uint8_t dev)
{
  switch(dev)
  {
    case _DEF_AUDIO_HEADPHONE :
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
      break;
    case _DEF_AUDIO_SPEAKER :
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
      break;
    default :
      break;
  }
}

void drvAudioOutBeep(uint8_t note, uint32_t duration_ms)
{
  cs43l22_Beep(AUDIO_I2C_ADDRESS, note, duration_ms);
}

err_code_t drvAudioOutPause(void)
{
  err_code_t ret = OK;

  if(audio_drv->Pause(AUDIO_I2C_ADDRESS) != 0)
  {
    ret =  ERR_AUDIO;
  }

  if(ret == OK)
  {
    if (HAL_SAI_DMAPause(&haudio_out_sai)!= HAL_OK)
    {
      ret =  ERR_AUDIO;
    }
  }

  return ret;
}


err_code_t drvAudioOutResume(void)
{
  err_code_t ret = OK;

  if(audio_drv->Resume(AUDIO_I2C_ADDRESS) != 0)
  {
    ret =  ERR_AUDIO;
  }

  if(ret == OK)
  {
    if (HAL_SAI_DMAResume(&haudio_out_sai)!= HAL_OK)
    {
      ret =  ERR_AUDIO;
    }
  }

  return ret;
}

err_code_t drvAudioOutStop(void)
{
  err_code_t ret = OK;
  uint32_t option = CODEC_PDWN_SW;

  if(audio_drv->Stop(AUDIO_I2C_ADDRESS, option) != 0)
  {
    ret = ERR_AUDIO;
  }

  if(ret == OK)
  {
    if(option == CODEC_PDWN_HW)
    {
      HAL_Delay(2); /* Wait at least 100us */
    }

    if (HAL_SAI_DMAStop(&haudio_out_sai)!= HAL_OK)
    {
      ret = ERR_AUDIO;
    }
  }

  return ret;
}

err_code_t drvAudioOutSetVolume(uint8_t volume)
{
  err_code_t ret = OK;
  volume = constrain(volume, 0, 100);

  if(audio_drv->SetVolume(AUDIO_I2C_ADDRESS, volume) != 0)
  {
    ret =  ERR_AUDIO;
  }

  return ret;
}

void    drvAudioOutSetFrequency(uint32_t audio_freq)
{
  /* PLL clock is set depending by the audio_freq (44.1khz vs 48khz groups) */
  drvAudioOutClockConfig(&haudio_out_sai, audio_freq, NULL);

  /* Disable SAI peripheral to allow access to SAI internal registers */
  __HAL_SAI_DISABLE(&haudio_out_sai);

  /* Update the SAI audio frequency configuration */
  haudio_out_sai.Init.AudioFrequency = audio_freq;
  HAL_SAI_Init(&haudio_out_sai);

  /* Enable SAI peripheral to generate MCLK */
  __HAL_SAI_ENABLE(&haudio_out_sai);
}

void    drvAudioOutSetAudioFrameSlot(uint32_t frame_slot)
{
  __HAL_SAI_DISABLE(&haudio_out_sai);

  haudio_out_sai.SlotInit.SlotActive = frame_slot;
  HAL_SAI_Init(&haudio_out_sai);

  __HAL_SAI_ENABLE(&haudio_out_sai);
}

err_code_t drvAudioOutSetMute(uint32_t cmd)
{
  err_code_t ret = OK;
  uint32_t t_cmd = AUDIO_MUTE_OFF;

  if(cmd == _DEF_AUDIO_MUTE_ON)
  {
    t_cmd = AUDIO_MUTE_ON;
  }

  if(audio_drv->SetMute(AUDIO_I2C_ADDRESS, t_cmd) != 0)
  {
    ret = ERR_AUDIO;
  }

  return ret;
}

void    drvAudioOutChangeBuffer(uint16_t *p_data, uint16_t size)
{
  HAL_SAI_Transmit_DMA(&haudio_out_sai, (uint8_t*) p_data, size);
}

void drvAudioOutChangeConfig(uint32_t audio_out_option)
{
  /********** Playback Buffer circular/normal mode **********/
  if(audio_out_option & BSP_AUDIO_OUT_CIRCULARMODE)
  {
    /* Deinitialize the Stream to update DMA mode */
    HAL_DMA_DeInit(haudio_out_sai.hdmatx);

    /* Update the SAI audio Transfer DMA mode */
    haudio_out_sai.hdmatx->Init.Mode = DMA_CIRCULAR;

    /* Configure the DMA Stream with new Transfer DMA mode */
    HAL_DMA_Init(haudio_out_sai.hdmatx);
  }
  else /* BSP_AUDIO_OUT_NORMALMODE */
  {
    /* Deinitialize the Stream to update DMA mode */
    HAL_DMA_DeInit(haudio_out_sai.hdmatx);

    /* Update the SAI audio Transfer DMA mode */
    haudio_out_sai.hdmatx->Init.Mode = DMA_NORMAL;

    /* Configure the DMA Stream with new Transfer DMA mode */
    HAL_DMA_Init(haudio_out_sai.hdmatx);
  }

  /********** Playback Buffer stereo/mono mode **********/
  if(audio_out_option & BSP_AUDIO_OUT_STEREOMODE)
  {
    /* Disable SAI peripheral to allow access to SAI internal registers */
    __HAL_SAI_DISABLE(&haudio_out_sai);

    /* Update the SAI audio frame slot configuration */
    haudio_out_sai.Init.MonoStereoMode = SAI_STEREOMODE;
    HAL_SAI_Init(&haudio_out_sai);

    /* Enable SAI peripheral to generate MCLK */
    __HAL_SAI_ENABLE(&haudio_out_sai);
  }
  else /* BSP_AUDIO_OUT_MONOMODE */
  {
    /* Disable SAI peripheral to allow access to SAI internal registers */
    __HAL_SAI_DISABLE(&haudio_out_sai);

    /* Update the SAI audio frame slot configuration */
    haudio_out_sai.Init.MonoStereoMode = SAI_MONOMODE;
    HAL_SAI_Init(&haudio_out_sai);

    /* Enable SAI peripheral to generate MCLK */
    __HAL_SAI_ENABLE(&haudio_out_sai);
  }
}



void DMA2_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(haudio_out_sai.hdmatx);
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
	//BSP_AUDIO_OUT_ChangeBuffer((uint16_t*)&haudio.buff[0], AUDIO_OUT_BUFFER_SIZE /2);
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	//BSP_AUDIO_OUT_ChangeBuffer((uint16_t*)&haudio.buff[AUDIO_OUT_BUFFER_SIZE /2], AUDIO_OUT_BUFFER_SIZE /2);
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{

}

void DMA1_Stream2_IRQHandler(void)
{
HAL_DMA_IRQHandler(haudio_in_i2s.hdmarx);
}


static void  drvAudioOutClockConfig(SAI_HandleTypeDef *hsai, uint32_t audio_freq, void *Params)
{
  RCC_PeriphCLKInitTypeDef rcc_ex_clk_init_struct;

  HAL_RCCEx_GetPeriphCLKConfig(&rcc_ex_clk_init_struct);

  /* Set the PLL configuration according to the audio frequency */
  if((audio_freq == AUDIO_FREQUENCY_11K) || (audio_freq == AUDIO_FREQUENCY_22K) || (audio_freq == AUDIO_FREQUENCY_44K))
  {
    /* Configure PLLI2S prescalers */
    /* PLLI2S_VCO: VCO_429M
    I2S_CLK(first level) = PLLI2S_VCO/PLLI2SQ = 429/2 = 214.5 Mhz
    I2S_CLK_x = I2S_CLK(first level)/PLLI2SDIVQ = 214.5/19 = 11.289 Mhz */
    rcc_ex_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_SAI_PLLI2S;
    rcc_ex_clk_init_struct.PLLI2S.PLLI2SN = 429;
    rcc_ex_clk_init_struct.PLLI2S.PLLI2SQ = 2;
    rcc_ex_clk_init_struct.PLLI2SDivQ = 19;

    HAL_RCCEx_PeriphCLKConfig(&rcc_ex_clk_init_struct);

  }
  else /* AUDIO_FREQUENCY_8K, AUDIO_FREQUENCY_16K, AUDIO_FREQUENCY_48K), AUDIO_FREQUENCY_96K */
  {
    /* SAI clock config
    PLLSAI_VCO: VCO_344M
    I2S_CLK(first level) = PLLI2S_VCO/PLLI2SQ = 344/7 = 49.142 Mhz
    I2S_CLK_x = SAI_CLK(first level)/PLLI2SDIVQ = 49.142/1 = 49.142 Mhz */
    rcc_ex_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_SAI_PLLI2S;
    rcc_ex_clk_init_struct.PLLI2S.PLLI2SN = 344;
    rcc_ex_clk_init_struct.PLLI2S.PLLI2SQ = 7;
    rcc_ex_clk_init_struct.PLLI2SDivQ = 2;

    HAL_RCCEx_PeriphCLKConfig(&rcc_ex_clk_init_struct);
  }
}






void  HAL_SAI_MspInit(SAI_HandleTypeDef *hsai)
{
  static DMA_HandleTypeDef hdma_sai_tx;
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Put CS43L2 codec reset high -----------------------------------*/
  __HAL_RCC_GPIOE_CLK_ENABLE();

  GPIO_InitStruct.Pin =  GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);

  /* Enable SAI clock */
  __HAL_RCC_SAI1_CLK_ENABLE();

  /* Enable GPIO clock */
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /* CODEC_SAI pins configuration: MCK pin -----------------------------------*/
  GPIO_InitStruct.Pin =  GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* CODEC_SAI pins configuration: FS, SCK, MCK and SD pins ------------------*/
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* Enable the DMA clock */
  __HAL_RCC_DMA2_CLK_ENABLE();

  if(hsai->Instance == SAI1_Block_A)
  {
    /* Configure the hdma_saiTx handle parameters */
    hdma_sai_tx.Init.Channel             = DMA_CHANNEL_0;
    hdma_sai_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_sai_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_sai_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_sai_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_sai_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_sai_tx.Init.Mode                = DMA_NORMAL; // DMA_CIRCULAR;
    hdma_sai_tx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_sai_tx.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    hdma_sai_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_sai_tx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_sai_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;

    hdma_sai_tx.Instance = DMA2_Stream3;

    /* Associate the DMA handle */
    __HAL_LINKDMA(hsai, hdmatx, hdma_sai_tx);

    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hdma_sai_tx);

    /* Configure the DMA Stream */
    HAL_DMA_Init(&hdma_sai_tx);
  }

  /* SAI DMA IRQ Channel configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}


void  HAL_SAI_MspDeInit(SAI_HandleTypeDef *hsai)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* SAI DMA IRQ Channel deactivation */
  HAL_NVIC_DisableIRQ(DMA2_Stream3_IRQn);

  if(hsai->Instance == SAI1_Block_A)
  {
    /* Deinitialize the DMA stream */
    HAL_DMA_DeInit(hsai->hdmatx);
  }

  /* Disable SAI peripheral */
  __HAL_SAI_DISABLE(hsai);

  /* Put CS43L2 codec reset low -----------------------------------*/
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);

  /* Deactives CODEC_SAI pins FS, SCK, MCK and SD by putting them in input mode */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  HAL_GPIO_DeInit(GPIOG, GPIO_InitStruct.Pin);

  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
  HAL_GPIO_DeInit(GPIOE, GPIO_InitStruct.Pin);

  GPIO_InitStruct.Pin = GPIO_PIN_2;
  HAL_GPIO_DeInit(GPIOE, GPIO_InitStruct.Pin);


  /* Disable SAI clock */
  __HAL_RCC_SAI1_CLK_DISABLE();
}


void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s)
{
  static DMA_HandleTypeDef hdma_i2s_rx;
  GPIO_InitTypeDef  gpio_init_structure;

  /* Configure the Timer which clocks the MEMS */
  /* Moved inside MSP to allow applic to redefine the TIMx_MspInit */
  TIMx_Init();

  /* Enable I2S clock */
  __HAL_RCC_SPI3_CLK_ENABLE();

  /* Enable SCK and SD GPIO clock */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /* CODEC_I2S pins configuration: SCK and SD pins */
  gpio_init_structure.Pin = GPIO_PIN_3;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FAST;
  gpio_init_structure.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOB, &gpio_init_structure);

  gpio_init_structure.Pin = GPIO_PIN_6;
  gpio_init_structure.Alternate = GPIO_AF5_I2S3ext;
  HAL_GPIO_Init(GPIOD, &gpio_init_structure);

  /* Enable PD12 (I2S3_CLK) connected to PB3 via jamper JP4 */
  /* on Eval this was provided by PC6 (initialized in TIMx section) */
/*
  gpio_init_structure.Pin = GPIO_PIN_12;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FAST;
  gpio_init_structure.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOD, &gpio_init_structure); */


  /* Enable the DMA clock */
  __HAL_RCC_DMA1_CLK_ENABLE();

  if(hi2s->Instance == SPI3)
  {
    /* Configure the hdma_i2sRx handle parameters */
    hdma_i2s_rx.Init.Channel             = DMA_CHANNEL_0;
    hdma_i2s_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_i2s_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_i2s_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_i2s_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_i2s_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_i2s_rx.Init.Mode                = DMA_CIRCULAR;
    hdma_i2s_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_i2s_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_i2s_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_i2s_rx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_i2s_rx.Init.PeriphBurst         = DMA_MBURST_SINGLE;

    hdma_i2s_rx.Instance = DMA1_Stream2;

    /* Associate the DMA handle */
    __HAL_LINKDMA(hi2s, hdmarx, hdma_i2s_rx);

    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hdma_i2s_rx);

    /* Configure the DMA Stream */
    HAL_DMA_Init(&hdma_i2s_rx);
  }

  /* I2S DMA IRQ Channel configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
}

void HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s)
{
  GPIO_InitTypeDef  gpio_init_structure;

  /* I2S DMA IRQ Channel deactivation */
  HAL_NVIC_DisableIRQ(DMA1_Stream2_IRQn);

  if(hi2s->Instance == SPI3)
  {
    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(hi2s->hdmarx);
  }

 /* Disable I2S block */
  __HAL_I2S_DISABLE(hi2s);

  /* Disable pins: SCK and SD pins */
  gpio_init_structure.Pin = GPIO_PIN_3;
  HAL_GPIO_DeInit(GPIOB, gpio_init_structure.Pin);
  gpio_init_structure.Pin = GPIO_PIN_6;
  HAL_GPIO_DeInit(GPIOD, gpio_init_structure.Pin);

  /* Disable I2S clock */
__HAL_RCC_SPI3_CLK_DISABLE();
}





/**
  * @brief  Configure TIM as a clock divider by 2.
  *         I2S_SCK is externally connected to TIMx input channel
  */
static void TIMx_Init(void)
{
  TIM_IC_InitTypeDef     s_ic_config;
  TIM_OC_InitTypeDef     s_oc_config;
  TIM_ClockConfigTypeDef s_clk_source_config;
  TIM_SlaveConfigTypeDef s_slave_config;

  /* Configure the TIM peripheral --------------------------------------------*/
  /* Set TIMx instance */
  haudio_tim.Instance = TIM4;
  /* Timer Input Capture Configuration Structure declaration */
   /* Initialize TIMx peripheral as follow:
       + Period = 0xFFFF
       + Prescaler = 0
       + ClockDivision = 0
       + Counter direction = Up
  */
  haudio_tim.Init.Period        = 1;
  haudio_tim.Init.Prescaler     = 0;
  haudio_tim.Init.ClockDivision = 0;
  haudio_tim.Init.CounterMode   = TIM_COUNTERMODE_UP;

  /* Initialize the TIMx peripheral with the structure above */
  TIMx_IC_MspInit(&haudio_tim);
  HAL_TIM_IC_Init(&haudio_tim);

  /* Configure the Input Capture channel -------------------------------------*/
  /* Configure the Input Capture of channel 2 */
  s_ic_config.ICPolarity  = TIM_ICPOLARITY_FALLING;
  s_ic_config.ICSelection = TIM_ICSELECTION_DIRECTTI;
  s_ic_config.ICPrescaler = TIM_ICPSC_DIV1;
  s_ic_config.ICFilter    = 0;
  HAL_TIM_IC_ConfigChannel(&haudio_tim, &s_ic_config, TIM_CHANNEL_1);

  /* Select external clock mode 1 */
  s_clk_source_config.ClockSource = TIM_CLOCKSOURCE_ETRMODE1;
  s_clk_source_config.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  s_clk_source_config.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  s_clk_source_config.ClockFilter = 0;
  HAL_TIM_ConfigClockSource(&haudio_tim, &s_clk_source_config);

  /* Select Input Channel as input trigger */
  s_slave_config.InputTrigger = TIM_TS_TI1FP1;
  s_slave_config.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  s_slave_config.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
  s_slave_config.TriggerPrescaler = TIM_CLOCKPRESCALER_DIV1;
  s_slave_config.TriggerFilter = 0;
  HAL_TIM_SlaveConfigSynchronization(&haudio_tim, &s_slave_config);

  /* Output Compare PWM Mode configuration: Channel2 */
  s_oc_config.OCMode = TIM_OCMODE_PWM1;
  s_oc_config.OCIdleState = TIM_OCIDLESTATE_SET;
  s_oc_config.Pulse = 1;
  s_oc_config.OCPolarity = TIM_OCPOLARITY_HIGH;
  s_oc_config.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  s_oc_config.OCFastMode = TIM_OCFAST_DISABLE;
  s_oc_config.OCNIdleState = TIM_OCNIDLESTATE_SET;

  /* Initialize the TIM3 Channel2 with the structure above */
  HAL_TIM_PWM_ConfigChannel(&haudio_tim, &s_oc_config, TIM_CHANNEL_2);

  /* Start the TIM3 Channel2 */
  HAL_TIM_PWM_Start(&haudio_tim, TIM_CHANNEL_2);

  /* Start the TIM3 Channel1 */
  HAL_TIM_IC_Start(&haudio_tim, TIM_CHANNEL_1);
}

/**
  * @brief  Configure TIM as a clock divider by 2.
  *         I2S_SCK is externally connected to TIMx input channel
  */
static void TIMx_DeInit(void)
{
  haudio_tim.Instance = TIM4;

  /* Stop the TIM3 Channel2 */
  HAL_TIM_PWM_Stop(&haudio_tim, TIM_CHANNEL_2);
  /* Stop the TIM3 Channel1 */
  HAL_TIM_IC_Stop(&haudio_tim, TIM_CHANNEL_1);

  HAL_TIM_IC_DeInit(&haudio_tim);

  /* Initialize the TIMx peripheral with the structure above */
  TIMx_IC_MspDeInit(&haudio_tim);
}


/**
  * @brief  Initializes the TIM INput Capture MSP.
  * @param  htim: TIM handle
  */
static void TIMx_IC_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   gpio_init_structure;

  /* Enable peripherals and GPIO Clocks --------------------------------------*/
  /* TIMx Peripheral clock enable */
  __HAL_RCC_TIM4_CLK_ENABLE();

  /* Enable GPIO Channels Clock */
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Configure I/Os ----------------------------------------------------------*/
  /* Common configuration for all channels */
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  gpio_init_structure.Alternate = GPIO_AF2_TIM4;

  /* Configure TIM input channel */
  gpio_init_structure.Pin = GPIO_PIN_12;
  HAL_GPIO_Init(GPIOD, &gpio_init_structure);

  /* Configure TIM output channel */
  gpio_init_structure.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOD, &gpio_init_structure);
}

/**
  * @brief  Initializes the TIM INput Capture MSP.
  * @param  htim: TIM handle
  */
static void TIMx_IC_MspDeInit(TIM_HandleTypeDef *htim)
{
    /* Disable TIMx Peripheral clock  */
    __HAL_RCC_TIM4_CLK_DISABLE();

    /* GPIO pins clock and DMA clock can be shut down in the applic
       by surcgarging this __weak function */
}








/*******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/

/********************************* LINK AUDIO *********************************/

static I2C_HandleTypeDef hi2c;

static void AUDIO_I2C_MspInit(void);

void AUDIO_IO_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  if(HAL_I2C_GetState(&hi2c) == HAL_I2C_STATE_RESET)
  {
    hi2c.Instance = I2C2;
    hi2c.Init.ClockSpeed      = 100000;
    hi2c.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c.Init.OwnAddress1     = 0;
    hi2c.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c.Init.OwnAddress2     = 0;
    hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    AUDIO_I2C_MspInit();
    HAL_I2C_Init(&hi2c);
  }

  /* Initialize SPKR/HP select pin */
  GPIO_InitStruct.Pin =  GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void AUDIO_IO_DeInit(void)
{

}

void AUDIO_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  HAL_I2C_Mem_Write(&hi2c, (uint16_t)Addr, (uint16_t) Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&Value, 1, 1000);
}

uint8_t AUDIO_IO_Read(uint8_t Addr, uint8_t Reg)
{
  uint8_t read_value = 0;

  HAL_I2C_Mem_Read(&hi2c, (uint16_t)Addr, (uint16_t) Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&read_value, 1, 1000);

  return read_value;
}

void AUDIO_IO_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}



static void AUDIO_I2C_MspInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /*** Configure the GPIOs ***/
  /* Enable GPIO clock */
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* Configure I2C Tx as alternate function */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* Configure I2C Rx as alternate function */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*** Configure the I2C peripheral ***/
  /* Enable I2C clock */
  __HAL_RCC_I2C2_CLK_ENABLE();

  /* Force the I2C peripheral clock reset */
  __HAL_RCC_I2C2_FORCE_RESET();

  /* Release the I2C peripheral clock reset */
  __HAL_RCC_I2C2_RELEASE_RESET();

  /* Enable and set I2C1 Interrupt to a lower priority */
  HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0x05, 0);
  HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);

  /* Enable and set I2C1 Interrupt to a lower priority */
  HAL_NVIC_SetPriority(I2C2_ER_IRQn, 0x05, 0);
  HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
}
