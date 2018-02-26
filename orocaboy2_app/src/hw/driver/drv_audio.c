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

#define AUDIODATA_SIZE                      2   /* 16-bits audio data size */

#define DEFAULT_OUTPUT_DEVICE   OUTPUT_DEVICE_AUTO
#define DEFAULT_OUTPUT_VOLUME   70

AUDIO_DrvTypeDef          *audio_drv;
SAI_HandleTypeDef         haudio_out_sai;
TIM_HandleTypeDef         haudio_tim;

static void drvAudioOutClockConfig(SAI_HandleTypeDef *hsai, uint32_t audio_freq, void *Params);

err_code_t drvAudioOutInit(uint32_t audio_freq)
{
  err_code_t ret = OK;
  uint16_t output_device = DEFAULT_OUTPUT_DEVICE;
  uint8_t volume = DEFAULT_OUTPUT_VOLUME;

  HAL_SAI_DeInit(&haudio_out_sai);

  drvAudioOutClockConfig(&haudio_out_sai, audio_freq, NULL);

  haudio_out_sai.Instance = SAI1_Block_A;
  haudio_out_sai.Init.AudioFrequency = audio_freq;
  haudio_out_sai.Init.ClockSource = SAI_CLKSOURCE_PLLI2S;
  haudio_out_sai.Init.AudioMode = SAI_MODEMASTER_TX;
  haudio_out_sai.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  haudio_out_sai.Init.Protocol = SAI_FREE_PROTOCOL;
  haudio_out_sai.Init.DataSize = SAI_DATASIZE_16;
  haudio_out_sai.Init.FirstBit = SAI_FIRSTBIT_MSB;
  haudio_out_sai.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  haudio_out_sai.Init.Synchro = SAI_ASYNCHRONOUS;
  haudio_out_sai.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;
  haudio_out_sai.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
  haudio_out_sai.Init.MonoStereoMode = SAI_MONOMODE;

  haudio_out_sai.FrameInit.FrameLength = 64;
  haudio_out_sai.FrameInit.ActiveFrameLength = 32;
  haudio_out_sai.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  haudio_out_sai.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  haudio_out_sai.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;

  haudio_out_sai.SlotInit.FirstBitOffset = 0;
  haudio_out_sai.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  haudio_out_sai.SlotInit.SlotNumber = 4;
  haudio_out_sai.SlotInit.SlotActive = CODEC_AUDIOFRAME_SLOT_0123;

  if (HAL_SAI_Init(&haudio_out_sai) != HAL_OK)
  {
    ret = ERR_AUDIO;
  }

  __HAL_SAI_ENABLE(&haudio_out_sai);

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

//TODO : Add beep functions.
err_code_t drvAudioOutBeep(uint32_t freq, uint32_t duration)
{

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

  /* Call the Audio Codec Pause/Resume function */
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
  /* Manage the remaining file size and new address offset: This function
     should be coded by user (its prototype is already declared in stm32469i_discovery_audio.h) */
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  /* Manage the remaining file size and new address offset: This function
     should be coded by user (its prototype is already declared in stm32469i_discovery_audio.h) */
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{

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
    rcc_ex_clk_init_struct.PLLI2SDivQ = 1;

    HAL_RCCEx_PeriphCLKConfig(&rcc_ex_clk_init_struct);
  }
}






void  HAL_SAI_MspInit(SAI_HandleTypeDef *hsai)
{
  static DMA_HandleTypeDef hdma_sai_tx;
  GPIO_InitTypeDef  gpio_init_structure;

  /* Put CS43L2 codec reset high -----------------------------------*/
  __HAL_RCC_GPIOE_CLK_ENABLE();

  gpio_init_structure.Pin =  GPIO_PIN_2;
  gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOE, &gpio_init_structure);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);

  /* Enable SAI clock */
  __HAL_RCC_SAI1_CLK_ENABLE();

  /* Enable GPIO clock */
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /* CODEC_SAI pins configuration: MCK pin -----------------------------------*/
  gpio_init_structure.Pin =  GPIO_PIN_7;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  gpio_init_structure.Alternate = GPIO_AF6_SAI1;
  HAL_GPIO_Init(GPIOG, &gpio_init_structure);

  /* CODEC_SAI pins configuration: FS, SCK, MCK and SD pins ------------------*/
  gpio_init_structure.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  gpio_init_structure.Alternate = GPIO_AF6_SAI1;
  HAL_GPIO_Init(GPIOE, &gpio_init_structure);

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
    hdma_sai_tx.Init.Mode                = DMA_CIRCULAR;
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
  GPIO_InitTypeDef  gpio_init_structure;

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
  gpio_init_structure.Pin = GPIO_PIN_7;
  HAL_GPIO_DeInit(GPIOG, gpio_init_structure.Pin);

  gpio_init_structure.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
  HAL_GPIO_DeInit(GPIOE, gpio_init_structure.Pin);

  gpio_init_structure.Pin = GPIO_PIN_2;
  HAL_GPIO_DeInit(GPIOE, gpio_init_structure.Pin);


  /* Disable SAI clock */
  __HAL_RCC_SAI1_CLK_DISABLE();
}




/*******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/

/********************************* LINK AUDIO *********************************/

static I2C_HandleTypeDef hi2c;

static void AUDIO_I2C_MspInit(void);

void AUDIO_IO_Init(void)
{
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

