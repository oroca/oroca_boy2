/*
 * drv_sdram.c
 *
 *  Created on: Feb 10, 2018
 *      Author: opus
 */


#include "drv_sdram.h"


#include "hw.h"


static bool is_init = false;


static SDRAM_HandleTypeDef hsdram;
static FMC_SDRAM_TimingTypeDef timing;
static FMC_SDRAM_CommandTypeDef cmd;

void drvSdramInitSequence(uint32_t refresh_cnt);
void drvSdramMspInit(SDRAM_HandleTypeDef  *hsdram, void *Params);
void drvSdramMspDeinit(SDRAM_HandleTypeDef  *hsdram, void *Params);

/**
  * @brief  Initializes the SDRAM device.
  * @retval SDRAM status
  */
err_code_t drvSdramInit(void)
{
  static err_code_t sdramstatus = ERR_SDRAM;

  if (is_init == true) return OK;
  is_init = true;

  /* SDRAM device configuration */
  hsdram.Instance = FMC_SDRAM_DEVICE;

  /* timing configuration for 90 MHz as SD clock frequency (System clock is up to 180 MHz) */
  timing.LoadToActiveDelay    = 2;
  timing.ExitSelfRefreshDelay = 7;
  timing.SelfRefreshTime      = 4;
  timing.RowCycleDelay        = 7;
  timing.WriteRecoveryTime    = 2;
  timing.RPDelay              = 2;
  timing.RCDDelay             = 2;

  hsdram.Init.SDBank             = FMC_SDRAM_BANK1;
  hsdram.Init.ColumnBitsNumber   = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram.Init.RowBitsNumber      = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram.Init.MemoryDataWidth    = FMC_SDRAM_MEM_BUS_WIDTH_32;
  hsdram.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram.Init.CASLatency         = FMC_SDRAM_CAS_LATENCY_3;
  hsdram.Init.WriteProtection    = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram.Init.SDClockPeriod      = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram.Init.ReadBurst          = FMC_SDRAM_RBURST_ENABLE;
  hsdram.Init.ReadPipeDelay      = FMC_SDRAM_RPIPE_DELAY_0;

  /* SDRAM controller initialization */
  /* __weak function can be surcharged by the application code */
  drvSdramMspInit(&hsdram, (void *)NULL);
  if(HAL_SDRAM_Init(&hsdram, &timing) != HAL_OK)
  {
    sdramstatus = ERR_SDRAM;
  }
  else
  {
    sdramstatus = OK;
  }

  /* SDRAM initialization sequence */
  drvSdramInitSequence(DRV_SDRAM_REFRESH_COUNT);

  return sdramstatus;
}

/**
  * @brief  DeInitializes the SDRAM device.
  * @retval SDRAM status : OK or ERR_SDRAM.
  */
err_code_t drvSdramDeinit(void)
{
  static err_code_t sdramstatus = ERR_SDRAM;

  /* SDRAM device configuration */
  hsdram.Instance = FMC_SDRAM_DEVICE;

  if(HAL_SDRAM_DeInit(&hsdram) == HAL_OK)
  {
    sdramstatus = OK;

  /* SDRAM controller De-initialization */
   drvSdramMspDeinit(&hsdram, (void *)NULL);
  }

  return sdramstatus;
}


/**
  * @brief  Programs the SDRAM device.
  * @param  refresh_cnt: SDRAM refresh counter value
  */
void drvSdramInitSequence(uint32_t refresh_cnt)
{
  __IO uint32_t tmpmrd = 0;

  /* Step 1: Configure a clock configuration enable command */
  cmd.CommandMode            = FMC_SDRAM_CMD_CLK_ENABLE;
  cmd.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
  cmd.AutoRefreshNumber      = 1;
  cmd.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(&hsdram, &cmd, DRV_SDRAM_TIMEOUT);

  /* Step 2: Insert 100 us minimum delay */
  /* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
  HAL_Delay(1);

  /* Step 3: Configure a PALL (precharge all) command */
  cmd.CommandMode            = FMC_SDRAM_CMD_PALL;
  cmd.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
  cmd.AutoRefreshNumber      = 1;
  cmd.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(&hsdram, &cmd, DRV_SDRAM_TIMEOUT);

  /* Step 4: Configure an Auto Refresh command */
  cmd.CommandMode            = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  cmd.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
  cmd.AutoRefreshNumber      = 8;
  cmd.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(&hsdram, &cmd, DRV_SDRAM_TIMEOUT);

  /* Step 5: Program the external memory mode register */
  tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_1          |\
                     SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |\
                     SDRAM_MODEREG_CAS_LATENCY_3           |\
                     SDRAM_MODEREG_OPERATING_MODE_STANDARD |\
                     SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

  cmd.CommandMode            = FMC_SDRAM_CMD_LOAD_MODE;
  cmd.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
  cmd.AutoRefreshNumber      = 1;
  cmd.ModeRegisterDefinition = tmpmrd;

  /* Send the command */
  HAL_SDRAM_SendCommand(&hsdram, &cmd, DRV_SDRAM_TIMEOUT);

  /* Step 6: Set the refresh rate counter */
  /* Set the device refresh rate */
  HAL_SDRAM_ProgramRefreshRate(&hsdram, refresh_cnt);
}

/**
  * @brief  Reads an mount of data from the SDRAM memory in DMA mode.
  * @param  uwStartAddress: Read start address
  * @param  pData: Pointer to data to be read
  * @param  uwDataSize: Size of read data from the memory
  * @retval SDRAM status : OK or ERR_SDRAM.
  */
err_code_t drvSdramReadData(uint32_t uwStartAddress, uint32_t *pData, uint32_t uwDataSize)
{
  if(HAL_SDRAM_Read_DMA(&hsdram, (uint32_t *)uwStartAddress, pData, uwDataSize) != HAL_OK)
  {
    return ERR_SDRAM;
  }
  else
  {
    return OK;
  }
}

/**
  * @brief  Writes an mount of data to the SDRAM memory in DMA mode.
  * @param  uwStartAddress: Write start address
  * @param  pData: Pointer to data to be written
  * @param  uwDataSize: Size of written data from the memory
  * @retval SDRAM status : OK or ERR_SDRAM.
  */
err_code_t drvSdramWriteData(uint32_t uwStartAddress, uint32_t *pData, uint32_t uwDataSize)
{
  if(HAL_SDRAM_Write_DMA(&hsdram, (uint32_t *)uwStartAddress, pData, uwDataSize) != HAL_OK)
  {
    return ERR_SDRAM;
  }
  else
  {
    return OK;
  }
}



/**
  * @brief  Writes an mount of data to the SDRAM memory in DMA mode.
  * @param  uwStartAddress: Write start address
  * @param  pData: Pointer to data to be written
  * @param  uwDataSize: Size of written data from the memory
  * @param  polling_timeout : Timeout duration(ms) for waiting transfer complete
  * @retval SDRAM status : OK or ERR_SDRAM.
  */
err_code_t drvSdramWriteDataUntilTimeout(uint32_t uwStartAddress, uint32_t *pData, uint32_t uwDataSize, uint32_t polling_timeout)
{
  if(HAL_SDRAM_Write_DMA(&hsdram, (uint32_t *)uwStartAddress, pData, uwDataSize) != HAL_OK)
  {
    return ERR_SDRAM;
  }
  else
  {
    HAL_DMA_PollForTransfer(hsdram.hdma, HAL_DMA_FULL_TRANSFER, polling_timeout);
    return OK;
  }
}


/**
  * @brief  Handles SDRAM DMA transfer interrupt request.
  */
void DMA2_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hsdram.hdma);
}


/**
  * @brief  Initializes SDRAM MSP.
  * @note   This function can be surcharged by application code.
  * @param  hsdram: pointer on SDRAM handle
  * @param  Params: pointer on additional configuration parameters, can be NULL.
  */
void drvSdramMspInit(SDRAM_HandleTypeDef  *hsdram, void *Params)
{
  static DMA_HandleTypeDef dma_handle;
  GPIO_InitTypeDef GPIO_InitStruct;

  if(hsdram != (SDRAM_HandleTypeDef  *)NULL)
  {
    /* Enable FMC clock */
    __HAL_RCC_FMC_CLK_ENABLE();

    /* Enable chosen DMAx clock */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* Enable GPIOs clock */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();

    /* Common GPIO configuration */
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;

    /* GPIOC configuration : PC0 is SDNWE */
    GPIO_InitStruct.Pin   = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* GPIOD configuration */
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8| GPIO_PIN_9 | GPIO_PIN_10 |\
                                GPIO_PIN_14 | GPIO_PIN_15;


    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* GPIOE configuration */
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_7| GPIO_PIN_8 | GPIO_PIN_9 |\
                                GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |\
                                GPIO_PIN_15;

    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* GPIOF configuration */
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2| GPIO_PIN_3 | GPIO_PIN_4 |\
                                GPIO_PIN_5 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |\
                                GPIO_PIN_15;

    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* GPIOG configuration */
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4| GPIO_PIN_5 | GPIO_PIN_8 |\
                                GPIO_PIN_15;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* GPIOH configuration */
    GPIO_InitStruct.Pin   = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_8 | GPIO_PIN_9 |\
                                GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |\
                                GPIO_PIN_15;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    /* GPIOI configuration */
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |\
                                GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    /* Configure common DMA parameters */
    dma_handle.Init.Channel             = DMA_CHANNEL_0;
    dma_handle.Init.Direction           = DMA_MEMORY_TO_MEMORY;
    dma_handle.Init.PeriphInc           = DMA_PINC_ENABLE;
    dma_handle.Init.MemInc              = DMA_MINC_ENABLE;
    dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    dma_handle.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    dma_handle.Init.Mode                = DMA_NORMAL;
    dma_handle.Init.Priority            = DMA_PRIORITY_HIGH;
    dma_handle.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    dma_handle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    dma_handle.Init.MemBurst            = DMA_MBURST_SINGLE;
    dma_handle.Init.PeriphBurst         = DMA_PBURST_SINGLE;

    dma_handle.Instance = DMA2_Stream0;

    /* Associate the DMA handle */
    __HAL_LINKDMA(hsdram, hdma, dma_handle);

    /* Deinitialize the stream for new transfer */
    HAL_DMA_DeInit(&dma_handle);

    /* Configure the DMA stream */
    HAL_DMA_Init(&dma_handle);

    /* NVIC configuration for DMA transfer complete interrupt */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

  } /* of if(hsdram != (SDRAM_HandleTypeDef  *)NULL) */
}

/**
  * @brief  DeInitializes SDRAM MSP.
  * @note   This function can be surcharged by application code.
  * @param  hsdram: pointer on SDRAM handle
  * @param  Params: pointer on additional configuration parameters, can be NULL.
  */
void drvSdramMspDeinit(SDRAM_HandleTypeDef  *hsdram, void *Params)
{
    static DMA_HandleTypeDef dma_handle;

    if(hsdram != (SDRAM_HandleTypeDef  *)NULL)
    {
      /* Disable NVIC configuration for DMA interrupt */
      HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);

      /* Deinitialize the stream for new transfer */
      dma_handle.Instance = DMA2_Stream0;
      HAL_DMA_DeInit(&dma_handle);

      /* DeInit GPIO pins can be done in the application
       (by surcharging this __weak function) */

      /* GPIO pins clock, FMC clock and DMA clock can be shut down in the application
       by surcharging this __weak function */

    } /* of if(hsdram != (SDRAM_HandleTypeDef  *)NULL) */
}


