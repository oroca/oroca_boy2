/*
 * drv_sd.c
 *
 *  Created on: 2018. 2. 11.
 *      Author: Baram
 */



#include <stdarg.h>
#include <stdbool.h>

#include "drv_sd.h"
#include "hw.h"




//-- Internal Variables
//
SD_HandleTypeDef uSdHandle;
bool   is_init = false;
static err_code_t err_code;


//-- External Variables
//


//-- Internal Functions
//
void drvSdInitDetectedPin(void);
void drvSdInitHw(void);


//-- External Functions
//







bool drvSdInit(void)
{
  bool ret = true;


  err_code = OK;

  /* uSD device interface configuration */
  uSdHandle.Instance = SDIO;

  uSdHandle.Init.ClockEdge           = SDIO_CLOCK_EDGE_RISING;
  uSdHandle.Init.ClockBypass         = SDIO_CLOCK_BYPASS_DISABLE;
  uSdHandle.Init.ClockPowerSave      = SDIO_CLOCK_POWER_SAVE_DISABLE;
  uSdHandle.Init.BusWide             = SDIO_BUS_WIDE_1B;
  uSdHandle.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_ENABLE;
  uSdHandle.Init.ClockDiv            = SDIO_TRANSFER_CLK_DIV;


  drvSdInitDetectedPin();


  if (drvSdIsDetected() == false)
  {
    //err_code = ??
    return false;
  }

  drvSdInitHw();


  if(HAL_SD_Init(&uSdHandle) != HAL_OK)
  {
    return false;
  }

  /* Enable wide operation */
  if(HAL_SD_ConfigWideBusOperation(&uSdHandle, SDIO_BUS_WIDE_4B) != HAL_OK)
  {
    ret = false;
  }

  is_init = ret;

  return ret;
}

bool drvSdDeInit(void)
{
  /* Disable NVIC for SDIO interrupts */
  HAL_NVIC_DisableIRQ(SDIO_IRQn);

  /* DeInit GPIO pins can be done in the application
     (by surcharging this __weak function) */

  /* Disable SDIO clock */
  __HAL_RCC_SDIO_CLK_DISABLE();

  return true;
}

void drvSdInitDetectedPin(void)
{
  GPIO_InitTypeDef  gpio_init_structure;

  __HAL_RCC_GPIOG_CLK_ENABLE();

  /* GPIO configuration in input for uSD_Detect signal */
  gpio_init_structure.Pin       = GPIO_PIN_2;
  gpio_init_structure.Mode      = GPIO_MODE_INPUT;
  gpio_init_structure.Pull      = GPIO_PULLUP;
  gpio_init_structure.Speed     = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOG, &gpio_init_structure);
}


void drvSdInitHw(void)
{
  GPIO_InitTypeDef gpio_init_structure;
  SD_HandleTypeDef *hsd = &uSdHandle;


  /* Enable SDIO clock */
  __HAL_RCC_SDIO_CLK_ENABLE();

  /* Enable GPIOs clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Common GPIO configuration */
  gpio_init_structure.Mode      = GPIO_MODE_AF_PP;
  gpio_init_structure.Pull      = GPIO_PULLUP;
  gpio_init_structure.Speed     = GPIO_SPEED_HIGH;
  gpio_init_structure.Alternate = GPIO_AF12_SDIO;

  /* GPIOC configuration */
  gpio_init_structure.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;

  HAL_GPIO_Init(GPIOC, &gpio_init_structure);

  /* GPIOD configuration */
  gpio_init_structure.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOD, &gpio_init_structure);

  /* NVIC configuration for SDIO interrupts */
  HAL_NVIC_SetPriority(SDIO_IRQn, 0x0E, 0);
  HAL_NVIC_EnableIRQ(SDIO_IRQn);
}


bool drvSdReadBlocks(uint32_t block_addr, uint8_t *p_data, uint32_t num_of_blocks, uint32_t timeout_ms)
{
  bool ret = false;

  if(HAL_SD_ReadBlocks(&uSdHandle, (uint8_t *)p_data, block_addr, num_of_blocks, timeout_ms) == HAL_OK)
  {
    ret = true;
  }

  return ret;
}

bool drvSdWriteBlocks(uint32_t block_addr, uint8_t *p_data, uint32_t num_of_blocks, uint32_t timeout_ms)
{
  bool ret = false;


  if(HAL_SD_WriteBlocks(&uSdHandle, (uint8_t *)p_data, block_addr, num_of_blocks, timeout_ms) == HAL_OK)
  {
    ret = true;
  }

  return ret;
}

bool drvSdEraseBlocks(uint32_t start_addr, uint32_t end_addr)
{
  bool ret = false;


  if(HAL_SD_Erase(&uSdHandle, start_addr, end_addr) == HAL_OK)
  {
    ret = true;
  }

  return ret;
}

bool drvSdIsBusy(void)
{
  bool is_busy;


  if (HAL_SD_GetCardState(&uSdHandle) == HAL_SD_CARD_TRANSFER )
  {
    is_busy = false;
  }
  else
  {
    is_busy = true;
  }

  return is_busy;
}

bool drvSdIsDetected(void)
{
  bool ret = false;


  if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_2) == GPIO_PIN_RESET)
  {
    ret = true;
  }

  return ret;
}

bool drvSdGetInfo(void *p_info)
{
  bool ret = false;
  sd_info_t *p_sd_info = (sd_info_t *)p_info;

  HAL_SD_CardInfoTypeDef card_info;


  if (is_init == true)
  {
    HAL_SD_GetCardInfo(&uSdHandle, &card_info);

    p_sd_info->card_type          = card_info.CardType;
    p_sd_info->card_version       = card_info.CardVersion;
    p_sd_info->card_class         = card_info.Class;
    p_sd_info->rel_card_Add       = card_info.RelCardAdd;
    p_sd_info->block_numbers      = card_info.BlockNbr;
    p_sd_info->block_size         = card_info.BlockSize;
    p_sd_info->log_block_numbers  = card_info.LogBlockNbr;
    p_sd_info->log_block_size     = card_info.LogBlockSize;
    p_sd_info->card_size          =  (uint32_t)((uint64_t)p_sd_info->block_numbers * (uint64_t)p_sd_info->block_size / (uint64_t)1024 / (uint64_t)1024);
    ret = true;
  }


  return ret;
}
