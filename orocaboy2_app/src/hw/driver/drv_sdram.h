/**
  ******************************************************************************
  * @file    stm32469i_discovery_sdram.h
  * @author  MCD Application Team
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32469i_discovery_sdram.c driver.
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
 * drv_sdram.h
 *
 *  Created on: Feb 10, 2018
 *      Author: opus
 */

#ifndef DRV_SDRAM_H_
#define DRV_SDRAM_H_


#ifdef __cplusplus
 extern "C" {
#endif

#include "hw_def.h"

/**
  * @}
  */

/** @defgroup STM32469I-Discovery_SDRAM_Exported_Constants STM32469I Discovery SDRAM Exported Constants
  * @{
  */
#define SDRAM_DEVICE_ADDR  ((uint32_t)0xC0000000)

/* SDRAM device size in Bytes */
#define SDRAM_DEVICE_SIZE  ((uint32_t)0x1000000)

/* SDRAM refresh counter (90 MHz SD clock) */
#define DRV_SDRAM_REFRESH_COUNT   ((uint32_t)0x0569)
#define DRV_SDRAM_TIMEOUT         ((uint32_t)0xFFFF)

/**
  * @brief  FMC SDRAM Mode definition register defines
  */
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)
/**
  * @}
  */

/** @defgroup STM32469I-Discovery_SDRAM_Exported_Macro STM32469I Discovery SDRAM Exported Macro
  * @{
  */
/**
  * @}
  */

/** @addtogroup STM32469I_Discovery_SDRAM_Exported_Functions
  * @{
  */
err_code_t drvSdramInit(void);
err_code_t drvSdramDeinit(void);
err_code_t drvSdramReadData(uint32_t uwStartAddress, uint32_t *pData, uint32_t uwDataSize);
err_code_t drvSdramWriteData(uint32_t uwStartAddress, uint32_t *pData, uint32_t uwDataSize);
err_code_t drvSdramWriteDataUntilTimeout(uint32_t uwStartAddress, uint32_t *pData, uint32_t uwDataSize, uint32_t polling_timeout);



/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif



#endif /* DRV_SDRAM_H_ */
