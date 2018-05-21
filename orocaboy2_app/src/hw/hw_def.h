/*
 *  hw_def.h
 *
 *  Created on: Feb 08, 2018
 *      Author: opus
 */





#ifndef HW_DEF_H
#define HW_DEF_H

#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>

#include "def.h"
#include "error_code.h"


#ifndef BOOL
#define BOOL uint8_t
#endif

#ifndef TRUE
#define TRUE  1
#endif

#ifndef FALSE
#define FALSE 0
#endif


#define _USE_HW_RTOS
#define _USE_HW_VCP
#define _USE_HW_LED
#define _USE_HW_BUTTON
#define _USE_HW_UART
#define _USE_HW_EEPROM
#define _USE_HW_FLASH
#define _USE_HW_RESET
#define _USE_HW_TIMER
#define _USE_HW_SW_TIMER
//#define _USE_HW_I2C
#define _USE_HW_ADC
#define _USE_HW_GPIO
#define _USE_HW_CMDIF
#define _USE_HW_AUDIO
#define _USE_HW_SDRAM
#define _USE_HW_LCD
#define _USE_HW_TS
#define _USE_HW_SD
#define _USE_HW_FATFS
#define _USE_HW_EXTI
#define _USE_HW_DAC
#define _USE_HW_CMDIF_EXTI


#define _HW_DEF_CMDIF_LIST_MAX  16


#define _USE_HW_CMDIF_LED
#define _USE_HW_CMDIF_BUTTON
#define _USE_HW_CMDIF_EEPROM
#define _USE_HW_CMDIF_FLASH
#define _USE_HW_CMDIF_SDRAM
#define _USE_HW_CMDIF_LCD
#define _USE_HW_CMDIF_TS
#define _USE_HW_CMDIF_SD
#define _USE_HW_CMDIF_FATFS
#define _USE_HW_CMDIF_AUDIO
#define _USE_HW_CMDIF_ADC
#define _USE_HW_CMDIF_DAC


#define _HW_DEF_LED_CH_MAX                4
#define _HW_DEF_BUTTON_CH_MAX             7
#define _HW_DEF_UART_CH_MAX               2
#define _HW_DEF_TIMER_CH_MAX              4
#define _HW_DEF_SW_TIMER_MAX              8
#define _HW_DEF_ADC_CH_MAX                2
#define _HW_DEF_I2C_CH_MAX                1
#define _HW_DEF_EXTI_CH_MAX               1

#define _HW_DEF_RTOS_MEM_SIZE(x)              ((x)/4)


#define _HW_DEF_TIMER_RESERVED1           _DEF_TIMER1
#define _HW_DEF_TIMER_SWTIMER             _DEF_TIMER2
#define _HW_DEF_TIMER_ADC                 _DEF_TIMER3



#define _HW_DEF_SDRAM_ADDR_START              0xC0000000
#define _HW_DEF_SDRAM_ADDR_LENGTH             0x01000000
#define _HW_DEF_SDRAM_ADDR_END                (_HW_DEF_SDRAM_ADDR_START + _HW_DEF_SDRAM_ADDR_LENGTH)

#define _HW_DEF_LCD_ADDR_LAYER1_START         _HW_DEF_SDRAM_ADDR_START
#define _HW_DEF_LCD_ADDR_LAYER2_START         (_HW_DEF_SDRAM_ADDR_START + (800*480*4))

#define _HW_DEF_SDRAM_HEAP_START              (_HW_DEF_SDRAM_ADDR_START + 4*1024*1024)
#define _HW_DEF_SDRAM_HEAP_LENGTH             (11*1024*1024)


#define _HW_DEF_FLASH_ADDR_GAME_LENGTH        (512*1024)
#define _HW_DEF_FLASH_ADDR_GAME_START         0x08180000
#define _HW_DEF_FLASH_ADDR_GAME_END           (_HW_DEF_FLASH_ADDR_GAME_START + _HW_DEF_FLASH_ADDR_GAME_LENGTH)

#define _HW_DEF_CMD_MAX_DATA_LENGTH           1024

#define _HW_DEF_GAME_API_ADDR                 0x2004FC00


#define _HW_DEF_ADC_X_AXIS                1
#define _HW_DEF_ADC_Y_AXIS                0

#define _HW_DEF_BUTTON_A                  1
#define _HW_DEF_BUTTON_B                  2
#define _HW_DEF_BUTTON_MENU               3
#define _HW_DEF_BUTTON_HOME               4
#define _HW_DEF_BUTTON_C                  5
#define _HW_DEF_BUTTON_D                  6


#endif

