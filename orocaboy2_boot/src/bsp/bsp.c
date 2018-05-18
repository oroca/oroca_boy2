/*
 * bsp.c
 *
 *  Created on: Feb 10, 2017
 *      Author: baram
 */



#include "bsp.h"
#include "hw.h"

#include "led.h"


//-- Internal Variables
//
USBD_HandleTypeDef USBD_Device;


//-- External Variables
//


//-- Internal Functions
static void SystemClock_Config(void);



//-- External Functions







void bspInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;


  HAL_DeInit();
  HAL_Init();

  // Clock Setup
  // SYSCLK(Hz)    = 168000000
  // HCLK(Hz)      = 168000000
  // HSE(Hz)       =  8000000
  SystemClock_Config();



  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();


  /* Configure DM(PA11) DP(PA12) Pins */
  GPIO_InitStruct.Pin = (GPIO_PIN_11 | GPIO_PIN_12);
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  //HAL_Delay(300);
}

void bspDeinit(void)
{
  uint32_t i;
  GPIO_InitTypeDef  GPIO_InitStruct;

  USBD_DeInit(&USBD_Device);

  HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
  HAL_NVIC_DisableIRQ(TIM4_IRQn);
  HAL_NVIC_DisableIRQ(TIM2_IRQn);
  HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
  HAL_NVIC_DisableIRQ(DMA2_Stream6_IRQn);
  HAL_NVIC_DisableIRQ(USART6_IRQn);


  // USB_DISCONNECT used as USB pull-up
  //
  /* Configure DM DP Pins */
  GPIO_InitStruct.Pin = (GPIO_PIN_11 | GPIO_PIN_12);
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);


  for (i=0; i<5; i++)
  {
    ledToggle(_DEF_LED1);
    delay(50);
  }
  ledOff(5);
}


void bspInitUSB(void)
{
  /* Init Device Library */
  USBD_Init(&USBD_Device, &VCP_Desc, 0);

  /* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);

  /* Add CDC Interface Class */
  USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);

  /* Start Device Process */
  USBD_Start(&USBD_Device);
}


void bspJumpToAddress(uint32_t address)
{
  bspDeinit();

  SCB->VTOR = address;

  __asm__ __volatile__("ldr r0, %0          \n"
                       "ldr sp, [r0]        \n"
                       "ldr pc, [r0, #4]    \n"
                       : : "m"(address));
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            PLL_R                          = 6
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 6;


  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while(1);
  }
  /* Enable the OverDrive to reach the 180 Mhz Frequency */
  if(HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    while(1);
  }

  /* Select PLLSAI output as USB clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CK48CLKSOURCE_PLLSAIP;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);


  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;


  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    while(1);
  }
}

