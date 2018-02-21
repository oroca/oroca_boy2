/*
 * drv_exti.c
 *
 *  Created on: 2017. 11. 13.
 *      Author: opus
 */

#include "hw.h"
#include "drv_exti.h"

typedef struct
{
  GPIO_TypeDef *port;
  uint32_t pin;
  IRQn_Type irq_num;
  void (*callback_func)(void*);
  void *func_arg;
} drv_exti_t;

void drvExtiCallbackTemplate(void *arg);

static drv_exti_t drv_exti_tbl[_HW_DEF_EXTI_CH_MAX] =
{
  { GPIOA, GPIO_PIN_0, EXTI0_IRQn, drvExtiCallbackTemplate, NULL },
};

void drvExtiInit(void)
{
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 11);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

bool drvExtiAttachInterrupt(uint8_t ch, uint32_t mode, void (*func)(void *),
    void *arg)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  if ((ch > _HW_DEF_EXTI_CH_MAX)||(func == NULL))
    return false;

  switch (mode)
  {
    case _DEF_EXTI_RISING:
      GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
      break;

    case _DEF_EXTI_FALLING:
      GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
      break;

    case _DEF_EXTI_BOTH:
      GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
      break;

    case _DEF_EVT_RISING:
      GPIO_InitStructure.Mode = GPIO_MODE_EVT_RISING;
      break;

    case _DEF_EVT_FALLING:
      GPIO_InitStructure.Mode = GPIO_MODE_EVT_FALLING;
      break;

    case _DEF_EVT_BOTH:
      GPIO_InitStructure.Mode = GPIO_MODE_EVT_RISING_FALLING;
      break;

    default:
      return false;
  }

  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = drv_exti_tbl[ch].pin;

  HAL_GPIO_Init(drv_exti_tbl[ch].port, &GPIO_InitStructure);

  drv_exti_tbl[ch].callback_func = func;
  drv_exti_tbl[ch].func_arg = arg;

  return true;
}

void drvExtiDetachInterrupt(uint8_t ch)
{
  HAL_NVIC_DisableIRQ(drv_exti_tbl[ch].irq_num);
  HAL_GPIO_DeInit(drv_exti_tbl[ch].port, drv_exti_tbl[ch].pin);
  drv_exti_tbl[ch].callback_func = drvExtiCallbackTemplate;
  HAL_NVIC_EnableIRQ(drv_exti_tbl[ch].irq_num);
}

void drvExtiCallbackTemplate(void *arg)
{
  printf(
      "You must register a callback function for EXTI channel you want to use \r\n");
}

/**
 * @brief EXTI line detection callbacks
 * @param GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case GPIO_PIN_0:
      (*drv_exti_tbl[0].callback_func)(drv_exti_tbl[0].func_arg);
      break;
    default:
      return;
  }
}

void __attribute__((used)) EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}
