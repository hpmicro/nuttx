/****************************************************************************
 * arch/risc-v/src/hpmicro/hpm_gpio.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>

#include "chip.h"
#include "riscv_internal.h"
#include "hpm_soc.h"
#include "hpm_gpio_drv.h"
#include "hpm_gpiom_drv.h"
#include "hpm_gpio.h"

/****************************************************************************
 * Name: hpm_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 * Returned Value:
 *   OK on success
 *   ERROR on invalid port.
 *
 ****************************************************************************/

int hpm_configgpio(GPIO_Type *ptr, gpio_pin_t pin, enum gpio_pintype_e type)
{
  int ret = 0;
  gpio_interrupt_trigger_t trigger;

  HPM_IOC->PAD[pin].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0);
#ifndef CONFIG_ARCH_CHIP_HPM5361_SDK
  if (pin >= IOC_PAD_PZ00)
    {
      HPM_BIOC->PAD[pin].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(3);
    }
#endif
  if (pin >= IOC_PAD_PY00)
    {
      HPM_PIOC->PAD[pin].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(3);
    }

  switch (type)
  {
  case GPIO_INPUT_PIN:
    gpio_set_pin_input(ptr, GPIO_GET_PORT_INDEX(pin),
                          GPIO_GET_PIN_INDEX(pin));
    break;
  case GPIO_INPUT_PIN_PULLUP:
    gpio_set_pin_input(ptr, GPIO_GET_PORT_INDEX(pin),
                          GPIO_GET_PIN_INDEX(pin));
    HPM_IOC->PAD[pin].PAD_CTL = IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_PE_SET(1);
    break;
  case GPIO_INPUT_PIN_PULLDOWN:
    gpio_set_pin_input(ptr, GPIO_GET_PORT_INDEX(pin),
                          GPIO_GET_PIN_INDEX(pin));
    HPM_IOC->PAD[pin].PAD_CTL = IOC_PAD_PAD_CTL_PS_SET(0) | IOC_PAD_PAD_CTL_PE_SET(1);
    break;
  case GPIO_OUTPUT_PIN:
    HPM_IOC->PAD[pin].PAD_CTL = IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_PE_SET(1);
    gpio_set_pin_output(ptr, GPIO_GET_PORT_INDEX(pin),
                           GPIO_GET_PIN_INDEX(pin));
    break;
  case GPIO_OUTPUT_PIN_OPENDRAIN:
    HPM_IOC->PAD[pin].PAD_CTL = IOC_PAD_PAD_CTL_OD_SET(1);
    gpio_set_pin_output(ptr, GPIO_GET_PORT_INDEX(pin),
                          GPIO_GET_PIN_INDEX(pin));
    break;
  case GPIO_INTERRUPT_RISING_PIN:
    HPM_IOC->PAD[pin].PAD_CTL = IOC_PAD_PAD_CTL_PS_SET(0) | IOC_PAD_PAD_CTL_PE_SET(1);
    gpio_set_pin_input(ptr, GPIO_GET_PORT_INDEX(pin),
                          GPIO_GET_PIN_INDEX(pin));
    trigger = gpio_interrupt_trigger_edge_rising;
    gpio_config_pin_interrupt(ptr, GPIO_GET_PORT_INDEX(pin),
                          GPIO_GET_PORT_INDEX(pin), trigger);
    break;
  case GPIO_INTERRUPT_FALLING_PIN:
    HPM_IOC->PAD[pin].PAD_CTL = IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_PE_SET(1);
    gpio_set_pin_input(ptr, GPIO_GET_PORT_INDEX(pin),
                          GPIO_GET_PIN_INDEX(pin));
    trigger = gpio_interrupt_trigger_edge_falling;
    gpio_config_pin_interrupt(ptr, GPIO_GET_PORT_INDEX(pin),
                          GPIO_GET_PIN_INDEX(pin), trigger);
    break;
  default:
    ret = -1;
    break;
  }
  return ret;
}

/****************************************************************************
 * Name: hpm_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void hpm_gpiowrite(GPIO_Type *ptr, gpio_pin_t pin, bool value)
{
  gpio_write_pin(ptr, GPIO_GET_PORT_INDEX(pin), GPIO_GET_PIN_INDEX(pin), value);
}

/****************************************************************************
 * Name: hpm_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool hpm_gpioread(GPIO_Type *ptr, gpio_pin_t pin, uint8_t mode)
{
  if (mode == GPIO_INPUT_MODE)
    {
      return gpio_read_pin(ptr, GPIO_GET_PORT_INDEX(pin), GPIO_GET_PIN_INDEX(pin));
    }
  else if (mode == GPIO_OUTPUT_MODE)
    {
      return (ptr->DO[GPIO_GET_PORT_INDEX(pin)].VALUE & (1 << GPIO_GET_PIN_INDEX(pin))) >> GPIO_GET_PIN_INDEX(pin);
    }
  else
    {
      return false;
    }
}


/****************************************************************************
 * ADD for PX4
 ****************************************************************************/
struct gpio_callback_s
{
  xcpt_t callback;
  void  *arg;
  GPIO_Type *ptr;
  uint16_t port;
  uint16_t pin;
};


/****************************************************************************
 * Private Data
 ****************************************************************************/

#define  HPM_GPIO_USE_MALLOC

/* Interrupt handlers attached to each EXTI */
#if defined(HPM_GPIO_USE_MALLOC)
#ifdef IRQn_GPIO0_A
static struct gpio_callback_s *g_gpioa_callbacks[32] = {NULL};
#endif
#ifdef IRQn_GPIO0_B
static struct gpio_callback_s *g_gpiob_callbacks[32] = {NULL};
#endif
#ifdef IRQn_GPIO0_C
static struct gpio_callback_s *g_gpioc_callbacks[32] = {NULL};
#endif
#ifdef IRQn_GPIO0_D
static struct gpio_callback_s *g_gpiod_callbacks[32] = {NULL};
#endif
#ifdef IRQn_GPIO0_E
static struct gpio_callback_s *g_gpioe_callbacks[32] = {NULL};
#endif
#ifdef IRQn_GPIO0_F
static struct gpio_callback_s *g_gpiof_callbacks[32] = {NULL};
#endif
#ifdef IRQn_GPIO0_X
static struct gpio_callback_s *g_gpiox_callbacks[32] = {NULL};
#endif
#ifdef IRQn_GPIO0_Y
static struct gpio_callback_s *g_gpioy_callbacks[32] = {NULL};
#endif
#ifdef IRQn_GPIO0_Z
static struct gpio_callback_s *g_gpioz_callbacks[32] = {NULL};
#endif
#else
#ifdef IRQn_GPIO0_A
static struct gpio_callback_s g_gpioa_callbacks[32] = {0};
#endif
#ifdef IRQn_GPIO0_B
static struct gpio_callback_s g_gpiob_callbacks[32] = {0};
#endif
#ifdef IRQn_GPIO0_C
static struct gpio_callback_s g_gpioc_callbacks[32] = {0};
#endif
#ifdef IRQn_GPIO0_D
static struct gpio_callback_s g_gpiod_callbacks[32] = {0};
#endif
#ifdef IRQn_GPIO0_E
static struct gpio_callback_s g_gpioe_callbacks[32] = {0};
#endif
#ifdef IRQn_GPIO0_F
static struct gpio_callback_s g_gpiof_callbacks[32] = {0};
#endif
#ifdef IRQn_GPIO0_X
static struct gpio_callback_s g_gpiox_callbacks[32] = {0};
#endif
#ifdef IRQn_GPIO0_Y
static struct gpio_callback_s g_gpioy_callbacks[32] = {0};
#endif
#ifdef IRQn_GPIO0_Z
static struct gpio_callback_s g_gpioz_callbacks[32] = {0};
#endif
#endif

/****************************************************************************
 * Name: hpm_gpio_interrupt
 *
 * Description:
 *   gpio interrupt.
 *
 ****************************************************************************/

static int hpm_gpio_interrupt(int irq, void *context, void *arg)
{
#if defined(HPM_GPIO_USE_MALLOC)
  int ret = -1;
  struct gpio_callback_s **cb = (struct gpio_callback_s **)arg;

  for(int i = 0; i < 32; i++){
    if(cb[i]){
      if(gpio_check_pin_interrupt_enabled(cb[i]->ptr, cb[i]->port, i)
         && gpio_check_pin_interrupt_flag(cb[i]->ptr, cb[i]->port, i)){// 判断中断标志
        gpio_clear_pin_interrupt_flag(cb[i]->ptr, cb[i]->port, i);// 清空中断标志
        if (cb[i]->callback != NULL){//执行回调函数
          xcpt_t callback = cb[i]->callback;
          void   *cbarg   = cb[i]->arg;
          ret = callback(irq, context, cbarg);
        }
      }
    }
  }
#else
 int ret = -1;
  struct gpio_callback_s *cb = (struct gpio_callback_s *)arg;

  for(int i = 0; i < 32; i++){
    if(cb[i].ptr != NULL){
      if(gpio_check_pin_interrupt_enabled(cb[i].ptr, cb[i].port, i)
         && gpio_check_pin_interrupt_flag(cb[i].ptr, cb[i].port, i)){// 判断中断标志
        gpio_clear_pin_interrupt_flag(cb[i].ptr, cb[i].port, i);// 清空中断标志
        if (cb[i].callback != NULL){//执行回调函数
          xcpt_t callback = cb[i].callback;
          void   *cbarg   = cb[i].arg;
          ret = callback(irq, context, cbarg);
        }
      }
    }
  }
#endif
  return ret;
}

/****************************************************************************
 * Name: hpm_config_gpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *   Once it is configured as Alternative (GPIO_ALT|GPIO_CNF_AFPP|...)
 *   function, it must be unconfigured with hpm_unconfiggpio() with
 *   the same cfgset first before it can be set to non-alternative function.
 *
 * Returned Value:
 *   OK on success
 *   A negated errno value on invalid port, or when pin is locked as ALT
 *   function.
 *
 * To-Do: Auto Power Enable
 ****************************************************************************/

int hpm_config_gpio(uint32_t cfgset)
{
  GPIO_Type *ptr;
  uint32_t pad_ctl;
  uint32_t fun_ctl= 0;
  bool output = false;
  irqstate_t flags;

  uint32_t port = (cfgset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  uint32_t pin = (cfgset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  uint32_t pad_index = port * 32 + pin;
  if(pad_index > sizeof(IOC_Type) / (2 * sizeof(uint32_t))){
    return -EINVAL;
  }

  flags = enter_critical_section();

  pad_ctl = HPM_IOC->PAD[pad_index].PAD_CTL;

  switch (cfgset & GPIO_CONTROLLER_MASK){
    default:
    case GPIO0:
      ptr = HPM_GPIO0;
      gpiom_set_pin_controller(HPM_GPIOM, port, pin, gpiom_soc_gpio0);
      break;
#ifdef HPM_GPIO1
    case GPIO1:
      ptr = HPM_GPIO1;
      gpiom_set_pin_controller(HPM_GPIOM, port, pin, gpiom_soc_gpio1);
      break;
#endif
#ifdef HPM_FGPIO
    case FGPIO:
      ptr = HPM_FGPIO;
      gpiom_set_pin_controller(HPM_GPIOM, port, pin, gpiom_core0_fast);
      break;
#endif
#ifdef HPM_PGPIO
    case PGPIO:
      ptr = HPM_PGPIO;
      break;
#endif
#ifdef HPM_BGPIO
    case BGPIO:
      ptr = HPM_BGPIO;
      break;
#endif
  }

  fun_ctl &= ~IOC_PAD_FUNC_CTL_ALT_SELECT_MASK;
  pad_ctl &= ~(IOC_PAD_PAD_CTL_PE_MASK | IOC_PAD_PAD_CTL_PS_MASK);
  pad_ctl &= ~IOC_PAD_PAD_CTL_OD_MASK;
  switch (cfgset & GPIO_PUPD_MASK)
  {
    default:
    case GPIO_FLOAT:
      break;
    case GPIO_PULLUP:
      pad_ctl |= IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_PE_SET(1);
      break;
    case GPIO_PULLDOWN:
      pad_ctl |= IOC_PAD_PAD_CTL_PE_SET(1);
      break;
  }

  if(cfgset & GPIO_OPENDRAIN){
    pad_ctl |= IOC_PAD_PAD_CTL_OD_SET(1);
  }

  switch (cfgset & GPIO_MODE_MASK)
  {
    default:
    case GPIO_INPUT:
      gpio_set_pin_input(ptr, port, pin);
      break;

    case GPIO_OUTPUT:
      output = true;
      break;

    case GPIO_ALT:
      fun_ctl |= IOC_PAD_FUNC_CTL_ALT_SELECT_SET((cfgset & GPIO_AF_MASK) >> GPIO_AF_SHIFT);
      break;

    case GPIO_ANALOG:
      fun_ctl |= IOC_PAD_FUNC_CTL_ANALOG_SET(1);
      break;
  }

  if(cfgset & GPIO_LOOPBACK){
    fun_ctl |= IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1);
  }

  pad_ctl &= ~IOC_PAD_PAD_CTL_DS_MASK;
  pad_ctl |= IOC_PAD_PAD_CTL_DS_SET((cfgset & GPIO_DS_MASK) >> GPIO_DS_SHIFT);

#if defined(CONFIG_ARCH_CHIP_HPM6750_SDK)
  if(cfgset & GPIO_1V8){
    pad_ctl |= IOC_PAD_PAD_CTL_MS_SET(1);
  }else{
    pad_ctl &= ~IOC_PAD_PAD_CTL_MS_MASK;
  }

  if(cfgset & GPIO_SMT){
    pad_ctl |= IOC_PAD_PAD_CTL_SMT_SET(1);
  }else{
    pad_ctl &= ~IOC_PAD_PAD_CTL_SMT_MASK;
  }
#else
  pad_ctl &= ~IOC_PAD_PAD_CTL_SPD_MASK;
  pad_ctl |= IOC_PAD_PAD_CTL_SPD_SET((cfgset & GPIO_SPEED_MASK) >> GPIO_SPEED_SHIFT);
  if((cfgset & GPIO_SPEED_MASK) > GPIO_SPEED_50MHz){
    pad_ctl |= IOC_PAD_PAD_CTL_SR_SET(1);
  }else{
    pad_ctl &= ~IOC_PAD_PAD_CTL_SR_MASK;
  }

  if(cfgset & GPIO_SMT){
    pad_ctl |= IOC_PAD_PAD_CTL_HYS_SET(1);
  }else{
    pad_ctl &= ~IOC_PAD_PAD_CTL_HYS_MASK;
  }
#endif

  switch (cfgset & GPIO_CONTROLLER_MASK){
    default:
    case GPIO0:
#ifdef HPM_GPIO1
    case GPIO1:
#endif
      HPM_IOC->PAD[pad_index].FUNC_CTL = fun_ctl;
      HPM_IOC->PAD[pad_index].PAD_CTL = pad_ctl;
      // PIOC 和 BIOC 可以把电源管理域 IO（PY）和电池备份域 IO（PZ）中的一个或者多个 IO 映射到系统电源
      // 域。之后，这些 IO 就可以由 IOC 控制。
#ifdef IOC_PAD_PZ00
      if (pad_index >= IOC_PAD_PZ00){
        HPM_BIOC->PAD[pad_index].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(3);
      }else
#endif
      if (pad_index >= IOC_PAD_PY00) {
        HPM_PIOC->PAD[pad_index].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(3);
      }
    break;
#ifdef HPM_PGPIO
    case PGPIO:
      if (pad_index >= IOC_PAD_PY00) {
        HPM_PIOC->PAD[pad_index].FUNC_CTL = fun_ctl;
        HPM_PIOC->PAD[pad_index].PAD_CTL = pad_ctl;
      }
      break;
#endif
#ifdef HPM_BGPIO
    case BGPIO:
#ifdef IOC_PAD_PZ00
      if (pad_index >= IOC_PAD_PZ00){
        HPM_BIOC->PAD[pad_index].FUNC_CTL = fun_ctl;
        HPM_BIOC->PAD[pad_index].PAD_CTL = pad_ctl;
      }
#endif
      break;
#endif
  }

  if(output){
    gpio_set_pin_output(ptr, port, pin);
    if(cfgset & GPIO_OUTPUT_SET){
      gpio_write_pin(ptr, port, pin, 1);
    }else{
      gpio_write_pin(ptr, port, pin, 0);
    }
  }

  leave_critical_section(flags);
  return OK;
}


/****************************************************************************
 * Name: hpm_unconfig_gpio
 *
 * Description:
 *   Unconfigure a GPIO pin based on bit-encoded description of the pin, set
 *   it into default HiZ state (and possibly mark it's unused) and unlock it
 *   whether it was previously selected as alternative function
 *   (GPIO_ALT|GPIO_CNF_AFPP|...).
 *
 *   This is a safety function and prevents hardware from shocks, as
 *   unexpected write to the Timer Channel Output GPIO to fixed '1' or '0'
 *   while it should operate in PWM mode could produce excessive on-board
 *   currents and trigger over-current/alarm function.
 *
 * Returned Value:
 *  OK on success
 *  A negated errno value on invalid port
 *
 * To-Do: Auto Power Disable
 ****************************************************************************/

int hpm_unconfig_gpio(uint32_t cfgset)
{
  /* Reuse port and pin number and set it to default HiZ INPUT */

  cfgset &= GPIO_PORT_MASK | GPIO_PIN_MASK;
  cfgset |= GPIO_INPUT | GPIO_FLOAT;

  /* To-Do: Mark its unuse for automatic power saving options */

  return hpm_config_gpio(cfgset);
}

/****************************************************************************
 * Name: hpm_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void hpm_gpio_write(uint32_t pinset, bool value)
{
  GPIO_Type *ptr;

  // GPIO控制器选择
  switch (pinset & GPIO_CONTROLLER_MASK){
    default:
    case GPIO0:
      ptr = HPM_GPIO0;
      break;
#ifdef HPM_GPIO1
    case GPIO1:
      ptr = HPM_GPIO1;
      break;
#endif
#ifdef HPM_FGPIO
    case FGPIO:
      ptr = HPM_FGPIO;
      break;
#endif
#ifdef HPM_PGPIO
    case PGPIO:
      ptr = HPM_PGPIO;
      break;
#endif
#ifdef HPM_BGPIO
    case BGPIO:
      ptr = HPM_BGPIO;
      break;
#endif
  }

  gpio_write_pin(ptr, (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT, (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT, value);
}

/****************************************************************************
 * Name: hpm_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool hpm_gpio_read(uint32_t pinset)
{
  GPIO_Type *ptr;

  // GPIO控制器选择
  switch (pinset & GPIO_CONTROLLER_MASK){
    default:
    case GPIO0:
      ptr = HPM_GPIO0;
      break;
#ifdef HPM_GPIO1
    case GPIO1:
      ptr = HPM_GPIO1;
      break;
#endif
#ifdef HPM_FGPIO
    case FGPIO:
      ptr = HPM_FGPIO;
      break;
#endif
#ifdef HPM_PGPIO
    case PGPIO:
      ptr = HPM_PGPIO;
      break;
#endif
#ifdef HPM_BGPIO
    case BGPIO:
      ptr = HPM_BGPIO;
      break;
#endif
  }

  uint32_t port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT; // 端口0~15
  uint32_t pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT; // 引脚0~31
  bool ret = 0;
  /* 模式 */
  switch (pinset & GPIO_MODE_MASK)
  {
    case GPIO_INPUT:      /* 输入 */
      ret = ((ptr->DI[port].VALUE & (1 << pin)) >> pin);
      break;
    case GPIO_OUTPUT:     /* 输出 */
      ret = ((ptr->DO[port].VALUE & (1 << pin)) >> pin);
      break;
  }

  return ret;
}


/****************************************************************************
 * Name: hpm_gpio_setevent
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 *
 * Input Parameters:
 *  - pinset:      GPIO pin configuration
 *  - risingedge:  Enables interrupt on rising edges
 *  - fallingedge: Enables interrupt on falling edges
 *  - event:       Generate event when set
 *  - func:        When non-NULL, generate interrupt
 *  - arg:         Argument passed to the interrupt callback
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure indicating the
 *   nature of the failure.
 *
 ****************************************************************************/

int hpm_gpio_setevent(uint32_t pinset, bool risingedge, bool fallingedge,
                       bool event, xcpt_t func, void *arg)
{

#if defined(HPM_GPIO_USE_MALLOC)
  GPIO_Type *ptr;
  irqstate_t flags;
  int      irq = -1;
  // xcpt_t   handler;
  struct gpio_callback_s **cb;

  /* 如果event或func不为空，则将引脚配置成中断 */
  if (event || func){
    pinset |= GPIO_EXTI;
  }

  uint32_t port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT; // 端口0~15
  uint32_t pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT; // 引脚0~31

 // GPIO控制器选择
  switch (pinset & GPIO_CONTROLLER_MASK){
    default:
    case GPIO0:
      ptr = HPM_GPIO0;
      break;
#ifdef HPM_GPIO1
    case GPIO1:
      ptr = HPM_GPIO1;
      break;
#endif
#ifdef HPM_FGPIO
    case FGPIO:
      return -1; // 不支持中断
#endif
#ifdef HPM_PGPIO
    case PGPIO:
      ptr = HPM_PGPIO;
      break;
#endif
#ifdef HPM_BGPIO
    case BGPIO:
      ptr = HPM_BGPIO;
      break;
#endif
  }

  if(ptr == HPM_GPIO0){
    switch(port){
#ifdef HPM_IRQn_GPIO0_A
      case 0:
        irq = HPM_IRQn_GPIO0_A;
        cb = &g_gpioa_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_B
      case 1:
        irq = HPM_IRQn_GPIO0_B;
        cb = &g_gpiob_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_C
      case 2:
        irq = HPM_IRQn_GPIO0_C;
        cb = &g_gpioc_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_D
      case 3:
        irq = HPM_IRQn_GPIO0_D;
        cb = &g_gpiod_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_E
      case 4:
        irq = HPM_IRQn_GPIO0_E;
        cb = &g_gpioe_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_F
      case 5:
        irq = HPM_IRQn_GPIO0_F;
        cb = &g_gpiof_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_X
      case 13:
        irq = HPM_IRQn_GPIO0_X;
        cb = &g_gpiox_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_Y
      case 14:
        irq = HPM_IRQn_GPIO0_Y;
        cb = &g_gpioy_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_Z
      case 15:
        irq = HPM_IRQn_GPIO0_Z;
        cb = &g_gpioz_callbacks[0];
        break;
#endif
    }

  }
#ifdef HPM_GPIO1
  else if(ptr == HPM_GPIO1){
    switch(port){
#ifdef HPM_IRQn_GPIO1_A
    case 0:
      irq = HPM_IRQn_GPIO1_A;
      cb = &g_gpioa_callbacks[0];
      break;
#endif
#ifdef HPM_IRQn_GPIO1_B
    case 1:
      irq = HPM_IRQn_GPIO1_B;
      cb = &g_gpiob_callbacks[0];
      break;
#endif
#ifdef HPM_IRQn_GPIO1_C
    case 2:
      irq = HPM_IRQn_GPIO1_C;
      cb = &g_gpioc_callbacks[0];
      break;
#endif
#ifdef HPM_IRQn_GPIO1_D
    case 3:
      irq = HPM_IRQn_GPIO1_D;
      cb = &g_gpiod_callbacks[0];
      break;
#endif
#ifdef HPM_IRQn_GPIO1_E
    case 4:
      irq = HPM_IRQn_GPIO1_E;
      cb = &g_gpioe_callbacks[0];
      break;
#endif
#ifdef HPM_IRQn_GPIO1_F
    case 5:
      irq = HPM_IRQn_GPIO1_F;
      cb = &g_gpiof_callbacks[0];
      break;
#endif
#ifdef HPM_IRQn_GPIO1_X
    case 13:
      irq = HPM_IRQn_GPIO1_X;
      cb = &g_gpiox_callbacks[0];
      break;
#endif
#ifdef HPM_IRQn_GPIO1_Y
    case 14:
      irq = HPM_IRQn_GPIO1_Y;
      cb = &g_gpioy_callbacks[0];
      break;
#endif
#ifdef HPM_IRQn_GPIO1_Z
    case 15:
      irq = HPM_IRQn_GPIO1_Z;
      cb = &g_gpioz_callbacks[0];
      break;
#endif
    }
  }
#endif
#ifdef HPM_IRQn_PGPIO
  else if(ptr == HPM_PGPIO){
    irq = HPM_IRQn_PGPIO;
    switch(port){
#ifdef HPM_IRQn_GPIO0_A
      case 0:
        cb = &g_gpioa_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_B
      case 1:
        cb = &g_gpiob_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_C
      case 2:
        cb = &g_gpioc_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_D
      case 3:
        cb = &g_gpiod_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_E
      case 4:
        cb = &g_gpioe_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_F
      case 5:
        cb = &g_gpiof_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_X
      case 13:
        cb = &g_gpiox_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_Y
      case 14:
        cb = &g_gpioy_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_Z
      case 15:
        cb = &g_gpioz_callbacks[0];
        break;
#endif
    }
  }
#endif
#ifdef HPM_IRQn_BGPIO
  else if(ptr == HPM_BGPIO){
    irq = HPM_IRQn_BGPIO;
    switch(port){
#ifdef HPM_IRQn_GPIO0_A
      case 0:
        cb = &g_gpioa_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_B
      case 1:
        cb = &g_gpiob_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_C
      case 2:
        cb = &g_gpioc_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_D
      case 3:
        cb = &g_gpiod_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_E
      case 4:
        cb = &g_gpioe_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_F
      case 5:
        cb = &g_gpiof_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_X
      case 13:
        cb = &g_gpiox_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_Y
      case 14:
        cb = &g_gpioy_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_Z
      case 15:
        cb = &g_gpioz_callbacks[0];
        break;
#endif
    }
  }
#endif

  if(irq < 0){
    return -1;
  }

  flags = enter_critical_section();

  if(risingedge && !fallingedge){
    gpio_config_pin_interrupt(ptr, port, pin, gpio_interrupt_trigger_edge_rising);

  }else if(!risingedge && fallingedge){
    gpio_config_pin_interrupt(ptr, port, pin, gpio_interrupt_trigger_edge_falling);

  }
#if defined(GPIO_SOC_HAS_EDGE_BOTH_INTERRUPT) && (GPIO_SOC_HAS_EDGE_BOTH_INTERRUPT == 1)
  else if(risingedge && fallingedge){
    gpio_config_pin_interrupt(ptr, port,pin,gpio_interrupt_trigger_edge_both);

  }
#endif

  hpm_config_gpio(pinset);

	gpio_clear_pin_interrupt_flag(ptr, port, pin);// 清空中断标志
  if(func){
    if(cb[pin] == NULL){
      cb[pin] = (struct gpio_callback_s *)malloc(sizeof(struct gpio_callback_s));
      if(cb[pin] == NULL){
        return -1;
      }
    }
    cb[pin]->callback = func;
    cb[pin]->arg      = arg;
    cb[pin]->ptr      = ptr;
    cb[pin]->port     = port;
    cb[pin]->pin      = pin;

		irq_attach(irq, hpm_gpio_interrupt, (void*)cb);
		up_enable_irq(irq);// 使能全局中断
		gpio_enable_pin_interrupt(ptr, port, pin);// 使能引脚中断

  }else{
    if(cb[pin]!= NULL){
      free(cb[pin]);
      cb[pin] = NULL;
    }
		gpio_disable_pin_interrupt(ptr, port, pin);//禁止引脚中断

		bool disirq =true;
		for (int i = 0; i < 32; i++){
			if (cb[i] != NULL){
				disirq = false;
				break;
			}
		}

		if (disirq){
			up_disable_irq(irq);// 禁止全局中断
		}
  }

  leave_critical_section(flags);

#else

  GPIO_Type *ptr;
  irqstate_t flags;
  int      irq = -1;
  // xcpt_t   handler;
  struct gpio_callback_s *cb;

  /* 如果event或func不为空，则将引脚配置成中断 */
  if (event || func){
    pinset |= GPIO_EXTI;
  }

  uint32_t port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT; // 端口0~15
  uint32_t pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT; // 引脚0~31

 // GPIO控制器选择
  switch (pinset & GPIO_CONTROLLER_MASK){
    default:
    case GPIO0:
      ptr = HPM_GPIO0;
      break;
#ifdef HPM_GPIO1
    case GPIO1:
      ptr = HPM_GPIO1;
      break;
#endif
#ifdef HPM_FGPIO
    case FGPIO:
      return -1; // 不支持中断
#endif
#ifdef HPM_PGPIO
    case PGPIO:
      ptr = HPM_PGPIO;
      break;
#endif
#ifdef HPM_BGPIO
    case BGPIO:
      ptr = HPM_BGPIO;
      break;
#endif
  }

  if(ptr == HPM_GPIO0){
    switch(port){
#ifdef HPM_IRQn_GPIO0_A
      case 0:
        irq = HPM_IRQn_GPIO0_A;
        cb = &g_gpioa_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_B
      case 1:
        irq = HPM_IRQn_GPIO0_B;
        cb = &g_gpiob_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_C
      case 2:
        irq = HPM_IRQn_GPIO0_C;
        cb = &g_gpioc_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_D
      case 3:
        irq = HPM_IRQn_GPIO0_D;
        cb = &g_gpiod_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_E
      case 4:
        irq = HPM_IRQn_GPIO0_E;
        cb = &g_gpioe_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_F
      case 5:
        irq = HPM_IRQn_GPIO0_F;
        cb = &g_gpiof_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_X
      case 13:
        irq = HPM_IRQn_GPIO0_X;
        cb = &g_gpiox_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_Y
      case 14:
        irq = HPM_IRQn_GPIO0_Y;
        cb = &g_gpioy_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_Z
      case 15:
        irq = HPM_IRQn_GPIO0_Z;
        cb = &g_gpioz_callbacks[0];
        break;
#endif
    }

  }
#ifdef HPM_GPIO1
  else if(ptr == HPM_GPIO1){
    switch(port){
#ifdef HPM_IRQn_GPIO1_A
    case 0:
      irq = HPM_IRQn_GPIO1_A;
      cb = &g_gpioa_callbacks[0];
      break;
#endif
#ifdef HPM_IRQn_GPIO1_B
    case 1:
      irq = HPM_IRQn_GPIO1_B;
      cb = &g_gpiob_callbacks[0];
      break;
#endif
#ifdef HPM_IRQn_GPIO1_C
    case 2:
      irq = HPM_IRQn_GPIO1_C;
      cb = &g_gpioc_callbacks[0];
      break;
#endif
#ifdef HPM_IRQn_GPIO1_D
    case 3:
      irq = HPM_IRQn_GPIO1_D;
      cb = &g_gpiod_callbacks[0];
      break;
#endif
#ifdef HPM_IRQn_GPIO1_E
    case 4:
      irq = HPM_IRQn_GPIO1_E;
      cb = &g_gpioe_callbacks[0];
      break;
#endif
#ifdef HPM_IRQn_GPIO1_F
    case 5:
      irq = HPM_IRQn_GPIO1_F;
      cb = &g_gpiof_callbacks[0];
      break;
#endif
#ifdef HPM_IRQn_GPIO1_X
    case 13:
      irq = HPM_IRQn_GPIO1_X;
      cb = &g_gpiox_callbacks[0];
      break;
#endif
#ifdef HPM_IRQn_GPIO1_Y
    case 14:
      irq = HPM_IRQn_GPIO1_Y;
      cb = &g_gpioy_callbacks[0];
      break;
#endif
#ifdef HPM_IRQn_GPIO1_Z
    case 15:
      irq = HPM_IRQn_GPIO1_Z;
      cb = &g_gpioz_callbacks[0];
      break;
#endif
    }
  }
#endif
#ifdef HPM_IRQn_PGPIO
  else if(ptr == HPM_PGPIO){
    irq = HPM_IRQn_PGPIO;
    switch(port){
#ifdef HPM_IRQn_GPIO0_A
      case 0:
        cb = &g_gpioa_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_B
      case 1:
        cb = &g_gpiob_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_C
      case 2:
        cb = &g_gpioc_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_D
      case 3:
        cb = &g_gpiod_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_E
      case 4:
        cb = &g_gpioe_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_F
      case 5:
        cb = &g_gpiof_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_X
      case 13:
        cb = &g_gpiox_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_Y
      case 14:
        cb = &g_gpioy_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_Z
      case 15:
        cb = &g_gpioz_callbacks[0];
        break;
#endif
    }
  }
#endif
#ifdef HPM_IRQn_BGPIO
  else if(ptr == HPM_BGPIO){
    irq = HPM_IRQn_BGPIO;
    switch(port){
#ifdef HPM_IRQn_GPIO0_A
      case 0:
        cb = &g_gpioa_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_B
      case 1:
        cb = &g_gpiob_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_C
      case 2:
        cb = &g_gpioc_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_D
      case 3:
        cb = &g_gpiod_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_E
      case 4:
        cb = &g_gpioe_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_F
      case 5:
        cb = &g_gpiof_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_X
      case 13:
        cb = &g_gpiox_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_Y
      case 14:
        cb = &g_gpioy_callbacks[0];
        break;
#endif
#ifdef HPM_IRQn_GPIO0_Z
      case 15:
        cb = &g_gpioz_callbacks[0];
        break;
#endif
    }
  }
#endif

  if(irq < 0){
    return -1;
  }

  /* Get the previous GPIO IRQ handler; Save the new IRQ handler. */
  cb[pin].callback = func;
  cb[pin].arg      = arg;
  cb[pin].ptr      = ptr;
  cb[pin].port     = port;
  cb[pin].pin      = pin;

  flags = enter_critical_section();

  if(risingedge && !fallingedge){
    gpio_config_pin_interrupt(ptr, port, pin, gpio_interrupt_trigger_edge_rising);

  }else if(!risingedge && fallingedge){
    gpio_config_pin_interrupt(ptr, port, pin, gpio_interrupt_trigger_edge_falling);

  }
#if defined(GPIO_SOC_HAS_EDGE_BOTH_INTERRUPT) && (GPIO_SOC_HAS_EDGE_BOTH_INTERRUPT == 1)
  else if(risingedge && fallingedge){
    gpio_config_pin_interrupt(ptr, port,pin,gpio_interrupt_trigger_edge_both);

  }
#endif

  hpm_config_gpio(pinset);

	gpio_clear_pin_interrupt_flag(ptr, port, pin);// 清空中断标志
  if(func){
		irq_attach(irq, hpm_gpio_interrupt, (void*)cb);
		up_enable_irq(irq);// 使能全局中断
		gpio_enable_pin_interrupt(ptr, port, pin);// 使能引脚中断

  }else{
		cb[pin].ptr = NULL;
		gpio_disable_pin_interrupt(ptr, port, pin);//禁止引脚中断

		bool disirq =true;
		for (int i = 0; i < 32; i++){
			if (cb[i].callback != NULL){
				disirq = false;
				break;
			}
		}

		if (disirq){
			up_disable_irq(irq);// 禁止全局中断
		}
  }

  leave_critical_section(flags);

#endif

return OK;
}
