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
  if (pin >= IOC_PAD_PZ00)
    {
      HPM_IOC->PAD[pin].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0);
      HPM_BIOC->PAD[pin].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(3); 
    }
  else if (pin >= IOC_PAD_PY00)
    {
      HPM_IOC->PAD[pin].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0);
      HPM_PIOC->PAD[pin].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(3); 
    }
  else
    {
      HPM_IOC->PAD[pin].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(0);
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
