/****************************************************************************
 * boards/risc-v/hpmicro/hpm5300evk-sdk/src/hpm5300_gpio.c
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
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <nuttx/arch.h>
#include <nuttx/ioexpander/gpio.h>
#include <arch/board/board.h>
#include "riscv_internal.h"
#include "hpm_config.h"
#include "board.h"
#include "chip.h"
#include "hpm.h"
#include "hpm_soc.h"
#include "hpm_gpiom_drv.h"
#include "hpm5300evk.h"
#include "hpm_gpio.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct hpm5300_gpio_dev_s
{
  struct gpio_dev_s gpio;
  uint32_t           id;
};

struct hpm5300_gpint_dev_s
{
  struct hpm5300_gpio_dev_s hpm5300gpio;
  pin_interrupt_t           callback;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gpout_read(struct gpio_dev_s *dev, bool *value);
static int gpout_write(struct gpio_dev_s *dev, bool value);
static int gpint_read(struct gpio_dev_s *dev, bool *value);
static int gpint_attach(struct gpio_dev_s *dev,
                        pin_interrupt_t callback);
static int gpint_enable(struct gpio_dev_s *dev, bool enable);
static int gpio_setpintype(struct gpio_dev_s *dev,
                           enum gpio_pintype_e    gp_pintype);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gpio_operations_s gpout_ops =
{
  .go_read       = gpout_read,
  .go_write      = gpout_write,
  .go_attach     = NULL,
  .go_enable     = NULL,
  .go_setpintype = NULL,
};

static const struct gpio_operations_s gpint_ops =
{
  .go_read       = gpint_read,
  .go_write      = NULL,
  .go_attach     = gpint_attach,
  .go_enable     = gpint_enable,
  .go_setpintype = gpio_setpintype,
};

#if BOARD_NGPIOOUT
/* This array maps the GPIO pins used as OUTPUT */

static const uint32_t g_gpiooutputs[BOARD_NGPIOOUT] =
{
  BOARD_GPIO_OUT1,
};

static struct hpm5300_gpio_dev_s g_gpout[BOARD_NGPIOOUT];
#endif

#if BOARD_NGPIOINT > 0
/* This array maps the GPIO pins used as INTERRUPT INPUTS */

static const uint32_t g_gpiointinputs[BOARD_NGPIOINT] =
{
  BOARD_GPIO_INT1,
};

static struct hpm5300_gpint_dev_s g_gpint[BOARD_NGPIOINT];
#endif

/****************************************************************************
 * Name: gpio_get_irq_index
 *
 * Description:
 *   read gpio irq number
 *
 ****************************************************************************/

static uint32_t gpio_get_irq_index(GPIO_Type *ptr, uint32_t pin)
{
    uint32_t offset = 0;
    uint32_t start_irqnum = 0;
    DEBUGASSERT(ptr != HPM_PGPIO);

    start_irqnum = HPM_IRQn_GPIO0_A;
    if (pin >= IOC_PAD_PX00)
      {
        offset = IOC_PAD_PX00;
        start_irqnum = HPM_IRQn_GPIO0_X;
      }

    return (((pin - offset) / PORT_PIN_COUNT) + start_irqnum);
}

/****************************************************************************
 * Name: hpm5300_gpio_interrupt
 *
 * Description:
 *   gpio interrupt.
 *
 ****************************************************************************/

static int hpm5300_gpio_interrupt(int irq, void *context, void *arg)
{
  struct hpm5300_gpint_dev_s *hpm5300xgpint =
    (struct hpm5300_gpint_dev_s *)arg;

  uint32_t gpio_pin;

  DEBUGASSERT(hpm5300xgpint != NULL && hpm5300xgpint->callback != NULL);
  gpioinfo("Interrupt! callback=%p\n", hpm5300xgpint->callback);

  gpio_pin = g_gpiointinputs[hpm5300xgpint->hpm5300gpio.id];
   
  gpio_clear_pin_interrupt_flag(BOARD_APP_GPIO_CTRL, GPIO_GET_PORT_INDEX(gpio_pin),
                          GPIO_GET_PIN_INDEX(gpio_pin));

  hpm5300xgpint->callback(&hpm5300xgpint->hpm5300gpio.gpio,
                        gpio_pin);

  return OK;
}


/****************************************************************************
 * Name: gpio_setpintype
 *
 * Description:
 *   set gpio pintype.
 *
 ****************************************************************************/

static int gpio_setpintype(struct gpio_dev_s *dev,
                           enum gpio_pintype_e    gp_pintype)
{
  struct hpm5300_gpint_dev_s *hpm5300gpint =
    (struct hpm5300_gpint_dev_s *)dev;
  uint32_t gpio_pin;
  uint8_t pintype = hpm5300gpint->hpm5300gpio.gpio.gp_pintype;
  int ret = 0;

  DEBUGASSERT(hpm5300gpint != NULL);
  gpioinfo("setpintype...\n");

  if (pintype >= GPIO_NPINTYPES)
    {
      gpioerr("pintype error\n");
      return -1;
    }

  gpio_pin = g_gpiointinputs[hpm5300gpint->hpm5300gpio.id];
  ret = hpm_configgpio(BOARD_APP_GPIO_CTRL, gpio_pin, gp_pintype);

  return ret;
}

/****************************************************************************
 * Name: gpout_read
 *
 * Description:
 *   read gpio output.
 *
 ****************************************************************************/

static int gpout_read(struct gpio_dev_s *dev, bool *value)
{
  struct hpm5300_gpio_dev_s *hpm5300xgpio =
    (struct hpm5300_gpio_dev_s *)dev;

  DEBUGASSERT(hpm5300xgpio != NULL && value != NULL);
  DEBUGASSERT(hpm5300xgpio->id < BOARD_NGPIOOUT);
  gpioinfo("Reading...\n");

  uint32_t gpio_pin = g_gpiooutputs[hpm5300xgpio->id];

  *value = hpm_gpioread(BOARD_APP_GPIO_CTRL, gpio_pin, GPIO_OUTPUT_MODE);

  return OK;
}

/****************************************************************************
 * Name: gpout_write
 *
 * Description:
 *   write gpio.
 *
 ****************************************************************************/

static int gpout_write(struct gpio_dev_s *dev, bool value)
{
  struct hpm5300_gpio_dev_s *hpm5300xgpio =
    (struct hpm5300_gpio_dev_s *)dev;

  DEBUGASSERT(hpm5300xgpio != NULL);
  DEBUGASSERT(hpm5300xgpio->id < BOARD_NGPIOOUT);
  gpioinfo("Writing %d\n", (int)value);

  hpm_gpiowrite(BOARD_APP_GPIO_CTRL, g_gpiooutputs[hpm5300xgpio->id], value);

  return OK;
}

/****************************************************************************
 * Name: gpint_read
 *
 * Description:
 *   read gpio.
 *
 ****************************************************************************/

static int gpint_read(struct gpio_dev_s *dev, bool *value)
{
  struct hpm5300_gpint_dev_s *hpm5300xgpint =
    (struct hpm5300_gpint_dev_s *)dev;

  DEBUGASSERT(hpm5300xgpint != NULL && value != NULL);
  DEBUGASSERT(hpm5300xgpint->hpm5300gpio.id < BOARD_NGPIOINT);
  gpioinfo("Reading int pin...\n");

  *value = hpm_gpioread(BOARD_APP_GPIO_CTRL, g_gpiointinputs[hpm5300xgpint->hpm5300gpio.id], GPIO_INPUT_MODE);

  return OK;
}

/****************************************************************************
 * Name: gpint_attach
 *
 * Description:
 *   gpio attach.
 *
 ****************************************************************************/

static int gpint_attach(struct gpio_dev_s *dev, pin_interrupt_t callback)
{
  struct hpm5300_gpint_dev_s *hpm5300xgpint =
    (struct hpm5300_gpint_dev_s *)dev;

  uint32_t gpio_pin = g_gpiointinputs[hpm5300xgpint->hpm5300gpio.id]; 
  gpioinfo("Attaching the callback\n");

  /* Make sure the interrupt is disabled */

  hpm5300xgpint->callback = callback;

  irq_attach(gpio_get_irq_index(BOARD_APP_GPIO_CTRL, gpio_pin), hpm5300_gpio_interrupt, (void *)dev);

  gpioinfo("Attach %p\n", callback);
  return OK;
}

/****************************************************************************
 * Name: gpint_enable
 *
 * Description:
 *   gpint enable.
 *
 ****************************************************************************/

static int gpint_enable(struct gpio_dev_s *dev, bool enable)
{
  struct hpm5300_gpint_dev_s *hpm5300xgpint =
    (struct hpm5300_gpint_dev_s *)dev;

  uint32_t gpio_pin = g_gpiointinputs[hpm5300xgpint->hpm5300gpio.id]; 
  if (enable)
    {
      if (hpm5300xgpint->callback != NULL)
        {
          gpioinfo("Enabling the interrupt\n");
          up_enable_irq(gpio_get_irq_index(BOARD_APP_GPIO_CTRL, gpio_pin));
          gpio_enable_pin_interrupt(BOARD_APP_GPIO_CTRL, GPIO_GET_PORT_INDEX(gpio_pin),
                           GPIO_GET_PIN_INDEX(gpio_pin));
        }
    }
  else
    {
      gpioinfo("Disable the interrupt\n");
      up_disable_irq(gpio_get_irq_index(BOARD_APP_GPIO_CTRL, gpio_pin));
      gpio_disable_pin_interrupt(BOARD_APP_GPIO_CTRL, GPIO_GET_PORT_INDEX(gpio_pin),
                           GPIO_GET_PIN_INDEX(gpio_pin));
    }

  return OK;
}

/****************************************************************************
 * Name: hpm5300_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

int hpm5300_gpio_initialize(void)
{
  int i;
  int pincount = 0;
  uint32_t gpio_pin = 0;

#if BOARD_NGPIOOUT > 0
  for (i = 0; i < BOARD_NGPIOOUT; i++)
    {

      /* Setup and register the GPIO pin */

      g_gpout[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpout[i].gpio.gp_ops     = &gpout_ops;
      g_gpout[i].id              = i;
      gpio_pin_register(&g_gpout[i].gpio, pincount);

      /* Configure the pin that will be used as output */

      gpio_pin = g_gpiooutputs[i];
      hpm_configgpio(BOARD_APP_GPIO_CTRL, gpio_pin, GPIO_OUTPUT_PIN);
      hpm_gpiowrite(BOARD_APP_GPIO_CTRL, gpio_pin, 0);
      gpiom_set_pin_controller(HPM_GPIOM, GPIO_GET_PORT_INDEX(gpio_pin), GPIO_GET_PIN_INDEX(gpio_pin), gpiom_soc_gpio0);
      pincount++;
    }
#endif

#if BOARD_NGPIOINT > 0
  for (i = 0; i < BOARD_NGPIOINT; i++)
    {

      /* Setup and register the GPIO pin */

      g_gpint[i].hpm5300gpio.gpio.gp_pintype = GPIO_INTERRUPT_PIN;
      g_gpint[i].hpm5300gpio.gpio.gp_ops     = &gpint_ops;
      g_gpint[i].hpm5300gpio.id              = i;
      gpio_pin_register(&g_gpint[i].hpm5300gpio.gpio, pincount);

      /* Configure the pin that will be used as interrupt input */

      gpio_pin = g_gpiointinputs[i];
      hpm_configgpio(BOARD_APP_GPIO_CTRL, gpio_pin, GPIO_INTERRUPT_FALLING_PIN);
      gpiom_set_pin_controller(HPM_GPIOM, GPIO_GET_PORT_INDEX(gpio_pin), GPIO_GET_PORT_INDEX(gpio_pin), gpiom_soc_gpio0);

      pincount++;
    }
#endif

  return 0;
}
