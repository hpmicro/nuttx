/****************************************************************************
 * arch/risc-v/src/hpmicro/hpm_gpio.h
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

#ifndef __ARCH_RISCV_SRC_HPMICRO_GPIO_H
#define __ARCH_RISCV_SRC_HPMICRO_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#include <stdint.h>
#include <stdbool.h>
#endif

#include <nuttx/irq.h>

#include "chip.h"
#include "nuttx/ioexpander/gpio.h"
#include "hpm_gpio_drv.h"

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/
#define  GPIO_INPUT_MODE       (0UL)
#define  GPIO_OUTPUT_MODE      (1UL)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The smallest integer type that can hold the GPIO encoding */

typedef uint32_t gpio_pin_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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

int hpm_configgpio(GPIO_Type *ptr, gpio_pin_t pin, enum gpio_pintype_e type);

/****************************************************************************
 * Name: hpm_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void hpm_gpiowrite(GPIO_Type *ptr, gpio_pin_t pin, bool value);

/****************************************************************************
 * Name: hpm_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool hpm_gpioread(GPIO_Type *ptr, gpio_pin_t pin, uint8_t mode);

/****************************************************************************
 * Function:  hpm6750_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int hpm6750_dumpgpio(gpio_pinset_t pinset, const char *msg);
#else
#define hpm6750_dumpgpio(p, m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_HPMICRO_GPIO_H */
