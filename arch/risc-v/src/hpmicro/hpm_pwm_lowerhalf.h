/****************************************************************************
 * arch/risc-v/src/hpmicro/common/hpm_pwm_lowerhalf.c
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

#ifndef __ARCH_RISCV_SRC_HPMICRO_COMMON_HPM_PWM_LOWERHALF_H
#define __ARCH_RISCV_SRC_HPMICRO_COMMON_HPM_PWM_LOWERHALF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/timers/pwm.h>

#include "chip.h"

/****************************************************************************
 * Public Function Prototypes
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

#if defined(CONFIG_PWM) && defined(CONFIG_HPM_PWM_DRV)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_init_pwm_pins
 *
 * Description:
 *   Initialize the selected PWM channel pins.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

int hpm_init_pwm_pins(int port);

/****************************************************************************
 * Name: hpm_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   pwm - A number identifying the pwm instance.
 *
 * Returned Value:
 *   On success, a pointer to the HPM lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *hpm_pwminitialize(int pwm);

#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_HPMICRO_COMMON_HPM_PWM_LOWERHALF_H */
