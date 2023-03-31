/****************************************************************************
 * arch/risc-v/src/hpmicro/hpm6750/hpm_tim_lowerhalf.c
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

#ifndef __ARCH_RISCV_SRC_HPMICRO_HPM6750_HPM_TIM_LOWERHALF_H
#define __ARCH_RISCV_SRC_HPMICRO_HPM6750_HPM_TIM_LOWERHALF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

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

#if defined(CONFIG_TIMER) && \
    (defined(CONFIG_HPM6750_TIMER0)  || defined(CONFIG_HPM6750_TIMER1)  || \
     defined(CONFIG_HPM6750_TIMER2)  || defined(CONFIG_HPM6750_TIMER3)  || \
     defined(CONFIG_HPM6750_TIMER4)  || defined(CONFIG_HPM6750_TIMER5)  || \
     defined(CONFIG_HPM6750_TIMER6)  || defined(CONFIG_HPM6750_TIMER7))

/****************************************************************************
 * Name: hpm_timer_initialize
 ****************************************************************************/

int hpm_timer_initialize(const char *devpath, int timer);

#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_HPMICRO_HPM6750_HPM_TIM_LOWERHALF_H */
