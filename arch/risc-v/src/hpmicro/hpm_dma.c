/****************************************************************************
 * arch/risc-v/src/hpmicro/hpm_dma.c
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
#include <nuttx/arch.h>

#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include "chip.h"
#include "riscv_internal.h"
#include "hpm_dma.h"

#if defined(CONFIG_HPM_DMA_DRV) && defined(CONFIG_HPM_COMPONENTS_DMA_MANAGER)


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_dmainterrupt
 *
 * Description:
 *  DMA interrupt handler
 *
 ****************************************************************************/

static int hpm_dmainterrupt(int irq, void *context, void *arg)
{
  if (irq == HPM_IRQn_HDMA)
  {
    dma_mgr_isr_handler(HPM_HDMA, 0);
  } else if (irq  == HPM_IRQn_XDMA)
  {
    dma_mgr_isr_handler(HPM_XDMA, 1);
  }
  return 0;
}

/****************************************************************************
 * Name: riscv_dma_initialize
 *
 * Description:
 *   Initialize DMA driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void weak_function riscv_dma_initialize(void)
{
  dma_mgr_init();

  /* Attach DMA interrupt vectors */

  irq_attach(HPM_IRQn_HDMA, hpm_dmainterrupt, NULL);
  irq_attach(HPM_IRQn_XDMA, hpm_dmainterrupt, NULL);

  /* Enable the IRQ at the NVIC (still disabled at the DMA controller) */

  up_enable_irq(HPM_IRQn_HDMA);
  up_enable_irq(HPM_IRQn_XDMA);

}

#endif
