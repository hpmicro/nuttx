/****************************************************************************
 * arch/risc-v/src/hpmicro/hpm_dma.h
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

#ifndef __ARCH_RISCV_SRC_HPMICRO_DMA_H
#define __ARCH_RISCV_SRC_HPMICRO_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#if (defined(CONFIG_HPM_DMA_DRV) || defined(CONFIG_HPM_DMAV2_DRV)) && defined(CONFIG_HPM_COMPONENTS_DMA_MANAGER)

#include "hpm_dma_mgr.h"


#ifndef __ASSEMBLY__
#ifdef __cplusplus
extern "C"
{
#endif


#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif

#endif /* __ARCH_RISCV_SRC_HPMICRO_DMA_H */

