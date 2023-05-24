/****************************************************************************
 * arch/risc-v/src/hpmicro/hpm_mbx.h
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

#ifndef __ARCH_RISCV_SRC_HPMICRO_HPM_MBX_H
#define __ARCH_RISCV_SRC_HPMICRO_HPM_MBX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#include "hpm_soc.h"
#include "hpm_mbx_drv.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct hpm_mbx_lowerhalf_s
{
  MBX_Type *base;
  mutex_t  mbx_lock;
  uint8_t o_count;
};


/****************************************************************************
 * Public Data
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

/****************************************************************************
 * Name: hpm_mbx_register
 *
 * Description:
 *   hpm_mbx_register the mbx device for hpmicro.
 *
 * Input Parameters:
 *   path - The path to the inode to create
 *   mbx_dev - the lowehalf mbx device
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
#ifdef CONFIG_HPM_MBX
int hpm_mbx_register(FAR const char *path, FAR struct hpm_mbx_lowerhalf_s *mbx_dev);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_HPMICRO_HPM_MBX_H */
