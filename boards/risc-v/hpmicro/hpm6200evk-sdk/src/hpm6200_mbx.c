/****************************************************************************
 * boards/risc-v/hpmicro/hpm6200evk/src/hpm6200_mbx.c
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
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "board.h"
#include "chip.h"
#include "hpm.h"
#include "hpm_mbx.h"

#ifdef CONFIG_HPM_MBX

static struct hpm_mbx_lowerhalf_s g_mbx_a_dev =
{
  .base     = HPM_MBX0A,
  .mbx_lock = NXMUTEX_INITIALIZER,
  .o_count  = 0,
};

static struct hpm_mbx_lowerhalf_s g_mbx_b_dev =
{
  .base     = HPM_MBX0B,
  .mbx_lock = NXMUTEX_INITIALIZER,
  .o_count  = 0,
};

int hpm6200evk_mbxdev_initialize(void)
{
  int ret;
  ret = hpm_mbx_register("/dev/mbx_a", &g_mbx_a_dev);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize /dev/mbx_a Driver: %d\n", ret);
    }
  ret = hpm_mbx_register("/dev/mbx_b", &g_mbx_b_dev);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize /dev/mbx_b Driver: %d\n", ret);
    }
  return ret;
}

#endif
