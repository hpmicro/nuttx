/****************************************************************************
 * boards/risc-v/hpmicro/hpm6750evk2-sdk/src/hpm6750_sdmmc.c
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

#include <debug.h>
#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/compiler.h>
#include <nuttx/sdio.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/mmcsd.h>
#include <nuttx/irq.h>
#include <nuttx/cache.h>
#include <arch/board/board.h>

#include "hpm_sdmmc.h"

static struct sdio_dev_s *g_sdio_dev;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_sdio_initialize
 *
 * Description:
 *   Called to configure SDXC
 *
 ****************************************************************************/
#if defined(CONFIG_HPM_SDXC0) || defined(CONFIG_HPM_SDXC1)
int hpm_sdio_initialize(void)
{
    struct sdio_dev_s *dev = NULL;
    int ret;
#ifdef CONFIG_HPM_SDXC0
    /* Initialize SDXC0 */
    dev = sdio_initialize(0);
    if (dev == NULL)
    {
      syslog(LOG_ERR, "ERROR: sdio_initialize(0) failed\n");\
      return -EINVAL;
    }

    g_sdio_dev = dev;
    ret = mmcsd_slotinitialize(0, g_sdio_dev);
    if (ret != OK)
    {
      ferr("ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
      return ret;
    }
#endif
#ifdef CONFIG_HPM_SDXC1
    /* Initialize SDXC1 */
    dev = sdio_initialize(1);
    if (dev == NULL)
    {
      syslog(LOG_ERR, "ERROR: sdio_initialize(1) failed\n");
      return -EINVAL;
    }
    g_sdio_dev = dev;
    ret = mmcsd_slotinitialize(1, g_sdio_dev);
    if (ret != OK)
    {
      ferr("ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
      return ret;
    }
#endif

    sdio_mediachange(g_sdio_dev, true);
    
    return ret;
}
#endif

