/****************************************************************************
 * arch/arm/src/hpmicro/hpm_cache.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *s
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
#include <nuttx/cache.h>

#include "hpm_l1c_drv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_enable_icache
 *
 * Description:
 *   Enable the I-Cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ICACHE
void up_enable_icache(void)
{
  l1c_ic_enable();
}
#endif

/****************************************************************************
 * Name: up_disable_icache
 *
 * Description:
 *   Disable the I-Cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ICACHE
void up_disable_icache(void)
{
  l1c_ic_disable();
}
#endif

/****************************************************************************
 * Name: up_invalidate_icache
 *
 * Description:
 *   Invalidate the instruction cache within the specified region.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ICACHE
void up_invalidate_icache(uintptr_t start, uintptr_t end)
{
  uint32_t ssize;

  ssize = end - start;
  l1c_ic_invalidate(start, ssize);
}
#endif /* CONFIG_ARCH_ICACHE */

/****************************************************************************
 * Name: up_invalidate_icache_all
 *
 * Description:
 *   Invalidate the entire contents of I cache.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ICACHE
void up_invalidate_icache_all(void)
{
  l1c_fence_i();
}
#endif

/****************************************************************************
 * Name: up_enable_dcache
 *
 * Description:
 *   Enable the D-Cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_DCACHE
void up_enable_dcache(void)
{
  l1c_dc_enable();
}
#endif /* CONFIG_ARCH_DCACHE */

/****************************************************************************
 * Name: up_disable_dcache
 *
 * Description:
 *   Disable the D-Cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_DCACHE
void up_disable_dcache(void)
{
  l1c_dc_disable();
}
#endif /* CONFIG_ARCH_DCACHE */

/****************************************************************************
 * Name: up_invalidate_dcache
 *
 * Description:
 *   Invalidate the data cache within the specified region; we will be
 *   performing a DMA operation in this region and we want to purge old data
 *   in the cache. Note that this function invalidates all cache ways
 *   in sets that could be associated with the address range, regardless of
 *   whether the address range is contained in the cache or not.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This operation is not atomic.  This function assumes that the caller
 *   has exclusive access to the address range so that no harm is done if
 *   the operation is pre-empted.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_DCACHE
void up_invalidate_dcache(uintptr_t start, uintptr_t end)
{
  uint32_t ssize;

  ssize = end - start;
  l1c_dc_invalidate(start, ssize);
}
#endif /* CONFIG_ARCH_DCACHE */

/****************************************************************************
 * Name: up_invalidate_dcache_all
 *
 * Description:
 *   Invalidate the entire contents of D cache.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_DCACHE
void up_invalidate_dcache_all(void)
{
  l1c_dc_invalidate_all();
}
#endif /* CONFIG_ARCH_DCACHE */

/****************************************************************************
 * Name: up_clean_dcache
 *
 * Description:
 *   Clean the data cache within the specified region by flushing the
 *   contents of the data cache to memory.
 *
 *   NOTE: This operation is un-necessary if the DCACHE is configured in
 *   write-through mode.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This operation is not atomic.  This function assumes that the caller
 *   has exclusive access to the address range so that no harm is done if
 *   the operation is pre-empted.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_DCACHE
void up_clean_dcache(uintptr_t start, uintptr_t end)
{
#ifndef CONFIG_ARCH_DCACHE_WRITETHROUGH
  uint32_t ssize;

  ssize = end - start;
  l1c_dc_writeback(start, ssize);
#endif /* !CONFIG_ARCH_DCACHE_WRITETHROUGH */
}
#endif /* CONFIG_ARCH_DCACHE */

/****************************************************************************
 * Name: up_clean_dcache_all
 *
 * Description:
 *   Clean the entire data cache within the specified region by flushing the
 *   contents of the data cache to memory.
 *
 *   NOTE: This operation is un-necessary if the DCACHE is configured in
 *   write-through mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This operation is not atomic.  This function assumes that the caller
 *   has exclusive access to the address range so that no harm is done if
 *   the operation is pre-empted.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_DCACHE
void up_clean_dcache_all(void)
{
#ifndef CONFIG_ARCH_DCACHE_WRITETHROUGH
  l1c_dc_writeback_all();
#endif /* !CONFIG_ARCH_DCACHE_WRITETHROUGH */
}
#endif /* CONFIG_ARCH_DCACHE */

/****************************************************************************
 * Name: up_flush_dcache
 *
 * Description:
 *   Flush the data cache within the specified region by cleaning and
 *   invalidating the D cache.
 *
 *   NOTE: If DCACHE write-through is configured, then this operation is the
 *   same as up_invalidate_cache().
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This operation is not atomic.  This function assumes that the caller
 *   has exclusive access to the address range so that no harm is done if
 *   the operation is pre-empted.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_DCACHE
void up_flush_dcache(uintptr_t start, uintptr_t end)
{
#ifndef CONFIG_ARCH_DCACHE_WRITETHROUGH
  uint32_t ssize;

  ssize = end - start;
  l1c_dc_flush(start, ssize);
#else
  up_invalidate_dcache(start, end);
#endif /* !CONFIG_ARCH_DCACHE_WRITETHROUGH */
}
#endif /* CONFIG_ARCH_DCACHE */

/****************************************************************************
 * Name: up_flush_dcache_all
 *
 * Description:
 *   Flush the entire data cache by cleaning and invalidating the D cache.
 *
 *   NOTE: If DCACHE write-through is configured, then this operation is the
 *   same as up_invalidate_cache_all().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This operation is not atomic.  This function assumes that the caller
 *   has exclusive access to the address range so that no harm is done if
 *   the operation is pre-empted.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_DCACHE
void up_flush_dcache_all(void)
{
#ifndef CONFIG_ARCH_DCACHE_WRITETHROUGH
  l1c_dc_flush_all();
#else
  up_invalidate_dcache_all();
#endif /* !CONFIG_ARCH_DCACHE_WRITETHROUGH */
}
#endif /* CONFIG_ARCH_DCACHE */

/****************************************************************************
 * Name: up_coherent_dcache
 *
 * Description:
 *   Ensure that the I and D caches are coherent within specified region
 *   by cleaning the D cache (i.e., flushing the D cache contents to memory
 *   and invalidating the I cache. This is typically used when code has been
 *   written to a memory region, and will be executed.
 *
 * Input Parameters:
 *   addr - virtual start address of region
 *   len  - Size of the address region in bytes
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_ICACHE
void up_coherent_dcache(uintptr_t addr, size_t len)
{
  uintptr_t end;

  if (len > 0)
    {
      /* Flush any dirtcy D-Cache lines to memory */

      end = addr + len;
      up_clean_dcache(addr, end);
      UNUSED(end);

      /* Invalidate the entire I-Cache */

      up_invalidate_icache_all();
    }
}
#endif
