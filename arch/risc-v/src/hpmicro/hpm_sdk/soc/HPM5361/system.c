/*
 * Copyright (c) 2021 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */


#include "hpm_common.h"
#include "hpm_soc.h"
#include "hpm_l1c_drv.h"

__attribute__((weak)) void system_init(void)
{
#ifndef CONFIG_NOT_ENALBE_ACCESS_TO_CYCLE_CSR
    uint32_t mcounteren = read_csr(CSR_MCOUNTEREN);
    write_csr(CSR_MCOUNTEREN, mcounteren | 1); /* Enable MCYCLE */
#endif
}
