/*
 * Copyright (c) 2021-2023 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "hpm_csr_regs.h"


    .extern __trap_vec
    .section .start, "ax"

    .global __start
    .type __start,@function

__start:
    /* Initialize global pointer */
    .option push
    .option norelax
    la gp, __global_pointer$
    la tp, __thread_pointer$
    .option pop

    /* reset mstatus to 0*/
    csrrw x0, mstatus, x0

#ifdef __riscv_flen
    /* Enable FPU */
    li t0, CSR_MSTATUS_FS_MASK
    csrrs t0, mstatus, t0

    /* Initialize FCSR */
    fscsr zero
#endif

    /* Enable LMM1 clock */
    la t0, 0xF4000800
    lw t1, 0(t0)
    ori t1, t1, 0x80
    sw t1, 0(t0)

#ifdef INIT_EXT_RAM_FOR_DATA
    la t0, _stack_safe
    mv sp, t0
    call _init_ext_ram
#endif

    /* Initialize stack pointer */
    la t0, _stack
    mv sp, t0

#ifdef CONFIG_NOT_ENABLE_ICACHE
    call l1c_ic_disable
#else
    call l1c_ic_enable
#endif
#ifdef CONFIG_NOT_ENABLE_DCACHE
    call l1c_dc_invalidate_all
    call l1c_dc_disable
#else
    call l1c_dc_enable
    call l1c_dc_invalidate_all
#endif

    /*
     * Initialize LMA/VMA sections.
     * Relocation for any sections that need to be copied from LMA to VMA.
     */
    call c_startup

    /* Do global constructors */
    call __libc_init_array

#ifndef NO_CLEANUP_AT_START
    /* clean up */
    call _clean_up
#endif

#ifdef __nds_execit
    /* Initialize EXEC.IT table */
    la t0, _ITB_BASE_
    csrw uitb, t0
#endif

    /* Use mscratch to store isr level */
    csrw mscratch, 0

    /* Initial machine trap-vector Base */
    la t0, __trap_vec
    csrw mtvec, t0

    /* Disable vectored external PLIC interrupt */
    csrci CSR_MMISC_CTL, 2

    /* System reset handler */
    call reset_handler

    /* Infinite loop, if returned accidentally */
1:    j 1b
