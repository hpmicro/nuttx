/*
 * Copyright (c) 2022 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include "hpm_common.h"
#include "hpm_soc.h"
#include "hpm_l1c_drv.h"
#include "hpm_interrupt.h"
#include "hpm_start.h"


extern void system_init(void);


__attribute__((weak)) void _clean_up(void)
{
    /* clean up plic, it will help while debugging */
    disable_irq_from_intc();
    intc_m_set_threshold(0);
    for (uint32_t irq = 0; irq < 128; irq++) {
        intc_m_complete_irq(irq);
    }
    /* clear any bits left in plic enable register */
    for (uint32_t i = 0; i < 4; i++) {
        *(volatile uint32_t *)(HPM_PLIC_BASE + HPM_PLIC_ENABLE_OFFSET + (i << 2)) = 0;
    }
}

__attribute__((weak)) void c_startup(void)
{
    uint32_t i, size;
    extern uint8_t __bss_start__[], __bss_end__[];
    extern uint8_t __tdata_start__[], __tdata_end__[];
    extern uint8_t __data_start__[], __data_end__[];
    extern uint8_t __ramfunc_start__[], __ramfunc_end__[];
    extern uint8_t __noncacheable_bss_start__[], __noncacheable_bss_end__[];
    extern uint8_t __noncacheable_init_start__[], __noncacheable_init_end__[];
    extern uint8_t __data_load_addr__[], __tdata_load_addr__[];
    extern uint8_t __fast_load_addr__[], __noncacheable_init_load_addr__[];

#if defined(FLASH_XIP) || defined(FLASH_UF2)
    extern uint8_t __vector_ram_start__[], __vector_ram_end__[], __vector_load_addr__[];
    size = __vector_ram_end__ - __vector_ram_start__;
    for (i = 0; i < size; i++) {
        *(__vector_ram_start__ + i) = *(__vector_load_addr__ + i);
    }
#endif

    /* bss section */
    size = __bss_end__ - __bss_start__;
    for (i = 0; i < size; i++) {
        *(__bss_start__ + i) = 0;
    }

    /* noncacheable bss section */
    size = __noncacheable_bss_end__ - __noncacheable_bss_start__;
    for (i = 0; i < size; i++) {
        *(__noncacheable_bss_start__ + i) = 0;
    }

    /* data section LMA: etext */
    size = __data_end__ - __data_start__;
    for (i = 0; i < size; i++) {
        *(__data_start__ + i) = *(__data_load_addr__ + i);
    }

    /* ramfunc section LMA: etext + data length */
    size = __ramfunc_end__ - __ramfunc_start__;
    for (i = 0; i < size; i++) {
        *(__ramfunc_start__ + i) = *(__fast_load_addr__ + i);
    }

    /* tdata section LMA: etext + data length + ramfunc length */
    size = __tdata_end__ - __tdata_start__;
    for (i = 0; i < size; i++) {
        *(__tdata_start__ + i) = *(__tdata_load_addr__ + i);
    }

    /* noncacheable init section LMA: etext + data length + ramfunc legnth + tdata length*/
    size = __noncacheable_init_end__ - __noncacheable_init_start__;
    for (i = 0; i < size; i++) {
        *(__noncacheable_init_start__ + i) = *(__noncacheable_init_load_addr__ + i);
    }
}

__attribute__((weak)) int main(void)
{
    while (1) {
        ;
    }
}

__attribute__((weak)) void reset_handler(void)
{
    fencei();

    /* Call platform specific hardware initialization */
    system_init();

    /* Entry function */
    __hpm_start();
}
