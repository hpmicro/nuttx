/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef _HPM_BOARD_H
#define _HPM_BOARD_H
#include <stdio.h>
#include "hpm_common.h"
#include "hpm_soc.h"
#include "hpm_soc_feature.h"
#include "hpm_clock_drv.h"
#include "pinmux.h"
#include "hpm_lcdc_drv.h"
#include "hpm_trgm_drv.h"
#ifdef CONFIG_HPM_PANEL
#include "hpm_panel.h"
#endif
#if !defined(CONFIG_NDEBUG_CONSOLE) || !CONFIG_NDEBUG_CONSOLE
#include "hpm_debug_console.h"
#endif

#define BOARD_NAME          "px4_fmu_v6xhpm_siyi"
#define BOARD_UF2_SIGNATURE (0x0A4D5048UL)

#define BOARD_CPU_FREQ (816000000UL)

#define SEC_CORE_IMG_START ILM_LOCAL_BASE

#ifndef BOARD_RUNNING_CORE
#define BOARD_RUNNING_CORE HPM_CORE0
#endif

#if !defined(CONFIG_NDEBUG_CONSOLE) || !CONFIG_NDEBUG_CONSOLE
#ifndef BOARD_CONSOLE_TYPE
#define BOARD_CONSOLE_TYPE CONSOLE_TYPE_UART
#endif

#if BOARD_CONSOLE_TYPE == CONSOLE_TYPE_UART
#ifndef BOARD_CONSOLE_UART_BASE
#if BOARD_RUNNING_CORE == HPM_CORE0
#define BOARD_CONSOLE_UART_BASE       HPM_UART0
#define BOARD_CONSOLE_UART_CLK_NAME   clock_uart0
#define BOARD_CONSOLE_UART_IRQ        IRQn_UART0
#define BOARD_CONSOLE_UART_TX_DMA_REQ HPM_DMA_SRC_UART0_TX
#define BOARD_CONSOLE_UART_RX_DMA_REQ HPM_DMA_SRC_UART0_RX
#else
#define BOARD_CONSOLE_UART_BASE       HPM_UART13
#define BOARD_CONSOLE_UART_CLK_NAME   clock_uart13
#define BOARD_CONSOLE_UART_IRQ        IRQn_UART13
#define BOARD_CONSOLE_UART_TX_DMA_REQ HPM_DMA_SRC_UART13_TX
#define BOARD_CONSOLE_UART_RX_DMA_REQ HPM_DMA_SRC_UART13_RX
#endif
#endif
#define BOARD_CONSOLE_UART_BAUDRATE (115200UL)
#endif
#endif

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

void board_init_console(void);
void board_ungate_mchtmr_at_lp_mode(void);
uint32_t board_init_i2c_clock(I2C_Type *ptr);
uint32_t board_init_uart_clock(UART_Type *ptr);
uint32_t board_init_spi_clock(SPI_Type *ptr);
void board_init_usb(USB_Type *ptr);
void board_init_pmp(void);
void board_init_clock(void);
void board_init_adc12_pins(void);
void board_init_adc16_pins(void);
uint32_t board_init_adc_clock(void *ptr, bool clk_src_bus);
void board_init_can(CAN_Type *ptr);
uint32_t board_init_can_clock(CAN_Type *ptr);
uint32_t board_sd_configure_clock(SDXC_Type *ptr, uint32_t freq, bool need_inverse);
uint32_t board_init_gptmr_clock(GPTMR_Type *ptr);

#if defined(__cplusplus)
}
#endif /* __cplusplus */
#endif /* _HPM_BOARD_H */
