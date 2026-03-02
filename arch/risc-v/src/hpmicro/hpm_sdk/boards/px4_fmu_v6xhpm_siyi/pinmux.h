/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef HPM_PINMUX_H
#define HPM_PINMUX_H
#include "hpm_soc.h"

#ifdef __cplusplus
extern "C" {
#endif

void init_uart_pins(UART_Type *ptr);
void init_adc12_pins(void);
void init_adc16_pins(void);
void init_usb_pins(USB_Type *ptr);
void init_can_pins(CAN_Type *ptr);
void init_sdxc_cmd_pin(SDXC_Type *ptr, bool open_drain, bool is_1v8);
void init_sdxc_cd_pin(SDXC_Type *ptr, bool as_gpio);
void init_sdxc_vsel_pin(SDXC_Type *ptr, bool as_gpio);
void init_sdxc_clk_data_pins(SDXC_Type *ptr, uint32_t width, bool is_1v8);
void init_enet_pins(ENET_Type *ptr);
void init_enet_pps_pins(void);
void init_enet_pps_capture_pins(void);
void init_gptmr_channel_pin(GPTMR_Type *ptr, uint32_t channel, bool as_comp);

#ifdef __cplusplus
}
#endif
#endif /* HPM_PINMUX_H */
