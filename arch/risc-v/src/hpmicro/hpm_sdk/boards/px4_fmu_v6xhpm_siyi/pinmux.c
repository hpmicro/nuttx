/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/*
 * Note:
 *   PY and PZ IOs: if any SOC pin function needs to be routed to these IOs,
 *  besides of IOC, PIOC/BIOC needs to be configured SOC_GPIO_X_xx, so that
 *  expected SoC function can be enabled on these IOs.
 *
 */

#include "board.h"

void init_uart_pins(UART_Type *ptr)
{
    if (ptr == HPM_UART0) {    /* Only for Console */
        HPM_IOC->PAD[IOC_PAD_PY07].FUNC_CTL = IOC_PY07_FUNC_CTL_UART0_RXD;
        HPM_IOC->PAD[IOC_PAD_PY06].FUNC_CTL = IOC_PY06_FUNC_CTL_UART0_TXD;
        /* PY port IO needs to configure PIOC as well */
        HPM_PIOC->PAD[IOC_PAD_PY07].FUNC_CTL = PIOC_PY07_FUNC_CTL_SOC_PY_07;
        HPM_PIOC->PAD[IOC_PAD_PY06].FUNC_CTL = PIOC_PY06_FUNC_CTL_SOC_PY_06;
    } else if (ptr == HPM_UART1) {
        HPM_IOC->PAD[IOC_PAD_PC18].FUNC_CTL = IOC_PC18_FUNC_CTL_UART1_RXD;
        HPM_IOC->PAD[IOC_PAD_PC19].FUNC_CTL = IOC_PC19_FUNC_CTL_UART1_TXD;
    } else if (ptr == HPM_UART2) {
        HPM_IOC->PAD[IOC_PAD_PC23].FUNC_CTL = IOC_PC23_FUNC_CTL_UART2_RXD;
        HPM_IOC->PAD[IOC_PAD_PC24].FUNC_CTL = IOC_PC24_FUNC_CTL_UART2_TXD;
    } else if (ptr == HPM_UART3) {
        HPM_IOC->PAD[IOC_PAD_PC30].FUNC_CTL = IOC_PC30_FUNC_CTL_UART3_RXD;
        HPM_IOC->PAD[IOC_PAD_PC31].FUNC_CTL = IOC_PC31_FUNC_CTL_UART3_TXD;
    } else if (ptr == HPM_UART5) {
        HPM_IOC->PAD[IOC_PAD_PD06].FUNC_CTL = IOC_PD06_FUNC_CTL_UART5_RXD;
        HPM_IOC->PAD[IOC_PAD_PD07].FUNC_CTL = IOC_PD07_FUNC_CTL_UART5_TXD;
    } else if (ptr == HPM_UART6) {
        HPM_IOC->PAD[IOC_PAD_PC27].FUNC_CTL = IOC_PC27_FUNC_CTL_UART6_RXD;
        HPM_IOC->PAD[IOC_PAD_PC28].FUNC_CTL = IOC_PC28_FUNC_CTL_UART6_TXD;
        HPM_IOC->PAD[IOC_PAD_PD09].FUNC_CTL = IOC_PD09_FUNC_CTL_UART6_CTS;
        HPM_IOC->PAD[IOC_PAD_PD10].FUNC_CTL = IOC_PD10_FUNC_CTL_UART6_RTS;
    } else if (ptr == HPM_UART8) {
        HPM_IOC->PAD[IOC_PAD_PC06].FUNC_CTL = IOC_PC06_FUNC_CTL_UART8_RXD;
        HPM_IOC->PAD[IOC_PAD_PC07].FUNC_CTL = IOC_PC07_FUNC_CTL_UART8_TXD;
        HPM_IOC->PAD[IOC_PAD_PD11].FUNC_CTL = IOC_PD11_FUNC_CTL_UART8_CTS;
        HPM_IOC->PAD[IOC_PAD_PD14].FUNC_CTL = IOC_PD14_FUNC_CTL_UART8_RTS;
    } else if (ptr == HPM_UART9) {
        HPM_IOC->PAD[IOC_PAD_PC11].FUNC_CTL = IOC_PC11_FUNC_CTL_UART9_RXD;
        HPM_IOC->PAD[IOC_PAD_PC12].FUNC_CTL = IOC_PC12_FUNC_CTL_UART9_TXD;
        HPM_IOC->PAD[IOC_PAD_PA12].FUNC_CTL = IOC_PA12_FUNC_CTL_UART9_CTS;
        HPM_IOC->PAD[IOC_PAD_PA13].FUNC_CTL = IOC_PA13_FUNC_CTL_UART9_RTS;
    } else {
        /* Unsupported UART instance */
    }
}

void init_adc12_pins(void)
{
    /* ADC0.VINP14 */
    HPM_IOC->PAD[IOC_PAD_PE28].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
}

void init_adc16_pins(void)
{
    /* ADC3.INA2 */
    HPM_IOC->PAD[IOC_PAD_PE29].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
}

void init_usb_pins(USB_Type *ptr)
{
    ;
}

void init_can_pins(CAN_Type *ptr)
{
    if (ptr == HPM_CAN1) {
        HPM_IOC->PAD[IOC_PAD_PE31].FUNC_CTL = IOC_PE31_FUNC_CTL_CAN1_TXD;
        HPM_IOC->PAD[IOC_PAD_PE30].FUNC_CTL = IOC_PE30_FUNC_CTL_CAN1_RXD;
    }
}

void init_sdxc_cmd_pin(SDXC_Type *ptr, bool open_drain, bool is_1v8)
{
    uint32_t cmd_func_ctl = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(17) | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1);
    uint32_t cmd_pad_ctl = IOC_PAD_PAD_CTL_MS_SET(is_1v8) | IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1) |
                           IOC_PAD_PAD_CTL_PS_SET(1);
    if (open_drain) {
        cmd_pad_ctl |= IOC_PAD_PAD_CTL_OD_MASK;
    }

    if (ptr == HPM_SDXC0) {
        /* SDXC0.CMD */
        HPM_IOC->PAD[IOC_PAD_PE22].FUNC_CTL = cmd_func_ctl;
        HPM_IOC->PAD[IOC_PAD_PE22].PAD_CTL = cmd_pad_ctl;
    }
}

void init_sdxc_cd_pin(SDXC_Type *ptr, bool as_gpio)
{
    ;
}

void init_sdxc_vsel_pin(SDXC_Type *ptr, bool as_gpio)
{
    ;
}

void init_sdxc_clk_data_pins(SDXC_Type *ptr, uint32_t width, bool is_1v8)
{
    uint32_t func_ctl = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(17);
    uint32_t pad_ctl = IOC_PAD_PAD_CTL_MS_SET(is_1v8) | IOC_PAD_PAD_CTL_DS_SET(6) | IOC_PAD_PAD_CTL_PE_SET(1) |
                       IOC_PAD_PAD_CTL_PS_SET(1);

    if (ptr == HPM_SDXC0) {
        /* SDXC0.CLK */
        HPM_IOC->PAD[IOC_PAD_PE27].FUNC_CTL = func_ctl;
        HPM_IOC->PAD[IOC_PAD_PE27].PAD_CTL = pad_ctl;

        /* SDXC0.DATA0 */
        HPM_IOC->PAD[IOC_PAD_PE26].FUNC_CTL = func_ctl;
        HPM_IOC->PAD[IOC_PAD_PE26].PAD_CTL = pad_ctl;
        if (width == 4) {
            /* SDXC0.DATA1 */
            HPM_IOC->PAD[IOC_PAD_PE21].FUNC_CTL = func_ctl;
            HPM_IOC->PAD[IOC_PAD_PE21].PAD_CTL = pad_ctl;
            /* SDXC0.DATA2 */
            HPM_IOC->PAD[IOC_PAD_PE28].FUNC_CTL = func_ctl;
            HPM_IOC->PAD[IOC_PAD_PE28].PAD_CTL = pad_ctl;
            /* SDXC0.DATA3 */
            HPM_IOC->PAD[IOC_PAD_PE23].FUNC_CTL = func_ctl;
            HPM_IOC->PAD[IOC_PAD_PE23].PAD_CTL = pad_ctl;
        }
    }

}

void init_enet_pins(ENET_Type *ptr)
{
    if (ptr == HPM_ENET1) {
        HPM_IOC->PAD[IOC_PAD_PD15].FUNC_CTL = IOC_PD15_FUNC_CTL_GPIO_D_15;

        HPM_IOC->PAD[IOC_PAD_PD11].FUNC_CTL = IOC_PD11_FUNC_CTL_ETH1_MDC;
        HPM_IOC->PAD[IOC_PAD_PD14].FUNC_CTL = IOC_PD14_FUNC_CTL_ETH1_MDIO;

        HPM_IOC->PAD[IOC_PAD_PE18].FUNC_CTL = IOC_PE18_FUNC_CTL_ETH1_RXD_1;
        HPM_IOC->PAD[IOC_PAD_PE19].FUNC_CTL = IOC_PE19_FUNC_CTL_ETH1_TXD_0;
        HPM_IOC->PAD[IOC_PAD_PE20].FUNC_CTL = IOC_PE20_FUNC_CTL_ETH1_RXD_0;

        HPM_IOC->PAD[IOC_PAD_PE14].FUNC_CTL = IOC_PE14_FUNC_CTL_ETH1_TXEN;
        HPM_IOC->PAD[IOC_PAD_PE15].FUNC_CTL = IOC_PE15_FUNC_CTL_ETH1_RXDV;
        HPM_IOC->PAD[IOC_PAD_PE17].FUNC_CTL = IOC_PE17_FUNC_CTL_ETH1_TXD_1;

        HPM_IOC->PAD[IOC_PAD_PE16].FUNC_CTL = IOC_PAD_FUNC_CTL_LOOP_BACK_MASK | IOC_PE16_FUNC_CTL_ETH1_REFCLK;
    }
}

void init_enet_pps_pins(void)
{
    HPM_IOC->PAD[IOC_PAD_PF05].FUNC_CTL = IOC_PF05_FUNC_CTL_ETH0_EVTO_0;
    HPM_IOC->PAD[IOC_PAD_PF06].FUNC_CTL = IOC_PF06_FUNC_CTL_ETH0_EVTO_1;
}

void init_enet_pps_capture_pins(void)
{
    HPM_IOC->PAD[IOC_PAD_PF00].FUNC_CTL = IOC_PF00_FUNC_CTL_ETH0_EVTI_0;
    HPM_IOC->PAD[IOC_PAD_PF01].FUNC_CTL = IOC_PF01_FUNC_CTL_ETH0_EVTI_1;
}

void init_gptmr_channel_pin(GPTMR_Type *ptr, uint32_t channel, bool as_comp)
{
    if (ptr == HPM_GPTMR5) {
        if (as_comp == true) {
            switch (channel) {
            case 0:
                HPM_IOC->PAD[IOC_PAD_PF04].FUNC_CTL = IOC_PF04_FUNC_CTL_GPTMR5_COMP_0;
                break;
            case 1:
                HPM_IOC->PAD[IOC_PAD_PF09].FUNC_CTL = IOC_PF09_FUNC_CTL_GPTMR5_COMP_1;
                break;
            case 2:
                HPM_IOC->PAD[IOC_PAD_PD24].FUNC_CTL = IOC_PD24_FUNC_CTL_TRGM2_P_10;
                trgm_enable_io_output(HPM_TRGM2, 1 << 10);
                trgm_output_t trgm2IoConfig0;
                trgm2IoConfig0.invert = 0;
                trgm2IoConfig0.type = trgm_output_same_as_input;
                trgm2IoConfig0.input = HPM_TRGM2_INPUT_SRC_GPTMR5_OUT2;
                trgm_output_config(HPM_TRGM2, HPM_TRGM2_OUTPUT_SRC_TRGM2_P10, &trgm2IoConfig0);
                break;
            default:
                break;
            }
        } else {
            if (channel == 0) {
                HPM_IOC->PAD[IOC_PAD_PF06].FUNC_CTL = IOC_PF06_FUNC_CTL_GPTMR5_CAPT_1;
            }
        }
    }
}
