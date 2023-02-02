/*
 * Copyright (c) 2021 HPMicro
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
    if (ptr == HPM_UART0) {
        HPM_IOC->PAD[IOC_PAD_PY07].FUNC_CTL = IOC_PY07_FUNC_CTL_UART0_RXD;
        HPM_IOC->PAD[IOC_PAD_PY06].FUNC_CTL = IOC_PY06_FUNC_CTL_UART0_TXD;
        /* PY port IO needs to configure PIOC as well */
        HPM_PIOC->PAD[IOC_PAD_PY07].FUNC_CTL = IOC_PY06_FUNC_CTL_SOC_PY_06;
        HPM_PIOC->PAD[IOC_PAD_PY06].FUNC_CTL = IOC_PY07_FUNC_CTL_SOC_PY_07;
    } else if (ptr == HPM_UART2) {
        HPM_IOC->PAD[IOC_PAD_PE16].FUNC_CTL = IOC_PE16_FUNC_CTL_UART2_TXD;
        HPM_IOC->PAD[IOC_PAD_PE21].FUNC_CTL = IOC_PE21_FUNC_CTL_UART2_RXD;
    } else if (ptr == HPM_UART13) {
        HPM_IOC->PAD[IOC_PAD_PZ08].FUNC_CTL = IOC_PZ08_FUNC_CTL_UART13_RXD;
        HPM_IOC->PAD[IOC_PAD_PZ09].FUNC_CTL = IOC_PZ09_FUNC_CTL_UART13_TXD;
        /* PZ port IO needs to configure BIOC as well */
        HPM_BIOC->PAD[IOC_PAD_PZ08].FUNC_CTL = IOC_PZ08_FUNC_CTL_SOC_PZ_08;
        HPM_BIOC->PAD[IOC_PAD_PZ09].FUNC_CTL = IOC_PZ09_FUNC_CTL_SOC_PZ_09;
    } else if (ptr == HPM_PUART) {
        HPM_PIOC->PAD[IOC_PAD_PY07].FUNC_CTL = IOC_PY07_FUNC_CTL_PUART_RXD;
        HPM_PIOC->PAD[IOC_PAD_PY06].FUNC_CTL = IOC_PY06_FUNC_CTL_PUART_TXD;
    }
}

void init_lcd_pins(LCDC_Type *ptr)
{
    HPM_IOC->PAD[IOC_PAD_PB03].FUNC_CTL = IOC_PB03_FUNC_CTL_DIS0_R_0;
    HPM_IOC->PAD[IOC_PAD_PB04].FUNC_CTL = IOC_PB04_FUNC_CTL_DIS0_R_1;
    HPM_IOC->PAD[IOC_PAD_PB00].FUNC_CTL = IOC_PB00_FUNC_CTL_DIS0_R_2;
    HPM_IOC->PAD[IOC_PAD_PA31].FUNC_CTL = IOC_PA31_FUNC_CTL_DIS0_R_3;
    HPM_IOC->PAD[IOC_PAD_PA26].FUNC_CTL = IOC_PA26_FUNC_CTL_DIS0_R_4;
    HPM_IOC->PAD[IOC_PAD_PA21].FUNC_CTL = IOC_PA21_FUNC_CTL_DIS0_R_5;
    HPM_IOC->PAD[IOC_PAD_PA27].FUNC_CTL = IOC_PA27_FUNC_CTL_DIS0_R_6;
    HPM_IOC->PAD[IOC_PAD_PA28].FUNC_CTL = IOC_PA28_FUNC_CTL_DIS0_R_7;
    HPM_IOC->PAD[IOC_PAD_PB06].FUNC_CTL = IOC_PB06_FUNC_CTL_DIS0_G_0;
    HPM_IOC->PAD[IOC_PAD_PB01].FUNC_CTL = IOC_PB01_FUNC_CTL_DIS0_G_1;
    HPM_IOC->PAD[IOC_PAD_PA22].FUNC_CTL = IOC_PA22_FUNC_CTL_DIS0_G_2;
    HPM_IOC->PAD[IOC_PAD_PA23].FUNC_CTL = IOC_PA23_FUNC_CTL_DIS0_G_3;
    HPM_IOC->PAD[IOC_PAD_PA29].FUNC_CTL = IOC_PA29_FUNC_CTL_DIS0_G_4;
    HPM_IOC->PAD[IOC_PAD_PA24].FUNC_CTL = IOC_PA24_FUNC_CTL_DIS0_G_5;
    HPM_IOC->PAD[IOC_PAD_PA30].FUNC_CTL = IOC_PA30_FUNC_CTL_DIS0_G_6;
    HPM_IOC->PAD[IOC_PAD_PA25].FUNC_CTL = IOC_PA25_FUNC_CTL_DIS0_G_7;
    HPM_IOC->PAD[IOC_PAD_PB05].FUNC_CTL = IOC_PB05_FUNC_CTL_DIS0_B_0;
    HPM_IOC->PAD[IOC_PAD_PB07].FUNC_CTL = IOC_PB07_FUNC_CTL_DIS0_B_1;
    HPM_IOC->PAD[IOC_PAD_PB02].FUNC_CTL = IOC_PB02_FUNC_CTL_DIS0_B_2;
    HPM_IOC->PAD[IOC_PAD_PA16].FUNC_CTL = IOC_PA16_FUNC_CTL_DIS0_B_3;
    HPM_IOC->PAD[IOC_PAD_PA12].FUNC_CTL = IOC_PA12_FUNC_CTL_DIS0_B_4;
    HPM_IOC->PAD[IOC_PAD_PA17].FUNC_CTL = IOC_PA17_FUNC_CTL_DIS0_B_5;
    HPM_IOC->PAD[IOC_PAD_PA13].FUNC_CTL = IOC_PA13_FUNC_CTL_DIS0_B_6;
    HPM_IOC->PAD[IOC_PAD_PA18].FUNC_CTL = IOC_PA18_FUNC_CTL_DIS0_B_7;
    HPM_IOC->PAD[IOC_PAD_PA20].FUNC_CTL = IOC_PA20_FUNC_CTL_DIS0_CLK;
    HPM_IOC->PAD[IOC_PAD_PA15].FUNC_CTL = IOC_PA15_FUNC_CTL_DIS0_EN;
    HPM_IOC->PAD[IOC_PAD_PA19].FUNC_CTL = IOC_PA19_FUNC_CTL_DIS0_HSYNC;
    HPM_IOC->PAD[IOC_PAD_PA14].FUNC_CTL = IOC_PA14_FUNC_CTL_DIS0_VSYNC;

    /* PWM */
    HPM_IOC->PAD[IOC_PAD_PB10].FUNC_CTL = IOC_PB10_FUNC_CTL_GPIO_B_10;
    /* RST */
    HPM_IOC->PAD[IOC_PAD_PB16].FUNC_CTL = IOC_PB16_FUNC_CTL_GPIO_B_16;

    HPM_IOC->PAD[IOC_PAD_PZ00].FUNC_CTL = IOC_PZ00_FUNC_CTL_GPIO_Z_00;
    HPM_IOC->PAD[IOC_PAD_PZ00].PAD_CTL = IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_PE_SET(1);
    HPM_BIOC->PAD[IOC_PAD_PZ00].FUNC_CTL = IOC_PZ00_FUNC_CTL_SOC_PZ_00;
}

void init_cap_pins(void)
{
    /* CAP_INT */
    HPM_IOC->PAD[IOC_PAD_PB08].PAD_CTL = IOC_PAD_PAD_CTL_PE_MASK;
    HPM_IOC->PAD[IOC_PAD_PB08].FUNC_CTL = IOC_PB08_FUNC_CTL_GPIO_B_08;
    /* CAP_RST */
    HPM_IOC->PAD[IOC_PAD_PB09].PAD_CTL = IOC_PAD_PAD_CTL_PE_MASK;
    HPM_IOC->PAD[IOC_PAD_PB09].FUNC_CTL = IOC_PB09_FUNC_CTL_GPIO_B_09;
}

void init_trgmux_pins(uint32_t pin)
{
    /* all trgmux pin ALT_SELECT fixed to 16*/
    HPM_IOC->PAD[pin].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(16);
}

void init_i2c_pins_as_gpio(I2C_Type *ptr)
{
    if (ptr == HPM_I2C0) {
        /* I2C0 */
        HPM_IOC->PAD[IOC_PAD_PZ11].FUNC_CTL = IOC_PZ11_FUNC_CTL_GPIO_Z_11;
        HPM_IOC->PAD[IOC_PAD_PZ10].FUNC_CTL = IOC_PZ10_FUNC_CTL_GPIO_Z_10;
        /* PZ port IO needs to configure BIOC as well */
        HPM_BIOC->PAD[IOC_PAD_PZ11].FUNC_CTL = 3;
        HPM_BIOC->PAD[IOC_PAD_PZ10].FUNC_CTL = 3;
    } else {
        while (1) {
        }
    }
}

void init_i2c_pins(I2C_Type *ptr)
{
    if (ptr == HPM_I2C0) {
        HPM_IOC->PAD[IOC_PAD_PZ11].FUNC_CTL = IOC_PB11_FUNC_CTL_I2C0_SCL
                                            | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
        HPM_IOC->PAD[IOC_PAD_PZ10].FUNC_CTL = IOC_PB10_FUNC_CTL_I2C0_SDA
                                            | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
        /* PZ port IO needs to configure BIOC as well */
        HPM_BIOC->PAD[IOC_PAD_PZ11].FUNC_CTL = 3;
        HPM_BIOC->PAD[IOC_PAD_PZ10].FUNC_CTL = 3;
        HPM_IOC->PAD[IOC_PAD_PZ11].PAD_CTL = IOC_PAD_PAD_CTL_OD_MASK;
        HPM_IOC->PAD[IOC_PAD_PZ10].PAD_CTL = IOC_PAD_PAD_CTL_OD_MASK;
    } else {
        while (1) {
        }
    }
}

void init_sdram_pins(void)
{
    HPM_IOC->PAD[IOC_PAD_PC01].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC00].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB31].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB30].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB29].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB28].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB27].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB26].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB25].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB24].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB23].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB22].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB21].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB20].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB19].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PB18].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);

    HPM_IOC->PAD[IOC_PAD_PD13].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PD12].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PD10].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PD09].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PD08].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PD07].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PD06].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PD05].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PD04].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PD03].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PD02].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PD01].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PD00].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC29].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC28].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC27].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);

    HPM_IOC->PAD[IOC_PAD_PC21].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC17].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC15].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC12].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC11].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC10].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC09].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC08].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC07].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC06].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC05].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC04].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);

    HPM_IOC->PAD[IOC_PAD_PC14].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC13].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC16].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12) | IOC_PAD_FUNC_CTL_LOOP_BACK_MASK;
    HPM_IOC->PAD[IOC_PAD_PC26].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC25].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC19].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC18].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC23].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC24].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC30].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC31].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC02].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
    HPM_IOC->PAD[IOC_PAD_PC03].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);
}

void init_sram_pins(void)
{
                                                                                /* Non-MUX */         /* MUX */
    HPM_IOC->PAD[IOC_PAD_PC08].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* A0 */            /* A16 */
    HPM_IOC->PAD[IOC_PAD_PC09].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* A1 */            /* A17 */
    HPM_IOC->PAD[IOC_PAD_PC04].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* A2 */            /* A18 */
    HPM_IOC->PAD[IOC_PAD_PC05].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* A3 */            /* A19 */
    HPM_IOC->PAD[IOC_PAD_PC06].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* A4 */            /* A20 */
    HPM_IOC->PAD[IOC_PAD_PC07].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* A5 */            /* A21 */
    HPM_IOC->PAD[IOC_PAD_PC10].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* A6 */            /* A22 */
    HPM_IOC->PAD[IOC_PAD_PC11].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* A7 */            /* A23 */
    HPM_IOC->PAD[IOC_PAD_PC01].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* A8 */
    HPM_IOC->PAD[IOC_PAD_PC00].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* A9 */
    HPM_IOC->PAD[IOC_PAD_PB31].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* A10 */
    HPM_IOC->PAD[IOC_PAD_PB28].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* A11 */
    HPM_IOC->PAD[IOC_PAD_PB27].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* A12 */
    HPM_IOC->PAD[IOC_PAD_PB26].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* A13 */
    HPM_IOC->PAD[IOC_PAD_PB23].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* A14 */
    HPM_IOC->PAD[IOC_PAD_PB20].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* A15 */
    HPM_IOC->PAD[IOC_PAD_PB19].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* A16 */
    HPM_IOC->PAD[IOC_PAD_PB18].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* A17 */
    HPM_IOC->PAD[IOC_PAD_PB22].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* A18 */
    HPM_IOC->PAD[IOC_PAD_PB21].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* A19 */
    HPM_IOC->PAD[IOC_PAD_PB25].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* A20 */
    HPM_IOC->PAD[IOC_PAD_PB24].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* A21 */
    HPM_IOC->PAD[IOC_PAD_PB30].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* A22 */
    HPM_IOC->PAD[IOC_PAD_PB29].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* A23 */

    HPM_IOC->PAD[IOC_PAD_PD08].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* D0 */             /* AD0 */
    HPM_IOC->PAD[IOC_PAD_PD05].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* D1 */             /* AD1 */
    HPM_IOC->PAD[IOC_PAD_PD00].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* D2 */             /* AD2 */
    HPM_IOC->PAD[IOC_PAD_PD01].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* D3 */             /* AD3 */
    HPM_IOC->PAD[IOC_PAD_PD02].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* D4 */             /* AD4 */
    HPM_IOC->PAD[IOC_PAD_PC27].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* D5 */             /* AD5 */
    HPM_IOC->PAD[IOC_PAD_PC28].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* D6 */             /* AD6 */
    HPM_IOC->PAD[IOC_PAD_PC29].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* D7 */             /* AD7 */
    HPM_IOC->PAD[IOC_PAD_PD04].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* D8 */             /* AD8 */
    HPM_IOC->PAD[IOC_PAD_PD03].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* D9 */             /* AD9 */
    HPM_IOC->PAD[IOC_PAD_PD07].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* D10 */            /* AD10 */
    HPM_IOC->PAD[IOC_PAD_PD06].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* D11 */            /* AD11 */
    HPM_IOC->PAD[IOC_PAD_PD10].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* D12 */            /* AD12 */
    HPM_IOC->PAD[IOC_PAD_PD09].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* D13 */            /* AD13 */
    HPM_IOC->PAD[IOC_PAD_PD13].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* D14 */            /* AD14 */
    HPM_IOC->PAD[IOC_PAD_PD12].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* D15 */            /* AD15 */

    HPM_IOC->PAD[IOC_PAD_PC20].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* #CE */
    HPM_IOC->PAD[IOC_PAD_PC22].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* #OE */
    HPM_IOC->PAD[IOC_PAD_PC21].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* #WE */
    HPM_IOC->PAD[IOC_PAD_PC31].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* #UB */
    HPM_IOC->PAD[IOC_PAD_PC30].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* #LB */
    HPM_IOC->PAD[IOC_PAD_PC14].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(12);    /* #ADV */
}

void init_gpio_pins(void)
{
    HPM_IOC->PAD[IOC_PAD_PB12].FUNC_CTL = IOC_PB12_FUNC_CTL_GPIO_B_12;
    HPM_IOC->PAD[IOC_PAD_PB12].PAD_CTL = IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(0);

#ifdef USING_GPIO0_FOR_GPIOZ
    HPM_IOC->PAD[IOC_PAD_PZ02].FUNC_CTL = IOC_PZ02_FUNC_CTL_GPIO_Z_02;
    HPM_IOC->PAD[IOC_PAD_PZ02].PAD_CTL = IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
    /* PZ port IO needs to configure BIOC as well */
    HPM_BIOC->PAD[IOC_PAD_PZ02].FUNC_CTL = IOC_PZ02_FUNC_CTL_SOC_PZ_02;
#endif
}

void init_spi_pins(SPI_Type *ptr)
{
    if (ptr == HPM_SPI2) {
        HPM_IOC->PAD[IOC_PAD_PE31].FUNC_CTL = IOC_PE31_FUNC_CTL_SPI2_CSN;
        HPM_IOC->PAD[IOC_PAD_PE30].FUNC_CTL = IOC_PE30_FUNC_CTL_SPI2_MOSI;
        HPM_IOC->PAD[IOC_PAD_PE28].FUNC_CTL = IOC_PE28_FUNC_CTL_SPI2_MISO;
        HPM_IOC->PAD[IOC_PAD_PE27].FUNC_CTL = IOC_PE27_FUNC_CTL_SPI2_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1);
    }
}

void init_spi_pins_with_gpio_as_cs(SPI_Type *ptr)
{
    if (ptr == HPM_SPI2) {
        HPM_IOC->PAD[IOC_PAD_PE31].FUNC_CTL = IOC_PE31_FUNC_CTL_GPIO_E_31;
        HPM_IOC->PAD[IOC_PAD_PE30].FUNC_CTL = IOC_PE30_FUNC_CTL_SPI2_MOSI;
        HPM_IOC->PAD[IOC_PAD_PE28].FUNC_CTL = IOC_PE28_FUNC_CTL_SPI2_MISO;
        HPM_IOC->PAD[IOC_PAD_PE27].FUNC_CTL = IOC_PE27_FUNC_CTL_SPI2_SCLK | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1);
    }
}

void init_pins(void)
{
    init_uart_pins(BOARD_CONSOLE_BASE);
    init_sdram_pins();
}

void init_gptmr_pins(GPTMR_Type *ptr)
{
    if (ptr == HPM_GPTMR3) {
        /* TMR3 compare 1 */
        HPM_IOC->PAD[IOC_PAD_PE24].FUNC_CTL = IOC_PE24_FUNC_CTL_GPTMR3_COMP_1;
    }
    if (ptr == HPM_GPTMR4) {
        /* TMR4 capture 1 */
        HPM_IOC->PAD[IOC_PAD_PE25].FUNC_CTL = IOC_PE25_FUNC_CTL_GPTMR4_CAPT_1;
    }
}

void init_hall_trgm_pins(void)
{
    HPM_IOC->PAD[IOC_PAD_PD16].FUNC_CTL = IOC_PD16_FUNC_CTL_TRGM2_P_06;
    HPM_IOC->PAD[IOC_PAD_PD20].FUNC_CTL = IOC_PD20_FUNC_CTL_TRGM2_P_07;
    HPM_IOC->PAD[IOC_PAD_PD25].FUNC_CTL = IOC_PD25_FUNC_CTL_TRGM2_P_08;
}

void init_qei_trgm_pins(void)
{
    HPM_IOC->PAD[IOC_PAD_PD19].FUNC_CTL = IOC_PD19_FUNC_CTL_TRGM2_P_09;
    HPM_IOC->PAD[IOC_PAD_PD24].FUNC_CTL = IOC_PD24_FUNC_CTL_TRGM2_P_10;
}

void init_i2s_pins(I2S_Type *ptr)
{
    if (ptr == HPM_I2S0) {
        HPM_IOC->PAD[IOC_PAD_PF02].FUNC_CTL = IOC_PF02_FUNC_CTL_I2S0_RXD_2;
        HPM_IOC->PAD[IOC_PAD_PF03].FUNC_CTL = IOC_PF03_FUNC_CTL_I2S0_MCLK;
        HPM_IOC->PAD[IOC_PAD_PF04].FUNC_CTL = IOC_PF04_FUNC_CTL_I2S0_TXD_2;
        HPM_IOC->PAD[IOC_PAD_PF06].FUNC_CTL = IOC_PF06_FUNC_CTL_I2S0_BCLK;
        HPM_IOC->PAD[IOC_PAD_PF09].FUNC_CTL = IOC_PF09_FUNC_CTL_I2S0_FCLK;
    }
}

void init_dao_pins(void)
{
    HPM_IOC->PAD[IOC_PAD_PY08].FUNC_CTL = IOC_PY08_FUNC_CTL_DAOR_P;
    HPM_IOC->PAD[IOC_PAD_PY09].FUNC_CTL = IOC_PY09_FUNC_CTL_DAOR_N;
    /* PY port IO needs to configure PIOC */
    HPM_PIOC->PAD[IOC_PAD_PY08].FUNC_CTL = IOC_PY08_FUNC_CTL_SOC_PY_08;
    HPM_PIOC->PAD[IOC_PAD_PY09].FUNC_CTL = IOC_PY09_FUNC_CTL_SOC_PY_09;
}

void init_pdm_pins(void)
{
    HPM_IOC->PAD[IOC_PAD_PY10].FUNC_CTL = IOC_PY10_FUNC_CTL_PDM0_CLK;
    HPM_IOC->PAD[IOC_PAD_PY11].FUNC_CTL = IOC_PY11_FUNC_CTL_PDM0_D_0;
    /* PY port IO needs to configure PIOC */
    HPM_PIOC->PAD[IOC_PAD_PY10].FUNC_CTL = IOC_PY10_FUNC_CTL_SOC_PY_10;
    HPM_PIOC->PAD[IOC_PAD_PY11].FUNC_CTL = IOC_PY11_FUNC_CTL_SOC_PY_11;
}

void init_vad_pins(void)
{
    HPM_PIOC->PAD[IOC_PAD_PY10].FUNC_CTL = IOC_PY10_FUNC_CTL_VAD_CLK;
    HPM_PIOC->PAD[IOC_PAD_PY11].FUNC_CTL = IOC_PY11_FUNC_CTL_VAD_DAT;
}

void init_cam_pins(void)
{
    /* configure rst pin function */
    HPM_IOC->PAD[IOC_PAD_PY05].FUNC_CTL = IOC_PY05_FUNC_CTL_GPIO_Y_05;
    /* PY port IO needs to configure PIOC */
    HPM_PIOC->PAD[IOC_PAD_PY05].FUNC_CTL = IOC_PY05_FUNC_CTL_SOC_PY_05;

    HPM_IOC->PAD[IOC_PAD_PA10].FUNC_CTL = IOC_PA10_FUNC_CTL_CAM0_XCLK;
    HPM_IOC->PAD[IOC_PAD_PA11].FUNC_CTL = IOC_PA11_FUNC_CTL_CAM0_PIXCLK;
    HPM_IOC->PAD[IOC_PAD_PA06].FUNC_CTL = IOC_PA06_FUNC_CTL_CAM0_VSYNC;
    HPM_IOC->PAD[IOC_PAD_PA05].FUNC_CTL = IOC_PA05_FUNC_CTL_CAM0_HSYNC;
    HPM_IOC->PAD[IOC_PAD_PA07].FUNC_CTL = IOC_PA07_FUNC_CTL_CAM0_D_2;
    HPM_IOC->PAD[IOC_PAD_PA03].FUNC_CTL = IOC_PA03_FUNC_CTL_CAM0_D_3;
    HPM_IOC->PAD[IOC_PAD_PA08].FUNC_CTL = IOC_PA08_FUNC_CTL_CAM0_D_4;
    HPM_IOC->PAD[IOC_PAD_PA09].FUNC_CTL = IOC_PA09_FUNC_CTL_CAM0_D_5;
    HPM_IOC->PAD[IOC_PAD_PA00].FUNC_CTL = IOC_PA00_FUNC_CTL_CAM0_D_6;
    HPM_IOC->PAD[IOC_PAD_PA04].FUNC_CTL = IOC_PA04_FUNC_CTL_CAM0_D_7;
    HPM_IOC->PAD[IOC_PAD_PA01].FUNC_CTL = IOC_PA01_FUNC_CTL_CAM0_D_8;
    HPM_IOC->PAD[IOC_PAD_PA02].FUNC_CTL = IOC_PA02_FUNC_CTL_CAM0_D_9;
}

void init_butn_pins(void)
{
    HPM_BIOC->PAD[IOC_PAD_PZ02].FUNC_CTL = IOC_PZ02_FUNC_CTL_PBUTN;
    HPM_BIOC->PAD[IOC_PAD_PZ03].FUNC_CTL = IOC_PZ03_FUNC_CTL_WBUTN;
    HPM_BIOC->PAD[IOC_PAD_PZ04].FUNC_CTL = IOC_PZ04_FUNC_CTL_PLED;
    HPM_BIOC->PAD[IOC_PAD_PZ05].FUNC_CTL = IOC_PZ05_FUNC_CTL_WLED;
}

void init_acmp_pins(void)
{
    /* configure to ACMP_COMP_1(ALT16) function */
    HPM_IOC->PAD[IOC_PAD_PE25].FUNC_CTL = IOC_PE25_FUNC_CTL_ACMP_COMP_1;
    /* configure to CMP1_INP7 function */
    HPM_IOC->PAD[IOC_PAD_PE23].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
    /* configure to CMP1_INN6 function */
    HPM_IOC->PAD[IOC_PAD_PE21].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
}

void init_enet_pins(ENET_Type *ptr)
{
    if (ptr == HPM_ENET0) {
        HPM_IOC->PAD[IOC_PAD_PF00].FUNC_CTL = IOC_PF00_FUNC_CTL_GPIO_F_00;

        HPM_IOC->PAD[IOC_PAD_PE22].FUNC_CTL = IOC_PE22_FUNC_CTL_ETH0_MDC;
        HPM_IOC->PAD[IOC_PAD_PE23].FUNC_CTL = IOC_PE23_FUNC_CTL_ETH0_MDIO;

        HPM_IOC->PAD[IOC_PAD_PD31].FUNC_CTL = IOC_PD31_FUNC_CTL_ETH0_RXD_0;
        HPM_IOC->PAD[IOC_PAD_PE04].FUNC_CTL = IOC_PE04_FUNC_CTL_ETH0_RXD_1;
        HPM_IOC->PAD[IOC_PAD_PE02].FUNC_CTL = IOC_PE02_FUNC_CTL_ETH0_RXD_2;
        HPM_IOC->PAD[IOC_PAD_PE07].FUNC_CTL = IOC_PE07_FUNC_CTL_ETH0_RXD_3;
        HPM_IOC->PAD[IOC_PAD_PE03].FUNC_CTL = IOC_PE03_FUNC_CTL_ETH0_RXCK;
        HPM_IOC->PAD[IOC_PAD_PD30].FUNC_CTL = IOC_PD30_FUNC_CTL_ETH0_RXDV;

        HPM_IOC->PAD[IOC_PAD_PE06].FUNC_CTL = IOC_PE06_FUNC_CTL_ETH0_TXD_0;
        HPM_IOC->PAD[IOC_PAD_PD29].FUNC_CTL = IOC_PD29_FUNC_CTL_ETH0_TXD_1;
        HPM_IOC->PAD[IOC_PAD_PD28].FUNC_CTL = IOC_PD28_FUNC_CTL_ETH0_TXD_2;
        HPM_IOC->PAD[IOC_PAD_PE05].FUNC_CTL = IOC_PE05_FUNC_CTL_ETH0_TXD_3;
        HPM_IOC->PAD[IOC_PAD_PE01].FUNC_CTL = IOC_PE01_FUNC_CTL_ETH0_TXCK;
        HPM_IOC->PAD[IOC_PAD_PE00].FUNC_CTL = IOC_PE00_FUNC_CTL_ETH0_TXEN;
    } else if (ptr == HPM_ENET1) {
        HPM_IOC->PAD[IOC_PAD_PE26].FUNC_CTL = IOC_PE26_FUNC_CTL_GPIO_E_26;

        HPM_IOC->PAD[IOC_PAD_PD11].FUNC_CTL = IOC_PD11_FUNC_CTL_ETH1_MDC;
        HPM_IOC->PAD[IOC_PAD_PD14].FUNC_CTL = IOC_PD14_FUNC_CTL_ETH1_MDIO;

        HPM_IOC->PAD[IOC_PAD_PE20].FUNC_CTL = IOC_PE20_FUNC_CTL_ETH1_RXD_0;
        HPM_IOC->PAD[IOC_PAD_PE18].FUNC_CTL = IOC_PE18_FUNC_CTL_ETH1_RXD_1;
        HPM_IOC->PAD[IOC_PAD_PE15].FUNC_CTL = IOC_PE15_FUNC_CTL_ETH1_RXDV;

        HPM_IOC->PAD[IOC_PAD_PE19].FUNC_CTL = IOC_PE19_FUNC_CTL_ETH1_TXD_0;
        HPM_IOC->PAD[IOC_PAD_PE17].FUNC_CTL = IOC_PE17_FUNC_CTL_ETH1_TXD_1;
        HPM_IOC->PAD[IOC_PAD_PE14].FUNC_CTL = IOC_PE14_FUNC_CTL_ETH1_TXEN;

        HPM_IOC->PAD[IOC_PAD_PE16].FUNC_CTL = IOC_PAD_FUNC_CTL_LOOP_BACK_MASK | IOC_PE16_FUNC_CTL_ETH1_REFCLK;
    }
}

void init_pwm_pins(PWM_Type *ptr)
{
    if (ptr == HPM_PWM3) {
        HPM_IOC->PAD[IOC_PAD_PE17].FUNC_CTL = IOC_PE17_FUNC_CTL_PWM3_P_6;
        HPM_IOC->PAD[IOC_PAD_PE18].FUNC_CTL = IOC_PE18_FUNC_CTL_PWM3_P_7;
    } else if (ptr == HPM_PWM2) {
        HPM_IOC->PAD[IOC_PAD_PD28].FUNC_CTL = IOC_PD28_FUNC_CTL_PWM2_P_5;
        HPM_IOC->PAD[IOC_PAD_PD29].FUNC_CTL = IOC_PD29_FUNC_CTL_PWM2_P_4;
        HPM_IOC->PAD[IOC_PAD_PD30].FUNC_CTL = IOC_PD30_FUNC_CTL_PWM2_P_1;
        HPM_IOC->PAD[IOC_PAD_PD31].FUNC_CTL = IOC_PD31_FUNC_CTL_PWM2_P_0;
        HPM_IOC->PAD[IOC_PAD_PE03].FUNC_CTL = IOC_PE03_FUNC_CTL_PWM2_P_3;
        HPM_IOC->PAD[IOC_PAD_PE04].FUNC_CTL = IOC_PE04_FUNC_CTL_PWM2_P_2;
    }
}

void init_adc12_pins(void)
{
    /* ADC0/1/2.VIN7 */
    HPM_IOC->PAD[IOC_PAD_PE21].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
    /* ADC0/1/2.VIN10 */
    HPM_IOC->PAD[IOC_PAD_PE24].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
    /* ADC0/1/2.VIN11 */
    HPM_IOC->PAD[IOC_PAD_PE25].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
}

void init_adc16_pins(void)
{
    /* ADC3.INA2 */
    HPM_IOC->PAD[IOC_PAD_PE29].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
}

void init_adc_bldc_pins(void)
{
   HPM_IOC->PAD[IOC_PAD_PE21].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
   HPM_IOC->PAD[IOC_PAD_PE24].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
   HPM_IOC->PAD[IOC_PAD_PE25].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
}

void init_usb_pins(void)
{
    /* USB0_ID */
    HPM_IOC->PAD[IOC_PAD_PF10].FUNC_CTL = IOC_PF10_FUNC_CTL_GPIO_F_10;
    HPM_IOC->PAD[IOC_PAD_PF10].PAD_CTL = IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_PE_SET(1);

    /* USB0_OC */
    HPM_IOC->PAD[IOC_PAD_PF08].FUNC_CTL = IOC_PF08_FUNC_CTL_GPIO_F_08;
    HPM_IOC->PAD[IOC_PAD_PF08].PAD_CTL = IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_PE_SET(1);

    /* USB1_ID */
    HPM_IOC->PAD[IOC_PAD_PF07].FUNC_CTL = IOC_PF07_FUNC_CTL_GPIO_F_07;
    HPM_IOC->PAD[IOC_PAD_PF07].PAD_CTL = IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_PE_SET(1);

    /* USB1_OC */
    HPM_IOC->PAD[IOC_PAD_PF05].FUNC_CTL = IOC_PF05_FUNC_CTL_GPIO_F_05;
    HPM_IOC->PAD[IOC_PAD_PF05].PAD_CTL = IOC_PAD_PAD_CTL_PS_SET(1) | IOC_PAD_PAD_CTL_PE_SET(1);
}

void init_can_pins(CAN_Type *ptr)
{
    if (ptr == HPM_CAN0) {
        HPM_IOC->PAD[IOC_PAD_PB15].FUNC_CTL = IOC_PB15_FUNC_CTL_CAN0_TXD;
        HPM_IOC->PAD[IOC_PAD_PB17].FUNC_CTL = IOC_PB17_FUNC_CTL_CAN0_RXD;
    }
}

void init_sdxc_pins(SDXC_Type *ptr, bool use_1v8)
{
    uint32_t cmd_func_ctl = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(17) | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1);
    uint32_t func_ctl = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(17);
    uint32_t pad_ctl = IOC_PAD_PAD_CTL_MS_SET(use_1v8) | IOC_PAD_PAD_CTL_DS_SET(7) | IOC_PAD_PAD_CTL_PE_SET(1) |
                       IOC_PAD_PAD_CTL_PS_SET(1);

    if (ptr == HPM_SDXC0) {
    } else if (ptr == HPM_SDXC1) {

        /* Power */
        HPM_IOC->PAD[IOC_PAD_PC20].FUNC_CTL = IOC_PC20_FUNC_CTL_GPIO_C_20;
        HPM_IOC->PAD[IOC_PAD_PC20].PAD_CTL = IOC_PAD_PAD_CTL_DS_SET(7) | IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);

        /* CLK */
        HPM_IOC->PAD[IOC_PAD_PD22].FUNC_CTL = func_ctl;
        HPM_IOC->PAD[IOC_PAD_PD22].PAD_CTL = pad_ctl;

        /* CMD */
        HPM_IOC->PAD[IOC_PAD_PD21].FUNC_CTL = cmd_func_ctl;
        HPM_IOC->PAD[IOC_PAD_PD21].PAD_CTL = pad_ctl;

        /* DATA0 */
        HPM_IOC->PAD[IOC_PAD_PD18].FUNC_CTL = func_ctl;
        HPM_IOC->PAD[IOC_PAD_PD18].PAD_CTL = pad_ctl;
        /* DATA1 */
        HPM_IOC->PAD[IOC_PAD_PD17].FUNC_CTL = func_ctl;
        HPM_IOC->PAD[IOC_PAD_PD17].PAD_CTL = pad_ctl;
        /* DATA2 */
        HPM_IOC->PAD[IOC_PAD_PD27].FUNC_CTL = func_ctl;
        HPM_IOC->PAD[IOC_PAD_PD27].PAD_CTL = pad_ctl;
        /* DATA3 */
        HPM_IOC->PAD[IOC_PAD_PD26].FUNC_CTL = func_ctl;
        HPM_IOC->PAD[IOC_PAD_PD26].PAD_CTL = pad_ctl;

        /* CDN */
        HPM_IOC->PAD[IOC_PAD_PD15].FUNC_CTL = IOC_PD15_FUNC_CTL_GPIO_D_15;
        HPM_IOC->PAD[IOC_PAD_PD15].PAD_CTL = pad_ctl;
        HPM_GPIO0->OE[GPIO_OE_GPIOD].CLEAR = 1UL << BOARD_APP_SDCARD_CARD_DETECTION_PIN_INDEX;

    }
}

void init_clk_obs_pins(void)
{
    HPM_IOC->PAD[IOC_PAD_PB02].FUNC_CTL = IOC_PB02_FUNC_CTL_SYSCTL_CLK_OBS_0;
}

void init_rgb_pwm_pins(void)
{
    /* Red */
    HPM_IOC->PAD[IOC_PAD_PB11].FUNC_CTL = IOC_PB11_FUNC_CTL_TRGM1_P_01;
    /* Green */
    HPM_IOC->PAD[IOC_PAD_PB12].FUNC_CTL = IOC_PB12_FUNC_CTL_TRGM0_P_06;
    /* BLUE */
    HPM_IOC->PAD[IOC_PAD_PB13].FUNC_CTL = IOC_PB13_FUNC_CTL_TRGM1_P_03;
}

void init_led_pins_as_gpio(void)
{
    HPM_IOC->PAD[IOC_PAD_PB11].FUNC_CTL = IOC_PB11_FUNC_CTL_GPIO_B_11;
    HPM_IOC->PAD[IOC_PAD_PB12].FUNC_CTL = IOC_PB12_FUNC_CTL_GPIO_B_12;
    HPM_IOC->PAD[IOC_PAD_PB13].FUNC_CTL = IOC_PB13_FUNC_CTL_GPIO_B_13;
}
