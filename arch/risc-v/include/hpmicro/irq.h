/****************************************************************************
 * arch/risc-v/include/hpmicro/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_HPMICRO_IRQ_H
#define __ARCH_RISCV_INCLUDE_HPMICRO_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Map RISC-V exception code to NuttX IRQ */
#define HPM_IRQ_PERI_START   (RISCV_IRQ_ASYNC + 20)

/* Machine Global External Interrupt */
#ifdef CONFIG_ARCH_CHIP_HPM6750
#define HPM_IRQn_GPIO0_A      (HPM_IRQ_PERI_START + IRQn_GPIO0_A)         /* GPIO0_A IRQ */
#define HPM_IRQn_GPIO0_B      (HPM_IRQ_PERI_START + IRQn_GPIO0_B)         /* GPIO0_B IRQ */
#define HPM_IRQn_GPIO0_C      (HPM_IRQ_PERI_START + IRQn_GPIO0_C)         /* GPIO0_C IRQ */
#define HPM_IRQn_GPIO0_D      (HPM_IRQ_PERI_START + IRQn_GPIO0_D)         /* GPIO0_D IRQ */
#define HPM_IRQn_GPIO0_E      (HPM_IRQ_PERI_START + IRQn_GPIO0_E)         /* GPIO0_E IRQ */
#define HPM_IRQn_GPIO0_F      (HPM_IRQ_PERI_START + IRQn_GPIO0_F)         /* GPIO0_F IRQ */
#define HPM_IRQn_GPIO0_X      (HPM_IRQ_PERI_START + IRQn_GPIO0_X)         /* GPIO0_X IRQ */
#define HPM_IRQn_GPIO0_Y      (HPM_IRQ_PERI_START + IRQn_GPIO0_Y)         /* GPIO0_Y IRQ */
#define HPM_IRQn_GPIO0_Z      (HPM_IRQ_PERI_START + IRQn_GPIO0_Z)         /* GPIO0_Z IRQ */
#define HPM_IRQn_GPIO1_A      (HPM_IRQ_PERI_START + IRQn_GPIO1_A)         /* GPIO1_A IRQ */
#define HPM_IRQn_GPIO1_B      (HPM_IRQ_PERI_START + IRQn_GPIO1_B)         /* GPIO1_B IRQ */
#define HPM_IRQn_GPIO1_C      (HPM_IRQ_PERI_START + IRQn_GPIO1_C)         /* GPIO1_C IRQ */
#define HPM_IRQn_GPIO1_D      (HPM_IRQ_PERI_START + IRQn_GPIO1_D)         /* GPIO1_D IRQ */
#define HPM_IRQn_GPIO1_E      (HPM_IRQ_PERI_START + IRQn_GPIO1_E)         /* GPIO1_E IRQ */
#define HPM_IRQn_GPIO1_F      (HPM_IRQ_PERI_START + IRQn_GPIO1_F)         /* GPIO1_F IRQ */
#define HPM_IRQn_GPIO1_X      (HPM_IRQ_PERI_START + IRQn_GPIO1_X)         /* GPIO1_X IRQ */
#define HPM_IRQn_GPIO1_Y      (HPM_IRQ_PERI_START + IRQn_GPIO1_Y)         /* GPIO1_Y IRQ */
#define HPM_IRQn_GPIO1_Z      (HPM_IRQ_PERI_START + IRQn_GPIO1_Z)         /* GPIO1_Z IRQ */
#define HPM_IRQn_ADC0         (HPM_IRQ_PERI_START + IRQn_ADC0)            /* ADC0 IRQ */
#define HPM_IRQn_ADC1         (HPM_IRQ_PERI_START + IRQn_ADC1)            /* ADC1 IRQ */
#define HPM_IRQn_ADC2         (HPM_IRQ_PERI_START + IRQn_ADC2)            /* ADC2 IRQ */
#define HPM_IRQn_ADC3         (HPM_IRQ_PERI_START + IRQn_ADC3)            /* ADC3 IRQ */
#define HPM_IRQn_ACMP_0       (HPM_IRQ_PERI_START + IRQn_ACMP_0)          /* ACMP[0] IRQ */
#define HPM_IRQn_ACMP_1       (HPM_IRQ_PERI_START + IRQn_ACMP_1)          /* ACMP[1] IRQ */
#define HPM_IRQn_ACMP_2       (HPM_IRQ_PERI_START + IRQn_ACMP_2)          /* ACMP[2] IRQ */
#define HPM_IRQn_ACMP_3       (HPM_IRQ_PERI_START + IRQn_ACMP_3)          /* ACMP[3] IRQ */
#define HPM_IRQn_SPI0         (HPM_IRQ_PERI_START + IRQn_SPI0)            /* SPI0 IRQ */
#define HPM_IRQn_SPI1         (HPM_IRQ_PERI_START + IRQn_SPI1)            /* SPI1 IRQ */
#define HPM_IRQn_SPI2         (HPM_IRQ_PERI_START + IRQn_SPI2)            /* SPI2 IRQ */
#define HPM_IRQn_SPI3         (HPM_IRQ_PERI_START + IRQn_SPI3)            /* SPI3 IRQ */
#define HPM_IRQn_UART0        (HPM_IRQ_PERI_START + IRQn_UART0)           /* UART0 IRQ */
#define HPM_IRQn_UART1        (HPM_IRQ_PERI_START + IRQn_UART1)           /* UART1 IRQ */
#define HPM_IRQn_UART2        (HPM_IRQ_PERI_START + IRQn_UART2)           /* UART2 IRQ */
#define HPM_IRQn_UART3        (HPM_IRQ_PERI_START + IRQn_UART3)           /* UART3 IRQ */
#define HPM_IRQn_UART4        (HPM_IRQ_PERI_START + IRQn_UART4)           /* UART4 IRQ */
#define HPM_IRQn_UART5        (HPM_IRQ_PERI_START + IRQn_UART5)           /* UART5 IRQ */
#define HPM_IRQn_UART6        (HPM_IRQ_PERI_START + IRQn_UART6)           /* UART6 IRQ */
#define HPM_IRQn_UART7        (HPM_IRQ_PERI_START + IRQn_UART7)           /* UART7 IRQ */
#define HPM_IRQn_UART8        (HPM_IRQ_PERI_START + IRQn_UART8)           /* UART8 IRQ */
#define HPM_IRQn_UART9        (HPM_IRQ_PERI_START + IRQn_UART9)           /* UART9 IRQ */
#define HPM_IRQn_UART10       (HPM_IRQ_PERI_START + IRQn_UART10)          /* UART10 IRQ */
#define HPM_IRQn_UART11       (HPM_IRQ_PERI_START + IRQn_UART11)          /* UART11 IRQ */
#define HPM_IRQn_UART12       (HPM_IRQ_PERI_START + IRQn_UART12)          /* UART12 IRQ */
#define HPM_IRQn_UART13       (HPM_IRQ_PERI_START + IRQn_UART13)          /* UART13 IRQ */
#define HPM_IRQn_UART14       (HPM_IRQ_PERI_START + IRQn_UART14)          /* UART14 IRQ */
#define HPM_IRQn_UART15       (HPM_IRQ_PERI_START + IRQn_UART15)          /* UART15 IRQ */
#define HPM_IRQn_CAN0         (HPM_IRQ_PERI_START + IRQn_CAN0)            /* CAN0 IRQ */
#define HPM_IRQn_CAN1         (HPM_IRQ_PERI_START + IRQn_CAN1)            /* CAN1 IRQ */
#define HPM_IRQn_CAN2         (HPM_IRQ_PERI_START + IRQn_CAN2)            /* CAN2 IRQ */
#define HPM_IRQn_CAN3         (HPM_IRQ_PERI_START + IRQn_CAN3)            /* CAN3 IRQ */
#define HPM_IRQn_PTPC         (HPM_IRQ_PERI_START + IRQn_PTPC)            /* PTPC IRQ */
#define HPM_IRQn_WDG0         (HPM_IRQ_PERI_START + IRQn_WDG0)            /* WDG0 IRQ */
#define HPM_IRQn_WDG1         (HPM_IRQ_PERI_START + IRQn_WDG1)            /* WDG1 IRQ */
#define HPM_IRQn_WDG2         (HPM_IRQ_PERI_START + IRQn_WDG2)            /* WDG2 IRQ */
#define HPM_IRQn_WDG3         (HPM_IRQ_PERI_START + IRQn_WDG3)            /* WDG3 IRQ */
#define HPM_IRQn_MBX0A        (HPM_IRQ_PERI_START + IRQn_MBX0A)           /* MBX0A IRQ */
#define HPM_IRQn_MBX0B        (HPM_IRQ_PERI_START + IRQn_MBX0B)           /* MBX0B IRQ */
#define HPM_IRQn_MBX1A        (HPM_IRQ_PERI_START + IRQn_MBX1A)           /* MBX1A IRQ */
#define HPM_IRQn_MBX1B        (HPM_IRQ_PERI_START + IRQn_MBX1B)           /* MBX1B IRQ */
#define HPM_IRQn_GPTMR0       (HPM_IRQ_PERI_START + IRQn_GPTMR0)          /* GPTMR0 IRQ */
#define HPM_IRQn_GPTMR1       (HPM_IRQ_PERI_START + IRQn_GPTMR1)          /* GPTMR1 IRQ */
#define HPM_IRQn_GPTMR2       (HPM_IRQ_PERI_START + IRQn_GPTMR2)          /* GPTMR2 IRQ */
#define HPM_IRQn_GPTMR3       (HPM_IRQ_PERI_START + IRQn_GPTMR3)          /* GPTMR3 IRQ */
#define HPM_IRQn_GPTMR4       (HPM_IRQ_PERI_START + IRQn_GPTMR4)          /* GPTMR4 IRQ */
#define HPM_IRQn_GPTMR5       (HPM_IRQ_PERI_START + IRQn_GPTMR5)          /* GPTMR5 IRQ */
#define HPM_IRQn_GPTMR6       (HPM_IRQ_PERI_START + IRQn_GPTMR6)          /* GPTMR6 IRQ */
#define HPM_IRQn_GPTMR7       (HPM_IRQ_PERI_START + IRQn_GPTMR7)          /* GPTMR7 IRQ */
#define HPM_IRQn_I2C0         (HPM_IRQ_PERI_START + IRQn_I2C0)            /* I2C0 IRQ */
#define HPM_IRQn_I2C1         (HPM_IRQ_PERI_START + IRQn_I2C1)            /* I2C1 IRQ */
#define HPM_IRQn_I2C2         (HPM_IRQ_PERI_START + IRQn_I2C2)            /* I2C2 IRQ */
#define HPM_IRQn_I2C3         (HPM_IRQ_PERI_START + IRQn_I2C3)            /* I2C3 IRQ */
#define HPM_IRQn_PWM0         (HPM_IRQ_PERI_START + IRQn_PWM0)            /* PWM0 IRQ */
#define HPM_IRQn_HALL0        (HPM_IRQ_PERI_START + IRQn_HALL0)           /* HALL0 IRQ */
#define HPM_IRQn_QEI0         (HPM_IRQ_PERI_START + IRQn_QEI0)            /* QEI0 IRQ */
#define HPM_IRQn_PWM1         (HPM_IRQ_PERI_START + IRQn_PWM1)            /* PWM1 IRQ */
#define HPM_IRQn_HALL1        (HPM_IRQ_PERI_START + IRQn_HALL1)           /* HALL1 IRQ */
#define HPM_IRQn_QEI1         (HPM_IRQ_PERI_START + IRQn_QEI1)            /* QEI1 IRQ */
#define HPM_IRQn_PWM2         (HPM_IRQ_PERI_START + IRQn_PWM2)            /* PWM2 IRQ */
#define HPM_IRQn_HALL2        (HPM_IRQ_PERI_START + IRQn_HALL2)           /* HALL2 IRQ */
#define HPM_IRQn_QEI2         (HPM_IRQ_PERI_START + IRQn_QEI2)            /* QEI2 IRQ */
#define HPM_IRQn_PWM3         (HPM_IRQ_PERI_START + IRQn_PWM3)            /* PWM3 IRQ */
#define HPM_IRQn_HALL3        (HPM_IRQ_PERI_START + IRQn_HALL3)           /* HALL3 IRQ */
#define HPM_IRQn_QEI3         (HPM_IRQ_PERI_START + IRQn_QEI3)            /* QEI3 IRQ */
#define HPM_IRQn_SDP          (HPM_IRQ_PERI_START + IRQn_SDP)             /* SDP IRQ */
#define HPM_IRQn_XPI0         (HPM_IRQ_PERI_START + IRQn_XPI0)            /* XPI0 IRQ */
#define HPM_IRQn_XPI1         (HPM_IRQ_PERI_START + IRQn_XPI1)            /* XPI1 IRQ */
#define HPM_IRQn_XDMA         (HPM_IRQ_PERI_START + IRQn_XDMA)            /* XDMA IRQ */
#define HPM_IRQn_HDMA         (HPM_IRQ_PERI_START + IRQn_HDMA)            /* HDMA IRQ */
#define HPM_IRQn_FEMC         (HPM_IRQ_PERI_START + IRQn_FEMC)            /* FEMC IRQ */
#define HPM_IRQn_RNG          (HPM_IRQ_PERI_START + IRQn_RNG)             /* RNG IRQ */
#define HPM_IRQn_I2S0         (HPM_IRQ_PERI_START + IRQn_I2S0)            /* I2S0 IRQ */
#define HPM_IRQn_I2S1         (HPM_IRQ_PERI_START + IRQn_I2S1)            /* I2S1 IRQ */
#define HPM_IRQn_I2S2         (HPM_IRQ_PERI_START + IRQn_I2S2)            /* I2S2 IRQ */
#define HPM_IRQn_I2S3         (HPM_IRQ_PERI_START + IRQn_I2S3)            /* I2S3 IRQ */
#define HPM_IRQn_DAO          (HPM_IRQ_PERI_START + IRQn_DAO)             /* DAO IRQ */
#define HPM_IRQn_PDM          (HPM_IRQ_PERI_START + IRQn_PDM)             /* PDM IRQ */
#define HPM_IRQn_CAM0         (HPM_IRQ_PERI_START + IRQn_CAM0)            /* CAM0 IRQ */
#define HPM_IRQn_CAM1         (HPM_IRQ_PERI_START + IRQn_CAM1)            /* CAM1 IRQ */
#define HPM_IRQn_LCDC_D0      (HPM_IRQ_PERI_START + IRQn_LCDC_D0)         /* LCDC_D0 IRQ */
#define HPM_IRQn_LCDC_D1      (HPM_IRQ_PERI_START + IRQn_LCDC_D1)         /* LCDC_D1 IRQ */
#define HPM_IRQn_PDMA_D0      (HPM_IRQ_PERI_START + IRQn_PDMA_D0)         /* PDMA_D0 IRQ */
#define HPM_IRQn_PDMA_D1      (HPM_IRQ_PERI_START + IRQn_PDMA_D1)         /* PDMA_D1 IRQ */
#define HPM_IRQn_JPEG         (HPM_IRQ_PERI_START + IRQn_JPEG)            /* JPEG IRQ */
#define HPM_IRQn_NTMR0        (HPM_IRQ_PERI_START + IRQn_NTMR0)           /* NTMR0 IRQ */
#define HPM_IRQn_NTMR1        (HPM_IRQ_PERI_START + IRQn_NTMR1)           /* NTMR1 IRQ */
#define HPM_IRQn_USB0         (HPM_IRQ_PERI_START + IRQn_USB0)            /* USB0 IRQ */
#define HPM_IRQn_USB1         (HPM_IRQ_PERI_START + IRQn_USB1)            /* USB1 IRQ */
#define HPM_IRQn_ENET0        (HPM_IRQ_PERI_START + IRQn_ENET0)           /* ENET0 IRQ */
#define HPM_IRQn_ENET1        (HPM_IRQ_PERI_START + IRQn_ENET1)           /* ENET1 IRQ */
#define HPM_IRQn_SDXC0        (HPM_IRQ_PERI_START + IRQn_SDXC0)           /* SDXC0 IRQ */
#define HPM_IRQn_SDXC1        (HPM_IRQ_PERI_START + IRQn_SDXC1)           /* SDXC1 IRQ */
#define HPM_IRQn_PSEC         (HPM_IRQ_PERI_START + IRQn_PSEC)            /* PSEC IRQ */
#define HPM_IRQn_PGPIO        (HPM_IRQ_PERI_START + IRQn_PGPIO)           /* PGPIO IRQ */
#define HPM_IRQn_PWDG         (HPM_IRQ_PERI_START + IRQn_PWDG)            /* PWDG IRQ */
#define HPM_IRQn_PTMR         (HPM_IRQ_PERI_START + IRQn_PTMR)            /* PTMR IRQ */
#define HPM_IRQn_PUART        (HPM_IRQ_PERI_START + IRQn_PUART)           /* PUART IRQ */
#define HPM_IRQn_VAD          (HPM_IRQ_PERI_START + IRQn_VAD)             /* VAD IRQ */
#define HPM_IRQn_FUSE         (HPM_IRQ_PERI_START + IRQn_FUSE)            /* FUSE IRQ */
#define HPM_IRQn_SECMON       (HPM_IRQ_PERI_START + IRQn_SECMON)          /* SECMON IRQ */
#define HPM_IRQn_RTC          (HPM_IRQ_PERI_START + IRQn_RTC)             /* RTC IRQ */
#define HPM_IRQn_BUTN         (HPM_IRQ_PERI_START + IRQn_BUTN)            /* BUTN IRQ */
#define HPM_IRQn_BGPIO        (HPM_IRQ_PERI_START + IRQn_BGPIO)           /* BGPIO IRQ */
#define HPM_IRQn_BVIO         (HPM_IRQ_PERI_START + IRQn_BVIO)            /* BVIO IRQ */
#define HPM_IRQn_BROWNOUT     (HPM_IRQ_PERI_START + IRQn_BROWNOUT)        /* BROWNOUT IRQ */
#define HPM_IRQn_SYSCTL       (HPM_IRQ_PERI_START + IRQn_SYSCTL)          /* SYSCTL IRQ */
#define HPM_IRQn_DEBUG_0      (HPM_IRQ_PERI_START + IRQn_DEBUG_0)         /* DEBUG[0] IRQ */
#define HPM_IRQn_DEBUG_1      (HPM_IRQ_PERI_START + IRQn_DEBUG_1)         /* DEBUG[1] IRQ */
#define HPM_NR_IRQS           127                                         /* Total number of IRQs */
#define NR_IRQS               (HPM_IRQ_PERI_START + HPM_NR_IRQS)
#endif

#ifdef CONFIG_ARCH_CHIP_HPM6360
#define HPM_IRQn_GPIO0_A      (HPM_IRQ_PERI_START + IRQn_GPIO0_A)         /* GPIO0_A IRQ */
#define HPM_IRQn_GPIO0_B      (HPM_IRQ_PERI_START + IRQn_GPIO0_B)         /* GPIO0_B IRQ */
#define HPM_IRQn_GPIO0_C      (HPM_IRQ_PERI_START + IRQn_GPIO0_C)         /* GPIO0_C IRQ */
#define HPM_IRQn_GPIO0_D      (HPM_IRQ_PERI_START + IRQn_GPIO0_D)         /* GPIO0_D IRQ */
#define HPM_IRQn_GPIO0_X      (HPM_IRQ_PERI_START + IRQn_GPIO0_X)         /* GPIO0_X IRQ */
#define HPM_IRQn_GPIO0_Y      (HPM_IRQ_PERI_START + IRQn_GPIO0_Y)         /* GPIO0_Y IRQ */
#define HPM_IRQn_GPIO0_Z      (HPM_IRQ_PERI_START + IRQn_GPIO0_Z)         /* GPIO0_Z IRQ */
#define HPM_IRQn_ADC0         (HPM_IRQ_PERI_START + IRQn_ADC0)            /* ADC0 IRQ */
#define HPM_IRQn_ADC1         (HPM_IRQ_PERI_START + IRQn_ADC1)            /* ADC1 IRQ */
#define HPM_IRQn_ADC2         (HPM_IRQ_PERI_START + IRQn_ADC2)            /* ADC2 IRQ */
#define HPM_IRQn_DAC          (HPM_IRQ_PERI_START + IRQn_DAC)             /* DAC IRQ */
#define HPM_IRQn_ACMP_0       (HPM_IRQ_PERI_START + IRQn_ACMP_0)          /* ACMP[0] IRQ */
#define HPM_IRQn_ACMP_1       (HPM_IRQ_PERI_START + IRQn_ACMP_1)          /* ACMP[1] IRQ */
#define HPM_IRQn_SPI0         (HPM_IRQ_PERI_START + IRQn_SPI0)            /* SPI0 IRQ */
#define HPM_IRQn_SPI1         (HPM_IRQ_PERI_START + IRQn_SPI1)            /* SPI1 IRQ */
#define HPM_IRQn_SPI2         (HPM_IRQ_PERI_START + IRQn_SPI2)            /* SPI2 IRQ */
#define HPM_IRQn_SPI3         (HPM_IRQ_PERI_START + IRQn_SPI3)            /* SPI3 IRQ */
#define HPM_IRQn_UART0        (HPM_IRQ_PERI_START + IRQn_UART0)           /* UART0 IRQ */
#define HPM_IRQn_UART1        (HPM_IRQ_PERI_START + IRQn_UART1)           /* UART1 IRQ */
#define HPM_IRQn_UART2        (HPM_IRQ_PERI_START + IRQn_UART2)           /* UART2 IRQ */
#define HPM_IRQn_UART3        (HPM_IRQ_PERI_START + IRQn_UART3)           /* UART3 IRQ */
#define HPM_IRQn_UART4        (HPM_IRQ_PERI_START + IRQn_UART4)           /* UART4 IRQ */
#define HPM_IRQn_UART5        (HPM_IRQ_PERI_START + IRQn_UART5)           /* UART5 IRQ */
#define HPM_IRQn_UART6        (HPM_IRQ_PERI_START + IRQn_UART6)           /* UART6 IRQ */
#define HPM_IRQn_UART7        (HPM_IRQ_PERI_START + IRQn_UART7)           /* UART7 IRQ */
#define HPM_IRQn_CAN0         (HPM_IRQ_PERI_START + IRQn_CAN0)            /* CAN0 IRQ */
#define HPM_IRQn_CAN1         (HPM_IRQ_PERI_START + IRQn_CAN1)            /* CAN1 IRQ */
#define HPM_IRQn_PTPC         (HPM_IRQ_PERI_START + IRQn_PTPC)            /* PTPC IRQ */
#define HPM_IRQn_WDG0         (HPM_IRQ_PERI_START + IRQn_WDG0)            /* WDG0 IRQ */
#define HPM_IRQn_WDG1         (HPM_IRQ_PERI_START + IRQn_WDG1)            /* WDG1 IRQ */
#define HPM_IRQn_TSNS         (HPM_IRQ_PERI_START + IRQn_TSNS)            /* TSNS IRQ */
#define HPM_IRQn_MBX0A        (HPM_IRQ_PERI_START + IRQn_MBX0A)           /* MBX0A IRQ */
#define HPM_IRQn_MBX0B        (HPM_IRQ_PERI_START + IRQn_MBX0B)           /* MBX0B IRQ */
#define HPM_IRQn_GPTMR0       (HPM_IRQ_PERI_START + IRQn_GPTMR0)          /* GPTMR0 IRQ */
#define HPM_IRQn_GPTMR1       (HPM_IRQ_PERI_START + IRQn_GPTMR1)          /* GPTMR1 IRQ */
#define HPM_IRQn_GPTMR2       (HPM_IRQ_PERI_START + IRQn_GPTMR2)          /* GPTMR2 IRQ */
#define HPM_IRQn_GPTMR3       (HPM_IRQ_PERI_START + IRQn_GPTMR3)          /* GPTMR3 IRQ */
#define HPM_IRQn_I2C0         (HPM_IRQ_PERI_START + IRQn_I2C0)            /* I2C0 IRQ */
#define HPM_IRQn_I2C1         (HPM_IRQ_PERI_START + IRQn_I2C1)            /* I2C1 IRQ */
#define HPM_IRQn_I2C2         (HPM_IRQ_PERI_START + IRQn_I2C2)            /* I2C2 IRQ */
#define HPM_IRQn_I2C3         (HPM_IRQ_PERI_START + IRQn_I2C3)            /* I2C3 IRQ */
#define HPM_IRQn_PWM0         (HPM_IRQ_PERI_START + IRQn_PWM0)            /* PWM0 IRQ */
#define HPM_IRQn_HALL0        (HPM_IRQ_PERI_START + IRQn_HALL0)           /* HALL0 IRQ */
#define HPM_IRQn_QEI0         (HPM_IRQ_PERI_START + IRQn_QEI0)            /* QEI0 IRQ */
#define HPM_IRQn_PWM1         (HPM_IRQ_PERI_START + IRQn_PWM1)            /* PWM1 IRQ */
#define HPM_IRQn_HALL1        (HPM_IRQ_PERI_START + IRQn_HALL1)           /* HALL1 IRQ */
#define HPM_IRQn_QEI1         (HPM_IRQ_PERI_START + IRQn_QEI1)            /* QEI1 IRQ */
#define HPM_IRQn_SDP          (HPM_IRQ_PERI_START + IRQn_SDP)             /* SDP IRQ */
#define HPM_IRQn_XPI0         (HPM_IRQ_PERI_START + IRQn_XPI0)            /* XPI0 IRQ */
#define HPM_IRQn_XPI1         (HPM_IRQ_PERI_START + IRQn_XPI1)            /* XPI1 IRQ */
#define HPM_IRQn_XDMA         (HPM_IRQ_PERI_START + IRQn_XDMA)            /* XDMA IRQ */
#define HPM_IRQn_HDMA         (HPM_IRQ_PERI_START + IRQn_HDMA)            /* HDMA IRQ */
#define HPM_IRQn_FEMC         (HPM_IRQ_PERI_START + IRQn_FEMC)            /* FEMC IRQ */
#define HPM_IRQn_RNG          (HPM_IRQ_PERI_START + IRQn_RNG)             /* RNG IRQ */
#define HPM_IRQn_I2S0         (HPM_IRQ_PERI_START + IRQn_I2S0)            /* I2S0 IRQ */
#define HPM_IRQn_I2S1         (HPM_IRQ_PERI_START + IRQn_I2S1)            /* I2S1 IRQ */
#define HPM_IRQn_DAO          (HPM_IRQ_PERI_START + IRQn_DAO)             /* DAO IRQ */
#define HPM_IRQn_PDM          (HPM_IRQ_PERI_START + IRQn_PDM)             /* PDM IRQ */
#define HPM_IRQn_FFA          (HPM_IRQ_PERI_START + IRQn_FFA)             /* FFA IRQ */
#define HPM_IRQn_NTMR0        (HPM_IRQ_PERI_START + IRQn_NTMR0)           /* NTMR0 IRQ */
#define HPM_IRQn_USB0         (HPM_IRQ_PERI_START + IRQn_USB0)            /* USB0 IRQ */
#define HPM_IRQn_ENET0        (HPM_IRQ_PERI_START + IRQn_ENET0)           /* ENET0 IRQ */
#define HPM_IRQn_SDXC0        (HPM_IRQ_PERI_START + IRQn_SDXC0)           /* SDXC0 IRQ */
#define HPM_IRQn_PSEC         (HPM_IRQ_PERI_START + IRQn_PSEC)            /* PSEC IRQ */
#define HPM_IRQn_PGPIO        (HPM_IRQ_PERI_START + IRQn_PGPIO)           /* PGPIO IRQ */
#define HPM_IRQn_PWDG         (HPM_IRQ_PERI_START + IRQn_PWDG)            /* PWDG IRQ */
#define HPM_IRQn_PTMR         (HPM_IRQ_PERI_START + IRQn_PTMR)            /* PTMR IRQ */
#define HPM_IRQn_PUART        (HPM_IRQ_PERI_START + IRQn_PUART)           /* PUART IRQ */
#define HPM_IRQn_FUSE         (HPM_IRQ_PERI_START + IRQn_FUSE)            /* FUSE IRQ */
#define HPM_IRQn_SECMON       (HPM_IRQ_PERI_START + IRQn_SECMON)          /* SECMON IRQ */
#define HPM_IRQn_RTC          (HPM_IRQ_PERI_START + IRQn_RTC)             /* RTC IRQ */
#define HPM_IRQn_BUTN         (HPM_IRQ_PERI_START + IRQn_BUTN)            /* BUTN IRQ */
#define HPM_IRQn_BGPIO        (HPM_IRQ_PERI_START + IRQn_BGPIO)           /* BGPIO IRQ */
#define HPM_IRQn_BVIO         (HPM_IRQ_PERI_START + IRQn_BVIO)            /* BVIO IRQ */
#define HPM_IRQn_BROWNOUT     (HPM_IRQ_PERI_START + IRQn_BROWNOUT)        /* BROWNOUT IRQ */
#define HPM_IRQn_SYSCTL       (HPM_IRQ_PERI_START + IRQn_SYSCTL)          /* SYSCTL IRQ */
#define HPM_IRQn_DEBUG_0      (HPM_IRQ_PERI_START + IRQn_DEBUG_0)         /* DEBUG[0] IRQ */
#define HPM_IRQn_DEBUG_1      (HPM_IRQ_PERI_START + IRQn_DEBUG_1)         /* DEBUG[1] IRQ */
#define HPM_NR_IRQS           78
#define NR_IRQS               (HPM_IRQ_PERI_START + HPM_NR_IRQS)
#endif

#endif /* __ARCH_RISCV_INCLUDE_HPMICRO_IRQ_H */
