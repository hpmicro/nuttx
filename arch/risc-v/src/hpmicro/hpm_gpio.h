/****************************************************************************
 * arch/risc-v/src/hpmicro/hpm_gpio.h
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

#ifndef __ARCH_RISCV_SRC_HPMICRO_GPIO_H
#define __ARCH_RISCV_SRC_HPMICRO_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#include <stdint.h>
#include <stdbool.h>
#endif

#include <nuttx/irq.h>

#include "chip.h"
#include "nuttx/ioexpander/gpio.h"
#include "hpm_gpio_drv.h"

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/
#define  GPIO_INPUT_MODE       (0UL)
#define  GPIO_OUTPUT_MODE      (1UL)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The smallest integer type that can hold the GPIO encoding */

typedef uint32_t gpio_pin_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 * Returned Value:
 *   OK on success
 *   ERROR on invalid port.
 *
 ****************************************************************************/

int hpm_configgpio(GPIO_Type *ptr, gpio_pin_t pin, enum gpio_pintype_e type);

/****************************************************************************
 * Name: hpm_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void hpm_gpiowrite(GPIO_Type *ptr, gpio_pin_t pin, bool value);

/****************************************************************************
 * Name: hpm_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool hpm_gpioread(GPIO_Type *ptr, gpio_pin_t pin, uint8_t mode);

/****************************************************************************
 * Function:  hpm6750_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int hpm6750_dumpgpio(gpio_pinset_t pinset, const char *msg);
#else
#define hpm6750_dumpgpio(p, m)
#endif

#if 1
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_SMT               (1 << 31)                   /* Bit31: 1=使能输入施密特触发器 */

/* 输出类型选择:开漏或推挽
 *
 */
#define GPIO_LOOPBACK                (1 << 30)                   /* Bit30: 1=使能回环 */

/* GPIO 控制器选择:
 * GPIO0、GPIO1、FGPIO、FGPIO1、PGPIO、BGPIO
 *
 */

#define GPIO_CONTROLLER_SHIFT        (29)      /* Bits 31-29: GPIO 控制器选择 */
#define GPIO_CONTROLLER_MASK      (7 << GPIO_CONTROLLER_SHIFT)
#  define GPIO0            (0 << GPIO_CONTROLLER_SHIFT) /* GPIO0 通过控制器 */
#  define GPIO1            (1 << GPIO_CONTROLLER_SHIFT) /* GPIO1 通过控制器 */
#  define FGPIO0           (2 << GPIO_CONTROLLER_SHIFT) /* FGPIO0 快速控制器 */
#  define FGPIO1           (3 << GPIO_CONTROLLER_SHIFT) /* FGPIO1 快速控制器 */
#  define PGPIO            (4 << GPIO_CONTROLLER_SHIFT) /* 电源管理域 GPIO 控制器 */
#  define BGPIO            (5 << GPIO_CONTROLLER_SHIFT) /* 和电池备份域 GPIO 控制器 (BGPIO) */

/* 模式选择：
 */

#define GPIO_MODE_SHIFT        (27)      /* Bits 28-27: Pin mode */
#define GPIO_MODE_MASK         (3 << GPIO_MODE_SHIFT)
#  define GPIO_INPUT           (0 << GPIO_MODE_SHIFT) /* GPIO input */
#  define GPIO_OUTPUT          (1 << GPIO_MODE_SHIFT) /* GPIO output */
#  define GPIO_ALT             (2 << GPIO_MODE_SHIFT) /* Peripheral */
#  define GPIO_ANALOG          (3 << GPIO_MODE_SHIFT) /* Interrupt input */


/* 输入或输出上下拉:
*/
#define GPIO_PUPD_SHIFT         (25)                       /* Bits 26-25: Pull-up/pull down */
#define GPIO_PUPD_MASK          (3 << GPIO_PUPD_SHIFT)
#  define GPIO_FLOAT            (0 << GPIO_PUPD_SHIFT)     /* No pull-up, pull-down */
#  define GPIO_PULLUP           (1 << GPIO_PUPD_SHIFT)     /* Pull-up */
#  define GPIO_PULLDOWN         (2 << GPIO_PUPD_SHIFT)     /* Pull-down */


/* 复用功能:
 */

#define GPIO_AF_SHIFT         (20)      /* Bits 24-20: Peripheral alternate function */
#define GPIO_AF_MASK          (0x1f << GPIO_AF_SHIFT)
#  define GPIO_AF0            (0 << GPIO_AF_SHIFT)  /* Alternate function 0 */
#  define GPIO_AF1            (1 << GPIO_AF_SHIFT)  /* Alternate function 1 */
#  define GPIO_AF2            (2 << GPIO_AF_SHIFT)  /* Alternate function 2 */
#  define GPIO_AF3            (3 << GPIO_AF_SHIFT)  /* Alternate function 3 */
#  define GPIO_AF4            (4 << GPIO_AF_SHIFT)  /* Alternate function 4 */
#  define GPIO_AF5            (5 << GPIO_AF_SHIFT)  /* Alternate function 5 */
#  define GPIO_AF6            (6 << GPIO_AF_SHIFT)  /* Alternate function 6 */
#  define GPIO_AF7            (7 << GPIO_AF_SHIFT)  /* Alternate function 7 */
#  define GPIO_AF8            (8 << GPIO_AF_SHIFT)  /* Alternate function 8 */
#  define GPIO_AF9            (9 << GPIO_AF_SHIFT)  /* Alternate function 9 */
#  define GPIO_AF10           (10 << GPIO_AF_SHIFT) /* Alternate function 10 */
#  define GPIO_AF11           (11 << GPIO_AF_SHIFT) /* Alternate function 11 */
#  define GPIO_AF12           (12 << GPIO_AF_SHIFT) /* Alternate function 12 */
#  define GPIO_AF13           (13 << GPIO_AF_SHIFT) /* Alternate function 13 */
#  define GPIO_AF14           (14 << GPIO_AF_SHIFT) /* Alternate function 14 */
#  define GPIO_AF15           (15 << GPIO_AF_SHIFT) /* Alternate function 15 */
#  define GPIO_AF16           (16 << GPIO_AF_SHIFT) /* Alternate function 16 */
#  define GPIO_AF17           (17 << GPIO_AF_SHIFT) /* Alternate function 17 */
#  define GPIO_AF18           (18 << GPIO_AF_SHIFT) /* Alternate function 18 */
#  define GPIO_AF19           (19 << GPIO_AF_SHIFT) /* Alternate function 19 */
#  define GPIO_AF20           (20 << GPIO_AF_SHIFT) /* Alternate function 20 */
#  define GPIO_AF21           (21 << GPIO_AF_SHIFT) /* Alternate function 21 */
#  define GPIO_AF22           (22 << GPIO_AF_SHIFT) /* Alternate function 22 */
#  define GPIO_AF23           (23 << GPIO_AF_SHIFT) /* Alternate function 23 */
#  define GPIO_AF24           (24 << GPIO_AF_SHIFT) /* Alternate function 24 */
#  define GPIO_AF25           (25 << GPIO_AF_SHIFT) /* Alternate function 25 */
#  define GPIO_AF26           (26 << GPIO_AF_SHIFT) /* Alternate function 26 */
#  define GPIO_AF27           (27 << GPIO_AF_SHIFT) /* Alternate function 27 */
#  define GPIO_AF28           (28 << GPIO_AF_SHIFT) /* Alternate function 28 */
#  define GPIO_AF29           (29 << GPIO_AF_SHIFT) /* Alternate function 29 */
#  define GPIO_AF30           (30 << GPIO_AF_SHIFT) /* Alternate function 30 */
#  define GPIO_AF31           (31 << GPIO_AF_SHIFT) /* Alternate function 31 */

/* 引脚供电电压选择, 此位只对高速引脚可用
 *
 */
#define GPIO_1V8               (1 << 16)                   /* Bit16: 1=3.3V */
#define GPIO_3V3               (0)                        /* Bit16: 0=1.8V */

#if defined(CONFIG_ARCH_CHIP_HPM5361_SDK) || defined(CONFIG_ARCH_CHIP_HPM5301_SDK)

#define GPIO_SPEED_SHIFT       (14)                       /* Bits 15-14: GPIO frequency selection */
#define GPIO_SPEED_MASK        (3 << GPIO_SPEED_SHIFT)
#  define GPIO_SPEED_50MHz     (0 << GPIO_SPEED_SHIFT)     /* 00: Slow frequency slew rate(50Mhz) */
#  define GPIO_SPEED_100MHz    (1 << GPIO_SPEED_SHIFT)     /* 01: Medium frequency slew rate(100 Mhz) */
#  define GPIO_SPEED_150MHz    (2 << GPIO_SPEED_SHIFT)     /* 10: Fast frequency slew rate(150 Mhz)  */
#  define GPIO_SPEED_200MHz    (3 << GPIO_SPEED_SHIFT)     /* 11: Max frequency slew rate(200Mhz)*/

/* 驱动强度
*/
#define GPIO_DS_SHIFT          (11)                       /* Bits 13-11: GPIO frequency selection */
#define GPIO_DS_MASK           (7 << GPIO_DS_SHIFT)
#  define GPIO_DS_1V8_260      (0 << GPIO_DS_SHIFT)     /* 2 MHz Low speed output */
#  define GPIO_DS_1V8_130      (2 << GPIO_DS_SHIFT)     /* 100 MHz High speed output */
#  define GPIO_DS_1V8_88       (3 << GPIO_DS_SHIFT)     /* 50 MHz Fast speed output  */
#  define GPIO_DS_1V8_65       (4 << GPIO_DS_SHIFT)     /* 100 MHz High speed output */
#  define GPIO_DS_1V8_52       (5 << GPIO_DS_SHIFT)     /* 100 MHz High speed output */
#  define GPIO_DS_1V8_43       (6 << GPIO_DS_SHIFT)     /* 100 MHz High speed output */
#  define GPIO_DS_1V8_37       (7 << GPIO_DS_SHIFT)     /* 100 MHz High speed output */
#  define GPIO_DS_3V3_157      (0 << GPIO_DS_SHIFT)     /* 2 MHz Low speed output */
#  define GPIO_DS_3V3_78       (2 << GPIO_DS_SHIFT)     /* 100 MHz High speed output */
#  define GPIO_DS_3V3_53       (3 << GPIO_DS_SHIFT)     /* 50 MHz Fast speed output  */
#  define GPIO_DS_3V3_39       (4 << GPIO_DS_SHIFT)     /* 100 MHz High speed output */
#  define GPIO_DS_3V3_32       (5 << GPIO_DS_SHIFT)     /* 100 MHz High speed output */
#  define GPIO_DS_3V3_26       (6 << GPIO_DS_SHIFT)     /* 100 MHz High speed output */
#  define GPIO_DS_3V3_23       (7 << GPIO_DS_SHIFT)     /* 100 MHz High speed output */
#else

/* 驱动强度
*/
#define GPIO_DS_SHIFT             (11)                       /* Bits 13-11: GPIO frequency selection */
#define GPIO_DS_MASK              (7 << GPIO_DS_SHIFT)
#  define GPIO_DS_4mA             (0 << GPIO_DS_SHIFT)     /* 2 MHz Low speed output */
#  define GPIO_DS_8mA             (1 << GPIO_DS_SHIFT)     /* 25 MHz Medium speed output */
#  define GPIO_DS_12mA            (3 << GPIO_DS_SHIFT)     /* 100 MHz High speed output */
#  define GPIO_DS_3V3_85P61       (0 << GPIO_DS_SHIFT)
#  define GPIO_DS_3V3_61P2        (1 << GPIO_DS_SHIFT)
#  define GPIO_DS_3V3_42P88       (2 << GPIO_DS_SHIFT)
#  define GPIO_DS_3V3_35P76       (3 << GPIO_DS_SHIFT)
#  define GPIO_DS_3V3_30P67       (7 << GPIO_DS_SHIFT)
#  define GPIO_DS_1V8_84P07       (0 << GPIO_DS_SHIFT)
#  define GPIO_DS_1V8_60P14        (1 << GPIO_DS_SHIFT)
#  define GPIO_DS_1V8_42P15       (2 << GPIO_DS_SHIFT)
#  define GPIO_DS_1V8_35P19       (3 << GPIO_DS_SHIFT)
#  define GPIO_DS_1V8_30P20       (7 << GPIO_DS_SHIFT)
#endif

/* 输出类型选择:开漏或推挽
 *
 */
#define GPIO_OPENDRAIN                (1 << 10)                   /* Bit10: 1=开漏输出 */
#define GPIO_PUSHPULL                 (0)                        /* Bit10: 0=推挽输出 */

/* 初始值 (仅输出):
 *
 */
#define GPIO_OUTPUT_SET        (1 << 9)    /* Bit 9: 输出模式时输出初始化值*/
#define GPIO_OUTPUT_CLEAR      (0)

/* 外部中断选择 (仅输入):
 *
 */

#define GPIO_EXTI              (1 << 9)   /* Bit 9: 配置作为外部中断 */


/* GPIO端口号：
 */

#define GPIO_PORT_SHIFT        (5)      /* Bits 8-5 */
#define GPIO_PORT_MASK         (0x0f << GPIO_PORT_SHIFT)
#  define GPIO_PORTA           (0 << GPIO_PORT_SHIFT) /* GPIOA */
#  define GPIO_PORTB           (1 << GPIO_PORT_SHIFT) /* GPIOB */
#  define GPIO_PORTC           (2 << GPIO_PORT_SHIFT) /* GPIOC */
#  define GPIO_PORTD           (3 << GPIO_PORT_SHIFT) /* GPIOD */
#  define GPIO_PORTE           (4 << GPIO_PORT_SHIFT) /* GPIOE */
#  define GPIO_PORTF           (5 << GPIO_PORT_SHIFT) /* GPIOF */
#  define GPIO_PORTX           (13 << GPIO_PORT_SHIFT) /* GPIOX */
#  define GPIO_PORTY           (14 << GPIO_PORT_SHIFT) /* GPIOY */
#  define GPIO_PORTZ           (15 << GPIO_PORT_SHIFT) /* GPIOZ */
#  define GPIO_PORTG           GPIO_PORTX /* 兼容ST命名规则*/
#  define GPIO_PORTH           GPIO_PORTY /* 兼容ST命名规则 */
#  define GPIO_PORTI           GPIO_PORTZ /* 兼容ST命名规则 */

/* GPIO引脚号：0-31
*/
#define GPIO_PIN_SHIFT         (0)      /* Bits 4-0:*/
#define GPIO_PIN_MASK          (0x1F << GPIO_PIN_SHIFT)
#  define GPIO_PIN0            (0 << GPIO_PIN_SHIFT)  /* Pin  0 */
#  define GPIO_PIN1            (1 << GPIO_PIN_SHIFT)  /* Pin  1 */
#  define GPIO_PIN2            (2 << GPIO_PIN_SHIFT)  /* Pin  2 */
#  define GPIO_PIN3            (3 << GPIO_PIN_SHIFT)  /* Pin  3 */
#  define GPIO_PIN4            (4 << GPIO_PIN_SHIFT)  /* Pin  4 */
#  define GPIO_PIN5            (5 << GPIO_PIN_SHIFT)  /* Pin  5 */
#  define GPIO_PIN6            (6 << GPIO_PIN_SHIFT)  /* Pin  6 */
#  define GPIO_PIN7            (7 << GPIO_PIN_SHIFT)  /* Pin  7 */
#  define GPIO_PIN8            (8 << GPIO_PIN_SHIFT)  /* Pin  8 */
#  define GPIO_PIN9            (9 << GPIO_PIN_SHIFT)  /* Pin  9 */
#  define GPIO_PIN10           (10 << GPIO_PIN_SHIFT) /* Pin 10 */
#  define GPIO_PIN11           (11 << GPIO_PIN_SHIFT) /* Pin 11 */
#  define GPIO_PIN12           (12 << GPIO_PIN_SHIFT) /* Pin 12 */
#  define GPIO_PIN13           (13 << GPIO_PIN_SHIFT) /* Pin 13 */
#  define GPIO_PIN14           (14 << GPIO_PIN_SHIFT) /* Pin 14 */
#  define GPIO_PIN15           (15 << GPIO_PIN_SHIFT) /* Pin 15 */
#  define GPIO_PIN16           (16 << GPIO_PIN_SHIFT) /* Pin 16 */
#  define GPIO_PIN17           (17 << GPIO_PIN_SHIFT) /* Pin 17 */
#  define GPIO_PIN18           (18 << GPIO_PIN_SHIFT) /* Pin 18 */
#  define GPIO_PIN19           (19 << GPIO_PIN_SHIFT) /* Pin 19 */
#  define GPIO_PIN20           (20 << GPIO_PIN_SHIFT) /* Pin 20 */
#  define GPIO_PIN21           (21 << GPIO_PIN_SHIFT) /* Pin 21 */
#  define GPIO_PIN22           (22 << GPIO_PIN_SHIFT) /* Pin 22 */
#  define GPIO_PIN23           (23 << GPIO_PIN_SHIFT) /* Pin 23 */
#  define GPIO_PIN24           (24 << GPIO_PIN_SHIFT) /* Pin 24 */
#  define GPIO_PIN25           (25 << GPIO_PIN_SHIFT) /* Pin 25 */
#  define GPIO_PIN26           (26 << GPIO_PIN_SHIFT) /* Pin 26 */
#  define GPIO_PIN27           (27 << GPIO_PIN_SHIFT) /* Pin 27 */
#  define GPIO_PIN28           (28 << GPIO_PIN_SHIFT) /* Pin 28 */
#  define GPIO_PIN29           (29 << GPIO_PIN_SHIFT) /* Pin 29 */
#  define GPIO_PIN30           (30 << GPIO_PIN_SHIFT) /* Pin 30 */
#  define GPIO_PIN31           (31 << GPIO_PIN_SHIFT) /* Pin 31 */


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_gpioirq_initialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

#ifdef CONFIG_HPM_GPIO_IRQ
void hpm_gpioirq_initialize(void);
#else
#  define hpm_gpioirq_initialize()
#endif

/****************************************************************************
 * Name: hpm_config_gpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int hpm_config_gpio(uint32_t pinset);


/****************************************************************************
 * Name: hpm_unconfig_gpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int hpm_unconfig_gpio(uint32_t pinset);


/****************************************************************************
 * Name: hpm_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void hpm_gpio_write(uint32_t pinset, bool value);

/****************************************************************************
 * Name: hpm_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool hpm_gpio_read(uint32_t pinset);


/****************************************************************************
 * Name: hpm_gpio_setevent
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 *
 * Input Parameters:
 *  - pinset:      GPIO pin configuration
 *  - risingedge:  Enables interrupt on rising edges
 *  - fallingedge: Enables interrupt on falling edges
 *  - event:       Generate event when set
 *  - func:        When non-NULL, generate interrupt
 *  - arg:         Argument passed to the interrupt callback
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure indicating the
 *   nature of the failure.
 *
 ****************************************************************************/

int hpm_gpio_setevent(uint32_t pinset, bool risingedge, bool fallingedge,
                       bool event, xcpt_t func, void *arg);

/****************************************************************************
 * Name: hpm_gpioirq_configure
 *
 * Description:
 *   Configure an interrupt for the specified GPIO pin.
 *
 ****************************************************************************/

#ifdef CONFIG_HPM_GPIO_IRQ
int hpm_gpioirq_configure(uint32_t pinset);
#else
#  define hpm_gpioirq_configure(pinset)
#endif

/****************************************************************************
 * Name: hpm_gpioirq_enable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_HPM_GPIO_IRQ
int hpm_gpioirq_enable(int irq);
#else
#  define hpm_gpioirq_enable(irq)
#endif

/****************************************************************************
 * Name: hpm_gpioirq_disable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_HPM_GPIO_IRQ
int hpm_gpioirq_disable(int irq);
#else
#  define hpm_gpioirq_disable(irq)
#endif
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_HPMICRO_GPIO_H */
