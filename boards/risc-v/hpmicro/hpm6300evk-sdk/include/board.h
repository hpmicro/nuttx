/****************************************************************************
 * boards/risc-v/hpmicro/hpm6300evk/include/board.h
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

#ifndef __BOARDS_RISCV_HPMICRO_HPM6300EVK_INCLUDE_BOARD_H
#define __BOARDS_RISCV_HPMICRO_HPM6300EVK_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef BOARD_APP_CORE
#define BOARD_APP_CORE  HPM_CORE0  
#endif

/* LED definitions **********************************************************/

/* Define how many LEDs this board has (needed by userleds) */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_LED3        2
#define BOARD_NLEDS       1

/* The board has only one controllable LED */

#define LED_STARTED       0  /* No LEDs */
#define LED_PANIC         7  /* LED1 / LED2 blinking */

/* GPIO Configuration */

#define BOARD_NGPIOIN     1 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    1 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

#define BOARD_GPIO_IN1    (IOC_PAD_PZ02)
#define BOARD_GPIO_OUT1   (IOC_PAD_PA07)
#define BOARD_GPIO_INT1   (IOC_PAD_PZ03)

/* ADC Configuration */

#define BOARD_APP_ADC_SEQ_DMA_BUFF_LEN_IN_4BYTES (1024U)
#define BOARD_APP_ADC_PMT_DMA_BUFF_LEN_IN_4BYTES (48U)
#define BOARD_APP_SEQ_START_POS                  (0U)


/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_RISCV_HPMICRO_HPM6300EVK_INCLUDE_BOARD_H */
