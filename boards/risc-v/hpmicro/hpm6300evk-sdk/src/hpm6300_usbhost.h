/****************************************************************************
 * boards/risc-v/hpmicro/hpm6300evk/src/hpm6300_usbhost.h
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

#ifndef __BOARDS_RISCV_HPMICRO_HPM6300EVK_SRC_HPM6300_USBHOST_H
#define __BOARDS_RISCV_HPMICRO_HPM6300EVK_SRC_HPM6300_USBHOST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_HPM_USBOTG) && defined(CONFIG_USBHOST)

int hpm6300_usbhost_initialize(void);

#endif /* CONFIG_HPM_USBOTG || CONFIG_USBHOST */

#endif /* __BOARDS_RISCV_HPMICRO_HPM6300EVK_SRC_HPM6300_USBHOST_H */

