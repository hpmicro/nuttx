/****************************************************************************
 * arch/risc-v/src/hpmicro/hpm6750/hpm_spi.h
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

#ifndef __ARCH_RISCV_SRC_HPMICRO_HPM6750_HPM_SPI_H
#define __ARCH_RISCV_SRC_HPMICRO_HPM6750_HPM_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
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

#define HPMICRO_SPI_CLK_MAX   (80000000UL)

/****************************************************************************
 * Name: hpm6750_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *hpm6750_spibus_initialize(int bus);

/****************************************************************************
 * Name:  hpm6750_spi1/2/...select and hpm6750_spi1/2/...status
 *
 * Description:
 *   The external functions, hpm6750_spi1/2/...select, hpm6750_spi1/2/...status,
 *   and hpm6750_spi1/2/...cmddata must be provided by board-specific logic.
 *   These are implementations of the select, status, and cmddata methods of
 *   the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi/spi.h). All other methods (including
 *   hpm6750_spibus_initialize()) are provided by common STM32 logic.  To use
 *   this common SPI logic on your board:
 *
 *   1. Provide logic in hpm6750_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide hpm6750_spi1/2/...select() and hpm6750_spi1/2/...status()
 *      functions in your board-specific logic.  These functions will
 *      perform chip selection and status operations using GPIOs in the way
 *      your board is configured.
 *   3. If CONFIG_SPI_CMDDATA is defined in your NuttX configuration file,
 *      then provide hpm6750_spi1/2/...cmddata() functions in your board-
 *      specific logic. These functions will perform cmd/data selection
 *      operations using GPIOs in the way your board is configured.
 *   4. Add a calls to hpm6750_spibus_initialize() in your low level
 *      application initialization logic
 *   5. The handle returned by hpm6750_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_HPM6750_SPI0
void hpm6750_spi0select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected);
uint8_t hpm6750_spi0status(struct spi_dev_s *dev, uint32_t devid);
int hpm6750_spi0cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_HPM6750_SPI1
void hpm6750_spi1select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected);
uint8_t hpm6750_spi1status(struct spi_dev_s *dev, uint32_t devid);
int hpm6750_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_HPM6750_SPI2
void hpm6750_spi2select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected);
uint8_t hpm6750_spi2status(struct spi_dev_s *dev, uint32_t devid);
int hpm6750_spi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_HPM6750_SPI3
void hpm6750_spi3select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected);
uint8_t hpm6750_spi3status(struct spi_dev_s *dev, uint32_t devid);
int hpm6750_spi3cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_HPMICRO_HPM6750_HPM_SPI_H */
