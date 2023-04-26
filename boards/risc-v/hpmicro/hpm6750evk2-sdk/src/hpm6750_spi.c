/****************************************************************************
 * boards/risc-v/hpmicro/hpm6750evk2/src/hpm6750_spi.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "board.h"
#include "chip.h"
#include "hpm.h"
#include "hpm6750evk2.h"
#include "hpm_spi.h"
#include "hpm_spi_drv.h"

#include "hpm_gpio_drv.h"
#include "hpm_gpiom_drv.h"

#ifdef CONFIG_HPM_SPI2
#  include <nuttx/spi/spi.h>
#  include <nuttx/lcd/lcd.h>
#  include <nuttx/lcd/st7789.h>
#endif 

#ifdef CONFIG_HPM_SPI_DRV

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_HPM_SPI2
struct spi_dev_s *g_spidev2 = NULL;
static struct lcd_dev_s *g_lcd = NULL;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm6750_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the hpm6750evk2
 *   board.
 *
 ****************************************************************************/

void weak_function hpm6750_spidev_initialize(void)
{
#ifdef CONFIG_HPM6750_SPI2
    board_init_spi_clock(BOARD_APP_SPI_BASE);
    board_init_spi_pins_with_gpio_as_cs(BOARD_APP_SPI_BASE);

    HPM_IOC->PAD[IOC_PAD_PZ08].FUNC_CTL = IOC_PZ08_FUNC_CTL_GPIO_Z_08;
    HPM_BIOC->PAD[IOC_PAD_PZ08].FUNC_CTL = IOC_PZ08_FUNC_CTL_SOC_PZ_08;

    gpio_set_pin_output(HPM_GPIO0, GPIO_OE_GPIOZ, 8);
    gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOZ, 8, gpiom_soc_gpio0);
    
    HPM_IOC->PAD[IOC_PAD_PZ09].FUNC_CTL = IOC_PZ09_FUNC_CTL_GPIO_Z_09;
    HPM_BIOC->PAD[IOC_PAD_PZ09].FUNC_CTL = IOC_PZ09_FUNC_CTL_SOC_PZ_09;

    gpio_set_pin_output(HPM_GPIO0, GPIO_OE_GPIOZ, 9);
    gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOZ, 9, gpiom_soc_gpio0);

    gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOZ , 9, 0);
    up_mdelay(400);
    gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOZ , 9, 1);
#endif
}

/****************************************************************************
 * Name:  hpm6750_spi0/1/2/3select and hpm6750_spi0/1/2/3/5status
 *
 * Description:
 *   The external functions, hpm6750_spi0/2/3select and stm32_spi1/2/3status
 *   must be provided by board-specific logic.  They are implementations of
 *   the select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h). All other methods (including
 *   stm32_spibus_initialize()) are provided by common STM32 logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide stm32_spi1/2/3select() and stm32_spi1/2/3status() functions
 *      in your board-specific logic.  These functions will perform chip
 *      selection and status operations using GPIOs in the way your board is
 *      configured.
 *   3. Add a calls to stm32_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by stm32_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_HPM_SPI0
void hpm_spi0select(struct spi_dev_s *dev,
                      uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n",
          (int)devid, selected ? "assert" : "de-assert");
}

uint8_t hpm_spi0status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_HPM_SPI1
void hpm_spi1select(struct spi_dev_s *dev,
                      uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n",
         (int)devid, selected ? "assert" : "de-assert");
}

uint8_t hpm_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif


#ifdef CONFIG_HPM_SPI2
void hpm_spi2select(struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n",
          (int)devid, selected ? "assert" : "de-assert");

#if defined(CONFIG_LCD_ST7789)
  if (devid == SPIDEV_DISPLAY(0))
    {
      board_write_spi_cs(BOARD_SPI_CS_PIN,!selected);
    }
#endif
}

uint8_t hpm_spi2status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_HPM_SPI3
void hpm_spi3select(struct spi_dev_s *dev,
                      uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n",
          (int)devid, selected ? "assert" : "de-assert");
}

uint8_t hpm_spi3status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif


/****************************************************************************
 * Name: stm32_spi1cmddata
 *
 * Description:
 *   Set or clear the SH1101A A0 or SD1306 D/C n bit to select data (true)
 *   or command (false). This function must be provided by platform-specific
 *   logic. This is an implementation of the cmddata method of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *
 * Input Parameters:
 *
 *   spi - SPI device that controls the bus the device that requires the CMD/
 *         DATA selection.
 *   devid - If there are multiple devices on the bus, this selects which one
 *         to select cmd or data.  NOTE:  This design restricts, for example,
 *         one one SPI display per SPI bus.
 *   cmd - true: select command; false: select data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
#ifdef CONFIG_HPM_SPI0
int hpm6750_spi0cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_HPM_SPI1
int hpm_spi1cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_HPM_SPI2
int hpm_spi2cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  if (devid == SPIDEV_DISPLAY(0))
    {
      /*  This is the Data/Command control pad which determines whether the
       *  data bits are data or a command.
       */
      gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOZ , 8, !cmd);

      return OK;
    }

  return -ENODEV;
}
#endif

#ifdef CONFIG_HPM6750_SPI3
int hpm6750_spi3cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif



#endif /* CONFIG_SPI_CMDDATA */

/****************************************************************************
 * Name: hpm6750_spi2initialize
 *
 * Description:
 *   Initialize the selected SPI port.
 *   As long as the method stm32_spibus_initialize recognized the initialized
 *   state of the spi device by the spi enable flag of the cr1 register, it
 *   isn't safe to disable the spi device outside of the nuttx spi interface
 *   structure. But this has to be done as long as the nuttx spi interface
 *   doesn't support bidirectional data transfer for multiple devices share
 *   one spi bus. This wrapper does nothing else than store the initialized
 *   state of the spi device after the first initializing and should be used
 *   by each driver who shares the spi5 bus.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

#ifdef CONFIG_HPM6750_SPI2
struct spi_dev_s *hpm6750_spi2initialize(void)
{
  if (!g_spidev2)
    {
      hpm6750_spidev_initialize();
      g_spidev2 = hpm_spibus_initialize(2);
    }

  return g_spidev2;
}

/****************************************************************************
 * Name:  board_lcd_getdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This
 *   allows support for multiple LCD devices.
 *
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int devno)
{
  g_lcd = st7789_lcdinitialize(g_spidev2);
  if (!g_lcd)
    {
      lcderr("ERROR: Failed to bind SPI port 2 to LCD %d\n",
      devno);
    }
  else
    {
      lcdinfo("SPI port 2 bound to LCD %d\n", devno);
      return g_lcd;
    }

  return NULL;
}

/****************************************************************************
 * Name:  board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is
 *   fully initialized, display memory cleared, and the LCD ready to use, but
 *   with the power setting at 0 (full off).
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
  hpm6750_spi2initialize();
  st7789_lcdinitialize(g_spidev2);
  return OK;
}


#endif



#endif /* CONFIG_STM32_SPI1 || ... CONFIG_STM32_SPI5 */
