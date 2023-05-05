/****************************************************************************
 * boards/risc-v/hpmicro/hpm6750evk2/src/hpm6750_bringup.c
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

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/input/buttons.h>

#include "board.h"

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_HPM_I2C_DRV
#  include "hpm6750_i2c.h"
#endif

#ifdef CONFIG_HPM_CAN_DRV
#  include"hpm6750_can.h"
#endif

#ifdef CONFIG_LCD_DEV
#  include <nuttx/lcd/lcd_dev.h>
#endif

#ifdef CONFIG_SPI_DRIVER
#  include <nuttx/spi/spi.h>
#  include <nuttx/spi/spi_transfer.h>
#endif

#ifdef CONFIG_TIMER
#  include <nuttx/timers/timer.h>
#  include "hpm_tim_lowerhalf.h"
#endif

#ifdef CONFIG_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "hpm_rtc.h"
#endif

#ifdef CONFIG_HPM_ADC12_DRV
#  include "hpm_adc12.h"
#endif

#ifdef CONFIG_HPM_ADC16_DRV
#  include "hpm_adc16.h"
#endif

#ifdef CONFIG_PWM
#  include "hpm_pwm_lowerhalf.h"
#endif

#if defined(CONFIG_HPM_USBOTG) && defined(CONFIG_USBHOST)
#  include "hpm6750_usbhost.h"
#endif

#ifdef CONFIG_HPM_USBDEV
#  include "hpm_usbdev.h"
#endif

#ifdef CONFIG_CDCACM
#  include <nuttx/usb/cdcacm.h>
#endif

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#ifdef CONFIG_HPM_SDXC_DRV
# include "hpm_sdmmc.h"
#include "hpm6750_sdmmc.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/
#ifdef CONFIG_HPM6750_SPI2
extern struct spi_dev_s *hpm6750_spi2initialize(void);
#endif
/****************************************************************************
 * Name: hpm6750_bringup
 ****************************************************************************/

int hpm6750_bringup(void)
{
  int ret = OK;

  board_init();
  board_ungate_mchtmr_at_lp_mode();
  
#ifdef CONFIG_FS_BINFS

  /* Mount the binfs file system */

  ret = nx_mount(NULL, "/bin", "binfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount binfs at /bin: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS

  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      serr("ERROR: Failed to mount procfs at %s: %d\n", "/proc", ret);
    }
#endif

#ifdef CONFIG_USERLED

  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_HPM6750_I2C0_MASTER)

  /* Initialize I2C buses */

  ret = hpm6750evk2_i2cdev_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: i2c0 hpm6750evk2_i2cdev_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_HPM_CAN_CHARDRIVER

  /* Initialize CAN and register the CAN driver. */

  ret = hpm6750_can_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: hpm6750_can_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_HPM_CAN_SOCKET

  /* Initialize CAN socket interface */

  ret = hpm6750_cansock_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: hpm6750_cansock_setup failed: %d\n", ret);
    }
#endif


#ifdef CONFIG_SPI_DRIVER
#  ifdef CONFIG_HPM6750_SPI2
  struct spi_dev_s *spi;
  spi = hpm6750_spi2initialize();
  if (spi == NULL)
    {
      syslog(LOG_ERR, "Failed to initialize SPI2 \n");
      return -ENODEV;
    }

  ret = spi_register(spi, 2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to register /dev/spi2: %d\n", ret);
    }
#  endif

#ifdef CONFIG_LCD_DEV
  ret = lcddev_register(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: lcddev_register() failed: %d\n", ret);
      return ret;
    }
#endif

#endif

#ifdef CONFIG_PWM
#  ifdef CONFIG_HPM6750_PWM2
  struct pwm_lowerhalf_s *pwm;

  /* Initialize PWM and register the PWM driver */

  pwm = hpm6750_pwminitialize(2);
  if (pwm == NULL)
    {
      syslog(LOG_DEBUG, "ERROR: hpm6750_pwminitialize failed\n");
    }
  else
    {
      ret = pwm_register("/dev/pwm2", pwm);
      if (ret < 0)
        {
          syslog(LOG_DEBUG, "ERROR: pwm_register failed: %d\n", ret);
        }
    }
#  endif
#endif

#if defined(CONFIG_TIMER)
#  if defined(CONFIG_HPM6750_TIMER0)
  ret = hpm_timer_initialize("/dev/timer0", 0);
  if (ret < 0)
    {
      syslog(LOG_DEBUG,
        "Failed to initialize /dev/timer0 Driver: %d\n", ret);
      return ret;
    }
#  endif
#  if defined(CONFIG_HPM6750_TIMER1)
  ret = hpm_timer_initialize("/dev/timer1", 1);
  if (ret < 0)
    {
      syslog(LOG_DEBUG,
        "Failed to initialize /dev/timer1 Driver: %d\n", ret);
      return ret;
    }
#  endif
#  if defined(CONFIG_HPM6750_TIMER2)
  ret = hpm_timer_initialize("/dev/timer2", 2);
  if (ret < 0)
    {
      syslog(LOG_DEBUG,
        "Failed to initialize /dev/timer2 Driver: %d\n", ret);
      return ret;
    }
#  endif
#  if defined(CONFIG_HPM6750_TIMER3)
  ret = hpm_timer_initialize("/dev/timer3", 3);
  if (ret < 0)
    {
      syslog(LOG_DEBUG,
        "Failed to initialize /dev/timer3 Driver: %d\n", ret);
      return ret;
    }
#  endif
#  if defined(CONFIG_HPM6750_TIMER4)
  ret = hpm_timer_initialize("/dev/timer4", 4);
  if (ret < 0)
    {
      syslog(LOG_DEBUG,
        "Failed to initialize /dev/timer4 Driver: %d\n", ret);
      return ret;
    }
#  endif
#  if defined(CONFIG_HPM6750_TIMER5)
  ret = hpm_timer_initialize("/dev/timer5", 5);
  if (ret < 0)
    {
      syslog(LOG_DEBUG,
        "Failed to initialize /dev/timer5 Driver: %d\n", ret);
      return ret;
    }
#  endif
#  if defined(CONFIG_HPM6750_TIMER6)
  ret = hpm_timer_initialize("/dev/timer6", 6);
  if (ret < 0)
    {
      syslog(LOG_DEBUG,
        "Failed to initialize /dev/timer6 Driver: %d\n", ret);
      return ret;
    }
#  endif
#  if defined(CONFIG_HPM6750_TIMER7)
  ret = hpm_timer_initialize("/dev/timer7", 7);
  if (ret < 0)
    {
      syslog(LOG_DEBUG,
        "Failed to initialize /dev/timer7 Driver: %d\n", ret);
      return ret;
    }
#  endif
#endif

#ifdef CONFIG_RTC_DRIVER
  struct rtc_lowerhalf_s *lower;
  lower = hpm6750_rtc_lowerhalf();
  rtc_initialize(0, lower);
#endif

#ifdef CONFIG_HPM_ADC12_DRV
	/* Initialize ADC12 and register the ADC driver. */
  ret = hpm_adc12_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: hpm_adc12_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_HPM_ADC16_DRV
  /* Initialize ADC16 and register the ADC driver. */
  ret = hpm_adc16_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: hpm_adc16_setup failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_HPM_USBOTG) && defined(CONFIG_USBHOST)
  ret = hpm6750_usbhost_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to start USB host services: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_HPM_USBDEV
  hpm_usbdev_initialize(CONFIG_HPM_USBDEV_INSTANCE);
#endif

#if defined(CONFIG_CDCACM) && !defined(CONFIG_CDCACM_CONSOLE) && !defined(CONFIG_CDCACM_COMPOSITE)
  /* Initialize CDCACM */

  ret = cdcacm_initialize(0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: cdcacm_initialize failed: %d\n", ret);
    }
#endif /* CONFIG_CDCACM & !CONFIG_CDCACM_CONSOLE */

#ifdef CONFIG_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to start USB monitor: %d\n", ret);
      return ret;
#endif	


#ifdef CONFIG_HPM_SDXC_DRV
  /* Initialize SDXC and register the SD driver. */
  ret = hpm_sdio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: hpm_sdio_initialize failed: %d\n", ret);
    }
#endif

  return ret;
}


