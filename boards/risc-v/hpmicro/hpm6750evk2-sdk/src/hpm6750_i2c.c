/****************************************************************************
 * boards/risc-v/hpmicro/hpm6750evk2-sdk/src/hpm6750_i2c.c
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
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/can/can.h>

#include "board.h"
#include "chip.h"
#include "hpm.h"
#include "hpm6750evk2.h"
#include "hpm_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define  HPM6750EVK2_I2C_PORT    (0)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_HPM6750_I2C0_MASTER
struct i2c_master_s * g_i2c0_dev;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm6750_i2cbus_pins_initialize
 *
 * Description:
 *   Initialize the selected I2C port pins
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple CAN interfaces)
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

int hpm6750_i2cbus_pins_initialize(int port)
{

#if (defined(CONFIG_HPM6750_I2C0_MASTER) || defined(CONFIG_HPM6750_I2C0_SLAVE))
  if (port == 0)
    {
      init_i2c_pins(HPM_I2C0);   
    }
  else
#endif
    {
      canerr("ERROR: hpm6750evk2 i2c pins Unsupported port %d\n", port);
      return -1;
    }
  
  return 0;
}

#if defined(CONFIG_HPM6750_I2C0_MASTER) || defined(CONFIG_HPM6750_I2C1_MASTER) || \
    defined(CONFIG_HPM6750_I2C2_MASTER) || defined(CONFIG_HPM6750_I2C3_MASTER)

/****************************************************************************
 * Name: hpm6750evk2_i2cdev_initialize
 *
 * Description:
 *   Called to configure I2C
 *
 ****************************************************************************/

int hpm6750evk2_i2cdev_initialize(uint8_t port)
{
  int ret = ERROR;

#ifdef CONFIG_HPM6750_I2C0_MASTER
  g_i2c0_dev = hpm6750_i2cbus_initialize(0);
  if (g_i2c0_dev == NULL)
    {
      syslog(LOG_ERR, "ERROR: hpm6750_i2cbus_initialize(0) failed: %d\n",
             ret);
      ret = -ENODEV;
    }
  else
    {
#  ifdef CONFIG_I2C_DRIVER
      ret = i2c_register(g_i2c0_dev, 0);
#  endif
    }

#endif

  return ret;
}
#endif


