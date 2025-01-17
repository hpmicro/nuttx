/****************************************************************************
 * boards/risc-v/hpmicro/hpm6750evk2-sdk/src/hpm6750_can.c
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
#include "hpm_can.h"

#ifdef CONFIG_HPM_CAN_DRV

#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define  HPM6750EVK2_CAN_PORT    (0)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_init_can_pins
 *
 * Description:
 *   Initialize the selected CAN port pins
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple CAN interfaces)
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

int hpm_init_can_pins(int port)
{
#ifdef CONFIG_HPM_CAN0
  if (port == 0)
    {
      board_init_can(HPM_CAN0);  
    }
  else
#endif
    {
      canerr("ERROR: hpm6750evk2 can pins Unsupported port %d\n", port);
      return -1;
    }
  
  return 0;
}

/****************************************************************************
 * Name: hpm_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/
#ifdef CONFIG_HPM_CAN_CHARDRIVER

int hpm6750_can_setup(void)
{
#if defined(CONFIG_HPM_CAN0)
  struct can_dev_s *can;
  int ret;

  /* Call hpm_caninitialize() to get an instance of the CAN interface */

  can = hpm_caninitialize(HPM6750EVK2_CAN_PORT);
  if (can == NULL)
    {
      canerr("ERROR:  Failed to get CAN interface\n");
      return -ENODEV;
    }

  /* Register the CAN driver at "/dev/can0" */

  ret = can_register("/dev/can0", can);
  if (ret < 0)
    {
      canerr("ERROR: can_register failed: %d\n", ret);
      return ret;
    }

  return OK;
#else
  return -ENODEV;
#endif
}

#endif

/****************************************************************************
 * Name: hpm_cansock_setup
 *
 * Description:
 *  Initialize CAN socket interface
 *
 ****************************************************************************/

#ifdef CONFIG_HPM_CAN_SOCKET

int hpm6750_cansock_setup(void)
{
  int ret = OK;

#if defined(CONFIG_HPM_CAN0)
  ret =  hpm_cansockinitialize(HPM6750EVK2_CAN_PORT);
  if (ret < 0)
    {
      canerr("ERROR:  Failed to get CAN sock interface %d\n", ret);
      goto errout;
    }
  return ret;
#endif
errout:return -1;
}

#endif
