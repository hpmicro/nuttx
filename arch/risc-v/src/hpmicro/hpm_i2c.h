/****************************************************************************
 * arch/risc-v/src/hpmicro/hpm_i2c.h
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

#ifndef __ARCH_ARM_SRC_HPMICRO_COMMON_HPM_I2C_H
#define __ARCH_ARM_SRC_HPMICRO_COMMON_HPM_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/i2c/i2c_slave.h>
#include "hpm_i2c.h"

#ifndef __ASSEMBLY__
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
#ifdef CONFIG_HPM_I2C_DRV

/****************************************************************************
 * Name: hpm_i2cbus_pins_initialize
 *
 * Description:
 *   Initialize the selected I2C port. And initializ the pins,include SCL and SDA
 *
 * Input Parameter:
 *   Port number (for hardware that has multiple I2C interfaces)
 *
 * Returned Value:
*   OK on success, ERROR when internal reference count mismatch or dev
 *   points to invalid hardware device.
 *
 ****************************************************************************/

int  hpm_i2cbus_pins_initialize(int port);

#endif

#ifdef CONFIG_HPM_I2C_MASTER
/****************************************************************************
 * Name: hpm_i2cbus_initialize
 *
 * Description:
 *   Initialize the selected I2C port. And return a unique instance of struct
 *   struct i2c_master_s.  This function may be called to obtain multiple
 *   instances of the interface, each of which may be set up with a
 *   different frequency and slave address.
 *
 * Input Parameter:
 *   Port number (for hardware that has multiple I2C interfaces)
 *
 * Returned Value:
 *   Valid I2C device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2c_master_s *hpm_i2cbus_initialize(int port);

/****************************************************************************
 * Name: hpm_i2cbus_uninitialize
 *
 * Description:
 *   De-initialize the selected I2C port, and power down the device.
 *
 * Input Parameter:
 *   Device structure as returned by the rp2040_i2cbus_initialize()
 *
 * Returned Value:
 *   OK on success, ERROR when internal reference count mismatch or dev
 *   points to invalid hardware device.
 *
 ****************************************************************************/

int hpm_i2cbus_uninitialize(struct i2c_master_s *dev);

#endif

#ifdef CONFIG_I2C_SLAVE

/****************************************************************************
 * Name: hpm_i2c_slave_initialize
 *
 * Description:
 *   Initialize I2C controller zero for slave operation, and return a pointer
 *   to the instance of struct i2c_slave_s.  This function should only be
 *   called once of a give controller.
 *
 * Input Parameters:
 *   ten_bin       - Set true for 10-bit I2C addressing.
 *   rx_buffer     - Buffer for data transmitted to us by an I2C master.
 *   rx_buffer_len - Length of rx_buffer.
 *   callback      - Callback function called when messages are received.
 *
 * Returned Value:
 *   Valid I2C device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2c_slave_s * hpm_i2c_slave_initialize
                           (
                            uint8_t              port,
                            uint8_t              *rx_buffer,
                            size_t                rx_buffer_len,
                            i2c_slave_callback_t *callback);

#endif 

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_HPMICRO_COMMON_HPM_I2C_H */
