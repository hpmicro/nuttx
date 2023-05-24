/****************************************************************************
 * arch/risc-v/src/hpmicro/hpm_i2c_slave.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_slave.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>

#include <arch/board/board.h>

#include "chip.h"
#include "riscv_internal.h"
#include "hpm_i2c.h"
#include "hpm_i2c_drv.h"
#include "hpm_i2c_regs.h"
#include "hpm_clock_drv.h"

#ifdef CONFIG_I2C_SLAVE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct 
{
  struct i2c_slave_s     dev;          /* Generic I2C device */
  I2C_Type               *ptr; 
  clock_name_t           i2c_clock;    /* i2c clock */
  i2c_config_t           i2c_config;   /* i2c config */
  uint16_t               irqid;        /* IRQ for this device */
  int8_t                 controller;   /* I2C controller number */
  uint16_t               slave_address;

  i2c_config_t           config;

  uint8_t               *rx_buffer;
  uint8_t               *rx_buf_ptr;
  uint8_t               *rx_buf_end;

  const uint8_t         *tx_buffer;
  const uint8_t         *tx_buf_ptr;
  const uint8_t         *tx_buf_end;

  i2c_slave_callback_t  *callback;     /* Callback function */
  void                  *callback_arg; /* Argument for callback */
} hpm_i2c_slave_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int hpm_i2c_slave_init(hpm_i2c_slave_t  *dev);

static int hpm_i2c_slave_interrupt(int   irq,
                         void *context,
                         void *arg);

static void hpm_i2c_rtxint(struct i2c_slave_s  *dev, bool enable);

static int hpm_set_own_address(struct i2c_slave_s  *dev,
                              int                  address,
                              int                  nbits);

static int hpm_write(struct i2c_slave_s  *dev,
                    const uint8_t       *buffer,
                    int                  length);

static int hpm_read(struct i2c_slave_s  *dev,
                   uint8_t             *buffer,
                   int                  length);

static int hpm_register_callback(struct i2c_slave_s   *dev,
                                i2c_slave_callback_t *callback,
                                void                 *arg);

static void hpm_enable_i2c_slave(struct i2c_slave_s *dev, bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct i2c_slaveops_s i2c_slaveops =
{
  .setownaddress                   = hpm_set_own_address,
  .write                           = hpm_write,
  .read                            = hpm_read,
  .registercallback                = hpm_register_callback,
};

#ifdef CONFIG_HPM_I2C0_SLAVE

hpm_i2c_slave_t i2c0_slave_dev =
{
  .dev.ops                         = &i2c_slaveops, /* Slave operations */
  .controller                      = 0,             /* I2C controller number */
  .ptr                             = HPM_I2C0,
  .i2c_clock                       = clock_i2c0,
  .slave_address                   = CONFIG_HPM_I2C0_SLAVE_ADDR
  .config.i2c_mode                 = CONFIG_HPM_I2C0_SLAVE_MODE,
  .config.is_10bit_addressing      = CONFIG_HPM_I2C0_SLAVE_10BIT_ADDR,
  .irqid                           = HPM_IRQn_I2C0,
};

#endif

#ifdef CONFIG_HPM_I2C1_SLAVE

hpm_i2c_slave_t i2c1_slave_dev =
{
  .dev.ops                         = &i2c_slaveops, /* Slave operations */
  .controller                      = 1,             /* I2C controller number */
  .ptr                             = HPM_I2C1,
  .i2c_clock                       = clock_i2c1,
  .slave_address                   = CONFIG_HPM_I2C1_SLAVE_ADDR
  .config.i2c_mode                 = CONFIG_HPM_I2C1_SLAVE_MODE,
  .config.is_10bit_addressing      = CONFIG_HPM_I2C1_SLAVE_10BIT_ADDR,
  .irqid                           = HPM_IRQn_I2C1,
};

#endif

#ifdef CONFIG_HPM_I2C2_SLAVE

hpm_i2c_slave_t i2c2_slave_dev =
{
  .dev.ops                         = &i2c_slaveops, /* Slave operations */
  .controller                      = 2,             /* I2C controller number */
  .ptr                             = HPM_I2C2,
  .i2c_clock                       = clock_i2c2,
  .slave_address                   = CONFIG_HPM_I2C2_SLAVE_ADDR,
  .config.i2c_mode                 = CONFIG_HPM_I2C2_SLAVE_MODE,
  .config.is_10bit_addressing      = CONFIG_HPM_I2C2_SLAVE_10BIT_ADDR,
  .irqid                           = HPM_IRQn_I2C2,
};

#endif

#ifdef CONFIG_HPM_I2C3_SLAVE

hpm_i2c_slave_t i2c3_slave_dev =
{
  .dev.ops                         = &i2c_slaveops, /* Slave operations */
  .controller                      = 3,             /* I2C controller number */
  .ptr                             = HPM_I2C3,
  .i2c_clock                       = clock_i2c3,
  .slave_address                   = CONFIG_HPM_I2C3_SLAVE_ADDR,
  .config.i2c_mode                 = CONFIG_HPM_I2C3_SLAVE_MODE,
  .config.is_10bit_addressing      = CONFIG_HPM_I2C3_SLAVE_10BIT_ADDR,
  .irqid                           = HPM_IRQn_I2C3,
};

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_i2c_slave_init
 *
 * Description:
 *   The I2C init
 *
 ****************************************************************************/

static int hpm_i2c_slave_init(hpm_i2c_slave_t *dev)
{
  uint32_t freq;
  hpm_stat_t stat;
  freq = clock_get_frequency(dev->i2c_clock);
  stat = i2c_init_slave(dev->ptr, freq, &dev->i2c_config, dev->slave_address);
  if (stat != status_success) 
    {
        return -1;
    }
  return 0;
}

/****************************************************************************
 * Name: hpm_i2c_slave_interrupt
 *
 * Description:
 *   The I2C Interrupt Handler
 *
 ****************************************************************************/

static int hpm_i2c_slave_interrupt(int irq, void *context, void *arg)
{
  hpm_i2c_slave_t *priv = (hpm_i2c_slave_t *)arg;
  volatile uint32_t status, _irq;
  status = i2c_get_status(priv->ptr);
  _irq = priv->ptr->INTEN;
  uint8_t dir = (uint8_t)I2C_CTRL_DIR_GET(priv->ptr->CTRL);
  /* address hit */

  if (status & I2C_EVENT_ADDRESS_HIT) 
    {
      if (I2C_DIR_SLAVE_READ == dir)
        {
            i2c_enable_irq(priv->ptr, I2C_EVENT_FIFO_FULL);
        } 
      else 
        {
            i2c_enable_irq(priv->ptr, I2C_EVENT_FIFO_EMPTY);
        }
        i2c_disable_irq(priv->ptr, I2C_EVENT_ADDRESS_HIT);
        i2c_clear_status(priv->ptr, I2C_EVENT_ADDRESS_HIT);
    }
  
  /* receive */
  
  if (status & I2C_EVENT_FIFO_FULL) 
    {
        while (!i2c_fifo_is_empty(priv->ptr)) 
          {
            if (priv->rx_buf_ptr < priv->rx_buf_end)
              {
                *(priv->rx_buf_ptr++) = (uint8_t)I2C_DATA_DATA_GET(priv->ptr->DATA);
              }
            else
              {
                i2c_disable_irq(priv->ptr, I2C_EVENT_FIFO_FULL);
                if (priv->callback)
                  {
                    priv->callback(priv, priv->rx_buf_ptr - priv->rx_buffer);
                    priv->rx_buf_ptr = priv->rx_buffer;
                  }               
              }
          }
        i2c_clear_status(priv->ptr, I2C_EVENT_FIFO_FULL);
    }

    /* transmit */

    if ((status & I2C_EVENT_FIFO_EMPTY) && (_irq & I2C_EVENT_FIFO_EMPTY)) 
      {
        i2c_clear_status(priv->ptr, I2C_EVENT_FIFO_EMPTY);
        status = i2c_get_status(priv->ptr);
        while (!i2c_fifo_is_full(priv->ptr)) 
          {
            if (priv->tx_buf_ptr < priv->tx_buf_end)
              {
                priv->ptr->DATA = I2C_DATA_DATA_SET(*(priv->tx_buf_ptr++));
              }
            else
              {
                i2c_disable_irq(priv->ptr, I2C_EVENT_FIFO_EMPTY);
                break;
              }      
          }
        i2c_clear_status(priv->ptr, I2C_EVENT_FIFO_FULL);
      }
    
    /* complete */

    if (status & I2C_EVENT_TRANSACTION_COMPLETE)
      {
        if (I2C_DIR_SLAVE_READ == dir)
          {
            while (!i2c_fifo_is_empty(priv->ptr)) 
              {
                if (priv->rx_buf_ptr < priv->rx_buf_end)
                  {
                    *(priv->rx_buf_ptr++) = (uint8_t)I2C_DATA_DATA_GET(priv->ptr->DATA);
                  }
              }
            priv->callback(priv, priv->rx_buf_ptr - priv->rx_buffer);
            priv->rx_buf_ptr = priv->rx_buffer;
          }
        else
          {
            if (priv->callback)
              {
                priv->callback(priv, priv->rx_buf_ptr - priv->rx_buffer);
                priv->rx_buf_ptr = priv->rx_buffer;
              }
          }         
        i2c_disable_irq(priv->ptr, I2C_EVENT_TRANSACTION_COMPLETE);
        i2c_clear_status(priv->ptr, I2C_EVENT_TRANSACTION_COMPLETE);
      }     
}

/****************************************************************************
 * Name: hpm_i2c_rtxint
 *
 * Description:
 *   The I2C Interrupt enbale
 *
 ****************************************************************************/

static void hpm_i2c_rtxint(struct i2c_slave_s  *dev, bool enable)
{
  hpm_i2c_slave_t *priv = (hpm_i2c_slave_t *) dev;
  if (enable) 
    {
      i2c_enable_irq(priv->ptr, I2C_EVENT_ADDRESS_HIT | I2C_EVENT_TRANSACTION_COMPLETE); 
    }
  else
    {
      i2c_disable_irq(priv->ptr, I2C_EVENT_ADDRESS_HIT | I2C_EVENT_TRANSACTION_COMPLETE); 
    }
}

/****************************************************************************
 * Name: hpm_enable_i2c_slave
 *
 * Description:
 *   Enable the I2C device as a slave and start handing I2C interrupts.
 *
 ****************************************************************************/

static void hpm_enable_i2c_slave(struct i2c_slave_s *dev, bool enable)
{
  hpm_i2c_slave_t *priv = (hpm_i2c_slave_t *) dev;
  irqstate_t flags;

  flags = enter_critical_section();
  priv->ptr->SETUP |= I2C_SETUP_IICEN_SET(enable); 
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: hpm_set_own_address
 *
 * Description:
 *   Called to set the address listened to, and enable I2C as a slave device.
 *
 ****************************************************************************/

static int hpm_set_own_address(struct i2c_slave_s  *dev,
                              int                  address,
                              int                  nbits)
{
  hpm_i2c_slave_t *priv = (hpm_i2c_slave_t *) dev;

  irqstate_t flags;

  flags = enter_critical_section();

  hpm_enable_i2c_slave(dev, false);

  if (nbits == 10) 
    {
      priv->ptr->SETUP = I2C_SETUP_ADDRESSING_SET(true);
    }
  else
    {
      priv->ptr->SETUP = I2C_SETUP_ADDRESSING_SET(false);
    }
  
  priv->ptr->ADDR = I2C_ADDR_ADDR_SET(address);

  hpm_enable_i2c_slave(dev, true);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: hpm_write
 *
 * Description:
 *   Called to set the data to be read on the next I2C read transaction.
 *
 ****************************************************************************/

static int hpm_write(struct i2c_slave_s  *dev,
                    const uint8_t       *buffer,
                    int                  length)
{
  hpm_i2c_slave_t *priv = (hpm_i2c_slave_t *) dev;
  irqstate_t flags;

  flags = enter_critical_section();

  priv->tx_buffer  = buffer;
  priv->tx_buf_ptr = buffer;
  priv->tx_buf_end = priv->tx_buffer + length;

  hpm_i2c_rtxint(dev,true);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: hpm_read
 *
 * Description:
 *   Called to register a buffer to receive data from the next I2C write
 *   transaction.
 *
 ****************************************************************************/

static int hpm_read(struct i2c_slave_s  *dev,
                   uint8_t             *buffer,
                   int                  length)
{
  hpm_i2c_slave_t *priv = (hpm_i2c_slave_t *) dev;
  irqstate_t flags;

  flags = enter_critical_section();

  priv->rx_buffer  = buffer;
  priv->rx_buf_ptr = buffer;
  priv->rx_buf_end = priv->rx_buffer + length;

  hpm_i2c_rtxint(dev,true);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: hpm_register_callback
 *
 * Description:
 *   Called to register a callback function that will be called when
 *   data becomes available due to an I2C write transaction.
 *
 ****************************************************************************/

static int hpm_register_callback(struct i2c_slave_s   *dev,
                                i2c_slave_callback_t *callback,
                                void                 *arg)
{
  hpm_i2c_slave_t *priv = (hpm_i2c_slave_t *) dev;
  irqstate_t flags;

  flags = enter_critical_section();

  priv->callback     = callback;
  priv->callback_arg = arg;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
                            i2c_slave_callback_t *callback)
{
  hpm_i2c_slave_t *priv;
 
#ifdef CONFIG_HPM_I2C0_SLAVE
  if (port == 0)
    {
      priv          = &i2c0_slave_dev;
    }
  else
#endif
#ifdef CONFIG_HPM_I2C1_SLAVE
  if (port == 1)
    {
      priv          = &i2c1_slave_dev;
    }
  else
#endif
#ifdef CONFIG_HPM_I2C2_SLAVE
  if (port == 1)
    {
      priv          = &i2c2_slave_dev;
    }
  else
#endif
#ifdef CONFIG_HPM_I2C3_SLAVE
  if (port == 1)
    {
      priv          = &i2c3_slave_dev;
    }
  else
#endif
    {
      i2cerr("I2C Only support 0,1,2,3\n");
      return NULL;
    }
  
  if (hpm_i2cbus_pins_initialize(port) < 0)
    {
      return NULL;
    }
  
  if(hpm_i2c_slave_init(priv) < 0)
    return NULL;

  return &(priv->dev);
}

#endif

