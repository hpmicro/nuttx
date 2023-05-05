/****************************************************************************
 * arch/risc-v/src/hpmicro/common/hpm_i2c_master.c
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
#include <nuttx/mutex.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "riscv_internal.h"

#include "board.h"
#include "hpm_i2c.h"
#include "hpm_i2c_drv.h"
#include "hpm_i2c_regs.h"
#include "hpm_clock_drv.h"

#ifdef CONFIG_HPM_I2C_MASTER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HPM_I2C_DRV_RETRY_COUNT  (5000)
#define I2C_FIFO_MAX_SIZE     (4)

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct hpm_i2cdev_s
{
  struct i2c_master_s dev;        /* Generic I2C device */
  I2C_Type            *base;       /* Base address of registers */
  clock_name_t        i2c_clock;  /* i2c clock */
  i2c_config_t        i2c_config; /* i2c config */
  uint16_t            irqid;      /* IRQ for this device */
  int8_t              port;       /* Port number */
  uint32_t            base_freq;  /* branch frequency */

  mutex_t             lock;       /* Only one thread can access at a time */
  sem_t               wait;       /* Place to wait for transfer completion */
  uint32_t            frequency;  /* Current I2C frequency */

  struct i2c_msg_s    *msgs;

  int                 rx_data_count;
  int                 tx_data_count;
  int                 rw_size;                  

  int                 error;      /* Error status of each transfers */
  int                 msg_count;
  int                 tx_start_index;
  int                 tx_stop_index;
  int                 rx_index;
  bool                is_read;
};

#ifdef CONFIG_HPM_I2C0_MASTER
static struct hpm_i2cdev_s g_i2c0dev =
{
  .port                           = 0,
  .base                           = HPM_I2C0,
  .i2c_clock                      = clock_i2c0,
  .frequency                      = 100000,
  .i2c_config.i2c_mode            = CONFIG_HPM_I2C0_MASTER_MODE,
  .i2c_config.is_10bit_addressing = CONFIG_HPM_I2C0_MASTER_10BIT_ADDR,
  .irqid                          = HPM_IRQn_I2C0,
  .lock                           = NXMUTEX_INITIALIZER,
  .wait                           = SEM_INITIALIZER(0),
  .rx_data_count                  = 0,
  .tx_start_index                 = 0,
  .tx_stop_index                  = 0,
  .tx_data_count                  = 0,
  .rw_size                        = 0,
  .msg_count                      = 0,
  .rx_index                       = 0,
  .is_read                        = false,
};
#endif

#ifdef CONFIG_HPM_I2C1_MASTER
static struct hpm_i2cdev_s g_i2c1dev =
{
  .port                           = 1,
  .base                           = HPM_I2C1,
  .i2c_clock                      = clock_i2c1,
  .i2c_config.i2c_mode            = CONFIG_HPM_I2C1_MASTER_MODE,
  .i2c_config.is_10bit_addressing = CONFIG_HPM_I2C1_MASTER_10BIT_ADDR,
  .irqid                          = HPM_IRQn_I2C1,
  .lock                           = NXMUTEX_INITIALIZER,
  .wait                           = SEM_INITIALIZER(0),
  .rx_data_count                  = 0,
  .tx_start_index                 = 0,
  .tx_stop_index                  = 0,
  .tx_data_count                  = 0,
  .rw_size                        = 0,
  .msg_count                      = 0,
  .rx_index                       = 0,
  .is_read                        = false,
};
#endif

#ifdef CONFIG_HPM_I2C2_MASTER
static struct hpm_i2cdev_s g_i2c2dev =
{
  .port                           = 2,
  .base                           = HPM_I2C2,
  .i2c_clock                      = clock_i2c2,
  .i2c_config.i2c_mode            = CONFIG_HPM_I2C2_MASTER_MODE,
  .i2c_config.is_10bit_addressing = CONFIG_HPM_I2C2_MASTER_10BIT_ADDR,
  .irqid                          = HPM_IRQn_I2C2,
  .lock                           = NXMUTEX_INITIALIZER,
  .wait                           = SEM_INITIALIZER(0),
  .rx_data_count                  = 0,
  .tx_start_index                 = 0,
  .tx_stop_index                  = 0,
  .tx_data_count                  = 0,
  .rw_size                        = 0,
  .msg_count                      = 0,
  .rx_index                       = 0,
  .is_read                        = false,
};
#endif

#ifdef CONFIG_HPM_I2C3_MASTER
static struct hpm_i2cdev_s g_i2c3dev =
{
  .port                           = 3,
  .base                           = HPM_I2C3,
  .i2c_clock                      = clock_i2c3,
  .i2c_config.i2c_mode            = CONFIG_HPM_I2C3_MASTER_MODE,
  .i2c_config.is_10bit_addressing = CONFIG_HPM_I2C3_MASTER_10BIT_ADDR,
  .irqid                          = HPM_IRQn_I2C3,
  .lock                           = NXMUTEX_INITIALIZER,
  .wait                           = SEM_INITIALIZER(0),
  .rx_data_count                  = 0,
  .tx_start_index                 = 0,
  .tx_stop_index                  = 0,
  .tx_data_count                  = 0,
  .rw_size                        = 0,
  .msg_count                      = 0,
  .rx_index                       = 0,
  .is_read                        = false,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static int  hpm_i2c_init(struct hpm_i2cdev_s *priv, uint32_t i2c_freq, bool addr_mode);
static void hpm_i2c_txinit(struct hpm_i2cdev_s *priv, bool enable);
static void hpm_i2c_rxinit(struct hpm_i2cdev_s *priv, bool enable);
static int  hpm_i2c_interrupt(int irq, void *context, void *arg);
static int  hpm_i2c_transfer(struct i2c_master_s *dev,
                                struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int hpm_i2c_reset(struct i2c_master_s *dev);
#endif

/****************************************************************************
 * I2C device operations
 ****************************************************************************/

struct i2c_ops_s hpm_i2c_ops =
{
  .transfer = hpm_i2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset = hpm_i2c_reset,
#endif
};

/****************************************************************************
 * Name: hpm_i2c_txinit
 *
 * Description:
 *   i2c send in
 *
 ****************************************************************************/

static void hpm_i2c_txinit(struct hpm_i2cdev_s *priv, bool enable)
{
  if (enable == true)
    {
      i2c_enable_irq(priv->base, I2C_EVENT_TRANSACTION_COMPLETE | I2C_EVENT_FIFO_EMPTY);
    }
  else
    {
      (i2c_disable_irq(priv->base, I2C_EVENT_TRANSACTION_COMPLETE | I2C_EVENT_FIFO_EMPTY));
    }                      
}

/****************************************************************************
 * Name: hpm_i2c_disable
 *
 * Description:
 *   disable i2c by diable clock
 *
 ****************************************************************************/

static void hpm_i2c_rxinit(struct hpm_i2cdev_s *priv, bool enable)
{
  if (enable == true)
    {
      i2c_enable_irq(priv->base, I2C_EVENT_TRANSACTION_COMPLETE | I2C_EVENT_FIFO_FULL);
    }
  else
    {
      (i2c_disable_irq(priv->base, I2C_EVENT_TRANSACTION_COMPLETE | I2C_EVENT_FIFO_FULL));
    } 
}

/****************************************************************************
 * Name: hpm_i2c_interrupt
 *
 * Description:
 *   The I2C Interrupt Handler
 *
 ****************************************************************************/

static int hpm_i2c_interrupt(int irq, void *context, void *arg)
{
  struct hpm_i2cdev_s *priv = (struct hpm_i2cdev_s *)arg;
  volatile uint32_t status, _irq;
  status = i2c_get_status(priv->base);
  _irq = priv->base->INTEN;
  uint8_t dir = (uint8_t)I2C_CTRL_DIR_GET(priv->base->CTRL);
  /* transmit */

  if ((status & I2C_EVENT_FIFO_EMPTY) && (_irq & I2C_EVENT_FIFO_EMPTY))
    {
      i2c_clear_status(priv->base, I2C_EVENT_FIFO_EMPTY);
      status = i2c_get_status(priv->base);
      while (!i2c_fifo_is_full(priv->base)) 
      {
        priv->base->DATA = I2C_DATA_DATA_SET(priv->msgs->buffer[priv->rw_size++]);
      }
      i2c_clear_status(priv->base, I2C_EVENT_FIFO_FULL);
      if (priv->rw_size == priv->tx_data_count)
        {
          i2c_disable_irq(priv->base, I2C_EVENT_FIFO_EMPTY);
        }
    }
  

  if ((status & I2C_EVENT_FIFO_FULL) && (_irq & I2C_EVENT_FIFO_FULL)) 
    {
      while (!i2c_fifo_is_empty(priv->base)) 
        {
          priv->msgs->buffer[priv->rw_size++] = (uint8_t)I2C_DATA_DATA_GET(priv->base->DATA); 
        }
      i2c_clear_status(priv->base, I2C_EVENT_FIFO_FULL);

      if (priv->rw_size == priv->rx_data_count) 
        {
          i2c_disable_irq(priv->base, I2C_EVENT_FIFO_FULL);
        }
    }
  
   /* complete */

  if (status & I2C_EVENT_TRANSACTION_COMPLETE) 
    {
        if (I2C_DIR_MASTER_READ == dir) 
          {
            while (!i2c_fifo_is_empty(priv->base)) 
              {
                priv->msgs->buffer[priv->rw_size++] = (uint8_t)I2C_DATA_DATA_GET(priv->base->DATA);
              }
            hpm_i2c_rxinit(priv, false);
          } 
        else
          {
            hpm_i2c_txinit(priv, false);
          }
        i2c_disable_irq(priv->base, I2C_EVENT_TRANSACTION_COMPLETE);
        i2c_clear_status(priv->base, I2C_EVENT_TRANSACTION_COMPLETE);
        priv->msgs->length = priv->rw_size;
        nxsem_post(&priv->wait);
    }
    return 0;
}

/****************************************************************************
 * Name: hpm_i2c_init
 *
 * Description:
 *   Initialize I2C based on frequency
 *
 ****************************************************************************/

static int hpm_i2c_init(struct hpm_i2cdev_s *priv, uint32_t i2c_freq, bool addr_mode)
{
  DEBUGASSERT(priv != NULL);

  uint32_t tmp_freq = 0;
  hpm_stat_t stat;

  if (i2c_freq <= 100000) 
    {
      tmp_freq = 100000;
      priv->i2c_config.i2c_mode = i2c_mode_normal;
    }
  else if ((i2c_freq > 100000) && (i2c_freq <= 400000))
    {
      tmp_freq = 400000;
      priv->i2c_config.i2c_mode = i2c_mode_fast;
    }
  else 
    {
      tmp_freq = 1000000;
      priv->i2c_config.i2c_mode = i2c_mode_fast_plus;
    }
  
  if (priv->frequency != tmp_freq)
    {
      priv->frequency = tmp_freq;
      priv->base_freq = clock_get_frequency(priv->i2c_clock);
      priv->i2c_config.is_10bit_addressing = addr_mode;
      stat = i2c_init_master(priv->base, priv->base_freq, &priv->i2c_config);
      if (stat != status_success)
        {
          return -1;
        }
    }
  
  return OK; 
}

/****************************************************************************
 * Name: hpm_i2c_transfer
 *
 * Description:
 *   Perform a sequence of I2C transfers
 *
 ****************************************************************************/

static int hpm_i2c_transfer(struct i2c_master_s *dev,
                               struct i2c_msg_s *msgs, int count)
{
  struct hpm_i2cdev_s *priv = (struct hpm_i2cdev_s *)dev;
  int ret = 0;
  int semval = 0;
  hpm_stat_t sta;
  bool is_ten_addr = false;

  DEBUGASSERT(dev != NULL);

  /* Get exclusive access to the I2C bus */

  nxmutex_lock(&priv->lock);

    /* Check wait semaphore value. If the value is not 0, the transfer can not
   * be performed normally.
   */

  ret = nxsem_get_value(&priv->wait, &semval);
  DEBUGASSERT(ret == OK && semval == 0);

  if (msgs[0].flags & I2C_M_TEN)
    {
      is_ten_addr = true;
    }
  
  hpm_i2c_init(priv, msgs[0].frequency, is_ten_addr);

  if (count == 1)
    {
      if (msgs[0].flags & I2C_M_READ)
        {
          sta = i2c_master_read(priv->base, msgs[0].addr, msgs[0].buffer, msgs[0].length);
        }
      else
        {
          sta = i2c_master_write(priv->base, msgs[0].addr, msgs[0].buffer, msgs[0].length);
        }
    }
  else if(count == 2)
    {
      if (msgs[1].flags & I2C_M_READ)
        {
          if (msgs[0].length <= 2)
            {
              sta = i2c_master_address_read(priv->base, msgs[0].addr, msgs[0].buffer, msgs[0].length, msgs[1].buffer, msgs[1].length);
            }
          else
            {
              sta = i2c_master_write(priv->base, msgs[0].addr, msgs[0].buffer, msgs[0].length);
              sta = i2c_master_read(priv->base, msgs[1].addr, msgs[1].buffer, msgs[1].length);
            }
        }
      else
        {
          if (msgs[0].length <= 2)
            {
              sta = i2c_master_address_write(priv->base, msgs[0].addr, msgs[0].buffer, msgs[0].length, msgs[1].buffer, msgs[1].length);
            }
          else
            {
              sta = i2c_master_write(priv->base, msgs[0].addr, msgs[0].buffer, msgs[0].length);
              sta = i2c_master_write(priv->base, msgs[1].addr, msgs[1].buffer, msgs[1].length);
            }
        }
      
    }
  (sta == status_success) ? (ret = 0) : (ret = -1);
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: hpm_i2c_reset
 *
 * Description:
 *   Perform an I2C bus reset in an attempt to break loose stuck I2C devices.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_RESET
static int hpm_i2c_reset(struct hpm_i2cdev_s *dev)
{
  struct hpm_i2cdev_s *priv = (struct hpm_i2cdev_s *)dev;

  /* Lock out other clients */

  nxmutex_lock(&priv->lock);

  priv->frequency                      = 100000;
  priv->i2c_config.i2c_mode            = CONFIG_HPM_I2C0_MASTER_MODE,
  priv->i2c_config.is_10bit_addressing = CONFIG_HPM_I2C0_MASTER_10BIT_ADDR,
  priv->base_freq = clock_get_frequency(priv.i2c_clock);
  stat = i2c_init_master(priv->base, priv->base_freq, &priv->i2c_config);

  nxmutex_unlock(&priv->lock);
  return ret;
}
#endif /* CONFIG_I2C_RESET */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_i2cbus_initialize
 *
 * Description:
 *   Initialise an I2C device
 *
 ****************************************************************************/

struct i2c_master_s *hpm_i2cbus_initialize(int port)
{
  struct hpm_i2cdev_s *priv;

#ifdef CONFIG_HPM_I2C0_MASTER
  if (port == 0)
    {
      priv          = &g_i2c0dev;
      priv->dev.ops = &hpm_i2c_ops;
    }
  else
#endif
#ifdef CONFIG_HPM_I2C1_MASTER
  if (port == 1)
    {
      priv          = &g_i2c1dev;
      priv->dev.ops = &hpm_i2c_ops;
    }
  else
#endif
#ifdef CONFIG_HPM_I2C2_MASTER
  if (port == 1)
    {
      priv          = &g_i2c2dev;
      priv->dev.ops = &hpm_i2c_ops;
    }
  else
#endif
#ifdef CONFIG_HPM_I2C3_MASTER
  if (port == 1)
    {
      priv          = &g_i2c3dev;
      priv->dev.ops = &hpm_i2c_ops;
    }
  else
#endif
    {
      i2cerr("I2C Only support 0,1,2,3\n");
      return NULL;
    }
    
  if (hpm_i2cbus_pins_initialize(priv->port) < 0)
    {
      return NULL;
    }
  hpm_i2c_init(priv, priv->frequency, false);
  nxmutex_lock(&priv->lock);

  /* Attach Interrupt Handler */

  irq_attach(priv->irqid, hpm_i2c_interrupt, priv);

  /* Enable Interrupt Handler */

  up_enable_irq(priv->irqid);

  nxmutex_unlock(&priv->lock);
  return &priv->dev;
}

/****************************************************************************
 * Name: hpm_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialise an I2C device
 *
 ****************************************************************************/

int hpm_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct hpm_i2cdev_s *priv = (struct hpm_i2cdev_s *)dev;

  nxmutex_lock(&priv->lock);

  up_disable_irq(priv->irqid);
  irq_detach(priv->irqid);

  nxmutex_unlock(&priv->lock);

  return OK;
}

#endif

