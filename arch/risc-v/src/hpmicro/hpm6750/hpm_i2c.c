/****************************************************************************
 * arch/risc-v/src/hpmicro/hpm6750/hpm_i2c.c
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

#if defined(CONFIG_HPM6750_I2C0_MASTER) || defined(CONFIG_HPM6750_I2C1_MASTER) || \
    defined(CONFIG_HPM6750_I2C2_MASTER) || defined(CONFIG_HPM6750_I2C3_MASTER)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HPM_I2C_DRV_RETRY_COUNT  (5000)
#define I2C_FIFO_MAX_SIZE     (4)

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct hpm6750_i2cdev_s
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

#ifdef CONFIG_HPM6750_I2C0_MASTER
static struct hpm6750_i2cdev_s g_i2c0dev =
{
  .port                           = 0,
  .base                           = HPM_I2C0,
  .i2c_clock                      = clock_i2c0,
  .frequency                      = 100000,
  .i2c_config.i2c_mode            = CONFIG_HPM6750_I2C0_MASTER_MODE,
  .i2c_config.is_10bit_addressing = CONFIG_HPM6750_I2C0_MASTER_10BIT_ADDR,
  .irqid                          = IRQn_I2C0 + HPM_IRQ_PERI_START,
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

#ifdef CONFIG_HPM6750_I2C1_MASTER
static struct hpm6750_i2cdev_s g_i2c1dev =
{
  .port                           = 1,
  .base                           = HPM_I2C1,
  .i2c_clock                      = clock_i2c1,
  .i2c_config.i2c_mode            = CONFIG_HPM6750_I2C1_MASTER_MODE,
  .i2c_config.is_10bit_addressing = CONFIG_HPM6750_I2C1_MASTER_10BIT_ADDR,
  .irqid                          = IRQn_I2C1 + HPM_IRQ_PERI_START,
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

#ifdef CONFIG_HPM6750_I2C2_MASTER
static struct hpm6750_i2cdev_s g_i2c2dev =
{
  .port                           = 2,
  .base                           = HPM_I2C2,
  .i2c_clock                      = clock_i2c2,
  .i2c_config.i2c_mode            = CONFIG_HPM6750_I2C2_MASTER_MODE,
  .i2c_config.is_10bit_addressing = CONFIG_HPM6750_I2C2_MASTER_10BIT_ADDR,
  .irqid                          = IRQn_I2C2 + HPM_IRQ_PERI_START,
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

#ifdef CONFIG_HPM6750_I2C3_MASTER
static struct hpm6750_i2cdev_s g_i2c3dev =
{
  .port                           = 3,
  .base                           = HPM_I2C3,
  .i2c_clock                      = clock_i2c3,
  .i2c_config.i2c_mode            = CONFIG_HPM6750_I2C3_MASTER_MODE,
  .i2c_config.is_10bit_addressing = CONFIG_HPM6750_I2C3_MASTER_10BIT_ADDR,
  .irqid                          = IRQn_I2C3 + HPM_IRQ_PERI_START,
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
static int  hpm6750_i2c_disable(struct hpm6750_i2cdev_s *priv);
static int  hpm6750_i2c_init(struct hpm6750_i2cdev_s *priv, uint32_t i2c_freq, bool addr_mode);
static int  hpm6750_i2c_enable(struct hpm6750_i2cdev_s *priv);
static void hpm6750_i2c_txinit(struct hpm6750_i2cdev_s *priv, bool enable);
static void hpm6750_i2c_rxinit(struct hpm6750_i2cdev_s *priv, bool enable);
static void hpm6750_i2c_master_configure_transfer(I2C_Type *i2c_ptr, const uint16_t device_address,\
                                                 uint32_t size, bool read,bool transfer);
static int  hpm6750_i2c_small_than_fifo_read(struct hpm6750_i2cdev_s *priv);
static int  hpm6750_i2c_small_than_fifo_write(struct hpm6750_i2cdev_s *priv);
static int  hpm6750_i2c_interrupt(int irq, void *context, void *arg);
static int  hpm6750_i2c_transfer(struct i2c_master_s *dev,
                                struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int hpm6750_i2c_reset(struct i2c_master_s *dev);
#endif

/****************************************************************************
 * I2C device operations
 ****************************************************************************/

struct i2c_ops_s hpm6750_i2c_ops =
{
  .transfer = hpm6750_i2c_transfer,
#ifdef CONFIG_I2C_RESET
  .reset = hpm6750_i2c_reset,
#endif
};

/****************************************************************************
 * Name: hpm6750_i2c_disable
 *
 * Description:
 *   disable i2c by diable clock
 *
 ****************************************************************************/

static int  hpm6750_i2c_disable(struct hpm6750_i2cdev_s *priv)
{
  clock_remove_from_group(priv->i2c_clock, BOARD_RUNNING_CORE);
  clock_disable(priv->i2c_clock);
  return OK;
}

/****************************************************************************
 * Name: hpm6750_i2c_disable
 *
 * Description:
 *   disable i2c by diable clock
 *
 ****************************************************************************/

static int  hpm6750_i2c_enable(struct hpm6750_i2cdev_s *priv)
{
  clock_enable(priv->i2c_clock);
  clock_add_to_group(priv->i2c_clock, BOARD_RUNNING_CORE);
  return OK;
}

/****************************************************************************
 * Name: hpm6750_i2c_txinit
 *
 * Description:
 *   i2c send in
 *
 ****************************************************************************/

static void hpm6750_i2c_txinit(struct hpm6750_i2cdev_s *priv, bool enable)
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
 * Name: hpm6750_i2c_disable
 *
 * Description:
 *   disable i2c by diable clock
 *
 ****************************************************************************/

static void hpm6750_i2c_rxinit(struct hpm6750_i2cdev_s *priv, bool enable)
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
 * Name: hpm6750_i2c_master_configure_transfer
 *
 * Description:
 *   configure i2c transmission paramenters
 *
 ****************************************************************************/

static void hpm6750_i2c_master_configure_transfer(I2C_Type *i2c_ptr, const uint16_t device_address,\
                                                 uint32_t size, bool read, bool transfer)
{
    i2c_ptr->ADDR = I2C_ADDR_ADDR_SET(device_address);            
    i2c_ptr->CMD  = I2C_CMD_CLEAR_FIFO;      
    i2c_ptr->CTRL = I2C_CTRL_PHASE_START_MASK
                | I2C_CTRL_PHASE_STOP_MASK
                | I2C_CTRL_PHASE_ADDR_MASK
                | I2C_CTRL_PHASE_DATA_MASK
                | I2C_CTRL_DIR_SET(read)
                | I2C_CTRL_DATACNT_SET(I2C_DATACNT_MAP(size));
    if (transfer)
      {
        i2c_ptr->CMD = I2C_CMD_ISSUE_DATA_TRANSMISSION; 
      }   
}

/****************************************************************************
 * Name: hpm6750_i2c_small_than_fifo_read
 *
 * Description:
 *   i2c read small than the i2c fifo
 *
 ****************************************************************************/

static int hpm6750_i2c_small_than_fifo_read(struct hpm6750_i2cdev_s *priv)
{
    uint32_t retry = 0;
    while (priv->rx_data_count) 
      {
          if (!(priv->base->STATUS & I2C_STATUS_FIFOEMPTY_MASK)) 
            {
                *(priv->msgs->buffer++) = priv->base->DATA;
                priv->rx_data_count--;
                retry = 0;             
            } 
          else 
            {
                if (retry > HPM_I2C_DRV_RETRY_COUNT) {
                    break;
                }
                retry++;
            }
      }
    if (retry > HPM_I2C_DRV_RETRY_COUNT) 
      {
          goto error;
      }

    retry = 0;

    while (!(priv->base->STATUS & I2C_STATUS_CMPL_MASK)) 
      {
          if (retry > HPM_I2C_DRV_RETRY_COUNT) 
            {
                break;
            }
          retry++;
      }

    if (retry > HPM_I2C_DRV_RETRY_COUNT) 
      {
          goto error;
      }

    if (!(priv->base->STATUS & I2C_STATUS_ADDRHIT_MASK)) 
      {

          /* I2C slave did not receive this transaction correctly. */

          goto error;
      }

    priv->base->STATUS |= I2C_STATUS_CMPL_MASK | I2C_STATUS_ADDRHIT_MASK;

    priv->base->INTEN = 0;
    if (i2c_get_data_count(priv->base)) 
      {
          goto error;
      }
    nxsem_post(&priv->wait);
    return 0;
error:nxsem_post(&priv->wait); return -1;
}

/****************************************************************************
 * Name: hpm6750_i2c_small_than_fifo_write
 *
 * Description:
 *   i2c write small than the i2c fifo
 *
 ****************************************************************************/

static int hpm6750_i2c_small_than_fifo_write(struct hpm6750_i2cdev_s *priv)
{
    uint32_t retry;
    retry = 0;
    uint32_t count = priv->tx_data_count;
    uint32_t tmp_count = 0;
    while (priv->tx_data_count) 
      {      
        for (size_t i = 0; i < priv->msg_count; i++)
          {
            tmp_count = 0;
            if (!(priv->base->STATUS & I2C_STATUS_FIFOFULL_MASK)) 
              {         
                  priv->base->DATA = *(priv->msgs[i].buffer++);  
                  tmp_count ++;   
                  retry = 0;
                  priv->tx_data_count--;
                  if (tmp_count == priv->msgs[i].length)
                  {
                    break;
                  }              
              } 
            else 
              {
                  if (retry > HPM_I2C_DRV_RETRY_COUNT) {
                      break;
                  }
                  retry++;
              }
          }
      }
    if (retry > HPM_I2C_DRV_RETRY_COUNT) 
      {
          goto error;
      }

    priv->base->CMD = I2C_CMD_ISSUE_DATA_TRANSMISSION;  

    retry = 0;
    while (!(priv->base->STATUS & I2C_STATUS_CMPL_MASK)) 
      {
        if (retry > HPM_I2C_DRV_RETRY_COUNT) 
          {
            goto error;
          }
        retry++;
      }
    if (retry > HPM_I2C_DRV_RETRY_COUNT) 
      {
        goto error;
      }
    priv->base->STATUS |= I2C_STATUS_CMPL_MASK;

    priv->base->INTEN = 0;
    if (i2c_get_data_count(priv->base)) 
    {
      goto error;
    }

    priv->base->CMD = I2C_CMD_RESET;
    nxsem_post(&priv->wait);
    return 0;
error:nxsem_post(&priv->wait); return -1;
}

/****************************************************************************
 * Name: hpm6750_i2c_interrupt
 *
 * Description:
 *   The I2C Interrupt Handler
 *
 ****************************************************************************/

static int hpm6750_i2c_interrupt(int irq, void *context, void *arg)
{
  struct hpm6750_i2cdev_s *priv = (struct hpm6750_i2cdev_s *)arg;
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
            hpm6750_i2c_rxinit(priv, false);
          } 
        else
          {
            hpm6750_i2c_txinit(priv, false);
          }
        i2c_disable_irq(priv->base, I2C_EVENT_TRANSACTION_COMPLETE);
        i2c_clear_status(priv->base, I2C_EVENT_TRANSACTION_COMPLETE);
        priv->msgs->length = priv->rw_size;
        nxsem_post(&priv->wait);
    }
}

/****************************************************************************
 * Name: hpm6750_i2c_init
 *
 * Description:
 *   Initialize I2C based on frequency
 *
 ****************************************************************************/

static int hpm6750_i2c_init(struct hpm6750_i2cdev_s *priv, uint32_t i2c_freq, bool addr_mode)
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
 * Name: hpm6750_i2c_transfer
 *
 * Description:
 *   Perform a sequence of I2C transfers
 *
 ****************************************************************************/

static int hpm6750_i2c_transfer(struct i2c_master_s *dev,
                               struct i2c_msg_s *msgs, int count)
{
  struct hpm6750_i2cdev_s *priv = (struct hpm6750_i2cdev_s *)dev;
  int  i;
  bool stop  = false;
  bool start = false;
  int ret = 0;
  int semval = 0;
  uint32_t start_index  = 0;
  uint32_t stop_index   = 0;
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
  
  hpm6750_i2c_init(priv, msgs[0].frequency, is_ten_addr);

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
 * Name: hpm6750_i2c_reset
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
static int hpm6750_i2c_reset(struct hpm6750_i2cdev_s *dev)
{
  struct hpm6750_i2cdev_s *priv = (struct hpm6750_i2cdev_s *)dev;

  /* Lock out other clients */

  nxmutex_lock(&priv->lock);

  priv->frequency                      = 100000;
  priv->i2c_config.i2c_mode            = CONFIG_HPM6750_I2C0_MASTER_MODE,
  priv->i2c_config.is_10bit_addressing = CONFIG_HPM6750_I2C0_MASTER_10BIT_ADDR,
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
 * Name: hpm6750_i2cbus_initialize
 *
 * Description:
 *   Initialise an I2C device
 *
 ****************************************************************************/

struct i2c_master_s *hpm6750_i2cbus_initialize(int port)
{
  struct hpm6750_i2cdev_s *priv;

#ifdef CONFIG_HPM6750_I2C0_MASTER
  if (port == 0)
    {
      priv          = &g_i2c0dev;
      priv->dev.ops = &hpm6750_i2c_ops;
    }
  else
#endif
#ifdef CONFIG_HPM6750_I2C1_MASTER
  if (port == 1)
    {
      priv          = &g_i2c1dev;
      priv->dev.ops = &hpm6750_i2c_ops;
    }
  else
#endif
#ifdef CONFIG_HPM6750_I2C2_MASTER
  if (port == 1)
    {
      priv          = &g_i2c2dev;
      priv->dev.ops = &hpm6750_i2c_ops;
    }
  else
#endif
#ifdef CONFIG_HPM6750_I2C3_MASTER
  if (port == 1)
    {
      priv          = &g_i2c3dev;
      priv->dev.ops = &hpm6750_i2c_ops;
    }
  else
#endif
    {
      i2cerr("I2C Only support 0,1,2,3\n");
      return NULL;
    }
    
  if (hpm6750_i2cbus_pins_initialize(priv->port) < 0)
    {
      return NULL;
    }
  hpm6750_i2c_init(priv, priv->frequency, false);
  nxmutex_lock(&priv->lock);

  /* Attach Interrupt Handler */

  irq_attach(priv->irqid, hpm6750_i2c_interrupt, priv);

  /* Enable Interrupt Handler */

  up_enable_irq(priv->irqid);

  nxmutex_unlock(&priv->lock);
  return &priv->dev;
}

/****************************************************************************
 * Name: hpm6750_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialise an I2C device
 *
 ****************************************************************************/

int hpm6750_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct hpm6750_i2cdev_s *priv = (struct hpm6750_i2cdev_s *)dev;

  nxmutex_lock(&priv->lock);

  up_disable_irq(priv->irqid);
  irq_detach(priv->irqid);

  nxmutex_unlock(&priv->lock);

  return OK;
}

#endif

