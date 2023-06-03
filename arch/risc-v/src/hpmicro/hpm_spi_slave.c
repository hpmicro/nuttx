/****************************************************************************
 * arch/risc-v/src/hpmicro/hpm_spi_slave.c
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

#include <assert.h>
#include <debug.h>
#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/slave.h>

#include <arch/board/board.h>

#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_spi_drv.h"
#include "hpm_spi_regs.h"
#include "hpm_soc_feature.h"

#if defined(CONFIG_HPM_SPI_DRV) && defined(CONFIG_SPI_SLAVE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

#define  HPM_MAX_SPI_COUNT      4
#define WORDS2BYTES(_priv, _wn)   ((_wn) * ((_priv)->nbits / 8))
#define BYTES2WORDS(_priv, _bn)   ((_bn) / ((_priv)->nbits / 8))

struct spislave_priv_s
{
  /* Externally visible part of the SPI Slave controller interface */

  struct spi_slave_ctrlr_s ctrlr;

  /* Reference to SPI Slave device interface */

  struct spi_slave_dev_s *dev;

  /* Port configuration */

  SPI_Type*        spibase;      /* SPIn base address */
  int refs;                   /* Reference count */
  int irqid;                 /* SPI interrupt ID */
  int irqint;
  enum spi_slave_mode_e mode; /* Current SPI Slave hardware mode */
  uint8_t nbits;              /* Current configured bit width */
  uint32_t tx_length;         /* Location of next TX value */

  /* SPI Slave TX queue buffer */

  uint16_t tx_processing_lenth;
  uint8_t tx_buffer[SPI_SOC_TRANSFER_COUNT_MAX];
  uint32_t rx_length;         /* Location of next RX value */

  /* SPI Slave RX queue buffer */

  uint8_t rx_buffer[SPI_SOC_TRANSFER_COUNT_MAX];

  /* Flag that indicates whether SPI Slave is currently processing */

  bool is_processing;
  bool is_tx_enable;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static hpm_stat_t spislave_tranfer_config(struct spislave_priv_s *priv, uint16_t transfer_size);
static int hpm_spi_slave_interrupt(int irq, void *context, void *arg);

/* SPI Slave controller operations */

static void   spislave_bind(struct spi_slave_ctrlr_s *ctrlr,
                          struct spi_slave_dev_s *dev,
                          enum spi_slave_mode_e mode,
                          int nbits);
static void   spislave_unbind(struct spi_slave_ctrlr_s *ctrlr);
static int    spislave_enqueue(struct spi_slave_ctrlr_s *ctrlr,
                            const void *data,
                            size_t nwords);
static bool   spislave_qfull(struct spi_slave_ctrlr_s *ctrlr);
static void   spislave_qflush(struct spi_slave_ctrlr_s *ctrlr);
static size_t spislave_qpoll(struct spi_slave_ctrlr_s *ctrlr);
/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_slave_ctrlrops_s hpm_spislave_ops =
{
  .bind     = spislave_bind,
  .unbind   = spislave_unbind,
  .enqueue  = spislave_enqueue,
  .qfull    = spislave_qfull,
  .qflush   = spislave_qflush,
  .qpoll    = spislave_qpoll
};

#ifdef CONFIG_HPM_SPI0
static struct spislave_priv_s hpm_spi0slave_priv =
{
  .ctrlr         =
                  {
                    .ops = &hpm_spislave_ops
                  },
  .dev           = NULL,
  .refs          = 0,
  .spibase       = HPM_SPI0,
  .irqid         = HPM_IRQn_SPI0,
  .irqint        = spi_tx_fifo_threshold_int | spi_rx_fifo_threshold_int | spi_end_int,
  .is_processing = false,
  .is_tx_enable  = false,
  .mode          = SPISLAVE_MODE0,
  .nbits         = 0,
  .tx_length     = 0,
  .tx_buffer     =
                  {
                    0
                  },
  .rx_length     = 0,
  .rx_buffer     =
                  {
                    0
                  },
  .tx_processing_lenth = 0,
};
#endif /* CONFIG_HPM_SPI0 */

#ifdef CONFIG_HPM_SPI1
static struct spislave_priv_s hpm_spi1slave_priv =
{
  .ctrlr         =
                  {
                    .ops = &hpm_spislave_ops
                  },
  .dev           = NULL,
  .refs          = 0,
  .spibase       = HPM_SPI1,
  .irqid         = HPM_IRQn_SPI1,
  .irqint        = spi_tx_fifo_threshold_int | spi_rx_fifo_threshold_int | spi_end_int,
  .is_processing = false,
  .is_tx_enable  = false,
  .mode          = SPISLAVE_MODE0,
  .nbits         = 0,
  .tx_length     = 0,
  .tx_buffer     =
                  {
                    0
                  },
  .rx_length     = 0,
  .rx_buffer     =
                  {
                    0
                  },
  .tx_processing_lenth = 0,
};
#endif /* CONFIG_HPM_SPI1 */

#ifdef CONFIG_HPM_SPI2
static struct spislave_priv_s hpm_spi2slave_priv =
{
  .ctrlr         =
                  {
                    .ops = &hpm_spislave_ops
                  },
  .dev           = NULL,
  .refs          = 0,
  .spibase       = HPM_SPI2,
  .irqid         = HPM_IRQn_SPI2,
  .irqint        = spi_tx_fifo_threshold_int | spi_rx_fifo_threshold_int | spi_end_int,
  .is_processing = false,
  .is_tx_enable  = false,
  .mode          = SPISLAVE_MODE0,
  .nbits         = 0,
  .tx_length     = 0,
  .tx_buffer     =
                  {
                    0
                  },
  .rx_length     = 0,
  .rx_buffer     =
                  {
                    0
                  },
  .tx_processing_lenth = 0,
};
#endif /* CONFIG_HPM_SPI2 */

#ifdef CONFIG_HPM_SPI3
static struct spislave_priv_s hpm_spi3slave_priv =
{
  .ctrlr         =
                  {
                    .ops = &hpm_spislave_ops
                  },
  .dev           = NULL,
  .refs          = 0,
  .spibase       = HPM_SPI3,
  .irqid         = HPM_IRQn_SPI3,
  .irqint        = spi_tx_fifo_threshold_int | spi_rx_fifo_threshold_int | spi_end_int,
  .is_processing = false,
  .is_tx_enable  = false,
  .mode          = SPISLAVE_MODE0,
  .nbits         = 0,
  .tx_length     = 0,
  .tx_buffer     =
                  {
                    0
                  },
  .rx_length     = 0,
  .rx_buffer     =
                  {
                    0
                  },
  .tx_processing_lenth = 0,
};
#endif /* CONFIG_HPM_SPI3 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static hpm_stat_t spislave_tranfer_config(struct spislave_priv_s *priv, uint16_t transfer_size)
{
  hpm_stat_t stat;
  spi_control_config_t control_config = {0};
  spi_slave_get_default_control_config(&control_config);
  control_config.slave_config.slave_data_only = true;
  control_config.common_config.dummy_cnt = spi_dummy_count_1;
  control_config.common_config.trans_mode = spi_trans_write_read_together;
  stat = spi_control_init(priv->spibase, &control_config, transfer_size , transfer_size);
  return stat;
}


/****************************************************************************
 * Name: hpm_spi_slave_interrupt
 *
 * Description:
 *   The SPI slave Interrupt Handler
 *
 ****************************************************************************/

static int hpm_spi_slave_interrupt(int irq, void *context, void *arg)
{
  struct spislave_priv_s *priv = (struct spislave_priv_s *)arg;
  volatile uint32_t irq_status;
  uint8_t data_len_in_bytes;
  uint8_t ch;

  data_len_in_bytes = spi_get_data_length_in_bytes(priv->spibase);
  irq_status = spi_get_interrupt_status(priv->spibase);
  if (irq_status & spi_end_int)
    {
      priv->is_processing = false;
      spi_disable_interrupt(priv->spibase, spi_end_int);
      spi_clear_interrupt_status(priv->spibase, spi_end_int);
    }
  if (irq_status & spi_rx_fifo_threshold_int)
    {
      spi_read_data(priv->spibase, data_len_in_bytes, &ch, 1);
      if ((priv->is_tx_enable == false) && spi_is_active(priv->spibase) && priv->is_processing)
        {
          if (priv->rx_length < sizeof(priv->rx_buffer))
            {
              priv->rx_buffer[priv->rx_length++] = ch;
            }
        }
      spi_clear_interrupt_status(priv->spibase, spi_rx_fifo_threshold_int);
    }
  if (irq_status & spi_tx_fifo_threshold_int)
    {
      if(priv->is_processing && priv->is_tx_enable && priv->tx_length)
      {
        spi_write_data(priv->spibase, data_len_in_bytes, &priv->tx_buffer[priv->tx_processing_lenth++], 1);
        if (priv->tx_processing_lenth == priv->tx_length)
          {
            priv->is_processing = false;
            priv->is_tx_enable = false;
            priv->tx_processing_lenth = 0;
            priv->tx_length = 0;
            spi_disable_interrupt(priv->spibase, spi_tx_fifo_threshold_int);
          }
      }
      spi_clear_interrupt_status(priv->spibase, spi_tx_fifo_threshold_int);
    }
  return 0;
}
/****************************************************************************
 * Name: spislave_bind
 *
 * Description:
 *   Bind the SPI Slave device interface to the SPI Slave controller
 *   interface and configure the SPI interface. Upon return, the SPI
 *   slave controller driver is fully operational and ready to perform
 *   transfers.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *   dev   - SPI Slave device interface instance
 *   mode  - The SPI mode requested
 *   nbits - The number of bits requests.
 *            If value is greater than 0, then it implies MSB first
 *            If value is less than 0, then it implies LSB first with -nbits
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This implementation currently supports only positive "nbits" values,
 *   i.e., it always configures the SPI Slave controller driver as MSB first.
 *
 ****************************************************************************/

static void spislave_bind(struct spi_slave_ctrlr_s *ctrlr,
                          struct spi_slave_dev_s *dev,
                          enum spi_slave_mode_e mode,
                          int nbits)
{
  struct spislave_priv_s *priv = (struct spislave_priv_s *)ctrlr;
  spi_format_config_t format_config = {0};
  spi_sclk_idle_state_t cpol;
  spi_sclk_sampling_clk_edges_t cpha;
  const void *data = NULL;
  irqstate_t flags;
  size_t num_words;

  spiinfo("ctrlr=%p dev=%p mode=%d nbits=%d\n", ctrlr, dev, mode, nbits);

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->dev == NULL);
  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(nbits > 0);

  flags = enter_critical_section();

  priv->dev = dev;

  SPIS_DEV_SELECT(dev, false);

  SPIS_DEV_CMDDATA(dev, false);

  priv->rx_length = 0;
  priv->tx_length = 0;

  /* set SPI format config for slave */
  
  spi_slave_get_default_format_config(&format_config);
  priv->nbits = nbits;
  priv->mode  = mode;
  priv->is_processing = false;
  priv->is_tx_enable  = false;
  priv->tx_processing_lenth = 0;
  priv->irqint = spi_tx_fifo_threshold_int | spi_rx_fifo_threshold_int | spi_end_int;
  format_config.common_config.data_len_in_bits = nbits;
  format_config.common_config.mode = spi_slave_mode;
  switch (mode)
  {
  case SPISLAVE_MODE0:
    cpol = spi_sclk_low_idle;
    cpha = spi_sclk_sampling_odd_clk_edges;
    break;
  case SPISLAVE_MODE1:
    cpol = spi_sclk_low_idle;
    cpha = spi_sclk_sampling_even_clk_edges;
    break;
  case SPISLAVE_MODE2:
    cpol = spi_sclk_high_idle;
    cpha = spi_sclk_sampling_odd_clk_edges;
    break;  
  case SPISLAVE_MODE3:
    cpol = spi_sclk_high_idle;
    cpha = spi_sclk_sampling_even_clk_edges;
    break;
  default:
    return;
    break;
  }
  format_config.common_config.cpol = cpol;
  format_config.common_config.cpha = cpha;
  spi_format_init(priv->spibase, &format_config);

  spi_set_tx_fifo_threshold(priv->spibase, SPI_SOC_FIFO_DEPTH - 1U);
  spi_set_rx_fifo_threshold(priv->spibase, 1U);

  num_words = SPIS_DEV_GETDATA(dev, &data);

  if (data != NULL && num_words > 0)
    {
      size_t num_bytes = WORDS2BYTES(priv, num_words);
      memcpy(priv->tx_buffer, data, num_bytes);
      priv->tx_length += num_bytes;
    }

  /* Attach Interrupt Handler */

  irq_attach(priv->irqid, hpm_spi_slave_interrupt, priv);

  /* Enable the CPU interrupt that is linked to the SPI Slave controller */

  up_enable_irq(priv->irqid);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: spislave_unbind
 *
 * Description:
 *   Un-bind the SPI Slave device interface from the SPI Slave controller
 *   interface. Reset the SPI interface and restore the SPI Slave
 *   controller driver to its initial state.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void spislave_unbind(struct spi_slave_ctrlr_s *ctrlr)
{
  struct spislave_priv_s *priv = (struct spislave_priv_s *)ctrlr;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->dev != NULL);

  spiinfo("Unbinding %p\n", priv->dev);

  flags = enter_critical_section();

  spi_disable_interrupt(priv->spibase, priv->irqint);
  up_disable_irq(priv->irqid);

  priv->dev = NULL;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: spislave_enqueue
 *
 * Description:
 *   Enqueue the next value to be shifted out from the interface. This adds
 *   the word to the controller driver for a subsequent transfer but has no
 *   effect on any in-process or currently "committed" transfers.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *   data  - Pointer to the command/data mode data to be shifted out.
 *           The data width must be aligned to the nbits parameter which was
 *           previously provided to the bind() method.
 *   len   - Number of units of "nbits" wide to enqueue,
 *           "nbits" being the data width previously provided to the bind()
 *           method.
 *
 * Returned Value:
 *   Number of data items successfully queued, or a negated errno:
 *         - "len" if all the data was successfully queued
 *         - "0..len-1" if queue is full
 *         - "-errno" in any other error
 *
 ****************************************************************************/

static int spislave_enqueue(struct spi_slave_ctrlr_s *ctrlr,
                            const void *data,
                            size_t len)
{
  struct spislave_priv_s *priv = (struct spislave_priv_s *)ctrlr;
  size_t num_bytes = WORDS2BYTES(priv, len);
  int bufsize;
  irqstate_t flags;
  int enqueued_words;

  spiinfo("ctrlr=%p, data=%p, num_bytes=%zu\n", ctrlr, data, num_bytes);

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->dev != NULL);

  flags = enter_critical_section();

  bufsize = SPI_SOC_TRANSFER_COUNT_MAX - priv->tx_length;

  num_bytes = MIN(num_bytes, bufsize);
  if (num_bytes <= 0)
    {
      return 0;
      leave_critical_section(flags);
    }
  memcpy(priv->tx_buffer + priv->tx_length, data, num_bytes);
  priv->tx_length += num_bytes;
  enqueued_words = BYTES2WORDS(priv, num_bytes);

  if (!priv->is_processing)
    {

      /* set SPI control config for slave */

      priv->is_processing = true;
      priv->is_tx_enable  = true;
      if (status_success != spislave_tranfer_config(priv, SPI_SOC_TRANSFER_COUNT_MAX))
        {
          return -1;
          priv->is_processing = false;
        }
      priv->irqint = spi_tx_fifo_threshold_int | spi_rx_fifo_threshold_int | spi_end_int;
      spi_disable_interrupt(priv->spibase, priv->irqint);
      priv->irqint = spi_tx_fifo_threshold_int | spi_end_int;
      spi_enable_interrupt(priv->spibase, priv->irqint);
    }

  if ((bufsize <= 0) && (priv->is_processing))
    {
      leave_critical_section(flags);
      return -ENOSPC;
    }

  leave_critical_section(flags);

  return enqueued_words;
}

/****************************************************************************
 * Name: spislave_qfull
 *
 * Description:
 *   Return true if the queue is full or false if there is space to add an
 *   additional word to the queue.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   true if the output queue is full, false otherwise.
 *
 ****************************************************************************/

static bool spislave_qfull(struct spi_slave_ctrlr_s *ctrlr)
{
  struct spislave_priv_s *priv = (struct spislave_priv_s *)ctrlr;
  irqstate_t flags;
  bool is_full = false;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->dev != NULL);

  spiinfo("ctrlr=%p\n", ctrlr);

  flags = enter_critical_section();
  is_full = (priv->tx_length == SPI_SOC_TRANSFER_COUNT_MAX);
  leave_critical_section(flags);

  return is_full;
}

/****************************************************************************
 * Name: spislave_qflush
 *
 * Description:
 *   Discard all saved values in the output queue. On return from this
 *   function the output queue will be empty. Any in-progress or otherwise
 *   "committed" output values may not be flushed.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void spislave_qflush(struct spi_slave_ctrlr_s *ctrlr)
{
  struct spislave_priv_s *priv = (struct spislave_priv_s *)ctrlr;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->dev != NULL);

  spiinfo("ctrlr=%p\n", ctrlr);

  flags = enter_critical_section();
  priv->tx_length = 0;
  priv->tx_processing_lenth = 0;
  if (priv->is_tx_enable)
    {
      priv->is_processing = false;
      priv->is_tx_enable = false;
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: spislave_qpoll
 *
 * Description:
 *   Tell the controller to output all the receive queue data.
 *
 * Input Parameters:
 *   ctrlr - SPI Slave controller interface instance
 *
 * Returned Value:
 *   Number of units of width "nbits" left in the RX queue. If the device
 *   accepted all the data, the return value will be 0.
 *
 ****************************************************************************/

static size_t spislave_qpoll(struct spi_slave_ctrlr_s *ctrlr)
{
  struct spislave_priv_s *priv = (struct spislave_priv_s *)ctrlr;
  irqstate_t flags;
  uint32_t tmp;
  uint32_t recv_n;
  size_t remaining_words;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->dev != NULL);

  spiinfo("ctrlr=%p\n", ctrlr);

  flags = enter_critical_section();
  tmp = SPIS_DEV_RECEIVE(priv->dev, priv->rx_buffer,
                         BYTES2WORDS(priv, priv->rx_length));
  if (priv->rx_length == 0)
    {
      if ((!priv->is_processing) && (!priv->is_tx_enable))
        {
          spislave_tranfer_config(priv, SPI_SOC_TRANSFER_COUNT_MAX);
          priv->irqint = spi_tx_fifo_threshold_int | spi_rx_fifo_threshold_int | spi_end_int;
          spi_disable_interrupt(priv->spibase, priv->irqint);
          priv->irqint = spi_rx_fifo_threshold_int | spi_end_int;
          spi_enable_interrupt(priv->spibase, priv->irqint); 
          priv->is_processing = true;
        }
      return 0;
    }

  recv_n = WORDS2BYTES(priv, tmp);
  if (recv_n == 0)
    {

    }
  if (recv_n < priv->rx_length)
    {
      /* If the upper layer does not receive all of the data from the receive
       * buffer, move the remaining data to the head of the buffer.
       */

      priv->rx_length -= recv_n;
      memmove(priv->rx_buffer, priv->rx_buffer + recv_n, priv->rx_length);
    }
  else
    {
      priv->rx_length = 0;
    }

  remaining_words = BYTES2WORDS(priv, priv->rx_length);

  leave_critical_section(flags);

  return remaining_words;
}

/****************************************************************************
 * Name: hpm_spislave_ctrlr_initialize
 *
 * Description:
 *   Initialize the selected SPI Slave bus.
 *
 * Input Parameters:
 *   port - Port number (for hardware that has multiple SPI Slave interfaces)
 *
 * Returned Value:
 *   Valid SPI Slave controller structure reference on success;
 *   NULL on failure.
 *
 ****************************************************************************/

struct spi_slave_ctrlr_s *hpm_spislave_ctrlr_initialize(int port)
{
  struct spi_slave_ctrlr_s *spislave_dev;
  struct spislave_priv_s *priv;

  switch (port)
    {
#ifdef CONFIG_HPM_SPI0
      case 0:
        priv = &hpm_spi0slave_priv;
        break;
#endif
#ifdef CONFIG_HPM_SPI1
      case 1:
        priv = &hpm_spi1slave_priv;
        break;
#endif
#ifdef CONFIG_HPM_SPI2
      case 2:
        priv = &hpm_spi2slave_priv;
        break;
#endif
#ifdef CONFIG_HPM_SPI3
      case 3:
        priv = &hpm_spi3slave_priv;
        break;
#endif
      default:
        return NULL;
    }

  spislave_dev = (struct spi_slave_ctrlr_s *)priv;
  return spislave_dev;
}

#endif

