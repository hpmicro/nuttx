/****************************************************************************
 * arch/risc-v/src/hpmicro/hpm_spi.c
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
 * The external functions, hpm_spi0/1/2/3select and
 * hpm_spi0/1/2/3status must be provided by board-specific logic.  They
 * are implementations of the select and status methods of the SPI interface
 * defined by struct spi_ops_s (see include/nuttx/spi/spi.h).  All other
 * methods (including hpm_spibus_initialize()) are provided by common HPMicro
 * logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in hpm_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide hpm_spi0/1/2/3select() and hpm_spi0/1/2/3status()
 *      functions in your board-specific logic.  These functions will perform
 *      chip selection and status operations using GPIOs in the way your
 *      board is configured.
 *   3. Add a calls to hpm_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by hpm_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/compiler.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>
#include <nuttx/power/pm.h>

#include <arch/board/board.h>

#include "board.h"
#include "spi/hpm_spi.h"
#include "hpm_spi_master.h"
#include "hpm_dma.h"
#include "hpm_clock_drv.h"
#include "hpm_spi_drv.h"
#include "hpm_spi_regs.h"
#include "hpm_soc_feature.h"
#include "hpm_sysctl_drv.h"
#include "hpm_pllctl_drv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define  HPM_MAX_SPI_COUNT      4
#define  HPM_MAX_SPI_DMA_COUNT  129   /* <= 4K for descriptor*/

#if defined (CONFIG_HPM_SPI_DRV) && defined (CONFIG_SPI_DRIVER)

/* Configuration ************************************************************/

/* SPI interrupts */

#ifdef CONFIG_HPM_SPI_INTERRUPTS
#  error "Interrupt driven SPI not yet supported"
#endif

/* Can't have both interrupt driven SPI and SPI DMA */

#if defined(CONFIG_HPM_SPI_INTERRUPTS) && defined(CONFIG_HPM_SPI_DMA)
#  error "Cannot enable both interrupt mode and DMA mode for SPI"
#endif


/****************************************************************************
 * Private Types
 ****************************************************************************/

enum spi_config_e
{
  FULL_DUPLEX = 0,
  SIMPLEX_TX,
  SIMPLEX_RX,
  HALF_DUPLEX
};

struct hpm_spidev_s
{
  struct spi_dev_s spidev;       /* Externally visible part of the SPI interface */
  SPI_Type*        spibase;      /* SPIn base address */
  uint32_t         spiclock;     /* Clocking for the SPI module */
  uint8_t          spiirq;       /* SPI IRQ number */
  sem_t            spiirqsem;    /* SPI IRQ SEM*/
#ifdef CONFIG_HPM_SPI_DMA
#ifdef CONFIG_SPI_TRIGGER
  bool             defertrig;    /* Flag indicating that trigger should be deferred */
  bool             trigarmed;    /* Flag indicating that the trigger is armed */
#endif
  dma_resource_t   dma_txresource; /* DMA channel RX resource */
  dma_resource_t   dma_rxresource; /* DMA channel TX resource */
  sem_t            rxsem;        /* Wait for RX DMA to complete */
  sem_t            txsem;        /* Wait for TX DMA to complete */
#endif
  bool             initialized;  /* Has SPI interface been initialized */
  sem_t            exclsem;      /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;    /* Requested clock frequency */
  uint32_t         actual;       /* Actual clock frequency */
  int8_t           nbits;        /* Width of word in bits */
  uint8_t          mode;         /* Mode 0,1,2,3 */
#ifdef CONFIG_PM
  struct pm_callback_s pm_cb;    /* PM callbacks */
#endif
  enum spi_config_e config;      /* full/half duplex, simplex transmit/read only */
#ifdef CONFIG_HPM_SPI_DMA
  spi_context_t    *spi_context; /* spi dma context*/
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

#ifdef CONFIG_DEBUG_SPI_INFO
static inline void spi_dumpregs(struct hpm_spidev_s *priv);
#endif

/* DMA support */

#ifdef CONFIG_HPM_SPI_DMA
static int         spi_dmarxwait(struct hpm_spidev_s *priv);
static int         spi_dmatxwait(struct hpm_spidev_s *priv);
static void        spi_rx_dma_channel_tc_callback(DMA_Type *ptr,
                                              uint32_t channel,
                                              void *user_data);
static void        spi_tx_dma_channel_tc_callback(DMA_Type *ptr,
                                              uint32_t channel,
                                              void *user_data);
#endif

/* SPI methods */

static int         spi_lock(struct spi_dev_s *dev, bool lock);
static uint32_t    spi_setfrequency(struct spi_dev_s *dev,
                                    uint32_t frequency);
#ifdef CONFIG_SPI_DELAY_CONTROL
static int         spi_setdelay(struct spi_dev_s *dev, uint32_t startdelay,
                                uint32_t stopdelay, uint32_t csdelay,
                                uint32_t ifdelay);
#endif
static void        spi_setmode(struct spi_dev_s *dev,
                               enum spi_mode_e mode);
static void        spi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int         spi_hwfeatures(struct spi_dev_s *dev,
                                spi_hwfeatures_t features);
#endif
static uint32_t    spi_send(struct spi_dev_s *dev, uint32_t wd);
static void        spi_exchange(struct spi_dev_s *dev,
                                const void *txbuffer, void *rxbuffer,
                                size_t nwords);
#ifdef CONFIG_SPI_TRIGGER
static int         spi_trigger(struct spi_dev_s *dev);
#endif
#ifndef CONFIG_SPI_EXCHANGE
static void        spi_sndblock(struct spi_dev_s *dev,
                                const void *txbuffer, size_t nwords);
static void        spi_recvblock(struct spi_dev_s *dev,
                                 void *rxbuffer, size_t nwords);
#endif

#ifdef CONFIG_HPM_SPI_INTERRUPTS
static int spi_interrupt(int irq, void *context, void *arg)
#endif

/* Initialization */

static void        spi_bus_initialize(struct hpm_spidev_s *priv);

/* PM interface */

#ifdef CONFIG_PM
static int         spi_pm_prepare(struct pm_callback_s *cb, int domain,
                                  enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_HPM_SPI0
static const struct spi_ops_s g_spi0ops =
{
  .lock                    = spi_lock,
  .select                  = hpm_spi0select,
  .setfrequency            = spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay                = spi_setdelay,
#endif
  .setmode                 = spi_setmode,
  .setbits                 = spi_setbits,
  .status                  = hpm_spi0status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata                 = hpm_spi0cmddata,
#endif
  .send                    = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange                = spi_exchange,
#else
  .sndblock                = spi_sndblock,
  .recvblock               = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger                 = spi_trigger,
#endif
  .registercallback        = 0,                   /* not implemented */
};

#ifdef CONFIG_HPM_SPI0_DMA
/* dma descriptors need align 8 bytes */
ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(8) dma_linked_descriptor_t spi0_dma_linked_descriptor[HPM_MAX_SPI_DMA_COUNT * SPI_DMA_DESC_COUNT_PER_TRANS];
ATTR_PLACE_AT_NONCACHEABLE uint32_t spi0_transctrl[HPM_MAX_SPI_DMA_COUNT];
spi_context_t spi0_context = {
    .ptr                   = HPM_SPI0,
    .write_cs              = NULL,
    .tx_buff               = 0,
    .tx_size               = 0,
    .tx_count              = 0,
    .rx_buff               = 0,
    .rx_size               = 0,
    .rx_count              = 0,
    .data_len_in_byte      = 1,
    .per_trans_max = SPI_SOC_TRANSFER_COUNT_MAX,
    .dma_context = {
            .dma_ptr       = 0,
            .rx_dma_ch     = 0,
            .tx_dma_ch     = 0,
            .dmamux_ptr    = HPM_DMAMUX,
            .rx_req        = HPM_DMA_SRC_SPI0_RX,
            .tx_req        = HPM_DMA_SRC_SPI0_TX,
            .data_width    = DMA_TRANSFER_WIDTH_BYTE,
    },
    .running_core          = HPM_CORE0,
    .dma_linked_descriptor = spi0_dma_linked_descriptor,
    .spi_transctrl         = spi0_transctrl,
};
#endif

static struct hpm_spidev_s g_spi0dev =
{
  .spidev                  =
  {
    .ops                   = &g_spi0ops,
  },
  .spibase                 = HPM_SPI0,
  .spiclock                = clock_spi0,
  .spiirq                  = IRQn_SPI0,
#ifdef CONFIG_HPM_SPI0_DMA
  .spi_context             = &spi0_context,
#endif
#ifdef CONFIG_PM
  .pm_cb.prepare           = spi_pm_prepare,
#endif
  .config                  = FULL_DUPLEX,
};

#endif /* CONFIG_HPMICRO_SPI0 */

#ifdef CONFIG_HPM_SPI1
static const struct spi_ops_s g_sp1iops =
{
  .lock                   = spi_lock,
  .select                 = hpm_spi1select,
  .setfrequency           = spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay               = spi_setdelay,
#endif
  .setmode                = spi_setmode,
  .setbits                = spi_setbits,
  .status                 = hpm_spi1status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata                = hpm_spi1cmddata,
#endif
  .send                   = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange               = spi_exchange,
#else
  .sndblock               = spi_sndblock,
  .recvblock              = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger                = spi_trigger,
#endif
  .registercallback       = 0,                   /* Not implemented */
};

#ifdef CONFIG_HPM_SPI1_DMA
/* dma descriptors need align 8 bytes */
ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(8) dma_linked_descriptor_t spi1_dma_linked_descriptor[HPM_MAX_SPI_DMA_COUNT * SPI_DMA_DESC_COUNT_PER_TRANS];
ATTR_PLACE_AT_NONCACHEABLE uint32_t spi1_transctrl[HPM_MAX_SPI_DMA_COUNT];
spi_context_t spi1_context = {
    .ptr                   = HPM_SPI1,
    .write_cs              = NULL,
    .tx_buff               = 0,
    .tx_size               = 0,
    .tx_count              = 0,
    .rx_buff               = 0,
    .rx_size               = 0,
    .rx_count              = 0,
    .data_len_in_byte      = 1,
    .per_trans_max         = SPI_SOC_TRANSFER_COUNT_MAX,
    .dma_context = {
            .dma_ptr       = 0,
            .rx_dma_ch     = 0,
            .tx_dma_ch     = 0,
            .dmamux_ptr    = HPM_DMAMUX,
            .rx_req        = HPM_DMA_SRC_SPI1_RX,
            .tx_req        = HPM_DMA_SRC_SPI1_TX,
            .data_width    = DMA_TRANSFER_WIDTH_BYTE,
    },
    .running_core          = HPM_CORE0,
    .dma_linked_descriptor = spi1_dma_linked_descriptor,
    .spi_transctrl         = spi1_transctrl,
};
#endif

static struct hpm_spidev_s g_spi1dev =
{
  .spidev                  =
  {
    .ops                   = &g_sp1iops,
  },
  .spibase                 = HPM_SPI1,
  .spiclock                = clock_spi1,
  .spiirq                  = IRQn_SPI1,
#if defined(CONFIG_HPM_SPI1_DMA) 
  .spi_context             = &spi1_context,
#endif
#ifdef CONFIG_PM
  .pm_cb.prepare           = spi_pm_prepare,
#endif
  .config                  = FULL_DUPLEX,
};

#endif /* CONFIG_HPM_SPI1 */

#ifdef CONFIG_HPM_SPI2
static const struct spi_ops_s g_sp2iops =
{
  .lock                    = spi_lock,
  .select                  = hpm_spi2select,
  .setfrequency            = spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay                = spi_setdelay,
#endif
  .setmode                 = spi_setmode,
  .setbits                 = spi_setbits,
  .status                  = hpm_spi2status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata                 = hpm_spi2cmddata,
#endif
  .send                    = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange                = spi_exchange,
#else
  .sndblock                = spi_sndblock,
  .recvblock               = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger                 = spi_trigger,
#endif
  .registercallback        = 0,  /* not implemented */
};

#ifdef CONFIG_HPM_SPI2_DMA
/* dma descriptors need align 8 bytes */
ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(8) dma_linked_descriptor_t spi2_dma_linked_descriptor[HPM_MAX_SPI_DMA_COUNT * SPI_DMA_DESC_COUNT_PER_TRANS];
ATTR_PLACE_AT_NONCACHEABLE uint32_t spi2_transctrl[HPM_MAX_SPI_DMA_COUNT];
spi_context_t spi2_context = {
    .ptr                   = HPM_SPI2,
    .write_cs              = NULL,
    .tx_buff               = 0,
    .tx_size               = 0,
    .tx_count              = 0,
    .rx_buff               = 0,
    .rx_size               = 0,
    .rx_count              = 0,
    .data_len_in_byte      = 1,
    .per_trans_max         = SPI_SOC_TRANSFER_COUNT_MAX,
    .dma_context = {
            .dma_ptr       = 0,
            .rx_dma_ch     = 0,
            .tx_dma_ch     = 0,
            .dmamux_ptr    = HPM_DMAMUX,
            .rx_req        = HPM_DMA_SRC_SPI2_RX,
            .tx_req        = HPM_DMA_SRC_SPI2_TX,
            .data_width    = DMA_TRANSFER_WIDTH_BYTE,
    },
    .running_core          = HPM_CORE0,
    .dma_linked_descriptor = spi2_dma_linked_descriptor,
    .spi_transctrl         = spi2_transctrl,
};
#endif

static struct hpm_spidev_s g_spi2dev =
{
  .spidev                  =
  {
    .ops                   = &g_sp2iops,
  },
  .spibase                 = HPM_SPI2,
  .spiclock                = clock_spi2,
  .spiirq                  = IRQn_SPI2,
#if defined(CONFIG_HPM_SPI2_DMA)
  .spi_context             = &spi2_context,
#endif
#ifdef CONFIG_PM
  .pm_cb.prepare           = spi_pm_prepare,
#endif
  .config                  = FULL_DUPLEX,
};

#endif /* CONFIG_HPM_SPI2 */

#ifdef CONFIG_HPM_SPI3
static const struct spi_ops_s g_sp3iops =
{
  .lock                    = spi_lock,
  .select                  = hpm_spi3select,
  .setfrequency            = spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay                = spi_setdelay,
#endif
  .setmode                 = spi_setmode,
  .setbits                 = spi_setbits,
  .status                  = hpm_spi3status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata                 = hpm_spi3cmddata,
#endif
  .send                    = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange                = spi_exchange,
#else
  .sndblock                = spi_sndblock,
  .recvblock               = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger                 = spi_trigger,
#endif
  .registercallback        = 0,                   /* not implemented */
};

#ifdef CONFIG_HPM_SPI2_DMA
/* dma descriptors need align 8 bytes */
ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(8) dma_linked_descriptor_t spi3_dma_linked_descriptor[HPM_MAX_SPI_DMA_COUNT * SPI_DMA_DESC_COUNT_PER_TRANS];
ATTR_PLACE_AT_NONCACHEABLE uint32_t spi3_transctrl[HPM_MAX_SPI_DMA_COUNT];
spi_context_t spi3_context = {
    .ptr                   = HPM_SPI3,
    .write_cs              = NULL,
    .tx_buff               = 0,
    .tx_size               = 0,
    .tx_count              = 0,
    .rx_buff               = 0,
    .rx_size               = 0,
    .rx_count              = 0,
    .data_len_in_byte      = 1,
    .per_trans_max         = SPI_SOC_TRANSFER_COUNT_MAX,
    .dma_context = {
            .dma_ptr       = 0,
            .rx_dma_ch     = 0,
            .tx_dma_ch     = 0,
            .dmamux_ptr    = HPM_DMAMUX,
            .rx_req        = HPM_DMA_SRC_SPI3_RX,
            .tx_req        = HPM_DMA_SRC_SPI3_TX,
            .data_width    = DMA_TRANSFER_WIDTH_BYTE,
    },
    .running_core          = HPM_CORE0,
    .dma_linked_descriptor = spi3_dma_linked_descriptor,
    .spi_transctrl         = spi3_transctrl,
};
#endif

static struct hpm_spidev_s g_spi3dev =
{
  .spidev                  =
  {
    .ops                   = &g_sp3iops,
  }, 
  .spibase                 = HPM_SPI3,
  .spiclock                = clock_spi3,
  .spiirq                  = IRQn_SPI3,
#if defined(CONFIG_HPM_SPI3_DMA)
  .spi_context             = &spi3_context,
#endif
#ifdef CONFIG_PM
  .pm_cb.prepare           = spi_pm_prepare,
#endif
  .config                  = FULL_DUPLEX,
};

#endif /* CONFIG_HPM_SPI3 */


/****************************************************************************
 * Private Functions
 ****************************************************************************/

uint32_t get_frequency_for_clock_source(clock_source_t source)
{
    uint32_t clk_freq = 0UL;
    uint32_t div = 1;
    switch (source) {
    case clock_source_osc0_clk0:
        clk_freq = 24000000UL;
        break;
    case clock_source_pll0_clk0:
        clk_freq = pllctl_get_pll_freq_in_hz(HPM_PLLCTL, 0U);
        break;
    case clock_source_pll1_clk0:
        div = pllctl_get_div(HPM_PLLCTL, 1, 0);
        clk_freq = pllctl_get_pll_freq_in_hz(HPM_PLLCTL, 1U) / div;
        break;
    case clock_source_pll1_clk1:
        div = pllctl_get_div(HPM_PLLCTL, 1, 1);
        clk_freq = pllctl_get_pll_freq_in_hz(HPM_PLLCTL, 1U) / div;
        break;
    case clock_source_pll2_clk0:
        div = pllctl_get_div(HPM_PLLCTL, 2, 0);
        clk_freq = pllctl_get_pll_freq_in_hz(HPM_PLLCTL, 2U) / div;
        break;
    case clock_source_pll2_clk1:
        div = pllctl_get_div(HPM_PLLCTL, 2, 1);
        clk_freq = pllctl_get_pll_freq_in_hz(HPM_PLLCTL, 2U) / div;
        break;
    case clock_source_pll3_clk0:
        clk_freq = pllctl_get_pll_freq_in_hz(HPM_PLLCTL, 3U);
        break;
    case clock_source_pll4_clk0:
        clk_freq = pllctl_get_pll_freq_in_hz(HPM_PLLCTL, 4U);
        break;
    default:
        clk_freq = 0UL;
        break;
    }

    return clk_freq;
}

/****************************************************************************
 * Name: spi_dumpregs
 ****************************************************************************/

#ifdef CONFIG_DEBUG_SPI_INFO
static void spi_dumpregs(struct hpm_spidev_s *priv)
{
    return;
}
#endif

/****************************************************************************
 * Name: spi_interrupt
 *
 * Description:
 *   In DMA transfer mode, handle the transmission complete interrupt
 *
 ****************************************************************************/
#ifdef CONFIG_HPM_SPI_INTERRUPTS
static int spi_interrupt(int irq, void *context, void *arg)
{
  struct hpm_spidev_s *priv = (struct hpm_spidev_s *)arg;
  volatile uint32_t irq_status;
  irq_status = spi_get_interrupt_status((SPI_Type *)priv->spibase); /* get interrupt stat */
  if (irq_status & spi_end_int)
    {
      /* Disable interrupt. This needs to be done
       * before disabling SPI to avoid spurious TXC interrupt
       * (ERRATA: Q2.14.4 TXP interrupt occurring while SPI/I2Sdisabled")
       * There is no need to clear the interrupt since it is disabled and
       * SPI will be disabled after transmit as well.
       */

      spi_clear_interrupt_status((SPI_Type *)priv->spibase, spi_end_int);

      /* Set result and release wait semaphore */
#ifdef CONFIG_HPM_SPI_DMA
      nxsem_post(&priv->txsem);
#endif
    }

  return 0;
}
#endif

/****************************************************************************
 * Name: spi_dmarxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ****************************************************************************/

#ifdef CONFIG_HPM_SPI_DMA
static int spi_dmarxwait(struct hpm_spidev_s *priv)
{
  int ret;

  /* Don't wait in tx only mode */

  if (priv->config == SIMPLEX_TX)
    {
      return OK;
    }

  /* Take the semaphore (perhaps waiting).  If the result is zero, then the
   * DMA must not really have completed???
   */

  ret = nxsem_wait_uninterruptible(&priv->rxsem);
  return ret;
}

/****************************************************************************
 * Name: spi_dmatxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ****************************************************************************/

static int spi_dmatxwait(struct hpm_spidev_s *priv)
{
  int ret;

  /* Don't wait in rx only mode */

  if (priv->config == SIMPLEX_RX)
    {
      return OK;
    }

  /* Take the semaphore (perhaps waiting).  If the result is zero, then the
   * DMA must not really have completed???
   */
  ret = nxsem_wait_uninterruptible(&priv->txsem);
  return ret;
}

/****************************************************************************
 * Name: spi_dmarx tx complete callback
 *
 * Description:
 *   Signal that DMA is complete
 *
 ****************************************************************************/

static void  spi_rx_dma_channel_tc_callback(DMA_Type *ptr, uint32_t channel, void *user_data)
{
  struct hpm_spidev_s *priv = (struct hpm_spidev_s *)user_data;
  UNUSED(channel);
  spiinfo("RX interrupt fired with tc status\n");  
  nxsem_post(&priv->rxsem);

}

static void  spi_tx_dma_channel_tc_callback(DMA_Type *ptr, uint32_t channel, void *user_data)
{
  struct hpm_spidev_s *priv = (struct hpm_spidev_s *)user_data;
  UNUSED(channel);
  spiinfo("TX interrupt fired with tc status\n");
  nxsem_post(&priv->txsem);
}
#endif

/****************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   On SPI buses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the buses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI bus is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct hpm_spidev_s *priv = (struct hpm_spidev_s *)dev;
  int ret;

  if (lock)
    {
      ret = nxsem_wait_uninterruptible(&priv->exclsem);
    }
  else
    {
      ret = nxsem_post(&priv->exclsem);
    }

  return ret;
}

/****************************************************************************
 * Name: spi_enable
 ****************************************************************************/

static inline int spi_enable(struct hpm_spidev_s *priv, bool state)
{
  int ret = OK;

  if (state == true)
    {
      /* Enable SPI TODO*/

    }
  else
    {
      /* Disable SPI TODO*/

    }

  return ret;
}

/****************************************************************************
 * Name: spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t spi_setfrequency(struct spi_dev_s *dev,
                                 uint32_t frequency)
{
  struct hpm_spidev_s *priv = (struct hpm_spidev_s *)dev;
  uint32_t actual  = 0;
  spi_timing_config_t timing_config = {0};
  SPI_Type *spi_ptr = (SPI_Type *)priv->spibase;
  int freq_list[clock_source_general_source_end] = {0};
  int min_diff_freq;
  int current_diff_freq;
  int best_freq;
  uint32_t div, i;
  clock_source_t clock_source;
  clk_src_t clk_src;
  uint32_t pll_clk = 0;
  /* Limit to max possible (if STM32_SPI_CLK_MAX is defined in board.h) */

  if (frequency > HPMICRO_SPI_CLK_MAX)
    {
      frequency = HPMICRO_SPI_CLK_MAX;
    }

  /* Has the frequency changed? */

  if (frequency != priv->frequency)
    {
      if (frequency <= 40000000)
        {
          /* set SPI sclk frequency for master */
          spi_master_get_default_timing_config(&timing_config);
          timing_config.master_config.clk_src_freq_in_hz = clock_get_frequency(priv->spiclock);
          timing_config.master_config.sclk_freq_in_hz = frequency;
          actual = frequency;
          if (spi_master_timing_init((SPI_Type *)priv->spibase, &timing_config) !=status_success)
            {
                spi_ptr->TIMING = (spi_ptr->TIMING & ~SPI_TIMING_SCLK_DIV_MASK) | SPI_TIMING_SCLK_DIV_SET(0); /* 40 M */
                actual = 40000000;
            }
        }
      else
        {
          spi_ptr->TIMING = (spi_ptr->TIMING & ~SPI_TIMING_SCLK_DIV_MASK) | SPI_TIMING_SCLK_DIV_SET(0xFF);
          for (clock_source = (clock_source_t)0; clock_source < clock_source_general_source_end; clock_source++)
            {
              pll_clk = get_frequency_for_clock_source(clock_source);
              div = pll_clk / frequency;
                /* The division factor ranges from 1 to 256 as any integer */
              if ((div > 0) && (div <= 0x100))
                {
                  freq_list[clock_source] = pll_clk / div;
                }
            }
            /* Find the best sclk frequency */
            min_diff_freq = abs(freq_list[0] - frequency);
            best_freq = freq_list[0];
            for (i = 1; i < clock_source_general_source_end; i++)
              {
                current_diff_freq = abs(freq_list[i] - frequency);
                if (current_diff_freq < min_diff_freq)
                {
                  min_diff_freq = current_diff_freq;
                  best_freq = freq_list[i];
                }
              }
            /* Find the best spi clock frequency */
            for (i = 0; i < clock_source_general_source_end; i++)
              {
                if (best_freq == freq_list[i])
                  {
                    pll_clk = get_frequency_for_clock_source((clock_source_t)i);
                    clk_src = MAKE_CLK_SRC(CLK_SRC_GROUP_COMMON, i);
                    div = pll_clk / best_freq;
                    clock_set_source_divider(priv->spiclock, clk_src, div);
                    break;
                  }
              }
            actual =  clock_get_frequency(priv->spiclock);
        }
      spiinfo("Frequency %" PRId32 "->%" PRId32 "\n", frequency, actual);
      priv->frequency = frequency;
      priv->actual    = actual;
    }

  return priv->actual;
}

/****************************************************************************
 * Name: spi_setdelay
 *
 * Description:
 *   Set the SPI Delays in nanoseconds. Optional.
 *
 * Input Parameters:
 *   dev        - Device-specific state data
 *   startdelay - The delay between CS active and first CLK
 *   stopdelay  - The delay between last CLK and CS inactive
 *   csdelay    - The delay between CS inactive and CS active again
 *   ifdelay    - The delay between frames
 *
 * Returned Value:
 *   Returns zero (OK) on success; a negated errno value is return on any
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_DELAY_CONTROL
static int spi_setdelay(struct spi_dev_s *dev, uint32_t startdelay,
                        uint32_t stopdelay, uint32_t csdelay,
                        uint32_t ifdelay)
{
  /* TODO */
  return OK;
}
#endif

/****************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode.  see enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static void spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct hpm_spidev_s *priv = (struct hpm_spidev_s *)dev;
  spi_format_config_t format_config = {0};

  spiinfo("mode=%" PRIx32 "\n", (uint32_t) mode);

  /* Has the mode changed? */
  uint32_t transfmt = priv->spibase->TRANSFMT;
  if (mode != priv->mode)
    {
      /* set SPI format config for master */
      format_config.common_config.data_len_in_bits  = SPI_TRANSFMT_DATALEN_GET(transfmt);
      format_config.common_config.data_merge        = false;
      format_config.common_config.lsb               = SPI_TRANSFMT_LSB_GET(transfmt);
      format_config.common_config.mode              = SPI_TRANSFMT_SLVMODE_SET(transfmt);
      format_config.common_config.mosi_bidir        = SPI_TRANSFMT_MOSIBIDIR_GET(transfmt);
      format_config.master_config.addr_len_in_bytes = SPI_TRANSFMT_ADDRLEN_GET(transfmt);
      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          format_config.common_config.cpol = spi_sclk_low_idle;
          format_config.common_config.cpha = spi_sclk_sampling_odd_clk_edges;
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          format_config.common_config.cpol = spi_sclk_low_idle;
          format_config.common_config.cpha = spi_sclk_sampling_even_clk_edges;
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          format_config.common_config.cpol = spi_sclk_high_idle;
          format_config.common_config.cpha = spi_sclk_sampling_odd_clk_edges;
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          format_config.common_config.cpol = spi_sclk_high_idle;
          format_config.common_config.cpha = spi_sclk_sampling_even_clk_edges;
          break;

        default:
          return;
        }

      /* Change SPI mode */

      spi_format_init((SPI_Type *)priv->spibase, &format_config);

      /* Save the mode so that subsequent re-configurations will be faster */

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   nbits - The number of bits requested
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct hpm_spidev_s *priv = (struct hpm_spidev_s *)dev;

  spi_format_config_t format_config = {0};

  spiinfo("nbits=%d\n", nbits);

  /* Has the number of bits changed? */
  uint32_t transfmt = priv->spibase->TRANSFMT;
  if (nbits != priv->nbits)
    {
      /* set SPI format config for master */    
      format_config.common_config.data_merge        = false;
      format_config.common_config.lsb               = SPI_TRANSFMT_LSB_GET(transfmt);
      format_config.common_config.mode              = SPI_TRANSFMT_SLVMODE_SET(transfmt);
      format_config.common_config.mosi_bidir        = SPI_TRANSFMT_MOSIBIDIR_GET(transfmt);
      format_config.master_config.addr_len_in_bytes = SPI_TRANSFMT_ADDRLEN_GET(transfmt);
      format_config.common_config.cpha              = SPI_TRANSFMT_CPHA_GET(transfmt);
      format_config.common_config.cpol              = SPI_TRANSFMT_CPOL_GET(transfmt);

      if (nbits < 4 || nbits > 32)
        {
          return;
        }

      format_config.common_config.data_len_in_bits  = nbits;

      spi_format_init((SPI_Type *)priv->spibase, &format_config);

      /* Save the selection so that subsequent re-configurations will be
       * faster.
       */

      priv->nbits = nbits;
    }
}

/****************************************************************************
 * Name: bl602_spi_hwfeatures
 *
 * Description:
 *   Set hardware-specific feature flags.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   features - H/W feature flags
 *
 * Returned Value:
 *   Zero (OK) if the selected H/W features are enabled; A negated errno
 *   value if any H/W feature is not supportable.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_HWFEATURES
static int spi_hwfeatures(struct spi_dev_s *dev,
                                spi_hwfeatures_t features)
{
  /* Other H/W features are not supported */

  spierr("SPI hardware specific feature not supported\n");
  DEBUGPANIC();

  return -1;
}
#endif

/****************************************************************************
 * Name: spi_send
 *
 * Description:
 *   Exchange one word on SPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct hpm_spidev_s *priv = (struct hpm_spidev_s *)dev;
  uint32_t regval = 0;
  spi_control_config_t control_config = {0};
  hpm_stat_t stat;

  DEBUGASSERT(priv && priv->spibase);

  /* Master transfer start */
  /* set SPI control config for master */

  spi_master_get_default_control_config(&control_config);
  control_config.master_config.cmd_enable = false;  /* cmd phase control for master */
  control_config.master_config.addr_enable = false; /* address phase control for master */

  if (priv->config == FULL_DUPLEX)
    {
      control_config.common_config.trans_mode = spi_trans_write_read_together;
    }
  else if(priv->config == SIMPLEX_RX)
    {
      control_config.common_config.trans_mode = spi_trans_read_only;
    }
  else if(priv->config == SIMPLEX_TX) 
    {
      control_config.common_config.trans_mode = spi_trans_write_only;
    }
  else
    {
      control_config.common_config.trans_mode = spi_trans_write_only;
    }
  // printf("spi_send %d\n", control_config.common_config.trans_mode);
  stat = hpm_spi_transfer((SPI_Type *)priv->spibase,
                &control_config,
                NULL, NULL,
                (uint8_t *)&wd, 1, (uint8_t *)&regval, 1);
  UNUSED(regval);
  return stat;
}

/****************************************************************************
 * Name: spi_exchange (no DMA).  aka spi_exchange_nodma
 *
 * Description:
 *   Exchange a block of data on SPI without using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
static void spi_exchange_nodma(struct spi_dev_s *dev,
                               const void *txbuffer, void *rxbuffer,
                               size_t nwords)
{
  struct hpm_spidev_s *priv = (struct hpm_spidev_s *)dev;
  spi_control_config_t control_config = {0};
  size_t len = nwords ;
  size_t inc_len = 0;
  size_t dummy_len = 0;
  uint8_t *tx_buffer = (uint8_t *)txbuffer;
  uint8_t *rx_buffer = (uint8_t *)rxbuffer;
  DEBUGASSERT(priv && priv->spibase);

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", tx_buffer, rx_buffer, nwords);

  /* set SPI control config for master */

  spi_master_get_default_control_config(&control_config);
  control_config.master_config.cmd_enable = false;  /* cmd phase control for master */
  control_config.master_config.addr_enable = false; /* address phase control for master */

  if(!tx_buffer)
    {
      control_config.common_config.trans_mode = spi_trans_read_only;
    }
  else if(!rx_buffer)
    {
      control_config.common_config.trans_mode = spi_trans_write_only;
    }
  else if(tx_buffer && rx_buffer)
    {
      control_config.common_config.trans_mode = spi_trans_write_read_together;
    }
  else
    {
      return;
    }
  // printf("spi_exchange_nodma: %d %d \n",len,spi_get_data_length_in_bytes(priv->spibase));
  while(len > 0)
    {
      dummy_len = (len > SPI_SOC_TRANSFER_COUNT_MAX) ? SPI_SOC_TRANSFER_COUNT_MAX : len;
      if(!tx_buffer)
        {
          hpm_spi_transfer(priv->spibase, &control_config, NULL, NULL, NULL, 0, (uint8_t *)&rx_buffer[inc_len], dummy_len);
        }
      else if(!rx_buffer)
        {
          hpm_spi_transfer(priv->spibase, &control_config, NULL, NULL, (uint8_t *)&tx_buffer[inc_len], dummy_len, NULL, 0);
        }
      else if(tx_buffer && rx_buffer)
        {
          hpm_spi_transfer(priv->spibase,
            &control_config,
            NULL, NULL,
            (uint8_t *)&tx_buffer[inc_len], dummy_len,(uint8_t *)&rx_buffer[inc_len], dummy_len);
        }
      len      -= dummy_len;
      inc_len  += dummy_len;
    }
}

#endif /* !CONFIG_HPM_SPI_DMA || CONFIG_HPMICRO_DMACAPABLE ||
        * CONFIG_HPMICRO_SPI_DMATHRESHOLD
        */

/****************************************************************************
 * Name: spi_exchange (with DMA capability)
 *
 * Description:
 *   Exchange a block of data on SPI using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
{
  struct hpm_spidev_s *priv = (struct hpm_spidev_s *)dev;
  DEBUGASSERT(priv != NULL);
  
#ifdef CONFIG_HPM_SPI_DMA
  spi_control_config_t control_config = {0};
  uint8_t cmd = 0x1a;
  uint32_t addr = 0x10;
  hpm_stat_t stat;
  size_t len = 0 ;
  size_t inc_len = 0;
  size_t dummy_len = 0;
  len = nwords;
  if ((priv->dma_rxresource.base == NULL) || (priv->dma_txresource.base == NULL))
    {
      spi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
      return;
    }
  else
    { 
    nwords = spi_get_data_length_in_bytes((SPI_Type *)priv->spibase) * nwords;
    len = nwords;
        /* set SPI control config for master */
    spi_master_get_default_control_config(&control_config);
    control_config.master_config.cmd_enable     = false;
    control_config.master_config.addr_enable    = false;
    control_config.master_config.addr_phase_fmt = spi_address_phase_format_single_io_mode;   
    control_config.common_config.data_phase_fmt = spi_single_io_mode;
    control_config.common_config.dummy_cnt      = spi_dummy_count_1;

    while(len > 0)
      {
        dummy_len = (len > SPI_SOC_TRANSFER_COUNT_MAX) ? SPI_SOC_TRANSFER_COUNT_MAX : len;
        if(!txbuffer)
          {
            control_config.common_config.trans_mode = spi_trans_read_only;
            control_config.common_config.rx_dma_enable  = true;
            priv->spi_context->tx_size                  = 0;
            priv->spi_context->rx_size                  = dummy_len;
            priv->config = SIMPLEX_RX;
          }
        else if(!rxbuffer)
          {
            control_config.common_config.trans_mode     = spi_trans_write_only;
            control_config.common_config.tx_dma_enable  = true;
            priv->spi_context->tx_size                  = dummy_len;
            priv->spi_context->rx_size                  = 0;
            priv->config = SIMPLEX_TX;
          }
        else if(txbuffer && rxbuffer)
          {
            control_config.common_config.trans_mode = spi_trans_write_read_together;
            control_config.common_config.tx_dma_enable  = true;
            control_config.common_config.rx_dma_enable  = true;
            priv->spi_context->tx_size                  = dummy_len;
            priv->spi_context->rx_size                  = dummy_len;
            priv->config = FULL_DUPLEX;
          }
        else
          {
            return;
          }
          priv->spi_context->cmd              = cmd;
          priv->spi_context->addr             = addr;
          priv->spi_context->data_len_in_byte = spi_get_data_length_in_bytes((SPI_Type *)priv->spibase);

          priv->spi_context->tx_buff          = &((uint8_t *)txbuffer)[inc_len];    
          priv->spi_context->tx_count         = priv->spi_context->tx_size / priv->spi_context->data_len_in_byte;

          priv->spi_context->rx_buff          = &((uint8_t *)&rxbuffer)[inc_len];
          priv->spi_context->rx_count         = priv->spi_context->rx_size / priv->spi_context->data_len_in_byte;

          priv->spi_context->dma_context.data_width = spi_get_data_length_in_bytes((SPI_Type *)priv->spibase) - 1;
          
          stat = hpm_spi_setup_dma_transfer(priv->spi_context, &control_config);
          if (stat != status_success) 
            {
              return;
            }
          spi_dmarxwait(priv);
          spi_dmatxwait(priv);
          len      -= dummy_len;
          inc_len  += dummy_len;
      }
      /* change config status */
      priv->config = FULL_DUPLEX;
    }
#else
      spi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
      return;
#endif

#ifdef CONFIG_SPI_TRIGGER
      priv->trigarmed = false;
#endif
}

/****************************************************************************
 * Name: spi_trigger
 *
 * Description:
 *   Trigger a previously configured DMA transfer.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   OK       - Trigger was fired
 *   ENOTSUP  - Trigger not fired due to lack of DMA support
 *   EIO      - Trigger not fired because not previously primed
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_TRIGGER
static int spi_trigger(struct spi_dev_s *dev)
{
  return OK;
}
#endif

/****************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   nwords   - the length of data to send from the buffer in number of
 *              words. The wordsize is determined by the number of
 *              bits-per-word selected for the SPI interface.  If nbits <= 8,
 *              the data is packed into uint8_t's; if nbits >8, the data is
 *              packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(struct spi_dev_s *dev,
                         const void *txbuffer,
                         size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%d\n", txbuffer, nwords);
  return spi_exchange(dev, txbuffer, NULL, nwords);
}
#endif

/****************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that can be received in the buffer in
 *              number of words.  The wordsize is determined by the number of
 *              bits-per-word selected for the SPI interface.  If nbits <= 8,
 *              the data is packed into uint8_t's; if nbits >8, the data is
 *              packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_recvblock(struct spi_dev_s *dev,
                          void *rxbuffer,
                          size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);
  return spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: spi_pm_prepare
 *
 * Description:
 *   Request the driver to prepare for a new power state. This is a
 *   warning that the system is about to enter into a new power state.  The
 *   driver should begin whatever operations that may be required to enter
 *   power state.  The driver may abort the state change mode by returning
 *   a non-zero value from the callback function.
 *
 * Input Parameters:
 *   cb      - Returned to the driver.  The driver version of the callback
 *             structure may include additional, driver-specific state
 *             data at the end of the structure.
 *   domain  - Identifies the activity domain of the state change
 *   pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   0 (OK) means the event was successfully processed and that the driver
 *   is prepared for the PM state change.  Non-zero means that the driver
 *   is not prepared to perform the tasks needed achieve this power setting
 *   and will cause the state change to be aborted.  NOTE:  The prepare
 *   method will also be recalled when reverting from lower back to higher
 *   power consumption modes (say because another driver refused a lower
 *   power state change).  Drivers are not permitted to return non-zero
 *   values when reverting back to higher power consumption modes!
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int spi_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate)
{
  struct hpm_spidev_s *priv =
      (struct hpm_spidev_s *)((char *)cb -
                                    offsetof(struct hpm_spidev_s, pm_cb));
  int sval;

  /* Logic to prepare for a reduced power state goes here. */

  switch (pmstate)
    {
    case PM_NORMAL:
    case PM_IDLE:
      break;

    case PM_STANDBY:
    case PM_SLEEP:

      /* Check if exclusive lock for SPI bus is held. */

      if (nxsem_get_value(&priv->exclsem, &sval) < 0)
        {
          DEBUGPANIC();
          return -EINVAL;
        }

      if (sval <= 0)
        {
          /* Exclusive lock is held, do not allow entry to deeper PM
           * states.
           */

          return -EBUSY;
        }

      break;

    default:

      /* Should not get here */

      break;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: spi_bus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus in its default state (Master, 8-bit,
 *   mode 0, etc.)
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_bus_initialize(struct hpm_spidev_s *priv)
{
  spi_setmode((struct spi_dev_s *)priv, SPIDEV_MODE0);
  spi_setbits((struct spi_dev_s *)priv, 8);

  /* Select a default frequency of approx. 400KHz */

  spi_setfrequency((struct spi_dev_s *)priv, 400000);
#ifdef CONFIG_HPM_SPI_DMA
  priv->dma_rxresource.base = NULL;
  priv->dma_txresource.base = NULL;
  if (dma_mgr_request_resource(&priv->dma_rxresource) == status_success)
  {
    dma_mgr_install_chn_tc_callback(&priv->dma_rxresource, spi_rx_dma_channel_tc_callback, (void *)priv);
    dma_mgr_enable_chn_irq(&priv->dma_rxresource, DMA_MGR_INTERRUPT_MASK_TC);
    priv->spi_context->dma_context.dma_ptr      = priv->dma_rxresource.base;
    priv->spi_context->dma_context.rx_dma_ch    = priv->dma_rxresource.channel;
    priv->spi_context->dma_context.rx_dmamux_ch = DMA_SOC_CHN_TO_DMAMUX_CHN(priv->dma_rxresource.base, priv->dma_rxresource.channel);
    dma_mgr_enable_chn_irq(&priv->dma_rxresource, DMA_INTERRUPT_MASK_TERMINAL_COUNT);
    dma_mgr_enable_channel(&priv->dma_rxresource);
  }

  if (dma_mgr_request_resource(&priv->dma_txresource) == status_success)
  {
    dma_mgr_install_chn_tc_callback(&priv->dma_txresource, spi_tx_dma_channel_tc_callback, (void *)priv);
    dma_mgr_enable_chn_irq(&priv->dma_txresource, DMA_MGR_INTERRUPT_MASK_TC);
    priv->spi_context->dma_context.dma_ptr      = priv->dma_txresource.base;
    priv->spi_context->dma_context.tx_dma_ch    = priv->dma_txresource.channel;
     priv->spi_context->dma_context.tx_dmamux_ch = DMA_SOC_CHN_TO_DMAMUX_CHN(priv->dma_txresource.base, priv->dma_txresource.channel);
    dma_mgr_enable_chn_irq(&priv->dma_txresource, DMA_INTERRUPT_MASK_TERMINAL_COUNT);
    dma_mgr_enable_channel(&priv->dma_txresource);
  }
#endif

#ifdef CONFIG_PM
  /* Register to receive power management callbacks */

  ret = pm_register(&priv->pm_cb);
  DEBUGASSERT(ret == OK);
  UNUSED(ret);
#endif

#ifdef CONFIG_DEBUG_SPI_INFO
  /* Dump registers after initialization */

  spi_dumpregs(priv);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_spibus_initialize
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

struct spi_dev_s *hpm_spibus_initialize(int bus)
{
  struct hpm_spidev_s *priv = NULL;

  irqstate_t flags = enter_critical_section();

#ifdef CONFIG_HPM_SPI0
  if (bus == 0)
    {
      /* Select SPI0 */

      priv = &g_spi0dev;

      /* Only configure if the bus is not already configured */

      if (!priv->initialized)
        {
          /* Configure SPI0 pins: SCK, MISO, and MOSI */

          hpm_spibus_pins_init(bus);

          /* Initialize the SPI semaphore that enforces mutually exclusive access. */

          nxsem_init(&priv->exclsem, 0, 1);

          /* Set up default configuration: Master, 8-bit, etc. */

#if defined(CONFIG_HPM_SPI0_DMA) 
          nxsem_init(&priv->rxsem, 0, 0);
          nxsem_init(&priv->txsem, 0, 0);

          nxsem_set_protocol(&priv->rxsem, SEM_PRIO_NONE);
          nxsem_set_protocol(&priv->txsem, SEM_PRIO_NONE);
          priv->spi_context->cs_pin   = hpm_spibus_get_cs_pin(bus);
          priv->spi_context->write_cs = write_spi0_cs;
#endif
          spi_bus_initialize(priv);
          priv->initialized = true;
        }
    }
  else
#endif
#ifdef CONFIG_HPM_SPI1
  if (bus == 1)
    {
      /* Select SPI1 */

      priv = &g_spi1dev;

      /* Only configure if the bus is not already configured */

      if (!priv->initialized)
        {
          /* Configure SPI1 pins: SCK, MISO, and MOSI */

          hpm_spibus_pins_init(bus);

          /* Initialize the SPI semaphore that enforces mutually exclusive access. */

          nxsem_init(&priv->exclsem, 0, 1);

          /* Set up default configuration: Master, 8-bit, etc. */

#if defined(CONFIG_HPM_SPI1_DMA)
          nxsem_init(&priv->rxsem, 0, 0);
          nxsem_init(&priv->txsem, 0, 0);

          nxsem_set_protocol(&priv->rxsem, SEM_PRIO_NONE);
          nxsem_set_protocol(&priv->txsem, SEM_PRIO_NONE);

          priv->spi_context->cs_pin   = hpm_spibus_get_cs_pin(bus);
          priv->spi_context->write_cs = write_spi1_cs;
#endif
          spi_bus_initialize(priv);
          priv->initialized = true;
        }
    }
  else
#endif
#ifdef CONFIG_HPM_SPI2
  if (bus == 2)
    {
      /* Select SPI2 */

      priv = &g_spi2dev;

      /* Only configure if the bus is not already configured */

      if (!priv->initialized)
        {
          /* Configure SPI2 pins: SCK, MISO, and MOSI */

          hpm_spibus_pins_init(bus);

          /* Initialize the SPI semaphore that enforces mutually exclusive access. */

          nxsem_init(&priv->exclsem, 0, 1);

          /* Set up default configuration: Master, 8-bit, etc. */
#if defined(CONFIG_HPM_SPI2_DMA) 
          nxsem_init(&priv->rxsem, 0, 0);
          nxsem_init(&priv->txsem, 0, 0);

          nxsem_set_protocol(&priv->rxsem, SEM_PRIO_NONE);
          nxsem_set_protocol(&priv->txsem, SEM_PRIO_NONE);

          priv->spi_context->cs_pin   = hpm_spibus_get_cs_pin(bus);
          priv->spi_context->write_cs = write_spi2_cs;
#endif
          spi_bus_initialize(priv);
          priv->initialized = true;
        }
    }
  else
#endif
#ifdef CONFIG_HPM_SPI3
  if (bus == 3)
    {
      /* Select SPI3 */

      priv = &g_spi3dev;

      /* Only configure if the bus is not already configured */

      if (!priv->initialized)
        {
          /* Configure SPI3 pins: SCK, MISO, and MOSI */

          hpm_spibus_pins_init(bus);

          /* Initialize the SPI semaphore that enforces mutually exclusive access. */

          nxsem_init(&priv->exclsem, 0, 1);

          /* Set up default configuration: Master, 8-bit, etc. */

#if defined(CONFIG_HPM_SPI3_DMA) 
          nxsem_init(&priv->rxsem, 0, 0);
          nxsem_init(&priv->txsem, 0, 0);

          nxsem_set_protocol(&priv->rxsem, SEM_PRIO_NONE);
          nxsem_set_protocol(&priv->txsem, SEM_PRIO_NONE);

          priv->spi_context->cs_pin   = hpm_spibus_get_cs_pin(bus);
          priv->spi_context->write_cs = write_spi3_cs;
#endif
          spi_bus_initialize(priv);
          priv->initialized = true;
        }
    }
  else
#endif
    {
      spierr("ERROR: Unsupported SPI bus: %d\n", bus);
    }

  leave_critical_section(flags);
  return (struct spi_dev_s *)priv;
}

