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
 *   4. The handle returned by hpm6750_spibus_initialize() may then be used to
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
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>
#include <nuttx/power/pm.h>

#include <arch/board/board.h>

#include "board.h"
#include "spi/hpm_spi.h"
#include "hpm6750/hpm_spi.h"
#include "hpm_dma.h"
#include "hpm_clock_drv.h"
#include "hpm_spi_drv.h"
#include "hpm_spi_regs.h"
#include "hpm_gpio_drv.h"
#include "hpm_soc_feature.h"
#include "pinmux.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define  HPM6750_MAX_SPI_COUNT      4
#define  HPM6750_MAX_SPI_DMA_COUNT  129   /*max len is 65535, spi max len is 512. 65535/512.*/

#if defined(CONFIG_HPM6750_SPI0) || defined(CONFIG_HPM6750_SPI1) || \
    defined(CONFIG_HPM6750_SPI2) || defined(CONFIG_HPM6750_SPI3)
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
#if defined(CONFIG_HPM6750_SPI0_DMA) || defined(CONFIG_HPM6750_SPI1_DMA) || \
    defined(CONFIG_HPM6750_SPI2_DMA) || defined(CONFIG_HPM6750_SPI3_DMA)
#ifdef CONFIG_SPI_TRIGGER
  bool             defertrig;    /* Flag indicating that trigger should be deferred */
  bool             trigarmed;    /* Flag indicating that the trigger is armed */
#endif
  int8_t           dma_txchan;   /* DMA channel handle for RX transfers */
  int8_t           dma_rxchan;   /* DMA channel handle for TX transfers */
  uint32_t         rx_dmamux_ch; /* The RX DMAMUX channel number */
  uint32_t         tx_dmamux_ch; /* The TX DMAMUX channel number */
  sem_t            rxsem;        /* Wait for RX DMA to complete */
  sem_t            txsem;        /* Wait for TX DMA to complete */
#endif
  bool             initialized;  /* Has SPI interface been initialized */
  mutex_t          lock;         /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;    /* Requested clock frequency */
  uint32_t         actual;       /* Actual clock frequency */
  int8_t           nbits;        /* Width of word in bits */
  uint8_t          mode;         /* Mode 0,1,2,3 */
#ifdef CONFIG_PM
  struct pm_callback_s pm_cb;    /* PM callbacks */
#endif
  enum spi_config_e config;      /* full/half duplex, simplex transmit/read only */
#if defined(CONFIG_HPM6750_SPI0_DMA) || defined(CONFIG_HPM6750_SPI1_DMA) || \
    defined(CONFIG_HPM6750_SPI2_DMA) || defined(CONFIG_HPM6750_SPI3_DMA)
  spi_context_t    *spi_context; /* spi dma context*/
#endif
};

/* DMA descriptor type */

typedef struct gpio_cfg
{
  uint16_t ioc_pad;                    /* gpio ioc pad*/
  uint16_t ico_func_ctl;              /* gpio function mux */
}gpio_cfg_t;

typedef struct spi_gpio_cfg
{
  gpio_cfg_t cs;
  gpio_cfg_t miso;
  gpio_cfg_t mosi;
  gpio_cfg_t sclk;
}spi_gpio_cfg_t;

const spi_gpio_cfg_t spi_gpio_cfg_table[4] =
{
  {{IOC_PAD_PD22, IOC_PD22_FUNC_CTL_GPIO_D_22}, {IOC_PAD_PD26, IOC_PD26_FUNC_CTL_SPI0_MISO}, {IOC_PAD_PD21, IOC_PD21_FUNC_CTL_SPI0_MOSI}, {IOC_PAD_PD27, IOC_PD27_FUNC_CTL_SPI0_SCLK}},
  {{IOC_PAD_PE03, IOC_PE03_FUNC_CTL_GPIO_E_03}, {IOC_PAD_PD30, IOC_PD30_FUNC_CTL_SPI1_MISO}, {IOC_PAD_PE04, IOC_PE04_FUNC_CTL_SPI1_MOSI}, {IOC_PAD_PD31, IOC_PD31_FUNC_CTL_SPI1_SCLK}},
  {{IOC_PAD_PE31, IOC_PE31_FUNC_CTL_GPIO_E_31}, {IOC_PAD_PE28, IOC_PE28_FUNC_CTL_SPI2_MISO}, {IOC_PAD_PE30, IOC_PE30_FUNC_CTL_SPI2_MOSI}, {IOC_PAD_PE27, IOC_PE27_FUNC_CTL_SPI2_SCLK}},
  {{IOC_PAD_PB29, IOC_PB29_FUNC_CTL_GPIO_B_29}, {IOC_PAD_PC03, IOC_PC03_FUNC_CTL_SPI3_MISO}, {IOC_PAD_PB30, IOC_PB30_FUNC_CTL_SPI3_MOSI}, {IOC_PAD_PC02, IOC_PC02_FUNC_CTL_SPI3_SCLK}},
};
/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static inline uint32_t spi_getreg(struct hpm_spidev_s *priv,
                                  uint32_t offset);
static inline void spi_putreg(struct hpm_spidev_s *priv,
                              uint32_t offset, uint32_t value);
static inline uint32_t spi_readword(struct hpm_spidev_s *priv);
static inline void spi_writeword(struct hpm_spidev_s *priv,
                                 uint32_t byte);
#ifdef CONFIG_DEBUG_SPI_INFO
static inline void spi_dumpregs(struct hpm_spidev_s *priv);
#endif

/* DMA support */

#if defined(CONFIG_HPM6750_SPI0_DMA) || defined(CONFIG_HPM6750_SPI1_DMA) || \
    defined(CONFIG_HPM6750_SPI2_DMA) || defined(CONFIG_HPM6750_SPI3_DMA)
static int         spi_dmarxwait(struct hpm_spidev_s *priv);
static int         spi_dmatxwait(struct hpm_spidev_s *priv);
static void        spi_rx_dma_channel_callback(DMA_Type *ptr, 
                                              uint32_t channel, 
                                              void *user_data, 
                                              uint32_t int_stat);
static void        spi_tx_dma_channel_callback(DMA_Type *ptr, 
                                              uint32_t channel, 
                                              void *user_data, 
                                              uint32_t int_stat);
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

#ifdef CONFIG_HPM6750_SPI0
static const struct spi_ops_s g_spi0ops =
{
  .lock              = spi_lock,
  .select            = hpm6750_spi0select,
  .setfrequency      = spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay          = spi_setdelay,
#endif
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = hpm6750_spi0status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = hpm6750_spi0cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = spi_trigger,
#endif
  .registercallback  = 0,                   /* not implemented */
};

#ifdef CONFIG_HPM6750_SPI0_DMA
/* dma descriptors need align 8 bytes */
ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(8) dma_linked_descriptor_t spi0_dma_linked_descriptor[HPM6750_MAX_SPI_DMA_COUNT * SPI_DMA_DESC_COUNT_PER_TRANS];
ATTR_PLACE_AT_NONCACHEABLE uint32_t spi0_transctrl[HPM6750_MAX_SPI_DMA_COUNT];
spi_context_t spi0_context = {
    .ptr = HPM_SPI0,
    .write_cs = NULL,
    .tx_buff = 0,
    .tx_size = 0,
    .tx_count = 0,
    .rx_buff = 0,
    .rx_size = 0,
    .rx_count = 0,
    .data_len_in_byte = 1,
    .per_trans_max = SPI_SOC_TRANSFER_COUNT_MAX,
    .dma_context = {
            .dma_ptr = 0,
            .rx_dma_ch = 0,
            .tx_dma_ch = 0,
            .dmamux_ptr = HPM_DMAMUX,
            .rx_dmamux_ch = 0,
            .tx_dmamux_ch = 0,
            .rx_req = HPM_DMA_SRC_SPI0_RX,
            .tx_req = HPM_DMA_SRC_SPI0_TX,
            .data_width = DMA_TRANSFER_WIDTH_BYTE,
    },
    .running_core = HPM_CORE0,
    .dma_linked_descriptor = spi0_dma_linked_descriptor,
    .spi_transctrl = spi0_transctrl,
};
#endif

static struct hpm_spidev_s g_spi0dev =
{
  .spidev     =
  {
    .ops         = &g_spi0ops,
  },
  .spibase       = HPM_SPI0,
  .spiclock      = clock_spi0,
  .spiirq        = IRQn_SPI0,
#ifdef CONFIG_HPM6750_SPI0_DMA
  .spi_context   = &spi0_context,
  .rxsem         = SEM_INITIALIZER(0),
  .txsem         = SEM_INITIALIZER(0),
#endif
  .lock          = NXMUTEX_INITIALIZER,
#ifdef CONFIG_PM
  .pm_cb.prepare = spi_pm_prepare,
#endif
  .config        = SIMPLEX_TX,
};

#endif /* CONFIG_HPMICRO_SPI0 */

#ifdef CONFIG_HPM6750_SPI1
static const struct spi_ops_s g_sp1iops =
{
  .lock              = spi_lock,
  .select            = hpm6750_spi1select,
  .setfrequency      = spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay          = spi_setdelay,
#endif
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = hpm6750_spi1status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = hpm6750_spi1cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = spi_trigger,
#endif
  .registercallback  = 0,                   /* Not implemented */
};

#ifdef CONFIG_HPM6750_SPI1_DMA
/* dma descriptors need align 8 bytes */
ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(8) dma_linked_descriptor_t spi1_dma_linked_descriptor[HPM6750_MAX_SPI_DMA_COUNT * SPI_DMA_DESC_COUNT_PER_TRANS];
ATTR_PLACE_AT_NONCACHEABLE uint32_t spi1_transctrl[HPM6750_MAX_SPI_DMA_COUNT];
spi_context_t spi1_context = {
    .ptr = HPM_SPI1,
    .write_cs = NULL,
    .tx_buff = 0,
    .tx_size = 0,
    .tx_count = 0
    .rx_buff = 0,
    .rx_size = 0,
    .rx_count = 0,
    .data_len_in_byte = 1,
    .per_trans_max = SPI_SOC_TRANSFER_COUNT_MAX,
    .dma_context = {
            .dma_ptr = 0,
            .rx_dma_ch = 0,
            .tx_dma_ch = 0,
            .dmamux_ptr = HPM_DMAMUX,
            .rx_dmamux_ch = 0,
            .tx_dmamux_ch = 0,
            .rx_req = HPM_DMA_SRC_SPI1_RX,
            .tx_req = HPM_DMA_SRC_SPI1_TX,
            .data_width = DMA_TRANSFER_WIDTH_BYTE,
    },
    .running_core = HPM_CORE0,
    .dma_linked_descriptor = spi1_dma_linked_descriptor,
    .spi_transctrl = spi1_transctrl,
};
#endif

static struct hpm_spidev_s g_spi1dev =
{
  .spidev   =
  {
    .ops        = &g_sp1iops,
  },
  .spibase       = HPM_SPI1,
  .spiclock      = clock_spi1,
  .spiirq        = IRQn_SPI1,
#if defined(CONFIG_HPM6750_SPI1_DMA) 
  .spi_context   = &spi1_context,
  .rxsem         = SEM_INITIALIZER(0),
  .txsem         = SEM_INITIALIZER(0),
#endif
  .lock          = NXMUTEX_INITIALIZER,
#ifdef CONFIG_PM
  .pm_cb.prepare = spi_pm_prepare,
#endif
  .config        = SIMPLEX_TX,
};

#endif /* CONFIG_HPM6750_SPI1 */

#ifdef CONFIG_HPM6750_SPI2
static const struct spi_ops_s g_sp2iops =
{
  .lock              = spi_lock,
  .select            = hpm6750_spi2select,
  .setfrequency      = spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay          = spi_setdelay,
#endif
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = hpm6750_spi2status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = hpm6750_spi2cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = spi_trigger,
#endif
  .registercallback  = 0,  /* not implemented */
};

#ifdef CONFIG_HPM6750_SPI2_DMA
/* dma descriptors need align 8 bytes */
ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(8) dma_linked_descriptor_t spi2_dma_linked_descriptor[HPM6750_MAX_SPI_DMA_COUNT * SPI_DMA_DESC_COUNT_PER_TRANS];
ATTR_PLACE_AT_NONCACHEABLE uint32_t spi2_transctrl[HPM6750_MAX_SPI_DMA_COUNT];
spi_context_t spi2_context = {
    .ptr = HPM_SPI2,
    .write_cs = NULL,
    .tx_buff = 0,
    .tx_size = 0,
    .tx_count = 0,
    .rx_buff = 0,
    .rx_size = 0,
    .rx_count = 0,
    .data_len_in_byte = 1,
    .per_trans_max = SPI_SOC_TRANSFER_COUNT_MAX,
    .dma_context = {
            .dma_ptr = 0,
            .rx_dma_ch = 0,
            .tx_dma_ch = 0,
            .dmamux_ptr = HPM_DMAMUX,
            .rx_dmamux_ch = 0,
            .tx_dmamux_ch = 0,
            .rx_req = HPM_DMA_SRC_SPI2_RX,
            .tx_req = HPM_DMA_SRC_SPI2_TX,
            .data_width = DMA_TRANSFER_WIDTH_BYTE,
    },
    .running_core = HPM_CORE0,
    .dma_linked_descriptor = spi2_dma_linked_descriptor,
    .spi_transctrl = spi2_transctrl,
};
#endif

static struct hpm_spidev_s g_spi2dev =
{
  .spidev        =
  {
    .ops         = &g_sp2iops,
  },
  .spibase       = HPM_SPI2,
  .spiclock      = clock_spi2,
  .spiirq        = IRQn_SPI2,
#if defined(CONFIG_HPM6750_SPI2_DMA)
  .spi_context   = &spi2_context,
  .rxsem         = SEM_INITIALIZER(0),
  .txsem         = SEM_INITIALIZER(0),
#endif
  .lock          = NXMUTEX_INITIALIZER,
#ifdef CONFIG_PM
  .pm_cb.prepare = spi_pm_prepare,
#endif
  .config        = SIMPLEX_TX,
};

#endif /* CONFIG_HPM6750_SPI2 */

#ifdef CONFIG_HPM6750_SPI3
static const struct spi_ops_s g_sp3iops =
{
  .lock              = spi_lock,
  .select            = hpm6750_spi3select,
  .setfrequency      = spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  .setdelay          = spi_setdelay,
#endif
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = hpm6750_spi3status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = hpm6750_spi3cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = spi_trigger,
#endif
  .registercallback  = 0,                   /* not implemented */
};

#ifdef CONFIG_HPM6750_SPI2_DMA
/* dma descriptors need align 8 bytes */
ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(8) dma_linked_descriptor_t spi3_dma_linked_descriptor[HPM6750_MAX_SPI_DMA_COUNT * SPI_DMA_DESC_COUNT_PER_TRANS];
ATTR_PLACE_AT_NONCACHEABLE uint32_t spi3_transctrl[HPM6750_MAX_SPI_DMA_COUNT];
spi_context_t spi3_context = {
    .ptr = HPM_SPI3,
    .write_cs = NULL,
    .tx_buff = 0,
    .tx_size = 0,
    .tx_count = 0,
    .rx_buff = 0,
    .rx_size = 0,
    .rx_count = 0,
    .data_len_in_byte = 1,
    .per_trans_max = SPI_SOC_TRANSFER_COUNT_MAX,
    .dma_context = {
            .dma_ptr = 0,
            .rx_dma_ch = 0,
            .tx_dma_ch = 0,
            .dmamux_ptr = HPM_DMAMUX,
            .rx_dmamux_ch = 0,
            .tx_dmamux_ch = 0,
            .rx_req = HPM_DMA_SRC_SPI3_RX,
            .tx_req = HPM_DMA_SRC_SPI3_TX,
            .data_width = DMA_TRANSFER_WIDTH_BYTE,
    },
    .running_core = HPM_CORE0,
    .dma_linked_descriptor = spi3_dma_linked_descriptor,
    .spi_transctrl = spi3_transctrl,
};
#endif

static struct hpm_spidev_s g_spi3dev =
{
  .spidev        =
  {
    .ops         = &g_sp3iops,
  },
  .spibase       = HPM_SPI3,
  .spiclock      = clock_spi3,
  .spiirq        = IRQn_SPI3,
#if defined(CONFIG_HPM6750_SPI3_DMA)
  .spi_context   = &spi3_context,
  .rxsem         = SEM_INITIALIZER(0),
  .txsem         = SEM_INITIALIZER(0),
#endif
  .lock          = NXMUTEX_INITIALIZER,
#ifdef CONFIG_PM
  .pm_cb.prepare = spi_pm_prepare,
#endif
  .config        = SIMPLEX_TX,
};

#endif /* CONFIG_HPM6750_SPI3 */

static void write_spi0_cs(uint32_t pin, uint8_t state)
{
  gpio_write_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(spi_gpio_cfg_table[0].cs.ioc_pad), GPIO_GET_PIN_INDEX(spi_gpio_cfg_table[0].cs.ioc_pad),state);
}

static void write_spi1_cs(uint32_t pin, uint8_t state)
{
  gpio_write_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(spi_gpio_cfg_table[1].cs.ioc_pad), GPIO_GET_PIN_INDEX(spi_gpio_cfg_table[1].cs.ioc_pad),state);
}

static void write_spi2_cs(uint32_t pin, uint8_t state)
{
  gpio_write_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(spi_gpio_cfg_table[2].cs.ioc_pad), GPIO_GET_PIN_INDEX(spi_gpio_cfg_table[2].cs.ioc_pad),state);
}

static void write_spi3_cs(uint32_t pin, uint8_t state)
{
  gpio_write_pin(HPM_GPIO0, GPIO_GET_PORT_INDEX(spi_gpio_cfg_table[3].cs.ioc_pad), GPIO_GET_PIN_INDEX(spi_gpio_cfg_table[3].cs.ioc_pad),state);
}


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_getreg
 *
 * Description:
 *   Get the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 16-bit register
 *
 ****************************************************************************/

static inline uint32_t spi_getreg(struct hpm_spidev_s *priv,
                                  uint32_t offset)
{
  return getreg32(priv->spibase + offset);
}

/****************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *   Write a 16-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 16-bit value to be written
 *
 * Returned Value:
 *   The contents of the 16-bit register
 *
 ****************************************************************************/

static inline void spi_putreg(struct hpm_spidev_s *priv,
                              uint32_t offset, uint32_t value)
{
  putreg32(value, priv->spibase + offset);
}

/****************************************************************************
 * Name: spi_readword
 *
 * Description:
 *   Read one byte from SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   Byte as read
 *
 ****************************************************************************/

static inline uint32_t spi_readword(struct hpm_spidev_s *priv)
{
  /* Can't receive in tx only mode */

  if (priv->config == SIMPLEX_TX)
    {
      return 0;
    }

  /* Then return the received 32 bit word */
  uint32_t data = 0;
  spi_read_data(priv->spibase, spi_get_data_length_in_bytes(priv->spibase), (uint8_t *)&data, 4);
  return data;
}

/****************************************************************************
 * Name: spi_writeword
 *
 * Description:
 *   Write one byte to SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   byte - Byte to send
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_writeword(struct hpm_spidev_s *priv,
                                 uint32_t word)
{
  /* Can't transmit in rx only mode */

  if (priv->config == SIMPLEX_RX)
    {
      return;
    }

  /* Then send the 32 bit word */

  spi_write_data(priv->spibase, spi_get_data_length_in_bytes(priv->spibase), (uint8_t *)&word, 4);
}

/****************************************************************************
 * Name: spi_readbyte
 *
 * Description:
 *   Read one byte from SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   Byte as read
 *
 ****************************************************************************/

static inline uint8_t spi_readbyte(struct hpm_spidev_s *priv)
{
  /* Can't receive in tx only mode */

  if (priv->config == SIMPLEX_TX)
    {
      return 0;
    }

  /* Then return the received byte */

  uint8_t data = 0;
  spi_read_data(priv->spibase, spi_get_data_length_in_bytes(priv->spibase), (uint8_t *)&data, 1);
  return data;
}

/****************************************************************************
 * Name: spi_writebyte
 *
 * Description:
 *   Write one 8-bit frame to the SPI FIFO
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   byte - Byte to send
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_writebyte(struct hpm_spidev_s *priv,
                                 uint8_t byte)
{
  /* Can't transmit in rx only mode */

  if (priv->config == SIMPLEX_RX)
    {
      return;
    }

  /* Then send the byte */

  spi_write_data(priv->spibase, spi_get_data_length_in_bytes(priv->spibase), (uint8_t *)&byte, 1);
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

static int spi_interrupt(int irq, void *context, void *arg)
{
  struct hpm_spidev_s *priv = (struct hpm_spidev_s *)arg;
  volatile uint32_t irq_status;
  uint8_t data_len_in_bytes;
  hpm_stat_t stat;
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
#ifdef CONFIG_HPM6750_SPI_DMA
      priv->txresult = 0x80;
      nxsem_post(&priv->txsem);
#endif
    }

  return 0;
}

/****************************************************************************
 * Name: spi_dmarxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ****************************************************************************/

#if defined(CONFIG_HPM6750_SPI0_DMA) || defined(CONFIG_HPM6750_SPI1_DMA) || \
    defined(CONFIG_HPM6750_SPI2_DMA) || defined(CONFIG_HPM6750_SPI3_DMA)
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

  do
    {
      ret = nxsem_wait_uninterruptible(&priv->rxsem);

      /* The only expected error is ECANCELED which would occur if the
       * calling thread were canceled.
       */

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (ret == OK);

  return ret;
}
#endif

/****************************************************************************
 * Name: spi_dmatxwait
 *
 * Description:
 *   Wait for DMA to complete.
 *
 ****************************************************************************/

#if defined(CONFIG_HPM6750_SPI0_DMA) || defined(CONFIG_HPM6750_SPI1_DMA) || \
    defined(CONFIG_HPM6750_SPI2_DMA) || defined(CONFIG_HPM6750_SPI3_DMA)
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

  do
    {
      ret = nxsem_wait_uninterruptible(&priv->txsem);

      /* The only expected error is ECANCELED which would occur if the
       * calling thread were canceled.
       */

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (ret == OK);

  return ret;
}
#endif

/****************************************************************************
 * Name: spi_dmarx tx complete callback
 *
 * Description:
 *   Signal that DMA is complete
 *
 ****************************************************************************/

#if defined(CONFIG_HPM6750_SPI0_DMA) || defined(CONFIG_HPM6750_SPI1_DMA) || \
    defined(CONFIG_HPM6750_SPI2_DMA) || defined(CONFIG_HPM6750_SPI3_DMA)
static void  spi_rx_dma_channel_callback(DMA_Type *ptr, uint32_t channel, void *user_data, uint32_t int_stat)
{
  struct hpm_spidev_s *priv = (struct hpm_spidev_s *)user_data;
  UNUSED(channel);
  spiinfo("RX interrupt fired with status %u\n", int_stat);
  if (int_stat == DMA_CHANNEL_STATUS_TC)
    {
      nxsem_post(&priv->rxsem);
    }
  else
    {
      spierr("DMA transfer failed for RX.\n");
    }
}

static void  spi_tx_dma_channel_callback(DMA_Type *ptr, uint32_t channel, void *user_data, uint32_t int_stat)
{
  struct hpm_spidev_s *priv = (struct hpm_spidev_s *)user_data;
  UNUSED(channel);
  spiinfo("TX interrupt fired with status %u\n", int_stat);
  if (int_stat == DMA_CHANNEL_STATUS_TC)
    {
      nxsem_post(&priv->txsem);
    }
  else
    {
      spierr("DMA transfer failed for TX.\n");
    }
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
      ret = nxmutex_lock(&priv->lock);
    }
  else
    {
      ret = nxmutex_unlock(&priv->lock);
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
  uint32_t setbits = 0;
  uint32_t actual  = 0;
  spi_timing_config_t timing_config = {0};

  /* Limit to max possible (if STM32_SPI_CLK_MAX is defined in board.h) */

  if (frequency > HPMICRO_SPI_CLK_MAX)
    {
      frequency = HPMICRO_SPI_CLK_MAX;
    }

  /* Has the frequency changed? */

  if (frequency != priv->frequency)
    {
      /* set SPI sclk frequency for master */
      spi_master_get_default_timing_config(&timing_config);
      timing_config.master_config.clk_src_freq_in_hz = clock_get_frequency(priv->spiclock);
      timing_config.master_config.sclk_freq_in_hz = frequency;
      spi_master_timing_init((SPI_Type *)priv->spibase, &timing_config);

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
  uint32_t transfmt = (SPI_Type *)priv->spibase->TRANSFMT;
  if (mode != priv->mode)
    {
      /* set SPI format config for master */
      format_config.common_config.data_len_in_bits  = SPI_TRANSFMT_DATALEN_GET(transfmt);
      format_config.common_config.data_merge        = SPI_TRANSFMT_DATAMERGE_GET(transfmt);
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
  uint32_t setbits = 0;
  uint32_t clrbits = 0;

  spi_format_config_t format_config = {0};

  spiinfo("nbits=%d\n", nbits);

  /* Has the number of bits changed? */
  uint32_t transfmt = (SPI_Type *)priv->spibase->TRANSFMT;
  if (nbits != priv->nbits)
    {
      /* set SPI format config for master */    
      format_config.common_config.data_merge        = SPI_TRANSFMT_DATAMERGE_GET(transfmt);
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
                (uint8_t *)&wd, ARRAY_SIZE(&wd), (uint8_t *)&regval, ARRAY_SIZE(&regval));
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
  hpm_stat_t stat;
  size_t len = nwords ;
  size_t inc_len = 0;
  size_t dummy_len = 0;
  uint32_t regval = 0;
  DEBUGASSERT(priv && priv->spibase);

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* set SPI control config for master */

  spi_master_get_default_control_config(&control_config);
  control_config.master_config.cmd_enable = false;  /* cmd phase control for master */
  control_config.master_config.addr_enable = false; /* address phase control for master */

  if(!txbuffer)
    {
      control_config.common_config.trans_mode = spi_trans_read_only;
    }
  else if(!rxbuffer)
    {
      control_config.common_config.trans_mode = spi_trans_write_only;
    }
  else if(txbuffer && rxbuffer)
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
      dummy_len = (len > 400) ? 400 : len;
      hpm_spi_transfer(priv->spibase,
            &control_config,
            NULL, NULL,
            (uint8_t *)&txbuffer[inc_len], dummy_len,(uint8_t *)&rxbuffer[inc_len], dummy_len);
      len      -= dummy_len;
      inc_len  += dummy_len;
    }
}

#endif /* !CONFIG_HPM6750_SPI_DMA || CONFIG_HPMICRO_DMACAPABLE ||
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
  spi_control_config_t control_config = {0};
  uint8_t cmd = 0x1a;
  uint32_t addr = 0x10;
  hpm_stat_t stat;
  DEBUGASSERT(priv != NULL);

#if defined(CONFIG_HPM6750_SPI0_DMA) || defined(CONFIG_HPM6750_SPI1_DMA) || \
    defined(CONFIG_HPM6750_SPI2_DMA) || defined(CONFIG_HPM6750_SPI3_DMA)
  if ((priv->dma_txchan < 0) || (priv->dma_rxchan < 0))
    {
      spi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
      return;
    }
  else
    { 
        /* set SPI control config for master */
    spi_master_get_default_control_config(&control_config);
    control_config.master_config.cmd_enable     = false;
    control_config.master_config.addr_enable    = false;
    control_config.master_config.addr_phase_fmt = spi_address_phase_format_single_io_mode;   
    control_config.common_config.data_phase_fmt = spi_single_io_mode;
    control_config.common_config.dummy_cnt      = spi_dummy_count_1;

      
    if(!txbuffer)
      {
        control_config.common_config.trans_mode = spi_trans_read_only;
        control_config.common_config.rx_dma_enable  = true;
        priv->spi_context->tx_size                  = 0;
        priv->spi_context->rx_size                  = nwords;
        priv->config = SIMPLEX_RX;
      }
    else if(!rxbuffer)
      {
        control_config.common_config.trans_mode     = spi_trans_write_only;
        control_config.common_config.tx_dma_enable  = true;
        priv->spi_context->tx_size                  = nwords;
        priv->spi_context->rx_size                  = 0;
        priv->config = SIMPLEX_TX;
      }
    else if(txbuffer && rxbuffer)
      {
        control_config.common_config.trans_mode = spi_trans_write_read_together;
        control_config.common_config.tx_dma_enable  = true;
        control_config.common_config.rx_dma_enable  = true;
        priv->spi_context->tx_size                  = nwords;
        priv->spi_context->rx_size                  = nwords;
        priv->config = FULL_DUPLEX;
      }
    else
      {
        return;
      }

      priv->spi_context->cmd              = cmd;
      priv->spi_context->addr             = addr;
      priv->spi_context->data_len_in_byte = spi_get_data_length_in_bytes((SPI_Type *)priv->spibase);

      priv->spi_context->tx_buff          = (uint8_t *)txbuffer;    
      priv->spi_context->tx_count         = priv->spi_context->tx_size / priv->spi_context->data_len_in_byte;

      priv->spi_context->rx_buff          = (uint8_t *)rxbuffer;
      priv->spi_context->rx_count         = priv->spi_context->rx_size / priv->spi_context->data_len_in_byte;
      
      stat = hpm_spi_setup_dma_transfer(priv->spi_context, &control_config);
      if (stat != status_success) 
        {
          return;
        }
      spi_dmarxwait(priv);
      spi_dmatxwait(priv);
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
  hpm_dma_resource_t dma_resource;
  hpm_dmamux_resource_t dmamux_resource;
  spi_setmode((struct spi_dev_s *)priv, SPIDEV_MODE0);
  spi_setbits((struct spi_dev_s *)priv, 8);

  /* Select a default frequency of approx. 400KHz */

  spi_setfrequency((struct spi_dev_s *)priv, 400000);


#if defined(CONFIG_HPM6750_SPI0_DMA) || defined(CONFIG_HPM6750_SPI1_DMA) || \
    defined(CONFIG_HPM6750_SPI2_DMA) || defined(CONFIG_HPM6750_SPI3_DMA)

  priv->dma_rxchan = hpm_dma_channel_request(spi_rx_dma_channel_callback, (void *)priv);
  priv->dma_txchan = hpm_dma_channel_request(spi_tx_dma_channel_callback, (void *)priv); 
  if(priv->dma_rxchan >= 0)
    {
      hpm_dma_channel_get_resource(priv->dma_rxchan, &dma_resource);
      if (hpm_dmamux_channel_request(priv->dma_rxchan, priv->spi_context->dma_context.rx_req) < 0)
        {
          return;
        }
      hpm_dmamux_channel_get_resource(priv->dma_rxchan, &dmamux_resource);
      priv->spi_context->dma_context.dma_ptr      = dma_resource.base;
      priv->spi_context->dma_context.rx_dma_ch    = dma_resource.channel;
      priv->spi_context->dma_context.rx_dmamux_ch = dmamux_resource.dmamux_channel;
      hpm_dma_channel_start(priv->dma_rxchan);
    }
  if(priv->dma_txchan >= 0)
    {
      hpm_dma_channel_get_resource(priv->dma_txchan, &dma_resource);
      if (hpm_dmamux_channel_request(priv->dma_txchan, priv->spi_context->dma_context.tx_req) < 0)
        {
          return;
        }
      hpm_dmamux_channel_get_resource(priv->dma_txchan, &dmamux_resource);
      priv->spi_context->dma_context.dma_ptr      = dma_resource.base;
      priv->spi_context->dma_context.tx_dma_ch    = dma_resource.channel;
      priv->spi_context->dma_context.tx_dmamux_ch = dmamux_resource.dmamux_channel;
      hpm_dma_channel_start(priv->dma_txchan);
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
 * Name: hpm6750_spibus_initialize
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

struct spi_dev_s *hpm6750_spibus_initialize(int bus)
{
  struct hpm_spidev_s *priv = NULL;

  irqstate_t flags = enter_critical_section();

#ifdef CONFIG_HPM6750_SPI0
  if (bus == 0)
    {
      /* Select SPI0 */

      priv = &g_spi0dev;

      /* Only configure if the bus is not already configured */

      if (!priv->initialized)
        {
          /* Configure SPI0 pins: SCK, MISO, and MOSI */

          HPM_IOC->PAD[spi_gpio_cfg_table[bus].cs.ioc_pad].FUNC_CTL   = spi_gpio_cfg_table[bus].cs.ico_func_ctl;
          HPM_IOC->PAD[spi_gpio_cfg_table[bus].mosi.ioc_pad].FUNC_CTL = spi_gpio_cfg_table[bus].mosi.ico_func_ctl;
          HPM_IOC->PAD[spi_gpio_cfg_table[bus].miso.ioc_pad].FUNC_CTL = spi_gpio_cfg_table[bus].miso.ico_func_ctl;
          HPM_IOC->PAD[spi_gpio_cfg_table[bus].sclk.ioc_pad].FUNC_CTL = spi_gpio_cfg_table[bus].sclk.ico_func_ctl | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1);

          /* Set up default configuration: Master, 8-bit, etc. */

#if defined(CONFIG_HPM6750_SPI0_DMA) 
          priv->spi_context->cs_pin   = spi_gpio_cfg_table[bus].cs.ioc_pad;
          priv->spi_context->write_cs = write_spi0_cs;
#endif
          spi_bus_initialize(priv);
          priv->initialized = true;
        }
    }
  else
#endif
#ifdef CONFIG_HPM6750_SPI1
  if (bus == 1)
    {
      /* Select SPI1 */

      priv = &g_spi1dev;

      /* Only configure if the bus is not already configured */

      if (!priv->initialized)
        {
          /* Configure SPI1 pins: SCK, MISO, and MOSI */

          HPM_IOC->PAD[spi_gpio_cfg_table[bus].cs.ioc_pad].FUNC_CTL   = spi_gpio_cfg_table[bus].cs.ico_func_ctl;
          HPM_IOC->PAD[spi_gpio_cfg_table[bus].mosi.ioc_pad].FUNC_CTL = spi_gpio_cfg_table[bus].mosi.ico_func_ctl;
          HPM_IOC->PAD[spi_gpio_cfg_table[bus].miso.ioc_pad].FUNC_CTL = spi_gpio_cfg_table[bus].miso.ico_func_ctl;
          HPM_IOC->PAD[spi_gpio_cfg_table[bus].sclk.ioc_pad].FUNC_CTL = spi_gpio_cfg_table[bus].sclk.ico_func_ctl | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1);

          /* Set up default configuration: Master, 8-bit, etc. */

#if defined(CONFIG_HPM6750_SPI1_DMA) 
          priv->spi_context->cs_pin   = spi_gpio_cfg_table[bus].cs.ioc_pad;
          priv->spi_context->write_cs = write_spi1_cs;
#endif
          spi_bus_initialize(priv);
          priv->initialized = true;
        }
    }
  else
#endif
#ifdef CONFIG_HPM6750_SPI2
  if (bus == 2)
    {
      /* Select SPI2 */

      priv = &g_spi2dev;

      /* Only configure if the bus is not already configured */

      if (!priv->initialized)
        {
          /* Configure SPI2 pins: SCK, MISO, and MOSI */

          HPM_IOC->PAD[spi_gpio_cfg_table[bus].cs.ioc_pad].FUNC_CTL   = spi_gpio_cfg_table[bus].cs.ico_func_ctl;
          HPM_IOC->PAD[spi_gpio_cfg_table[bus].mosi.ioc_pad].FUNC_CTL = spi_gpio_cfg_table[bus].mosi.ico_func_ctl;
          HPM_IOC->PAD[spi_gpio_cfg_table[bus].miso.ioc_pad].FUNC_CTL = spi_gpio_cfg_table[bus].miso.ico_func_ctl;
          HPM_IOC->PAD[spi_gpio_cfg_table[bus].sclk.ioc_pad].FUNC_CTL = spi_gpio_cfg_table[bus].sclk.ico_func_ctl | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1);

          /* Set up default configuration: Master, 8-bit, etc. */
#if defined(CONFIG_HPM6750_SPI2_DMA) 
          priv->spi_context->cs_pin   = spi_gpio_cfg_table[bus].cs.ioc_pad;
          priv->spi_context->write_cs = write_spi2_cs;
#endif
          spi_bus_initialize(priv);
          priv->initialized = true;
        }
    }
  else
#endif
#ifdef CONFIG_HPM6750_SPI3
  if (bus == 3)
    {
      /* Select SPI3 */

      priv = &g_spi3dev;

      /* Only configure if the bus is not already configured */

      if (!priv->initialized)
        {
          /* Configure SPI3 pins: SCK, MISO, and MOSI */

          HPM_IOC->PAD[spi_gpio_cfg_table[bus].cs.ioc_pad].FUNC_CTL   = spi_gpio_cfg_table[bus].cs.ico_func_ctl;
          HPM_IOC->PAD[spi_gpio_cfg_table[bus].mosi.ioc_pad].FUNC_CTL = spi_gpio_cfg_table[bus].mosi.ico_func_ctl;
          HPM_IOC->PAD[spi_gpio_cfg_table[bus].miso.ioc_pad].FUNC_CTL = spi_gpio_cfg_table[bus].miso.ico_func_ctl;
          HPM_IOC->PAD[spi_gpio_cfg_table[bus].sclk.ioc_pad].FUNC_CTL = spi_gpio_cfg_table[bus].sclk.ico_func_ctl | IOC_PAD_FUNC_CTL_LOOP_BACK_SET(1);

          /* Set up default configuration: Master, 8-bit, etc. */

#if defined(CONFIG_HPM6750_SPI3_DMA) 
          priv->spi_context->cs_pin   = spi_gpio_cfg_table[bus].cs.ioc_pad;
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

