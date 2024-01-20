/****************************************************************************
 * arch/risc-v/src/hpmicro/hpm_sdmmc.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/compiler.h>
#include <nuttx/sdio.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/mmcsd.h>
#include <nuttx/irq.h>
#include <nuttx/cache.h>
#include <arch/board/board.h>

#include "chip.h"
#include "board.h"
#include "hpm_sdxc_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_l1c_drv.h"
#include "hpm_misc.h"

#define HPM_SDMMC_CLK_INIT_FREQ (37500UL)
#define HPM_SDMMC_CLK_NORMAL_FREQ (25000000UL)
#define HPM_SDMMC_CLK_HIGH_FREQ (50000000UL)

#define HPM_SDMMC_CMDTIMEOUT (100000)
#define HPM_SDMMC_LONGTIMEOUT (0x7fffffff)

#define HPM_SDMMC_DMA_MODE_NONE (0)
#define HPM_SDMMC_DMA_MODE_ADMA2 (2)

#ifdef CONFIG_HPM_SDXC_DRV

typedef struct
{
    sdxc_adma2_descriptor_t adma_desc;
} hpm_sdmmc_noncacheable_ctx_t;

struct hpm_sdmmc_dev_s
{
    struct sdio_dev_s dev;
    SDXC_Type *base;
    int nirq;
    clock_name_t clock_name;

    uint32_t blocksize;
    uint32_t blockcnt;

    sdxc_command_t cmd;
    sdxc_adma_config_t adma_cfg;
    hpm_sdmmc_noncacheable_ctx_t *nc_ctx;
    uint32_t dma_mode;

    /* Event support */

    sem_t waitsem;                         /* Implements event waiting */
    sdio_eventset_t waitevents;            /* Set of events to be waited for */
    uint32_t waitmask;                     /* Interrupt enables for event waiting */
    volatile sdio_eventset_t wakeupevents; /* The event that caused the wakeup */
    struct wdog_s waitwdog;                /* Watchdog that handles event timeouts */

    /* Callback support */

    sdio_statset_t cdstatus;  /* Card status */
    sdio_eventset_t cbevents; /* Set of events to be cause callbacks */
    worker_t callback;        /* Registered callback function */
    void *cbarg;              /* Registered callback argument */
    struct work_s cbwork;     /* Callback work queue structure */

    /* Interrupt mode data transfer support */

    uint32_t *buffer; /* Address of current R/W buffer */
    size_t remaining; /* Number of bytes remaining in the transfer */
    uint32_t xfrmask; /* Interrupt enables for data transfer */

#ifdef CONFIG_HPM_SDXC_DRV
    /* Interrupt at SDIO_D1 pin, only for SDIO cards */

    uint32_t sdiointmask;        /* STM32 SDIO register mask */
    int (*do_sdio_card)(void *); /* SDIO card ISR */
    void *do_sdio_arg;           /* arg for SDIO card ISR */
#endif

    /* Fixed transfer block size support */

#ifdef CONFIG_SDIO_BLOCKSETUP
    uint8_t block_size;
#endif

    /* DMA data transfer support */

    uint32_t bus_width; /* Required for DMA support */
};

#ifdef CONFIG_SDIO_MUXBUS
static int hpm_sdmmc_lock(FAR struct sdio_dev_s *dev, bool lock);
#endif /* CONFIG_SDIO_MUXBUS */

/* Initialization /setup */
static void hpm_sdmmc_reset(FAR struct sdio_dev_s *dev);
static sdio_capset_t hpm_sdmmc_capabilities(FAR struct sdio_dev_s *dev);
static sdio_capset_t hpm_sdmmc_status(FAR struct sdio_dev_s *dev);
static void hpm_sdmmc_widebus(FAR struct sdio_dev_s *dev, bool wide);
static void hpm_sdmmc_clock(FAR struct sdio_dev_s *dev, enum sdio_clock_e rate);
static int hpm_sdmmc_attach(FAR struct sdio_dev_s *dev);

/* Command/Status/Data Transfer */
static int hpm_sdmmc_sendcmd(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t arg);

#ifdef CONFIG_SDIO_BLOCKSETUP
static void hpm_sdmmc_blocksetup(FAR struct sdio_dev_s *dev, unsigned int blocksize, unsigned int nblocks);
#endif
static int hpm_sdmmc_recvsetup(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer, size_t nbytes);
static int hpm_sdmmc_sendsetup(FAR struct sdio_dev_s *dev, FAR const uint8_t *buffer, size_t nbytes);
static int hpm_sdmmc_cancel(FAR struct sdio_dev_s *dev);
static int hpm_sdmmc_waitresponse(FAR struct sdio_dev_s *dev, uint32_t cmd);
static int hpm_sdmmc_recv_r1(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *R1);
static int hpm_sdmmc_recv_r2(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t R2[4]);
static int hpm_sdmmc_recv_r3(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *R3);
static int hpm_sdmmc_recv_r4(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *R4);
static int hpm_sdmmc_recv_r5(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *R5);
static int hpm_sdmmc_recv_r6(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *R6);
static int hpm_sdmmc_recv_r7(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *R7);

/* Event/Callback support */
static void hpm_sdmmc_waitenable(FAR struct sdio_dev_s *dev, sdio_eventset_t eventset, uint32_t timeout);
static sdio_eventset_t hpm_sdmmc_eventwait(FAR struct sdio_dev_s *dev);
static void hpm_sdmmc_callbackenable(FAR struct sdio_dev_s *dev, sdio_eventset_t eventset);

static int hpm_sdmmc_interrupt(int irq, void *context, void *arg);

#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_SCHED_HPWORK)
static int hpm_sdmmc_registercallback(FAR struct sdio_dev_s *dev, worker_t callback, void *arg);
#endif

#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
static int hpm_sdmmc_dmapreflight(FAR struct sdio_dev_s *dev, FAR const uint8_t *buffer, size_t buflen);
#endif
static int hpm_sdmmc_dmarecvsetup(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer, size_t buflen);
static int hpm_sdmmc_dmasendsetup(FAR struct sdio_dev_s *dev, FAR const uint8_t *buffer, size_t buflen);

static hpm_stat_t hpm_sdmmc_receive_response(SDXC_Type *base, sdxc_command_t *cmd);

static void hpm_sdmmc_sendfifo(struct hpm_sdmmc_dev_s *priv);
static void hpm_sdmmc_recvfifo(struct hpm_sdmmc_dev_s *priv);

static void hpm_sdmmc_config_wait_ints(struct hpm_sdmmc_dev_s *priv, uint32_t waitmask,
                                       sdio_eventset_t waitevents,
                                       sdio_eventset_t wakeupevents);

static void hpm_sdmmc_endwait(struct hpm_sdmmc_dev_s *priv, sdio_eventset_t wakeupevents);

static void hpm_sdmmc_callback(void *arg);

static void hpm_sdmmc_endxfer(struct hpm_sdmmc_dev_s *priv, sdio_eventset_t wakeupevents);

static void hpm_sdmmc_config_xfer_ints(struct hpm_sdmmc_dev_s *priv, uint32_t xfrmask);

static void hpm_sdmmc_event_timeout(wdparm_t arg);

#if defined(CONFIG_HPM_SDXC0)
ATTR_PLACE_AT_NONCACHEABLE hpm_sdmmc_noncacheable_ctx_t sdxc0_nc_ctx;
struct hpm_sdmmc_dev_s hpm_sdxc0_dev_s = {
    .dev =
        {
#if defined(CONFIG_SDIO_MUXBUS)
            .lock = hpm_sdmmc_lock,
#endif
            .reset = hpm_sdmmc_reset,
            .capabilities = hpm_sdmmc_capabilities,
            .status = hpm_sdmmc_status,
            .widebus = hpm_sdmmc_widebus,
            .clock = hpm_sdmmc_clock,
            .attach = hpm_sdmmc_attach,
            .sendcmd = hpm_sdmmc_sendcmd,
#if defined(CONFIG_SDIO_BLOCKSETUP)
            .blocksetup = hpm_sdmmc_blocksetup,
#endif
            .recvsetup = hpm_sdmmc_recvsetup,
            .sendsetup = hpm_sdmmc_sendsetup,
            .cancel = hpm_sdmmc_cancel,
            .waitresponse = hpm_sdmmc_waitresponse,
            .recv_r1 = hpm_sdmmc_recv_r1,
            .recv_r2 = hpm_sdmmc_recv_r2,
            .recv_r3 = hpm_sdmmc_recv_r3,
            .recv_r4 = hpm_sdmmc_recv_r4,
            .recv_r5 = hpm_sdmmc_recv_r5,
            .recv_r6 = hpm_sdmmc_recv_r6,
            .recv_r7 = hpm_sdmmc_recv_r7,
            .waitenable = hpm_sdmmc_waitenable,
            .eventwait = hpm_sdmmc_eventwait,
            .callbackenable = hpm_sdmmc_callbackenable,
#if defined(CONFIG_SCHED_WORKQUEUE)
            .registercallback = hpm_sdmmc_registercallback,
#endif
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
            .dmapreflight = hpm_sdmmc_dmapreflight,
#endif
            .dmarecvsetup = hpm_sdmmc_dmarecvsetup,
            .dmasendsetup = hpm_sdmmc_dmasendsetup,
        },
    .base = HPM_SDXC0,
    .clock_name = clock_sdxc0,
    .nirq = HPM_IRQn_SDXC0,
    .nc_ctx = &sdxc0_nc_ctx,
    .waitsem = SEM_INITIALIZER(0),
    .dma_mode = HPM_SDMMC_DMA_MODE_NONE,
    .bus_width = 4,
};
#endif

#if defined(CONFIG_HPM_SDXC1)
ATTR_PLACE_AT_NONCACHEABLE hpm_sdmmc_noncacheable_ctx_t sdxc1_nc_ctx;
struct hpm_sdmmc_dev_s hpm_sdxc1_dev_s = {
    .dev =
        {
#if defined(CONFIG_SDIO_MUXBUS)
            .lock = hpm_sdmmc_lock,
#endif
            .reset = hpm_sdmmc_reset,
            .capabilities = hpm_sdmmc_capabilities,
            .status = hpm_sdmmc_status,
            .widebus = hpm_sdmmc_widebus,
            .clock = hpm_sdmmc_clock,
            .attach = hpm_sdmmc_attach,
            .sendcmd = hpm_sdmmc_sendcmd,
#if defined(CONFIG_SDIO_BLOCKSETUP)
            .blocksetup = hpm_sdmmc_blocksetup,
#endif
            .recvsetup = hpm_sdmmc_recvsetup,
            .sendsetup = hpm_sdmmc_sendsetup,
            .cancel = hpm_sdmmc_cancel,
            .waitresponse = hpm_sdmmc_waitresponse,
            .recv_r1 = hpm_sdmmc_recv_r1,
            .recv_r2 = hpm_sdmmc_recv_r2,
            .recv_r3 = hpm_sdmmc_recv_r3,
            .recv_r4 = hpm_sdmmc_recv_r4,
            .recv_r5 = hpm_sdmmc_recv_r5,
            .recv_r6 = hpm_sdmmc_recv_r6,
            .recv_r7 = hpm_sdmmc_recv_r7,
            .waitenable = hpm_sdmmc_waitenable,
            .eventwait = hpm_sdmmc_eventwait,
            .callbackenable = hpm_sdmmc_callbackenable,
#if defined(CONFIG_SCHED_WORKQUEUE)
            .registercallback = hpm_sdmmc_registercallback,
#endif
#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
            .dmapreflight = hpm_sdmmc_dmapreflight,
#endif
            .dmarecvsetup = hpm_sdmmc_dmarecvsetup,
            .dmasendsetup = hpm_sdmmc_dmasendsetup,
        },
    .base = HPM_SDXC1,
    .clock_name = clock_sdxc1,
    .nirq = HPM_IRQn_SDXC1,
    .nc_ctx = &sdxc1_nc_ctx,
    .waitsem = SEM_INITIALIZER(0),
    .dma_mode = HPM_SDMMC_DMA_MODE_NONE,
    .bus_width = 4,
};
#endif

static void hpm_sdmmc_config_wait_ints(struct hpm_sdmmc_dev_s *priv, uint32_t waitmask,
                                       sdio_eventset_t waitevents,
                                       sdio_eventset_t wakeupevents)
{
    irqstate_t flags;

    priv->waitevents = waitevents;
    priv->wakeupevents = wakeupevents;
    priv->waitmask = waitmask;

    flags = enter_critical_section();

    sdxc_enable_interrupt_signal(priv->base, priv->waitmask, true);

    leave_critical_section(flags);
}

static void hpm_sdmmc_endwait(struct hpm_sdmmc_dev_s *priv, sdio_eventset_t wakeupevents)
{
    /* Cancel the watchdog timeout */
    wd_cancel(&priv->waitwdog);

    /* Disable event-related interrupts */
    hpm_sdmmc_config_wait_ints(priv, 0, 0, wakeupevents);

    /* Wake up the waiting thread */
    nxsem_post(&priv->waitsem);
}

static void hpm_sdmmc_endxfer(struct hpm_sdmmc_dev_s *priv, sdio_eventset_t wakeupevents)
{
    /* Disable all transfer related interrupts */
    sdxc_enable_interrupt_signal(priv->base, ~0U, false);

    if ((wakeupevents & ~SDIOWAIT_TRANSFERDONE) != 0)
    {
        /* FIXME */
    }
    /* Clear pending interrupts */
    sdxc_clear_interrupt_status(priv->base, SDXC_STS_ALL_FLAGS);

    /* Mark the transfer as finished */
    priv->remaining = 0;

    priv->dma_mode = HPM_SDMMC_DMA_MODE_NONE;

    if ((priv->waitevents & wakeupevents) != 0)
    {
        hpm_sdmmc_endwait(priv, wakeupevents);
    }
}

static void hpm_sdmmc_config_xfer_ints(struct hpm_sdmmc_dev_s *priv, uint32_t xfrmask)
{
    irqstate_t flags;

    flags = enter_critical_section();
    priv->xfrmask = xfrmask;

    sdxc_enable_interrupt_signal(priv->base, ~0U, false);
    sdxc_enable_interrupt_signal(priv->base, priv->xfrmask | priv->waitmask | priv->sdiointmask, true);

    leave_critical_section(flags);
}

static void hpm_sdmmc_event_timeout(wdparm_t arg)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)arg;

    /* There is always race conditions with timer expirations. */

    DEBUGASSERT((priv->waitevents & SDIOWAIT_TIMEOUT) != 0 ||
                priv->wakeupevents != 0);

    mcinfo("sta: %08" PRIx32 " enabled irq: %08" PRIx32 "\n",
           priv->base->INT_STAT,
           priv->base->INT_SIGNAL_EN);

    /* Is a data transfer complete event expected? */

    if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
        /* Yes.. wake up any waiting threads */

#ifdef CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE
        hpm_sdmmc_endwait(priv, SDIOWAIT_TIMEOUT |
                                    (priv->waitevents & SDIOWAIT_WRCOMPLETE));
#else
        hpm_sdmmc_endwait(priv, SDIOWAIT_TIMEOUT);
#endif
        mcerr("Timeout: remaining: %zu\n", priv->remaining);
    }
}

/* Initialization/ setup */
static void hpm_sdmmc_reset(FAR struct sdio_dev_s *dev)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;
    irqstate_t flags;

    flags = enter_critical_section();
    clock_add_to_group(priv->clock_name, 0);
    sdxc_config_t sdxc_config;
    sdxc_config.data_timeout = 0xf;
    sdxc_init(priv->base, &sdxc_config);
    leave_critical_section(flags);
}

/***************************************************************************
 * Name: hpm_sdmmc_capabilities
 *
 * Descriptions:
 *  Get capabilities (and limitations) of the SDIO driver (optional)
 *
 * Input Parameters:
 *  dev - Device-specific state data
 *
 * Returned Value:
 *  Returned a bitset of status values (see SDIO_CAPS_* defines)
 *
 ****************************************************************************/
static sdio_capset_t hpm_sdmmc_capabilities(FAR struct sdio_dev_s *dev)
{
    sdio_capset_t caps = 0;

    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;

    caps |= SDIO_CAPS_DMABEFOREWRITE | SDIO_CAPS_DMASUPPORTED;

    if (priv->bus_width == 4)
    {
        caps |= SDIO_CAPS_4BIT;
    }
    if (priv->bus_width == 8)
    {
        caps |= SDIO_CAPS_4BIT | SDIO_CAPS_8BIT;
    }

    return caps;
}

/**************************************************************************
 * Name: hpm_sdmmc_status
 *
 * Description:
 *  Get SDIO status
 *
 * Input Parameters:
 *  dev - Device-specific state data
 *
 * Returned Value:
 *  Returns a bitset of status values
 *
 ****************************************************************************/
static sdio_capset_t hpm_sdmmc_status(FAR struct sdio_dev_s *dev)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;

    return priv->cdstatus;
}

/**********************************************************************
 *
 *
 *
 *
 **/
static void hpm_sdmmc_widebus(FAR struct sdio_dev_s *dev, bool wide)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;
    sdxc_bus_width_t bus_width = wide ? sdxc_bus_width_4bit : sdxc_bus_width_1bit;
    sdxc_set_data_bus_width(priv->base, bus_width);
}

static void hpm_sdmmc_clock(FAR struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;

    bool need_disable = false;
    bool clock_freq = 0;
    switch (rate)
    {
    default:
    case CLOCK_SDIO_DISABLED:
        need_disable = true;
        break;
    case CLOCK_IDMODE:
        clock_freq = HPM_SDMMC_CLK_INIT_FREQ;
        break;
    case CLOCK_MMC_TRANSFER:
        clock_freq = HPM_SDMMC_CLK_HIGH_FREQ;
        break;
    case CLOCK_SD_TRANSFER_4BIT:
        clock_freq = HPM_SDMMC_CLK_HIGH_FREQ;
        break;
    case CLOCK_SD_TRANSFER_1BIT:
        clock_freq = HPM_SDMMC_CLK_NORMAL_FREQ;
        break;
    }
    if (need_disable)
    {
        clock_remove_from_group(priv->clock_name, 0);
    }
    else
    {
        clock_add_to_group(priv->clock_name, 0);
        board_sd_configure_clock(priv->base, clock_freq, true);
    }
}

static void hpm_sdmmc_sendfifo(struct hpm_sdmmc_dev_s *priv)
{
    union
    {
        uint32_t w;
        uint8_t b[4];
    } data;
    if ((sdxc_get_present_status(priv->base) & SDXC_PSTATE_BUF_WR_ENABLE_MASK) != 0)
    {
        while (priv->remaining > 0)
        {
            if (priv->remaining >= sizeof(uint32_t))
            {
                data.w = *priv->buffer++;
                priv->remaining -= sizeof(uint32_t);
            }
            else
            {
                uint32_t *ptr = (uint32_t *)priv->buffer;
                data.w = 0;
                for (uint32_t i = 0; i < priv->remaining; i++)
                {
                    data.b[i] = *ptr++;
                }
                priv->remaining = 0;
            }
            /* Put the word into the FIFO*/
            sdxc_write_data(priv->base, data.w);
        }
    }
}

static void hpm_sdmmc_recvfifo(struct hpm_sdmmc_dev_s *priv)
{
    if ((sdxc_get_present_status(priv->base) & SDXC_PSTATE_BUF_RD_ENABLE_MASK) != 0)
    {
        while (priv->remaining > 0)
        {
            *priv->buffer++ = sdxc_read_data(priv->base);
            if (priv->remaining >= sizeof(uint32_t))
            {
                priv->remaining -= sizeof(uint32_t);
            }
            else
            {
                priv->remaining = 0;
            }
        }
    }
}

static int hpm_sdmmc_interrupt(int irq, void *context, void *arg)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)arg;
    uint32_t mask;
    do {
        mask = sdxc_get_interrupt_status(priv->base) & ~SDXC_INT_STAT_CARD_INTERRUPT_MASK;
        if ((mask & SDXC_INT_STAT_BUF_RD_READY_MASK) != 0U)
        {
            hpm_sdmmc_recvfifo(priv);
            if (priv->remaining < 1)
            {
                hpm_sdmmc_endxfer(priv, SDIOWAIT_TRANSFERDONE);
                break;
            }
        }
        if ((mask & SDXC_INT_STAT_BUF_WR_READY_MASK) != 0U)
        {
            hpm_sdmmc_sendfifo(priv);
            if (priv->remaining < 1)
            {
                hpm_sdmmc_endxfer(priv, SDIOWAIT_TRANSFERDONE);
                break;
            }
        }

        if ((mask & SDXC_INT_STAT_DMA_INTERRUPT_MASK) != 0U)
        {
            hpm_sdmmc_endxfer(priv, SDIOWAIT_TRANSFERDONE);
            break;
        }
        if ((mask & SDXC_STS_ERROR) != 0)
        {
            priv->remaining = 0;
            hpm_sdmmc_endxfer(priv, SDIOWAIT_ERROR);
        }

        sdxc_clear_interrupt_status(priv->base, mask);
    } while(mask != 0);

    return 0;
}

static int hpm_sdmmc_attach(FAR struct sdio_dev_s *dev)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;
    DEBUGASSERT(priv != NULL);

    int ret;

    ret = irq_attach(priv->nirq, hpm_sdmmc_interrupt, priv);
    if (ret == OK)
    {
        /* Disable all interrupts and clear interrupt flags */
        sdxc_enable_interrupt_signal(priv->base, SDXC_STS_ALL_FLAGS, false);
        sdxc_clear_interrupt_status(priv->base, SDXC_STS_ALL_FLAGS);
        sdxc_enable_interrupt_signal(priv->base, SDXC_INT_STAT_CARD_INSERTION_MASK, true);
        /* Enable SDXC interrupt */
        up_enable_irq(priv->nirq);
        intc_m_enable_irq_with_priority(priv->nirq, 1);
    }
    return ret;
}

/* Command/Status/Data Transfer */
static int hpm_sdmmc_sendcmd(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t arg)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;

    sdxc_command_t *sdxc_cmd = &priv->cmd;
    SDXC_Type *base = priv->base;
    (void)memset(sdxc_cmd, 0, sizeof(sdxc_command_t));

    sdxc_cmd->cmd_index = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
    sdxc_cmd->cmd_argument = arg;

    switch (cmd & MMCSD_RESPONSE_MASK)
    {
    default:
    case MMCSD_NO_RESPONSE:
        sdxc_cmd->resp_type = sdxc_dev_resp_none;
        break;
    case MMCSD_R1_RESPONSE:
        sdxc_cmd->resp_type = sdxc_dev_resp_r1;
        break;
    case MMCSD_R1B_RESPONSE:
        sdxc_cmd->resp_type = sdxc_dev_resp_r1b;
        break;
    case MMCSD_R2_RESPONSE:
        sdxc_cmd->resp_type = sdxc_dev_resp_r2;
        break;
    case MMCSD_R3_RESPONSE:
        sdxc_cmd->resp_type = sdxc_dev_resp_r3;
        break;
    case MMCSD_R4_RESPONSE:
        sdxc_cmd->resp_type = sdxc_dev_resp_r4;
        break;
    case MMCSD_R5_RESPONSE:
        sdxc_cmd->resp_type = sdxc_dev_resp_r5;
        break;
    case MMCSD_R6_RESPONSE:
        sdxc_cmd->resp_type = sdxc_dev_resp_r6;
        break;
    case MMCSD_R7_RESPONSE:
        sdxc_cmd->resp_type = sdxc_dev_resp_r7;
        break;
    }

    switch (cmd & MMCSD_DATAXFR_MASK)
    {
    case MMCSD_RDDATAXFR:
        sdxc_cmd->cmd_flags |= SDXC_CMD_XFER_DATA_XFER_DIR_MASK | SDXC_CMD_XFER_DATA_PRESENT_SEL_MASK;

        break;
    case MMCSD_WRDATAXFR:
        sdxc_cmd->cmd_flags |= SDXC_CMD_XFER_DATA_PRESENT_SEL_MASK;
        break;
    case MMCSD_RDSTREAM:
        sdxc_cmd->cmd_flags |= SDXC_CMD_XFER_DATA_XFER_DIR_MASK | SDXC_CMD_XFER_DATA_PRESENT_SEL_MASK | SDXC_CMD_XFER_MULTI_BLK_SEL_MASK;
        break;
    case MMCSD_WRSTREAM:
        sdxc_cmd->cmd_flags |= SDXC_CMD_XFER_DATA_PRESENT_SEL_MASK | SDXC_CMD_XFER_MULTI_BLK_SEL_MASK;
        break;
    case MMCSD_NODATAXFR:
    default:
        break;
    }

    if ((sdxc_cmd->cmd_flags & SDXC_CMD_XFER_DATA_PRESENT_SEL_MASK) != 0U)
    {
        if (priv->dma_mode == HPM_SDMMC_DMA_MODE_ADMA2)
        {
            sdxc_cmd->cmd_flags |= SDXC_CMD_XFER_DMA_ENABLE_MASK;
            base->PROT_CTRL = (base->PROT_CTRL & ~SDXC_PROT_CTRL_DMA_SEL_MASK) | SDXC_PROT_CTRL_DMA_SEL_SET(priv->adma_cfg.dma_type);
            base->ADMA_SYS_ADDR = (uint32_t)priv->adma_cfg.adma_table;
        }
    }

    (void)sdxc_send_command(priv->base, sdxc_cmd);

    return OK;
}

static void hpm_sdmmc_blocksetup(FAR struct sdio_dev_s *dev, unsigned int blocksize, unsigned int nblocks)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;

    priv->blocksize = blocksize;
    priv->base->BLK_ATTR = blocksize;
    priv->base->SDMASA = nblocks;
}

static int hpm_sdmmc_recvsetup(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer, size_t nbytes)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;

    DEBUGASSERT((priv != NULL) && (buffer != NULL) && (nbytes > 0));
    DEBUGASSERT((uint32_t)buffer % 4 != 0);

    priv->buffer = (uint32_t *)buffer;
    priv->remaining = nbytes;

    hpm_sdmmc_config_xfer_ints(priv, SDXC_INT_STAT_BUF_RD_READY_MASK);

    return OK;
}

static int hpm_sdmmc_sendsetup(FAR struct sdio_dev_s *dev, FAR const uint8_t *buffer, size_t nbytes)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;

    DEBUGASSERT((priv != NULL) && (buffer != NULL) && (nbytes > 0));
    DEBUGASSERT((uint32_t)buffer % 4 != 0);

    priv->buffer = (uint32_t *)buffer;
    priv->remaining = nbytes;

    sdxc_enable_interrupt_signal(priv->base, SDXC_INT_STAT_BUF_WR_READY_MASK, true);

    return OK;
}

static int hpm_sdmmc_cancel(FAR struct sdio_dev_s *dev)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;

    /* Cancel any watchdog timeout */

    wd_cancel(&priv->waitwdog);

    /* Mark no transfer in progress */

    priv->remaining = 0;
    return OK;
}

static int hpm_sdmmc_waitresponse(FAR struct sdio_dev_s *dev, uint32_t cmd)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;

    int32_t timeout;
    uint32_t events = SDXC_INT_STAT_CMD_COMPLETE_MASK;
    switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
        break;
    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R2_RESPONSE:
    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
        timeout = HPM_SDMMC_LONGTIMEOUT;
        break;
    case MMCSD_R3_RESPONSE:
        timeout = HPM_SDMMC_CMDTIMEOUT;
    case MMCSD_R7_RESPONSE:
        break;
    }

    while ((sdxc_get_interrupt_status(priv->base) & events) == 0)
    {
        if (--timeout <= 0)
        {
            return -ETIMEDOUT;
        }
    }

    return OK;
}

static hpm_stat_t hpm_sdmmc_receive_response(SDXC_Type *base, sdxc_command_t *cmd)
{
    hpm_stat_t status = sdxc_parse_interrupt_status(base);
    if (status == status_success)
    {
        sdxc_command_t *sdxc_cmd = cmd;
        sdxc_clear_interrupt_status(base, SDXC_INT_STAT_CMD_COMPLETE_MASK);
        status = sdxc_receive_cmd_response(base, sdxc_cmd);
    }
    return status;
}

static int hpm_sdmmc_recv_r1(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *R1)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;
    if (hpm_sdmmc_receive_response(priv->base, &priv->cmd) == status_success)
    {
        *R1 = priv->cmd.response[0];
        return OK;
    }
    return -ECANCELED;
}

static int hpm_sdmmc_recv_r2(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t R2[4])
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;
    if (hpm_sdmmc_receive_response(priv->base, &priv->cmd) == status_success)
    {
        for (uint32_t i = 0; i < 4; i++)
        {
            R2[i] = priv->cmd.response[3 - i];
        }
        return OK;
    }
    return -ECANCELED;
}

static int hpm_sdmmc_recv_r3(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *R3)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;
    if (hpm_sdmmc_receive_response(priv->base, &priv->cmd) == status_success)
    {
        *R3 = priv->cmd.response[0];
        return OK;
    }
    return -ECANCELED;
}

static int hpm_sdmmc_recv_r4(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *R4)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;
    if (hpm_sdmmc_receive_response(priv->base, &priv->cmd) == status_success)
    {
        *R4 = priv->cmd.response[0];
        return OK;
    }
    return -ECANCELED;
}

static int hpm_sdmmc_recv_r5(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *R5)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;
    if (hpm_sdmmc_receive_response(priv->base, &priv->cmd) == status_success)
    {
        *R5 = priv->cmd.response[0];
        return OK;
    }
    return -ECANCELED;
}

static int hpm_sdmmc_recv_r6(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *R6)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;
    if (hpm_sdmmc_receive_response(priv->base, &priv->cmd) == status_success)
    {
        *R6 = priv->cmd.response[0];
        return OK;
    }
    return -ECANCELED;
}

static int hpm_sdmmc_recv_r7(FAR struct sdio_dev_s *dev, uint32_t cmd, uint32_t *R7)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;
    if (hpm_sdmmc_receive_response(priv->base, &priv->cmd) == status_success)
    {
        *R7 = priv->cmd.response[0];
        return OK;
    }
    return -ECANCELED;
}

/* Event/Callback support */
static void hpm_sdmmc_waitenable(FAR struct sdio_dev_s *dev, sdio_eventset_t eventset, uint32_t timeout)
{
    /*FIXME*/
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;
    uint32_t waitmask = 0;
    DEBUGASSERT(priv != NULL);

    /* Disable event-related interrupts */
    hpm_sdmmc_config_wait_ints(priv, 0, 0, 0);

#if defined(CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE)
    if (eventset & SDIOWAIT_WRCOMPLETE) != 0)
        {
            if ((sdxc_get_data3_0_level(priv->base) & (1UL << 0)) != 0)
            {
                event &= ~(SDIOWAIT_TIMEOUT | SDIOWAIT_WRCOMPLETE);
            }
        }
    else
#endif
    {
        if ((eventset & SDIOWAIT_CMDDONE) != 0)
        {
            waitmask |= SDXC_INT_STAT_CMD_COMPLETE_MASK;
        }
        if ((eventset & SDIOWAIT_RESPONSEDONE) != 0)
        {
            waitmask |= SDXC_INT_STAT_CMD_COMPLETE_MASK;
        }
        if ((eventset & SDIOWAIT_TRANSFERDONE) != 0)
        {
            waitmask |= SDXC_INT_STAT_XFER_COMPLETE_MASK;
        }
        sdxc_enable_interrupt_signal(priv->base, waitmask, true);
    }
    hpm_sdmmc_config_wait_ints(priv, waitmask, eventset, true);

    /* Check if the timeout event is specified in the event set */

    if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
        int delay;
        int ret;

        /* Yes.. Handle a cornercase: The user request a timeout event but
         * with timeout == 0?
         */

        if (!timeout)
        {
            priv->wakeupevents = SDIOWAIT_TIMEOUT;
            return;
        }

        /* Start the watchdog timer */
        delay = MSEC2TICK(timeout);
        ret = wd_start(&priv->waitwdog, delay,
                       hpm_sdmmc_event_timeout, (wdparm_t)priv);
        if (ret < OK)
        {
            mcerr("ERROR: wd_start failed: %d\n", ret);
        }
    }
}

static sdio_eventset_t hpm_sdmmc_eventwait(FAR struct sdio_dev_s *dev)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;
    DEBUGASSERT(priv != NULL);

    sdio_eventset_t wakeupevents = 0;

#if 1
    irqstate_t flags;
    int ret;

    /* There is a race condition here... the event may have completed before
     * we get here.  In this case waitevents will be zero, but wakeupevents will
     * be non-zero (and, hopefully, the semaphore count will also be non-zero.
     */

    flags = enter_critical_section();
#if defined(CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE)
    /* A card ejected while in SDIOWAIT_WRCOMPLETE can lead to a
     * condition where there is no waitevents set and no wakeupevents
     */

    if (priv->waitevents == 0 && priv->wakeupevents == 0)
    {
        wakeupevents = SDIOWAIT_ERROR;
        goto errout_with_waitints;
    }

#else
    DEBUGASSERT(priv->waitevents != 0 || priv->wakeupevents != 0);
#endif

#if defined(CONFIG_MMCSD_SDIOWAIT_WRCOMPLETE)
    /* FIXME */
#endif

    /* Loop until the event (or the timeout occurs). Race conditions are
     * avoided by calling hpm_sdmmc_waitenable prior to triggering the logic that
     * will cause the wait to terminate.  Under certain race conditions, the
     * waited-for may have already occurred before this function was called!
     */

    for (;;)
    {
        /* Wait for an event in event set to occur.  If this the event has
         * already occurred, then the semaphore will already have been
         * incremented and there will be no wait.
         */

        ret = nxsem_wait_uninterruptible(&priv->waitsem);
        if (ret < 0)
        {
            /* Task canceled.  Cancel the wdog (assuming it was started) and
             * return an SDIO error.
             */

            wd_cancel(&priv->waitwdog);
            wakeupevents = SDIOWAIT_ERROR;
            goto errout_with_waitints;
        }

        wakeupevents = priv->wakeupevents;

        /* Check if the event has occurred.  When the event has occurred, then
         * evenset will be set to 0 and wakeupevents will be set to a nonzero
         * value.
         */

        if (wakeupevents != 0)
        {
            /* Yes... break out of the loop with wakeupevents non-zero */

            break;
        }
    }

    /* Disable event-related interrupts */

errout_with_waitints:
    leave_critical_section(flags);
#endif
    return wakeupevents;
}

static void hpm_sdmmc_callback(void *arg)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)arg;
    DEBUGASSERT(priv != NULL);

    mcinfo("Callback %p(%p) cbevents: %02" PRIx8 " cdstatus: %02" PRIx8 "\n",
           priv->callback, priv->cbarg, priv->cbevents, priv->cdstatus);

    if (priv->callback)
    {
        /* Yes.. Check for enabled callback events */

        if ((priv->cdstatus & SDIO_STATUS_PRESENT) != 0)
        {
            /* Media is present.  Is the media inserted event enabled? */
            if ((priv->cbevents & SDIOMEDIA_INSERTED) == 0)
            {
                /* No... return without performing the callback */
                return;
            }
        }
        else
        {
            /* Media is not present.  Is the media eject event enabled? */

            if ((priv->cbevents & SDIOMEDIA_EJECTED) == 0)
            {
                /* No... return without performing the callback */

                return;
            }
        }

        /* Perform the callback, disabling further callbacks.  Of course, the
         * the callback can (and probably should) re-enable callbacks.
         */

        priv->cbevents = 0;

        /* Callbacks cannot be performed in the context of an interrupt
         * handler.  If we are in an interrupt handler, then queue the
         * callback to be performed later on the work thread.
         */

        if (up_interrupt_context())
        {
            /* Yes.. queue it */

            mcinfo("Queuing callback to %p(%p)\n",
                   priv->callback, priv->cbarg);

            work_queue(HPWORK, &priv->cbwork, priv->callback,
                       priv->cbarg, 0);
        }
        else
        {
            /* No.. then just call the callback here */

            mcinfo("Callback to %p(%p)\n", priv->callback, priv->cbarg);
            priv->callback(priv->cbarg);
        }
    }
}

static void hpm_sdmmc_callbackenable(FAR struct sdio_dev_s *dev, sdio_eventset_t eventset)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;
    DEBUGASSERT(priv != NULL);
    mcinfo("eventset: %02" PRIx8 "\n", eventset);
    priv->cbevents = eventset;
}

#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_SCHED_HPWORK)
static int hpm_sdmmc_registercallback(FAR struct sdio_dev_s *dev, worker_t callback, void *arg)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;
    DEBUGASSERT(priv != NULL);

    /* Disable callbacks and register this callback and is argument */

    mcinfo("Register %p(%p)\n", callback, arg);

    priv->cbevents = 0;
    priv->cbarg = arg;
    priv->callback = callback;
    return OK;
}
#endif

#ifdef CONFIG_ARCH_HAVE_SDIO_PREFLIGHT
static int hpm_sdmmc_dmapreflight(FAR struct sdio_dev_s *dev, FAR const uint8_t *buffer, size_t buflen)
{
    /**/
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;
    DEBUGASSERT((priv != NULL) && (buffer != NULL) && (buflen > 0));
    DEBUGASSERT((uint32_t)buffer % 4 != 0);

    return OK;
}
#endif

static int hpm_sdmmc_dmarecvsetup(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer, size_t buflen)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;
    DEBUGASSERT((priv != NULL) && (buffer != NULL) && (buflen > 0));
    DEBUGASSERT((uint32_t)buffer % 4 != 0);

    /* Prepare DMA parameter */
    uint32_t sys_addr = core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)buffer);
    hpm_sdmmc_noncacheable_ctx_t *nc_ctx = priv->nc_ctx;
    nc_ctx->adma_desc.addr = (uint32_t*)sys_addr;
    nc_ctx->adma_desc.len_attr = 0;
    nc_ctx->adma_desc.len_lower = buflen & 0xFFFFU;
    nc_ctx->adma_desc.len_upper = (buflen >> 16) & 0xFFFFU;
    nc_ctx->adma_desc.valid = 1;
    nc_ctx->adma_desc.interrupt = 1;
    nc_ctx->adma_desc.act = SDXC_ADMA2_DESC_TYPE_TRANS;
    nc_ctx->adma_desc.end = 1;

    priv->adma_cfg.adma_table = (uint32_t*)&nc_ctx->adma_desc;
    priv->adma_cfg.dma_type = sdxc_dmasel_adma2;
    priv->adma_cfg.adma_table_words = sizeof(nc_ctx->adma_desc) / sizeof(uint32_t);
    priv->dma_mode = HPM_SDMMC_DMA_MODE_ADMA2;

    /* Flush data to memory */
    if (!ADDRESS_IN_ILM((uint32_t)buffer) && !ADDRESS_IN_DLM((uint32_t)buffer))
    {
        /* Cache coherency maintenance
         *  In case the buffer address is not cache-line aligned, the software need to flush all data
         *  in the real memory first
         */
        uint32_t aligned_start = HPM_L1C_CACHELINE_ALIGN_DOWN(sys_addr);
        uint32_t aligned_end = HPM_L1C_CACHELINE_ALIGN_UP(sys_addr + buflen - 1U);
        uint32_t aligned_size = aligned_end - aligned_start;
        l1c_dc_flush(aligned_start, aligned_size);
        l1c_dc_invalidate(aligned_start, aligned_size);
    }
    return OK;
}

static int hpm_sdmmc_dmasendsetup(FAR struct sdio_dev_s *dev, FAR const uint8_t *buffer, size_t buflen)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;
    DEBUGASSERT((priv != NULL) && (buffer != NULL) && (buflen > 0));
    DEBUGASSERT((uint32_t)buffer % 4 != 0);

    /* Prepare DMA parameter */
    uint32_t sys_addr = core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)buffer);
    hpm_sdmmc_noncacheable_ctx_t *nc_ctx = priv->nc_ctx;
    nc_ctx->adma_desc.addr = (uint32_t*)sys_addr;
    nc_ctx->adma_desc.len_attr = 0;
    nc_ctx->adma_desc.len_lower = buflen & 0xFFFFU;
    nc_ctx->adma_desc.len_upper = (buflen >> 16) & 0xFFFFU;
    nc_ctx->adma_desc.valid = 1;
    nc_ctx->adma_desc.interrupt = 1;
    nc_ctx->adma_desc.act = SDXC_ADMA2_DESC_TYPE_TRANS;
    nc_ctx->adma_desc.end = 1;

    priv->adma_cfg.adma_table = (uint32_t*)&nc_ctx->adma_desc;
    priv->adma_cfg.dma_type = sdxc_dmasel_adma2;
    priv->adma_cfg.adma_table_words = sizeof(nc_ctx->adma_desc) / sizeof(uint32_t);
    priv->dma_mode = HPM_SDMMC_DMA_MODE_ADMA2;

    if (!ADDRESS_IN_ILM((uint32_t)buffer) && !ADDRESS_IN_DLM((uint32_t)buffer))
    {
        /* Cache coherency maintenance */
        uint32_t aligned_start = HPM_L1C_CACHELINE_ALIGN_DOWN(sys_addr);
        uint32_t aligned_end = HPM_L1C_CACHELINE_ALIGN_UP(sys_addr + buflen - 1U);
        uint32_t aligned_size = aligned_end - aligned_start;
        l1c_dc_flush(aligned_start, aligned_size);
    }

    return OK;
}

struct sdio_dev_s *sdio_initialize(int slotno)
{
    struct hpm_sdmmc_dev_s *priv = NULL;

#if defined(CONFIG_HPM_SDXC0)
    if (slotno == 0)
    {
        priv = &hpm_sdxc0_dev_s;
    }
#endif
#if defined(CONFIG_HPM_SDXC1)
    if (slotno == 1)
    {
        priv = &hpm_sdxc1_dev_s;
    }
#endif
    if (priv != NULL)
    {
#if defined(BOARD_APP_SDCARD_SUPPORT_POWER_SWITCH) && (BOARD_APP_SDCARD_SUPPORT_POWER_SWITCH == 1)
        bool as_gpio = false;
#if defined(BOARD_APP_SDCARD_POWER_SWITCH_USING_GPIO) && (BOARD_APP_SDCARD_POWER_SWITCH_USING_GPIO == 1)
        as_gpio = true;
#endif
        init_sdxc_pwr_pin(priv->base, as_gpio);
        if (as_gpio) {
            uint32_t gpio_index = BOARD_APP_SDCARD_POWER_SWITCH_PIN / 32;
            uint32_t pin_index = BOARD_APP_SDCARD_POWER_SWITCH_PIN % 32;
            HPM_GPIO0->OE[gpio_index].SET = (1UL << pin_index);
            HPM_GPIO0->DO[gpio_index].SET = (1UL << pin_index);
        }
#endif
        init_sdxc_cmd_pin(priv->base, false, false);
        init_sdxc_clk_data_pins(priv->base, priv->bus_width, false);
        board_sd_configure_clock(priv->base, HPM_SDMMC_CLK_INIT_FREQ, true);
        hpm_sdmmc_reset(&priv->dev);
        return &priv->dev;
    }
    return NULL;
}

void sdio_mediachange(struct sdio_dev_s *dev, bool cardinslot)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;
    sdio_statset_t cdstatus;
    irqstate_t flags;

    /* Update card status */

    flags = enter_critical_section();
    cdstatus = priv->cdstatus;
    if (cardinslot)
    {
        priv->cdstatus |= SDIO_STATUS_PRESENT;
    }
    else
    {
        priv->cdstatus &= ~SDIO_STATUS_PRESENT;
    }

    leave_critical_section(flags);

    mcinfo("cdstatus OLD: %02" PRIx8 " NEW: %02" PRIx8 "\n",
           cdstatus, priv->cdstatus);

    /* Perform any requested callback if the status has changed */
    if (cdstatus != priv->cdstatus)
    {
        hpm_sdmmc_callback(priv);
    }
}

#endif /* #ifdef CONFIG_HPM_SDXC_DRV */
