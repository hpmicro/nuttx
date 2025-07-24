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

#define HPM_SDMMC_CLK_INIT_FREQ             (37500UL)
#define HPM_SDMMC_CLK_NORMAL_FREQ           (25000000UL)
#define HPM_SDMMC_CLK_HIGH_FREQ             (50000000UL)
#define HPM_SDMMC_CLK_SDR50                 (100000000UL)
#define HPM_SDMMC_CLK_SDR104_HS200_HS400    (200000000UL)

#define HPM_SDMMC_CMDTIMEOUT (100000)
#define HPM_SDMMC_LONGTIMEOUT (0x7fffffff)

#define HPM_SDMMC_DMA_MODE_NONE (0)
#define HPM_SDMMC_DMA_MODE_ADMA2 (2)

#define HPM_SDMMC_PIN_NOT_SUPPORTED (0xffff)

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
    uint32_t *dst_buf;
    uint32_t xfer_size;
    bool need_free_buf;
    bool need_copy_data;

#ifdef CONFIG_HPM_SDXC_DRV
    /* Interrupt at SDIO_D1 pin, only for SDIO cards */

    uint32_t sdiointmask;        /* HPM SDIO register mask */
    int (*do_sdio_card)(void *); /* SDIO card ISR */
    void *do_sdio_arg;           /* arg for SDIO card ISR */
    bool support_1v8;
    bool support_3v3;
    uint32_t vsel_pin;
    uint32_t power_switch_pin;
    bool is_1v8_signaling;
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
static void hpm_sdmmc_widebus(FAR struct sdio_dev_s *dev, uint8_t wide);
static void hpm_sdmmc_clock(FAR struct sdio_dev_s *dev, enum sdio_clock_e rate);
static int hpm_sdmmc_attach(FAR struct sdio_dev_s *dev);

static int hpm_sdmmc_switch_uhs_voltage(FAR struct sdio_dev_s *dev);

void hpm_sdmmc_switch_to_1v8(struct hpm_sdmmc_dev_s *dev);
void hpm_sdmmc_switch_to_3v3(struct hpm_sdmmc_dev_s *dev);
void hpm_sdmmc_power_on(struct hpm_sdmmc_dev_s *dev);
void hpm_sdmmc_power_off(struct hpm_sdmmc_dev_s *dev);

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

static int hpm_sdmmc_tuning(FAR struct sdio_dev_s *dev, uint8_t tuning_cmd);
static int hpm_sdmmc_timing(FAR struct sdio_dev_s *dev, enum sdio_clock_e timing);

static uint32_t hpm_sdmmc_pin_get(const char *name);
static void hpm_sdmmc_vsel_pin_init(struct hpm_sdmmc_dev_s *priv);
static void hpm_sdmmc_pwr_pin_init(struct hpm_sdmmc_dev_s *priv);

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
            .switch_uhs_voltage = hpm_sdmmc_switch_uhs_voltage,
            .timing = hpm_sdmmc_timing,
        },
    .base = HPM_SDXC0,
    .clock_name = clock_sdxc0,
    .nirq = HPM_IRQn_SDXC0,
    .nc_ctx = &sdxc0_nc_ctx,
    .waitsem = SEM_INITIALIZER(0),
    .dma_mode = HPM_SDMMC_DMA_MODE_NONE,
#if defined(CONFIG_BOARD_SDXC0_BUSWIDTH_4BIT)
    .bus_width = 4,
#elif defined(CONFIG_BOARD_SDXC0_BUSWIDTH_8BIT)
    .bus_width = 8,
#else
    .bus_width  = 1,
#endif
    .vsel_pin = HPM_SDMMC_PIN_NOT_SUPPORTED,
    .power_switch_pin = HPM_SDMMC_PIN_NOT_SUPPORTED,
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
            .switch_uhs_voltage = hpm_sdmmc_switch_uhs_voltage,
            .timing = hpm_sdmmc_timing,
        },
    .base = HPM_SDXC1,
    .clock_name = clock_sdxc1,
    .nirq = HPM_IRQn_SDXC1,
    .nc_ctx = &sdxc1_nc_ctx,
    .waitsem = SEM_INITIALIZER(0),
    .dma_mode = HPM_SDMMC_DMA_MODE_NONE,
#if defined(CONFIG_BOARD_SDXC1_BUSWIDTH_4BIT)
    .bus_width = 4,
#elif defined(CONFIG_BOARD_SDXC1_BUSWIDTH_8BIT)
    .bus_width = 8,
#else
    .bus_width  = 1,
#endif
    .vsel_pin = HPM_SDMMC_PIN_NOT_SUPPORTED,
    .power_switch_pin = HPM_SDMMC_PIN_NOT_SUPPORTED,
};
#endif

static uint32_t hpm_sdmmc_pin_get(const char *name)
{
    uint32_t pad_idx = HPM_SDMMC_PIN_NOT_SUPPORTED;

    if (!((strlen(name) == 4) &&
          (name[0] == 'P') &&
          ((('A' <= name[1]) && (name[1] <= 'F')) || (('V' <= name[1]) && (name[1] <= 'Z'))) &&
          (('0' <= name[2]) && (name[2] <= '9')) &&
          (('0' <= name[3]) && (name[3] <= '9'))))
    {
        return pad_idx;
    }

    uint32_t gpio_idx = (name[1] <= 'F') ? (name[1] - 'A') : (11 + name[1] - 'V');
    uint32_t pin_idx = (uint32_t)(name[2] - '0') * 10 + (name[3] - '0');
    pad_idx = (gpio_idx * 32 + pin_idx);

    return pad_idx;
}

static void hpm_sdmmc_vsel_pin_init(struct hpm_sdmmc_dev_s *priv)
{
    if (priv->vsel_pin != HPM_SDMMC_PIN_NOT_SUPPORTED)
    {
        uint32_t pad_idx = priv->vsel_pin;
        uint32_t gpio_idx = pad_idx / 32;
        uint32_t pin_idx = pad_idx % 32;
#if defined(GPIO_DO_GPIOY)
        if (gpio_idx == GPIO_DO_GPIOY)
        {
            HPM_PIOC->PAD[pad_idx].FUNC_CTL = 3;
        }

#endif
#if defined(GPIO_DO_GPIOZ)
        if (gpio_idx == GPIO_DO_GPIOZ)
        {
            HPM_BIOC->PAD[pad_idx].FUNC_CTL = 3;
        }
#endif
        HPM_IOC->PAD[pad_idx].FUNC_CTL = 0;
#if defined(IOC_PAD_PAD_CTL_MS_MASK)
        HPM_IOC->PAD[pad_idx].PAD_CTL = 0x1E;
#else
        HPM_IOC->PAD[pad_idx].PAD_CTL = IOC_PAD_PAD_CTL_PS_MASK | IOC_PAD_PAD_CTL_PE_MASK | IOC_PAD_PAD_CTL_SR_MASK | IOC_PAD_PAD_CTL_SPD_SET(1);
#endif
        HPM_GPIO0->OE[gpio_idx].SET = (1UL << pin_idx);
    }
}
static void hpm_sdmmc_pwr_pin_init(struct hpm_sdmmc_dev_s *priv)
{
    if (priv->power_switch_pin != HPM_SDMMC_PIN_NOT_SUPPORTED)
    {
        uint32_t pad_idx = priv->power_switch_pin;
        uint32_t gpio_idx = pad_idx / 32;
        uint32_t pin_idx = pad_idx % 32;
#if defined(GPIO_DO_GPIOY)
        if (gpio_idx == GPIO_DO_GPIOY)
        {
            HPM_PIOC->PAD[pad_idx].FUNC_CTL = 3;
        }

#endif
#if defined(GPIO_DO_GPIOZ)
        if (gpio_idx == GPIO_DO_GPIOZ)
        {
            HPM_BIOC->PAD[pad_idx].FUNC_CTL = 3;
        }
#endif
        HPM_IOC->PAD[pad_idx].FUNC_CTL = 0;
#if defined(IOC_PAD_PAD_CTL_MS_MASK)
        HPM_IOC->PAD[pad_idx].PAD_CTL = 0x1E;
#else
        HPM_IOC->PAD[pad_idx].PAD_CTL = IOC_PAD_PAD_CTL_PS_MASK | IOC_PAD_PAD_CTL_PE_MASK | IOC_PAD_PAD_CTL_SR_MASK | IOC_PAD_PAD_CTL_SPD_SET(1);
#endif
        HPM_GPIO0->OE[gpio_idx].SET = (1UL << pin_idx);
    }
}

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
    up_mdelay(10);
    sdxc_config_t sdxc_config;
    sdxc_config.data_timeout = 1000;
    sdxc_init(priv->base, &sdxc_config);
    sdxc_wait_card_active(priv->base);
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
    if (priv->support_3v3)
    {
        caps |= SDIO_CAPS_3V3;
    }
    if (priv->support_1v8)
    {
        caps |= SDIO_CAPS_1V8 | SDIO_CAPS_MMC_HS200 | SDIO_CAPS_SD_SDR50 | SDIO_CAPS_SD_SDR104;
        if (priv->bus_width == 8)
        {
            caps |= SDIO_CAPS_MMC_HS400 | SDIO_CAPS_MMC_ENH_DQS;
        }
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

static int hpm_sdmmc_switch_uhs_voltage(FAR struct sdio_dev_s *dev)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;
    SDXC_Type *base = priv->base;

    /* 1. Stop providing clock to the card */
    sdxc_enable_inverse_clock(base, false);
    sdxc_enable_sd_clock(base, false);

    /* 2. Wait until DAT[3:0] are 4'b0000 */
    uint32_t data3_0_level;
    uint32_t delay_cnt = 1000000UL;
    do
    {
        data3_0_level = sdxc_get_data3_0_level(base);
        --delay_cnt;
    } while ((data3_0_level != 0U) && (delay_cnt > 0U));
    if (delay_cnt < 1)
    {
        return -ETIMEDOUT;
    }

    /* 3. Switch signaling to 1.8v */
    hpm_sdmmc_switch_to_1v8(priv);
    /* 4. delay 5ms */
    up_mdelay(7);
    /* 5. Provide SD clock the card again */
    sdxc_enable_sd_clock(base, true);
    /* 6. wait 1ms */
    up_mdelay(2);
    /* 7. Check DAT[3:0], make sure the value is 4'b0000 */
    delay_cnt = 1000000UL;
    do
    {
        data3_0_level = sdxc_get_data3_0_level(base);
        --delay_cnt;
    } while ((data3_0_level == 0U) && (delay_cnt > 0));
    if (delay_cnt < 1)
    {
        return -ETIMEDOUT;
    }

    priv->is_1v8_signaling = true;

    sdxc_enable_sd_clock(base, false);
    init_sdxc_cmd_pin(base, false, true);
    init_sdxc_clk_data_pins(base, priv->bus_width, true);
    up_udelay(100);
    sdxc_enable_sd_clock(base, true);

    return OK;
}

static int hpm_sdmmc_tuning(FAR struct sdio_dev_s *dev, uint8_t tuning_cmd)
{
    int ret = OK;
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;
    /* Tuning can work in 1.8V signaling only */
    if (!priv->is_1v8_signaling)
    {
        return ERROR;
    }

    SDXC_Type *base = priv->base;
    /* Prepare the Auto tuning environment */
    sdxc_stop_clock_during_phase_code_change(base, true);
    sdxc_set_post_change_delay(base, 3U);
    sdxc_select_cardclk_delay_source(base, false);
    sdxc_enable_power(base, true);

    bool need_inverse = sdxc_is_inverse_clock_enabled(base);
    sdxc_enable_inverse_clock(base, false);
    sdxc_enable_sd_clock(base, false);
    sdxc_enable_auto_tuning(base, true);
    sdxc_enable_inverse_clock(base, need_inverse);
    sdxc_enable_sd_clock(base, true);

    hpm_stat_t status = status_success;

    /* Turn off Sampling clock */
    sdxc_enable_sd_clock(base, false);
    sdxc_execute_tuning(base);
    uint32_t block_size = SDXC_PROT_CTRL_EXT_DAT_XFER_GET(base->PROT_CTRL) ? 128U : 64U;
    sdxc_command_t cmd;
    (void) memset(&cmd, 0, sizeof(cmd));
    cmd.cmd_index = tuning_cmd;
    cmd.cmd_argument = 0;
    cmd.cmd_flags = SDXC_CMD_XFER_DATA_PRESENT_SEL_MASK | SDXC_CMD_XFER_DATA_XFER_DIR_MASK;
    cmd.resp_type = sdxc_dev_resp_r1;
    clock_t start = clock_systime_ticks();
    clock_t elapsed_ticks;
    sdxc_enable_sd_clock(base, true);
    do {
        base->BLK_ATTR = block_size;
        base->SDMASA = 1;
        status = sdxc_send_command(base, &cmd);
        while (!IS_HPM_BITMASK_SET(base->INT_STAT, SDXC_INT_STAT_BUF_RD_READY_MASK)) {
            elapsed_ticks = clock_systime_ticks() - start;
            if (elapsed_ticks > TICK_PER_SEC)
            {
                status = status_timeout;
                break;
            }
        }
        sdxc_clear_interrupt_status(base, SDXC_INT_STAT_BUF_RD_READY_MASK);
    } while (IS_HPM_BITMASK_SET(base->AC_HOST_CTRL, SDXC_AC_HOST_CTRL_EXEC_TUNING_MASK) && (status == status_success));

    if (!IS_HPM_BITMASK_SET(base->AC_HOST_CTRL, SDXC_AC_HOST_CTRL_SAMPLE_CLK_SEL_MASK)) {
        /*FIXME*/
        ret = ERROR;
    }

    return ret;
}

static int hpm_sdmmc_timing(FAR struct sdio_dev_s *dev, enum sdio_clock_e timing)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;

    int ret = OK;

    bool is_sd = true;
    bool need_tuning = false;
    sdxc_speed_mode_t speed_mode = sdxc_sd_speed_normal;
    switch (timing)
    {
    case CLOCK_IDMODE:
        break;
    case CLOCK_SD_TRANSFER_1BIT:
        speed_mode = sdxc_sd_speed_normal;
        break;
    case CLOCK_SD_TRANSFER_4BIT:
        speed_mode = sdxc_sd_speed_high;
        break;
    case CLOCK_SD_SDR50:
        speed_mode = sdxc_sd_speed_sdr50;
        need_tuning = true;
        break;
    case CLOCK_SD_SDR104:
        speed_mode = sdxc_sd_speed_sdr104;
        need_tuning = true;
        break;
    case CLOCK_SD_DDR50:
        speed_mode = sdxc_sd_speed_ddr50;
        break;
    case CLOCK_MMC_TRANSFER:
        speed_mode = sdxc_emmc_speed_high_speed_sdr;
        is_sd = false;
        break;
    case CLOCK_MMC_HS200:
        speed_mode = sdxc_emmc_speed_hs200;
        need_tuning = true;
        is_sd = false;
        break;
    case CLOCK_MMC_HS400:
        speed_mode = sdxc_emmc_speed_hs400;
        need_tuning = true;
        is_sd = false;
        break;
    case CLOCK_MMC_HS400_ENH_DQS:
        speed_mode = sdxc_emmc_speed_hs400;
        is_sd = false;
        break;
    case CLOCK_MMC_HS_DDR:
        break;
    default:
        break;
    }
    sdxc_set_speed_mode(priv->base, speed_mode);

    hpm_sdmmc_clock(dev, timing);

    if (need_tuning)
    {
        uint8_t tuning_cmd = is_sd ? 19 : 21;
        ret = hpm_sdmmc_tuning(dev, tuning_cmd);
    }
    if (ret != OK)
    {
        return ret;
    }
    return OK;
}

static void hpm_sdmmc_widebus(FAR struct sdio_dev_s *dev, uint8_t wide)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;
    sdxc_bus_width_t bus_width = wide ? sdxc_bus_width_4bit : sdxc_bus_width_1bit;
    switch (wide)
    {
    default:
        bus_width = sdxc_bus_width_1bit;
        break;
    case 1:
        bus_width = sdxc_bus_width_4bit;
        break;
    case 2:
        bus_width = sdxc_bus_width_8bit;
        break;
    case 5:
        bus_width = sdxc_bus_width_4bit;
        break;
    case 6:
        bus_width = sdxc_bus_width_8bit;
        break;
    case 0x86:
        bus_width = sdxc_bus_width_8bit;
        break;
    }
    sdxc_set_data_bus_width(priv->base, bus_width);
}

static void hpm_sdmmc_clock(FAR struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;

    bool need_disable = false;
    uint32_t clock_freq = 0;
    bool clock_inverse = true;
    bool is_emmc = false;
    bool enable_enh_strobe = false;
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
    case CLOCK_SD_SDR50:
        clock_freq = HPM_SDMMC_CLK_SDR50;
        break;
    case CLOCK_SD_SDR104:
        clock_freq = HPM_SDMMC_CLK_SDR104_HS200_HS400;
        break;
    case CLOCK_SD_DDR50:
        clock_freq = HPM_SDMMC_CLK_HIGH_FREQ;
        clock_inverse = false;
        break;
    case CLOCK_MMC_HS200:
        clock_freq = HPM_SDMMC_CLK_SDR104_HS200_HS400;
        is_emmc = true;
        break;
    case CLOCK_MMC_HS400:
        clock_freq = HPM_SDMMC_CLK_SDR104_HS200_HS400;
        clock_inverse = false;
        is_emmc = true;
        break;
    case CLOCK_MMC_HS400_ENH_DQS:
        clock_freq = HPM_SDMMC_CLK_SDR104_HS200_HS400;
        clock_inverse = false;
        is_emmc = true;
        enable_enh_strobe = true;
        break;
    case CLOCK_MMC_HS_DDR:
        clock_freq = HPM_SDMMC_CLK_HIGH_FREQ;
        clock_inverse = false;
        is_emmc = true;
        break;
    }
    if (need_disable)
    {
        clock_remove_from_group(priv->clock_name, 0);
    }
    else
    {
        clock_add_to_group(priv->clock_name, 0);
        board_sd_configure_clock(priv->base, clock_freq, clock_inverse);
    }
    sdxc_enable_emmc_support(priv->base, is_emmc);
    sdxc_enable_enhanced_strobe(priv->base, enable_enh_strobe);
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
            break;
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

    if (cmd & MMCSD_MULTIBLOCK)
    {
        sdxc_cmd->cmd_flags |= SDXC_CMD_XFER_MULTI_BLK_SEL_MASK | SDXC_CMD_XFER_BLOCK_COUNT_ENABLE_MASK;
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
    DEBUGASSERT(((uint32_t)buffer & 3) == 0);

    priv->buffer = (uint32_t *)buffer;
    priv->remaining = nbytes;

    hpm_sdmmc_config_xfer_ints(priv, SDXC_INT_STAT_BUF_RD_READY_MASK);

    return OK;
}

static int hpm_sdmmc_sendsetup(FAR struct sdio_dev_s *dev, FAR const uint8_t *buffer, size_t nbytes)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;

    DEBUGASSERT((priv != NULL) && (buffer != NULL) && (nbytes > 0));
    DEBUGASSERT(((uint32_t)buffer & 3) == 0);

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
    int ret = OK;
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;

    int32_t timeout = HPM_SDMMC_CMDTIMEOUT;
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

    uint32_t int_stat;
    while (((int_stat = sdxc_get_interrupt_status(priv->base)) & events) == 0)
    {
        if ((int_stat & SDXC_STS_ERROR) != 0U)
        {
            ret = ERROR;
            break;
        }
        if (--timeout <= 0)
        {
            ret = -ETIMEDOUT;
            break;
        }

    }

    return ret;
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
    else
    {
        sdxc_reset(base, sdxc_reset_cmd_line, 0xffff);
        sdxc_clear_interrupt_status(base, ~0UL);
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

    if ((wakeupevents & SDIOWAIT_TRANSFERDONE) != 0)
    {
        if (priv->need_copy_data)
        {
            memcpy(priv->dst_buf, priv->buffer, priv->xfer_size);
            priv->need_copy_data = false;
        }
    }
    if (((wakeupevents & SDIOWAIT_TRANSFERDONE) != 0) || ((wakeupevents & SDIOWAIT_ERROR) != 0))
    {
        if (priv->need_free_buf)
        {
            free(priv->buffer);
            priv->need_free_buf = false;
        }
    }

    return wakeupevents;
}

static void hpm_sdmmc_callback(void *arg)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)arg;
    DEBUGASSERT(priv != NULL);

    mcinfo("Callback %p(%p) cbevents: %04" PRIx16 " cdstatus: %04" PRIx16 "\n",
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
    DEBUGASSERT(((uint32_t)buffer & 3) == 0);

    return OK;
}
#endif

static int hpm_sdmmc_dmarecvsetup(FAR struct sdio_dev_s *dev, FAR uint8_t *buffer, size_t buflen)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;
    DEBUGASSERT((priv != NULL) && (buffer != NULL) && (buflen > 0));

    uint8_t *recv_buf = buffer;
    if ((uint32_t)buffer % 4 != 0)
    {
        uint8_t *new_buf = (uint8_t*)malloc(buflen + HPM_L1C_CACHE_SIZE);
        if (new_buf == NULL)
        {
            return -ENOMEM;
        }
        recv_buf = HPM_L1C_CACHELINE_ALIGN_UP((uint32_t)new_buf);
        priv->buffer = new_buf;
        priv->need_free_buf = true;
        priv->dst_buf = buffer;
        priv->xfer_size = buflen;
    }

    /* Prepare DMA parameter */
    uint32_t sys_addr = core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)recv_buf);
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
    if (!ADDRESS_IN_ILM((uint32_t)recv_buf) && !ADDRESS_IN_DLM((uint32_t)recv_buf))
    {
        /* Cache coherency maintenance
         *  In case the buffer address is not cache-line aligned, the software need to flush all data
         *  in the real memory first
         */
        uint32_t aligned_start = HPM_L1C_CACHELINE_ALIGN_DOWN(sys_addr);
        uint32_t aligned_end = HPM_L1C_CACHELINE_ALIGN_UP(sys_addr + buflen - 1U);
        uint32_t aligned_size = aligned_end - aligned_start;
        l1c_dc_flush(aligned_start, aligned_size);
    }
    return OK;
}

static int hpm_sdmmc_dmasendsetup(FAR struct sdio_dev_s *dev, FAR const uint8_t *buffer, size_t buflen)
{
    struct hpm_sdmmc_dev_s *priv = (struct hpm_sdmmc_dev_s *)dev;
    DEBUGASSERT((priv != NULL) && (buffer != NULL) && (buflen > 0));
    uint8_t *send_buf = buffer;
    priv->need_free_buf = false;
    if ((uint32_t)buffer % 4 != 0)
    {
        uint8_t *new_buf = (uint8_t*)malloc(buflen + HPM_L1C_CACHE_SIZE);
        if (new_buf == NULL)
        {
            return -ENOMEM;
        }
        send_buf = HPM_L1C_CACHELINE_ALIGN_UP((uint32_t)new_buf);
        memcpy(send_buf, buffer, buflen);
        priv->buffer = new_buf;
        priv->need_free_buf = true;
    }

    /* Prepare DMA parameter */
    uint32_t sys_addr = core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)send_buf);
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

    if (!ADDRESS_IN_ILM((uint32_t)send_buf) && !ADDRESS_IN_DLM((uint32_t)send_buf))
    {
        /* Cache coherency maintenance */
        uint32_t aligned_start = HPM_L1C_CACHELINE_ALIGN_DOWN(sys_addr);
        uint32_t aligned_end = HPM_L1C_CACHELINE_ALIGN_UP(sys_addr + buflen - 1U);
        uint32_t aligned_size = aligned_end - aligned_start;
        l1c_dc_flush(aligned_start, aligned_size);
    }

    return OK;
}

void hpm_sdmmc_switch_to_1v8(struct hpm_sdmmc_dev_s *dev)
{
    uint32_t vsel_pin = dev->vsel_pin;
    if (vsel_pin != HPM_SDMMC_PIN_NOT_SUPPORTED)
    {
        uint32_t gpio_index = vsel_pin / 32;
        uint32_t pin_index = vsel_pin % 32;
        HPM_GPIO0->OE[gpio_index].SET = (1UL << pin_index);
        HPM_GPIO0->DO[gpio_index].SET = (1UL << pin_index);
    }
}
void hpm_sdmmc_switch_to_3v3(struct hpm_sdmmc_dev_s *dev)
{
    uint32_t vsel_pin = dev->vsel_pin;
    if (vsel_pin != HPM_SDMMC_PIN_NOT_SUPPORTED)
    {
        uint32_t gpio_index = vsel_pin / 32;
        uint32_t pin_index = vsel_pin % 32;
        HPM_GPIO0->OE[gpio_index].SET = (1UL << pin_index);
        HPM_GPIO0->DO[gpio_index].CLEAR = (1UL << pin_index);
    }
}
void hpm_sdmmc_power_on(struct hpm_sdmmc_dev_s *dev)
{
    uint32_t power_switch_pin = dev->power_switch_pin;
    if (power_switch_pin != HPM_SDMMC_PIN_NOT_SUPPORTED)
    {
        uint32_t gpio_index = power_switch_pin / 32;
        uint32_t pin_index = power_switch_pin % 32;
        HPM_GPIO0->OE[gpio_index].SET = (1UL << pin_index);
        HPM_GPIO0->DO[gpio_index].SET = (1UL << pin_index);
    }
}
void hpm_sdmmc_power_off(struct hpm_sdmmc_dev_s *dev)
{
    uint32_t power_switch_pin = dev->power_switch_pin;
    if (power_switch_pin != HPM_SDMMC_PIN_NOT_SUPPORTED)
    {
        uint32_t gpio_index = power_switch_pin / 32;
        uint32_t pin_index = power_switch_pin % 32;
        HPM_GPIO0->OE[gpio_index].SET = (1UL << pin_index);
        HPM_GPIO0->DO[gpio_index].CLEAR = (1UL << pin_index);
    }
}
struct sdio_dev_s *sdio_initialize(int slotno)
{
    struct hpm_sdmmc_dev_s *priv = NULL;

#if defined(CONFIG_HPM_SDXC0)
    if (slotno == 0)
    {
        priv = &hpm_sdxc0_dev_s;
#if defined(CONFIG_BOARD_SDXC0_VSEL_PIN)
        priv->vsel_pin = hpm_sdmmc_pin_get(CONFIG_BOARD_SDXC0_VSEL_PIN);
#endif
#if defined(CONFIG_BOARD_SDXC0_PWR_PIN)
        priv->power_switch_pin = hpm_sdmmc_pin_get(CONFIG_BOARD_SDXC0_PWR_PIN);
#endif
#if defined(CONFIG_BOARD_SDXC0_VOLTAGE_1V8) || defined(CONFIG_BOARD_SDXC0_VOLTAGE_DUAL)
        priv->support_1v8 = true;
#endif
#if defined(CONFIG_BOARD_SDXC0_VOLTAGE_3V3) || defined(CONFIG_BOARD_SDXC0_VOLTAGE_DUAL)
        priv->support_3v3 = true;
#endif
    }
#endif
#if defined(CONFIG_HPM_SDXC1)
    if (slotno == 1)
    {
        priv = &hpm_sdxc1_dev_s;
#if defined(CONFIG_BOARD_SDXC1_VSEL_PIN)
        priv->vsel_pin = hpm_sdmmc_pin_get(CONFIG_BOARD_SDXC1_VSEL_PIN);
#endif
#if defined(CONFIG_BOARD_SDXC1_PWR_PIN)
        priv->power_switch_pin = hpm_sdmmc_pin_get(CONFIG_BOARD_SDXC1_PWR_PIN);
#endif
#if defined(CONFIG_BOARD_SDXC1_VOLTAGE_1V8) || defined(CONFIG_BOARD_SDXC1_VOLTAGE_DUAL)
        priv->support_1v8 = true;
#endif
#if defined(CONFIG_BOARD_SDXC1_VOLTAGE_3V3) || defined(CONFIG_BOARD_SDXC1_VOLTAGE_DUAL)
        priv->support_3v3 = true;
#endif
    }
#endif
    if (priv != NULL)
    {
        bool support_dual_voltage = (priv->support_1v8 && priv->support_3v3);
        hpm_sdmmc_vsel_pin_init(priv);
        hpm_sdmmc_pwr_pin_init(priv);

        if (support_dual_voltage || priv->support_3v3)
        {
            hpm_sdmmc_switch_to_3v3(priv);
            priv->is_1v8_signaling = false;
        }
        else
        {
            hpm_sdmmc_switch_to_1v8(priv);
            priv->is_1v8_signaling = true;
        }

        /* Power up the SD/MMC card */
        hpm_sdmmc_power_off(priv);
        up_mdelay(100);
        hpm_sdmmc_power_on(priv);
        up_mdelay(10);

        /* Initialize the pins */
        bool is_1v8 = support_dual_voltage ? false : (priv->support_1v8 ? true : false);
        init_sdxc_cmd_pin(priv->base, false, is_1v8);
        init_sdxc_clk_data_pins(priv->base, priv->bus_width, is_1v8);
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
