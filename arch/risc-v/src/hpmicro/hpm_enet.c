/****************************************************************************
 * arch/risc-v/src/hpmicro/hpm_ethernet.c
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
#include <time.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <arpa/inet.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/ioctl.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/netdev.h>
#if defined(CONFIG_NET_PKT)
#  include <nuttx/net/pkt.h>
#endif

#include <arch/board/board.h>
#include "board.h"
#include "hpm_enet_drv.h"
#include "hpm_enet.h"

#if defined(CONFIG_ENET_PHY) && CONFIG_ENET_PHY
#if defined(CONFIG_ENET_PHY_RTL8211) && CONFIG_ENET_PHY_RTL8211
#define __USE_RTL8211 1
#elif defined(CONFIG_ENET_PHY_RTL8201) && CONFIG_ENET_PHY_RTL8201
#define __USE_RTL8201 1
#endif
#include "hpm_enet_phy_common.h"
#endif

#if (defined(CONFIG_HPM_ENET_RGMII) && CONFIG_HPM_ENET_RGMII) || \
    (defined(CONFIG_HPM_ENET_RMII) && CONFIG_HPM_ENET_RMII)
#  include "hpm_enet_phy.h"
#  if defined(CONFIG_HPM_ENET_RGMII) && CONFIG_HPM_ENET_RGMII
#    include "hpm_rtl8211.h"
#  elif defined(CONFIG_HPM_ENET_RMII) && CONFIG_HPM_ENET_RMII
#    include "hpm_rtl8201.h"
#  endif
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Driver-local logging (not tied to CONFIG_DEBUG_NET_*). */

#ifdef CONFIG_HPM_ENET_LOG_DEBUG
#  define hpm_enet_info(fmt, ...) \
     syslog(LOG_INFO, "hpm_enet: " fmt, ##__VA_ARGS__)
#else
#  define hpm_enet_info(fmt, ...) ((void)0)
#endif

#ifdef CONFIG_HPM_ENET_LOG_ERROR
#  define hpm_enet_err(fmt, ...) \
     syslog(LOG_ERR, "hpm_enet: " fmt, ##__VA_ARGS__)
#else
#  define hpm_enet_err(fmt, ...) ((void)0)
#endif

#ifdef CONFIG_HPM_ENET_LOG_WARN
#  define hpm_enet_warn(fmt, ...) \
     syslog(LOG_WARNING, "hpm_enet: " fmt, ##__VA_ARGS__)
#else
#  define hpm_enet_warn(fmt, ...) ((void)0)
#endif

#if defined(CONFIG_HPM_ENET_LOG_LINK)
#  define hpm_enet_link_notice(fmt, ...) \
     syslog(LOG_NOTICE, "hpm_enet: " fmt, ##__VA_ARGS__)
#else
#  define hpm_enet_link_notice(fmt, ...) ((void)0)
#endif

#define ETHWORK LPWORK

#if defined(CONFIG_HPM_ENET_RGMII) && CONFIG_HPM_ENET_RGMII
#define ENET_INF_TYPE       enet_inf_rgmii
#elif defined(CONFIG_HPM_ENET_RMII) && CONFIG_HPM_ENET_RMII
#define ENET_INF_TYPE       enet_inf_rmii
#endif

#if defined(CONFIG_HPM_ENET0) && CONFIG_HPM_ENET0
#define ENET HPM_ENET0
#define ENET_IRQ HPM_IRQn_ENET0
#elif defined(CONFIG_HPM_ENET1) && CONFIG_HPM_ENET1
#define ENET HPM_ENET1
#define ENET_IRQ HPM_IRQn_ENET1
#endif

#define BUF ((struct eth_hdr_s *)priv->dev.d_buf)

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required
#endif

ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ENET_SOC_DESC_ADDR_ALIGNMENT)
__RW enet_rx_desc_t dma_rx_desc_tab[ENET_RX_BUFF_COUNT] ; /* Ethernet Rx DMA Descriptor */

ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ENET_SOC_DESC_ADDR_ALIGNMENT)
__RW enet_tx_desc_t dma_tx_desc_tab[ENET_TX_BUFF_COUNT] ; /* Ethernet Tx DMA Descriptor */

ATTR_PLACE_AT_WITH_ALIGNMENT(".fast_ram", ENET_SOC_BUFF_ADDR_ALIGNMENT)
__RW uint8_t rx_buff[ENET_RX_BUFF_COUNT][ENET_RX_BUFF_SIZE]; /* Ethernet Receive Buffer */

ATTR_PLACE_AT_WITH_ALIGNMENT(".fast_ram", ENET_SOC_BUFF_ADDR_ALIGNMENT)
__RW uint8_t tx_buff[ENET_TX_BUFF_COUNT][ENET_TX_BUFF_SIZE]; /* Ethernet Transmit Buffer */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The hpm_enet_mac_s encapsulates all state information for a single
 * hardware interface
 */
struct hpm_enet_mac_s
{
  uint8_t              ifup    : 1; /* true:ifup false:ifdown */
  struct work_s        irqwork;     /* For deferring interrupt work to the work queue */
  struct work_s        pollwork;    /* For deferring poll work to the work queue */
#ifdef CONFIG_HPM_ENET_LINK_MONITOR
  struct work_s        linkwork;    /* PHY link polling */
  bool                 link_up;     /* Last notified carrier / link state */
#endif

  /* This holds the information visible to the NuttX network */
  struct net_driver_s  dev;         /* Interface understood by the network */

  enet_tx_desc_t       *txtail;     /* First "in_flight" TX descriptor */
  uint16_t             inflight;    /* Number of TX transfers "in_flight" */

  ENET_Type *base;
  enet_desc_t desc;
  enet_mac_config_t mac_config;
  bool cold_enet_init_done; /* Full hpm_enet_init (cold MAC/DMA+PHY) succeeded this boot */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct hpm_enet_mac_s g_hpmethmac;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int hpm_enet_config(struct hpm_enet_mac_s *priv);
static int hpm_transmit(struct hpm_enet_mac_s *priv);
#ifdef CONFIG_NETDEV_IOCTL
static int hpm_enet_ioctl(struct net_driver_s *dev, int cmd, unsigned long arg);
#endif

#if (defined(CONFIG_HPM_ENET_RGMII) && CONFIG_HPM_ENET_RGMII) || \
    (defined(CONFIG_HPM_ENET_RMII) && CONFIG_HPM_ENET_RMII)
static hpm_stat_t hpm_enet_ifup_resume_only(struct hpm_enet_mac_s *priv);
#endif

#if (defined(CONFIG_HPM_ENET_RGMII) && CONFIG_HPM_ENET_RGMII) || \
    (defined(CONFIG_HPM_ENET_RMII) && CONFIG_HPM_ENET_RMII)
/****************************************************************************
 * Function: hpm_mac_sync_phy_line
 *
 * Description:
 *   Match MAC MII speed/duplex bits to PHY negotiated values. enet_mac_init
 *   defaults (e.g. RGMII at 1000M full) must be updated after autoneg or the
 *   link may show up while frames fail.
 *
 ****************************************************************************/

static void hpm_mac_sync_phy_line(ENET_Type *base, const enet_phy_status_t *st)
{
  enet_line_speed_t line;
  enet_duplex_mode_t dpl;

  switch ((enet_phy_port_speed_t)st->enet_phy_speed)
    {
    case enet_phy_port_speed_10mbps:
      line = enet_line_speed_10mbps;
      break;
    case enet_phy_port_speed_100mbps:
      line = enet_line_speed_100mbps;
      break;
    case enet_phy_port_speed_1000mbps:
      line = enet_line_speed_1000mbps;
      break;
    default:
      line = enet_line_speed_100mbps;
      break;
    }

  dpl = (st->enet_phy_duplex == enet_phy_duplex_full) ?
        enet_full_duplex : enet_half_duplex;

  enet_set_line_speed(base, line);
  enet_set_duplex_mode(base, dpl);
}
#endif /* RGMII || RMII PHY */

#ifdef CONFIG_HPM_ENET_LINK_MONITOR
/****************************************************************************
 * Function: hpm_link_notice_up
 *
 * Description:
 *   Log negotiated line rate and duplex (from PHY status) on carrier on.
 *
 ****************************************************************************/

static void hpm_link_notice_up(const enet_phy_status_t *st)
{
  const char *spd;
  const char *dpl;

  switch ((enet_phy_port_speed_t)st->enet_phy_speed)
    {
    case enet_phy_port_speed_10mbps:
      spd = "10Mbps";
      break;
    case enet_phy_port_speed_100mbps:
      spd = "100Mbps";
      break;
    case enet_phy_port_speed_1000mbps:
      spd = "1000Mbps";
      break;
    default:
      spd = "?";
      break;
    }

  dpl = (st->enet_phy_duplex == enet_phy_duplex_full) ? "full" : "half";
  hpm_enet_link_notice("PHY link up (%s, %s duplex)\n", spd, dpl);
}

/****************************************************************************
 * Function: hpm_link_read_and_notify
 *
 * Description:
 *   Read PHY link via HPM SDK and update NuttX carrier (IFF_RUNNING) on
 *   change. Caller must hold the network lock when this runs in a context
 *   that can race with the stack (work queue).
 *
 ****************************************************************************/

static void hpm_link_read_and_notify(struct hpm_enet_mac_s *priv)
{
  enet_phy_status_t st = {0};
  bool up;

  /* RGMII/RMII: fill st via MDIO; otherwise st stays zero (link down). */

#if defined(CONFIG_HPM_ENET_RGMII) && CONFIG_HPM_ENET_RGMII
  rtl8211_get_phy_status(priv->base, RTL8211_ADDR, &st);
#elif defined(CONFIG_HPM_ENET_RMII) && CONFIG_HPM_ENET_RMII
  rtl8201_get_phy_status(priv->base, RTL8201_ADDR, &st);
#endif

  up = (st.enet_phy_link == enet_phy_link_up);
  if (up != priv->link_up)
    {
      priv->link_up = up;
      if (up)
        {
          hpm_mac_sync_phy_line(priv->base, &st);
          hpm_link_notice_up(&st);
          netdev_carrier_on(&priv->dev);
        }
      else
        {
          hpm_enet_link_notice("PHY link down\n");
          netdev_carrier_off(&priv->dev);
        }
    }
}

/****************************************************************************
 * Function: hpm_link_mon_work
 *
 * Description:
 *   Periodic PHY link poll on the low-priority work queue.
 *
 ****************************************************************************/

static void hpm_link_mon_work(void *arg)
{
  struct hpm_enet_mac_s *priv = (struct hpm_enet_mac_s *)arg;

  net_lock();

  if (priv->ifup)
    {
      hpm_link_read_and_notify(priv);
    }

  if (priv->ifup)
    {
      work_queue(ETHWORK, &priv->linkwork, hpm_link_mon_work, priv,
                 MSEC2TICK(CONFIG_HPM_ENET_LINK_MON_INTERVAL_MS));
    }

  net_unlock();
}
#endif /* CONFIG_HPM_ENET_LINK_MONITOR */

/****************************************************************************
 * Function: hpm_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int hpm_ifup(struct net_driver_s *dev)
{
  struct hpm_enet_mac_s *priv =
    (struct hpm_enet_mac_s *)dev->d_private;
  int ret;

#ifdef CONFIG_NET_IPv4
#  ifdef CONFIG_NETINIT_DHCPC
  if (dev->d_ipaddr != 0)
    {
      hpm_enet_info("Bringing up: %d.%d.%d.%d (may be replaced by DHCP)\n",
                    (int)(dev->d_ipaddr & 0xff),
                    (int)((dev->d_ipaddr >> 8) & 0xff),
                    (int)((dev->d_ipaddr >> 16) & 0xff),
                    (int)(dev->d_ipaddr >> 24));
    }
  else
    {
      hpm_enet_info("Bringing up link (IPv4 assigned via DHCP after link is up)\n");
    }
#  else
  if (dev->d_ipaddr != 0)
    {
      hpm_enet_info("Bringing up (static): %d.%d.%d.%d\n",
                    (int)(dev->d_ipaddr & 0xff),
                    (int)((dev->d_ipaddr >> 8) & 0xff),
                    (int)((dev->d_ipaddr >> 16) & 0xff),
                    (int)(dev->d_ipaddr >> 24));
    }
  else
    {
      hpm_enet_info("Bringing up link (IPv4 not set; check ipcfg or NETINIT)\n");
    }
#  endif
#endif

  /* Configure the Ethernet interface for DMA operation. */
  ret = hpm_enet_config(priv);
  if (ret < 0)
    {
      hpm_enet_warn("ifup: configuration failed (%d)\n", ret);
      return ret;
    }

  /* Enable the Ethernet interrupt */

  priv->ifup = true;
  hpm_enet_info("interface up\n");
  up_enable_irq(ENET_IRQ);

#ifdef CONFIG_HPM_ENET_LINK_MONITOR
  /* Caller (netdev_ifup) holds the network lock. */

  priv->link_up = false;
  netdev_carrier_off(&priv->dev);
  hpm_link_read_and_notify(priv);
  work_queue(ETHWORK, &priv->linkwork, hpm_link_mon_work, priv,
             MSEC2TICK(CONFIG_HPM_ENET_LINK_MON_INTERVAL_MS));
#endif

  return OK;
}

/****************************************************************************
 * Function: hpm_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   Returns zero on success; a negated errno value is returned on any
 *   failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int hpm_ifdown(struct net_driver_s *dev)
{
  struct hpm_enet_mac_s *priv = (struct hpm_enet_mac_s *)dev->d_private;
  irqstate_t flags;
  int ret = OK;
  const bool was_up = priv->ifup;

#ifdef CONFIG_HPM_ENET_LINK_MONITOR
  /* Stop link polling; caller holds the network lock (netdev_ifdown). */

  work_cancel(ETHWORK, &priv->linkwork);
  priv->link_up = false;
  netdev_carrier_off(&priv->dev);
#endif

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
  up_disable_irq(ENET_IRQ);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the hpm_ifup() always
   * successfully brings the interface back up.
   */

  /* Mark the device "down" */

  priv->ifup = false;
  if (was_up)
    {
      /* Avoid noise when netdev/stack calls ifdown on an already-down IF
       * (e.g. driver init ending in ifdown before first bring-up).
       */

      hpm_enet_info("interface down\n");
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Function: hpm_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send. This is a callback from devif_poll().
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int hpm_txpoll(struct net_driver_s *dev)
{
    struct hpm_enet_mac_s *priv = (struct hpm_enet_mac_s *)dev->d_private;

    DEBUGASSERT(priv->dev.d_buf != NULL);

    if (priv->dev.d_len > 0)
    {
        arp_out(&priv->dev);

        if (!devif_loopback(&priv->dev)) 
        {
            hpm_transmit(priv);
            DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);

            if (priv->desc.tx_desc_list_cur->tdes0_bm.own != 0) {
                return -EBUSY;
            } 

            /* CPU owns the descriptor and continues the poll. */          
            dev->d_buf = (uint8_t *)sys_address_to_core_local_mem(0, (uint32_t)priv->desc.tx_desc_list_cur->tdes2_bm.buffer1); 
        }
    }

    return 0;
}

/****************************************************************************
 * Function: hpm_dopoll
 *
 * Description:
 *   The function is called in order to perform an out-of-sequence TX poll.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void hpm_dopoll(struct hpm_enet_mac_s *priv)
{
    enet_tx_desc_t  *tx_desc_list_cur = priv->desc.tx_desc_list_cur;
    struct net_driver_s *dev = &priv->dev;

    if (tx_desc_list_cur->tdes0_bm.own == 0) {
        /* If we have the descriptor, then poll the network for new XMIT data. */
        DEBUGASSERT(dev->d_len == 0 && dev->d_buf == NULL);

        dev->d_buf = (uint8_t *)sys_address_to_core_local_mem(0, (uint32_t)tx_desc_list_cur->tdes2_bm.buffer1); 
      
        devif_poll(dev, hpm_txpoll);

        if (dev->d_buf) {
            DEBUGASSERT(dev->d_len == 0);
            dev->d_buf = NULL;
        }
    }
}


/****************************************************************************
 * Function: hpm_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Input Parameters:
 *   arg  - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void hpm_txavail_work(void *arg)
{
  struct hpm_enet_mac_s *priv = (struct hpm_enet_mac_s *)arg;

  /* Ignore the notification if the interface is not yet up */
  net_lock();
  if (priv->ifup)
    {
      /* Poll the network for new XMIT data */

      hpm_dopoll(priv);
    }

  net_unlock();
}

/****************************************************************************
 * Function: hpm_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int hpm_txavail(struct net_driver_s *dev)
{
  struct hpm_enet_mac_s *priv =
    (struct hpm_enet_mac_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->pollwork, hpm_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: hpm_enet_ioctl
 *
 * Description:
 *   PHY MDIO ioctl entry points used by upper layers (for example netinit
 *   link monitoring via SIOCGMIIPHY / SIOCGMIIREG).
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int hpm_enet_ioctl(struct net_driver_s *dev, int cmd,
                          unsigned long arg)
{
#ifdef CONFIG_NETDEV_PHY_IOCTL
  struct hpm_enet_mac_s *priv =
    (struct hpm_enet_mac_s *)dev->d_private;
#endif
  int ret;

  switch (cmd)
    {
#ifdef CONFIG_NETDEV_PHY_IOCTL
#if defined(CONFIG_ENET_PHY) && CONFIG_ENET_PHY
      case SIOCGMIIPHY: /* Get MII PHY address */
        {
          struct mii_ioctl_data_s *req =
            (struct mii_ioctl_data_s *)((uintptr_t)arg);

#if defined(CONFIG_HPM_ENET_RGMII) && CONFIG_HPM_ENET_RGMII
          req->phy_id = RTL8211_ADDR;
          ret = OK;
#elif defined(CONFIG_HPM_ENET_RMII) && CONFIG_HPM_ENET_RMII
          req->phy_id = RTL8201_ADDR;
          ret = OK;
#else
          ret = -ENOTSUP;
#endif
        }
        break;

      case SIOCGMIIREG: /* Read MII PHY register */
        {
          struct mii_ioctl_data_s *req =
            (struct mii_ioctl_data_s *)((uintptr_t)arg);
          ENET_Type *base = priv->base;
          uint32_t addr = req->phy_id;

          /* Align link status with the same PHY semantics as HPM link monitor /
           * rtl8211_get_phy_status / rtl8201_get_phy_status.  NETINIT_MONITOR
           * reads MII_MSR (BMSR); RTL8211 real-time link is in PHYSR, not BMSR.
           */

#if defined(CONFIG_HPM_ENET_RGMII) && CONFIG_HPM_ENET_RGMII
          if (req->reg_num == MII_MSR)
            {
              uint16_t physr = enet_read_phy(base, addr, RTL8211_PHYSR);
              uint16_t bmsr = enet_read_phy(base, addr, MII_MSR);

              req->val_out = bmsr;
              if (RTL8211_PHYSR_LINK_REAL_TIME_GET(physr))
                {
                  req->val_out |= MII_MSR_LINKSTATUS;
                }
              else
                {
                  req->val_out &= (uint16_t)~MII_MSR_LINKSTATUS;
                }
            }
          else
            {
              req->val_out = enet_read_phy(base, addr, req->reg_num);
            }
#elif defined(CONFIG_HPM_ENET_RMII) && CONFIG_HPM_ENET_RMII
          if (req->reg_num == MII_MSR)
            {
              enet_write_phy(base, addr, RTL8201_PAGESEL, 0);
              (void)enet_read_phy(base, addr, RTL8201_BMSR);
              req->val_out = enet_read_phy(base, addr, RTL8201_BMSR);
            }
          else
            {
              req->val_out = enet_read_phy(base, addr, req->reg_num);
            }
#else
          req->val_out = enet_read_phy(base, addr, req->reg_num);
#endif
          ret = OK;
        }
        break;

      case SIOCSMIIREG: /* Write MII PHY register */
        {
          struct mii_ioctl_data_s *req =
            (struct mii_ioctl_data_s *)((uintptr_t)arg);

          enet_write_phy(priv->base, req->phy_id, req->reg_num,
                         req->val_in);
          ret = OK;
        }
        break;
#endif /* CONFIG_ENET_PHY */
#endif /* CONFIG_NETDEV_PHY_IOCTL */

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}
#endif /* CONFIG_NETDEV_IOCTL */

/****************************************************************************
 * Function: hpm_enet_gpioconfig
 *
 * Description:
 *  Configure GPIOs for the Ethernet interface.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

static inline void hpm_enet_gpioconfig(struct hpm_enet_mac_s *priv)
{
    /* Initialize GPIOs */
    board_init_enet_pins(priv->base);
}

/****************************************************************************
 * Function: hpm_freesegment
 *
 * Description:
 *   The function is called when a received frame is completely processed.
 *   interrupt.  
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 *
 ****************************************************************************/

static void hpm_freesegment(struct hpm_enet_mac_s *priv)
{
    enet_rx_frame_info_t *rx_frame_info;
    enet_rx_desc_t *rx_desc;

    rx_frame_info = &priv->desc.rx_frame_info;
    rx_desc = rx_frame_info->fs_rx_desc;

    /* Release buffers to DMA */
    for (int i = 0; i < rx_frame_info->seg_count; i++) {
        rx_desc->rdes0_bm.own = 1;
        rx_desc = rx_desc->rdes3_bm.next_desc;
    }

    rx_frame_info->seg_count = 0;

    enet_rx_resume(priv->base);
}

/****************************************************************************
 * Function: hpm_recvframe
 *
 * Description:
 *   The function is called when a frame is received using the DMA receive
 *   interrupt.  It scans the RX descriptors of the received frame.
 *
 *   NOTE: This function will silently discard any packets containing errors.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK if a packet was successfully returned; -EAGAIN if there are no
 *   further packets available
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static int hpm_recvframe(struct hpm_enet_mac_s *priv)
{
    enet_frame_t frame = {0, 0, 0};
    int ret = -EAGAIN;

    frame = enet_get_received_frame_interrupt(&priv->desc.rx_desc_list_cur, &priv->desc.rx_frame_info, ENET_RX_BUFF_COUNT);
    priv->dev.d_buf = (uint8_t *)sys_address_to_core_local_mem(0, (uint32_t)frame.buffer); //frame.buffer;
    priv->dev.d_len = (uint16_t)frame.length;

    if (priv->dev.d_len > 0 && priv->dev.d_buf != NULL) {
   
        if (priv->dev.d_len > ENET_RX_BUFF_SIZE || priv->desc.rx_frame_info.seg_count > 1) 
        {
            printf("ERROR: Dropped, RX descriptor Too big: %d in %d segments\n", priv->dev.d_len, priv->desc.rx_frame_info.seg_count);            
            hpm_freesegment(priv);
        } else if (priv->desc.rx_frame_info.ls_rx_desc->rdes0_bm.es) {
            printf("ERROR: Dropped, RX descriptor errors: %08x\n", priv->desc.rx_frame_info.fs_rx_desc);
            hpm_freesegment(priv);
        }
        else {
            ret = OK;
        }
    } 

    return ret;
}

/****************************************************************************
 * Function: hpm_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int hpm_transmit(struct hpm_enet_mac_s *priv)
{
    enet_tx_desc_t *tx_desc_list_cur;
    enet_tx_desc_t *tx_first;    
    uint16_t frame_length = 0;

    tx_desc_list_cur = priv->desc.tx_desc_list_cur;
    tx_first = tx_desc_list_cur;

    if (core_local_mem_to_sys_address(BOARD_APP_CORE, (uint32_t)priv->dev.d_buf) == priv->desc.rx_frame_info.fs_rx_desc->rdes2_bm.buffer1) {      

        priv->desc.rx_frame_info.fs_rx_desc->rdes2_bm.buffer1 = priv->desc.tx_desc_list_cur->tdes2_bm.buffer1;
    }

    /* Set the buffer1 address pointer */
    priv->desc.tx_desc_list_cur->tdes2_bm.buffer1 = core_local_mem_to_sys_address(BOARD_APP_CORE, (uint32_t)priv->dev.d_buf);

    /* Set frame size */
    frame_length = priv->dev.d_len + 4;
    
    /* Prepare transmit descriptors to DMA */
    enet_prepare_tx_desc(priv->base, &priv->desc.tx_desc_list_cur, &priv->desc.tx_control_config, frame_length, priv->desc.tx_buff_cfg.size);

    /* Detach the buffer from priv->dev structure. That buffer is now "in-flight" */
    priv->dev.d_buf = NULL;
    priv->dev.d_len = 0;
    
    /* If there is no other TX buffer, in flight, then remember the location
     * of the TX descriptor.  This is the location to check for TX done events.
     */
    if (!priv->txtail)
    {
        DEBUGASSERT(priv->inflight == 0);
        priv->txtail = tx_first;
     }

    /* Increment the number of TX transfer in-flight */    
    priv->inflight++;

    /* If all TX descriptors are in-flight, then we have to disable receive
     * interrupts too. This is because receive events can trigger more
     * un-stoppable transmit events.
     */
    if (priv->inflight >= ENET_TX_BUFF_COUNT) {
        priv->base->DMA_INTR_EN &= ~ENET_DMA_INTR_EN_RIE_MASK;
    }

    if (ENET_DMA_STATUS_TU_GET(priv->base->DMA_STATUS) != 0) {
        priv->base->DMA_STATUS |= ENET_DMA_STATUS_TU_MASK;
        priv->base->DMA_TX_POLL_DEMAND = 0;
    }

    /* Enable Tx interrupt */
    if (ENET_DMA_INTR_EN_TIE_GET(priv->base->DMA_INTR_EN) == 0) {
        priv->base->DMA_INTR_EN |= ENET_DMA_INTR_EN_TIE_MASK;
    }

    return OK;
}

/****************************************************************************
 * Function: hpm_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void hpm_receive(struct hpm_enet_mac_s *priv)
{
  struct net_driver_s *dev = &priv->dev;

  /* Loop while while hpm_recvframe() successfully retrieves valid
   * Ethernet frames.
   */
  while (hpm_recvframe(priv) == OK)
    {
#ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the tap */
     pkt_input(&priv->dev);
#endif

      /* We only accept IP packets of the configured type and ARP packets */
#ifdef CONFIG_NET_IPv4
      if (BUF->type == HTONS(ETHTYPE_IP))
        {
          /* Receive an IPv4 packet from the network device */
          arp_ipin(&priv->dev);
          ipv4_input(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           */
          if (priv->dev.d_len > 0)
            {
              /* And send the packet */
              arp_out(&priv->dev);
              hpm_transmit(priv);
            }
        }
      else
#endif

#ifdef CONFIG_NET_ARP
      if (BUF->type == HTONS(ETHTYPE_ARP))
        {
          /* Handle ARP packet */
          arp_arpin(&priv->dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, d_len field will set to a value > 0.
           */
          if (priv->dev.d_len > 0)
            {
              hpm_transmit(priv);
            }
        }
      else
#endif
        {
          hpm_enet_warn("dropped frame, unknown ethertype: 0x%04x\n",
                        BUF->type);
        }

      /* We are finished with the RX buffer.  NOTE:  If the buffer is
       * re-used for transmission, the dev->d_buf field will have been
       * nullified.
       */
      if (dev->d_buf)
        {
          /* Free the receive packet segment */
          dev->d_buf = NULL;
          dev->d_len = 0;
        }

        hpm_freesegment(priv);
    }
}

static void hpm_freeframe(struct hpm_enet_mac_s *priv)
{
    enet_tx_desc_t *txdesc;

    txdesc = priv->txtail;
    if (txdesc) {
        for (int i = 0; txdesc->tdes0_bm.own == 0; i++) {
            /* Check if this is the last segment of a TX frame */
            if (txdesc->tdes0_bm.ls) {
                /* Decrement the number of frames "in-flight". */
                priv->inflight--;

                /* If all of the TX descriptors were in-flight,
                 * then RX interrupts may have been disabled...
                 * we can re-enable them now.
                 */
                priv->base->DMA_INTR_EN |= ENET_DMA_INTR_EN_RIE_MASK;

                /* If there are no more frames in-flight, then bail. */
                if (priv->inflight <= 0) {
                    priv->txtail = NULL;
                    priv->inflight = 0;                
                    return;
                }

                /* Try the next descriptor in the TX chain */
                txdesc = (enet_tx_desc_t *)(txdesc->tdes3_bm.next_desc);
            }
        }

        priv->txtail = txdesc;
    }
}

static void hpm_txdone(struct hpm_enet_mac_s *priv)
{
    /* Scan the TX descriptor change */
    hpm_freeframe(priv);
    
    /* If no further xmits are pending, then cancel the TX timeout */
    if (priv->inflight <= 0) {

        /* And disable further TX interrupt. */
        priv->base->DMA_INTR_EN &= ~ENET_DMA_INTR_EN_TIE_MASK;
    }

    hpm_dopoll(priv);
}

/****************************************************************************
 * Function: hpm_interrupt_work
 *
 * Description:
 *   Perform interrupt related work from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() was called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   Ethernet interrupts are disabled
 *
 ****************************************************************************/

static void hpm_interrupt_work(void *arg)
{
    struct hpm_enet_mac_s *priv = (struct hpm_enet_mac_s *)arg;
    uint32_t status;
    uint32_t rxgbfrmis;
    uint32_t intr_status;

    DEBUGASSERT(priv);

    net_lock();

    /* Process pending Ethernet interrupts */
    status = priv->base->DMA_STATUS;
    rxgbfrmis = priv->base->MMC_INTR_RX;
    intr_status = priv->base->INTR_STATUS;

  /* Check if there are pending "normal" interrupts */

    if (ENET_DMA_STATUS_GLPII_GET(status)) {
       /* read LPI_CSR to clear interrupt status */
       priv->base->LPI_CSR;
    }

    if (ENET_INTR_STATUS_RGSMIIIS_GET(intr_status)) {
        /* read XMII_CSR to clear interrupt status */
        priv->base->XMII_CSR;
    }

    if (ENET_DMA_STATUS_TI_GET(status)) {
        priv->base->DMA_STATUS |= ENET_DMA_STATUS_TI_MASK;
        /* Handle the transmitted package */
        hpm_txdone(priv);
    }

    if (ENET_DMA_STATUS_RI_GET(status)) {
        priv->base->DMA_STATUS |= ENET_DMA_STATUS_RI_MASK;
        /* Handle the received package */
        hpm_receive(priv);
    }
    
    if (ENET_MMC_INTR_RX_RXCTRLFIS_GET(rxgbfrmis)) {
        priv->base->RXFRAMECOUNT_GB;
    }

  net_unlock();

  /* Re-enable Ethernet interrupts at the NVIC */

  up_enable_irq(ENET_IRQ);
}

/****************************************************************************
 * Function: hpm_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *
 ****************************************************************************/
static int hpm_interrupt(int irq, void *context, void *arg)
{
  struct hpm_enet_mac_s *priv = &g_hpmethmac;
  uint32_t dmasr;

  /* Get the DMA interrupt status bits (no MAC interrupts are expected) */
  dmasr = priv->base->DMA_STATUS;

  if (dmasr != 0)
    {
      /* Disable further Ethernet interrupts.  Because Ethernet interrupts
       * are also disabled if the TX timeout event occurs, there can be no
       * race condition here.
       */
      up_disable_irq(ENET_IRQ);

      /* Schedule to perform the interrupt processing on the worker thread. */
      work_queue(ETHWORK, &priv->irqwork, hpm_interrupt_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Function: hpm_enet_init
 *
 * Description:
 *  Configure the Ethernet Controller and DMA
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static hpm_stat_t hpm_enet_init(struct hpm_enet_mac_s *priv)
{
    /* Same as cold_boot_hw in hpm_enet_config(): cold_enet_init_done set only after OK. */

    const bool cold_phy_reset = !priv->cold_enet_init_done;

#if (defined(CONFIG_HPM_ENET_RGMII) && CONFIG_HPM_ENET_RGMII) || \
    (defined(CONFIG_HPM_ENET_RMII) && CONFIG_HPM_ENET_RMII)
    enet_phy_status_t st = {0};
#endif

    enet_int_config_t int_config = {.int_enable = 0, .int_mask = 0};
    enet_mac_config_t enet_config;

    #if defined(CONFIG_HPM_ENET_RGMII) && CONFIG_HPM_ENET_RGMII
        rtl8211_config_t phy_config;
    #elif defined(CONFIG_HPM_ENET_RMII) && CONFIG_HPM_ENET_RMII
        rtl8201_config_t phy_config;
    #endif

    /* Initialize td, rd and the corresponding buffers */
    memset((uint8_t *)dma_tx_desc_tab, 0x00, sizeof(dma_tx_desc_tab));
    memset((uint8_t *)dma_rx_desc_tab, 0x00, sizeof(dma_rx_desc_tab));
    memset((uint8_t *)rx_buff, 0x00, sizeof(rx_buff));
    memset((uint8_t *)tx_buff, 0x00, sizeof(tx_buff));

    priv->desc.tx_desc_list_head = (enet_tx_desc_t *)core_local_mem_to_sys_address(BOARD_APP_CORE, (uint32_t)dma_tx_desc_tab);
    priv->desc.rx_desc_list_head = (enet_rx_desc_t *)core_local_mem_to_sys_address(BOARD_APP_CORE, (uint32_t)dma_rx_desc_tab);

    priv->desc.tx_buff_cfg.buffer = core_local_mem_to_sys_address(BOARD_APP_CORE, (uint32_t)tx_buff);
    priv->desc.tx_buff_cfg.count = ENET_TX_BUFF_COUNT;
    priv->desc.tx_buff_cfg.size = ENET_TX_BUFF_SIZE;

    priv->desc.rx_buff_cfg.buffer = core_local_mem_to_sys_address(BOARD_APP_CORE, (uint32_t)rx_buff);
    priv->desc.rx_buff_cfg.count = ENET_RX_BUFF_COUNT;
    priv->desc.rx_buff_cfg.size = ENET_RX_BUFF_SIZE;

    /*Get a default control config for tx descriptor */
    enet_get_default_tx_control_config(priv->base, &priv->desc.tx_control_config);

    priv->desc.tx_control_config.cic = 0;
    priv->desc.tx_control_config.enable_ioc = true;

    /* Set MAC0 address */
    enet_config.mac_addr_high[0] = priv->dev.d_mac.ether.ether_addr_octet[5] << 8 |
                                   priv->dev.d_mac.ether.ether_addr_octet[4];
    enet_config.mac_addr_low[0]  = priv->dev.d_mac.ether.ether_addr_octet[3] << 24 |
                                   priv->dev.d_mac.ether.ether_addr_octet[2] << 16 |
                                   priv->dev.d_mac.ether.ether_addr_octet[1] << 8 |
                                   priv->dev.d_mac.ether.ether_addr_octet[0];

    enet_config.valid_max_count  = 1;

    /* Set DMA PBL */
    enet_config.dma_pbl = board_get_enet_dma_pbl(priv->base);

    /* Set SARC */
    enet_config.sarc = enet_sarc_replace_mac0;

    /* Enable Enet IRQ */
    board_enable_enet_irq(priv->base);

    /* Set the interrupt enable mask */
    int_config.int_enable = enet_normal_int_sum_en    /* Enable normal interrupt summary */
                          | enet_receive_int_en       /* Enable receive interrupt */
                          | enet_transmit_int_en;

    int_config.int_mask = enet_rgsmii_int_mask; /* Disable RGSMII interrupt */

    if (enet_controller_init(priv->base, ENET_INF_TYPE, &priv->desc, &enet_config,
                             &int_config) != status_success)
      {
        hpm_enet_err("Enet controller init failed\n");
        return status_fail;
      }

    /* Disable LPI interrupt */
    enet_disable_lpi_interrupt(priv->base);

    /* PHY: cold path resets/program PHY after MAC/DMA (SDK order); warm path sync only */

#if defined(CONFIG_HPM_ENET_RGMII) && CONFIG_HPM_ENET_RGMII
    if (cold_phy_reset)
      {
        rtl8211_reset(priv->base, RTL8211_ADDR);
        rtl8211_basic_mode_default_config(priv->base, &phy_config);

        if (rtl8211_basic_mode_init(priv->base, RTL8211_ADDR, &phy_config) != true)
          {
            hpm_enet_err("Enet phy init failed\n");
            return status_fail;
          }

        hpm_enet_info("Enet phy init passed\n");
      }
    else
      {
        rtl8211_get_phy_status(priv->base, RTL8211_ADDR, &st);
        if (st.enet_phy_link == (uint8_t)enet_phy_link_up)
          {
            hpm_mac_sync_phy_line(priv->base, &st);
          }

        hpm_enet_info("Enet PHY warm ifup (link retained)\n");
      }
#elif defined(CONFIG_HPM_ENET_RMII) && CONFIG_HPM_ENET_RMII
    if (cold_phy_reset)
      {
        rtl8201_reset(priv->base, RTL8201_ADDR);
        rtl8201_basic_mode_default_config(priv->base, &phy_config);
        phy_config.media_interface  = ENET_INF_TYPE;
        phy_config.rmii_refclk_dir  = (uint8_t)CONFIG_RMII_REFCLK;

        if (rtl8201_basic_mode_init(priv->base, RTL8201_ADDR, &phy_config) != true)
          {
            hpm_enet_err("Enet phy init failed\n");
            return status_fail;
          }

        hpm_enet_info("Enet phy init passed\n");
      }
    else
      {
        rtl8201_get_phy_status(priv->base, RTL8201_ADDR, &st);
        if (st.enet_phy_link == (uint8_t)enet_phy_link_up)
          {
            hpm_mac_sync_phy_line(priv->base, &st);
          }

        hpm_enet_info("Enet PHY warm ifup (link retained)\n");
      }
#endif

    return status_success;
}

#if (defined(CONFIG_HPM_ENET_RGMII) && CONFIG_HPM_ENET_RGMII) || \
    (defined(CONFIG_HPM_ENET_RMII) && CONFIG_HPM_ENET_RMII)
/****************************************************************************
 * Name: hpm_enet_ifup_resume_only
 *
 * Description:
 *   After one successful full init since power-on, further netdev ifup only
 *   re-syncs MAC line speed/duplex from PHY and restarts RX; MAC/DMA/PHY
 *   are not reset or re-programmed.
 *
 ****************************************************************************/

static hpm_stat_t hpm_enet_ifup_resume_only(struct hpm_enet_mac_s *priv)
{
  enet_phy_status_t st = {0};

#if defined(CONFIG_HPM_ENET_RGMII) && CONFIG_HPM_ENET_RGMII
  rtl8211_get_phy_status(priv->base, RTL8211_ADDR, &st);
  if (st.enet_phy_link == (uint8_t)enet_phy_link_up)
    {
      hpm_mac_sync_phy_line(priv->base, &st);
    }
#elif defined(CONFIG_HPM_ENET_RMII) && CONFIG_HPM_ENET_RMII
  rtl8201_get_phy_status(priv->base, RTL8201_ADDR, &st);
  if (st.enet_phy_link == (uint8_t)enet_phy_link_up)
    {
      hpm_mac_sync_phy_line(priv->base, &st);
    }
#else
  return status_fail;
#endif

  enet_rx_resume(priv->base);
  return status_success;
}
#endif /* RGMII || RMII PHY */

/****************************************************************************
 * Function: hpm_enet_config
 *
 * Description:
 *  First successful ifup after boot: full board + MAC/DMA + PHY init.
 *  Later ifups: PHY MDIO sync + RX resume only (ENET core not re-init).
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int hpm_enet_config(struct hpm_enet_mac_s *priv)
{
  /* Same !cold_enet_init_done notion as cold_phy_reset in hpm_enet_init(). */

  const bool cold_boot_hw = !priv->cold_enet_init_done;
  hpm_stat_t hs;

#if (defined(CONFIG_HPM_ENET_RGMII) && CONFIG_HPM_ENET_RGMII) || \
    (defined(CONFIG_HPM_ENET_RMII) && CONFIG_HPM_ENET_RMII)
  if (priv->cold_enet_init_done)
    {
      hs = hpm_enet_ifup_resume_only(priv);
      if (hs != status_success)
        {
          return -EIO;
        }

      priv->txtail   = NULL;
      priv->inflight = 0;
      return OK;
    }
#endif

  if (cold_boot_hw)
    {
      board_reset_enet_phy(priv->base);

  #if defined(CONFIG_HPM_ENET_RGMII) && CONFIG_HPM_ENET_RGMII
      board_init_enet_rgmii_clock_delay(priv->base);
  #elif defined(CONFIG_HPM_ENET_RMII) && CONFIG_HPM_ENET_RMII
      board_init_enet_rmii_reference_clock(priv->base, CONFIG_RMII_REFCLK);
      hpm_enet_info("Reference Clock: %s\n",
                    CONFIG_RMII_REFCLK ? "Internal Clock" : "External Clock");
  #endif

      hpm_enet_info("Initialize the MAC and DMA\n");
    }
  else
    {
      hpm_enet_info("Warm ifup: re-init MAC/DMA only (PHY not reset)\n");
    }

  hs = hpm_enet_init(priv);
  if (hs != status_success)
    {
      return -EIO;
    }

  priv->cold_enet_init_done = true;

  priv->txtail             = NULL;
  priv->inflight           = 0;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: hpm_enet_initialize
 *
 * Description:
 *   Initialize the Ethernet driver for one interface.  If the HPM chip
 *   supports multiple Ethernet controllers, then board specific logic
 *   must implement riscv_netinitialize() and call this function to initialize
 *   the desired interfaces.
 *
 * Input Parameters:
 *   intf - In the case where there are multiple EMACs, this value
 *          identifies which EMAC is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int hpm_enet_initialize(int intf)
{
  struct hpm_enet_mac_s *priv;
  int ret;

  /* Get the interface structure associated with this interface number. */
  priv = &g_hpmethmac;

  /* Initialize the driver structure */
  memset(priv, 0, sizeof(struct hpm_enet_mac_s));
  priv->dev.d_ifup    = hpm_ifup;     /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = hpm_ifdown;   /* I/F down callback */
  priv->dev.d_txavail = hpm_txavail;  /* New TX data callback */
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = hpm_enet_ioctl;
#endif
  priv->dev.d_private = &g_hpmethmac;  /* Used to recover private state from dev */
  priv->base = ENET;

  /* Configure MAC Address */
  priv->dev.d_mac.ether.ether_addr_octet[0] = CONFIG_MAC_ADDR0;
  priv->dev.d_mac.ether.ether_addr_octet[1] = CONFIG_MAC_ADDR1;
  priv->dev.d_mac.ether.ether_addr_octet[2] = CONFIG_MAC_ADDR2;
  priv->dev.d_mac.ether.ether_addr_octet[3] = CONFIG_MAC_ADDR3;
  priv->dev.d_mac.ether.ether_addr_octet[4] = CONFIG_MAC_ADDR4;
  priv->dev.d_mac.ether.ether_addr_octet[5] = CONFIG_MAC_ADDR5;

  /* Configure GPIO pins to support Ethernet */
  hpm_enet_gpioconfig(priv);

  /* Attach the IRQ to the driver */
  if (irq_attach(ENET_IRQ, hpm_interrupt, NULL))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }
  /* Put the interface in the down state. */

  ret = hpm_ifdown(&priv->dev);
  if (ret < 0)
    {
      hpm_enet_err("Initialization of Ethernet block failed: %d\n", ret);
      return ret;
    }
  /* Register the device with the OS so that socket IOCTLs can be performed */
  netdev_register(&priv->dev, NET_LL_ETHERNET);
  return OK;
}

/****************************************************************************
 * Function: riscv_netinitialize
 *
 * Description:
 *   This is the "standard" network initialization logic called from the
 *   low-level initialization logic in arm_initialize.c.  If HPM_NETHERNET
 *   greater than one, then board specific logic will have to supply a
 *   version of riscv_netinitialize() that calls hpm_enet_initialize() with
 *   the appropriate interface number.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

void riscv_netinitialize(void)
{
    hpm_enet_initialize(0);
}