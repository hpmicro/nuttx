/****************************************************************************
 * arch/arm/src/hpmicro/hpm_can_sock.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *s
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
#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <nuttx/wqueue.h>
#include <nuttx/can.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/can.h>
#include <netpacket/can.h>

#if defined(CONFIG_NET_CAN_RAW_TX_DEADLINE) || defined(CONFIG_NET_TIMESTAMP)
#  include <sys/time.h>
#endif

#include "board.h"
#include "hpm_can_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_can_regs.h"
#include "hpm_can.h"

#if (defined(CONFIG_HPM_CAN_DRV)) && (defined CONFIG_NET_CAN || defined CONFIG_NET_CANFD) 

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CAN_FILTER_NUM_MAX       (16U)      /* CAN Hardware Filter number */

/* Pool configuration *******************************************************/

#define POOL_SIZE     (1)

#if defined(CONFIG_NET_CAN_RAW_TX_DEADLINE) || defined(CONFIG_NET_TIMESTAMP)
#define MSG_DATA                    sizeof(struct timeval)
#else
#define MSG_DATA                    0
#endif

#define MASKSTDID                   0x000007ff
#define MASKEXTID                   0x1fffffff
#define FLAGEFF                     (1 << 31) /* Extended frame format */
#define FLAGRTR                     (1 << 30) /* Remote transmission request */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct hpm_can_driver_s
{
  /* The constant configuration */

  uint8_t             port;
  CAN_Type *          can_base;                                  /* CAN Base address  */
  clock_name_t        clock_name;                                /* CAN clock name */
  int32_t             irq_num;                                   /* CAN IRQ index */
  uint32_t            fifo_index;                                /* FIFO index, it is a fake value to satisfy the driver framework */
  can_config_t        can_config;                                /* CAN configuration for IP */
  uint32_t            filter_num;                                /* Filter number */
  uint32_t            filter_index;                              /* Filter index */
  can_filter_config_t filter_list[CAN_FILTER_NUM_MAX];           /* Filter list */
  bool                tx_irq_enable;                             /* CAN TX interrupt enable*/
  bool                rx_irq_enable;                             /* CAN RX interrupt enable*/

  bool canfd_capable;

  bool bifup;               /* true:ifup false:ifdown */
  struct net_driver_s dev;  /* Interface understood by the network */

#ifdef TX_TIMEOUT_WQ
  WDOG_ID txtimeout[TXMBCOUNT]; /* TX timeout timer */
#endif
  struct work_s irqwork;            /* For deferring interrupt work to the wq */
  struct work_s pollwork;           /* For deferring poll work to the work wq */
  struct canfd_frame *txdesc_fd;    /* A pointer to the list of TX descriptor for FD frames */
  struct canfd_frame *rxdesc_fd;    /* A pointer to the list of RX descriptors for FD frames */
  struct can_frame *txdesc;         /* A pointer to the list of TX descriptor */
  struct can_frame *rxdesc;         /* A pointer to the list of RX descriptors */

};

  /* TX/RX pool */

#ifdef CONFIG_NET_CAN_CANFD
static uint8_t g_tx_pool[(sizeof(struct canfd_frame)+MSG_DATA)*POOL_SIZE];
static uint8_t g_rx_pool[(sizeof(struct canfd_frame)+MSG_DATA)*POOL_SIZE];
#else
static uint8_t g_tx_pool[sizeof(struct can_frame)*POOL_SIZE];
static uint8_t g_rx_pool[sizeof(struct can_frame)*POOL_SIZE];
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* CAN Register access */

static int  can_set_cfg(struct hpm_can_driver_s *dev);

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
static int  can_set_add_exdfilters(struct hpm_can_driver_s *dev, struct canioc_extfilter_s *filter);
static int  can_set_add_stdfilters(struct hpm_can_driver_s *dev, struct canioc_stdfilter_s *filter);
static int  can_set_del_exdfilters(struct hpm_can_driver_s *dev, struct canioc_extfilter_s *filter);
static int  can_set_del_stdfilters(struct hpm_can_driver_s *dev, struct canioc_stdfilter_s *filter);
#endif

static uint8_t can_get_data_bytes_from_dlc(uint32_t dlc);
/* CAN methods */
static void hpm_can_reset(struct hpm_can_driver_s *dev);
static int  hpm_can_setup(struct hpm_can_driver_s *dev);
static void hpm_can_shutdown(struct hpm_can_driver_s *dev);
static void hpm_can_rxint(struct hpm_can_driver_s *dev, bool enable);
static void hpm_can_txint(struct hpm_can_driver_s *dev, bool enable);
/* Common TX logic */

static int  hpm_can_send(struct hpm_can_driver_s *priv);
static int  hpm_can_txpoll(struct net_driver_s *dev);
static bool hpm_can_txready(struct hpm_can_driver_s *dev);
static void hpm_can_txdone_work(void *arg);
static void hpm_can_txdone(struct hpm_can_driver_s *priv);
static void hpm_can_rxinterrupt_work(void *arg);

/* NuttX callback functions */

static int  hpm_can_ifup(struct net_driver_s *dev);
static int  hpm_can_ifdown(struct net_driver_s *dev);

static void hpm_can_txavail_work(void *arg);
static int  hpm_can_txavail(struct net_driver_s *dev);

#ifdef CONFIG_NETDEV_IOCTL
static int  hpm_can_netdev_ioctl(struct net_driver_s *dev, int cmd,
                               unsigned long arg);
#endif

/* CAN interrupts */

static int hpm_can_interrupt(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_HPM_CAN0

/* CAN0 variable driver state */

static struct hpm_can_driver_s g_hpmcan0priv;

#endif

#ifdef CONFIG_HPM_CAN1

/* CAN1 variable driver state */

static struct hpm_can_driver_s g_hpmcan1priv;

#endif


#ifdef CONFIG_HPM_CAN2

/* CAN2 variable driver state */

static struct hpm_can_driver_s g_hpmcan2priv;

#endif


#ifdef CONFIG_HPM_CAN3

/* CAN3 variable driver state */

static struct hpm_can_driver_s g_hpmcan3priv;

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_can_interrupt
 *
 * Description:
 *   TWAI0 RX/TX interrupt handler
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *   arg - The pointer to driver structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int hpm_can_interrupt(int irq, void *context, void *arg)
{
  struct hpm_can_driver_s *priv = (struct hpm_can_driver_s *)arg;
  uint8_t error_flags;

  if (irq == priv->irq_num) 
    {
      uint8_t flags = can_get_tx_rx_flags(priv->can_base);
      if ((flags & CAN_EVENT_RECEIVE) != 0) 
        {

          hpm_can_rxint(priv, false);
          work_queue(LPWORK, &priv->irqwork,
                      hpm_can_rxinterrupt_work, priv, 0);
        }
      
      if ((flags & (CAN_EVENT_TX_PRIMARY_BUF | CAN_EVENT_TX_SECONDARY_BUF)))
       {

          /* Tell the upper half that the transfer is finished. */

          /* Disable further TX CAN interrupts. here can be no race
          * condition here.
          */

          hpm_can_txint(priv, false);
          work_queue(LPWORK, &priv->irqwork, hpm_can_txdone_work, priv, 0);
       }

      can_clear_tx_rx_flags(priv->can_base, flags);
      error_flags = can_get_error_interrupt_flags(priv->can_base);
      can_clear_error_interrupt_flags(priv->can_base, error_flags);
    }

  return OK;
}

/****************************************************************************
 * Name: can_set_cfg
 *
 * Description:
 *   Set the config of an CAN register.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the error.
 *
 ****************************************************************************/

static int can_set_cfg(struct hpm_can_driver_s *dev)
{

  dev->can_config.filter_list_num = dev->filter_num;
  dev->can_config.filter_list = &dev->filter_list[0];
  uint32_t can_clk = board_init_can_clock(dev->can_base);

  if(status_success == can_init(dev->can_base, &dev->can_config, can_clk))
    {
      return 0;
    }

  return -1;
}

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
/****************************************************************************
 * Name: can_set_add_exdfilters
 *
 * Description:
 *   Set the extern filter of an CAN .
 *
 * Input Parameters:
 *   dev     - An instance of the "upper half" CAN driver state structure.
 *   filter  - The configuration of the filter
 * Returned Value:
 *   A non-negative filter ID is returned on success.  Otherwise a negated
 *   errno value is returned to indicate the nature of the error.
 *
 ****************************************************************************/
static int can_set_add_exdfilters(struct hpm_can_driver_s *dev, struct canioc_extfilter_s *filter)
{
  struct hpm_can_driver_s *priv = (struct hpm_can_driver_s *)dev;
  int index = 0;

  if (filter == NULL)
    {
      canerr("ERROR: filter is null\n");
      priv->filter_num   = 0;
      priv->filter_index = 0;
      return -1;
    }
  
  if (priv->filter_num > CAN_FILTER_NUM_MAX)
    {
      canerr("ERROR: filter number max: %d\n", priv->filter_num);
      return -1;
    }
  
  index = priv->filter_index;
  priv->filter_list[index].index   = index;
  priv->filter_list[index].code    = filter->xf_id1;
  priv->filter_list[index].enable  = true;
  priv->filter_list[index].id_mode = can_filter_id_mode_extended_frames;

  if (filter->xf_type == CAN_FILTER_MASK)
    {
      priv->filter_list[index].mask = (~filter->xf_id2);
    }
  else if (filter->xf_type == CAN_FILTER_RANGE)
    {
      uint32_t num = (filter->xf_id2 - filter->xf_id1 + 1);
      uint32_t tmp;
      uint32_t mask;
      for (int i = 0; i < num; i++)                      /* Mask the result of the exclusive OR of each member of the code bit array */
        {
          tmp = (filter->xf_id1 + 1) ^ (~filter->xf_id1); /* Both perform NXOR operation with the first data member */
          mask &= tmp;
          mask = ~mask;
        }
      priv->filter_list[index].mask = mask;      
    }
  else
    {
      canerr("ERROR: the xf type is not support: %d\n", filter->xf_type);
      return -1;
    }
  
  priv->filter_num ++;
  priv->filter_index ++;

  return index;
}

/****************************************************************************
 * Name: can_set_add_stdfilters
 *
 * Description:
 *   Set the standard filter of an CAN .
 *
 * Input Parameters:
 *   dev     - An instance of the "upper half" CAN driver state structure.
 *   filter  - The configuration of the filter
 * Returned Value:
 *   A non-negative filter ID is returned on success.  Otherwise a negated
 *   errno value is returned to indicate the nature of the error.
 *
 ****************************************************************************/

static int can_set_add_stdfilters(struct can_dev_s *dev, struct canioc_stdfilter_s *filter)
{
  struct hpm_can_driver_s *priv = (struct hpm_can_driver_s *)dev;
  int index = 0;

  if (filter == NULL)
    {
      canerr("ERROR: filter is null\n");
      priv->filter_num   = 0;
      priv->filter_index = 0;
      return -1;
    }
  
  if (priv->filter_num > CAN_FILTER_NUM_MAX)
    {
      canerr("ERROR: filter number max: %d\n", priv->filter_num);
      return -1;
    }
  
  index = priv->filter_index;
  priv->filter_list[index].index   = index;
  priv->filter_list[index].code    = filter->xf_id1;
  priv->filter_list[index].enable  = true;
  priv->filter_list[index].id_mode = can_filter_id_mode_standard_frames;

  if (filter->xf_type == CAN_FILTER_MASK)
    {
      priv->filter_list[index].mask = (~filter->xf_id2);
    }
  else if (filter->xf_type == CAN_FILTER_RANGE)
    {
      uint32_t num = (filter->xf_id2 - filter->xf_id1 + 1);
      uint32_t tmp;
      uint32_t mask;
      for (int i = 0; i < num; i++)                      /* Mask the result of the exclusive OR of each member of the code bit array */
        {
          tmp = (filter->xf_id1 + 1) ^ (~filter->xf_id1); /* Both perform NXOR operation with the first data member */
          mask &= tmp;
          mask = ~mask;
        }
      priv->filter_list[index].mask = mask;      
    }
  else
    {
      canerr("ERROR: the xf type is not support: %d\n", filter->xf_type);
      return -1;
    }
  
  priv->filter_num ++;
  priv->filter_index ++;   

  return index;
}

/****************************************************************************
 * Name: can_set_del_filters
 *
 * Description:
 *   Remove an address filter
 *
 * Input Parameters:
 *   priv - An instance of the FDCAN driver state structure.
 *   filter  - The configuration of the filter
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the error.
 *
 ****************************************************************************/
static int  can_set_del_exdfilters(struct hpm_can_driver_s *dev, struct canioc_extfilter_s *filter)
{
  struct hpm_can_driver_s *priv = (struct hpm_can_driver_s *)dev;
  if (filter == NULL)
    {
      canerr("ERROR: filter is null\n");
      return -1;
    }
  
  /* TODO */
}

/****************************************************************************
 * Name: can_set_del_filters
 *
 * Description:
 *   Remove an address filter
 *
 * Input Parameters:
 *   priv - An instance of the FDCAN driver state structure.
 *   filter  - The configuration of the filter
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the error.
 *
 ****************************************************************************/

static int  can_set_del_stdfilters(struct hpm_can_driver_s *dev, struct canioc_stdfilter_s *filter)
{
  struct hpm_can_driver_s *priv = (struct hpm_can_driver_s *)dev;
  if (filter == NULL)
    {
      canerr("ERROR: filter is null\n");
      return -1;
    }
  
  /* TODO */
}
#endif

/****************************************************************************
 * Name: can_get_data_bytes_from_dlc
 *
 * Description:
 *   get data bytes len from dlc
 *
 * Input Parameters:
 *   dlc  - can dlc
 *
 * Returned Value:
 *   data bytes lens
 *
 ****************************************************************************/

static uint8_t can_get_data_bytes_from_dlc(uint32_t dlc)
{
    uint32_t data_bytes = 0;

    dlc &= 0xFU;
    if (dlc <= 8U) 
      {
          data_bytes = dlc;
      } 
    else 
      {
        switch (dlc) 
          {
            case can_payload_size_12:
                data_bytes = 12U;
                break;
            case can_payload_size_16:
                data_bytes = 16U;
                break;
            case can_payload_size_20:
                data_bytes = 20U;
                break;
            case can_payload_size_24:
                data_bytes = 24U;
                break;
            case can_payload_size_32:
                data_bytes = 32U;
                break;
            case can_payload_size_48:
                data_bytes = 48U;
                break;
            case can_payload_size_64:
                data_bytes = 64U;
                break;
            default:
                /* Code should never touch here */
                break;
          }
      }

    return data_bytes;
}

/****************************************************************************
 * Name: hpm_can_reset
 *
 * Description:
 *   Reset the can device.  Called early to initialize the hardware. This
 *   function is called, before hpm_setup() and on error conditions.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void hpm_can_reset(struct hpm_can_driver_s *dev)
{
  irqstate_t flags;

  caninfo("CAN%" PRIu8 "\n", dev->port);

  flags = enter_critical_section();
  can_reset(dev->can_base,true);
  up_disable_irq(dev->irq_num);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: hpm_can_setup
 *
 * Description:
 *   Configure the CAN. This method is called the first time that the CAN
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching CAN interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int hpm_can_setup(struct hpm_can_driver_s *dev)
{
  irqstate_t flags;
  int ret = OK;

  caninfo("CAN%" PRIu8 "\n", dev->port);

  flags = enter_critical_section();

  clock_enable(dev->clock_name);
  clock_add_to_group(dev->clock_name, BOARD_RUNNING_CORE);
  
  if (hpm_init_can_pins(dev->port) < 0)
    {
      return -1;
    }

  if (can_set_cfg(dev) < 0)
    {
      return -1;
    }
  
  ret = irq_attach(dev->irq_num, hpm_can_interrupt, dev);
  if (ret != OK)
    {
      leave_critical_section(flags);

      return ret;
    }

  up_enable_irq(dev->irq_num);

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: hpm_can_shutdown
 *
 * Description:
 *   Disable the CAN.  This method is called when the CAN device is closed.
 *   This method reverses the operation the setup method.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void hpm_can_shutdown(struct hpm_can_driver_s *dev)
{
  /* the interface not necessarily implement*/

#ifdef CONFIG_DEBUG_CAN_INFO
  caninfo("shutdown CAN%" PRIu8 "\n", priv->port);
#endif
 
  clock_remove_from_group(dev->clock_name, BOARD_RUNNING_CORE);
  clock_disable(dev->clock_name);

  /* Disable can interrupt */

  up_disable_irq(dev->irq_num);

  /* Dissociate the IRQ from the ISR */

  irq_detach(dev->irq_num);

}

/****************************************************************************
 * Name: hpm_can_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *   enable - Enable or disable receive interrupt.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void hpm_can_rxint(struct hpm_can_driver_s *dev, bool enable)
{
  irqstate_t flags;

  caninfo("CAN%" PRIu8 " enable: %d\n", dev->port, enable);

  flags = enter_critical_section();
  dev->rx_irq_enable   = enable;
  uint8_t irq_txrx_mask = CAN_EVENT_RECEIVE | CAN_EVENT_RX_BUF_ALMOST_FULL | CAN_EVENT_RX_BUF_FULL | CAN_EVENT_RX_BUF_OVERRUN;
  (enable == true) ? (can_enable_tx_rx_irq(dev->can_base, irq_txrx_mask)) : (can_disable_tx_rx_irq(dev->can_base, irq_txrx_mask));
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: hpm_can_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *   enable - Enable or disable transmit interrupt.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void hpm_can_txint(struct hpm_can_driver_s *dev, bool enable)
{
  irqstate_t flags;

  caninfo("CAN%" PRIu8 " enable: %d\n", dev->port, enable);

  flags = enter_critical_section();
  dev->tx_irq_enable   = enable;
  uint8_t irq_txrx_mask = CAN_EVENT_TX_PRIMARY_BUF | CAN_EVENT_TX_SECONDARY_BUF;
  (enable == true) ? (can_enable_tx_rx_irq(dev->can_base, irq_txrx_mask)) : (can_disable_tx_rx_irq(dev->can_base, irq_txrx_mask));
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: hpm_can_txready
 *
 * Description:
 *   Return true if the CAN hardware can accept another TX message.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *
 * Returned Value:
 *   True if the CAN hardware is ready to accept another TX message.
 *
 ****************************************************************************/

static bool hpm_can_txready(struct hpm_can_driver_s *dev)
{
  if (can_is_secondary_transmit_buffer_full(dev->can_base) == true) 
  {
    return false;
  }
  
  return true;
}

/****************************************************************************
 * Name: hpm_can_txdone_work
 ****************************************************************************/

static void hpm_can_txdone_work(void *arg)
{
  struct hpm_can_driver_s *priv = (struct hpm_can_driver_s *)arg;

  hpm_can_txdone(priv);

  /* There should be space for a new TX in any event.  Poll the network for
   * new XMIT data
   */

  net_lock();
  devif_poll(&priv->dev, hpm_can_txpoll);
  net_unlock();
}

/****************************************************************************
 * Name: hpm_can_rxinterrupt_work
 ****************************************************************************/

static void hpm_can_rxinterrupt_work(void *arg)
{
  struct hpm_can_driver_s *priv = (struct hpm_can_driver_s *)arg;
  can_receive_buf_t s_can_rx_buf;

# ifdef CONFIG_NET_CAN_CANFD
  uint32_t i;
# endif

  can_read_received_message(priv->can_base, (can_receive_buf_t *)&s_can_rx_buf);

#ifdef CONFIG_NET_CAN_CANFD
  if (s_can_rx_buf.canfd_frame) /* CAN FD frame */
    {
    struct canfd_frame *frame = (struct canfd_frame *)priv->rxdesc_fd;

      if (s_can_rx_buf.extend_id)
        {
          frame->can_id = MASKEXTID & s_can_rx_buf.id;
          frame->can_id |= CAN_EFF_FLAG;
        }
      else
        {
          frame->can_id = MASKSTDID & s_can_rx_buf.id;
        }

      if (s_can_rx_buf.remote_frame)
        {
          frame->can_id |= CAN_RTR_FLAG;
        }

      frame->len = can_dlc_to_len[s_can_rx_buf.dlc];

      for (i = 0; i < frame->len ; i++)
        {
          frame->data[i] = s_can_rx_buf.data[i];
        }

      /* Copy the buffer pointer to priv->dev..  Set amount of data
        * in priv->dev.d_len
        */

      priv->dev.d_len = sizeof(struct canfd_frame);
      priv->dev.d_buf = (uint8_t *)frame;
    }
  else /* CAN 2.0 Frame */
#endif
    {
    struct can_frame *frame = (struct can_frame *)priv->rxdesc;

      if (s_can_rx_buf.extend_id)
        {
          frame->can_id = MASKEXTID & s_can_rx_buf.id;
          frame->can_id |= CAN_EFF_FLAG;
        }
      else
        {
          frame->can_id = MASKSTDID & s_can_rx_buf.id;
        }

      if (s_can_rx_buf.remote_frame)
        {
          frame->can_id |= CAN_RTR_FLAG;
        }

      frame->can_dlc = s_can_rx_buf.dlc;

      for (i = 0; i < frame->can_dlc; i++)
      {
        frame->data[i] = s_can_rx_buf.data[i];
      }
      
      /* Copy the buffer pointer to priv->dev..  Set amount of data
        * in priv->dev.d_len
        */

      priv->dev.d_len = sizeof(struct can_frame);
      priv->dev.d_buf = (uint8_t *)frame;
    }
  /* Send to socket interface */

  NETDEV_RXPACKETS(&priv->dev);

  can_input(&priv->dev);

  /* Point the packet buffer back to the next Tx buffer that will be
  * used during the next write.  If the write queue is full, then
  * this will point at an active buffer, which must not be written
  * to.  This is OK because devif_poll won't be called unless the
  * queue is not full.
  */

  if (priv->canfd_capable)
    {
      priv->dev.d_buf = (uint8_t *)priv->txdesc_fd;
    }
  else
    {
      priv->dev.d_buf = (uint8_t *)priv->txdesc;
    }

  hpm_can_rxint(priv, true);

}

/****************************************************************************
 * Name: hpm_can_txdone
 ****************************************************************************/

static void hpm_can_txdone(struct hpm_can_driver_s *priv)
{
  hpm_can_txint(priv, true);

  NETDEV_TXDONE(&priv->dev);
}

/****************************************************************************
 * Name: hpm_can_send
 *
 * Description:
 *    Send one CAN message.
 *
 *    One CAN-message consists of a maximum of 10 bytes.  A message is
 *    composed of at least the first 2 bytes (when there are no data bytes).
 *
 *    Byte 0:      Bits 0-7: Bits 3-10 of the 11-bit CAN identifier
 *    Byte 1:      Bits 5-7: Bits 0-2 of the 11-bit CAN identifier
 *                 Bit 4:    Remote Transmission Request (RTR)
 *                 Bits 0-3: Data Length Code (DLC)
 *    Bytes 2-10: CAN data
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *   msg - A message to send.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int hpm_can_send(struct hpm_can_driver_s *dev)
{
  can_transmit_buf_t tx_buf = { 0 };
  uint32_t len;
  int ret = OK;
  irqstate_t flags;

  flags = enter_critical_section();

  if (dev->dev.d_len == sizeof(struct can_frame))
    {
      struct can_frame *frame = (struct can_frame *)dev->dev.d_buf;

      if (frame->can_id & CAN_EFF_FLAG)
        {
          tx_buf.extend_id = true;
          tx_buf.id        = frame->can_id & MASKEXTID;
        }  
      else
        {
          tx_buf.extend_id = false;
          tx_buf.id        = frame->can_id & MASKSTDID;

        }

      if (frame->can_id & CAN_RTR_FLAG)
        {
          tx_buf.remote_frame = true;
        }
      else
        {
          tx_buf.remote_frame = false;
        }

      len = can_get_data_bytes_from_dlc((uint32_t)frame->can_dlc);

      if (len > 8)
        {
          len = 8;
        }

      for (uint32_t i = 0; i < len; i++)
        {
            tx_buf.data[i] = frame->data[i];
        }
      tx_buf.dlc = frame->can_dlc;

      can_send_message_nonblocking(dev->can_base, &tx_buf);      
    }
#ifdef CONFIG_NET_CAN_CANFD
  else /* CAN FD frame */
    {
      struct canfd_frame *frame = (struct canfd_frame *)dev->dev.d_buf;

      tx_buf.canfd_frame    = 1;  /* CAN FD Frame */
      // tx_buf.bitrate_switch = 1;

      if (frame->can_id & CAN_EFF_FLAG)
        {
          tx_buf.extend_id = true;
          tx_buf.id        = frame->can_id & MASKEXTID;
        }  
      else
        {
          tx_buf.extend_id = false;
          tx_buf.id        = frame->can_id & MASKSTDID;

        }

      if (frame->can_id & CAN_RTR_FLAG )
        {
          tx_buf.remote_frame = true;
        }
      else
        {
          tx_buf.remote_frame = false;
        }

      len = frame->len;

      if (len > 64)
        {
          len = 64;
        }

      for (uint32_t i = 0; i < len; i++)
        {
            tx_buf.data[i] = frame->data[i];
        }
      tx_buf.dlc = len_to_can_dlc[frame->len];

      can_send_message_nonblocking(dev->can_base, &tx_buf);  
    }
#endif

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Function: hpm_can_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
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

static int hpm_can_txpoll(struct net_driver_s *dev)
{
  struct hpm_can_driver_s *priv =
    (struct hpm_can_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      hpm_can_txdone(priv);

      /* Send the packet */

      hpm_can_send(priv);

      /* Check if there is room in the device to hold another packet. If
       * not, return a non-zero value to terminate the poll.
       */

      if (hpm_can_txready(priv) == false)
        {
          return -EBUSY;
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: hpm_can_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int hpm_can_ifup(struct net_driver_s *dev)
{
    struct hpm_can_driver_s *priv =
    (struct hpm_can_driver_s *)dev->d_private;

  DEBUGASSERT(priv);

  /* Setup CAN */

  hpm_can_setup(priv);

  priv->bifup = true;

  priv->txdesc = (struct can_frame *)&g_tx_pool;
  priv->rxdesc = (struct can_frame *)&g_rx_pool;
  if (priv->canfd_capable)
    {
      priv->txdesc_fd = (struct canfd_frame *)&g_tx_pool;
      priv->rxdesc_fd = (struct canfd_frame *)&g_rx_pool;
      priv->dev.d_buf = (uint8_t *)priv->txdesc_fd;
    }
  else
    {
      priv->dev.d_buf = (uint8_t *)priv->txdesc;
    }
  return OK;
}

/****************************************************************************
 * Function: hpm_can_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int hpm_can_ifdown(struct net_driver_s *dev)
{
  struct hpm_can_driver_s *priv =
  (struct hpm_can_driver_s *)dev->d_private;

  /* Disable CAN interrupts */

  hpm_can_shutdown(priv);

  /* Reset CAN */

  hpm_can_reset(priv);

  priv->bifup = false;

  return OK;
}

/****************************************************************************
 * Function: hpm_can_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Input Parameters:
 *   arg - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void hpm_can_txavail_work(void *arg)
{
    struct hpm_can_driver_s *priv = (struct hpm_can_driver_s *)arg;

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->bifup)
    {
      /* Check if there is room in the hardware to hold another outgoing
       * packet.
       */

      if (hpm_can_txready(priv))
        {
          /* No, there is space for another transfer.  Poll the network for
           * new XMIT data.
           */

          devif_poll(&priv->dev, hpm_can_txpoll);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Function: hpm_can_txavail
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

static int hpm_can_txavail(struct net_driver_s *dev)
{
  struct hpm_can_driver_s *priv = (struct hpm_can_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      hpm_can_txavail_work(priv);
    }

  return OK;
}

/****************************************************************************
 * Function: hpm_can_netdev_ioctl
 *
 * Description:
 *   PHY ioctl command handler
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   cmd  - ioctl command
 *   arg  - Argument accompanying the command
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int  hpm_can_netdev_ioctl(struct net_driver_s *dev, int cmd,
                               unsigned long arg)
{
  struct hpm_can_driver_s *priv =
    (struct hpm_can_driver_s *)dev->d_private;
  int                       ret  = OK;

  DEBUGASSERT(priv);
  switch (cmd)
    {
#ifdef CONFIG_NETDEV_CAN_BITRATE_IOCTL
      case SIOCGCANBITRATE: /* Get bitrate from a CAN controller */
        {
          struct can_ioctl_data_s *req =
              (struct can_ioctl_data_s *)((uintptr_t)arg);
          req->arbi_bitrate = priv->can_config.can_config.baudrate / 1000; /* kbit/s */
          req->arbi_samplep = priv->can_config.can_config.can20_samplepoint_min;
          if (priv->canfd_capable)
            {
              req->data_bitrate = priv->can_config.can_config.baudrate_fd / 1000; /* kbit/s */
              req->data_samplep = priv->can_config.can_config.canfd_samplepoint_min;
            }
          else
            {
              req->data_bitrate = 0;
              req->data_samplep = 0;
            }

          ret = OK;
        }
        break;

      case SIOCSCANBITRATE: /* Set bitrate of a CAN controller */
        {
          struct can_ioctl_data_s *req =
              (struct can_ioctl_data_s *)((uintptr_t)arg);

          priv->can_config.can_config.baudrate.bitrate      = req->arbi_bitrate * 1000;
          priv->can_config.can_config.can20_samplepoint_min = req->arbi_samplep;
          priv->can_config.can_config.can20_samplepoint_max = req->arbi_samplep;
          if (priv->canfd_capable)
          {
            priv->can_config.can_config.baudrate.baudrate_fd  = req->data_bitrate * 1000;
            priv->can_config.can_config.canfd_samplepoint_min = req->data_samplep;
            priv->can_config.can_config.canfd_samplepoint_max = req->data_samplep;
          }

          /* Reset CAN controller and start with new timings */

          ret = hpm_can_ifup(dev);
        }
        break;
#endif

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
      case SIOCACANEXTFILTER:
        {
          can_set_add_exdfilters(priv, (struct canioc_extfilter_s *)arg);
          can_set_cfg(priv);
        }
        break;

      case SIOCDCANEXTFILTER:
        {
          can_set_add_stdfilters(priv, (struct canioc_extfilter_s *)arg);
          can_set_cfg(priv);
        }
        break;

      case SIOCACANSTDFILTER:
        {
          can_set_del_exdfilters(priv, (struct canioc_stdfilter_s *)arg);
          can_set_cfg(priv);
        }
        break;

      case SIOCDCANSTDFILTER:
        {
          can_set_del_stdfilters(priv, (struct canioc_stdfilter_s *)arg);
          can_set_cfg(priv);
        }
        break;
#endif

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}
#endif  /* CONFIG_NETDEV_IOCTL */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_cansockinitialize
 *
 * Description:
 *   Initialize the selected FDCAN port as CAN socket interface
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple FDCAN interfaces)
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int hpm_cansockinitialize(int port)
{
  struct hpm_can_driver_s   *priv   = NULL;
  int                        ret    = OK;

  ninfo("CAN%d\n", port);

  /* Select CAN peripheral to be initialized */

#ifdef CONFIG_HPM_CAN0
  if (port == 0)
    {
      /* Select the CAN0 device structure */

      can_get_default_config(&g_hpmcan0priv.can_config);

      priv                                         = &g_hpmcan0priv;     
      priv->can_base                               = HPM_CAN0;
      priv->irq_num                                = HPM_IRQn_CAN0;
      priv->clock_name                             = clock_can0;
      priv->port                                   = 0;
#  ifdef CONFIG_HPM_CAN0_USER_SET_TINING

      /* Assume the CAN clock is 80MHz, configure the nominal baudrate to 500kbit/s, configure the canfd baudrate to 5Mbit/s */

      priv->can_config.use_lowlevel_timing_setting = true;

      /* bitrate = can_freq / prescale / (seq1 + seg2) */

      priv->can_config.can_timing.num_seg1         = CONFIG_HPM_CAN0_NUM_SEG1;
      priv->can_config.can_timing.num_seg2         = CONFIG_HPM_CAN0_NUM_SEG2;
      priv->can_config.can_timing.num_sjw          = CONFIG_HPM_CAN0_NUM_SJW;
      priv->can_config.can_timing.prescaler        = CONFIG_HPM_CAN0_PRESCALER;
      priv->can_config.canfd_timing.num_seg1       = CONFIG_HPM_CAN0_FD_NUM_SEG1;
      priv->can_config.canfd_timing.num_seg2       = CONFIG_HPM_CAN0_FD_NUM_SEG2;
      priv->can_config.canfd_timing.num_sjw        = CONFIG_HPM_CAN0_FD_NUM_SJW;
      priv->can_config.canfd_timing.prescaler      = CONFIG_HPM_CAN0_FD_PRESCALER;
#  else
      priv->can_config.can20_samplepoint_min       = CONFIG_HPM_CAN0_MIN_SAMPLEPOINT;
      priv->can_config.can20_samplepoint_max       = CONFIG_HPM_CAN0_MAX_SAMPLEPOINT;
      priv->can_config.baudrate                    = CONFIG_HPM_CAN0_BAUDRATE;

#    ifdef CONFIG_NET_CAN_CANFD
      priv->can_config.canfd_samplepoint_min       = CONFIG_HPM_CAN0_FD_MIN_SAMPLEPOINT;
      priv->can_config.canfd_samplepoint_max       = CONFIG_HPM_CAN0_FD_MAX_SAMPLEPOINT;
      priv->can_config.enable_canfd                = true;
      priv->can_config.baudrate_fd                 = CONFIG_HPM_CAN0_FD_BAUDRATE;
#    endif

#  endif
      priv->can_config.enable_can_fd_iso_mode     = CONFIG_HPM_CAN0_FD_ISO_MODE;
      priv->can_config.mode                       = CONFIG_HPM_CAN0_MODE;
    }
  else
#endif
#ifdef CONFIG_HPM_CAN1
  if (port == 1)
    {
      /* Select the CAN1 device structure */

      can_get_default_config(&g_hpmcan1priv.can_config);

      priv                                         = &g_hpmcan1priv;
      priv->can_base                               = HPM_CAN1;
      priv->irq_num                                = HPM_IRQn_CAN1;
      priv->clock_name                             = clock_can1;
      priv->port                                   = 1;
#  ifdef CONFIG_HPM_CAN1_USER_SET_TINING
      /* Assume the CAN clock is 80MHz, configure the nominal baudrate to 500kbit/s, configure the canfd baudrate to 5Mbit/s */

      priv->can_config.use_lowlevel_timing_setting = true;

      /* bitrate = can_freq / prescale / (seq1 + seg2) */

      priv->can_config.can_timing.num_seg1         = CONFIG_HPM_CAN1_NUM_SEG1;
      priv->can_config.can_timing.num_seg2         = CONFIG_HPM_CAN1_NUM_SEG2;
      priv->can_config.can_timing.num_sjw          = CONFIG_HPM_CAN1_NUM_SJW;
      priv->can_config.can_timing.prescaler        = CONFIG_HPM_CAN1_PRESCALER;
      priv->can_config.canfd_timing.num_seg1       = CONFIG_HPM_CAN1_FD_NUM_SEG1;
      priv->can_config.canfd_timing.num_seg2       = CONFIG_HPM_CAN1_FD_NUM_SEG2;
      priv->can_config.canfd_timing.num_sjw        = CONFIG_HPM_CAN1_FD_NUM_SJW;
      priv->can_config.canfd_timing.prescaler      = CONFIG_HPM_CAN1_FD_PRESCALER;
#  else
      priv->can_config.can20_samplepoint_min       = CONFIG_HPM_CAN1_MIN_SAMPLEPOINT;
      priv->can_config.can20_samplepoint_max       = CONFIG_HPM_CAN1_MAX_SAMPLEPOINT;
      priv->can_config.baudrate                    = CONFIG_HPM_CAN1_BAUDRATE;

#    ifdef CONFIG_NET_CAN_CANFD
      priv->can_config.canfd_samplepoint_min       = CONFIG_HPM_CAN1_FD_MIN_SAMPLEPOINT;
      priv->can_config.canfd_samplepoint_max       = CONFIG_HPM_CAN1_FD_MAX_SAMPLEPOINT;
      priv->can_config.enable_canfd                = true;
      priv->can_config.baudrate_fd                 = CONFIG_HPM_CAN1_FD_BAUDRATE;
#    endif

#  endif
      priv->can_config.enable_can_fd_iso_mode     = CONFIG_HPM_CAN1_FD_ISO_MODE;
      priv->can_config.mode                       = CONFIG_HPM_CAN1_MODE;
    }
  else
#endif
#ifdef CONFIG_HPM_CAN2
  if (port == 2)
    {
      /* Select the CAN2 device structure */

      can_get_default_config(&g_hpmcan2priv.can_config);

      priv                                        = &g_hpmcan2priv;
      priv->can_base                              = HPM_CAN2;
      priv->irq_num                               = HPM_IRQn_CAN2;
      priv->clock_name                            = clock_can2;
      priv->port                                  = 2;
#  ifdef CONFIG_HPM_CAN2_USER_SET_TINING

      /* Assume the CAN clock is 80MHz, configure the nominal baudrate to 500kbit/s, configure the canfd baudrate to 5Mbit/s */

      priv->can_config.use_lowlevel_timing_setting = true;

      /* bitrate = can_freq / prescale / (seq1 + seg2) */

      priv->can_config.can_timing.num_seg1         = CONFIG_HPM_CAN2_NUM_SEG1;
      priv->can_config.can_timing.num_seg2         = CONFIG_HPM_CAN2_NUM_SEG2;
      priv->can_config.can_timing.num_sjw          = CONFIG_HPM_CAN2_NUM_SJW;
      priv->can_config.can_timing.prescaler        = CONFIG_HPM_CAN2_PRESCALER;
      priv->can_config.canfd_timing.num_seg1       = CONFIG_HPM_CAN2_FD_NUM_SEG1;
      priv->can_config.canfd_timing.num_seg2       = CONFIG_HPM_CAN2_FD_NUM_SEG2;
      priv->can_config.canfd_timing.num_sjw        = CONFIG_HPM_CAN2_FD_NUM_SJW;
      priv->can_config.canfd_timing.prescaler      = CONFIG_HPM_CAN2_FD_PRESCALER;
#  else
      priv->can_config.can20_samplepoint_min       = CONFIG_HPM_CAN2_MIN_SAMPLEPOINT;
      priv->can_config.can20_samplepoint_max       = CONFIG_HPM_CAN2_MAX_SAMPLEPOINT;
      priv->can_config.baudrate                    = CONFIG_HPM_CAN2_BAUDRATE;

#    ifdef CONFIG_NET_CAN_CANFD
      priv->can_config.canfd_samplepoint_min       = CONFIG_HPM_CAN2_FD_MIN_SAMPLEPOINT;
      priv->can_config.canfd_samplepoint_max       = CONFIG_HPM_CAN2_FD_MAX_SAMPLEPOINT;
      priv->can_config.enable_canfd                = true;
      priv->can_config.baudrate_fd                 = CONFIG_HPM_CAN2_FD_BAUDRATE;
#    endif

#  endif
      priv->can_config.enable_can_fd_iso_mode     = CONFIG_HPM_CAN2_FD_ISO_MODE;
      priv->can_config.mode                       = CONFIG_HPM_CAN2_MODE;
    }
  else
#endif
#ifdef CONFIG_HPM_CAN3
  if (port == 3)
    {
      /* Select the CAN3 device structure */

      can_get_default_config(&g_hpmcan3priv.can_config);

      priv                                        = &g_hpmcan3priv;
      priv->can_base                              = HPM_CAN3;
      priv->irq_num                               = HPM_IRQn_CAN3;
      priv->clock_name                            = clock_can3;
      priv->port                                  = 3;
#  ifdef CONFIG_HPM_CAN3_USER_SET_TINING

      /* Assume the CAN clock is 80MHz, configure the nominal baudrate to 500kbit/s, configure the canfd baudrate to 5Mbit/s */

      priv->can_config.use_lowlevel_timing_setting = true;

      /* bitrate = can_freq / prescale / (seq1 + seg2) */

      priv->can_config.can_timing.num_seg1         = CONFIG_HPM_CAN3_NUM_SEG1;
      priv->can_config.can_timing.num_seg2         = CONFIG_HPM_CAN3_NUM_SEG2;
      priv->can_config.can_timing.num_sjw          = CONFIG_HPM_CAN3_NUM_SJW;
      priv->can_config.can_timing.prescaler        = CONFIG_HPM_CAN3_PRESCALER;
      priv->can_config.canfd_timing.num_seg1       = CONFIG_HPM_CAN3_FD_NUM_SEG1;
      priv->can_config.canfd_timing.num_seg2       = CONFIG_HPM_CAN3_FD_NUM_SEG2;
      priv->can_config.canfd_timing.num_sjw        = CONFIG_HPM_CAN3_FD_NUM_SJW;
      priv->can_config.canfd_timing.prescaler      = CONFIG_HPM_CAN3_FD_PRESCALER;
#  else
      priv->can_config.can20_samplepoint_min       = CONFIG_HPM_CAN3_MIN_SAMPLEPOINT;
      priv->can_config.can20_samplepoint_max       = CONFIG_HPM_CAN3_MAX_SAMPLEPOINT;
      priv->can_config.baudrate                    = CONFIG_HPM_CAN3_BAUDRATE;

#    ifdef CONFIG_NET_CAN_CANFD
      priv->can_config.canfd_samplepoint_min       = CONFIG_HPM_CAN3_FD_MIN_SAMPLEPOINT;
      priv->can_config.canfd_samplepoint_max       = CONFIG_HPM_CAN3_FD_MAX_SAMPLEPOINT;
      priv->can_config.enable_canfd                = true;
      priv->can_config.baudrate_fd                 = CONFIG_HPM_CAN3_FD_BAUDRATE;
#    endif

#  endif
      priv->can_config.enable_can_fd_iso_mode     = CONFIG_HPM_CAN3_FD_ISO_MODE;
      priv->can_config.mode                       = CONFIG_HPM_CAN3_MODE;
    }
  else
#endif
    {
      nerr("ERROR: Unsupported port %d\n", port);
      ret = -EINVAL;
      goto errout;
    }

#ifdef CONFIG_NET_CAN_CANFD
  priv->canfd_capable = true;
#else
  priv->canfd_capable = false;
#endif

  priv->filter_num                            = 0;
  priv->filter_index                          = 0;
  priv->can_config.enable_tdc                 = true;
  priv->fifo_index                            = 0;
  priv->rx_irq_enable                         = 0;
  priv->tx_irq_enable                         = 0;

  /* Initialize the driver structure */

  priv->dev.d_ifup    = hpm_can_ifup;
  priv->dev.d_ifdown  = hpm_can_ifdown;
  priv->dev.d_txavail = hpm_can_txavail;
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = hpm_can_netdev_ioctl;
#endif
  priv->dev.d_private = priv;

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling fdcan_ifdown().
   */

  ninfo("callbacks done\n");

  hpm_can_ifdown(&priv->dev);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  ret = netdev_register(&priv->dev, NET_LL_CAN);

errout:
  return ret;
}

/****************************************************************************
 * Name: riscv_netinitialize
 *
 * Description:
 *   Initialize the first network interface.  If there are more than one
 *   interface in the chip, then board-specific logic will have to provide
 *   this function to determine which, if any, Ethernet controllers should
 *   be initialized.
 *
 ****************************************************************************/

#if !defined(CONFIG_NETDEV_LATEINIT)
void riscv_netinitialize(void)
{
#ifdef CONFIG_HPM_CAN0
  hpm_cansockinitialize(0);
#endif

#ifdef CONFIG_HPM_CAN1
  hpm_cansockinitialize(1);
#endif

#ifdef CONFIG_HPM_CAN2
  hpm_cansockinitialize(2);
#endif

#ifdef CONFIG_HPM_CAN3
  hpm_cansockinitialize(3);
#endif
}

#endif

#endif
