/****************************************************************************
 * arch/risc-v/src/hpmicro/hpm6750/hpm6750_can.c
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

#include <stdio.h>
#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/can/can.h>

#include "riscv_internal.h"

#include "board.h"
#include "hpm_can_drv.h"
#include "hpm_can_regs.h"
#include "hpm_clock_drv.h"
#include "hpm_can.h"

#if defined(CONFIG_HPM6750_CAN0) || defined(CONFIG_HPM6750_CAN1) || \
    defined(CONFIG_HPM6750_CAN2) || defined(CONFIG_HPM6750_CAN3)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define CAN_SEND_WAIT_MS_MAX     (1000U)    /* CAN maximum wait time for transmission */
#define CAN_SENDBOX_NUM          (1U)       /* CAN Hardware Transmission buffer number */
#define CAN_FILTER_NUM_MAX       (16U)      /* CAN Hardware Filter number */

#define CAN_SAMPLEPOINT_MIN      (750U)
#define CAN_SAMPLEPOINT_MAX      (875U)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* CAN hardware-dependent bit-timing constant
 * Used for calculating and checking bit-timing parameters
 */

struct hpmcan_dev_s
{
  /* Device configuration */

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
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* CAN Register access */

static int  can_set_cfg(struct can_dev_s *dev);
static int  can_set_add_exdfilters(struct can_dev_s *dev, struct canioc_extfilter_s *filter);
static int  can_set_add_stdfilters(struct can_dev_s *dev, struct canioc_stdfilter_s *filter);
static int  can_set_del_filters(struct can_dev_s *dev, int ndx);
static uint8_t can_get_data_bytes_from_dlc(uint32_t dlc);

/* CAN methods */

static void hpm_can_reset(struct can_dev_s *dev);
static int  hpm_can_setup(struct can_dev_s *dev);
static void hpm_can_shutdown(struct can_dev_s *dev);
static void hpm_can_rxint(struct can_dev_s *dev, bool enable);
static void hpm_can_txint(struct can_dev_s *dev, bool enable);
static int  hpm_can_ioctl(struct can_dev_s *dev, int cmd,
                              unsigned long arg);
static int  hpm_can_remoterequest(struct can_dev_s *dev,
                                      uint16_t id);
static int  hpm_can_send(struct can_dev_s *dev,
                             struct can_msg_s *msg);
static bool hpm_can_txready(struct can_dev_s *dev);
static bool hpm_can_txempty(struct can_dev_s *dev);

/* CAN interrupts */

static int hpm_can_interrupt(int irq, void *context, void *arg);

/* CAN acceptance filter */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_HPM6750_CAN0

static const struct can_ops_s g_can0ops       =
{
  .co_reset                                   = hpm_can_reset,
  .co_setup                                   = hpm_can_setup,
  .co_shutdown                                = hpm_can_shutdown,
  .co_rxint                                   = hpm_can_rxint,
  .co_txint                                   = hpm_can_txint,
  .co_ioctl                                   = hpm_can_ioctl,
  .co_remoterequest                           = hpm_can_remoterequest,
  .co_send                                    = hpm_can_send,
  .co_txready                                 = hpm_can_txready,
  .co_txempty                                 = hpm_can_txempty,
};

static struct hpmcan_dev_s g_can0priv         =
{
  .can_base                                   = HPM_CAN0,
  .irq_num                                    = IRQn_CAN0 + HPM_IRQ_PERI_START,
  .clock_name                                 = clock_can0,
  .port                                       = 0,
  .fifo_index                                 = 0,
  .filter_num                                 = 0,
  .filter_index                               = 0,
  .can_config.enable_tdc                      = true,
#  ifdef CONFIG_HPM6750_CAN0_USER_SET_TINING

  /* Assume the CAN clock is 80MHz, configure the nominal baudrate to 500kbit/s, configure the canfd baudrate to 5Mbit/s */

  .can_config.use_lowlevel_timing_setting     = true;
  /* bitrate = can_freq / prescale / (seq1 + seg2) */

  .can_config.can_timing.num_seg1             = CONFIG_HPM6750_CAN0_NUM_SEG1;
  .can_config.can_timing.num_seg2             = CONFIG_HPM6750_CAN0_NUM_SEG2;
  .can_config.can_timing.num_sjw              = CONFIG_HPM6750_CAN0_NUM_SJW;
  .can_config.can_timing.prescaler            = CONFIG_HPM6750_CAN0_PRESCALER;
#    ifdef CONFIG_CAN_FD
  .can_config.canfd_timing.num_seg1           = CONFIG_HPM6750_CAN0_FD_NUM_SEG1;
  .can_config.canfd_timing.num_seg2           = CONFIG_HPM6750_CAN0_FD_NUM_SEG2;
  .can_config.canfd_timing.num_sjw            = CONFIG_HPM6750_CAN0_FD_NUM_SJW;
  .can_config.canfd_timing.prescaler          = CONFIG_HPM6750_CAN0_FD_PRESCALER;
#    endif
#  else
  .can_config.can20_samplepoint_min           = CONFIG_HPM6750_CAN0_MIN_SAMPLEPOINT,
  .can_config.can20_samplepoint_max           = CONFIG_HPM6750_CAN0_MAX_SAMPLEPOINT,
  .can_config.baudrate                        = CONFIG_HPM6750_CAN0_BAUDRATE,
#  ifdef CONFIG_CAN_FD
  .can_config.canfd_samplepoint_min           = CONFIG_HPM6750_CAN0_FD_MIN_SAMPLEPOINT,
  .can_config.canfd_samplepoint_max           = CONFIG_HPM6750_CAN0_FD_MAX_SAMPLEPOINT,
  .can_config.enable_canfd                    = true,
  .can_config.baudrate_fd                     = CONFIG_HPM6750_CAN0_FD_BAUDRATE,
#  endif

#endif
  .can_config.disable_re_transmission_for_ptb = false,
  .can_config.disable_re_transmission_for_stb = false,
  .can_config.enable_self_ack                 = false,
  .can_config.enable_tx_buffer_priority_mode  = false,
  .can_config.enable_tdc                      = false,
  .can_config.irq_txrx_enable_mask            = false,
  .can_config.irq_error_enable_mask           = false,
  .can_config.enable_can_fd_iso_mode          = CONFIG_HPM6750_CAN0_FD_ISO_MODE,
  .can_config.mode                            = CONFIG_HPM6750_CAN0_MODE,
  .fifo_index                                 = 0,
  .rx_irq_enable                              = 0,
  .tx_irq_enable                              = 0,
};

static struct can_dev_s g_can0dev =
{
  .cd_ops                                     = &g_can0ops,
  .cd_priv                                    = &g_can0priv,
};
#endif

#ifdef CONFIG_HPM6750_CAN1

static const struct can_ops_s g_can1ops       =
{
  .co_reset                                   = hpm_can_reset,
  .co_setup                                   = hpm_can_setup,
  .co_shutdown                                = hpm_can_shutdown,
  .co_rxint                                   = hpm_can_rxint,
  .co_txint                                   = hpm_can_txint,
  .co_ioctl                                   = hpm_can_ioctl,
  .co_remoterequest                           = hpm_can_remoterequest,
  .co_send                                    = hpm_can_send,
  .co_txready                                 = hpm_can_txready,
  .co_txempty                                 = hpm_can_txempty,
};

static struct hpmcan_dev_s g_can1priv         =
{
  .can_base                                   = HPM_CAN1,
  .irq_num                                    = IRQn_CAN1 + HPM_IRQ_PERI_START,
  .clock_name                                 = clock_can1,
  .port                                       = 1,
  .fifo_index                                 = 0,
  .filter_num                                 = 0,
  .filter_index                               = 0,
  .can_config.enable_tdc                      = true,
#ifdef CONFIG_HPM6750_CAN1_USER_SET_TINING

  /* Assume the CAN clock is 80MHz, configure the nominal baudrate to 500kbit/s, configure the canfd baudrate to 5Mbit/s */

  .can_config.use_lowlevel_timing_setting     = true;

  /* bitrate = can_freq / prescale / (seq1 + seg2) */

  .can_config.can_timing.num_seg1             = CONFIG_HPM6750_CAN1_NUM_SEG1;
  .can_config.can_timing.num_seg2             = CONFIG_HPM6750_CAN1_NUM_SEG2;
  .can_config.can_timing.num_sjw              = CONFIG_HPM6750_CAN1_NUM_SJW;
  .can_config.can_timing.prescaler            = CONFIG_HPM6750_CAN1_PRESCALER;
#    ifdef CONFIG_CAN_FD
  .can_config.canfd_timing.num_seg1           = CONFIG_HPM6750_CAN1_FD_NUM_SEG1;
  .can_config.canfd_timing.num_seg2           = CONFIG_HPM6750_CAN1_FD_NUM_SEG2;
  .can_config.canfd_timing.num_sjw            = CONFIG_HPM6750_CAN1_FD_NUM_SJW;
  .can_config.canfd_timing.prescaler          = CONFIG_HPM6750_CAN1_FD_PRESCALER;
#    endif
#  else
  .can_config.can20_samplepoint_min           = CONFIG_HPM6750_CAN1_MIN_SAMPLEPOINT,
  .can_config.can20_samplepoint_max           = CONFIG_HPM6750_CAN1_MAX_SAMPLEPOINT,
  .can_config.baudrate                        = CONFIG_HPM6750_CAN1_BAUDRATE,
#  ifdef CONFIG_CAN_FD
  .can_config.canfd_samplepoint_min           = CONFIG_HPM6750_CAN1_FD_MIN_SAMPLEPOINT,
  .can_config.canfd_samplepoint_max           = CONFIG_HPM6750_CAN1_FD_MAX_SAMPLEPOINT,
  .can_config.enable_canfd                    = true,
  .can_config.baudrate_fd                     = CONFIG_HPM6750_CAN1_FD_BAUDRATE,
#  endif

#endif

  .can_config.disable_re_transmission_for_ptb = false,
  .can_config.disable_re_transmission_for_stb = false,
  .can_config.enable_self_ack                 = false,
  .can_config.enable_tx_buffer_priority_mode  = false,
  .can_config.enable_tdc                      = false,
  .can_config.irq_txrx_enable_mask            = false,
  .can_config.irq_error_enable_mask           = false,
  .can_config.enable_can_fd_iso_mode          = CONFIG_HPM6750_CAN1_FD_ISO_MODE,
  .can_config.mode                            = CONFIG_HPM6750_CAN1_MODE,
  .fifo_index                                 = 0,
  .rx_irq_enable                              = 0,
  .tx_irq_enable                              = 0,
};

static struct can_dev_s g_can1dev             =
{
  .cd_ops                                     = &g_can1ops,
  .cd_priv                                    = &g_can1priv,
};
#endif

#ifdef CONFIG_HPM6750_CAN2

static const struct can_ops_s g_can2ops       =
{
  .co_reset                                   = hpm_can_reset,
  .co_setup                                   = hpm_can_setup,
  .co_shutdown                                = hpm_can_shutdown,
  .co_rxint                                   = hpm_can_rxint,
  .co_txint                                   = hpm_can_txint,
  .co_ioctl                                   = hpm_can_ioctl,
  .co_remoterequest                           = hpm_can_remoterequest,
  .co_send                                    = hpm_can_send,
  .co_txready                                 = hpm_can_txready,
  .co_txempty                                 = hpm_can_txempty,
};

static struct hpmcan_dev_s g_can2priv         =
{
  .can_base                                   = HPM_CAN2,
  .irq_num                                    = IRQn_CAN2 + HPM_IRQ_PERI_START,
  .clock_name                                 = clock_can0,
  .port                                       = 2,
  .fifo_index                                 = 0,
  .filter_num                                 = 0,
  .filter_index                               = 0,
  .can_config.enable_tdc                      = true,
#ifdef CONFIG_HPM6750_CAN2_USER_SET_TINING

  /* Assume the CAN clock is 80MHz, configure the nominal baudrate to 500kbit/s, configure the canfd baudrate to 5Mbit/s */

  .can_config.use_lowlevel_timing_setting     = true;

  /* bitrate = can_freq / prescale / (seq1 + seg2) */

  .can_config.can_timing.num_seg1             = CONFIG_HPM6750_CAN2_NUM_SEG1;
  .can_config.can_timing.num_seg2             = CONFIG_HPM6750_CAN2_NUM_SEG2;
  .can_config.can_timing.num_sjw              = CONFIG_HPM6750_CAN2_NUM_SJW;
  .can_config.can_timing.prescaler            = CONFIG_HPM6750_CAN2_PRESCALER;
#    ifdef CONFIG_CAN_FD
  .can_config.canfd_timing.num_seg1           = CONFIG_HPM6750_CAN2_FD_NUM_SEG1;
  .can_config.canfd_timing.num_seg2           = CONFIG_HPM6750_CAN2_FD_NUM_SEG2;
  .can_config.canfd_timing.num_sjw            = CONFIG_HPM6750_CAN2_FD_NUM_SJW;
  .can_config.canfd_timing.prescaler          = CONFIG_HPM6750_CAN2_FD_PRESCALER;
#    endif
#  else
  .can_config.can20_samplepoint_min           = CONFIG_HPM6750_CAN2_MIN_SAMPLEPOINT,
  .can_config.can20_samplepoint_max           = CONFIG_HPM6750_CAN2_MAX_SAMPLEPOINT,
  .can_config.baudrate                        = CONFIG_HPM6750_CAN2_BAUDRATE,
#  ifdef CONFIG_CAN_FD
  .can_config.canfd_samplepoint_min           = CONFIG_HPM6750_CAN2_FD_MIN_SAMPLEPOINT,
  .can_config.canfd_samplepoint_max           = CONFIG_HPM6750_CAN2_FD_MAX_SAMPLEPOINT,
  .can_config.enable_canfd                    = true,
  .can_config.baudrate_fd                     = CONFIG_HPM6750_CAN2_FD_BAUDRATE,
#  endif

#endif

  .can_config.disable_re_transmission_for_ptb = false,
  .can_config.disable_re_transmission_for_stb = false,
  .can_config.enable_self_ack                 = false,
  .can_config.enable_tx_buffer_priority_mode  = false,
  .can_config.enable_tdc                      = false,
  .can_config.irq_txrx_enable_mask            = false,
  .can_config.irq_error_enable_mask           = false,
  .can_config.enable_can_fd_iso_mode          = CONFIG_HPM6750_CAN2_FD_ISO_MODE,
  .can_config.mode                            = CONFIG_HPM6750_CAN2_MODE,
  .fifo_index                                 = 0,
  .rx_irq_enable                              = 0,
  .tx_irq_enable                              = 0,
};

static struct can_dev_s g_can2dev             =
{
  .cd_ops                                     = &g_can2ops,
  .cd_priv                                    = &g_can2priv,
};
#endif

#ifdef CONFIG_HPM6750_CAN3

static const struct can_ops_s g_can3ops       =
{
  .co_reset                                   = hpm_can_reset,
  .co_setup                                   = hpm_can_setup,
  .co_shutdown                                = hpm_can_shutdown,
  .co_rxint                                   = hpm_can_rxint,
  .co_txint                                   = hpm_can_txint,
  .co_ioctl                                   = hpm_can_ioctl,
  .co_remoterequest                           = hpm_can_remoterequest,
  .co_send                                    = hpm_can_send,
  .co_txready                                 = hpm_can_txready,
  .co_txempty                                 = hpm_can_txempty,
};

static struct hpmcan_dev_s g_can3priv         =
{
  .can_base                                   = HPM_CAN3,
  .irq_num                                    = IRQn_CAN3 + HPM_IRQ_PERI_START,
  .clock_name                                 = clock_can3,
  .port                                       = 3,
  .fifo_index                                 = 0,
  .filter_num                                 = 0,
  .filter_index                               = 0,
  .can_config.enable_tdc                      = true,
#ifdef CONFIG_HPM6750_CAN3_USER_SET_TINING

  /* Assume the CAN clock is 80MHz, configure the nominal baudrate to 500kbit/s, configure the canfd baudrate to 5Mbit/s */

  .can_config.use_lowlevel_timing_setting = true;

  /* bitrate = can_freq / prescale / (seq1 + seg2) */

  .can_config.can_timing.num_seg1             = CONFIG_HPM6750_CAN3_NUM_SEG1;
  .can_config.can_timing.num_seg2             = CONFIG_HPM6750_CAN3_NUM_SEG2;
  .can_config.can_timing.num_sjw              = CONFIG_HPM6750_CAN3_NUM_SJW;
  .can_config.can_timing.prescaler            = CONFIG_HPM6750_CAN3_PRESCALER;
#    ifdef CONFIG_CAN_FD
  .can_config.canfd_timing.num_seg1           = CONFIG_HPM6750_CAN0_FD_NUM_SEG1;
  .can_config.canfd_timing.num_seg2           = CONFIG_HPM6750_CAN0_FD_NUM_SEG2;
  .can_config.canfd_timing.num_sjw            = CONFIG_HPM6750_CAN0_FD_NUM_SJW;
  .can_config.canfd_timing.prescaler          = CONFIG_HPM6750_CAN0_FD_PRESCALER;
#    endif
#  else
  .can_config.can20_samplepoint_min           = CONFIG_HPM6750_CAN0_MIN_SAMPLEPOINT,
  .can_config.can20_samplepoint_max           = CONFIG_HPM6750_CAN0_MAX_SAMPLEPOINT,
  .can_config.baudrate                        = CONFIG_HPM6750_CAN0_BAUDRATE,
#  ifdef CONFIG_CAN_FD
  .can_config.canfd_samplepoint_min           = CONFIG_HPM6750_CAN0_FD_MIN_SAMPLEPOINT,
  .can_config.canfd_samplepoint_max           = CONFIG_HPM6750_CAN0_FD_MAX_SAMPLEPOINT,
  .can_config.enable_canfd                    = true,
  .can_config.baudrate_fd                     = CONFIG_HPM6750_CAN0_FD_BAUDRATE,
#  endif

#endif

  .can_config.disable_re_transmission_for_ptb = false,
  .can_config.disable_re_transmission_for_stb = false,
  .can_config.enable_self_ack                 = false,
  .can_config.enable_tx_buffer_priority_mode  = false,
  .can_config.enable_tdc                      = false,
  .can_config.irq_txrx_enable_mask            = false,
  .can_config.irq_error_enable_mask           = false,
  .can_config.enable_can_fd_iso_mode          = CONFIG_HPM6750_CAN3_FD_ISO_MODE,
  .can_config.mode                            = CONFIG_HPM6750_CAN3_MODE,
  .fifo_index                                 = 0,
  .rx_irq_enable                              = 0,
  .tx_irq_enable                              = 0,
};

static struct can_dev_s g_can3dev             =
{
  .cd_ops                                     = &g_can3ops,
  .cd_priv                                    = &g_can3priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

static int can_set_cfg(struct can_dev_s *dev)
{
  struct hpmcan_dev_s *priv = (struct hpmcan_dev_s *)dev->cd_priv;

  priv->can_config.filter_list_num = priv->filter_num;
  priv->can_config.filter_list = &priv->filter_list[0];
  uint32_t can_clk = board_init_can_clock(priv->can_base);

  if(status_success == can_init(priv->can_base, &priv->can_config, can_clk))
    {
      return 0;
    }

  return -1;
}

/****************************************************************************
 * Name: can_set_add_exdfilters
 *
 * Description:
 *   Set the extern filter of an CAN .
 *
 * Input Parameters:
 *   dev     - An instance of the "upper half" CAN driver state structure.
 *   type    - filters id type
 *   filter  - The configuration of the filter
 * Returned Value:
 *   A non-negative filter ID is returned on success.  Otherwise a negated
 *   errno value is returned to indicate the nature of the error.
 *
 ****************************************************************************/
static int can_set_add_exdfilters(struct can_dev_s *dev, struct canioc_extfilter_s *filter)
{
  struct hpmcan_dev_s *priv = (struct hpmcan_dev_s *)dev->cd_priv;
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
      canerr("ERROR: filter number max: %ld\n", priv->filter_num);
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
 *   type    - filters id type
 *   filter  - The configuration of the filter
 * Returned Value:
 *   A non-negative filter ID is returned on success.  Otherwise a negated
 *   errno value is returned to indicate the nature of the error.
 *
 ****************************************************************************/

static int can_set_add_stdfilters(struct can_dev_s *dev, struct canioc_stdfilter_s *filter)
{
  struct hpmcan_dev_s *priv = (struct hpmcan_dev_s *)dev->cd_priv;
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
      canerr("ERROR: filter number max: %ld\n", priv->filter_num);
      return -1;
    }
  
  index = priv->filter_index;
  priv->filter_list[index].index   = index;
  priv->filter_list[index].code    = filter->sf_id1;
  priv->filter_list[index].enable  = true;
  priv->filter_list[index].id_mode = can_filter_id_mode_standard_frames;

  if (filter->sf_type == CAN_FILTER_MASK)
    {
      priv->filter_list[index].mask = (~filter->sf_id2);
    }
  else if (filter->sf_type == CAN_FILTER_RANGE)
    {
      uint32_t num = (filter->sf_id2 - filter->sf_id1 + 1);
      uint32_t tmp;
      uint32_t mask;
      for (int i = 0; i < num; i++)                      /* Mask the result of the exclusive OR of each member of the code bit array */
        {
          tmp = (filter->sf_id1 + 1) ^ (~filter->sf_id1); /* Both perform NXOR operation with the first data member */
          mask &= tmp;
          mask = ~mask;
        }
      priv->filter_list[index].mask = mask;      
    }
  else
    {
      canerr("ERROR: the xf type is not support: %d\n", filter->sf_type);
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
 *   ndx  - The filter index previously returned by the
 *          can_set_add_**filter().
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the error.
 *
 ****************************************************************************/

static int can_set_del_filters(struct can_dev_s *dev, int ndx)
{
  struct hpmcan_dev_s *priv = (struct hpmcan_dev_s *)dev->cd_priv;
  if (ndx > CAN_FILTER_NUM_MAX)
    {
      canerr("ERROR: filter number is max: %d\n", ndx);
      return -1;
    }
  priv->filter_num --;
  priv->filter_index --; 

  return 0;
}

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

static void hpm_can_reset(struct can_dev_s *dev)
{
  struct hpmcan_dev_s *priv = (struct hpmcan_dev_s *)dev->cd_priv;
  irqstate_t flags;
  int ret;

  caninfo("CAN%" PRIu8 "\n", priv->port);

  flags = enter_critical_section();
  can_reset(priv->can_base,true);
  up_disable_irq(priv->irq_num);
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

static int hpm_can_setup(struct can_dev_s *dev)
{
  struct hpmcan_dev_s *priv = (struct hpmcan_dev_s *)dev->cd_priv;
  irqstate_t flags;
  int ret = OK;

  caninfo("CAN%" PRIu8 "\n", priv->port);

  flags = enter_critical_section();

  clock_enable(priv->clock_name);
  clock_add_to_group(priv->clock_name, BOARD_RUNNING_CORE);
  
  if (hpm_init_can_pins(priv->port) <0)
    {
      return -1;
    }

  if (can_set_cfg(dev) < 0)
    {
      return -1;
    }
  
  ret = irq_attach(priv->irq_num, hpm_can_interrupt, dev);
  if (ret != OK)
    {
      leave_critical_section(flags);

      return ret;
    }

  up_enable_irq(priv->irq_num);

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

static void hpm_can_shutdown(struct can_dev_s *dev)
{
  /* the interface not necessarily implement*/

  struct hpmcan_dev_s *priv = (struct hpmcan_dev_s *)dev->cd_priv;

#ifdef CONFIG_DEBUG_CAN_INFO
  caninfo("shutdown CAN%" PRIu8 "\n", priv->port);
#endif
 
  clock_remove_from_group(priv->clock_name, BOARD_RUNNING_CORE);
  clock_disable(priv->clock_name);

  /* Disable can interrupt */

  up_disable_irq(priv->irq_num);

  /* Dissociate the IRQ from the ISR */

  irq_detach(priv->irq_num);

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

static void hpm_can_rxint(struct can_dev_s *dev, bool enable)
{
  struct hpmcan_dev_s *priv = (struct hpmcan_dev_s *)dev->cd_priv;
  irqstate_t flags;

  caninfo("CAN%" PRIu8 " enable: %d\n", priv->port, enable);

  flags = enter_critical_section();
  priv->rx_irq_enable   = enable;
  uint8_t irq_txrx_mask = CAN_EVENT_RECEIVE | CAN_EVENT_RX_BUF_ALMOST_FULL | CAN_EVENT_RX_BUF_FULL | CAN_EVENT_RX_BUF_OVERRUN;
  (enable == true) ? (can_enable_tx_rx_irq(priv->can_base, irq_txrx_mask)) : (can_disable_tx_rx_irq(priv->can_base, irq_txrx_mask));
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

static void hpm_can_txint(struct can_dev_s *dev, bool enable)
{
  struct hpmcan_dev_s *priv = (struct hpmcan_dev_s *)dev->cd_priv;
  irqstate_t flags;

  caninfo("CAN%" PRIu8 " enable: %d\n", priv->port, enable);

  flags = enter_critical_section();
  priv->tx_irq_enable   = enable;
  uint8_t irq_txrx_mask = CAN_EVENT_TX_PRIMARY_BUF | CAN_EVENT_TX_SECONDARY_BUF;
  (enable == true) ? (can_enable_tx_rx_irq(priv->can_base, irq_txrx_mask)) : (can_disable_tx_rx_irq(priv->can_base, irq_txrx_mask));
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: hpm_can_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *   cmd - A ioctl command.
 *   arg - A ioctl argument.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int hpm_can_ioctl(struct can_dev_s *dev, int cmd,
                             unsigned long arg)
{
  struct hpmcan_dev_s *priv = (struct hpmcan_dev_s *)dev->cd_priv;
  uint8_t flag = 0;
  int ret = -ENOTTY;

  caninfo("CAN%" PRIu8 " cmd=%04x arg=%lu\n", priv->port, cmd, arg);

  /* Handle the command */

  switch (cmd)
    {
      /* CANIOC_GET_BITTIMING:
       *   Description:    Return the current bit timing settings
       *   Argument:       A pointer to a write-able instance of struct
       *                   canioc_bittiming_s in which current bit timing
       *                   values will be returned.
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR) is returned with the errno variable set
       *                   to indicate the nature of the error.
       *   Dependencies:   None
       */

      case CANIOC_GET_BITTIMING:
        {
          struct canioc_bittiming_s *bt =
            (struct canioc_bittiming_s *)arg;

          DEBUGASSERT(bt != NULL);

          bt->bt_baud  = priv->can_config.can_timing.prescaler;
          bt->bt_tseg1 = priv->can_config.can_timing.num_seg1;
          bt->bt_tseg2 = priv->can_config.can_timing.num_seg2;
          bt->bt_sjw   = priv->can_config.can_timing.num_sjw;

          ret = OK;
        }
        break;

      /* CANIOC_SET_BITTIMING:
       *   Description:    Set new current bit timing values
       *   Argument:       A pointer to a read-able instance of struct
       *                   canioc_bittiming_s in which the new bit timing
       *                   values are provided.
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR) is returned with the errno variable set
       *                   to indicate the nature of the error.
       *   Dependencies:   None
       *
       * REVISIT: There is probably a limitation here:  If there are
       * multiple threads trying to send CAN packets, when one of these
       * threads reconfigures the bitrate, the FDCAN hardware will be reset
       * and the context of operation will be lost.  Hence, this IOCTL can
       * only safely be executed in quiescent time periods.
       */

      case CANIOC_SET_BITTIMING:
        {
          struct canioc_bittiming_s *bt =
            (struct canioc_bittiming_s *)arg;

          DEBUGASSERT(bt != NULL);   
          priv->can_config.use_lowlevel_timing_setting = true; 
          priv->can_config.can_timing.prescaler   = bt->bt_baud;
          priv->can_config.can_timing.num_seg1    = bt->bt_tseg1;
          priv->can_config.can_timing.num_seg2    = bt->bt_tseg2;
          priv->can_config.can_timing.num_sjw     = bt->bt_sjw;
#ifdef CONFIG_CAN_FD
          priv->can_config.enable_canfd = true;
          priv->can_config.canfd_timing.prescaler = bt->bt_baud;
          priv->can_config.canfd_timing.num_seg1  = bt->bt_tseg1;
          priv->can_config.canfd_timing.num_seg2  = bt->bt_tseg2;
          priv->can_config.canfd_timing.num_sjw   = bt->bt_sjw;
#endif
          ret = OK;
        }
        break;

      /* CANIOC_GET_CONNMODES:
       *   Description:    Get the current bus connection modes
       *   Argument:       A pointer to a write-able instance of struct
       *                   canioc_connmodes_s in which the new bus modes will
       *                   be returned.
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR)is returned with the errno variable set
       *                   to indicate the nature of the error.
       *   Dependencies:   None
       */

      case CANIOC_GET_CONNMODES:
        {
          struct canioc_connmodes_s *bm =
            (struct canioc_connmodes_s *)arg;

          DEBUGASSERT(bm != NULL);

          if (priv->can_config.mode == can_mode_loopback_internal)
            {
              bm->bm_loopback = 1;
            }
          else
            {
              bm->bm_loopback = 0;
            }
          
          if (priv == can_mode_listen_only)
            {
              bm->bm_silent  = 1;
            }
          else
            {
              bm->bm_silent  = 0;
            }

          ret = OK;      
        }
        break;
        
      /* CANIOC_SET_CONNMODES:
       *   Description:    Set new bus connection modes values
       *   Argument:       A pointer to a read-able instance of struct
       *                   canioc_connmodes_s in which the new bus modes
       *                   are provided.
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR) is returned with the errno variable set
       *                   to indicate the nature of the error.
       *   Dependencies:   None
       */

      case CANIOC_SET_CONNMODES:
        {
          struct canioc_connmodes_s *bm =
            (struct canioc_connmodes_s *)arg;

          DEBUGASSERT(bm != NULL);

          if (bm->bm_loopback)
            {
              priv->can_config.mode = can_mode_loopback_internal;
              can_set_node_mode(priv->can_base, can_mode_loopback_internal);
            }
          else
            {
              priv->can_config.mode = can_mode_normal;
              can_set_node_mode(priv->can_base, can_mode_normal);
            }

          if (bm->bm_silent)
            {
              priv->can_config.mode = can_mode_loopback_internal;
              can_set_node_mode(priv->can_base, can_mode_loopback_internal);
            }
          else
            {
              priv->can_config.mode = can_mode_normal;
              can_set_node_mode(priv->can_base, can_mode_normal);
            }

          ret = OK;
        }
        break;
      

#ifdef CONFIG_CAN_EXTID
      /* CANIOC_ADD_EXTFILTER:
       *   Description:    Add an address filter for a extended 29 bit
       *                   address.
       *   Argument:       A reference to struct canioc_extfilter_s
       *   Returned Value: A non-negative filter ID is returned on success.
       *                   Otherwise -1 (ERROR) is returned with the errno
       *                   variable set to indicate the nature of the error.
       */

      case CANIOC_ADD_EXTFILTER:
        {
          if (can_set_add_exdfilters(dev, (struct canioc_extfilter_s *)arg) >= 0)
            {
              flag =1;
            } 
        }
        break;

      /* CANIOC_DEL_EXTFILTER:
       *   Description:    Remove an address filter for a standard 29 bit
       *                   address.
       *   Argument:       The filter index previously returned by the
       *                   CANIOC_ADD_EXTFILTER command
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR) is returned with the errno variable set
       *                   to indicate the nature of the error.
       */

      case CANIOC_DEL_EXTFILTER:
        {
          if (can_set_del_filters(dev, (int)arg) >= 0)
            {
              flag = 1;
            }
        }
        break;
#endif

      /* CANIOC_ADD_STDFILTER:
       *   Description:    Add an address filter for a standard 11 bit
       *                   address.
       *   Argument:       A reference to struct canioc_stdfilter_s
       *   Returned Value: A non-negative filter ID is returned on success.
       *                   Otherwise -1 (ERROR) is returned with the errno
       *                   variable set to indicate the nature of the error.
       */

      case CANIOC_ADD_STDFILTER:
        {
          DEBUGASSERT(arg != 0);
          
          if (can_set_add_stdfilters(dev, (struct canioc_stdfilter_s *)arg) >= 0)
            {
              flag =1;
            } 
        }
        break;

      /* CANIOC_DEL_STDFILTER:
       *   Description:    Remove an address filter for a standard 11 bit
       *                   address.
       *   Argument:       The filter index previously returned by the
       *                   CANIOC_ADD_STDFILTER command
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR) is returned with the errno variable set
       *                   to indicate the nature of the error.
       */

      case CANIOC_DEL_STDFILTER:
        {
          if (can_set_del_filters(dev, (int)arg) >= 0)
            {
              flag = 1;
            }
        }
        break;

      /* Description:    Initiates the BUS-OFF recovery sequence
        *   Argument:       None
        *   Returned Value: Zero (OK) is returned on success.  Otherwise -1 (ERROR)
        *                   is returned with the errno variable set to indicate the
        *                   nature of the error.
        *   Dependencies:   None
        */

      case CANIOC_BUSOFF_RECOVERY:
        {
          if (can_is_in_bus_off_mode(priv->can_base) == true)
            {
              priv->can_base->CMD_STA_CMD_CTRL &= ~CAN_CMD_STA_CMD_CTRL_BUSOFF_MASK;
            }

          ret = OK;
        }
        break;

      /*   Description:    Enable/Disable ABOM (Automatic Bus-off Management)
        *   Argument:       Set to 1 to enable ABOM, 0 to disable. Default is
        *                   disabled.
        *   Returned Value: Zero (OK) is returned on success.  Otherwise -1 (ERROR)
        *                   is returned with the errno variable set to indicate the
        *                   nature of the error.
        *   Dependencies:   None
        */
      
      case CANIOC_SET_ABOM:
        {
          if (can_is_in_bus_off_mode(priv->can_base) == false)
            {
              priv->can_base->CMD_STA_CMD_CTRL |= CAN_CMD_STA_CMD_CTRL_BUSOFF_MASK;
            }

          ret = OK;
        }

      /* Unsupported/unrecognized command */

      default:
        canerr("ERROR: Unrecognized command: %04x\n", cmd);
        break;
    }

  if(flag)
    {
      ret = can_set_cfg(priv);
    }

  return ret;
}

static int hpm_can_remoterequest(struct can_dev_s *dev, uint16_t id)
{
  canwarn("Remote request not implemented\n");
  return -ENOSYS;
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

static int hpm_can_send(struct can_dev_s *dev,
                            struct can_msg_s *msg)
{
  struct hpmcan_dev_s *priv = (struct hpmcan_dev_s *)dev->cd_priv;
  uint32_t i;
  uint32_t len;
  irqstate_t flags;
  int ret = OK;

  flags = enter_critical_section();

  caninfo("CAN%" PRIu8 " ID: %" PRIu32 " DLC: %" PRIu8 "\n",
          priv->port, (uint32_t)msg->cm_hdr.ch_id, msg->cm_hdr.ch_dlc);

  can_transmit_buf_t tx_buf = { 0 };
  
  tx_buf.id = msg->cm_hdr.ch_id;

#ifdef CONFIG_CAN_EXTID
  tx_buf.extend_id = true;
#else
  tx_buf.extend_id = false;
#endif
  
  if (msg->cm_hdr.ch_rtr == 1)
    {
      tx_buf.remote_frame = true;
    }
  else
    {
      tx_buf.remote_frame = false;
    }
 
#ifdef CONFIG_CAN_FD
  if (msg->cm_hdr.ch_edl == 1)
    {
      tx_buf.canfd_frame    = 1;
      tx_buf.bitrate_switch = 1;
    }
#endif
  
  len = can_get_data_bytes_from_dlc((uint32_t)msg->cm_hdr.ch_dlc);

  if (len > CAN_MAXDATALEN)
    {
      len = CAN_MAXDATALEN;
    }

  for (i = 0; i < len; i++)
    {
        tx_buf.data[i] = msg->cm_data[i];
    }
  tx_buf.dlc = msg->cm_hdr.ch_dlc;

  can_send_message_nonblocking(priv->can_base, &tx_buf);

  leave_critical_section(flags);

  return ret;
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

static bool hpm_can_txready(struct can_dev_s *dev)
{
  struct hpmcan_dev_s *priv = dev->cd_priv;

  if (can_is_secondary_transmit_buffer_full(priv->can_base) == true) 
  {
    return false;
  }
  
  return true;
}

/****************************************************************************
 * Name: hpm_can_txempty
 *
 * Description:
 *   Return true if all message have been sent.  If for example, the CAN
 *   hardware implements FIFOs, then this would mean the transmit FIFO is
 *   empty.  This method is called when the driver needs to make sure that
 *   all characters are "drained" from the TX hardware before calling
 *   co_shutdown().
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" CAN driver state structure.
 *
 * Returned Value:
 *   True if there are no pending TX transfers in the CAN hardware.
 *
 ****************************************************************************/

static bool hpm_can_txempty(struct can_dev_s *dev)
{
  struct hpmcan_dev_s *priv = dev->cd_priv;

  if (can_get_secondary_transmit_buffer_status(priv->can_base) == 0) 
  {
    return true;
  }

  return false;

}

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
  struct can_dev_s *dev = (struct can_dev_s *)arg;
  struct hpmcan_dev_s *priv = dev->cd_priv;
  can_receive_buf_t s_can_rx_buf;
  struct can_hdr_s hdr;
  uint8_t error_flags;

  if (irq == priv->irq_num) 
    {
      uint8_t flags = can_get_tx_rx_flags(priv->can_base);
      if ((flags & CAN_EVENT_RECEIVE) != 0) 
        {
          can_read_received_message(priv->can_base, (can_receive_buf_t *)&s_can_rx_buf);
          hdr.ch_id    = s_can_rx_buf.id;
          hdr.ch_dlc   = s_can_rx_buf.dlc;
          hdr.ch_rtr   = s_can_rx_buf.remote_frame;

#ifdef CONFIG_CAN_EXTID         
          hdr.ch_extid = s_can_rx_buf.extend_id;
#endif          

#ifdef CONFIG_CAN_FD
          hdr.ch_edl   = s_can_rx_buf.canfd_frame;
          hdr.ch_brs   = s_can_rx_buf.bitrate_switch;
          hdr.ch_esi   = s_can_rx_buf.error_state_indicator;
#endif

#ifdef CONFIG_CAN_ERRORS
      hdr.ch_error  = 0; /* Error reporting not supported */
#endif
          can_receive(dev, &hdr, s_can_rx_buf.data);
        }
      
      if ((flags & (CAN_EVENT_TX_PRIMARY_BUF | CAN_EVENT_TX_SECONDARY_BUF)))
       {

          /* Indicate that the TX is done and a new TX buffer is available */

          can_txdone(dev);
       }

      can_clear_tx_rx_flags(priv->can_base, flags);
      error_flags = can_get_error_interrupt_flags(priv->can_base);
      can_clear_error_interrupt_flags(priv->can_base, error_flags);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm6750_caninitialize
 *
 * Description:
 *   Initialize the selected CAN port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple CAN interfaces)
 *
 * Returned Value:
 *   Valid CAN device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct can_dev_s *hpm6750_caninitialize(int port)
{
  struct can_dev_s *candev;
  irqstate_t flags;

  caninfo("CAN%" PRIu8 "\n",  port);

  flags = enter_critical_section();

#ifdef CONFIG_HPM6750_CAN0
  if (port == 0)
    {
      candev = &g_can0dev;
    }
  else
#endif
#ifdef CONFIG_HPM6750_CAN1
  if (port == 1)
    {
      candev = &g_can1dev;
    }
  else
#endif
#ifdef CONFIG_HPM6750_CAN2
  if (port == 2)
    {
      candev = &g_can2dev;
    }
  else
#endif
#ifdef CONFIG_HPM6750_CAN3
  if (port == 3)
    {
      candev = &g_can3dev;
    }
  else
#endif
    {
      canerr("ERROR: Unsupported port %d\n", port);
      return NULL;
    }

  /* Then just perform a CAN reset operation */

  hpm_can_reset(candev);

  hpm_can_setup(candev);
  
  leave_critical_section(flags);

  return candev;
}

#endif
