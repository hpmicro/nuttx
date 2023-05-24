/****************************************************************************
 * arch/risc-v/src/hpmicro/hpm_tim_lowerhalf.c
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
#include <nuttx/arch.h>

#include <sys/types.h>

#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/timers/timer.h>

#include <arch/board/board.h>
#include "riscv_internal.h"

#include "board.h"
#include "hpm_gptmr_drv.h"
#include "hpm_clock_drv.h"

#if defined(CONFIG_TIMER) && defined(CONFIG_HPM_GPTMR_DRV)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TIMER_MAX_VALUE        (0xFFFFFFFFU)
#define TIMER_CLK_DIV          (40U) /* the timer src_clock is pll1clk1, pll1clk1 is 400M so the timer clk is 10M ,tick is 0.1us*/
#define TIMER_1US_MULTIPLE     (10U) /* 1us is 0.1 * 10*/
#define TIMER_CHANNEL          (0U)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct 
{
  const struct timer_ops_s *ops;     /* Lower half operations */

  bool                      started;  /* True: Timer has been started */
  tccb_t                    callback; /* Current upper half interrupt callback */
  void                     *arg;      /* Argument passed to upper half callback */
  GPTMR_Type               *base;
  uint32_t                 channel;
  clock_name_t             clock_name;
  int32_t                  irq_num;
  int                      time_out_value;
  int                      timer_freq;
  
}hpm_tim_lowerhalf_s;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int hpm_timer_handler(int irq, void *context, void *arg);

static int hpm_timer_init(hpm_tim_lowerhalf_s *lower);

/* "Lower half" driver methods */

static int  hpm_tim_start(struct timer_lowerhalf_s *lower);
static int  hpm_tim_stop(struct timer_lowerhalf_s *lower);
static int  hpm_tim_getstatus(struct timer_lowerhalf_s *lower,
                                struct timer_status_s *status);
static int  hpm_tim_settimeout(struct timer_lowerhalf_s *lower,
                                 uint32_t timeout);
static void hpm_tim_setcallback(struct timer_lowerhalf_s *lower,
                                  tccb_t callback, void *arg);
static int  hpm_tim_maxtimeout(struct timer_lowerhalf_s *lower,
                                  uint32_t *maxtimeout);
/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct timer_ops_s g_timer_ops =
{
  .start       = hpm_tim_start,
  .stop        = hpm_tim_stop,
  .getstatus   = hpm_tim_getstatus,
  .settimeout  = hpm_tim_settimeout,
  .setcallback = hpm_tim_setcallback,
  .maxtimeout  = hpm_tim_maxtimeout,
  .ioctl       = NULL,
};

#ifdef CONFIG_HPM_TIMER0
static hpm_tim_lowerhalf_s g_tim0_lowerhalf =
{
  .ops            = &g_timer_ops,
  .irq_num        = HPM_IRQn_GPTMR0,
  .base           = HPM_GPTMR0,
  .clock_name     = clock_gptmr0,
  .channel        = TIMER_CHANNEL,
  .time_out_value = 0,
};
#endif

#ifdef CONFIG_HPM_TIMER1
static hpm_tim_lowerhalf_s g_tim1_lowerhalf =
{
  .ops            = &g_timer_ops,
  .irq_num        = HPM_IRQn_GPTMR1,
  .base           = HPM_GPTMR1,
  .clock_name     = clock_gptmr1,
  .channel        = TIMER_CHANNEL,
  .time_out_value = 0,
};
#endif

#ifdef CONFIG_HPM_TIMER2
static hpm_tim_lowerhalf_s g_tim2_lowerhalf =
{
  .ops            = &g_timer_ops,
  .irq_num        = HPM_IRQn_GPTMR2,
  .base           = HPM_GPTMR2,
  .clock_name     = clock_gptmr2,
  .channel        = TIMER_CHANNEL,
  .time_out_value = 0,
};
#endif

#ifdef CONFIG_HPM_TIMER3
static hpm_tim_lowerhalf_s g_tim3_lowerhalf =
{
  .ops            = &g_timer_ops,
  .irq_num        = HPM_IRQn_GPTMR3,
  .base           = HPM_GPTMR3,
  .clock_name     = clock_gptmr3,
  .channel        = TIMER_CHANNEL,
  .time_out_value = 0,
};
#endif

#ifdef CONFIG_HPM_TIMER4
static hpm_tim_lowerhalf_s g_tim4_lowerhalf =
{
  .ops            = &g_timer_ops,
  .irq_num        = HPM_IRQn_GPTMR4,
  .base           = HPM_GPTMR4,
  .clock_name     = clock_gptmr4,
  .channel        = TIMER_CHANNEL,
  .time_out_value = 0,
};
#endif

#ifdef CONFIG_HPM_TIMER5
static hpm_tim_lowerhalf_s g_tim5_lowerhalf =
{
  .ops            = &g_timer_ops,
  .irq_num        = HPM_IRQn_GPTMR5,
  .base           = HPM_GPTMR5,
  .clock_name     = clock_gptmr5,
  .channel        = TIMER_CHANNEL,
  .time_out_value = 0,
};
#endif

#ifdef CONFIG_HPM_TIMER6
static hpm_tim_lowerhalf_s g_tim6_lowerhalf =
{
  .ops            = &g_timer_ops,
  .irq_num        = HPM_IRQn_GPTMR6,
  .base           = HPM_GPTMR6,
  .clock_name     = clock_gptmr6,
  .channel        = TIMER_CHANNEL,
  .time_out_value = 0,
};
#endif

#ifdef CONFIG_HPM_TIMER7
static hpm_tim_lowerhalf_s g_tim7_lowerhalf =
{
  .ops            = &g_timer_ops,
  .irq_num        = HPM_IRQn_GPTMR7,
  .base           = HPM_GPTMR7,
  .clock_name     = clock_gptmr7,
  .channel        = TIMER_CHANNEL,
  .time_out_value = 0,
};
#endif


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_timer_handler
 *
 * Description:
 *   Timer interrupt handler
 *
 * Input Parameters:
 *
 * Returned Value:
*   Zero on success; a negated errno value on failure.
 ****************************************************************************/

static int hpm_timer_handler(int irq, void *context, void *arg)
{
  hpm_tim_lowerhalf_s *priv = (hpm_tim_lowerhalf_s *)arg;
  gptmr_channel_config_t config;
  uint32_t next_interval_us = 0;

  if (gptmr_check_status(priv->base,GPTMR_CH_RLD_STAT_MASK(priv->channel))) 
    {
      if (priv->callback(&next_interval_us, priv->arg))
        {
          if(next_interval_us > 0)
            {
              gptmr_stop_counter(priv->base, priv->channel);
              gptmr_channel_get_default_config(priv->base, &config);
              config.reload = (next_interval_us * TIMER_1US_MULTIPLE);
              priv->time_out_value = (next_interval_us * TIMER_1US_MULTIPLE);
              gptmr_channel_config(priv->base, priv->channel, &config, true);

            }
        }
      else
        {
          gptmr_stop_counter(priv->base, priv->channel);
        }
    }
  gptmr_clear_status(priv->base, GPTMR_CH_RLD_STAT_MASK(priv->channel));
  return OK;
}

/****************************************************************************
 * Name: hpm_timer_init
 *
 * Description:
 *   Timer initialization
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 ****************************************************************************/

static int hpm_timer_init(hpm_tim_lowerhalf_s *lower)
{
  /* Configure the Timer clock to 1MHz */

  clock_set_source_divider(lower->clock_name, clk_src_pll1_clk1, TIMER_CLK_DIV);
  clock_add_to_group(lower->clock_name, BOARD_RUNNING_CORE);
  lower->timer_freq = clock_get_frequency(lower->clock_name);
  return OK;
}

/****************************************************************************
 * Name: hpm_tim_start
 *
 * Description:
 *   Start the timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the
 *           "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int hpm_tim_start(struct timer_lowerhalf_s *lower)
{
  hpm_tim_lowerhalf_s *priv = (hpm_tim_lowerhalf_s *)lower;
  irqstate_t flags;

  if (!priv->started)
    {
      if ((priv->callback == NULL) || (priv->time_out_value == 0))
        {
          return -EPERM;
        }
      flags = enter_critical_section();
      gptmr_enable_irq(priv->base, GPTMR_CH_RLD_IRQ_MASK(priv->channel));
      gptmr_start_counter(priv->base, priv->channel); 
      up_enable_irq(priv->irq_num);
      leave_critical_section(flags);
      priv->started = true;
      return OK;
    }

  /* Return EBUSY to indicate that the timer was already running */

  return -EBUSY;
}

/****************************************************************************
 * Name: hpm_tim_stop
 *
 * Description:
 *   Stop the timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of
 *           the "lower-half" driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int hpm_tim_stop(struct timer_lowerhalf_s *lower)
{
  hpm_tim_lowerhalf_s*priv = (hpm_tim_lowerhalf_s *)lower;
  int ret = OK;
  irqstate_t flags;

  DEBUGASSERT(priv);

  if (priv->started == false)
    {
      /* Return ENODEV to indicate that the timer was not running */

      ret = -ENODEV;
      goto errout;
    }

  flags = enter_critical_section();
  gptmr_disable_irq(priv->base, GPTMR_CH_RLD_IRQ_MASK(priv->channel));
  gptmr_stop_counter(priv->base, priv->channel);
  up_disable_irq(priv->irq_num);
  leave_critical_section(flags);

  priv->started = false;
  priv->callback = NULL;
  priv->time_out_value = 0;

errout:
  return ret;
}

/****************************************************************************
 * Name: hpm_tim_getstatus
 *
 * Description:
 *   Get timer status.
 *
 * Input Parameters:
 *   lower  - A pointer the publicly visible representation of the "lower-
 *            half" driver state structure.
 *   status - The location to return the status information. the timeout value in microseconds
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int hpm_tim_getstatus(struct timer_lowerhalf_s *lower,
                                 struct timer_status_s *status)
{
  hpm_tim_lowerhalf_s *priv = (hpm_tim_lowerhalf_s *)lower;
  int ret = OK;
  uint64_t current_counter_value;

  DEBUGASSERT(priv);
  DEBUGASSERT(status);

  /* Return the status bit */

  status->flags = 0;

  if (priv->started == true)
    {
      /* TIMER is running */

      status->flags |= TCFLAGS_ACTIVE;
    }

  if (priv->callback != NULL)
    {
      /* TIMER has a user callback function to be called when
       * expiration happens
       */

      status->flags |= TCFLAGS_HANDLER;
    }

  status->timeout = (priv->base->CHANNEL[priv->channel].RLD + 1)/TIMER_1US_MULTIPLE;
  
  current_counter_value = (gptmr_channel_get_counter(priv->base, priv->channel, gptmr_counter_type_normal))/TIMER_1US_MULTIPLE;

  if (current_counter_value < status->timeout)
    {
      status->timeleft = status->timeout - current_counter_value;
    }
  else
    {
      status->timeleft = 0;
    }

  return ret;
}

/****************************************************************************
 * Name: hpm_tim_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the
 *"lower-half" driver state structure. timeout - The new timeout value in
 *microseconds.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int hpm_tim_settimeout(struct timer_lowerhalf_s *lower,
                                uint32_t timeout)
{
  hpm_tim_lowerhalf_s *priv = (hpm_tim_lowerhalf_s *)lower;
  gptmr_channel_config_t config;

  gptmr_channel_get_default_config(priv->base, &config);
  config.reload = (timeout * TIMER_1US_MULTIPLE);
  gptmr_channel_config(priv->base, priv->channel, &config, false);
  priv->time_out_value = (timeout * TIMER_1US_MULTIPLE);
  return OK;
}

/****************************************************************************
 * Name: hpm_tim_setcallback
 *
 * Description:
 *   Call this user provided timeout handler.
 *
 * Input Parameters:
 *   lower    - A pointer the publicly visible representation of the
 *              "lower-half" driver state structure.
 *   callback - The new timer expiration function pointer.  If this function
 *              pointer is NULL, then the reset-on-expiration behavior is
 *              restored.
 *   arg      - Argument that will be provided in the callback
 *
 * Returned Value:
 *   The previous timer expiration function pointer or NULL is there was
 *   no previous function pointer.
 *
 ****************************************************************************/

static void hpm_tim_setcallback(struct timer_lowerhalf_s *lower,
                                  tccb_t callback, void *arg)
{
  hpm_tim_lowerhalf_s *priv = (hpm_tim_lowerhalf_s *)lower;
  irqstate_t flags = enter_critical_section();

  /* Save the new callback */

  priv->callback = callback;
  priv->arg      = arg;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: hpm_tim_maxtimeout
 *
 * Description:
 *   Get the maximum timeout value
 *
 * Input Parameters:
 *   lower       - A pointer the publicly visible representation of
 *                 the "lower-half" driver state structure.
 *   maxtimeout  - A pointer to the variable that will store the max timeout.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/
static int hpm_tim_maxtimeout(struct timer_lowerhalf_s *lower,
                                  uint32_t *maxtimeout)
{
  DEBUGASSERT(maxtimeout);

  *maxtimeout = (TIMER_MAX_VALUE/TIMER_1US_MULTIPLE);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_timer_initialize
 *
 * Description:
 *   Bind the configuration timer to a timer lower half instance and
 *   register the timer drivers at 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the timer device.  This should be of the
 *     form /dev/timer0
 *   timer - the timer's number.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int hpm_timer_initialize(const char *devpath, int timer)
{
  hpm_tim_lowerhalf_s *lower;

  switch (timer)
    {
    case 0:
#ifdef CONFIG_HPM_TIMER0
      lower = &g_tim0_lowerhalf;
#endif
      break;
    case 1:
#ifdef CONFIG_HPM_TIMER1
      lower = &g_tim1_lowerhalf;
#endif
    case 2:
#ifdef CONFIG_HPM_TIMER2
      lower = &g_tim2_lowerhalf;
#endif
      break;
    case 3:
#ifdef CONFIG_HPM_TIMER3
      lower = &g_tim3_lowerhalf;
#endif
    case 4:
#ifdef CONFIG_HPM_TIMER4
      lower = &g_tim4_lowerhalf;
#endif
      break;
    case 5:
#ifdef CONFIG_HPM_TIMER5
      lower = &g_tim5_lowerhalf;
#endif
    case 6:
#ifdef CONFIG_HPM_TIMER6
      lower = &g_tim6_lowerhalf;
#endif
      break;
    case 7:
#ifdef CONFIG_HPM_TIMER7
      lower = &g_tim7_lowerhalf;
#endif
      break;
    default:
      return -ENODEV;
    }

  hpm_timer_init(lower);

  /* Attach GPTMR interrupt vectors */

  irq_attach(lower->irq_num, hpm_timer_handler, (void *)lower);

  /* Initialize the elements of lower half state structure */

  lower->started  = false;
  lower->callback = NULL;

  /* Register the timer driver as /dev/timerX.  The returned value from
   * timer_register is a handle that could be used with timer_unregister().
   * REVISIT: The returned handle is discard here.
   */

  void *drvr = timer_register(devpath, (struct timer_lowerhalf_s *)lower);
  if (drvr == NULL)
    {
      /* The actual cause of the failure may have been a failure to allocate
       * perhaps a failure to register the timer driver (such as if the
       * 'depath' were not unique).  We know here but we return EEXIST to
       * indicate the failure (implying the non-unique devpath).
       */

      return -EEXIST;
    }

  return OK;
}

#endif
