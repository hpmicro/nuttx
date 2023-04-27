/****************************************************************************
 * arch/risc-v/src/hpmicro/common/hpm_pwm_lowerhalf.c
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
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "hpm_pwm_lowerhalf.h"
#include "riscv_internal.h"

#include "hpm_soc.h"
#include "hpm_pwm_drv.h"
#include "hpm_clock_drv.h"

#if defined(CONFIG_PWM) && defined(CONFIG_HPM_PWM_DRV)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_PWM_NCHANNELS
#  if (CONFIG_PWM_NCHANNELS > 8)
#    error "The maximum number of PWM channels is 8!"
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct 
{
  const struct pwm_ops_s *ops;         /* PWM operations */
  int                     pwm_port;
  uint32_t                pwm_freq;
  uint32_t                pwm_reload;
  PWM_Type                *base;
  clock_name_t            clock_name; 
}hpm_pwm_s;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int motsys_generate_pwm(hpm_pwm_s *pwm, const struct pwm_info_s *info);

/* PWM driver methods */

static int hpm_pwm_setup(struct pwm_lowerhalf_s *dev);
static int hpm_pwm_shutdown(struct pwm_lowerhalf_s *dev);
static int hpm_pwm_start(struct pwm_lowerhalf_s *dev,
                           const struct pwm_info_s *info);
static int hpm_pwm_stop(struct pwm_lowerhalf_s *dev);
static int hpm_pwm_ioctl(struct pwm_lowerhalf_s *dev,
                           int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the list of lower half PWM driver methods used by the upper half
 * driver.
 */

static const struct pwm_ops_s g_hpm_pwmops =
{
  .setup       = hpm_pwm_setup,
  .shutdown    = hpm_pwm_shutdown,
  .start       = hpm_pwm_start,
  .stop        = hpm_pwm_stop,
  .ioctl       = hpm_pwm_ioctl,
};

#ifdef CONFIG_HPM_PWM0
hpm_pwm_s g_hpm_pwm0 =
{
  .ops         = &g_hpm_pwmops,
  .pwm_port    = 0,
  .base        = HPM_PWM0,
  .clock_name  = clock_mot0,
};
#endif

#ifdef CONFIG_HPM_PWM1
hpm_pwm_s g_hpm_pwm1 =
{
  .ops         = &g_hpm_pwmops,
  .pwm_port    = 1,
  .base        = HPM_PWM1,
  .clock_name  = clock_mot1,
};
#endif

#ifdef CONFIG_HPM_PWM2
hpm_pwm_s g_hpm_pwm2 =
{
  .ops         = &g_hpm_pwmops,
  .pwm_port    = 2,
  .base        = HPM_PWM2,
  .clock_name  = clock_mot2,
};
#endif 

#ifdef CONFIG_HPM_PWM3
hpm_pwm_s g_hpm_pwm3 =
{
  .ops         = &g_hpm_pwmops,
  .pwm_port    = 3,
  .base        = HPM_PWM3,
  .clock_name  = clock_mot3,
};
#endif 

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int motsys_generate_pwm(hpm_pwm_s *pwm, const struct pwm_info_s *info)
{
  hpm_stat_t sta = status_invalid_argument;

#ifdef CONFIG_PWM_NCHANNELS

    uint8_t cmp_index = 0;
    uint32_t cmp = 0;
    int i;
    int8_t chan;
    uint8_t _duty;
    pwm_cmp_config_t cmp_config;
    pwm_config_t pwm_config = {0};

    pwm_get_default_pwm_config(pwm->base, &pwm_config);

    pwm_stop_counter(pwm->base);
    pwm_config.enable_output = true;
    pwm_config.dead_zone_in_half_cycle = 0;
    pwm_config.invert_output = false;

    uint32_t mot_pwm_freq     = clock_get_frequency(pwm->clock_name);
    pwm->pwm_reload           = mot_pwm_freq/info->frequency; 

    pwm_set_reload(pwm->base, 0, pwm->pwm_reload);
    pwm_set_start_count(pwm->base, 0, 0);

    cmp_config.mode           = pwm_cmp_mode_output_compare;
    cmp_config.cmp            = 0;
    cmp_config.update_trigger = pwm_shadow_register_update_on_modify;
    pwm_load_cmp_shadow_on_match(pwm->base, 0, &cmp_config);

    cmp_config.update_trigger = pwm_shadow_register_update_on_hw_event;

    for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
      {
        chan = info->channels[i].channel;
        _duty = b16toi(info->channels[i].duty * 100);
        cmp = (pwm->pwm_reload * _duty)/100;
        cmp_config.cmp            = cmp;

        /* Break the loop if all following channels are not configured */

        if (chan == -1)
          {
            break;
          }

        sta = pwm_setup_waveform(pwm->base, chan, &pwm_config, chan, &cmp_config, 1);
      }
    if (sta != status_success)
      {
        return -1;
      }
    pwm_start_counter(pwm->base);
    pwm_issue_shadow_register_lock_event(pwm->base);
    for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
      {
        _duty = b16toi(info->channels[i].duty * 100);
        cmp = (pwm->pwm_reload * _duty)/100;
        chan = info->channels[i].channel;

        /* Break the loop if all following channels are not configured */

        if (chan == -1)
          {
            break;
          }
        
        pwm_update_raw_cmp_edge_aligned(pwm->base, chan, cmp);
      }  
#endif
  return (sta != status_success)? -1 : 0;
}

/****************************************************************************
 * Name: hpm_pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 ****************************************************************************/

static int hpm_pwm_setup(struct pwm_lowerhalf_s *dev)
{
  int i;
  int ret = OK;
  hpm_pwm_s *priv = (hpm_pwm_s *)dev;

  UNUSED(i);
  priv->pwm_freq = 0;
  ret = hpm_init_pwm_pins(priv->pwm_port);

  return ret;
}

/****************************************************************************
 * Name: hpm_pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 ****************************************************************************/

static int hpm_pwm_shutdown(struct pwm_lowerhalf_s *dev)
{
  int i;
  int ret = OK;
  hpm_pwm_s *priv = (hpm_pwm_s *)dev;

  UNUSED(i);
  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
  {
    pwm_disable_output(priv->base, i);
  }
  
  return ret;
}

/****************************************************************************
 * Name: hpm_pwm_start
 *
 * Description:
 *   (Re-)initialize the PWM and start the pulsed output
 *
 ****************************************************************************/

static int hpm_pwm_start(struct pwm_lowerhalf_s *dev,
                           const struct pwm_info_s *info)
{
  hpm_pwm_s *priv = (hpm_pwm_s *)dev;
  int                 ret  = OK;
  int                 i;
  int _duty = 0;
  UNUSED(i);
  ret = motsys_generate_pwm(priv, info);
  return ret;
}

/****************************************************************************
 * Name: hpm_pwm_stop
 *
 * Description:
 *   Stop the PWM
 *
 ****************************************************************************/

static int hpm_pwm_stop(struct pwm_lowerhalf_s *dev)
{
  int i;
  hpm_pwm_s *priv = (hpm_pwm_s *)dev;

  UNUSED(priv);
  UNUSED(i);
 
#ifdef CONFIG_PWM_NCHANNELS
  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      pwm_disable_output(priv->base, i);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: hpm_pwm_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 ****************************************************************************/

static int hpm_pwm_ioctl(struct pwm_lowerhalf_s *dev,
                           int cmd, unsigned long arg)
{
  hpm_pwm_s *priv = (hpm_pwm_s *)dev;

  DEBUGASSERT(dev);

  /* There are no platform-specific ioctl commands */

  UNUSED(priv);

  return -ENOTTY;
}

/****************************************************************************
 * Public Function
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   pwm - A number identifying the pwm instance.
 *
 * Returned Value:
 *   On success, a pointer to the HPM lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *hpm_pwminitialize(int pwm)
{
  hpm_pwm_s *lower = NULL;

  if (pwm > 4)
    {
      pwminfo("Initialize PWM%u failed\n", pwm);
      return NULL;
    }

  pwminfo("Initialize PWM%u\n", pwm);

#ifdef CONFIG_HPM_PWM0
  lower = &g_hpm_pwm0;
#endif

#ifdef CONFIG_HPM_PWM1
  lower = &g_hpm_pwm1;
#endif

#ifdef CONFIG_HPM_PWM2
  lower = &g_hpm_pwm2;
#endif

#ifdef CONFIG_HPM_PWM3
  lower = &g_hpm_pwm3;
#endif

  return (struct pwm_lowerhalf_s *)lower;
}

#endif

