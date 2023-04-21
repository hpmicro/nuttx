/****************************************************************************
 * arch/risc-v/src/hpmicro/hpm6750/hpm_rtc_lowerhalf.c
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

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/timers/rtc.h>

#include "chip.h"
#include "riscv_internal.h"

#include "board.h"
#include "hpm_soc.h"
#include "hpm_rtc_drv.h"

#ifdef CONFIG_RTC_DRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HPM_ALARMS_NUM      (2)
#define HPM_ALARM0          (0)
#define HPM_ALARM1          (1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_RTC_ALARM
typedef struct 
{
  volatile rtc_alarm_callback_t cb; /* Callback when the alarm expires */
  volatile void *priv;              /* Private argument to accompany callback */
  uint8_t id;                       /* Identifies the alarm */
}hpm_alarm_cbinfo_s;
#endif

/* This is the private type for the RTC state.  It must be cast compatible
 * with struct rtc_lowerhalf_s.
 */

typedef struct 
{
  /* This is the contained reference to the read-only, lower-half
   * operations vtable (which may lie in FLASH or ROM)
   */

  const struct rtc_ops_s *ops;

  /* Data following is private to this driver and not visible outside of
   * this file.
   */

  mutex_t devlock;                  /* Threads can only exclusively access the RTC */

  bool is_time_set;

  int  irq_num;

#ifdef CONFIG_RTC_ALARM
  /* Alarm callback information */

  hpm_alarm_cbinfo_s alarm_cbinfo[HPM_ALARMS_NUM];

#endif
}hpm_lowerhalf_s;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Prototypes for static methods in struct rtc_ops_s */

static int hpm_rtcinterrupt(int irq, void *context, void *arg);
static hpm_stat_t hpm_rtc_config_absolute_alarm(RTC_Type *base, rtc_alarm_config_t *config);

static int hpm_rdtime(struct rtc_lowerhalf_s *lower,
                        struct rtc_time *rtctime);
static int hpm_settime(struct rtc_lowerhalf_s *lower,
                         const struct rtc_time *rtctime);
static bool hpm_havesettime(struct rtc_lowerhalf_s *lower);

#ifdef CONFIG_RTC_ALARM
static int hpm_setalarm(struct rtc_lowerhalf_s *lower,
                          const struct lower_setalarm_s *alarminfo);
static int hpm_setrelative(struct rtc_lowerhalf_s *lower,
                            const struct lower_setrelative_s *alarminfo);
static int hpm_cancelalarm(struct rtc_lowerhalf_s *lower,
                             int alarmid);
static int hpm_rdalarm(struct rtc_lowerhalf_s *lower,
                         struct lower_rdalarm_s *alarminfo);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* HPMicro RTC driver operations */

static const struct rtc_ops_s g_rtc_ops =
{
  .rdtime         = hpm_rdtime,
  .settime        = hpm_settime,
  .havesettime    = hpm_havesettime,
#ifdef CONFIG_RTC_ALARM
  .setalarm       = hpm_setalarm,
  .setrelative    = hpm_setrelative,
  .cancelalarm    = hpm_cancelalarm,
  .rdalarm        = hpm_rdalarm,
#endif
#ifdef CONFIG_RTC_PERIODIC
  .setperiodic    = NULL,  /* Not implemented */
  .cancelperiodic = NULL,
#endif
#ifdef CONFIG_RTC_IOCTL
  .ioctl          = NULL,
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  .destroy        = NULL,
#endif
};

/* HPMicro RTC device state */

static hpm_lowerhalf_s  g_rtc_lowerhalf =
{
  .ops         = &g_rtc_ops,
  .devlock     = NXMUTEX_INITIALIZER,
  .is_time_set = false,
  .irq_num     = HPM_IRQn_RTC,
};

/****************************************************************************
 * Name: hpm_rtcinterrupt
 *
 * Description:
 *  RTC interrupt handler
 *
 ****************************************************************************/

static int hpm_rtcinterrupt(int irq, void *context, void *arg)
{
#ifdef CONFIG_RTC_ALARM
  if (irq == g_rtc_lowerhalf.irq_num)
    {
      for (uint8_t i = 0; i < HPM_ALARMS_NUM; i++)
        {
          if (rtc_is_alarm_flag_asserted(HPM_RTC, g_rtc_lowerhalf.alarm_cbinfo[i].id))
            {
              if (g_rtc_lowerhalf.alarm_cbinfo[i].cb) 
                {
                  g_rtc_lowerhalf.alarm_cbinfo[i].cb((void *)g_rtc_lowerhalf.alarm_cbinfo[i].priv, g_rtc_lowerhalf.alarm_cbinfo[i].id);
                }             
              rtc_clear_alarm_flag(HPM_RTC, g_rtc_lowerhalf.alarm_cbinfo[i].id);
            }
        }
      
    }
#endif
  return 0;
}

/****************************************************************************
 * Name: hpm_rtcinterrupt
 *
 * Description:
 *  RTC interrupt handler
 *
 ****************************************************************************/

static hpm_stat_t hpm_rtc_config_absolute_alarm(RTC_Type *base, rtc_alarm_config_t *config)
{
  hpm_stat_t status = status_invalid_argument;
  do {
    if ((config == NULL) || (config->index > 1U) || (config->type > RTC_ALARM_TYPE_PERIODIC)) 
      {
        break;
      }
    uint32_t alarm_inc = 0;
    uint32_t current_sec = base->SECOND;
    uint32_t alarm = config->period;
    if (config->type == RTC_ALARM_TYPE_ONE_SHOT) 
      {
        alarm_inc = 0;
      } 
    else 
      {
        alarm_inc = config->period;
      }
    if (alarm < current_sec) 
      {
        break;
      }

    if (config->index == 0U) 
      {
        base->ALARM0 = alarm;
        base->ALARM0_INC = alarm_inc;
      } 
    else 
      {
        base->ALARM1 = alarm;
        base->ALARM1_INC = alarm_inc;
      }
    status = status_success;
  } while (false);

  return status;
}

/****************************************************************************
 * Name: hpm_rdtime
 *
 * Description:
 *   Implements the rdtime() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower   - A reference to RTC lower half driver state structure
 *   rcttime - The location in which to return the current RTC time.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

static int hpm_rdtime(struct rtc_lowerhalf_s *lower,
                        struct rtc_time *rtctime)
{
  time_t timer;

  /* The resolution of time is only 1 second */

  timer = rtc_get_time(HPM_RTC);

  /* Convert the one second epoch time to a struct tm */

  if (gmtime_r(&timer, (struct tm *)rtctime) == 0)
    {
      int errcode = get_errno();
      DEBUGASSERT(errcode > 0);

      rtcerr("ERROR: gmtime_r failed: %d\n", errcode);
      return -errcode;
    }

  return OK;
}

/****************************************************************************
 * Name: hpm_settime
 *
 * Description:
 *   Implements the settime() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower   - A reference to RTC lower half driver state structure
 *   rcttime - The new time to set
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

static int hpm_settime(struct rtc_lowerhalf_s *lower,
                         const struct rtc_time *rtctime)
{
  struct timespec ts;
  hpm_lowerhalf_s *priv = (hpm_lowerhalf_s *)lower;
  hpm_stat_t sta = status_success;
  int ret = 0;

  /* Convert the struct rtc_time to a time_t.  Here we assume that struct
   * rtc_time is cast compatible with struct tm.
   */

  ts.tv_sec  = timegm((struct tm *)rtctime);
  ts.tv_nsec = 0;

  /* Now set the time (to one second accuracy) */

  rtc_config_time(HPM_RTC, ts.tv_sec);
  priv->is_time_set = true;

  (sta == status_success) ? (ret = 0) : (ret = -1);

  return ret;
}

/****************************************************************************
 * Name: hpm_havesettime
 *
 * Description:
 *   Implements the havesettime() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower   - A reference to RTC lower half driver state structure
 *
 * Returned Value:
 *   Returns true if RTC date-time have been previously set.
 *
 ****************************************************************************/

static bool hpm_havesettime(struct rtc_lowerhalf_s *lower)
{
  hpm_lowerhalf_s *priv = (hpm_lowerhalf_s *)lower;

  DEBUGASSERT(priv != NULL);

  return priv->is_time_set;
}

#ifdef CONFIG_RTC_ALARM

/****************************************************************************
 * Name: hpm_setalarm
 *
 * Description:
 *   Set a new alarm.  This function implements the setalarm() method of the
 *   RTC driver interface
 *
 * Input Parameters:
 *   lower     - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to set the alarm
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

static int hpm_setalarm(struct rtc_lowerhalf_s *lower,
                          const struct lower_setalarm_s *alarminfo)
{
  int ret = -1;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->id == 0);

  hpm_lowerhalf_s *priv = (hpm_lowerhalf_s *)lower;
  struct timespec ts;

    /* Get exclusive access to the alarm */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      rtcerr("ERROR: mutex_lock failed: %d\n", ret);
      return ret;
    }
  ret = -EINVAL;

  if (alarminfo->id == HPM_ALARM0 || alarminfo->id == HPM_ALARM1)
    {

      rtc_enable_alarm_interrupt(HPM_RTC, alarminfo->id, false);

      /* Remember the callback information */

      priv->alarm_cbinfo[alarminfo->id].id   = alarminfo->id;
      priv->alarm_cbinfo[alarminfo->id].cb   = alarminfo->cb;
      priv->alarm_cbinfo[alarminfo->id].priv = alarminfo->priv;

      /* Convert the RTC time to a timespec (1 second accuracy) */

      ts.tv_sec  = timegm((struct tm *)&alarminfo->time);
      ts.tv_nsec = 0;

      /* Configure RTC alarm */ 

      rtc_alarm_config_t alarm_cfg;
      alarm_cfg.index = alarminfo->id;
      alarm_cfg.period = ts.tv_sec;
      alarm_cfg.type = RTC_ALARM_TYPE_ONE_SHOT;
      if(hpm_rtc_config_absolute_alarm(HPM_RTC, &alarm_cfg) == status_success)
        {
          rtc_clear_alarm_flag(HPM_RTC, alarminfo->id);
          rtc_enable_alarm_interrupt(HPM_RTC, alarminfo->id, true);
          ret = 0;
        }
    }

  nxmutex_unlock(&priv->devlock);
  return ret;

}

/****************************************************************************
 * Name: hpm_setrelative
 *
 * Description:
 *   Set a new alarm relative to the current time.  This function implements
 *   the setrelative() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to set the alarm
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

static int hpm_setrelative(struct rtc_lowerhalf_s *lower,
                             const struct lower_setrelative_s *alarminfo)
{
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->id == 0);
  hpm_lowerhalf_s *priv = (hpm_lowerhalf_s *)lower;

  /* Get exclusive access to the alarm */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      rtcerr("ERROR: nxmutex_lock failed: %d\n", ret);
      return ret;
    }

  ret = -EINVAL;

  if ((alarminfo->id == HPM_ALARM0 || alarminfo->id == HPM_ALARM1) && (alarminfo->reltime > 0))
    {

      rtc_enable_alarm_interrupt(HPM_RTC, alarminfo->id, false);

      /* Remember the callback information */

      priv->alarm_cbinfo[alarminfo->id].id   = alarminfo->id;
      priv->alarm_cbinfo[alarminfo->id].cb   = alarminfo->cb;
      priv->alarm_cbinfo[alarminfo->id].priv = alarminfo->priv;

      /* Convert the RTC time to a timespec (1 second accuracy) */
      /* Configure RTC alarm */ 

      rtc_alarm_config_t alarm_cfg;
      alarm_cfg.index = alarminfo->id;
      alarm_cfg.period = alarminfo->reltime;
      alarm_cfg.type = RTC_ALARM_TYPE_ONE_SHOT;
      rtc_config_alarm(HPM_RTC, &alarm_cfg);
      rtc_clear_alarm_flag(HPM_RTC, alarminfo->id);
      rtc_enable_alarm_interrupt(HPM_RTC, alarminfo->id, true);
      ret = 0;
    }

  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: hpm_cancelalarm
 *
 * Description:
 *   Cancel the current alarm.  This function implements the cancelalarm()
 *   method of the RTC driver interface
 *
 * Input Parameters:
 *   lower - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to set the alarm
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

static int hpm_cancelalarm(struct rtc_lowerhalf_s *lower, int alarmid)
{
  /* We cancel the alarm by alarm by disabling the alarm and the alarm
   * interrupt.
   */

  rtc_clear_alarm_flag(HPM_RTC, alarmid);
  rtc_enable_alarm_interrupt(HPM_RTC, alarmid, true);
  return OK;
}

/****************************************************************************
 * Name: hpm_rdalarm
 *
 * Description:
 *   Query the RTC alarm.
 *
 * Input Parameters:
 *   lower     - A reference to RTC lower half driver state structure
 *   alarminfo - Provided information needed to query the alarm
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

static int hpm_rdalarm(struct rtc_lowerhalf_s *lower,
                         struct lower_rdalarm_s *alarminfo)
{
  int ret = -EINVAL;

  DEBUGASSERT(lower != NULL && alarminfo != NULL && alarminfo->time != NULL);

  /* There is only one alarm */

  if (alarminfo->id == HPM_ALARM0 || alarminfo->id == HPM_ALARM1)
    {
      time_t alarm;

      /* Get the current alarm setting in seconds */

      alarm = (time_t)rtc_get_time(HPM_RTC);

      /* Convert the one second epoch time to a struct tm */

      ret = OK;
      if (gmtime_r(&alarm, (struct tm *)alarminfo->time) == 0)
        {
          int errcode = get_errno();
          DEBUGASSERT(errcode > 0);
          ret = -errcode;
        }
    }

  return ret;
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm6750_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the STM32.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "hpm_rtc.h>
 *
 *     struct rtc_lowerhalf_s *lower;
 *     lower = hpm6750_rtc_lowerhalf();
 *     rtc_initialize(0, lower);
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, a non-NULL RTC lower interface is returned.  NULL is
 *   returned on any failure.
 *
 ****************************************************************************/

struct rtc_lowerhalf_s *hpm6750_rtc_lowerhalf(void)
{
    /* Attach RTC interrupt vectors */

  irq_attach(g_rtc_lowerhalf.irq_num, hpm_rtcinterrupt, NULL);

  /* Enable the IRQ at the NVIC (still disabled at the RTC controller) */

  up_enable_irq(g_rtc_lowerhalf.irq_num);

  return (struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;
}

/****************************************************************************
 * Name: up_rtc_time
 *
 * Description:
 *   Get the current time in seconds.  This is similar to the standard time()
 *   function.  This interface is only required if the low-resolution
 *   RTC/counter hardware implementation selected.  It is only used by the
 *   RTOS during initialization to set up the system time when CONFIG_RTC
 *   is set but neither CONFIG_RTC_HIRES nor CONFIG_RTC_DATETIME are set.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current time in seconds
 *
 ****************************************************************************/

time_t up_rtc_time(void)
{
  /* Delegate to rtc_get_time() */

  return rtc_get_time(HPM_RTC);
}

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.
 *   This function is called once during the OS initialization sequence
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_initialize(void)
{
  return 0;
}

#endif /* CONFIG_RTC_DRIVER */
