/****************************************************************************
 * arch/risc-v/src/hpmicro/hpm_rtc_lowerhalf.c
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
 * Public Data
 ****************************************************************************/

/* g_rtc_enabled is set true after the RTC has successfully initialized */

volatile bool g_rtc_enabled = false;
static struct rtc_time g_rtc_alarm[HPM_ALARMS_NUM];

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
  time_t rtc_ts;

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

      rtc_ts = rtc_get_time(HPM_RTC);
      if (ts.tv_sec <= rtc_ts)
        {
          return -1;
        }
      memcpy(&g_rtc_alarm[alarminfo->id], &alarminfo->time, sizeof(struct rtc_time));

      /* Convert the RTC time to a timespec (1 second accuracy) */
      /* Configure RTC alarm */

      rtc_alarm_config_t alarm_cfg;
      alarm_cfg.index = alarminfo->id;
      alarm_cfg.period = ts.tv_sec - rtc_ts;
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
  struct lower_setalarm_s setalarm;
  struct tm *time;
  time_t rtc_ts;
  time_t seconds;

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
      rtc_ts = rtc_get_time(HPM_RTC);
      time = gmtime(&rtc_ts);
      seconds = timegm(time);
      seconds += alarminfo->reltime;
      gmtime_r(&seconds, (struct tm *)&setalarm.time);
      setalarm.id   = alarminfo->id;
      setalarm.cb   = alarminfo->cb;
      setalarm.priv = alarminfo->priv;
      nxmutex_unlock(&priv->devlock);
      ret = hpm_setalarm(lower, &setalarm);
    }
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

  if (alarmid == HPM_ALARM0 || alarmid == HPM_ALARM1)
   {
      memset(&g_rtc_alarm[alarmid], 0, sizeof(struct rtc_time));
      rtc_clear_alarm_flag(HPM_RTC, alarmid);
      rtc_enable_alarm_interrupt(HPM_RTC, alarmid, false);
      return OK;
   }
   return ERROR;

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
      ret = OK;
      memcpy(alarminfo->time, &g_rtc_alarm[alarminfo->id], sizeof(struct rtc_time));
    }

  return ret;
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the STM32.  General usage:
 *
 *     #include <nuttx/timers/rtc.h>
 *     #include "hpm_rtc.h>
 *
 *     struct rtc_lowerhalf_s *lower;
 *     lower = hpm_rtc_lowerhalf();
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

struct rtc_lowerhalf_s *hpm_rtc_lowerhalf(void)
{
    /* Attach RTC interrupt vectors */

  irq_attach(g_rtc_lowerhalf.irq_num, hpm_rtcinterrupt, NULL);

  /* Enable the IRQ at the NVIC (still disabled at the RTC controller) */

  up_enable_irq(g_rtc_lowerhalf.irq_num);

  g_rtc_enabled = true;

  return (struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;
}

#if defined(CONFIG_RTC) && !defined(CONFIG_RTC_HIRES)
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
#endif

#if defined(CONFIG_RTC_DATETIME)
int up_rtc_getdatetime(struct tm *tp)
{
  struct tm *d_tm;
  time_t rtc_tm;
  if (!tp)
    {
      return -1;
    }
  rtc_tm = rtc_get_time(HPM_RTC);
  d_tm = gmtime(&rtc_tm);
  memcpy(tp, d_tm, sizeof(struct tm));
  return 0;
}
#endif

#if defined(CONFIG_RTC_HIRES)
int up_rtc_gettime(FAR struct timespec *tp)
{
  struct timeval tm;
  tm = rtc_get_timeval(HPM_RTC);
  tp->tv_nsec = tm.tv_usec;
  tp->tv_sec = tm.tv_sec;
  return 0;
}
#endif

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
  struct tm cfg_date_time;
  cfg_date_time.tm_year = CONFIG_START_YEAR - 1900;        /* Year: (start from 1900) */
  cfg_date_time.tm_mon = CONFIG_START_MONTH - 1;   /* Month:(start from 0) */
  cfg_date_time.tm_mday = CONFIG_START_DAY;      /* Day in Month */
  cfg_date_time.tm_hour = 1;               /* Hour */
  cfg_date_time.tm_min = 0;              /* Minute */
  cfg_date_time.tm_sec = 0;              /* Second */
  time_t cfg_time = mktime(&cfg_date_time); /* Convert date time to tick */
  rtc_config_time(HPM_RTC, cfg_time);
  return 0;
}

#endif /* CONFIG_RTC_DRIVER */
