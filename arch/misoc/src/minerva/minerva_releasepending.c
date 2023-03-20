/****************************************************************************
 * arch/misoc/src/minerva/minerva_releasepending.c
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

#include <sched.h>
#include <syscall.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include "sched/sched.h"
#include "group/group.h"
#include "minerva.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_release_pending
 *
 * Description:
 *   Release and ready-to-run tasks that have
 *   collected in the pending task list.  This can call a
 *   context switch if a new task is placed at the head of
 *   the ready to run list.
 *
 ****************************************************************************/

void up_release_pending(void)
{
  struct tcb_s *rtcb = this_task();

  sinfo("From TCB=%p\n", rtcb);

  /* Merge the g_pendingtasks list into the ready-to-run task list */

  /* sched_lock(); */

  if (nxsched_merge_pending())
    {
      /* The currently active task has changed! We will need to switch
       * contexts. Update scheduler parameters.
       */

      nxsched_suspend_scheduler(rtcb);

      /* Are we operating in interrupt context? */

      if (g_current_regs)
        {
          /* Yes, then we have to do things differently. Just copy the
           * g_current_regs into the OLD rtcb.
           */

          misoc_savestate(rtcb->xcp.regs);

          /* Restore the exception context of the rtcb at the (new) head of
           * the ready-to-run task list.
           */

          rtcb = this_task();

          /* Update scheduler parameters */

          nxsched_resume_scheduler(rtcb);

          /* Then switch contexts.  Any necessary address environment changes
           * will be made when the interrupt returns.
           */

          misoc_restorestate(rtcb->xcp.regs);
        }

      /* No, then we will need to perform the user context switch */

      else
        {
          /* Switch context to the context of the task at the head of the
           * ready to run list.
           */

          struct tcb_s *nexttcb = this_task();

          /* Update scheduler parameters */

          nxsched_resume_scheduler(nexttcb);

          /* Then switch contexts */

          misoc_switchcontext(rtcb->xcp.regs, nexttcb->xcp.regs);

          /* misoc_switchcontext forces a context switch to the task at the
           * head of the ready-to-run list.  It does not 'return' in the
           * normal sense.  When it does return, it is because the blocked
           * task is again ready to run and has execution priority.
           */
        }
    }
}