/****************************************************************************
 * arch/risc-v/src/hpmicro/hpm_mbx.c
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
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>

#include "hpm_mbx.h"

#ifdef CONFIG_HPM_MBX
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int     mbx_open(FAR struct file *filep);
static int     mbx_close(FAR struct file *filep);
static ssize_t mbx_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen);
static ssize_t mbx_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations hpm_mbx_fops =
{
  mbx_open,                     /* open */
  mbx_close,                    /* close */
  mbx_read,                     /* read */
  mbx_write,                    /* write */
  NULL,                         /* seek */
  NULL,                    /* ioctl */
  NULL                          /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL                        /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mbx_open
 ****************************************************************************/

static int   mbx_open(FAR struct file *filep)
{
  int ret = 0;
  DEBUGASSERT(filep != NULL);
  DEBUGASSERT(filep->f_inode->i_private != NULL);
  struct hpm_mbx_lowerhalf_s *mbx_dev = (struct hpm_mbx_lowerhalf_s *)filep->f_inode->i_private;
  nxmutex_lock(&mbx_dev->mbx_lock);
  if ((mbx_dev->base == NULL) || ((mbx_dev->base != HPM_MBX0A) && (mbx_dev->base != HPM_MBX0B)))
    {
      ret = -ENOENT;
    }
  else
    {
      if (mbx_dev->o_count == 0)
        {
          mbx_dev->o_count++;
          mbx_init(mbx_dev->base);
        }
      else
        {
          ret = -EMFILE;
        }
    }
  nxmutex_unlock(&mbx_dev->mbx_lock);

  return ret;
}

/****************************************************************************
 * Name: mbx_close
 ****************************************************************************/

static int  mbx_close(FAR struct file *filep)
{
  DEBUGASSERT(filep != NULL);
  DEBUGASSERT(filep->f_inode->i_private != NULL);
  struct hpm_mbx_lowerhalf_s *mbx_dev = (struct hpm_mbx_lowerhalf_s *)filep->f_inode->i_private;
  int ret = 0;
  nxmutex_lock(&mbx_dev->mbx_lock);
  if ((mbx_dev->base == NULL) || ((mbx_dev->base != HPM_MBX0A) && (mbx_dev->base != HPM_MBX0B)))
    {
      ret = -ENOENT;
    }
  else
    {
      if (mbx_dev->o_count > 0)
        {
          mbx_dev->o_count--;
        }
      else
        {
          ret = -ENODEV;
        }
    }
  nxmutex_unlock(&mbx_dev->mbx_lock);

  return ret;
}

/****************************************************************************
 * Name: mbx_read
 *
 * Description:
 *   the fifo depth is 4 word(4*32bit),so buflen must be 4-byte aligned
 *
 ****************************************************************************/

static ssize_t mbx_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  DEBUGASSERT(filep != NULL);
  DEBUGASSERT(filep->f_inode->i_private != NULL);
  ssize_t ret = 0;
  size_t rx_msg_count;
  hpm_stat_t stat; 
  struct hpm_mbx_lowerhalf_s *mbx_dev = (struct hpm_mbx_lowerhalf_s *)filep->f_inode->i_private;
  if ((buflen % 4))
    {
      return EINVAL;
    }

  nxmutex_lock(&mbx_dev->mbx_lock);
  if ((mbx_dev->base == NULL) || ((mbx_dev->base != HPM_MBX0A) && (mbx_dev->base != HPM_MBX0B)))
    {
      ret = -ENOENT;
    }
  else
    {
      rx_msg_count = (mbx_dev->base->SR & MBX_SR_RFVC_MASK) >> MBX_SR_RFVC_SHIFT;
      stat = mbx_retrieve_fifo(mbx_dev->base, (uint32_t *)buffer, rx_msg_count);
      if (stat != status_success)
        {
          ret = -EIO;
        }
      else
        {
          ret = (rx_msg_count << 2);
        }
    }
  nxmutex_unlock(&mbx_dev->mbx_lock);

  return ret;
}

/****************************************************************************
 * Name: mbx_open
 *
 * Description:
 *   the fifo depth is 4 word(4*32bit),so buflen must be 4-byte aligned
 *
 ****************************************************************************/

static ssize_t mbx_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
  DEBUGASSERT(filep != NULL);
  DEBUGASSERT(filep->f_inode->i_private != NULL);
  ssize_t ret = 0;
  size_t tx_slot_count;
  hpm_stat_t stat;
  struct hpm_mbx_lowerhalf_s *mbx_dev = (struct hpm_mbx_lowerhalf_s *)filep->f_inode->i_private;
  if ((buflen % 4))
    {
      return EINVAL;
    }

  nxmutex_lock(&mbx_dev->mbx_lock);
  if ((mbx_dev->base == NULL) || ((mbx_dev->base != HPM_MBX0A) && (mbx_dev->base != HPM_MBX0B)))
    {
      ret = -ENOENT;
    }
  else
    {
      tx_slot_count = (mbx_dev->base->SR & MBX_SR_TFEC_MASK) >> MBX_SR_TFEC_SHIFT;
      stat = mbx_send_fifo(mbx_dev->base, (uint32_t *)buffer, tx_slot_count);
      if (stat != status_success)
        {
          ret = -EIO;
        }
      else
        {
          ret = (tx_slot_count << 2);
        }
    }
  nxmutex_unlock(&mbx_dev->mbx_lock);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_mbx_register
 ****************************************************************************/

int hpm_mbx_register(FAR const char *path, FAR struct hpm_mbx_lowerhalf_s *mbx_dev)
{
  int ret;
  DEBUGASSERT(path != NULL);
  DEBUGASSERT(mbx_dev != NULL);

  /* Register the mbx character driver */

  ret = register_driver(path, &hpm_mbx_fops, 0666, mbx_dev);

  return ret;
}

#endif
