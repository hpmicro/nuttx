/****************************************************************************
 * boards/risc-v/hpmicro/hpm6750evk2-sdk/src/hpm6750_usbmsc.c
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
#include <syslog.h>
#include <errno.h>


#ifdef CONFIG_USBMSC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/



/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_usbmsc_initialize
 *
 * Description:
 *   Perform architecture specific initialization as needed to establish
 *   the mass storage device that will be exported by the USB MSC device.
 *   The ram disk is /dev/ram1.
 * 
 *   When NSH first comes up, you must manually create the RAM disk
 *   before exporting it:

 *  a) Create a 64Kb RAM disk at /dev/ram1:

 *    nsh> mkrd -m 1 -s 512 128

 *  b) Put a FAT file system on the RAM disk:

 *    nsh> mkfatfs /dev/ram1

 *  c) Now the 'msconn' command will connect to the host and
 *     export /dev/ram1 as the USB logical unit:

 *    nsh> msconn
 *
 ****************************************************************************/

int board_usbmsc_initialize(int port)
{
  /* please initialize the ram disk in nsh*/

  return OK;
}

#endif /* CONFIG_USBMSC */
