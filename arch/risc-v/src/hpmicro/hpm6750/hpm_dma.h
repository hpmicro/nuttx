/****************************************************************************
 * arch/risc-v/src/bl602/bl602_dma.h
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

#ifndef __ARCH_RISCV_SRC_HPMICRO_HPM6750_DMA_H
#define __ARCH_RISCV_SRC_HPMICRO_HPM6750_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#if defined(CONFIG_HPM_DMA_DRV) && defined(CONFIG_HPM_COMPONENTS_DMA_MANAGER)

#include "hpm_dma_manager.h"
#include "hpm_dma_drv.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/
typedef struct _dmamux_resource {
    DMA_Type *base;                    /**< The DMA intance that the allocated channel belongs to */
    uint32_t dmamux_channel;           /**< dmamux Channel index */
    uint32_t dma_req;                  /*dma request*/               
} hpm_dmamux_resource_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/


/****************************************************************************
 * Name: hpm_dma_channel_request
 *
 * Description:
 *   Allocate a new DMA channel.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0-15: DMA channel
 *   -1: Failed
 *
 ****************************************************************************/

int8_t hpm_dma_channel_request(hpm_dma_channel_callback_t callback, void *arg);

/****************************************************************************
 * Name: hpm_dma_channel_release
 *
 * Description:
 *   Release a DMA channel.
 *
 * Input Parameters:
 *   channel: DMA channel.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int hpm_dma_channel_release(uint8_t channel_id);

/****************************************************************************
 * Name: hpm_dma_channel_start
 *
 * Description:
 *   Start a DMA channel.
 *
 * Input Parameters:
 *   channel: DMA channel.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int hpm_dma_channel_start(uint8_t channel_id);

/****************************************************************************
 * Name: hpm_dma_channel_stop
 *
 * Description:
 *   Stop a DMA channel.
 *
 * Input Parameters:
 *   channel: DMA channel.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int hpm_dma_channel_stop(uint8_t channel_id);

/****************************************************************************
 * Name: hpm_dma_channel_get_resource
 *
 * Description:
 *   get DMA channel resooure
 *
 * Input Parameters:
 *   channel: DMA channel.
 *
 * Output Parameters: 
 *   resource: DMA channel resource strut
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int hpm_dma_channel_get_resource(uint8_t channel_id, hpm_dma_resource_t *resource);

/****************************************************************************
 * Name: hpm_dmamux_channel_request
 *
 * Description:
 *   Allocate a new dmamux channel.
 *
 * Input Parameters:
 *   dma_channel_id: dma channel id ,it's from hpm_dma_channel_request return value
 *
 * Returned Value:
 *  Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int hpm_dmamux_channel_request(uint8_t dma_channel_id, uint16_t dmamux_req);

/****************************************************************************
 * Name: hpm_dmamux_channel_release
 *
 * Description:
 *   Release a DMAmux channel.
 *
 * Input Parameters:
 *   channel: DMA channel.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int hpm_dmamux_channel_release(uint8_t dma_channel_id);

/****************************************************************************
 * Name: hpm_dmamux_channel_get_resource
 *
 * Description:
 *   get DMAMUX channel resooure
 *
 * Input Parameters:
 *   channel: DMA channel.
 *
 * Output Parameters: 
 *   resource: DMAMUX channel resource strut
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int hpm_dmamux_channel_get_resource(uint8_t dma_channel_id, hpm_dmamux_resource_t *resource);

/****************************************************************************
 * Name: hpm_dma_init
 *
 * Description:
 *   Initialize DMA driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void hpm_dma_init(void);

#endif

#endif /* __ARCH_RISCV_SRC_HPMICRO_HPM6750_DMA_H */

