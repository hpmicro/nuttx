/****************************************************************************
 * arch/risc-v/src/hpmicro/hpm6750/hpm_dma.c
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

#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include "chip.h"
#include "riscv_internal.h"
#include "hpm_dma.h"

#if defined(CONFIG_HPM_DMA_DRV) && defined(CONFIG_HPM_COMPONENTS_DMA_MANAGER)

/****************************************************************************
 * Pre-processor Macros
 ****************************************************************************/

#define DMA_RESOURCE_NUM 16

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct _dmamux_channel_context {
    DMA_Type *base;             /**< The DMA intance that the allocated channel belongs to */
    uint32_t channel;           /**< Channel index */
    bool     is_allocated;      /**< Whether DMA channel was allocated */
} hpm_dmamux_channel_context_t;

typedef struct _dma_channel_source {  
    DMA_Type *base;                         /**< The DMA intance that the allocated channel belongs to */
    uint32_t channel;                       /**< Channel index */
    int      os_irq_num;                       /*irq number*/
    void     *user_data;
    hpm_dma_channel_callback_t callback;    /*dma callback*/
}dma_channel_source_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static hpm_dma_resource_t   dma_resource_pools[DMA_RESOURCE_NUM];
static dma_channel_source_t dmasource_pools[DMA_RESOURCE_NUM];

static hpm_dmamux_channel_context_t dmamux_context_pools[DMA_SOC_MAX_COUNT][DMA_SOC_CHANNEL_NUM];;
static hpm_dmamux_resource_t dmamux_resource_pools[DMA_SOC_MAX_COUNT][DMA_SOC_CHANNEL_NUM];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_dmainterrupt
 *
 * Description:
 *  DMA interrupt handler
 *
 ****************************************************************************/

static int hpm_dmainterrupt(int irq, void *context, void *arg)
{
  for(uint8_t i = 0; i < DMA_RESOURCE_NUM; i++)
    {
      if(irq == dmasource_pools[i].os_irq_num)
        {
          uint32_t int_disable_mask = dma_check_channel_interrupt_mask(dmasource_pools[i].base, dmasource_pools[i].channel);
          /* If Channel interrupt is enabled */
          if (int_disable_mask != (DMA_INTERRUPT_MASK_ERROR | DMA_INTERRUPT_MASK_ABORT | DMA_INTERRUPT_MASK_TERMINAL_COUNT)) 
            {
                uint32_t chn_int_stat = dma_check_transfer_status(dmasource_pools[i].base, dmasource_pools[i].channel);
                if (chn_int_stat != DMA_CHANNEL_STATUS_ONGOING) 
                  {
                    if(dmasource_pools[i].callback != NULL)
                      {
                        dmasource_pools[i].callback(dmasource_pools[i].base, dmasource_pools[i].channel, dmasource_pools[i].user_data, chn_int_stat);
                      }
                  } 
            }     
        }
    }
}

/****************************************************************************
 * Name: hpm_dma_channel_request
 *
 * Description:
 *   Allocate a new DMA channel.
 *
 * Input Parameters:
 *   callback: interrupt callback
 *    arg: user point data
 *
 * Returned Value:
 *   0-15: DMA channel
 *   -1: Failed
 *
 ****************************************************************************/

int8_t hpm_dma_channel_request(hpm_dma_channel_callback_t callback, void *arg)
{
  for (uint32_t i = 0; i < ARRAY_SIZE(dma_resource_pools); i++)
    {
      if (!dma_resource_pools[i].base)
        {
          hpm_dma_resource_t *resource = &dma_resource_pools[i];
          if(dma_manager_request_resource(resource) == status_success)
            {
              dmasource_pools[i].base        = resource->base;
              dmasource_pools[i].channel     = resource->channel;
              dmasource_pools[i].os_irq_num  = (resource->irq_num + HPM_IRQ_PERI_START);
              dmasource_pools[i].callback    = callback;
              dmasource_pools[i].user_data   = arg;
              return i;
            }
        }
    }

  return -1;
}

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

int hpm_dma_channel_release(uint8_t channel_id)
{
  if ((channel_id >= DMA_RESOURCE_NUM))
    {
      return -1;
    }

  if (!dma_resource_pools[channel_id].base)
    {
      return -1;
    }
  
  hpm_dma_resource_t *resource = &dma_resource_pools[channel_id];
  dma_manager_disable_channel_interrupt(resource, DMA_INTERRUPT_MASK_TERMINAL_COUNT);
  dma_manager_release_resource(resource);

  memset(&dma_resource_pools[channel_id], 0, sizeof(dma_resource_pools[channel_id]));

  return 0;
}

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

int hpm_dma_channel_start(uint8_t channel_id)
{
  if ((channel_id >= DMA_RESOURCE_NUM))
    {
      return -1;
    }
  
  hpm_dma_resource_t *resource = &dma_resource_pools[channel_id];
  dma_manager_enable_dma_interrupt(resource, DMA_INTERRUPT_MASK_TERMINAL_COUNT);
  if (dma_enable_channel(resource->base, resource->channel) == status_success)
    {
      return 0;
    }

  return -1;
}

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

int hpm_dma_channel_stop(uint8_t channel_id)
{
  if ((channel_id >= DMA_RESOURCE_NUM))
    {
      return -1;
    }
  
  hpm_dma_resource_t *resource = &dma_resource_pools[channel_id];
  dma_manager_disable_channel_interrupt(resource, DMA_INTERRUPT_MASK_TERMINAL_COUNT);
  dma_disable_channel(resource->base, resource->channel);

  return 0;
}

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

int hpm_dma_channel_get_resource(uint8_t channel_id, hpm_dma_resource_t *resource)
{
  if ((channel_id >= DMA_RESOURCE_NUM) || (!resource))
    {
      return -1;
    }
  
  memcpy(resource, &dma_resource_pools[channel_id], sizeof(dma_resource_pools[channel_id]));
    
  return 0;
}

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

int hpm_dmamux_channel_request(uint8_t dma_channel_id, uint16_t dmamux_req)
{
  if ((dma_channel_id >= DMA_RESOURCE_NUM))
    {
      return -1;
    }
  int32_t instance;
  uint32_t channel;
  hpm_dma_resource_t *resource = &dma_resource_pools[dma_channel_id];
  for (instance = 0; instance < DMA_SOC_MAX_COUNT; instance++)
    {
      if (resource->base == dmamux_context_pools[instance]->base)
        {
          for (channel = 0; channel < DMA_SOC_CHANNEL_NUM; channel++)
          {
            if (dmamux_context_pools[instance][channel].is_allocated== false)
              {
                dmamux_context_pools[instance][channel].is_allocated    = true;
                dmamux_context_pools[instance][channel].channel         = resource->channel;
                dmamux_resource_pools[instance][channel].base           = resource->base;
                dmamux_resource_pools[instance][channel].dma_req        = dmamux_req;
                dmamux_resource_pools[instance][channel].dmamux_channel = DMA_SOC_CHN_TO_DMAMUX_CHN(resource->base, resource->channel);
                return dma_channel_id;
              }
          }       
        }
    } 

    return -1;
}

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

int hpm_dmamux_channel_release(uint8_t dma_channel_id)
{
  if ((dma_channel_id >= DMA_RESOURCE_NUM))
    {
      return -1;
    }
  
  int32_t instance;
  uint32_t channel;
  hpm_dma_resource_t *resource = &dma_resource_pools[dma_channel_id];
  if (!resource->base)
    {
      return -1;
    }

  for (instance = 0; instance < DMA_SOC_MAX_COUNT; instance++)
    {
      if (resource->base == dmamux_context_pools[instance]->base)
        {
          for (channel= 0; channel < DMA_SOC_CHANNEL_NUM; channel++)
            {
              if ((dmamux_context_pools[instance][channel].is_allocated == true) &&
                  (dmamux_context_pools[instance][channel].channel      == resource->channel))
                {
                  dmamux_context_pools[instance][channel].is_allocated    = false;
                }
            }
        }
    }

  return 0;
}

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

int hpm_dmamux_channel_get_resource(uint8_t dma_channel_id, hpm_dmamux_resource_t *resource)
{
  if ((dma_channel_id >= DMA_RESOURCE_NUM) || (!resource))
    {
      return -1;
    }
  
  int32_t instance;
  uint32_t channel;
  hpm_dma_resource_t *resource_dma = &dma_resource_pools[dma_channel_id];
   if (!resource_dma->base)
    {
      return -1;
    }

  for (instance = 0; instance < DMA_SOC_MAX_COUNT; instance++)
    {
      if (resource_dma->base == dmamux_context_pools[instance]->base)
        {
          for (channel= 0; channel < DMA_SOC_CHANNEL_NUM; channel++)
            {
              if ((dmamux_context_pools[instance][channel].is_allocated == true) &&
                  (dmamux_context_pools[instance][channel].channel      == resource_dma->channel))
                {
                  memcpy(resource, &dmamux_resource_pools[instance][channel].base, sizeof(dmamux_resource_pools[instance][channel]));
                  return 0;
                }
            }
        }
    }

  return -1;
}

/****************************************************************************
 * Name: riscv_dma_initialize
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

void weak_function riscv_dma_initialize(void)
{
  memset(dma_resource_pools, 0, sizeof(dma_resource_pools));
  memset(dmamux_context_pools, 0, sizeof(dmamux_context_pools));
  dmamux_context_pools[0]->base = HPM_HDMA;
  dmamux_context_pools[1]->base = HPM_XDMA;
  dma_manager_init();

  /* Attach DMA interrupt vectors */

  irq_attach(HPM_IRQn_HDMA, hpm_dmainterrupt, NULL);
  irq_attach(HPM_IRQn_XDMA, hpm_dmainterrupt, NULL);

  /* Enable the IRQ at the NVIC (still disabled at the DMA controller) */

  up_enable_irq(HPM_IRQn_HDMA);
  up_enable_irq(HPM_IRQn_XDMA);

}

#endif
