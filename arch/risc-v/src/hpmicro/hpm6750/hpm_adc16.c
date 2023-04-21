/****************************************************************************
 * arch/risc-v/src/hpmicro/hpm6750/hpm_adc16.c
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
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "board.h"
#include "hpm_adc16_drv.h"

#if CONFIG_HPM_ADC16_DRV
  
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef BOARD_APP_CORE
#define BOARD_APP_CORE  HPM_CORE0  
#endif

#define BOARD_APP_ADC_SEQ_DMA_BUFF_LEN_IN_4BYTES (1024U)
#define BOARD_APP_ADC_PMT_DMA_BUFF_LEN_IN_4BYTES (48U)
#define BOARD_APP_SEQ_START_POS                  (0U)
#define BOARD_APP_PMT_START_POS                  CONFIG_HPM6750_ADC3_TRIG_SOURCE     

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{ 
    const struct adc_callback_s *cb;
    char                        *name;
    char                        *path;
    ADC16_Type                  *base;
    int                          irq;
    xcpt_t                       isr; 
    uint32_t                     mask;
    adc16_config_t               cfg;
    adc16_channel_config_t       ch_cfg;
    adc16_prd_config_t           prd_cfg;
    adc16_seq_config_t           seq_cfg;    
    adc16_dma_config_t           dma_cfg;   
    adc16_pmt_config_t           pmt_cfg; 
    uint32_t                     *seq_buff;
    uint32_t                     *pmt_buff;           
    uint8_t                      *seq_channel_list;
    uint8_t                      *trig_channel_list; 
    __IO uint8_t                 seq_full_complete_flag;
    __IO uint8_t                 trig_complete_flag;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* ADC methods */
static int  adc_bind(struct adc_dev_s *dev, const struct adc_callback_s *callback);
static void adc_reset(struct adc_dev_s *dev);
static int  adc_setup(struct adc_dev_s *dev);
static void adc_shutdown(struct adc_dev_s *dev);
static void adc_rxint(struct adc_dev_s *dev, bool enable);
static int  adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg);

/* ADC Interrupt Handler */
static int adc_interrupt(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/
 
#if defined(CONFIG_HPM_ADC3) && CONFIG_HPM_ADC3
    #if defined(CONFIG_HPM_ADC3_CONV_MODE) && (CONFIG_HPM_ADC3_CONV_MODE == 2)
    static uint8_t adc3_seq_channel_list[] = {CONFIG_HPM_ADC3_CH};
    static ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT) uint32_t adc3_seq_buff[BOARD_APP_ADC_SEQ_DMA_BUFF_LEN_IN_4BYTES];
    #elif defined(CONFIG_HPM_ADC3_CONV_MODE) && (CONFIG_HPM_ADC3_CONV_MODE == 3)
    static uint8_t adc3_trig_channel_list[] = {CONFIG_HPM_ADC3_CH};
    static ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT) uint32_t adc3_pmt_buff[BOARD_APP_ADC_PMT_DMA_BUFF_LEN_IN_4BYTES];
    #endif
#endif

static const struct adc_ops_s g_adc_ops =
{
  .ao_bind     = adc_bind,    
  .ao_reset    = adc_reset,   
  .ao_setup    = adc_setup,  
  .ao_shutdown = adc_shutdown,
  .ao_rxint    = adc_rxint,   
  .ao_ioctl    = adc_ioctl,   
};
  
#if defined(CONFIG_HPM_ADC3) && CONFIG_HPM_ADC3 
static struct up_dev_s g_adc3_priv = 
{  
    /* instance name */
    .name                   = "ADC3",
    /* device path */
    .path                   = "/dev/adc3",
    /* instance base */
    .base                   = HPM_ADC3,  
    /* interrupt config */     
    .irq                    = HPM_IRQn_ADC3,  
    .isr                    = adc_interrupt, 
    .mask                   = 0, 
    /* common config */    
    .cfg                    = { .res          = CONFIG_HPM_ADC3_RES,    
                                .conv_mode    = CONFIG_HPM_ADC3_CONV_MODE,   
                                .adc_clk_div  = adc16_clock_divider_4,                             
                                .sel_sync_ahb = false,                     
                                .wait_dis     = true,                        
                                .adc_ahb_en   = false,                     
                                },
    /* channel config */
    .ch_cfg                 = {
                                .ch                 = CONFIG_HPM_ADC3_CH,                
                                .thshdh             = 0,
                                .thshdl             = 0,
                                .sample_cycle_shift = 0,       
                                .sample_cycle       = 20       
                                },
    /* oneshot mode */
    #if CONFIG_HPM_ADC3_CONV_MODE == 0             
 
    /* period mode */
    #elif CONFIG_HPM_ADC3_CONV_MODE == 1 
    .prd_cfg                = {
                                .ch           = CONFIG_HPM_ADC3_CH,
                                .prescale     = 22,                
                                .period_count = 5            
                                },
    /* sequence mode */
    #elif CONFIG_HPM_ADC3_CONV_MODE == 2 
    .seq_cfg                = {
                                .seq_len    = sizeof(adc3_seq_channel_list),   
                                .restart_en = false,               
                                .cont_en    = true,                
                                .sw_trig_en = true,                
                                .hw_trig_en = true                 
                                 },                             
    .dma_cfg                = {
                                .start_addr         = (uint32_t)adc3_seq_buff,
                                .buff_len_in_4bytes = sizeof(adc3_seq_channel_list),
                                .stop_en            = false,   
                                .stop_pos           = 0           
                                },
    .seq_buff               = adc3_seq_buff,
    .seq_channel_list       = adc3_seq_channel_list,
    /* preemption mode */                             
    #elif CONFIG_HPM_ADC3_CONV_MODE == 3
    .pmt_cfg                = {                            
                                .trig_ch = BOARD_APP_PMT_START_POS, 
                                .trig_len = sizeof(adc3_trig_channel_list)
                                },
    .pmt_buff               = adc3_pmt_buff,
    .trig_channel_list      = adc3_trig_channel_list
    #else
        #error "Not supported mode!"
    #endif                             
}; 
#endif

static struct adc_dev_s g_adc_dev[] = 
{
    #if defined(CONFIG_HPM_ADC3) && CONFIG_HPM_ADC3
    {   
        .ad_ops  = &g_adc_ops,
        .ad_priv = &g_adc3_priv
    },
    #endif
};
       
/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void init_trigger_cfg(struct adc_dev_s *dev, uint8_t trig_ch, bool inten)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->ad_priv;  

    for (int i = 0; i < priv->pmt_cfg.trig_len; i++) {
        priv->pmt_cfg.adc_ch[i] = priv->trig_channel_list[i];
        priv->pmt_cfg.inten[i] = false;
    }

    priv->pmt_cfg.inten[priv->pmt_cfg.trig_len - 1] = inten;

    adc16_set_pmt_config(priv->base, &priv->pmt_cfg);
}

static void init_common_config(struct adc_dev_s *dev)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->ad_priv; 

    if (priv->cfg.conv_mode == adc16_conv_mode_sequence || priv->cfg.conv_mode == adc16_conv_mode_preemption) {
        priv->cfg.adc_ahb_en = true;
    }
   
    #if defined(CONFIG_HPM_ADC3_NON_BLOCKING_READ) && CONFIG_HPM_ADC3_NON_BLOCKING_READ
    priv->cfg.wait_dis = true;
    #else 
    priv->cfg.wait_dis = false;
    #endif

    adc16_init(priv->base, &priv->cfg); 
    up_enable_irq(priv->irq);  
}

static void init_oneshot_config(struct adc_dev_s *dev)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->ad_priv;

    adc16_init_channel(priv->base, &priv->ch_cfg);
}

static void init_period_config(struct adc_dev_s *dev)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->ad_priv;

    adc16_init_channel(priv->base, &priv->ch_cfg);
    adc16_set_prd_config(priv->base, &priv->prd_cfg);
}

static void init_sequence_config(struct adc_dev_s *dev)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->ad_priv;  
     
    for (int i = 0; i < priv->seq_cfg.seq_len; i++) {
        priv->ch_cfg.ch = priv->seq_channel_list[i];
        adc16_init_channel(priv->base, &priv->ch_cfg);
    }

    for (int i = 0; i < priv->seq_cfg.seq_len; i++) { 
        priv->seq_cfg.queue[i].seq_int_en = 0;
        priv->seq_cfg.queue[i].ch = priv->seq_channel_list[i];
    }

    /* Initialize a sequence */    
    adc16_set_seq_config(priv->base, &priv->seq_cfg);

    /* Set DMA config */
    priv->dma_cfg.start_addr = (uint32_t *)core_local_mem_to_sys_address(BOARD_APP_CORE, (uint32_t)priv->seq_buff);   

    /* Initialize DMA for the sequence mode */
    adc16_init_seq_dma(priv->base, &priv->dma_cfg);

    /* Set interrupt mask */
    priv->mask = adc16_event_seq_full_complete;  
}
  
static void init_preemption_config(struct adc_dev_s *dev)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->ad_priv;    

    for (int i = 0; i < priv->pmt_cfg.trig_len; i++) {
        priv->ch_cfg.ch = priv->trig_channel_list[i];
        adc16_init_channel(priv->base, &priv->ch_cfg);
    }

    /* Trigger config initialization */
    init_trigger_cfg(dev, priv->pmt_cfg.trig_ch, true);

    /* Set DMA start address for preemption mode */
    adc16_init_pmt_dma(priv->base, core_local_mem_to_sys_address(BOARD_APP_CORE, (uint32_t)priv->pmt_buff));   

    /* Set interrupt mask */
    priv->mask = adc16_event_trig_complete;
}


static hpm_stat_t process_seq_data(struct adc_dev_s *dev)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->ad_priv;
    adc16_seq_dma_data_t *dma_data = (adc16_seq_dma_data_t *)priv->seq_buff;
    uint32_t len = priv->seq_cfg.seq_len;

    if (ADC16_IS_SEQ_DMA_BUFF_LEN_INVLAID(len)) {
        return status_invalid_argument;
    }    

    for (int i = 0; i < len; i++) {
        ainfo("Sequence Mode - %s - ", priv->name);
        ainfo("Sequence Number:%02d ", dma_data[i].seq_num);
        ainfo("Cycle Bit: %02d ", dma_data[i].cycle_bit);
        ainfo("ADC Channel: %02d ", dma_data[i].adc_ch);
        ainfo("Result: 0x%04x\n", dma_data[i].result);
    }    

    return status_success;
}

static hpm_stat_t process_pmt_data(struct adc_dev_s *dev)
{
    struct up_dev_s *priv     = (struct up_dev_s *)dev->ad_priv;
    adc16_pmt_dma_data_t *dma_data = (adc16_pmt_dma_data_t *)priv->pmt_buff;
    int start_pos = priv->pmt_cfg.trig_ch * sizeof(adc16_pmt_dma_data_t);
    uint32_t len = priv->pmt_cfg.trig_len;    

    if (ADC16_IS_PMT_DMA_BUFF_LEN_INVLAID(len)) {
        return status_invalid_argument;
    }

    for (int i = start_pos; i < start_pos + len; i++) {
        ainfo("Cycle Bit: %02d ", dma_data[i].cycle_bit);
        if (dma_data[i].cycle_bit) {
            ainfo("Preemption Mode - %s - ", priv->name); 
            ainfo("ADC Channel: %02d ", dma_data[i].adc_ch);
            ainfo("Trig Index: %02d ", dma_data[i].trig_index);
            ainfo("Trig Channel: %02d ", dma_data[i].trig_ch);
            ainfo("Result: 0x%04x\n", dma_data[i].result);
            dma_data[i].cycle_bit = 0;
        } else {
            ainfo("invalid data\n");
        }
    }

    return status_success;
}

static void sequence_handler(struct adc_dev_s *dev)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->ad_priv;
 
    /* SW trigger */
    adc16_trigger_seq_by_sw(priv->base);

    while (priv->seq_full_complete_flag == 0) {
 
    }
    /* Process data */
    process_seq_data(dev);

    /* Clear the flag */
    priv->seq_full_complete_flag = 0;
}
 
static void preemption_handler(struct adc_dev_s *dev)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->ad_priv;

    /* SW trigger */
    adc16_trigger_pmt_by_sw(priv->base, priv->pmt_cfg.trig_ch);

    /* Wait for a complete of conversion */
    while (priv->trig_complete_flag == 0) {

    }

    /* Process data */
    process_pmt_data(dev);

    /* Clear the flag */
    priv->trig_complete_flag = 0;
}

static int adc_interrupt_handler(struct adc_dev_s *dev)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->ad_priv;
    uint32_t status;

    status = adc16_get_status_flags(priv->base);

    if (ADC16_INT_STS_SEQ_CMPT_GET(status)) {
        /* Clear seq_complete status */
        adc16_clear_status_flags(priv->base, adc16_event_seq_full_complete);
        /* Set flag to read memory data */
        priv->seq_full_complete_flag = 1;
    }

    if (ADC16_INT_STS_TRIG_CMPT_GET(status)) {
        /* Clear trig_cmpt status */
        adc16_clear_status_flags(priv->base, adc16_event_trig_complete);
        /* Set flag to read memory data */
        priv->trig_complete_flag = 1;
    }

    return OK;
}

/****************************************************************************
 * Name: adc_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int adc_bind(struct adc_dev_s *dev, const struct adc_callback_s *callback)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->ad_priv;

    DEBUGASSERT(priv != NULL);
    if (priv == NULL) {
        return -ENOBUFS;
    }
    priv->cb = callback;
    
    return OK;
}

/****************************************************************************
 * Name: adc_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware. This
 *   is called, before adc_setup() and on error conditions.
 *
 ****************************************************************************/

static void adc_reset(struct adc_dev_s *dev)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->ad_priv;
    irqstate_t flags;

    flags = enter_critical_section();

    /* ADC pin initialization */
    board_init_adc16_pins();

    /* ADC clock initialization */
    board_init_adc16_clock(priv->base);

    leave_critical_section(flags);
}

/****************************************************************************
 * Name: adc_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching ADC interrupts.
 *   Interrupts are all disabled upon return.
 *
 ****************************************************************************/

static int adc_setup(struct adc_dev_s *dev)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->ad_priv;

    /* Attach the ADC interrupt */
    int ret = irq_attach(priv->irq, priv->isr, dev);
    if (ret < 0) {
        ainfo("irq_attach failed: %d\n", ret);
        return ret;
    }

    up_enable_irq(priv->irq);
  
    /* ADC16 common initialization */
    init_common_config(dev);

    /* ADC16 read patter and DMA initialization */
    switch (priv->cfg.conv_mode) {
        case adc16_conv_mode_oneshot:
            init_oneshot_config(dev);
            break;

        case adc16_conv_mode_period:
            init_period_config(dev);
            break;

        case adc16_conv_mode_sequence:
            init_sequence_config(dev);
            break;

        case adc16_conv_mode_preemption:
            init_preemption_config(dev);
            break;

        default:
            break;
    }

    return ret;
}

/****************************************************************************
 * Name: adc_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the ADC device is closed.
 *   This method reverses the operation the setup method.
 *
 ****************************************************************************/

static void adc_shutdown(struct adc_dev_s *dev)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->ad_priv;

    /* Disable ADC interrupts */
    up_disable_irq(priv->irq);

    /* Then detach the ADC interrupt handler. */
    irq_detach(priv->irq);
}

/****************************************************************************
 * Name: adc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void adc_rxint(struct adc_dev_s *dev, bool enable)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->ad_priv;

    if (enable == true) {
        priv->base->INT_EN |= priv->mask;
    } else {
        priv->base->INT_EN &= ~priv->mask;
    }
}

/****************************************************************************
 * Name: adc_ioctl
 *
 * Description:
 *  All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg) 
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->ad_priv;
    uint16_t result;
    int cnt = 0;
    int ret = OK;

    switch (cmd) { 
        case ANIOC_TRIGGER:
            {            
                if (priv->cfg.conv_mode == adc16_conv_mode_oneshot) {
                    while (cnt++ < 10) {
                        if (adc16_get_oneshot_result(priv->base, priv->ch_cfg.ch, &result) == status_success) {
                            ainfo("oneshot mode: read result\n");
                            priv->cb->au_receive(dev, priv->ch_cfg.ch, (int32_t)result);
                            break;
                        } 
                    }
                    ret = cnt > 10 ? -EAGAIN : OK;
                } else if (priv->cfg.conv_mode == adc16_conv_mode_period) {
                    ainfo("period mode: read result\n");
                    up_mdelay(120);
                    adc16_get_prd_result(priv->base, priv->ch_cfg.ch, &result);
                    priv->cb->au_receive(dev, priv->ch_cfg.ch, (int32_t)result);
                } else if (priv->cfg.conv_mode == adc16_conv_mode_sequence){
                    adc16_seq_dma_data_t *dma_data = (adc16_seq_dma_data_t *)priv->seq_buff;
                    ainfo("sequence mode: read reuslt\n");
                    sequence_handler(dev);
                    priv->cb->au_receive(dev, dma_data[BOARD_APP_SEQ_START_POS].adc_ch, (int32_t)dma_data[BOARD_APP_SEQ_START_POS].result);
                } else if (priv->cfg.conv_mode == adc16_conv_mode_preemption) {
                    adc16_pmt_dma_data_t *dma_data = (adc16_pmt_dma_data_t *)priv->pmt_buff;
                    uint32_t offset = priv->pmt_cfg.trig_ch * sizeof(adc16_pmt_dma_data_t);
                    ainfo("preemption mode: read reuslt\n");
                    preemption_handler(dev);
                    priv->cb->au_receive(dev, dma_data[offset].adc_ch, (int32_t)dma_data[offset].result);
                } else {
                    ret = -EINVAL;
                }
                break;
            }
        default: break;
    }

    return ret;
}

/****************************************************************************
 * Name: adc_interrupt
 *
 * Description:
 *   ADC interrupt handler
 *
 ****************************************************************************/

static int adc_interrupt(int irq, void *context, void *arg)
{
    struct adc_dev_s *dev = (struct adc_dev_s *)arg;
    adc_interrupt_handler(dev);

    return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hpm_adcinitialize
 *
 * Description:
 *   Initialize the adc
 *
 * Returned Value:
 *   Valid adc device structure reference on success; a NULL on failure
 *
 ****************************************************************************/ 
int hpm_adc16_setup(void)
{ 
    static bool initialized = false; 
    int ret;  
    
    /* Check if we have already initialized */
    if (!initialized)
    { 
        /* Register the ADC driver at "/dev/adcx" */
        for (int i = 0; i < sizeof(g_adc_dev) / sizeof(g_adc_dev[0]); i++) {           
            ret = adc_register(((struct up_dev_s *)(g_adc_dev[i].ad_priv))->path, &g_adc_dev[i]);
            if (ret < 0)
            {
                aerr("ERROR: adc_register failed: %d\n", ret);
                return ret;
            }
        }
        /* Now we are initialized */
        initialized = true;
    }

    return OK;
}
    
#endif 