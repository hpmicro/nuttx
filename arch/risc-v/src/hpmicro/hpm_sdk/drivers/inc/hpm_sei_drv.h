/*
 * Copyright (c) 2023 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef HPM_SEI_DRV_H
#define HPM_SEI_DRV_H

#include "hpm_common.h"
#include "hpm_sei_regs.h"
#include "hpm_soc_feature.h"

/**
 * @brief sei arming action
 */
typedef enum {
    sei_arming_direct_exec = 0,
    sei_arming_wait_trigger
} sei_arming_mode_t;

/**
 * @brief sei watchdog action
 */
typedef enum {
    sei_wdg_exec_next_instr = 0,
    sei_wdg_exec_exception_instr
} sei_wdg_action_t;

/**
 * @brief sei transfer mode
 */
typedef enum {
    sei_synchronous_master_mode = 0,
    sei_synchronous_slave_mode,
    sei_asynchronous_mode
} sei_tranceiver_mode_t;

/**
 * @brief sei asynchronous mode parity
 */
typedef enum {
    sei_asynchronous_parity_even = 0,
    sei_asynchronous_parity_odd
} sei_asynchronous_parity_t;

/**
 * @brief sei ilde state
 */
typedef enum {
    sei_idle_low_state = 0,
    sei_idle_high_state,
} sei_idle_state_t;

/**
 * @brief sei data mode
 */
typedef enum {
    sei_data_mode = 0,
    sei_check_mode,
    sei_crc_mode
} sei_data_mode_t;

/**
 * @brief sei data bit order
 */
typedef enum {
    sei_bit_lsb_first = 0,
    sei_bit_msb_first
} sei_data_bit_order_t;

/**
 * @brief sei data word order
 */
typedef enum {
    sei_word_nonreverse = 0,
    sei_word_reverse
} sei_data_word_order_t;

/**
 * @brief sei state transition condition
 */
typedef enum {
    sei_state_transition_condition_high = 0,
    sei_state_transition_condition_low,
    sei_state_transition_condition_rise,
    sei_state_transition_condition_fall
} sei_state_transition_condition_t;

/**
 * @brief sei select command or data
 */
#define SEI_SELECT_CMD true
#define SEI_SELECT_DATA false

/**
 * @brief sei state transition enabled state
 */
#define SEI_STATE_TRANSITION_ENABLED_STATE0 1u
#define SEI_STATE_TRANSITION_ENABLED_STATE1 2u
#define SEI_STATE_TRANSITION_ENABLED_STATE2 4u
#define SEI_STATE_TRANSITION_ENABLED_STATE3 8u

/**
 * @brief sei instruction operation command
 */
#define SEI_INSTR_OP_HALT 0u
#define SEI_INSTR_OP_JUMP 1u
#define SEI_INSTR_OP_SEND_WDG 2u
#define SEI_INSTR_OP_SEND 3u
#define SEI_INSTR_OP_WAIT_WDG 4u
#define SEI_INSTR_OP_WAIT 5u
#define SEI_INSTR_OP_RECV_WDG 6u
#define SEI_INSTR_OP_RECV 7u

/**
 * @brief sei instruction synchronous master clock type
 */
#define SEI_INSTR_CK_LOW 0u
#define SEI_INSTR_CK_RISE_FALL 1u
#define SEI_INSTR_CK_FALL_RISE 2u
#define SEI_INSTR_CK_HIGH 3u

/**
 * @brief sei instruction synchronous slave clock type
 */
#define SEI_INSTR_CK_DEFAULT 0u
#define SEI_INSTR_CK_TRX_EXCH 1u
#define SEI_INSTR_CK_TO_EN 2u
#define SEI_INSTR_CK_TRX_EXCH_TO_EN 3u

/**
 * @brief sei engine config structure
 */
typedef struct {
    sei_arming_mode_t arming_mode;
    uint8_t data_cdm_idx;
    uint8_t data_base_idx;
    uint8_t init_instr_ptr_idx;
    bool wdg_enable;
    sei_wdg_action_t wdg_action;
    uint8_t wdg_instr_ptr_idx;
    uint16_t wdg_time;
} sei_engine_config_t;

/**
 * @brief sei tranceiver synchronous master mode config structure
 */
typedef struct {
    bool data_idle_high_z;
    sei_idle_state_t data_idle_state;
    bool clock_idle_high_z;
    sei_idle_state_t clock_idle_state;
    uint32_t baudrate;
} sei_tranceiver_synchronous_master_config_t;

/**
 * @brief sei tranceiver synchronous master mode config structure
 */
typedef struct {
    bool data_idle_high_z;
    sei_idle_state_t data_idle_state;
    bool clock_idle_high_z;
    sei_idle_state_t clock_idle_state;
    uint32_t max_baudrate;
    uint16_t ck0_timeout_us;
    uint16_t ck1_timeout_us;
} sei_tranceiver_synchronous_slave_config_t;

/**
 * @brief sei tranceiver asynchronous mode config structure
 */
typedef struct {
    uint8_t wait_len;
    uint8_t data_len;
    bool parity_enable;
    sei_asynchronous_parity_t parity;
    bool data_idle_high_z;
    sei_idle_state_t data_idle_state;
    uint32_t baudrate;
} sei_tranceiver_asynchronous_config_t;

/**
 * @brief sei tranceiver config structure
 */
typedef struct {
    sei_tranceiver_mode_t mode;
    bool tri_sample;
    sei_tranceiver_synchronous_master_config_t synchronous_master_config;
    sei_tranceiver_synchronous_slave_config_t synchronous_slave_config;
    sei_tranceiver_asynchronous_config_t asynchronous_config;
} sei_tranceiver_config_t;

/**
 * @brief sei trigger input config structure
 */
typedef struct {
    bool trig0_enable;
    uint8_t trig0_select;
    bool trig1_enable;
    uint8_t trig1_select;
    bool trig_period_enable;
    sei_arming_mode_t trig_period_arming_mode;
    bool trig_period_sync_enable;
    uint8_t trig_period_sync_select;
    uint32_t trig_period_time;
} sei_trigger_input_config_t;

/**
 * @brief sei trigger output config structure
 */
typedef struct {
    bool trig0_enable;
    uint8_t trig0_select;
    bool trig1_enable;
    uint8_t trig1_select;
    bool trig2_enable;
    uint8_t trig2_select;
    bool trig3_enable;
    uint8_t trig3_select;
} sei_trigger_output_config_t;

/**
 * @brief sei data format config structure
 */
typedef struct {
    sei_data_mode_t mode;
    bool signed_flag;
    sei_data_bit_order_t bit_order;
    sei_data_word_order_t word_order;
    uint8_t word_len;
    bool crc_invert;
    bool crc_shift_mode;
    uint8_t crc_len;
    uint8_t last_bit_index;
    uint8_t first_bit_index;
    uint8_t highest_bit_index;
    uint8_t lowest_bit_index;
    uint32_t gold_value;
    uint32_t crc_init_value;
    uint32_t crc_poly;
} sei_data_format_config_t;

/**
 * @brief sei command table config structure
 */
typedef struct {
    uint32_t cmd_min_value;
    uint32_t cmd_max_value;
    uint32_t cmd_mask_value;
    uint8_t instr_idx[16];
} sei_command_table_config_t;

/**
 * @brief sei state transition config structure
 */
typedef struct {
    uint8_t enabled_state;
    bool override_instr_ptr_check;
    sei_state_transition_condition_t instr_ptr_cfg;
    uint8_t instr_ptr_value;
    bool override_clk_check;
    sei_state_transition_condition_t clk_cfg;
    bool override_txd_check;
    sei_state_transition_condition_t txd_cfg;
    bool override_rxd_check;
    sei_state_transition_condition_t rxd_cfg;
    bool override_timeout_check;
    sei_state_transition_condition_t timeout_cfg;
} sei_state_transition_config_t;

/**
 * @brief sei state transition latch config structure
 */
typedef struct {
    bool enable;
    uint8_t output_select;
    uint16_t delay;
} sei_state_transition_latch_config_t;

/**
 * @brief sei sample config structure
 */
typedef struct {
    bool enable_acc;
    uint8_t acc_data_idx;
    bool enable_spd;
    uint8_t spd_data_idx;
    bool enable_rev;
    uint8_t rev_data_idx;
    bool enable_pos;
    uint8_t pos_data_idx;
    uint8_t latch_select;
    bool sample_once;
    uint16_t sample_window;
    uint32_t data_register_select;
} sei_sample_config_t;

/**
 * @brief sei update config structure
 */
typedef struct {
    bool enable_acc;
    uint8_t acc_data_idx;
    bool enable_spd;
    uint8_t spd_data_idx;
    bool enable_rev;
    uint8_t rev_data_idx;
    bool enable_pos;
    uint8_t pos_data_idx;
    bool use_override_value;
    uint32_t override_acc_value;
    uint32_t override_spd_value;
    uint32_t override_rev_value;
    uint32_t override_pos_value;
    uint32_t override_time_value;
    bool update_on_err;
    uint8_t latch_select;
    uint32_t data_register_select;
} sei_update_config_t;


#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

static inline void sei_set_engine_enable(SEI_Type *ptr, uint8_t idx, bool enable)
{
    if (enable) {
        ptr->CTRL[idx].ENGINE.ENG_CTRL |= SEI_CTRL_ENGINE_ENG_CTRL_ENABLE_MASK;
    } else {
        ptr->CTRL[idx].ENGINE.ENG_CTRL &= ~SEI_CTRL_ENGINE_ENG_CTRL_ENABLE_MASK;
    }
}

static inline void sei_set_engine_rewind(SEI_Type *ptr, uint8_t idx)
{
    ptr->CTRL[idx].ENGINE.ENG_CTRL |= SEI_CTRL_ENGINE_ENG_CTRL_REWIND_MASK;
}

static inline void sei_set_trgger_input_trig0_enable(SEI_Type *ptr, uint8_t idx, bool enable)
{
    if (enable) {
        ptr->CTRL[idx].TRIGER.TRG_IN_CFG |= SEI_CTRL_TRIGER_TRG_IN_CFG_IN0_EN_MASK;
    } else {
        ptr->CTRL[idx].TRIGER.TRG_IN_CFG &= ~SEI_CTRL_TRIGER_TRG_IN_CFG_IN0_EN_MASK;
    }
}

static inline void sei_set_trgger_input_trig1_enable(SEI_Type *ptr, uint8_t idx, bool enable)
{
    if (enable) {
        ptr->CTRL[idx].TRIGER.TRG_IN_CFG |= SEI_CTRL_TRIGER_TRG_IN_CFG_IN1_EN_MASK;
    } else {
        ptr->CTRL[idx].TRIGER.TRG_IN_CFG &= ~SEI_CTRL_TRIGER_TRG_IN_CFG_IN1_EN_MASK;
    }
}

static inline void sei_set_trgger_input_period_enable(SEI_Type *ptr, uint8_t idx, bool enable)
{
    if (enable) {
        ptr->CTRL[idx].TRIGER.TRG_IN_CFG |= SEI_CTRL_TRIGER_TRG_IN_CFG_PRD_EN_MASK;
    } else {
        ptr->CTRL[idx].TRIGER.TRG_IN_CFG &= ~SEI_CTRL_TRIGER_TRG_IN_CFG_PRD_EN_MASK;
    }
}

static inline void sei_set_trigger_input_soft_enable(SEI_Type *ptr, uint8_t idx)
{
    ptr->CTRL[idx].TRIGER.TRG_SW |= SEI_CTRL_TRIGER_TRG_SW_SOFT_MASK;
}

static inline void sei_set_trig_command_value(SEI_Type *ptr, uint8_t idx, uint8_t table_idx, uint32_t data)
{
    ptr->CTRL[idx].TRG_TABLE.TRG_CMD[table_idx] = data;
}

static inline uint32_t sei_get_trig_time(SEI_Type *ptr, uint8_t idx, uint8_t table_idx)
{
    return ptr->CTRL[idx].TRG_TABLE.TRG_TIME[table_idx];
}

static inline void sei_set_xcvr_rx_point(SEI_Type *ptr, uint8_t idx, uint16_t point)
{
    uint32_t tmp;

    assert(point > 0);
    tmp = ptr->CTRL[idx].XCVR.TRX_DATA_CFG;
    tmp &= ~SEI_CTRL_XCVR_TRX_DATA_CFG_RXD_POINT_MASK;
    tmp |= SEI_CTRL_XCVR_TRX_DATA_CFG_RXD_POINT_SET(point);
    ptr->CTRL[idx].XCVR.TRX_DATA_CFG = tmp;
}

static inline void sei_set_xcvr_tx_point(SEI_Type *ptr, uint8_t idx, uint16_t point)
{
    uint32_t tmp;

    assert(point > 0);
    tmp = ptr->CTRL[idx].XCVR.TRX_DATA_CFG;
    tmp &= ~SEI_CTRL_XCVR_TRX_DATA_CFG_TXD_POINT_MASK;
    tmp |= SEI_CTRL_XCVR_TRX_DATA_CFG_TXD_POINT_SET(point);
    ptr->CTRL[idx].XCVR.TRX_DATA_CFG = tmp;
}

static inline void sei_set_xcvr_ck0_point(SEI_Type *ptr, uint8_t idx, uint16_t point)
{
    uint32_t tmp;

    assert(point > 0);
    tmp = ptr->CTRL[idx].XCVR.TRX_CLK_CFG;
    tmp &= ~SEI_CTRL_XCVR_TRX_CLK_CFG_CK0_POINT_MASK;
    tmp |= SEI_CTRL_XCVR_TRX_CLK_CFG_CK0_POINT_SET(point);
    ptr->CTRL[idx].XCVR.TRX_CLK_CFG = tmp;
}

static inline void sei_set_xcvr_ck1_point(SEI_Type *ptr, uint8_t idx, uint16_t point)
{
    uint32_t tmp;

    assert(point > 0);
    tmp = ptr->CTRL[idx].XCVR.TRX_CLK_CFG;
    tmp &= ~SEI_CTRL_XCVR_TRX_CLK_CFG_CK1_POINT_MASK;
    tmp |= SEI_CTRL_XCVR_TRX_CLK_CFG_CK1_POINT_SET(point);
    ptr->CTRL[idx].XCVR.TRX_CLK_CFG = tmp;
}

static inline uint16_t sei_get_xcvr_ck0_point(SEI_Type *ptr, uint8_t idx)
{
    return SEI_CTRL_XCVR_TRX_CLK_CFG_CK0_POINT_GET(ptr->CTRL[idx].XCVR.TRX_CLK_CFG);
}

static inline uint16_t sei_get_xcvr_ck1_point(SEI_Type *ptr, uint8_t idx)
{
    return SEI_CTRL_XCVR_TRX_CLK_CFG_CK1_POINT_GET(ptr->CTRL[idx].XCVR.TRX_CLK_CFG);
}

static inline void sei_set_command_value(SEI_Type *ptr, uint8_t idx, uint32_t cmd)
{
    ptr->CTRL[idx].COMMAND.CMD = cmd;
}

static inline void sei_set_command_rewind(SEI_Type *ptr, uint8_t idx)
{
    ptr->CTRL[idx].COMMAND.CMD_MODE |= SEI_CTRL_COMMAND_CMD_MODE_REWIND_MASK;
}

static inline uint32_t sei_get_data_value(SEI_Type *ptr, uint8_t idx)
{
    return ptr->DAT[idx].DATA;
}

static inline void sei_set_data_value(SEI_Type *ptr, uint8_t idx, uint32_t data)
{
    ptr->DAT[idx].DATA = data;
}

static inline void sei_set_data_rewind(SEI_Type *ptr, uint8_t idx)
{
    ptr->DAT[idx].DATA_MODE |= SEI_DAT_DATA_MODE_REWIND_MASK;
}

static inline void sei_set_sample_pos_override_value(SEI_Type *ptr, uint8_t idx, uint32_t data)
{
    ptr->CTRL[idx].POSITION.SMP_POS = data;
}

static inline void sei_set_sample_rev_override_value(SEI_Type *ptr, uint8_t idx, uint32_t data)
{
    ptr->CTRL[idx].POSITION.SMP_REV = data;
}

static inline void sei_set_sample_spd_override_value(SEI_Type *ptr, uint8_t idx, uint32_t data)
{
    ptr->CTRL[idx].POSITION.SMP_SPD = data;
}

static inline void sei_set_sample_acc_override_value(SEI_Type *ptr, uint8_t idx, uint32_t data)
{
    ptr->CTRL[idx].POSITION.SMP_ACC = data;
}

static inline void sei_set_irq_enable(SEI_Type *ptr, uint8_t idx, uint32_t irq_mask, bool enable)
{
    if (enable) {
        ptr->CTRL[idx].IRQ.INT_EN |= irq_mask;
    } else {
        ptr->CTRL[idx].IRQ.INT_EN &= ~irq_mask;
    }
}

static inline void sei_set_irq_pointer0(SEI_Type *ptr, uint8_t idx, uint8_t ptr_idx)
{
    ptr->CTRL[idx].IRQ.POINTER0 = ptr_idx;
}

static inline void sei_set_irq_pointer1(SEI_Type *ptr, uint8_t idx, uint8_t ptr_idx)
{
    ptr->CTRL[idx].IRQ.POINTER1 = ptr_idx;
}

static inline void sei_set_irq_instr0(SEI_Type *ptr, uint8_t idx, uint8_t instr_idx)
{
    ptr->CTRL[idx].IRQ.INSTR0 = instr_idx;
}

static inline void sei_set_irq_instr1(SEI_Type *ptr, uint8_t idx, uint8_t instr_idx)
{
    ptr->CTRL[idx].IRQ.INSTR1 = instr_idx;
}

static inline bool sei_get_irq_status(SEI_Type *ptr, uint8_t idx, uint32_t irq_mask)
{
    return ((ptr->CTRL[idx].IRQ.INT_FLAG & irq_mask) == irq_mask) ? true : false;
}

static inline void sei_clear_irq_flag(SEI_Type *ptr, uint8_t idx, uint32_t irq_mask)
{
    ptr->CTRL[idx].IRQ.INT_FLAG = irq_mask;
}

static inline void sei_set_instr(SEI_Type *ptr, uint8_t idx, uint8_t op, uint8_t ck, uint8_t crc, uint8_t data, uint8_t opr)
{
    uint32_t tmp;

    if ((op != SEI_INSTR_OP_HALT) && (op != SEI_INSTR_OP_JUMP) && (opr > 0)) {
        opr--;
    }
    tmp = SEI_INSTR_OP_SET(op)
        | SEI_INSTR_CK_SET(ck)
        | SEI_INSTR_CRC_SET(crc)
        | SEI_INSTR_DAT_SET(data)
        | SEI_INSTR_OPR_SET(opr);

    ptr->INSTR[idx] = tmp;
}

hpm_stat_t sei_tranceiver_config_init(SEI_Type *ptr, uint8_t idx, sei_tranceiver_config_t *config);
hpm_stat_t sei_cmd_data_format_config_init(SEI_Type *ptr, bool cmd_data_select, uint8_t idx, sei_data_format_config_t *config);
hpm_stat_t sei_cmd_table_config_init(SEI_Type *ptr, uint8_t idx, uint8_t table_idx, sei_command_table_config_t *config);
hpm_stat_t sei_state_transition_config_init(SEI_Type *ptr, uint8_t idx, uint8_t latch_idx, uint8_t state, sei_state_transition_config_t *config);
hpm_stat_t sei_state_transition_latch_config_init(SEI_Type *ptr, uint8_t idx, uint8_t latch_idx, sei_state_transition_latch_config_t *config);
hpm_stat_t sei_sample_config_init(SEI_Type *ptr, uint8_t idx, sei_sample_config_t *config);
hpm_stat_t sei_update_config_init(SEI_Type *ptr, uint8_t idx, sei_update_config_t *config);
hpm_stat_t sei_trigger_input_config_init(SEI_Type *ptr, uint8_t idx, sei_trigger_input_config_t *config);
hpm_stat_t sei_trigger_output_config_init(SEI_Type *ptr, uint8_t idx, sei_trigger_output_config_t *config);
hpm_stat_t sei_engine_config_init(SEI_Type *ptr, uint8_t idx, sei_engine_config_t *config);


#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif
