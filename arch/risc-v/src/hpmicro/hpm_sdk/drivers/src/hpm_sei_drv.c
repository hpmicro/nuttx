/*
 * Copyright (c) 2023 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "hpm_sei_drv.h"

#define SEI_CLK_SRC_FREQ 25000000U

hpm_stat_t sei_tranceiver_config_init(SEI_Type *ptr, uint8_t idx, sei_tranceiver_config_t *config)
{
    uint32_t tmp;
    uint32_t baud_div;
    uint32_t sync_point;
    uint8_t data_len;
    uint32_t ck0_point;
    uint32_t ck1_point;
    uint32_t txd_point;
    uint32_t rxd_point;

    tmp = SEI_CTRL_XCVR_TRX_CTRL_TRISMP_SET(config->tri_sample)
        | SEI_CTRL_XCVR_TRX_CTRL_MODE_SET(config->mode);
    ptr->CTRL[idx].XCVR.TRX_CTRL = tmp;

    switch (config->mode) {
    case sei_synchronous_master_mode:
        tmp = SEI_CTRL_XCVR_TRX_TYPE_CFG_DA_IDLEZ_SET(config->synchronous_master_config.data_idle_high_z)
            | SEI_CTRL_XCVR_TRX_TYPE_CFG_DA_IDLEV_SET(config->synchronous_master_config.data_idle_state)
            | SEI_CTRL_XCVR_TRX_TYPE_CFG_CK_IDLEZ_SET(config->synchronous_master_config.clock_idle_high_z)
            | SEI_CTRL_XCVR_TRX_TYPE_CFG_CK_IDLEV_SET(config->synchronous_master_config.clock_idle_state);
        ptr->CTRL[idx].XCVR.TRX_TYPE_CFG = tmp;

        baud_div = (SEI_CLK_SRC_FREQ + (config->synchronous_master_config.baudrate >> 1u)) / config->synchronous_master_config.baudrate;
        sync_point = baud_div >> 1u;
        tmp = SEI_CTRL_XCVR_TRX_BAUD_CFG_SYNC_POINT_SET(sync_point)
            | SEI_CTRL_XCVR_TRX_BAUD_CFG_BAUD_DIV_SET(baud_div - 1u);
        ptr->CTRL[idx].XCVR.TRX_BAUD_CFG = tmp;

        ck0_point = baud_div >> 2u;
        ck1_point = ck0_point * 3u;
        tmp = SEI_CTRL_XCVR_TRX_CLK_CFG_CK0_POINT_SET(ck0_point)
            | SEI_CTRL_XCVR_TRX_CLK_CFG_CK1_POINT_SET(ck1_point);
        ptr->CTRL[idx].XCVR.TRX_CLK_CFG = tmp;
        break;

    case sei_synchronous_slave_mode:
        tmp = SEI_CTRL_XCVR_TRX_TYPE_CFG_DA_IDLEZ_SET(config->synchronous_slave_config.data_idle_high_z)
            | SEI_CTRL_XCVR_TRX_TYPE_CFG_DA_IDLEV_SET(config->synchronous_slave_config.data_idle_state)
            | SEI_CTRL_XCVR_TRX_TYPE_CFG_CK_IDLEZ_SET(config->synchronous_slave_config.clock_idle_high_z)
            | SEI_CTRL_XCVR_TRX_TYPE_CFG_CK_IDLEV_SET(config->synchronous_slave_config.clock_idle_state);
        ptr->CTRL[idx].XCVR.TRX_TYPE_CFG = tmp;

        baud_div = (SEI_CLK_SRC_FREQ + (config->synchronous_slave_config.max_baudrate >> 1u)) / config->synchronous_slave_config.max_baudrate;
        sync_point = (baud_div * 3u) >> 3u;
        tmp = SEI_CTRL_XCVR_TRX_BAUD_CFG_SYNC_POINT_SET(sync_point)
            | SEI_CTRL_XCVR_TRX_BAUD_CFG_BAUD_DIV_SET(baud_div - 1u);
        ptr->CTRL[idx].XCVR.TRX_BAUD_CFG = tmp;

        ck0_point = config->synchronous_slave_config.ck0_timeout_us * (SEI_CLK_SRC_FREQ / 1000000u);
        ck1_point = config->synchronous_slave_config.ck1_timeout_us * (SEI_CLK_SRC_FREQ / 1000000u) + 0x8000u;
        tmp = SEI_CTRL_XCVR_TRX_CLK_CFG_CK0_POINT_SET(ck0_point)
            | SEI_CTRL_XCVR_TRX_CLK_CFG_CK1_POINT_SET(ck1_point);
        ptr->CTRL[idx].XCVR.TRX_CLK_CFG = tmp;
    break;

    case sei_asynchronous_mode:
    default:
        data_len = config->asynchronous_config.data_len;
        if (data_len > 0u) {
            data_len--;
        }
        tmp = SEI_CTRL_XCVR_TRX_TYPE_CFG_WAIT_LEN_SET(config->asynchronous_config.wait_len)
            | SEI_CTRL_XCVR_TRX_TYPE_CFG_DATA_LEN_SET(data_len)
            | SEI_CTRL_XCVR_TRX_TYPE_CFG_PAR_POL_SET(config->asynchronous_config.parity)
            | SEI_CTRL_XCVR_TRX_TYPE_CFG_PAR_EN_SET(config->asynchronous_config.parity_enable)
            | SEI_CTRL_XCVR_TRX_TYPE_CFG_DA_IDLEZ_SET(config->asynchronous_config.data_idle_high_z)
            | SEI_CTRL_XCVR_TRX_TYPE_CFG_DA_IDLEV_SET(config->asynchronous_config.data_idle_state);
        ptr->CTRL[idx].XCVR.TRX_TYPE_CFG = tmp;

        baud_div = (SEI_CLK_SRC_FREQ + (config->asynchronous_config.baudrate >> 1u)) / config->asynchronous_config.baudrate;
        sync_point = baud_div >> 1u;
        tmp = SEI_CTRL_XCVR_TRX_BAUD_CFG_SYNC_POINT_SET(sync_point)
            | SEI_CTRL_XCVR_TRX_BAUD_CFG_BAUD_DIV_SET(baud_div - 1u);
        ptr->CTRL[idx].XCVR.TRX_BAUD_CFG = tmp;

        txd_point = 0;
        rxd_point = baud_div >> 1u;
        tmp = SEI_CTRL_XCVR_TRX_DATA_CFG_TXD_POINT_SET(txd_point)
            | SEI_CTRL_XCVR_TRX_DATA_CFG_RXD_POINT_SET(rxd_point);
        ptr->CTRL[idx].XCVR.TRX_DATA_CFG = tmp;
        break;
    }

    return status_success;
}

hpm_stat_t sei_cmd_data_format_config_init(SEI_Type *ptr, bool cmd_data_select, uint8_t idx, sei_data_format_config_t *config)
{
    uint32_t tmp;
    uint8_t word_len;
    uint8_t crc_len;

    word_len = config->word_len;
    if (word_len > 0u) {
        word_len--;
    }
    crc_len = config->crc_len;
    if (crc_len > 0u) {
        crc_len--;
    }
    tmp = SEI_DAT_DATA_MODE_MODE_SET(config->mode)
        | SEI_DAT_DATA_MODE_SIGNED_SET(config->signed_flag)
        | SEI_DAT_DATA_MODE_BORDER_SET(config->bit_order)
        | SEI_DAT_DATA_MODE_WORDER_SET(config->word_order)
        | SEI_DAT_DATA_MODE_CRC_INV_SET(config->crc_invert)
        | SEI_DAT_DATA_MODE_CRC_SHIFT_SET(config->crc_shift_mode)
        | SEI_DAT_DATA_MODE_WLEN_SET(word_len)
        | SEI_DAT_DATA_MODE_CRC_LEN_SET(crc_len);
    if (cmd_data_select) {
        ptr->CTRL[idx].COMMAND.CMD_MODE = tmp;
    } else {
        ptr->DAT[idx].DATA_MODE = tmp;
    }

    tmp = SEI_DAT_DATA_IDX_LAST_BIT_SET(config->last_bit_index)
        | SEI_DAT_DATA_IDX_FIRST_BIT_SET(config->first_bit_index)
        | SEI_DAT_DATA_IDX_MAX_BIT_SET(config->highest_bit_index)
        | SEI_DAT_DATA_IDX_MIN_BIT_SET(config->lowest_bit_index);
    if (cmd_data_select) {
        ptr->CTRL[idx].COMMAND.CMD_IDX = tmp;
    } else {
        ptr->DAT[idx].DATA_IDX = tmp;
    }

    tmp = SEI_DAT_DATA_GOLD_GOLD_VALUE_SET(config->gold_value);
    if (cmd_data_select) {
        ptr->CTRL[idx].COMMAND.CMD_GOLD = tmp;
    } else {
        ptr->DAT[idx].DATA_GOLD = tmp;
    }

    tmp = SEI_DAT_DATA_CRCINIT_CRC_INIT_SET(config->crc_init_value);
    if (cmd_data_select) {
        ptr->CTRL[idx].COMMAND.CMD_CRCINIT = tmp;
    } else {
        ptr->DAT[idx].DATA_CRCINIT = tmp;
    }

    tmp = SEI_DAT_DATA_CRCPOLY_CRC_POLY_SET(config->crc_poly);
    if (cmd_data_select) {
        ptr->CTRL[idx].COMMAND.CMD_CRCPOLY = tmp;
    } else {
        ptr->DAT[idx].DATA_CRCPOLY = tmp;
    }

    if (cmd_data_select) {
        ptr->CTRL[idx].COMMAND.CMD_MODE |= SEI_CTRL_COMMAND_CMD_MODE_REWIND_MASK;
    } else {
        ptr->DAT[idx].DATA_MODE |= SEI_DAT_DATA_MODE_REWIND_MASK;
    }

    return status_success;
}

hpm_stat_t sei_cmd_table_config_init(SEI_Type *ptr, uint8_t idx, uint8_t table_idx, sei_command_table_config_t *config)
{
    uint32_t tmp;

    tmp = SEI_CTRL_COMMAND_TABLE_CMD_MIN_CMD_MIN_SET(config->cmd_min_value);
    ptr->CTRL[idx].COMMAND_TABLE[table_idx].CMD_MIN = tmp;

    tmp = SEI_CTRL_COMMAND_TABLE_CMD_MAX_CMD_MAX_SET(config->cmd_max_value);
    ptr->CTRL[idx].COMMAND_TABLE[table_idx].CMD_MAX = tmp;

    tmp = SEI_CTRL_COMMAND_TABLE_CMD_MSK_CMD_MASK_SET(config->cmd_mask_value);
    ptr->CTRL[idx].COMMAND_TABLE[table_idx].CMD_MSK = tmp;

    tmp = SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR3_SET(config->instr_idx[3])
        | SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR2_SET(config->instr_idx[2])
        | SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR1_SET(config->instr_idx[1])
        | SEI_CTRL_COMMAND_TABLE_CMD_PTRA_PTR0_SET(config->instr_idx[0]);
    ptr->CTRL[idx].COMMAND_TABLE[table_idx].CMD_PTRA = tmp;

    tmp = SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR7_SET(config->instr_idx[7])
        | SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR6_SET(config->instr_idx[6])
        | SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR5_SET(config->instr_idx[5])
        | SEI_CTRL_COMMAND_TABLE_CMD_PTRB_PTR4_SET(config->instr_idx[4]);
    ptr->CTRL[idx].COMMAND_TABLE[table_idx].CMD_PTRB = tmp;

    tmp = SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR11_SET(config->instr_idx[11])
        | SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR10_SET(config->instr_idx[10])
        | SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR9_SET(config->instr_idx[9])
        | SEI_CTRL_COMMAND_TABLE_CMD_PTRC_PTR8_SET(config->instr_idx[8]);
    ptr->CTRL[idx].COMMAND_TABLE[table_idx].CMD_PTRC = tmp;

    tmp = SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR15_SET(config->instr_idx[15])
        | SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR14_SET(config->instr_idx[14])
        | SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR13_SET(config->instr_idx[13])
        | SEI_CTRL_COMMAND_TABLE_CMD_PTRD_PTR12_SET(config->instr_idx[12]);
    ptr->CTRL[idx].COMMAND_TABLE[table_idx].CMD_PTRD = tmp;

    return status_success;
}

hpm_stat_t sei_state_transition_config_init(SEI_Type *ptr, uint8_t idx, uint8_t latch_idx, uint8_t state, sei_state_transition_config_t *config)
{
    uint32_t tmp;

    tmp = SEI_CTRL_LATCH_TRAN_CFG_POINTER_SET(config->instr_ptr_value)
        | SEI_CTRL_LATCH_TRAN_CFG_STATE_SET(config->enabled_state)
        | SEI_CTRL_LATCH_TRAN_CFG_CFG_TM_SET(config->timeout_cfg)
        | SEI_CTRL_LATCH_TRAN_CFG_CFG_RXD_SET(config->rxd_cfg)
        | SEI_CTRL_LATCH_TRAN_CFG_CFG_TXD_SET(config->txd_cfg)
        | SEI_CTRL_LATCH_TRAN_CFG_CFG_CLK_SET(config->clk_cfg)
        | SEI_CTRL_LATCH_TRAN_CFG_CFG_PTR_SET(config->instr_ptr_cfg)
        | SEI_CTRL_LATCH_TRAN_CFG_OV_TM_SET(config->override_timeout_check)
        | SEI_CTRL_LATCH_TRAN_CFG_OV_RXD_SET(config->override_rxd_check)
        | SEI_CTRL_LATCH_TRAN_CFG_OV_TXD_SET(config->override_txd_check)
        | SEI_CTRL_LATCH_TRAN_CFG_OV_CLK_SET(config->override_clk_check)
        | SEI_CTRL_LATCH_TRAN_CFG_OV_PTR_SET(config->override_instr_ptr_check);
    ptr->CTRL[idx].LATCH[latch_idx].TRAN_CFG[state] = tmp;

    return status_success;
}

hpm_stat_t sei_state_transition_latch_config_init(SEI_Type *ptr, uint8_t idx, uint8_t latch_idx, sei_state_transition_latch_config_t *config)
{
    uint32_t tmp;

    tmp = SEI_CTRL_LATCH_LAT_CFG_DELAY_SET(config->delay)
        | SEI_CTRL_LATCH_LAT_CFG_SELECT_SET(config->output_select)
        | SEI_CTRL_LATCH_LAT_CFG_EN_SET(config->enable);
    ptr->CTRL[idx].LATCH[latch_idx].LAT_CFG = tmp;

    return status_success;
}

hpm_stat_t sei_sample_config_init(SEI_Type *ptr, uint8_t idx, sei_sample_config_t *config)
{
    uint32_t tmp;

    tmp = SEI_CTRL_POSITION_SMP_CFG_ONCE_SET(config->sample_once)
        | SEI_CTRL_POSITION_SMP_CFG_LAT_SEL_SET(config->latch_select)
        | SEI_CTRL_POSITION_SMP_CFG_WINDOW_SET(config->sample_window);
    ptr->CTRL[idx].POSITION.SMP_CFG = tmp;

    ptr->CTRL[idx].POSITION.SMP_DAT = SEI_CTRL_POSITION_SMP_DAT_DAT_SEL_SET(config->data_register_select);

    tmp = SEI_CTRL_POSITION_SMP_EN_ACC_EN_SET(config->enable_acc)
        | SEI_CTRL_POSITION_SMP_EN_ACC_SEL_SET(config->acc_data_idx)
        | SEI_CTRL_POSITION_SMP_EN_SPD_EN_SET(config->enable_spd)
        | SEI_CTRL_POSITION_SMP_EN_SPD_SEL_SET(config->spd_data_idx)
        | SEI_CTRL_POSITION_SMP_EN_REV_EN_SET(config->enable_rev)
        | SEI_CTRL_POSITION_SMP_EN_REV_SEL_SET(config->rev_data_idx)
        | SEI_CTRL_POSITION_SMP_EN_POS_EN_SET(config->enable_pos)
        | SEI_CTRL_POSITION_SMP_EN_POS_SEL_SET(config->pos_data_idx);
    ptr->CTRL[idx].POSITION.SMP_EN = tmp;

    return status_success;
}

hpm_stat_t sei_update_config_init(SEI_Type *ptr, uint8_t idx, sei_update_config_t *config)
{
    uint32_t tmp;

    tmp = SEI_CTRL_POSITION_UPD_CFG_OVRD_SET(config->use_override_value)
        | SEI_CTRL_POSITION_UPD_CFG_ONERR_SET(config->update_on_err)
        | SEI_CTRL_POSITION_UPD_CFG_LAT_SEL_SET(config->latch_select);
    ptr->CTRL[idx].POSITION.UPD_CFG = tmp;

    ptr->CTRL[idx].POSITION.UPD_DAT = SEI_CTRL_POSITION_UPD_DAT_DAT_SEL_SET(config->data_register_select);

    ptr->CTRL[idx].POSITION.UPD_TIME = SEI_CTRL_POSITION_UPD_TIME_TIME_SET(config->override_time_value);

    ptr->CTRL[idx].POSITION.UPD_POS = SEI_CTRL_POSITION_UPD_TIME_TIME_SET(config->override_pos_value);

    ptr->CTRL[idx].POSITION.UPD_REV = SEI_CTRL_POSITION_UPD_TIME_TIME_SET(config->override_rev_value);

    ptr->CTRL[idx].POSITION.UPD_SPD = SEI_CTRL_POSITION_UPD_TIME_TIME_SET(config->override_spd_value);

    ptr->CTRL[idx].POSITION.UPD_ACC = SEI_CTRL_POSITION_UPD_TIME_TIME_SET(config->override_acc_value);

    tmp = SEI_CTRL_POSITION_UPD_EN_ACC_EN_SET(config->enable_acc)
        | SEI_CTRL_POSITION_UPD_EN_ACC_SEL_SET(config->acc_data_idx)
        | SEI_CTRL_POSITION_UPD_EN_SPD_EN_SET(config->enable_spd)
        | SEI_CTRL_POSITION_UPD_EN_SPD_SEL_SET(config->spd_data_idx)
        | SEI_CTRL_POSITION_UPD_EN_REV_EN_SET(config->enable_rev)
        | SEI_CTRL_POSITION_UPD_EN_REV_SEL_SET(config->rev_data_idx)
        | SEI_CTRL_POSITION_UPD_EN_POS_EN_SET(config->enable_pos)
        | SEI_CTRL_POSITION_UPD_EN_POS_SEL_SET(config->pos_data_idx);
    ptr->CTRL[idx].POSITION.UPD_EN = tmp;

    return status_success;
}

hpm_stat_t sei_trigger_input_config_init(SEI_Type *ptr, uint8_t idx, sei_trigger_input_config_t *config)
{
    uint32_t tmp;
    uint32_t period;

    tmp = SEI_CTRL_TRIGER_TRG_PRD_CFG_ARMING_SET(config->trig_period_arming_mode)
        | SEI_CTRL_TRIGER_TRG_PRD_CFG_SYNC_SET(config->trig_period_sync_enable);
    ptr->CTRL[idx].TRIGER.TRG_PRD_CFG = tmp;

    period = config->trig_period_time;
    if (period > 0) {
        period--;
    }
    ptr->CTRL[idx].TRIGER.TRG_PRD = SEI_CTRL_TRIGER_TRG_PRD_PERIOD_SET(period);

    tmp = SEI_CTRL_TRIGER_TRG_IN_CFG_PRD_EN_SET(config->trig_period_enable)
        | SEI_CTRL_TRIGER_TRG_IN_CFG_SYNC_SEL_SET(config->trig_period_sync_select)
        | SEI_CTRL_TRIGER_TRG_IN_CFG_IN1_EN_SET(config->trig1_enable)
        | SEI_CTRL_TRIGER_TRG_IN_CFG_IN1_SEL_SET(config->trig1_select)
        | SEI_CTRL_TRIGER_TRG_IN_CFG_IN0_EN_SET(config->trig0_enable)
        | SEI_CTRL_TRIGER_TRG_IN_CFG_IN0_SEL_SET(config->trig0_select);
    ptr->CTRL[idx].TRIGER.TRG_IN_CFG = tmp;

    return status_success;
}

hpm_stat_t sei_trigger_output_config_init(SEI_Type *ptr, uint8_t idx, sei_trigger_output_config_t *config)
{
    uint32_t tmp;

    tmp = SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT3_EN_SET(config->trig3_enable)
        | SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT3_SEL_SET(config->trig3_select)
        | SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT2_EN_SET(config->trig2_enable)
        | SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT2_SEL_SET(config->trig2_select)
        | SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT1_EN_SET(config->trig1_enable)
        | SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT1_SEL_SET(config->trig1_select)
        | SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT0_EN_SET(config->trig0_enable)
        | SEI_CTRL_TRIGER_TRG_OUT_CFG_OUT0_SEL_SET(config->trig0_select);
    ptr->CTRL[idx].TRIGER.TRG_OUT_CFG = tmp;

    return status_success;
}

hpm_stat_t sei_engine_config_init(SEI_Type *ptr, uint8_t idx, sei_engine_config_t *config)
{
    uint32_t tmp;

    tmp = SEI_CTRL_ENGINE_ENG_CTRL_WATCH_SET(config->wdg_enable)
        | SEI_CTRL_ENGINE_ENG_CTRL_EXCEPT_SET(config->wdg_action)
        | SEI_CTRL_ENGINE_ENG_CTRL_ARMING_SET(config->arming_mode);
    ptr->CTRL[idx].ENGINE.ENG_CTRL = tmp;

    tmp = SEI_CTRL_ENGINE_ENG_CFG_DAT_CDM_SET(config->data_cdm_idx)
        | SEI_CTRL_ENGINE_ENG_CFG_DAT_BASE_SET(config->data_base_idx)
        | SEI_CTRL_ENGINE_ENG_CFG_POINTER_WDOG_SET(config->wdg_instr_ptr_idx)
        | SEI_CTRL_ENGINE_ENG_CFG_POINTER_INIT_SET(config->init_instr_ptr_idx);
    ptr->CTRL[idx].ENGINE.ENG_CFG = tmp;

    ptr->CTRL[idx].ENGINE.ENG_WDG = SEI_CTRL_ENGINE_ENG_WDG_WDOG_TIME_SET(config->wdg_time);

    return status_success;
}
