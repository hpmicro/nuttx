/*
 * Copyright (c) 2023 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef HPM_MMC_DRV_H
#define HPM_MMC_DRV_H

#include "hpm_common.h"
#include "hpm_mmc_regs.h"
/**
 * @brief MMC driver APIs
 * @defgroup mmc_interface MMC driver APIs
 * @ingroup io_interfaces
 * @{
 */

typedef enum {
    mmc_pos_init_by_timestamp = 0,
    mmc_pos_init_by_intrgr0_rise_edge = 1,
    mmc_pos_init_by_intrgr1_rise_edge = 2,
    mmc_pos_init_by_outtrgr0_rise_edge = 3,
    mmc_pos_init_by_outtrgr1_rise_edge = 4,
    mmc_pos_init_by_self_pos_trigger = 5, /* 跟踪器自己的位置触发信号触发 */
    mmc_pos_init_by_self_speed_trigger = 6, /* 跟踪器自己的速度触发信号触发 */
} mmc_pos_init_trigger_t;

typedef enum {
    mmc_delta_not_update = 0,
    mmc_delta_position_update = 1 << 0,   /* 位置增量 */
    mmc_delta_revolution_update = 1 << 1, /* 转速增量 */
    mmc_delta_speed_update = 1 << 2, /* 速度增量 */
    mmc_delta_accel_update = 1 << 3, /* 加速度增量 */
} mmc_delta_pos_init_trigger_t;

typedef enum {
    mmc_init_not_update = 0,
    mmc_init_position_update = 1 << 0,   /* 位置 */
    mmc_init_revolution_update = 1 << 1, /* 转速 */
    mmc_init_speed_update = 1 << 2, /* 速度 */
    mmc_init_accel_update = 1 << 3, /* 加速度 */
} mmc_pos_init_cmd_mask_t;

typedef enum {
    mmc_pos_type_32bit = 0,
    mmc_pos_type_16bit = 1,
} mmc_pos_type_t;

typedef enum {
    mmc_coef_not_update = 0,
    mmc_coef_p_update = 1 << 0,
    mmc_coef_i_update = 1 << 1,
    mmc_coef_a_update = 1 << 2,
} mmc_coef_init_cmd_mask_t;

typedef struct {
    bool force_accel_to_zero;
    bool en_ms_coef; /* 使能多种系数自动选择 */
    bool open_loop_mode;
    bool pos_16bit_type;             /* 不用 */
    bool en_shadow_read;
    bool sync_new_pos;               /* 采用新输入数据为新跟踪数据的起点 */
    /* bool discrete_pos_mode;  */         /* 离散位置输入模式 */
    uint8_t init_delta_pos_trigger;  /* 增量位置初始化信号 */
    uint8_t init_delta_pos_cmd_mask; /* 增量位置初始化命令 */
    bool reload_delta_pos_cmd;       /* 重新加载增量位置初始化命令， 命令修改之后要重新加载 */
    uint8_t init_pos_trigger;        /* 位置初始化信号 */
    uint8_t init_pos_cmd_mask;       /* 位置初始化命令 */
    bool reload_pos_cmd;             /* 重新加载位置初始化命令， 命令修改之后要重新加载 */
    uint8_t init_coef_cmd_mask;      /* 参数初始化命令 */
    bool reload_coef_cmd;            /* 重新加载参数初始化命令， 命令修改之后要重新加载 */

    uint32_t oosync_theta_thr;       /* 失步门限 */
} mmc_track_ctrl_t;

typedef struct {
    bool discrete_pos_mode;
    uint32_t discrete_line;
    uint32_t continuous_step_thr;
    uint32_t continuous_circ_thr;
} mmc_track_pos_mode_t;

typedef struct {
    uint32_t pos_time;
    uint32_t position;
    int32_t revolution;
    double speed;
    double accel;
} mmc_pos_input_t;

typedef struct {
    uint32_t coef_time;
    double coef_p;
    double coef_i;
    double coef_a;
} mmc_coef_input_t;

typedef struct {
    uint32_t err_thr; /* 误差门限 */
    uint32_t hold_time; /* 使用该系数时限制的保持时间 */
    double coef_p;
    double coef_i;
    double coef_a;
} mmc_coef_config_t;

typedef struct {
    uint32_t pos_time;
    uint32_t position;
    int32_t revolution;
    double speed;
    double accel;
} mmc_delta_input_t;

typedef struct {
    uint32_t time;
    uint32_t position;
    int32_t revolution;
    double speed;
    double accel;
} mmc_pos_out_t;

typedef struct {
    double coef_p;
    double coef_i;
    double coef_a;
} mmc_coef_out_t;

typedef enum {
    mmc_track_shadow_read_done = 1 << 0,
    mmc_track_init_coefs_done = 1 << 1,
    mmc_track_init_pos_cmd_done = 1 << 2,
    mmc_track_oosync = 1 << 4,
    mmc_track_idle = 1 << 5, /* 该事件没有中断 */
    mmc_pred1_pos_reload_cmd_done = 1 << 6,
    mmc_pred0_pos_reload_cmd_done = 1 << 7,
    mmc_track_delta_pos_reload_cmd_done = 1 << 8,
    mmc_track_pos_trig_valid = 1 << 9,
    mmc_track_speed_trig_valid = 1 << 10,
} mmc_track_event_t;

typedef enum {
    mmc_pred_idle = MMC_BR_BR_ST_IDLE_MASK,
    mmc_pred_init_delta_pos_done = MMC_BR_BR_ST_INI_DELTA_POS_DONE_MASK,
    mmc_pred_pos_trig_valid = MMC_BR_BR_ST_POS_TRG_VLD_MASK,
    mmc_pred_speed_trig_valid = MMC_BR_BR_ST_SPEED_TRG_VLD_MASK,
    mmc_pred_open_loop = MMC_BR_BR_ST_OPEN_LOOP_ST_MASK,
} mmc_pred_event_t;


typedef struct {
    bool speed_trig_int;
    bool position_trig_int;
    uint8_t init_pos_trigger;        /* 位置初始化信号 */
    uint8_t init_pos_cmd_mask;       /* 位置初始化命令 */
    uint8_t init_delta_pos_trigger;  /* 增量位置初始化信号 */
    bool delta_pos_done_trig_int;
    uint8_t init_delta_pos_cmd_mask; /* 增量位置初始化命令 */
    bool reload_delta_pos_cmd;       /* 重新加载增量位置初始化命令， 命令修改之后要重新加载 */

    bool open_loop_mode;
    uint8_t pred_mode;
    uint8_t not_first_pred_trig_type;
    uint8_t first_pred_trig_type;
} mmc_pred_ctrl_t;

typedef enum {
    mmc_pred_not_reload_pos_cmd = 0,
    mmc_pred_0_reload_pos_cmd = 2,
    mmc_pred_1_reload_pos_cmd = 1,
    mmc_pred_both_reload_pos_cmd = 3,
} mmc_pred_reload_pos_cmd_t;

typedef struct {
    uint32_t offset_time; /* 只能提前 是吗？ */
    uint32_t period_time;
    uint32_t first_time;
} mmc_pred_time_cfg_t;

typedef enum {
    mmc_pred_by_period = 0,
    mmc_pred_continuously_repeat = 1,
    mmc_pred_only_once = 2,
} mmc_pred_mode_t;

typedef struct {
    bool less_than; /* 真 - 小于检测， 假 - 大于检测 */
    bool enable; /* 使能该检查 */
    uint32_t position_thr;  /* 是否是有符号数 */
    int32_t revolution_thr; /* 圈数 */
} mmc_pos_trig_t;

typedef struct {
    bool absolute_compare; /* 真 - 绝对值比较  假 - 补码比较， 例如设置20， 小于-20 比较超限  */
    bool less_than; /* 真 - 小于触发， 假 - 大于触发 */
    bool enable;
    int32_t speed_thr;
} mmc_speed_trig_t;


#ifdef __cplusplus
extern "C" {
#endif

static inline void mmc_software_reset(MMC_Type *base)
{
    base->CR |= MMC_CR_SFTRST_MASK;
    /* 是否自清0 */
    base->CR &= ~MMC_CR_SFTRST_MASK;
}

static inline void mmc_track_reload_delta_pos_init_cmd(MMC_Type *base, uint8_t cmd_mask)
{
    base->CR &= ~(MMC_CR_INI_DELTA_POS_CMD_MSK_MASK | MMC_CR_INI_DELTA_POS_REQ_MASK);
    base->CR |= MMC_CR_INI_DELTA_POS_CMD_MSK_SET(cmd_mask);
    base->CR |= MMC_CR_INI_DELTA_POS_REQ_MASK;
}

static inline void mmc_track_reload_pos_init_cmd(MMC_Type *base, uint8_t cmd_mask)
{
    base->CR &= ~(MMC_CR_INI_POS_CMD_MSK_MASK | MMC_CR_INI_POS_REQ_MASK);
    base->CR |= MMC_CR_INI_POS_CMD_MSK_SET(cmd_mask);
    base->CR |= MMC_CR_INI_POS_REQ_MASK;
}

static inline void mmc_track_reload_coef_init_cmd(MMC_Type *base, uint8_t cmd_mask)
{
    base->CR &= ~(MMC_CR_INI_COEFS_CMD_MSK_MASK | MMC_CR_INI_COEFS_CMD_MASK);
    base->CR |= MMC_CR_INI_COEFS_CMD_MSK_SET(cmd_mask);
    base->CR |= MMC_CR_INI_COEFS_CMD_MASK;
}


static inline void mmc_track_set_open_loop_mode(MMC_Type *base, bool open_loop)
{
    if (open_loop) {
        base->CR |= MMC_CR_OPEN_LOOP_MODE_MASK;
    } else {
        base->CR &= ~MMC_CR_OPEN_LOOP_MODE_MASK;
    }
}

static inline void mmc_track_set_pos_type(MMC_Type *base, mmc_pos_type_t type)
{
    base->CR &= ~MMC_CR_POS_TYPE_MASK;
    base->CR |= MMC_CR_POS_TYPE_SET(type);
}

static inline void mmc_track_enable_shadow_read(MMC_Type *base)
{
    base->CR |= MMC_CR_SHADOW_RD_REQ_MASK;
}

static inline void mmc_track_enable_module(MMC_Type *base)
{
    base->CR |= MMC_CR_MOD_EN_MASK;
}

static inline void mmc_track_disable_module(MMC_Type *base)
{
    base->CR &= ~MMC_CR_MOD_EN_MASK;
}

static inline void mmc_config_interrupt(MMC_Type *base, uint32_t int_mask)
{
    base->INT_EN = int_mask;
}

static inline uint32_t mmc_get_status(MMC_Type *base)
{
    return base->STA;
}

static inline uint32_t mmc_get_status_error_id(MMC_Type *base)
{
    return MMC_STA_ERR_ID_GET(base->STA);
}

/* 是否对预测器起作用 */
static inline void mmc_set_oosync_theta_threshold(MMC_Type *base, uint32_t threshold)
{
    base->OOSYNC_THETA_THR = MMC_OOSYNC_THETA_THR_VAL_SET(threshold);
}

static inline void mmc_set_sysclk_freq(MMC_Type *base, uint32_t freq)
{
    uint32_t period;
    base->SYSCLK_FREQ = freq;
    /* 1/freq *(2^24)*(2^20) */
    period = (uint32_t)((double)(1 << 20) * (1 << 24) / freq);
    base->SYSCLK_PERIOD = period;
}

void mmc_track_config_pos_mode(MMC_Type *base, mmc_track_pos_mode_t *mode);

void mmc_get_default_ctrl_config(MMC_Type *base, mmc_track_ctrl_t *config);

void mmc_track_ctrl_config(MMC_Type *base, mmc_track_ctrl_t *config);

void mmc_track_init_pos_para(MMC_Type *base, mmc_pos_input_t *para);

void mmc_track_init_coef_para(MMC_Type *base, mmc_coef_input_t *para);

void mmc_track_init_delta_para(MMC_Type *base, mmc_delta_input_t *para);

void mmc_track_get_result(MMC_Type *base, mmc_pos_out_t *pos_out, mmc_coef_out_t *coef_out);

void mmc_track_position_trig_config(MMC_Type *base, mmc_pos_trig_t *trig);

void mmc_track_speed_trig_config(MMC_Type *base, mmc_speed_trig_t *trig);

void mmc_track_coef_trig_config(MMC_Type *base, uint8_t index, mmc_coef_config_t *config);

/* 预测器 */
static inline void mmc_enable_pred(MMC_Type *base, uint8_t index)
{
    base->BR[index].BR_CTRL |= MMC_BR_BR_CTRL_BR_EN_MASK;
}

static inline void mmc_disable_pred(MMC_Type *base, uint8_t index)
{
    base->BR[index].BR_CTRL &= ~MMC_BR_BR_CTRL_BR_EN_MASK;
}

/* 预测器重载位置初始化命令 仅开环模式使用 */
/* cmd 0/1/2/3 */
static inline void mmc_pred_reload_pos_cmd(MMC_Type *base, mmc_pred_reload_pos_cmd_t cmd)
{
    base->CR &= ~(MMC_CR_INI_BR0_POS_REQ_MASK | MMC_CR_INI_BR0_POS_REQ_MASK);
    base->CR |= cmd << MMC_CR_INI_BR1_POS_REQ_SHIFT;
}

void mmc_pred_get_default_ctrl_config(MMC_Type *base, mmc_pred_ctrl_t *config);

void mmc_pred_ctrl_config(MMC_Type *base, uint8_t index, mmc_pred_ctrl_t *config);

void mmc_pred_init_pos_para(MMC_Type *base, uint8_t index, mmc_pos_input_t *para);

void mmc_pred_init_delta_para(MMC_Type *base, uint8_t index, mmc_delta_input_t *para);

void mmc_pred_get_result(MMC_Type *base, uint8_t index, mmc_pos_out_t *pos_out);

void mmc_pred_time_config(MMC_Type *base, uint8_t index, mmc_pred_time_cfg_t *time);

void mmc_pred_position_trig_config(MMC_Type *base, uint8_t index, mmc_pos_trig_t *trig);

void mmc_pred_speed_trig_config(MMC_Type *base, uint8_t index, mmc_speed_trig_t *trig);


#ifdef __cplusplus
}
#endif
/**
 * @}
 */
#endif /* HPM_MMC_DRV_H */
