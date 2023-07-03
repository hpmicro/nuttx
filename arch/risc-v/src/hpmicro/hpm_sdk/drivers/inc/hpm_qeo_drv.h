/*
 * Copyright (c) 2023 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef HPM_QEO_DRV_H
#define HPM_QEO_DRV_H

#include "hpm_common.h"
#include "hpm_qeo_regs.h"
/**
 * @brief QEO driver APIs
 * @defgroup qeo_interface QEO driver APIs
 * @ingroup io_interfaces
 * @{
 */

typedef enum {
    qeo_wave_cosine = 0,
    qeo_wave_saddle = 1, /*< 马鞍波 */
    qeo_wave_abs_cosine = 2,
    qeo_wave_saw = 3, /*< 锯齿波 */
} qeo_wave_type_t;

typedef enum {
    qeo_wave_above_max_limit_max_val = 0,
    qeo_wave_above_max_limit_zero = 1,
    qeo_wave_above_max_limit_max_level0_val = 2,

    qeo_wave_high_area_limit_max_val = 0,
    qeo_wave_high_area_limit_max_level0_val = 1,

    qeo_wave_low_area_limit_zero = 0,
    qeo_wave_low_area_limit_min_level1_val = 1,

    qeo_wave_below_min_limit_zero = 0,
    qeo_wave_below_min_limit_max_val = 1,
    qeo_wave_below_min_limit_min_level1_val = 2,
} qeo_wave_limit_t;

typedef struct {
    uint8_t above_max_limit;
    uint8_t high_area0_limit;
    uint8_t high_area1_limit;
    uint8_t low_area0_limit;
    uint8_t low_area1_limit;
    uint8_t below_min_limit;
    bool en_vd_vq_inject;
} qeo_wave_limit_config_t;

typedef struct {
    qeo_wave_limit_config_t wave0;
    qeo_wave_limit_config_t wave1;
    qeo_wave_limit_config_t wave2;
    uint8_t wave_type;
    uint8_t saddle_type;
} qeo_wave_mode_t;

typedef enum {
    qeo_z_as_zero_point_25_percent = 0,
    qeo_z_as_zero_point_75_percent = 1,
    qeo_z_as_zero_point_100_percent = 2,
    qeo_z_as_wave_in_third_phase = 3,

    qeo_b_as_wave_in_two_phase = 0,    /*< 2相正交b */
    qeo_b_as_direction_pulse = 1,      /*< 方向脉冲 */
    qeo_b_as_reverse_pulse = 2,        /*< 反向脉冲 */
    qeo_b_as_wave_in_three_phase = 3,  /*< 3相正交b */

    qeo_a_as_wave_in_two_phase = 0,    /*< 2相正交a */
    qeo_a_as_speed_pulse = 1,          /*< 速度脉冲 */
    qeo_a_as_forward_pulse = 2,        /*< 正向脉冲 */
    qeo_a_as_wave_in_three_phase = 3,  /*< 3相正交a */
} qeo_abz_type_t;

typedef struct {
    bool en_wdog;
    bool z_inv_pol;
    bool b_inv_pol;
    bool a_inv_pol;
    uint8_t z_type;
    uint8_t b_type;
    uint8_t a_type;
} qeo_abz_mode_t;


/* 与PWM相关，需要放到一个公共的文件中 */
typedef enum {
    qeo_pwm_output_force_0 = 2,
    qeo_pwm_output_force_1 = 3,
    qeo_pwm_output_no_force = 0,  /*< PWM mode */
} qeo_pwm_output_force_t;

typedef struct {
    uint8_t pwm0_output; /*< PWMA */
    uint8_t pwm1_output; /*< PWMA */
    uint8_t pwm2_output; /*< PWMB */
    uint8_t pwm3_output; /*< PWMB */
    uint8_t pwm4_output; /*< PWMB */
    uint8_t pwm5_output; /*< PWMB */
    uint8_t pwm6_output; /*< PWMB */
    uint8_t pwm7_output; /*< PWMB */
} qeo_pwm_force_output_table_t;

typedef struct {
    uint8_t pwm0_output; /*< PWMA */
    uint8_t pwm1_output; /*< PWMA */
    uint8_t pwm2_output; /*< PWMB */
    uint8_t pwm3_output; /*< PWMB */
    uint8_t pwm4_output; /*< PWMB */
    uint8_t pwm5_output; /*< PWMB */
    uint8_t pwm6_output; /*< PWMB */
    uint8_t pwm7_output; /*< PWMB */
} qeo_pwm_fault_output_table_t;

typedef struct {
    qeo_pwm_fault_output_table_t fault;
    uint8_t phase_num;
    bool fault_shield_trigger;
    bool revise_pairs_output;
} qeo_pwm_mode_t;



#ifdef __cplusplus
extern "C" {
#endif

/* WAVE API */
static inline void qeo_wave_set_resolution_sincos(QEO_Type *base, uint32_t sincos)
{
    base->WAVE.RESOLUTION = QEO_WAVE_RESOLUTION_SINCOS_SET(sincos);
}

static inline void qeo_wave_set_phase_shift(QEO_Type *base, uint8_t index, uint32_t shift)
{
    base->WAVE.PHASE_SHIFT[index] = QEO_WAVE_PHASE_SHIFT_VAL_SET(shift);
}

static inline void qeo_wave_set_vd_vq(QEO_Type *base, uint8_t index, int32_t vd_val, int32_t vq_val)
{
    base->WAVE.VD_VQ_INJECT[index] = QEO_WAVE_VD_VQ_INJECT_VQ_VAL_SET(vq_val) | QEO_WAVE_VD_VQ_INJECT_VD_VAL_SET(vq_val);
}

static inline void qeo_wave_load_vd_vq(QEO_Type *base)
{
    base->WAVE.VD_VQ_LOAD = QEO_WAVE_VD_VQ_LOAD_LOAD_MASK;
}

static inline void qeo_wave_enable_amplitude(QEO_Type *base, uint8_t index, uint32_t amp)
{
    base->WAVE.AMPLITUDE[index] = QEO_WAVE_AMPLITUDE_EN_SCAL_MASK | QEO_WAVE_AMPLITUDE_AMP_VAL_SET(amp);
}

static inline void qeo_wave_disable_amplitude(QEO_Type *base, uint8_t index)
{
    base->WAVE.AMPLITUDE[index] &= ~QEO_WAVE_AMPLITUDE_EN_SCAL_MASK;
}

static inline void qeo_wave_set_mid_point(QEO_Type *base, uint8_t index, uint32_t point)
{
    base->WAVE.MID_POINT[index] = QEO_WAVE_MID_POINT_VAL_SET(point);
}

static inline void qeo_wave_set_max_limit(QEO_Type *base, uint8_t index, uint32_t limit0, uint32_t limit1)
{
    base->WAVE.LIMIT[index].MAX = QEO_WAVE_LIMIT_MAX_LIMIT0_SET(limit0) | QEO_WAVE_LIMIT_MAX_LIMIT1_SET(limit1);
}

static inline void qeo_wave_set_min_limit(QEO_Type *base, uint8_t index, uint32_t limit0, uint32_t limit1)
{
    base->WAVE.LIMIT[index].MIN = QEO_WAVE_LIMIT_MIN_LIMIT0_SET(limit0) | QEO_WAVE_LIMIT_MIN_LIMIT1_SET(limit1);
}

static inline void qeo_wave_set_deadzone_shift(QEO_Type *base, uint8_t index, int16_t shift)
{
    base->WAVE.DEADZONE_SHIFT[index] = QEO_WAVE_DEADZONE_SHIFT_VAL_SET(shift);
}

/* ABZ API */
static inline void qeo_abz_set_resolution_sincos(QEO_Type *base, uint32_t sincos)
{
    base->ABZ.RESOLUTION = QEO_ABZ_RESOLUTION_SINCOS_SET(sincos);
}

static inline void qeo_abz_set_phase_shift(QEO_Type *base, uint8_t index, uint32_t shift)
{
    base->ABZ.PHASE_SHIFT[index] = QEO_ABZ_PHASE_SHIFT_VAL_SET(shift);
}

static inline void qeo_abz_set_line_width(QEO_Type *base, uint32_t width)
{
    base->ABZ.LINE_WIDTH = QEO_ABZ_LINE_WIDTH_LINE_SET(width);
}

static inline void qeo_abz_set_wdog_width(QEO_Type *base, uint32_t width)
{
    base->ABZ.WDOG_WIDTH = QEO_ABZ_WDOG_WIDTH_WIDTH_SET(width);
}

static inline void qeo_abz_enable_postion_sync(QEO_Type *base)
{
    base->ABZ.POSTION_SYNC = QEO_ABZ_POSTION_SYNC_POSTION_MASK;
}

static inline void qeo_abz_disable_postion_sync(QEO_Type *base)
{
    base->ABZ.POSTION_SYNC &= ~QEO_ABZ_POSTION_SYNC_POSTION_MASK;
}

/* PWM API */
static inline void qeo_pwm_set_resolution_sincos(QEO_Type *base, uint32_t sincos)
{
    base->PWM.RESOLUTION = QEO_PWM_RESOLUTION_SINCOS_SET(sincos);
}

static inline void qeo_pwm_set_phase_shift(QEO_Type *base, uint8_t index, uint32_t shift)
{
    base->PWM.PHASE_SHIFT[index] = QEO_PWM_PHASE_SHIFT_VAL_SET(shift);
}

static inline bool qeo_pwm_in_fault(QEO_Type *base)
{
    return ((base->STATUS & QEO_STATUS_PWM_FAULT_MASK) == QEO_STATUS_PWM_FAULT_MASK) ? true : false;
}

static inline void qeo_pwm_use_posedge_phase_table(QEO_Type *base)
{
    base->PWM.MODE &= ~QEO_PWM_MODE_REVISE_UP_DN_MASK;
}

static inline void qeo_pwm_use_negedge_phase_table(QEO_Type *base)
{
    base->PWM.MODE |= QEO_PWM_MODE_REVISE_UP_DN_MASK;
}


void qeo_wave_get_default_mode_config(QEO_Type *base, qeo_wave_mode_t *config);
void qeo_wave_mode_config(QEO_Type *base, qeo_wave_mode_t *config);

void qeo_abz_get_default_mode_config(QEO_Type *base, qeo_abz_mode_t *config);
void qeo_abz_mode_config(QEO_Type *base, qeo_abz_mode_t *config);

void qeo_pwm_get_default_mode_config(QEO_Type *base, qeo_pwm_mode_t *config);
void qeo_pwm_mode_config(QEO_Type *base, qeo_pwm_mode_t *config);
void qeo_pwm_set_phase_table(QEO_Type *base, uint8_t index, qeo_pwm_force_output_table_t *table);

static inline void qeo_software_position_inject(QEO_Type *base, uint32_t position)
{
    base->POSTION_SEL = QEO_POSTION_SEL_POSTION_SEL_MASK;
    base->POSTION_SOFTWARE = QEO_POSTION_SOFTWARE_POSTION_SOFTWAVE_SET(position);
}

static inline void qeo_disable_software_position_inject(QEO_Type *base)
{
    base->POSTION_SEL &= ~QEO_POSTION_SEL_POSTION_SEL_MASK;
}

#ifdef __cplusplus
}
#endif
/**
 * @}
 */
#endif /* HPM_QEO_DRV_H */
