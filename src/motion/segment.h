/**
 * @file segment.h
 * @brief Calculates individual segments for motion blocks
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * Adapted from Grbl
 * @copyright Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
 * @copyright Copyright (c) 2009-2011 Simen Svale Skogsrud
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup motion_segment
 *
 * @{
 */

#ifndef OPENGLOW_CNC_SEGMENT_H
#define OPENGLOW_CNC_SEGMENT_H

#include "../common.h"
#include "planner.h"
#include "../system/system.h"

#define SEGMENT_BUFFER_SIZE 256

volatile uint16_t segment_buffer_tail;
volatile uint16_t segment_buffer_head;

/**
 * @brief Motion block Bresenham data
 */
typedef struct {
    uint32_t steps[N_AXIS];
    uint32_t step_event_count;
    uint8_t direction_bits;
    uint8_t is_pwm_rate_adjusted; /*!< Tracks motions that require constant laser power/rate */
} st_block_t;

st_block_t st_block_buffer[SEGMENT_BUFFER_SIZE - 1];

/**
 * @brief Primary stepper segment ring buffer data
 */
typedef struct {
    uint16_t n_step;           /*!< Number of step events to be executed for this segment */
    uint32_t cycles_per_tick;  /*!< Step distance traveled per ISR tick, aka step rate. */
    uint8_t st_block_index;    /*!<  Stepper block data index. Uses this information to execute this segment. */
    uint8_t spindle_pwm;
} segment_t;

segment_t segment_buffer[SEGMENT_BUFFER_SIZE];

/**
 * @brief Segment preparation data struct.
 *
 * Contains all the necessary information to compute new segments based on the current executing motion block.
 */
typedef struct {
    uint8_t st_block_index;  /*!< Index of stepper common data block being prepped */
    uint8_t recalculate_flag;

    float dt_remainder;
    float steps_remaining;
    float step_per_mm;
    float req_mm_increment;

    uint8_t ramp_type;      /*!< Current segment ramp state */
    float mm_complete;      /*!< End of velocity profile from end of current motion block in (mm).
                                @note This value must coincide with a step(no mantissa) when converted. */
    float current_speed;    /*!< Current speed at the end of the segment buffer (mm/min) */
    float maximum_speed;    /*!< Maximum speed of executing block. Not always nominal speed. (mm/min) */
    float exit_speed;       /*!< Exit speed of executing block (mm/min) */
    float accelerate_until; /*!< Acceleration ramp end measured from end of block (mm) */
    float decelerate_after; /*!< Deceleration ramp start measured from end of block (mm) */

    float inv_rate;    /*!< Used by PWM laser mode to speed up segment calculations. */
    uint8_t current_spindle_pwm;
} st_prep_t;

st_prep_t prep;

plan_block_t *pl_block;

void segment_reset();

void segment_prep_buffer();

void st_update_plan_block_parameters();

#endif //OPENGLOW_CNC_SEGMENT_H

/** @} */
