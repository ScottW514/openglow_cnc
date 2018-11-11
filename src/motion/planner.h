/**
 * @file planner.h
 * @brief Buffers movement commands and manages the acceleration profile plan
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
 * @addtogroup motion_plan
 *
 * @{
 */

#ifndef OPENGLOW_CNC_PLANNER_H
#define OPENGLOW_CNC_PLANNER_H

#include "../common.h"
#include "../system/system.h"
#include "grbl_glue.h"

/**
 * @brief Plan Block Buffer size
 *
 * The number of linear motions that can be in the plan buffer at any give time
 */
#define BLOCK_BUFFER_SIZE 512

// Define motion data condition flags. Used to denote running conditions of a block.
#define PL_COND_FLAG_RAPID_MOTION      bit(0) /*!< Rapid Motion */
#define PL_COND_FLAG_SYSTEM_MOTION     bit(1) /*!< Single motion. Circumvents planner state. Used by home/park. */
#define PL_COND_FLAG_INVERSE_TIME      bit(3) /*!< Interprets feed rate value as inverse time when set. */
#define PL_COND_FLAG_SPINDLE_CW        bit(4) /*!< Spindle Clockwise*/
#define PL_COND_FLAG_SPINDLE_CCW       bit(5) /*!< Spindle Counter-Clockwise */
#define PL_COND_FLAG_COOLANT_FLOOD     bit(6) /*!< Flood Coolant */
#define PL_COND_FLAG_COOLANT_MIST      bit(7) /*!< Mist Coolant */

/**
 * @brief Linear movement block
 *
 * This struct stores a linear movement of a g-code block motion with its
 * critical "nominal" values are as specified in the source g-code.
 */
typedef struct {
    // Fields used by the bresenham algorithm for tracing the line
    // NOTE: Used by stepper algorithm to execute the block correctly. Do not alter these values.
    uint32_t steps[N_AXIS];    /*!< Step count along each axis */
    uint32_t step_event_count; /*!< The maximum step axis count and number of steps required to complete this block. */
    uint8_t direction_bits;    /*!< The direction bit set for this block (refers to *_DIRECTION_BIT in config.h) */

    // Block condition data to ensure correct execution depending on states and overrides.
    uint8_t condition;      /*!< Block bitflag variable defining block run conditions. Copied from pl_line_data. */

    // Fields used by the motion motion to manage acceleration. Some of these values may be updated
    // by the stepper module during execution of special motion cases for replanning purposes.
    float entry_speed_sqr;     /*!< The current planned entry speed at block junction in (mm/min)^2 */
    float max_entry_speed_sqr; /*!< Maximum allowable entry speed based on the minimum of junction limit and
                                    neighboring nominal speeds with overrides in (mm/min)^2 */
    float acceleration;        /*!< Axis-limit adjusted line acceleration in (mm/min^2). Does not change. */
    float millimeters;         /*!< The remaining distance for this block to be executed in (mm).
                                    @note This value may be altered by stepper algorithm during execution. */

    // Stored rate limiting data used by motion when changes occur.
    float max_junction_speed_sqr; /*!< Junction entry speed limit based on direction vectors in (mm/min)^2 */
    float rapid_rate;             /*!< Axis-limit adjusted maximum rate for this block direction in (mm/min) */
    float programmed_rate;        /*!< Programmed rate of this block (mm/min). */

    // Stored spindle speed data used by spindle overrides and resuming methods.
    float spindle_speed;    /*!< Block spindle speed. Copied from pl_line_data. */
} plan_block_t;

/**
 * @brief Planner Line Data
 *
 * Must be used when passing new motions to the motion planner.
 */
typedef struct {
    float feed_rate;          /*!< Desired feed rate for line motion. Value is ignored, if rapid motion. */
    float spindle_speed;      /*!< Desired spindle speed through line motion. */
    uint8_t condition;        /*!<  Bitflag variable to indicate motion conditions. See defines above. */
} plan_line_data_t;


bool plan_buffer_line(float *target, plan_line_data_t *pl_data);

bool plan_check_full_buffer();

float plan_compute_profile_nominal_speed(plan_block_t *block);

void plan_discard_current_block();

plan_block_t *plan_get_current_block();

float plan_get_exec_block_exit_speed_sqr();

plan_block_t *plan_get_system_motion_block();

uint16_t plan_next_block_index(uint16_t block_index);

void plan_reset(); // Reset all

void plan_reset_buffer(); // Reset buffer only.

void plan_sync_position();

#endif //OPENGLOW_CNC_PLANNER_H

/** @} */
