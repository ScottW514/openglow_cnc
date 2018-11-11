/**
 * @file motion.h
 * @brief Motion Controller
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup motion_init
 *
 * @{
 */

#ifndef OPENGLOW_CNC_MOTION_H
#define OPENGLOW_CNC_MOTION_H

#include "../common.h"

/**
 * @brief Valid states for the Motion Finite State Machine
 */
enum mot_fsm_states {
    MOT_STATE_INIT,
    MOT_STATE_IDLE,
    MOT_STATE_RUN,
    MOT_STATE_HOLD,
    MOT_STATE_ALARM,
    MOT_STATE_FAULT,
    MOT_STATE_UNINITIALIZED = 255
};

ssize_t motion_init();

void motion_reset();

void motion_state_update(enum mot_fsm_states state);

#endif //OPENGLOW_CNC_MOTION_H

/** @} */
