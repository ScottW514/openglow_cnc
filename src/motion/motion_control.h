/**
 * @file motion_control.h
 * @brief High level interface for issuing motion commands
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
 * @addtogroup motion_control
 *
 * @{
 */

#ifndef OPENGLOW_CNC_MOTION_CONTROL_H
#define OPENGLOW_CNC_MOTION_CONTROL_H

#include "../common.h"
#include "planner.h"

void mc_arc(float *target, plan_line_data_t *pl_data, float *position, const float *offset, float radius,
            uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear, uint8_t is_clockwise_arc);

void mc_dwell(float seconds);

void mc_line(float *target, plan_line_data_t *pl_data);

#endif //OPENGLOW_CNC_MOTION_CONTROL_H

/** @} */
