/**
 * @file stepgen.h
 * @brief Step generator
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * Adapted from Grbl
 * @copyright Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
 * @copyright Copyright (c) 2009-2011 Simen Svale Skogsrud

 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup hardware_stepgen
 *
 * @{
 */

#ifndef OPENGLOW_CNC_STEPGEN_H
#define OPENGLOW_CNC_STEPGEN_H

#include "../common.h"

int32_t sys_position[N_AXIS];

void stepgen_clear();

ssize_t stepgen_go_idle();

ssize_t stepgen_init();

ssize_t stepgen_wake_up();

#endif //OPENGLOW_CNC_STEPGEN_H

/** @} */
