/**
 * @file limits.h
 * @brief Limit switches and homing cycle
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup hardware_limits
 *
 * @{
 */

#ifndef OPENGLOW_CNC_LIMITS_H
#define OPENGLOW_CNC_LIMITS_H

#include "../common.h"

ssize_t limits_init();

void limits_reset();

#endif //OPENGLOW_CNC_LIMITS_H

/** @} */
