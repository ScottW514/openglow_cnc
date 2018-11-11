/**
 * @file switches.h
 * @brief Input switch interface
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup hardware_sw
 *
 * @{
 */

#ifndef OPENGLOW_CNC_SWITCHES_H
#define OPENGLOW_CNC_SWITCHES_H

#include "../common.h"

ssize_t switches_init();

void switches_reset();

#endif // OPENGLOW_CNC_SWITCHES_H

/** @} */
