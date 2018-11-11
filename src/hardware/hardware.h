/**
 * @file hardware.h
 * @brief Hardware control
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup hardware_init Initialization
 *
 * @{
 */

#ifndef OPENGLOW_CNC_HARDWARE_H
#define OPENGLOW_CNC_HARDWARE_H

#include "../common.h"

ssize_t hardware_init();

void hardware_reset();

#endif //OPENGLOW_CNC_HARDWARE_H

/** @} */
