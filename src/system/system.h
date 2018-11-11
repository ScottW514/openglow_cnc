/**
 * @file system.h
 * @brief System Contro
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup system_init
 *
 * @{
 */

#ifndef OPENGLOW_CNC_SYSTEM_H
#define OPENGLOW_CNC_SYSTEM_H

#include <arpa/inet.h>
#include "../cli/cli.h"

void system_buffer_synchronize();

ssize_t system_control_init();

#endif // OPENGLOW_CNC_SYSTEM_H

/** @} */
