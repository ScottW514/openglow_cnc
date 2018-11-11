/**
 * @file console.h
 * @brief Console Interface
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup cli_console
 *
 * @{
 */

#ifndef OPENGLOW_CNC_CONSOLE_H
#define OPENGLOW_CNC_CONSOLE_H

#include "../common.h"

ssize_t console_init();

void console_reset();

ssize_t console_write(char *line, va_list args);

#endif //OPENGLOW_CNC_CONSOLE_H

/** @} */
