/**
 * @file settings.c
 * @brief System settings
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup system
 *
 * @{
 * @defgroup system_settings System Settings
 *
 * System Settings
 *
 * @{
 */

#include "../openglow-cnc.h"

/**
 * @brief System Settings
 */
settings_t settings = {
    .cli = {
            .auto_cycle  = true,
            .mdi_mode = MDI_MODE,
            .report_units = REPORT_UNITS,
    },
    .soft_limits = true,
    .laser_power_correction = true,

    .steps_per_mm[X_AXIS] = X_STEPS_PER_MM,
    .steps_per_mm[Y_AXIS] = Y_STEPS_PER_MM,
    .steps_per_mm[Z_AXIS] = Z_STEPS_PER_MM,

    .max_rate[X_AXIS] = X_MAX_RATE,
    .max_rate[Y_AXIS] = Y_MAX_RATE,
    .max_rate[Z_AXIS] = Z_MAX_RATE,

    .acceleration[X_AXIS] = X_ACCELERATION,
    .acceleration[Y_AXIS] = Y_ACCELERATION,
    .acceleration[Z_AXIS] = Z_ACCELERATION,

    .max_travel[X_AXIS] = (-X_MAX_TRAVEL),
    .max_travel[Y_AXIS] = (-Y_MAX_TRAVEL),
    .max_travel[Z_AXIS] = (-Z_MAX_TRAVEL)
};

/** @} */
/** @} */
