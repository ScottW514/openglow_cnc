/**
 * @file settings.h
 * @brief System settings
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup system_settings
 *
 * @{
 */

#ifndef OPENGLOW_CNC_SETTINGS_H
#define OPENGLOW_CNC_SETTINGS_H

#include "../common.h"
#include "../config.h"

/**
 * @brief CLI settings struct
 */
typedef struct cli_s {
    uint8_t comm_mode;  /*!< Transport mode for CLI */
    // Socket settings
    struct in_addr listen_ip;   /*!< IP to listen on for socket mode */
    uint16_t listen_port;       /*!< Port to listen on for socket mode */

    // Interface defaults
    bool auto_cycle;    /*!< et to true, machine will automatically cycle start when motion buffer fills. */
    bool mdi_mode;      /*!< Set to true, machine will execute each line of G-code as it is entered. */
    uint8_t report_units; /*!< Measurement units for CLI messages */
} cli_t;

/**
 * @brief System Settings struct
 */
typedef struct settings_s {
    cli_t cli;  /*!< CLI settings */

    bool daemon;    /*!< Run in dameon mode */
    bool laser_power_correction;    /*!< Laser power correction for speed */
    bool soft_limits;   /*!< Enable soft limit checks */

    float steps_per_mm[N_AXIS];
    float acceleration[N_AXIS];
    float max_rate[N_AXIS];
    float max_travel[N_AXIS];
} settings_t;

settings_t settings;

#endif //OPENGLOW_CNC_SETTINGS_H

/** @} */
