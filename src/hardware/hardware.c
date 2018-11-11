/**
 * @file hardware.c
 * @brief Hardware control
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @defgroup hardware Hardware Interface
 *
 * Hardware Interfaces
 *
 * Functions for communicating with the OpenGlow hardware.
 *
 * @{
 * @defgroup hardware_init Initialization
 *
 * Initialization of the hardware.
 *
 * @{
 */

#include "../openglow-cnc.h"

/**
 * @brief Initialize hardware
 * @return 0 on success, negative on error.
 */
ssize_t hardware_init() {
    ssize_t ret = 0;
    // Startup OpenGlow control board
    if ((ret = openglow_init()) < 0) {
        fprintf(stderr, "hardware_init: openglow_init returned %zd\n", ret);
        return ret;
    }

    // Startup switch thread
    if ((ret = switches_init()) < 0) {
        fprintf(stderr, "system_control_loop: switches_init returned %zd\n", ret);
        return ret;
    }

    // Startup limits thread
    if ((ret = limits_init()) < 0) {
        fprintf(stderr, "system_control_loop: limits_init returned %zd\n", ret);
        return ret;
    }

    // Startup STEPPER_LOOP thread
    if ((ret = stepgen_init()) < 0) {
        fprintf(stderr, "system_control_loop: stepgen_init returned %zd\n", ret);
        return ret;
    }
    return ret;
}

/**
 * @brief Reset hardware
 */
void hardware_reset() {
    openglow_reset();
    switches_reset();
    limits_reset();
}

/** @} */
/** @} */
