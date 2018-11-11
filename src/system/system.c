/**
 * @file system.c
 * @brief System Contro
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @defgroup system System Control
 *
 * System Control
 *
 * Functions for system control.
 *
 * @{
 * @defgroup system_init Initialization
 *
 * Initialization of the system control.
 *
 * @{
 */

#include "../openglow-cnc.h"

/**
 * @brief Start all subsystems
 * @return 0 on success, negative on error.
 */
ssize_t system_control_init() {
    ssize_t ret = 0;

    // Sync cleared gcode and motion positions to current system position.
    plan_sync_position();
    gc_sync_position();

    // Startup FSM
    if ((ret = fsm_init()) < 0) {
        fprintf(stderr, "system_control_init: fsm_init returned %zd\n", ret);
        return ret;
    }

    // Startup CLI
    if ((ret = cli_init()) < 0) {
        fprintf(stderr, "system_control_init: cli_init returned %zd\n", ret);
        return ret;
    }

    // Startup Hardware
    if ((ret = hardware_init()) < 0) {
        fprintf(stderr, "system_control_init: hardware_init returned %zd\n", ret);
        return ret;
    }

    // Startup Motion
    if ((ret = motion_init()) < 0) {
        fprintf(stderr, "system_control_init: motion_init returned %zd\n", ret);
        return ret;
    }

    // Everything initialized, send out the welcome message
    message_write(MSG_WELCOME_BANNER, OPENGLOW_CNC_VER);

    return rt_task_join(&rt_fsm_loop);
}

/**
 * @brief Block until all buffered steps are executed or in a cycle state.
 * Works with feed hold during a synchronize call, if it should happen.
 * Also, waits for clean cycle end.
 */
void system_buffer_synchronize() {
    if (verbose) printf("system_buffer_synchronize: init\n");
    // If system is queued, ensure cycle resumes if the auto start flag is present.
//    system_auto_cycle_start();
//    do {
//        if (state.abort) { return; } // Check for system abort
//    } while (plan_get_current_block() || (sys.state.mode == STATE_CYCLE));
}

/** @} */
/** @} */
