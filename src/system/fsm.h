/**
 * @file fsm.h
 * @brief System Level Finite State Machine
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup system_fsm
 *
 * @{
 */

#ifndef OPENGLOW_CNC_STATE_H
#define OPENGLOW_CNC_STATE_H

#include <alchemy/task.h>
#include "../common.h"
#include "../config.h"

RT_TASK rt_fsm_loop;

enum system_state_type {
    SYS_STATE_CONSENSUS,   // State requires a consensus
    SYS_STATE_PRIORITY,    // State only requires a single sub fsm
};

// Define system states.
// These are the main states the system can be operating in. This is the highest level in the FSM hierarchy.
enum system_state {
    SYS_STATE_INIT,  // Default state.  Cannot exit until all sub-states have reached non-init state
    SYS_STATE_SLEEP,
    SYS_STATE_IDLE,
    SYS_STATE_HOMING,
    SYS_STATE_RUN,
    SYS_STATE_HOLD,
    SYS_STATE_ALARM,
    SYS_STATE_FAULT,
    N_SYS_STATES,
    FSM_STATE_NO_REQ = 254,
    FSM_STATE_UNINITIALIZED = 255,
};

// Sub-state machines
// These are sub-state machines local each control domain
enum sub_fsm {
    FSM_CLI,
    FSM_OPENGLOW,
    FSM_SWITCHES,
    FSM_MOTION,
    FSM_LIMITS,
    N_FSM,
};

// Sub-State Update Message
typedef struct sub_fsm_message {
    enum sub_fsm sub_fsm;   // Sub state machine
    uint8_t sub_state;
} sub_fsm_message_t;

// Sub-state to system-state map entry
typedef struct sys_to_sub {
    enum system_state system_state;
    uint8_t sub_state;
} sys_to_sub_map_t;

// Sub-state mappings
typedef struct sub_state_map {
    uint8_t num;   // Number map entries
    const sys_to_sub_map_t *maps; // Pointer to array of mappings
    void (*fsm_handler)(void);  // Sub FMS's handler for notifications of system state changes
} sub_state_map_t;

volatile enum system_state sys_state;

volatile enum system_state sys_req_state;

ssize_t fsm_init(void);

void fsm_register(enum sub_fsm sub_fsm, sub_state_map_t map);

void fsm_request(enum system_state state);

void fsm_reset(void);

ssize_t fsm_update(enum sub_fsm sub_fsm, uint8_t state);

#endif //OPENGLOW_CNC_STATE_H

/** @} */
