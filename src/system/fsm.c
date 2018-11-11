/**
 * @file fsm.c
 * @brief System Level Finite State Machine
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
 * @defgroup system_fsm Finite State Machine
 *
 * Finite State Machine
 *
 * @{
 */

#include <alchemy/queue.h>
#include <alchemy/task.h>
#include "../openglow-cnc.h"

/**
 * @brief FSM update queue
 */
static RT_QUEUE rt_fsm_queue;

// Static declarations
static bool _all_fsm_initialized();
static void _fsm_loop();
static void _update_system_state(enum system_state state);
static void _system_state_notify();

/**
 * @brief System State priority map
 */
static const uint8_t state_priority[N_SYS_STATES] = {
    [SYS_STATE_INIT] = SYS_STATE_PRIORITY,
    [SYS_STATE_SLEEP] = SYS_STATE_CONSENSUS,
    [SYS_STATE_IDLE] = SYS_STATE_CONSENSUS,
    [SYS_STATE_HOMING] = SYS_STATE_CONSENSUS,
    [SYS_STATE_RUN] = SYS_STATE_CONSENSUS,
    [SYS_STATE_HOLD] = SYS_STATE_CONSENSUS,
    [SYS_STATE_ALARM] = SYS_STATE_PRIORITY,
    [SYS_STATE_FAULT] = SYS_STATE_PRIORITY,
};

/**
 * @brief Current System State
 *
 * Stores the current system state.
 */
volatile enum system_state sys_state = FSM_STATE_UNINITIALIZED;

/**
 * @brief Requested System State
 *
 * Stores the requested system state.
 */
volatile enum system_state sys_req_state = FSM_STATE_UNINITIALIZED;

/**
 * @brief Reported Sub States
 *
 * Stores the reported sub system states.
 */
volatile static uint8_t _sub_state[N_FSM];

/**
 * @brief Sys-State to Sub-State Mappings
 *
 * This is used to map which sub-states are acceptable for each system state.
 * The System FSM compares each sub-state against this mapping to determine which
 * state the System FSM should be in.
 */
static sub_state_map_t _state_map[N_FSM];

/**
 * @brief Check if all sub fsm's have initialized
 *
 * If all sub fsm's have registered their state mappings, we know they initialized.
 * When a system wide reset is called for, all sub states are set to FSM_STATE_UNINITIALIZED
 * to indicate uninitialized.
 * @return true if all FSM are initialized, false otherwise.
 */
static bool _all_fsm_initialized() {
    bool s = true;
    for (uint8_t i = 0; i < N_FSM; i++) {
        if (_sub_state[i] == FSM_STATE_UNINITIALIZED) s = false;
    }
    return s;
}

/**
 * @brief Initializes state handlers
 * @return 0 on success, negative on error.
 */
ssize_t fsm_init() {
    ssize_t ret = 0;

    // Initialize rt_fsm_queue
    if ((ret = rt_queue_create(&rt_fsm_queue, "rt_fsm_queue",
                               sizeof(sub_fsm_message_t) * N_FSM, N_FSM, Q_FIFO)) < 0) {
        fprintf(stderr, "fsm_init: rt_queue_create for rt_fsm_queue returned %zd\n", ret);
        return ret;
    }

    // Start STATE_LOOP thread
    if ((ret = rt_task_spawn(&rt_fsm_loop, "rt_fsm_loop", 0, 50, T_JOINABLE, &_fsm_loop, 0)) < 0) {
        fprintf(stderr, "fsm_init: rt_task_spawn for rt_fsm_loop returned %zd\n", ret);
        return ret;
    }

    // Set all sub states to FSM_STATE_UNINITIALIZED
    for (uint8_t i = 0; i < N_FSM; i++) {
        _sub_state[i] = FSM_STATE_UNINITIALIZED;
    }

    sys_state = SYS_STATE_INIT;
    sys_req_state = SYS_STATE_IDLE;
    return ret;
}

/**
 * @brief Finite State Machine process loop
 */
static void _fsm_loop() {
    ssize_t ret = 0;
    sub_fsm_message_t status;
    while ((ret = rt_queue_read(&rt_fsm_queue, &status, sizeof(sub_fsm_message_t), 0)) > 0) {
        if (verbose) printf("_fsm_loop: read sub_fsm %d state %d from queue\n", status.sub_fsm, status.sub_state);
        _sub_state[status.sub_fsm] = status.sub_state;

        if (_all_fsm_initialized()) {
            uint16_t p_state[N_SYS_STATES];
            uint16_t p_mask = ((1 << N_FSM) - 1);
            memset(p_state, 0, N_SYS_STATES * sizeof(p_state[0]));

            // Cycle through state maps to find match
            for (uint8_t f = 0; f < N_FSM; f++) {
                for (uint8_t m = 0; m < _state_map[f].num; m++) {
                    if (_state_map[f].maps[m].sub_state == _sub_state[f]) {
                        // We have a match between sys and sub states.  Set the sys state bit for this sub.
                        p_state[_state_map[f].maps[m].system_state] |= (1 << f);
                    }
                }
            }

            uint8_t match_state = FSM_STATE_UNINITIALIZED;
            uint8_t matches = 0;
            // Check for priority states
            for (uint8_t i = 0; i < N_SYS_STATES; i ++) {
                if ((p_state[i] > 0) & (state_priority[i] == SYS_STATE_PRIORITY)) {
                    match_state = i; // We'll take the highest match
                }
            }
            if (match_state != FSM_STATE_UNINITIALIZED) {
                // We have a Priority State match!
                if (verbose) printf("_fsm_loop: priority state match\n");
                _update_system_state(match_state);;
            } else {
                if (p_state[sys_req_state] == p_mask) {
                    // We have a consensus on the requested state! Switch states
                    if (verbose) printf("_fsm_loop: consensus and requested states match\n");
                    _update_system_state(sys_req_state);
                } else {
                    // See if we have any consensus
                    for (uint8_t i = 0; i < N_SYS_STATES; i ++) {
                        if (p_state[i] == p_mask) {
                            match_state = i;
                            matches++;
                        }
                    }
                    if (matches == 1) {
                        // We have a consensus! Switch states
                        if (verbose) printf("_fsm_loop: found state consensus\n");
                        _update_system_state(match_state);
                    } else if (matches > 1) {
                        // Houston, we have a problem...
                        fprintf(stderr, "_fsm_loop: conflicting state consensus\n");
                        for (uint8_t i = 0; i < N_SYS_STATES; i ++) {
                            if (p_state[i] == 0xFF) {
                                fprintf(stderr, "_fsm_loop: consensus found for state %d\n", i);
                            }
                        }
                        // TODO: POSSIBLY RAISE ALARM
                    }
                }
            }
            // If we don't have a consensus or new priority state we don't change states.
        } else {
            // If any sub states are FSM_STATE_UNINITIALIZED, then the system state must be SYS_STATE_INIT
            _update_system_state(SYS_STATE_INIT);
        }
        if (verbose) {
            printf("_fsm_loop: %d/%d: ", sys_state, sys_req_state);
            for (uint8_t i = 0; i < N_FSM; i++) printf("%d ", _sub_state[i]);
            printf("\n");
        }
    }
    fprintf(stderr, "state_loop: exited %zd\n", ret);
}

/**
 * @brief Register sub state to system state mappings
 *
 * Used by sub fsm's to register their mappings
 * @param subfsm FSM that is registering
 * @param map Struct containing registration data
 */
void fsm_register(enum sub_fsm subfsm, sub_state_map_t map) {
    _sub_state[subfsm] = 0; // Set to 0 to indicate this sub fsm has initialized
    _state_map[subfsm] = map;
}

/**
 * @brief Request new state
 *
 * Used to request the system to enter a new state
 * @param state Desired new state
 */
void fsm_request(enum system_state state) {
    if (sys_req_state != state) {
        sys_req_state = state;
        if (verbose) printf("fsm_request: state %d requested\n", state);
        _system_state_notify();
        if ((sys_state != SYS_STATE_RUN) && (sys_req_state == SYS_STATE_RUN)) openglow_button_led(btn_led_green);
    }
}

/**
 * @brief Resets all state components
 */
void fsm_reset() {
    rt_task_delete(&rt_fsm_loop);
    rt_queue_delete(&rt_fsm_queue);
    sys_state = FSM_STATE_UNINITIALIZED;
}

/**
 * @brief Submit sub state update
 *
 * Used by sub fsm's to notify system fsm of their new state
 * @param subfsm  FSM submitting
 * @param state FSM's new state
 * @return
 */
ssize_t fsm_update(enum sub_fsm subfsm, uint8_t state) {
    ssize_t ret = 0;
    sub_fsm_message_t status;
    status.sub_fsm = subfsm;
    status.sub_state = state;

    if (status.sub_fsm > N_FSM) {
        fprintf(stderr, "fsm_update: sub_fsm %d invalid\n", status.sub_fsm);
        return -1;
    }
    if (_sub_state[status.sub_fsm] == FSM_STATE_UNINITIALIZED) {
        fprintf(stderr, "fsm_update: uninitialized sub_fsm %d submitted a state update - ignoring\n", status.sub_fsm);
        return -1;
    }
    if ((ret = rt_queue_write(&rt_fsm_queue, &status, sizeof(sub_fsm_message_t), Q_NORMAL)) < 0) {
        fprintf(stderr, "fsm_update: rt_fsm_queue return %zd\n", ret);
        // TODO: RAISE ALARM
    }
    return ret;
}

/**
 * @brief Notify FSM's of new system state or requested state
 *
 * Calls each FSM's notifier
 */
static void _system_state_notify() {
    for (uint8_t i = 0; i < N_FSM; i++) {
        if (_state_map[i].fsm_handler != NULL) {
            _state_map[i].fsm_handler();
        }
    }
}

/**
 * @brief Update System State
 *
 * Checks if provided state is different than existing state.
 * Notifies FSM's of update to condition var if system state changed.
 * @param state New system state
 */
static void _update_system_state(enum system_state state) {
    if (sys_state != state) {
        if (verbose) printf("_update_system_state: state changed from %d to %d\n", sys_state, state);
        sys_state = state;
        if (sys_state == sys_req_state) sys_req_state = FSM_STATE_NO_REQ;
        _system_state_notify();
    }
}

/** @} */
/** @} */
