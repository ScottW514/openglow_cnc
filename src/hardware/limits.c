/**
 * @file limits.c
 * @brief Limit switches and homing cycle
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup hardware
 * @{
 * @defgroup hardware_limits Limits and Homing
 *
 * Limit switches and homing cycle.
 * @{
 */

#include <alchemy/task.h>
#include <fcntl.h>
#include <linux/input.h>
#include <semaphore.h>
#include <math.h>
#include <memory.h>
#include <sys/ioctl.h>
#include "../openglow-cnc.h"

#define LIMIT_DEVICE "/dev/input/event1"

#define LIMIT_X_POS_BIT    1
#define LIMIT_X_NEG_BIT    2
#define LIMIT_Y1_POS_BIT   3
#define LIMIT_Y1_NEG_BIT   4
#define LIMIT_Y2_POS_BIT   5
#define LIMIT_Y2_NEG_BIT   6

/**
 * @brief Limits event loop real time task
 */
static RT_TASK rt_limits_event_loop_task;

/**
 * @brief Limits device input file descriptor
 */
static int limits_fd = 0;

/**
 * @brief Limits FSM state mutex
 * Used to ensure atomic writes to the Limits FSM state variable
 */
static sem_t limits_mutex;

// Static function declarations
static void _limits_event_loop();
static bool _limits_ok();
static inline void _limit_print_debug();
static void _limits_fsm_handler();

/**
 * @brief Limit Switches
 */
enum limit_switches {
    LIMIT_X_POS,
    LIMIT_X_NEG,
    LIMIT_Y1_POS,
    LIMIT_Y1_NEG,
    LIMIT_Y2_POS,
    LIMIT_Y2_NEG,
    N_LIMIT_SW,
};

/**
 * @brief Valid states for the Limits Finite State Machine
 */
enum limit_fsm_states {
    LIMIT_STATE_INIT,
    LIMIT_STATE_SAFE,
    LIMIT_STATE_HOMING,
    LIMIT_STATE_ALARM,
    LIMIT_STATE_FAULT,
    LIMIT_STATE_UNINITIALIZED = 255
};

/**
 * @brief System FSM to Limits FSM Mapping
 *
 * Describes which Limits FSM states are valid for System FSM states.
 */
static const sys_to_sub_map_t limit_sys_sub[] = {
        {SYS_STATE_INIT, LIMIT_STATE_INIT},
        {SYS_STATE_SLEEP, LIMIT_STATE_SAFE},
        {SYS_STATE_IDLE, LIMIT_STATE_SAFE},
        {SYS_STATE_IDLE, LIMIT_STATE_HOMING},
        {SYS_STATE_HOMING, LIMIT_STATE_HOMING},
        {SYS_STATE_RUN, LIMIT_STATE_SAFE},
        {SYS_STATE_HOLD, LIMIT_STATE_SAFE},
        {SYS_STATE_FAULT, LIMIT_STATE_FAULT},
        {SYS_STATE_ALARM, LIMIT_STATE_ALARM},
};

/**
 * @brief FSM State Map registration structure
 *
 * Used to register the System FSM to Limits FSM Mapping with the System FSM
 */
static const sub_state_map_t limit_state_map = {
        .num = sizeof(limit_sys_sub) / sizeof(sys_to_sub_map_t),
        .maps = &limit_sys_sub[0],
        .fsm_handler = _limits_fsm_handler,
};

/**
 * @brief Limits FSM State
 *
 * Current state of the Limits Finite State Machine
 */
static volatile enum limit_fsm_states limit_fsm_state = LIMIT_STATE_UNINITIALIZED;

/**
 * @brief Limits input status structure
 */
static volatile input_status_t limit_status[N_LIMIT_SW] = {
        [LIMIT_X_POS]   = {false, LIMIT_X_POS_BIT, false},
        [LIMIT_X_NEG]   = {false, LIMIT_X_NEG_BIT, false},
        [LIMIT_Y1_POS]  = {false, LIMIT_Y1_POS_BIT, false},
        [LIMIT_Y1_NEG]  = {false, LIMIT_Y1_NEG_BIT, false},
        [LIMIT_Y2_POS]  = {false, LIMIT_Y2_POS_BIT, false},
        [LIMIT_Y2_NEG]  = {false, LIMIT_Y2_NEG_BIT, false},
};

/**
 * @brief Input device query mode structure
 */
static struct query_mode limit_query = {
        .name = "EV_SW",
        .event_type = EV_SW,
        .max = SW_MAX,
        .rq = EVIOCGSW(SW_MAX),
};

/**
 * @brief Limits event loop
 *
 * @note Runs as Xenomai Alchemy Task with a priority of 40. Blocked while waiting for event.
 */
static void _limits_event_loop() {
    struct input_event ev[64];
    ssize_t i, rd;
    fd_set rdfs;

    FD_ZERO(&rdfs);
    FD_SET(limits_fd, &rdfs);

    while (limits_fd) {
        select(limits_fd + 1, &rdfs, NULL, NULL, NULL);
        rd = read(limits_fd, ev, sizeof(ev));

        sem_wait(&limits_mutex);
        bool prev_state = _limits_ok();

        for (i = 0; i < rd / sizeof(struct input_event); i++) {
            if (ev[i].type == 5) {
                for (uint8_t j = 0; j < N_LIMIT_SW; j++) {
                    if (ev[i].code == limit_status[j].bit) {
                        if (verbose) printf("limits_loop: code %d value %d\n", ev[i].code, ev[i].value);
                        limit_status[j].state = (ev[i].value) ? true : false;
                        if (limit_status[i].state & limit_status[i].invert) limit_status[i].state = false;
                    }
                }
            }
        }
        if (prev_state & !_limits_ok() ) {
            if (verbose) printf("limits_loop: limit_ok state changed from true to false\n");
            limit_fsm_state = LIMIT_STATE_ALARM;
        }
        if (prev_state != limit_fsm_state) {
            fsm_update(FSM_LIMITS, limit_fsm_state);
        }
        sem_post(&limits_mutex);
    }
    ioctl(limits_fd, EVIOCGRAB, (void*)0);
    close(limits_fd);
    fprintf(stderr, "limits_loop: exited\n");
    fsm_update(FSM_LIMITS, LIMIT_STATE_FAULT);
}

/**
 * @brief System FSM Message handler
 */
static void _limits_fsm_handler() {
    sem_wait(&limits_mutex);
    bool prev_state = limit_fsm_state;

    if (_limits_ok()) {
        limit_fsm_state = LIMIT_STATE_SAFE;
    } else limit_fsm_state = LIMIT_STATE_ALARM;

    if (prev_state != limit_fsm_state) {
        fsm_update(FSM_LIMITS, limit_fsm_state);
    }
    sem_post(&limits_mutex);
}

/**
 * @brief Initialize Limits hardware
 * @return 0 on success, negative on failure
 */
ssize_t limits_init() {
    ssize_t ret = 0;
    sem_init(&limits_mutex, 0, 1);

#ifdef TARGET_BUILD
    // Read current limit states
    if ((limits_fd = open(LIMIT_DEVICE, O_RDWR)) < 0) {
        fprintf(stderr, "limits_init: open returned %d\n", limits_fd);
        return limits_fd;
    }
    unsigned long state[NBITS(limit_query.max)];
    memset(state, 0, sizeof(state));
    if ((ret = ioctl(limits_fd, limit_query.rq, state)) < 0) {
        fprintf(stderr, "limits_init: ioctl returned %zd\n", ret);
        return ret;
    }

#endif // TARGET_BUILD
    for (uint8_t i = 0; i < N_LIMIT_SW; i++) {
#ifdef TARGET_BUILD
        limit_status[i].state = (state[0] & bit(limit_status[i].bit)) ? true : false;
        if (limit_status[i].state & limit_status[i].invert) limit_status[i].state = false;
#else // !TARGET_BUILD
        limit_status[i].state = true;
#endif // TARGET_BUILD
    }

    limit_fsm_state = (_limits_ok()) ? LIMIT_STATE_SAFE : LIMIT_STATE_ALARM;

    fsm_register(FSM_LIMITS, limit_state_map);
    fsm_update(FSM_LIMITS, limit_fsm_state);

    if (verbose) _limit_print_debug();

#ifdef TARGET_BUILD
    // Grab the limits device
    if ((ret = ioctl(limits_fd, EVIOCGRAB, (void*)1)) < 0) {
        fprintf(stderr, "limits_init: ioctl EVIOCGRAB returned %zd\n", ret);
        return ret;
    }

    if ((rt_task_spawn(&rt_limits_event_loop_task, "rt_limits_event_loop_task", 0, 40, 0, &_limits_event_loop, 0)) < 0) {
        fprintf(stderr, "limits_init: rt_task_spawn returned %zd\n", ret);
        return ret;
    }
#endif

    return ret;
}

/**
 * @brief Check limit switch status
 * @return True if all switches are ok, False if not
 */
static bool _limits_ok() {
    bool stat = true;
#ifdef TARGET_BUILD
    for (uint8_t i = 0; i < N_LIMIT_SW; i++) {
        if (!limit_status[i].state) stat = false;
    }
#endif // TARGET_BUILD
    return stat;
}

/**
 * @brief Limits print debug information
 */
static inline void _limit_print_debug() {
    printf("limits_init: X+%d-%d Y1+%d-%d Y2+%d-%d \n",
           limit_status[LIMIT_X_POS].state, limit_status[LIMIT_X_NEG].state,
           limit_status[LIMIT_Y1_POS].state, limit_status[LIMIT_Y1_NEG].state,
           limit_status[LIMIT_Y2_POS].state, limit_status[LIMIT_Y2_NEG].state);
}

/**
 * @brief Reset Limits hardware
 */
void limits_reset() {
    rt_task_delete(&rt_limits_event_loop_task);
}

/** @} */
/** @} */
