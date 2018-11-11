/**
 * @file switches.c
 * @brief Input switch interface
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup hardware
 * @{
 * @defgroup hardware_sw Input Switches
 *
 * Input Switch interface.
 * @{
 */

#include <alchemy/task.h>
#include <fcntl.h>
#include <linux/input.h>
#include <semaphore.h>
#include <sys/ioctl.h>
#include "../openglow-cnc.h"

#define SWITCH_DEVICE "/dev/input/event0"

#define SW_BEAM_DETECT_BIT  0
#define SW_INTERLOCK_BIT    1
#define SW_LID_SW1_BIT      2
#define SW_LID_SW2_BIT      3
#define SW_BUTTON_BIT       7

/**
 * @brief Switch event loop real time task
 */
static RT_TASK rt_sw_event_loop_task;

/**
 * @brief Switch device input file descriptor
 */
static int switches_fd = 0;

/**
 * @brief Switch FSM state mutex
 * Used to ensure atomic writes to the Switch FSM state variable
 */
static sem_t sw_fsm_state_mutex;

// Static function declarations
static void _switches_fsm_handler();
static inline void _switch_print_debug();
static bool _switches_safe();
static void _switches_event_loop();

/**
 * @brief Switches
 */
enum switches {
    SW_BEAM_DETECT,
    SW_INTERLOCK,
    SW_LID_SW1,
    SW_LID_SW2,
    SW_BUTTON,
    N_SWITCHES,
};

/**
 * @brief Valid states for the Switch Finite State Machine
 */
enum sw_fsm_states {
    SW_STATE_INIT,
    SW_STATE_SAFE,
    SW_STATE_RUN,
    SW_STATE_HOLD,
    SW_STATE_ALARM,
    SW_STATE_FAULT,
    SW_STATE_UNINITIALIZED = 255
};

/**
 * @brief System FSM to Switch FSM Mapping
 *
 * Describes which Switch FSM states are valid for System FSM states.
 */
static const sys_to_sub_map_t sw_sys_sub[] = {
        {SYS_STATE_INIT, SW_STATE_INIT},
        {SYS_STATE_SLEEP, SW_STATE_SAFE},
        {SYS_STATE_IDLE, SW_STATE_SAFE},
        {SYS_STATE_HOMING, SW_STATE_SAFE},
        {SYS_STATE_RUN, SW_STATE_RUN},
        {SYS_STATE_HOLD, SW_STATE_HOLD},
        {SYS_STATE_FAULT, SW_STATE_FAULT},
        {SYS_STATE_ALARM, SW_STATE_ALARM},
};

/**
 * @brief FSM State Map registration structure
 *
 * Used to register the System FSM to Switch FSM Mapping with the System FSM
 */
static const sub_state_map_t sw_state_map = {
        .num = sizeof(sw_sys_sub) / sizeof(sys_to_sub_map_t),
        .maps = &sw_sys_sub[0],
        .fsm_handler = _switches_fsm_handler,
};

/**
 * @brief Switch FSM State
 *
 * Current state of the Switch Finite State Machine
 */
static volatile enum sw_fsm_states sw_fsm_state = SW_STATE_UNINITIALIZED;

/**
 * @brief Switch input status structure
 */
static volatile input_status_t sw_status[N_SWITCHES] = {
        [SW_BEAM_DETECT]    = {false, SW_BEAM_DETECT_BIT, false},
        [SW_INTERLOCK]      = {false, SW_INTERLOCK_BIT, false},
        [SW_LID_SW1]        = {false, SW_LID_SW1_BIT, false},
        [SW_LID_SW2]        = {false, SW_LID_SW2_BIT, false},
        [SW_BUTTON]         = {false, SW_BUTTON_BIT, true},
};

/**
 * @brief Input device query mode structure
 */
static struct query_mode sw_query = {
        .name = "EV_SW",
        .event_type = EV_SW,
        .max = SW_MAX,
        .rq = EVIOCGSW(SW_MAX),
};

/**
 * @brief Switch event loop
 *
 * @note Runs as Xenomai Alchemy Task with a priority of 40. Blocked while waiting for event.
 */
void _switches_event_loop() {
    struct input_event ev[64];
    ssize_t i, rd;
    fd_set rdfs;

    FD_ZERO(&rdfs);
    FD_SET(switches_fd, &rdfs);

    if (sw_fsm_state == SW_STATE_INIT) {
        sw_fsm_state = SW_STATE_SAFE;
        fsm_update(FSM_SWITCHES, sw_fsm_state);
    }

    while (switches_fd) {
        select(switches_fd + 1, &rdfs, NULL, NULL, NULL);
        rd = read(switches_fd, ev, sizeof(ev));
        bool prev_state = _switches_safe();

        sem_wait(&sw_fsm_state_mutex);

        for (i = 0; i < rd / sizeof(struct input_event); i++) {
            if (ev[i].type == 5) {
                for (uint8_t j = 0; j < N_SWITCHES; j++) {
                    if (ev[i].code == sw_status[j].bit) {
                        if (verbose) printf("_switches_event_loop: code %d value %d\n", ev[i].code, ev[i].value);
                        sw_status[j].state = (ev[i].value) ? true : false;
                        if (sw_status[i].state & sw_status[i].invert) sw_status[i].state = false;
                    }
                }
            }
        }
        if (prev_state & !_switches_safe()) {
            if (verbose) printf("_switches_event_loop: safe state changed from true to false\n");
            sw_fsm_state = SW_STATE_ALARM;
        } else if (!prev_state & _switches_safe()) {
            if (verbose) printf("_switches_event_loop: safe state changed from false to true\n");
            sw_fsm_state = SW_STATE_SAFE;
        } else if ((sys_req_state == SYS_STATE_RUN) & sw_status[SW_BUTTON].state) {
            if (verbose) printf("_switches_event_loop: button pressed while run requested, switch to run\n");
            sw_fsm_state = SW_STATE_RUN;
            stepgen_wake_up();
        } else if ((sys_state == SYS_STATE_RUN) & sw_status[SW_BUTTON].state) {
//            if (verbose) printf("_switches_event_loop: button pressed while running, request hold\n");
//            fsm_request(SYS_STATE_HOLD);
//            sw_fsm_state = SW_STATE_HOLD;
//            openglow_write_attr_str(ATTR_STOP, "1\n");
        } else {
            sw_fsm_state = SW_STATE_SAFE;
        }
        if (prev_state != sw_fsm_state) {
            fsm_update(FSM_SWITCHES, sw_fsm_state);
        }
        sem_post(&sw_fsm_state_mutex);
    }
    ioctl(switches_fd, EVIOCGRAB, (void*)0);
    close(switches_fd);
    fprintf(stderr, "switches_loop: exited\n");
    fsm_update(FSM_SWITCHES, SW_STATE_FAULT);
    switches_reset();
}

/**
 * @brief System FSM Message handler
 */
static void _switches_fsm_handler() {
    sem_wait(&sw_fsm_state_mutex);
    uint8_t prev_state = sw_fsm_state;

    if (_switches_safe()) {
        if ((sys_state == SYS_STATE_HOLD) & (sw_fsm_state == SW_STATE_HOLD)); // Do nothing
        else if (sys_state == SYS_STATE_RUN) sw_fsm_state = SW_STATE_SAFE;
    } else sw_fsm_state = SW_STATE_ALARM;

    if (prev_state != sw_fsm_state) {
        fsm_update(FSM_SWITCHES, sw_fsm_state);
    }
    sem_post(&sw_fsm_state_mutex);
}

/**
 * @brief Initialize Switch hardware
 * @return 0 on success, negative on failure
 */
ssize_t switches_init() {
    ssize_t ret = 0;
    sem_init(&sw_fsm_state_mutex, 0, 1);

#ifdef TARGET_BUILD
    // Read current switch states
    if ((switches_fd = open(SWITCH_DEVICE, O_RDWR)) < 0) {
        fprintf(stderr, "switches_init: open returned %d\n", switches_fd);
        return switches_fd;
    }
    unsigned long state[NBITS(sw_query.max)];
    memset(state, 0, sizeof(state));
    if ((ret = ioctl(switches_fd, sw_query.rq, state)) < 0) {
        fprintf(stderr, "switches_init: ioctl returned %zd\n", ret);
        return ret;
    }

#endif // TARGET_BUILD
    for (uint8_t i = 0; i < N_SWITCHES; i++) {
#ifdef TARGET_BUILD
        sw_status[i].state = (state[0] & bit(sw_status[i].bit)) ? true : false;
        if (sw_status[i].state & sw_status[i].invert) sw_status[i].state = false;
#else // !TARGET_BUILD
        sw_status[i].state = true;
#endif // TARGET_BUILD
    }

#ifdef TARGET_BUILD
    sw_fsm_state = (_switches_safe()) ? SW_STATE_INIT : SW_STATE_ALARM;
#else // !TARGET_BUILD
    sw_fsm_state = SW_STATE_SAFE;
#endif // TARGET_BUILD

    fsm_register(FSM_SWITCHES, sw_state_map);
    fsm_update(FSM_SWITCHES, sw_fsm_state);

    if (verbose) _switch_print_debug();

#ifdef TARGET_BUILD
    // Grab the switch device
    if ((ret = ioctl(switches_fd, EVIOCGRAB, (void*)1)) < 0) {
        fprintf(stderr, "switches_init: ioctl EVIOCGRAB returned %zd\n", ret);
        return ret;
    }

    if ((rt_task_spawn(&rt_sw_event_loop_task, "rt_sw_event_loop_task", 0, 40, 0, &_switches_event_loop, 0)) < 0) {
        fprintf(stderr, "switches_init: rt_task_spawn for rt_sw_event_loop_task returned %zd\n", ret);
        return ret;
    }
#endif // TARGET_BUILD

    return ret;
}

/**
 * @brief Switch print debug information
 */
static inline void _switch_print_debug() {
    printf("switch: BD:%d IL%d L1%d L2%d BTN%d\n",
           sw_status[SW_BEAM_DETECT].state, sw_status[SW_INTERLOCK].state,
           sw_status[SW_LID_SW1].state, sw_status[SW_LID_SW2].state,
           sw_status[SW_BUTTON].state);
}

/**
 * @brief Reset Switch hardware
 */
void switches_reset() {
    rt_task_delete(&rt_sw_event_loop_task);
    sw_fsm_state = SW_STATE_UNINITIALIZED;
}

static bool _switches_safe() {
    bool stat = true;
#ifdef TARGET_BUILD
    for (uint8_t i = 0; i < N_SWITCHES; i++) {
        if (!sw_status[i].state && (i != SW_BUTTON) && (i != SW_BEAM_DETECT)) stat = false;
    }
#endif // TARGET_BUILD
    return stat;
}

/** @} */
/** @} */
