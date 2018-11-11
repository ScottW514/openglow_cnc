/**
 * @file openglow.c
 * @brief Interface to OpenGlow drivers
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup hardware
 * @{
 * @defgroup hardware_og OpenGlow Interface
 *
 * OpenGlow interface.
 * @{
 */

#include <alchemy/task.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include "../openglow-cnc.h"

/**
 * @brief OpenGlow state polling loop real time task
 */
static RT_TASK rt_openglow_poll_task;

/**
 * @brief OpenGlow device state file descriptor
 */
static FILE *openglow_pulse_fd;

/**
 * @brief OpenGlow device state file descriptor
 */
static int openglow_state_fd = 0;

const led_color_t btn_led_red = { .red = 100, .green = 0, .blue = 0 };      /*!< Predefined LED Color - RED */
const led_color_t btn_led_green = { .red = 0, .green = 100, .blue = 0 };    /*!< Predefined LED Color - GREEN */
const led_color_t btn_led_white = { .red = 60, .green = 80, .blue = 100 };  /*!< Predefined LED Color - WHITE */
const led_color_t btn_led_off = { .red = 0, .green = 0, .blue = 0 };        /*!< Predefined LED Color - OFF */

// Static function declarations
static void _openglow_state_poll();

/**
 * @brief Valid states for the OpenGlow Finite State Machine
 */
enum og_fsm_states {
        OG_STATE_INIT,
        OG_STATE_IDLE,
        OG_STATE_RUN,
        OG_STATE_DISABLED,
        OG_STATE_FAULT,
        OG_STATE_UNINITIALIZED = 255
};

/**
 * @brief System FSM to OpenGlow FSM Mapping
 *
 * Describes which OpenGlow FSM states are valid for System FSM states.
 */
static const sys_to_sub_map_t og_sys_sub[] = {
        {SYS_STATE_INIT, OG_STATE_INIT},
        {SYS_STATE_SLEEP, OG_STATE_DISABLED},
        {SYS_STATE_IDLE, OG_STATE_IDLE},
        {SYS_STATE_HOMING, OG_STATE_RUN},
        {SYS_STATE_HOMING, OG_STATE_IDLE},
        {SYS_STATE_RUN, OG_STATE_RUN},
        {SYS_STATE_HOLD, OG_STATE_IDLE},
        {SYS_STATE_FAULT, OG_STATE_FAULT},
};

/**
 * @brief FSM State Map registration structure
 *
 * Used to register the System FSM to OpenGlow FSM Mapping with the System FSM
 */
static const sub_state_map_t og_state_map = {
        .num = sizeof(og_sys_sub) / sizeof(sys_to_sub_map_t),
        .maps = &og_sys_sub[0],
};

/**
 * @brief OpenGlow FSM State
 *
 * Current state of the OpenGlow Finite State Machine
 */
static volatile enum og_fsm_states og_fsm_state = OG_STATE_UNINITIALIZED;

void openglow_button_led(led_color_t color) {
#ifdef TARGET_BUILD
    char buf[8];
    sprintf(buf, "%d\n", color.red);
    if (openglow_write_attr_str(ATTR_BUTTON_RED_PWM, buf) < 0)
        fprintf(stderr, "openglow_button_led: failed to write %d to '%s'\n", color.red, ATTR_BUTTON_RED_PWM);
    sprintf(buf, "%d\n", color.blue);
    if (openglow_write_attr_str(ATTR_BUTTON_BLUE_PWM, buf) < 0)
        fprintf(stderr, "openglow_button_led: failed to write %d to '%s'\n", color.red, ATTR_BUTTON_RED_PWM);
    sprintf(buf, "%d\n", color.green);
    if (openglow_write_attr_str(ATTR_BUTTON_GREEN_PWM, buf) < 0)
        fprintf(stderr, "openglow_button_led: failed to write %d to '%s'\n", color.red, ATTR_BUTTON_RED_PWM);
#endif // TARGET_BUILD
}

/**
 * @brief Clear OpenGlow buffer and counters
 *
 * @param cmd Clear command - OG_CLEAR
 * @return 0 on success, negative on failure.
 */
ssize_t openglow_clear(uint8_t cmd) {
    ssize_t ret = 0;
#ifdef TARGET_BUILD
    if ((ret = openglow_pulse_open()) < 0) {
        fprintf(stderr, "openglow_clear: failed to open '%s'\n", ATTR_PULSE);
        return ret;
    }
    if (cmd & OG_CLEAR_ALL) {
        ret = fseek(openglow_pulse_fd, 0, SEEK_SET);
    } else if (cmd & (OG_CLEAR_DATA | OG_CLEAR_DATA_CNTR)) {
        ret = fseek(openglow_pulse_fd, 1, SEEK_SET);
    } else if (cmd & OG_CLEAR_POSITION) {
        ret = fseek(openglow_pulse_fd, 2, SEEK_SET);
    }
    openglow_pulse_close();
#endif // TARGET_BUILD
    return ret;
}

/**
 * @brief Initialize OpenGlow hardware
 * @return 0 on success, negative on failure
 */
ssize_t openglow_init(){
    ssize_t ret = 0;
#ifdef TARGET_BUILD
    if((ret = openglow_write_attr_str(ATTR_ENABLE, "1")) < 0) {
        fprintf(stderr, "openglow_init: failed to enable OpenGlow controller\n");
        return ret;
    }

    char buf[256];
    sprintf(buf, "%d", STEP_FREQUENCY);
    if((ret = openglow_write_attr_str(ATTR_STEP_FREQ, buf)) < 0) {
        fprintf(stderr, "openglow_init: failed to set step frequency\n");
        return ret;
    }
#endif // TARGET_BUILD
    fsm_register(FSM_OPENGLOW, og_state_map);
#ifdef TARGET_BUILD
    og_fsm_state = OG_STATE_INIT;

    // Start state polling thread
    if ((ret = rt_task_spawn(&rt_openglow_poll_task, "rt_openglow_poll_task", 0, 50, 0, &_openglow_state_poll, 0)) < 0) {
        fprintf(stderr, "openglow_init: rt_task_spawn for rt_openglow_poll_task returned %zd\n", ret);
        return ret;
    }

#else // !TARGET_BUILD
    og_fsm_state = OG_STATE_IDLE;
    fsm_update(FSM_OPENGLOW, og_fsm_state);
#endif // TARGET_BUILD
    return ret;
}

/**
 * @brief Set Lid LED brightness
 * @param brightness 0-100 percent
 */
void openglow_lid_led(uint8_t brightness) {
    char buf[4];
    sprintf(buf, "%d\n", brightness);
    openglow_write_attr_str(ATTR_LID_LED_PWM, buf);
}

/**
 * @brief Read string value from sysfs attribute
 * @param attr Attribute to read
 * @param buf Buffer to store result in
 * @param length Length of buffer. MUST be large enough to hold the data
 * @return Length of data read on success, negative on failure
 */
ssize_t openglow_read_attr_str(const char *attr, char *buf, size_t length) {
    ssize_t ret;
    int fd;
    if ((fd = open(attr, O_RDONLY)) < 0) {
        fprintf(stderr, "openglow_read_attr_str: failed to open '%s'\n", attr);
        return fd;
    }
    buf[0] = 0;
    if ((ret = read(fd, buf, length)) > 0) buf[ret - 1] = 0; // Set the null terminator
    if (fd) close(fd);
    return ret;
}

/**
 * @brief Read unsigned long int value from sysfs attribute
 * @param attr Attribute to read
 * @param value unsigned long int to store result in
 * @return Length of data read on success, negative on failure
 */
ssize_t openglow_read_attr_uint32(const char *attr, uint32_t *value) {
    ssize_t ret;
    char buf_val[32];
    ret = openglow_read_attr_str(attr, buf_val, 31);
    *value = (uint32_t)strtoul(buf_val, NULL, 0);
    return ret;
};

/**
 * @brief Reset OpenGlow hardware
 */
void openglow_reset(){
    rt_task_delete(&rt_openglow_poll_task);
    openglow_write_attr_str(ATTR_DISABLE, "1");
    if (openglow_state_fd) close(openglow_state_fd);
#ifdef TARGET_BUILD
    if (openglow_pulse_fd) fclose(openglow_pulse_fd);
    openglow_button_led(btn_led_off);
#endif // TARGET_BUILD
}

/**
 * @brief OpenGlow state polling loop
 *
 * Polls OpenGlow state attribute and responds to changes in state.
 *
 * @note Runs as Xenomai Alchemy Task with a priority of 50. Blocked while waiting for event.
 */
static void _openglow_state_poll() {
    ssize_t ret = 0;
    uint8_t read_state;
    char buf[32];
    if ((openglow_state_fd = open(ATTR_STATE, O_RDONLY)) < 0) {
        fprintf(stderr, "_openglow_state_poll: failed to open %s. %d\n", ATTR_STATE, openglow_state_fd);
        og_fsm_state = OG_STATE_FAULT;
        fsm_update(FSM_OPENGLOW, og_fsm_state);

    }

#ifndef TARGET_BUILD
    og_fsm_state = OG_STATE_IDLE;
    fsm_update(FSM_OPENGLOW, og_fsm_state);
#endif // !TARGET_BUILD
    struct pollfd pollfd = { .fd = openglow_state_fd, .events = POLLPRI };
    while ((ret = poll(&pollfd, 1, -1)) >= 0) {
        lseek(pollfd.fd, 0, SEEK_SET);
        if ((ret = read(openglow_state_fd, buf, 31)) > 0) buf[ret - 1] = 0; // Set the null terminator
        if (strcmp(buf, "disabled") == 0) {
            if (sys_req_state == SYS_STATE_SLEEP) read_state = OG_STATE_DISABLED;
            else {
                read_state = OG_STATE_FAULT;
                fprintf(stderr, "_openglow_state_poll: unexpected disabled state\n");
                // TODO: Raise Alarm
            }
        } else if (strcmp(buf, "idle") == 0) read_state = OG_STATE_IDLE;
        else if (strcmp(buf, "running") == 0) read_state = OG_STATE_RUN;
        else read_state = OG_STATE_FAULT;
        if (read_state != og_fsm_state) {
            if ((og_fsm_state == OG_STATE_RUN) && (read_state == OG_STATE_IDLE)) fsm_request(SYS_STATE_IDLE);
            og_fsm_state = read_state;
            fsm_update(FSM_OPENGLOW, og_fsm_state);
            openglow_button_led((og_fsm_state == OG_STATE_RUN) ? btn_led_white : btn_led_off);
        }
    }
    fprintf(stderr, "_openglow_state_poll: poll returned %zd\n", ret);
    og_fsm_state = OG_STATE_FAULT;
    fsm_update(FSM_OPENGLOW, og_fsm_state);
}

/**
 * @brief Wirte string to sysfs attribute
 * @param attr Attribute to write to
 * @param value String to write
 * @return Length of data written on success, negative on failure
 */
ssize_t openglow_write_attr_str(const char *attr, char *value) {
    int fd;
    ssize_t ret = 0;
    fd = open(attr, O_WRONLY);
    if (fd < 0) {
        return (ssize_t)fd;
    }
    ret = write(fd, value, strlen(value));
    if (fd) close(fd);
    return ret;
}

/**
 * @brief Wirte unsigned long long int to sysfs attribute as hex string
 * @param attr Attribute to write to
 * @param value Unsigned long long int to write
 * @return Length of data written on success, negative on failure
 */
ssize_t openglow_write_attr_uint64(const char *attr, uint64_t value) {
    char buf_val[32];
    sprintf(buf_val, "0x%x", (uint32_t)(value & 0xFFFFFFFF));
    return openglow_write_attr_str(attr, buf_val);
};

/**
 * @brief Close OpenGlow pulse device
 */
void openglow_pulse_close() {
    if (openglow_pulse_fd > 0) {
        fclose(openglow_pulse_fd);
    }
    openglow_pulse_fd = 0;
}

/**
 * @brief Flush data written to OpenGlow pulse device
 */
void openglow_pulse_flush() {
    if (openglow_pulse_fd > 0) {
        fflush(openglow_pulse_fd);
    }
}

/**
 * @brief Open OpenGlow pulse device
 * @return 0 on success, negative on error.
 */
ssize_t openglow_pulse_open() {
    if (openglow_pulse_fd > 0) return 0;
    if ((openglow_pulse_fd = fopen(ATTR_PULSE, "wb")) < 0) {
        fprintf(stderr, "openglow_pulse_open: failed to open '%s'\n", ATTR_PULSE);
        return -1;
    }
    return 0;
}

/**
 * @brief Write single byte of data to OpenGlow pulse device
 * @param data Byte of data to write
 * @return 0 and success, negative on error.
 */
ssize_t openglow_pulse_write(uint8_t data) {
    if (openglow_pulse_fd > 0) {
        return fputc(data, openglow_pulse_fd);
    }
    return -1;
}

/** @} */
/** @} */
