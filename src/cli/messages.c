/**
 * @file messages.c
 * @brief Message outputs to user
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup cli
 * @{
 * @defgroup cli_messages User Message Functions
 *
 * Send messages to the user via the CLI transport.
 *
 * @{
 */

#include <stdarg.h>
#include "../openglow-cnc.h"

/**
 * @brief Message type strings
 */
static const messages_t messages[N_MESSAGES] = {
        [MSG_ALARM]                 = {"ALARM:%zd", false},
        [MSG_ERROR]                 = {"error:%zd", false},
        [MSG_FEEDBACK]              = {"[MSG:%s]", false},
        [MSG_HELP]                  = {"[HLP:$$ $# $G $I $N $SLP $C $X $H ~ ! ? X]", true},
        [MSG_OK]                    = {"ok", false},
        [MSG_PLAIN_TEXT]            = {"%s", false},
        [MSG_STATUS_REPORT]         = {"<%s,MPos:%1.3f,%1.3f,%1.3f>", true},
        [MSG_WELCOME_BANNER]        = {"OpenGlow CNC v%s ['$' for help]", false},
};

/**
 * @brief Message state strings
 */
const messages_t states[N_SYS_STATES] = {
        [SYS_STATE_INIT]            = {"Init", 0},
        [SYS_STATE_IDLE]            = {"Idle", 0},
        [SYS_STATE_ALARM]           = {"Run", 0},
        [SYS_STATE_FAULT]           = {"Fault", 0},
        [SYS_STATE_HOMING]          = {"Home", 0},
        [SYS_STATE_RUN]             = {"Run", 0},
        [SYS_STATE_HOLD]            = {"Hold", 0},
        [SYS_STATE_SLEEP]           = {"Sleep", 0},
};

void _message_write(char *msg, va_list args);

/**
 * @brief Display alarm message
 * @param alarm STATUS_CODE for the alarm
 */
void message_alarm(ssize_t alarm) {
    message_write(MSG_ALARM, alarm);
}

/**
 * @brief Display feedback message
 * @param feeback Feedback message string
 */
void message_feedback(char *feeback) {
    message_write(MSG_FEEDBACK, feeback);
}

/**
 * @brief Display status message
 * @param status STATUS_CODE for the message
 */
void message_status(ssize_t status) {
    if (status == 0) {
        message_write(MSG_OK);
    } else {
        message_write(MSG_ERROR, status);
    }
}

/**
 * @brief Write message to proper transport
 * Internal use
 * @param msg Message type
 * @param args printf format data
 */
void _message_write(char *msg, va_list args) {
    switch (settings.cli.comm_mode) {
        case CLI_CONSOLE: {
            console_write(msg, args);
            return;
        }
        case CLI_SOCKET: {
            socket_write(msg, args);
            return;
        }
        default:;
    }
}

/**
 * @brief Write Message to CLI
 * @param msg Message type
 * @param ... Args for message type
 */
void message_write(ssize_t msg, ...) {
    va_list args;
    va_start(args, msg);
    _message_write(messages[msg].text, args);
    if (messages[msg].ok) {
        _message_write(messages[MSG_OK].text, args);
    }
}

/**
 * @brief Convert steps to distance as float
 * @param steps Raw number of steps - array
 * @param idx The axis to convert
 * @return Distance in units from settings
 */
float steps_to_float(int32_t steps, uint8_t idx) {
    if (settings.cli.report_units) {
        return ((steps / settings.steps_per_mm[idx]) * (float) INCH_PER_MM);
    } else {
        return (steps / settings.steps_per_mm[idx]);
    }
}

/** @} */
/** @} */
