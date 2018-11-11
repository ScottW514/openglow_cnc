/**
 * @file messages.h
 * @brief Message outputs to user
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup cli_messages
 *
 * @{
 */

#ifndef OPENGLOW_CNC_MESSAGES_H
#define OPENGLOW_CNC_MESSAGES_H

#include "../common.h"
#include "../system/system.h"
#include "../system/fsm.h"

/**
 * @brief Message text map
 */
typedef struct messages_s {
    char *text;
    bool ok;
} messages_t;

const messages_t states[N_SYS_STATES];

/**
 * @brief CLI Message types
 */
enum MESSAGE_TYPES {
    MSG_ALARM,
    MSG_ERROR,
    MSG_FEEDBACK,
    MSG_HELP,
    MSG_OK,
    MSG_PLAIN_TEXT,
    MSG_STATUS_REPORT,
    MSG_WELCOME_BANNER,
    N_MESSAGES,
};

/**
 * @brief Status codes
 */
enum STATUS_CODES {
        STATUS_OK = 0,
        STATUS_EXPECTED_COMMAND_LETTER = 1,
        STATUS_BAD_NUMBER_FORMAT = 2,
        STATUS_INVALID_STATEMENT = 3,
        STATUS_NEGATIVE_VALUE = 4,
        STATUS_IDLE_ERROR = 8,
        STATUS_SYSTEM_GC_LOCK = 9,
        STATUS_SOFT_LIMIT_ERROR = 10,
        STATUS_OVERFLOW = 11,
        STATUS_MAX_STEP_RATE_EXCEEDED = 12,
        STATUS_CHECK_DOOR = 13,
        STATUS_LINE_LENGTH_EXCEEDED = 14,
        STATUS_TRAVEL_EXCEEDED = 15,
        STATUS_SETTING_DISABLED_LASER = 17,

        STATUS_UNSUPPORTED_COMMAND = 20,
        STATUS_MODAL_GROUP_VIOLATION = 21,
        STATUS_UNDEFINED_FEED_RATE = 22,
        STATUS_COMMAND_VALUE_NOT_INTEGER = 23,
        STATUS_AXIS_COMMAND_CONFLICT = 24,
        STATUS_WORD_REPEATED = 25,
        STATUS_NO_AXIS_WORDS = 26,
        STATUS_INVALID_LINE_NUMBER = 27,
        STATUS_VALUE_WORD_MISSING = 28,
        STATUS_AXIS_WORDS_EXIST = 31,
        STATUS_NO_AXIS_WORDS_IN_PLANE = 32,
        STATUS_INVALID_TARGET = 33,
        STATUS_ARC_RADIUS_ERROR = 34,
        STATUS_NO_OFFSETS_IN_PLANE = 35,
        STATUS_UNUSED_WORDS = 36,
        STATUS_MAX_VALUE_EXCEEDED = 38,
};


#define MESSAGE_CRITICAL_EVENT "Reset to continue"
#define MESSAGE_ALARM_LOCK "'$H'|'$X' to unlock"
#define MESSAGE_ALARM_UNLOCK "Caution: Unlocked"
#define MESSAGE_ENABLED "Enabled"
#define MESSAGE_DISABLED "Disabled"
#define MESSAGE_SAFETY_DOOR_AJAR "Check Door"
#define MESSAGE_CHECK_LIMITS "Check Limits"
#define MESSAGE_PROGRAM_END "Pgm End"
#define MESSAGE_SLEEP_MODE "Sleeping"

void message_alarm(ssize_t alarm);

void message_feedback(char *feeback);

void message_status(ssize_t status);

void message_write(ssize_t msg, ...);

float steps_to_float(int32_t steps, uint8_t idx);

#endif //OPENGLOW_CNC_MESSAGES_H

/** @} */
