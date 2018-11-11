/**
 * @file cli.h
 * @brief Command processor initialization
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup cli_cli
 *
 * @{
 */

#ifndef OPENGLOW_CNC_CLI_H
#define OPENGLOW_CNC_CLI_H

#include "../common.h"

/**
 * @brief Maximum line length for CLI inputs/outputs.
 *
 * Buffer sizes to use for CLI IO.
 */
#define CLI_LINE_LENGTH    512 // Maximum length of a line sent/received by cli.

/**
 * @brief CLI Transport Mode
 *
 * Indicate if we are using the console a TCP socket for the CLI.
 */
enum CLI_TRANSPORT {
    CLI_SOCKET,     /*!< Use TCP Socket for CLI */
    CLI_CONSOLE     /*!< Use STDIN/OUT Console for CLI */
};

/**
 * @brief User CLI Commands
 *
 * Commands availalbe to the user via the CLI.
 * @note We break with Grbl compatibility by requiring these to be followed with a line break.
 * We do not pick them off of the incoming stream.
 * Any Grbl type client will need to be patched to support this.
 */
enum USER_COMMANDS {
    USR_CHECK_GCODE_MODE,   /*!< Validate G-Code only. Do not execute motion. */
    USR_CYCLE_START,        /*!< Execute currently queued G-Code, or resume from hold. */
    USR_FEED_HOLD,          /*!< Halt current motion, in a resumable manner. */
    USR_HELP,               /*!< Show help information. */
    USR_RESET,              /*!< Reset all system states. Will require re-homing. */
    USR_RUN_HOMING_CYCLE,   /*!< Execute Homing Cycle */
    USR_SLEEP,              /*!< Enter low power mode. Will require re-homing. */
    USR_STATUS_REPORT,      /*!< Print status report. */
    USR_TEST_CYCLE,         /*!< Runs test cycle. */
    NUMBER_OF_USER_COMMANDS,/*!< Number of User Commands - For internal use. */
};

ssize_t cli_init();

void cli_process_line(char *line);

void cli_reset();

#endif // OPENGLOW_CNC_CLI_H

/** @} */
