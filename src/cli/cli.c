/**
 * @file cli.c
 * @brief Command processor initialization
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @defgroup cli Command Line Interface
 *
 * Command Line Interfaces
 *
 * Interfaces and interpreter for users and 3rd party applications to control the OpenGlow.
 *
 * @{
 *
 * @defgroup cli_cli Initialization and Processing
 *
 * Initialization of the CLI transport and processing of the input.
 *
 * @{
 */

#include <string.h>
#include "../openglow-cnc.h"

/**
 * @brief Valid states for the CLI Finite State Machine
 */
enum cli_fsm_states {
    CLI_STATE_INIT,                 /*!< Initializing */
    CLI_STATE_OPERATIONAL,          /*!< Operational */
    CLI_STATE_UNINITIALIZED = 255   /*!< Uninitialized */
};

/**
 * @brief System FSM to CLI FSM Mapping
 *
 * Describes which CLI FSM states are valid for System FSM states.
 */
static const sys_to_sub_map_t cli_sys_sub[] = {
        {SYS_STATE_INIT,   CLI_STATE_INIT},
        {SYS_STATE_SLEEP,  CLI_STATE_OPERATIONAL},
        {SYS_STATE_IDLE,   CLI_STATE_OPERATIONAL},
        {SYS_STATE_HOMING, CLI_STATE_OPERATIONAL},
        {SYS_STATE_RUN,    CLI_STATE_OPERATIONAL},
        {SYS_STATE_HOLD,   CLI_STATE_OPERATIONAL},
};

/**
 * @brief FSM State Map registration structure
 *
 * Used to register the System FSM to CLI FSM Mapping with the System FSM
 */
static const sub_state_map_t cli_fsm_state_map = {
        .num = sizeof(cli_sys_sub) / sizeof(sys_to_sub_map_t),  /*!< Size of the provided map array */
        .maps = &cli_sys_sub[0],    /*!< System FSM to CLI FSM map array */
        .fsm_handler = NULL,        /*!< Local handler function for System FSM notifications */
};

/**
 * @brief CLI FSM State
 *
 * Current state of the CLI Finite State Machine
 */
static volatile enum cli_fsm_states cli_fsm_state = CLI_STATE_UNINITIALIZED;

/**
 * @brief User command type
 *
 * Contains the text of the available user commands, and a flag to indicate if they have arguments.
 */
typedef struct command {
    char *string;   /*!< User Command String */
    bool args;      /*!< Accepts arguments */
} command_t;

/**
 * @brief User command table
 *
 * Table of user commands, the associated text, and if they accept arguments.
 */
static const command_t commands[NUMBER_OF_USER_COMMANDS] = {
        [USR_CYCLE_START]           = {"~", false},
        [USR_CHECK_GCODE_MODE]      = {"$C", false},
        [USR_FEED_HOLD]             = {"!", false},
        [USR_HELP]                  = {"$", false},
        [USR_RESET]                 = {"X", false},
        [USR_RUN_HOMING_CYCLE]      = {"$H", false},
        [USR_SLEEP]                 = {"$SLP", false},
        [USR_STATUS_REPORT]         = {"?", false},
        [USR_TEST_CYCLE]            = {"$T", false},
};

/**
 * @brief Test Cycle Commands
 */
static const char *test_commands[] = {
        "G0 X495.300 Y000.000",
        "G0 X495.300 Y279.400",
        "G0 X000.000 Y279.400",
        "G0 X200.000 Y135.000",
        "G2 X200.000 Y135.000 I050.000 J000.000 F3000",
        "G0 X000.000 Y279.400",
        "G0 X495.300 Y279.400",
        "G0 X000.000 Y000.000",
        ";END"
};

/**
 * @brief Initialize CLI
 *
 * Called by the main control process.  This launches a new thread that handles input from cli
 * interpreter via console or IP socket.  Depending on the input, it either handles the cli from beginning until
 * resulting output, or sends the cli off to another process for handling.
 *
 * @return Zero on success. Negative otherwise.
 */
ssize_t cli_init() {
    ssize_t ret = 0;
    switch (settings.cli.comm_mode) {
        case CLI_CONSOLE: {
            ret = console_init();
            break;
        }
        case CLI_SOCKET: {
            ret = socket_init();
            break;
        }
        default:;
    }
    if (ret >= 0) {
        cli_fsm_state = CLI_STATE_INIT;
        fsm_register(FSM_CLI, cli_fsm_state_map);
        cli_fsm_state = CLI_STATE_OPERATIONAL;
        fsm_update(FSM_CLI, cli_fsm_state);
    }
    return ret;
}

/**
 * @brief Process a line of input from the cli interface
 *
 * Called by console or socket read loop. Executes a command or forwards it to the g-code parser.
 *
 * @param line The line recieved from the cli to parse.
 *
 * @return None
 */
void cli_process_line(char *line) {
    if ((line[0] == '\n') | (line[0] == '\r')) {
        message_write(MSG_OK);
        return;
    }
    strtok(line, "\r");
    strtok(line, "\n");
    // Check for commands
    for (int i = 0; i < NUMBER_OF_USER_COMMANDS; i++) {
        if (((!commands[i].args & (strlen(commands[i].string) == strlen(line))) | commands[i].args) &
            (strncmp(commands[i].string, line, strlen(commands[i].string)) == 0)) {
            switch (i) {
                case USR_CHECK_GCODE_MODE: {
                    message_status(STATUS_UNSUPPORTED_COMMAND);
                    return;
                }
                case USR_CYCLE_START: {
                    if (sys_state == SYS_STATE_IDLE || sys_state == SYS_STATE_HOLD) {
                        fsm_request(SYS_STATE_RUN);
                        stepgen_wake_up();
                    }
                    return;
                }
                case USR_FEED_HOLD: {
                    message_status(STATUS_UNSUPPORTED_COMMAND);
                    return;
                }
                case USR_HELP: {
                    message_write(MSG_HELP);
                    return;
                }
                case USR_RESET: {
                    message_status(STATUS_UNSUPPORTED_COMMAND);
                    return;
                }
                case USR_RUN_HOMING_CYCLE: {
                    message_status(STATUS_UNSUPPORTED_COMMAND);
                    return;
                }
                case USR_SLEEP: {
                    message_status(STATUS_UNSUPPORTED_COMMAND);
                    return;
                }
                case USR_STATUS_REPORT: {
                    message_write(MSG_STATUS_REPORT, states[sys_state].text,
                                  steps_to_float(sys_position[X_AXIS], X_AXIS),
                                  steps_to_float(sys_position[Y_AXIS], Y_AXIS),
                                  steps_to_float(sys_position[Z_AXIS], Z_AXIS));
                    return;
                }
                case USR_TEST_CYCLE: {
                    if (sys_state == SYS_STATE_IDLE && sys_req_state == FSM_STATE_NO_REQ) {
                        message_feedback("Queuing Test Code");
                        test_run = true;
                        uint8_t g = 0;
                        char buf[CLI_LINE_LENGTH];
                        char gline[CLI_LINE_LENGTH];
                        while (strcmp(test_commands[g], ";END") != 0) {
                            memset(buf, 0, sizeof(buf));
                            memset(gline, 0, sizeof(gline));
                            strcpy(gline, test_commands[g]);
                            message_write(MSG_PLAIN_TEXT, gline);
                            gc_process_line(gline, buf);
                            gc_queue_line(buf);
                            g++;
                        }
                        message_feedback("Test Queued. '~' to cycle.");
                    } else {
                        message_status(STATUS_IDLE_ERROR);
                    }
                    return;
                }
                default: {
                }
            }
        }
    }
    // Not a command - continue processing as G-Code
    ssize_t ret = 0;
    char buf[CLI_LINE_LENGTH];
    memset(buf, 0, sizeof(buf));
    gc_process_line(line, buf);
    if ((ret = gc_queue_line(buf)) < 0) {
        fprintf(stderr, "cli_process_line: gc_queue_line returned %zd\n", ret);
    }
}

/**
 * @brief Reset CLI
 *
 * Kills read threads and clears the current state.
 *
 * @return Zero on success. Negative otherwise.
 */
void cli_reset() {
    switch (settings.cli.comm_mode) {
        case CLI_CONSOLE: {
            console_reset();
            break;
        }
        case CLI_SOCKET: {
            socket_reset();
            break;
        }
        default:;
    }
    cli_fsm_state = CLI_STATE_UNINITIALIZED;
}

/** @} */
/** @} */
