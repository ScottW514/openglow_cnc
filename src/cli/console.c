/**
 * @file console.c
 * @brief Console Interface
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup cli
 * @{
 * @defgroup cli_console Console Functions
 *
 * Functions for providing the console transport to the CLI.
 *
 * @{
 */

#include <alchemy/task.h>
#include "../common.h"
#include "../openglow-cnc.h"

/**
 * @brief Console read real time task
 */
static RT_TASK console_read_task;

// Static function declarations
static void _console_read();

/**
 * @brief Initializes console interface
 *
 * Launches the console read task.
 *
 * @return Zero is returned upon success, negative otherwise.
 */
ssize_t console_init() {
    ssize_t ret = 0;
    // Launch Console Read Task
    if ((ret = rt_task_spawn(&console_read_task, "console_read_task", 0, 30, 0, &_console_read, 0)) < 0) {
        fprintf(stderr, "console_init: rt_task_spawn for console_read_task returned %zd\n", ret);
        return ret;
    }
    return ret;
}

/**
 * @brief Tears down the console
 */
void console_reset() {
    rt_task_delete(&console_read_task);
}

/**
 * @brief Write to console
 *
 * Writes message to console interface
 *
 * @param line The string to write. Uses printf formatting.
 * @param args Variable Arguments to submit to vsprintf
 * @return The number of characters printed. Negative otherwise.
 */
ssize_t console_write(char *line, va_list args) {
    char buf[CLI_LINE_LENGTH];
    vsprintf(buf, line, args);
    return printf("%s\n", buf);
}

/**
 * @brief Console read task
 *
 * Reads line from console and submits it to input_process_line().
 *
 * @note Runs as Xenomai Alchemy Task with a priority of 30. Blocked while waiting for line.
 */
static void _console_read() {
    char *line = NULL;
    size_t len = 0;
    while (getline(&line, &len, stdin)) {
        cli_process_line(line);
    }
}
/** @} */
/** @} */
