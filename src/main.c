/**
 * @file main.c
 * @brief Main Program File
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @defgroup main Main Program
 *
 * Main Program
 *
 * @{
 */

#include <argp.h>
#include <arpa/inet.h>
#include <signal.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>

#include "common.h"
#include "openglow-cnc.h"

// Arparse handling
static struct argp_option options[] = {
        {"verbose",     'v', 0,        0, "Produce verbose output"},
        {"daemon",      'd', 0,        0, "Run as daemon"},
        {"socket",      's', 0,        0, "Listen on socket (default console)"},
        {"listen-port", 'p', "IPADDR", 0, "IP Address to listen on"},
        {"listen-ip",   'i', "PORT",   0, "IP Port to listen on"},
        {0}
};

typedef struct arguments {
    uint8_t daemon, socket, verbose;
    char *listen_ip, *listen_port;
} arguments_t;

static error_t

parse_opt(int key, char *arg, struct argp_state *state) {
    struct arguments *arguments = state->input;
    switch (key) {
        case 'q':
        case 'd': {
            arguments->daemon = 1;
            break;
        }
        case 's': {
            arguments->socket = 1;
            break;
        }
        case 'v': {
            arguments->verbose = 1;
            break;
        }
        case 'i': {
            arguments->listen_ip = arg;
            break;
        }
        case 'p': {
            arguments->listen_port = arg;
            break;
        }
        default:
            return ARGP_ERR_UNKNOWN;
    }
    return 0;
}

static struct argp argp = {options, parse_opt, "", ""};

void sig_handler(int sig);
void graceful_shutdown(void);

int main(int argc, char *argv[]) {
    // Set our signal handler so we can exit gracefully
    signal(SIGTERM, sig_handler);
    signal(SIGINT, sig_handler);

    // Lock the memory to avoid memory swapping for this program
    mlockall(MCL_CURRENT | MCL_FUTURE);

    // Set this global var to while loops to run (workaround for annoying CLion behavior)
    loop_run = true;

    // Clear test run flag
    test_run = false;

    /* Default values. */
    arguments_t arguments =
            {
                    .verbose = 0,
                    .daemon = 0,
                    .socket = 0,
                    .listen_ip = COMM_LISTEN_ADDR,
                    .listen_port = COMM_LISTEN_PORT,
            };

    /* Parse arguments */
    argp_parse(&argp, argc, argv, 0, 0, &arguments);
    verbose = arguments.verbose;
    settings.daemon = arguments.daemon;
    settings.cli.listen_port = (uint16_t) strtol(arguments.listen_port, NULL, 10);
    struct in_addr listen_ip = settings.cli.listen_ip;
    if (inet_pton(AF_INET, arguments.listen_ip, &listen_ip) == 0) {
        perror("Invalid IP address");
        exit(-1);
    }
    settings.cli.comm_mode = (uint8_t) ((arguments.socket) ? CLI_SOCKET : CLI_CONSOLE);

    // Turn over control to the loop
    ssize_t ret = 0;
    ret = system_control_init();
    graceful_shutdown();
    exit((int) ret);
}

// Gracefully Shutdown System
void graceful_shutdown(void) {
    cli_reset();
    hardware_reset();
    motion_reset();
    fsm_reset();
}

// Signal handler
void sig_handler(int sig) {
    if ((sig == SIGTERM) | (sig == SIGINT)) {
        fprintf(stderr, "Caught SIGTERM/SIGINT: Gracefully exiting...\n");
        graceful_shutdown();
        exit(0);
    }
}

/** @} */
