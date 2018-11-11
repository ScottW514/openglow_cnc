/**
 * @file socket.c
 * @brief IP cocket interface
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup cli
 * @{
 * @defgroup cli_socket Socket Functions
 *
 * Functions for providing the socket transport to the CLI.
 *
 * @{
 */

#include "../openglow-cnc.h"

#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>

/**
 * @brief Server socket descriptor
 */
static int s_sock = 0;

/**
 * @brief Client socket descriptor
 */
static int c_sock = 0;

/**
 * @brief Socket write mutex
 */
static sem_t write_mutex;

/**
 * @brief Transmit ring buffer
 */
int comm_tx_buffer[TX_RING_BUFFER];

/**
 * @brief Transmit ring buffer head
 */
volatile int comm_tx_buffer_head = 0;

/**
 * @brief Transmit ring buffer tail
 */
volatile int comm_tx_buffer_tail = 0;

// Static function declarations
static ssize_t _socket_flush_buffer();
static void *_socket_read();

/**
 * @brief Flush the transmit ring buffer to the socket
 * @return Bytes written on success, negative on failure
 */
static ssize_t _socket_flush_buffer() {
    ssize_t ret = 0;
    if (verbose) printf("socket_flush_buffer: init\n");
    sem_wait(&write_mutex);
    if (verbose) printf("socket_flush_buffer: write_mutex acquired\n");
    int i = 0;
    char buf[TX_RING_BUFFER + 1];
    memset(buf, 0, sizeof(buf));

    while (comm_tx_buffer_head != comm_tx_buffer_tail) {
        buf[i] = (char) comm_tx_buffer[comm_tx_buffer_tail];
        i++;
        comm_tx_buffer_tail++;
        if (comm_tx_buffer_tail == TX_RING_BUFFER) comm_tx_buffer_tail = 0;
    }
    ret = write(c_sock, buf, strlen(buf));
    sem_post(&write_mutex);
    if (verbose) printf("socket_flush_buffer: write_mutex released\n");
    return ret;
}

/**
 * @brief Initialize the socket
 * @return 0 on success, negative on failure.
 */
ssize_t socket_init() {
    if (verbose) printf("socket_init: init\n");
    sem_init(&write_mutex, 0, 1);

    /* Create the socket. */
    s_sock = socket(PF_INET, SOCK_STREAM, 0);
    if (s_sock < 0) {
        perror("telnet init: socket creation error");
        return -1;
    }

    struct sockaddr_in sock_in;
    sock_in.sin_family = AF_INET;
    sock_in.sin_port = htons(settings.cli.listen_port);
    sock_in.sin_addr.s_addr = settings.cli.listen_ip.s_addr;
    if (bind(s_sock, (struct sockaddr *) &sock_in, sizeof(sock_in)) < 0) {
        perror("socket_init: socket bind error");
        return -1;
    }
    listen(s_sock, 1);
    pthread_t t;
    return pthread_create(&t, NULL, _socket_read, NULL);
}

/**
 * @brief Read user input from socket
 * Calls cli_process_line with received line
 * @return NULL
 */
static void *_socket_read() {
    if (verbose) printf("socket_read: init\n");
    char line[CLI_LINE_LENGTH];
    struct sockaddr_in client_addr;
    int c = sizeof(struct sockaddr_in);

    // Connection Loop
    // waits if no connection
    while ((c_sock = accept(s_sock, (struct sockaddr *) &client_addr, (socklen_t *) &c))) {
        // New connection. We'll dump the buffer, just in case there are any outgoing messages waiting.
        _socket_flush_buffer();
        // RX Loop
        while (recv(c_sock, line, CLI_LINE_LENGTH, 0) > 0) {
            // Process each character received from client
            cli_process_line(line);
            memset(line, 0, sizeof(line));
        }
        c_sock = 0;
    }
    return NULL;
}

/**
 * @brief Closes socket
 */
void socket_reset() {
    close(s_sock);
}

/**
 * @brief Write string to socket
 * @param line String to write
 * @param args vprintf va_list
 * @return Bytes written to socket on success, negative on error.
 */
ssize_t socket_write(char *line, va_list args) {
    if (verbose) printf("socket_write: init\n");
    sem_wait(&write_mutex);
    if (verbose) printf("socket_write: write_mutex acquired\n");
    char buf[CLI_LINE_LENGTH];
    vsprintf(buf, line, args);
    sprintf(buf, "%s\n\r", line);
    buf[strlen(line) + 2] = 0; // Make sure the buffer is zero padded
    if (c_sock > 0) {
        return write(c_sock, buf, strlen(buf));
    } else {
        // We don't have an active connection, buffer the output until we do.
        for (int i = 0; i < strlen(buf); i++) {
            comm_tx_buffer[comm_tx_buffer_head] = line[i];
            comm_tx_buffer_head++;
            if (comm_tx_buffer_head == TX_RING_BUFFER) { comm_tx_buffer_head = 0; }
        }
    }
    sem_post(&write_mutex);
    if (verbose) printf("socket_write: write_mutex released\n");
    return 0;
}

/** @} */
/** @} */
