/**
 * @file socket.h
 * @brief IP cocket interface
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup cli_socket
 *
 * @{
 */

#ifndef OPENGLOW_CNC_SOCKET_H
#define OPENGLOW_CNC_SOCKET_H

#include <sys/socket.h>
#include <netinet/in.h>

#include "../common.h"

/**
 * @brief Maximum amount of output to store when socket is not connected.
 */
#define TX_RING_BUFFER 1024

ssize_t socket_init();

void socket_reset();

ssize_t socket_write(char *line, va_list args);

#endif //OPENGLOW_CNC_SOCKET_H

/** @} */
