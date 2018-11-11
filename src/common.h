/**
 * @file common.h
 * @brief Common Includes
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup main
 *
 * @{
 */

#ifndef OPENGLOW_CNC_COMMON_H
#define OPENGLOW_CNC_COMMON_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define NBITS(x) ((((x)-1)/(sizeof(long) * 8))+1)

#define bit(n) ((uint8_t)(1 << (n)))

// val = value to encode, lsb = least significant bit, bits = number of bits
#define bits(lsb, val, bits) (((val) & ((1 << (bits)) - 1)) << (lsb))

enum axis {
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
    N_AXIS,
};

/**
 * @brief Used by event loops to wait on notifications via select()
 */
struct query_mode {
    const char *name;
    int event_type;
    int max;
    unsigned long rq;
};

/**
 * @brief Used by Limits and Switches to hold state information
 */
typedef struct input_status_s {
    bool state;
    u_int32_t bit;
    bool invert;
} input_status_t;

/**
 * Used for indeterminate while loops
 */
bool loop_run;

/**
 * Test Run indicator
 */
bool test_run;

#endif //OPENGLOW_CNC_COMMON_H

/** @} */
