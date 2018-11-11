/**
 * @file config.h
 * @brief System Configuration Settings
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

#ifndef OPENGLOW_CNC_CONFIG_H
#define OPENGLOW_CNC_CONFIG_H

#include <arpa/inet.h>
#include "common.h"

bool verbose;

#define DEBUG_STEP_TO_FILE 1
#define TARGET_BUILD 1

#define COMM_LISTEN_PORT    "51401"
#define COMM_LISTEN_ADDR    "127.0.0.1"
#define MDI_MODE true    // Automatically execute each line of G-code as it is entered.
#define REPORT_UNITS 0 // 0 = mm, 1 = inches

#define STEP_FREQUENCY 40000

#define X_MM_PER_FULL_STEP 0.15
#define Y_MM_PER_FULL_STEP 0.15
#define Z_MM_PER_FULL_STEP 0.70612

#define X_MICROSTEPS 16
#define Y_MICROSTEPS 16
#define Z_MICROSTEPS 16

#define X_STEPS_PER_MM ((1/X_MM_PER_FULL_STEP) * X_MICROSTEPS)
#define Y_STEPS_PER_MM ((1/Y_MM_PER_FULL_STEP) * Y_MICROSTEPS)
#define Z_STEPS_PER_MM ((1/Z_MM_PER_FULL_STEP) * Z_MICROSTEPS)

#define X_MAX_RATE 5000.0 // mm/min
#define Y_MAX_RATE 5000.0 // mm/min
#define Z_MAX_RATE 50.0 // mm/min
#define MINIMUM_FEED_RATE 1.0 // (mm/min)

#define X_ACCELERATION (200.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
#define Y_ACCELERATION (200.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
#define Z_ACCELERATION (200.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
#define ACCELERATION_TICKS_PER_SECOND 1000

#define X_MAX_TRAVEL 495.3 // mm
#define Y_MAX_TRAVEL (-279.4) // mm
#define Z_MAX_TRAVEL 12.0 // mm

#define JUNCTION_DEVIATION 0.01 // mm
#define ARC_TOLERANCE 0.002 // mm

#define X_AXIS_STEP_BIT     bit(0)
#define Y_AXIS_STEP_BIT     bit(2)
#define Z_AXIS_STEP_BIT     bit(5)
#define X_AXIS_DIR_BIT      bit(1)
#define Y_AXIS_DIR_BIT      bit(3)
#define Z_AXIS_DIR_BIT      bit(6)
#define LASER_ON_BIT        bit(4)
#define LASER_PWR_BIT       bit(7)
#define LASER_PWR_MASK      0x7F

/**
 * @brief Maximum line number for G-Code
 * @note Max line number is defined by the g-code standard to be 99999. It seems to be an
 * arbitrary value, and some GUIs may require more. So we increased it based on a max safe
 * value when converting a float (7.2 digit precision)s to an integer.
 */
#define MAX_G_CODE_LINE_NUMBER 10000000

/**
 * @brief CPU affinity for Step Generator RT Task
 */
#define STEP_GEN_CPU_AFFINITY   3

/**
 * @brief Priority for Step Generator RT Task
 */
#define STEP_GEN_PRIORITY   50

#endif //OPENGLOW_CNC_CONFIG_H

/** @} */
