/**
 * @file gcode.h
 * @brief rs274/ngc parser
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * Adapted from Grbl
 * @copyright Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
 * @copyright Copyright (c) 2009-2011 Simen Svale Skogsrud
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup motion_gc
 *
 * @{
 */

#ifndef OPENGLOW_CNC_GCODE_H
#define OPENGLOW_CNC_GCODE_H

#include "../common.h"
#include "../config.h"

#define GCODE_QUEUE_SIZE 16

#define LINE_FLAG_COMMENT_PARENTHESES   (1 << 0)
#define LINE_FLAG_COMMENT_SEMICOLON     (1 << 1)

// Modal Group G0: Non-modal actions
#define NON_MODAL_DWELL 4 // G4 (Do not alter value)

// Modal Group G1: Motion modes
#define MOTION_MODE_SEEK 0 // G0 (Default: Must be zero)
#define MOTION_MODE_LINEAR 1 // G1 (Do not alter value)
#define MOTION_MODE_CW_ARC 2  // G2 (Do not alter value)
#define MOTION_MODE_CCW_ARC 3  // G3 (Do not alter value)
#define MOTION_MODE_NONE 80 // G80 (Do not alter value)

// Modal Group G2: Plane select
#define PLANE_SELECT_XY 0 // G17 (Default: Must be zero)
#define PLANE_SELECT_ZX 1 // G18 (Do not alter value)

// Modal Group G3: Distance mode
#define DISTANCE_MODE_ABSOLUTE 0 // G90 (Default: Must be zero)

// Modal Group M4: Program flow
#define PROGRAM_FLOW_RUNNING 0 // (Default: Must be zero)
#define PROGRAM_FLOW_PAUSED 3 // M0

// Modal Group G5: Feed rate mode
#define FEED_RATE_MODE_UNITS_PER_MIN  0 // G94 (Default: Must be zero)
#define FEED_RATE_MODE_INVERSE_TIME   1 // G93 (Do not alter value)

// Modal Group G6: Units mode
#define UNITS_MODE_MM 0 // G21 (Default: Must be zero)
#define UNITS_MODE_INCHES 1 // G20 (Do not alter value)

// Modal Group M7: Spindle control
#define LASER_DISABLE 0 // M5 (Default: Must be zero)
#define SPINDLE_ENABLE_CW   PL_COND_FLAG_SPINDLE_CW // M3 (NOTE: Uses planner condition bit flag)
#define LASER_ENABLE  PL_COND_FLAG_SPINDLE_CCW // M4 (NOTE: Uses planner condition bit flag)

// Modal Group M8: Coolant control
#define COOLANT_DISABLE 0 // M9 (Default: Must be zero)
#define COOLANT_FLOOD_ENABLE  PL_COND_FLAG_COOLANT_FLOOD // M8 (NOTE: Uses planner condition bit flag)
#define COOLANT_MIST_ENABLE   PL_COND_FLAG_COOLANT_MIST  // M7 (NOTE: Uses planner condition bit flag)

// Define g-code parser position updating flags
#define GC_UPDATE_POS_TARGET   0 // Must be zero
#define GC_UPDATE_POS_SYSTEM   1


// Define gcode parser flags for handling special cases.
#define GC_PARSER_NONE                  0 // Must be zero.
#define GC_PARSER_CHECK_MANTISSA        bit(1)
#define GC_PARSER_ARC_IS_CLOCKWISE      bit(2)
#define GC_PARSER_LASER_FORCE_SYNC      bit(5)
#define GC_PARSER_LASER_DISABLE         bit(6)
#define GC_PARSER_LASER_ISMOTION        bit(7)

ssize_t gc_init();

void gc_process_line(char *line, char *buf);

ssize_t gc_queue_line(char *line);

void gc_sync_position();

#endif //OPENGLOW_CNC_GCODE_H

/** @} */
