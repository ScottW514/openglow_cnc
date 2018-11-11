/**
 * @file openglow-cnc.h
 * @brief Shared headers
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

#ifndef OPENGLOW_CNC_OPENGLOW_CNC_H
#define OPENGLOW_CNC_OPENGLOW_CNC_H

#include "config.h"
#include "cli/console.h"
#include "cli/socket.h"
#include "cli/cli.h"
#include "cli/messages.h"
#include "hardware/hardware.h"
#include "hardware/laser.h"
#include "hardware/limits.h"
#include "hardware/openglow.h"
#include "hardware/step_drv.h"
#include "hardware/switches.h"
#include "hardware/stepgen.h"
#include "motion/gcode.h"
#include "motion/motion.h"
#include "motion/motion_control.h"
#include "motion/planner.h"
#include "motion/segment.h"
#include "system/system.h"
#include "system/settings.h"
#include "system/fsm.h"

#define OPENGLOW_CNC_VER    "DEV"

#endif //OPENGLOW_CNC_OPENGLOW_CNC_H

/** @} */
