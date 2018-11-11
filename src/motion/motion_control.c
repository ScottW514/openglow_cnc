/**
 * @file motion_control.c
 * @brief High level interface for issuing motion commands
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
 * @addtogroup motion
 *
 * @{
 * @defgroup motion_control Motion Control
 *
 * High level interface for issuing motion commands
 *
 * @{
 */

#include <alchemy/task.h>
#include <math.h>
#include "../openglow-cnc.h"
#include "grbl_glue.h"

/**
 * @brief Execute an arc in offset mode format
 *
 * The arc is approximated by generating a huge number of tiny, linear segments. The chordal tolerance
 * of each segment is configured in config.h ARC_TOLERANCE, which is defined to be the maximum normal
 * distance from segment to the circle when the end points both lie on the circle.
 * @param target target xyz
 * @param pl_data Planner block data
 * @param position current xyz
 * @param offset offset from current xyz
 * @param radius circle radius
 * @param axis_0 axis_X defines circle plane in tool space
 * @param axis_1 axis_X defines circle plane in tool space
 * @param axis_linear axis_linear is for vector transformation direction.
 * @param is_clockwise_arc Vector transformation direction
 */
void mc_arc(float *target, plan_line_data_t *pl_data, float *position, const float *offset, float radius,
            uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear, uint8_t is_clockwise_arc) {
    float center_axis0 = position[axis_0] + offset[axis_0];
    float center_axis1 = position[axis_1] + offset[axis_1];
    float r_axis0 = -offset[axis_0];  // Radius vector from center to current location
    float r_axis1 = -offset[axis_1];
    float rt_axis0 = target[axis_0] - center_axis0;
    float rt_axis1 = target[axis_1] - center_axis1;

    // CCW angle between position and target from circle center. Only one atan2() trig computation required.
    float angular_travel = atan2f(r_axis0 * rt_axis1 - r_axis1 * rt_axis0, r_axis0 * rt_axis0 + r_axis1 * rt_axis1);
    if (is_clockwise_arc) { // Correct atan2 output per direction
        if (angular_travel >= -ARC_ANGULAR_TRAVEL_EPSILON) { angular_travel -= 2 * M_PI; }
    } else {
        if (angular_travel <= ARC_ANGULAR_TRAVEL_EPSILON) { angular_travel += 2 * M_PI; }
    }

    /* NOTE: Segment end points are on the arc, which can lead to the arc diameter being smaller by up to
       (2x) config.h ARC_TOLERANCE. For 99% of users, this is just fine. If a different arc segment fit
       is desired, i.e. least-squares, midpoint on arc, just change the mm_per_arc_segment calculation.
       For the intended uses of Grbl, this value shouldn't exceed 2000 for the strictest of cases. */
    uint16_t segments = (uint16_t) floor(fabs(0.5 * angular_travel * radius) /
                                         sqrt(ARC_TOLERANCE * (2 * radius - ARC_TOLERANCE)));

    if (segments) {
        /* Multiply inverse feed_rate to compensate for the fact that this movement is approximated
           by a number of discrete segments. The inverse feed_rate should be correct for the sum of
           all segments. */
        if (pl_data->condition & PL_COND_FLAG_INVERSE_TIME) {
            pl_data->feed_rate *= segments;
            bit_false(pl_data->condition, PL_COND_FLAG_INVERSE_TIME); // Force as feed absolute mode over arc segments.
        }

        float theta_per_segment = angular_travel / segments;
        float linear_per_segment = (target[axis_linear] - position[axis_linear]) / segments;

        /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
           and phi is the angle of rotation. Solution approach by Jens Geisler.
               r_T = [cos(phi) -sin(phi);
                      sin(phi)  cos(phi] * r ;

           For arc generation, the center of the circle is the axis of rotation and the radius vector is
           defined from the circle center to the initial position. Each line segment is formed by successive
           vector rotations. Single precision values can accumulate error greater than tool precision in rare
           cases. So, exact arc path correction is implemented. This approach avoids the problem of too many very
           expensive trig operations [sin(),cos(),tan()] which can take 100-200 usec each to compute.

           Small angle approximation may be used to reduce computation overhead further. A third-order approximation
           (second order sin() has too much error) holds for most, if not, all CNC applications. Note that this
           approximation will begin to accumulate a numerical drift error when theta_per_segment is greater than
           ~0.25 rad(14 deg) AND the approximation is successively used without correction several dozen times. This
           scenario is extremely unlikely, since segment lengths and theta_per_segment are automatically generated
           and scaled by the arc tolerance setting. Only a very large arc tolerance setting, unrealistic for CNC
           applications, would cause this numerical drift error. However, it is best to set N_ARC_CORRECTION from a
           low of ~4 to a high of ~20 or so to avoid trig operations while keeping arc generation accurate.

           This approximation also allows mc_arc to immediately insert a line segment into the motion
           without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
           a correction, the motion should have caught up to the lag caused by the initial mc_arc overhead.
           This is important when there are successive arc motions.
        */
        // Computes: cos_T = 1 - theta_per_segment^2/2, sin_T = theta_per_segment - theta_per_segment^3/6) in ~52usec
        float cos_T = (float) 2.0 - theta_per_segment * theta_per_segment;
        float sin_T = theta_per_segment * (float) 0.16666667 * (cos_T + (float) 4.0);
        cos_T *= 0.5;

        float sin_Ti;
        float cos_Ti;
        float r_axisi;
        uint16_t i;
        uint8_t count = 0;

        for (i = 1; i < segments; i++) { // Increment (segments-1).

            if (count < N_ARC_CORRECTION) {
                // Apply vector rotation matrix. ~40 usec
                r_axisi = r_axis0 * sin_T + r_axis1 * cos_T;
                r_axis0 = r_axis0 * cos_T - r_axis1 * sin_T;
                r_axis1 = r_axisi;
                count++;
            } else {
                // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments. ~375 usec
                // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
                cos_Ti = cosf(i * theta_per_segment);
                sin_Ti = sinf(i * theta_per_segment);
                r_axis0 = -offset[axis_0] * cos_Ti + offset[axis_1] * sin_Ti;
                r_axis1 = -offset[axis_0] * sin_Ti - offset[axis_1] * cos_Ti;
                count = 0;
            }

            // Update arc_target location
            position[axis_0] = center_axis0 + r_axis0;
            position[axis_1] = center_axis1 + r_axis1;
            position[axis_linear] += linear_per_segment;

            mc_line(position, pl_data);

            // Bail mid-circle on system abort. Runtime cli check already performed by mc_line.
            if ((sys_state == SYS_STATE_FAULT) || (sys_state == SYS_STATE_ALARM)) { return; }
        }
    }
    // Ensure last segment arrives at target location.
    mc_line(target, pl_data);
}

/**
 * @brief Execute dwell in seconds.
 * @param seconds Dwell time in seconds
 */
void mc_dwell(float seconds) {
    if (verbose) printf("mc_dwell: init\n");
//    if (sys.state.mode & STATE_G_CODE_CHECK) { return; }
//    protocol_buffer_synchronize();
    delay_sec(seconds);
}

/**
 * @brief Execute linear motion in absolute millimeter coordinates.
 *
 * Feed rate given in millimeters/second unless invert_feed_rate is true. Then the feed_rate
 * means that the motion should be completed in (1 minute)/feed_rate time.
 * @note This is the primary gateway to the grbl motion. All line motions, including arc line
 * segments, must pass through this routine before being passed to the motion. The seperation of
 * mc_line and plan_buffer_line is done primarily to place non-motion-type functions from being
 * in the motion and to let backlash compensation or canned cycle integration simple and direct.
 * @param target target xyz
 * @param pl_data Planner block data
 */
void mc_line(float *target, plan_line_data_t *pl_data) {
    // If enabled, check for soft limit violations. Placed here all line motions are picked up
    // from everywhere in Grbl.
//    if (settings.soft_limits) {
//        // NOTE: Block jog state. Jogging is a special case and soft limits are handled independently.
//        limits_soft_check(target);
//    }

    // If in check gcode mode, prevent motion by blocking motion. Soft limits still work.
//    if (sys.state.mode & STATE_G_CODE_CHECK) { return; }

    // If the buffer is full: good! That means we are well ahead of the robot.
    // Remain in this loop until there is room in the buffer.
    do {
        if ((sys_state == SYS_STATE_FAULT) || (sys_state == SYS_STATE_ALARM)) { return; } // Bail, if system abort.
        if (plan_check_full_buffer()) {
            // Auto-cycle start when buffer is full, and we're not already in STATE_RUN
            if (settings.cli.auto_cycle && (sys_state != SYS_STATE_RUN)) fsm_request(SYS_STATE_RUN);
            // TODO: Find a better way than this to wait for room in the buffer - i.e. thread message
            rt_task_sleep(100000000); // Sleep for .1 seconds to reduce CPU overhead
        } else { break; }
    } while (1);

    // Plan and queue motion into motion buffer
    if (!plan_buffer_line(target, pl_data)) { // Empty block returned
        if (settings.laser_power_correction) {
            // Correctly set spindle state, if there is a coincident position passed. Forces a buffer
            // sync while in M3 laser mode only.
            if (pl_data->condition & PL_COND_FLAG_SPINDLE_CW) {
//                laser_sync(PL_COND_FLAG_SPINDLE_CW, pl_data->spindle_speed);
            }
        }
    }
}

/** @} */
/** @} */
