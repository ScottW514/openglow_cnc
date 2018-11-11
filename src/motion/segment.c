/**
 * @file segment.c
 * @brief Calculates individual segments for motion blocks
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
 * @defgroup motion_segment Segment Calculation
 *
 * Calculates individual segments for motion blocks
 *
 * @{
 */

#include <math.h>
#include "../openglow-cnc.h"
#include "grbl_glue.h"

// Some useful constants.
#define DT_SEGMENT (1.0/(ACCELERATION_TICKS_PER_SECOND*60.0)) // min/segment
#define REQ_MM_INCREMENT_SCALAR 1.25

/**
 * @brief Define step segment ramp flags.
 */
enum RAMP {
    RAMP_ACCEL,
    RAMP_CRUISE,
    RAMP_DECEL,
    RAMP_DECEL_OVERRIDE,
};

/**
 * @brief Define step segment generator prep flags.
 */
enum PREP_FLAG {
    PREP_FLAG_RECALCULATE       = bit(0),
    PREP_FLAG_DECEL_OVERRIDE    = bit(3),
};

/**
 * @brief Define step segment generator state flags.
 */
enum STEP_CONTROL {
    STEP_CONTROL_NORMAL_OP,
    STEP_CONTROL_END_MOTION           = bit(0),
    STEP_CONTROL_EXECUTE_HOLD         = bit(1),
    STEP_CONTROL_EXECUTE_SYS_MOTION   = bit(2),
    STEP_CONTROL_UPDATE_SPINDLE_PWM   = bit(3),
};

/**
 * @brief Step segment step control state
 */
static uint8_t step_control = 0;

/**
 * @brief Motion block Bresenham data
 * Stores the motion block Bresenham algorithm execution data for the segments in the segment
 * buffer. Normally, this buffer is partially in-use, but, for the worst case scenario, it will
 * never exceed the number of accessible stepper buffer segments (SEGMENT_BUFFER_SIZE-1).
 * @note This data is copied from the prepped motion blocks so that the motion blocks may be
 * discarded when entirely consumed and completed by the segment buffer.
 */
st_block_t st_block_buffer[SEGMENT_BUFFER_SIZE - 1];

/**
 * @brief Primary stepper segment ring buffer.
 * Contains small, short line segments for the stepper algorithm to execute, which are
 * "checked-out" incrementally from the first block in the motion buffer. Once "checked-out",
 * the steps in the segments buffer cannot be modified by the motion, where the remaining
 * motion block steps still can.
 */
segment_t segment_buffer[SEGMENT_BUFFER_SIZE];



/**
 * @brief Points to the beginning of the segment buffer. First to be executed or being executed.
 */
volatile uint16_t segment_buffer_tail;

/**
 * @brief Points to the segment after the last segment in the buffer.
 * Used to indicate whether the buffer is full or empty.
 * As described for standard ring buffers, this block is always empty.
 */
volatile uint16_t segment_buffer_head;

/**
 * @brief Points to next motion buffer block after the buffer head block.
 * When equal to the buffer tail, this indicates the buffer is full.
 */
volatile uint16_t segment_next_head;

/**
 * @brief Pointer to the motion block being prepped
 */
plan_block_t *pl_block;

/**
 * @brief Pointer to the stepper block data being prepped
 */
static st_block_t *st_prep_block;

/**
 * @brief Segment preparation data
 *
 * Contains all the necessary information to compute new segments based on the current executing motion block.
 */
st_prep_t prep;

/**
 * @brief Increments the step segment buffer block data ring buffer.
 * @param block_index Index to calculate from
 * @return Next segment block
 */
static uint8_t segment_next_block_index(uint8_t block_index) {
    block_index++;
    if (block_index == (SEGMENT_BUFFER_SIZE - 1)) { return (0); }
    return (block_index);
}

/**
 * @brief Prepares step segment buffer.
 */
void segment_prep_buffer() {
    while (segment_buffer_tail != segment_next_head) { // Check if we need to fill the buffer.

        // Determine if we need to load a new motion block or if the block needs to be recomputed.
        if (pl_block == NULL) {

            // Query motion for a queued block
            if (step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) { pl_block = plan_get_system_motion_block(); }
            else { pl_block = plan_get_current_block(); }
            if (pl_block == NULL) { goto segment_prep_buffer_exit; } // No motion blocks. Exit.

            // Check if we need to only recompute the velocity profile or load a new block.
            if (prep.recalculate_flag & PREP_FLAG_RECALCULATE) {
                prep.recalculate_flag = false;
            } else {

                // Load the Bresenham stepping data for the block.
                prep.st_block_index = segment_next_block_index(prep.st_block_index);

                // Prepare and copy Bresenham algorithm segment data from the new motion block, so that
                // when the segment buffer completes the motion block, it may be discarded when the
                // segment buffer finishes the prepped block, but the stepper ISR is still executing it.
                st_prep_block = &st_block_buffer[prep.st_block_index];
                st_prep_block->direction_bits = pl_block->direction_bits;
                uint8_t idx;
                for (idx=0; idx<N_AXIS; idx++) { st_prep_block->steps[idx] = (pl_block->steps[idx] << 1); }
                st_prep_block->step_event_count = (pl_block->step_event_count << 1);

                // Initialize segment buffer data for generating the segments.
                prep.steps_remaining = pl_block->step_event_count;
                prep.step_per_mm = prep.steps_remaining / pl_block->millimeters;
                prep.req_mm_increment = (float) REQ_MM_INCREMENT_SCALAR / prep.step_per_mm;
                prep.dt_remainder = 0.0; // Reset for new segment block

                if ((step_control & STEP_CONTROL_EXECUTE_HOLD) ||
                    (prep.recalculate_flag & PREP_FLAG_DECEL_OVERRIDE)) {
                    // New block loaded mid-hold. Override motion block entry speed to enforce deceleration.
                    prep.current_speed = prep.exit_speed;
                    pl_block->entry_speed_sqr = prep.exit_speed * prep.exit_speed;
                    prep.recalculate_flag &= ~(PREP_FLAG_DECEL_OVERRIDE);
                } else {
                    prep.current_speed = sqrtf(pl_block->entry_speed_sqr);
                }

                // Setup laser mode variables. PWM rate adjusted motions will always complete a motion with the
                // spindle off.
                st_prep_block->is_pwm_rate_adjusted = false;
                if (settings.laser_power_correction) {
                    if (pl_block->condition & PL_COND_FLAG_SPINDLE_CCW) {
                        // Pre-compute inverse programmed rate to speed up PWM updating per step segment.
                        prep.inv_rate = (float) 1.0 / pl_block->programmed_rate;
                        st_prep_block->is_pwm_rate_adjusted = true;
                    }
                }
            }

            /* ---------------------------------------------------------------------------------
             Compute the velocity profile of a new motion block based on its entry and exit
             speeds, or recompute the profile of a partially-completed motion block if the
             motion has updated it. For a commanded forced-deceleration, such as from a feed
             hold, override the motion velocities and decelerate to the target exit speed.
            */
            prep.mm_complete = 0.0; // Default velocity profile complete at 0.0mm from end of block.
            float inv_2_accel = (float) 0.5 / pl_block->acceleration;
            if (step_control & STEP_CONTROL_EXECUTE_HOLD) { // [Forced Deceleration to Zero Velocity]
                // Compute velocity profile parameters for a feed hold in-progress. This profile overrides
                // the motion block profile, enforcing a deceleration to zero speed.
                prep.ramp_type = RAMP_DECEL;
                // Compute decelerate distance relative to end of block.
                float decel_dist = pl_block->millimeters - inv_2_accel * pl_block->entry_speed_sqr;
                if (decel_dist < 0.0) {
                    // Deceleration through entire motion block. End of feed hold is not in this block.
                    prep.exit_speed = sqrtf(
                            pl_block->entry_speed_sqr - 2 * pl_block->acceleration * pl_block->millimeters);
                } else {
                    prep.mm_complete = decel_dist; // End of feed hold.
                    prep.exit_speed = 0.0;
                }
            } else { // [Normal Operation]
                // Compute or recompute velocity profile parameters of the prepped motion block.
                prep.ramp_type = RAMP_ACCEL; // Initialize as acceleration ramp.
                prep.accelerate_until = pl_block->millimeters;

                float exit_speed_sqr;
                float nominal_speed;
                if (step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
                    prep.exit_speed = exit_speed_sqr = 0.0; // Enforce stop at end of system motion.
                } else {
                    exit_speed_sqr = plan_get_exec_block_exit_speed_sqr();
                    prep.exit_speed = sqrtf(exit_speed_sqr);
                }

                nominal_speed = plan_compute_profile_nominal_speed(pl_block);
                float nominal_speed_sqr = nominal_speed * nominal_speed;
                float intersect_distance =
                        (float) 0.5 *
                        (pl_block->millimeters + inv_2_accel * (pl_block->entry_speed_sqr - exit_speed_sqr));

                if (pl_block->entry_speed_sqr > nominal_speed_sqr) { // Only occurs during override reductions.
                    prep.accelerate_until =
                            pl_block->millimeters - inv_2_accel * (pl_block->entry_speed_sqr - nominal_speed_sqr);
                    if (prep.accelerate_until <= 0.0) { // Deceleration-only.
                        prep.ramp_type = RAMP_DECEL;
                        // prep.decelerate_after = pl_block->millimeters;
                        // prep.maximum_speed = prep.current_speed;

                        // Compute override block exit speed since it doesn't match the motion exit speed.
                        prep.exit_speed = sqrtf(
                                pl_block->entry_speed_sqr - 2 * pl_block->acceleration * pl_block->millimeters);
                        prep.recalculate_flag |= PREP_FLAG_DECEL_OVERRIDE; // Flag to load next block as deceleration override.

                        // TODO: Determine correct handling of parameters in deceleration-only.
                        // Can be tricky since entry speed will be current speed, as in feed holds.
                        // Also, look into near-zero speed handling issues with this.

                    } else {
                        // Decelerate to cruise or cruise-decelerate types. Guaranteed to intersect updated plan.
                        prep.decelerate_after = inv_2_accel * (nominal_speed_sqr -
                                                               exit_speed_sqr); // Should always be >= 0.0 due to motion reinit.
                        prep.maximum_speed = nominal_speed;
                        prep.ramp_type = RAMP_DECEL_OVERRIDE;
                    }
                } else if (intersect_distance > 0.0) {
                    if (intersect_distance < pl_block->millimeters) { // Either trapezoid or triangle types
                        // NOTE: For acceleration-cruise and cruise-only types, following calculation will be 0.0.
                        prep.decelerate_after = inv_2_accel * (nominal_speed_sqr - exit_speed_sqr);
                        if (prep.decelerate_after < intersect_distance) { // Trapezoid type
                            prep.maximum_speed = nominal_speed;
                            if (pl_block->entry_speed_sqr == nominal_speed_sqr) {
                                // Cruise-deceleration or cruise-only type.
                                prep.ramp_type = RAMP_CRUISE;
                            } else {
                                // Full-trapezoid or acceleration-cruise types
                                prep.accelerate_until -= inv_2_accel * (nominal_speed_sqr - pl_block->entry_speed_sqr);
                            }
                        } else { // Triangle type
                            prep.accelerate_until = intersect_distance;
                            prep.decelerate_after = intersect_distance;
                            prep.maximum_speed = sqrtf(
                                    (float) 2.0 * pl_block->acceleration * intersect_distance + exit_speed_sqr);
                        }
                    } else { // Deceleration-only type
                        prep.ramp_type = RAMP_DECEL;
                        // prep.decelerate_after = pl_block->millimeters;
                        // prep.maximum_speed = prep.current_speed;
                    }
                } else { // Acceleration-only type
                    prep.accelerate_until = 0.0;
                    // prep.decelerate_after = 0.0;
                    prep.maximum_speed = prep.exit_speed;
                }
            }
            bit_true(step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM); // Force update whenever updating block.
        }

        // Initialize new segment
        segment_t *prep_segment = &segment_buffer[segment_buffer_head];

        // Set new segment to point to the current segment data block.
        prep_segment->st_block_index = prep.st_block_index;

        /*------------------------------------------------------------------------------------
            Compute the average velocity of this new segment by determining the total distance
          traveled over the segment time DT_SEGMENT. The following code first attempts to create
          a full segment based on the current ramp conditions. If the segment time is incomplete
          when terminating at a ramp state change, the code will continue to loop through the
          progressing ramp states to fill the remaining segment execution time. However, if
          an incomplete segment terminates at the end of the velocity profile, the segment is
          considered completed despite having a truncated execution time less than DT_SEGMENT.
            The velocity profile is always assumed to progress through the ramp sequence:
          acceleration ramp, cruising state, and deceleration ramp. Each ramp's travel distance
          may range from zero to the length of the block. Velocity profiles can end either at
          the end of motion block (typical) or mid-block at the end of a forced deceleration,
          such as from a feed hold.
        */
        float dt_max = (float) DT_SEGMENT; // Maximum segment time
        float dt = 0.0; // Initialize segment time
        float time_var = dt_max; // Time worker variable
        float mm_var; // mm-Distance worker variable
        float speed_var; // Speed worker variable
        float mm_remaining = pl_block->millimeters; // New segment distance from end of block.
        float minimum_mm = mm_remaining - prep.req_mm_increment; // Guarantee at least one step.
        if (minimum_mm < 0.0) { minimum_mm = 0.0; }

        do {
            switch (prep.ramp_type) {
                case RAMP_DECEL_OVERRIDE:
                    speed_var = pl_block->acceleration * time_var;
                    if (prep.current_speed - prep.maximum_speed <= speed_var) {
                        // Cruise or cruise-deceleration types only for deceleration override.
                        mm_remaining = prep.accelerate_until;
                        time_var = (float) 2.0 * (pl_block->millimeters - mm_remaining) /
                                   (prep.current_speed + prep.maximum_speed);
                        prep.ramp_type = RAMP_CRUISE;
                        prep.current_speed = prep.maximum_speed;
                    } else { // Mid-deceleration override ramp.
                        mm_remaining -= time_var * (prep.current_speed - 0.5 * speed_var);
                        prep.current_speed -= speed_var;
                    }
                    break;
                case RAMP_ACCEL:
                    // NOTE: Acceleration ramp only computes during first do-while loop.
                    speed_var = pl_block->acceleration * time_var;
                    mm_remaining -= time_var * (prep.current_speed + 0.5 * speed_var);
                    if (mm_remaining < prep.accelerate_until) { // End of acceleration ramp.
                        // Acceleration-cruise, acceleration-deceleration ramp junction, or end of block.
                        mm_remaining = prep.accelerate_until; // NOTE: 0.0 at EOB
                        time_var = (float) 2.0 * (pl_block->millimeters - mm_remaining) /
                                   (prep.current_speed + prep.maximum_speed);
                        if (mm_remaining == prep.decelerate_after) { prep.ramp_type = RAMP_DECEL; }
                        else { prep.ramp_type = RAMP_CRUISE; }
                        prep.current_speed = prep.maximum_speed;
                    } else { // Acceleration only.
                        prep.current_speed += speed_var;
                    }
                    break;
                case RAMP_CRUISE:
                    // NOTE: mm_var used to retain the last mm_remaining for incomplete segment time_var calculations.
                    // NOTE: If maximum_speed*time_var value is too low, round-off can cause mm_var to not change. To
                    //   prevent this, simply enforce a minimum speed threshold in the motion.
                    mm_var = mm_remaining - prep.maximum_speed * time_var;
                    if (mm_var < prep.decelerate_after) { // End of cruise.
                        // Cruise-deceleration junction or end of block.
                        time_var = (mm_remaining - prep.decelerate_after) / prep.maximum_speed;
                        mm_remaining = prep.decelerate_after; // NOTE: 0.0 at EOB
                        prep.ramp_type = RAMP_DECEL;
                    } else { // Cruising only.
                        mm_remaining = mm_var;
                    }
                    break;
                default: // case RAMP_DECEL:
                    // NOTE: mm_var used as a misc worker variable to prevent errors when near zero speed.
                    speed_var = pl_block->acceleration * time_var; // Used as delta speed (mm/min)
                    if (prep.current_speed > speed_var) { // Check if at or below zero speed.
                        // Compute distance from end of segment to end of block.
                        mm_var = mm_remaining - time_var * (prep.current_speed - (float) 0.5 * speed_var); // (mm)
                        if (mm_var > prep.mm_complete) { // Typical case. In deceleration ramp.
                            mm_remaining = mm_var;
                            prep.current_speed -= speed_var;
                            break; // Segment complete. Exit switch-case statement. Continue do-while loop.
                        }
                    }
                    // Otherwise, at end of block or end of forced-deceleration.
                    time_var = (float) 2.0 * (mm_remaining - prep.mm_complete) / (prep.current_speed + prep.exit_speed);
                    mm_remaining = prep.mm_complete;
                    prep.current_speed = prep.exit_speed;
            }
            dt += time_var; // Add computed ramp time to total segment time.
            if (dt < dt_max) { time_var = dt_max - dt; } // **Incomplete** At ramp junction.
            else {
                if (mm_remaining > minimum_mm) { // Check for very slow segments with zero steps.
                    // Increase segment time to ensure at least one step in segment. Override and loop
                    // through distance calculations until minimum_mm or mm_complete.
                    dt_max += DT_SEGMENT;
                    time_var = dt_max - dt;
                } else {
                    break; // **Complete** Exit loop. Segment execution time maxed.
                }
            }
        } while (mm_remaining > prep.mm_complete); // **Complete** Exit loop. Profile complete.

        /* -----------------------------------------------------------------------------------
        Compute spindle speed PWM output for step segment
      */
        if (st_prep_block->is_pwm_rate_adjusted || (step_control & STEP_CONTROL_UPDATE_SPINDLE_PWM)) {
//            if (pl_block->condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)) {
//                float rpm = pl_block->spindle_speed;
                // NOTE: Feed and rapid overrides are independent of PWM value and do not alter laser power/rate.
//                if (st_prep_block->is_pwm_rate_adjusted) { rpm *= (prep.current_speed * prep.inv_rate); }
                // If current_speed is zero, then may need to be rpm_min*(100/MAX_SPINDLE_SPEED_OVERRIDE)
                // but this would be instantaneous only and during a motion. May not matter at all.
//                prep.current_spindle_pwm = spindle_compute_pwm_value(rpm);
//            } else {
//                sys.spindle_speed = 0.0;
//                prep.current_spindle_pwm = SPINDLE_PWM_OFF_VALUE;
//            }
            bit_false(step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM);
        }
        prep_segment->spindle_pwm = prep.current_spindle_pwm; // Reload segment PWM value

        /* -----------------------------------------------------------------------------------
           Compute segment step rate, steps to execute, and apply necessary rate corrections.
           NOTE: Steps are computed by direct scalar conversion of the millimeter distance
           remaining in the block, rather than incrementally tallying the steps executed per
           segment. This helps in removing floating point round-off issues of several additions.
        */
        float step_dist_remaining = (prep.step_per_mm * mm_remaining); // Convert mm_remaining to steps
        float n_steps_remaining = ceilf(step_dist_remaining); // Round-up current steps remaining
        float last_n_steps_remaining = ceilf(prep.steps_remaining); // Round-up last steps remaining
        prep_segment->n_step = (uint16_t) (last_n_steps_remaining -
                                           n_steps_remaining); // Compute number of steps to execute.

        // Bail if we are at the end of a feed hold and don't have a step to execute.
        if (prep_segment->n_step == 0) {
            if (step_control & STEP_CONTROL_EXECUTE_HOLD) {
                // Less than one step to decelerate to zero speed, but already very close. AMASS
                // requires full steps to execute. So, just bail.
                bit_true(step_control, STEP_CONTROL_END_MOTION);
                goto segment_prep_buffer_exit; // Segment not generated, but current step data still retained.
            }
        }

        /* Compute segment step rate. Since steps are integers and mm distances traveled are not,
           the end of every segment can have a partial step of varying magnitudes that are not
           executed, because the stepper ISR requires whole steps due to the AMASS algorithm. To
           compensate, we track the time to execute the previous segment's partial step and simply
           apply it with the partial step distance to the current segment, so that it minutely
           adjusts the whole segment rate to keep step output exact. These rate adjustments are
           typically very small and do not adversely effect performance, but ensures that Grbl
           outputs the exact acceleration and velocity profiles as computed by the motion. */
        dt += prep.dt_remainder; // Apply previous segment partial step execute time
        float inv_rate = dt / (last_n_steps_remaining - step_dist_remaining); // Compute adjusted step rate inverse

        // Compute CPU cycles per step for the prepped segment.
        uint32_t cycles = (uint32_t) ceilf(STEP_FREQUENCY * 60 * inv_rate); // (cycles/step)
        prep_segment->cycles_per_tick = cycles;

        // Segment complete! Increment segment buffer indices, so stepper ISR can immediately execute it.
        segment_buffer_head = segment_next_head;
        if (++segment_next_head == SEGMENT_BUFFER_SIZE) { segment_next_head = 0; }

        // Update the appropriate motion and segment data.
        pl_block->millimeters = mm_remaining;
        prep.steps_remaining = n_steps_remaining;
        prep.dt_remainder = (n_steps_remaining - step_dist_remaining) * inv_rate;

        // Check for exit conditions and flag to load next motion block.
        if (mm_remaining == prep.mm_complete) {
            // End of motion block or forced-termination. No more distance to be executed.
            if (mm_remaining > 0.0) { // At end of forced-termination.
                // Reset prep parameters for resuming and then bail. Allow the stepper ISR to complete
                // the segment queue, where realtime protocol will set new state upon receiving the
                // cycle stop flag from the ISR. Prep_segment is blocked until then.
                bit_true(step_control, STEP_CONTROL_END_MOTION);
                goto segment_prep_buffer_exit; // Bail!
            } else { // End of motion block
                // The motion block is complete. All steps are set to be executed in the segment buffer.
                if (step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
                    bit_true(step_control, STEP_CONTROL_END_MOTION);
                    goto segment_prep_buffer_exit;
                }
                pl_block = NULL; // Set pointer to indicate check and load next motion block.
                plan_discard_current_block();
            }
        }

    }
segment_prep_buffer_exit:
    return;
}

/**
 * @brief Reset Segment buffer
 */
void segment_reset() {
    segment_buffer_tail = 0;
    segment_buffer_head = 0;
    segment_next_head = 1;
}

/**
 * @brief Called by planner_recalculate() when the executing block is updated by the new plan.
 */
void st_update_plan_block_parameters() {
    if (pl_block != NULL) { // Ignore if at start of a new block.
        prep.recalculate_flag |= PREP_FLAG_RECALCULATE;
        pl_block->entry_speed_sqr = prep.current_speed * prep.current_speed; // Update entry speed.
        pl_block = NULL; // Flag st_prep_segment() to load and check active velocity profile.
    }
}

/** @} */
/** @} */
