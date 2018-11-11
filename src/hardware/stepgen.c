/**
 * @file stepgen.c
 * @brief Step generator
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * Adapted from Grbl
 * @copyright Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
 * @copyright Copyright (c) 2009-2011 Simen Svale Skogsrud

 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup hardware
 * @{
 * @defgroup hardware_stepgen Step Pulse Generator
 *
 * Step Pulse Generator
 * @{
 */

#include <alchemy/task.h>
#include <memory.h>
#include <sched.h>
#include <fcntl.h>
#include "../openglow-cnc.h"

/**
 * @brief Switch event loop real time task
 */
static cpu_set_t rt_cpus;

/**
 * @brief Switch event loop real time task
 */
int32_t sys_position[N_AXIS] = {0};

/**
 * @brief Switch event loop real time task
 */
static RT_TASK rt_stepgen_loop_task;

// Static function declarations
static void _stepgen_loop();

#ifdef DEBUG_STEP_TO_FILE
FILE *f_step; // 'pulse' file output
FILE *f_cnt;  // Segment cycles file output
#endif // DEBUG_STEP_TO_FILE


/**
 * @brief Step Generator data struct. Contains the running data for the step generator loop.
 */
typedef struct {
    // Used by the bresenham line algorithm
    uint32_t counter_x, /*!< X counter variable for the bresenham line tracer */
            counter_y,  /*!< Y counter variable for the bresenham line tracer */
            counter_z;  /*!< Z counter variable for the bresenham line tracer */

    uint8_t step_outbits;   /*!< The next stepping bits to be output */
    uint8_t dir_outbits;    /*!< The next direction bits to be output */

    uint16_t step_count;    /*!< Steps remaining in line segment motion */
    uint8_t exec_block_index; /*!< Tracks the current st_block index. Change indicates new block. */
    st_block_t *exec_block;   /*!< Pointer to the block data for the segment being executed */
    segment_t *exec_segment;  /*!< Pointer to the segment being executed */
} stepgen_t;

/**
 * Running data for the step generator loop
 */
static stepgen_t st;

/**
 * @brief Reset and clear step generator variables
 */
void stepgen_clear() {
    // Initialize step generator idle state.
    stepgen_go_idle();

    // Initialize step generator algorithm variables.
    memset(&prep, 0, sizeof(st_prep_t));
    memset(&st, 0, sizeof(stepgen_t));
    st.exec_segment = NULL;
    pl_block = NULL;  // Planner block pointer used by segment buffer
    segment_reset();

    st.dir_outbits = 0; // Initialize direction bits.
}

/**
 * @brief Step Generator switch to idle
 * @return 0 on success, negative on failure
 */
ssize_t stepgen_go_idle() {
    ssize_t ret = 0;
    if (verbose) printf("stepgen_go_idle: init\n");
#ifdef DEBUG_STEP_TO_FILE
    if (f_step > 0) {
        fclose(f_step);
        f_step = 0;
    }
    if (f_cnt > 0) {
        fclose(f_cnt);
        f_cnt = 0;
    }
#endif // DEBUG_STEP_TO_FILE
#ifdef TARGET_BUILD
    openglow_pulse_close();
#endif // TARGET_BUILD
    return ret;
}

/**
 * @brief Initialize the Step Generator and OpenGlow pulse device and stepper drivers
 * @return 0 on success, negative on failure.
 */
ssize_t stepgen_init() {
    ssize_t ret = 0;
    if ((rt_task_spawn(&rt_stepgen_loop_task, "rt_stepgen_loop_task", 0,
            STEP_GEN_PRIORITY, 0, &_stepgen_loop, 0)) < 0) {
        fprintf(stderr, "stepgen_init: rt_task_spawn returned %zd\n", ret);
        return ret;
    }

    // Initialize the stepper drivers
    if ((ret = step_drv_init()) < 0) {
        fprintf(stderr, "stepgen_init: step_drv_init returned %zd\n", ret);
        return ret;
    }

    // Initialize OpenGlow pulse device
    if ((ret = openglow_clear(OG_CLEAR_ALL)) < 0) {
        fprintf(stderr, "stepgen_init: openglow_clear returned %zd\n", ret);
        return ret;
    }

    // Set up CPU Affinity for the reserved RT processor
    CPU_ZERO(&rt_cpus);
    CPU_SET(STEP_GEN_CPU_AFFINITY, &rt_cpus);

    // Set task affinity
    rt_task_set_affinity(&rt_stepgen_loop_task, &rt_cpus);
    return ret;
}

/**
 * @brief Step Generator run loop
 *
 * Generates step out for the OpenGlow's SDMA stepper interface. Woken by run stepgen_wake_up(), and runs
 * until all blocks in the buffer have been run, a feed hold is requested, or the system goes into FAULT or ALARM.
 *
 * @note Runs as Xenomai Alchemy Task with a priority of STEP_GEN_PRIORITY.
 */
static void _stepgen_loop() {
    ssize_t ret = 0;
    bool sdma_run = false;
    uint32_t cycle_count = 0;
    uint32_t segment_count = 0;
    uint16_t step_cycle_count = 0;
    rt_task_suspend(NULL);
    while (loop_run) {
        cycle_count++;
        step_cycle_count++;
        // If there is no step segment, attempt to pop one from the stepper buffer
        if (st.exec_segment == NULL) {
#ifdef TARGET_BUILD
            openglow_pulse_flush();
#endif // TARGET_BUILD
            // Anything in the buffer? If so, load and initialize next step segment.
            if (segment_buffer_head != segment_buffer_tail) {
                // We want at least one second of data before we start the SDMA engine.
#ifdef TARGET_BUILD
                if ((sys_state != SYS_STATE_RUN) && (sys_state != SYS_STATE_HOMING)
                    && !sdma_run && (cycle_count > STEP_FREQUENCY)) {
                    sdma_run = true;
                    if (verbose) printf("_stepper_loop: SDMA run during cycles\n");
                    if ((ret = openglow_write_attr_str(ATTR_RUN, "1\n")) < 0)
                        fprintf(stderr, "_stepper_loop: openglow_write_attr_str returned %zd\n", ret);
                }
#endif // TARGET_BUILD

                // Initialize new step segment and load number of steps to execute
                st.exec_segment = &segment_buffer[segment_buffer_tail];

                // Initialize step segment timing per step and load number of steps to execute.
                st.step_count = st.exec_segment->n_step; // NOTE: Can sometimes be zero when moving slow.
                // If the new segment starts a new motion block, initialize stepper variables and counters.
                // NOTE: When the segment data index changes, this indicates a new motion block.
                if (st.exec_block_index != st.exec_segment->st_block_index) {
                    st.exec_block_index = st.exec_segment->st_block_index;
                    st.exec_block = &st_block_buffer[st.exec_block_index];

                    // Initialize Bresenham line and distance counters
                    st.counter_x = st.counter_y = st.counter_z = (st.exec_block->step_event_count >> 1);
                }
                st.dir_outbits = st.exec_block->direction_bits;

                // Set real-time spindle output as segment is loaded, just prior to the first step.
//            spindle_set_speed(st.exec_segment->spindle_pwm);
                step_cycle_count = 0;
#ifdef DEBUG_STEP_TO_FILE
                fprintf(f_cnt, "%d\n", st.exec_segment->cycles_per_tick);
#endif // DEBUG_STEP_TO_FILE

            } else {
                // Segment buffer empty. TODO: Set this to check if motion is still crunching
                // Ensure pwm is set properly upon completion of rate-controlled motion.
                // TODO: Change the whole way this is working....
                if (verbose) printf("stepper_loop: suspend after %d cycles, %d segments\n", cycle_count, segment_count);
                cycle_count = 0;
                step_cycle_count = 0;
                // If over 1 second wasn't written to the buffer, run the SDMA now
                if ((sys_req_state == SYS_STATE_RUN) && !sdma_run) {
#ifdef TARGET_BUILD
                    if (verbose) printf("_stepper_loop: SDMA run after cycles\n");
                    if ((ret = openglow_write_attr_str(ATTR_RUN, "1\n")) < 0)
                        fprintf(stderr, "_stepper_loop: openglow_write_attr_str returned %zd\n", ret);
#endif
                } else fsm_request(SYS_STATE_IDLE);
                rt_task_suspend(NULL);
                if (verbose) printf("stepper_loop: resume\n");
                sdma_run = false;
                continue;
            }
        }

        step_cycle_count++;
        if (step_cycle_count < st.exec_segment->cycles_per_tick) {
            // Output spacer pulse
#ifdef DEBUG_STEP_TO_FILE
            putc(0x00, f_step);
#endif // DEBUG_STEP_TO_FILE
#ifdef TARGET_BUILD
            openglow_pulse_write(0x00);
#endif // TARGET_BUILD
            continue;
        }
        step_cycle_count = 0;

        // Reset step out bits.
        st.step_outbits = 0;

        // Execute step displacement profile by Bresenham line algorithm
        st.counter_x += st.exec_block->steps[X_AXIS];
        if (st.counter_x > st.exec_block->step_event_count) {
            st.step_outbits |= X_AXIS_STEP_BIT;
            st.counter_x -= st.exec_block->step_event_count;
            if (st.exec_block->direction_bits & X_AXIS_DIR_BIT) { sys_position[X_AXIS]--; }
            else { sys_position[X_AXIS]++; }
        }
        st.counter_y += st.exec_block->steps[Y_AXIS];
        if (st.counter_y > st.exec_block->step_event_count) {
            st.step_outbits |= Y_AXIS_STEP_BIT;
            st.counter_y -= st.exec_block->step_event_count;
            if (st.exec_block->direction_bits & Y_AXIS_DIR_BIT) { sys_position[Y_AXIS]--; }
            else { sys_position[Y_AXIS]++; }
        }
        st.counter_z += st.exec_block->steps[Z_AXIS];
        if (st.counter_z > st.exec_block->step_event_count) {
            st.step_outbits |= Z_AXIS_STEP_BIT;
            st.counter_z -= st.exec_block->step_event_count;
            if (st.exec_block->direction_bits & Z_AXIS_DIR_BIT) { sys_position[Z_AXIS]--; }
            else { sys_position[Z_AXIS]++; }
        }

            // Output step pulse
#ifdef DEBUG_STEP_TO_FILE
        putc(st.step_outbits, f_step);
#endif // DEBUG_STEP_TO_FILE
#ifdef TARGET_BUILD
        openglow_pulse_write(st.step_outbits | st.exec_block->direction_bits);
#endif // TARGET_BUILD

        // During a homing cycle, lock out and prevent desired axes from moving.
//        if (sys.state.mode == STATE_HOMING) { st.step_outbits &= sys.state.homing_axis_lock; }

        st.step_count--; // Decrement step events count
        if (st.step_count == 0) {
            // Segment is complete. Discard current segment and advance segment indexing.
            st.exec_segment = NULL;
            if (++segment_buffer_tail == SEGMENT_BUFFER_SIZE) { segment_buffer_tail = 0; }
            segment_count++;
            // Tickle segment worker to keep our buffer full
            segment_prep_buffer();
        }
    }

}

/**
 * @brief Initialized OpenGlow pulse interface and starts _stepgen_loop().
 * @return 0 on success, negative on error.
 */
ssize_t stepgen_wake_up() {
    if (verbose) printf("stepgen_wake_up: init\n");
    ssize_t ret = 0;

    // Charge the segment buffers
    segment_prep_buffer();
    // Enable stepper drivers
    // TODO
#ifdef TARGET_BUILD
    if ((ret = openglow_pulse_open()) < 0) {
        fprintf(stderr, "stepper_wake_up: openglow_pulse_open returned %zd\n", ret);
        fsm_update(FSM_MOTION, MOT_STATE_FAULT);
        return ret;
    }
#endif // TARGET_BUILD

    // Initialize stepper output bits
    st.step_outbits = 0;

#ifdef DEBUG_STEP_TO_FILE
    f_step = fopen("dbg_step.bin", "w");
    f_cnt = fopen("dbg_cnt.txt", "w");
#endif // DEBUG_STEP_TO_FILE
    // Release STEPPER_LOOP
    if ((ret = rt_task_resume(&rt_stepgen_loop_task)) < 0) {
        fprintf(stderr, "stepgen_wake_up: rt_task_resume returned %zd\n", ret);
        return ret;
    }
    return ret;
}

/** @} */
/** @} */
