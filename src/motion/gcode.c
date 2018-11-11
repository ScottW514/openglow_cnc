/**
 * @file gcode.c
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
 * @addtogroup motion
 *
 * @{
 * @defgroup motion_gc G Code Parser
 *
 * G Code Parser
 *
 * @{
 */

#include <alchemy/task.h>
#include <alchemy/queue.h>
#include <math.h>
#include <memory.h>
#include "../openglow-cnc.h"
#include "grbl_glue.h"


#define AXIS_COMMAND_NONE 0
#define AXIS_COMMAND_NON_MODAL 1
#define AXIS_COMMAND_MOTION_MODE 2

/**
 * @brief Modal group internal numbers
 *
 * Define modal group internal numbers for checking multiple cli violations and tracking the
 * type of cli that is called in the block. A modal group is a group of g-code commands that are
 * mutually exclusive, or cannot exist on the same line, because they each toggle a state or execute
 * a unique motion. These are defined in the NIST RS274-NGC v3 g-code standard, available online,
 * and are similar/identical to other g-code interpreters by manufacturers (Haas,Fanuc,Mazak,etc).
 * @note Modal group define values must be sequential and starting from zero.
 */
enum MODAL_GROUP {
    MODAL_GROUP_G0,  /*!< [G4,G10,G28,G28.1,G30,G30.1,G53,G92,G92.1] Non-modal */
    MODAL_GROUP_G1,  /*!< [G0,G1,G2,G3,G38.2,G38.3,G38.4,G38.5,G80] Motion */
    MODAL_GROUP_G2,  /*!< [G17,G18,G19] Plane selection */
    MODAL_GROUP_G3,  /*!< [G90,G91] Distance mode */
    MODAL_GROUP_G4,  /*!< [G91.1] Arc IJK distance mode */
    MODAL_GROUP_G5,  /*!< [G93,G94] Feed rate mode */
    MODAL_GROUP_G6,  /*!< [G20,G21] Units */
    MODAL_GROUP_G7,  /*!< [G40] Cutter radius compensation mode. G41/42 NOT SUPPORTED. */
    MODAL_GROUP_G12, /*!< [G54,G55,G56,G57,G58,G59] Coordinate system selection */
    MODAL_GROUP_G13, /*!< [G61] Control mode */
    MODAL_GROUP_M4,  /*!< [M0,M1,M2,M30] Stopping */
    MODAL_GROUP_M7,  /*!< [M3,M4,M5] Spindle turning */
    MODAL_GROUP_M8,  /*!< [M7,M8,M9] Coolant control */
};

/**
 * @brief Parameter word mapping
 */
enum WORD {
    WORD_F,
    WORD_I,
    WORD_J,
    WORD_K,
    WORD_L,
    WORD_N,
    WORD_P,
    WORD_R,
    WORD_S,
    WORD_T,
    WORD_X,
    WORD_Y,
    WORD_Z,
};

// NOTE: When this struct is zeroed, the above defines set the defaults for the system.
/**
 * @brief Modal values for current G-Code command
 */
typedef struct {
    uint8_t motion;          /*!< {G0,G1,G2,G3,G38.2,G80} */
    uint8_t feed_rate;       /*!< {G93,G94} */
    uint8_t units;           /*!< {G20,G21} */
    uint8_t distance;        /*!< {G90,G91} */
    uint8_t plane_select;    /*!< {G17,G18,G19} */
    uint8_t coord_select;    /*!< {G54,G55,G56,G57,G58,G59} */
    uint8_t program_flow;    /*!< {M0,M1,M2,M30} */
    uint8_t coolant;         /*!< {M7,M8,M9} */
    uint8_t spindle;         /*!< {M3,M4,M5} */
} gc_modal_t;

/**
 * @brief Hold values for current G-Code command
 */
typedef struct {
    float f;         /*!< Feed */
    float ijk[3];    /*!< I,J,K Axis arc offsets */
    uint8_t l;       /*!< G10 or canned cycles parameters */
    int32_t n;       /*!< Line number */
    float p;         /*!< G10 or dwell parameters */
    float r;         /*!< Arc radius */
    float s;         /*!< Spindle speed */
    float xyz[3];    /*!< X,Y,Z Translational axes */
} gc_values_t;


/**
 * @brief State for the current G-Code command
 */
typedef struct {
    gc_modal_t modal;       /*!< Modal values */
    float spindle_speed;    /*!< RPM */
    float feed_rate;        /*!< Millimeters/min */
    int32_t line_number;    /*!< Last line number sent */
    float position[N_AXIS]; /*!< Where the interpreter considers the tool to be at this point in the code */
} parser_state_t;

/**
 * @brief State for the current command
 */
parser_state_t gc_state;


/**
 * @brief G-Code block data
 */
typedef struct {
    uint8_t non_modal_command;
    gc_modal_t modal;
    gc_values_t values;
} parser_block_t;

/**
 * @brief G-Code block data
 */
parser_block_t gc_block;

/**
 * @brief G-Code Parser loop real time task
 */
static RT_TASK rt_gc_loop_task;

/**
 * @brief G-Code Parser inbound queue
 */
static RT_QUEUE rt_gc_queue;

// Static function declarations
static void _gc_loop();
uint8_t _gc_execute_line(char *line);

/**
 * @brief Executes one line of 0-terminated G-Code.
 *
 * Executes one line of 0-terminated G-Code. The line is assumed to contain only uppercase
 * characters and signed floating point values (no whitespace). Comments and block delete
 * characters have been removed. In this function, all units and positions are converted and
 * exported to grbl's internal functions in terms of (mm, mm/min) and absolute machine
 * coordinates, respectively.
 * @param line
 * @return
 */
uint8_t _gc_execute_line(char *line) {
    /* -------------------------------------------------------------------------------------
       STEP 1: Initialize parser block struct and copy current g-code state modes. The parser
       updates these modes and commands as the block line is parser and will only be used and
       executed after successful error-checking. The parser block struct also contains a block
       values struct, word tracking variables, and a non-modal commands tracker for the new
       block. This struct contains all of the necessary information to execute the block. */

    memset(&gc_block, 0, sizeof(parser_block_t)); // Initialize the parser block struct.
    memcpy(&gc_block.modal, &gc_state.modal, sizeof(gc_modal_t)); // Copy current modes

    uint8_t axis_command = AXIS_COMMAND_NONE;
    uint8_t axis_0, axis_1, axis_linear;

    // Initialize bitflag tracking variables for axis indices compatible operations.
    uint8_t axis_words = 0; // XYZ tracking
    uint8_t ijk_words = 0; // IJK tracking

    // Initialize cli and value words and parser flags variables.
    uint16_t command_words = 0; // Tracks G and M cli words. Also used for modal group violations.
    uint16_t value_words = 0; // Tracks value words.
    uint8_t gc_parser_flags = GC_PARSER_NONE;

    /* -------------------------------------------------------------------------------------
       STEP 2: Import all g-code words in the block line. A g-code word is a letter followed by
       a number, which can either be a 'G'/'M' cli or sets/assigns a cli value. Also,
       perform initial error-checks for cli word modal group violations, for any repeated
       words, and for negative values set for the value words F, N, P, T, and S. */

    uint8_t word_bit; // Bit-value for assigning tracking variables
    uint8_t char_counter;
    char letter;
    float value;
    uint8_t int_value = 0;
    uint16_t mantissa = 0;
    char_counter = 0;

    while (line[char_counter] != 0) { // Loop until no more g-code words in line.

        // Import the next g-code word, expecting a letter followed by a value. Otherwise, error out.
        letter = line[char_counter];
        if ((letter < 'A') || (letter > 'Z')) { return STATUS_EXPECTED_COMMAND_LETTER; } // [Expected word letter]
        char_counter++;
        if (!read_float(line, &char_counter, &value)) { return STATUS_BAD_NUMBER_FORMAT; } // [Expected word value]

        /* Convert values to smaller uint8 significance and mantissa values for parsing this word.
           NOTE: Mantissa is multiplied by 100 to catch non-integer cli values. This is more
           accurate than the NIST gcode requirement of x10 when used for commands, but not quite
           accurate enough for value words that require integers to within 0.0001. This should be
           a good enough compromise and catch most all non-integer errors. To make it compliant,
           we would simply need to change the mantissa to int16, but this add compiled flash space.
           Maybe update this later. */
        int_value = (uint8_t) truncf(value);
        mantissa = (uint16_t) roundf(100 * (value - int_value)); // Compute mantissa for Gxx.x commands.
        // NOTE: Rounding must be used to catch small floating point errors.

        // Check if the g-code word is supported or errors due to modal group violations or has
        // been repeated in the g-code block. If ok, update the cli or record its value.
        switch (letter) {

            /* 'G' and 'M' Command Words: Parse commands and check for modal group violations.
               NOTE: Modal group numbers are defined in Table 4 of NIST RS274-NGC v3, pg.20 */

            case 'G':
                // Determine 'G' cli and its modal group
                switch (int_value) {
                    case 10:
                    case 28:
                    case 30:
                    case 92:
                        // Check for G10/28/30/92 being called with G0/1/2/3/38 on same block.
                        // * G43.1 is also an axis cli but is not explicitly defined this way.
                        if (mantissa == 0) { // Ignore G28.1, G30.1, and G92.1
                            if (axis_command) { return STATUS_AXIS_COMMAND_CONFLICT; } // [Axis word/cli conflict]
                            axis_command = AXIS_COMMAND_NON_MODAL;
                        }
                        // No break. Continues to next line.
                    case 4:
                    case 53:
                        word_bit = MODAL_GROUP_G0;
                        gc_block.non_modal_command = int_value;
                        if ((int_value == 28) || (int_value == 30) || (int_value == 92)) {
                            if (!((mantissa == 0) || (mantissa == 10))) { return STATUS_UNSUPPORTED_COMMAND; }
                            gc_block.non_modal_command += mantissa;
                            mantissa = 0; // Set to zero to indicate valid non-integer G cli.
                        }
                        break;
                    case 0:
                    case 1:
                    case 2:
                    case 3:
                    case 38:
                        // Check for G0/1/2/3/38 being called with G10/28/30/92 on same block.
                        // * G43.1 is also an axis cli but is not explicitly defined this way.
                        if (axis_command) { return STATUS_AXIS_COMMAND_CONFLICT; } // [Axis word/cli conflict]
                        axis_command = AXIS_COMMAND_MOTION_MODE;
                        // No break. Continues to next line.
                    case 80:
                        word_bit = MODAL_GROUP_G1;
                        gc_block.modal.motion = int_value;
                        if (int_value == 38) {
                            if (!((mantissa == 20) || (mantissa == 30) || (mantissa == 40) || (mantissa == 50))) {
                                return STATUS_UNSUPPORTED_COMMAND; // [Unsupported G38.x cli]
                            }
                            gc_block.modal.motion += (mantissa / 10) + 100;
                            mantissa = 0; // Set to zero to indicate valid non-integer G cli.
                        }
                        break;
                    case 17:
                    case 18:
                    case 19:
                        word_bit = MODAL_GROUP_G2;
                        gc_block.modal.plane_select = int_value - (uint8_t) 17;
                        break;
                    case 90:
                    case 91:
                        if (mantissa == 0) {
                            word_bit = MODAL_GROUP_G3;
                            gc_block.modal.distance = int_value - (uint8_t) 90;
                        } else {
                            word_bit = MODAL_GROUP_G4;
                            if ((mantissa != 10) || (int_value == 90)) {
                                return STATUS_UNSUPPORTED_COMMAND;
                            } // [G90.1 not supported]
                            mantissa = 0; // Set to zero to indicate valid non-integer G cli.
                            // Otherwise, arc IJK incremental mode is default. G91.1 does nothing.
                        }
                        break;
                    case 93:
                    case 94:
                        word_bit = MODAL_GROUP_G5;
                        gc_block.modal.feed_rate = (uint8_t) 94 - int_value;
                        break;
                    case 20:
                    case 21:
                        word_bit = MODAL_GROUP_G6;
                        gc_block.modal.units = (uint8_t) 21 - int_value;
                        break;
                    case 40:
                        word_bit = MODAL_GROUP_G7;
                        // NOTE: Not required since cutter radius compensation is always disabled. Only here
                        // to support G40 commands that often appear in g-code program headers to setup defaults.
                        // gc_block.modal.cutter_comp = CUTTER_COMP_DISABLE; // G40
                        break;
                    case 43:
                    case 49:
                    case 54:
                    case 55:
                    case 56:
                    case 57:
                    case 58:
                    case 59:
                        // NOTE: G59.x are not supported. (But their int_values would be 60, 61, and 62.)
                        word_bit = MODAL_GROUP_G12;
                        gc_block.modal.coord_select = int_value - (uint8_t) 54; // Shift to array indexing.
                        break;
                    case 61:
                        word_bit = MODAL_GROUP_G13;
                        if (mantissa != 0) { return STATUS_UNSUPPORTED_COMMAND; } // [G61.1 not supported]
                        break;
                    default:
                        return STATUS_UNSUPPORTED_COMMAND; // [Unsupported G cli]
                }
                if (mantissa > 0) {
                    return STATUS_COMMAND_VALUE_NOT_INTEGER;
                } // [Unsupported or invalid Gxx.x cli]
                // Check for more than one cli per modal group violations in the current block
                // NOTE: Variable 'word_bit' is always assigned, if the cli is valid.
                if (bit_istrue(command_words, bit(word_bit))) { return STATUS_MODAL_GROUP_VIOLATION; }
                command_words |= bit(word_bit);
                break;

            case 'M':

                // Determine 'M' cli and its modal group
                if (mantissa > 0) { return STATUS_COMMAND_VALUE_NOT_INTEGER; } // [No Mxx.x commands]
                switch (int_value) {
                    case 0:
                    case 1:
                    case 2:
                    case 30:
                        word_bit = MODAL_GROUP_M4;
                        switch (int_value) {
                            case 0:
                                gc_block.modal.program_flow = PROGRAM_FLOW_PAUSED;
                                break; // Program pause
                            case 1:
                                break; // Optional stop not supported. Ignore.
                            default:
                                gc_block.modal.program_flow = int_value; // Program end and reset
                        }
                        break;
                    case 3:
                    case 4:
                    case 5:
                        word_bit = MODAL_GROUP_M7;
                        switch (int_value) {
                            case 4:
                                gc_block.modal.spindle = LASER_ENABLE;
                                break;
                            case 5:
                                gc_block.modal.spindle = LASER_DISABLE;
                                break;
                            default:;
                        }
                        break;
                    case 8:
                    case 9:
                        word_bit = MODAL_GROUP_M8;
                        switch (int_value) {
                            case 8:
                                gc_block.modal.coolant |= COOLANT_FLOOD_ENABLE;
                                break;
                            case 9:
                                gc_block.modal.coolant = COOLANT_DISABLE;
                                break; // M9 disables both M7 and M8.
                            default:;
                        }
                        break;
                    default:
                        return STATUS_UNSUPPORTED_COMMAND; // [Unsupported M cli]
                }

                // Check for more than one cli per modal group violations in the current block
                // NOTE: Variable 'word_bit' is always assigned, if the cli is valid.
                if (bit_istrue(command_words, bit(word_bit))) { return STATUS_MODAL_GROUP_VIOLATION; }
                command_words |= bit(word_bit);
                break;

                // NOTE: All remaining letters assign values.
            default:

                /* Non-Command Words: This initial parsing phase only checks for repeats of the remaining
                   legal g-code words and stores their value. Error-checking is performed later since some
                   words (I,J,K,L,P,R) have multiple connotations and/or depend on the issued commands. */
                switch (letter) {
                    // case 'A': // Not supported
                    // case 'B': // Not supported
                    // case 'C': // Not supported
                    // case 'D': // Not supported
                    case 'F':
                        word_bit = WORD_F;
                        gc_block.values.f = value;
                        break;
                        // case 'H': // Not supported
                    case 'I':
                        word_bit = WORD_I;
                        gc_block.values.ijk[X_AXIS] = value;
                        ijk_words |= (1 << X_AXIS);
                        break;
                    case 'J':
                        word_bit = WORD_J;
                        gc_block.values.ijk[Y_AXIS] = value;
                        ijk_words |= (1 << Y_AXIS);
                        break;
                    case 'K':
                        word_bit = WORD_K;
                        gc_block.values.ijk[Z_AXIS] = value;
                        ijk_words |= (1 << Z_AXIS);
                        break;
                    case 'L':
                        word_bit = WORD_L;
                        gc_block.values.l = int_value;
                        break;
                    case 'N':
                        word_bit = WORD_N;
                        gc_block.values.n = (int32_t) truncf(value);
                        break;
                    case 'P':
                        word_bit = WORD_P;
                        gc_block.values.p = value;
                        break;
                        // NOTE: For certain commands, P value must be an integer, but none of these commands are supported.
                        // case 'Q': // Not supported
                    case 'R':
                        word_bit = WORD_R;
                        gc_block.values.r = value;
                        break;
                    case 'S':
                        word_bit = WORD_S;
                        gc_block.values.s = value;
                        break;
                    case 'X':
                        word_bit = WORD_X;
                        gc_block.values.xyz[X_AXIS] = value;
                        axis_words |= (1 << X_AXIS);
                        break;
                    case 'Y':
                        word_bit = WORD_Y;
                        gc_block.values.xyz[Y_AXIS] = value;
                        axis_words |= (1 << Y_AXIS);
                        break;
                    case 'Z':
                        word_bit = WORD_Z;
                        gc_block.values.xyz[Z_AXIS] = value;
                        axis_words |= (1 << Z_AXIS);
                        break;
                    default:
                        return STATUS_UNSUPPORTED_COMMAND;
                }

                // NOTE: Variable 'word_bit' is always assigned, if the non-cli letter is valid.
                if (bit_istrue(value_words, bit(word_bit))) { return STATUS_WORD_REPEATED; } // [Word repeated]
                // Check for invalid negative values for words F, N, P, T, and S.
                // NOTE: Negative value check is done here simply for code-efficiency.
                if (bit(word_bit) & (bit(WORD_F) | bit(WORD_N) | bit(WORD_P) | bit(WORD_T) | bit(WORD_S))) {
                    if (value < 0.0) { return STATUS_NEGATIVE_VALUE; } // [Word value cannot be negative]
                }
                value_words |= bit(word_bit); // Flag to indicate parameter assigned.

        }
    }
    // Parsing complete!


    /* -------------------------------------------------------------------------------------
       STEP 3: Error-check all commands and values passed in this block. This step ensures all of
       the commands are valid for execution and follows the NIST standard as closely as possible.
       If an error is found, all commands and values in this block are dumped and will not update
       the active system g-code modes. If the block is ok, the active system g-code modes will be
       updated based on the commands of this block, and signal for it to be executed.

       Also, we have to pre-convert all of the values passed based on the modes set by the parsed
       block. There are a number of error-checks that require target information that can only be
       accurately calculated if we convert these values in conjunction with the error-checking.
       This relegates the next execution step as only updating the system g-code modes and
       performing the programmed actions in order. The execution step should not require any
       conversion calculations and would only require minimal checks necessary to execute.
    */

    /* NOTE: At this point, the g-code block has been parsed and the block line can be freed.
       NOTE: It's also possible, at some future point, to break up STEP 2, to allow piece-wise
       parsing of the block on a per-word basis, rather than the entire block. This could remove
       the need for maintaining a large string variable for the entire block and free up some memory.
       To do this, this would simply need to retain all of the data in STEP 1, such as the new block
       data struct, the modal group and value bitflag tracking variables, and axis array indices
       compatible variables. This data contains all of the information necessary to error-check the
       new g-code block when the EOL character is received. However, this would break Grbl's startup
       lines in how it currently works and would require some refactoring to make it compatible.
    */

    // [0. Non-specific/common error-checks and miscellaneous setup]:

    // Determine implicit axis cli conditions. Axis words have been passed, but no explicit axis
    // cli has been sent. If so, set axis cli to current motion mode.
    if (axis_words) {
        if (!axis_command) { axis_command = AXIS_COMMAND_MOTION_MODE; } // Assign implicit motion-mode
    }

    // Check for valid line number N value.
    if (bit_istrue(value_words, bit(WORD_N))) {
        // Line number value cannot be less than zero (done) or greater than max line number.
        if (gc_block.values.n > MAX_G_CODE_LINE_NUMBER) {
            return STATUS_INVALID_LINE_NUMBER;
        } // [Exceeds max line number]
    }
    // bit_false(value_words,bit(WORD_N)); // NOTE: Single-meaning value word. Set at end of error-checking.

    /* Track for unused words at the end of error-checking.
       NOTE: Single-meaning value words are removed all at once at the end of error-checking, because
       they are always used when present. This was done to save a few bytes of flash. For clarity, the
       single-meaning value words may be removed as they are used. Also, axis words are treated in the
       same way. If there is an explicit/implicit axis cli, XYZ words are always used and are
       are removed at the end of error-checking. */

    // [1. Comments ]: MSG's NOT SUPPORTED. Comment handling performed by protocol.

    // [2. Set feed rate mode ]: G93 F word missing with G1,G2/3 active, implicitly or explicitly. Feed rate
    //   is not defined after switching to G94 from G93.  Enforce G94 and check for required F word.
    if (gc_block.modal.feed_rate == FEED_RATE_MODE_INVERSE_TIME) { // = G93
        // NOTE: G38 can also operate in inverse time, but is undefined as an error. Missing F word check added here.
        if (axis_command == AXIS_COMMAND_MOTION_MODE) {
            if ((gc_block.modal.motion != MOTION_MODE_NONE) && (gc_block.modal.motion != MOTION_MODE_SEEK)) {
                if (bit_isfalse(value_words, bit(WORD_F))) {
                    return STATUS_UNDEFINED_FEED_RATE;
                } // [F word missing]
            }
        }
        /* NOTE: It seems redundant to check for an F word to be passed after switching from G94 to G93. We would
           accomplish the exact same thing if the feed rate value is always reset to zero and undefined after each
           inverse time block, since the commands that use this value already perform undefined checks. This would
           also allow other commands, following this switch, to execute and not error out needlessly. This code is
           combined with the above feed rate mode and the below set feed rate error-checking. */

        // [3. Set feed rate ]: F is negative (done.)
        /* - In inverse time mode: Always implicitly zero the feed rate value before and after block completion.
           NOTE: If in G93 mode or switched into it from G94, just keep F value as initialized zero or passed F word
           value in the block. If no F word is passed with a motion cli that requires a feed rate, this will error
           out in the motion modes error-checking. However, if no F word is passed with NO motion cli that requires
           a feed rate, we simply move on and the state feed rate value gets updated to zero and remains undefined. */
    } else { // = G94
        // - In units per mm mode: If F word passed, ensure value is in mm/min, otherwise push last state value.
        if (gc_state.modal.feed_rate == FEED_RATE_MODE_UNITS_PER_MIN) { // Last state is also G94
            if (bit_istrue(value_words, bit(WORD_F))) {
                if (gc_block.modal.units == UNITS_MODE_INCHES) { gc_block.values.f *= MM_PER_INCH; }
            } else {
                gc_block.values.f = gc_state.feed_rate; // Push last state feed rate
            }
        } // Else, switching to G94 from G93, so don't push last state feed rate. Its undefined or the passed F word value.
    }
    // bit_false(value_words,bit(WORD_F)); // NOTE: Single-meaning value word. Set at end of error-checking.

    // [4. Set spindle speed ]: S is negative (done.)
    if (bit_isfalse(value_words, bit(WORD_S))) { gc_block.values.s = gc_state.spindle_speed; }
    // bit_false(value_words,bit(WORD_S)); // NOTE: Single-meaning value word. Set at end of error-checking.

    // [5. Select tool ]: NOT SUPPORTED.
    // [6. Change tool ]: N/A
    // [7. Spindle control ]: N/A
    // [8. Coolant control ]: N/A
    // [9. Override control ]: Not supported except for a Grbl-only parking motion override control.
    // [10. Dwell ]: P value missing. P is negative (done.) NOTE: See below.
    if (gc_block.non_modal_command == NON_MODAL_DWELL) {
        if (bit_isfalse(value_words, bit(WORD_P))) { return STATUS_VALUE_WORD_MISSING; } // [P word missing]
        bit_false(value_words, bit(WORD_P));
    }

    // [11. Set active plane ]: N/A
    switch (gc_block.modal.plane_select) {
        case PLANE_SELECT_XY:
            axis_0 = X_AXIS;
            axis_1 = Y_AXIS;
            axis_linear = Z_AXIS;
            break;
        case PLANE_SELECT_ZX:
            axis_0 = Z_AXIS;
            axis_1 = X_AXIS;
            axis_linear = Y_AXIS;
            break;
        default: // case PLANE_SELECT_YZ:
            axis_0 = Y_AXIS;
            axis_1 = Z_AXIS;
            axis_linear = X_AXIS;
    }

    // [12. Set length units ]: N/A
    // Pre-convert XYZ coordinate values to millimeters, if applicable.
    uint8_t idx;
    if (gc_block.modal.units == UNITS_MODE_INCHES) {
        for (idx = 0; idx < N_AXIS; idx++) { // Axes indices are consistent, so loop may be used.
            if (bit_istrue(axis_words, bit(idx))) {
                gc_block.values.xyz[idx] *= MM_PER_INCH;
            }
        }
    }

    // [13. Cutter radius compensation ]: G41/42 NOT SUPPORTED. Error, if enabled while G53 is active.
    /* [G40 Errors]: G2/3 arc is programmed after a G40. The linear move after disabling is less than tool diameter.
         NOTE: Since cutter radius compensation is never enabled, these G40 errors don't apply. Grbl supports G40
         only for the purpose to not error when G40 is sent with a g-code program header to setup the default modes. */

    // [14. Cutter length compensation ]: G43 NOT SUPPORTED, but G43.1 and G49 are.
    /* [G43.1 Errors]: Motion cli in same line.
         NOTE: Although not explicitly stated so, G43.1 should be applied to only one valid
         axis that is configured (in config.h). There should be an error if the configured axis
         is absent or if any of the other axis words are present. */

    // [15. Coordinate system selection ]: *N/A.
    // [16. Set path control mode ]: N/A. Only G61. G61.1 and G64 NOT SUPPORTED.
    // [17. Set distance mode ]: N/A. Only G91.1. G90.1 NOT SUPPORTED.
    // [18. Set retract mode ]: NOT SUPPORTED.

    // [19. Remaining non-modal actions ]: Check go to predefined position, set G10, or set axis offsets.
    /* NOTE: We need to separate the non-modal commands that are axis word-using (G10/G28/G30/G92), as these
       commands all treat axis words differently. G10 as absolute offsets or computes current position as
       the axis value, G92 similarly to G10 L20, and G28/30 as an intermediate target position that observes
       all the current coordinate system and G92 offsets. */

    // [20. Motion modes ]:
    if (gc_block.modal.motion == MOTION_MODE_NONE) {
        // [G80 Errors]: Axis word are programmed while G80 is active.
        // NOTE: Even non-modal commands or TLO that use axis words will throw this strict error.
        if (axis_words) { return STATUS_AXIS_WORDS_EXIST; } // [No axis words allowed]

        // Check remaining motion modes, if axis word are implicit (exist and not used by G10/28/30/92), or
        // was explicitly commanded in the g-code block.
    } else if (axis_command == AXIS_COMMAND_MOTION_MODE) {

        if (gc_block.modal.motion == MOTION_MODE_SEEK) {
            // [G0 Errors]: Axis letter not configured or without real value (done.)
            // Axis words are optional. If missing, set axis cli flag to ignore execution.
            if (!axis_words) { axis_command = AXIS_COMMAND_NONE; }

            // All remaining motion modes (all but G0 and G80), require a valid feed rate value. In units per mm mode,
            // the value must be positive. In inverse time mode, a positive value must be passed with each block.
        } else {
            // Check if feed rate is defined for the motion modes that require it.
            if (gc_block.values.f == 0.0) { return STATUS_UNDEFINED_FEED_RATE; } // [Feed rate undefined]

            switch (gc_block.modal.motion) {
                case MOTION_MODE_LINEAR:
                    // [G1 Errors]: Feed rate undefined. Axis letter not configured or without real value.
                    // Axis words are optional. If missing, set axis cli flag to ignore execution.
                    if (!axis_words) { axis_command = AXIS_COMMAND_NONE; }
                    break;
                case MOTION_MODE_CW_ARC:
                    gc_parser_flags |= GC_PARSER_ARC_IS_CLOCKWISE; // No break intentional.
                case MOTION_MODE_CCW_ARC:
                    // [G2/3 Errors All-Modes]: Feed rate undefined.
                    // [G2/3 Radius-Mode Errors]: No axis words in selected plane. Target point is same as current.
                    // [G2/3 Offset-Mode Errors]: No axis words and/or offsets in selected plane. The radius to the current
                    //   point and the radius to the target point differs more than 0.002mm (EMC def. 0.5mm OR 0.005mm and 0.1% radius).
                    // [G2/3 Full-Circle-Mode Errors]: NOT SUPPORTED. Axis words exist. No offsets programmed. P must be an integer.
                    // NOTE: Both radius and offsets are required for arc tracing and are pre-computed with the error-checking.

                    if (!axis_words) { return STATUS_NO_AXIS_WORDS; } // [No axis words]
                    if (!(axis_words & (bit(axis_0) | bit(axis_1)))) {
                        return STATUS_NO_AXIS_WORDS_IN_PLANE;
                    } // [No axis words in plane]

                    // Calculate the change in position along each selected axis
                    float x, y;
                    x = gc_block.values.xyz[axis_0] -
                        gc_state.position[axis_0]; // Delta x between current position and target
                    y = gc_block.values.xyz[axis_1] -
                        gc_state.position[axis_1]; // Delta y between current position and target

                    if (value_words & bit(WORD_R)) { // Arc Radius Mode
                        bit_false(value_words, bit(WORD_R));
                        if (isequal_position_vector(gc_state.position, gc_block.values.xyz)) {
                            return STATUS_INVALID_TARGET;
                        } // [Invalid target]

                        // Convert radius value to proper units.
                        if (gc_block.modal.units == UNITS_MODE_INCHES) { gc_block.values.r *= MM_PER_INCH; }
                        /*  We need to calculate the center of the circle that has the designated radius and passes
                            through both the current position and the target position. This method calculates the following
                            set of equations where [x,y] is the vector from current to target position, d == magnitude of
                            that vector, h == hypotenuse of the triangle formed by the radius of the circle, the distance to
                            the center of the travel vector. A vector perpendicular to the travel vector [-y,x] is scaled to the
                            length of h [-y/d*h, x/d*h] and added to the center of the travel vector [x/2,y/2] to form the new point
                            [i,j] at [x/2-y/d*h, y/2+x/d*h] which will be the center of our arc.

                            d^2 == x^2 + y^2
                            h^2 == r^2 - (d/2)^2
                            i == x/2 - y/d*h
                            j == y/2 + x/d*h

                                                                                 O <- [i,j]
                                                                              -  |
                                                                    r      -     |
                                                                        -        |
                                                                     -           | h
                                                                  -              |
                                                    [0,0] ->  C -----------------+--------------- T  <- [x,y]
                                                              | <------ d/2 ---->|

                            C - Current position
                            T - Target position
                            O - center of circle that pass through both C and T
                            d - distance from C to T
                            r - designated radius
                            h - distance from center of CT to O

                            Expanding the equations:

                            d -> sqrt(x^2 + y^2)
                            h -> sqrt(4 * r^2 - x^2 - y^2)/2
                            i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
                            j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2

                            Which can be written:

                            i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
                            j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2

                            Which we for size and speed reasons optimize to:

                            h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
                            i = (x - (y * h_x2_div_d))/2
                            j = (y + (x * h_x2_div_d))/2
                        */

                        // First, use h_x2_div_d to compute 4*h^2 to check if it is negative or r is smaller
                        // than d. If so, the sqrt of a negative number is complex and error out.
                        float h_x2_div_d = (float) 4.0 * gc_block.values.r * gc_block.values.r - x * x - y * y;

                        if (h_x2_div_d < 0) { return STATUS_ARC_RADIUS_ERROR; } // [Arc radius error]

                        // Finish computing h_x2_div_d.
                        h_x2_div_d = -sqrtf(h_x2_div_d) / hypot_f(x, y); // == -(h * 2 / d)
                        // Invert the sign of h_x2_div_d if the circle is counter clockwise (see sketch below)
                        if (gc_block.modal.motion == MOTION_MODE_CCW_ARC) { h_x2_div_d = -h_x2_div_d; }

                        /* The counter clockwise circle lies to the left of the target direction. When offset is positive,
                           the left hand circle will be generated - when it is negative the right hand circle is generated.

                                                                               T  <-- Target position

                                                                               ^
                                    Clockwise circles with this center         |          Clockwise circles with this center will have
                                    will have > 180 deg of angular travel      |          < 180 deg of angular travel, which is a good thing!
                                                                     \         |          /
                        center of arc when h_x2_div_d is positive ->  x <----- | -----> x <- center of arc when h_x2_div_d is negative
                                                                               |
                                                                               |

                                                                               C  <-- Current position

                           Negative R is g-code-alese for "I want a circle with more than 180 degrees of travel" (go figure!),
                           even though it is advised against ever generating such circles in a single line of g-code. By
                           inverting the sign of h_x2_div_d the center of the circles is placed on the opposite side of the line of
                           travel and thus we get the unadvisably long arcs as prescribed. */
                        if (gc_block.values.r < 0) {
                            h_x2_div_d = -h_x2_div_d;
                            gc_block.values.r = -gc_block.values.r; // Finished with r. Set to positive for mc_arc
                        }
                        // Complete the operation by calculating the actual center of the arc
                        gc_block.values.ijk[axis_0] = (float) 0.5 * (x - (y * h_x2_div_d));
                        gc_block.values.ijk[axis_1] = (float) 0.5 * (y + (x * h_x2_div_d));

                    } else { // Arc Center Format Offset Mode
                        if (!(ijk_words & (bit(axis_0) | bit(axis_1)))) {
                            return STATUS_NO_OFFSETS_IN_PLANE;
                        } // [No offsets in plane]
                        bit_false(value_words, (bit(WORD_I) | bit(WORD_J) | bit(WORD_K)));

                        // Convert IJK values to proper units.
                        if (gc_block.modal.units == UNITS_MODE_INCHES) {
                            // Axes indices are consistent, so loop may be used to save flash space.
                            for (idx = 0; idx < N_AXIS; idx++) {
                                if (ijk_words & bit(idx)) { gc_block.values.ijk[idx] *= MM_PER_INCH; }
                            }
                        }

                        // Arc radius from center to target
                        x -= gc_block.values.ijk[axis_0]; // Delta x between circle center and target
                        y -= gc_block.values.ijk[axis_1]; // Delta y between circle center and target
                        float target_r = hypot_f(x, y);

                        // Compute arc radius for mc_arc. Defined from current location to center.
                        gc_block.values.r = hypot_f(gc_block.values.ijk[axis_0], gc_block.values.ijk[axis_1]);

                        // Compute difference between current location and target radii for final error-checks.
                        float delta_r = fabsf(target_r - gc_block.values.r);
                        if (delta_r > 0.005) {
                            if (delta_r > 0.5) { return STATUS_INVALID_TARGET; } // [Arc definition error] > 0.5mm
                            if (delta_r > (0.001 * gc_block.values.r)) {
                                return STATUS_INVALID_TARGET;
                            } // [Arc definition error] > 0.005mm AND 0.1% radius
                        }
                    }
                    break;
                default:;
            }
        }
    }

    // [21. Program flow ]: No error checks required.

    // [0. Non-specific error-checks]: Complete unused value words check, i.e. IJK used when in arc
    // radius mode, or axis words that aren't used in the block.
    bit_false(value_words,
              (bit(WORD_N) | bit(WORD_F) | bit(WORD_S) | bit(WORD_T))); // Remove single-meaning value words.
    if (axis_command) { bit_false(value_words, (bit(WORD_X) | bit(WORD_Y) | bit(WORD_Z))); } // Remove axis words.
    if (value_words) { return STATUS_UNUSED_WORDS; } // [Unused words]

    /* -------------------------------------------------------------------------------------
       STEP 4: EXECUTE!!
       Assumes that all error-checking has been completed and no failure modes exist. We just
       need to update the state and execute the block according to the order-of-execution.
    */

    // Initialize motion data struct for motion blocks.
    plan_line_data_t plan_data;
    plan_line_data_t *pl_data = &plan_data;
    memset(pl_data, 0, sizeof(plan_line_data_t)); // Zero pl_data struct

    // If in laser mode, setup laser power based on current and past parser conditions.
    if (settings.laser_power_correction) {
        if (!((gc_block.modal.motion == MOTION_MODE_LINEAR) || (gc_block.modal.motion == MOTION_MODE_CW_ARC)
              || (gc_block.modal.motion == MOTION_MODE_CCW_ARC))) {
            gc_parser_flags |= GC_PARSER_LASER_DISABLE;
        }

        // Any motion mode with axis words is allowed to be passed from a spindle speed update.
        // NOTE: G1 and G0 without axis words sets axis_command to none. G28/30 are intentionally omitted.
        // TODO: Check sync conditions for M3 enabled motions that don't enter the motion. (zero length).
        if (axis_words && (axis_command == AXIS_COMMAND_MOTION_MODE)) {
            gc_parser_flags |= GC_PARSER_LASER_ISMOTION;
        } else {
            // M3 constant power laser requires motion syncs to update the laser when changing between
            // a G1/2/3 motion mode state and vice versa when there is no motion in the line.
            if (gc_state.modal.spindle == SPINDLE_ENABLE_CW) {
                if ((gc_state.modal.motion == MOTION_MODE_LINEAR) || (gc_state.modal.motion == MOTION_MODE_CW_ARC)
                    || (gc_state.modal.motion == MOTION_MODE_CCW_ARC)) {
                    if (bit_istrue(gc_parser_flags, GC_PARSER_LASER_DISABLE)) {
                        gc_parser_flags |= GC_PARSER_LASER_FORCE_SYNC; // Change from G1/2/3 motion mode.
                    }
                } else {
                    // When changing to a G1 motion mode without axis words from a non-G1/2/3 motion mode.
                    if (bit_isfalse(gc_parser_flags, GC_PARSER_LASER_DISABLE)) {
                        gc_parser_flags |= GC_PARSER_LASER_FORCE_SYNC;
                    }
                }
            }
        }
    }

    // [0. Non-specific/common error-checks and miscellaneous setup]:
    // NOTE: If no line number is present, the value is zero.
    gc_state.line_number = gc_block.values.n;
    // [1. Comments feedback ]:  NOT SUPPORTED

    // [2. Set feed rate mode ]:
    gc_state.modal.feed_rate = gc_block.modal.feed_rate;
    if (gc_state.modal.feed_rate) { pl_data->condition |= PL_COND_FLAG_INVERSE_TIME; } // Set condition flag for motion use.

    // [3. Set feed rate ]:
    gc_state.feed_rate = gc_block.values.f; // Always copy this value. See feed rate error-checking.
    pl_data->feed_rate = gc_state.feed_rate; // Record data for motion use.

    // [4. Set spindle speed ]:
    if ((gc_state.spindle_speed != gc_block.values.s) || bit_istrue(gc_parser_flags, GC_PARSER_LASER_FORCE_SYNC)) {
        if (gc_state.modal.spindle != LASER_DISABLE) {
            if (bit_isfalse(gc_parser_flags, GC_PARSER_LASER_ISMOTION)) {
                if (bit_istrue(gc_parser_flags, GC_PARSER_LASER_DISABLE)) {
                    /* laser_sync(gc_state.modal.spindle, 0.0); */
                } else { /*laser_sync(gc_state.modal.spindle, gc_block.values.s);*/ }
            }
        }
        gc_state.spindle_speed = gc_block.values.s; // Update spindle speed state.
    }
    // NOTE: Pass zero spindle speed for all restricted laser motions.
    if (bit_isfalse(gc_parser_flags, GC_PARSER_LASER_DISABLE)) {
        pl_data->spindle_speed = gc_state.spindle_speed; // Record data for motion use.
    } // else { pl_data->spindle_speed = 0.0; } // Initialized as zero already.

    // [5. Select tool ]: NOT SUPPORTED
    // [6. Change tool ]: NOT SUPPORTED

    // [7. Spindle control ]:
    if (gc_state.modal.spindle != gc_block.modal.spindle) {
        // Update spindle control and apply spindle speed when enabling it in this block.
        // NOTE: All spindle state changes are synced, even in laser mode. Also, pl_data,
        // rather than gc_state, is used to manage laser state for non-laser motions.
//        laser_sync(gc_block.modal.spindle, pl_data->spindle_speed);
        gc_state.modal.spindle = gc_block.modal.spindle;
    }
    pl_data->condition |= gc_state.modal.spindle; // Set condition flag for motion use.

    // [8. Coolant control ]:

    // [9. Override control ]: NOT SUPPORTED. Always enabled. Except for a Grbl-only parking control.
    // [10. Dwell ]:
    if (gc_block.non_modal_command == NON_MODAL_DWELL) { mc_dwell(gc_block.values.p); }

    // [11. Set active plane ]:
    gc_state.modal.plane_select = gc_block.modal.plane_select;

    // [12. Set length units ]:
    gc_state.modal.units = gc_block.modal.units;

    // [13. Cutter radius compensation ]: G41/42 NOT SUPPORTED
    // gc_state.modal.cutter_comp = gc_block.modal.cutter_comp; // NOTE: Not needed since always disabled.

    // [14. Cutter length compensation ]: G43.1 and G49 supported. G43 NOT SUPPORTED.
    // NOTE: If G43 were supported, its operation wouldn't be any different from G43.1 in terms
    // of execution. The error-checking step would simply load the offset value into the correct
    // axis of the block XYZ value array.

    // [15. Coordinate system selection ]:

    // [16. Set path control mode ]: G61.1/G64 NOT SUPPORTED
    // gc_state.modal.control = gc_block.modal.control; // NOTE: Always default.

    // [17. Set distance mode ]:
    gc_state.modal.distance = gc_block.modal.distance;

    // [18. Set retract mode ]: NOT SUPPORTED

    // [19. Go to predefined position, Set G10, or Set axis offsets ]:

    // [20. Motion modes ]:
    // NOTE: Commands G10,G28,G30,G92 lock out and prevent axis words from use in motion modes.
    // Enter motion modes only if there are axis words or a motion mode cli word in the block.
    gc_state.modal.motion = gc_block.modal.motion;
    if (gc_state.modal.motion != MOTION_MODE_NONE) {
        if (axis_command == AXIS_COMMAND_MOTION_MODE) {
            uint8_t gc_update_pos = GC_UPDATE_POS_TARGET;
            if (gc_state.modal.motion == MOTION_MODE_LINEAR) {
                mc_line(gc_block.values.xyz, pl_data);
            } else if (gc_state.modal.motion == MOTION_MODE_SEEK) {
                pl_data->condition |= PL_COND_FLAG_RAPID_MOTION; // Set rapid motion condition flag.
                mc_line(gc_block.values.xyz, pl_data);
            } else if ((gc_state.modal.motion == MOTION_MODE_CW_ARC) ||
                       (gc_state.modal.motion == MOTION_MODE_CCW_ARC)) {
                mc_arc(gc_block.values.xyz, pl_data, gc_state.position, gc_block.values.ijk, gc_block.values.r,
                       axis_0, axis_1, axis_linear, (uint8_t) bit_istrue(gc_parser_flags, GC_PARSER_ARC_IS_CLOCKWISE));
            }

            // As far as the parser is concerned, the position is now == target. In reality the
            // motion control system might still be processing the action and the real tool position
            // in any intermediate location.
            if (gc_update_pos == GC_UPDATE_POS_TARGET) {
                memcpy(gc_state.position, gc_block.values.xyz,
                       sizeof(gc_block.values.xyz)); // gc_state.position[] = gc_block.values.xyz[]
            } else if (gc_update_pos == GC_UPDATE_POS_SYSTEM) {
                gc_sync_position(); // gc_state.position[] = sys_position
            } // == GC_UPDATE_POS_NONE
        }
    }

    // [21. Program flow ]:
    // M0,M1,M2,M30: Perform non-running program flow actions. During a program pause, the buffer may
    // refill and can only be resumed by the cycle start run-time cli.
    gc_state.modal.program_flow = gc_block.modal.program_flow;
    if (gc_state.modal.program_flow) {
        system_buffer_synchronize(); // Sync and finish all remaining buffered motions before moving on.
        if (gc_state.modal.program_flow == PROGRAM_FLOW_PAUSED) {
//            if (sys.state.mode == STATE_G_CODE_CHECK) {
//                system_command(SYS_FEED_HOLD, CMD_PRIORITY_NORMAL); // Use feed hold for program pause.
//            }
        } else { // == PROGRAM_FLOW_COMPLETED
            /* Upon program complete, only a subset of g-codes reset to certain defaults, according to
               LinuxCNC's program end descriptions and testing. Only modal groups [G-code 1,2,3,5,7,12]
               and [M-code 7,8,9] reset to [G1,G17,G90,G94,G40,G54,M5,M9,M48]. The remaining modal groups
               [G-code 4,6,8,10,13,14,15] and [M-code 4,5,6] and the modal words [F,S,T,H] do not reset. */
            gc_state.modal.motion = MOTION_MODE_LINEAR;
            gc_state.modal.plane_select = PLANE_SELECT_XY;
            gc_state.modal.distance = DISTANCE_MODE_ABSOLUTE;
            gc_state.modal.feed_rate = FEED_RATE_MODE_UNITS_PER_MIN;
            // gc_state.modal.cutter_comp = CUTTER_COMP_DISABLE; // Not supported.
            gc_state.modal.coord_select = 0; // G54
            gc_state.modal.spindle = LASER_DISABLE;
            gc_state.modal.coolant = COOLANT_DISABLE;
            // Execute coordinate change and spindle/coolant stop.
//            if (sys.state.mode != STATE_G_CODE_CHECK) {
//                laser_set_state(LASER_DISABLE, 0.0);
//            }
            message_feedback(MESSAGE_PROGRAM_END);
        }
        gc_state.modal.program_flow = PROGRAM_FLOW_RUNNING; // Reset program flow.
    }

    // Execute line, if in mdi_mode mode
    if (settings.cli.mdi_mode) {
        if (verbose) printf("_gc_execute_line: mdi mode wait for run\n");
        fsm_request(SYS_STATE_RUN);
#ifndef TARGET_BUILD
        //        stepgen_wake_up();
#endif // !TARGET_BUILD
    }

    return STATUS_OK;
}

/**
 * @brief G-Code Parser loop
 *
 * Pulls G-Code entries from the queue, and sends them to _gc_execute_line().
 *
 * @note Runs as Xenomai Alchemy Task with a priority of 40.
 */
static void _gc_loop() {
    ssize_t ret = 0;
    char *line;
    while ((ret = rt_queue_read(&rt_gc_queue, &line, sizeof(ssize_t), TM_INFINITE))) {
        if (ret != -ETIMEDOUT) {
            message_status(_gc_execute_line(line));
        }
    }
    fprintf(stderr, "_gc_loop: rt_queue_read exited %zd\n", ret);
}

/**
 * @brief Initialized G-Code parser
 * @return 0 on success, negative on error.
 */
ssize_t gc_init() {
    ssize_t ret = 0;
    memset(&gc_state, 0, sizeof(parser_state_t));
    if ((ret = rt_queue_create(&rt_gc_queue, "rt_gc_queue",
                               (sizeof(char) * CLI_LINE_LENGTH) * GCODE_QUEUE_SIZE, GCODE_QUEUE_SIZE, Q_PRIO)) < 0) {
        fprintf(stderr, "gc_init: rt_gc_queue returned %zd\n", ret);
        return ret;
    }
    if ((ret = rt_task_spawn(&rt_gc_loop_task, "rt_gc_loop_task", 0, 40, 0, &_gc_loop, 0)) < 0) {
        fprintf(stderr, "gc_init: rt_task_spawn for rt_gc_loop_task returned %zd\n", ret);
        return ret;
    }
    return ret;
}

/**
 * @brief Pre-procces G-Code line
 *
 * Grooms the incoming G-Code line to make it ready for execution.
 * @param line The incoming G-Code text
 * @param buf Where to store the process line
 */
void gc_process_line(char *line, char *buf) {
    uint8_t line_flags = 0;
    uint16_t char_cnt = 0;
    for (int i = 0; i < strlen(line); i++) {
        if (line_flags) {
            // Throw away all (except EOL) comment characters and overflow characters.
            if (line[i] == ')') {
                // End of '()' comment. Resume line allowed.
                if (line_flags & LINE_FLAG_COMMENT_PARENTHESES) { line_flags &= ~(LINE_FLAG_COMMENT_PARENTHESES); }
            }
        } else {
            if (line[i] <= ' ') {
                // Throw away whitepace and control characters
            } else if (line[i] == '/') {
                // Block delete NOT SUPPORTED. Ignore character.
                // NOTE: If supported, would simply need to check the system if block delete is enabled.
            } else if (line[i] == '(') {
                // Enable comments flag and ignore all characters until ')' or EOL.
                // NOTE: This doesn't follow the NIST definition exactly, but is good enough for now.
                // In the future, we could simply remove the items within the comments, but retain the
                // comment control characters, so that the g-code parser can error-check it.
                line_flags |= LINE_FLAG_COMMENT_PARENTHESES;
            } else if (line[i] == ';') {
                // NOTE: ';' comment to EOL is a LinuxCNC definition. Not NIST.
                line_flags |= LINE_FLAG_COMMENT_SEMICOLON;
                // TODO: Install '%' feature
                // } else if (c == '%') {
                // Program start-end percent sign NOT SUPPORTED.
                // NOTE: This maybe installed to tell Grbl when a program is running vs manual input,
                // where, during a program, the system auto-cycle start will continue to execute
                // everything until the next '%' sign. This will help fix resuming issues with certain
                // functions that empty the motion buffer to execute its task on-time.
            } else if (line[i] >= 'a' && line[i] <= 'z') { // Upcase lowercase
                buf[char_cnt++] = (char) (line[i] - 'a' + 'A');
            } else {
                buf[char_cnt++] = line[i];
            }
        }
    }
}

/**
 * @brief Add G-Code line to parser queue
 * @param line G-Code line to add
 * @return 0 on success, negative on error.
 */
ssize_t gc_queue_line(char *line) {
    ssize_t ret = 0;
    if ((ret = rt_queue_write(&rt_gc_queue, &line, sizeof(ssize_t), Q_NORMAL)) < 0) {
        fprintf(stderr, "gc_queue_line: rt_queue_write returned %zd\n", ret);
    }
    return ret;
}


/**
 * @brief Sets g-code parser position in mm.
 */
void gc_sync_position() {
    if (verbose) printf("gc_sync_position: init\n");
    system_convert_array_steps_to_mpos(gc_state.position, sys_position);
}
/** @} */
/** @} */
