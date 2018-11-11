/**
 * @file step_drv.c
 * @brief Step Driver for OpenGlow Control Board
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup hardware
 * @{
 * @defgroup hardware_step Trinamic Step Drivers
 *
 * Trinamic Step Driver interface.
 * @{
 */

#include "../openglow-cnc.h"

/**
 * @brief Driver sysfs attribute to rw map
 */
static const struct attr_s drv_attr_map[NUM_DRV_ATTR_PATHS] = {
        [DRV_CHOPCONF]       = {"chopconf", DRV_RW},
        [DRV_COOLCONF]       = {"coolconf", DRV_WONLY},
        [DRV_DCCTRL]         = {"dcctrl", DRV_WONLY},
        [DRV_ENCM_CTRL]      = {"encm_ctrl", DRV_WONLY},
        [DRV_GCONF]          = {"gconf", DRV_RW},
        [DRV_GSTAT]          = {"gstat"},
        [DRV_IHOLD_RUN]      = {"ihold_irun", DRV_WONLY},
        [DRV_IOIN]           = {"ioin", DRV_RONLY},
        [DRV_LOST_STEPS]     = {"lost_steps", DRV_RONLY},
        [DRV_MSCNT]          = {"mscnt", DRV_RONLY},
        [DRV_MSCURACT]       = {"mscuract", DRV_RONLY},
        [DRV_MSLUT0]         = {"mslut0", DRV_WONLY},
        [DRV_MSLUT1]         = {"mslut1", DRV_WONLY},
        [DRV_MSLUT2]         = {"mslut2", DRV_WONLY},
        [DRV_MSLUT3]         = {"mslut3", DRV_WONLY},
        [DRV_MSLUT4]         = {"mslut4", DRV_WONLY},
        [DRV_MSLUT5]         = {"mslut5", DRV_WONLY},
        [DRV_MSLUT6]         = {"mslut6", DRV_WONLY},
        [DRV_MSLUT7]         = {"mslut7", DRV_WONLY},
        [DRV_MSLUTSEL]       = {"mslutsel", DRV_WONLY},
        [DRV_MSLUTSTART]     = {"mslutstart", DRV_WONLY},
        [DRV_PWM_SCALE]      = {"pwm_scale", DRV_RONLY},
        [DRV_PWMCONF]        = {"pwmconf", DRV_WONLY},
        [DRV_STATUS]         = {"drv_status", DRV_RONLY},
        [DRV_STATUS_FULL]    = {"status", DRV_RONLY},
        [DRV_TCOOLTHRS]      = {"tcoolthrs", DRV_WONLY},
        [DRV_THIGH]          = {"thigh", DRV_WONLY},
        [DRV_TPOWERDOWN]     = {"tpowerdown", DRV_WONLY},
        [DRV_TPWMTHRS]       = {"tpwmthrs", DRV_WONLY},
        [DRV_TSTEP]          = {"tstep", DRV_RONLY},
        [DRV_VDCMIN]         = {"vdcmin", DRV_WONLY},
        [DRV_XDIRECT]        = {"xdirect", DRV_RW},
};

/**
 * @brief Axis to sysfs directory map
 */
static const char *axis_attr[NUM_DRV_AXIS] = {
        [DRV_X_AXIS]         = "x-axis",
        [DRV_Y1_AXIS]        = "y1-axis",
        [DRV_Y2_AXIS]        = "y2-axis",
};

/**
 * Axis driver settings
 */
static const uint64_t axis_settings[NUM_DRV_AXIS][NUM_DRV_ATTR_PATHS] = {
        // X AXIS SETTINGS
        [DRV_X_AXIS] = {
                [DRV_CHOPCONF]       =
                        CHOPCONF_TOFF(3) | CHOPCONF_HSTRT(4) | CHOPCONF_HEND(1) | CHOPCONF_TBL(2) |
                        CHOPCONF_INTPOL | CHOPCONF_MRES(MSTEPS_16),
                [DRV_COOLCONF]       = ATTR_NOT_SET,
                [DRV_DCCTRL]         = ATTR_NOT_SET,
                [DRV_ENCM_CTRL]      = ATTR_NOT_SET,
                [DRV_GCONF]          = GCONF_EN_PWM_MODE,
                [DRV_IHOLD_RUN]      =
                        IHOLD_IRUN_IHOLD(5) | IHOLD_IRUN_IRUN(5) | IHOLD_IRUN_IHOLDDELAY(6),
                [DRV_MSLUT0]         = ATTR_NOT_SET,
                [DRV_MSLUT1]         = ATTR_NOT_SET,
                [DRV_MSLUT2]         = ATTR_NOT_SET,
                [DRV_MSLUT3]         = ATTR_NOT_SET,
                [DRV_MSLUT4]         = ATTR_NOT_SET,
                [DRV_MSLUT5]         = ATTR_NOT_SET,
                [DRV_MSLUT6]         = ATTR_NOT_SET,
                [DRV_MSLUT7]         = ATTR_NOT_SET,
                [DRV_MSLUTSEL]       = ATTR_NOT_SET,
                [DRV_MSLUTSTART]     = ATTR_NOT_SET,
                [DRV_PWMCONF]        =
                        PWMCONF_PWM_AMPL(200) | PWMCONF_PWM_GRAD(1) |
                        PWMCONF_PWM_FREQ(FPWM_2_1024) | PWMCONF_PWM_PWM_AUTOSCALE,
                [DRV_TCOOLTHRS]      = ATTR_NOT_SET,
                [DRV_THIGH]          = ATTR_NOT_SET,
                [DRV_TPOWERDOWN]     = TPOWERDOWN(10),
                [DRV_TPWMTHRS]       = TPWMTHRS(500),
                [DRV_VDCMIN]         = ATTR_NOT_SET,
                [DRV_XDIRECT]        = ATTR_NOT_SET
        },
        // Y1 AXIS SETTINGS
        [DRV_Y1_AXIS] = {
                [DRV_CHOPCONF]       =
                        CHOPCONF_TOFF(3) | CHOPCONF_HSTRT(4) | CHOPCONF_HEND(1) | CHOPCONF_TBL(2) |
                        CHOPCONF_INTPOL | CHOPCONF_MRES(MSTEPS_16),
                [DRV_COOLCONF]       = ATTR_NOT_SET,
                [DRV_DCCTRL]         = ATTR_NOT_SET,
                [DRV_ENCM_CTRL]      = ATTR_NOT_SET,
                [DRV_GCONF]          = GCONF_EN_PWM_MODE,
                [DRV_IHOLD_RUN]      =
                        IHOLD_IRUN_IHOLD(5) | IHOLD_IRUN_IRUN(5) | IHOLD_IRUN_IHOLDDELAY(6),
                [DRV_MSLUT0]         = ATTR_NOT_SET,
                [DRV_MSLUT1]         = ATTR_NOT_SET,
                [DRV_MSLUT2]         = ATTR_NOT_SET,
                [DRV_MSLUT3]         = ATTR_NOT_SET,
                [DRV_MSLUT4]         = ATTR_NOT_SET,
                [DRV_MSLUT5]         = ATTR_NOT_SET,
                [DRV_MSLUT6]         = ATTR_NOT_SET,
                [DRV_MSLUT7]         = ATTR_NOT_SET,
                [DRV_MSLUTSEL]       = ATTR_NOT_SET,
                [DRV_MSLUTSTART]     = ATTR_NOT_SET,
                [DRV_PWMCONF]        =
                        PWMCONF_PWM_AMPL(200) | PWMCONF_PWM_GRAD(1) |
                        PWMCONF_PWM_FREQ(FPWM_2_1024) | PWMCONF_PWM_PWM_AUTOSCALE,
                [DRV_TCOOLTHRS]      = ATTR_NOT_SET,
                [DRV_THIGH]          = ATTR_NOT_SET,
                [DRV_TPOWERDOWN]     = TPOWERDOWN(10),
                [DRV_TPWMTHRS]       = TPWMTHRS(500),
                [DRV_VDCMIN]         = ATTR_NOT_SET,
                [DRV_XDIRECT]        = ATTR_NOT_SET,
        },
        // Y2 AXIS SETTINGS
        [DRV_Y2_AXIS] = {
                [DRV_CHOPCONF]       =
                        CHOPCONF_TOFF(3) | CHOPCONF_HSTRT(4) | CHOPCONF_HEND(1) |
                        CHOPCONF_TBL(2) | CHOPCONF_INTPOL | CHOPCONF_MRES(MSTEPS_16),
                [DRV_COOLCONF]       = ATTR_NOT_SET,
                [DRV_DCCTRL]         = ATTR_NOT_SET,
                [DRV_ENCM_CTRL]      = ATTR_NOT_SET,
                [DRV_GCONF]          = GCONF_EN_PWM_MODE,
                [DRV_IHOLD_RUN]      =
                        IHOLD_IRUN_IHOLD(5) | IHOLD_IRUN_IRUN(5) | IHOLD_IRUN_IHOLDDELAY(6),
                [DRV_MSLUT0]         = ATTR_NOT_SET,
                [DRV_MSLUT1]         = ATTR_NOT_SET,
                [DRV_MSLUT2]         = ATTR_NOT_SET,
                [DRV_MSLUT3]         = ATTR_NOT_SET,
                [DRV_MSLUT4]         = ATTR_NOT_SET,
                [DRV_MSLUT5]         = ATTR_NOT_SET,
                [DRV_MSLUT6]         = ATTR_NOT_SET,
                [DRV_MSLUT7]         = ATTR_NOT_SET,
                [DRV_MSLUTSEL]       = ATTR_NOT_SET,
                [DRV_MSLUTSTART]     = ATTR_NOT_SET,
                [DRV_PWMCONF]        =
                        PWMCONF_PWM_AMPL(200) | PWMCONF_PWM_GRAD(1) |
                        PWMCONF_PWM_FREQ(FPWM_2_1024) | PWMCONF_PWM_PWM_AUTOSCALE,
                [DRV_TCOOLTHRS]      = ATTR_NOT_SET,
                [DRV_THIGH]          = ATTR_NOT_SET,
                [DRV_TPOWERDOWN]     = TPOWERDOWN(10),
                [DRV_TPWMTHRS]       = TPWMTHRS(500),
                [DRV_VDCMIN]         = ATTR_NOT_SET,
                [DRV_XDIRECT]        = ATTR_NOT_SET,
        }
};

/**
 * @brief Initialize Stepper Drivers
 * Initializes and sends configurations to step drivers
 *
 * @return 0 on success, negative otherwise
 */
ssize_t step_drv_init(void) {
#ifdef TARGET_BUILD
    char buf_val[32], buf_cmp[32], buf_attr[64], buf_err[256];
    ssize_t ret;

    // Make sure stepper drivers are up and going
    int axis_stat[NUM_DRV_AXIS] = {0};
    int count = 10, axis = 0, cmp = 0;
    while (count > 0) {
        for (axis = 0; axis < NUM_DRV_AXIS; axis++) {
            if (!axis_stat[axis]) {
                sprintf(buf_attr, "%s%s/%s", DRV_ATTR_PATH, axis_attr[axis], drv_attr_map[DRV_IOIN].attr);
                ret = openglow_read_attr_str(buf_attr, buf_cmp, 31); // Read the attr
                if(ret < 0) {
                    sprintf(buf_err, "step_drv_init: open %s failed for status", buf_attr);
                    perror(buf_err);
                    return ret;
                }
                cmp = (int)strtoul(buf_cmp, NULL, 0);
                if (bit_false(cmp ,IOIN_DRV_ENN_CFG6)) {
                    axis_stat[axis] = 1;
                }
            }
        }
        cmp = 0;
        for (axis = 0; axis < NUM_DRV_AXIS; axis++) {
            cmp += axis_stat[axis];
        }
        if (cmp == NUM_DRV_AXIS) {
            count = 0;
        } else {
            usleep(100);
            count--;
            if (count <= 0) {
                perror("step_drv_init: timeout waiting for driver ready state");
                return -1;
            }
        }
    }

    // Send initial settings
    uint32_t cmp_val = 0;
    for (axis = 0; axis < NUM_DRV_AXIS; axis++) {
        for (int attr = 0; attr < NUM_DRV_ATTR_PATHS; attr++) {
            if ((drv_attr_map[attr].mode != DRV_RONLY) & (axis_settings[axis][attr] != ATTR_NOT_SET)) {
                sprintf(buf_attr, "%s%s/%s", DRV_ATTR_PATH, axis_attr[axis], drv_attr_map[attr].attr);
                ret = openglow_write_attr_uint64(buf_attr, axis_settings[axis][attr]); // Set the attr
                if(ret < 0) {
                    sprintf(buf_err, "step_drv_init: write %s to %s failed", buf_val, buf_attr);
                    perror(buf_err);
                    return ret;
                }
                if (drv_attr_map[attr].mode == DRV_RW) { // If the attr is R/W, we'll read it back to verify
                    ret = openglow_read_attr_uint32(buf_attr, &cmp_val); // Read the attr
                    if(ret < 0) {
                        sprintf(buf_err, "step_drv_init: open %s failed for verify", buf_attr);
                        perror(buf_err);
                        return ret;
                    }
                    if (cmp_val != axis_settings[axis][attr]) {
                        sprintf(buf_err, "step_drv_init: verify %s failed: read %s, expected %s",
                                buf_attr, buf_cmp, buf_val);
                        perror(buf_err);
                        return -1;
                    }
                }
            }
        }
    }
#endif
    return 0;
};

/** @} */
/** @} */
