/**
 * @file step_drv.h
 * @brief Step Driver for OpenGlow Control Board
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup hardware_step
 *
 * @{
 */

#ifndef OPENGLOW_CNC_STEP_DRV_H
#define OPENGLOW_CNC_STEP_DRV_H

#include "../common.h"

#define DRV_ATTR_PATH   "/sys/openglow/"    /*!< SYSFS root path for driver */

#define ATTR_NOT_SET     0x1FFFFFFFF        /*!< Value for unset attributes */

/**
 * @brief Available sysfs driver attributes
 * See Trinamic driver documentation for full descriptions
 */
enum DRV_ATTR {
    DRV_CHOPCONF,
    DRV_COOLCONF,
    DRV_DCCTRL,
    DRV_ENCM_CTRL,
    DRV_GCONF,
    DRV_GSTAT,
    DRV_IHOLD_RUN,
    DRV_IOIN,
    DRV_LOST_STEPS,
    DRV_MSCNT,
    DRV_MSCURACT,
    DRV_MSLUT0,
    DRV_MSLUT1,
    DRV_MSLUT2,
    DRV_MSLUT3,
    DRV_MSLUT4,
    DRV_MSLUT5,
    DRV_MSLUT6,
    DRV_MSLUT7,
    DRV_MSLUTSEL,
    DRV_MSLUTSTART,
    DRV_PWM_SCALE,
    DRV_PWMCONF,
    DRV_STATUS,
    DRV_STATUS_FULL,
    DRV_TCOOLTHRS,
    DRV_THIGH,
    DRV_TPOWERDOWN,
    DRV_TPWMTHRS,
    DRV_TSTEP,
    DRV_VDCMIN,
    DRV_XDIRECT,
    NUM_DRV_ATTR_PATHS
};

/**
 * @brief sysfs attribute rw permissions
 */
enum DRV_ATTR_RW {
    DRV_RONLY,
    DRV_WONLY,
    DRV_RW,
};

/**
 * @brief Available sysfs drivers
 */
enum DRV_ATTR_AXIS {
    DRV_X_AXIS,
    DRV_Y1_AXIS,
    DRV_Y2_AXIS,
    NUM_DRV_AXIS
};

/**
 * Driver sysfs attribute rw map
 */
struct attr_s {
    char *attr;
    int mode;
};

#define MSTEPS_256      0
#define MSTEPS_128      1
#define MSTEPS_64       2
#define MSTEPS_32       3
#define MSTEPS_16       4
#define MSTEPS_8        5
#define MSTEPS_4        6
#define MSTEPS_2        7
#define MSTEPS_FULL     8

#define FPWM_2_1024     0
#define FPWM_2_683      1
#define FPWM_2_512      2
#define FPWM_2_410      3


#define CHOPCONF_TOFF(x)            bits(0, x, 4)
#define CHOPCONF_HSTRT(x)           bits(4, x, 3)
#define CHOPCONF_HEND(x)            bits(7, x, 4)
#define CHOPCONF_FD3                bit(8)
#define CHOPCONF_DISFDCC            bit(12)
#define CHOPCONF_RNDTF              bit(13)
#define CHOPCONF_CHM                bit(14)
#define CHOPCONF_TBL(x)             bits(15, x, 2)
#define CHOPCONF_VSENSE             bit(17)
#define CHOPCONF_VHIGHFS            bit(18)
#define CHOPCONF_VHIGHCHM           bit(19)
#define CHOPCONF_SYNC(x)            bits(20, x, 4)
#define CHOPCONF_MRES(x)            bits(24, x, 4)
#define CHOPCONF_INTPOL             bit(28)
#define CHOPCONF_DEDGE              bit(29)
#define CHOPCONF_DISS2G             bit(30)

#define COOLCONF_SEMIN              bits(0, x, 4)
#define COOLCONF_SEUP               bits(5, x, 2)
#define COOLCONF_SEMAX              bits(8, x, 4)
#define COOLCONF_SEDN               bits(13, x, 2)
#define COOLCONF_SEIMIN             bit(15)
#define COOLCONF_SGT                bits(16, x, 7)
#define COOLCONF_SFILT              bit(24)

#define DCCTRL(x)                   bits(0, x, 24)

#define DRV_STATUS_SG_RESULT(x)     bits(0, 0x7FF, 10)
#define DRV_STATUS_FSACTIVE         bit(15)
#define DRV_STATUS_CSACTUAL(x)      bits(16, 0x1F, 5)
#define DRV_STATUS_STALLGUARD       bit(24)
#define DRV_STATUS_OT               bit(25)
#define DRV_STATUS_OTPW             bit(26)
#define DRV_STATUS_S2GA             bit(27)
#define DRV_STATUS_S2GB             bit(28)
#define DRV_STATUS_OLA              bit(29)
#define DRV_STATUS_OLB              bit(30)
#define DRV_STATUS_STST             bit(31)

#define ENCM_CTRL(x)                bits(0, x, 2)

#define GCONF_I_SCALE_ANALOG        bit(0)
#define GCONF_INTERNAL_RSENSE       bit(1)
#define GCONF_EN_PWM_MODE           bit(2)
#define GCONF_ENC_COMMUTATION       bit(3)
#define GCONF_SHAFT                 bit(4)
#define GCONF_DIAG0_ERROR           bit(5)
#define GCONF_DIAG0_OTPW            bit(6)
#define GCONF_DIAG0_STALL           bit(7)
#define GCONF_DIAG1_STALL           bit(8)
#define GCONF_DIAG1_INDEX           bit(9)
#define GCONF_DIAG1_ONSTATE         bit(10)
#define GCONF_DIAG1_STEPS_SKIPPED   bit(11)
#define GCONF_DIAG0_INT_PUSHPULL    bit(12)
#define GCONF_DIAG1_PUSHPULL        bit(13)
#define GCONF_SMALL_HYSTERESIS      bit(14)
#define GCONF_STOP_ENABLE           bit(15)
#define GCONF_DIRECT_MODE           bit(16)
#define GCONF_TEST_MODE             bit(17)

#define GSTAT_RESET                 bit(0)
#define GSTAT_ERR                   bit(1)
#define GSTAT_UV_CP                 bit(2)


#define IHOLD_IRUN_IHOLD(x)         bits(0, x, 5)
#define IHOLD_IRUN_IRUN(x)          bits(8, x, 5)
#define IHOLD_IRUN_IHOLDDELAY(x)    bits(16, x, 4)

#define IOIN_STEP                   bit(0)
#define IOIN_DIR                    bit(1)
#define IOIN_DCEN_CFG4              bit(2)
#define IOIN_DCIN_CFG5              bit(3)
#define IOIN_DRV_ENN_CFG6           bit(4)
#define IOIN_DCO                    bit(5)
#define IOIN_VERSION(x)             bits(24, 0xFF, 8)

#define LOST_STEPS(x)               bits(0, 0xFFFFF, 20)

#define MSLUT_0(x)                  bits(0, x, 32)
#define MSLUT_1(x)                  bits(0, x, 32)
#define MSLUT_2(x)                  bits(0, x, 32)
#define MSLUT_3(x)                  bits(0, x, 32)
#define MSLUT_4(x)                  bits(0, x, 32)
#define MSLUT_5(x)                  bits(0, x, 32)
#define MSLUT_6(x)                  bits(0, x, 32)
#define MSLUT_7(x)                  bits(0, x, 32)

#define MSLUTSEL_W0(x)              bits(0, x, 2)
#define MSLUTSEL_W1(x)              bits(2, x, 2)
#define MSLUTSEL_W2(x)              bits(4, x, 2)
#define MSLUTSEL_W3(x)              bits(6, x, 2)
#define MSLUTSEL_X1(x)              bits(8, x, 8)
#define MSLUTSEL_X2(x)              bits(16, x, 8)
#define MSLUTSEL_X3(x)              bits(24, x, 8)

#define MSLUTSTART(x)               bits(0, x, 32)

#define MSCNT(x)                    bits(0, 0xFFFFFFFF, 32)

#define MSCURACT(x)                 bits(0, 0xFFFFFFFF, 32)

#define PWMCONF_PWM_AMPL(x)         bits(0, x, 8)
#define PWMCONF_PWM_GRAD(x)         bits(8, x, 8)
#define PWMCONF_PWM_FREQ(x)         bits(16, x, 2)
#define PWMCONF_PWM_PWM_AUTOSCALE   bit(18)
#define PWMCONF_PWM_PWM_SYMMETRIC   bit(19)
#define PWMCONF_PWM_FREEWHEEL(x)    bits(20, x, 2)

#define PWM_SCALE(x)                bits(0, 0xFF, 8)

#define TCOOLTHRS(x)                bits(0, x, 20)

#define THIGH(x)                    bits(0, x, 20)

#define TPOWERDOWN(x)               bits(0, x, 8)

#define TPWMTHRS(x)                 bits(0, x, 20)

#define VDCMIN(x)                   bits(0, x, 23)

#define XDIRECT(x)                  bits(0, x, 32)


ssize_t step_drv_init(void);

#endif //OPENGLOW_CNC_STEP_DRV_H

/** @} */
