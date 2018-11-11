/**
 * @file openglow.h
 * @brief Interface to OpenGlow drivers
 *
 * Part of OpenGlow-CNC
 *
 * @copyright Copyright (c) 2018 Scott Wiederhold <s.e.wiederhold@gmail.com>
 *
 * SPDX-License-Identifier:    GPL-3.0-or-later
 *
 * @addtogroup hardware_og
 *
 * @{
 */

#ifndef OPENGLOW_CNC_OPENGLOW_H
#define OPENGLOW_CNC_OPENGLOW_H

#include "../common.h"

#define ATTR_ENABLE             "/sys/openglow/cnc/enable"
#define ATTR_DISABLE            "/sys/openglow/cnc/disable"
#define ATTR_STATE              "/sys/openglow/cnc/state"
#define ATTR_STEP_FREQ          "/sys/openglow/cnc/step_freq"
#define ATTR_RUN                "/sys/openglow/cnc/run"
#define ATTR_STOP               "/sys/openglow/cnc/stop"
#define ATTR_X_STEP             "/sys/openglow/cnc/x_step"
#define ATTR_Y_STEP             "/sys/openglow/cnc/y_step"
#define ATTR_Y1_STEP            "/sys/openglow/cnc/y1_step"
#define ATTR_Y2_STEP            "/sys/openglow/cnc/y2_step"
#define ATTR_Z_STEP             "/sys/openglow/cnc/z_step"

#define ATTR_AIR_ASSIST_PWM     "/sys/openglow/head_fans/air_assist_pwm"
#define ATTR_LENS_PURGE_PWM     "/sys/openglow/head_fans/lens_purge_pwm"
#define ATTR_AIR_ASSIST_TACH    "/sys/openglow/head_fans/lens_purge_tach"

#define ATTR_LID_LED_PWM        "/sys/openglow/leds/lid_led_pwm"
#define ATTR_BUTTON_RED_PWM     "/sys/openglow/leds/btn_red_led_pwm"
#define ATTR_BUTTON_BLUE_PWM    "/sys/openglow/leds/btn_blue_led_pwm"
#define ATTR_BUTTON_GREEN_PWM   "/sys/openglow/leds/btn_green_led_pwm"

#define ATTR_EXHAUST_FAN_PWM    "/sys/openglow/thermal/exhaust_pwm"
#define ATTR_EXHAUST_FAN_TACH   "/sys/openglow/thermal/exhaust_tach"
#define ATTR_INTAKE_FAN_PWM     "/sys/openglow/thermal/intake_pwm"
#define ATTR_INTAKE_FAN1_TACH   "/sys/openglow/thermal/intake_1_tach"
#define ATTR_NTAKE_FAN2_TACH    "/sys/openglow/thermal/intake_2_tach"
#define ATTR_WATER_HEATER_PWM   "/sys/openglow/thermal/water_heater_pwm"
#define ATTR_WATER_PUMP         "/sys/openglow/thermal/water_pump_on"

#define ATTR_PULSE              "/dev/openglow"
/**
 * @brief OpenGlow clear commands
 */
enum OG_CLEAR {
    OG_CLEAR_DATA = bit(0),     /*!< Clear step data buffer */
    OG_CLEAR_DATA_CNTR = bit(1),    /*!< Clear just data pointers */
    OG_CLEAR_POSITION = bit(2),     /*!< Clear step counters */
    OG_CLEAR_ALL = (OG_CLEAR_DATA | OG_CLEAR_DATA_CNTR | OG_CLEAR_POSITION),    /*!< Clear all */
};

/**
 * @brief LED color struct
 */
typedef struct led_color {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} led_color_t;

// Predefined LED colors
const led_color_t btn_led_red;
const led_color_t btn_led_green;
const led_color_t btn_led_blue;
const led_color_t btn_led_white;
const led_color_t btn_led_off;

void openglow_button_led(led_color_t color);

ssize_t openglow_clear(uint8_t cmd);

ssize_t openglow_init(void);

void openglow_lid_led(uint8_t brightness);

void openglow_pulse_close();

void openglow_pulse_flush();

ssize_t openglow_pulse_open();

ssize_t openglow_pulse_write(uint8_t data);

ssize_t openglow_read_attr_str(const char *attr, char *buf, size_t length);

ssize_t openglow_read_attr_uint32(const char *attr, uint32_t *value);

void openglow_reset(void);

ssize_t openglow_write_attr_str(const char *attr, char *buf);

ssize_t openglow_write_attr_uint64(const char *attr, uint64_t value);

#endif //OPENGLOW_CNC_OPENGLOW_H

/** @} */
