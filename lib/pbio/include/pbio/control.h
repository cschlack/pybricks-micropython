// SPDX-License-Identifier: MIT
// Copyright (c) 2019 Laurens Valk

#ifndef _PBIO_CONTROL_H_
#define _PBIO_CONTROL_H_

#include <stdint.h>
#include <stdio.h>

#include <pbdrv/config.h>
#include <pbdrv/motor.h>

#include <pbio/error.h>
#include <pbio/port.h>
#include <pbio/motorref.h>

#include <pbio/iodev.h>

/**
 * Control states
 */
typedef enum {
    /* Passive control statuses: No PID Control Active */
    PBIO_CONTROL_COASTING,
    PBIO_CONTROL_BRAKING,
    PBIO_CONTROL_USRDUTY,
    PBIO_CONTROL_ERRORED,
    /* Active control statuses: PID Control Active */   
    PBIO_CONTROL_TRACKING,      /**< Motor is tracking a position or holding position after completing command */
    PBIO_CONTROL_RUNNING_TIME,  /**< Motor is executing angle based maneuver by doing speed/position control */
    PBIO_CONTROL_RUNNING_ANGLE, /**< Motor is executing time based  maneuver by doing speed/position control */
} pbio_control_state_t;

/**
 * Control settings
 */
typedef struct _pbio_control_settings_t {
    int32_t stall_rate_limit;       /**< If this speed cannnot be reached even with the maximum duty value (equal to stall_torque_limit), the motor is considered to be stalled */
    int32_t stall_time;             /**< Minimum stall time before the run_stalled action completes */
    int32_t max_rate;               /**< Soft limit on the reference encoder rate in all run commands */
    int32_t rate_tolerance;         /**< Allowed deviation (counts/s) from target speed. Hence, if speed target is zero, any speed below this tolerance is considered to be standstill. */
    int32_t count_tolerance;        /**< Allowed deviation (counts) from target before motion is considered complete */
    int32_t abs_acceleration;       /**< Encoder acceleration/deceleration rate when beginning to move or stopping. Positive value in counts per second per second */
    int32_t tight_loop_time;        /**< When a run function is called twice in this interval, assume that the user is doing their own speed control.  */
    int16_t pid_kp;                 /**< Proportional position control constant (and integral speed control constant) */
    int16_t pid_ki;                 /**< Integral position control constant */
    int16_t pid_kd;                 /**< Derivative position control constant (and proportional speed control constant) */
} pbio_control_settings_t;


typedef enum {
    /**< Motor is not stalled */
    STALLED_NONE = 0x00,
    /**< The proportional duty control term is larger than the maximum and still the motor moves slower than specified limit */
    STALLED_PROPORTIONAL = 0x01,
    /**< The integral duty control term is larger than the maximum and still the motor moves slower than specified limit */
    STALLED_INTEGRAL = 0x02,
} stalled_status_t;

/**
 * Motor PID control status
 */
typedef struct _pbio_angular_control_status_t {
    bool ref_time_running;         /**< Whether the time at which the reference is evaluated is progressing (true) or paused (false) */
    count_t err_integral;          /**< Integral of position error */
    count_t count_err_prev;        /**< Position error in the previous control iteration */
    ustime_t time_prev;            /**< Time at the previous control iteration */
    ustime_t time_paused;          /**< The amount of time the speed integrator has spent paused */
    ustime_t time_stopped;         /**< Time at which the time was paused */
} pbio_angular_control_status_t;

typedef struct _pbio_timed_control_status_t {
    bool speed_integrator_running;   /**< Whether the speed integrator is active (true) or paused to prevent windup (false) */
    count_t speed_integrator;        /**< State of the speed integrator */
    ustime_t integrator_time_stopped;/**< Time at which the speed integrator last stopped */
    count_t integrator_ref_start;    /**< Integrated speed value prior to enabling integrator */
    count_t integrator_start;        /**< Integrated reference speed value prior to enabling integrator */
} pbio_timed_control_status_t;

#endif // _PBIO_CONTROL_H_
