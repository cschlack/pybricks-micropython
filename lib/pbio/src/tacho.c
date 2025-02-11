// SPDX-License-Identifier: MIT
// Copyright (c) 2018-2020 The Pybricks Authors

#include <pbio/config.h>

#if PBIO_CONFIG_TACHO

#include <inttypes.h>

#include <pbio/math.h>
#include <pbio/port.h>
#include <pbio/tacho.h>

struct _pbio_tacho_t {
    pbio_direction_t direction;
    int32_t offset;
    fix16_t counts_per_degree;
    pbdrv_counter_dev_t *counter;
};

static pbio_tacho_t tachos[PBDRV_CONFIG_NUM_MOTOR_CONTROLLER];

static pbio_error_t pbio_tacho_reset_count(pbio_tacho_t *tacho, int32_t reset_count) {
    int32_t count;
    pbio_error_t err;

    // See pbio_tacho_get_count for a definition of count. Here we want to set
    // the offset such that afterwards, count will equal reset_count. So:
    // new_offset = raw_count * direction - reset_count
    // which, given the definition of pbio_tacho_get_count, can be written as:
    // new_offset = pbio_tacho_get_count + old_offset - reset_count

    // First get the counter value with the existing offset subtracted.
    err = pbio_tacho_get_count(tacho, &count);
    if (err != PBIO_SUCCESS) {
        return err;
    }

    // Calculate the new offset
    tacho->offset = count + tacho->offset - reset_count;

    return PBIO_SUCCESS;
}

static pbio_error_t pbio_tacho_reset_count_to_abs(pbio_tacho_t *tacho, int32_t *abs_count) {

    pbio_error_t err = pbdrv_counter_get_abs_count(tacho->counter, abs_count);
    if (err != PBIO_SUCCESS) {
        return err;
    }

    if (tacho->direction == PBIO_DIRECTION_COUNTERCLOCKWISE) {
        *abs_count = -*abs_count;
    }

    return pbio_tacho_reset_count(tacho, *abs_count);
}

pbio_error_t pbio_tacho_setup(pbio_tacho_t *tacho, pbio_direction_t direction, fix16_t gear_ratio, bool reset_angle) {
    // Assert that scaling factors are positive
    if (gear_ratio < 0) {
        return PBIO_ERROR_INVALID_ARG;
    }
    // Get overal ratio from counts to output variable, including gear train
    tacho->counts_per_degree = fix16_mul(F16C(PBDRV_CONFIG_COUNTER_COUNTS_PER_DEGREE, 0), gear_ratio);

    // Configure direction
    tacho->direction = direction;

    // If there's no need to reset the angle, we are done here.
    if (!reset_angle) {
        // We still do one test read to ensure a tacho exists.
        int32_t test_count;
        return pbio_tacho_get_count(tacho, &test_count);
    }

    // Reset count to absolute value if supported
    int32_t abs_count;
    pbio_error_t err = pbio_tacho_reset_count_to_abs(tacho, &abs_count);
    if (err == PBIO_ERROR_NOT_SUPPORTED) {
        // If not available, set it to 0
        err = pbio_tacho_reset_count(tacho, 0);
    }
    return err;
}

pbio_error_t pbio_tacho_get_tacho(pbio_port_id_t port, pbio_tacho_t **tacho) {
    // Validate port
    if (port < PBDRV_CONFIG_FIRST_MOTOR_PORT || port > PBDRV_CONFIG_LAST_MOTOR_PORT) {
        return PBIO_ERROR_INVALID_PORT;
    }

    // Get pointer to tacho.
    *tacho = &tachos[port - PBDRV_CONFIG_FIRST_MOTOR_PORT];

    // Get counter device
    return pbdrv_counter_get_dev(port - PBDRV_CONFIG_FIRST_MOTOR_PORT, &((*tacho)->counter));
}

pbio_error_t pbio_tacho_get_count(pbio_tacho_t *tacho, int32_t *count) {

    // In the tacho module, count is determined as:
    // count = raw_count * direction - offset
    // This is done in three steps below.
    pbio_error_t err;

    // Get raw counter value.
    err = pbdrv_counter_get_count(tacho->counter, count);
    if (err != PBIO_SUCCESS) {
        return err;
    }

    // Set sign.
    if (tacho->direction == PBIO_DIRECTION_COUNTERCLOCKWISE) {
        *count = -*count;
    }

    // Subtract offset.
    *count -= tacho->offset;

    return PBIO_SUCCESS;
}



pbio_error_t pbio_tacho_get_angle(pbio_tacho_t *tacho, int32_t *angle) {
    int32_t encoder_count;
    pbio_error_t err;

    err = pbio_tacho_get_count(tacho, &encoder_count);
    if (err != PBIO_SUCCESS) {
        return err;
    }

    *angle = pbio_math_div_i32_fix16(encoder_count, tacho->counts_per_degree);

    return PBIO_SUCCESS;
}

pbio_error_t pbio_tacho_reset_angle(pbio_tacho_t *tacho, int32_t *reset_angle, bool reset_to_abs) {
    if (reset_to_abs) {
        pbio_error_t err = pbio_tacho_reset_count_to_abs(tacho, reset_angle);
        *reset_angle = pbio_math_div_i32_fix16(*reset_angle, tacho->counts_per_degree);
        return err;
    } else {
        return pbio_tacho_reset_count(tacho, pbio_math_mul_i32_fix16(*reset_angle, tacho->counts_per_degree));
    }
}

pbio_error_t pbio_tacho_get_rate(pbio_tacho_t *tacho, int32_t *rate) {
    pbio_error_t err;

    err = pbdrv_counter_get_rate(tacho->counter, rate);
    if (err != PBIO_SUCCESS) {
        return err;
    }

    if (tacho->direction == PBIO_DIRECTION_COUNTERCLOCKWISE) {
        *rate = -*rate;
    }

    return PBIO_SUCCESS;
}

pbio_error_t pbio_tacho_get_angular_rate(pbio_tacho_t *tacho, int32_t *angular_rate) {
    int32_t encoder_rate;
    pbio_error_t err;

    err = pbio_tacho_get_rate(tacho, &encoder_rate);
    if (err != PBIO_SUCCESS) {
        return err;
    }

    *angular_rate = pbio_math_div_i32_fix16(encoder_rate, tacho->counts_per_degree);

    return PBIO_SUCCESS;
}

#endif // PBIO_CONFIG_TACHO
