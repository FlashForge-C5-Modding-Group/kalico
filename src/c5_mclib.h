// Creator 5 mainBoardGD portable motor controller
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#ifndef __C5_MCLIB_H
#define __C5_MCLIB_H

#include <stdint.h>

struct c5_mclib_pi {
    float kp, ki;
    float integral_min, integral_max;
    float output_min, output_max;
    float error, integral, output;
};

struct c5_mclib_observer {
    float a, b;
    float injection_magnitude, error_threshold, inner_slope;
    float filter_coefficient, compensation_denominator;
    float estimated_current_a, estimated_current_b;
    float injection_a, injection_b;
    float filtered_injection_a, filtered_injection_b;
    float raw_angle, previous_raw_angle, compensation;
    float instantaneous_speed, filtered_speed, scaled_speed;
    float angle;
};

struct c5_mclib_motor {
    uint32_t last_step, last_period, elapsed;
    uint32_t stop_timeout, hold_delay, ramp_ticks;
    uint32_t crossover_threshold, slow_threshold;
    float command_angle, stall_threshold;
    float resistance, inductance, motor_constant;
    float run_current, hold_current, active_current, decay_increment;
    float previous_voltage_a, previous_voltage_b;
    float measured_d, measured_q;
    float reference_d, reference_q;
    float requested_d, requested_q;
    float filtered_torque, filtered_angle_error;
    float resonance_amplitude[3];
    float resonance_phase_forward[3];
    float resonance_phase_reverse[3];
    struct c5_mclib_observer observer;
    struct c5_mclib_pi d_pi, q_pi, outer_pi;
    uint16_t phase, phase_increment, exponent;
    uint8_t axis, interpolation, direction;
    uint8_t mode, fast_mode, hold_transition, slow;
    uint8_t quarter_count, stall;
};

struct c5_mclib_acq {
    int32_t sum_a, sum_b;
    int16_t offset_a, offset_b;
    uint16_t count;
    uint8_t calibrating, signs;
};

struct c5_mclib_output {
    uint32_t compare[8];
    uint8_t signs;
};

void c5_mclib_init(struct c5_mclib_motor *m, uint8_t axis);
void c5_mclib_configure(struct c5_mclib_motor *m,
                        uint32_t rs, uint32_t ls, uint32_t km);
void c5_mclib_microstep(struct c5_mclib_motor *m,
                        uint8_t interpolate, uint16_t exponent);
void c5_mclib_current(struct c5_mclib_motor *m,
                      uint32_t run_ma, uint32_t hold_ma);
void c5_mclib_stall_threshold(struct c5_mclib_motor *m,
                              uint32_t threshold);
void c5_mclib_resonance(struct c5_mclib_motor *m,
                        uint8_t tdx, uint32_t amp, uint32_t phase1,
                        uint32_t phase2);
void c5_mclib_direction(struct c5_mclib_motor *m, uint8_t direction);
void c5_mclib_step(struct c5_mclib_motor *m, uint32_t now);
void c5_mclib_enable(struct c5_mclib_motor *m, uint32_t now);
void c5_mclib_disable(struct c5_mclib_motor *m);
uint8_t c5_mclib_update(struct c5_mclib_motor *m, uint32_t now,
                        float ia, float ib, struct c5_mclib_output *out);
uint8_t c5_mclib_stalled(const struct c5_mclib_motor *m);
void c5_mclib_acq_init(struct c5_mclib_acq *a);
uint8_t c5_mclib_acquire(struct c5_mclib_acq *a,
                         int16_t raw0, int16_t raw1, float *ia, float *ib);
void c5_mclib_acq_polarity(struct c5_mclib_acq *a, uint8_t signs);

#endif // c5_mclib.h
