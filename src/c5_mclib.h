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
    float injection_scale;
    float raw_angle, previous_raw_angle, compensation;
    float instantaneous_speed, filtered_speed, scaled_speed;
    float angle;
};

// Motion health since the last c5_mclib_take_health(): stall edges,
// electrical cycles lost or gained (a cycle is four full steps), the
// torque range, and how far the angle error drifted from where it was when
// the stall detector armed.
struct c5_mclib_health {
    float torque_min, torque_max, error_min, error_max, last_error;
    float drift;
    int32_t slips, level;
    uint32_t slip_total, stall_count, samples, tracked;
    uint8_t last_valid, stall_prev;
};

// Control loop duration bookkeeping: max and total cycles spent in the
// control update per axis, over the samples counted since the last
// c5_mclib_take_timing(). The caller measures the time around
// c5_mclib_update(); this only accumulates.
struct c5_mclib_timing {
    uint32_t max, total, samples;
    float d_integral_sum;
    uint32_t d_integral_samples;
    // squared current error while holding still (current sense noise)
    float hold_noise_sum;
    uint32_t hold_noise_samples;
};


#define C5_RIPPLE_HARMONICS 8
#define C5_ANALYZE_HARMONICS 8
#define C5_RIPPLE_POINTS 6

struct c5_mclib_motor {
    uint32_t last_step, last_period, elapsed, next_step;
    uint32_t stop_timeout, hold_delay, ramp_ticks;
    uint32_t crossover_threshold, slow_threshold;
    uint32_t crossover_hysteresis, damping_threshold;
    float feedforward_gain;
    // current PI gains at 100% (1 kHz bandwidth), for c5_mclib_pi_scale()
    float pi_kp_base, pi_ki_base;
    // slow part of the load angle, taken out before its harmonics are summed
    float load_angle_lp;
    // load angle harmonics against the commanded angle, per direction
    float harm_cos[2][C5_ANALYZE_HARMONICS], harm_sin[2][C5_ANALYZE_HARMONICS];
    uint32_t harm_count[2];
    uint8_t harm_armed;
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
    // Torque ripple correction for harmonic k+1 of the electrical angle,
    // per direction: a table of electrical frequency (Hz) against the
    // correction current (A, as sin and cos parts), while running
    struct c5_mclib_ripple_table {
        float freq[C5_RIPPLE_POINTS], sin_part[C5_RIPPLE_POINTS];
        float cos_part[C5_RIPPLE_POINTS];
        uint8_t count;
    } ripple[C5_RIPPLE_HARMONICS][2];
    uint32_t step_period;
    uint8_t ripple_active;
    struct c5_mclib_observer observer;
    struct c5_mclib_pi d_pi, q_pi, outer_pi;
    uint16_t phase, phase_increment, exponent;
    uint8_t axis, interpolation, direction;
    uint8_t mode, fast_mode, hold_transition, slow;
    uint8_t quarter_count, stall, slip_mode, next_state;
    // Observer back-EMF estimate while (nearly) still, in the command frame:
    // the bridge's dead time and the resistance mismatch. Taken out of the
    // estimate to follow skipped steps below the fast-move speed.
    float emf_base_d, emf_base_q, emf_resid_d, emf_resid_q;
    uint16_t emf_learned, skipped;
    uint8_t track_source;
    struct c5_mclib_health health;
    struct c5_mclib_timing timing;
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
// Point index of count for harmonic (1..C5_RIPPLE_HARMONICS) ripple
// correction in direction (0 reverse, 1 forward): electrical frequency in
// millihertz, amplitude in milliamps, phase in milliradians. Points go in
// rising frequency; count 0 clears the table.
void c5_mclib_ripple(struct c5_mclib_motor *m, uint8_t harmonic,
                     uint8_t direction, uint8_t index, uint8_t count,
                     uint32_t freq_mhz, int32_t amp_ma, int32_t phase_mrad);
void c5_mclib_direction(struct c5_mclib_motor *m, uint8_t direction);
void c5_mclib_step(struct c5_mclib_motor *m, uint32_t now);
// When the step queue knows its next step: C5_NEXT_AT with its motor time,
// or C5_NEXT_NONE when no step follows. A step resets it to C5_NEXT_UNKNOWN,
// where interpolation assumes the last step period repeats.
#define C5_NEXT_UNKNOWN 0u
#define C5_NEXT_AT 1u
#define C5_NEXT_NONE 2u
void c5_mclib_next_step(struct c5_mclib_motor *m, uint32_t when,
                        uint8_t state);
void c5_mclib_enable(struct c5_mclib_motor *m, uint32_t now);
void c5_mclib_disable(struct c5_mclib_motor *m);
uint8_t c5_mclib_update(struct c5_mclib_motor *m, uint32_t now,
                        float ia, float ib, struct c5_mclib_output *out);
uint8_t c5_mclib_stalled(const struct c5_mclib_motor *m);
void c5_mclib_take_health(struct c5_mclib_motor *m,
                          struct c5_mclib_health *out);
void c5_mclib_note_timing(struct c5_mclib_motor *m, uint32_t cycles);
void c5_mclib_take_timing(struct c5_mclib_motor *m,
                          struct c5_mclib_timing *out);
// Observer high-frequency injection scale in percent (100 = stock).
// Lower values reduce the injected correction voltage; the angle
// estimate degrades, so this is for bench experiments only.
void c5_mclib_injection_scale(struct c5_mclib_motor *m, uint32_t percent);
// d-axis back-EMF and cross-coupling feedforward in percent (0 = off)
void c5_mclib_feedforward(struct c5_mclib_motor *m, uint32_t percent);
// Current PI gains in percent of the stock 1 kHz bandwidth (100 = stock)
void c5_mclib_pi_scale(struct c5_mclib_motor *m, uint32_t percent);
// Load angle harmonic analysis: arm clears and starts it; the sums are read
// with c5_mclib_take_harmonics
void c5_mclib_arm_harmonics(struct c5_mclib_motor *m);
void c5_mclib_take_harmonics(struct c5_mclib_motor *m, uint8_t dir,
                             float *cosines, float *sines, uint32_t *count);
// Slip tracking: C5_SLIP_MOVING only in fast moves (with the stall
// detector), C5_SLIP_HOLDING also while holding still, C5_SLIP_ALWAYS
// whenever the motor is enabled.
#define C5_SLIP_MOVING 0u
#define C5_SLIP_HOLDING 1u
#define C5_SLIP_ALWAYS 2u
void c5_mclib_slip_mode(struct c5_mclib_motor *m, uint8_t mode);
// TRACKING: C5_TRACK_OFF restarts the drift on the next sample,
// C5_TRACK_ON follows it, C5_TRACK_GAP skips this sample (the angle estimate
// can't be trusted) and keeps the reference.
#define C5_TRACK_OFF 0u
#define C5_TRACK_ON 1u
#define C5_TRACK_GAP 2u
void c5_mclib_track_health(struct c5_mclib_motor *m, float error,
                           uint8_t armed, uint8_t tracking);
void c5_mclib_acq_init(struct c5_mclib_acq *a);
uint8_t c5_mclib_acquire(struct c5_mclib_acq *a,
                         int16_t raw0, int16_t raw1, float *ia, float *ib);
void c5_mclib_acq_polarity(struct c5_mclib_acq *a, uint8_t signs);

#endif // c5_mclib.h
