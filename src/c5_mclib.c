// Creator 5 mainBoardGD portable motor controller
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <limits.h>
#include <stdint.h>
#include <string.h>

#include "c5_mclib.h"

#define MODE_DISABLED 0u
#define MODE_HOLD 1u
#define MODE_RUN 2u

#define ANGLE_TAU 0x1.921fb6p+2f
#define ANGLE_HALF_PI 0x1.921fb4p+0f
#define ANGLE_PI 0x1.921fb4p+1f
#define ANGLE_THREE_HALF_PI 0x1.2d97c8p+2f
#define ANGLE_TAU_CORRECTION 0x1.921fb8p+2f
#define ANGLE_PI_CORRECTION 0x1.921fb8p+1f
#define TRIG_SCALE 0x1.45f306p+6f
#define VOLTAGE_LIMIT 0x1.7fd70ap+4f
#define VOLTAGE_LIMIT_SQUARED 0x1.1fc292p+9f
#define MIN_DUTY 0x1.47ae14p-5f
#define DUTY_SCALE 0x1.555556p-5f

static const float atan_table[102] = {
    0x0.0p+0f, 0x1.47aaba0000000p-7f, 0x1.47a2c20000000p-6f,
    0x1.eb5f600000000p-6f, 0x1.4781340000000p-5f, 0x1.9942260000000p-5f,
    0x1.eaee560000000p-5f, 0x1.1e40c80000000p-4f, 0x1.46fbb80000000p-4f,
    0x1.6fa6300000000p-4f, 0x1.983e1a0000000p-4f, 0x1.c0c1760000000p-4f,
    0x1.e92e480000000p-4f, 0x1.08c1540000000p-3f, 0x1.1cde500000000p-3f,
    0x1.30ed300000000p-3f, 0x1.44ed040000000p-3f, 0x1.58dce80000000p-3f,
    0x1.6cbbf80000000p-3f, 0x1.8089500000000p-3f, 0x1.9444180000000p-3f,
    0x1.a7eb7a0000000p-3f, 0x1.bb7eba0000000p-3f, 0x1.cefce60000000p-3f,
    0x1.e2655e0000000p-3f, 0x1.f5b7580000000p-3f, 0x1.04790c0000000p-2f,
    0x1.0e0a780000000p-2f, 0x1.178f980000000p-2f, 0x1.2108160000000p-2f,
    0x1.2a73a00000000p-2f, 0x1.33d1f40000000p-2f, 0x1.3d22c20000000p-2f,
    0x1.4665be0000000p-2f, 0x1.4f9aae0000000p-2f, 0x1.58c1480000000p-2f,
    0x1.61d94e0000000p-2f, 0x1.6ae2900000000p-2f, 0x1.73dcce0000000p-2f,
    0x1.7cc7d20000000p-2f, 0x1.85a3720000000p-2f, 0x1.8e6f800000000p-2f,
    0x1.972bc40000000p-2f, 0x1.9fd8280000000p-2f, 0x1.a874780000000p-2f,
    0x1.b1009c0000000p-2f, 0x1.b97c6c0000000p-2f, 0x1.c1e7cc0000000p-2f,
    0x1.ca42a80000000p-2f, 0x1.d28ce60000000p-2f, 0x1.dac6700000000p-2f,
    0x1.e2ef2c0000000p-2f, 0x1.eb07140000000p-2f, 0x1.f30e1c0000000p-2f,
    0x1.fb04320000000p-2f, 0x1.0174aa0000000p-1f, 0x1.055eb80000000p-1f,
    0x1.0940460000000p-1f, 0x1.0d194e0000000p-1f, 0x1.10e9d80000000p-1f,
    0x1.14b1de0000000p-1f, 0x1.1871600000000p-1f, 0x1.1c28660000000p-1f,
    0x1.1fd6f00000000p-1f, 0x1.237d020000000p-1f, 0x1.271aa60000000p-1f,
    0x1.2aafde0000000p-1f, 0x1.2e3cae0000000p-1f, 0x1.31c1220000000p-1f,
    0x1.353d400000000p-1f, 0x1.38b1100000000p-1f, 0x1.3c1c9c0000000p-1f,
    0x1.3f7ff20000000p-1f, 0x1.42db140000000p-1f, 0x1.462e140000000p-1f,
    0x1.4978fa0000000p-1f, 0x1.4cbbd00000000p-1f, 0x1.4ff6a80000000p-1f,
    0x1.5329860000000p-1f, 0x1.5654820000000p-1f, 0x1.5977a40000000p-1f,
    0x1.5c92f80000000p-1f, 0x1.5fa68e0000000p-1f, 0x1.62b2760000000p-1f,
    0x1.65b6bc0000000p-1f, 0x1.68b3700000000p-1f, 0x1.6ba8a40000000p-1f,
    0x1.6e96620000000p-1f, 0x1.717cbc0000000p-1f, 0x1.745bc40000000p-1f,
    0x1.77338a0000000p-1f, 0x1.7a04180000000p-1f, 0x1.7ccd860000000p-1f,
    0x1.7f8fe20000000p-1f, 0x1.824b380000000p-1f, 0x1.84ff9e0000000p-1f,
    0x1.87ad220000000p-1f, 0x1.8a53d80000000p-1f, 0x1.8cf3c80000000p-1f,
    0x1.8f8d0c0000000p-1f, 0x1.921fb40000000p-1f, 0x1.94abcc0000000p-1f,
};

static const float sine_table[131] = {
    -0x1.921cce0000000p-7f, 0x0.0p+0f, 0x1.921cce0000000p-7f,
    0x1.9215400000000p-6f, 0x1.2d864c0000000p-5f, 0x1.91f6380000000p-5f,
    0x1.f656d40000000p-5f, 0x1.2d51f80000000p-4f, 0x1.5f6cfe0000000p-4f,
    0x1.917a600000000p-4f, 0x1.c3785a0000000p-4f, 0x1.f564d20000000p-4f,
    0x1.139f0c0000000p-3f, 0x1.2c80fc0000000p-3f, 0x1.4557660000000p-3f,
    0x1.5e21380000000p-3f, 0x1.76dd920000000p-3f, 0x1.8f8b800000000p-3f,
    0x1.a829f80000000p-3f, 0x1.c0b8220000000p-3f, 0x1.d934fe0000000p-3f,
    0x1.f19f8c0000000p-3f, 0x1.04fb7c0000000p-2f, 0x1.111d220000000p-2f,
    0x1.1d343e0000000p-2f, 0x1.29405e0000000p-2f, 0x1.3541080000000p-2f,
    0x1.4135c60000000p-2f, 0x1.4d1e1e0000000p-2f, 0x1.58f9a40000000p-2f,
    0x1.64c7d80000000p-2f, 0x1.7088500000000p-2f, 0x1.7c3a8c0000000p-2f,
    0x1.87de280000000p-2f, 0x1.9372a40000000p-2f, 0x1.9ef7940000000p-2f,
    0x1.aa6c7e0000000p-2f, 0x1.b5d0fa0000000p-2f, 0x1.c1249a0000000p-2f,
    0x1.cc66e80000000p-2f, 0x1.d797740000000p-2f, 0x1.e2b5d20000000p-2f,
    0x1.edc1900000000p-2f, 0x1.f8ba480000000p-2f, 0x1.01cfc60000000p-1f,
    0x1.0738780000000p-1f, 0x1.0c97020000000p-1f, 0x1.11eb340000000p-1f,
    0x1.1734d40000000p-1f, 0x1.1c73b20000000p-1f, 0x1.21a79a0000000p-1f,
    0x1.26d0520000000p-1f, 0x1.2bedb00000000p-1f, 0x1.30ff800000000p-1f,
    0x1.36058a0000000p-1f, 0x1.3affa00000000p-1f, 0x1.3fed920000000p-1f,
    0x1.44cf300000000p-1f, 0x1.49a4480000000p-1f, 0x1.4e6caa0000000p-1f,
    0x1.5328260000000p-1f, 0x1.57d6920000000p-1f, 0x1.5c77bc0000000p-1f,
    0x1.610b740000000p-1f, 0x1.6591900000000p-1f, 0x1.6a09e40000000p-1f,
    0x1.6e74440000000p-1f, 0x1.72d0800000000p-1f, 0x1.771e740000000p-1f,
    0x1.7b5df20000000p-1f, 0x1.7f8ecc0000000p-1f, 0x1.83b0e00000000p-1f,
    0x1.87c3fe0000000p-1f, 0x1.8bc8040000000p-1f, 0x1.8fbcca0000000p-1f,
    0x1.93a2240000000p-1f, 0x1.9777f00000000p-1f, 0x1.9b3e040000000p-1f,
    0x1.9ef4400000000p-1f, 0x1.a29a7a0000000p-1f, 0x1.a630920000000p-1f,
    0x1.a9b6620000000p-1f, 0x1.ad2bca0000000p-1f, 0x1.b090a40000000p-1f,
    0x1.b3e4d00000000p-1f, 0x1.b728340000000p-1f, 0x1.ba5aa40000000p-1f,
    0x1.bd7c080000000p-1f, 0x1.c08c400000000p-1f, 0x1.c38b2c0000000p-1f,
    0x1.c678b20000000p-1f, 0x1.c954b20000000p-1f, 0x1.cc1f0e0000000p-1f,
    0x1.ced7ac0000000p-1f, 0x1.d17e740000000p-1f, 0x1.d4134c0000000p-1f,
    0x1.d696160000000p-1f, 0x1.d906bc0000000p-1f, 0x1.db65240000000p-1f,
    0x1.ddb13c0000000p-1f, 0x1.dfeae60000000p-1f, 0x1.e2120e0000000p-1f,
    0x1.e426a40000000p-1f, 0x1.e6288c0000000p-1f, 0x1.e817ba0000000p-1f,
    0x1.e9f4140000000p-1f, 0x1.ebbd8c0000000p-1f, 0x1.ed740c0000000p-1f,
    0x1.ef17880000000p-1f, 0x1.f0a7ee0000000p-1f, 0x1.f2252e0000000p-1f,
    0x1.f38f3a0000000p-1f, 0x1.f4e6020000000p-1f, 0x1.f6297a0000000p-1f,
    0x1.f759980000000p-1f, 0x1.f8764e0000000p-1f, 0x1.f97f920000000p-1f,
    0x1.fa75580000000p-1f, 0x1.fb57960000000p-1f, 0x1.fc26460000000p-1f,
    0x1.fce15e0000000p-1f, 0x1.fd88da0000000p-1f, 0x1.fe1cb00000000p-1f,
    0x1.fe9cd80000000p-1f, 0x1.ff09560000000p-1f, 0x1.ff621c0000000p-1f,
    0x1.ffa72c0000000p-1f, 0x1.ffd8860000000p-1f, 0x1.fff6220000000p-1f,
    0x1.0000000000000p+0f, 0x1.fff6220000000p-1f,
};

static inline int32_t
trunc_s32(float x)
{
    if (x != x)
        return 0;
    if (x >= 0x1p31f)
        return INT32_MAX;
    if (x <= -0x1p31f)
        return INT32_MIN;
    return (int32_t)x;
}

static inline uint32_t
trunc_u32(float x)
{
    if (!(x > 0.0f))
        return 0;
    if (x >= 0x1p32f)
        return UINT32_MAX;
    return (uint32_t)x;
}

static inline uint32_t
arm_lsl(uint32_t value, uint32_t shift)
{
    shift &= 0xffu;
    return shift < 32u ? value << shift : 0u;
}

static inline uint32_t
arm_udiv(uint32_t numerator, uint32_t denominator)
{
    return denominator ? numerator / denominator : 0u;
}

static inline float
clip_upper_lower(float value, float lower, float upper)
{
    if (value > upper)
        value = upper;
    if (value < lower)
        value = lower;
    return value;
}

static inline float
reduce_angle(float angle)
{
    while (angle >= ANGLE_TAU)
        angle -= ANGLE_TAU;
    while (angle < 0.0f)
        angle += ANGLE_TAU;
    return angle;
}

static void
sine_cosine(float angle, float *sine, float *cosine)
{
    float x = reduce_angle(angle);
    float z;
    uint8_t sine_negative, cosine_negative;
    if (x < ANGLE_HALF_PI) {
        z = x;
        sine_negative = 0;
        cosine_negative = 0;
    } else if (x < ANGLE_PI) {
        z = ANGLE_PI - x;
        sine_negative = 0;
        cosine_negative = 1;
    } else if (x < ANGLE_THREE_HALF_PI) {
        z = x - ANGLE_PI;
        sine_negative = 1;
        cosine_negative = 1;
    } else {
        z = ANGLE_TAU - x;
        sine_negative = 1;
        cosine_negative = 0;
    }

    float q = z * TRIG_SCALE;
    int32_t n = trunc_s32(q);
    float fraction = q - (float)n;
    float sv = sine_table[n + 1]
        + fraction * (sine_table[n + 2] - sine_table[n + 1]);
    float cv = sine_table[129 - n]
        + fraction * (sine_table[128 - n] - sine_table[129 - n]);
    *sine = sine_negative ? -sv : sv;
    *cosine = cosine_negative ? -cv : cv;
}

static float
scalar_sine(float angle)
{
    float x = reduce_angle(angle);
    float z;
    uint8_t negative;
    if (x < ANGLE_HALF_PI) {
        z = x;
        negative = 0u;
    } else if (x < ANGLE_PI) {
        z = ANGLE_PI - x;
        negative = 0u;
    } else if (x < ANGLE_THREE_HALF_PI) {
        z = x - ANGLE_PI;
        negative = 1u;
    } else {
        z = ANGLE_TAU - x;
        negative = 1u;
    }

    float q = z * TRIG_SCALE;
    int32_t n = trunc_s32(q);
    float integer = (float)n;
    float base = sine_table[n + 1];
    float difference = sine_table[n + 2] - base;
    if (negative)
        return (integer - q) * difference - base;
    return base + (q - integer) * difference;
}

static float
atan_interpolate(float q, uint8_t negative, uint8_t guard,
                 float addition, uint8_t add_before)
{
    int32_t converted = trunc_s32(q);
    float integer = (float)converted;
    float fraction = q - integer;
    uint32_t index = (uint32_t)converted;
    if (guard && index > 100u)
        index = 100u;
    float a = atan_table[index];
    float difference = atan_table[index + 1u] - a;
    if (add_before)
        return (a + addition) + fraction * difference;
    if (negative)
        return (integer - q) * difference - a + addition;
    return a + fraction * difference + addition;
}

static float
atan_like(float x, float y)
{
    float q;
    if (y > 0.0f) {
        if (x >= 0.0f) {
            if (!(x > y)) {
                q = x * 100.0f / y;
                return atan_interpolate(q, 0, 0, 0.0f, 0);
            }
            q = y * 100.0f / x;
            return atan_interpolate(q, 1, 0, ANGLE_HALF_PI, 0);
        }
        if (!(-x > y)) {
            q = x * -100.0f / y;
            return atan_interpolate(q, 1, 0, ANGLE_TAU, 0);
        }
        q = y * 100.0f / -x;
        return atan_interpolate(q, 0, 0, ANGLE_THREE_HALF_PI, 0);
    }
    if (y >= 0.0f)
        return 0.0f;
    if (x >= 0.0f) {
        if (-y >= x) {
            q = x * 100.0f / -y;
            return atan_interpolate(q, 1, 1, ANGLE_PI, 0);
        }
        q = y * -100.0f / x;
        return atan_interpolate(q, 0, 1, ANGLE_HALF_PI, 0);
    }
    if (!(x > y)) {
        q = y * 100.0f / x;
        return atan_interpolate(q, 1, 0, ANGLE_THREE_HALF_PI, 0);
    }
    q = x * 100.0f / y;
    return atan_interpolate(q, 0, 1, ANGLE_PI, 1);
}

static void
pi_init(struct c5_mclib_pi *pi, float kp, float ki,
        float integral_limit, float output_limit)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->integral_min = -integral_limit;
    pi->integral_max = integral_limit;
    pi->output_min = -output_limit;
    pi->output_max = output_limit;
}

static float
pi_update(struct c5_mclib_pi *pi, float reference, float measurement)
{
    float error = reference - measurement;
    float integral = pi->integral + pi->ki * error;
    if (integral > pi->integral_max)
        integral = pi->integral_max;
    if (integral < pi->integral_min)
        integral = pi->integral_min;
    float output = integral + pi->kp * error;
    if (output < pi->output_min)
        output = pi->output_min;
    if (output > pi->output_max)
        output = pi->output_max;
    pi->error = error;
    pi->integral = integral;
    pi->output = output;
    return output;
}

static void
pi_reset(struct c5_mclib_pi *pi)
{
    pi->error = 0.0f;
    pi->integral = 0.0f;
    pi->output = 0.0f;
}

static void
forward_transform(float ia, float ib, float sine, float cosine,
                  float *d, float *q)
{
    *d = ib * sine + ia * cosine;
    *q = ib * cosine - ia * sine;
}

static void
inverse_transform(float vd, float vq, float sine, float cosine,
                  float *va, float *vb)
{
    *va = vd * cosine - vq * sine;
    *vb = vd * sine + vq * cosine;
}

static inline float
sqrt_positive(float value)
{
    return value > 0.0f ? __builtin_sqrtf(value) : 0.0f;
}

static void
observer_reset(struct c5_mclib_observer *o)
{
    o->previous_raw_angle = 0.0f;
    o->estimated_current_a = 0.0f;
    o->estimated_current_b = 0.0f;
    o->filtered_injection_a = 0.0f;
    o->filtered_injection_b = 0.0f;
}

static float
observer_phase(struct c5_mclib_observer *o, float current,
               float previous_voltage, float old_estimate,
               float *injection, float *filtered)
{
    float error = old_estimate - current;
    float value;
    if (error > o->error_threshold)
        value = o->injection_magnitude;
    else if (error < -o->error_threshold)
        value = -o->injection_magnitude;
    else
        value = o->inner_slope * error;
    *injection = value;
    float estimate = o->b * (previous_voltage - value) + o->a * old_estimate;
    *filtered += o->filter_coefficient * (value - *filtered);
    return estimate;
}

static void
observer_update(struct c5_mclib_motor *m, float ia, float ib)
{
    struct c5_mclib_observer *o = &m->observer;
    float old_a = o->estimated_current_a;
    float old_b = o->estimated_current_b;
    o->estimated_current_a = observer_phase(
        o, ia, m->previous_voltage_a, old_a,
        &o->injection_a, &o->filtered_injection_a);
    o->estimated_current_b = observer_phase(
        o, ib, m->previous_voltage_b, old_b,
        &o->injection_b, &o->filtered_injection_b);

    float raw = atan_like(-o->filtered_injection_a,
                          o->filtered_injection_b);
    float difference = raw - o->previous_raw_angle;
    if (difference > ANGLE_PI_CORRECTION)
        difference -= ANGLE_TAU_CORRECTION;
    else if (difference < -ANGLE_PI_CORRECTION)
        difference += ANGLE_TAU_CORRECTION;
    o->raw_angle = raw;
    o->previous_raw_angle = raw;
    o->instantaneous_speed = difference * 20000.0f;
    o->filtered_speed += 0.1f * (o->instantaneous_speed - o->filtered_speed);
    o->scaled_speed = o->filtered_speed * 0x1.8723a0p-3f;
    o->compensation = atan_like(o->filtered_speed,
                                o->compensation_denominator);
    o->angle = reduce_angle(raw + o->compensation);
}

void
c5_mclib_init(struct c5_mclib_motor *m, uint8_t axis)
{
    memset(m, 0, sizeof(*m));
    m->axis = axis;
    m->phase = 8192u;
    m->hold_delay = 100000000u;
    m->ramp_ticks = 200000000u;
    m->crossover_threshold = 2343u;
    m->slow_threshold = 37500u;
    m->interpolation = 1u;
    m->slow = 1u;

    float pi_limit = 0x1.7fd70ap+4f;
    if (axis < 2u) {
        m->resistance = 1.4f;
        m->inductance = 0.003f;
        m->motor_constant = 0.005f;
        pi_init(&m->d_pi, 0x1.2d97cap+4f, 0x1.c260f6p-2f,
                pi_limit, pi_limit);
        pi_init(&m->q_pi, 0x1.2d97cap+4f, 0x1.c260f6p-2f,
                pi_limit, pi_limit);
        m->observer.a = 0x1.f40da8p-1f;
        m->observer.b = 0x1.111110p-6f;
        m->run_current = 1.5f;
        m->hold_current = 1.5f;
        m->exponent = 5u;
        m->stall_threshold = 1.0f;
    } else {
        m->resistance = 1.6f;
        m->inductance = 0.0026f;
        m->motor_constant = 0.00645f;
        pi_init(&m->d_pi, 0x1.05616cp+4f, 0x1.015bfcp-1f,
                pi_limit, pi_limit);
        pi_init(&m->q_pi, 0x1.05616cp+4f, 0x1.015bfcp-1f,
                pi_limit, pi_limit);
        m->observer.a = 0x1.f03f04p-1f;
        m->observer.b = 0x1.3b13b0p-6f;
        m->run_current = 0.8f;
        m->hold_current = 0.8f;
        m->exponent = 4u;
        m->stall_threshold = 3.1f;
    }
    m->phase_increment = (uint16_t)arm_lsl(1u, 14u - m->exponent);
    m->stop_timeout = arm_lsl(8333333u, 8u - m->exponent);
    pi_init(&m->outer_pi, 0.005f, 0x1.a36e2ep-15f, 1.0f, 1.0f);
    pi_reset(&m->d_pi);
    pi_reset(&m->q_pi);
    pi_reset(&m->outer_pi);
    m->observer.injection_magnitude = 36.0f;
    m->observer.error_threshold = 0.5f;
    m->observer.inner_slope = 72.0f;
    m->observer.filter_coefficient = 0x1.a8ebd0p-3f;
    m->observer.compensation_denominator = 0x1.473fd2p+12f;
}

void
c5_mclib_configure(struct c5_mclib_motor *m,
                    uint32_t rs, uint32_t ls, uint32_t km)
{
    float resistance = (float)rs / 1000.0f;
    float inductance = (float)ls / 1000000.0f;
    float motor_constant = (float)km / 1000000.0f;
    motor_constant = motor_constant / 50.0f;
    float proportional = inductance * 0x1.88b2fap+12f;
    float ratio = resistance / inductance;
    float integral = proportional * ratio;
    integral = integral * 0x1.a36e2ep-15f;
    float observer_factor = -0x1.a36e2ep-15f / inductance;
    float observer_a = __builtin_fmaf(observer_factor, resistance, 1.0f);
    float observer_b = 0x1.a36e2ep-15f / inductance;

    m->resistance = resistance;
    m->inductance = inductance;
    m->motor_constant = motor_constant;
    m->d_pi.kp = proportional;
    m->q_pi.kp = proportional;
    m->d_pi.ki = integral;
    m->q_pi.ki = integral;
    m->observer.a = observer_a;
    m->observer.b = observer_b;
}

void
c5_mclib_microstep(struct c5_mclib_motor *m,
                    uint8_t interpolate, uint16_t exponent)
{
    m->interpolation = interpolate;
    m->exponent = exponent;
    m->phase_increment = (uint16_t)arm_lsl(1u, 14u - exponent);
    m->stop_timeout = arm_lsl(8333333u, 8u - exponent);
}

void
c5_mclib_current(struct c5_mclib_motor *m,
                  uint32_t run_ma, uint32_t hold_ma)
{
    m->run_current = (float)run_ma / 1000.0f;
    float requested_hold = (float)hold_ma / 1000.0f;
    m->hold_current = requested_hold < m->run_current
        ? requested_hold : m->run_current;
    if (m->ramp_ticks)
        m->decay_increment = (m->run_current - m->hold_current)
            / (float)m->ramp_ticks * 5000.0f;
    else
        m->decay_increment = m->run_current - m->hold_current;
}

void
c5_mclib_stall_threshold(struct c5_mclib_motor *m, uint32_t threshold)
{
    m->stall_threshold = (float)threshold / 1000.0f;
}

void
c5_mclib_resonance(struct c5_mclib_motor *m,
                    uint8_t tdx, uint32_t amp, uint32_t phase1,
                    uint32_t phase2)
{
    uint8_t slot = tdx >> 1;
    if (slot >= 3u)
        return;
    m->resonance_amplitude[slot] = (float)amp / 1000.0f;
    m->resonance_phase_forward[slot] = (float)phase1 / 1000.0f;
    m->resonance_phase_reverse[slot] = (float)phase2 / 1000.0f;
}

void
c5_mclib_direction(struct c5_mclib_motor *m, uint8_t direction)
{
    m->direction = direction;
}

void
c5_mclib_step(struct c5_mclib_motor *m, uint32_t now)
{
    m->last_period = now - m->last_step;
    m->last_step = now;
    if (m->direction)
        m->phase = (uint16_t)(m->phase + m->phase_increment);
    else
        m->phase = (uint16_t)(m->phase - m->phase_increment);
    m->command_angle = (float)m->phase * ANGLE_PI_CORRECTION;
    m->command_angle = m->command_angle * 0x1p-15f;
    if (!(m->phase & 0x3fffu) && m->quarter_count != UINT8_MAX)
        m->quarter_count++;
    if (m->mode == MODE_HOLD) {
        m->hold_transition = 0u;
        m->active_current = m->run_current;
        m->last_period = m->stop_timeout;
        m->mode = MODE_RUN;
    }
}

void
c5_mclib_enable(struct c5_mclib_motor *m, uint32_t now)
{
    m->last_step = now;
    m->phase = 8192u;
    m->command_angle = 0x1.921fb8p-1f;
    m->phase_increment = (uint16_t)arm_lsl(1u, 14u - m->exponent);
    m->active_current = m->run_current;
    float sine, cosine;
    sine_cosine(m->command_angle, &sine, &cosine);
    m->reference_d = m->active_current * cosine;
    m->reference_q = m->active_current * sine;
    m->mode = MODE_HOLD;
}

void
c5_mclib_disable(struct c5_mclib_motor *m)
{
    m->active_current = 0.0f;
    m->reference_d = 0.0f;
    m->reference_q = 0.0f;
    observer_reset(&m->observer);
    m->mode = MODE_DISABLED;
    m->stall = 0u;
    m->slow = 1u;
    m->hold_transition = 0u;
}

static void
update_stall_state(struct c5_mclib_motor *m, float ia, float ib,
                   float initial_error)
{
    float torque = ia * m->observer.filtered_injection_b
        - ib * m->observer.filtered_injection_a;
    m->filtered_torque += 0x1.47ae14p-8f
        * (torque - m->filtered_torque);
    m->filtered_angle_error += 0x1.47ae14p-8f
        * (initial_error - m->filtered_angle_error);
    uint8_t decision = __builtin_fabsf(m->filtered_torque)
        < m->stall_threshold;
    m->stall = decision && !m->hold_transition && !m->slow
        && m->quarter_count >= 17u;
}

static void
run_state_update(struct c5_mclib_motor *m, float initial_error)
{
    if (m->axis < 2u) {
        if (m->last_period < m->crossover_threshold - 100u
            && !m->fast_mode) {
            float sine, cosine;
            sine_cosine(initial_error, &sine, &cosine);
            m->reference_d = m->measured_q * cosine;
            m->reference_q = m->measured_q * sine;
            m->outer_pi.integral = m->reference_d;
            float speed_term = m->inductance * m->observer.filtered_speed;
            m->d_pi.integral = m->resistance * m->reference_d
                - speed_term * m->reference_q;
            m->fast_mode = 1u;
        }
        if (m->last_period > m->crossover_threshold + 100u)
            m->fast_mode = 0u;
    }

    if ((int32_t)m->elapsed > (int32_t)m->stop_timeout) {
        m->hold_transition = 1u;
        m->quarter_count = 0u;
        m->mode = MODE_HOLD;
    }

    if (m->interpolation) {
        float ratio = (float)(int32_t)m->elapsed / (float)m->last_period;
        int32_t delta = trunc_s32(ratio * (float)m->phase_increment);
        uint16_t interpolated;
        if (m->direction) {
            if (m->elapsed < m->last_period)
                interpolated = (uint16_t)(m->phase + delta);
            else
                interpolated = (uint16_t)(m->phase
                                           + m->phase_increment - 1u);
        } else {
            if (m->elapsed < m->last_period)
                interpolated = (uint16_t)(m->phase - delta);
            else
                interpolated = (uint16_t)(m->phase
                                           - m->phase_increment + 1u);
        }
        m->command_angle = (float)interpolated * 0x1.921fb8p-14f;
    }
}

static void
hold_current_update(struct c5_mclib_motor *m)
{
    uint32_t ramp_start = m->stop_timeout + m->hold_delay;
    if ((int32_t)m->elapsed <= (int32_t)ramp_start)
        return;
    float next = m->active_current - m->decay_increment;
    m->active_current = next > m->hold_current ? next : m->hold_current;
}

static void
nonfast_control(struct c5_mclib_motor *m, float ia, float ib,
                float *sine, float *cosine)
{
    m->reference_d = 0.0f;
    m->reference_q = m->active_current;
    if (m->axis < 2u && m->last_period >= 2468u) {
        static const float multiplier[3] = { 1.0f, 2.0f, 4.0f };
        float compensation = 0.0f;
        for (uint8_t i = 0; i < 3u; i++) {
            float phase = m->direction ? m->resonance_phase_forward[i]
                : m->resonance_phase_reverse[i];
            float angle = multiplier[i] * m->command_angle + phase;
            compensation += m->resonance_amplitude[i]
                * scalar_sine(angle);
        }
        m->reference_d -= compensation;
    }
    sine_cosine(m->command_angle, sine, cosine);
    forward_transform(ia, ib, *sine, *cosine,
                      &m->measured_d, &m->measured_q);
    m->requested_d = pi_update(&m->d_pi, m->reference_d, m->measured_d);
    m->requested_q = pi_update(&m->q_pi, m->reference_q, m->measured_q);

    if (m->requested_d > VOLTAGE_LIMIT) {
        m->requested_d = VOLTAGE_LIMIT;
        m->requested_q = 0.0f;
    } else if (m->requested_d < -VOLTAGE_LIMIT) {
        m->requested_d = -VOLTAGE_LIMIT;
        m->requested_q = 0.0f;
    } else {
        float magnitude = m->requested_d * m->requested_d
            + m->requested_q * m->requested_q;
        if (magnitude > VOLTAGE_LIMIT_SQUARED) {
            float q = sqrt_positive(VOLTAGE_LIMIT_SQUARED
                                    - m->requested_d * m->requested_d);
            m->requested_q = m->requested_q < 0.0f ? -q : q;
        }
    }
}

static void
fast_control(struct c5_mclib_motor *m, float ia, float ib,
             float *sine, float *cosine)
{
    float late_error = m->command_angle - m->observer.angle;
    if (late_error > ANGLE_PI_CORRECTION)
        late_error -= ANGLE_TAU_CORRECTION;
    else if (late_error < -ANGLE_PI_CORRECTION)
        late_error += ANGLE_TAU_CORRECTION;
    uint32_t denominator = arm_lsl(m->last_period, m->exponent);
    float speed_target = (float)arm_udiv(30000000u, denominator);
    if (!m->direction)
        speed_target = -speed_target;
    speed_target += 10.0f * late_error;

    m->reference_d = pi_update(&m->outer_pi, m->observer.scaled_speed,
                               speed_target);
    sine_cosine(m->observer.angle, sine, cosine);
    forward_transform(ia, ib, *sine, *cosine,
                      &m->measured_d, &m->measured_q);
    m->requested_d = pi_update(&m->d_pi, m->reference_d, m->measured_d);
    m->requested_q = sqrt_positive(VOLTAGE_LIMIT_SQUARED
                                    - m->requested_d * m->requested_d);
}

static void
pwm_phase(float voltage, uint32_t *compare, uint8_t *negative)
{
    float normalized = voltage * DUTY_SCALE;
    *negative = normalized < 0.0f;
    float magnitude = *negative ? -normalized : normalized;
    float primary = magnitude > MIN_DUTY ? magnitude : MIN_DUTY;
    float auxiliary = magnitude >= MIN_DUTY ? 0.0f : MIN_DUTY - magnitude;
    uint32_t a = trunc_u32((1.0f + primary) * 7500.0f);
    uint32_t b = trunc_u32((1.0f - primary) * 7500.0f);
    uint32_t d = trunc_u32(__builtin_fmaf(auxiliary, 15000.0f, (float)a));
    if (!*negative) {
        compare[0] = a;
        compare[1] = d;
        compare[2] = b;
        compare[3] = a;
    } else {
        compare[0] = b;
        compare[1] = a;
        compare[2] = a;
        compare[3] = d;
    }
}

uint8_t
c5_mclib_update(struct c5_mclib_motor *m, uint32_t now,
                 float ia, float ib, struct c5_mclib_output *out)
{
    if (m->mode == MODE_DISABLED)
        return 0;

    m->elapsed = now - m->last_step;
    m->slow = m->last_period > m->slow_threshold
        || (int32_t)m->elapsed > (int32_t)m->slow_threshold;
    if (m->slow) {
        m->quarter_count = 0u;
        if (m->axis < 2u)
            m->fast_mode = 0u;
    }

    observer_update(m, ia, ib);
    float initial_error;
    if (m->axis < 2u)
        initial_error = m->command_angle - m->observer.angle
            + 0x1.921fb8p+0f;
    else if (m->direction)
        initial_error = m->command_angle - m->observer.angle
            + 0x1.921fb8p+0f;
    else
        initial_error = m->observer.angle - m->command_angle
            + 0x1.921fb8p+0f;
    if (initial_error > ANGLE_PI_CORRECTION)
        initial_error -= ANGLE_TAU_CORRECTION;
    else if (initial_error < -ANGLE_PI_CORRECTION)
        initial_error += ANGLE_TAU_CORRECTION;
    update_stall_state(m, ia, ib, initial_error);

    if (m->mode == MODE_RUN)
        run_state_update(m, initial_error);
    if (m->mode == MODE_HOLD)
        hold_current_update(m);

    float sine, cosine;
    if (m->axis < 2u && m->fast_mode)
        fast_control(m, ia, ib, &sine, &cosine);
    else
        nonfast_control(m, ia, ib, &sine, &cosine);

    float va, vb;
    inverse_transform(m->requested_d, m->requested_q,
                      sine, cosine, &va, &vb);
    const float maximum_finite = 0x1.fffffep+127f;
    if (!(va >= -maximum_finite && va <= maximum_finite
          && vb >= -maximum_finite && vb <= maximum_finite))
        return 2;
    va = clip_upper_lower(va, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
    vb = clip_upper_lower(vb, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
    m->previous_voltage_a = va;
    m->previous_voltage_b = vb;

    uint8_t negative_a, negative_b;
    pwm_phase(va, &out->compare[0], &negative_a);
    pwm_phase(vb, &out->compare[4], &negative_b);
    out->signs = negative_a | (negative_b << 1);
    return 1;
}

uint8_t
c5_mclib_stalled(const struct c5_mclib_motor *m)
{
    return m->stall;
}

void
c5_mclib_acq_init(struct c5_mclib_acq *a)
{
    memset(a, 0, sizeof(*a));
    a->offset_a = 8192;
    a->offset_b = 8192;
    a->sum_a = (8192 << 8) - 8192;
    a->sum_b = (8192 << 8) - 8192;
    a->calibrating = 1u;
}

uint8_t
c5_mclib_acquire(struct c5_mclib_acq *a,
                  int16_t raw0, int16_t raw1, float *ia, float *ib)
{
    if (a->calibrating) {
        a->sum_a += raw0;
        a->sum_b += raw1;
        int32_t filtered_a = a->sum_a >> 8;
        int32_t filtered_b = a->sum_b >> 8;
        a->sum_a -= filtered_a;
        a->sum_b -= filtered_b;
        a->count++;
        if (a->count < 2000u)
            return 0;
        a->offset_a = (int16_t)filtered_a;
        a->offset_b = (int16_t)filtered_b;
        a->calibrating = 0u;
    }

    float scaled_a = (float)((int32_t)raw0 - (int32_t)a->offset_a)
        * 0x1.19999ap-12f;
    float scaled_b = (float)((int32_t)raw1 - (int32_t)a->offset_b)
        * 0x1.19999ap-12f;
    *ia = (a->signs & 1u) ? MIN_DUTY - scaled_a : scaled_a - MIN_DUTY;
    *ib = (a->signs & 2u) ? MIN_DUTY - scaled_b : scaled_b - MIN_DUTY;
    return 1;
}

void
c5_mclib_acq_polarity(struct c5_mclib_acq *a, uint8_t signs)
{
    a->signs = signs;
}
