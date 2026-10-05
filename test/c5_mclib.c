// Creator 5 mainBoardGD motor-control regressions
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.


#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "c5_mclib.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define MODE_DISABLED 0u
#define MODE_HOLD 1u
#define MODE_RUN 2u
#define MOTOR_STOP_TIMEOUT 0x07f28150u
#define MOTOR_HOLD_DELAY 100000000u

static int failures;

static void check_pwm_shape(const char *name,
                            const struct c5_mclib_output *out);

#define CHECK(condition, message) do {                                  \
        if (!(condition)) {                                             \
            fprintf(stderr, "%s\n", (message));                        \
            failures++;                                                 \
        }                                                               \
    } while (0)

static uint32_t
float_bits(float value)
{
    union {
        float f;
        uint32_t u;
    } bits = { .f = value };
    return bits.u;
}

// Force explicit PI gains (milli-units) for controller regressions.
static void
set_pi_gains(struct c5_mclib_motor *m, uint32_t kp, uint32_t ki)
{
    m->d_pi.kp = m->q_pi.kp = (float)kp / 1000.0f;
    m->d_pi.ki = m->q_pi.ki = (float)ki / 1000.0f;
}

static void
expect_u32(const char *name, uint32_t actual, uint32_t expected)
{
    if (actual == expected)
        return;
    fprintf(stderr, "%s: expected %u, got %u\n", name, expected, actual);
    failures++;
}

static void
expect_float_close(const char *name, float actual, float expected,
                   float tolerance)
{
    if (fabsf(actual - expected) <= tolerance)
        return;
    fprintf(stderr, "%s: expected %.9g, got %.9g\n",
            name, (double)expected, (double)actual);
    failures++;
}

static void
expect_output(const char *name, const struct c5_mclib_output *out,
              uint8_t signs, const uint32_t expected[8])
{
    if (out->signs != signs) {
        fprintf(stderr, "%s signs: expected %u, got %u\n",
                name, signs, out->signs);
        failures++;
    }
    for (uint8_t i = 0; i < 8; i++) {
        if (out->compare[i] == expected[i])
            continue;
        fprintf(stderr, "%s compare[%u]: expected %u, got %u\n",
                name, i, expected[i], out->compare[i]);
        failures++;
    }
    check_pwm_shape(name, out);
}

static uint8_t
outputs_equal(const struct c5_mclib_output *a,
              const struct c5_mclib_output *b)
{
    return a->signs == b->signs
        && !memcmp(a->compare, b->compare, sizeof(a->compare));
}

static void
fill_output(struct c5_mclib_output *out)
{
    for (uint8_t i = 0; i < 8; i++)
        out->compare[i] = 0xa5a50000u + i;
    out->signs = 0x5au;
}

static uint8_t
output_is_sentinel(const struct c5_mclib_output *out)
{
    if (out->signs != 0x5au)
        return 0;
    for (uint8_t i = 0; i < 8; i++)
        if (out->compare[i] != 0xa5a50000u + i)
            return 0;
    return 1;
}

static void
check_pwm_shape(const char *name, const struct c5_mclib_output *out)
{
    if (out->signs & ~3u) {
        fprintf(stderr, "%s invalid sign bits: %u\n", name, out->signs);
        failures++;
    }
    for (uint8_t phase = 0; phase < 2; phase++) {
        const uint32_t *v = &out->compare[phase * 4];
        uint8_t negative = (out->signs >> phase) & 1u;
        if (!negative) {
            if (!(v[0] == v[3] && v[0] >= v[2] && v[1] >= v[0])) {
                fprintf(stderr, "%s phase %u positive PWM layout\n",
                        name, phase);
                failures++;
            }
        } else if (!(v[1] == v[2] && v[1] >= v[0] && v[3] >= v[1])) {
            fprintf(stderr, "%s phase %u negative PWM layout\n",
                    name, phase);
            failures++;
        }
        for (uint8_t i = 0; i < 4; i++) {
            if (v[i] <= 14999u)
                continue;
            fprintf(stderr, "%s phase %u compare %u exceeds period\n",
                    name, phase, v[i]);
            failures++;
        }
    }
}

static void
setup_z(struct c5_mclib_motor *m, uint32_t run_ma, uint32_t hold_ma)
{
    memset(m, 0xa5, sizeof(*m));
    c5_mclib_init(m, 2);
    c5_mclib_configure(m, 1700, 2600, 172900);
    c5_mclib_microstep(m, 1, 4);
    c5_mclib_current(m, run_ma, hold_ma);
}

static void
setup_xy(struct c5_mclib_motor *m, uint8_t axis,
         uint32_t run_ma, uint32_t hold_ma)
{
    memset(m, 0xa5, sizeof(*m));
    c5_mclib_init(m, axis);
    c5_mclib_configure(m, 1400, 3000, 250000);
    c5_mclib_microstep(m, 1, 4);
    c5_mclib_current(m, run_ma, hold_ma);
}

static uint32_t
step_many(struct c5_mclib_motor *m, uint32_t now,
          uint32_t period, uint16_t count)
{
    for (uint16_t i = 0; i < count; i++) {
        now += period;
        c5_mclib_step(m, now);
    }
    return now;
}

static void
test_reference_waveforms(void)
{
    static const uint32_t zero_current[8] = {
        7799, 8399, 7200, 7799, 7799, 8399, 7200, 7799,
    };
    static const uint32_t one_amp[8] = {
        3772, 11227, 11227, 11227, 11227, 11227, 3772, 11227,
    };
    static const uint32_t forward_step[8] = {
        3424, 11575, 11575, 11575, 10844, 10844, 4155, 10844,
    };
    static const uint32_t reverse_step[8] = {
        4155, 10844, 10844, 10844, 11575, 11575, 3424, 11575,
    };
    static const uint32_t noninterpolated_step[8] = {
        7799, 8398, 7200, 7799, 2228, 12771, 12771, 12771,
    };
    struct c5_mclib_motor m;
    struct c5_mclib_output out;

    setup_z(&m, 0, 0);
    c5_mclib_enable(&m, 0);
    CHECK(c5_mclib_update(&m, 0, 0.0f, 0.0f, &out) == 1,
          "zero-current update admission");
    expect_output("zero-current waveform", &out, 0, zero_current);

    setup_z(&m, 1000, 1000);
    c5_mclib_enable(&m, 0);
    CHECK(c5_mclib_update(&m, 0, 0.0f, 0.0f, &out) == 1,
          "one-amp update admission");
    expect_output("one-amp waveform", &out, 1, one_amp);

    setup_z(&m, 1000, 1000);
    c5_mclib_direction(&m, 1);
    c5_mclib_enable(&m, 0);
    c5_mclib_step(&m, 0);
    CHECK(c5_mclib_update(&m, 0, 0.0f, 0.0f, &out) == 1,
          "forward-step update admission");
    expect_output("forward-step waveform", &out, 1, forward_step);

    setup_z(&m, 1000, 1000);
    c5_mclib_microstep(&m, 0, 4);
    c5_mclib_direction(&m, 1);
    c5_mclib_enable(&m, 0);
    uint32_t now = step_many(&m, 0, 20000, 24);
    expect_u32("noninterpolated step angle bits",
               float_bits(m.command_angle), 0x40490fdcu);
    CHECK(c5_mclib_update(&m, now, 0.0f, 0.0f, &out) == 1,
          "noninterpolated step update admission");
    expect_output("noninterpolated step waveform", &out, 2,
                  noninterpolated_step);

    setup_z(&m, 1000, 1000);
    c5_mclib_direction(&m, 0);
    c5_mclib_enable(&m, 0);
    c5_mclib_step(&m, 0);
    CHECK(c5_mclib_update(&m, 0, 0.0f, 0.0f, &out) == 1,
          "reverse-step update admission");
    expect_output("reverse-step waveform", &out, 1, reverse_step);
}

static void
test_electrical_cycle(void)
{
    struct c5_mclib_motor reference, cycle, single;
    struct c5_mclib_output reference_out, cycle_out, single_out;

    setup_z(&reference, 1000, 1000);
    c5_mclib_enable(&reference, 0);
    c5_mclib_update(&reference, 0, 0.0f, 0.0f, &reference_out);

    setup_z(&cycle, 1000, 1000);
    c5_mclib_enable(&cycle, 0);
    uint32_t now = step_many(&cycle, 0, 20000, 64);
    c5_mclib_update(&cycle, now, 0.0f, 0.0f, &cycle_out);
    CHECK(outputs_equal(&reference_out, &cycle_out),
          "64 exponent-4 steps must wrap one electrical cycle");

    setup_z(&single, 1000, 1000);
    c5_mclib_enable(&single, 0);
    c5_mclib_step(&single, 20000);
    c5_mclib_update(&single, 20000, 0.0f, 0.0f, &single_out);
    CHECK(!outputs_equal(&reference_out, &single_out),
          "one step must change the electrical waveform");
}

static void
test_stall_gating_and_wrap(void)
{
    struct c5_mclib_motor m;
    struct c5_mclib_output out;

    setup_z(&m, 1000, 1000);
    c5_mclib_stall_threshold(&m, 600);
    c5_mclib_enable(&m, 0);
    uint32_t now = step_many(&m, 0, 20000, 248);
    c5_mclib_update(&m, now, 0.0f, 0.0f, &out);
    CHECK(c5_mclib_stalled(&m) == 0,
          "quarter marker 16 must not expose stall");
    now = step_many(&m, now, 20000, 16);
    c5_mclib_update(&m, now, 0.0f, 0.0f, &out);
    CHECK(c5_mclib_stalled(&m) == 1,
          "quarter marker 17 may expose stall");
    c5_mclib_update(&m, now + 37500u, 0.0f, 0.0f, &out);
    CHECK(c5_mclib_stalled(&m) == 1,
          "elapsed 37500 is the non-slow boundary");
    c5_mclib_update(&m, now + 37501u, 0.0f, 0.0f, &out);
    CHECK(c5_mclib_stalled(&m) == 0,
          "elapsed 37501 must clear stall history");
    CHECK(m.quarter_count == 0 && m.slow == 1,
          "slow update must clear quarter history");

    setup_z(&m, 1000, 1000);
    c5_mclib_stall_threshold(&m, 600);
    uint32_t start = UINT32_MAX - 100000u;
    c5_mclib_enable(&m, start);
    now = step_many(&m, start, 20000, 264);
    c5_mclib_update(&m, now, 0.0f, 0.0f, &out);
    CHECK(c5_mclib_stalled(&m) == 1,
          "modulo-32 timer wrap must preserve stall intervals");
    expect_u32("wrapped step period", m.last_period, 20000u);
}

static void
test_hold_run_and_current_ramp(void)
{
    static const uint32_t run_current[8] = {
        6631, 8368, 8368, 8368, 8799, 8799, 6200, 8799,
    };
    static const uint32_t hold_current[8] = {
        7065, 7934, 7934, 7934, 8149, 8149, 6850, 8149,
    };
    struct c5_mclib_motor m;
    struct c5_mclib_output out;

    setup_z(&m, 1000, 500);
    set_pi_gains(&m, 5000, 0);
    c5_mclib_enable(&m, 10);
    CHECK(m.mode == MODE_HOLD, "enable must enter HOLD");
    c5_mclib_step(&m, 20);
    CHECK(m.mode == MODE_RUN, "first step must enter RUN");
    expect_u32("configured exponent-4 timeout", m.last_period,
               MOTOR_STOP_TIMEOUT);

    c5_mclib_update(&m, 20u + MOTOR_STOP_TIMEOUT,
                    0.0f, 0.0f, &out);
    CHECK(m.mode == MODE_RUN,
          "stop timeout equality must remain RUN");
    expect_output("timeout equality waveform", &out, 1, run_current);

    c5_mclib_update(&m, 21u + MOTOR_STOP_TIMEOUT,
                    0.0f, 0.0f, &out);
    CHECK(m.mode == MODE_HOLD,
          "stop timeout plus one must enter HOLD");
    expect_float_close("current before ramp", m.active_current,
                       1.0f, 0.0f);
    expect_output("hold transition waveform", &out, 1, run_current);

    c5_mclib_update(&m, 20u + MOTOR_STOP_TIMEOUT + MOTOR_HOLD_DELAY,
                    0.0f, 0.0f, &out);
    expect_float_close("ramp delay equality", m.active_current,
                       1.0f, 0.0f);
    expect_output("ramp equality waveform", &out, 1, run_current);

    c5_mclib_update(&m, 21u + MOTOR_STOP_TIMEOUT + MOTOR_HOLD_DELAY,
                    0.0f, 0.0f, &out);
    CHECK(m.active_current < 1.0f && m.active_current >= 0.5f,
          "ramp starts strictly after hold delay");
    expect_output("first ramp waveform", &out, 1, run_current);

    for (uint32_t i = 0; i < 50000; i++)
        c5_mclib_update(&m,
                        21u + MOTOR_STOP_TIMEOUT + MOTOR_HOLD_DELAY,
                        0.0f, 0.0f, &out);
    CHECK(m.active_current >= 0.5f,
          "current ramp must not cross hold current");
    expect_float_close("current ramp floor", m.active_current,
                       0.5f, 0x1p-20f);
    expect_output("ramp floor waveform", &out, 1, hold_current);

    c5_mclib_step(&m, 22u + MOTOR_STOP_TIMEOUT + MOTOR_HOLD_DELAY);
    CHECK(m.mode == MODE_RUN, "step after hold must restore RUN");
    expect_float_close("step restores run current", m.active_current,
                       1.0f, 0.0f);
    c5_mclib_update(&m,
                    22u + MOTOR_STOP_TIMEOUT + MOTOR_HOLD_DELAY,
                    0.0f, 0.0f, &out);
    expect_output("restored run waveform", &out, 1, run_current);
}

static void
test_acquisition(void)
{
    struct c5_mclib_acq a;
    float ia = NAN, ib = NAN;
    memset(&a, 0xa5, sizeof(a));
    c5_mclib_acq_init(&a);
    for (uint16_t i = 1; i < 2000; i++)
        CHECK(c5_mclib_acquire(&a, 8192, 8192, &ia, &ib) == 0,
              "samples 1..1999 must remain in calibration");
    CHECK(c5_mclib_acquire(&a, 8192, 8192, &ia, &ib) == 1,
          "sample 2000 must install offsets and be admitted");
    expect_u32("calibrated phase A bits", float_bits(ia), 0xbd23d70au);
    expect_u32("calibrated phase B bits", float_bits(ib), 0xbd23d70au);

    CHECK(c5_mclib_acquire(&a, 8292, 8292, &ia, &ib) == 1,
          "ready acquisition must admit samples");
    expect_u32("delta-100 phase A bits", float_bits(ia), 0xbc575c28u);
    expect_u32("delta-100 phase B bits", float_bits(ib), 0xbc575c28u);
    c5_mclib_acq_polarity(&a, 1);
    c5_mclib_acquire(&a, 8292, 8292, &ia, &ib);
    expect_u32("phase A polarity bits", float_bits(ia), 0x3c575c28u);
    expect_u32("phase B unchanged polarity bits", float_bits(ib),
               0xbc575c28u);
    c5_mclib_acq_polarity(&a, 2);
    c5_mclib_acquire(&a, 8292, 8292, &ia, &ib);
    expect_u32("phase A restored polarity bits", float_bits(ia),
               0xbc575c28u);
    expect_u32("phase B polarity bits", float_bits(ib), 0x3c575c28u);

    c5_mclib_acq_init(&a);
    CHECK(c5_mclib_acquire(&a, 8192, 8192, &ia, &ib) == 0,
          "acquisition reinit must restart calibration");

    c5_mclib_acq_init(&a);
    int16_t raw = 0;
    for (uint16_t i = 0; i < 2000; i++) {
        raw = (int16_t)(8192 + ((i * 37u) % 257u) - 128);
        uint8_t admitted = c5_mclib_acquire(&a, raw, raw, &ia, &ib);
        CHECK(admitted == (i == 1999),
              "nonconstant calibration admission boundary");
    }
    expect_u32("shift-8 IIR phase A bits", float_bits(ia), 0xbca07ae1u);
    expect_u32("shift-8 IIR phase B bits", float_bits(ib), 0xbca07ae1u);
}

static uint32_t
prepare_xy_period(struct c5_mclib_motor *m, uint8_t direction,
                  uint32_t period)
{
    setup_xy(m, 0, 1000, 1000);
    c5_mclib_direction(m, direction);
    c5_mclib_enable(m, 0);
    c5_mclib_step(m, 10);
    c5_mclib_step(m, 10u + period);
    return 10u + period;
}

static void
test_xy_crossover_resonance_and_limits(void)
{
    static const uint32_t enter_fast[8] = {
        1472, 13527, 13527, 13527, 11957, 11957, 3042, 11957,
    };
    static const uint32_t stay_slow[8] = {
        2487, 12512, 12512, 12512, 10848, 10848, 4151, 10848,
    };
    static const uint32_t retain_fast[8] = {
        1334, 13665, 13665, 13665, 11765, 11765, 3234, 11765,
    };
    static const uint32_t leave_fast[8] = {
        1825, 13174, 13174, 13174, 9552, 9552, 5447, 9552,
    };
    struct c5_mclib_motor m;
    struct c5_mclib_output out;
    uint32_t now = prepare_xy_period(&m, 1, 2242);
    c5_mclib_update(&m, now, 0.0f, 0.0f, &out);
    CHECK(m.fast_mode == 1, "XY enters fast mode below 2243");
    expect_output("crossover 2242 waveform", &out, 1, enter_fast);

    now = prepare_xy_period(&m, 1, 2243);
    c5_mclib_update(&m, now, 0.0f, 0.0f, &out);
    CHECK(m.fast_mode == 0, "XY does not enter fast mode at 2243");
    expect_output("crossover 2243 waveform", &out, 1, stay_slow);

    now = prepare_xy_period(&m, 1, 2242);
    c5_mclib_update(&m, now, 0.0f, 0.0f, &out);
    c5_mclib_step(&m, now + 2443u);
    now += 2443u;
    c5_mclib_update(&m, now, 0.0f, 0.0f, &out);
    CHECK(m.fast_mode == 1, "XY retains fast mode at 2443");
    expect_output("crossover 2443 waveform", &out, 1, retain_fast);
    c5_mclib_step(&m, now + 2444u);
    now += 2444u;
    c5_mclib_update(&m, now, 0.0f, 0.0f, &out);
    CHECK(m.fast_mode == 0, "XY exits fast mode above 2443");
    expect_output("crossover 2444 waveform", &out, 1, leave_fast);

    const uint8_t slots[] = { 1, 2, 4 };
    for (uint8_t direction = 0; direction < 2; direction++) {
        for (uint8_t i = 0; i < ARRAY_SIZE(slots); i++) {
            struct c5_mclib_motor baseline, damped;
            struct c5_mclib_output base_out, damped_out;
            uint8_t tdx = slots[i];

            now = prepare_xy_period(&baseline, direction, 2467);
            prepare_xy_period(&damped, direction, 2467);
            c5_mclib_resonance(&damped, tdx, 500, 300, 1700);
            c5_mclib_update(&baseline, now, 0.0f, 0.0f, &base_out);
            c5_mclib_update(&damped, now, 0.0f, 0.0f, &damped_out);
            CHECK(outputs_equal(&base_out, &damped_out),
                  "TDX compensation must be off at period 2467");

            now = prepare_xy_period(&baseline, direction, 2468);
            prepare_xy_period(&damped, direction, 2468);
            c5_mclib_resonance(&damped, tdx, 500, 300, 1700);
            c5_mclib_update(&baseline, now, 0.0f, 0.0f, &base_out);
            c5_mclib_update(&damped, now, 0.0f, 0.0f, &damped_out);
            CHECK(!outputs_equal(&base_out, &damped_out),
                  "TDX slots 1/2/4 must affect period 2468");

            struct c5_mclib_motor selected_phase, unselected_phase;
            struct c5_mclib_output selected_out, unselected_out;
            prepare_xy_period(&selected_phase, direction, 2468);
            prepare_xy_period(&unselected_phase, direction, 2468);
            if (direction) {
                c5_mclib_resonance(&selected_phase, tdx,
                                    500, 900, 1700);
                c5_mclib_resonance(&unselected_phase, tdx,
                                    500, 300, 900);
            } else {
                c5_mclib_resonance(&selected_phase, tdx,
                                    500, 300, 900);
                c5_mclib_resonance(&unselected_phase, tdx,
                                    500, 900, 1700);
            }
            c5_mclib_update(&selected_phase, now,
                            0.0f, 0.0f, &selected_out);
            c5_mclib_update(&unselected_phase, now,
                            0.0f, 0.0f, &unselected_out);
            CHECK(!outputs_equal(&damped_out, &selected_out),
                  "changing the direction-selected TDX phase must matter");
            CHECK(outputs_equal(&damped_out, &unselected_out),
                  "changing the unselected TDX phase must not matter");
        }
    }

    struct c5_mclib_motor baseline, ignored;
    struct c5_mclib_output base_out, ignored_out;
    now = prepare_xy_period(&baseline, 1, 2468);
    prepare_xy_period(&ignored, 1, 2468);
    c5_mclib_resonance(&ignored, 6, 5000, 300, 1700);
    c5_mclib_update(&baseline, now, 0.0f, 0.0f, &base_out);
    c5_mclib_update(&ignored, now, 0.0f, 0.0f, &ignored_out);
    CHECK(outputs_equal(&base_out, &ignored_out),
          "TDX values at or above 6 must be ignored");

    setup_z(&m, 100000, 100000);
    set_pi_gains(&m, 1000000, 1000000);
    c5_mclib_enable(&m, 0);
    c5_mclib_step(&m, 20000);
    c5_mclib_update(&m, 20000, -1000.0f, 1000.0f, &out);
    check_pwm_shape("limited high-error waveform", &out);
    float normalized[2];
    for (uint8_t phase = 0; phase < 2; phase++) {
        const uint32_t *v = &out.compare[phase * 4];
        uint32_t primary = (out.signs & (1u << phase)) ? v[1] : v[0];
        normalized[phase] = (float)primary / 7500.0f - 1.0f;
    }
    float norm_squared = normalized[0] * normalized[0]
        + normalized[1] * normalized[1];
    float normalized_limit = 23.99f / 24.0f;
    CHECK(norm_squared <= normalized_limit * normalized_limit + 0.001f,
          "circular voltage limit must bound the phase vector");
    CHECK(norm_squared > 0.90f,
          "high-error vector must exercise the PI/circular limit");
}

static void
expect_disabled_untouched(const char *name, struct c5_mclib_motor *m,
                          uint32_t now)
{
    struct c5_mclib_output out;
    fill_output(&out);
    CHECK(c5_mclib_update(m, now, 0.0f, 0.0f, &out) == 0, name);
    CHECK(output_is_sentinel(&out),
          "disabled update must not touch output storage");
    CHECK(c5_mclib_stalled(m) == 0,
          "disabled motor must not expose stall");
}

static void
test_disable_admission(void)
{
    struct c5_mclib_motor m;
    struct c5_mclib_output out;

    memset(&m, 0xa5, sizeof(m));
    c5_mclib_init(&m, 2);
    c5_mclib_disable(&m);
    expect_disabled_untouched("disable before configuration", &m, 0);

    setup_z(&m, 1000, 1000);
    c5_mclib_enable(&m, 0);
    c5_mclib_disable(&m);
    expect_disabled_untouched("disable from HOLD", &m, 0);

    setup_z(&m, 1000, 1000);
    c5_mclib_enable(&m, 0);
    c5_mclib_step(&m, 20000);
    c5_mclib_disable(&m);
    expect_disabled_untouched("disable from RUN", &m, 20000);
    expect_disabled_untouched("disable persists without re-enable", &m,
                              20001);
    c5_mclib_enable(&m, 20001);
    CHECK(c5_mclib_update(&m, 20001, 0.0f, 0.0f, &out) == 1,
          "explicit enable must restore update admission");

    uint32_t now = prepare_xy_period(&m, 1, 2242);
    c5_mclib_update(&m, now, 0.0f, 0.0f, &out);
    CHECK(m.fast_mode == 1, "fast setup for disable test");
    c5_mclib_disable(&m);
    expect_disabled_untouched("disable from fast RUN", &m, now);
}

static void
test_retained_fast_history(void)
{
    struct c5_mclib_motor retained, fresh;
    struct c5_mclib_output before, after, fresh_out;
    setup_xy(&retained, 0, 1000, 1000);
    c5_mclib_stall_threshold(&retained, 1000);
    c5_mclib_enable(&retained, 0);
    uint32_t now = step_many(&retained, 0, 20000, 264);
    c5_mclib_step(&retained, now + 2242u);
    now += 2242u;
    c5_mclib_update(&retained, now, 0.0f, 0.0f, &before);
    CHECK(retained.fast_mode == 1 && retained.quarter_count >= 17,
          "setup must enter fast mode with quarter history");

    uint8_t quarters = retained.quarter_count;
    c5_mclib_disable(&retained);
    CHECK(retained.fast_mode == 1 && retained.quarter_count == quarters,
          "disable must retain fast and quarter history");
    c5_mclib_enable(&retained, now);
    c5_mclib_update(&retained, now, 0.0f, 0.0f, &after);
    CHECK(retained.fast_mode == 1 && retained.quarter_count == quarters,
          "rapid re-enable must retain short-period history");
    CHECK(c5_mclib_stalled(&retained) == 1,
          "retained quarter history may immediately expose stall");

    setup_xy(&fresh, 0, 1000, 1000);
    c5_mclib_enable(&fresh, now);
    c5_mclib_update(&fresh, now, 0.0f, 0.0f, &fresh_out);
    CHECK(!outputs_equal(&after, &fresh_out),
          "retained-history waveform must differ from a cold enable");

    c5_mclib_update(&retained, now + 37501u, 0.0f, 0.0f, &after);
    CHECK(retained.fast_mode == 0 && retained.quarter_count == 0,
          "slow update must clear retained fast/quarter history");
    CHECK(c5_mclib_stalled(&retained) == 0,
          "slow update must gate retained stall");
}

static void
test_pi_saturation_reversal_recovery(void)
{
    static const uint32_t positive_saturated[8] = {
        2198, 12801, 12801, 12801, 12801, 12801, 2198, 12801,
    };
    static const uint32_t reversed_through_zero[8] = {
        7799, 8394, 7200, 7799, 7200, 7799, 7799, 8394,
    };
    static const uint32_t negative_saturated[8] = {
        12801, 12801, 2198, 12801, 2198, 12801, 12801, 12801,
    };
    static const uint32_t recovered_through_zero[8] = {
        7200, 7799, 7799, 8394, 7799, 8394, 7200, 7799,
    };
    struct c5_mclib_motor m;
    struct c5_mclib_output out;

    setup_z(&m, 1000, 1000);
    set_pi_gains(&m, 0, 100);
    c5_mclib_enable(&m, 0);

    for (uint16_t i = 0; i < 300; i++)
        c5_mclib_update(&m, 0, 0.0f, 0.0f, &out);
    expect_output("PI positive saturation", &out, 1,
                  positive_saturated);
    for (uint16_t i = 0; i < 100; i++)
        c5_mclib_update(&m, 0, 0.0f, 0.0f, &out);
    expect_output("PI clipped positive plateau", &out, 1,
                  positive_saturated);

    // These currents transform to d=0 and q just below 2 at the enabled
    // 45-degree command angle, reversing the one-amp q-axis error.
    for (uint16_t i = 0; i < 240; i++)
        c5_mclib_update(&m, 0,
                        -0x1.6a09e6p+0f, 0x1.6a09e2p+0f, &out);
    expect_output("PI reversal through zero", &out, 2,
                  reversed_through_zero);
    for (uint16_t i = 0; i < 260; i++)
        c5_mclib_update(&m, 0,
                        -0x1.6a09e6p+0f, 0x1.6a09e2p+0f, &out);
    expect_output("PI negative saturation", &out, 2,
                  negative_saturated);
    for (uint16_t i = 0; i < 100; i++)
        c5_mclib_update(&m, 0,
                        -0x1.6a09e6p+0f, 0x1.6a09e2p+0f, &out);
    expect_output("PI clipped negative plateau", &out, 2,
                  negative_saturated);

    for (uint16_t i = 0; i < 240; i++)
        c5_mclib_update(&m, 0, 0.0f, 0.0f, &out);
    expect_output("PI recovery through zero", &out, 1,
                  recovered_through_zero);
    for (uint16_t i = 0; i < 240; i++)
        c5_mclib_update(&m, 0, 0.0f, 0.0f, &out);
    expect_output("PI recovered positive saturation", &out, 1,
                  positive_saturated);
}


static void
test_zero_period_steps(void)
{
    static const uint32_t zero_period[8] = {
        7171, 7828, 7828, 7828, 14989, 14989, 10, 14989,
    };
    struct c5_mclib_motor m;
    struct c5_mclib_output out;

    setup_xy(&m, 0, 1000, 1000);
    c5_mclib_direction(&m, 1);
    c5_mclib_enable(&m, 100);
    c5_mclib_step(&m, 100);
    c5_mclib_step(&m, 100);
    expect_u32("same-timestamp XY period", m.last_period, 0);
    CHECK(c5_mclib_update(&m, 100, 0.0f, 0.0f, &out) == 1,
          "zero-period XY update must be defined");
    expect_output("zero-period XY UDIV/interpolation waveform",
                  &out, 1, zero_period);
}

static void
test_stock_boot_microstep_defaults(void)
{
    struct c5_mclib_motor m;

    c5_mclib_init(&m, 0);
    c5_mclib_direction(&m, 1);
    c5_mclib_step(&m, 0);
    expect_u32("XY boot phase increment", m.phase, 8192u + 512u);
    c5_mclib_enable(&m, 0);
    c5_mclib_step(&m, 0);
    expect_u32("XY boot stop timeout", m.last_period, 8333333u << 3);

    c5_mclib_init(&m, 2);
    c5_mclib_direction(&m, 1);
    c5_mclib_step(&m, 0);
    expect_u32("Z boot phase increment", m.phase, 8192u + 1024u);
    c5_mclib_enable(&m, 0);
    c5_mclib_step(&m, 0);
    expect_u32("Z boot stop timeout", m.last_period, 8333333u << 4);
}

static void
test_nonfinite_control_faults(void)
{
    struct c5_mclib_motor m;
    struct c5_mclib_output out;

    setup_z(&m, 1000, 1000);
    c5_mclib_enable(&m, 0);
    m.q_pi.integral = NAN;
    CHECK(c5_mclib_update(&m, 0, 0.0f, 0.0f, &out) == 2,
          "NaN control state must report an invalid-output fault");

    setup_z(&m, 1000, 1000);
    c5_mclib_enable(&m, 0);
    CHECK(c5_mclib_update(&m, 0, INFINITY, INFINITY, &out) == 2,
          "infinite currents must report an invalid-output fault");
}

int
main(void)
{
    test_reference_waveforms();
    test_electrical_cycle();
    test_stall_gating_and_wrap();
    test_hold_run_and_current_ramp();
    test_acquisition();
    test_xy_crossover_resonance_and_limits();
    test_disable_admission();
    test_retained_fast_history();
    test_pi_saturation_reversal_recovery();
    test_zero_period_steps();
    test_stock_boot_microstep_defaults();
    test_nonfinite_control_faults();
    return failures ? 1 : 0;
}
