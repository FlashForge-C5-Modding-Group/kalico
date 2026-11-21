// Creator 5 mainBoardGD motor ownership and stock commands
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h>
#include <string.h> // memset
#include "autoconf.h" // CONFIG_CLOCK_FREQ
#include "basecmd.h" // oid_alloc
#include "board/internal.h" // DWT
#include "board/irq.h" // irq_save
#include "board/misc.h" // timer_read_time
#include "c5_mainboardgd.h" // c5_mainboardgd_init_motors
#include "command.h" // DECL_COMMAND
#include "sched.h" // sched_is_shutdown

#define C5_MOTOR_COUNT 3

struct c5_mclib_oid {
    struct c5_mclib_motor *motor;
};

static struct c5_mclib_motor motors[C5_MOTOR_COUNT];
void command_config_mclib(uint32_t *args);

DECL_ENUMERATION("stepper", "stepper_x", 0);
DECL_ENUMERATION("stepper", "stepper_y", 1);
DECL_ENUMERATION("stepper", "stepper_z", 2);
DECL_ENUMERATION("stepper", "extruder", 3);

static struct c5_mclib_motor *
get_motor(uint8_t axis)
{
    if (axis >= C5_MOTOR_COUNT)
        shutdown("Invalid mclib configuration");
    return &motors[axis];
}

void
c5_mainboardgd_init_motors(void)
{
    uint_fast8_t axis;
    for (axis = 0; axis < C5_MOTOR_COUNT; axis++)
        c5_mclib_init(&motors[axis], axis);
}

void
c5_mainboardgd_direction(uint8_t axis, uint8_t value)
{
    irqstatus_t flag = irq_save();
    c5_mclib_direction(get_motor(axis), value);
    irq_restore(flag);
}

#define CLOCKS_PER_MOTOR_TICK (CONFIG_CLOCK_FREQ / C5_MOTOR_TIME_FREQ)
_Static_assert(CONFIG_CLOCK_FREQ % C5_MOTOR_TIME_FREQ == 0,
               "mclib motor time must divide the MCU clock");

// Scheduled MCU clock of the step event being run, set by the step queue
// just before it toggles the step pin
static uint32_t step_clock;
static uint8_t step_clock_valid;

void
c5_mainboardgd_step_clock(uint32_t clock)
{
    step_clock = clock;
    step_clock_valid = 1;
}

// Motor time of an MCU clock at most a second in the past
static uint32_t
motor_time_of(uint32_t clock)
{
    uint32_t late = timer_read_time() - clock;
    if (late > CONFIG_CLOCK_FREQ)
        late = 0;
    return c5_mainboardgd_motor_time() - late / CLOCKS_PER_MOTOR_TICK;
}

void
c5_mainboardgd_step(uint8_t axis)
{
    irqstatus_t flag = irq_save();
    uint32_t when = step_clock_valid ? motor_time_of(step_clock)
        : c5_mainboardgd_motor_time();
    step_clock_valid = 0;
    c5_mclib_step(get_motor(axis), when);
    irq_restore(flag);
}

// Cycles run by each preempting interrupt (running totals), and the longest
// single Klipper timer dispatch since the last timing query (it masks every
// interrupt, the motor control included)
static volatile uint32_t irq_cycles[C5_IRQ_SOURCES];
static uint32_t timer_irq_max;

void
c5_mainboardgd_irq_cycles(uint8_t source, uint32_t cycles)
{
    irq_cycles[source] += cycles;
    if (source == C5_IRQ_TIMER && cycles > timer_irq_max)
        timer_irq_max = cycles;
}

// Per axis since the last timing query: the longest update counting only
// its own cycles, and the most each interrupt took out of a single update
struct preempt_stats {
    uint32_t own_max, stolen_max[C5_IRQ_SOURCES];
};
static struct preempt_stats preempt[C5_MOTOR_COUNT];

// The step queue's next step for an axis, in MCU clock ticks
void
c5_mainboardgd_set_next_step(uint8_t axis, uint32_t clock, uint8_t due)
{
    struct c5_mclib_motor *motor = get_motor(axis);
    irqstatus_t flag = irq_save();
    if (!due) {
        c5_mclib_next_step(motor, 0, C5_NEXT_NONE);
    } else {
        int32_t delta = clock - timer_read_time();
        if (delta < 0)
            delta = 0;
        if (delta > CONFIG_CLOCK_FREQ)
            // Too far ahead to place reliably; guess from the last period
            c5_mclib_next_step(motor, 0, C5_NEXT_UNKNOWN);
        else
            c5_mclib_next_step(motor, c5_mainboardgd_motor_time()
                               + delta / CLOCKS_PER_MOTOR_TICK, C5_NEXT_AT);
    }
    irq_restore(flag);
}

void
c5_mainboardgd_enable(uint8_t axis, uint8_t enable)
{
    struct c5_mclib_motor *motor = get_motor(axis);
    irqstatus_t flag = irq_save();
    if (enable) {
        if (!sched_is_shutdown()) {
            c5_mclib_enable(motor, c5_mainboardgd_motor_time());
            c5_mainboardgd_pwm_enable(axis, 1);
        }
    } else {
        c5_mainboardgd_pwm_enable(axis, 0);
        c5_mclib_disable(motor);
        c5_mainboardgd_acq_reinit(axis);
    }
    irq_restore(flag);
}

uint8_t
c5_mainboardgd_stalled(uint8_t axis)
{
    return c5_mclib_stalled(get_motor(axis));
}

C5_FAST uint8_t
c5_mainboardgd_control(uint8_t axis, uint32_t now, float ia, float ib,
                       struct c5_mclib_output *out)
{
    // axis comes from the board's own sample routing, always 0..2
    struct c5_mclib_motor *motor = &motors[axis];
    uint32_t before[C5_IRQ_SOURCES];
    uint_fast8_t i;
    for (i = 0; i < C5_IRQ_SOURCES; i++)
        before[i] = irq_cycles[i];
    uint32_t start = DWT->CYCCNT;
    uint8_t status = c5_mclib_update(motor, now, ia, ib, out);
    uint32_t cycles = DWT->CYCCNT - start;
    c5_mclib_note_timing(motor, cycles, now);
    struct preempt_stats *p = &preempt[axis];
    uint32_t stolen_total = 0;
    for (i = 0; i < C5_IRQ_SOURCES; i++) {
        uint32_t stolen = irq_cycles[i] - before[i];
        stolen_total += stolen;
        if (stolen > p->stolen_max[i])
            p->stolen_max[i] = stolen;
    }
    // Nested interrupts can count twice; never report less than zero
    uint32_t own = cycles > stolen_total ? cycles - stolen_total : 0;
    if (own > p->own_max)
        p->own_max = own;
    return status;
}

static struct c5_mclib_motor *
lookup_motor(uint8_t oid)
{
    struct c5_mclib_oid *binding = oid_lookup(oid, command_config_mclib);
    return binding->motor;
}

void
command_config_mclib(uint32_t *args)
{
    uint32_t selector = args[1];
    if (selector == 3)
        shutdown("Unsupported mainBoardGD motor");
    if (selector > 3 || !args[3])
        shutdown("Invalid mclib configuration");

    struct c5_mclib_oid *binding = oid_alloc(
        args[0], command_config_mclib, sizeof(*binding));
    binding->motor = &motors[selector];

    struct c5_mclib_motor prepared;
    c5_mclib_configure(&prepared, args[2], args[3], args[4]);
    irqstatus_t flag = irq_save();
    binding->motor->resistance = prepared.resistance;
    binding->motor->inductance = prepared.inductance;
    binding->motor->motor_constant = prepared.motor_constant;
    binding->motor->d_pi.kp = prepared.d_pi.kp;
    binding->motor->q_pi.kp = prepared.q_pi.kp;
    binding->motor->d_pi.ki = prepared.d_pi.ki;
    binding->motor->q_pi.ki = prepared.q_pi.ki;
    binding->motor->pi_kp_base = prepared.pi_kp_base;
    binding->motor->pi_ki_base = prepared.pi_ki_base;
    binding->motor->observer.a = prepared.observer.a;
    binding->motor->observer.b = prepared.observer.b;
    irq_restore(flag);
}
DECL_COMMAND(command_config_mclib,
             "config_mclib oid=%c stepper=%u rs=%u ls=%u km=%u");

void
command_mclib_config_microstep(uint32_t *args)
{
    struct c5_mclib_motor *motor = lookup_motor(args[0]);
    struct c5_mclib_motor prepared;
    c5_mclib_microstep(&prepared, args[1], args[2]);
    irqstatus_t flag = irq_save();
    motor->interpolation = prepared.interpolation;
    motor->exponent = prepared.exponent;
    motor->phase_increment = prepared.phase_increment;
    motor->stop_timeout = prepared.stop_timeout;
    motor->crossover_threshold = prepared.crossover_threshold;
    motor->crossover_hysteresis = prepared.crossover_hysteresis;
    motor->damping_threshold = prepared.damping_threshold;
    motor->slow_threshold = prepared.slow_threshold;
    irq_restore(flag);
}
DECL_COMMAND(command_mclib_config_microstep,
             "mclib_config_microstep oid=%c interpolate=%c mstep=%u");

void
command_mclib_config_stalldetect(uint32_t *args)
{
    struct c5_mclib_motor *motor = lookup_motor(args[0]);
    struct c5_mclib_motor prepared;
    c5_mclib_stall_threshold(&prepared, args[1]);
    irqstatus_t flag = irq_save();
    motor->stall_threshold = prepared.stall_threshold;
    irq_restore(flag);
}
DECL_COMMAND(command_mclib_config_stalldetect,
             "mclib_config_stalldetect oid=%c stallthrs=%u");

void
command_mclib_set_current(uint32_t *args)
{
    struct c5_mclib_motor *motor = lookup_motor(args[0]);
    struct c5_mclib_motor prepared;
    prepared.ramp_ticks = motor->ramp_ticks;
    c5_mclib_current(&prepared, args[1], args[2]);
    irqstatus_t flag = irq_save();
    motor->run_current = prepared.run_current;
    motor->hold_current = prepared.hold_current;
    motor->decay_increment = prepared.decay_increment;
    irq_restore(flag);
}
DECL_COMMAND(command_mclib_set_current,
             "mclib_set_current oid=%c run_current=%u hold_current=%u");

void
command_mclib_set_resonance_damp(uint32_t *args)
{
    struct c5_mclib_motor *motor = lookup_motor(args[0]);
    uint_fast8_t slot = args[1] >> 1;
    if (slot >= 3)
        return;
    struct c5_mclib_motor prepared;
    c5_mclib_resonance(&prepared, args[1], args[2], args[3], args[4]);
    irqstatus_t flag = irq_save();
    motor->resonance_amplitude[slot] = prepared.resonance_amplitude[slot];
    motor->resonance_phase_forward[slot] =
        prepared.resonance_phase_forward[slot];
    motor->resonance_phase_reverse[slot] =
        prepared.resonance_phase_reverse[slot];
    irq_restore(flag);
}
DECL_COMMAND(command_mclib_set_resonance_damp,
             "mclib_set_resonance_damp oid=%c tdx=%c amp=%u"
                          " phase1=%u phase2=%u");

static int32_t
milli(float value)
{
    float scaled = value * 1000.0f;
    if (scaled > 2000000000.0f)
        return 2000000000;
    if (scaled < -2000000000.0f)
        return -2000000000;
    return (int32_t)scaled;
}

void
command_mclib_query_status(uint32_t *args)
{
    uint8_t oid = args[0];
    struct c5_mclib_motor *motor = lookup_motor(oid);
    struct c5_mclib_health h;
    irqstatus_t flag = irq_save();
    c5_mclib_take_health(motor, &h);
    uint8_t stall = motor->stall;
    irq_restore(flag);
    sendf("mclib_status oid=%c stall=%c stalls=%u slips=%i slip_total=%u"
          " samples=%u tracked=%u torque_min=%i torque_max=%i err_min=%i"
          " err_max=%i"
          , oid, stall, h.stall_count, h.slips, h.slip_total, h.samples
          , h.tracked, milli(h.torque_min), milli(h.torque_max)
          , milli(h.error_min), milli(h.error_max));
}
DECL_COMMAND(command_mclib_query_status, "mclib_query_status oid=%c");

// One slip log entry per query (a burst overflows the transmit buffer)
void
command_mclib_query_slip_event(uint32_t *args)
{
    uint8_t oid = args[0], index = args[1];
    struct c5_mclib_motor *motor = lookup_motor(oid);
    struct c5_mclib_slip_event e;
    irqstatus_t flag = irq_save();
    uint8_t ok = c5_mclib_slip_event(motor, index, &e);
    irq_restore(flag);
    if (!ok) {
        sendf("mclib_slip_event oid=%c index=%c valid=%c kind=%c source=%c"
              " state=%c level=%i segment=%u step_time=%u period=%u"
              " error=%i step=%i drift=%i speed=%i omega=%i",
              oid, index, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        return;
    }
    uint8_t state = e.mode | (e.fast_mode << 2) | (e.slow << 3)
        | (e.direction << 4);
    sendf("mclib_slip_event oid=%c index=%c valid=%c kind=%c source=%c"
          " state=%c level=%i segment=%u step_time=%u period=%u"
          " error=%i step=%i drift=%i speed=%i omega=%i",
          oid, index, 1, e.kind, e.source, state, (int32_t)e.level,
          e.segment, e.step_time, e.period, milli(e.error), milli(e.step),
          milli(e.drift), (int32_t)e.speed, (int32_t)e.omega);
}
DECL_COMMAND(command_mclib_query_slip_event,
             "mclib_query_slip_event oid=%c index=%c");

void
command_mclib_clear_slip_log(uint32_t *args)
{
    struct c5_mclib_motor *motor = lookup_motor(args[0]);
    irqstatus_t flag = irq_save();
    c5_mclib_clear_slip_log(motor);
    irq_restore(flag);
}
DECL_COMMAND(command_mclib_clear_slip_log, "mclib_clear_slip_log oid=%c");

void
command_mclib_set_interpolation(uint32_t *args)
{
    struct c5_mclib_motor *motor = lookup_motor(args[0]);
    irqstatus_t flag = irq_save();
    c5_mclib_interpolation(motor, args[1]);
    irq_restore(flag);
}
DECL_COMMAND(command_mclib_set_interpolation,
             "mclib_set_interpolation oid=%c mode=%c");

void
command_mclib_query_timing(uint32_t *args)
{
    uint8_t oid = args[0];
    struct c5_mclib_motor *motor = lookup_motor(oid);
    struct c5_mclib_timing t;
    uint8_t axis = motor - motors;
    irqstatus_t flag = irq_save();
    c5_mclib_take_timing(motor, &t);
    uint32_t timer_max = timer_irq_max;
    timer_irq_max = 0;
    struct preempt_stats p = preempt[axis];
    memset(&preempt[axis], 0, sizeof(preempt[axis]));
    irq_restore(flag);
    uint32_t d_integral = t.d_integral_samples
        ? (uint32_t)(t.d_integral_sum * 1000.0f
                     / (float)t.d_integral_samples) : 0u;
    // RMS current error while holding, in microamps
    uint32_t hold_noise = t.hold_noise_samples
        ? (uint32_t)(__builtin_sqrtf(t.hold_noise_sum
                                     / (float)t.hold_noise_samples)
                     * 1000000.0f) : 0u;
    sendf("mclib_timing oid=%c max=%u total=%u samples=%u"
          " d_integral=%u d_samples=%u hold_noise=%u hold_samples=%u",
          oid, t.max, t.total, t.samples, d_integral, t.d_integral_samples,
          hold_noise, t.hold_noise_samples);
    // A second message: one with every field can pass the 64 byte limit
    sendf("mclib_timing_run oid=%c run_max=%u run_avg=%u run_samples=%u"
          " gap_max=%u missed=%u timer_max=%u own_max=%u serial_max=%u"
          " timer_steal=%u tick_steal=%u",
          oid, t.run_max,
          t.run_samples ? (uint32_t)(t.run_total / t.run_samples) : 0u,
          t.run_samples, t.gap_max, t.missed, timer_max, p.own_max,
          p.stolen_max[C5_IRQ_SERIAL], p.stolen_max[C5_IRQ_TIMER],
          p.stolen_max[C5_IRQ_TICK]);
    // Current tracking ripple RMS in microamps
    uint32_t ripple = t.ripple_samples
        ? (uint32_t)(__builtin_sqrt(t.ripple_sum / t.ripple_samples)
                     * 1000000.0) : 0u;
    sendf("mclib_quality oid=%c ripple=%u ripple_samples=%u",
          oid, ripple, t.ripple_samples);
}
DECL_COMMAND(command_mclib_query_timing, "mclib_query_timing oid=%c");

void
command_mclib_config_slip(uint32_t *args)
{
    struct c5_mclib_motor *motor = lookup_motor(args[0]);
    irqstatus_t flag = irq_save();
    c5_mclib_slip_mode(motor, args[1]);
    irq_restore(flag);
}
DECL_COMMAND(command_mclib_config_slip, "mclib_config_slip oid=%c mode=%c");

void
command_mclib_set_injection(uint32_t *args)
{
    struct c5_mclib_motor *motor = lookup_motor(args[0]);
    irqstatus_t flag = irq_save();
    c5_mclib_injection_scale(motor, args[1]);
    irq_restore(flag);
}
DECL_COMMAND(command_mclib_set_injection,
             "mclib_set_injection oid=%c scale=%u");

void
command_mclib_set_feedforward(uint32_t *args)
{
    struct c5_mclib_motor *motor = lookup_motor(args[0]);
    irqstatus_t flag = irq_save();
    c5_mclib_feedforward(motor, args[1]);
    irq_restore(flag);
}
DECL_COMMAND(command_mclib_set_feedforward,
             "mclib_set_feedforward oid=%c gain=%u");

void
command_mclib_set_pi_scale(uint32_t *args)
{
    struct c5_mclib_motor *motor = lookup_motor(args[0]);
    irqstatus_t flag = irq_save();
    c5_mclib_pi_scale(motor, args[1]);
    irq_restore(flag);
}
DECL_COMMAND(command_mclib_set_pi_scale,
             "mclib_set_pi_scale oid=%c percent=%u");

void
command_mclib_arm_harmonics(uint32_t *args)
{
    struct c5_mclib_motor *motor = lookup_motor(args[0]);
    irqstatus_t flag = irq_save();
    c5_mclib_arm_harmonics(motor);
    irq_restore(flag);
}
DECL_COMMAND(command_mclib_arm_harmonics, "mclib_arm_harmonics oid=%c");

// Mean load angle harmonic (cos, sin) in 1e-7 electrical radians. One
// reply per query: a burst of replies overflows the transmit buffer.
void
command_mclib_query_harmonics(uint32_t *args)
{
    uint8_t oid = args[0], dir = args[1] ? 1u : 0u, harmonic = args[2];
    struct c5_mclib_motor *motor = lookup_motor(oid);
    if (harmonic < 1u || harmonic > C5_ANALYZE_HARMONICS)
        shutdown("Invalid mclib harmonic");
    float c[C5_ANALYZE_HARMONICS], s[C5_ANALYZE_HARMONICS];
    uint32_t n;
    irqstatus_t flag = irq_save();
    c5_mclib_take_harmonics(motor, dir, c, s, &n);
    irq_restore(flag);
    float scale = n ? 2.0f * 1e7f / (float)n : 0.0f;
    sendf("mclib_harmonics oid=%c dir=%c harmonic=%c count=%u c=%i s=%i",
          oid, dir, harmonic, n, (int32_t)(c[harmonic - 1u] * scale),
          (int32_t)(s[harmonic - 1u] * scale));
}
DECL_COMMAND(command_mclib_query_harmonics,
             "mclib_query_harmonics oid=%c dir=%c harmonic=%c");

void
command_mclib_set_ripple(uint32_t *args)
{
    struct c5_mclib_motor *motor = lookup_motor(args[0]);
    irqstatus_t flag = irq_save();
    c5_mclib_ripple(motor, args[1], args[2], args[3], args[4], args[5],
                    args[6], args[7]);
    irq_restore(flag);
}
DECL_COMMAND(command_mclib_set_ripple,
             "mclib_set_ripple oid=%c harmonic=%c dir=%c index=%c count=%c"
             " freq=%u amp=%i phase=%i");
