// Creator 5 mainBoardGD motor ownership and stock commands
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h>
#include "basecmd.h" // oid_alloc
#include "board/irq.h" // irq_save
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

void
c5_mainboardgd_step(uint8_t axis)
{
    irqstatus_t flag = irq_save();
    c5_mclib_step(get_motor(axis), c5_mainboardgd_motor_time());
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

uint8_t
c5_mainboardgd_control(uint8_t axis, uint32_t now, float ia, float ib,
                       struct c5_mclib_output *out)
{
    return c5_mclib_update(get_motor(axis), now, ia, ib, out);
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
