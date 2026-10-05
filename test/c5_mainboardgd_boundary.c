// Creator 5 mainBoardGD application-layer boundary tests
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "c5_mainboardgd.h"
#include "board/irq.h"

static int failures;

#define CHECK(condition, message) do {                                  \
        if (!(condition)) {                                             \
            fprintf(stderr, "FAIL %s\n", (message));                    \
            failures++;                                                 \
        }                                                               \
    } while (0)

// Hardware boundary spies: the app layer must hand ownership of the current
// acquisition back to the hardware layer when a motor is turned off so the
// zero-current offset is measured again before the next move.
static uint8_t reinit_calls;
static uint8_t reinit_axis = 0xff;
static uint8_t pwm_calls;
static uint8_t pwm_axis = 0xff;
static uint8_t pwm_enable = 0xff;
static uint8_t shutdown_state;

void
c5_mainboardgd_acq_reinit(uint8_t axis)
{
    reinit_calls++;
    reinit_axis = axis;
}

void
c5_mainboardgd_pwm_enable(uint8_t axis, uint8_t enable)
{
    pwm_calls++;
    pwm_axis = axis;
    pwm_enable = enable;
}

uint32_t
c5_mainboardgd_motor_time(void)
{
    return 0;
}

uint8_t
sched_is_shutdown(void)
{
    return shutdown_state;
}

void *
oid_alloc(uint8_t oid, void *type, uint16_t size)
{
    static uint8_t slot[256];
    (void)type;
    (void)size;
    return &slot[oid];
}

void *
oid_lookup(uint8_t oid, void *type)
{
    static uint8_t slot[256];
    (void)type;
    return &slot[oid];
}

irqstatus_t
irq_save(void)
{
    return 0;
}

void
irq_restore(irqstatus_t flag)
{
    (void)flag;
}

void
sched_shutdown(uint_fast8_t reason)
{
    fprintf(stderr, "unexpected shutdown: %u\n", (unsigned)reason);
    exit(2);
}

uint8_t
ctr_lookup_static_string(const char *str)
{
    (void)str;
    return 0;
}

const struct command_encoder *
ctr_lookup_encoder(const char *str)
{
    (void)str;
    return 0;
}

void
command_sendf(const struct command_encoder *enc, ...)
{
    (void)enc;
}

static void
reset_spies(void)
{
    reinit_calls = 0;
    reinit_axis = 0xff;
    pwm_calls = 0;
    pwm_axis = 0xff;
    pwm_enable = 0xff;
}

static void
test_disable_rearms_calibration(void)
{
    for (uint8_t axis = 0; axis < 3; axis++) {
        reset_spies();
        c5_mainboardgd_enable(axis, 0);
        CHECK(reinit_calls == 1 && reinit_axis == axis,
              "disabling a motor must re-arm its current acquisition");
        CHECK(pwm_calls == 1 && pwm_axis == axis && pwm_enable == 0,
              "disabling a motor must turn its bridge off first");
    }

    reset_spies();
    c5_mainboardgd_enable(1, 1);
    CHECK(reinit_calls == 0,
          "enabling a motor must not disturb a calibrated acquisition");
}

static void
test_enable_after_shutdown(void)
{
    reset_spies();
    shutdown_state = 1;
    c5_mainboardgd_enable(2, 1);
    shutdown_state = 0;
    CHECK(pwm_calls == 0,
          "a shutdown motor must never re-arm its bridge");
}

int
main(void)
{
    c5_mainboardgd_init_motors();
    test_disable_rearms_calibration();
    test_enable_after_shutdown();
    return failures != 0;
}
