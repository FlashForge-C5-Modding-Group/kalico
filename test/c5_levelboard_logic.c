// Creator 5 levelBoard production-source behavior regressions
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define __COMMAND_H
#define __SCHED_H
#define DECL_COMMAND(func, message)
#define DECL_CONSTANT(name, value)
#define DECL_INIT(func)
#define DECL_TASK(func)
static void test_sendf(const char *format, ...);
#define sendf(format, args...) test_sendf((format), ##args)
#define SF_RESCHEDULE 1

typedef unsigned int irqstatus_t;
struct timer {
    struct timer *next;
    uint_fast8_t (*func)(struct timer *);
    uint32_t waketime;
};
struct task_wake {
    uint8_t wake;
};

irqstatus_t irq_save(void);
void irq_restore(irqstatus_t flag);
uint32_t timer_read_time(void);
uint32_t timer_from_us(uint32_t us);
int timer_is_before(uint32_t time1, uint32_t time2);
void sched_wake_task(struct task_wake *wake);
uint8_t sched_check_wake(struct task_wake *wake);
void sched_add_timer(struct timer *timer);

#include "../src/c5_levelboard.c"

static void
test_sendf(const char *format, ...)
{
    (void)format;
}

#define CLOCKS_PER_US 128u
#define STABILITY_TICKS (2000000u * CLOCKS_PER_US)

static uint32_t model_clock;

irqstatus_t irq_save(void) { return 0; }
void irq_restore(irqstatus_t flag) { (void)flag; }
uint32_t timer_read_time(void) { return model_clock; }
uint32_t timer_from_us(uint32_t us) { return us * CLOCKS_PER_US; }
int timer_is_before(uint32_t time1, uint32_t time2)
{
    return (int32_t)(time1 - time2) < 0;
}
void sched_wake_task(struct task_wake *wake) { wake->wake = 1; }
uint8_t sched_check_wake(struct task_wake *wake)
{
    uint8_t pending = wake->wake;
    wake->wake = 0;
    return pending;
}
void sched_add_timer(struct timer *timer) { (void)timer; }

static int
expect_u32(const char *name, uint32_t actual, uint32_t expected)
{
    if (actual == expected)
        return 0;
    fprintf(stderr, "%s: expected %u, got %u\n", name, expected, actual);
    return 1;
}

static void
reset_detector(uint16_t initial, uint32_t clock)
{
    memset(&levelboard, 0, sizeof(levelboard));
    memset(&classifier_wake, 0, sizeof(classifier_wake));
    memset(&calibration_wake, 0, sizeof(calibration_wake));
    model_clock = clock;
    levelboard.current = initial;
    c5_levelboard_init();
}

static void
calibration_sample(uint16_t sample, uint32_t clock)
{
    model_clock = clock;
    c5_levelboard_capture(sample);
    c5_process_calibration();
}

static void
fill_stable_ring(uint16_t sample, uint32_t clock)
{
    for (uint_fast8_t i = 0; i < C5_RING_SIZE; i++)
        calibration_sample(sample, clock);
}

static int
run_wrap_vector(void)
{
    int failures = 0;
    const uint32_t start = 0xfc2f7000u;

    reset_detector(100, start);
    fill_stable_ring(100, start);
    calibration_sample(100, 0x03d09000u);
    failures += expect_u32("wrap vector remains calibrating after one second",
                           levelboard.calibration_active, 1);
    calibration_sample(100, start + STABILITY_TICKS - 1u);
    failures += expect_u32("wrap vector one tick before deadline",
                           levelboard.calibration_active, 1);
    calibration_sample(100, start + STABILITY_TICKS);
    failures += expect_u32("wrap vector exact deadline",
                           levelboard.calibration_active, 0);
    return failures;
}

static int
run_zero_and_boundary(void)
{
    int failures = 0;

    reset_detector(100, 0);
    fill_stable_ring(100, 0);
    calibration_sample(100, STABILITY_TICKS - 1);
    failures += expect_u32("raw tick zero one tick before deadline",
                           levelboard.calibration_active, 1);
    calibration_sample(100, STABILITY_TICKS);
    failures += expect_u32("raw tick zero exact deadline",
                           levelboard.calibration_active, 0);
    return failures;
}

static int
run_mad_reset(void)
{
    int failures = 0;
    const uint32_t first_start = 1000u;
    const uint32_t unstable_time = first_start + STABILITY_TICKS / 2u;
    const uint32_t fresh_start = unstable_time + 1000u;

    reset_detector(100, first_start);
    fill_stable_ring(100, first_start);
    for (uint16_t sample = 95; sample <= 105; sample++)
        calibration_sample(sample, unstable_time);
    for (uint_fast8_t i = 0; i < C5_RING_SIZE; i++)
        calibration_sample(100, fresh_start);

    calibration_sample(100, first_start + STABILITY_TICKS);
    failures += expect_u32("MAD reset rejects original deadline",
                           levelboard.calibration_active, 1);
    calibration_sample(100, fresh_start + STABILITY_TICKS - 1u);
    failures += expect_u32("MAD reset requires complete fresh interval",
                           levelboard.calibration_active, 1);
    calibration_sample(100, fresh_start + STABILITY_TICKS);
    failures += expect_u32("MAD reset fresh exact deadline",
                           levelboard.calibration_active, 0);
    return failures;
}

static int
run_normal_threshold_behavior(void)
{
    int failures = 0;
    const uint32_t start = 500000000u;

    reset_detector(100, start);
    fill_stable_ring(100, start);
    calibration_sample(100, start + STABILITY_TICKS);
    failures += expect_u32("normal calibration completes",
                           levelboard.calibration_active, 0);
    failures += expect_u32("normal calibration threshold",
                           (uint16_t)levelboard.effective_threshold, 20);

    for (uint_fast8_t i = 0; i < 4; i++)
        c5_classify_sample(140);
    failures += expect_u32("normal threshold remains clear before debounce",
                           c5_levelboard_eddy_state(), 1);
    for (uint_fast8_t i = 0; i < 2; i++)
        c5_classify_sample(140);
    failures += expect_u32("normal threshold asserts after debounce",
                           c5_levelboard_eddy_state(), 0);
    for (uint_fast8_t i = 0; i < 4; i++)
        c5_classify_sample(100);
    failures += expect_u32(
        "normal threshold remains asserted before clear debounce",
        c5_levelboard_eddy_state(), 0);
    c5_classify_sample(100);
    failures += expect_u32("normal threshold clears after debounce",
                           c5_levelboard_eddy_state(), 1);
    return failures;
}

int
main(void)
{
    int failures = 0;
    failures += run_wrap_vector();
    failures += run_zero_and_boundary();
    failures += run_mad_reset();
    failures += run_normal_threshold_behavior();
    return failures ? 1 : 0;
}
