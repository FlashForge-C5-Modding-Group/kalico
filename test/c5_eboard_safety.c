// Creator 5 eBoard production-source safety regressions
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define __COMMAND_H
#define __SCHED_H
#define DECL_COMMAND(func, message)
#define DECL_CONSTANT(name, value)
#define DECL_INIT(func)
#define DECL_TASK(func)
#define DECL_SHUTDOWN(func)
#define SF_RESCHEDULE 1

struct timer {
    struct timer *next;
    uint_fast8_t (*func)(struct timer *);
    uint32_t waketime;
};
struct task_wake { uint8_t wake; };
typedef unsigned int irqstatus_t;

static void test_sendf(const char *format, ...);
static void test_shutdown(const char *reason);
#define sendf(format, args...) test_sendf((format), ##args)
#define shutdown(reason) test_shutdown(reason)

irqstatus_t irq_save(void);
void irq_restore(irqstatus_t flag);
uint32_t timer_read_time(void);
uint32_t timer_from_us(uint32_t us);
int timer_is_before(uint32_t time1, uint32_t time2);
void sched_wake_task(struct task_wake *wake);
uint8_t sched_check_wake(struct task_wake *wake);
void sched_wake_tasks(void);
void sched_add_timer(struct timer *timer);
void c5_eboard_set_pa_mode(uint8_t active);
uint8_t c5_eboard_pa_matches(const volatile int32_t *samples, uint16_t count);

#include "../src/c5_eboard.c"

#define CLOCKS_PER_US 144u
#define TIMEOUT_TICKS (5000u * CLOCKS_PER_US)
#define STABILITY_TICKS (2000000u * CLOCKS_PER_US)

static uint32_t model_clock;
static unsigned irq_depth;
static uint8_t irq_capture_pending;
static uint16_t irq_capture_interval;
static uint32_t irq_capture_clock;
static unsigned wake_all_count;
static unsigned timer_add_count;
static unsigned timer_active;
static unsigned pa_mode;
static unsigned pa_mode_calls;
static unsigned shutdown_count;
static char shutdown_reason[64];
static unsigned send_count;
static char send_format[80];
static int32_t sent_value;

irqstatus_t
irq_save(void)
{
    if (irq_capture_pending && !irq_depth) {
        irq_capture_pending = 0;
        model_clock = irq_capture_clock;
        c5_eboard_capture(irq_capture_interval);
    }
    return irq_depth++;
}
void irq_restore(irqstatus_t flag) { irq_depth = flag; }
uint32_t timer_read_time(void) { return model_clock; }
uint32_t timer_from_us(uint32_t us) { return us * CLOCKS_PER_US; }
int timer_is_before(uint32_t time1, uint32_t time2)
{
    return (int32_t)(time1 - time2) < 0;
}
void sched_wake_task(struct task_wake *wake) { wake->wake = 1; }
uint8_t sched_check_wake(struct task_wake *wake)
{
    uint8_t result = wake->wake;
    wake->wake = 0;
    return result;
}
void sched_wake_tasks(void) { wake_all_count++; }
void sched_add_timer(struct timer *timer)
{
    (void)timer;
    timer_add_count++;
    timer_active++;
}
void c5_eboard_set_pa_mode(uint8_t active)
{
    pa_mode = active;
    pa_mode_calls++;
}

static void
test_sendf(const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    send_count++;
    snprintf(send_format, sizeof(send_format), "%s", format);
    if (!strcmp(format, "trigger_threshold threshold=%i")
        || !strcmp(format, "peel_data value=%i"))
        sent_value = va_arg(ap, int32_t);
    va_end(ap);
}

static void
test_shutdown(const char *reason)
{
    shutdown_count++;
    snprintf(shutdown_reason, sizeof(shutdown_reason), "%s", reason);
}

static int
expect_u32(const char *name, uint32_t actual, uint32_t expected)
{
    if (actual == expected)
        return 0;
    fprintf(stderr, "%s: expected %u, got %u\n", name, expected, actual);
    return 1;
}

static int
expect_true(const char *name, int condition)
{
    if (condition)
        return 0;
    fprintf(stderr, "%s: condition was false\n", name);
    return 1;
}

static void
reset_model(uint32_t clock)
{
    memset(&eboard, 0, sizeof(eboard));
    memset(&pa, 0, sizeof(pa));
    memset(&classifier_wake, 0, sizeof(classifier_wake));
    eboard.effective_threshold = -20;
    model_clock = clock;
    irq_depth = wake_all_count = timer_add_count = timer_active = 0;
    irq_capture_pending = 0;
    pa_mode = pa_mode_calls = shutdown_count = send_count = 0;
    shutdown_reason[0] = send_format[0] = '\0';
    sent_value = 0;
    c5_eboard_init();
}

static void
capture_at(uint16_t interval, uint32_t clock)
{
    model_clock = clock;
    c5_eboard_capture(interval);
}

struct known_sample {
    uint16_t value;
    uint8_t crc;
};

static const struct known_sample shaped_samples[] = {
    { 103, 0x6b }, { 166, 0xeb }, { 230, 0xe5 },
    { 293, 0x14 }, { 356, 0x93 }, { 420, 0x9a },
    { 387, 0x28 }, { 355, 0x3d }, { 322, 0xa8 },
    { 290, 0xba }, { 257, 0xe8 }, { 225, 0x4b },
    { 192, 0xde }, { 160, 0xcc }, { 175, 0x12 },
    { 190, 0xa3 }, { 205, 0xc7 }, { 220, 0x76 },
    { 235, 0xfc }, { 250, 0x4d }, { 265, 0x98 },
    { 280, 0x29 },
};
static const struct known_sample falling_samples[] = {
    { 260, 0x81 }, { 240, 0xfa }, { 220, 0x76 },
    { 200, 0xae }, { 180, 0x14 }, { 160, 0xcc },
    { 140, 0x40 }, { 120, 0x8d }, { 100, 0x25 },
    { 80, 0xe1 }, { 60, 0x63 }, { 40, 0xbb },
};

enum frame_variant {
    FRAME_VALID,
    FRAME_WRONG_PREFIX,
    FRAME_BAD_CRC,
};

static uint8_t
calculate_frame_crc(const uint8_t *data, uint_fast8_t length)
{
    uint8_t crc = 0;
    for (uint_fast8_t i = 0; i < length; i++) {
        uint8_t current = data[i];
        for (uint_fast8_t bit = 0; bit < 8; bit++) {
            if ((crc >> 7) ^ (current & 1))
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
            current >>= 1;
        }
    }
    return crc;
}

static void
receive_sample(uint16_t value, uint8_t crc, uint16_t remaining,
               enum frame_variant variant)
{
    uint8_t frame[15] = {
        0x05, 0x00, 0x41, 0xcf, 0x05, 0xff, 0x41, 0x00, 0x00,
    };
    frame[9] = value >> 8;
    frame[10] = value;
    frame[11] = crc;
    if (variant == FRAME_WRONG_PREFIX) {
        frame[5] = 0xfe;
        frame[11] = calculate_frame_crc(&frame[4], 7);
    } else if (variant == FRAME_BAD_CRC) {
        frame[11] ^= 1;
    }
    c5_eboard_pa_receive(frame, remaining);
}

static void
receive_scorer_positive_sequence(uint16_t remaining,
                                 enum frame_variant variant)
{
    uint16_t sent = 0;
    for (uint16_t i = 0; i < 100; i++, sent++)
        receive_sample(40, 0xbb, remaining, variant);
    for (uint_fast8_t i = 0;
         i < sizeof(shaped_samples) / sizeof(shaped_samples[0]); i++, sent++)
        receive_sample(shaped_samples[i].value, shaped_samples[i].crc,
                       remaining, variant);
    for (uint_fast8_t i = 0; i < 80; i++, sent++)
        receive_sample(280, 0x29, remaining, variant);
    for (uint_fast8_t i = 0;
         i < sizeof(falling_samples) / sizeof(falling_samples[0]); i++, sent++)
        receive_sample(falling_samples[i].value, falling_samples[i].crc,
                       remaining, variant);
    while (sent < 1990) {
        receive_sample(40, 0xbb, remaining, variant);
        sent++;
    }
}

static int
run_stream_liveness(void)
{
    int failures = 0;
    const uint32_t start = 0xffff0000u;
    reset_model(start);
    eboard.eddy_state = 1;
    failures += expect_u32("startup without pulses is fail-safe",
                           c5_eboard_eddy_state(), 0);

    capture_at(45500, start + 100u);
    failures += expect_u32("first capture remains fail-safe",
                           c5_eboard_eddy_state(), 0);
    capture_at(45500, start + 45500u);
    failures += expect_u32("second capture validates stream",
                           c5_eboard_eddy_state(), 1);

    model_clock = start + 45500u + TIMEOUT_TICKS - 1u;
    failures += expect_u32("one tick before timeout remains valid",
                           c5_eboard_eddy_state(), 1);
    model_clock++;
    failures += expect_u32("exact timeout remains valid",
                           c5_eboard_eddy_state(), 1);
    model_clock++;
    failures += expect_u32("stale stream fails safe",
                           c5_eboard_eddy_state(), 0);
    eboard.eddy_state = 1;

    capture_at(45500, model_clock + 45500u);
    failures += expect_u32("first recovery capture remains fail-safe",
                           c5_eboard_eddy_state(), 0);
    capture_at(45500, model_clock + 45500u);
    failures += expect_u32("second recovery capture validates",
                           c5_eboard_eddy_state(), 1);

    capture_at(1234, model_clock + 0x10000u);
    failures += expect_u32("ambiguous DWT interval rejected",
                           c5_eboard_eddy_state(), 0);
    eboard.eddy_state = 1;
    capture_at(45500, model_clock + 45500u);
    failures += expect_u32("ambiguous recovery first capture",
                           c5_eboard_eddy_state(), 0);
    capture_at(45500, model_clock + 45500u);
    failures += expect_u32("ambiguous recovery second capture",
                           c5_eboard_eddy_state(), 1);
    irq_capture_interval = 45500;
    irq_capture_clock = model_clock + 45500u;
    irq_capture_pending = 1;
    failures += expect_u32("capture before irq exclusion stays fresh",
                           c5_eboard_eddy_state(), 1);
    failures += expect_u32("interleaved capture remains validated",
                           eboard.fresh_capture_count, 2);
    return failures;
}

static int
run_timeout_wrap(void)
{
    int failures = 0;
    const uint32_t second = 0xfffffff0u;
    reset_model(second - 100u);
    capture_at(45500, second - 100u);
    capture_at(45500, second);
    eboard.eddy_state = 1;
    model_clock = second + TIMEOUT_TICKS;
    failures += expect_u32("timeout exact boundary across wrap",
                           c5_eboard_eddy_state(), 1);
    model_clock++;
    failures += expect_u32("timeout after boundary across wrap",
                           c5_eboard_eddy_state(), 0);
    return failures;
}
static int
run_periodic_stale_latch(void)
{
    int failures = 0;
    reset_model(1000u);
    capture_at(45500, 1000u);
    capture_at(45500, 46500u);
    eboard.eddy_state = 1;

    model_clock = 46500u + TIMEOUT_TICKS + 1u;
    c5_calibration_event(&eboard.calibration_timer);
    failures += expect_u32("periodic timer latches stale capture count",
                           eboard.fresh_capture_count, 0);

    model_clock += (uint32_t)((1ull << 32) + 1234u);
    eboard.eddy_state = 1;
    failures += expect_u32("latched stale survives a full DWT wrap",
                           c5_eboard_eddy_state(), 0);
    capture_at(45500, model_clock + 100u);
    failures += expect_u32("post-wrap first capture remains fail-safe",
                           c5_eboard_eddy_state(), 0);
    capture_at(45500, model_clock + 45500u);
    failures += expect_u32("post-wrap second capture restores stream",
                           c5_eboard_eddy_state(), 1);
    return failures;
}

static void
process_sample(uint32_t sample, uint32_t clock)
{
    model_clock = clock;
    eboard.current = sample;
    eboard.capture_pending = 1;
    c5_process_calibration();
}

static void
fill_stable_ring(uint32_t sample, uint32_t clock)
{
    for (uint_fast8_t i = 0; i < C5_RING_SIZE; i++)
        process_sample(sample, clock);
}

static int
run_calibration_deadline(void)
{
    int failures = 0;
    const uint32_t start = 0xf8000000u;
    reset_model(start);
    fill_stable_ring(100, start);
    process_sample(100, start + STABILITY_TICKS - 1u);
    failures += expect_u32("calibration wrap one tick before deadline",
                           eboard.calibration_active, 1);
    process_sample(100, start + STABILITY_TICKS);
    failures += expect_u32("calibration wrap exact deadline",
                           eboard.calibration_active, 0);

    reset_model(0);
    fill_stable_ring(100, 0);
    process_sample(100, STABILITY_TICKS - 1u);
    failures += expect_u32("zero start one tick before deadline",
                           eboard.calibration_active, 1);
    process_sample(100, STABILITY_TICKS);
    failures += expect_u32("zero start exact deadline",
                           eboard.calibration_active, 0);

    reset_model(1000u);
    fill_stable_ring(100, 1000u);
    uint32_t unstable = 1000u + STABILITY_TICKS / 2u;
    for (uint32_t sample = 95; sample <= 105; sample++)
        process_sample(sample, unstable);
    uint32_t fresh = unstable + 1000u;
    for (uint_fast8_t i = 0; i < C5_RING_SIZE; i++)
        process_sample(100, fresh);
    process_sample(100, 1000u + STABILITY_TICKS);
    failures += expect_u32("MAD reset rejects old deadline",
                           eboard.calibration_active, 1);
    process_sample(100, fresh + STABILITY_TICKS - 1u);
    failures += expect_u32("MAD reset one tick before fresh deadline",
                           eboard.calibration_active, 1);
    process_sample(100, fresh + STABILITY_TICKS);
    failures += expect_u32("MAD reset fresh exact deadline",
                           eboard.calibration_active, 0);
    return failures;
}
static int
run_calibration_interruption(void)
{
    int failures = 0;
    uint32_t start_args[2] = { 11, 0 };
    uint32_t stop_args[2] = { 0, 0 };
    const uint32_t start = 1000u;

    reset_model(start);
    fill_stable_ring(100, start);
    failures += expect_u32("pre-interruption calibration active",
                           eboard.calibration_active, 1);
    failures += expect_u32("pre-interruption stability pending",
                           eboard.stability_pending, 1);

    command_pa_action(start_args);
    model_clock = start + STABILITY_TICKS + 1000u;
    command_pa_action(stop_args);
    failures += expect_u32("interruption preserves calibration mode",
                           eboard.calibration_active, 1);
    failures += expect_u32("interruption cancels stability deadline",
                           eboard.stability_pending, 0);
    failures += expect_u32("interruption clears calibration ring",
                           eboard.ring_count, 0);

    uint32_t fresh = model_clock + 1000u;
    for (uint_fast8_t i = 0; i < C5_RING_SIZE - 1; i++)
        process_sample(100, fresh);
    failures += expect_u32("partial fresh ring cannot resume stability",
                           eboard.stability_pending, 0);
    process_sample(100, fresh);
    failures += expect_u32("full fresh ring starts new stability window",
                           eboard.stability_pending, 1);
    process_sample(100, fresh + STABILITY_TICKS - 1u);
    failures += expect_u32("pause time cannot complete calibration",
                           eboard.calibration_active, 1);
    process_sample(100, fresh + STABILITY_TICKS);
    failures += expect_u32("fresh two-second window commits calibration",
                           eboard.calibration_active, 0);
    return failures;
}

static int
run_pa_frames_and_transitions(void)
{
    int failures = 0;
    uint32_t start_args[2] = { 11, 0 };
    uint32_t stop_args[2] = { 0, 0 };

    reset_model(1000u);
    eboard.eddy_state = 1;
    command_pa_action(start_args);
    failures += expect_u32("PA start hardware active", pa_mode, 1);
    failures += expect_u32("PA start is fail-safe", c5_eboard_eddy_state(), 0);
    receive_scorer_positive_sequence(3, FRAME_VALID);
    command_pa_action(stop_args);
    c5_eboard_task();
    failures += expect_u32("fixed CRC frames reach production scorer",
                           pa.verdict, 9);

    reset_model(1000u);
    command_pa_action(start_args);
    receive_scorer_positive_sequence(4, FRAME_VALID);
    command_pa_action(stop_args);
    c5_eboard_task();
    failures += expect_u32("11-byte window verdict", pa.verdict, 0);

    reset_model(1000u);
    command_pa_action(start_args);
    receive_scorer_positive_sequence(2, FRAME_VALID);
    command_pa_action(stop_args);
    c5_eboard_task();
    failures += expect_u32("13-byte window verdict", pa.verdict, 0);

    reset_model(1000u);
    command_pa_action(start_args);
    receive_scorer_positive_sequence(3, FRAME_WRONG_PREFIX);
    command_pa_action(stop_args);
    c5_eboard_task();
    failures += expect_u32("valid-CRC wrong-prefix verdict", pa.verdict, 0);

    reset_model(1000u);
    command_pa_action(start_args);
    receive_scorer_positive_sequence(3, FRAME_BAD_CRC);
    command_pa_action(stop_args);
    c5_eboard_task();
    failures += expect_u32("CRC-bit-flip verdict", pa.verdict, 0);

    reset_model(1000u);
    command_pa_action(start_args);
    command_pa_action(stop_args);
    receive_scorer_positive_sequence(3, FRAME_VALID);
    c5_eboard_task();
    failures += expect_u32("late-frame verdict", pa.verdict, 0);
    return failures;
}

static int
run_shutdown_recovery(void)
{
    int failures = 0;
    uint32_t start_args[2] = { 11, 0 };

    reset_model(2000u);
    command_pa_action(start_args);
    timer_active = 0; // sched_timer_reset() runs before shutdown handlers.
    c5_eboard_shutdown();
    failures += expect_u32("shutdown quiesces PA hardware", pa_mode, 0);
    failures += expect_u32("shutdown PA state idle", pa.state, 0);
    failures += expect_u32("shutdown invalidates probe",
                           c5_eboard_eddy_state(), 0);
    failures += expect_u32("shutdown restores normal acquisition",
                           eboard.acquisition_active, 1);
    failures += expect_u32("shutdown re-adds one permanent timer",
                           timer_active, 1);

    eboard.eddy_state = 1;
    capture_at(45500, model_clock + 100u);
    capture_at(45500, model_clock + 45500u);
    failures += expect_u32("clear recovers after two fresh captures",
                           c5_eboard_eddy_state(), 1);

    command_pa_action(start_args);
    failures += expect_u32("PA can restart after clear", pa.state, 1);
    timer_active = 0; // A second sched_timer_reset() removes the old timer.
    model_clock += 1000u;
    c5_eboard_shutdown();
    failures += expect_u32("repeated shutdown has one permanent timer",
                           timer_active, 1);
    failures += expect_u32("repeated shutdown remains fail-safe",
                           c5_eboard_eddy_state(), 0);
    return failures;
}

static int
run_commands(void)
{
    int failures = 0;
    uint32_t args[1];

    reset_model(0);
    args[0] = (uint32_t)-200;
    command_set_trigger_threshold(args);
    failures += expect_u32("threshold -200 stored exactly",
                           (uint16_t)eboard.effective_threshold,
                           (uint16_t)(int16_t)-200);
    failures += expect_u32("threshold -200 response", sent_value,
                           (uint32_t)-200);
    args[0] = 200;
    command_set_trigger_threshold(args);
    failures += expect_u32("threshold 200 stored exactly",
                           eboard.effective_threshold, 200);
    failures += expect_u32("threshold 200 response", sent_value, 200);

    unsigned responses = send_count;
    args[0] = 201;
    command_set_trigger_threshold(args);
    failures += expect_u32("threshold above range shuts down",
                           shutdown_count, 1);
    failures += expect_u32("invalid high threshold has no response",
                           send_count, responses);
    failures += expect_u32("invalid high threshold does not alter setting",
                           eboard.effective_threshold, 200);
    args[0] = (uint32_t)-201;
    command_set_trigger_threshold(args);
    failures += expect_u32("threshold below range shuts down",
                           shutdown_count, 2);
    failures += expect_u32("invalid low threshold has no response",
                           send_count, responses);
    failures += expect_true("threshold shutdown reason",
                            !strcmp(shutdown_reason,
                                    "Invalid trigger threshold"));

    eboard.baseline = 1000;
    eboard.current = 940;
    eboard.signed_delta = (uint32_t)-60;
    command_remove_peel(args);
    failures += expect_u32("remove_peel reports prior delta",
                           sent_value, (uint32_t)-60);
    failures += expect_u32("remove_peel rebases baseline",
                           eboard.baseline, 940);
    return failures;
}

int
main(void)
{
    int failures = 0;
    failures += run_stream_liveness();
    failures += run_timeout_wrap();
    failures += run_periodic_stale_latch();
    failures += run_calibration_deadline();
    failures += run_calibration_interruption();
    failures += run_pa_frames_and_transitions();
    failures += run_shutdown_recovery();
    failures += run_commands();
    return failures ? 1 : 0;
}
