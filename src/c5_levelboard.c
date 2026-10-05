// Creator 5 levelBoard detector and vendor commands
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h> // uint32_t
#include "board/irq.h" // irq_save
#include "board/misc.h" // timer_read_time
#include "c5_levelboard.h" // c5_levelboard_capture
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK

#define C5_RING_SIZE 11
#define C5_CALIBRATION_US 500
#define C5_STABILITY_TIME_US 2000000
#define C5_STABILITY_MAD 2
#define C5_LARGE_DELTA 150

struct c5_detector {
    struct timer calibration_timer;
    uint32_t ring[C5_RING_SIZE];
    uint32_t offset, baseline, reference, accumulator;
    uint32_t stability_deadline, median, mad;
    int32_t commanded_threshold, filtered, signed_delta;
    uint16_t current;
    int16_t effective_threshold;
    uint8_t ring_count, ring_index;
    uint8_t eddy_state, moderate_count, large_count;
    uint8_t assert_count, clear_count;
    uint8_t immediate, capture_pending;
    uint8_t calibration_active, initialization_latch, stability_started;
};

static struct c5_detector levelboard;
static struct task_wake classifier_wake, calibration_wake;

static uint32_t
c5_abs_difference(uint32_t first, uint32_t second)
{
    return first >= second ? first - second : second - first;
}

static uint16_t
c5_abs_difference16(uint32_t first, uint32_t second)
{
    return (uint16_t)c5_abs_difference(first, second);
}

static uint16_t
c5_threshold_magnitude(int16_t threshold)
{
    int32_t value = threshold;
    return value < 0 ? (uint16_t)-value : (uint16_t)value;
}

static void
c5_full_reset(void)
{
    uint32_t base = levelboard.current + levelboard.offset;
    levelboard.baseline = base;
    levelboard.reference = base;
    levelboard.accumulator = base * 4u;
    levelboard.ring_count = 0;
    levelboard.ring_index = 0;
    levelboard.eddy_state = 1;
    levelboard.filtered = 0;
    levelboard.moderate_count = 0;
    levelboard.large_count = 0;
    levelboard.assert_count = 0;
    levelboard.clear_count = 0;
    levelboard.stability_deadline = 0;
    levelboard.stability_started = 0;
    levelboard.median = 0;
    levelboard.mad = 0;
    levelboard.immediate = 0;
    levelboard.signed_delta = 0;
    levelboard.calibration_active = 1;
    levelboard.initialization_latch = 1;
}

void
c5_levelboard_capture(uint16_t delta)
{
    irqstatus_t flag = irq_save();
    levelboard.current = delta;
    levelboard.capture_pending = 1;
    sched_wake_task(&classifier_wake);
    irq_restore(flag);
}

uint8_t
c5_levelboard_eddy_state(void)
{
    return levelboard.eddy_state;
}

void
c5_levelboard_cancel(void)
{
    irqstatus_t flag = irq_save();
    levelboard.eddy_state = 1;
    irq_restore(flag);
}

void
c5_levelboard_recover(void)
{
    irqstatus_t flag = irq_save();
    c5_full_reset();
    irq_restore(flag);
}

static uint32_t
c5_select_calibration(uint32_t sample)
{
    uint16_t difference = c5_abs_difference16(sample, levelboard.reference);
    uint16_t limit = c5_threshold_magnitude(levelboard.effective_threshold);

    if (difference <= limit && difference <= C5_LARGE_DELTA) {
        levelboard.moderate_count = 0;
        levelboard.large_count = 0;
        return sample;
    }
    if (difference <= C5_LARGE_DELTA) {
        levelboard.large_count = 0;
        levelboard.moderate_count++;
        if (levelboard.moderate_count == 100) {
            levelboard.moderate_count = 0;
            levelboard.reference = sample;
            return sample;
        }
        return levelboard.reference;
    }

    levelboard.large_count++;
    if (levelboard.large_count == 15) {
        levelboard.immediate = 1;
        sched_wake_task(&classifier_wake);
    }
    return sample;
}

static void
c5_sort(uint32_t *values, uint_fast8_t count)
{
    for (uint_fast8_t i = 1; i < count; i++) {
        uint32_t value = values[i];
        uint_fast8_t position = i;
        while (position && values[position - 1] > value) {
            values[position] = values[position - 1];
            position--;
        }
        values[position] = value;
    }
}

static void
c5_compute_statistics(uint32_t *median, uint32_t *mad)
{
    uint_fast8_t count = levelboard.ring_count;
    if (!count) {
        *median = 0;
        *mad = 0;
        return;
    }

    uint32_t sorted[C5_RING_SIZE];
    for (uint_fast8_t i = 0; i < count; i++)
        sorted[i] = levelboard.ring[i];
    c5_sort(sorted, count);
    uint32_t middle = sorted[count / 2];

    for (uint_fast8_t i = 0; i < count; i++)
        sorted[i] = c5_abs_difference(levelboard.ring[i], middle);
    c5_sort(sorted, count);
    *median = middle;
    *mad = sorted[count / 2];
}

static int32_t
c5_square_term(int32_t difference)
{
    uint32_t value = (uint32_t)difference;
    return (int32_t)(value * value);
}

static uint32_t
c5_integer_sqrt(uint32_t value)
{
    uint32_t result = 0;
    uint32_t bit = 1u << 30;
    while (bit > value)
        bit >>= 2;
    while (bit) {
        if (value >= result + bit) {
            value -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    return result;
}

static uint32_t
c5_compute_rms(uint32_t median)
{
    uint_fast8_t count = levelboard.ring_count;
    if (!count)
        return 0;

    int64_t sum = 0;
    for (uint_fast8_t i = 0; i < count; i++) {
        int32_t difference = (int32_t)(levelboard.ring[i] - median);
        sum += c5_square_term(difference);
    }
    return c5_integer_sqrt((uint32_t)(sum / count));
}

static int16_t
c5_derive_threshold(uint32_t rms, uint32_t mad)
{
    uint32_t mad_noise = (mad * 3u) / 2u;
    uint32_t noise = rms > mad_noise ? rms : mad_noise;
    if (!noise)
        noise = 1;
    int16_t threshold = (int16_t)(uint16_t)(noise * 3u);
    if (threshold < 20)
        return 20;
    if (threshold > 60)
        return 60;
    return threshold;
}

static void
c5_clip_ring(uint32_t median, uint32_t mad)
{
    uint32_t limit = mad ? mad * 3u : C5_STABILITY_MAD;
    for (uint_fast8_t i = 0; i < levelboard.ring_count; i++)
        if (c5_abs_difference(levelboard.ring[i], median) > limit)
            levelboard.ring[i] = median;
}

static void
c5_finish_calibration(uint32_t median, uint32_t mad)
{
    uint32_t rms = c5_compute_rms(median);
    int16_t threshold = c5_derive_threshold(rms, mad);
    c5_clip_ring(median, mad);

    uint32_t base = levelboard.offset + levelboard.accumulator / 4u;
    levelboard.baseline = base;
    levelboard.reference = base;
    levelboard.effective_threshold = threshold;
    levelboard.calibration_active = 0;
}


static void
c5_process_calibration(void)
{
    irqstatus_t flag = irq_save();
    uint8_t pending = levelboard.capture_pending;
    uint16_t sample = levelboard.current;
    levelboard.capture_pending = 0;
    irq_restore(flag);

    if (!pending || !levelboard.calibration_active)
        return;

    if (!levelboard.initialization_latch) {
        uint32_t base = sample + levelboard.offset;
        levelboard.baseline = base;
        levelboard.reference = base;
        levelboard.accumulator = base * 4u;
        levelboard.initialization_latch = 1;
    }

    uint32_t selected = c5_select_calibration(sample);
    levelboard.ring[levelboard.ring_index] = selected;
    levelboard.ring_index++;
    if (levelboard.ring_index == C5_RING_SIZE)
        levelboard.ring_index = 0;
    if (levelboard.ring_count < C5_RING_SIZE)
        levelboard.ring_count++;
    levelboard.accumulator = sample + (3u * levelboard.accumulator) / 4u;

    if (levelboard.ring_count < C5_RING_SIZE)
        return;

    c5_compute_statistics(&levelboard.median, &levelboard.mad);
    if (levelboard.mad > C5_STABILITY_MAD) {
        levelboard.stability_started = 0;
        return;
    }

    uint32_t now = timer_read_time();
    if (!levelboard.stability_started) {
        levelboard.stability_deadline = (now
                                         + timer_from_us(
                                             C5_STABILITY_TIME_US));
        levelboard.stability_started = 1;
        return;
    }
    if (!timer_is_before(now, levelboard.stability_deadline))
        c5_finish_calibration(levelboard.median, levelboard.mad);
}

static int32_t
c5_classify_sample(uint16_t sample)
{
    irqstatus_t flag = irq_save();
    uint8_t immediate = levelboard.immediate;
    levelboard.immediate = 0;
    if (!sample) {
        int32_t metric = levelboard.filtered >> 4;
        irq_restore(flag);
        return metric;
    }

    levelboard.signed_delta = (int32_t)((uint32_t)sample
                                        - levelboard.baseline);
    if (immediate) {
        levelboard.eddy_state = 0;
        levelboard.assert_count = 0;
        levelboard.clear_count = 0;
        int32_t metric = levelboard.filtered >> 4;
        irq_restore(flag);
        return metric;
    }

    uint16_t difference = c5_abs_difference16(sample, levelboard.baseline);
    levelboard.filtered += ((int32_t)(16u * difference)
                            - levelboard.filtered) >> 2;
    int32_t metric = levelboard.filtered >> 4;
    int32_t threshold = levelboard.effective_threshold;
    if (metric >= threshold) {
        levelboard.clear_count = 0;
        levelboard.assert_count++;
        if (levelboard.assert_count >= 4) {
            levelboard.eddy_state = 0;
            levelboard.assert_count = 0;
        }
    } else if (metric < threshold - 2) {
        levelboard.assert_count = 0;
        levelboard.clear_count++;
        if (levelboard.clear_count >= 3) {
            levelboard.eddy_state = 1;
            levelboard.clear_count = 0;
        }
    } else {
        levelboard.assert_count = 0;
        levelboard.clear_count = 0;
    }
    irq_restore(flag);
    return metric;
}

static uint_fast8_t
c5_levelboard_calibration_event(struct timer *timer)
{
    sched_wake_task(&calibration_wake);
    timer->waketime += timer_from_us(C5_CALIBRATION_US);
    return SF_RESCHEDULE;
}

// Arming the calibration timer needs a clock reading, so it must not happen
// in an init function: init functions run before timer_init() has started
// the clock unless link order happens to place them after it.  Arm it from
// the task instead, which runs only once every init function has completed.
static uint8_t calibration_unarmed;

void
c5_levelboard_task(void)
{
    if (calibration_unarmed) {
        calibration_unarmed = 0;
        levelboard.calibration_timer.waketime = (timer_read_time()
                                                 + timer_from_us(
                                                     C5_CALIBRATION_US));
        sched_add_timer(&levelboard.calibration_timer);
    }
    if (sched_check_wake(&calibration_wake))
        c5_process_calibration();
    if (sched_check_wake(&classifier_wake)) {
        irqstatus_t flag = irq_save();
        uint16_t sample = levelboard.current;
        irq_restore(flag);
        c5_classify_sample(sample);
    }
}
DECL_TASK(c5_levelboard_task);

void
c5_levelboard_init(void)
{
    levelboard.commanded_threshold = 25;
    levelboard.effective_threshold = 25;
    c5_full_reset();
    levelboard.calibration_timer.func = c5_levelboard_calibration_event;
    calibration_unarmed = 1;
}
DECL_INIT(c5_levelboard_init);

void
command_set_trigger_threshold(uint32_t *args)
{
    int32_t threshold = (int32_t)args[0];
    levelboard.commanded_threshold = threshold;
    levelboard.effective_threshold = (int16_t)(uint16_t)threshold;
    sendf("trigger_threshold threshold=%i", threshold);
}
DECL_COMMAND(command_set_trigger_threshold,
             "set_trigger_threshold threshold=%i");

void
command_get_basic_param(uint32_t *args)
{
    (void)args;
    irqstatus_t flag = irq_save();
    uint32_t old_baseline = levelboard.baseline;
    uint32_t current = levelboard.current;
    uint32_t reserve = c5_abs_difference(current, old_baseline);
    c5_full_reset();
    irq_restore(flag);
    sendf("param_value value=%u reserve=%u", old_baseline, reserve);
}
DECL_COMMAND(command_get_basic_param, "get_basic_param num=%u");

void
command_remove_peel(uint32_t *args)
{
    (void)args;
    irqstatus_t flag = irq_save();
    int32_t value = levelboard.signed_delta;
    levelboard.baseline = levelboard.current;
    irq_restore(flag);
    sendf("peel_data value=%i", value);
}
DECL_COMMAND(command_remove_peel, "remove_peel action=%u");

DECL_CONSTANT("ADC_MAX", 4095);
