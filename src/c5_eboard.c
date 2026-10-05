// Creator 5 eBoard detector, calibration, and vendor commands
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h> // uint32_t
#include "board/irq.h" // irq_save
#include "board/misc.h" // timer_read_time
#include "c5_eboard.h" // c5_eboard_capture
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK

#define C5_RING_SIZE 11
#define C5_CALIBRATION_US 500
#define C5_CAPTURE_TIMEOUT_US 5000
#define C5_STABILITY_TIME_US 2000000
#define C5_LARGE_DELTA 150
#define C5_PA_SAMPLE_COUNT 2000

struct c5_detector {
    struct timer calibration_timer;
    uint32_t ring[C5_RING_SIZE];
    uint32_t current, baseline, reference, accumulator;
    uint32_t stable_deadline_clock, last_capture_clock;
    uint32_t capture_timeout_ticks, mad, filtered, signed_delta;
    uint16_t moderate_count, large_count;
    int16_t effective_threshold;
    uint8_t ring_count, ring_index, assert_count, clear_count;
    uint8_t eddy_state, immediate, capture_pending;
    uint8_t ready, normal_mode, calibration_active;
    uint8_t stability_pending, acquisition_active, fresh_capture_count;
};

enum c5_pa_mode {
    C5_PA_IDLE,
    C5_PA_ACQUIRING,
    C5_PA_READY,
};

struct c5_pa_state {
    uint32_t action, verdict;
    volatile uint8_t state;
    volatile uint16_t index;
    volatile int32_t samples[C5_PA_SAMPLE_COUNT];
};

static struct c5_detector eboard = {
    .effective_threshold = -20,
};
static struct c5_pa_state pa;
static struct task_wake classifier_wake;

static uint32_t
c5_abs_s32_wrapped(uint32_t value)
{
    return value & 0x80000000 ? 0u - value : value;
}

static uint16_t
c5_threshold_magnitude(int16_t threshold)
{
    uint16_t bits = threshold;
    return threshold < 0 ? (uint16_t)(0u - bits) : bits;
}

static void
c5_invalidate_stream(uint8_t active)
{
    eboard.acquisition_active = active;
    eboard.fresh_capture_count = 0;
    eboard.capture_pending = 0;
    classifier_wake.wake = 0;
    eboard.ring_count = 0;
    eboard.ring_index = 0;
    eboard.accumulator = 0;
    eboard.mad = 0;
    eboard.stability_pending = 0;
    eboard.moderate_count = 0;
    eboard.large_count = 0;
    eboard.immediate = 0;
    eboard.eddy_state = 0;
}

static void
c5_full_reset(void)
{
    uint32_t current = eboard.current;
    eboard.baseline = current;
    eboard.reference = current;
    eboard.accumulator = current << 2;
    eboard.ring_count = 0;
    eboard.ring_index = 0;
    eboard.moderate_count = 0;
    eboard.large_count = 0;
    eboard.stability_pending = 0;
    eboard.filtered = 0;
    eboard.assert_count = 0;
    eboard.clear_count = 0;
    eboard.immediate = 0;
    eboard.signed_delta = 0;
    eboard.eddy_state = 1;
    eboard.calibration_active = 1;
    eboard.ready = 1;
}

void
c5_eboard_capture(uint16_t interval)
{
    uint32_t now = timer_read_time();
    irqstatus_t flag = irq_save();
    if (!eboard.acquisition_active) {
        irq_restore(flag);
        return;
    }
    if (!eboard.fresh_capture_count) {
        eboard.last_capture_clock = now;
        eboard.fresh_capture_count = 1;
        irq_restore(flag);
        return;
    }
    uint32_t elapsed = now - eboard.last_capture_clock;
    eboard.last_capture_clock = now;
    if (elapsed > 0xffffu || !interval) {
        c5_invalidate_stream(1);
        irq_restore(flag);
        return;
    }
    if (eboard.fresh_capture_count < 2)
        eboard.fresh_capture_count++;
    eboard.current = interval;
    eboard.capture_pending = 1;
    sched_wake_task(&classifier_wake);
    irq_restore(flag);
}

uint8_t
c5_eboard_eddy_state(void)
{
    irqstatus_t flag = irq_save();
    uint32_t now = timer_read_time();
    if (!eboard.acquisition_active || eboard.fresh_capture_count < 2) {
        irq_restore(flag);
        return 0;
    }
    if (now - eboard.last_capture_clock > eboard.capture_timeout_ticks) {
        c5_invalidate_stream(1);
        irq_restore(flag);
        return 0;
    }
    uint8_t state = eboard.eddy_state;
    irq_restore(flag);
    return state;
}

void
c5_eboard_arm(void)
{
    irqstatus_t flag = irq_save();
    eboard.eddy_state = 0;
    eboard.filtered = 0;
    eboard.assert_count = 0;
    eboard.clear_count = 0;
    eboard.immediate = 0;
    eboard.signed_delta = 0;
    irq_restore(flag);
}

static uint32_t
c5_select_calibration(uint32_t sample)
{
    if (!eboard.normal_mode) {
        eboard.moderate_count = 0;
        eboard.large_count = 0;
        return sample;
    }

    uint16_t difference = c5_abs_s32_wrapped(sample - eboard.reference);
    uint16_t limit = c5_threshold_magnitude(eboard.effective_threshold);
    if (difference <= limit && difference <= C5_LARGE_DELTA) {
        eboard.moderate_count = 0;
        eboard.large_count = 0;
        return sample;
    }
    if (difference <= C5_LARGE_DELTA) {
        eboard.moderate_count++;
        eboard.large_count = 0;
        if (eboard.moderate_count == 200) {
            eboard.moderate_count = 0;
            eboard.baseline = sample;
            eboard.reference = sample;
            return sample;
        }
        return eboard.baseline;
    }

    eboard.large_count++;
    if (eboard.large_count == 15) {
        eboard.immediate = 1;
        eboard.large_count = 0;
    }
    return sample;
}

static void
c5_sort(uint32_t *values)
{
    for (uint_fast8_t i = 1; i < C5_RING_SIZE; i++) {
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
    uint32_t sorted[C5_RING_SIZE];
    for (uint_fast8_t i = 0; i < C5_RING_SIZE; i++)
        sorted[i] = eboard.ring[i];
    c5_sort(sorted);
    *median = sorted[C5_RING_SIZE / 2];
    for (uint_fast8_t i = 0; i < C5_RING_SIZE; i++)
        sorted[i] = eboard.ring[i] >= *median
            ? eboard.ring[i] - *median : *median - eboard.ring[i];
    c5_sort(sorted);
    *mad = sorted[C5_RING_SIZE / 2];
}

static uint32_t
c5_integer_sqrt(uint32_t mean)
{
    uint32_t x = mean;
    if (!mean)
        return 0;
    uint32_t y = (mean + 1u) >> 1;
    while (x > y) {
        x = y;
        y = ((x ? mean / x : 0u) + x) >> 1;
    }
    return x;
}

static uint32_t
c5_compute_rms(uint32_t median)
{
    uint64_t sum = 0;
    for (uint_fast8_t i = 0; i < C5_RING_SIZE; i++) {
        uint32_t difference = eboard.ring[i] - median;
        uint32_t square = difference * difference;
        sum += (int64_t)(int32_t)square;
    }
    return c5_integer_sqrt((uint32_t)(sum / C5_RING_SIZE));
}

static int16_t
c5_derive_threshold(uint32_t rms, uint32_t mad)
{
    uint32_t mad_noise = mad + (mad >> 1);
    uint32_t noise = rms > mad_noise ? rms : mad_noise;
    if (!noise)
        noise = 1;
    int16_t threshold = (int16_t)(uint16_t)(0u - 3u * noise);
    if (threshold < -60)
        return -60;
    if (threshold > -20)
        return -20;
    return threshold;
}

static void
c5_finish_calibration(uint32_t median, uint32_t mad)
{
    uint32_t rms = c5_compute_rms(median);
    int16_t threshold = c5_derive_threshold(rms, mad);
    uint32_t limit = mad ? 3u * mad : 2u;
    for (uint_fast8_t i = 0; i < eboard.ring_count; i++) {
        uint32_t difference = eboard.ring[i] >= median
            ? eboard.ring[i] - median : median - eboard.ring[i];
        if (difference > limit)
            eboard.ring[i] = median;
    }

    eboard.baseline = eboard.accumulator >> 2;
    eboard.reference = eboard.baseline;
    eboard.effective_threshold = threshold;
    eboard.calibration_active = 0;
    eboard.ring_count = 0;
    eboard.ring_index = 0;
    eboard.stability_pending = 0;
    eboard.moderate_count = 0;
    eboard.large_count = 0;
    eboard.ready = 1;
}

static void
c5_process_calibration(void)
{
    irqstatus_t flag = irq_save();
    uint8_t pending = eboard.capture_pending;
    uint32_t sample = eboard.current;
    if (pending)
        eboard.capture_pending = 0;
    irq_restore(flag);

    if (pending) {
        uint32_t selected = c5_select_calibration(sample);
        eboard.ring[eboard.ring_index++] = selected;
        if (eboard.ring_index == C5_RING_SIZE)
            eboard.ring_index = 0;
        if (eboard.ring_count < C5_RING_SIZE)
            eboard.ring_count++;

        uint32_t now = timer_read_time();
        if (!eboard.calibration_active) {
            if (eboard.ring_count == C5_RING_SIZE) {
                uint32_t median, mad;
                c5_compute_statistics(&median, &mad);
                eboard.accumulator = median * 4u;
                eboard.calibration_active = 1;
                eboard.stability_pending = 1;
                eboard.stable_deadline_clock = (now
                    + timer_from_us(C5_STABILITY_TIME_US));
            }
        } else if (eboard.ring_count == C5_RING_SIZE) {
            uint32_t median;
            c5_compute_statistics(&median, &eboard.mad);
            if (eboard.mad > 2) {
                eboard.stability_pending = 0;
            } else {
                if (!eboard.stability_pending) {
                    eboard.accumulator = median * 4u;
                    eboard.stability_pending = 1;
                    eboard.stable_deadline_clock = (now
                        + timer_from_us(C5_STABILITY_TIME_US));
                } else {
                    eboard.accumulator = (selected
                        + ((3u * eboard.accumulator) >> 2));
                }
                if (!timer_is_before(now, eboard.stable_deadline_clock))
                    c5_finish_calibration(median, eboard.mad);
            }
        }
    }

    if (eboard.ready && !eboard.normal_mode) {
        eboard.normal_mode = 1;
        eboard.reference = eboard.baseline;
    }
}

static uint_fast8_t
c5_calibration_event(struct timer *timer)
{
    irqstatus_t flag = irq_save();
    uint32_t now = timer_read_time();
    if (eboard.acquisition_active && eboard.fresh_capture_count
        && now - eboard.last_capture_clock > eboard.capture_timeout_ticks)
        c5_invalidate_stream(1);
    irq_restore(flag);
    c5_process_calibration();
    timer->waketime += timer_from_us(C5_CALIBRATION_US);
    return SF_RESCHEDULE;
}

static void
c5_classify_sample(void)
{
    irqstatus_t flag = irq_save();
    uint32_t current = eboard.current;
    uint8_t immediate = eboard.immediate;
    uint8_t raw = eboard.eddy_state;
    eboard.immediate = 0;
    if ((uint32_t)(current - 1u) > 0xfffeu) {
        irq_restore(flag);
        return;
    }
    if (immediate) {
        eboard.eddy_state = 0;
        eboard.assert_count = 0;
        eboard.clear_count = 0;
        irq_restore(flag);
        return;
    }

    uint32_t delta = current - eboard.baseline;
    eboard.signed_delta = delta;
    uint32_t error = (delta << 4) - eboard.filtered;
    eboard.filtered += (uint32_t)((int32_t)error >> 2);
    int32_t metric = (int32_t)eboard.filtered >> 4;
    int32_t threshold = eboard.effective_threshold;
    if (metric <= threshold) {
        eboard.assert_count = 0;
        if (++eboard.clear_count >= 4) {
            eboard.clear_count = 4;
            eboard.eddy_state = 0;
        }
    } else if (metric >= (int16_t)(eboard.effective_threshold + 2)) {
        eboard.clear_count = 0;
        if (++eboard.assert_count >= 3) {
            eboard.assert_count = 3;
            eboard.eddy_state = 1;
        }
    } else if (raw) {
        eboard.clear_count = 0;
    } else {
        eboard.assert_count = 0;
    }
    irq_restore(flag);
}

static uint8_t
c5_tmc_crc8(volatile uint8_t *data, uint_fast8_t length)
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

void
c5_eboard_pa_receive(volatile uint8_t *frame, uint16_t remaining)
{
    static const uint8_t prefix[9] = { 0x05, 0x00, 0x41, 0xcf, 0x05,
                                       0xff, 0x41, 0x00, 0x00 };
    if (pa.state != C5_PA_ACQUIRING)
        return;
    uint_fast8_t i;
    if (remaining == 3) {
        for (i = 0; i < sizeof(prefix); i++)
            if (frame[i] != prefix[i])
                break;
        if (i == sizeof(prefix) && c5_tmc_crc8(&frame[4], 7) == frame[11]) {
            uint16_t index = pa.index;
            pa.samples[index] = ((uint16_t)frame[9] << 8) | frame[10];
            index++;
            pa.index = index > 1989 ? 1990 : index;
        }
    }
    for (i = 0; i < 15; i++)
        frame[i] = 0;
}

// Arming the calibration timer needs a clock reading, so it must not happen
// in an init function: init functions run before timer_init() has started
// the clock unless link order happens to place them after it.  Arm it from
// the task instead, which runs only once every init function has completed.
static uint8_t calibration_unarmed;

static void
c5_schedule_calibration(void)
{
    eboard.calibration_timer.waketime = (timer_read_time()
                                         + timer_from_us(C5_CALIBRATION_US));
    sched_add_timer(&eboard.calibration_timer);
}

void
c5_eboard_task(void)
{
    if (calibration_unarmed) {
        calibration_unarmed = 0;
        c5_schedule_calibration();
    }
    irqstatus_t flag = irq_save();
    uint8_t ready = pa.state == C5_PA_READY;
    if (ready)
        pa.state = C5_PA_IDLE;
    irq_restore(flag);
    if (ready && c5_eboard_pa_matches(pa.samples, C5_PA_SAMPLE_COUNT))
        pa.verdict = 9;
    if (sched_check_wake(&classifier_wake))
        c5_classify_sample();
}
DECL_TASK(c5_eboard_task);

void
c5_eboard_init(void)
{
    eboard.capture_timeout_ticks = timer_from_us(C5_CAPTURE_TIMEOUT_US);
    eboard.calibration_timer.func = c5_calibration_event;
    c5_invalidate_stream(1);
    calibration_unarmed = 1;
}

void
c5_eboard_shutdown(void)
{
    c5_eboard_set_pa_mode(0);
    pa.state = C5_PA_IDLE;
    pa.action = 0;
    pa.verdict = 0;
    pa.index = 0;
    c5_invalidate_stream(1);
    c5_schedule_calibration();
}
DECL_SHUTDOWN(c5_eboard_shutdown);

void
command_set_trigger_threshold(uint32_t *args)
{
    int32_t threshold = (int32_t)args[0];
    if (threshold < -200 || threshold > 200) {
        shutdown("Invalid trigger threshold");
        return;
    }
    eboard.effective_threshold = threshold;
    sendf("trigger_threshold threshold=%i", threshold);
}
DECL_COMMAND(command_set_trigger_threshold,
             "set_trigger_threshold threshold=%i");

void
command_get_basic_param(uint32_t *args)
{
    (void)args;
    irqstatus_t flag = irq_save();
    uint32_t old_baseline = eboard.baseline;
    uint32_t current = eboard.current;
    uint32_t reserve = c5_abs_s32_wrapped(current - old_baseline);
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
    int32_t value = (int32_t)eboard.signed_delta;
    eboard.baseline = eboard.current;
    irq_restore(flag);
    sendf("peel_data value=%i", value);
}
DECL_COMMAND(command_remove_peel, "remove_peel action=%u");

void
command_pa_action(uint32_t *args)
{
    uint32_t action = args[0];
    irqstatus_t flag = irq_save();
    pa.action = action;
    if (action == 11) {
        c5_invalidate_stream(0);
        pa.verdict = 0;
        pa.index = 0;
        for (uint_fast16_t i = 0; i < sizeof(pa.samples); i++)
            ((volatile uint8_t *)pa.samples)[i] = 0;
        pa.state = C5_PA_ACQUIRING;
        c5_eboard_set_pa_mode(1);
    } else {
        c5_eboard_set_pa_mode(0);
        if (pa.state == C5_PA_ACQUIRING)
            pa.state = C5_PA_READY;
        c5_invalidate_stream(1);
        if (pa.state == C5_PA_READY)
            sched_wake_tasks();
    }
    irq_restore(flag);
}
DECL_COMMAND(command_pa_action, "pa_action action=%u pc=%u");

void
command_get_emcu_pa_value(uint32_t *args)
{
    (void)args;
    sendf("pa_value value=%u", pa.verdict);
}
DECL_COMMAND(command_get_emcu_pa_value, "get_emcu_pa_value");

DECL_CONSTANT("ADC_MAX", 4095);
