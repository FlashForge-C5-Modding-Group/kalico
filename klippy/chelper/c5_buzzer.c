// Multi-voice music on the Creator 5's host PWM buzzer
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.
//
// A real-time thread mixes square waves and noise and writes each sample as a
// PWM duty cycle, which Python is too slow and too jittery to do.
#define _GNU_SOURCE
#include <fcntl.h> // open
#include <pthread.h> // pthread_create
#include <sched.h> // cpu_set_t
#include <stdint.h> // uint32_t
#include <stdlib.h> // malloc
#include <string.h> // memset
#include <sys/ioctl.h> // ioctl
#include <time.h> // clock_gettime
#include <unistd.h> // close
#include "compiler.h" // __visible

#define SAMPLE_RATE 16000
#define SAMPLES_PER_MS (SAMPLE_RATE / 1000)
#define PWM_PERIOD 1000 // counts: 64 MHz at prescale 6 makes a 64 kHz carrier
#define PWM_MARGIN 20
#define PWM_REQUEST 0x8001500B // /dev/jz_pwm, as libhardware2 calls it
#define PWM_SET_WC 0xC004502D
#define PWM_RETRIES 20 // refused until the last sample has latched
#define SPIN_S 20e-6 // sleep to this close to each sample, then spin
#define RESYNC_S 0.005 // further behind than this, skip rather than catch up
#define RT_PRIORITY 20
#define AUDIO_CPU 1

#define VOICES 24
#define DRUMS 4
#define CHANNELS 16
#define DRUM_CHANNEL 9
#define GAIN 0.6f
#define HELD_FLOOR 0.35f // held notes fade to this, so they don't bury the tune
#define HELD_FADE 0.0015f // per ms

enum { EV_NOTE_OFF, EV_NOTE_ON, EV_VOLUME, EV_ALL_OFF };

struct c5_buzzer_event {
    uint32_t at_ms;
    uint8_t kind, channel, note, value;
};

struct voice {
    uint32_t phase, step;
    float amp, env;
    uint8_t channel, note, on;
};

struct drum {
    float env, fade, amp, value;
    int hold, count;
};

static pthread_t thread;
static volatile int thread_started, keep_running, finished;
static struct c5_buzzer_event *events;
static int event_count, next_event;
static uint32_t length_ms, sample_count, note_steps[128], noise = 0xace1;
static int pwm_fd = -1, pwm_index, next_drum;
static float master;
static struct voice voices[VOICES];
static struct drum drums[DRUMS];
static float channel_volume[CHANNELS];

static double
now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static void
start_drum(int note, float velocity)
{
    struct drum *d = &drums[next_drum];
    next_drum = (next_drum + 1) % DRUMS;
    d->env = 1.f;
    d->count = 0;
    if (note == 35 || note == 36) { // kick: low, dull noise
        d->hold = 8;
        d->fade = 0.992f;
        d->amp = velocity * 1.2f;
    } else if (note == 38 || note == 40) { // snare
        d->hold = 1;
        d->fade = 0.996f;
        d->amp = velocity;
    } else { // hats and cymbals
        d->hold = 1;
        d->fade = 0.985f;
        d->amp = velocity * 0.5f;
    }
}

static void
note_on(int channel, int note, float velocity)
{
    struct voice *v = NULL;
    int i;
    for (i = 0; i < VOICES && !v; i++)
        if (!voices[i].on)
            v = &voices[i];
    if (!v) { // none free: take over whichever has faded most
        v = &voices[0];
        for (i = 1; i < VOICES; i++)
            if (voices[i].env < v->env)
                v = &voices[i];
    }
    v->step = note_steps[note & 127];
    v->amp = velocity;
    v->env = 1.f;
    v->channel = channel;
    v->note = note;
    v->on = 1;
}

static void
run_event(const struct c5_buzzer_event *ev)
{
    const int channel = ev->channel & (CHANNELS - 1);
    int i;
    switch (ev->kind) {
    case EV_NOTE_ON:
        if (channel == DRUM_CHANNEL)
            start_drum(ev->note, ev->value / 127.f);
        else
            note_on(channel, ev->note, ev->value / 127.f);
        break;
    case EV_NOTE_OFF:
        for (i = 0; i < VOICES; i++) {
            if (voices[i].on && voices[i].channel == channel
                && voices[i].note == ev->note) {
                voices[i].on = 0;
                break;
            }
        }
        break;
    case EV_VOLUME:
        channel_volume[channel] = ev->value / 127.f;
        break;
    case EV_ALL_OFF:
        for (i = 0; i < VOICES; i++)
            if (voices[i].channel == channel)
                voices[i].on = 0;
        break;
    }
}

// -1 to 1
static float
next_sample(void)
{
    const uint32_t now_ms = sample_count / SAMPLES_PER_MS;
    const int tick_ms = sample_count % SAMPLES_PER_MS == 0;
    float mix = 0.f;
    int i;
    while (next_event < event_count && events[next_event].at_ms <= now_ms)
        run_event(&events[next_event++]);
    sample_count++;
    for (i = 0; i < VOICES; i++) {
        struct voice *v = &voices[i];
        float level;
        if (!v->on)
            continue;
        v->phase += v->step;
        level = v->amp * v->env * channel_volume[v->channel];
        mix += v->phase & 0x80000000u ? level : -level;
        if (tick_ms && v->env > HELD_FLOOR)
            v->env -= v->env * HELD_FADE;
    }
    for (i = 0; i < DRUMS; i++) {
        struct drum *d = &drums[i];
        if (d->env < 0.01f)
            continue;
        if (d->count-- <= 0) {
            d->count = d->hold - 1;
            noise = noise * 1664525u + 1013904223u;
            d->value = noise & 0x80000000u ? 1.f : -1.f;
        }
        mix += d->value * d->amp * d->env * channel_volume[DRUM_CHANNEL];
        d->env *= d->fade;
    }
    mix *= GAIN * master;
    return mix > 1.f ? 1.f : mix < -1.f ? -1.f : mix;
}

static void
write_wc(int high, int low)
{
    uint32_t arg[2] = {pwm_index, (uint32_t)high << 16 | (uint32_t)low};
    int i;
    for (i = 0; i < PWM_RETRIES; i++)
        if (ioctl(pwm_fd, PWM_SET_WC, arg) == 0)
            return;
}

static void *
sound_thread(void *unused)
{
    // Sleeping most of each sample keeps it under the kernel's real-time limit,
    // which otherwise stops the thread for 50 ms every second
    cpu_set_t cpus;
    CPU_ZERO(&cpus);
    CPU_SET(AUDIO_CPU, &cpus);
    pthread_setaffinity_np(pthread_self(), sizeof(cpus), &cpus);
    struct sched_param param = {.sched_priority = RT_PRIORITY};
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

    const double step = 1.0 / SAMPLE_RATE;
    const int swing = PWM_PERIOD / 2 - PWM_MARGIN;
    double next = now_s();
    while (keep_running && sample_count / SAMPLES_PER_MS < length_ms) {
        const int high = PWM_PERIOD / 2 + (int)(next_sample() * swing);
        const double wait = next - now_s() - SPIN_S;
        double now;
        if (wait > 0) {
            struct timespec ts = {0, (long)(wait * 1e9)};
            nanosleep(&ts, NULL);
        }
        while ((now = now_s()) < next)
            ;
        if (now - next > RESYNC_S)
            next = now;
        write_wc(high, PWM_PERIOD - high);
        next += step;
    }
    write_wc(1, 0);
    finished = 1;
    return NULL;
}

// Waits for the thread and frees the song; safe to call at any time
void __visible
c5_buzzer_stop(void)
{
    if (thread_started) {
        keep_running = 0;
        pthread_join(thread, NULL);
        thread_started = 0;
    }
    if (pwm_fd >= 0) {
        close(pwm_fd);
        pwm_fd = -1;
    }
    free(events);
    events = NULL;
}

// Plays until the song ends or c5_buzzer_stop. The channel must already be
// configured and enabled at prescale 6. 0 on success.
int __visible
c5_buzzer_play(const char *channel, const struct c5_buzzer_event *song,
               int count, uint32_t song_length_ms, const uint32_t *steps,
               float level)
{
    int i;
    c5_buzzer_stop();
    pwm_fd = open("/dev/jz_pwm", O_RDWR);
    if (pwm_fd < 0)
        return -1;
    pwm_index = ioctl(pwm_fd, PWM_REQUEST, channel);
    events = malloc(sizeof(*events) * count);
    if (pwm_index < 0 || !events) {
        c5_buzzer_stop();
        return -1;
    }
    memcpy(events, song, sizeof(*events) * count);
    memcpy(note_steps, steps, sizeof(note_steps));
    memset(voices, 0, sizeof(voices));
    memset(drums, 0, sizeof(drums));
    for (i = 0; i < CHANNELS; i++)
        channel_volume[i] = 100 / 127.f;
    event_count = count;
    next_event = 0;
    sample_count = 0;
    length_ms = song_length_ms;
    master = level;
    finished = 0;
    keep_running = 1;
    if (pthread_create(&thread, NULL, sound_thread, NULL) != 0) {
        c5_buzzer_stop();
        return -1;
    }
    thread_started = 1;
    return 0;
}

int __visible
c5_buzzer_active(void)
{
    return thread_started && !finished;
}
