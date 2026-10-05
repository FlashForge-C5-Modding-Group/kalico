// Creator 5 eBoard pressure-advance scorer regressions
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h>
#include <stdio.h>

#include "c5_eboard.h"

#define SAMPLE_COUNT 2000

static int32_t samples[SAMPLE_COUNT];

static void
fill(int32_t value)
{
    for (uint16_t i = 0; i < SAMPLE_COUNT; i++)
        samples[i] = value;
}

static uint16_t
ramp(uint16_t index, uint16_t length, int32_t from, int32_t to)
{
    for (int32_t k = 1; k <= length; k++)
        samples[index++] = from + (to - from) * k / length;
    return index;
}

static int
expect(const char *name, uint8_t expected)
{
    uint8_t actual = c5_eboard_pa_matches(samples, SAMPLE_COUNT);
    if (actual == expected)
        return 0;
    fprintf(stderr, "%s: expected %u, got %u\n", name, expected, actual);
    return 1;
}

int
main(void)
{
    int failures = 0;

    fill(40);
    uint16_t i = 100;
    i = ramp(i, 6, 40, 420);
    i = ramp(i, 8, 420, 160);
    i = ramp(i, 8, 160, 280);
    for (uint16_t end = i + 80; i < end; i++)
        samples[i] = 280;
    ramp(i, 12, 280, 40);
    failures += expect("shaped rise", 1);

    fill(40);
    i = 100;
    i = ramp(i, 15, 40, 300);
    for (uint16_t end = i + 100; i < end; i++)
        samples[i] = 300;
    ramp(i, 15, 300, 40);
    failures += expect("flat pulse", 0);

    fill(40);
    failures += expect("low constant", 0);
    fill(400);
    failures += expect("high constant", 0);

    for (i = 0; i < 1000; i++)
        samples[i] = 40 + i;
    for (; i < SAMPLE_COUNT; i++)
        samples[i] = 1040;
    failures += expect("monotonic rise", 0);

    for (i = 0; i < SAMPLE_COUNT; i++)
        samples[i] = 40 + ((37 * i) % 50);
    failures += expect("noise", 0);

    if (c5_eboard_pa_matches(samples, 0)) {
        fprintf(stderr, "empty: expected 0, got 1\n");
        failures++;
    }
    return failures ? 1 : 0;
}
