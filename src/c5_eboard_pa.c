// Creator 5 eBoard pressure-advance classifier
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h> // int32_t

#include "c5_eboard.h" // c5_eboard_pa_matches

uint8_t
c5_eboard_pa_matches(const volatile int32_t *samples, uint16_t count)
{
    int32_t size = count, s = 0;
    uint8_t matched = 0;

    while (s < size) {
        int64_t rise = 0;
        int32_t i;
        for (i = s + 1; i < size; i++) {
            int32_t left = samples[i - 1], right = samples[i];
            if (right >= left) {
                rise += (int64_t)right - left;
                if (rise > 99)
                    break;
            } else {
                s = i;
                rise = 0;
            }
        }
        if (i >= size)
            break;

        int32_t e = size - 1, low_run = 0;
        for (i = s; i < size; i++) {
            if (samples[i] <= 70) {
                if (++low_run == 200) {
                    e = i - 199;
                    break;
                }
            } else {
                low_run = 0;
            }
        }

        int32_t h0 = s, h1 = e;
        for (i = s; i <= e; i++) {
            if (samples[i] > 150) {
                h0 = i;
                break;
            }
        }
        for (i = e; i >= s; i--) {
            if (samples[i] > 150) {
                h1 = i;
                break;
            }
        }

        double mean = 0.0;
        if (h1 - h0 > 59) {
            int64_t sum = 0;
            for (i = h0 + 30; i <= h1 - 30; i++)
                sum += samples[i];
            mean = sum / (double)(h1 - h0 - 59);
        }

        int32_t mid = (s + e) / 2, peak = 0;
        uint16_t above = 0;
        for (i = s; i <= mid; i++) {
            int32_t value = samples[i];
            if (value > peak)
                peak = value;
            if ((double)value > mean * 1.2 && above < 10)
                above++;
        }

        if (mean > 150.0 && (uint32_t)(h1 - h0 - 81) <= 68
            && (double)peak > mean * 1.2 && above > 2) {
            int64_t variation = 0;
            int32_t j = e;
            while (j > mid) {
                while (j > mid && samples[j - 1] <= samples[j]) {
                    variation += (int64_t)samples[j] - samples[j - 1];
                    j--;
                }
                int64_t anchor = samples[j];
                double settled = (double)(anchor + variation);
                if ((double)anchor <= 1.5 * (double)variation
                    && mean * 0.5 <= settled && settled <= mean) {
                    matched = 1;
                    break;
                }
                while (j > mid && samples[j - 1] > samples[j])
                    j--;
            }
        }
        s = e + 200;
    }
    return matched;
}
