// Creator 5 mainBoardGD application and hardware boundary
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#ifndef __C5_MAINBOARDGD_H
#define __C5_MAINBOARDGD_H

#include <stdint.h>
#include "board/gpio.h" // struct gpio_out
#include "c5_mclib.h"

// TIMER22: 300MHz APB1 timer clock / (PSC 2 + 1)
#define C5_MOTOR_TIME_FREQ 100000000

void c5_mainboardgd_init_motors(void);
void c5_mainboardgd_direction(uint8_t axis, uint8_t value);
void c5_mainboardgd_step(uint8_t axis);
void c5_mainboardgd_next_step(struct gpio_out pin, uint32_t clock,
                              uint8_t due);
void c5_mainboardgd_set_next_step(uint8_t axis, uint32_t clock,
                                  uint8_t due);
void c5_mainboardgd_enable(uint8_t axis, uint8_t enable);
uint8_t c5_mainboardgd_stalled(uint8_t axis);
uint8_t c5_mainboardgd_control(uint8_t axis, uint32_t now,
                              float ia, float ib,
                              struct c5_mclib_output *out);
uint32_t c5_mainboardgd_motor_time(void);
void c5_mainboardgd_pwm_enable(uint8_t axis, uint8_t enable);
void c5_mainboardgd_acq_reinit(uint8_t axis);

#endif // c5_mainboardgd.h
