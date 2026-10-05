// GPIO configuration for N32G430
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "internal.h" // gpio_peripheral

// Set a pin's mode, alternate function, pull, type, slew, and drive strength
void
gpio_peripheral(uint32_t gpio, uint32_t mode, int pullup)
{
    GPIO_TypeDef *regs = gpio_pin_to_regs(gpio);
    gpio_clock_enable(regs);

    uint32_t mode_bits = mode & 0x0f, func = (mode >> 4) & 0x0f;
    uint32_t od = (mode >> 8) & 0x01;
    uint32_t slow = mode & GPIO_HIGH_SPEED ? 0 : 1;
    uint32_t drive = mode & N32G430_GPIO_DRIVE_4MA ? 2 : 0;
    uint32_t pup = pullup ? (pullup > 0 ? 1 : 2) : 0;
    uint32_t pos = gpio % 16, af_reg = pos / 8;
    uint32_t af_shift = (pos % 8) * 4, af_mask = 0x0f << af_shift;
    uint32_t mode_shift = pos * 2, mode_mask = 0x03 << mode_shift;
    uint32_t pin_mask = 1U << pos;

    regs->AFR[af_reg] = (regs->AFR[af_reg] & ~af_mask) | (func << af_shift);
    regs->MODER = (regs->MODER & ~mode_mask) | (mode_bits << mode_shift);
    regs->PUPDR = (regs->PUPDR & ~mode_mask) | (pup << mode_shift);
    regs->OTYPER = (regs->OTYPER & ~pin_mask) | (od << pos);
    regs->OSPEEDR = (regs->OSPEEDR & ~pin_mask) | (slow << pos);
    regs->DSCR = (regs->DSCR & ~mode_mask) | (drive << mode_shift);
}
