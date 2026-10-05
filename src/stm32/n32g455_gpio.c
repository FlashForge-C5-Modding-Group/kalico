// Package GPIO pin mapping for Creator 5 N32G455 boards
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // ffs
#include "command.h" // DECL_ENUMERATION_RANGE
#include "internal.h" // gpio_pin_to_regs
#include "sched.h" // sched_shutdown

DECL_ENUMERATION_RANGE("pin", "PA0", GPIO('A', 0), 16);
DECL_ENUMERATION_RANGE("pin", "PB0", GPIO('B', 0), 16);
#if CONFIG_C5_EBOARD
DECL_ENUMERATION("pin", "PG0", GPIO('G', 0));
DECL_ENUMERATION_RANGE("pin", "PC13", GPIO('C', 13), 3);
#define N32G455_PORT_C_MASK 0xe000
#elif CONFIG_C5_HEATERBOARD
DECL_ENUMERATION_RANGE("pin", "PC0", GPIO('C', 0), 16);
DECL_ENUMERATION("pin", "PD2", GPIO('D', 2));
#define N32G455_PORT_C_MASK 0xffff
#else
DECL_ENUMERATION_RANGE("pin", "PC0", GPIO('C', 0), 16);
DECL_ENUMERATION_RANGE("pin", "PD0", GPIO('D', 0), 16);
DECL_ENUMERATION_RANGE("pin", "PE0", GPIO('E', 0), 16);
DECL_ENUMERATION_RANGE("pin", "PF0", GPIO('F', 0), 16);
DECL_ENUMERATION_RANGE("pin", "PG0", GPIO('G', 0), 16);
#define N32G455_PORT_C_MASK 0xffff
#endif

static GPIO_TypeDef * const digital_regs[] = {
    ['A' - 'A'] = GPIOA,
    ['B' - 'A'] = GPIOB,
    ['C' - 'A'] = GPIOC,
#if CONFIG_C5_HEATERBOARD
    ['D' - 'A'] = GPIOD,
#elif !CONFIG_C5_EBOARD
    ['D' - 'A'] = GPIOD, GPIOE, GPIOF, GPIOG,
#endif
};

static const uint16_t digital_pin_masks[] = {
    ['A' - 'A'] = 0xffff,
    ['B' - 'A'] = 0xffff,
    ['C' - 'A'] = N32G455_PORT_C_MASK,
#if CONFIG_C5_HEATERBOARD
    ['D' - 'A'] = 0x0004,
#elif !CONFIG_C5_EBOARD
    ['D' - 'A'] = 0xffff, 0xffff, 0xffff, 0xffff,
#endif
};

// Convert a register and bit location back to an integer pin identifier.
int
gpio_regs_to_pin(GPIO_TypeDef *regs, uint32_t bit)
{
    int i;
    for (i=0; i<ARRAY_SIZE(digital_regs); i++)
        if (digital_regs[i] == regs)
            return GPIO('A' + i, ffs(bit)-1);
    return 0;
}

// Verify that a GPIO is bonded in the selected package.
GPIO_TypeDef *
gpio_pin_to_regs(uint32_t pin)
{
    uint32_t port = GPIO2PORT(pin);
    if (port >= ARRAY_SIZE(digital_regs) || !digital_regs[port]
        || port >= ARRAY_SIZE(digital_pin_masks)
        || !(digital_pin_masks[port] & GPIO2BIT(pin)))
        shutdown("Not a valid pin");
    return digital_regs[port];
}
