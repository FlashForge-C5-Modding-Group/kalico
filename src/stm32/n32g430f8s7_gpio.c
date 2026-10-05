// GPIO pin mapping for N32G430F8S7
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // ffs
#include "command.h" // DECL_ENUMERATION_RANGE
#include "internal.h" // gpio_pin_to_regs
#include "sched.h" // sched_shutdown

DECL_ENUMERATION_RANGE("pin", "PA0", GPIO('A', 0), 8);
DECL_ENUMERATION("pin", "PA9", GPIO('A', 9));
DECL_ENUMERATION("pin", "PA10", GPIO('A', 10));
DECL_ENUMERATION("pin", "PB1", GPIO('B', 1));
DECL_ENUMERATION("pin", "PD0", GPIO('D', 0));

static GPIO_TypeDef * const digital_regs[] = {
    ['A' - 'A'] = GPIOA, GPIOB, GPIOC, GPIOD,
};

static const uint16_t digital_pin_masks[] = {
    ['A' - 'A'] = 0x06ff,
    ['B' - 'A'] = 0x0002,
    ['D' - 'A'] = 0x0001,
};

// Convert a register and bit location back to an integer pin identifier
int
gpio_regs_to_pin(GPIO_TypeDef *regs, uint32_t bit)
{
    uint32_t i;
    for (i=0; i<ARRAY_SIZE(digital_regs); i++)
        if (digital_regs[i] == regs)
            return GPIO('A' + i, ffs(bit)-1);
    return 0;
}

// Verify that a gpio is a bonded pin and return its hardware register
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
