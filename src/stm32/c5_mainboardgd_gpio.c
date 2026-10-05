// GPIO and virtual motor pins for Creator 5 mainBoardGD
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // ffs
#include "board/irq.h" // irq_save
#include "c5_mainboardgd.h" // c5_mainboardgd_step
#include "command.h" // DECL_ENUMERATION_RANGE
#include "gpio.h" // gpio_out_setup
#include "internal.h" // gpio_peripheral
#include "sched.h" // shutdown

DECL_ENUMERATION_RANGE("pin", "PA0", GPIO('A', 0), 11);
DECL_ENUMERATION_RANGE("pin", "PA13", GPIO('A', 13), 3);
DECL_ENUMERATION_RANGE("pin", "PB0", GPIO('B', 0), 16);
DECL_ENUMERATION_RANGE("pin", "PC0", GPIO('C', 0), 16);
DECL_ENUMERATION_RANGE("pin", "PD0", GPIO('D', 0), 16);
DECL_ENUMERATION_RANGE("pin", "PE0", GPIO('E', 0), 16);
DECL_ENUMERATION_RANGE("pin", "PH2", GPIO('H', 2), 2);
DECL_ENUMERATION_RANGE("pin", "PJ0", GPIO('J', 0), 16);

static GPIO_TypeDef * const digital_regs[] = {
    GPIOA, GPIOB, GPIOC, GPIOD, GPIOE,
};

static uint8_t virtual_ph_bank, virtual_pj_bank;
static uint16_t pj_shadow;
static uint8_t ph_shadow, master_inhibit;

static uint8_t
is_digital_pin(uint32_t pin)
{
    uint32_t port = GPIO2PORT(pin), bit = pin % 16;
    if (port >= ARRAY_SIZE(digital_regs))
        return 0;
    if (port == 0 && (bit == 11 || bit == 12))
        return 0;
    if (port == 2 && (bit == 2 || bit == 3))
        return 0;
    return 1;
}

GPIO_TypeDef *
gpio_pin_to_regs(uint32_t pin)
{
    if (!is_digital_pin(pin))
        shutdown("Not a valid pin");
    return digital_regs[GPIO2PORT(pin)];
}

int
gpio_regs_to_pin(GPIO_TypeDef *regs, uint32_t bit)
{
    int port;
    for (port = 0; port < ARRAY_SIZE(digital_regs); port++)
        if (digital_regs[port] == regs) {
            uint32_t pin = GPIO('A' + port, ffs(bit) - 1);
            if (!bit || !is_digital_pin(pin))
                shutdown("Not a valid pin");
            return pin;
        }
    shutdown("Not a valid pin");
    return 0;
}

static uint8_t
virtual_bit_index(uint32_t bit)
{
    if (!bit || (bit & (bit - 1)))
        shutdown("Not a valid pin");
    return ffs(bit) - 1;
}

static uint8_t
pj_axis(uint8_t index)
{
    static const uint8_t axes[] = { 1, 0, 2 };
    if (index >= 12)
        shutdown("Unsupported mainBoardGD motor");
    return axes[index >> 2];
}

static uint8_t
is_pj_output(uint8_t index)
{
    if (index >= 12)
        shutdown("Unsupported mainBoardGD motor");
    return (index & 3U) != 3U;
}

static void
virtual_out_reset(struct gpio_out g, uint32_t val)
{
    uint8_t index = virtual_bit_index(g.bit);
    if (g.regs == &virtual_ph_bank) {
        if (index != 2 && index != 3)
            shutdown("Not a valid pin");
        if (val)
            ph_shadow |= g.bit;
        else
            ph_shadow &= ~g.bit;
        return;
    }
    if (g.regs != &virtual_pj_bank || !is_pj_output(index))
        shutdown("Not a valid pin");
    if (val)
        pj_shadow |= g.bit;
    else
        pj_shadow &= ~g.bit;
    if ((index & 3U) == 1U)
        c5_mainboardgd_direction(pj_axis(index), !!val);
}

struct gpio_out
gpio_out_setup(uint32_t pin, uint32_t val)
{
    if (pin == GPIO('H', 2) || pin == GPIO('H', 3)) {
        struct gpio_out g = {
            .regs = &virtual_ph_bank, .bit = GPIO2BIT(pin)
        };
        gpio_out_reset(g, val);
        return g;
    }
    if (GPIO2PORT(pin) == 'J' - 'A') {
        uint8_t index = pin % 16;
        if (!is_pj_output(index))
            shutdown("Not a valid pin");
        struct gpio_out g = {
            .regs = &virtual_pj_bank, .bit = GPIO2BIT(pin)
        };
        gpio_out_reset(g, val);
        return g;
    }

    GPIO_TypeDef *regs = gpio_pin_to_regs(pin);
    gpio_clock_enable(regs);
    struct gpio_out g = { .regs = regs, .bit = GPIO2BIT(pin) };
    gpio_out_reset(g, val);
    return g;
}

void
gpio_out_reset(struct gpio_out g, uint32_t val)
{
    if (g.regs == &virtual_ph_bank || g.regs == &virtual_pj_bank) {
        irqstatus_t flag = irq_save();
        virtual_out_reset(g, val);
        irq_restore(flag);
        return;
    }

    GPIO_TypeDef *regs = g.regs;
    int pin = gpio_regs_to_pin(regs, g.bit);
    irqstatus_t flag = irq_save();
    if (val)
        regs->BOP = g.bit;
    else
        regs->BC = g.bit;
    gpio_peripheral(pin, GPIO_OUTPUT, 0);
    irq_restore(flag);
}

static void
virtual_out_toggle_noirq(struct gpio_out g)
{
    uint8_t index = virtual_bit_index(g.bit);
    if (g.regs == &virtual_ph_bank) {
        if (index != 2 && index != 3)
            shutdown("Not a valid pin");
        ph_shadow ^= g.bit;
        return;
    }
    if (g.regs != &virtual_pj_bank || !is_pj_output(index))
        shutdown("Not a valid pin");

    pj_shadow ^= g.bit;
    uint8_t value = !!(pj_shadow & g.bit);
    switch (index & 3U) {
    case 0:
        if (value && !master_inhibit)
            c5_mainboardgd_step(pj_axis(index));
        break;
    case 1:
        c5_mainboardgd_direction(pj_axis(index), value);
        break;
    default:
        break;
    }
}

void
gpio_out_toggle_noirq(struct gpio_out g)
{
    if (g.regs == &virtual_ph_bank || g.regs == &virtual_pj_bank) {
        virtual_out_toggle_noirq(g);
        return;
    }
    GPIO_TypeDef *regs = g.regs;
    regs->TG = g.bit;
}

void
gpio_out_toggle(struct gpio_out g)
{
    irqstatus_t flag = irq_save();
    gpio_out_toggle_noirq(g);
    irq_restore(flag);
}

static void
virtual_out_write(struct gpio_out g, uint32_t val)
{
    uint8_t index = virtual_bit_index(g.bit);
    if (g.regs == &virtual_ph_bank) {
        if (index != 2 && index != 3)
            shutdown("Not a valid pin");
        if (val) {
            ph_shadow |= g.bit;
            master_inhibit |= 1U << (index - 2);
        } else {
            ph_shadow &= ~g.bit;
            master_inhibit &= ~(1U << (index - 2));
        }
        return;
    }
    if (g.regs != &virtual_pj_bank || !is_pj_output(index))
        shutdown("Not a valid pin");

    if (val)
        pj_shadow |= g.bit;
    else
        pj_shadow &= ~g.bit;
    switch (index & 3U) {
    case 1:
        c5_mainboardgd_direction(pj_axis(index), !!val);
        break;
    case 2:
        c5_mainboardgd_enable(pj_axis(index), !val);
        break;
    default:
        break;
    }
}

void
gpio_out_write(struct gpio_out g, uint32_t val)
{
    if (g.regs == &virtual_ph_bank || g.regs == &virtual_pj_bank) {
        irqstatus_t flag = irq_save();
        virtual_out_write(g, val);
        irq_restore(flag);
        return;
    }

    GPIO_TypeDef *regs = g.regs;
    if (val)
        regs->BOP = g.bit;
    else
        regs->BC = g.bit;
}

static uint8_t
is_stall_pin(uint8_t index)
{
    if (index >= 12)
        shutdown("Unsupported mainBoardGD motor");
    return (index & 3U) == 3U;
}

struct gpio_in
gpio_in_setup(uint32_t pin, int32_t pull_up)
{
    if (GPIO2PORT(pin) == 'J' - 'A') {
        uint8_t index = pin % 16;
        if (!is_stall_pin(index))
            shutdown("Not a valid pin");
        struct gpio_in g = {
            .regs = &virtual_pj_bank, .bit = GPIO2BIT(pin)
        };
        gpio_in_reset(g, pull_up);
        return g;
    }
    if (pin == GPIO('H', 2) || pin == GPIO('H', 3))
        shutdown("Not a valid pin");

    GPIO_TypeDef *regs = gpio_pin_to_regs(pin);
    struct gpio_in g = { .regs = regs, .bit = GPIO2BIT(pin) };
    gpio_in_reset(g, pull_up);
    return g;
}

void
gpio_in_reset(struct gpio_in g, int32_t pull_up)
{
    if (g.regs == &virtual_pj_bank) {
        (void)pull_up;
        if (!is_stall_pin(virtual_bit_index(g.bit)))
            shutdown("Not a valid pin");
        return;
    }

    GPIO_TypeDef *regs = g.regs;
    int pin = gpio_regs_to_pin(regs, g.bit);
    irqstatus_t flag = irq_save();
    gpio_peripheral(pin, GPIO_INPUT, pull_up);
    irq_restore(flag);
}

uint8_t
gpio_in_read(struct gpio_in g)
{
    if (g.regs == &virtual_pj_bank) {
        uint8_t index = virtual_bit_index(g.bit);
        if (!is_stall_pin(index))
            shutdown("Not a valid pin");
        return c5_mainboardgd_stalled(pj_axis(index));
    }
    GPIO_TypeDef *regs = g.regs;
    return !!(regs->IDR & g.bit);
}
