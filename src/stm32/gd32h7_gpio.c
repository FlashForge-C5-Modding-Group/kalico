// GPIO support for GD32H737
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // ffs
#include "board/irq.h" // irq_save
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

static GPIO_TypeDef * const digital_regs[] = {
    GPIOA, GPIOB, GPIOC, GPIOD, GPIOE,
};

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

struct gpio_out
gpio_out_setup(uint32_t pin, uint32_t val)
{
    GPIO_TypeDef *regs = gpio_pin_to_regs(pin);
    gpio_clock_enable(regs);
    struct gpio_out g = { .regs = regs, .bit = GPIO2BIT(pin) };
    gpio_out_reset(g, val);
    return g;
}

void
gpio_out_reset(struct gpio_out g, uint32_t val)
{
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

void
gpio_out_toggle_noirq(struct gpio_out g)
{
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

void
gpio_out_write(struct gpio_out g, uint32_t val)
{
    GPIO_TypeDef *regs = g.regs;
    if (val)
        regs->BOP = g.bit;
    else
        regs->BC = g.bit;
}

struct gpio_in
gpio_in_setup(uint32_t pin, int32_t pull_up)
{
    GPIO_TypeDef *regs = gpio_pin_to_regs(pin);
    struct gpio_in g = { .regs = regs, .bit = GPIO2BIT(pin) };
    gpio_in_reset(g, pull_up);
    return g;
}

void
gpio_in_reset(struct gpio_in g, int32_t pull_up)
{
    GPIO_TypeDef *regs = g.regs;
    int pin = gpio_regs_to_pin(regs, g.bit);
    irqstatus_t flag = irq_save();
    gpio_peripheral(pin, GPIO_INPUT, pull_up);
    irq_restore(flag);
}

uint8_t
gpio_in_read(struct gpio_in g)
{
    GPIO_TypeDef *regs = g.regs;
    return !!(regs->IDR & g.bit);
}
