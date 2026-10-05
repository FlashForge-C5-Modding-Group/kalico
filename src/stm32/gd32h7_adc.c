// ADC2 polling support for GD32H737
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <limits.h>
#include "board/irq.h" // irq_save
#include "board/misc.h" // timer_from_us
#include "command.h" // shutdown
#include "generic/armcm_timer.h" // udelay
#include "gpio.h" // gpio_adc_setup
#include "internal.h" // GPIO
#include "sched.h" // shutdown

DECL_CONSTANT("ADC_MAX", 4095);

enum adc_state {
    ADC_IDLE,
    ADC_ACTIVE,
    ADC_DISCARD_PENDING,
};

static enum adc_state adc_state;
static uint32_t active_token;
static uint32_t owner_sequence;

static uint32_t
pin_to_channel(uint32_t pin)
{
    if (pin == GPIO('C', 0))
        return 10;
    if (pin == GPIO('C', 1))
        return 11;
    if (pin == GPIO('C', 3))
        return 1;
    shutdown("Not a valid ADC pin");
    return 0;
}

static void
adc_init(void)
{
    if (is_enabled_pclock(ADC2))
        return;
    enable_pclock(ADC2);
    ADC_SYNCCTL(ADC2) =
        (ADC_SYNCCTL(ADC2) & ~(ADC_SYNCCTL_ADCSCK | ADC_SYNCCTL_ADCCK))
        | ADC_CLK_SYNC_HCLK_DIV4;
    ADC_CTL0(ADC2) &= ~ADC_CTL0_DRES;
    ADC_CTL1(ADC2) = (ADC_CTL1(ADC2) & ~ADC_CTL1_DAL) | ADC_CTL1_ADCON;
    udelay(1);
    ADC_CTL1(ADC2) |= ADC_CTL1_RSTCLB;
    while (ADC_CTL1(ADC2) & ADC_CTL1_RSTCLB)
        ;
    ADC_CTL1(ADC2) |= ADC_CTL1_CLB;
    while (ADC_CTL1(ADC2) & ADC_CTL1_CLB)
        ;
}

struct gpio_adc
gpio_adc_setup(uint32_t pin)
{
    uint32_t channel = pin_to_channel(pin);
    adc_init();
    if (pin != GPIO('C', 3))
        gpio_peripheral(pin, GPIO_ANALOG, 0);

    irqstatus_t flag = irq_save();
    if (owner_sequence == (UINT32_MAX >> 5)) {
        irq_restore(flag);
        shutdown("ADC owner token overflow");
    }
    uint32_t token = (++owner_sequence << 5) | channel;
    irq_restore(flag);
    return (struct gpio_adc){ .adc = (void *)ADC2, .chan = token };
}

static void
adc_drain_completion(void)
{
    (void)ADC_RDATA(ADC2);
}

uint32_t
gpio_adc_sample(struct gpio_adc g)
{
    irqstatus_t flag = irq_save();
    if (g.adc != (void *)ADC2) {
        irq_restore(flag);
        shutdown("Not a valid ADC pin");
    }

    if (adc_state == ADC_DISCARD_PENDING) {
        if (!(ADC_STAT(ADC2) & ADC_STAT_EOC)) {
            irq_restore(flag);
            return timer_from_us(20);
        }
        adc_drain_completion();
        adc_state = ADC_IDLE;
        active_token = 0;
    }

    if (adc_state == ADC_ACTIVE) {
        uint8_t ready = active_token == g.chan
                        && (ADC_STAT(ADC2) & ADC_STAT_EOC);
        irq_restore(flag);
        return ready ? 0 : timer_from_us(20);
    }

    if (ADC_STAT(ADC2) & ADC_STAT_EOC)
        adc_drain_completion();
    active_token = g.chan;
    adc_state = ADC_ACTIVE;
    ADC_RSQ8(ADC2) = SQX_SMP(638U) | (g.chan & ADC_RSQX_RSQN);
    ADC_CTL1(ADC2) |= ADC_CTL1_SWRCST;
    irq_restore(flag);
    return timer_from_us(20);
}

uint16_t
gpio_adc_read(struct gpio_adc g)
{
    irqstatus_t flag = irq_save();
    if (g.adc != (void *)ADC2 || adc_state != ADC_ACTIVE
        || active_token != g.chan || !(ADC_STAT(ADC2) & ADC_STAT_EOC)) {
        irq_restore(flag);
        return 0;
    }
    uint16_t value = ADC_RDATA(ADC2);
    adc_state = ADC_IDLE;
    active_token = 0;
    irq_restore(flag);
    return value;
}

void
gpio_adc_cancel_sample(struct gpio_adc g)
{
    irqstatus_t flag = irq_save();
    if (g.adc == (void *)ADC2 && adc_state == ADC_ACTIVE
        && active_token == g.chan) {
        if (ADC_STAT(ADC2) & ADC_STAT_EOC) {
            adc_drain_completion();
            adc_state = ADC_IDLE;
            active_token = 0;
        } else {
            adc_state = ADC_DISCARD_PENDING;
        }
    }
    irq_restore(flag);
}
