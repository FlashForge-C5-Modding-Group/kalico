#include <setjmp.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifndef TEST_LEVELBOARD
#define TEST_LEVELBOARD 0
#endif

#define CONFIG_C5_EBOARD (!TEST_LEVELBOARD)
#define CONFIG_C5_HEATERBOARD 0
#define CONFIG_C5_LEVELBOARD TEST_LEVELBOARD
#define CONFIG_MACH_N32G430 TEST_LEVELBOARD
#define CONFIG_MACH_N32G45x (!TEST_LEVELBOARD)

#define __BASECMD_H
#define __C5_EBOARD_H
#define __C5_LEVELBOARD_H
#define __COMMAND_H
#define __SCHED_H
#define __STM32_GPIO_H
#define __STM32_INTERNAL_H
#define __TRSYNC_H

static int
model_ffs(int value)
{
    int bit = 1;
    while (value && !(value & 1)) {
        value >>= 1;
        bit++;
    }
    return value ? bit : 0;
}
#define ffs model_ffs

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define GPIO(PORT, NUM) (((PORT) - 'A') * 16 + (NUM))
#define GPIO2PORT(PIN) ((PIN) / 16)
#define GPIO2BIT(PIN) (1u << ((PIN) % 16))
#define GPIO_INPUT 0
#define GPIO_OUTPUT 1
#define container_of(ptr, type, member) \
    ((type *)((char *)(ptr) - offsetof(type, member)))

#define DECL_COMMAND(FUNC, MSG)
#define DECL_ENUMERATION(ENUM, NAME, VALUE)
#define DECL_ENUMERATION_RANGE(ENUM, NAME, VALUE, COUNT)

struct gpio_registers {
    volatile uint32_t BSRR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
};
typedef struct gpio_registers GPIO_TypeDef;

static GPIO_TypeDef gpio_ports[7];
#define GPIOA (&gpio_ports[0])
#define GPIOB (&gpio_ports[1])
#define GPIOC (&gpio_ports[2])
#define GPIOD (&gpio_ports[3])
#define GPIOE (&gpio_ports[4])
#define GPIOF (&gpio_ports[5])
#define GPIOG (&gpio_ports[6])

struct gpio_out {
    void *regs;
    uint32_t bit;
};
struct gpio_in {
    void *regs;
    uint32_t bit;
};
struct timer {
    struct timer *next;
    uint_fast8_t (*func)(struct timer *);
    uint32_t waketime;
};
struct trsync { uint8_t unused; };
typedef unsigned int irqstatus_t;
enum { SF_DONE = 0, SF_RESCHEDULE = 1 };

static jmp_buf shutdown_jump;
static unsigned int physical_config_count;
static unsigned int getter_count;
static union {
    uint64_t alignment;
    unsigned char bytes[256];
} oid_storage;

static void
model_shutdown(void)
{
    longjmp(shutdown_jump, 1);
}
#define shutdown(MSG) model_shutdown()

static void
model_sendf(const char *format, ...)
{
    (void)format;
}
#define sendf(FORMAT, ...) model_sendf(FORMAT, __VA_ARGS__)

void *oid_alloc(uint8_t oid, void (*type)(uint32_t *), uint16_t size);
void *oid_lookup(uint8_t oid, void (*type)(uint32_t *));
void sched_add_timer(struct timer *timer);
void sched_del_timer(struct timer *timer);
void irq_disable(void);
void irq_enable(void);
struct trsync *trsync_oid_lookup(uint8_t oid);
void trsync_do_trigger(struct trsync *ts, uint8_t reason);
uint8_t c5_eboard_eddy_state(void);
void c5_eboard_arm(void);
uint8_t c5_levelboard_eddy_state(void);
void c5_levelboard_cancel(void);
void c5_levelboard_recover(void);
void gpio_clock_enable(GPIO_TypeDef *regs);
void gpio_peripheral(uint32_t pin, uint32_t mode, int pull_up);
irqstatus_t irq_save(void);
void irq_restore(irqstatus_t status);
struct gpio_out gpio_out_setup(uint32_t pin, uint32_t value);
void gpio_out_reset(struct gpio_out gpio, uint32_t value);
void gpio_out_toggle_noirq(struct gpio_out gpio);
void gpio_out_toggle(struct gpio_out gpio);
void gpio_out_write(struct gpio_out gpio, uint32_t value);
struct gpio_in gpio_in_setup(uint32_t pin, int32_t pull_up);
void gpio_in_reset(struct gpio_in gpio, int32_t pull_up);
uint8_t gpio_in_read(struct gpio_in gpio);

#if TEST_LEVELBOARD
#include "../src/stm32/n32g430f8s7_gpio.c"
#define VIRTUAL_PIN GPIO('D', 0)
#define VIRTUAL_REGS GPIOD
#else
#include "../src/stm32/n32g455_gpio.c"
#define VIRTUAL_PIN GPIO('G', 0)
#define VIRTUAL_REGS GPIOG
#endif
#include "../src/stm32/gpio.c"
#include "../src/c5_endstop.c"

void *
oid_alloc(uint8_t oid, void (*type)(uint32_t *), uint16_t size)
{
    (void)oid;
    (void)type;
    if (size > sizeof(oid_storage.bytes))
        return NULL;
    memset(oid_storage.bytes, 0, sizeof(oid_storage.bytes));
    return oid_storage.bytes;
}

void *
oid_lookup(uint8_t oid, void (*type)(uint32_t *))
{
    (void)oid;
    (void)type;
    return oid_storage.bytes;
}

void sched_add_timer(struct timer *timer) { (void)timer; }
void sched_del_timer(struct timer *timer) { (void)timer; }
void irq_disable(void) { }
void irq_enable(void) { }
irqstatus_t irq_save(void) { return 0; }
void irq_restore(irqstatus_t status) { (void)status; }

struct trsync *
trsync_oid_lookup(uint8_t oid)
{
    static struct trsync value;
    (void)oid;
    return &value;
}

void
trsync_do_trigger(struct trsync *ts, uint8_t reason)
{
    (void)ts;
    (void)reason;
}

uint8_t
c5_eboard_eddy_state(void)
{
    getter_count++;
    return 0;
}

void c5_eboard_arm(void) { }
uint8_t
c5_levelboard_eddy_state(void)
{
    getter_count++;
    return 0;
}
void c5_levelboard_cancel(void) { }
void c5_levelboard_recover(void) { }
void gpio_clock_enable(GPIO_TypeDef *regs) { (void)regs; }

void
gpio_peripheral(uint32_t pin, uint32_t mode, int pull_up)
{
    (void)pin;
    (void)mode;
    (void)pull_up;
    physical_config_count++;
}

static int
input_setup_rejected(uint32_t pin)
{
    if (setjmp(shutdown_jump))
        return 1;
    (void)gpio_in_setup(pin, 0);
    return 0;
}

#if !TEST_LEVELBOARD
static int
output_setup_rejected(uint32_t pin)
{
    if (setjmp(shutdown_jump))
        return 1;
    (void)gpio_out_setup(pin, 0);
    return 0;
}
#endif

static int
expect(int condition, const char *message)
{
    if (condition)
        return 0;
    fprintf(stderr, "%s\n", message);
    return 1;
}

int
main(void)
{
    int failures = 0;
    uint32_t args[] = { 1, VIRTUAL_PIN, 1 };

    if (setjmp(shutdown_jump)) {
        fprintf(stderr, "config_endstop rejected virtual pin\n");
        return 1;
    }
    command_config_endstop(args);
    struct endstop *endstop = (struct endstop *)oid_storage.bytes;
    failures += expect(endstop->pin.regs == VIRTUAL_REGS,
                       "virtual handle has wrong register identity");
    failures += expect(endstop->pin.bit == GPIO2BIT(VIRTUAL_PIN),
                       "virtual handle has wrong bit identity");
    failures += expect(physical_config_count == 0,
                       "virtual endstop configured physical GPIO hardware");

    VIRTUAL_REGS->IDR = GPIO2BIT(VIRTUAL_PIN);
    failures += expect(c5_endstop_read(endstop->pin) == 0,
                       "virtual pin did not use fail-safe eddy getter");
    failures += expect(getter_count == 1,
                       "virtual pin did not call eddy getter exactly once");

#if TEST_LEVELBOARD
    failures += expect(input_setup_rejected(GPIO('D', 1)),
                       "generic input setup accepted unbonded PD1");
#else
    failures += expect(input_setup_rejected(GPIO('G', 0)),
                       "generic input setup accepted PG0");
    failures += expect(output_setup_rejected(GPIO('G', 0)),
                       "generic output setup accepted PG0");
    failures += expect(input_setup_rejected(GPIO('G', 1)),
                       "generic input setup accepted unbonded PG1");
#endif
    failures += expect(input_setup_rejected(GPIO('C', 0)),
                       "generic input setup accepted unbonded PC0");

    if (setjmp(shutdown_jump)) {
        fprintf(stderr, "generic input setup rejected bonded PA0\n");
        return 1;
    }
    struct gpio_in physical = gpio_in_setup(GPIO('A', 0), 0);
    failures += expect(physical.regs == GPIOA,
                       "bonded PA0 resolved to wrong register");
    failures += expect(physical_config_count == 1,
                       "bonded PA0 was not physically configured");

    if (!failures)
        puts("virtual-endstop-model-ok");
    return failures != 0;
}
