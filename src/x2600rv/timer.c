// Polled timer for the X2600 RISC-V core
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/io.h" // readl
#include "board/misc.h" // timer_read_time
#include "internal.h" // RV_MTIME_LO
#include "sched.h" // DECL_INIT

static uint32_t timer_compare;
static uint8_t interrupt_seen;

uint32_t
timer_read_time(void)
{
    return readl((void *)RV_MTIME_LO);
}

uint8_t
timer_interrupt_pending(void)
{
    if (interrupt_seen)
        return 0;
    return !timer_is_before(timer_read_time(), timer_compare);
}

void
timer_clear_interrupt(void)
{
    interrupt_seen = 1;
}

void
timer_set(uint32_t value)
{
    timer_compare = value;
    interrupt_seen = 0;
}

void
timer_kick(void)
{
    timer_set(timer_read_time() + 50);
}

void
timer_init(void)
{
    interrupt_seen = 1;
    timer_kick();
}
DECL_INIT(timer_init);
