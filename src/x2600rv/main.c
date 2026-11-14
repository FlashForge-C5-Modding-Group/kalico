// Main entry point for the Ingenic X2600 RISC-V core
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memmove
#include "board/irq.h" // irq_disable
#include "board/misc.h" // dynmem_start
#include "command.h" // DECL_CONSTANT_STR
#include "generic/timer_irq.h" // timer_dispatch_many
#include "internal.h" // ring_init
#include "sched.h" // sched_main

DECL_CONSTANT_STR("MCU", "x2600-riscv");

static char dynmem_pool[32 * 1024];

void *
dynmem_start(void)
{
    return dynmem_pool;
}

void *
dynmem_end(void)
{
    return &dynmem_pool[sizeof(dynmem_pool)];
}


/****************************************************************
 * Interrupts (none: everything is polled)
 ****************************************************************/

static struct task_wake console_wake;

void
irq_disable(void)
{
}

void
irq_enable(void)
{
}

irqstatus_t
irq_save(void)
{
    return 0;
}

void
irq_restore(irqstatus_t flag)
{
}

void
irq_wait(void)
{
    irq_poll();
}

void
irq_poll(void)
{
    if (timer_interrupt_pending()) {
        timer_clear_interrupt();
        timer_set(timer_dispatch_many());
    }
    ring_poll();
    if (ring_readable())
        sched_wake_task(&console_wake);
}


/****************************************************************
 * Console
 ****************************************************************/

static uint8_t receive_buf[256];
static uint32_t receive_pos;

void *
console_receive_buffer(void)
{
    return receive_buf;
}

void
console_task(void)
{
    if (!sched_check_wake(&console_wake))
        return;
    uint32_t len = receive_pos + ring_read(&receive_buf[receive_pos]
                                           , sizeof(receive_buf) - receive_pos);
    uint_fast8_t pop_count, msglen = len > MESSAGE_MAX ? MESSAGE_MAX : len;
    int ret = command_find_and_dispatch(receive_buf, msglen, &pop_count);
    if (ret) {
        len -= pop_count;
        if (len) {
            memmove(receive_buf, &receive_buf[pop_count], len);
            sched_wake_task(&console_wake);
        }
    }
    receive_pos = len;
}
DECL_TASK(console_task);

void
console_sendf(const struct command_encoder *ce, va_list args)
{
    uint8_t buf[MESSAGE_MAX];
    uint_fast8_t msglen = command_encode_and_frame(buf, ce, args);
    ring_write(buf, msglen);
}


/****************************************************************
 * Startup and reset
 ****************************************************************/

extern char __data_start, __data_end, __copy_start;
extern void _start(void);

void
command_reset(uint32_t *args)
{
    memcpy(&__data_start, &__copy_start, &__data_end - &__data_start);
    _start();
}
DECL_COMMAND_FLAGS(command_reset, HF_IN_SHUTDOWN, "reset");

void main(void);
void
main(void)
{
    // Keep a pristine copy of .data for "reset" (restored before restarting)
    memcpy(&__copy_start, &__data_start, &__data_end - &__data_start);
    ring_init();
    sched_main();
}
