// Creator 5 mainBoardGD failure handling
//
// Upstream's DefaultHandler spins forever on an unhandled CPU fault, which
// leaves the motor PWM timers running and the windings energised at whatever
// vector was last written until the free watchdog resets the board half a
// second later.  That is how this board produced an audible grind while
// otherwise appearing dead.
//
// This file cuts motor drive first, then reports what happened over the
// existing Klipper serial link:
//
//   - real fault handlers for NMI, HardFault, MemManage, BusFault and
//     UsageFault, which disable the motor outputs and report the faulting
//     frame;
//   - a main-loop stall detector, driven from two priority-0 contexts so it
//     can preempt a stuck motor interrupt and still fire when the host is
//     not transmitting;
//   - the reset cause latched from RCU_RSTSCK, and the low-voltage detector,
//     reported once the host is actually listening.
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdarg.h>
#include <stdint.h>
#include "board/internal.h" // SCB
#include "board/misc.h" // timer_read_time
#include "c5_mainboardgd.h" // c5_mainboardgd_pwm_enable
#include "command.h" // _DECL_OUTPUT
#include "generic/armcm_boot.h" // DECL_ARMCM_IRQ
#include "generic/serial_irq.h" // serial_get_tx_byte
#include "sched.h" // DECL_TASK

// Report a stop once the main task loop has not advanced for this long.  The
// free watchdog fires 412-512ms after the last feed, so this must be
// comfortably below it.
#define STALL_REPORT_TICKS (CONFIG_CLOCK_FREQ / 8)

// Period of the self-generated priority-0 stall tick.
#define STALL_TICK_HZ 100U

// Bound the polled transmit so a dead USART cannot hold off the watchdog.
#define POLL_TBE_LIMIT 2000000U

static uint32_t heartbeat_time;
static uint8_t stall_reported, heartbeat_started;

/****************************************************************
 * Reporting
 ****************************************************************/

// Push the pending serial transmit buffer out with polled writes.  The TX
// interrupt cannot run from a fault handler, and cannot preempt the USART
// priority when the stall check itself is what is running.
static void
flush_polled(void)
{
    uint8_t data;
    while (!serial_get_tx_byte(&data)) {
        uint32_t guard = POLL_TBE_LIMIT;
        while (!(USART0->STAT & USART_STAT_TBE))
            if (!--guard)
                return;
        USART0->TDATA = data;
    }
}

// command_sendf() silently drops a message raised from an interrupt while the
// main code was already inside sendf.  That loss is indistinguishable from
// the failure being reported, so encode directly instead.  Every caller here
// ends in a spin or is already past the point of no return, so the preempted
// sendf never resumes and cannot be corrupted by this.
static void
report_sendf(const struct command_encoder *ce, ...)
{
    va_list args;
    va_start(args, ce);
    console_sendf(ce, args);
    va_end(args);
    flush_polled();
}

#define c5_report(FMT, args...) report_sendf(_DECL_OUTPUT(FMT) , ##args )

/****************************************************************
 * Unhandled CPU fault
 ****************************************************************/

void __visible __noreturn
c5_fault_report(uint32_t *frame, uint32_t exc_return)
{
    // Cut motor drive before anything else.  Reporting is worth nothing if
    // the windings stay energised until the watchdog fires.
    uint_fast8_t axis;
    for (axis = 0; axis < 3; axis++)
        c5_mainboardgd_pwm_enable(axis, 0);

    c5_report("c5 fault: cfsr=%u hfsr=%u mmfar=%u bfar=%u"
              " pc=%u lr=%u psr=%u excret=%u"
              , SCB->CFSR, SCB->HFSR, SCB->MMFAR, SCB->BFAR
              , frame[6], frame[5], frame[7], exc_return);
    // Leave the board stopped; the free watchdog resets it.
    for (;;)
        ;
}

// Capture the exception frame of the faulting context and report it.
void __visible __attribute__((naked))
c5_fault_handler(void)
{
    asm volatile(
        "mrs r0, msp\n"
        "tst lr, #4\n"
        "beq 1f\n"
        "mrs r0, psp\n"
        "1:\n"
        "mov r1, lr\n"
        "b c5_fault_report\n");
}
DECL_ARMCM_IRQ(c5_fault_handler, NonMaskableInt_IRQn);
DECL_ARMCM_IRQ(c5_fault_handler, HardFault_IRQn);
DECL_ARMCM_IRQ(c5_fault_handler, MemoryManagement_IRQn);
DECL_ARMCM_IRQ(c5_fault_handler, BusFault_IRQn);
DECL_ARMCM_IRQ(c5_fault_handler, UsageFault_IRQn);

/****************************************************************
 * Main loop stall
 ****************************************************************/

// Stamp the time the task loop actually ran.  Stamping this from the USART
// interrupt instead would re-arm the window on the first host byte after a
// hang and make the detector unable to ever fire.
void
c5_fault_heartbeat_task(void)
{
    heartbeat_time = timer_read_time();
    heartbeat_started = 1;
    stall_reported = 0;
}
DECL_TASK(c5_fault_heartbeat_task);

// Called from priority-0 contexts that can preempt a stuck motor interrupt.
// frame is the preempted context's exception frame.
void
c5_fault_stall_check(uint32_t *frame)
{
    if (stall_reported || !heartbeat_started)
        return;
    uint32_t now = timer_read_time();
    if (timer_is_before(now, heartbeat_time + STALL_REPORT_TICKS))
        return;
    stall_reported = 1;
    c5_report("c5 stall: icsr=%u shcsr=%u cfsr=%u pc=%u lr=%u psr=%u ticks=%u"
              , SCB->ICSR, SCB->SHCSR, SCB->CFSR
              , frame[6], frame[5], frame[7], now - heartbeat_time);
}

/****************************************************************
 * Supply and startup reporting
 ****************************************************************/

static uint32_t boot_reset_flags, lvd_events;
static uint8_t boot_reported, host_seen;

// Called from the USART0 interrupt when the host transmits.  The boot report
// is held until then: stock Klippy connects well after the two seconds a
// fixed delay would have used, so an unconditional early report is sent into
// a closed port.
void
c5_fault_note_host(void)
{
    host_seen = 1;
}

void __visible
c5_stall_tick_body(uint32_t *frame)
{
    TIMER_INTF(TIMER6) = ~TIMER_INTF_UPIF;
    // Brown-out reset is disabled in this part's factory option bytes, so a
    // supply sag short of the power-down threshold is otherwise invisible.
    // Sampling here rather than in the motor interrupt keeps this off the
    // 20kHz path; it catches a sustained sag, not a single-period transient.
    if (PMU_CS & PMU_CS_LVDF)
        lvd_events++;
    c5_fault_stall_check(frame);
}

void __visible __attribute__((naked))
TIMER6_IRQHandler(void)
{
    asm volatile(
        "mrs r0, msp\n"
        "tst lr, #4\n"
        "beq 1f\n"
        "mrs r0, psp\n"
        "1:\n"
        "b c5_stall_tick_body\n");
}
DECL_ARMCM_IRQ(TIMER6_IRQHandler, TIMER6_IRQn);

void
c5_fault_startup(void)
{
    boot_reset_flags = RCU_RSTSCK;
    RCU_RSTSCK |= RCU_RSTSCK_RSTFC;

    // Arm the low-voltage detector at 2.9V, well above the operating minimum
    // and below the nominal rail.  It raises no reset and no interrupt.
    RCU_APB4EN |= RCU_APB4EN_PMUEN;
    PMU_CTL0 = (PMU_CTL0 & ~PMU_CTL0_LVDT) | PMU_LVDT_2V9 | PMU_CTL0_LVDEN;

    // Periodic priority-0 stall tick.  Independent of host traffic.
    RCU_APB1EN |= RCU_APB1EN_TIMER6EN;
    TIMER_CTL0(TIMER6) = 0;
    TIMER_PSC(TIMER6) = (get_pclock_frequency(TIMER1) / 1000000U) - 1U;
    TIMER_CAR(TIMER6) = (1000000U / STALL_TICK_HZ) - 1U;
    TIMER_SWEVG(TIMER6) = TIMER_SWEVG_UPG;
    TIMER_INTF(TIMER6) = ~TIMER_INTF_UPIF;
    TIMER_DMAINTEN(TIMER6) = TIMER_DMAINTEN_UPIE;
    armcm_enable_irq(TIMER6_IRQHandler, TIMER6_IRQn, 0);
    TIMER_CTL0(TIMER6) = TIMER_CTL0_CEN;
}
DECL_INIT(c5_fault_startup);

// Report the reset cause and any supply events once the host is listening.
void
c5_fault_boot_task(void)
{
    if (boot_reported || !host_seen)
        return;
    boot_reported = 1;
    c5_report("c5 boot: rstsck=%u lvd=%u", boot_reset_flags, lvd_events);
}
DECL_TASK(c5_fault_boot_task);
