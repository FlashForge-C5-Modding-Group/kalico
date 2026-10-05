#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define N32G430_REGISTER_MODEL 1
#include "n32g430_register_model.h"
#include "../src/stm32/n32g430.c"
#include "../src/stm32/clockline.c"
#include "../src/stm32/c5_levelboard.c"
_Static_assert(N32G430_CLOCK_TIMEOUT > N32G430_HSI_CLOCK_FREQ / 10u,
               "clock polling must exceed 100ms at the HSI frequency");

RCC_TypeDef model_rcc;
FLASH_TypeDef model_flash;
SCB_Type model_scb;
CoreDebug_Type model_core_debug;
DWT_Type model_dwt;
GPIO_TypeDef model_gpio_ports[4];
TIM_TypeDef model_tim1, model_tim8;
DMA_TypeDef model_dma1;
DMA_Channel_TypeDef model_dma1_channels[DMA1_CHANNEL_COUNT];
USART_TypeDef model_usart1;
uint32_t VectorTable[1];

static int stalled_wait;
static int wait_index;
static uint32_t wait_polls;
static volatile uint32_t *last_wait_reg;
static uint32_t last_wait_mask, last_wait_expected;
static int scheduler_reached;
static unsigned hse_starts;
static int hse_sequence_error;
static uint16_t captured_delta;
static unsigned capture_count;
static void (*installed_irq)(void);
static IRQn_Type installed_irqn;
static uint32_t installed_priority;
static unsigned dma_write_count;
static int unsafe_dma_write;
static volatile uint32_t *dma_write_regs[12];
static uint32_t dma_write_values[12];

#define SEEDED_PRIGROUP (5u << 8)
// Past 0x80000000, which is what breaks sched.c's timer list invariant.
#define SEEDED_CYCCNT 0x9abcdef0u

static int
fail(const char *name, uint32_t actual, uint32_t expected)
{
    if (actual == expected)
        return 0;
    fprintf(stderr, "%s: expected 0x%08x, got 0x%08x\n",
            name, expected, actual);
    return 1;
}

// Record a new wait whenever the arguments change or the previous wait
// used its complete polling budget (a timed-out attempt being retried).
static int
model_new_wait(volatile uint32_t *reg, uint32_t mask, uint32_t expected)
{
    if (reg == last_wait_reg && mask == last_wait_mask
        && expected == last_wait_expected
        && wait_polls < N32G430_CLOCK_TIMEOUT) {
        wait_polls++;
        return 0;
    }
    int restarted = (last_wait_reg == &model_rcc.CR
                     && last_wait_mask == RCC_CR_HSERDY
                     && last_wait_expected == 0);
    wait_index++;
    wait_polls = 1;
    last_wait_reg = reg;
    last_wait_mask = mask;
    last_wait_expected = expected;
    return restarted ? 2 : 1;
}

void
n32g430_model_poll(volatile uint32_t *reg, uint32_t mask,
                    uint32_t expected)
{
    int new_wait = model_new_wait(reg, mask, expected);
    if (new_wait && reg == &model_rcc.CR && mask == RCC_CR_HSERDY) {
        uint32_t cr = model_rcc.CR;
        if (expected == 0) {
            // The HSE may only stop once nothing is clocked from it.
            if ((cr & (RCC_CR_HSEON | RCC_CR_PLLON))
                || (model_rcc.CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSI)
                hse_sequence_error = 1;
        } else {
            // Every start must follow a completed stop, in crystal mode.
            hse_starts++;
            if (new_wait != 2 || !(cr & RCC_CR_HSEON)
                || (cr & RCC_CR_HSEBYP))
                hse_sequence_error = 1;
        }
    }
    if (wait_index == stalled_wait)
        return;
    *reg = (*reg & ~mask) | expected;
}

void
n32g430_model_dma_channel_write(volatile uint32_t *reg, uint32_t value)
{
    // While active, the only legal channel write is an exact EN clear.
    uint32_t control = model_dma1_channels[0].CCR;
    if ((control & DMA_CCR_EN)
        && (reg != &model_dma1_channels[0].CCR
            || value != (control & ~DMA_CCR_EN)))
        unsafe_dma_write = 1;
    if (dma_write_count < sizeof(dma_write_regs) / sizeof(dma_write_regs[0])) {
        dma_write_regs[dma_write_count] = reg;
        dma_write_values[dma_write_count] = value;
    }
    dma_write_count++;
    *reg = value;
}

void model_dsb(void) { }
void model_isb(void) { }
irqstatus_t irq_save(void) { return 0; }
void irq_restore(irqstatus_t flag) { (void)flag; }
void gpio_peripheral(uint32_t gpio, uint32_t mode, int pullup)
{
    (void)gpio;
    (void)mode;
    (void)pullup;
}
void
armcm_enable_irq(void (*func)(void), IRQn_Type irq, uint32_t priority)
{
    installed_irq = func;
    installed_irqn = irq;
    installed_priority = priority;
}
void
c5_levelboard_capture(uint16_t delta)
{
    captured_delta = delta;
    capture_count++;
}
void sched_main(void) { scheduler_reached = 1; }

static void
reset_model(int stall)
{
    memset(&model_rcc, 0, sizeof(model_rcc));
    // Boot-stage handoff: SYSCLK from the PLL, driven by a running HSE.
    model_rcc.CR = RCC_CR_HSEON | RCC_CR_HSERDY | RCC_CR_PLLON
                   | RCC_CR_PLLRDY;
    model_rcc.CFGR = RCC_CFGR_SW_PLL | RCC_CFGR_SWS_PLL;
    memset(&model_flash, 0, sizeof(model_flash));
    memset(&model_scb, 0, sizeof(model_scb));
    model_scb.AIRCR = SEEDED_PRIGROUP;
    // The debug power domain is not reset by SYSRESETREQ: the cycle counter
    // keeps running through the reset and through the boot stage's wait.
    model_core_debug.DEMCR = CoreDebug_DEMCR_TRCENA_Msk;
    model_dwt.CYCCNT = SEEDED_CYCCNT;
    memset(model_gpio_ports, 0, sizeof(model_gpio_ports));
    memset(&model_tim1, 0, sizeof(model_tim1));
    memset(&model_tim8, 0, sizeof(model_tim8));
    memset(&model_dma1, 0, sizeof(model_dma1));
    memset(model_dma1_channels, 0, sizeof(model_dma1_channels));
    memset(&model_usart1, 0, sizeof(model_usart1));
    // Boot-stage handoff: its USART1 DMA channels are still configured and
    // still pointed at what is now application RAM, while both peripheral
    // clocks are gated off.  Gating does not clear the registers, so the
    // channels become live again the moment the application ungates DMA.
    model_rcc.AHBENR = 0;
    model_rcc.APB2ENR = 0;
    model_usart1.CR1 = 0x202cu;
    model_usart1.CR3 = 0x00c0u;
    model_dma1_channels[3].CCR = 0x3092u;
    model_dma1_channels[3].CPAR = USART1_BASE + 4u;
    model_dma1_channels[3].CMAR = 0x20001ddfu;
    model_dma1_channels[4].CCR = 0x3081u;
    model_dma1_channels[4].CNDTR = 0x40bu;
    model_dma1_channels[4].CPAR = USART1_BASE + 4u;
    model_dma1_channels[4].CMAR = 0x20000040u;
    stalled_wait = stall;
    wait_index = 0;
    wait_polls = 0;
    last_wait_reg = NULL;
    last_wait_mask = 0;
    last_wait_expected = 0;
    hse_starts = 0;
    hse_sequence_error = 0;
    scheduler_reached = 0;
    dma_write_count = 0;
    unsafe_dma_write = 0;
}

static uint32_t
decode_hclk(void)
{
    uint32_t cfgr = model_rcc.CFGR;
    uint32_t pll_input = 8000000u;
    if (cfgr & RCC_CFGR_PLLXTPRE_HSE_DIV2)
        pll_input /= 2u;
    uint32_t multiplier = ((cfgr & (1u << 27))
                           ? ((cfgr >> 18) & 0x0fu) + 17u
                           : ((cfgr >> 18) & 0x0fu) + 2u);
    uint32_t hclk = pll_input * multiplier;
    static const uint16_t divisors[8] = {2, 4, 8, 16, 64, 128, 256, 512};
    uint32_t hpre = (cfgr >> 4) & 0x0fu;
    if (hpre >= 8)
        hclk /= divisors[hpre - 8];
    return hclk;
}

static uint32_t
decode_pclock(uint32_t shift)
{
    uint32_t prescaler = (model_rcc.CFGR >> shift) & 0x07u;
    if (prescaler < 4)
        return decode_hclk();
    return decode_hclk() / (2u << (prescaler - 4u));
}

// HSI ready, HSI selected, PLL stopped, HSE stopped, HSE ready, PLL ready
// and PLL selected.
#define CLOCK_WAITS 7

static int
check_clock_result(int expected_waits, unsigned expected_hse_starts)
{
    int failures = 0;
    failures += fail("wait count", wait_index, expected_waits);
    failures += fail("scheduler reached", scheduler_reached, 1);
    failures += fail("no reset requested", model_scb.AIRCR, SEEDED_PRIGROUP);
    failures += fail("HSE starts", hse_starts, expected_hse_starts);
    failures += fail("HSE stop/start order", hse_sequence_error, 0);
    failures += fail("HSE running in crystal mode",
                     model_rcc.CR & (RCC_CR_HSEON | RCC_CR_HSERDY
                                     | RCC_CR_HSEBYP),
                     RCC_CR_HSEON | RCC_CR_HSERDY);
    failures += fail("HCLK", decode_hclk(), 128000000u);
    failures += fail("PCLK1", decode_pclock(8), 32000000u);
    failures += fail("PCLK2", decode_pclock(11), 64000000u);
    failures += fail("PLL selected",
                     model_rcc.CFGR & RCC_CFGR_SWS_Msk, RCC_CFGR_SWS_PLL);
    failures += fail("HSE PLL source",
                     model_rcc.CFGR & RCC_CFGR_PLLSRC_HSE,
                     RCC_CFGR_PLLSRC_HSE);
    failures += fail("HSE divided by two",
                     model_rcc.CFGR & RCC_CFGR_PLLXTPRE_HSE_DIV2,
                     RCC_CFGR_PLLXTPRE_HSE_DIV2);
    failures += fail("PLL multiplier",
                     model_rcc.CFGR & RCC_CFGR_PLLMUL_Msk,
                     RCC_CFGR_PLLMUL32);
    failures += fail("TIM1/TIM8 use 128MHz timer clock",
                     model_rcc.CFGR2 & RCC_CFGR2_TIM1_8_SEL, 0);
    failures += fail("flash latency and cache", model_flash.ACR,
                     FLASH_ACR_LATENCY_3 | FLASH_ACR_ICEN);
    return failures;
}

// Every leftover channel must be stopped before the application can rely on
// its RAM or its serial port.
static int
check_handoff_quiesce(void)
{
    int failures = 0;
    failures += fail("boot USART receiver stopped", model_usart1.CR1, 0);
    failures += fail("boot USART DMA requests stopped", model_usart1.CR3, 0);
    for (int ch = 1; ch < DMA1_CHANNEL_COUNT; ch++) {
        failures += fail("leftover channel control",
                         model_dma1_channels[ch].CCR, 0);
        failures += fail("leftover channel count",
                         model_dma1_channels[ch].CNDTR, 0);
    }
    // An inherited count past 0x80000000 makes the first timer added by an
    // init function insert past sched.c's sentinel.
    failures += fail("inherited cycle count cleared", model_dwt.CYCCNT, 0);
    failures += fail("cycle counter enabled", model_core_debug.DEMCR
                     & CoreDebug_DEMCR_TRCENA_Msk,
                     CoreDebug_DEMCR_TRCENA_Msk);
    return failures;
}

static int
run_normal_clock_path(void)
{
    reset_model(0);
    armcm_main();
    int failures = check_clock_result(CLOCK_WAITS, 1);
    failures += check_handoff_quiesce();
    failures += fail("USART1 reported PCLK2",
                     get_pclock_frequency(USART1_BASE), 64000000u);
    failures += fail("TIM1 reported timer clock",
                     get_pclock_frequency(TIM1_BASE), 128000000u);
    failures += fail("IWDG has no false pclock",
                     get_pclock_frequency(IWDG_BASE), 0);
    if (failures)
        fprintf(stderr, "normal clock path failed\n");
    return failures;
}

// A clock wait that times out must not reset the chip: the resident boot
// stage would then wait for a host wake-up that is never sent again.  The
// whole switch is retried, restarting the HSE each time.
static int
run_clock_retry(int stall)
{
    reset_model(stall);
    armcm_main();
    int failures = check_clock_result(stall + CLOCK_WAITS,
                                      stall >= 5 ? 2 : 1);
    if (failures)
        fprintf(stderr, "clock retry after stalled wait %d failed\n", stall);
    return failures;
}

static int
run_warm_dma_path(void)
{
    int failures = 0;
    reset_model(0);
    const uint32_t reset_sentinel = 0xa55a5aa5u;
    model_rcc.AHBRSTR = reset_sentinel;
    model_dma1.ISR = 0x0fu;
    model_dma1.IFCR = 0xa5u;
    model_dma1_channels[0].CCR = 0xffffu;
    model_dma1_channels[0].CNDTR = 0x1234u;
    model_dma1_channels[0].CPAR = 0x11111111u;
    model_dma1_channels[0].CMAR = 0x22222222u;
    model_dma1_channels[0].CHSEL = 0x3fu;
    installed_irq = NULL;
    installed_irqn = -1;
    installed_priority = UINT32_MAX;
    capture_count = 0;
    captured_delta = 0;
    previous_snapshot = 0;

    c5_levelboard_acquisition_init();
    volatile uint32_t *expected_regs[] = {
        &model_dma1_channels[0].CCR, &model_dma1_channels[0].CCR,
        &model_dma1_channels[0].CPAR, &model_dma1_channels[0].CMAR,
        &model_dma1_channels[0].CNDTR, &model_dma1_channels[0].CHSEL,
        &model_dma1_channels[0].CCR, &model_dma1_channels[0].CCR,
        &model_dma1_channels[0].CCR,
    };
    uint32_t expected_values[] = {
        0xfffeu, 0, (uint32_t)(uintptr_t)&model_tim1.CNT,
        (uint32_t)(uintptr_t)&dma_snapshot, 1, 0x32u, 0x2520u, 0x2522u,
        0x2523u,
    };
    failures += fail("DMA write count", dma_write_count,
                     sizeof(expected_regs) / sizeof(expected_regs[0]));
    failures += fail("DMA writes only while disabled", unsafe_dma_write, 0);
    unsigned compared_writes = dma_write_count;
    if (compared_writes > sizeof(expected_regs) / sizeof(expected_regs[0]))
        compared_writes = sizeof(expected_regs) / sizeof(expected_regs[0]);
    for (unsigned i = 0; i < compared_writes; i++) {
        failures += fail("DMA write register order",
                         dma_write_regs[i] == expected_regs[i], 1);
        failures += fail("DMA write value order", dma_write_values[i],
                         expected_values[i]);
    }

    failures += fail("DMA clock enabled",
                     model_rcc.AHBENR & RCC_AHBENR_DMA1EN,
                     RCC_AHBENR_DMA1EN);
    failures += fail("reserved AHBRSTR untouched", model_rcc.AHBRSTR,
                     reset_sentinel);
    failures += fail("all channel 1 flags cleared", model_dma1.IFCR, 0x0fu);
    failures += fail("DMA CPAR", model_dma1_channels[0].CPAR,
                     (uint32_t)(uintptr_t)&model_tim1.CNT);
    failures += fail("DMA CMAR", model_dma1_channels[0].CMAR,
                     (uint32_t)(uintptr_t)&dma_snapshot);
    failures += fail("DMA count", model_dma1_channels[0].CNDTR, 1);
    failures += fail("DMA request", model_dma1_channels[0].CHSEL, 0x32u);
    failures += fail("DMA final control", model_dma1_channels[0].CCR, 0x2523u);
    failures += fail("DMA IRQ number", (uint32_t)installed_irqn,
                     DMA1_Channel1_IRQn);
    failures += fail("DMA IRQ priority", installed_priority, 0);
    failures += fail("DMA IRQ handler installed",
                     installed_irq == DMA1_Channel1_IRQHandler, 1);

    model_dma1_channels[0].CCR = 0x2523u;
    model_dma1.IFCR = 0;
    model_dma1.ISR = DMA_ISR_TCIF1;
    dma_snapshot = 5;
    installed_irq();
    failures += fail("IRQ clears TC only", model_dma1.IFCR,
                     DMA_IFCR_CTCIF1);
    failures += fail("IRQ first modulo delta", captured_delta, 5);
    failures += fail("IRQ leaves DMA enabled", model_dma1_channels[0].CCR,
                     0x2523u);

    model_dma1.IFCR = 0;
    dma_snapshot = 0;
    installed_irq();
    failures += fail("IRQ wrapped modulo delta", captured_delta, 65531u);
    failures += fail("IRQ capture count", capture_count, 2);
    failures += fail("IRQ still clears TC only", model_dma1.IFCR,
                     DMA_IFCR_CTCIF1);
    return failures;
}

int
main(void)
{
    int failures = run_normal_clock_path();
    for (int stall = 1; stall <= CLOCK_WAITS; stall++)
        failures += run_clock_retry(stall);
    failures += run_warm_dma_path();
    return failures ? 1 : 0;
}
