// Startup and clock support for the N32G430F8S7
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stddef.h> // NULL
#ifdef N32G430_REGISTER_MODEL
#include "n32g430_register_model.h"
#else
#include "autoconf.h" // CONFIG_CLOCK_FREQ
#include "board/armcm_boot.h" // VectorTable
#include "internal.h" // struct cline
#include "sched.h" // sched_main
#endif

// Clock setup runs from the fixed 8MHz HSI.  Two million polls give at
// least 250ms even if a complete poll could execute in a single cycle.
#define N32G430_HSI_CLOCK_FREQ 8000000u
#define N32G430_CLOCK_TIMEOUT_MS 250u
#define N32G430_CLOCK_TIMEOUT \
    ((N32G430_HSI_CLOCK_FREQ / 1000u) * N32G430_CLOCK_TIMEOUT_MS)
#ifndef N32G430_WAIT_POLL
#define N32G430_WAIT_POLL(reg, mask, expected) do { } while (0)
#endif

// Return the enable and reset controls for the peripherals supported here.
struct cline
lookup_clock_line(uint32_t periph_base)
{
    switch (periph_base) {
    case DMA1_BASE:
        return (struct cline){ .en=&RCC->AHBENR, .rst=NULL,
                              .bit=RCC_AHBENR_DMA1EN };
    case GPIOA_BASE:
        return (struct cline){ .en=&RCC->AHBENR, .rst=&RCC->AHBRSTR,
                              .bit=RCC_AHBENR_GPIOAEN };
    case GPIOB_BASE:
        return (struct cline){ .en=&RCC->AHBENR, .rst=&RCC->AHBRSTR,
                              .bit=RCC_AHBENR_GPIOBEN };
    case GPIOC_BASE:
        return (struct cline){ .en=&RCC->AHBENR, .rst=&RCC->AHBRSTR,
                              .bit=RCC_AHBENR_GPIOCEN };
    case GPIOD_BASE:
        return (struct cline){ .en=&RCC->AHBENR, .rst=&RCC->AHBRSTR,
                              .bit=RCC_AHBENR_GPIODEN };
    case TIM1_BASE:
        return (struct cline){ .en=&RCC->APB2ENR, .rst=&RCC->APB2RSTR,
                              .bit=RCC_APB2ENR_TIM1EN };
    case TIM8_BASE:
        return (struct cline){ .en=&RCC->APB2ENR, .rst=&RCC->APB2RSTR,
                              .bit=RCC_APB2ENR_TIM8EN };
    case USART1_BASE:
        return (struct cline){ .en=&RCC->APB2ENR, .rst=&RCC->APB2RSTR,
                              .bit=RCC_APB2ENR_USART1EN };
    default:
        return (struct cline){ .en=&RCC->AHBENR, .rst=&RCC->AHBRSTR };
    }
}

// Return the operating clock supplied to a supported peripheral.
uint32_t
get_pclock_frequency(uint32_t periph_base)
{
    switch (periph_base) {
    case USART1_BASE:
        return CONFIG_CLOCK_FREQ / 2;
    case TIM1_BASE:
    case TIM8_BASE:
    case DMA1_BASE:
    case GPIOA_BASE:
    case GPIOB_BASE:
    case GPIOC_BASE:
    case GPIOD_BASE:
        return CONFIG_CLOCK_FREQ;
    default:
        return 0;
    }
}

// Enable a GPIO bank without inferring gates for unimplemented banks.
void
gpio_clock_enable(GPIO_TypeDef *regs)
{
    uint32_t bit;
    if (regs == GPIOA)
        bit = RCC_AHBENR_GPIOAEN;
    else if (regs == GPIOB)
        bit = RCC_AHBENR_GPIOBEN;
    else if (regs == GPIOC)
        bit = RCC_AHBENR_GPIOCEN;
    else if (regs == GPIOD)
        bit = RCC_AHBENR_GPIODEN;
    else
        return;
    RCC->AHBENR |= bit;
    RCC->AHBENR;
}

// Wait for a clock status field; report whether it settled in time.
static int noinline
n32g430_wait_mask(volatile uint32_t *reg, uint32_t mask, uint32_t expected)
{
    for (uint32_t timeout = N32G430_CLOCK_TIMEOUT; timeout; timeout--) {
        N32G430_WAIT_POLL(reg, mask, expected);
        if ((*reg & mask) == expected)
            return 1;
    }
    return 0;
}

// Make one complete switch to the 128MHz HSE/PLL clock.  Every attempt
// starts from the HSI with the PLL and HSE stopped, so a failed attempt
// can simply be repeated.
static int
clock_try_setup(void)
{
    // Revert to the HSI regardless of the clock state the boot stage
    // leaves behind; the PLL can not be disabled while it drives SYSCLK.
    RCC->CR |= RCC_CR_HSION;
    if (!n32g430_wait_mask(&RCC->CR, RCC_CR_HSIRDY, RCC_CR_HSIRDY))
        return 0;
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_HSI;
    if (!n32g430_wait_mask(&RCC->CFGR, RCC_CFGR_SWS_Msk, RCC_CFGR_SWS_HSI))
        return 0;

    // Clock-tree fields may only change while the PLL is disabled.
    RCC->CR &= ~RCC_CR_PLLON;
    if (!n32g430_wait_mask(&RCC->CR, RCC_CR_PLLRDY, 0))
        return 0;

    // Establish the public 128MHz flash timing before raising SYSCLK.
    uint32_t acr = FLASH->ACR;
    acr &= ~(FLASH_ACR_LATENCY_Msk | FLASH_ACR_PRFTEN | FLASH_ACR_ICRST);
    FLASH->ACR = acr | FLASH_ACR_LATENCY_3 | FLASH_ACR_ICEN;

    // Stop and restart the crystal/resonator HSE, as the stock application
    // does.  The boot stage leaves the HSE enabled even when it did not
    // start, and the bypass bit may only change while the HSE is off.
    RCC->CR &= ~RCC_CR_HSEON;
    if (!n32g430_wait_mask(&RCC->CR, RCC_CR_HSERDY, 0))
        return 0;
    RCC->CR &= ~RCC_CR_HSEBYP;
    RCC->CR |= RCC_CR_HSEON;
    if (!n32g430_wait_mask(&RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY))
        return 0;

    // HSE / 2 * 32 gives 128MHz HCLK; APB1 and APB2 are 32/64MHz.
    uint32_t cfgr = RCC->CFGR;
    cfgr &= ~(RCC_CFGR_SW_Msk | RCC_CFGR_HPRE_Msk
              | RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk
              | RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLXTPRE_HSE_DIV2
              | RCC_CFGR_PLLMUL_Msk);
    cfgr |= (RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2
             | RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLXTPRE_HSE_DIV2
             | RCC_CFGR_PLLMUL32);
    RCC->CFGR = cfgr;
    RCC->CFGR2 &= ~RCC_CFGR2_TIM1_8_SEL;

    RCC->CR |= RCC_CR_PLLON;
    if (!n32g430_wait_mask(&RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY))
        return 0;

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_PLL;
    return n32g430_wait_mask(&RCC->CFGR, RCC_CFGR_SWS_Msk, RCC_CFGR_SWS_PLL);
}

static void
clock_setup(void)
{
    // Never reset on a clock failure.  After a reset the resident boot
    // stage waits for a host wake-up that is sent only once per restart,
    // so the board would stay unreachable until power is cycled.
    while (!clock_try_setup())
        ;
}

// The resident boot stage jumps to the application with its USART1 DMA
// channels still enabled and pointed at application RAM, so quiesce every
// channel before any application state depends on memory or the serial port.
// Gating a peripheral clock does not clear its registers: the boot stage
// leaves the DMA clock off, and its channel setup becomes live again as soon
// as the application turns that clock back on.  So enable both clocks here
// rather than skipping a peripheral that looks inactive.
static void
handoff_quiesce(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    RCC->AHBENR;
    // The serial port stops first; it is what still requests transfers.
    USART1->CR1 = 0;
    USART1->CR3 = 0;
    for (int ch = 0; ch < DMA1_CHANNEL_COUNT; ch++) {
        DMA_Channel_TypeDef *regs = DMA1_Channel(ch);
        regs->CCR &= ~DMA_CCR_EN;
        regs->CCR = 0;
        regs->CNDTR = 0;
        DMA1->IFCR = DMA_IFCR_CHANNEL1_ALL << (4 * ch);
    }
}

// The DWT cycle counter is Klipper's clock, and it lives in the debug power
// domain: a SYSRESETREQ system reset does not clear it, so it keeps counting
// through the reset and through however long the boot stage waits for its
// host wake-up.  timer_init() zeroes it, but init functions that run earlier
// read it first, and sched.c's timer list assumes every waketime is less
// than half the counter range ahead of the sentinel.  An inherited count
// past 0x80000000 therefore walks insert_timer() off the end of the list.
// Start every instance from zero, as a power-on reset would.
static void
cycle_counter_reset(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
}

// Main entry point - called from armcm_boot.c:ResetHandler().
void
armcm_main(void)
{
    handoff_quiesce();
    cycle_counter_reset();
    clock_setup();

    SCB->VTOR = (uint32_t)(uintptr_t)VectorTable;
    __DSB();
    __ISB();

    sched_main();
}
