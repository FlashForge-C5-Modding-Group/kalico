// Startup and clock support for the Nations N32G45x
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#ifdef N32G45X_REGISTER_MODEL
#include "n32g45x_register_model.h"
#else
#include "autoconf.h" // CONFIG_CLOCK_FREQ
#include "board/armcm_boot.h" // VectorTable
#include "internal.h" // enable_pclock
#include "sched.h" // sched_main
#endif

#ifndef N32G45X_CLOCK_TIMEOUT
#define N32G45X_CLOCK_TIMEOUT 1000000u
#endif
#ifndef N32G45X_WAIT_POLL
#define N32G45X_WAIT_POLL(reg, mask, expected) do { } while (0)
#endif
#ifndef N32G45X_RESET_REQUESTED
#define N32G45X_RESET_REQUESTED() do { } while (0)
#endif
#define N32G45X_PLLMUL_EXT (1u << 27)
#define N32G45X_PLLMUL_MASK (RCC_CFGR_PLLMULL_Msk | N32G45X_PLLMUL_EXT)
#define N32G45X_USBPRES_MASK (3u << 22)
#define N32G45X_FLASH_ICRST (1u << 6)
#define N32G45X_FLASH_ICEN (1u << 7)
#define N32G45X_PWR_CTRL3 ((volatile uint32_t *)(PWR_BASE + 0x0cu))

#if !CONFIG_STM32_CLOCK_REF_INTERNAL
#if !(2 * CONFIG_CLOCK_FREQ % CONFIG_CLOCK_REF_FREQ) \
    && 2 * CONFIG_CLOCK_FREQ / CONFIG_CLOCK_REF_FREQ >= 2 \
    && 2 * CONFIG_CLOCK_FREQ / CONFIG_CLOCK_REF_FREQ <= 32
#define N32G45X_PLL_HSE_DIV2 1
#define N32G45X_PLL_MULTIPLIER \
    (2 * CONFIG_CLOCK_FREQ / CONFIG_CLOCK_REF_FREQ)
#elif !(CONFIG_CLOCK_FREQ % CONFIG_CLOCK_REF_FREQ) \
    && CONFIG_CLOCK_FREQ / CONFIG_CLOCK_REF_FREQ >= 2 \
    && CONFIG_CLOCK_FREQ / CONFIG_CLOCK_REF_FREQ <= 32
#define N32G45X_PLL_HSE_DIV2 0
#define N32G45X_PLL_MULTIPLIER \
    (CONFIG_CLOCK_FREQ / CONFIG_CLOCK_REF_FREQ)
#else
#error "Unable to generate the requested clock rate from this crystal"
#endif
#else
#if (2 * CONFIG_CLOCK_FREQ) % 8000000 \
    || 2 * CONFIG_CLOCK_FREQ / 8000000 < 2 \
    || 2 * CONFIG_CLOCK_FREQ / 8000000 > 32
#error "Unable to generate the requested clock rate from HSI"
#endif
#define N32G45X_PLL_HSE_DIV2 0
#define N32G45X_PLL_MULTIPLIER 0
#endif
#if CONFIG_USB && CONFIG_CLOCK_FREQ != 96000000
#error "Unable to generate a 48Mhz usb clock at this system clock rate"
#endif

void noinline __noreturn
n32g45x_clock_fail(void)
{
    __disable_irq();
    __DSB();
    SCB->AIRCR = ((0x5fau << SCB_AIRCR_VECTKEY_Pos)
                  | (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk)
                  | SCB_AIRCR_SYSRESETREQ_Msk);
    __DSB();
    N32G45X_RESET_REQUESTED();
    for (;;)
        __NOP();
}

void noinline
n32g45x_wait_mask_or_reset(volatile uint32_t *reg, uint32_t mask,
                           uint32_t expected)
{
    for (uint32_t timeout = N32G45X_CLOCK_TIMEOUT; timeout; timeout--) {
        N32G45X_WAIT_POLL(reg, mask, expected);
        if ((*reg & mask) == expected)
            return;
    }
    n32g45x_clock_fail();
}

static uint32_t
n32g45x_pll_multiplier_bits(uint32_t mul)
{
    if (mul > 16)
        return ((mul - 17) << RCC_CFGR_PLLMULL_Pos) | N32G45X_PLLMUL_EXT;
    return (mul - 2) << RCC_CFGR_PLLMULL_Pos;
}

void noinline
n32g45x_clock_setup(void)
{
    // A stock boot stage may jump here with PLL already driving SYSCLK.
    RCC->CR |= RCC_CR_HSION;
    n32g45x_wait_mask_or_reset(&RCC->CR, RCC_CR_HSIRDY, RCC_CR_HSIRDY);

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_HSI;
    n32g45x_wait_mask_or_reset(&RCC->CFGR, RCC_CFGR_SWS_Msk,
                               RCC_CFGR_SWS_HSI);

    // Return the clock tree to its reset state like the stock SystemInit:
    // stop PLL/HSE/CSS and clear MCO, prescalers, PLL, and RCC interrupts
    // that a boot stage may have left configured.
    RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_CSSON | RCC_CR_HSEON);
    n32g45x_wait_mask_or_reset(&RCC->CR, RCC_CR_PLLRDY, 0);
    RCC->CR &= ~RCC_CR_HSEBYP;
    RCC->CFGR &= ~(RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE1_Msk
                   | RCC_CFGR_PPRE2_Msk | RCC_CFGR_MCO_Msk
                   | RCC_CFGR_PLLSRC_Msk | RCC_CFGR_PLLXTPRE_Msk
                   | N32G45X_PLLMUL_MASK | N32G45X_USBPRES_MASK);
    RCC->CIR = 0x009f0000;

    // Reset inherited peripheral clocks only after the HSI takeover.
    RCC->AHBENR = 0x14;
    RCC->APB1ENR = 0;
    RCC->APB2ENR = 0;

    // Enable the N32 extended operating mode used by the official SDK.
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    *N32G45X_PWR_CTRL3 |= 1u;
    RCC->APB1ENR &= ~RCC_APB1ENR_PWREN;

    // Establish flash timing, prefetch, and I-cache state before
    // increasing SYSCLK, matching the stock eBoard/heaterBoard startup.
    uint32_t latency = (CONFIG_CLOCK_FREQ - 1) / 32000000;
    uint32_t acr = FLASH->ACR;
    acr &= ~(FLASH_ACR_LATENCY_Msk | N32G45X_FLASH_ICRST);
    FLASH->ACR = acr | latency | FLASH_ACR_PRFTBE | N32G45X_FLASH_ICEN;

    uint32_t pll_bits;
    if (!CONFIG_STM32_CLOCK_REF_INTERNAL) {
        // Prefer HSE/2 when its integral PLL multiplier is representable.
        RCC->CR |= RCC_CR_HSEON;
        n32g45x_wait_mask_or_reset(&RCC->CR, RCC_CR_HSERDY,
                                   RCC_CR_HSERDY);
        pll_bits = ((1u << RCC_CFGR_PLLSRC_Pos)
                    | (N32G45X_PLL_HSE_DIV2
                       ? RCC_CFGR_PLLXTPRE_HSE_DIV2 : 0)
                    | n32g45x_pll_multiplier_bits(
                        N32G45X_PLL_MULTIPLIER));
    } else {
        uint32_t mul = (2 * CONFIG_CLOCK_FREQ) / 8000000;
        pll_bits = n32g45x_pll_multiplier_bits(mul);
    }

    // HCLK=SYSCLK, PCLK1=HCLK/4, and PCLK2=HCLK/2 at 144MHz.
    uint32_t cfgr = RCC->CFGR;
    cfgr &= ~RCC_CFGR_SW_Msk;
    if (CONFIG_CLOCK_FREQ > 72000000)
        cfgr |= RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
    else if (CONFIG_CLOCK_FREQ > 36000000)
        cfgr |= RCC_CFGR_PPRE1_DIV2;
    cfgr |= pll_bits;
    if (CONFIG_CLOCK_FREQ == 96000000)
        cfgr |= 2u << 22;
    RCC->CFGR = cfgr;

    RCC->CR |= RCC_CR_PLLON;
    n32g45x_wait_mask_or_reset(&RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY);

    RCC->CFGR = cfgr | RCC_CFGR_SW_PLL;
    n32g45x_wait_mask_or_reset(&RCC->CFGR, RCC_CFGR_SWS_Msk,
                               RCC_CFGR_SWS_PLL);
}

// Main entry point - called from armcm_boot.c:ResetHandler().
void noinline
armcm_main(void)
{
    n32g45x_clock_setup();

    SCB->VTOR = (uint32_t)(uintptr_t)VectorTable;

    // Disable JTAG to free PA15, PB3, and PB4 while retaining SWD.
    enable_pclock(AFIO_BASE);
    if (CONFIG_STM32F103GD_DISABLE_SWD)
        stm32f1_alternative_remap(AFIO_MAPR_SWJ_CFG_Msk,
                                  AFIO_MAPR_SWJ_CFG_DISABLE);
    else
        stm32f1_alternative_remap(AFIO_MAPR_SWJ_CFG_Msk,
                                  AFIO_MAPR_SWJ_CFG_JTAGDISABLE);

    sched_main();
}
