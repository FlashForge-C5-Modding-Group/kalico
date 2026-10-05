#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define N32G45X_REGISTER_MODEL 1
#include "n32g45x_register_model.h"
// The register model supplies the MCU declarations used by stm32f1.c.
#define __STM32_INTERNAL_H
#include "../src/stm32/stm32f1.c"
#include "../src/stm32/n32g45x.c"

RCC_TypeDef model_rcc;
FLASH_TypeDef model_flash;
AFIO_TypeDef model_afio;
uint32_t model_pwr_words[4];
BKP_TypeDef model_bkp;
SCB_Type model_scb;
GPIO_TypeDef model_gpio_ports[8];
uint32_t VectorTable[1];

static jmp_buf reset_jump;
static int stalled_condition;
static int reset_requested;
static int scheduler_reached;
static int model_error;

static void
model_fail(int code)
{
    model_error = code;
    longjmp(reset_jump, 2);
}
static int check_clock_plan(void);

void
n32g45x_model_poll(volatile uint32_t *reg, uint32_t mask,
                    uint32_t expected)
{
    int condition;
    if (reg == &RCC->CR && mask == RCC_CR_HSIRDY)
        condition = 0;
    else if (reg == &RCC->CFGR && mask == RCC_CFGR_SWS_Msk
             && expected == RCC_CFGR_SWS_HSI)
        condition = 1;
    else if (reg == &RCC->CR && mask == RCC_CR_PLLRDY && !expected)
        condition = 2;
    else if (reg == &RCC->CR && mask == RCC_CR_HSERDY)
        condition = 3;
    else if (reg == &RCC->CR && mask == RCC_CR_PLLRDY
             && expected == RCC_CR_PLLRDY)
        condition = 4;
    else if (reg == &RCC->CFGR && mask == RCC_CFGR_SWS_Msk
             && expected == RCC_CFGR_SWS_PLL)
        condition = 5;
    else
        model_fail(10);

    if (condition == 0) {
        if (!(RCC->CR & RCC_CR_HSION) || !(RCC->CR & RCC_CR_PLLON)
            || (RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL)
            model_fail(20);
    } else if (condition == 1) {
        if (!(RCC->CR & RCC_CR_HSIRDY) || !(RCC->CR & RCC_CR_PLLON)
            || (RCC->CFGR & RCC_CFGR_SW_Msk) != RCC_CFGR_SW_HSI)
            model_fail(21);
    } else if (condition == 2) {
        if ((RCC->CR & (RCC_CR_PLLON | RCC_CR_HSEON | RCC_CR_CSSON))
            || (RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSI)
            model_fail(22);
    } else if (condition == 3) {
        if ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSI
            || (RCC->CR & (RCC_CR_PLLON | RCC_CR_PLLRDY | RCC_CR_HSEBYP))
            || !(RCC->CR & RCC_CR_HSEON)
            || (RCC->CFGR & RCC_CFGR_MCO_Msk)
            || RCC->CIR != 0x009f0000u)
            model_fail(23);
    } else if (condition == 4) {
        if ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSI
            || !(RCC->CR & RCC_CR_PLLON) || !(RCC->CR & RCC_CR_HSEON)
            || !(RCC->CR & RCC_CR_HSERDY) || check_clock_plan())
            model_fail(24);
    } else {
        uint32_t flash_mask = (FLASH_ACR_LATENCY_Msk | FLASH_ACR_PRFTBE
                               | N32G45X_FLASH_ICRST | N32G45X_FLASH_ICEN);
        uint32_t flash_expected = ((CONFIG_CLOCK_FREQ - 1) / 32000000
                                   | FLASH_ACR_PRFTBE | N32G45X_FLASH_ICEN);
        if ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSI
            || !(RCC->CR & RCC_CR_PLLRDY)
            || (RCC->CFGR & RCC_CFGR_SW_Msk) != RCC_CFGR_SW_PLL
            || (FLASH->ACR & flash_mask) != flash_expected)
            model_fail(25);
    }

    if (condition == stalled_condition)
        return;

    if (condition == 0)
        RCC->CR |= RCC_CR_HSIRDY;
    else if (condition == 1)
        RCC->CFGR = ((RCC->CFGR & ~RCC_CFGR_SWS_Msk)
                     | RCC_CFGR_SWS_HSI);
    else if (condition == 2)
        RCC->CR &= ~(RCC_CR_PLLRDY | RCC_CR_HSERDY);
    else if (condition == 3)
        RCC->CR |= RCC_CR_HSERDY;
    else if (condition == 4)
        RCC->CR |= RCC_CR_PLLRDY;
    else
        RCC->CFGR = ((RCC->CFGR & ~RCC_CFGR_SWS_Msk)
                     | RCC_CFGR_SWS_PLL);
}

void
n32g45x_model_reset_requested(void)
{
    uint32_t reset_mask = (0xffffu << SCB_AIRCR_VECTKEY_Pos
                           | SCB_AIRCR_PRIGROUP_Msk
                           | SCB_AIRCR_SYSRESETREQ_Msk);
    uint32_t expected = (0x5fau << SCB_AIRCR_VECTKEY_Pos
                         | (3u << 8) | SCB_AIRCR_SYSRESETREQ_Msk);
    if ((SCB->AIRCR & reset_mask) != expected)
        model_fail(11);
    reset_requested = 1;
    longjmp(reset_jump, 1);
}

void model_disable_irq(void) { }
void model_dsb(void) { }
void model_nop(void) { }
void irq_disable(void) { }
void try_request_canboot(void) { }
void NVIC_SystemReset(void) { }
void enable_pclock(uint32_t periph_base) { (void)periph_base; }

void
sched_main(void)
{
    scheduler_reached = 1;
}

GPIO_TypeDef *
gpio_pin_to_regs(uint32_t pin)
{
    return &model_gpio_ports[pin / 16];
}

static void
reset_model(int stall)
{
    memset(&model_rcc, 0, sizeof(model_rcc));
    memset(&model_flash, 0, sizeof(model_flash));
    memset(&model_afio, 0, sizeof(model_afio));
    memset(&model_pwr_words, 0, sizeof(model_pwr_words));
    memset(&model_bkp, 0, sizeof(model_bkp));
    memset(&model_scb, 0, sizeof(model_scb));
    memset(&model_gpio_ports, 0, sizeof(model_gpio_ports));
    // Model a boot stage that leaves PLL from HSE, CSS, MCO, and RCC
    // interrupts configured with the prefetch buffer disabled.
    RCC->CR = (RCC_CR_PLLON | RCC_CR_PLLRDY | RCC_CR_HSEON | RCC_CR_HSERDY
               | RCC_CR_CSSON);
    RCC->CFGR = (RCC_CFGR_SW_PLL | RCC_CFGR_SWS_PLL | RCC_CFGR_MCO_Msk
                 | RCC_CFGR_PLLSRC_Msk);
    RCC->CIR = 0x00001f00u;
    SCB->AIRCR = 3u << 8;
    stalled_condition = stall;
    reset_requested = 0;
    scheduler_reached = 0;
    model_error = 0;
}

static int
check_clock_plan(void)
{
    uint32_t cfgr = RCC->CFGR;
    uint32_t encoded = ((cfgr & RCC_CFGR_PLLMULL_Msk)
                        >> RCC_CFGR_PLLMULL_Pos);
    uint32_t multiplier = (cfgr & (1u << 27)) ? encoded + 17 : encoded + 2;
    uint32_t input = CONFIG_CLOCK_REF_FREQ;
    if (cfgr & RCC_CFGR_PLLXTPRE_HSE_DIV2)
        input /= 2;

    if (!(cfgr & RCC_CFGR_PLLSRC_Msk))
        return 30;
    if ((cfgr & RCC_CFGR_PPRE1_Msk) != RCC_CFGR_PPRE1_DIV4)
        return 31;
    if ((cfgr & RCC_CFGR_PPRE2_Msk) != RCC_CFGR_PPRE2_DIV2)
        return 32;
    if (input * multiplier != CONFIG_CLOCK_FREQ)
        return 33;
    uint32_t divided_multiplier = (2u * CONFIG_CLOCK_FREQ
                                   / CONFIG_CLOCK_REF_FREQ);
    int divide_hse = !(2u * CONFIG_CLOCK_FREQ % CONFIG_CLOCK_REF_FREQ)
                     && divided_multiplier >= 2 && divided_multiplier <= 32;
    uint32_t expected_multiplier =
        (divide_hse ? divided_multiplier
                    : CONFIG_CLOCK_FREQ / CONFIG_CLOCK_REF_FREQ);
    if (multiplier != expected_multiplier)
        return 34;
    if (!!(cfgr & RCC_CFGR_PLLXTPRE_HSE_DIV2) != divide_hse)
        return 35;
    return 0;
}

static int
run_normal_path(void)
{
    reset_model(-1);
    int jumped = setjmp(reset_jump);
    if (jumped)
        return model_error ? model_error : 40;
    armcm_main();
    if (!scheduler_reached || reset_requested)
        return 41;
    if ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL)
        return 42;
    int plan = check_clock_plan();
    if (plan)
        return plan;
    uint32_t expected_acr = ((CONFIG_CLOCK_FREQ - 1) / 32000000
                             | FLASH_ACR_PRFTBE | N32G45X_FLASH_ICEN);
    uint32_t acr_mask = (FLASH_ACR_LATENCY_Msk | FLASH_ACR_PRFTBE
                         | N32G45X_FLASH_ICRST | N32G45X_FLASH_ICEN);
    if ((FLASH->ACR & acr_mask) != expected_acr)
        return 43;
    if (!(model_pwr_words[3] & 1u))
        return 44;
    return 0;
}

static int
run_stalled_path(int condition)
{
    reset_model(condition);
    int jumped = setjmp(reset_jump);
    if (!jumped) {
        armcm_main();
        return 50 + condition;
    }
    if (jumped != 1 || model_error)
        return model_error ? model_error : 60 + condition;
    if (!reset_requested || scheduler_reached)
        return 70 + condition;
    if (!(SCB->AIRCR & SCB_AIRCR_SYSRESETREQ_Msk))
        return 80 + condition;
    if ((SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) != (3u << 8))
        return 90 + condition;
    if ((SCB->AIRCR >> SCB_AIRCR_VECTKEY_Pos) != 0x5fau)
        return 100 + condition;
    return 0;
}

static int
run_gpio_path(void)
{
    model_gpio_ports[2].CRH = 0xffffffffu;
    gpio_peripheral(GPIO('C', 14), GPIO_OUTPUT, 0);
    return ((model_gpio_ports[2].CRH >> 24) & 0xfu) == 1u ? 0 : 100;
}

int
main(void)
{
    int result = run_normal_path();
    if (result) {
        fprintf(stderr, "normal path failed: %d\n", result);
        return result;
    }
    for (int condition = 0; condition < 6; condition++) {
        result = run_stalled_path(condition);
        if (result) {
            fprintf(stderr, "stalled condition %d failed: %d\n",
                    condition, result);
            return result;
        }
    }
    result = run_gpio_path();
    if (result) {
        fprintf(stderr, "PC14 output configuration failed: %d\n", result);
        return result;
    }
    printf("ref=%u clock=%u model-ok\n",
           (unsigned int)CONFIG_CLOCK_REF_FREQ,
           (unsigned int)CONFIG_CLOCK_FREQ);
    return 0;
}
