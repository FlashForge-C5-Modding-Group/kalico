// Creator 5 eBoard low-level quiesce/rearm register regressions
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define __STM32_INTERNAL_H
#define __SCHED_H
#define DECL_INIT(func)
#define CONFIG_C5_EBOARD 1
#define GPIO(PORT, NUM) (((PORT) - 'A') * 16 + (NUM))
#define GPIO2BIT(PIN) (1u << ((PIN) % 16))
#define GPIO_INPUT 0
#define GPIO_OUTPUT 1
#define GPIO_FUNCTION(fn) (2 | ((fn) << 4))

#define TIM_CR1_CEN 1u
#define TIM_SR_UIF 1u
#define TIM_EGR_UG 1u
#define TIM_SMCR_ECE (1u << 14)
#define TIM_DIER_UDE (1u << 8)
#define TIM_DIER_UIE 1u
#define USART_CR1_IDLEIE (1u << 4)
#define USART_SR_IDLE (1u << 4)
#define USART_SR_TC (1u << 6)

typedef int IRQn_Type;
enum {
    DMA1_Channel2_IRQn = 12,
    DMA1_Channel3_IRQn = 13,
    DMA1_Channel5_IRQn = 15,
    TIM4_IRQn = 30,
    USART3_IRQn = 39,
};

typedef struct {
    volatile uint32_t ISR, IFCR;
} DMA_TypeDef;
typedef struct {
    volatile uint32_t CCR, CNDTR, CPAR, CMAR;
    volatile uint32_t CHSEL;
} DMA_Channel_TypeDef;
typedef struct {
    volatile uint32_t CR1, CR2, SMCR, DIER, SR, EGR;
    volatile uint32_t CCMR1, CCMR2, CCER, CNT, PSC, ARR, RCR;
} TIM_TypeDef;
typedef struct {
    volatile uint32_t SR, DR, BRR, CR1, CR2, CR3;
} USART_TypeDef;
typedef struct { volatile uint32_t BSRR; } GPIO_TypeDef;

static DMA_TypeDef model_dma1;
static DMA_Channel_TypeDef model_dma2, model_dma3, model_dma5;
static TIM_TypeDef model_tim1, model_tim4, model_tim8;
static USART_TypeDef model_usart3;
static GPIO_TypeDef model_gpiob;
static uint32_t model_dma_extension[48];
static uint32_t model_rcc_extension[24];

#define DMA1 (&model_dma1)
#define DMA1_Channel2 (&model_dma2)
#define DMA1_Channel3 (&model_dma3)
#define DMA1_Channel5 (&model_dma5)
#define TIM1 (&model_tim1)
#define TIM4 (&model_tim4)
#define TIM8 (&model_tim8)
#define USART3 (&model_usart3)
#define GPIOB (&model_gpiob)
#define DMA1_BASE ((uintptr_t)&model_dma_extension[0])
#define RCC_BASE ((uintptr_t)&model_rcc_extension[0])
#define TIM1_BASE 0x100u
#define TIM4_BASE 0x104u
#define TIM8_BASE 0x108u
#define USART3_BASE 0x10cu

void gpio_peripheral(uint32_t gpio, uint32_t mode, int pullup);
void enable_pclock(uint32_t base);
void armcm_enable_irq(void (*handler)(void), IRQn_Type irq, uint32_t priority);
void c5_eboard_capture(uint16_t interval);
void c5_eboard_pa_receive(volatile uint8_t *frame, uint16_t remaining);
void c5_eboard_init(void);
static void trace_event(uint32_t event);
#define C5_EBOARD_REGISTER_TRACE(event) trace_event(event)

#include "../src/stm32/c5_eboard.c"

static unsigned receive_count;
static uint16_t received_remaining;
static uint16_t captured_interval;
static unsigned capture_count;
static uint8_t trace_log[32];
static uint8_t trace_count, usart_status_read, usart_clear_violation;

static void
trace_event(uint32_t event)
{
    if (trace_count < sizeof(trace_log))
        trace_log[trace_count++] = event;
    if (event == C5_TRACE_USART_SR_READ) {
        usart_status_read = 1;
    } else if (event == C5_TRACE_USART_DR_READ) {
        if (!usart_status_read)
            usart_clear_violation = 1;
        else
            USART3->SR &= ~USART_SR_IDLE;
        usart_status_read = 0;
    }
}

void gpio_peripheral(uint32_t gpio, uint32_t mode, int pullup)
{
    (void)gpio; (void)mode; (void)pullup;
}
void enable_pclock(uint32_t base) { (void)base; }
void armcm_enable_irq(void (*handler)(void), IRQn_Type irq, uint32_t priority)
{
    (void)handler; (void)irq; (void)priority;
}
void c5_eboard_capture(uint16_t interval)
{
    captured_interval = interval;
    capture_count++;
}
void c5_eboard_pa_receive(volatile uint8_t *frame, uint16_t remaining)
{
    receive_count++;
    received_remaining = remaining;
    for (uint_fast8_t i = 0; i < 15; i++)
        frame[i] = 0;
}
void c5_eboard_init(void) { }

static int
expect_u32(const char *name, uint32_t actual, uint32_t expected)
{
    if (actual == expected)
        return 0;
    fprintf(stderr, "%s: expected 0x%x, got 0x%x\n", name, expected, actual);
    return 1;
}
static int
expect_trace(const char *name, const uint8_t *expected, uint8_t count)
{
    if (trace_count != count) {
        fprintf(stderr, "%s: expected %u events, got %u\n",
                name, count, trace_count);
        return 1;
    }
    for (uint_fast8_t i = 0; i < count; i++) {
        if (trace_log[i] != expected[i]) {
            fprintf(stderr, "%s[%u]: expected %u, got %u\n",
                    name, (unsigned)i, expected[i], trace_log[i]);
            return 1;
        }
    }
    return 0;
}

static void
seed_model(void)
{
    memset(&model_dma1, 0, sizeof(model_dma1));
    memset(&model_dma2, 0, sizeof(model_dma2));
    memset(&model_dma3, 0, sizeof(model_dma3));
    memset(&model_dma5, 0, sizeof(model_dma5));
    memset(&model_tim1, 0, sizeof(model_tim1));
    memset(&model_tim4, 0, sizeof(model_tim4));
    memset(&model_tim8, 0, sizeof(model_tim8));
    memset(&model_usart3, 0, sizeof(model_usart3));
    memset((void *)tmc_response, 0xa5, sizeof(tmc_response));
    model_dma1.ISR = 0xffffffffu;
    model_dma2.CCR = model_dma3.CCR = model_dma5.CCR = 0xffffu;
    model_dma2.CNDTR = 2;
    model_dma3.CNDTR = 7;
    model_dma5.CNDTR = 9;
    model_tim1.CR1 = model_tim4.CR1 = model_tim8.CR1 = TIM_CR1_CEN;
    model_tim1.CNT = 22;
    model_tim4.CNT = 33;
    model_tim8.CNT = 44;
    model_tim4.SR = TIM_SR_UIF;
    model_usart3.CR1 = USART_CR1_IDLEIE;
    model_usart3.SR = USART_SR_IDLE | USART_SR_TC;
    poll_divider = 1;
    dma_snapshot = 0x1234;
    previous_snapshot = 0x5678;
    receive_count = capture_count = 0;
    trace_count = usart_status_read = usart_clear_violation = 0;
}

static int
run_enter_pa(void)
{
    static const uint8_t expected_enter[] = {
        C5_TRACE_PA_TIMER_DISABLE,
        C5_TRACE_TX_DMA_DISABLE,
        C5_TRACE_RX_DMA_DISABLE,
        C5_TRACE_USART_SR_READ,
        C5_TRACE_USART_DR_READ,
        C5_TRACE_NORMAL_DMA_DISABLE,
        C5_TRACE_NORMAL_TIM1_DISABLE,
        C5_TRACE_NORMAL_TIM8_DISABLE,
        C5_TRACE_RX_DMA_ENABLE,
        C5_TRACE_PA_TIMER_ENABLE,
    };
    int failures = 0;
    seed_model();
    c5_eboard_set_pa_mode(1);
    failures += expect_trace("enter quiesce/rearm order", expected_enter,
                             sizeof(expected_enter));
    failures += expect_u32("enter SR-then-DR clears IDLE",
                           USART3->SR & USART_SR_IDLE, 0);
    failures += expect_u32("enter USART clear ordering violation",
                           usart_clear_violation, 0);
    failures += expect_u32("enter stops normal TIM1",
                           TIM1->CR1 & TIM_CR1_CEN, 0);
    failures += expect_u32("enter stops normal TIM8",
                           TIM8->CR1 & TIM_CR1_CEN, 0);
    failures += expect_u32("enter stops normal DMA",
                           DMA1_Channel5->CCR & 1u, 0);
    failures += expect_u32("enter aborts stale TX", DMA1_Channel2->CCR & 1u, 0);
    failures += expect_u32("enter rearms RX", DMA1_Channel3->CCR & 1u, 1);
    failures += expect_u32("enter resets TX window", DMA1_Channel2->CNDTR, 4);
    failures += expect_u32("enter resets RX window", DMA1_Channel3->CNDTR, 15);
    failures += expect_u32("enter resets poll divider", poll_divider, 0);
    failures += expect_u32("enter resets TIM4 count", TIM4->CNT, 0);
    failures += expect_u32("enter clears TIM4 pending",
                           TIM4->SR & TIM_SR_UIF, 0);
    failures += expect_u32("enter starts PA timer last-state",
                           TIM4->CR1 & TIM_CR1_CEN, 1);
    for (uint_fast8_t i = 0; i < sizeof(tmc_response); i++)
        failures += expect_u32("enter clears stale RX bytes",
                               tmc_response[i], 0);

    TIM4->SR = TIM_SR_UIF;
    TIM4_IRQHandler();
    failures += expect_u32("first PA tick does not transmit",
                           DMA1_Channel2->CCR & 1u, 0);
    TIM4->SR = TIM_SR_UIF;
    TIM4_IRQHandler();
    failures += expect_u32("second PA tick starts exact request",
                           DMA1_Channel2->CCR & 1u, 1);
    failures += expect_u32("request length remains four",
                           DMA1_Channel2->CNDTR, 4);
    return failures;
}

static int
run_exit_and_late_irq(void)
{
    static const uint8_t expected_receive[] = {
        C5_TRACE_RX_DMA_DISABLE,
        C5_TRACE_USART_SR_READ,
        C5_TRACE_USART_DR_READ,
        C5_TRACE_RX_DMA_ENABLE,
    };
    static const uint8_t expected_exit[] = {
        C5_TRACE_PA_TIMER_DISABLE,
        C5_TRACE_TX_DMA_DISABLE,
        C5_TRACE_RX_DMA_DISABLE,
        C5_TRACE_USART_SR_READ,
        C5_TRACE_USART_DR_READ,
        C5_TRACE_NORMAL_DMA_DISABLE,
        C5_TRACE_NORMAL_TIM1_DISABLE,
        C5_TRACE_NORMAL_TIM8_DISABLE,
        C5_TRACE_NORMAL_DMA_ENABLE,
        C5_TRACE_NORMAL_TIM8_ENABLE,
        C5_TRACE_NORMAL_TIM1_ENABLE,
    };
    int failures = 0;
    seed_model();
    c5_eboard_set_pa_mode(1);
    trace_count = 0;
    memcpy((void *)tmc_response, "abcdefghijkl", 12);
    DMA1_Channel3->CNDTR = 3;
    USART3->SR |= USART_SR_IDLE;
    USART3_IRQHandler();
    failures += expect_trace("active receive order", expected_receive,
                             sizeof(expected_receive));
    failures += expect_u32("active receive clears IDLE",
                           USART3->SR & USART_SR_IDLE, 0);
    failures += expect_u32("active IDLE forwards frame", receive_count, 1);
    failures += expect_u32("active IDLE forwards remaining count",
                           received_remaining, 3);
    failures += expect_u32("active IDLE rearms RX", DMA1_Channel3->CCR & 1u, 1);

    trace_count = 0;
    USART3->SR |= USART_SR_IDLE;
    c5_eboard_set_pa_mode(0);
    failures += expect_trace("exit quiesce/rearm order", expected_exit,
                             sizeof(expected_exit));
    failures += expect_u32("exit SR-then-DR clears IDLE",
                           USART3->SR & USART_SR_IDLE, 0);
    failures += expect_u32("exit stops PA timer", TIM4->CR1 & TIM_CR1_CEN, 0);
    failures += expect_u32("exit aborts TX", DMA1_Channel2->CCR & 1u, 0);
    failures += expect_u32("exit disables RX", DMA1_Channel3->CCR & 1u, 0);
    failures += expect_u32("exit resets poll divider", poll_divider, 0);
    failures += expect_u32("exit resets normal snapshot", dma_snapshot, 0);
    failures += expect_u32("exit resets previous snapshot",
                           previous_snapshot, 0);
    failures += expect_u32("exit rearms normal DMA count",
                           DMA1_Channel5->CNDTR, 1);
    failures += expect_u32("exit enables normal DMA",
                           DMA1_Channel5->CCR & 1u, 1);
    failures += expect_u32("exit starts TIM8", TIM8->CR1 & TIM_CR1_CEN, 1);
    failures += expect_u32("exit starts TIM1", TIM1->CR1 & TIM_CR1_CEN, 1);
    failures += expect_u32("exit resets TIM8 count", TIM8->CNT, 0);
    failures += expect_u32("exit resets TIM1 count", TIM1->CNT, 0);

    trace_count = 0;
    memcpy((void *)tmc_response, "late-late!!!", 12);
    DMA1_Channel3->CNDTR = 3;
    USART3->SR |= USART_SR_IDLE;
    USART3_IRQHandler();
    failures += expect_u32("late receive trace has no rearm", trace_count, 3);
    failures += expect_u32("late receive clears IDLE",
                           USART3->SR & USART_SR_IDLE, 0);
    failures += expect_u32("late IDLE is not forwarded", receive_count, 1);
    failures += expect_u32("late IDLE cannot reenable RX",
                           DMA1_Channel3->CCR & 1u, 0);
    return failures;
}

static int
run_capture_irq(void)
{
    int failures = 0;
    seed_model();
    DMA1->ISR = N32_DMA1_CH5_TC;
    dma_snapshot = 50000;
    previous_snapshot = 4500;
    DMA1_Channel5_IRQHandler();
    failures += expect_u32("normal IRQ forwards unchanged interval",
                           captured_interval, 45500);
    failures += expect_u32("normal IRQ count", capture_count, 1);
    return failures;
}

int
main(void)
{
    int failures = 0;
    failures += run_enter_pa();
    failures += run_exit_and_late_irq();
    failures += run_capture_irq();
    return failures ? 1 : 0;
}
