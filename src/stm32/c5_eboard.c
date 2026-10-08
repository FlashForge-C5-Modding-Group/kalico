// Creator 5 eBoard private acquisition and TMC transport
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h> // uintptr_t
#include "autoconf.h" // CONFIG_C5_EBOARD
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/irq.h" // irq_save
#include "board/misc.h" // timer_read_time
#include "c5_eboard.h" // c5_eboard_capture
#include "command.h" // DECL_COMMAND
#include "internal.h" // enable_pclock
#include "sched.h" // DECL_INIT


#define N32_RCC_CFG2_SYSCLK_TIM18 0x20000000u
#define N32_DMA1_CH5_TC 0x00020000u
#define N32_DMA1_CH5_ALL 0x000f0000u
#define N32_DMA1_CH2_TC 0x00000020u
#define N32_DMA1_CH2_ALL 0x000000f0u
#define N32_DMA1_CH3_TC 0x00000200u
#define N32_DMA1_CH3_ALL 0x00000f00u
#define C5_TRACE_USART_SR_READ 1u
#define C5_TRACE_USART_DR_READ 2u
#define C5_TRACE_PA_TIMER_DISABLE 3u
#define C5_TRACE_TX_DMA_DISABLE 4u
#define C5_TRACE_RX_DMA_DISABLE 5u
#define C5_TRACE_NORMAL_DMA_DISABLE 6u
#define C5_TRACE_NORMAL_TIM1_DISABLE 7u
#define C5_TRACE_NORMAL_TIM8_DISABLE 8u
#define C5_TRACE_RX_DMA_ENABLE 9u
#define C5_TRACE_PA_TIMER_ENABLE 10u
#define C5_TRACE_NORMAL_DMA_ENABLE 11u
#define C5_TRACE_NORMAL_TIM8_ENABLE 12u
#define C5_TRACE_NORMAL_TIM1_ENABLE 13u
#define C5_TRACE_TX_DMA_ENABLE 14u

#ifndef C5_EBOARD_REGISTER_TRACE
#define C5_EBOARD_REGISTER_TRACE(event) ((void)0)
#endif

static volatile uint32_t dma_snapshot;
static uint32_t previous_snapshot, poll_divider;
static volatile uint8_t pa_mode_active;
static uint8_t tmc_request[4] = { 0x05, 0x00, 0x41, 0xcf };
static uint8_t tmc_transfer[8];
static volatile uint8_t tmc_response[15];

static volatile uint32_t *
n32_dma_channel_selector(DMA_Channel_TypeDef *channel)
{
    return (volatile uint32_t *)((uintptr_t)channel + 0x10);
}

static volatile uint32_t *
n32_dma1_channel_map_enable(void)
{
    return (volatile uint32_t *)(DMA1_BASE + 0xa8);
}

static volatile uint32_t *
n32_rcc_cfg2(void)
{
    return (volatile uint32_t *)(RCC_BASE + 0x2c);
}

static void
c5_clear_tmc_response(void)
{
    for (uint_fast8_t i = 0; i < sizeof(tmc_response); i++)
        tmc_response[i] = 0;
}

static void
c5_clear_usart_idle(void)
{
    uint32_t status = USART3->SR;
    C5_EBOARD_REGISTER_TRACE(C5_TRACE_USART_SR_READ);
    uint32_t data = USART3->DR;
    C5_EBOARD_REGISTER_TRACE(C5_TRACE_USART_DR_READ);
    (void)status;
    (void)data;
}

void
DMA1_Channel5_IRQHandler(void)
{
    if (!(DMA1->ISR & N32_DMA1_CH5_TC))
        return;
    DMA1->IFCR = N32_DMA1_CH5_ALL;
    uint32_t snapshot = dma_snapshot;
    uint16_t interval = snapshot - previous_snapshot;
    previous_snapshot = snapshot;
    c5_eboard_capture(interval);
}

void
DMA1_Channel2_IRQHandler(void)
{
    if (!(DMA1->ISR & N32_DMA1_CH2_TC))
        return;
    DMA1->IFCR = N32_DMA1_CH2_ALL;
    DMA1_Channel2->CCR &= ~1u;
}

void
DMA1_Channel3_IRQHandler(void)
{
    if (DMA1->ISR & N32_DMA1_CH3_TC)
        DMA1->IFCR = N32_DMA1_CH3_ALL;
}

void
TIM4_IRQHandler(void)
{
    if (!(TIM4->SR & TIM_SR_UIF))
        return;
    TIM4->SR = 0;
    if (!pa_mode_active || ++poll_divider != 2)
        return;
    poll_divider = 0;
    DMA1_Channel2->CCR &= ~1u;
    C5_EBOARD_REGISTER_TRACE(C5_TRACE_TX_DMA_DISABLE);
    DMA1->IFCR = N32_DMA1_CH2_ALL;
    DMA1_Channel2->CNDTR = sizeof(tmc_request);
    DMA1_Channel2->CCR |= 1u;
    C5_EBOARD_REGISTER_TRACE(C5_TRACE_TX_DMA_ENABLE);
}

void
USART3_IRQHandler(void)
{
    uint32_t sr = USART3->SR;
    if (!(USART3->CR1 & USART_CR1_IDLEIE) || !(sr & USART_SR_IDLE))
        return;
    DMA1_Channel3->CCR &= ~1u;
    C5_EBOARD_REGISTER_TRACE(C5_TRACE_RX_DMA_DISABLE);
    uint16_t remaining = DMA1_Channel3->CNDTR;
    c5_clear_usart_idle();
    if (pa_mode_active)
        c5_eboard_pa_receive(tmc_response, remaining);
    else
        c5_clear_tmc_response();
    DMA1->IFCR = N32_DMA1_CH3_ALL;
    DMA1_Channel3->CNDTR = sizeof(tmc_response);
    if (pa_mode_active) {
        DMA1_Channel3->CCR |= 1u;
        C5_EBOARD_REGISTER_TRACE(C5_TRACE_RX_DMA_ENABLE);
    }
}

void
c5_eboard_set_pa_mode(uint8_t active)
{
    pa_mode_active = 0;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    C5_EBOARD_REGISTER_TRACE(C5_TRACE_PA_TIMER_DISABLE);
    TIM4->CNT = 0;
    TIM4->SR = 0;
    poll_divider = 0;

    DMA1_Channel2->CCR &= ~1u;
    C5_EBOARD_REGISTER_TRACE(C5_TRACE_TX_DMA_DISABLE);
    DMA1_Channel3->CCR &= ~1u;
    C5_EBOARD_REGISTER_TRACE(C5_TRACE_RX_DMA_DISABLE);
    DMA1_Channel2->CNDTR = sizeof(tmc_request);
    DMA1_Channel3->CNDTR = sizeof(tmc_response);
    DMA1->IFCR = N32_DMA1_CH2_ALL | N32_DMA1_CH3_ALL;
    c5_clear_tmc_response();
    c5_clear_usart_idle();

    DMA1_Channel5->CCR &= ~1u;
    C5_EBOARD_REGISTER_TRACE(C5_TRACE_NORMAL_DMA_DISABLE);
    TIM1->CR1 &= ~TIM_CR1_CEN;
    C5_EBOARD_REGISTER_TRACE(C5_TRACE_NORMAL_TIM1_DISABLE);
    TIM8->CR1 &= ~TIM_CR1_CEN;
    C5_EBOARD_REGISTER_TRACE(C5_TRACE_NORMAL_TIM8_DISABLE);
    DMA1->IFCR = N32_DMA1_CH5_ALL;
    DMA1_Channel5->CNDTR = 1;

    if (active) {
        DMA1_Channel3->CCR |= 1u;
        C5_EBOARD_REGISTER_TRACE(C5_TRACE_RX_DMA_ENABLE);
        pa_mode_active = 1;
        TIM4->EGR = TIM_EGR_UG;
        TIM4->SR = 0;
        TIM4->CR1 |= TIM_CR1_CEN;
        C5_EBOARD_REGISTER_TRACE(C5_TRACE_PA_TIMER_ENABLE);
        return;
    }

    dma_snapshot = 0;
    previous_snapshot = 0;
    TIM8->CNT = 0;
    TIM8->EGR = TIM_EGR_UG;
    TIM8->SR = 0;
    TIM1->CNT = 0;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR = 0;
    DMA1_Channel5->CCR |= 1u;
    C5_EBOARD_REGISTER_TRACE(C5_TRACE_NORMAL_DMA_ENABLE);
    TIM8->CR1 |= TIM_CR1_CEN;
    C5_EBOARD_REGISTER_TRACE(C5_TRACE_NORMAL_TIM8_ENABLE);
    TIM1->CR1 |= TIM_CR1_CEN;
    C5_EBOARD_REGISTER_TRACE(C5_TRACE_NORMAL_TIM1_ENABLE);
}

// Host-controlled TMC2209 access uses the eBoard's dedicated USART3 and
// DMA receive path. GPIO bitbanging PB10/PB11 would contend with pressure-
// advance sampling, so only this command may transfer on that UART.
void
command_c5_tmc_uart_send(uint32_t *args)
{
    uint8_t bus = args[0], write_len = args[1], read_len = args[3];
    uint8_t *write = command_decode_ptr(args[2]);
    uint8_t reply[15], reply_len = 0;
    if (bus || !((write_len == 4 && read_len == 8)
                 || (write_len == 8 && read_len == 0))) {
        sendf("c5_tmc_uart_response bus=%c read=%*s", bus, 0, reply);
        return;
    }

    irqstatus_t flag = irq_save();
    uint8_t resume_pa = pa_mode_active;
    pa_mode_active = 0;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    DMA1_Channel2->CCR &= ~1u;
    DMA1_Channel3->CCR &= ~1u;
    USART3->CR1 &= ~USART_CR1_IDLEIE;
    DMA1->IFCR = N32_DMA1_CH2_ALL | N32_DMA1_CH3_ALL;
    c5_clear_usart_idle();
    c5_clear_tmc_response();
    DMA1_Channel3->CNDTR = sizeof(tmc_response);
    if (read_len)
        DMA1_Channel3->CCR |= 1u;
    for (uint_fast8_t i = 0; i < write_len; i++)
        tmc_transfer[i] = write[i];
    DMA1_Channel2->CMAR = (uint32_t)(uintptr_t)tmc_transfer;
    DMA1_Channel2->CNDTR = write_len;
    DMA1_Channel2->CCR |= 1u;
    irq_restore(flag);

    uint32_t deadline = timer_read_time() + timer_from_us(3000);
    // Use the same DMA transmitter as the stock eBoard PA query.  USART3
    // remains at its stock baud rate and the complete datagram is contiguous.
    while (DMA1_Channel2->CCR & 1u)
        if (!timer_is_before(timer_read_time(), deadline))
            goto done;
    while (!(USART3->SR & USART_SR_TC))
        if (!timer_is_before(timer_read_time(), deadline))
            goto done;
    if (read_len) {
        // Capture the whole DMA frame, including the request echo.  Klippy
        // validates the echo and CRC and can report a partial frame.
        while (DMA1_Channel3->CNDTR > 3)
            if (!timer_is_before(timer_read_time(), deadline))
                goto done;
        // A stale leading byte can precede the request echo.  Once twelve
        // bytes have arrived, allow the final CRC byte to reach DMA too.
        uint32_t settle_deadline = (timer_read_time()
                                    + timer_from_us(300));
        while (timer_is_before(timer_read_time(), settle_deadline))
            if (!DMA1_Channel3->CNDTR)
                break;
    }

done:
    flag = irq_save();
    if (read_len) {
        reply_len = sizeof(tmc_response) - DMA1_Channel3->CNDTR;
        for (uint_fast8_t i = 0; i < reply_len; i++)
            reply[i] = tmc_response[i];
    }
    DMA1_Channel2->CCR &= ~1u;
    DMA1_Channel3->CCR &= ~1u;
    DMA1->IFCR = N32_DMA1_CH2_ALL | N32_DMA1_CH3_ALL;
    c5_clear_usart_idle();
    c5_clear_tmc_response();
    DMA1_Channel2->CMAR = (uint32_t)(uintptr_t)tmc_request;
    DMA1_Channel2->CNDTR = sizeof(tmc_request);
    DMA1_Channel3->CNDTR = sizeof(tmc_response);
    if (resume_pa) {
        DMA1_Channel3->CCR |= 1u;
        pa_mode_active = 1;
        TIM4->SR = 0;
        TIM4->CR1 |= TIM_CR1_CEN;
    }
    USART3->CR1 |= USART_CR1_IDLEIE;
    irq_restore(flag);
    sendf("c5_tmc_uart_response bus=%c read=%*s", bus, reply_len, reply);
}
DECL_COMMAND(command_c5_tmc_uart_send,
             "c5_tmc_uart_send bus=%c write=%*s read=%c");
DECL_CONSTANT("C5_EBOARD_TMC_UART", 1);

void
c5_eboard_hardware_init(void)
{
    gpio_peripheral(GPIO('B', 8), GPIO_OUTPUT, 0);
    GPIOB->BSRR = GPIO2BIT(GPIO('B', 8));

    *n32_rcc_cfg2() |= N32_RCC_CFG2_SYSCLK_TIM18;
    enable_pclock(TIM8_BASE);
    TIM8->PSC = 0;
    TIM8->ARR = 0xffff;
    TIM8->CNT = 0;
    TIM8->EGR = TIM_EGR_UG;
    TIM8->CR1 = TIM_CR1_CEN;

    gpio_peripheral(GPIO('A', 12), GPIO_INPUT, 0);
    enable_pclock(TIM1_BASE);
    TIM1->PSC = 0;
    TIM1->ARR = 499;
    TIM1->CNT = 0;
    TIM1->RCR = 0;
    TIM1->SMCR = TIM_SMCR_ECE;
    TIM1->DIER = TIM_DIER_UDE;

    enable_pclock(DMA1_BASE);
    *n32_dma1_channel_map_enable() = 1;
    DMA1_Channel5->CCR = 0;
    DMA1_Channel5->CPAR = (uint32_t)(uintptr_t)&TIM8->CNT;
    DMA1_Channel5->CMAR = (uint32_t)(uintptr_t)&dma_snapshot;
    DMA1_Channel5->CNDTR = 1;
    *n32_dma_channel_selector(DMA1_Channel5) = 0x18;
    DMA1_Channel5->CCR = 0x2920;
    DMA1->IFCR = N32_DMA1_CH5_ALL;
    armcm_enable_irq(DMA1_Channel5_IRQHandler, DMA1_Channel5_IRQn, 2);
    DMA1_Channel5->CCR = 0x2922;
    DMA1_Channel5->CCR = 0x2923;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->CR1 = TIM_CR1_CEN;

    gpio_peripheral(GPIO('B', 10), GPIO_FUNCTION(0), 0);
    gpio_peripheral(GPIO('B', 11), GPIO_INPUT, 0);
    enable_pclock(USART3_BASE);
    USART3->BRR = 156;
    USART3->CR2 = 0;
    USART3->CR3 = 0x00c0;
    USART3->CR1 = 0x201c;

    DMA1_Channel2->CCR = 0;
    DMA1_Channel2->CPAR = (uint32_t)(uintptr_t)&USART3->DR;
    DMA1_Channel2->CMAR = (uint32_t)(uintptr_t)tmc_request;
    DMA1_Channel2->CNDTR = sizeof(tmc_request);
    *n32_dma_channel_selector(DMA1_Channel2) = 5;
    DMA1_Channel2->CCR = 0x3092;

    DMA1_Channel3->CCR = 0;
    DMA1_Channel3->CPAR = (uint32_t)(uintptr_t)&USART3->DR;
    DMA1_Channel3->CMAR = (uint32_t)(uintptr_t)tmc_response;
    DMA1_Channel3->CNDTR = sizeof(tmc_response);
    *n32_dma_channel_selector(DMA1_Channel3) = 11;
    DMA1_Channel3->CCR = 0x2082;

    enable_pclock(TIM4_BASE);
    TIM4->PSC = 719;
    TIM4->ARR = 99;
    TIM4->CNT = 0;
    TIM4->EGR = TIM_EGR_UG;
    TIM4->DIER = TIM_DIER_UIE;
    TIM4->CR1 = 0;

    armcm_enable_irq(DMA1_Channel2_IRQHandler, DMA1_Channel2_IRQn, 14);
    armcm_enable_irq(DMA1_Channel3_IRQHandler, DMA1_Channel3_IRQn, 14);
    armcm_enable_irq(TIM4_IRQHandler, TIM4_IRQn, 0);
    armcm_enable_irq(USART3_IRQHandler, USART3_IRQn, 14);

    c5_eboard_init();
}
DECL_INIT(c5_eboard_hardware_init);
