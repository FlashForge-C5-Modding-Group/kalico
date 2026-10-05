// Creator 5 levelBoard hardware acquisition
//
// Copyright (C) 2026
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/armcm_boot.h" // armcm_enable_irq
#include "c5_levelboard.h" // c5_levelboard_capture
#include "internal.h" // enable_pclock
#include "sched.h" // DECL_INIT
#ifndef N32G430_DMA_CHANNEL_WRITE
#define N32G430_DMA_CHANNEL_WRITE(reg, value) \
    do { DMA1_Channel1->reg = (value); } while (0)
#endif

static volatile uint16_t dma_snapshot;
static uint16_t previous_snapshot;

void
DMA1_Channel1_IRQHandler(void)
{
    if (!(DMA1->ISR & DMA_ISR_TCIF1))
        return;
    DMA1->IFCR = DMA_IFCR_CTCIF1;

    uint16_t snapshot = dma_snapshot;
    uint16_t delta = (uint16_t)(snapshot - previous_snapshot);
    previous_snapshot = snapshot;
    c5_levelboard_capture(delta);
}

void
c5_levelboard_acquisition_init(void)
{
    gpio_clock_enable(GPIOA);
    GPIOA->BSRR = GPIO2BIT(GPIO('A', 1));
    gpio_peripheral(GPIO('A', 1),
                    GPIO_OUTPUT | GPIO_HIGH_SPEED
                    | N32G430_GPIO_DRIVE_4MA, 0);

    gpio_peripheral(GPIO('A', 0),
                    GPIO_FUNCTION(8) | GPIO_HIGH_SPEED
                    | N32G430_GPIO_DRIVE_4MA, 0);
    enable_pclock(TIM8_BASE);
    TIM8->PSC = 0;
    TIM8->ARR = 500;
    TIM8->SMCR = TIM_SMCR_ECE;
    TIM8->DIER = TIM_DIER_UDE;
    TIM8->CR1 = TIM_CR1_CEN;

    enable_pclock(TIM1_BASE);
    TIM1->PSC = 0;
    TIM1->ARR = 0xffff;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->CR1 = TIM_CR1_CEN;

    enable_pclock(DMA1_BASE);
    N32G430_DMA_CHANNEL_WRITE(CCR, DMA1_Channel1->CCR & ~DMA_CCR_EN);
    N32G430_DMA_CHANNEL_WRITE(CCR, 0);
    DMA1->IFCR = DMA_IFCR_CHANNEL1_ALL;
    N32G430_DMA_CHANNEL_WRITE(CPAR, (uint32_t)(uintptr_t)&TIM1->CNT);
    N32G430_DMA_CHANNEL_WRITE(CMAR, (uint32_t)(uintptr_t)&dma_snapshot);
    N32G430_DMA_CHANNEL_WRITE(CNDTR, 1);
    N32G430_DMA_CHANNEL_WRITE(CHSEL, 0x32 & DMA_CHSEL_REQUEST_Msk);
    N32G430_DMA_CHANNEL_WRITE(CCR, DMA_CCR_CIRC | DMA_CCR_PSIZE_16
                             | DMA_CCR_MSIZE_16 | DMA_CCR_PL_HIGH);
    armcm_enable_irq(DMA1_Channel1_IRQHandler, DMA1_Channel1_IRQn, 0);
    N32G430_DMA_CHANNEL_WRITE(CCR, DMA1_Channel1->CCR | DMA_CCR_TCIE);
    N32G430_DMA_CHANNEL_WRITE(CCR, DMA1_Channel1->CCR | DMA_CCR_EN);
}
DECL_INIT(c5_levelboard_acquisition_init);
