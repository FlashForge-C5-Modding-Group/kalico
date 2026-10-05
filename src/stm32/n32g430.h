#ifndef __N32G430_H
#define __N32G430_H

#include <stdint.h>

#define N32G430_GPIO_DRIVE_4MA 0x400

// Cortex-M4 core configuration used by the repository CMSIS core.
#define __CM4_REV                 0x0001U
#define __MPU_PRESENT             1U
#define __NVIC_PRIO_BITS          4U
#define __Vendor_SysTickConfig    0U
#define __FPU_PRESENT             1U

typedef enum {
    SysTick_IRQn = -1,
    DMA1_Channel1_IRQn = 11,
    USART1_IRQn = 36,
} IRQn_Type;

#include "core_cm4.h"

// Reset and clock control: public N32G430 register map.
typedef struct {
    __IO uint32_t CR;             // 0x00
    __IO uint32_t CFGR;           // 0x04
    __IO uint32_t CIR;            // 0x08
    __IO uint32_t APB2RSTR;       // 0x0c
    __IO uint32_t APB1RSTR;       // 0x10
    __IO uint32_t AHBENR;         // 0x14
    __IO uint32_t APB2ENR;        // 0x18
    __IO uint32_t APB1ENR;        // 0x1c
    __IO uint32_t BDCR;           // 0x20
    __IO uint32_t CSR;            // 0x24
    __IO uint32_t AHBRSTR;        // 0x28
    __IO uint32_t CFGR2;          // 0x2c
} RCC_TypeDef;

// Flash access control is the only flash register consumed by this target.
typedef struct {
    __IO uint32_t ACR;            // 0x00
} FLASH_TypeDef;

// Physical operations share IDR, ODR, and BSRR with the STM32 GPIO backend.
typedef struct {
    __IO uint32_t MODER;          // 0x00, two mode bits per pin
    __IO uint32_t OTYPER;         // 0x04, one output-type bit per pin
    __IO uint32_t OSPEEDR;        // 0x08, one slew bit per pin
    __IO uint32_t PUPDR;          // 0x0c, two pull bits per pin
    __I  uint32_t IDR;            // 0x10
    __IO uint32_t ODR;            // 0x14
    __O  uint32_t BSRR;           // 0x18
    __IO uint32_t LCKR;           // 0x1c
    __IO uint32_t AFR[2];         // 0x20, 0x24
    __O  uint32_t BRR;            // 0x28
    __IO uint32_t DSCR;           // 0x2c, two drive bits per pin
} GPIO_TypeDef;

typedef struct {
    __IO uint32_t SR;             // 0x00
    __IO uint32_t DR;             // 0x04
    __IO uint32_t BRR;            // 0x08
    __IO uint32_t CR1;            // 0x0c
    __IO uint32_t CR2;            // 0x10
    __IO uint32_t CR3;            // 0x14
    __IO uint32_t GTPR;           // 0x18
} USART_TypeDef;

typedef struct {
    __O  uint32_t KR;             // 0x00
    __IO uint32_t PR;             // 0x04
    __IO uint32_t RLR;            // 0x08
    __I  uint32_t SR;             // 0x0c
} IWDG_TypeDef;

// Only registers used by the TIM1 time base and TIM8 edge divider are named.
typedef struct {
    __IO uint32_t CR1;            // 0x00
    uint32_t RESERVED0;           // 0x04
    __IO uint32_t SMCR;           // 0x08
    __IO uint32_t DIER;           // 0x0c
    uint32_t RESERVED1;           // 0x10
    __O  uint32_t EGR;            // 0x14
    uint32_t RESERVED2[3];        // 0x18..0x20
    __IO uint32_t CNT;            // 0x24
    __IO uint32_t PSC;            // 0x28
    __IO uint32_t ARR;            // 0x2c
} TIM_TypeDef;

typedef struct {
    __I uint32_t ISR;             // 0x00
    __O uint32_t IFCR;            // 0x04
} DMA_TypeDef;

typedef struct {
    __IO uint32_t CCR;            // controller offset 0x08
    __IO uint32_t CNDTR;          // controller offset 0x0c
    __IO uint32_t CPAR;           // controller offset 0x10
    __IO uint32_t CMAR;           // controller offset 0x14
    __IO uint32_t CHSEL;          // controller offset 0x18
} DMA_Channel_TypeDef;

#define IWDG_BASE              0x40003000UL
#define TIM1_BASE              0x40012C00UL
#define TIM8_BASE              0x40013400UL
#define USART1_BASE            0x40013800UL
#define DMA1_BASE              0x40020000UL
#define RCC_BASE               0x40021000UL
#define FLASH_R_BASE           0x40022000UL
#define GPIOA_BASE             0x40023400UL
#define GPIOB_BASE             0x40023800UL
#define GPIOC_BASE             0x40023C00UL
#define GPIOD_BASE             0x40024000UL

#define IWDG                   ((IWDG_TypeDef *)IWDG_BASE)
#define TIM1                   ((TIM_TypeDef *)TIM1_BASE)
#define TIM8                   ((TIM_TypeDef *)TIM8_BASE)
#define USART1                 ((USART_TypeDef *)USART1_BASE)
#define DMA1                   ((DMA_TypeDef *)DMA1_BASE)
#define DMA1_Channel1          ((DMA_Channel_TypeDef *)(DMA1_BASE + 0x08UL))
#define DMA1_CHANNEL_COUNT     6
#define DMA1_Channel(n)        (DMA1_Channel1 + (n))
#define RCC                    ((RCC_TypeDef *)RCC_BASE)
#define FLASH                  ((FLASH_TypeDef *)FLASH_R_BASE)
#define GPIOA                  ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB                  ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC                  ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD                  ((GPIO_TypeDef *)GPIOD_BASE)

#define RCC_CR_HSION           (1U << 0)
#define RCC_CR_HSIRDY          (1U << 1)
#define RCC_CR_HSEON           (1U << 16)
#define RCC_CR_HSERDY          (1U << 17)
#define RCC_CR_HSEBYP          (1U << 18)
#define RCC_CR_PLLON           (1U << 24)
#define RCC_CR_PLLRDY          (1U << 25)
#define RCC_CFGR_SW_Msk        (3U << 0)
#define RCC_CFGR_SW_HSI        (0U << 0)
#define RCC_CFGR_SW_PLL        (2U << 0)
#define RCC_CFGR_SWS_Msk       (3U << 2)
#define RCC_CFGR_SWS_HSI       (0U << 2)
#define RCC_CFGR_SWS_PLL       (2U << 2)
#define RCC_CFGR_HPRE_Msk      (15U << 4)
#define RCC_CFGR_PPRE1_Msk     (7U << 8)
#define RCC_CFGR_PPRE1_DIV4    (5U << 8)
#define RCC_CFGR_PPRE2_Msk     (7U << 11)
#define RCC_CFGR_PPRE2_DIV2    (4U << 11)
#define RCC_CFGR_PLLSRC_HSE    (1U << 16)
#define RCC_CFGR_PLLXTPRE_HSE_DIV2 (1U << 17)
#define RCC_CFGR_PLLMUL_Msk    ((15U << 18) | (1U << 27))
#define RCC_CFGR_PLLMUL32      RCC_CFGR_PLLMUL_Msk
#define RCC_CFGR2_TIM1_8_SEL   (1U << 29)

#define RCC_AHBENR_DMA1EN      (1U << 0)
#define RCC_AHBENR_GPIOAEN     (1U << 7)
#define RCC_AHBENR_GPIOBEN     (1U << 8)
#define RCC_AHBENR_GPIOCEN     (1U << 9)
#define RCC_AHBENR_GPIODEN     (1U << 10)
#define RCC_APB2ENR_AFIOEN     (1U << 0)
#define RCC_APB2ENR_TIM1EN     (1U << 11)
#define RCC_APB2ENR_TIM8EN     (1U << 13)
#define RCC_APB2ENR_USART1EN   (1U << 14)

#define FLASH_ACR_LATENCY_Msk  (7U << 0)
#define FLASH_ACR_LATENCY_3    (3U << 0)
#define FLASH_ACR_PRFTEN       (1U << 4)
#define FLASH_ACR_ICRST        (1U << 6)
#define FLASH_ACR_ICEN         (1U << 7)

#define USART_SR_ORE           (1U << 3)
#define USART_SR_RXNE          (1U << 5)
#define USART_SR_TXE           (1U << 7)
#define USART_BRR_DIV_Fraction_Pos 0U
#define USART_BRR_DIV_Mantissa_Pos 4U
#define USART_CR1_RE           (1U << 2)
#define USART_CR1_TE           (1U << 3)
#define USART_CR1_RXNEIE       (1U << 5)
#define USART_CR1_TXEIE        (1U << 7)
#define USART_CR1_UE           (1U << 13)

#define TIM_CR1_CEN            (1U << 0)
#define TIM_SMCR_ECE           (1U << 14)
#define TIM_DIER_UDE           (1U << 8)
#define TIM_EGR_UG             (1U << 0)

#define DMA_ISR_TCIF1          (1U << 1)
#define DMA_IFCR_CTCIF1        (1U << 1)
#define DMA_IFCR_CHANNEL1_ALL  0x0FU
#define DMA_CCR_EN             (1U << 0)
#define DMA_CCR_TCIE           (1U << 1)
#define DMA_CCR_CIRC           (1U << 5)
#define DMA_CCR_PSIZE_16       (1U << 8)
#define DMA_CCR_MSIZE_16       (1U << 10)
#define DMA_CCR_PL_HIGH        (2U << 12)
#define DMA_CHSEL_REQUEST_Msk  0x3FU

#endif // __N32G430_H
