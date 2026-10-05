#ifndef __N32G430_REGISTER_MODEL_H
#define __N32G430_REGISTER_MODEL_H

#include <stddef.h>
#include <stdint.h>

#define __STM32_INTERNAL_H
#define __SCHED_H
#define CONFIG_CLOCK_FREQ 128000000u
#define CONFIG_MACH_N32G430 1
#define DECL_INIT(func)
#define noinline __attribute__((noinline))

void n32g430_model_poll(volatile uint32_t *reg, uint32_t mask,
                        uint32_t expected);
void n32g430_model_dma_channel_write(volatile uint32_t *reg, uint32_t value);
#define N32G430_WAIT_POLL(reg, mask, expected) \
    n32g430_model_poll((reg), (mask), (expected))
#define N32G430_DMA_CHANNEL_WRITE(reg, value) \
    n32g430_model_dma_channel_write(&DMA1_Channel1->reg, (value))

void model_dsb(void);
void model_isb(void);
#define __DSB() model_dsb()
#define __ISB() model_isb()

typedef unsigned int irqstatus_t;
typedef int IRQn_Type;

typedef struct {
    volatile uint32_t CR, CFGR, CIR, APB2RSTR, APB1RSTR;
    volatile uint32_t AHBENR, APB2ENR, APB1ENR, BDCR, CSR, AHBRSTR, CFGR2;
} RCC_TypeDef;
typedef struct { volatile uint32_t ACR; } FLASH_TypeDef;
typedef struct { volatile uint32_t VTOR, AIRCR; } SCB_Type;
typedef struct { volatile uint32_t DEMCR; } CoreDebug_Type;
typedef struct { volatile uint32_t CTRL, CYCCNT; } DWT_Type;
#define CoreDebug_DEMCR_TRCENA_Msk (1u << 24)
typedef struct {
    volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR;
    volatile uint32_t BSRR, LCKR, AFR[2], BRR, DSCR;
} GPIO_TypeDef;
typedef struct {
    volatile uint32_t CR1, RESERVED0, SMCR, DIER, RESERVED1, EGR;
    volatile uint32_t RESERVED2[3], CNT, PSC, ARR;
} TIM_TypeDef;
typedef struct { volatile uint32_t ISR, IFCR; } DMA_TypeDef;
typedef struct {
    volatile uint32_t CCR, CNDTR, CPAR, CMAR, CHSEL;
} DMA_Channel_TypeDef;
typedef struct {
    volatile uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR;
} USART_TypeDef;

extern RCC_TypeDef model_rcc;
extern FLASH_TypeDef model_flash;
extern SCB_Type model_scb;
extern CoreDebug_Type model_core_debug;
extern DWT_Type model_dwt;
extern GPIO_TypeDef model_gpio_ports[4];
extern TIM_TypeDef model_tim1, model_tim8;
extern DMA_TypeDef model_dma1;
#define DMA1_CHANNEL_COUNT 6
extern DMA_Channel_TypeDef model_dma1_channels[DMA1_CHANNEL_COUNT];
extern USART_TypeDef model_usart1;
extern uint32_t VectorTable[];

#define RCC (&model_rcc)
#define FLASH (&model_flash)
#define SCB (&model_scb)
#define CoreDebug (&model_core_debug)
#define DWT (&model_dwt)
#define GPIOA (&model_gpio_ports[0])
#define GPIOB (&model_gpio_ports[1])
#define GPIOC (&model_gpio_ports[2])
#define GPIOD (&model_gpio_ports[3])
#define TIM1 (&model_tim1)
#define TIM8 (&model_tim8)
#define DMA1 (&model_dma1)
#define DMA1_Channel1 (&model_dma1_channels[0])
#define DMA1_Channel(n) (DMA1_Channel1 + (n))
#define USART1 (&model_usart1)

#define IWDG_BASE 0x40003000u
#define TIM1_BASE 0x40012c00u
#define TIM8_BASE 0x40013400u
#define USART1_BASE 0x40013800u
#define DMA1_BASE 0x40020000u
#define GPIOA_BASE 0x40023400u
#define GPIOB_BASE 0x40023800u
#define GPIOC_BASE 0x40023c00u
#define GPIOD_BASE 0x40024000u

#define RCC_CR_HSION (1u << 0)
#define RCC_CR_HSIRDY (1u << 1)
#define RCC_CR_HSEON (1u << 16)
#define RCC_CR_HSERDY (1u << 17)
#define RCC_CR_HSEBYP (1u << 18)
#define RCC_CR_PLLON (1u << 24)
#define RCC_CR_PLLRDY (1u << 25)
#define RCC_CFGR_SW_Msk (3u << 0)
#define RCC_CFGR_SW_HSI (0u << 0)
#define RCC_CFGR_SW_PLL (2u << 0)
#define RCC_CFGR_SWS_Msk (3u << 2)
#define RCC_CFGR_SWS_HSI (0u << 2)
#define RCC_CFGR_SWS_PLL (2u << 2)
#define RCC_CFGR_HPRE_Msk (15u << 4)
#define RCC_CFGR_PPRE1_Msk (7u << 8)
#define RCC_CFGR_PPRE1_DIV4 (5u << 8)
#define RCC_CFGR_PPRE2_Msk (7u << 11)
#define RCC_CFGR_PPRE2_DIV2 (4u << 11)
#define RCC_CFGR_PLLSRC_HSE (1u << 16)
#define RCC_CFGR_PLLXTPRE_HSE_DIV2 (1u << 17)
#define RCC_CFGR_PLLMUL_Msk ((15u << 18) | (1u << 27))
#define RCC_CFGR_PLLMUL32 RCC_CFGR_PLLMUL_Msk
#define RCC_CFGR2_TIM1_8_SEL (1u << 29)

#define RCC_AHBENR_DMA1EN (1u << 0)
#define RCC_AHBENR_GPIOAEN (1u << 7)
#define RCC_AHBENR_GPIOBEN (1u << 8)
#define RCC_AHBENR_GPIOCEN (1u << 9)
#define RCC_AHBENR_GPIODEN (1u << 10)
#define RCC_APB2ENR_TIM1EN (1u << 11)
#define RCC_APB2ENR_TIM8EN (1u << 13)
#define RCC_APB2ENR_USART1EN (1u << 14)

#define FLASH_ACR_LATENCY_Msk (7u << 0)
#define FLASH_ACR_LATENCY_3 (3u << 0)
#define FLASH_ACR_PRFTEN (1u << 4)
#define FLASH_ACR_ICRST (1u << 6)
#define FLASH_ACR_ICEN (1u << 7)

#define GPIO(PORT, NUM) (((PORT) - 'A') * 16 + (NUM))
#define GPIO2BIT(PIN) (1u << ((PIN) % 16))
#define GPIO_OUTPUT 1u
#define GPIO_HIGH_SPEED 0x200u
#define GPIO_FUNCTION(fn) (2u | ((fn) << 4))
#define N32G430_GPIO_DRIVE_4MA 0x400u

#define TIM_CR1_CEN (1u << 0)
#define TIM_SMCR_ECE (1u << 14)
#define TIM_DIER_UDE (1u << 8)
#define TIM_EGR_UG (1u << 0)

#define DMA_ISR_TCIF1 (1u << 1)
#define DMA_IFCR_CGIF1 (1u << 0)
#define DMA_IFCR_CTCIF1 (1u << 1)
#define DMA_IFCR_CHTIF1 (1u << 2)
#define DMA_IFCR_CTEIF1 (1u << 3)
#define DMA_IFCR_CHANNEL1_ALL 0x0fu
#define DMA_CCR_EN (1u << 0)
#define DMA_CCR_TCIE (1u << 1)
#define DMA_CCR_CIRC (1u << 5)
#define DMA_CCR_PSIZE_16 (1u << 8)
#define DMA_CCR_MSIZE_16 (1u << 10)
#define DMA_CCR_PL_HIGH (2u << 12)
#define DMA_CHSEL_REQUEST_Msk 0x3fu
#define DMA1_Channel1_IRQn 11

struct cline { volatile uint32_t *en, *rst; uint32_t bit; };
struct cline lookup_clock_line(uint32_t periph_base);
void gpio_clock_enable(GPIO_TypeDef *regs);
void gpio_peripheral(uint32_t gpio, uint32_t mode, int pullup);
void enable_pclock(uint32_t periph_base);
irqstatus_t irq_save(void);
void irq_restore(irqstatus_t flag);
void armcm_enable_irq(void (*func)(void), IRQn_Type irq, uint32_t priority);
void c5_levelboard_capture(uint16_t delta);
void sched_main(void);

#endif
