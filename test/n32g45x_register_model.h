#ifndef __N32G45X_REGISTER_MODEL_H
#define __N32G45X_REGISTER_MODEL_H

#include <stddef.h>
#include <stdint.h>

#define CONFIG_MACH_N32G45x 1
#ifndef CONFIG_CLOCK_FREQ
#define CONFIG_CLOCK_FREQ 144000000
#endif
#define CONFIG_STM32_CLOCK_REF_INTERNAL 0
#define CONFIG_USB 0
#define CONFIG_STM32F103GD_DISABLE_SWD 0
#define CONFIG_STM32_FLASH_START_800 0
#define CONFIG_STM32_FLASH_START_2000 0

#if defined(_MSC_VER)
#define noinline __declspec(noinline)
#else
#define noinline __attribute__((noinline))
#endif
#define __noreturn

#define N32G45X_CLOCK_TIMEOUT 8u
void n32g45x_model_poll(volatile uint32_t *reg, uint32_t mask,
                        uint32_t expected);
void n32g45x_model_reset_requested(void);
#define N32G45X_WAIT_POLL(reg, mask, expected) \
    n32g45x_model_poll((reg), (mask), (expected))
#define N32G45X_RESET_REQUESTED() n32g45x_model_reset_requested()

void model_disable_irq(void);
void model_dsb(void);
void model_nop(void);
#define __disable_irq() model_disable_irq()
#define __DSB() model_dsb()
#define __NOP() model_nop()

typedef struct {
    volatile uint32_t CRL;
    volatile uint32_t CRH;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    uint8_t reserved[0x400 - 5 * sizeof(uint32_t)];
} GPIO_TypeDef;

typedef struct {
    volatile uint32_t CR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t APB1RSTR;
    volatile uint32_t AHBENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t APB1ENR;
} RCC_TypeDef;

typedef struct { volatile uint32_t ACR; } FLASH_TypeDef;
typedef struct { volatile uint32_t MAPR; } AFIO_TypeDef;
typedef struct { volatile uint32_t CR; } PWR_TypeDef;
typedef struct {
    volatile uint32_t reserved0[4];
    volatile uint32_t DR4;
    volatile uint32_t reserved1[6];
    volatile uint32_t DR10;
} BKP_TypeDef;
typedef struct {
    volatile uint32_t AIRCR;
    volatile uint32_t VTOR;
} SCB_Type;

extern RCC_TypeDef model_rcc;
extern FLASH_TypeDef model_flash;
extern AFIO_TypeDef model_afio;
extern uint32_t model_pwr_words[4];
extern BKP_TypeDef model_bkp;
extern SCB_Type model_scb;
extern GPIO_TypeDef model_gpio_ports[8];
extern uint32_t VectorTable[];

#define RCC (&model_rcc)
#define FLASH (&model_flash)
#define AFIO (&model_afio)
#define PWR ((PWR_TypeDef *)&model_pwr_words[0])
#define BKP (&model_bkp)
#define SCB (&model_scb)
#define GPIOA (&model_gpio_ports[0])
#define GPIOB (&model_gpio_ports[1])
#define GPIOC (&model_gpio_ports[2])
#define GPIOD (&model_gpio_ports[3])
#define PWR_BASE ((uintptr_t)&model_pwr_words[0])
#define APB1PERIPH_BASE 0x00000000u
#define APB2PERIPH_BASE ((uint32_t)(uintptr_t)&model_gpio_ports[0])
#define AHBPERIPH_BASE 0xf0000000u
#define AFIO_BASE 0x40010000u

#define RCC_CR_HSION (1u << 0)
#define RCC_CR_HSIRDY (1u << 1)
#define RCC_CR_HSEON (1u << 16)
#define RCC_CR_HSERDY (1u << 17)
#define RCC_CR_HSEBYP (1u << 18)
#define RCC_CR_CSSON (1u << 19)
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
#define RCC_CFGR_PPRE1_DIV2 (4u << 8)
#define RCC_CFGR_PPRE1_DIV4 (5u << 8)
#define RCC_CFGR_PPRE2_Msk (7u << 11)
#define RCC_CFGR_PPRE2_DIV2 (4u << 11)
#define RCC_CFGR_ADCPRE_DIV8 (3u << 14)
#define RCC_CFGR_PLLSRC_Pos 16
#define RCC_CFGR_PLLSRC_Msk (1u << RCC_CFGR_PLLSRC_Pos)
#define RCC_CFGR_PLLXTPRE_Msk (1u << 17)
#define RCC_CFGR_PLLXTPRE_HSE_DIV2 (1u << 17)
#define RCC_CFGR_PLLMULL_Pos 18
#define RCC_CFGR_PLLMULL_Msk (15u << RCC_CFGR_PLLMULL_Pos)
#define RCC_CFGR_MCO_Msk (7u << 24)
#define RCC_APB1ENR_PWREN (1u << 28)
#define RCC_APB1ENR_BKPEN (1u << 27)
#define FLASH_ACR_LATENCY_Msk 7u
#define FLASH_ACR_PRFTBE (1u << 4)
#define SCB_AIRCR_VECTKEY_Pos 16
#define SCB_AIRCR_PRIGROUP_Msk (7u << 8)
#define SCB_AIRCR_SYSRESETREQ_Msk (1u << 2)
#define PWR_CR_DBP (1u << 8)

#define AFIO_MAPR_SWJ_CFG_Msk (7u << 24)
#define AFIO_MAPR_SWJ_CFG_DISABLE (4u << 24)
#define AFIO_MAPR_SWJ_CFG_JTAGDISABLE (2u << 24)
#define AFIO_MAPR_TIM2_REMAP_Msk (3u << 8)
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1 (1u << 8)
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2 (2u << 8)
#define AFIO_MAPR_TIM3_REMAP_Msk (3u << 10)
#define AFIO_MAPR_TIM3_REMAP_PARTIALREMAP (2u << 10)
#define AFIO_MAPR_TIM3_REMAP_FULLREMAP (3u << 10)
#define AFIO_MAPR_TIM4_REMAP_Msk (1u << 12)
#define AFIO_MAPR_TIM4_REMAP (1u << 12)
#define AFIO_MAPR_I2C1_REMAP_Msk (1u << 1)
#define AFIO_MAPR_I2C1_REMAP (1u << 1)
#define AFIO_MAPR_SPI1_REMAP_Msk (1u << 0)
#define AFIO_MAPR_SPI1_REMAP (1u << 0)
#define AFIO_MAPR_USART1_REMAP_Msk (1u << 2)
#define AFIO_MAPR_USART1_REMAP (1u << 2)
#define AFIO_MAPR_USART2_REMAP_Msk (1u << 3)
#define AFIO_MAPR_USART2_REMAP (1u << 3)
#define AFIO_MAPR_USART3_REMAP_Msk (3u << 4)
#define AFIO_MAPR_USART3_REMAP_FULLREMAP (3u << 4)
#define AFIO_MAPR_CAN_REMAP_Msk (3u << 13)
#define AFIO_MAPR_CAN_REMAP_REMAP2 (2u << 13)
#define AFIO_MAPR_CAN_REMAP_REMAP3 (3u << 13)

#define GPIO(PORT, NUM) (((PORT) - 'A') * 16 + (NUM))
#define GPIO_INPUT 0
#define GPIO_OUTPUT 1
#define GPIO_OPEN_DRAIN 0x100
#define GPIO_ANALOG 3

struct cline { volatile uint32_t *en, *rst; uint32_t bit; };
GPIO_TypeDef *gpio_pin_to_regs(uint32_t pin);
void enable_pclock(uint32_t periph_base);
void sched_main(void);
void irq_disable(void);
void try_request_canboot(void);
void NVIC_SystemReset(void);
void stm32f1_alternative_remap(uint32_t mapr_mask, uint32_t mapr_value);

#endif
