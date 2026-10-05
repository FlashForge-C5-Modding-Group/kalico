#ifndef __GD32H7_H
#define __GD32H7_H

#include <stddef.h>
#include <stdint.h>

// Cortex-M7 device facts consumed by the repository CMSIS core.
#define __CM7_REV                 0x0102U
#define __MPU_PRESENT             1U
#define __NVIC_PRIO_BITS          4U
#define __Vendor_SysTickConfig    0U
#define __FPU_PRESENT             1U
#define __ICACHE_PRESENT          1U
#define __DCACHE_PRESENT          1U

typedef enum {
    NonMaskableInt_IRQn = -14,
    HardFault_IRQn = -13,
    MemoryManagement_IRQn = -12,
    BusFault_IRQn = -11,
    UsageFault_IRQn = -10,
    SysTick_IRQn = -1,
    DMA0_Channel0_IRQn = 11,
    DMA0_Channel1_IRQn = 12,
    ADC0_1_IRQn = 18,
    USART0_IRQn = 37,
    TIMER6_IRQn = 55,
} IRQn_Type;


#include "core_cm7.h"

// Pool excluded from the data cache by the MPU, for DMA-written buffers.
// The MPU SIZE field encodes 2^(SIZE+1) bytes, so 8 selects 512 bytes.
#define NONCACHED_POOL_BYTES     512U
#define NONCACHED_POOL_WORDS     (NONCACHED_POOL_BYTES / 4U)
#define NONCACHED_POOL_MPU_SIZE  8U
extern volatile uint32_t noncached_pool[NONCACHED_POOL_WORDS];

#define REG32(addr) (*(volatile uint32_t *)(uint32_t)(addr))
#define BIT(x) ((uint32_t)((uint32_t)0x01U << (x)))
#define BITS(start, end) \
    ((0xffffffffUL << (start)) & (0xffffffffUL >> (31U - (uint32_t)(end))))

typedef struct {
    __IO uint32_t MODER;
    __IO uint32_t OTYPER;
    __IO uint32_t OSPEEDR;
    __IO uint32_t PUPDR;
    __I  uint32_t IDR;
    __IO uint32_t ODR;
    __O  uint32_t BOP;
    __IO uint32_t LCKR;
    __IO uint32_t AFR[2];
    __O  uint32_t BC;
    __O  uint32_t TG;
} GPIO_TypeDef;

typedef struct {
    __IO uint32_t CTL0;
    __IO uint32_t CTL1;
    __IO uint32_t CTL2;
    __IO uint32_t BAUD;
    __IO uint32_t GP;
    __IO uint32_t RT;
    __O  uint32_t CMD;
    __I  uint32_t STAT;
    __O  uint32_t INTC;
    __I  uint32_t RDATA;
    __O  uint32_t TDATA;
} USART_TypeDef;

#define APB1_BUS_BASE         ((uint32_t)0x40000000U)
#define APB2_BUS_BASE         ((uint32_t)0x40010000U)
#define APB4_BUS_BASE         ((uint32_t)0x58000000U)
#define AHB1_BUS_BASE         ((uint32_t)0x40020000U)
#define AHB4_BUS_BASE         ((uint32_t)0x58020000U)

#define TIMER_BASE            (APB1_BUS_BASE + 0x00000000U)
#define USART_BASE            (APB1_BUS_BASE + 0x00004400U)
#define ADC_BASE              (APB2_BUS_BASE + 0x00002400U)
#define TRIGSEL_BASE          (APB2_BUS_BASE + 0x00008400U)
#define SYSCFG_BASE           (APB4_BUS_BASE + 0x00000400U)
#define FWDGT_BASE            (APB4_BUS_BASE + 0x00004800U)
#define DMA_BASE              (AHB1_BUS_BASE + 0x00000000U)
#define DMAMUX_BASE           (AHB1_BUS_BASE + 0x00000800U)
#define GPIO_BASE             (AHB4_BUS_BASE + 0x00000000U)
#define RCU_BASE              (AHB4_BUS_BASE + 0x00004400U)

#define TIMER1                (TIMER_BASE + 0x00000000U)
#define TIMER2                (TIMER_BASE + 0x00000400U)
#define TIMER3                (TIMER_BASE + 0x00000800U)
#define TIMER4                (TIMER_BASE + 0x00000C00U)
#define TIMER7                (TIMER_BASE + 0x00010400U)
#define TIMER22               (TIMER_BASE + 0x0000E000U)
#define ADC0                  ADC_BASE
#define ADC1                  (ADC_BASE + 0x00000400U)
#define ADC2                  (ADC_BASE + 0x00000800U)
#define DMA0                  DMA_BASE
#define DMAMUX                DMAMUX_BASE
#define TRIGSEL               TRIGSEL_BASE
#define SYSCFG                SYSCFG_BASE
#define RCU                   RCU_BASE
#define FWDGT                 FWDGT_BASE

#define GPIOA_BASE            (GPIO_BASE + 0x00000000U)
#define GPIOB_BASE            (GPIO_BASE + 0x00000400U)
#define GPIOC_BASE            (GPIO_BASE + 0x00000800U)
#define GPIOD_BASE            (GPIO_BASE + 0x00000C00U)
#define GPIOE_BASE            (GPIO_BASE + 0x00001000U)
#define USART0_BASE           (USART_BASE + 0x0000CC00U)

#define GPIOA                 ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB                 ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC                 ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD                 ((GPIO_TypeDef *)GPIOD_BASE)
#define GPIOE                 ((GPIO_TypeDef *)GPIOE_BASE)
#define USART0                ((USART_TypeDef *)USART0_BASE)

// Reset and clock unit.
#define RCU_CTL               REG32(RCU + 0x00000000U)
#define RCU_PLL0              REG32(RCU + 0x00000004U)
#define RCU_CFG0              REG32(RCU + 0x00000008U)
#define RCU_INT               REG32(RCU + 0x0000000CU)
#define RCU_AHB4RST           REG32(RCU + 0x0000001CU)
#define RCU_APB1RST           REG32(RCU + 0x00000020U)
#define RCU_APB2RST           REG32(RCU + 0x00000024U)
#define RCU_AHB1EN            REG32(RCU + 0x00000030U)
#define RCU_AHB4EN            REG32(RCU + 0x0000003CU)
#define RCU_APB1EN            REG32(RCU + 0x00000040U)
#define RCU_APB2EN            REG32(RCU + 0x00000044U)
#define RCU_APB4EN            REG32(RCU + 0x0000004CU)
#define RCU_RSTSCK            REG32(RCU + 0x00000074U)
#define RCU_PLLADDCTL         REG32(RCU + 0x00000080U)
#define RCU_PLL1              REG32(RCU + 0x00000084U)
#define RCU_PLL2              REG32(RCU + 0x00000088U)
#define RCU_CFG1              REG32(RCU + 0x0000008CU)
#define RCU_PLLALL            REG32(RCU + 0x00000098U)
#define RCU_PLL0FRA           REG32(RCU + 0x0000009CU)
#define RCU_PLL1FRA           REG32(RCU + 0x000000A0U)
#define RCU_PLL2FRA           REG32(RCU + 0x000000A4U)
#define RCU_PLL0_RESET_VALUE ((uint32_t)0x01002020U)
#define RCU_PLL1_RESET_VALUE ((uint32_t)0x01012020U)
#define RCU_PLL2_RESET_VALUE ((uint32_t)0x01012020U)
#define RCU_INT_RESET_VALUE  ((uint32_t)0x14FF0000U)

#define RCU_CTL_HXTALEN       BIT(16)
#define RCU_CTL_HXTALSTB      BIT(17)
#define RCU_CTL_PLL0EN        BIT(24)
#define RCU_CTL_PLL0STB       BIT(25)
#define RCU_CTL_PLL1EN        BIT(26)
#define RCU_CTL_PLL1STB       BIT(27)
#define RCU_CTL_PLL2EN        BIT(28)
#define RCU_CTL_PLL2STB       BIT(29)
#define RCU_CTL_IRC64MEN      BIT(30)
#define RCU_CTL_IRC64MSTB     BIT(31)
#define RCU_CFG0_SCS          BITS(0, 1)
#define RCU_CFG0_SCSS         BITS(2, 3)
#define RCU_CFG1_USART0SEL    BITS(0, 1)
#define RCU_PLLADDCTL_PLL0Q   BITS(0, 6)
#define RCU_PLLADDCTL_PLL0QEN BIT(23)
#define RCU_PLLADDCTL_PLL0REN BIT(24)
#define RCU_PLLADDCTL_PLL0PEN BIT(25)
#define RCU_RSTSCK_IRC32KEN   BIT(0)
#define RCU_RSTSCK_IRC32KSTB  BIT(1)
#define RCU_RSTSCK_RSTFC      BIT(24)
#define RCU_RSTSCK_BORRSTF    BIT(25)
#define RCU_RSTSCK_EPRSTF     BIT(26)
#define RCU_RSTSCK_PORRSTF    BIT(27)
#define RCU_RSTSCK_SWRSTF     BIT(28)
#define RCU_RSTSCK_FWDGTRSTF  BIT(29)
#define RCU_RSTSCK_WWDGTRSTF  BIT(30)
#define RCU_RSTSCK_LPRSTF     BIT(31)

#define CFG0_SCS(regval) \
    (BITS(0, 1) & ((uint32_t)(regval) << 0U))
#define CFG0_SCSS(regval) \
    (BITS(2, 3) & ((uint32_t)(regval) << 2U))
#define CFG0_AHBPSC(regval) \
    (BITS(4, 7) & ((uint32_t)(regval) << 4U))
#define CFG0_APB1PSC(regval) \
    (BITS(10, 12) & ((uint32_t)(regval) << 10U))
#define CFG0_APB2PSC(regval) \
    (BITS(13, 15) & ((uint32_t)(regval) << 13U))
#define CFG0_APB3PSC(regval) \
    (BITS(27, 29) & ((uint32_t)(regval) << 27U))
#define CFG0_APB4PSC(regval) \
    (BITS(24, 26) & ((uint32_t)(regval) << 24U))
#define RCU_CKSYSSRC_PLL0P   CFG0_SCS(3)
#define RCU_SCSS_PLL0P       CFG0_SCSS(3)
#define RCU_AHB_CKSYS_DIV2   CFG0_AHBPSC(8)
#define RCU_APB1_CKAHB_DIV2  CFG0_APB1PSC(4)
#define RCU_APB2_CKAHB_DIV1  CFG0_APB2PSC(0)
#define RCU_APB3_CKAHB_DIV2  CFG0_APB3PSC(4)
#define RCU_APB4_CKAHB_DIV2  CFG0_APB4PSC(4)
#define CFG1_USART0SEL(regval) \
    (BITS(0, 1) & ((uint32_t)(regval) << 0U))
#define RCU_USARTSRC_APB     CFG1_USART0SEL(0)
#define PLLALL_PLLSEL(regval) \
    (BITS(16, 17) & ((uint32_t)(regval) << 16U))
#define PLLALL_PLL0RNG(regval) \
    (BITS(0, 1) & ((uint32_t)(regval) << 0U))
#define RCU_PLLSRC_HXTAL     PLLALL_PLLSEL(2)
#define RCU_PLL0RNG_4M_8M    PLLALL_PLL0RNG(2)
#define RCU_PLLNOFFSET       ((uint32_t)6U)
#define RCU_PLLPOFFSET       ((uint32_t)16U)
#define RCU_PLLROFFSET       ((uint32_t)24U)
#define RCU_PLL0P            RCU_PLLADDCTL_PLL0PEN
#define RCU_PLL0Q            RCU_PLLADDCTL_PLL0QEN
#define RCU_PLL0R            RCU_PLLADDCTL_PLL0REN

#define RCU_AHB4EN_PAEN       BIT(0)
#define RCU_AHB4EN_PBEN       BIT(1)
#define RCU_AHB4EN_PCEN       BIT(2)
#define RCU_AHB4EN_PDEN       BIT(3)
#define RCU_AHB4EN_PEEN       BIT(4)
#define RCU_AHB4RST_PARST     BIT(0)
#define RCU_AHB4RST_PBRST     BIT(1)
#define RCU_AHB4RST_PCRST     BIT(2)
#define RCU_AHB4RST_PDRST     BIT(3)
#define RCU_AHB4RST_PERST     BIT(4)
#define RCU_APB1EN_TIMER1EN  BIT(0)
#define RCU_APB1EN_TIMER2EN  BIT(1)
#define RCU_APB1EN_TIMER3EN  BIT(2)
#define RCU_APB1EN_TIMER4EN  BIT(3)
#define RCU_APB1EN_TIMER22EN BIT(6)
#define RCU_APB1RST_TIMER1RST BIT(0)
#define RCU_APB1RST_TIMER2RST BIT(1)
#define RCU_APB1RST_TIMER3RST BIT(2)
#define RCU_APB1RST_TIMER4RST BIT(3)
#define RCU_APB1RST_TIMER22RST BIT(6)
#define RCU_APB2EN_TIMER7EN   BIT(1)
#define RCU_APB2EN_USART0EN   BIT(4)
#define RCU_APB2EN_ADC0EN     BIT(8)
#define RCU_APB2EN_ADC1EN     BIT(9)
#define RCU_APB2EN_ADC2EN     BIT(10)
#define RCU_APB2RST_TIMER7RST BIT(1)
#define RCU_APB2RST_USART0RST BIT(4)
#define RCU_APB2RST_ADC0RST   BIT(8)
#define RCU_APB2RST_ADC1RST   BIT(9)
#define RCU_APB2RST_ADC2RST   BIT(10)
#define RCU_AHB1EN_DMA0EN     BIT(21)
#define RCU_AHB1EN_DMAMUXEN   BIT(23)
#define RCU_APB2EN_TRGSELEN   BIT(31)
#define RCU_APB4EN_SYSCFGEN   BIT(0)

// System configuration controller.
#define SYSCFG_SRAMCFG1      REG32(SYSCFG + 0x00000068U)
#define SYSCFG_SRAMCFG1_TCM_WAITSTATE BIT(0)
#define SYSCFG_TIMERCFG_TSCFG5 BITS(26, 30)
#define SYSCFG_TIMERCFG0(syscfg_timerx) \
    REG32(SYSCFG + 0x100U + (syscfg_timerx) * 0x0CU)
#define SYSCFG_TIMERCFG1(syscfg_timerx) \
    REG32(SYSCFG + 0x104U + (syscfg_timerx) * 0x0CU)
#define SYSCFG_TIMERCFG2(syscfg_timerx) \
    REG32(SYSCFG + 0x108U + (syscfg_timerx) * 0x0CU)
#define SYSCFG_TIMERCFG_TSCFG5_VALUE(regval) \
    (SYSCFG_TIMERCFG_TSCFG5 & ((uint32_t)(regval) << 26U))
#define SYSCFG_TIMER0       ((uint8_t)0x00U)
#define SYSCFG_TIMER1       ((uint8_t)0x01U)
#define SYSCFG_TIMER3       ((uint8_t)0x03U)
#define SYSCFG_TIMER4       ((uint8_t)0x04U)
#define SYSCFG_TIMER7       ((uint8_t)0x05U)
#define TIMER_SMCFG_TRGSEL_ITI2  ((uint8_t)0x03U)
#define TIMER_SMCFG_TRGSEL_ITI14 ((uint8_t)0x13U)

// Timer registers and fields.
#define TIMER_CTL0(timerx)        REG32((timerx) + 0x00000000U)
#define TIMER_CTL1(timerx)        REG32((timerx) + 0x00000004U)
#define TIMER_SMCFG(timerx)       REG32((timerx) + 0x00000008U)
#define TIMER_SWEVG(timerx)       REG32((timerx) + 0x00000014U)
#define TIMER_CHCTL0(timerx)      REG32((timerx) + 0x00000018U)
#define TIMER_CHCTL1(timerx)      REG32((timerx) + 0x0000001CU)
#define TIMER_CHCTL2(timerx)      REG32((timerx) + 0x00000020U)
#define TIMER_CNT(timerx)         REG32((timerx) + 0x00000024U)
#define TIMER_PSC(timerx)         REG32((timerx) + 0x00000028U)
#define TIMER_CAR(timerx)         REG32((timerx) + 0x0000002CU)
#define TIMER_CREP0(timerx)       REG32((timerx) + 0x00000030U)
#define TIMER_CH0CV(timerx)       REG32((timerx) + 0x00000034U)
#define TIMER_CH1CV(timerx)       REG32((timerx) + 0x00000038U)
#define TIMER_CH2CV(timerx)       REG32((timerx) + 0x0000003CU)
#define TIMER_CH3CV(timerx)       REG32((timerx) + 0x00000040U)
#define TIMER_CCHP(timerx)        REG32((timerx) + 0x00000044U)
#define TIMER_CH0COMV_ADD(timerx) REG32((timerx) + 0x00000064U)
#define TIMER_CH1COMV_ADD(timerx) REG32((timerx) + 0x00000068U)
#define TIMER_CH2COMV_ADD(timerx) REG32((timerx) + 0x0000006CU)
#define TIMER_CH3COMV_ADD(timerx) REG32((timerx) + 0x00000070U)
#define TIMER_CTL2(timerx)        REG32((timerx) + 0x00000074U)
#define TIMER_CFG(timerx)         REG32((timerx) + 0x000000FCU)

#define TIMER_CTL0_CEN            BIT(0)
#define TIMER_CTL0_DIR            BIT(4)
#define TIMER_CTL0_CAM            BITS(5, 6)
#define TIMER_CTL0_CKDIV          BITS(8, 9)
#define TIMER_CTL1_MMC0           BITS(4, 6)
#define TIMER_CTL1_ISO0           BIT(8)
#define TIMER_CTL1_ISO0N          BIT(9)
#define TIMER_CTL1_ISO1           BIT(10)
#define TIMER_CTL1_ISO1N          BIT(11)
#define TIMER_CTL1_ISO2           BIT(12)
#define TIMER_CTL1_ISO2N          BIT(13)
#define TIMER_CTL1_ISO3           BIT(14)
#define TIMER_CTL1_ISO3N          BIT(15)
#define TIMER_SMCFG_MSM           BIT(7)
#define TIMER_SWEVG_UPG           BIT(0)
#define TIMER_CHCTL0_CH0MS        (BITS(0, 1) | BIT(30))
#define TIMER_CHCTL0_CH0COMSEN    BIT(3)
#define TIMER_CHCTL0_CH0COMCTL    (BITS(4, 6) | BIT(16))
#define TIMER_CHCTL0_CH1MS        (BITS(8, 9) | BIT(31))
#define TIMER_CHCTL0_CH1COMSEN    BIT(11)
#define TIMER_CHCTL0_CH1COMCTL    (BITS(12, 14) | BIT(24))
#define TIMER_CHCTL0_CH0COMADDSEN BIT(28)
#define TIMER_CHCTL0_CH1COMADDSEN BIT(29)
#define TIMER_CHCTL1_CH2MS        (BITS(0, 1) | BIT(30))
#define TIMER_CHCTL1_CH2COMSEN    BIT(3)
#define TIMER_CHCTL1_CH2COMCTL    (BITS(4, 6) | BIT(16))
#define TIMER_CHCTL1_CH3MS        (BITS(8, 9) | BIT(31))
#define TIMER_CHCTL1_CH3COMSEN    BIT(11)
#define TIMER_CHCTL1_CH3COMCTL    (BITS(12, 14) | BIT(24))
#define TIMER_CHCTL1_CH2COMADDSEN BIT(28)
#define TIMER_CHCTL1_CH3COMADDSEN BIT(29)
#define TIMER_CHCTL2_CH0EN        BIT(0)
#define TIMER_CHCTL2_CH0P         BIT(1)
#define TIMER_CHCTL2_CH0NEN       BIT(2)
#define TIMER_CHCTL2_CH0NP        BIT(3)
#define TIMER_CHCTL2_CH1EN        BIT(4)
#define TIMER_CHCTL2_CH1P         BIT(5)
#define TIMER_CHCTL2_CH1NEN       BIT(6)
#define TIMER_CHCTL2_CH1NP        BIT(7)
#define TIMER_CHCTL2_CH2EN        BIT(8)
#define TIMER_CHCTL2_CH2P         BIT(9)
#define TIMER_CHCTL2_CH2NEN       BIT(10)
#define TIMER_CHCTL2_CH2NP        BIT(11)
#define TIMER_CHCTL2_CH3EN        BIT(12)
#define TIMER_CHCTL2_CH3P         BIT(13)
#define TIMER_CHCTL2_CH3NEN       BIT(14)
#define TIMER_CHCTL2_CH3NP        BIT(15)
#define TIMER_CCHP_POEN           BIT(15)
#define TIMER_CTL2_CH0CPWMEN      BIT(28)
#define TIMER_CTL2_CH1CPWMEN      BIT(29)
#define TIMER_CTL2_CH2CPWMEN      BIT(30)
#define TIMER_CTL2_CH3CPWMEN      BIT(31)
#define TIMER_CFG_CREPSEL         BIT(2)
#define TIMER_OC_MODE_PWM0        ((uint32_t)0x00000060U)
#define TIMER_OC_MODE_PWM1        ((uint32_t)0x00000070U)
#define TIMER_TRI_OUT0_SRC_ENABLE (BITS(4, 6) & ((uint32_t)1U << 4U))
#define TIMER_CHCTL0_CH0COMCTL_VALUE(value) \
    ((uint32_t)(value) & TIMER_CHCTL0_CH0COMCTL)
#define TIMER_CHCTL0_CH1COMCTL_VALUE(value) \
    (((uint32_t)(value) << 8U) & TIMER_CHCTL0_CH1COMCTL)
#define TIMER_CHCTL1_CH2COMCTL_VALUE(value) \
    ((uint32_t)(value) & TIMER_CHCTL1_CH2COMCTL)
#define TIMER_CHCTL1_CH3COMCTL_VALUE(value) \
    (((uint32_t)(value) << 8U) & TIMER_CHCTL1_CH3COMCTL)

// ADC registers and fields.
#define ADC_STAT(adcx)      REG32((adcx) + 0x00000000U)
#define ADC_CTL0(adcx)      REG32((adcx) + 0x00000004U)
#define ADC_CTL1(adcx)      REG32((adcx) + 0x00000008U)
#define ADC_RSQ0(adcx)      REG32((adcx) + 0x00000024U)
#define ADC_RSQ7(adcx)      REG32((adcx) + 0x00000040U)
#define ADC_RSQ8(adcx)      REG32((adcx) + 0x00000044U)
#define ADC_ISQ0(adcx)      REG32((adcx) + 0x00000048U)
#define ADC_ISQ1(adcx)      REG32((adcx) + 0x0000004CU)
#define ADC_IDATA0(adcx)    REG32((adcx) + 0x00000054U)
#define ADC_IDATA1(adcx)    REG32((adcx) + 0x00000058U)
#define ADC_RDATA(adcx)     REG32((adcx) + 0x00000064U)
#define ADC_SYNCCTL(adcx)   REG32((adcx) + 0x00000304U)

#define ADC_STAT_EOC        BIT(1)
#define ADC_STAT_EOIC       BIT(2)
#define ADC_STAT_ROVF       BIT(5)
#define ADC_CTL0_EOICIE     BIT(7)
#define ADC_CTL0_SM         BIT(8)
#define ADC_CTL0_DRES       BITS(24, 25)
#define ADC_CTL1_ADCON      BIT(0)
#define ADC_CTL1_CLB        BIT(2)
#define ADC_CTL1_RSTCLB     BIT(3)
#define ADC_CTL1_CALNUM     BITS(4, 6)
#define ADC_CTL1_DMA        BIT(8)
#define ADC_CTL1_DDM        BIT(9)
#define ADC_CTL1_DAL        BIT(11)
#define ADC_CTL1_ETMIC      BITS(20, 21)
#define ADC_CTL1_CALMOD     BIT(27)
#define ADC_CTL1_ETMRC      BITS(28, 29)
#define ADC_CTL1_SWRCST     BIT(30)
#define ADC_RSQX_RSQN       BITS(0, 4)
#define ADC_RSQX_RSMPN      BITS(5, 14)
#define ADC_RSQ0_RL         BITS(20, 23)
#define ADC_ISQX_ISQN       BITS(0, 4)
#define ADC_ISQX_ISMPN      BITS(5, 14)
#define ADC_ISQ0_IL         BITS(20, 21)
#define ADC_SYNCCTL_ADCSCK  BITS(16, 19)
#define ADC_SYNCCTL_ADCCK   BITS(20, 23)
#define SQX_SMP(regval) \
    (BITS(5, 14) & ((uint32_t)(regval) << 5U))
#define RSQ0_RL(regval) \
    (BITS(20, 23) & ((uint32_t)(regval) << 20U))
#define ISQ0_IL(regval) \
    (BITS(20, 21) & ((uint32_t)(regval) << 20U))
#define SYNCCTL_ADCSCK(regval) \
    (BITS(16, 19) & ((uint32_t)(regval) << 16U))
#define SYNCCTL_ADCCK(regval) \
    (BITS(20, 23) & ((uint32_t)(regval) << 20U))
#define ADC_CLK_SYNC_HCLK_DIV4 \
    (SYNCCTL_ADCCK(0) | SYNCCTL_ADCSCK(9))
#define ADC_CLK_SYNC_HCLK_DIV6 \
    (SYNCCTL_ADCCK(0) | SYNCCTL_ADCSCK(10))
#define EXTERNAL_TRIGGER_RISING ((uint32_t)0x00000001U)
#define ROUTINE_TRIGGER_MODE    ((uint8_t)28U)
#define INSERTED_TRIGGER_MODE   ((uint8_t)20U)
#define ADC_INSERTED_CHANNEL_SHIFT_LENGTH ((uint8_t)16U)

// DMA and DMAMUX registers and fields.
#define DMA_INTF0(dmax)          REG32((dmax) + 0x00000000U)
#define DMA_INTC0(dmax)          REG32((dmax) + 0x00000008U)
#define DMA_CHCTL(dma, channel) \
    REG32(((dma) + 0x00000010U) + 0x00000018U * (channel))
#define DMA_CHCNT(dma, channel) \
    REG32(((dma) + 0x00000014U) + 0x00000018U * (channel))
#define DMA_CHPADDR(dma, channel) \
    REG32(((dma) + 0x00000018U) + 0x00000018U * (channel))
#define DMA_CHM0ADDR(dma, channel) \
    REG32(((dma) + 0x0000001CU) + 0x00000018U * (channel))
#define DMA_CHM1ADDR(dma, channel) \
    REG32(((dma) + 0x00000020U) + 0x00000018U * (channel))
#define DMA_CHFCTL(dma, channel) \
    REG32(((dma) + 0x00000024U) + 0x00000018U * (channel))
#define DMAMUX_RM_CHXCFG(channel) \
    REG32(DMAMUX + 0x04U * (uint32_t)(channel))

#define DMA_INTF_FEEIF          BIT(0)
#define DMA_INTF_SDEIF          BIT(2)
#define DMA_INTF_TAEIF          BIT(3)
#define DMA_INTF_FTFIF          BIT(5)
#define DMA_INTC_FEEIFC         BIT(0)
#define DMA_INTC_SDEIFC         BIT(2)
#define DMA_INTC_TAEIFC         BIT(3)
#define DMA_INTC_HTFIFC         BIT(4)
#define DMA_INTC_FTFIFC         BIT(5)
#define DMA_CHXCTL_CHEN         BIT(0)
#define DMA_CHXCTL_FTFIE        BIT(4)
#define DMA_CHXCTL_CMEN         BIT(8)
#define DMA_CHXCTL_MNAGA        BIT(10)
#define DMA_CHXCTL_PWIDTH       BITS(11, 12)
#define DMA_CHXCTL_MWIDTH       BITS(13, 14)
#define DMA_CHXCTL_PRIO         BITS(16, 17)
#define DMAMUX_RM_CHXCFG_MUXID  BITS(0, 7)
#define DMA_FLAG_ADD(flag, channel) \
    ((uint32_t)((flag) << ((((uint32_t)(channel) * 6U)) \
     + ((uint32_t)(((uint32_t)(channel)) >> 1U) & 0x01U) * 4U)))
#define CHCTL_PRIO(regval) \
    (DMA_CHXCTL_PRIO & ((uint32_t)(regval) << 16U))
#define CHCTL_MWIDTH(regval) \
    (DMA_CHXCTL_MWIDTH & ((uint32_t)(regval) << 13U))
#define CHCTL_PWIDTH(regval) \
    (DMA_CHXCTL_PWIDTH & ((uint32_t)(regval) << 11U))
#define DMA_PRIORITY_ULTRA_HIGH CHCTL_PRIO(3)
#define DMA_MEMORY_WIDTH_32BIT  CHCTL_MWIDTH(2)
#define DMA_PERIPH_WIDTH_32BIT  CHCTL_PWIDTH(2)
#define RM_CHXCFG_MUXID(regval) \
    (BITS(0, 7) & ((uint32_t)(regval) << 0U))
#define DMA_REQUEST_ADC0        RM_CHXCFG_MUXID(9U)
#define DMA_REQUEST_ADC1        RM_CHXCFG_MUXID(10U)

// Trigger selector registers and descriptors.
#define TRIGSEL_TIMER7ITI14     REG32(TRIGSEL + 0x000000A0U)
#define TRIGSEL_TARGET_LK       BIT(31)
#define TRIGSEL_TARGET_REG(target_periph) \
    REG32(TRIGSEL + ((uint32_t)(target_periph) & BITS(2, 31)))
#define TRIGSEL_TARGET_PERIPH_SHIFT(target_periph) \
    (((uint32_t)(target_periph) & BITS(0, 1)) << 3U)
#define TRIGSEL_TARGET_PERIPH_MASK(target_periph) \
    ((uint32_t)(BITS(0, 7) << TRIGSEL_TARGET_PERIPH_SHIFT(target_periph)))
#define TRIGSEL_INPUT_TIMER2_CH0 ((uint8_t)0x2CU)
#define TRIGSEL_INPUT_TIMER2_CH1 ((uint8_t)0x2DU)
#define TRIGSEL_INPUT_TIMER2_CH2 ((uint8_t)0x2EU)
#define TRIGSEL_INPUT_TIMER2_CH3 ((uint8_t)0x2FU)
#define TRIGSEL_INPUT_TIMER4_CH0 ((uint8_t)0x38U)
#define TRIGSEL_INPUT_TIMER4_CH1 ((uint8_t)0x39U)
#define TRIGSEL_INPUT_TIMER4_CH2 ((uint8_t)0x3AU)
#define TRIGSEL_INPUT_TIMER4_CH3 ((uint8_t)0x3BU)
#define TRIGSEL_OUTPUT_ADC0_ROUTRG  ((uint8_t)0x10U)
#define TRIGSEL_OUTPUT_ADC0_INSTRG  ((uint8_t)0x11U)
#define TRIGSEL_OUTPUT_ADC1_ROUTRG  ((uint8_t)0x14U)
#define TRIGSEL_OUTPUT_ADC1_INSTRG  ((uint8_t)0x15U)
#define TRIGSEL_OUTPUT_TIMER0_ITI14 ((uint8_t)0x8CU)
#define TRIGSEL_OUTPUT_TIMER1_ITI14 ((uint8_t)0x90U)
#define TRIGSEL_OUTPUT_TIMER3_ITI14 ((uint8_t)0x98U)
#define TRIGSEL_OUTPUT_TIMER7_ITI14 ((uint8_t)0xA0U)

// USART fields.
#define USART_CTL0_UEN          BIT(0)
#define USART_CTL0_REN          BIT(2)
#define USART_CTL0_TEN          BIT(3)
#define USART_CTL0_RBNEIE       BIT(5)
#define USART_CTL0_TBEIE        BIT(7)
#define USART_STAT_PERR         BIT(0)
#define USART_STAT_FERR         BIT(1)
#define USART_STAT_NERR         BIT(2)
#define USART_STAT_ORERR        BIT(3)
#define USART_STAT_RBNE         BIT(5)
#define USART_STAT_TBE          BIT(7)
#define USART_INTC_PEC          BIT(0)
#define USART_INTC_FEC          BIT(1)
#define USART_INTC_NEC          BIT(2)
#define USART_INTC_OREC         BIT(3)

// Free watchdog.
#define FWDGT_CTL               REG32(FWDGT + 0x00000000U)
#define FWDGT_PSC               REG32(FWDGT + 0x00000004U)
#define FWDGT_RLD               REG32(FWDGT + 0x00000008U)
#define FWDGT_STAT              REG32(FWDGT + 0x0000000CU)
#define FWDGT_WND               REG32(FWDGT + 0x00000010U)
#define FWDGT_STAT_PUD          BIT(0)
#define FWDGT_STAT_RUD          BIT(1)
#define FWDGT_STAT_WUD          BIT(2)
#define FWDGT_RLD_RLD           BITS(0, 11)
#define FWDGT_WND_WND           BITS(0, 11)
#define FWDGT_PSC_DIV4          ((uint8_t)0U)
#define FWDGT_WRITEACCESS_ENABLE ((uint16_t)0x5555U)
#define FWDGT_KEY_RELOAD         ((uint16_t)0xAAAAU)
#define FWDGT_KEY_ENABLE         ((uint16_t)0xCCCCU)

// Power management unit.  The low-voltage detector is a monitor only: brown
// out reset is disabled in this part's factory option bytes, so a supply sag
// short of the power-down threshold is otherwise invisible.
#define PMU_BASE                (APB4_BUS_BASE + 0x00005800U)
#define PMU_CTL0                REG32(PMU_BASE + 0x00000000U)
#define PMU_CS                  REG32(PMU_BASE + 0x00000004U)
#define PMU_CTL0_LVDEN          BIT(4)
#define PMU_CTL0_LVDT           BITS(5, 7)
#define PMU_LVDT_2V9            ((uint32_t)6U << 5)
#define PMU_CS_LVDF             BIT(2)
#define RCU_APB4EN_PMUEN        BIT(4)

// TIMER6 is a basic timer.  It is not part of the stock motor trigger
// topology and is used only to give the stall check a periodic
// priority-0 interrupt that does not depend on host traffic.
#define TIMER6                  (TIMER_BASE + 0x00001400U)
#define RCU_APB1EN_TIMER6EN     BIT(5)
#define TIMER_DMAINTEN(timerx)  REG32((timerx) + 0x0000000CU)
#define TIMER_INTF(timerx)      REG32((timerx) + 0x00000010U)
#define TIMER_DMAINTEN_UPIE     BIT(0)
#define TIMER_INTF_UPIF         BIT(0)

_Static_assert((uintptr_t)&PMU_CTL0 == 0x58005800U
               && (uintptr_t)&PMU_CS == 0x58005804U
               && TIMER6 == 0x40001400U,
               "GD32H7 PMU or basic timer address mismatch");

_Static_assert(offsetof(GPIO_TypeDef, MODER) == 0x00U
               && offsetof(GPIO_TypeDef, OTYPER) == 0x04U
               && offsetof(GPIO_TypeDef, OSPEEDR) == 0x08U
               && offsetof(GPIO_TypeDef, PUPDR) == 0x0CU
               && offsetof(GPIO_TypeDef, IDR) == 0x10U
               && offsetof(GPIO_TypeDef, ODR) == 0x14U
               && offsetof(GPIO_TypeDef, BOP) == 0x18U
               && offsetof(GPIO_TypeDef, LCKR) == 0x1CU
               && offsetof(GPIO_TypeDef, AFR) == 0x20U
               && offsetof(GPIO_TypeDef, BC) == 0x28U
               && offsetof(GPIO_TypeDef, TG) == 0x2CU,
               "GD32H7 GPIO register layout mismatch");
_Static_assert(offsetof(USART_TypeDef, CTL0) == 0x00U
               && offsetof(USART_TypeDef, CTL1) == 0x04U
               && offsetof(USART_TypeDef, CTL2) == 0x08U
               && offsetof(USART_TypeDef, BAUD) == 0x0CU
               && offsetof(USART_TypeDef, GP) == 0x10U
               && offsetof(USART_TypeDef, RT) == 0x14U
               && offsetof(USART_TypeDef, CMD) == 0x18U
               && offsetof(USART_TypeDef, STAT) == 0x1CU
               && offsetof(USART_TypeDef, INTC) == 0x20U
               && offsetof(USART_TypeDef, RDATA) == 0x24U
               && offsetof(USART_TypeDef, TDATA) == 0x28U,
               "GD32H7 USART register layout mismatch");
_Static_assert(RCU_BASE == 0x58024400U && SYSCFG_BASE == 0x58000400U
               && DMA0 == 0x40020000U && DMAMUX == 0x40020800U
               && TRIGSEL == 0x40018400U && ADC0 == 0x40012400U,
               "GD32H7 critical peripheral bases mismatch");
_Static_assert((uintptr_t)&RCU_CTL == 0x58024400U
               && (uintptr_t)&RCU_PLL1 == 0x58024484U
               && (uintptr_t)&RCU_PLL2 == 0x58024488U
               && (uintptr_t)&RCU_CFG1 == 0x5802448CU,
               "GD32H7 RCU register addresses mismatch");
_Static_assert((uintptr_t)&TIMER_SWEVG(TIMER2) == 0x40000414U
               && (uintptr_t)&TIMER_CH0COMV_ADD(TIMER7) == 0x40010464U
               && (uintptr_t)&ADC_RSQ8(ADC2) == 0x40012C44U
               && (uintptr_t)&ADC_SYNCCTL(ADC0) == 0x40012704U,
               "GD32H7 timer/ADC register addresses mismatch");
_Static_assert((uintptr_t)&DMA_CHCTL(DMA0, 1) == 0x40020028U
               && (uintptr_t)&DMAMUX_RM_CHXCFG(1) == 0x40020804U
               && (uintptr_t)&SYSCFG_TIMERCFG0(SYSCFG_TIMER7) == 0x5800053CU
               && (uintptr_t)&TRIGSEL_TIMER7ITI14 == 0x400184A0U,
               "GD32H7 DMA/routing register addresses mismatch");
_Static_assert((uintptr_t)&FWDGT_WND == 0x58004810U,
               "GD32H7 FWDGT register address mismatch");
_Static_assert(RCU_AHB4EN_PAEN == RCU_AHB4RST_PARST
               && RCU_AHB4EN_PBEN == RCU_AHB4RST_PBRST
               && RCU_AHB4EN_PCEN == RCU_AHB4RST_PCRST
               && RCU_AHB4EN_PDEN == RCU_AHB4RST_PDRST
               && RCU_AHB4EN_PEEN == RCU_AHB4RST_PERST
               && RCU_APB1EN_TIMER1EN == RCU_APB1RST_TIMER1RST
               && RCU_APB1EN_TIMER2EN == RCU_APB1RST_TIMER2RST
               && RCU_APB1EN_TIMER3EN == RCU_APB1RST_TIMER3RST
               && RCU_APB1EN_TIMER4EN == RCU_APB1RST_TIMER4RST
               && RCU_APB1EN_TIMER22EN == RCU_APB1RST_TIMER22RST
               && RCU_APB2EN_TIMER7EN == RCU_APB2RST_TIMER7RST
               && RCU_APB2EN_USART0EN == RCU_APB2RST_USART0RST
               && RCU_APB2EN_ADC0EN == RCU_APB2RST_ADC0RST
               && RCU_APB2EN_ADC1EN == RCU_APB2RST_ADC1RST
               && RCU_APB2EN_ADC2EN == RCU_APB2RST_ADC2RST,
               "GD32H7 clock/reset bit positions mismatch");

#endif // gd32h7.h
