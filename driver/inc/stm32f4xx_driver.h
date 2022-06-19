#ifndef _STM32_HEADER_
#define _STM32_HEADER_
#include "stdint.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_i2c.h"
#include <stddef.h>

#define __vo    volatile

/* other implementation macros */

#define GPIO_BASEADDR_TO_CODE(x)                       (x == GPIOA) ? 0 : \
                                                  (x == GPIOB) ? 1 : \
                                                  (x == GPIOC) ? 2 : \
                                                  (x == GPIOD) ? 3 : \
                                                  (x == GPIOE) ? 4 : \
                                                  (x == GPIOF) ? 5 : \
                                                  (x == GPIOG) ? 6 : \
                                                  (x == GPIOH) ? 7 : \
                                                  (x == GPIOI) ? 8 : 0

/****************************************************** PROCESSOR SPECIFIC DETAIL ******************************************************/

#define NVIC_BASEADDR                             0xE000E100               /* nested vector interupt controller base address */

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4



/* base address of FLASH and SRAM memories */

#define FLASH_BASEADDR                          0x08000000U             /* base address of FLASh area       */
#define SRAM1_BASEADDR                          0x20000000U             /* base address of SRAM1 area       */
#define SRAM2_BASEADDR                          0x2001C000U             /* base address of SRAM2 area       */
#define ROM_ BASEADDR                           0x1FFF0000              /* base address of ROM area         */
#define SRAM                                    SRAM1_BASEADDR          /* base address of SRAM area        */
#define APB1PERIPH_BASE                         0x40000000U             /* base address of APB1 bus         */
#define APB2PERIPH_BASE                         0x40010000U             /* base address of APB2 bus         */
#define AHB1PERIPH_BASE                         0x40020000U             /* base address of AHB1 bus         */
#define AHB2PERIPH_BASE                         0x50000000U             /* base address of AHB2 bus         */
#define AHB3PERIPH_BASE                         0xA0000000U             /* base address of AHB3 bus         */
#define PERIPH_BASE                             APB1PERIPH_BASE         /* base address of peripheral       */

/*
 * Base address of peripherals which are hangin on AHB1 bus
 */

#define USBOTGHS_BASEADDR                       0x40040000U             /* base address of USBOTGHS         */
#define DMA2_BASEADDR                           0x40026400U             /* base address of DMA2             */
#define DAM1_BASEADDR                           0x40026000U             /* base address of DMA1             */
#define BKSRAM_BASEADDR                         0x40024000U             /* base address of BKSRAM           */
#define FLINTERFACE_BASEADDR                    0x40023C00U             /* base address of FLASH INTERFACE  */
#define RRC_BASEADDR                            0x40023800U             /* base address of RRC              */
#define CRC_BASEADDR                            0x40023000U             /* base address of CRC              */
#define GPIOK_BASEADDR                          0x40022800U            /* base address of GPIOK            */
#define GPIOJ_BASEADDR                          0x40022400U             /* base address of GPIOJ            */
#define GPIOI_BASEADDR                          0x40022000U             /* base address of GPIOI            */
#define GPIOH_BASEADDR                          0x40021C00U             /* base address of GPIOH            */
#define GPIOG_BASEADDR                          0x40021800U             /* base address of GPIOG            */
#define GPIOF_BASEADDR                          0x40021400U             /* base address of GPIOF            */
#define GPIOE_BASEADDR                          0x40021000U             /* base address of GPIOE            */
#define GPIOD_BASEADDR                          0x40020C00U             /* base address of GPIOD            */
#define GPIOC_BASEADDR                          0x40020800U             /* base address of GPIOC            */
#define GPIOB_BASEADDR                          0x40020400U             /* base address of GPIOB            */
#define GPIOA_BASEADDR                          0x40020000U             /* base address of GPIOA            */


/*
 * End of define base address which are hanging on AHB1 bus
 */


/*
 * Base address of peripherals which are hangin on AHB2 bus
 */

#define RNG_BASEADDR                            0x50060800U             /* base address of RNG              */
#define HASH_BASEADDR                           0x50060400U             /* base address of HASH             */
#define CRYTO_BASEADDR                          0x50060000U             /* base address of CRYPTO           */
#define DCMI_BASEADDR                           0x50060000U             /* base address of DCMI             */
#define USBOTGFS_BASEADDR                       AHB2PERIPH_BASE         /* base address of USBOTFSG         */

/*
 * End of define base address which are hanging on AHB2 bus
 */

/*
 * Base address of peripherals which are hangin on APB1 bus
 */

#define TIM2_BASEADDR                           (APB1PERIPH_BASE + 0x0000)                      /* base address of TIMER 2                          */
#define TIM3_BASEADDR                           (APB1PERIPH_BASE + 0x0400)                      /* base address of TIMER 3                          */
#define TIM4_BASEADDR                           (APB1PERIPH_BASE + 0x0800)                      /* base address of TIMER 4                          */
#define TIM5_BASEADDR                           (APB1PERIPH_BASE + 0x0C00)                      /* base address of TIMER 5                          */
#define TIM6_BASEADDR                           (APB1PERIPH_BASE + 0x1000)                      /* base address of TIMER 6                          */
#define TIM7_BASEADDR                           (APB1PERIPH_BASE + 0x1400)                      /* base address of TIMER 7                          */
#define TIM12_BASEADDR                          (APB1PERIPH_BASE + 0x1800)                      /* base address of TIMER 12                         */
#define TIM13_BASEADDR                          (APB1PERIPH_BASE + 0x1C00)                      /* base address of TIMER 13                         */
#define TIM14_BASEADDR                          (APB1PERIPH_BASE + 0x2000)                      /* base address of TIMER 14                         */
#define RTCBK_BASEADDR                          (APB1PERIPH_BASE + 0x2800)                      /* base address of RTC BACKUP                       */
#define WWDG_BASEADDR                           (APB1PERIPH_BASE + 0x2C00)                      /* base address of WWDG                             */
#define IWDG_BASEADDR                           (APB1PERIPH_BASE + 0x3000)                      /* base address of IWDG                             */
#define I2C1_BASEADDR                           (APB1PERIPH_BASE + 0x5400)                      /* base address of I2C1                             */
#define I2C2_BASEADDR                           (APB1PERIPH_BASE + 0x5800)                      /* base address of I2C2                             */
#define I2C3_BASEADDR                           (APB1PERIPH_BASE + 0x6000)                      /* base address of I2C3                             */
#define UART7_BASEADDR                          (APB1PERIPH_BASE + 0x7800)                      /* base address of UART7                            */
#define UART8_BASEADDR                          (APB1PERIPH_BASE + 0x7C00)                      /* base address of UART8                            */
#define USART2_BASEADDR                         (APB1PERIPH_BASE + 0x4400)                      /* base address of USART2                           */
#define USART3_BASEADDR                         (APB1PERIPH_BASE + 0x4800)                      /* base address of USART3                           */
#define UART4_BASEADDR                          (APB1PERIPH_BASE + 0x4C00)                      /* base address of UART4                            */
#define UART5_BASEADDR                          (APB1PERIPH_BASE + 0x5000)                      /* base address of UART5                            */
#define SPI2_BASEADDR                           (APB1PERIPH_BASE + 0x3800)                      /* base address of SPI2                             */
#define SPI3_BASEADDR                           (APB1PERIPH_BASE + 0x3C00)                      /* base address of SPI3                             */
/*
 * End of define base address which are hanging on APB1 bus
 */

/*
 * Base address of peripherals which are hangin on APB2 bus
 */

#define EXTI_BASEADDR                           (APB2PERIPH_BASE+0x3C00)
#define SPI1_BASEADDR                           (APB2PERIPH_BASE+0x3000)
#define SPI6_BASEADDR                           (APB2PERIPH_BASE+0x5400)
#define SPI5_BASEADDR                           (APB2PERIPH_BASE+0x5000)
#define SPI4_BASEADDR                           (APB2PERIPH_BASE+0x3400)
#define USART6_BASEADDR                         (APB2PERIPH_BASE+0x1400)
#define USART1_BASEADDR                         (APB2PERIPH_BASE+0x1000)
#define SYSCFG_BASEADDR                         (APB2PERIPH_BASE+0x3800)

/*
 * End of define base address which are hanging on APB2 bus
 */

/******************************************************  peripheral register definition structure of processor ******************************************************/

typedef struct
{
     /* data */
     __vo uint32_t ISER[8];
     	  uint8_t Reserved1[96];
     __vo uint32_t ICER[8];
     uint8_t Reserved2[96];
     __vo uint32_t ISPR[8];
     uint8_t Reserved3[96];
     __vo uint32_t ICPR[8];
     uint8_t Reserved4[96];
     __vo uint32_t IABR[8];
     uint8_t Reserved5[224];
     __vo uint32_t IPR[60];
     uint8_t Reserved6[96];
     __vo uint32_t STIR;
}__attribute__((packed)) NVIC_RegDef_t;


/******************************************************  peripheral register definition structures  ******************************************************/


typedef struct
{
    /* data */
    __vo uint32_t  MODER;                           /* GPIO port mode register                      Address offset 0x00 */
    __vo uint32_t  OTYPER;                          /* GPIO port output type register               Address offset 0x04 */
    __vo uint32_t  OSPEEDR;                         /* GPIO port output speed register              Address offset 0x08 */
    __vo uint32_t  PUPDR;                           /* GPIO port pull-up/pull-down register         Address offset 0x0C */
    __vo uint32_t  IDR;                             /* GPIO port input data register                Address offset 0x10 */
    __vo uint32_t  ODR;                             /* GPIO port output data register               Address offset 0x14 */
    __vo uint32_t  BSRR;                            /* GPIO port bit set/reset register             Address offset 0x18 */
    __vo uint32_t  LCKR;                            /* GPIO port configuration lock register        Address offset 0x1C */
    __vo uint32_t  AFR[2];                          /* GPIO alternate function register             Address offset 0x20 */
}GPIO_RegDef_t;

typedef struct
{
     /* data */
     __vo uint32_t MEMRMP;                        /* SYSCFG memory remap register                             Address offset 0x00                                         */
     __vo uint32_t PMC;                           /* SYSCFG peripheral mode configuration register            Address offset 0x04                                         */
     __vo uint32_t EXTICR[4];                     /* SYSCFG external interrupt configuration register 1       Address offset 0x08                                         */
     __vo uint32_t Reserved[2];
     __vo uint32_t CMPCR;                         /* Compensation cell control register                       Address offset 0x20                                         */

}SYSCFG_RegDef_t;

typedef struct
{
    /* data */
    __vo uint32_t CR;                               /* RCC clock control register                                                       Address offset 0x00 */
    __vo uint32_t PLLCFGR;                          /* RCC PLL configuration register                                                   Address offset 0x04 */
    __vo uint32_t CFGR;                             /* RCC clock configuration register                                                 Address offset 0x08 */
    __vo uint32_t CIR;                              /* RCC clock interrupt register                                                     Address offset 0x0C */
    __vo uint32_t AHB1RSTR;                         /* RCC AHB1 peripheral reset register                                               Address offset 0x10 */
    __vo uint32_t AHB2RSTR;                         /* RCC AHB2 peripheral reset register                                               Address offset 0x14 */
    __vo uint32_t AHB3RSTR;                         /* RCC AHB3 peripheral reset register                                               Address offset 0x18 */
         uint32_t Reserved;
    __vo uint32_t APB1RSTR;                         /* RCC APB1 peripheral reset register                                               Address offset 0x20 */
    __vo uint32_t APB2RSTR;                         /* RCC APB2 peripheral reset register                                               Address offset 0x24 */
         uint32_t Reserved1;
         uint32_t Reserved2;
    __vo uint32_t AHB1ENR;                          /* RCC AHB1 peripheral clock enable register                                        Address offset 0x30 */
    __vo uint32_t AHB2ENR;                          /* RCC AHB2 peripheral clock enable register                                        Address offset 0x34 */
    __vo uint32_t AHB3ENR;                          /* RCC AHB3 peripheral clock enable register                                        Address offset 0x38 */
         uint32_t Reserved3;
    __vo uint32_t APB1ENR;                          /* RCC APB1 peripheral clock enable register                                        Address offset 0x40 */
    __vo uint32_t APB2ENR;                          /* RCC APB2 peripheral clock enable register                                        Address offset 0x44 */
         uint32_t Reserved4;
         uint32_t Reserved5;
    __vo uint32_t AHB1LPENR;                        /* RCC AHB1 peripheral clock enable in low power mode register                      Address offset 0x50 */
    __vo uint32_t AHB2LPENR;                        /* RCC AHB2 peripheral clock enable in low power mode register                      Address offset 0x54 */
    __vo uint32_t AHB3LPENR;                        /* RCC AHB3 peripheral clock enable in low power mode register                      Address offset 0x58 */
         uint32_t Reserved6;
    __vo uint32_t APB1LPENR;                        /* RCC APB1 peripheral clock enable in low power mode register                      Address offset 0x60 */
    __vo uint32_t APB2LPENR;                        /* RCC APB2 peripheral clock enabled in low power mode                              Address offset 0x64 */
         uint32_t Reserved7;
         uint32_t Reserved8;
    __vo uint32_t BDCR;                             /* RCC Backup domain control register                                               Address offset 0x70 */
    __vo uint32_t CSR;                              /* RCC clock control & status register                                              Address offset 0x74 */
         uint32_t Reserved9;
         uint32_t Reserved10;
    __vo uint32_t SSCGR;                            /* RCC spread spectrum clock generation register                                    Address offset 0x80 */
    __vo uint32_t PLLI2SCFGR;                       /* RCC PLLI2S configuration register                                                Address offset 0x84 */

}RCC_RegDef_t;

typedef struct
{
     /* data */
     __vo uint32_t IMR;                                /* Interrupt mask register                                                       Address offset 0x00 */
     __vo uint32_t EMR;                                /* Event mask register                                                           Address offset 0x04 */
     __vo uint32_t RTSR;                               /* Rising trigger selection register                                             Address offset 0x08 */
     __vo uint32_t FTSR;                               /* Falling trigger selection register                                            Address offset 0x0C */
     __vo uint32_t SWIER;                              /* Software interrupt event register                                             Address offset 0x10 */
     __vo uint32_t PR;                                 /* Pending register                                                              Address offset 0x14 */

}EXTI_RegDef_t;


/* processor peripherals */

#define NVIC                        ((__vo NVIC_RegDef_t*)NVIC_BASEADDR)

/* peripherals definition */

#define GPIOA                       ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB                       ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC                       ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD                       ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE                       ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF                       ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG                       ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH                       ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI                       ((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ                       ((GPIO_RegDef_t*)GPIOJ_BASEADDR)


#define SPI1						((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2						((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3						((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4						((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5						((SPI_RegDef_t*)SPI5_BASEADDR)
#define SPI6						((SPI_RegDef_t*)SPI6_BASEADDR)


#define I2C1                            ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2                            ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3                            ((I2C_RegDef_t*)I2C3_BASEADDR)



#define RCC                         ((RCC_RegDef_t*)RRC_BASEADDR)
#define EXTI                        ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG                      ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/* Reset GPIOx peripherals */

#define GPIOA_REG_RESET()          do { RCC->AHB1RSTR |= (1<<0); RCC->AHB1RSTR &= ~(1<<0); } while(0)
#define GPIOB_REG_RESET()          do { RCC->AHB1RSTR |= (1<<1); RCC->AHB1RSTR &= ~(1<<1); } while(0)
#define GPIOC_REG_RESET()          do { RCC->AHB1RSTR |= (1<<2); RCC->AHB1RSTR &= ~(1<<2); } while(0)
#define GPIOD_REG_RESET()          do { RCC->AHB1RSTR |= (1<<3); RCC->AHB1RSTR &= ~(1<<3); } while(0)
#define GPIOE_REG_RESET()          do { RCC->AHB1RSTR |= (1<<4); RCC->AHB1RSTR &= ~(1<<4); } while(0)
#define GPIOF_REG_RESET()          do { RCC->AHB1RSTR |= (1<<5); RCC->AHB1RSTR &= ~(1<<5); } while(0)
#define GPIOG_REG_RESET()          do { RCC->AHB1RSTR |= (1<<6); RCC->AHB1RSTR &= ~(1<<6); } while(0)
#define GPIOH_REG_RESET()          do { RCC->AHB1RSTR |= (1<<7); RCC->AHB1RSTR &= ~(1<<7); } while(0)
#define GPIOI_REG_RESET()          do { RCC->AHB1RSTR |= (1<<8); RCC->AHB1RSTR &= ~(1<<8); } while(0)
#define GPIOJ_REG_RESET()          do { RCC->AHB1RSTR |= (1<<9); RCC->AHB1RSTR &= ~(1<<9); } while(0)


/* Clock enable macros for GPIOx peripherals */

#define GPIOA_PCLK_EN()         (RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()         (RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()         (RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()         (RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()         (RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()         (RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()         (RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()         (RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()         (RCC->AHB1ENR |= (1<<8))
#define GPIOJ_PCLK_EN()         (RCC->AHB1ENR |= (1<<9))
#define GPIOK_PCLK_EN()         (RCC->AHB1ENR |= (1<<10))

/* Clock enable macros for I2C peripherals */

#define I2C1_PCLK_EN()          (RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()          (RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()          (RCC->APB1ENR |= (1<<23))

/* Clock enable macros for SPIx peripherals */

#define SPI1_PCLK_EN()          (RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()          (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()          (RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()          (RCC->APB2ENR |= (1<<13))
#define SPI5_PCLK_EN()          (RCC->APB2ENR |= (1<<20))
#define SPI6_PCLK_EN()          (RCC->APB2ENR |= (1<<21))

/* Clock enable macros for USARTx peripherals*/

#define USART2_PCLK_EN()         (RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()         (RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN()          (RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()          (RCC->APB1ENR |= (1<<20))

/* Clcok enable macros for SYSCFG peripherals */

#define SYSCFG_PCLK_EN()         (RCC->APB2ENR |= (1<<14))

/* Clock disable macros for GPIOx peripherals */

#define GPIOA_PCLK_DI()          (RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()          (RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()          (RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()          (RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()          (RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()          (RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()          (RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()          (RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()          (RCC->AHB1ENR &= ~(1<<8))
#define GPIOJ_PCLK_DI()          (RCC->AHB1ENR &= ~(1<<9))
#define GPIOK_PCLK_DI()          (RCC->AHB1ENR &= ~(1<<10))



/* Clock disable macros for I2Cx peripherals */

#define I2C1_PCLK_DI()          (RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()          (RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()          (RCC->APB1ENR &= ~(1<<23))

/* Clock disable macros for SPIx peripherals */

#define SPI1_PCLK_DI()          (RCC->APB2RSTR &= ~(1<<12))
#define SPI2_PCLK_DI()          (RCC->APB1RSTR &= ~(1<<14))
#define SPI3_PCLK_DI()          (RCC->APB1RSTR &= ~(1<<15))
#define SPI4_PCLK_DI()          (RCC->APB2RSTR &= ~(1<<13))
#define SPI5_PCLK_DI()          (RCC->APB2RSTR &= ~(1<<20))
#define SPI6_PCLK_DI()          (RCC->APB2RSTR &= ~(1<<21))

/* Clock disable macros for USARTx peripherals */



/* Clock disable macros for SYSCFG peripherals */


#define ENABLE 						1
#define DISABLE 					0
#define SET 						ENABLE
#define RESET 						DISABLE
#define GPIO_PIN_SET 				1
#define GPIO_PIN_RESET 				0


/* Interupt request of microcontroller */

#define IRQ_NO_EXTI0 				6
#define IRQ_NO_EXTI1 				7
#define IRQ_NO_EXTI2 				8
#define IRQ_NO_EXTI3 				9
#define IRQ_NO_EXTI4 				10
#define IRQ_NO_EXTI9_5 				23
#define IRQ_NO_EXTI15_10 			40
#define IRQ_NO_SPI2					36
#define IRQ_NO_SPI1					35
#define IRQ_NO_SPI3					51


#define IMPLEMENTED_BITS 			4

/*macro for SPI peripheral*/

/*SPI device mode*/
#define SPI_DEVICE_MASTER 			1
#define SPI_DEVICE_SLAVE 			0
/*SPI bus config*/

#define SPI_BUS_FULL_DUPLEX			0
#define SPI_BUS_HAFL_DUPLEX			1
#define SPI_BUS_SIMPLEX_RX_ONLY		2

/*SPI Clock Speed*/

#define SPI_CLOCK_DIV2				0
#define SPI_CLOCK_DIV4				1
#define SPI_CLOCK_DIV8				2
#define SPI_CLOCK_DIV16				3
#define SPI_CLOCK_DIV32				4
#define SPI_CLOCK_DIV64				5
#define SPI_CLOCK_DIV128			6
#define SPI_CLOCK_DIV256			7

/*SPI data frame*/

#define SPI_DFF_8BITS 				0
#define SPI_DFF_16BITS				1

/*SPI CPOL and CPHA*/


#define SPI_CPOL_HIGH				1
#define SPI_CPOL_LOW				0
#define SPI_CPHA_HIGH				1
#define SPI_CPHA_LOW				0

/*SPI software management*/

#define SPI_SSM_EN					1
#define SPI_SSM_DI					0

/* Bit position in SPI_CR1*/

#define CPHA_BIT_POSITION			0
#define CPOL_BIT_POSITION			1
#define MSRT_BIT_POSITION			2
#define BR_BIT_POSITION				3
#define SPE_BIT_POSITION			6
#define LSB_FIRST_BIT_POSITION		7
#define SSI_BIT_POSITION			8
#define SSM_BIT_POSITION			9
#define RX_ONLY_BIT_POSITION		10
#define DFF_BIT_POSITION			11
#define CRC_NEXT_BIT_POSITION		12
#define CRC_EN_BIT_POSITION			13
#define BIDI_OE_BIT_POSITION		14
#define BIDI_MODE_BIT_POSITION		15

/*Bit position in SPI_CR2*/
#define RXDMAEN_BIT_POSITION		0
#define TXDMAEN_BIT_POSITIOn		1
#define SSOE_BIT_POSIITON			2
#define FRF_BIT_POSITION			4
#define ERRIE_BIT_POSITION			5
#define RXNEIE_BIT_POSITION			6
#define TXEIE_BIT_POSITION			7

/*Bit position in SPI_SR*/

#define RXNE_BIT_POSITION			0
#define TXE_BIT_POSITION			1
#define CHSIDE_BIT_POSITION			2
#define UDR_BIT_POSITION			3
#define CRCERR_BIT_POSITION			4
#define MODF_BIT_POSITION			5
#define SPI_OVR_BIT_POSITION		6
#define BSY_BIT_POSITION			7
#define FRE_BIT_POSITIOn			8


/* macro for I2C peripherals */

/* bit position I2C_CR1 */
#define PE_BIT_POSITION  0
#define SMBUS_MODE_BIT_POSITION 1
#define SMBUS_TYPE_BIT_POSITION 3
#define ARP_BIT_POSITION 4
#define ENPEC_BIT_POSITION 5
#define ENGC_BIT_POSITION 6
#define NOSTRETCH_BIT_POSITION 7
#define START_BIT_POSITION 8
#define STOP_BIT_POSITION 9
#define ACK_BIT_POSITION 10
#define POS_BIT_POSITION 11
#define SPI_PEC_BIT_POSITION 12
#define ALERT_BIT_POSITION 13
#define SWRST_BIT_POSITION 15

/* bit position for I2C_CR2 */
#define FREQ_BIT_POSITION 0
#define ITERREN_BIT_POSITION 8
#define ITEVTEN_BIT_POSITION 9
#define ITBUFEN_BIT_POSITION 10
#define DMAEN_BIT_POSITION 11
#define LAST_BIT_POSITION 12

/* bit position for I2C_OAR1 */
#define ADD0_BIT_POSITION 0
#define ADD_BIT_POSITION 1
#define ADD_98_BIT_POSITION 8
#define ADDMODE_BIT_POSITION 15

/* bit position for I2C_SR1 */
#define SB_BIT_POSITION 0
#define ADDR_BIT_POSITION 1
#define BTF_BIT_POSITION 2
#define ADD10_BIT_POSITION 3
#define STOPF_BIT_POSITION 4

#define RxNE_BIT_POSITION 6
#define TxE_BIT_POSITION 7
#define BERR_BIT_POSITION 8
#define ARLO_BIT_POSITION 9
#define AF_BIT_POSITION 10
#define I2C_OVR_BIT_POSITION 11
#define PECERR_BIT_POSITION 12
#define TIMEOUT_BIT_POSITION 14
#define SMBALERT_BIT_POSITION 15

/* bit position for I2C_SR2 */
#define MSL_BIT_POSITION 0
#define BUSY_BIT_POSITION 1
#define TRA_BIT_POSITION 2
#define GENCALL_BIT_POSITION 4
#define SMBDEFAULT_BIT_POSITION 5
#define SMBHOST_BIT_POSITION 6
#define DUALF_BIT_POSITION 7
#define I2C_PEC_BIT_POSITION 8

/* bit position for I2C_CCR */

#define CCR_BIT_POSITION 0
#define DUTY_BIT_POSITION 14
#define FS_BIT_POSITION 15

/*other macro*/
#define EMPTY 						0
//__attribute((weak))void __SPI_ApplicationCallback(SPI_Handle_t* pSPI_handle,uint8_t EventNumber);
#endif
