/*
 * stm32f334xx.h
 *
 *  Created on: Mar 30, 2024
 *      Author: ardau
 */
#ifndef INC_STM32F334XX_H_
#define INC_STM32F334XX_H_

#include <stdint.h>
#define __vo volatile
/***********************************************START:Processor Specific Details**********************************************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx Register Addresses
*/
#define NVIC_ISER0                                      ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1                                      ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2                                      ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3                                      ( (__vo uint32_t*)0xE000E10C )
#define NVIC_ISER4                                      ( (__vo uint32_t*)0xE000E110 )
#define NVIC_ISER5                                      ( (__vo uint32_t*)0xE000E114 )
#define NVIC_ISER6                                      ( (__vo uint32_t*)0xE000E118 )
#define NVIC_ISER7                                      ( (__vo uint32_t*)0xE000E11C )

/*
 * ARM Cortex Mx Processor NVIC ICERx Register Addresses
*/
#define NVIC_ICER0                                      ( (__vo uint32_t*)0XE000E180 )
#define NVIC_ICER1                                      ( (__vo uint32_t*)0XE000E184 )
#define NVIC_ICER2                                      ( (__vo uint32_t*)0XE000E188 )
#define NVIC_ICER3                                      ( (__vo uint32_t*)0XE000E18C )
#define NVIC_ICER4                                      ( (__vo uint32_t*)0XE000E190 )
#define NVIC_ICER5                                      ( (__vo uint32_t*)0XE000E194 )
#define NVIC_ICER6                                      ( (__vo uint32_t*)0XE000E198 )
#define NVIC_ICER7                                      ( (__vo uint32_t*)0XE000E19C )

/*
 * ARM Cortex Mx Processor Priority Register Base Address
*/
#define NVIC_PS_BASEADDR                                ( (__vo uint32_t*)0xE000E400 )

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
*/
#define NO_PR_BITS_IMPLEMENTED                          4

/*
 * Memory Peripherals
*/
#define FLASH_BASEADDR                                  0x08000000U                          // FLASH BASE address
#define SRAM1_BASEADDR                                  0x20000000U                          // SRAM base address
#define SRAM2_BASEADDR                                  0x20003000U                          // 12 kB after SRAM1_BASEADDR = 12*1024 = 12288 or 3000 in HEX
#define ROM                                             0x1FFFD800U                          // Also called system memory
#define SRAM                                            SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
*/
#define PERIPH_BASEADDR                                 0x40000000U
#define APB1PERIPH_BASEADDR                             PERIPH_BASEADDR 
#define APB2PERIPH_BASEADDR                             0x40010000U
#define AHB1PERIPH_BASEADDR                             0x40020000U
#define AHB2PERIPH_BASEADDR                             0x48000000U
#define AHB3PERIPH_BASEADDR                             0x50000000U

/*
 * Base addresses of peripherals which are hanging on APB1 bus
*/
#define TIM2_BASEADDR                                   (APB1PERIPH_BASEADDR + 0x0000)
#define TIM3_BASEADDR                                   (APB1PERIPH_BASEADDR + 0x0400)
#define TIM6_BASEADDR                                   (APB1PERIPH_BASEADDR + 0x1000)
#define TIM7_BASEADDR                                   (APB1PERIPH_BASEADDR + 0x1400)
#define RTC_BASEADDR                                    (APB1PERIPH_BASEADDR + 0x2800)
#define WWDOG_BASEADDR                                  (APB1PERIPH_BASEADDR + 0x2C00)
#define IWDOG_BASEADDR                                  (APB1PERIPH_BASEADDR + 0x3000)
#define USART2_BASEADDR                                 (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR                                 (APB1PERIPH_BASEADDR + 0x4800)
#define I2C1_BASEADDR                                   (APB1PERIPH_BASEADDR + 0x5400)
#define bxCAN_BASEADDR                                  (APB1PERIPH_BASEADDR + 0x6400)
#define PWR_BASEADDR                                    (APB1PERIPH_BASEADDR + 0x7000)
#define DAC1_BASEADDR                                   (APB1PERIPH_BASEADDR + 0x7400)
#define DAC2_BASEADDR                                   (APB1PERIPH_BASEADDR + 0x9800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
*/
#define SYSCFG_BASEADDR                                 (APB2PERIPH_BASEADDR + 0x0000)
#define COMP_BASEADDR                                   (APB2PERIPH_BASEADDR + 0x0020)
#define OPAMP_BASEADDR                                  (APB2PERIPH_BASEADDR + 0x003C)
#define EXTI_BASEADDR                                   (APB2PERIPH_BASEADDR + 0x0400)
#define TIM1_BASEADDR                                   (APB2PERIPH_BASEADDR + 0x2C00)
#define SPI1_BASEADDR                                   (APB2PERIPH_BASEADDR + 0x3000)
#define USART1_BASEADDR                                 (APB2PERIPH_BASEADDR + 0x3800)
#define TIM15_BASEADDR                                  (APB2PERIPH_BASEADDR + 0x4000)
#define TIM16_BASEADDR                                  (APB2PERIPH_BASEADDR + 0x4400)
#define TIM17_BASEADDR                                  (APB2PERIPH_BASEADDR + 0x4800)
#define HRTIM1_BASEADDR                                 (APB2PERIPH_BASEADDR + 0x7400)

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
*/
#define DMA1_BASEADDR                                   (AHB1PERIPH_BASEADDR + 0x0000)
#define RCC_BASEADDR                                    (AHB1PERIPH_BASEADDR + 0x1000)
#define FLASH_INTERFACE_BASEADDR                        (AHB1PERIPH_BASEADDR + 0x2000)
#define CRC_BASEADDR                                    (AHB1PERIPH_BASEADDR + 0x3000)
#define TSC_BASEADDR                                    (AHB1PERIPH_BASEADDR + 0x4000)

/*
 * Base addresses of peripherals which are hanging on AHB2 bus
*/
#define GPIOA_BASEADDR                                  (AHB2PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                                  (AHB2PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR                                  (AHB2PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR                                  (AHB2PERIPH_BASEADDR + 0x0C00)
#define GPIOF_BASEADDR                                  (AHB2PERIPH_BASEADDR + 0x1400)

/*
 * Base addresses of peripherals which are hanging on AHB2 bus
*/
#define ADC1_BASEADDR                                   (AHB3PERIPH_BASEADDR + 0x0000)
#define ADC2_BASEADDR                                   (AHB3PERIPH_BASEADDR + 0x0100)

/************************************** Peripheral Register Definition Structures **********************************************************/

typedef struct 
{
    __vo uint32_t MODER;                                                                    // GPIO port mode register                         Address offset: 0x00
    __vo uint32_t OTYPER;                                                                   // GPIO port output type register                  Address offset: 0x04
    __vo uint32_t OSPEEDR;                                                                  // GPIO port output speed register                 Address offset: 0x08
    __vo uint32_t PUPDR;                                                                    // GPIO port pull-up/pull-down register            Address offset: 0x0C
    __vo uint32_t IDR;                                                                      // GPIO port input data register                   Address offset: 0x10
    __vo uint32_t ODR;                                                                      // GPIO port output data register                  Address offset: 0x14
    __vo uint32_t BSRR;                                                                     // GPIO port bit set/reset register                Address offset: 0x18
    __vo uint32_t LCKR;                                                                     // GPIO port configuration lock register           Address offset: 0x1C
    __vo uint32_t AFR[2];                                                                   // AFR[0]: GPIO alternate function low register    Address offset: 0x20 
                                                                                            // AFR[1]: GPIO alternate function High register   Address offset: 0x24
    __vo uint32_t BRR;                                                                      // GPIO port bit reset register                    Address offset: 0x28
    
}GPIO_RegDef_t; 

typedef struct 
{
    __vo uint32_t RCC_CR;                                                                    // Clock control register                          Address offset: 0x00
    __vo uint32_t RCC_CFGR;                                                                  // Clock configuration register                    Address offset: 0x04
    __vo uint32_t RCC_CIR;                                                                   // Clock interrupt register                        Address offset: 0x08
    __vo uint32_t RCC_APB2RSTR;                                                              // APB2 peripheral reset register                  Address offset: 0x0C
    __vo uint32_t RCC_APB1RSTR;                                                              // APB1 peripheral reset register                  Address offset: 0x10
    __vo uint32_t RCC_AHBENR;                                                                // AHB peripheral clock enable register            Address offset: 0x14
    __vo uint32_t RCC_APB2ENR;                                                               // APB2 peripheral clock enable register           Address offset: 0x18
    __vo uint32_t RCC_APB1ENR;                                                               // APB1 peripheral clock enable register           Address offset: 0x1C
    __vo uint32_t RCC_BDCR;                                                                  // RTC domain control register                     Address offset: 0x20
    __vo uint32_t RCC_CSR;                                                                   // Control/status register                         Address offset: 0x24
    __vo uint32_t RCC_AHBRSTR;                                                               // AHB peripheral reset register                   Address offset: 0x28
    __vo uint32_t RCC_CFGR2;                                                                 // Clock configuration register 2                  Address offset: 0x2C 
    __vo uint32_t RCC_CFGR3;                                                                 // Clock configuration register 3                  Address offset: 0x30 

}RCC_RegDef_t;

typedef struct
{
    __vo uint32_t EXTI_IMR1;                                                                 // Interrupt mask register                         Address offset: 0x00
    __vo uint32_t EXTI_EMR1;                                                                 // Event mask register                             Address offset: 0x04
    __vo uint32_t EXTI_RTSR1;                                                                // Rising trigger selection register               Address offset: 0x08
    __vo uint32_t EXTI_FTSR1;                                                                // Falling trigger selection register              Address offset: 0x0C
    __vo uint32_t EXTI_SWIER1;                                                               // Software interrupt event register               Address offset: 0x10
    __vo uint32_t EXTI_PR1;                                                                  // Pending register                                Address offset: 0x14
    __vo uint32_t EXTI_IMR2;                                                                 // Interrupt mask register 2                       Address offset: 0x20
    __vo uint32_t EXTI_EMR2;                                                                 // Event mask register 2                           Address offset: 0x24
    __vo uint32_t EXTI_RTSR2;                                                                // Rising trigger selection register 2             Address offset: 0x28
    __vo uint32_t EXTI_FTSR2;                                                                // Falling trigger selection register 2            Address offset: 0x2C
    __vo uint32_t EXTI_SWIER2;                                                               // Software interrupt event register 2             Address offset: 0x30
    __vo uint32_t EXTI_PR2;                                                                  // Pending register 2                              Address offset: 0x34

}EXTI_RegDef_t;

typedef struct
{
    __vo uint32_t SYSCFG_CFGR1;                                                               // SYSCFG configuration register 1                Address offset: 0x00
    __vo uint32_t SYSCFG_RCR;                                                                 // SYSCFG CCM SRAM protection register            Address offset: 0x04
    __vo uint32_t SYSCFG_EXTICR[4];                                                           // SYSCFG external interrupt config reg 1         Address offset: 0x08
                                                                                              // SYSCFG external interrupt config reg 2         Address offset: 0x0C
                                                                                              // SYSCFG external interrupt config reg 3         Address offset: 0x10
                                                                                              // SYSCFG external interrupt config reg 4         Address offset: 0x14
    __vo uint32_t SYSCFG_CFGR2;                                                               // SYSCFG configuration register 2                Address offset: 0x18
    __vo uint32_t reserved[12];                                                               // RESERVED REGISTERS                             Address offset: 0x1C-0X4C
    __vo uint32_t SYSCFG_CFGR3;                                                               // SYSCFG configuration register 3                Address offset: 0x50

}SYSCFG_RegDef_t;

/*
 * Peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
*/

#define GPIOA                                           ( (GPIO_RegDef_t*)   GPIOA_BASEADDR  )
#define GPIOB                                           ( (GPIO_RegDef_t*)   GPIOB_BASEADDR  )
#define GPIOC                                           ( (GPIO_RegDef_t*)   GPIOC_BASEADDR  )
#define GPIOD                                           ( (GPIO_RegDef_t*)   GPIOD_BASEADDR  )
#define GPIOF                                           ( (GPIO_RegDef_t*)   GPIOF_BASEADDR  )
#define RCC                                             ( (RCC_RegDef_t*)    RCC_BASEADDR    )
#define EXTI                                            ( (EXTI_RegDef_t*)   EXTI_BASEADDR   )
#define SYSCFG                                          ( (SYSCFG_RegDef_t*) SYSCFG_BASEADDR )

/*
 * Clock Enable Macros For GPIOx Peripherals
*/
#define GPIOA_PCLK_EN()                                 ( RCC->RCC_AHBENR |= ( 1 << 17 ) )  // I/O port A clock enable
#define GPIOB_PCLK_EN()                                 ( RCC->RCC_AHBENR |= ( 1 << 18 ) )  // I/O port B clock enable
#define GPIOC_PCLK_EN()                                 ( RCC->RCC_AHBENR |= ( 1 << 19 ) )  // I/O port C clock enable
#define GPIOD_PCLK_EN()                                 ( RCC->RCC_AHBENR |= ( 1 << 20 ) )  // I/O port D clock enable
#define GPIOF_PCLK_EN()                                 ( RCC->RCC_AHBENR |= ( 1 << 22 ) )  // I/O port F clock enable

/*
 * Clock Enable Macros For I2Cx Peripherals
*/
#define I2C1_PCLK_EN()                                  ( RCC->RCC_APB1ENR |= ( 1 << 21 ) )  // I2C1 clock enable 

/*
 * Clock Enable Macros For SPIx Peripherals
*/
#define SPI1_PCLK_EN()                                  ( RCC->RCC_APB2ENR |= ( 1 << 12 ) )  // SPI1 clock enable 

/*
 * Clock Enable Macros For USARTx Peripherals
*/
#define USART1_PCLK_EN()                                ( RCC->RCC_APB2ENR |= ( 1 << 14 ) )  // USART1 clock enable 
#define USART2_PCLK_EN()                                ( RCC->RCC_APB1ENR |= ( 1 << 17 ) )  // USART2 clock enable
#define USART3_PCLK_EN()                                ( RCC->RCC_APB1ENR |= ( 1 << 18 ) )  // USART3 clock enable

/*
 * Clock Enable Macros For SYSCFG Peripherals
*/
#define SYSCFG_PCLK_EN()                                ( RCC->RCC_APB2ENR |= ( 1 << 0 ) )   // SYSCFG clock enable 

/*
 * Clock Disable Macros for GPIOx Peripherals
*/
#define GPIOA_PCLK_DI()                                 ( RCC->RCC_AHBENR &= ~( 1 << 17 ) )  // I/O port A clock disable
#define GPIOB_PCLK_DI()                                 ( RCC->RCC_AHBENR &= ~( 1 << 18 ) )  // I/O port B clock disable
#define GPIOC_PCLK_DI()                                 ( RCC->RCC_AHBENR &= ~( 1 << 19 ) )  // I/O port C clock disable
#define GPIOD_PCLK_DI()                                 ( RCC->RCC_AHBENR &= ~( 1 << 20 ) )  // I/O port D clock disable
#define GPIOF_PCLK_DI()                                 ( RCC->RCC_AHBENR &= ~( 1 << 22 ) )  // I/O port F clock disable

/*
 * Clock Disable Macros for I2Cx Peripherals
*/
#define I2C1_PCLK_DI()                                  ( RCC->RCC_APB1ENR &= ~( 1 << 21 ) ) // I2C1 clock disable 

/*
 * Clock Disable Macros for SPIx Peripherals
*/
#define SPI1_PCLK_DI()                                  ( RCC->RCC_APB2ENR &= ~( 1 << 12 ) )  // SPI1 clock disable 

/*
 * Clock Disable Macros For USARTx Peripherals
*/
#define USART1_PCLK_DI()                                ( RCC->RCC_APB2ENR &= ~( 1 << 14 ) )  // USART1 clock disable 
#define USART2_PCLK_DI()                                ( RCC->RCC_APB1ENR &= ~( 1 << 17 ) )  // USART2 clock disable
#define USART3_PCLK_DI()                                ( RCC->RCC_APB1ENR &= ~( 1 << 18 ) )  // USART3 clock disable

/*
 * Clock Enable Macros For SYSCFG Peripherals
*/
#define SYSCFG_PCLK_DI()                                ( RCC->RCC_APB2ENR &= ~( 1 << 0 ) )   // SYSCFG clock disable 

/*
 * Macros to reset GPIOx peripherals
*/
#define GPIOA_REG_RESET()                               do{ ( RCC->RCC_AHBRSTR |= 1<<17 ); ( RCC->RCC_AHBRSTR &= ~(1<<17) ); }while(0) 
#define GPIOB_REG_RESET()                               do{ ( RCC->RCC_AHBRSTR |= 1<<18 ); ( RCC->RCC_AHBRSTR &= ~(1<<18) ); }while(0) 
#define GPIOC_REG_RESET()                               do{ ( RCC->RCC_AHBRSTR |= 1<<19 ); ( RCC->RCC_AHBRSTR &= ~(1<<19) ); }while(0) 
#define GPIOD_REG_RESET()                               do{ ( RCC->RCC_AHBRSTR |= 1<<20 ); ( RCC->RCC_AHBRSTR &= ~(1<<20) ); }while(0) 
#define GPIOF_REG_RESET()                               do{ ( RCC->RCC_AHBRSTR |= 1<<22 ); ( RCC->RCC_AHBRSTR &= ~(1<<22) ); }while(0) 

/*
 * Returns port code for given GPIOx base address 
*/
#define GPIOA_BASEADDR_TO_CODE(x)                       ( (x == GPIOA)?0:\
														  (x == GPIOB)?1:\
                                                          (x == GPIOC)?2:\
                                                          (x == GPIOD)?3:\
                                                          (x == GPIOF)?5:0 )

/*
 * IRQ (Interrupt Request) Numbers of STM32F334x MCU
*/
#define IRQ_NO_EXTI0            6
#define IRQ_NO_EXTI1            7
#define IRQ_NO_EXTI2            8
#define IRQ_NO_EXTI3            9
#define IRQ_NO_EXTI4            10
#define IRQ_NO_EXTI9_5          23
#define IRQ_NO_EXTI15_10        40

/*
 * Macros for the all the possible priority levels
*/
#define NVIC_IRQ_PRIO1          1
#define NVIC_IRQ_PRIO2          2
#define NVIC_IRQ_PRIO3          3
#define NVIC_IRQ_PRIO4          4
#define NVIC_IRQ_PRIO5          5
#define NVIC_IRQ_PRIO6          6
#define NVIC_IRQ_PRIO7          7
#define NVIC_IRQ_PRIO8          8
#define NVIC_IRQ_PRIO9          9
#define NVIC_IRQ_PRIO10         10
#define NVIC_IRQ_PRIO11         11
#define NVIC_IRQ_PRIO12         12
#define NVIC_IRQ_PRIO13         13
#define NVIC_IRQ_PRIO14         14
#define NVIC_IRQ_PRIO15         15


// Some Generic Macros
#define ENABLE                  1
#define DISABLE                 0
#define SET                     ENABLE
#define RESET                   DISABLE
#define GPIO_PIN_SET            SET
#define GPIO_PIN_RESET          RESET

#include "stm32f334xx_gpio_driver.h"

#endif  //INC_STM32F334XX_H_ 
