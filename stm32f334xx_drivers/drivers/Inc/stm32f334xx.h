/*
 * stm32f334xx.h
 *
 *  Created on: Mar 30, 2024
 *      Author: ardau
 */
#ifndef INC_STM32F334XX_H_
#define INC_STM32F334XX_H_

#include <stdint.h>
#include <stddef.h>

#define __vo volatile
#define __weak __attribute__((weak))
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

typedef struct
{
    __vo uint32_t SPIx_CR1;                                                                   // SPI control register 1                         Address offset: 0x00
    __vo uint32_t SPIx_CR2;                                                                   // SPI control register 2                         Address offset: 0x04
    __vo uint32_t SPIx_SR;                                                                    // SPI status register                            Address offset: 0x08
    __vo uint32_t SPIx_DR;                                                                    // SPI data register                              Address offset: 0x0C
    __vo uint32_t SPIx_CRCPR;                                                                 // SPI CRC polynomial register                    Address offset: 0x10
    __vo uint32_t SPIx_RXCRCR;                                                                // SPI Rx CRC register                            Address offset: 0x14
    __vo uint32_t SPIx_TXCRCR;                                                                // SPI Tx CRC register                            Address offset: 0x18

}SPI_RegDef_t;

typedef struct
{
    __vo uint32_t I2CX_CR1;                                                                   // I2C control register 1                         Address offset: 0x00
    __vo uint32_t I2CX_CR2;                                                                   // I2C control register 2                         Address offset: 0x04
    __vo uint32_t I2CX_OAR1;                                                                  // I2C own address 1 register                     Address offset: 0x08
    __vo uint32_t I2CX_OAR2;                                                                  // I2C own address 2 register                     Address offset: 0x0C
    __vo uint32_t I2CX_TIMINGR;                                                               // I2C timing register                            Address offset: 0x10
    __vo uint32_t I2CX_TIMEOUTR;                                                              // I2C timeout register                           Address offset: 0x14
    __vo uint32_t I2CX_ISR;                                                                   // I2C interrupt and status register              Address offset: 0x18
    __vo uint32_t I2CX_ICR;                                                                   // I2C interrupt clear register                   Address offset: 0x1C
    __vo uint32_t I2CX_PECR;                                                                  // I2C PEC register                               Address offset: 0x20
    __vo uint32_t I2CX_RXDR;                                                                  // I2C receive data register                      Address offset: 0x24
    __vo uint32_t I2CX_TXDR;                                                                  // I2C transmit data register                     Address offset: 0x28

}I2C_RegDef_t;

typedef struct
{
    __vo uint32_t USART_CR1;                                                                  // USART control register 1                       Address offset: 0x00
    __vo uint32_t USART_CR2;                                                                  // USART control register 2                       Address offset: 0x04
    __vo uint32_t USART_CR3;                                                                  // USART control register 3                       Address offset: 0x08
    __vo uint32_t USART_BRR;                                                                  // USART baud rate register                       Address offset: 0x0C
    __vo uint32_t USART_GTPR;                                                                 // USART guard time and prescaler register        Address offset: 0x10
    __vo uint32_t USART_RTOR;                                                                 // USART receiver timeout register                Address offset: 0x14
    __vo uint32_t USART_RQR;                                                                  // USART request register                         Address offset: 0x18
    __vo uint32_t USART_ISR;                                                                  // USART interrupt and status register            Address offset: 0x1C
    __vo uint32_t USART_ICR;                                                                  // USART interrupt flag clear register            Address offset: 0x20
    __vo uint32_t USART_RDR;                                                                  // USART receive data register                    Address offset: 0x24
    __vo uint32_t USART_TDR;                                                                  // USART transmit data register                   Address offset: 0x28

}USART_RegDef_t;

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
#define SPI1                                            ( (SPI_RegDef_t*)    SPI1_BASEADDR   )
#define I2C1                                            ( (I2C_RegDef_t*)    I2C1_BASEADDR   )
#define USART1                                          ( (USART_RegDef_t*)  USART1_BASEADDR )
#define USART2                                          ( (USART_RegDef_t*)  USART2_BASEADDR )
#define USART3                                          ( (USART_RegDef_t*)  USART3_BASEADDR )

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
#define IRQ_NO_SPI1             35

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
#define FLAG_RESET              RESET
#define FLAG_SET                SET

/***************************************************************************
 * Bit position definitions of SPI Peripheral
****************************************************************************/
/*
 * Bit position definitions SPI_CR1
*/
#define SPI_CR1_CPHA            0
#define SPI_CR1_CPOL            1
#define SPI_CR1_MSTR            2
#define SPI_CR1_BR              3
#define SPI_CR1_SPE             6
#define SPI_CR1_LSBFIRST        7
#define SPI_CR1_SSI             8
#define SPI_CR1_SSM             9
#define SPI_CR1_RXONLY          10
#define SPI_CR1_CRCL            11
#define SPI_CR1_CRCNEXT         12
#define SPI_CR1_CRCEN           13
#define SPI_CR1_BIDIOE          14
#define SPI_CR1_BIDIMODE        15

/*
 * Bit position definitions SPI_CR2
*/
#define SPI_CR2_RXDMAEN         0
#define SPI_CR2_TXDMAEN         1
#define SPI_CR2_SSOE            2
#define SPI_CR2_NSSP            3
#define SPI_CR2_FRF             4
#define SPI_CR2_ERRIE           5
#define SPI_CR2_RXNEIE          6
#define SPI_CR2_TXEIE           7
#define SPI_CR2_DS              8
#define SPI_CR2_FRXTH           12
#define SPI_CR2_LDMA_RX         13
#define SPI_CR2_LDMA_TX         14

/*
 * Bit position definitions SPI_SR
*/
#define SPI_SR_RXNE             0
#define SPI_SR_TXE              1
#define SPI_SR_CRCERR           4
#define SPI_SR_MODF             5
#define SPI_SR_OVR              6
#define SPI_SR_BSY              7
#define SPI_SR_FRE              8
#define SPI_SR_FRLVL            9
#define SPI_SR_FTLVL            11

/***************************************************************************
 * Bit position definitions of I2C Peripheral
****************************************************************************/
/*
 * Bit position definitions I2C_CR1
*/
#define I2C_CR1_PE              0
#define I2C_CR1_TXIE            1
#define I2C_CR1_RXIE            2
#define I2C_CR1_ADDRIE          3
#define I2C_CR1_NACKIE          4
#define I2C_CR1_STOPIE          5
#define I2C_CR1_TCIE            6
#define I2C_CR1_ERRIE           7
#define I2C_CR1_DNF             8
#define I2C_CR1_ANFOFF          12
#define I2C_CR1_TXDMAEN         14
#define I2C_CR1_RXDMAEN         15
#define I2C_CR1_SBC             16
#define I2C_CR1_NOSTRETCH       17
#define I2C_CR1_WUPEN           18
#define I2C_CR1_GCEN            19
#define I2C_CR1_SMBHEN          20
#define I2C_CR1_SMBDEN          21
#define I2C_CR1_ALERTEN         22
#define I2C_CR1_PECEN           23

/*
 * Bit position definitions I2C_CR2
*/
#define I2C_CR2_SADD            0
#define I2C_CR2_RD_WRN          10
#define I2C_CR2_ADD10           11
#define I2C_CR2_HEAD10R         12
#define I2C_CR2_START           13
#define I2C_CR2_STOP            14
#define I2C_CR2_NACK            15
#define I2C_CR2_NBYTES          16
#define I2C_CR2_RELOAD          24
#define I2C_CR2_AUTOEND         25
#define I2C_CR2_PECBYTE         26

/*
 * Bit position definitions I2C_OAR1
*/
#define I2C_OAR1_OA1             0
#define I2C_OAR1_OA1MODE         10
#define I2C_OAR1_OA1EN           15

/*
 * Bit position definitions I2C_OAR2
*/
#define I2C_OAR2_OA2             0
#define I2C_OAR1_OA2MSK          8
#define I2C_OAR1_OA2EN           15

/*
 * Bit position definitions I2C_ISR
*/
#define I2C_ISR_TXE            0
#define I2C_ISR_TXIS           1
#define I2C_ISR_RXNE           2
#define I2C_ISR_ADDR           3
#define I2C_ISR_NACKF          4
#define I2C_ISR_STOPF          5
#define I2C_ISR_TC             6
#define I2C_ISR_TCR            7
#define I2C_ISR_BERR           8
#define I2C_ISR_ARLO           9
#define I2C_ISR_OVR            10
#define I2C_ISR_PECERR         11
#define I2C_ISR_TIMEOUT        12
#define I2C_ISR_ALERT          13
#define I2C_ISR_BUSY           15
#define I2C_ISR_DIR            16
#define I2C_ISR_ADDCODE        17

/*
 * Bit position definitions I2C_TIMINGR
*/
#define I2C_TIMINGR_SCLL       0
#define I2C_TIMINGR_SCLH       8
#define I2C_TIMINGR_SDADEL     16
#define I2C_TIMINGR_SCLDEL     20
#define I2C_TIMINGR_PRESC      28

/*
 * Bit position definitions I2C_ISR
*/
#define I2C_ISR_TXE            0
#define I2C_ISR_TXIS           1
#define I2C_ISR_RXNE           2
#define I2C_ISR_ADDR           3
#define I2C_ISR_NACKF          4
#define I2C_ISR_STOPF          5
#define I2C_ISR_TC             6
#define I2C_ISR_TCR            7
#define I2C_ISR_BERR           8
#define I2C_ISR_ARLO           9
#define I2C_ISR_OVR            10
#define I2C_ISR_PECERR         11
#define I2C_ISR_TIMEOUT        12
#define I2C_ISR_ALERT          13
#define I2C_ISR_BUSY           15
#define I2C_ISR_DIR            16
#define I2C_ISR_ADDCODE        17

/*
 * Bit position definitions I2C_ICR
*/
#define I2C_ICR_ADDRCF         3
#define I2C_ICR_NACKCF         4
#define I2C_ICR_STOPCF         5
#define I2C_ICR_BERRCF         8
#define I2C_ICR_ARLOCF         9
#define I2C_ICR_OVRCF          10
#define I2C_ICR_PECCF          11
#define I2C_ICR_TIMOUTCF       12
#define I2C_ICR_ALERTCF        13


/***************************************************************************
 * Bit position definitions of USART Peripheral
****************************************************************************/
/*
 * Bit position definitions USART_CR1
*/
#define USART_CR1_UE              0
#define USART_CR1_UESM            1
#define USART_CR1_RE              2
#define USART_CR1_TE              3
#define USART_CR1_IDLEIE          4
#define USART_CR1_RXNEIE          5
#define USART_CR1_TCIE            6
#define USART_CR1_TXEIE           7
#define USART_CR1_PEIE            8
#define USART_CR1_PS              9
#define USART_CR1_PCE             10
#define USART_CR1_WAKE            11
#define USART_CR1_M0              12
#define USART_CR1_MME             13
#define USART_CR1_CMIE            14
#define USART_CR1_OVER8           15
#define USART_CR1_DEDT            16
#define USART_CR1_DEAT            21
#define USART_CR1_RTOIE           26
#define USART_CR1_EOBIE           27
#define USART_CR1_M1              28

/*
 * Bit position definitions USART_CR2
*/
#define USART_CR2_ADDM7           4
#define USART_CR2_LBDL            5
#define USART_CR2_LBDIE           6
#define USART_CR2_LBCL            8
#define USART_CR2_CPHA            9
#define USART_CR2_CPOL            10
#define USART_CR2_CLKEN           11
#define USART_CR2_STOP            12
#define USART_CR2_LINEN           14
#define USART_CR2_SWAP            15
#define USART_CR2_RXINV           16
#define USART_CR2_TXINV           17
#define USART_CR2_DATAINV         18
#define USART_CR2_MSBFIRST        19
#define USART_CR2_ABREN           20
#define USART_CR2_ABRMOD          21
#define USART_CR2_RTOEN           13
#define USART_CR2_ADD             24

/*
 * Bit position definitions USART_CR3
*/
#define USART_CR3_EIE             0
#define USART_CR3_IREN            1
#define USART_CR3_IRLP            2
#define USART_CR3_HDSEL           3
#define USART_CR3_NACK            4
#define USART_CR3_SCEN            5
#define USART_CR3_DMAR            6
#define USART_CR3_DMAT            7
#define USART_CR3_RTSE            8
#define USART_CR3_CTSE            9
#define USART_CR3_CTSIE           10
#define USART_CR3_ONEBIT          11
#define USART_CR3_OVRDIS          12
#define USART_CR3_DDRE            13
#define USART_CR3_DEM             14
#define USART_CR3_DEP             15
#define USART_CR3_SCARCNT0        17
#define USART_CR3_SCARCNT1        18
#define USART_CR3_SCARCNT2        19
#define USART_CR3_WUS0            20
#define USART_CR3_WUS1            21
#define USART_CR3_WUFIE           22

/*
 * Bit position definitions USART_BRR
*/
#define USART_BRR_BRR             0

/*
 * Bit position definitions USART_GTPR
*/
#define USART_GTPR_PSC            0
#define USART_GTPR_GT             8

/*
 * Bit position definitions USART_RTOR
*/
#define USART_RTOR_RTO            0
#define USART_RTOR_BLEN           24

/*
 * Bit position definitions USART_RQR
*/
#define USART_RQR_ABRRQ           0
#define USART_RQR_SBKRQ           1
#define USART_RQR_MMRQ            2
#define USART_RQR_RXFRQ           3
#define USART_RQR_TXFRQ           4

/*
 * Bit position definitions USART_ISR
*/
#define USART_ISR_PE              0
#define USART_ISR_FE              1
#define USART_ISR_NF              2
#define USART_ISR_ORE             3
#define USART_ISR_IDLE            4
#define USART_ISR_RXNE            5
#define USART_ISR_TC              6
#define USART_ISR_TXE             7
#define USART_ISR_LBDF            8
#define USART_ISR_CTSIF           9
#define USART_ISR_CTS             10
#define USART_ISR_RTOF            11
#define USART_ISR_EOBF            12
#define USART_ISR_ABRE            14
#define USART_ISR_ABRF            15
#define USART_ISR_BUSY            16
#define USART_ISR_CMF             17
#define USART_ISR_SBKF            18
#define USART_ISR_RWU             19
#define USART_ISR_WUF             20
#define USART_ISR_TEACK           21
#define USART_ISR_REACK           22

/*
 * Bit position definitions USART_ICR
*/
#define USART_ICR_PECF            0
#define USART_ICR_FECF            1
#define USART_ICR_NCF             2
#define USART_ICR_ORECF           3
#define USART_ICR_IDLECF          4
#define USART_ICR_TCCF            6
#define USART_ICR_LBDCF           8
#define USART_ICR_CTSCF           9
#define USART_ICR_RTOCF           11
#define USART_ICR_EOBCF           12
#define USART_ICR_CMCF            17
#define USART_ICR_WUCF            20

/*
 * Bit position definitions USART_RDR
*/
#define USART_RDR_RDR             0

/*
 * Bit position definitions USART_TDR
*/
#define USART_TDR_TDR             0


#include "stm32f334xx_gpio_driver.h"
#include "stm32f334xx_spi_driver.h"
#include "stm32f334xx_usart_driver.h"
#include "stm32f334xx_rcc_driver.h" 

#endif  //INC_STM32F334XX_H_ 
