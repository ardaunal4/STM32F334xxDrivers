/*
 * stm32f334xx_gpio_driver.c
 *
 *  Created on: Mar 30, 2024
 *      Author: ardau
 */

#include "stm32f334xx_gpio_driver.h"

/*
 * Peripheral Clock Setup
*/

/*****************************************************************
 * @fn          - GPIO_PeriClockControl
 *
 * @brief       - This function enables or disables peripheral
 *                clock for the given GPIO port
 *
 * @param[in]   - Base address of the GPIO peripheral
 * @param[in]   - Macros: Enable or Disable
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
    }else
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DI();
        }else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DI();
        }else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DI();
        }else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DI();
        }else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_DI();
        }
    }
}

/*
 * Init and De-init
*/

/*****************************************************************
 * @fn          - GPIO_Init
 *
 * @brief       - This function initialize GPIO peripherals
 *
 * @param[in]   - Pointer to GPIO Handle structure
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp = 0;                  //temp variable

    // 1. Configure the mode of the GPIO pin
    if (pGPIOHandle->GPIO_PinConfig->GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        temp = ( pGPIOHandle->GPIO_PinConfig->GPIO_PinMode << ( 2 * pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber ) );
        pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber );
        pGPIOHandle->pGPIOx->MODER |= temp;
        
    }else
    {
        // interrupt mode!
    }

    temp = 0;
    
    // 2. Configure the speed
    temp = pGPIOHandle->GPIO_PinConfig->GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber );
    pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber );
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;

    temp = 0;

    // 3. Configure the PullUp/PullDown settings
    temp = pGPIOHandle->GPIO_PinConfig->GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber );
    pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber );
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    temp = 0;

    // 4. Configure the Output Type
    temp = pGPIOHandle->GPIO_PinConfig->GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber;
    pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber );
    pGPIOHandle->pGPIOx->OTYPER |= temp;

    temp = 0;

    // 5. Configure the Alternate Functionality
    if (pGPIOHandle->GPIO_PinConfig->GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        uint8_t temp1, temp2;
        temp1 = pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber / 8;
        temp2 = pGPIOHandle->GPIO_PinConfig->GPIO_PinNumber % 8;
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) );
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig->GPIO_PinAltFunMode << ( 4 * temp2 ) );
    }

    temp = 0;
}


/*****************************************************************
 * @fn          - GPIO_DeInit
 *
 * @brief       - This function de-initialize GPIO peripherals
 *
 * @param[in]   - Base address of the GPIO peripheral
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if (pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }else if (pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }else if (pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }else if (pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }else if (pGPIOx == GPIOF)
    {
        GPIOF_REG_RESET();
    }
}

/*
 * Data read and write
*/

/*****************************************************************
 * @fn          - GPIO_ReadFromInputPin
 *
 * @brief       - This function reads value of input pin, on
 *                a specific port
 *
 * @param[in]   - Base address of the GPIO peripheral
 * @param[in]   - Pin number
 *
 * @return      - Content of the input data
 *
 * @Note        - 0 or 1
 *
 *****************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t value;
    value = (uint8_t)( ( pGPIOx->IDR >> PinNumber ) & 0x00000001 );
    return value;
}

/*****************************************************************
 * @fn          - GPIO_ReadFromInputPort
 *
 * @brief       - This function reads value of input port
 *
 * @param[in]   - Base address of the GPIO peripheral
 *
 * @return      - Content of the input data
 *
 * @Note        - None
 *
 *****************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    uint16_t value;
    value = (uint16_t)pGPIOx->IDR;
    return value;
}

/*****************************************************************
 * @fn          - GPIO_WriteToOutputPin
 *
 * @brief       - This function writes value on a specific
 *                output pin
 *
 * @param[in]   - Base address of the GPIO peripheral
 * @param[in]   - Pin number
 * @param[in]   - Value (Set/Reset Macro)
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{   
    if (Value ==  GPIO_PIN_SET)
    {
        pGPIOx->ODR |= ( 1 << PinNumber );
    }else
    {
        pGPIOx->ODR &= ~( 1 << PinNumber );
    }
    

}

/*****************************************************************
 * @fn          - GPIO_WriteToOutputPort
 *
 * @brief       - This function writes value on a specific
 *                output port
 *
 * @param[in]   - Base address of the GPIO peripheral
 * @param[in]   - Value (Set/Reset Macro)
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value;
}

/*****************************************************************
 * @fn          - GPIO_ToggleOutputPin
 *
 * @brief       - This function toggles specific output pin
 *
 * @param[in]   - Base address of the GPIO peripheral
 * @param[in]   - Pin number
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= ( 1 << PinNumber );
}

/*
 * IRQ Configuration and ISR handling
*/

/*****************************************************************
 * @fn          - GPIO_IRQInterruptConfig
 *
 * @brief       - This function configures interrupt
 *
 * @param[in]   - IRQ Interrupt number
 * @param[in]   - Macro: Enable/Disable
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}

/*****************************************************************
 * @fn          - GPIO_IRQHandling
 *
 * @brief       - This function handle interrupts
 *
 * @param[in]   - Pin number
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{

}