/*
 * stm32f334xx_gpio_driver.h
 *
 *  Created on: Mar 30, 2024
 *      Author: ardau
 */

#ifndef INC_STM32F334XX_GPIO_DRIVER_H_
#define INC_STM32F334XX_GPIO_DRIVER_H_

#include "stm32f334xx.h"

/*
 * This is a Configuration structure for a GPIO pin
*/
typedef struct
{
    uint8_t GPIO_PinNumber;                                                /*!< possible values from @GPIO_PIN_NUMBERS >*/
    uint8_t GPIO_PinMode;                                                  /*!< possible values from @GPIO_PIN_MODES >*/
    uint8_t GPIO_PinSpeed;                                                 /*!< possible values from @GPIO_PIN_SPEED >*/
    uint8_t GPIO_PinPuPdControl;                                           /*!< possible values from @GPIO_PU_PD_CONF >*/
    uint8_t GPIO_PinOPType;                                                /*!< possible values from @GPIO_PIN_OUTPUT_TYPES >*/
    uint8_t GPIO_PinAltFunMode;                                            /*!< Only for port A and B! possible values from @GPIO_ALT_FNC >*/

}GPIO_PinConfig_t;

/*
 * This is a Handle structure for a GPIO pin
*/
typedef struct
{
    // Pointer to hold the base address of the GPIO peripheral
    GPIO_RegDef_t      *pGPIOx;                                            /*!< This holds the base address of the GPIO port to which the pin belongs >*/
    GPIO_PinConfig_t   *GPIO_PinConfig;                                    /*!< This holds GPIO pin configuration settings >*/

}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
*/
#define GPIO_PIN_NO_0                   0
#define GPIO_PIN_NO_1                   1
#define GPIO_PIN_NO_2                   2
#define GPIO_PIN_NO_3                   3
#define GPIO_PIN_NO_4                   4
#define GPIO_PIN_NO_5                   5
#define GPIO_PIN_NO_6                   6
#define GPIO_PIN_NO_7                   7
#define GPIO_PIN_NO_8                   8
#define GPIO_PIN_NO_9                   9
#define GPIO_PIN_NO_10                  10
#define GPIO_PIN_NO_11                  11
#define GPIO_PIN_NO_12                  12
#define GPIO_PIN_NO_13                  13
#define GPIO_PIN_NO_14                  14
#define GPIO_PIN_NO_15                  15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible output modes
*/
#define GPIO_MODE_IN                    0
#define GPIO_MODE_OUT                   1
#define GPIO_MODE_ALTFN                 2
#define GPIO_MODE_ANALOG                3
#define GPIO_MODE_IT_FT                 4                                  // IT: INPUT, FT: FALLING EDGE
#define GPIO_MODE_IT_RT                 5
#define GPIO_MODE_IT_RFT                6                                  // RFT: RASING AND FALLING EDGE

/*
 * @GPIO_PIN_OUTPUT_TYPES
 * GPIO pin possible output types
*/ 
#define GPIO_OP_TYPE_PP                 0                                  // PP: PUSH-PULL
#define GPIO_OP_TYPE_OD                 1                                  // OD: OPEN-DRAIN

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speed
*/ 
#define GPIO_SPEED_LS                   0                                  // LS: LOW SPEED
#define GPIO_SPEED_MS                   1                                  // MS: MEDIUM SPEED
#define GPIO_SPEED_HS                   3                                  // HS: HIGH SPEED

/*
 * @GPIO_PU_PD_CONF
 * GPIO pin Pull-up and pull-down configuration macros
*/ 
#define GPIO_NO_PUPD                    0                                  // NO_PUPD: NO PULL UP OR DOWN
#define GPIO_PIN_PU                     1                                  // PU: PULL UP
#define GPIO_PIN_PD                     2                                  // PD: PULL DOWN

/*
 * @GPIO_ALT_FNC
 * GPIO pin Pull-up and pull-down configuration macros
*/ 
#define GPIO_AFN_AF0                    0                                  
#define GPIO_AFN_AF1                    1                                  
#define GPIO_AFN_AF2                    2                                  
#define GPIO_AFN_AF3                    3                                  
#define GPIO_AFN_AF4                    4                                  
#define GPIO_AFN_AF5                    5                                  
#define GPIO_AFN_AF6                    6                                  
#define GPIO_AFN_AF7                    7                                  
#define GPIO_AFN_AF8                    8                                  
#define GPIO_AFN_AF9                    9                                  
#define GPIO_AFN_AF10                   10                                 
#define GPIO_AFN_AF11                   11                                 
#define GPIO_AFN_AF12                   12                                 
#define GPIO_AFN_AF13                   13                                 
#define GPIO_AFN_AF14                   14                                 
#define GPIO_AFN_AF15                   15                                 

/**********************************************************************************************************
 *                                   APIs supported by this driver 
 *               For more information about the APIs check the function definitions
***********************************************************************************************************/

/*
 * Peripheral Clock Setup
*/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F334XX_GPIO_DRIVER_H_ */