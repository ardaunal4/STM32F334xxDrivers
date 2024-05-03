/*
 * stm32f334xx_rcc_driver.h
 *
 *  Created on: May 2, 2024
 *      Author: ardau
 */

#ifndef INC_STM32F334XX_RCC_DRIVER_H_
#define INC_STM32F334XX_RCC_DRIVER_H_

#include "stm32f334xx.h"

uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
uint32_t  RCC_GetPLLOutputClock();

#endif /* INC_STM32F334XX_RCC_DRIVER_H_ */
