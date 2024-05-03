/*
 * stm32f334xx_rcc_driver.c
 *
 *  Created on: May 2, 2024
 *      Author: ardau
 */

#include "stm32f334xx_rcc_driver.h" 

uint16_t AHB_PreScaler[9] = {2, 4, 8, 16, 32, 64, 128, 256, 512};
uint16_t APB1_PreScaler[4] = {2, 4, 8, 16};

uint32_t  RCC_GetPLLOutputClock()
{
	return 0;
}

/*********************************************************************
 * @fn      		  - RCC_GetPCLK1Value
 *
 * @brief             - It calculates APB1 clock frequency.
 *
 * @param[in]         -
 *
 * @return            - Clock frequency
 *
 * @Note              -
 */
uint32_t RCC_GetPCLK1Value(void)
{
    uint32_t pclk1, SystemCLK;
    uint8_t clksrc, temp, ahbp, apb1p;

    clksrc = ( ( RCC->RCC_CFGR >> 2 ) & 0x3 );

    if (clksrc == 0)
    {
        SystemCLK = 16000000;
    }else if (clksrc == 1)
    {
        SystemCLK = 8000000;
    }else if (clksrc == 2)
    {
        SystemCLK = RCC_GetPLLOutputClock();
    }

    // For AHB
    temp = ( ( RCC->RCC_CFGR >> 4 ) & 0xF );
    if (temp < 8)
    {
        ahbp = 1;
    }else
    {
        ahbp = AHB_PreScaler[temp - 8];
    }

    // For APB1
    temp = ( ( RCC->RCC_CFGR >> 8 ) & 0x7 );
    if (temp < 4)
    {
        apb1p = 1;
    }else
    {
        apb1p = APB1_PreScaler[temp - 4];
    }
    
    pclk1 = (SystemCLK / ahbp) / apb1p;

    return pclk1;
}

/*********************************************************************
 * @fn      		  - RCC_GetPCLK2Value
 *
 * @brief             - It calculates APB2 bus clock frequency
 *
 *
 * @return            - APB2 clock frequency 
 *
 * @Note              -
 */
uint32_t RCC_GetPCLK2Value(void)
{
    uint32_t pclk2, SystemCLK;
    uint8_t clksrc, temp, ahbp, apb2p;

    clksrc = ( ( RCC->RCC_CFGR >> 2 ) & 0x3 );

    if (clksrc == 0)
    {
        SystemCLK = 16000000;
    }else if (clksrc == 1)
    {
        SystemCLK = 8000000;
    }else if (clksrc == 2)
    {
        SystemCLK = RCC_GetPLLOutputClock();
    }

    // For AHB
    temp = ( ( RCC->RCC_CFGR >> 4 ) & 0xF );
    if (temp < 8)
    {
        ahbp = 1;
    }else
    {
        ahbp = AHB_PreScaler[temp - 8];
    }

    // For APB2
    temp = ( ( RCC->RCC_CFGR >> 11 ) & 0x7 );
    if (temp < 4)
    {
        apb2p = 1;
    }else
    {
        apb2p = APB1_PreScaler[temp - 4];
    }

    pclk2 = (SystemCLK / ahbp) / apb2p;

    return pclk2;
}

