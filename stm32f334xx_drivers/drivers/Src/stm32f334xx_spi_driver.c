/*
 * stm32f334xx_spi_driver.c
 *
 *  Created on: Apr 2, 2024
 *      Author: ardau
 */

#include "stm32f334xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*
 * Peripheral Clock Setup
*/

/*****************************************************************
 * @fn          - SPI_PeriClockControl
 *
 * @brief       - This function enables or disables peripheral
 *                clock for the given SPI port
 *
 * @param[in]   - Base address of the SPI peripheral
 * @param[in]   - Macros: Enable or Disable
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
     if (EnorDi == ENABLE)
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
    }else
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_DI();
        }
    }
}

/*
 * Init and De-init
*/

/*****************************************************************
 * @fn          - SPI_Init
 *
 * @brief       - This function initialize SPI peripherals
 *
 * @param[in]   - Pointer to SPI Handle structure
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
    // Enable the SPI Peripheral clock
    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
    
    uint32_t tempreg = 0;

    // 1. Configure the device mode
    tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

    // 2. Configure the bus config
    if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
    {
        // BIDIMODE must be cleared
        tempreg &= ~( 1 << SPI_CR1_BIDIMODE );

        // RXONLY bit must be cleared
        tempreg &= ~( 1 << SPI_CR1_RXONLY );

    }else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
    {
        // BIDIMODE must be setted
        tempreg |= ( 1 << SPI_CR1_BIDIMODE );

        // RXONLY bit must be cleared
        tempreg &= ~( 1 << SPI_CR1_RXONLY );

    }else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
    {
        // BIDIMODE must be cleared
        tempreg &= ~( 1 << SPI_CR1_BIDIMODE );

        // RXONLY bit must be setted
        tempreg |= ( 1 << SPI_CR1_RXONLY );
    }

    // 3. Configure the spi serial clock speed (baud rate)
    tempreg |= ( pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

    // 4. Configure the CPOL
    tempreg |= ( pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

    // 5. Configure the CPHA
    tempreg |= ( pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

    pSPIHandle->pSPIx->SPIx_CR1 = tempreg;
}

/*****************************************************************
 * @fn          - SPI_DeInit
 *
 * @brief       - This function de-initialize SPI peripherals
 *
 * @param[in]   - Base address of the SPI peripheral
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void SPI_DeInit(SPI_Handle_t *pSPIHandle)
{
    // Disable the SPI Peripheral
    SPI_PeripheralControl(pSPIHandle->pSPIx, DISABLE);
    
    // Disable the SPI Peripheral clock
    SPI_PeriClockControl(pSPIHandle->pSPIx, DISABLE);
}

/*****************************************************************
 * @fn          - SPI_GetFlagStatus
 *
 * @brief       - This function returns status of SPI flags
 *
 * @param[in]   - Base address of the SPI peripheral
 * @param[in]   - Flag name
 *
 * @return      - Content of the input data
 *
 * @Note        - This is blocking call
 *
 *****************************************************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
    if (pSPIx->SPIx_SR & FlagName)
    {
        return FLAG_SET;
    }
    
    return FLAG_RESET;
}


/*****************************************************************
 * @fn          - SPI_SendData
 *
 * @brief       - This function writes on to data register
 *
 * @param[in]   - Base address of the SPI peripheral
 * @param[in]   - Message to send - 8 Bits!
 * @param[in]   - Message's length
 *
 * @return      - Content of the input data
 *
 * @Note        - This is blocking call
 *
 *****************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
    while (Len > 0)
    {
        // 1. Wait until TXE is setted
        while( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET );

        // 2. Load the data into the DR
        pSPIx->SPIx_DR = *pTxBuffer;
        Len--;
        pTxBuffer++;
    }
}

/*****************************************************************
 * @fn          - SPI_ReceiveData
 *
 * @brief       - This function reads data register
 *
 * @param[in]   - Base address of the SPI peripheral
 * @param[in]   - Message to receive
 * @param[in]   - Message's length
 *
 * @return      - Content of the input data
 *
 * @Note        - 0 or 1
 *
 *****************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
    while (Len > 0)
    {
        // 1. Wait until RXE is setted
        while( SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET );

        // 2. Load the data from  DR to Rxbuffer address
        *pRxBuffer = pSPIx->SPIx_DR;
        Len--;
        pRxBuffer++;
    }
}

/*****************************************************************
 * @fn          - SPI_PeripheralControl
 *
 * @brief       - This function initialize SPI peripheral clock
 *
 * @param[in]   - Pointer to SPI Handle structure
 * @param[in]   - Enable or Disable
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        pSPIx->SPIx_CR1 |= ( 1 << SPI_CR1_SPE );
    }else
    {
        pSPIx->SPIx_CR1 &= ~( 1 << SPI_CR1_SPE );
    }  
}

/*****************************************************************
 * @fn          - SPI_SSIConfig
 *
 * @brief       - This function makes NSS signal internally high
 *                and avoids MODF error.
 *
 * @param[in]   - Pointer to SPI Handle structure
 * @param[in]   - Enable or Disable
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        pSPIx->SPIx_CR1 |= ( 1 << SPI_CR1_SSI );
    }else
    {
        pSPIx->SPIx_CR1 &= ~( 1 << SPI_CR1_SSI );
    }
}

/*****************************************************************
 * @fn          - SPI_SSOEConfig
 *
 * @brief       - This function sets one master mode for SPI 
 *                communication
 *
 * @param[in]   - Pointer to SPI Handle structure
 * @param[in]   - Enable or Disable
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        pSPIx->SPIx_CR2 |= ( 1 << SPI_CR2_SSOE );
    }else
    {
        pSPIx->SPIx_CR2 &= ~( 1 << SPI_CR2_SSOE );
    }
}

/*
 * IRQ and ISR
*/

/*****************************************************************
 * @fn          - SPI_IRQInterruptConfig
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
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (IRQNumber <= 31)
        {
            // Program ISER0 register
            *NVIC_ISER0 |= ( 1 << IRQNumber );
        } else if (IRQNumber > 31  && IRQNumber < 64)
        {
            // Program ISER1 register
            *NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
        }else if (IRQNumber >= 64  && IRQNumber < 96)
        {
            // Program ISER2 register
            *NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
        }
    }else
    {
        if (IRQNumber <= 31)
        {
            // Program ICER0 register
            *NVIC_ICER0 |= ( 1 << IRQNumber );
        } else if (IRQNumber > 31  && IRQNumber < 64)
        {
            // Program ICER1 register
            *NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
        }else if (IRQNumber >= 64  && IRQNumber < 96)
        {
            // Program ICER2 register
            *NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
        }
    } 
}

/*****************************************************************
 * @fn          - SPI_IRQPriorityConfig
 *
 * @brief       - This function configures priority of interrupts
 *
 * @param[in]   - IRQ Interrupt number
 * @param[in]   - IRQ priority level
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx           = IRQNumber / 4;
    uint8_t iprx_section   = IRQNumber % 4;

    uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED);
    *( NVIC_PS_BASEADDR + iprx ) |= ( IRQPriority << shift_amount );
}

/*****************************************************************
 * @fn          - SPI_IRQHandling
 *
 * @brief       - This function handle interrupts
 *
 * @param[in]   - SPI handle pointer
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
    uint8_t temp1 = 0, temp2 = 0;

    // First check for TXE
    temp1 = pHandle->pSPIx->SPIx_SR & ( 1 << SPI_SR_TXE );
    temp2 = pHandle->pSPIx->SPIx_CR2 & ( 1 << SPI_CR2_TXEIE );

    if (temp1 && temp2)
    {
        // handle TXE
        spi_txe_interrupt_handle(pHandle);
    }

    // Second check for RXNE
    temp1 = pHandle->pSPIx->SPIx_SR & ( 1 << SPI_SR_RXNE );
    temp2 = pHandle->pSPIx->SPIx_CR2 & ( 1 << SPI_CR2_RXNEIE );

    if (temp1 && temp2)
    {
        // handle RXNE
        spi_rxne_interrupt_handle(pHandle);
    }

    // Third check for OVR
    temp1 = pHandle->pSPIx->SPIx_SR & ( 1 << SPI_SR_OVR );
    temp2 = pHandle->pSPIx->SPIx_CR2 & ( 1 << SPI_CR2_ERRIE );

    if (temp1 && temp2)
    {
        // handle RXNE
        spi_ovr_err_interrupt_handle(pHandle);
    }
}

/*****************************************************************
 * @fn          - SPI_SendDataIT
 *
 * @brief       - This function sends data through interrupt
 *
 * @param[in]   - SPI handle pointer
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    uint8_t state = pSPIHandle->TxStatus;
    
    if (state != SPI_BUSY_IN_TX)
    {
        // 1. Save the Tx buffer address and Len information in some global variables
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = Len;

        // 2. Mark the SPI state as busy in transmission so that no other code can
        // take over same SPI peripheral until transmission is over 
        pSPIHandle->TxStatus = SPI_BUSY_IN_TX;

        // 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
        pSPIHandle->pSPIx->SPIx_CR2 |= ( 1 << SPI_CR2_TXEIE );

        // 4. Data transmission will be handled by the ISR code  
    }
    
    return state;
}

/*****************************************************************
 * @fn          - SPI_ReceiveDataIT
 *
 * @brief       - This function reads data through interrupt
 *
 * @param[in]   - SPI handle pointer
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    uint8_t state = pSPIHandle->RxStatus;
    
    if (state != SPI_BUSY_IN_RX)
    {
        // 1. Save the Rx buffer address and Len information in some global variables
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLen = Len;

        // 2. Mark the SPI state as busy in transmission so that no other code can
        // take over same SPI peripheral until transmission is over 
        pSPIHandle->RxStatus = SPI_BUSY_IN_RX;

        // 3. Enable the RXNEIE control bit to get interrupt whenever RXNE flag is set in SR
        pSPIHandle->pSPIx->SPIx_CR2 |= ( 1 << SPI_CR2_RXNEIE );

        // 4. Data transmission will be handled by the ISR code  
    }
    
    return state;
}

/**********************************************************************
*************** Some helper function implementations ******************
***********************************************************************/
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->SPIx_DR = *pSPIHandle->pTxBuffer;
    pSPIHandle->TxLen--;
    pSPIHandle->pTxBuffer++;

    if ( !pSPIHandle->TxLen )
    {
        // TxLen is zero, so close the SPI transmission and inform the application that
        // TX is over.

        // This prevents interrupts from setting up TXE flag
        SPI_CloseTransmission(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
    }
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    *(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->SPIx_DR;
    pSPIHandle->RxLen--;
    pSPIHandle->pRxBuffer--;

    if ( !pSPIHandle->RxLen )
    {
        // RxLen is zero, so close the SPI transmission and inform the application that
        // RX is over.

        // Turn off the RXNEIE interrupt
        SPI_CloseReception(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
    }
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    uint8_t temp = 0;

    // 1. Clear the ovr flag
    if( pSPIHandle->TxStatus != SPI_BUSY_IN_TX )
    {
        SPI_ClearOVRFlag( pSPIHandle->pSPIx );
    }

    (void) temp;

    // 2. Inform the application
    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->SPIx_CR2 &= ~( 1 << SPI_CR2_TXEIE );                // This prevents interrupts from setting up of TXE flag
    pSPIHandle->pTxBuffer = NULL;
    pSPIHandle->TxLen = 0;
    pSPIHandle->TxStatus = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->SPIx_CR2 &= ~( 1 << SPI_CR2_RXNEIE );
    pSPIHandle->pRxBuffer = NULL;
    pSPIHandle->RxLen = 0;
    pSPIHandle->RxStatus = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
    int8_t temp = 0;
    temp = pSPIx->SPIx_DR;
    temp = pSPIx->SPIx_SR;
    (void) temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
    // This is a weak implementation. The application may override this function.
}