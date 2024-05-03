/*
 * stm32f334xx_usart_driver.c
 *
 *  Created on: May 2, 2024
 *      Author: ardau
 */

#include "stm32f334xx_usart_driver.h"

/*
 * Peripheral Clock Setup
*/
/*****************************************************************
 * @fn          - USART_PeriClockControl
 *
 * @brief       - This function enables or disables peripheral
 *                clock for the given USART port
 *
 * @param[in]   - Base address of the USART peripheral
 * @param[in]   - Macros: Enable or Disable
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
     if (EnorDi == ENABLE)
    {
        if (pUSARTx == USART1)
        {
            USART1_PCLK_EN();
        }else if (pUSARTx == USART2)
        {
            USART2_PCLK_EN();
        }else if (pUSARTx == USART3)
        {
            USART3_PCLK_EN();
        }
    }else
    {
        if (pUSARTx == USART1)
        {
            USART1_PCLK_DI();
        }else if (pUSARTx == USART2)
        {
            USART2_PCLK_DI();
        }else if (pUSARTx == USART3)
        {
            USART3_PCLK_DI();
        }
    }
}

/*****************************************************************
 * @fn          - USART_PeripheralControl
 *
 * @brief       - This function initialize USART peripheral clock
 *
 * @param[in]   - Pointer to USART Handle structure
 * @param[in]   - Enable or Disable
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pUSARTx->USART_CR1 |= ( 1 << USART_CR1_UE );
    }else
    {
        pUSARTx->USART_CR1 &= ~( 1 << USART_CR1_UE );
    }  
}

/*
 * Init and De-init
*/
/*****************************************************************
 * @fn          - USART_Init
 *
 * @brief       - This function initialize USART peripherals
 *
 * @param[in]   - Pointer to USART Handle structure
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void USART_Init(USART_Handle_t *pUSARTHandle)
{
    //Temporary variable
	uint32_t tempreg = 0;

    USART_PeriClockControl(USART1, ENABLE);

    /******************************** Configuration of CR1******************************************/

	// Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		tempreg |= (1 << USART_CR1_RE);

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		tempreg |= ( 1 << USART_CR1_TE );

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		tempreg |= ( ( 1 << USART_CR1_TE) | ( 1 << USART_CR1_RE) );
	}

    // Configuration of the Word length configuration item 
    if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_7BITS)
    {
        tempreg &= ~(1 << USART_CR1_M0);
        tempreg |= 1 << USART_CR1_M1;
    }else
    {
        tempreg &= ~(1 << USART_CR1_M1);
        tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M0;
    }
    
    // Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{ 
        tempreg |= 1 << USART_CR1_PCE;
		tempreg &= ~( 1 << USART_CR1_PS);

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		tempreg |= 1 << USART_CR1_PCE;
		tempreg |= 1 << USART_CR1_PS;
	}

   // Program the CR1 register 
	pUSARTHandle->pUSARTx->USART_CR1 = tempreg;

    /******************************** Configuration of CR2******************************************/

	tempreg=0;
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP; 
	pUSARTHandle->pUSARTx->USART_CR2 = tempreg;

    /******************************** Configuration of CR3******************************************/

	tempreg=0;
	
	// Configuration of USART hardware flow control 
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		tempreg |= ( 1 << USART_CR3_CTSE);
	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		tempreg |= ( 1 << USART_CR3_RTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		tempreg |= ( ( 1 << USART_CR3_RTSE) | ( 1 << USART_CR3_CTSE) );
	}

	pUSARTHandle->pUSARTx->USART_CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	// configure the baud rate
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_BAUD);
}

/*****************************************************************
 * @fn          - USART_DeInit
 *
 * @brief       - This function de-initialize USART peripherals
 *
 * @param[in]   - Base address of the USART peripheral
 *
 * @return      - None
 *
 * @Note        - None
 *
 *****************************************************************/
void USART_DeInit(USART_RegDef_t *pUSARTx)
{

}

/*****************************************************************
 * @fn          - USART_GetFlagStatus
 *
 * @brief       - This function returns status of USART flags
 *
 * @param[in]   - Base address of the USART peripheral
 * @param[in]   - Flag name
 *
 * @return      - Content of the input data
 *
 * @Note        - This is blocking call
 *
 *****************************************************************/
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
    if (pUSARTx->USART_ISR & FlagName)
    {
        return FLAG_SET;
    }
    
    return FLAG_RESET;
}

/*********************************************************************
 * @fn      		  - USART_ClearFlag
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Applicable to only USART_CTS_FLAG , USART_LBD_FLAG
 * USART_BUSY_FLAG, USART_TC_FLAG flags
 *
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
    pUSARTx->USART_ISR &= ~( StatusFlagName);
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
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx           = IRQNumber / 4;
    uint8_t iprx_section   = IRQNumber % 4;

    uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED);
    *( NVIC_PS_BASEADDR + iprx ) |= ( IRQPriority << shift_amount );
}

/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - 
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint16_t *pdata;

   // Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		// Wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

         // Check the USART_WordLength item for 7BIT, 8BIT or 9BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			// If 9BIT, load the DR with 2bytes masking the bits other than first 9 bits 
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->USART_TDR = (*pdata & (uint16_t)0x01FF);
			
			// Check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				// No parity is used in this transfer. so, 9bits of user data will be sent
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				// Parity bit is used in this transfer. So, 8bits of user data will be sent
				// The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}else if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS)
		{
			// This is 8bit data transfer 
			pUSARTHandle->pUSARTx->USART_TDR = (*pTxBuffer  & (uint8_t)0xFF);
			pTxBuffer++;

		}else if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_7BITS)
		{
			// This is 7bit data transfer 
			pUSARTHandle->pUSARTx->USART_TDR = (*pTxBuffer  & (uint8_t)0x7F);
			pTxBuffer++;
		}
	}

	// wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC) );
}


/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    // Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		// Wait until RXNE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

		// Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if( pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS )
		{
			// Check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				// No parity is used. so, all 9bits will be of user data
				// Read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = ( pUSARTHandle->pUSARTx->USART_RDR & (uint16_t) 0x01FF );
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				// Parity is used, so, 8bits will be of user data and 1 bit is parity
				*pRxBuffer = (pUSARTHandle->pUSARTx->USART_RDR & (uint8_t)0xFF);
				pRxBuffer++;
			}
		}
		else
		{
			// Check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				// No parity is used , so all 8bits will be of user data

				// Read 8 bits from DR
				 *pRxBuffer = (pUSARTHandle->pUSARTx->USART_RDR & (uint8_t)0xFF);
			}else
			{
				// Parity is used, so , 7 bits will be of user data and 1 bit is parity

				// Read only 7 bits , hence mask the DR with 0X7F
				*pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->USART_RDR & (uint8_t)0X7F);
			}
			pRxBuffer++;
		}
	}
}

/*********************************************************************
 * @fn      		  - USART_SendDataWithIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs 
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		// Enable interrupt for TXE
		pUSARTHandle->pUSARTx->USART_CR1 |= ( 1 << USART_CR1_TXEIE);
		
		// Enable interrupt for TC 
		pUSARTHandle->pUSARTx->USART_CR1 |= ( 1 << USART_CR1_TCIE);		
	}
	return txstate;
}

/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - 

 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if( rxstate != USART_BUSY_IN_RX )
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

        (void)pUSARTHandle->pUSARTx->USART_RDR;

		// Enable interrupt for RXNE
		pUSARTHandle->pUSARTx->USART_CR1 |= ( 1 << USART_CR1_RXNEIE );	
	}
	return rxstate;
}

/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  

 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{

	// Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	// Variables to hold Mantissa and Fraction values
	uint32_t M_part, F_part;

	uint32_t tempreg=0;

	// Get the value of APB bus clock in to the variable PCLKx
	if( pUSARTx == USART1 )
	{
		// USART1 is hanging on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	}else
	{
		PCLKx = RCC_GetPCLK1Value();
	}

	// Check for OVER8 configuration bit
	if( pUSARTx->USART_CR1 & (1 << USART_CR1_OVER8) )
	{
		// OVER8 = 1 , over sampling by 8
		usartdiv = ( (25 * PCLKx) / (2 * BaudRate) );
	}else
	{
		// Over sampling by 16
		usartdiv = ( (25 * PCLKx) / (4 * BaudRate) );
	}

	// Calculate the Mantissa part
	M_part = usartdiv/100;

	// Place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= M_part << 4;

	// Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	// Calculate the final fractional
	if( pUSARTx->USART_CR1 & ( 1 << USART_CR1_OVER8) )
	{
		// OVER8 = 1 , over sampling by 8
		F_part = ( (( F_part * 8)+ 50) / 100 ) & ( (uint8_t)0x07 );
	}else
	{
		// Over sampling by 16
		F_part = ( (( F_part * 16)+ 50) / 100 ) & ( (uint8_t)0x0F );
	}

	// Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= F_part;

	// Copy the value of tempreg in to BRR register
	pUSARTx->USART_BRR = tempreg;
}

/*********************************************************************
 * @fn      		  - USART_IRQHandler
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - 

 */
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t temp1 , temp2, temp3;
	uint16_t *pdata;

	/*************************Check for TC flag ********************************************/
    // Check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->USART_ISR & ( 1 << USART_ISR_TC);
	
	 // Check the state of TXEIE bit 
	temp2 = pUSARTHandle->pUSARTx->USART_CR1 & ( 1 << USART_CR1_TXEIE);

	if( temp1 && temp2 )
	{
		// This interrupt is because of TC
		
		// Close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			// Check the TxLen . If it is zero then close the data transmission
			if(! pUSARTHandle->TxLen )
			{
				// Clear the TC flag
				pUSARTHandle->pUSARTx->USART_ISR &= ~( 1 << USART_ISR_TC);
				
				// Clear the TCIE control bit 
				pUSARTHandle->pUSARTx->USART_CR1 &= ~( 1 << USART_CR1_TCIE);

				//Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;
				
				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;
				
				//Reset the length to zero
				pUSARTHandle->TxLen = 0;
				
				//Call the applicaton call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}

	/*************************Check for TXE flag ********************************************/
	// Check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->USART_ISR & ( 1 << USART_ISR_TXE);
	
	// Check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->USART_CR1 & ( 1 << USART_CR1_TXEIE);;

	if( temp1 && temp2 )
	{
		// This interrupt is because of TXE
		
		if( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX )
		{
			// Keep sending data until Txlen reaches to zero
			if( pUSARTHandle->TxLen > 0 )
			{
				// Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					// If 9BIT , load the DR with 2bytes masking the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
					
					// Loading only first 9 bits , so we have to mask with the value 0x01FF
					pUSARTHandle->pUSARTx->USART_TDR = (*pdata & (uint16_t)0x01FF);

					// Check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						// No parity is used in this transfer , so, 9bits of user data will be sent
						// Increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						
						// Decrement the length
						pUSARTHandle->TxLen -= 2;
					}
					else
					{
						// Parity bit is used in this transfer . so , 8bits of user data will be sent
						// The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;
						
						// Decrement the length
						pUSARTHandle->TxLen--;
					}
				}
				else
				{
					// This is 8bit data transfer
					pUSARTHandle->pUSARTx->USART_TDR = (*pUSARTHandle->pTxBuffer++  & (uint8_t)0xFF);

					// Increment the buffer address
					pUSARTHandle->pTxBuffer++;
					
					// Decrement the length
					pUSARTHandle->TxLen--;
				}
				
			}
			if (pUSARTHandle->TxLen == 0 )
			{
				// TxLen is zero 
				// Clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->USART_CR1 &= ~( 1 << USART_CR1_TXEIE);
			}
		}
	}
	
	/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->USART_ISR & ( 1 << USART_ISR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->USART_CR1 & ( 1 << USART_CR1_RXNEIE);


	if( temp1 && temp2 )
	{
		// This interrupt is because of rxne and txe
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			// TXE is set so send data
			if(pUSARTHandle->RxLen > 0)
			{
				// Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					// We are going to receive 9bit data in a frame

					// Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						// No parity is used. so, all 9bits will be of user data

						// Read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->USART_RDR  & (uint16_t)0x01FF);

						// Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						
						// Decrement the length
						pUSARTHandle->RxLen -= 2;
					}
					else
					{
						// Parity is used. so, 8bits will be of user data and 1 bit is parity
						*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->USART_RDR  & (uint8_t)0xFF);
						 
						// Now increment the pRxBuffer
						pUSARTHandle->pRxBuffer++;
						 
						// Decrement the length
						pUSARTHandle->RxLen--;
					}
				}
				else
				{
					// We are going to receive 8bit data in a frame

					// Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						// No parity is used , so all 8bits will be of user data

						// Read 8 bits from DR
						*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->USART_RDR  & (uint8_t)0xFF);
					}

					else
					{
						// Parity is used, so , 7 bits will be of user data and 1 bit is parity

						// Read only 7 bits , hence mask the DR with 0X7F
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->USART_RDR  & (uint8_t)0x7F);
					}
					// Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;
					
					// Decrement the length
					pUSARTHandle->RxLen--;
				}			
			}//if of >0

			if(! pUSARTHandle->RxLen)
			{
				//disable the rxne
				pUSARTHandle->pUSARTx->USART_CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
			}
		}
	}
		
	/*************************Check for CTS flag ********************************************/
	//Note : CTS feature is not applicable for UART4 and UART5

	// Check the status of CTS bit in the SR
	temp1 = pUSARTHandle->pUSARTx->USART_ISR & ( 1 << USART_ISR_CTS);
	
	// Check the state of CTSE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->USART_CR3 & ( 1 << USART_CR3_CTSE);
	
	// Check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	temp3 = pUSARTHandle->pUSARTx->USART_CR3 & ( 1 << USART_CR3_CTSIE);


	if(temp1  && temp2 )
	{
		// Clear the CTS flag in SR
		pUSARTHandle->pUSARTx->USART_ISR &=  ~( 1 << USART_ISR_CTS);
		
		// This interrupt is because of cts
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_CTS);
	}

/*************************Check for IDLE detection flag ********************************************/

	// Check the status of IDLE flag bit in the SR
	temp1 = pUSARTHandle->pUSARTx->USART_ISR & ( 1 << USART_ISR_IDLE);
	
	// Check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->USART_CR3 & ( 1 << USART_CR3_CTSE);

	if(temp1 && temp2)
	{
		// Clear the IDLE flag. Refer to the RM to understand the clear sequence 
		temp1 = pUSARTHandle->pUSARTx->USART_ISR &= ~( 1 << USART_ISR_IDLE);

		// This interrupt is because of idle
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}

/*************************Check for Overrun detection flag ********************************************/

	// Check the status of ORE flag  in the SR
	temp1 = pUSARTHandle->pUSARTx->USART_ISR & USART_ISR_ORE;
	
	// Check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->USART_CR1 & USART_CR1_RXNEIE;


	if(temp1  && temp2 )
	{
		// Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag . 
		
		// This interrupt is because of Overrun error 
		USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
	}



	/*************************Check for Error Flag ********************************************/

	// Noise Flag, Overrun error and Framing Error in multibuffer communication
	// We dont discuss multibuffer communication in this course. please refer to the RM
	// The blow code will get executed in only if multibuffer mode is used. 

	temp2 = pUSARTHandle->pUSARTx->USART_CR2 & ( 1 << USART_CR3_EIE) ;

	if(temp2 )
	{
		temp1 = pUSARTHandle->pUSARTx->USART_ISR;
		if(temp1 & (1 << USART_ISR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_FE);
		}

		if(temp1 & ( 1 << USART_ISR_NF) )
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_NF);
		}

		if(temp1 & ( 1 << USART_ISR_ORE) )
		{
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
		}
	}
} 
