/*
 * usart_tx_testing.c
 *
 *  Created on: May 3, 2024
 *      Author: ardau
 */
/*
#include "stm32f334xx.h"
#include <string.h>
*/
/*
 * USART1_TX  -> PA9
 * USART1_RX  -> PA10
 * Alternate functionality is AF7
 */
/*
USART_Handle_t usart1_handle;
void USART1_GPIOInits(void);
void USART1_Inits(void);
void GPIOButtonInit(void);
void delay(void);

int main(void)
{
	// User data
	char message[] = "Hello world!";

	// This function is used to initialize the GPIO A0 button parameters
	GPIOButtonInit();

	// This function is used to initialize the GPIO pins to behave as USART1 pins
	USART1_GPIOInits();

	// This function is used to initialize the USART1 peripheral parameters
	USART1_Inits();
	USART_PeripheralControl(USART1, ENABLE);

    while(1)
	{
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );
		delay();

		USART_SendData(&usart1_handle, (uint8_t*)message, strlen(message) );
		while( !USART_GetFlagStatus(USART1, USART_FLAG_TC) );
	}
}

void USART1_GPIOInits(void)
{
	GPIO_Handle_t USART1Pins;
    memset(&USART1Pins, 0, sizeof(USART1Pins));

	USART1Pins.pGPIOx = GPIOA;
	USART1Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USART1Pins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AFN_AF7;
	USART1Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USART1Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	USART1Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HS;

	// USART1_TX
	USART1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&USART1Pins);

	// USART1_RX
	USART1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&USART1Pins);
}

void USART1_Inits(void)
{
    memset(&usart1_handle, 0, sizeof(usart1_handle));

    usart1_handle.pUSARTx = USART1;
    usart1_handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
    usart1_handle.USART_Config.USART_BAUD = USART_STD_BAUD_9600;
    usart1_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    usart1_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
    usart1_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
    usart1_handle.USART_Config.USART_ParityControl = NO_PR_BITS_IMPLEMENTED;
    USART_Init(&usart1_handle);
}

void GPIOButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;
    memset(&GPIOBtn, 0, sizeof(GPIOBtn));                              // Clear all structures values before implement it

	GPIOBtn.pGPIOx                              = GPIOA;               // Choose GPIO port as GPIOA
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_0;       // Pin number 0
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_IT_FT;     // Pin mode is arranged as INTERRUPT FALLING EDGE DETECTION
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_HS;       // Output speed is configured as medium speed
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_NO_PUPD;        // No pull-up or pull-down resistors
	GPIO_Init(&GPIOBtn);                                               // Initialize the GPIOA pin 0
}

void delay(void)
{
	for (int i = 0; i < 50000; i++);
}
*/
