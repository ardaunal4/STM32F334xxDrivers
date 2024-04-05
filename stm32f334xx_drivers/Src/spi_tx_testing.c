/*
 * spi_tx_testing.c
 *
 *  Created on: Apr 2, 2024
 *      Author: ardau
 */

// #include "stm32f334xx.h"
// #include <string.h>

// /*
//  * SPI1_NSS  -> PA4
//  * SPI1_SCK  -> PA5
//  * SPI1_MISO -> PA6
//  * SPI1_MOSI -> PA7
//  * Alternate functionality is AF5
//  */

// void SPI1_GPIOInits(void);
// void SPI1_Inits(void);
// void GPIOButtonInit(void);
// void delay(void);

// int main(void)
// {
// 	// User data
// 	char message[] = "Hello world!";

// 	// This function is used to initialize the GPIO A0 button parameters
// 	GPIOButtonInit();

// 	// This function is used to initialize the GPIO pins to behave as SPI1 pins
// 	SPI1_GPIOInits();

// 	// This function is used to initialize the SPI1 peripheral parameters
// 	SPI1_Inits();

// 	/*
// 	 * Making SSOE 1 does NSS output enable
// 	 * The NSS pin is automatically managed by the hardware
// 	 * i.e. when SPE=1, NSS will be pulled to low.
// 	 * and NSS pin will be high when SPE=0
// 	*/ 
// 	SPI_SSOEConfig(SPI1, ENABLE);

//     while(1)
// 	{
// 		//wait till button is pressed
// 		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );
		
// 		delay();
// 		SPI_PeripheralControl(SPI1, ENABLE);                       // Enable the SPI1 peripheral
// 		uint8_t dataLen = strlen(message);                         // Length of the message
// 		SPI_SendData(SPI1, &dataLen, 1);   						   // First set data length
// 		SPI_SendData(SPI1, (uint8_t*) message, strlen(message));   // Secondly send the data
// 		while( SPI_GetFlagStatus(SPI1, SPI_BSY_FLAG) );            // Confirm SPI is not busy
// 		SPI_PeripheralControl(SPI1, DISABLE);                      // Disable the SPI1 peripheral
// 	}
// }

// void SPI1_GPIOInits(void)
// {
// 	GPIO_Handle_t SPIPins;

// 	SPIPins.pGPIOx = GPIOA;
// 	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
// 	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AFN_AF5;
// 	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
// 	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
// 	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HS;

// 	// SCLK
// 	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
// 	GPIO_Init(&SPIPins);

// 	// MOSI
// 	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
// 	GPIO_Init(&SPIPins);

// 	// MISO
// 	// SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
// 	// GPIO_Init(&SPIPins);

// 	// NSS
// 	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
// 	GPIO_Init(&SPIPins);
// }

// void SPI1_Inits(void)
// {
// 	SPI_Handle_t SPI1Handle;

// 	SPI1Handle.pSPIx                    = SPI1;
// 	SPI1Handle.SPIConfig.SPI_BusConfig  = SPI_BUS_CONFIG_FD;
// 	SPI1Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
// 	SPI1Handle.SPIConfig.SPI_SclkSpeed  = SPI_SCLK_SPEED_DIV256;            // Generates sclk of 8MHz
// 	SPI1Handle.SPIConfig.SPI_CPHA       = SPI_CPHA_LOW;
// 	SPI1Handle.SPIConfig.SPI_CPOL       = SPI_CPOL_HIGH;
// 	SPI1Handle.SPIConfig.SPI_SSM        = SPI_SSM_DI;                     // Hardware slave management enabled for NSS pin

// 	SPI_Init(&SPI1Handle);
// }

// void GPIOButtonInit(void)
// {
// 	GPIO_Handle_t GPIOBtn;

//     memset(&GPIOBtn, 0, sizeof(GPIOBtn));                              // Clear all structures values before implement it
// 	GPIOBtn.pGPIOx                              = GPIOA;               // Choose GPIO port as GPIOA
// 	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_0;       // Pin number 0
// 	GPIOBtn.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_IT_FT;     // Pin mode is arranged as INTERRUPT FALLING EDGE DETECTION
// 	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_HS;       // Output speed is configured as medium speed
// 	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_NO_PUPD;        // No pull-up or pull-down resistors
// 	GPIO_Init(&GPIOBtn);                                               // Initialize the GPIOA pin 0
// }

// void delay(void)
// {
// 	for (int i = 0; i < 50000; i++);
// }