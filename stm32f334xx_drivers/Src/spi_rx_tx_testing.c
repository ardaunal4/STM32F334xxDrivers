// /*
//  * spi_rx_tx_testing.c
//  *
//  *  Created on: May 1, 2024
//  *      Author: ardau
//  */


// #include "stm32f334xx.h"
// #include <string.h>

// #define NACK       					0xA5U
// #define ACK        					0xF5U
// #define COMMAND_LED_CTRL      		0x40U

// /*
//  * SPI1_NSS  -> PA4
//  * SPI1_SCK  -> PA5
//  * SPI1_MISO -> PA6
//  * SPI1_MOSI -> PA7
//  * Alternate functionality is AF5
//  */

// void SPI1_GPIOInits(void);
// void SPI1_Inits(void);
// void delay(void);
// void GPIOLedInit(void);
// void ledFunction(uint8_t led_number, uint8_t led_state);

// int main(void)
// {
// 	uint8_t dummy_write = 0xff;
// 	uint8_t dummy_read;

// 	GPIOLedInit();           
// 	SPI1_GPIOInits();          
// 	SPI1_Inits();               
// 	SPI_SSOEConfig(SPI1, ENABLE);
// 	SPI_IRQInterruptConfig(IRQ_NO_SPI1, ENABLE);

// 	uint8_t command = 0, data1, data2, ack = ACK, nack = NACK;

//     while(1)
// 	{	
// 		SPI_PeripheralControl(SPI1, ENABLE);
		
// 		SPI_SendData(SPI1, &dummy_write, 1);
// 		SPI_ReceiveData(SPI1, &command, 1);

// 		while( !GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_6) )
// 		{
			
// 			if( command == COMMAND_LED_CTRL )
// 			{
// 				SPI_SendData(SPI1, &ack, 1);
// 				SPI_ReceiveData(SPI1, &dummy_read, 1);

// 				delay();


// 				SPI_SendData(SPI1, &dummy_write, 1);
// 				SPI_ReceiveData(SPI1, &data1, 1);

// 				SPI_SendData(SPI1, &dummy_write, 1);
// 				SPI_ReceiveData(SPI1, &data2, 1);


// 				ledFunction(data1, data2);

// 			}else		
// 			{
// 				SPI_SendData(SPI1, &nack, 1);
// 				SPI_ReceiveData(SPI1, &dummy_read, 1);

// 				SPI_SendData(SPI1, &dummy_write, 1);
// 				SPI_ReceiveData(SPI1, &command, 1);
// 			}
// 		}		
		
// 		while( SPI_GetFlagStatus(SPI1, SPI_BSY_FLAG) );
// 		SPI_PeripheralControl(SPI1, DISABLE);
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
// 	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
// 	GPIO_Init(&SPIPins);

// 	// NSS
// 	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
// 	GPIO_Init(&SPIPins);
// }

// void SPI1_Inits(void)
// {
// 	SPI_Handle_t SPI1Handle;

// 	SPI1Handle.pSPIx                    = SPI1;
// 	SPI1Handle.SPIConfig.SPI_BusConfig  = SPI_BUS_CONFIG_FD;
// 	SPI1Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_SLAVE;
// 	SPI1Handle.SPIConfig.SPI_SclkSpeed  = SPI_SCLK_SPEED_DIV32;              // Generates sclk of 0.5MHz
// 	SPI1Handle.SPIConfig.SPI_CPHA       = SPI_CPHA_LOW;
// 	SPI1Handle.SPIConfig.SPI_CPOL       = SPI_CPOL_LOW;
// 	SPI1Handle.SPIConfig.SPI_SSM        = SPI_SSM_DI;                       // Hardware slave management enabled for NSS pin

// 	SPI_Init(&SPI1Handle);
// }

// void delay(void)
// {
// 	for (int i = 0; i < 50000; i++);
// }

// void ledFunction(uint8_t led_number, uint8_t led_state)
// {
// 	if (led_number == 6)
// 	{
//     	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_6, led_state); 
// 	}else if (led_number == 7)
// 	{
//     	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_7, led_state); 
// 	}else if (led_number == 8)
// 	{
//     	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_8, led_state); 
// 	}else if (led_number == 9)
// 	{
//     	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_9, led_state); 
// 	}                                                             
// }

// void GPIOLedInit(void)
// {
// 	GPIO_Handle_t GPIOLed;
//     memset(&GPIOLed, 0, sizeof(GPIOLed)); 							   

// 	GPIOLed.pGPIOx                              = GPIOB;               
// 	GPIOLed.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_6;       
// 	GPIOLed.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_OUT;       
// 	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_MS;       
// 	GPIOLed.GPIO_PinConfig.GPIO_PinOPType       = GPIO_OP_TYPE_PP;     
// 	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_NO_PUPD;        
// 	GPIO_Init(&GPIOLed);      

// 	GPIOLed.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_7; 
// 	GPIO_Init(&GPIOLed);    

// 	GPIOLed.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_8; 
// 	GPIO_Init(&GPIOLed); 

// 	GPIOLed.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_9; 
// 	GPIO_Init(&GPIOLed);                                      
// }
