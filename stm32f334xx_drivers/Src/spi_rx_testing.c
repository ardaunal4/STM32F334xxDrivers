// #include "stm32f334xx.h"
// #include <string.h>

// #define MAX_LEN 500
// char RcvBuff[MAX_LEN];

// volatile char ReadByte;
// volatile uint8_t rcvStop = 0;

// /* This flag will be set in the interrupt handler of the other MCU interrupt GPIO */
// volatile uint8_t dataAvailable = 0;

// SPI_Handle_t SPI1handle;

// /*
//  * SPI1_NSS  -> PA4
//  * SPI1_SCK  -> PA5
//  * SPI1_MISO -> PA6
//  * SPI1_MOSI -> PA7
//  * Alternate functionality is AF5
//  */

// void SPI1_GPIOInits(void);
// void SPI1_Inits(void);
// void Slave_GPIO_InterruptPinInit(void);
// void delay(void);

// int main(void)
// {
// 	uint8_t dummy = 0xff;

// 	Slave_GPIO_InterruptPinInit();

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

// 	SPI_IRQInterruptConfig(IRQ_NO_SPI1, ENABLE);

//     while(1)
// 	{
// 		rcvStop = 0;
// 		while(!dataAvailable); // Wait till data available interrupt from transmitter device(slave)
// 		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, DISABLE);

// 		// Enable the SPI1 peripheral
// 		SPI_PeripheralControl(SPI1, ENABLE);

// 		while(!rcvStop)
// 		{
// 			/* fetch the data from the SPI peripheral byte by byte in interrupt mode */
// 			while ( SPI_SendDataIT(&SPI1handle, &dummy, 1) == SPI_BUSY_IN_TX );
// 			while ( SPI_ReceiveDataIT(&SPI1handle, &ReadByte, 1) == SPI_BUSY_IN_RX );
// 		}

// 		// Confirm SPI is not busy
// 		while( SPI_GetFlagStatus(SPI1, SPI_BSY_FLAG) );

// 		// Disable the SPI1 peripheral
// 		SPI_PeripheralControl(SPI1, DISABLE);

// 		printf("Rcvd data = %s\n", RcvBuff);
// 		dataAvailable = 0;
// 		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
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
// 	SPI1Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
// 	SPI1Handle.SPIConfig.SPI_SclkSpeed  = SPI_SCLK_SPEED_DIV256;            // Generates sclk of 8MHz
// 	SPI1Handle.SPIConfig.SPI_CPHA       = SPI_CPHA_LOW;
// 	SPI1Handle.SPIConfig.SPI_CPOL       = SPI_CPOL_HIGH;
// 	SPI1Handle.SPIConfig.SPI_SSM        = SPI_SSM_DI;                     // Hardware slave management enabled for NSS pin

// 	SPI_Init(&SPI1Handle);
// }

// void Slave_GPIO_InterruptPinInit(void)
// {
// 	/* This function configures the GPIO pin over which SPI peripheral issues data available interrupt. */
// 	GPIO_Handle_t spiInitPin;
// 	memset(&spiInitPin, 0, sizeof(spiInitPin));

// 	spiInitPin.pGPIOx = GPIOA;
// 	spiInitPin.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_3;
// 	spiInitPin.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_IT_FT;
// 	spiInitPin.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_LS;
// 	spiInitPin.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_NO_PUPD;

// 	GPIO_Init(&spiInitPin);

// 	GPIO_IRQPriorityConfig(IRQ_NO_EXTI3, NVIC_IRQ_PRIO15);
// 	GPIO_IRQInterruptConfig(IRQ_NO_EXTI3, ENABLE);
// }

// /* Runs when a data byte is received from the peripheral over SPI. */
// void SPI1_IRQHandler(void)
// {
// 	SPI_IRQHandling(&SPI1handle);
// }

// void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
// {
// 	static uint32_t i = 0;

// 	/* In the RX complete event , copy data in to rcv buffer . '\0' indicates end of message(rcvStop = 1) */
// 	if (AppEv == SPI_EVENT_RX_CMPLT)
// 	{
// 		RcvBuff[i++] = ReadByte;
// 		if(ReadByte == '\0' || ( i == MAX_LEN))
// 		{
// 			rcvStop = 1;
// 			RcvBuff[i-1] = '\0';
// 			i = 0;
// 		}
// 	}
// }

// /* Slave data available interrupt handler */
// void EXTI3_IRQHandler(void)
// {
// 	GPIO_IRQHandling(GPIO_PIN_NO_3);
// 	dataAvailable = 1;
// }

// void delay(void)
// {
// 	for (int i = 0; i < 50000; i++);
// }
