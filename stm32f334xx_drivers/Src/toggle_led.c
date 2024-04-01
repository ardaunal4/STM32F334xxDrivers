/*
 * toggle_led.c
 *
 *  Created on: Mar 31, 2024
 *      Author: ardau
 */

/*
 * Example1 LED Toggle with push-pull configuration
 * It toggles red LED on the discovery board
*/
/*
#include "stm32f334xx.h"


void delay(void)
{
	for (int i = 0; i < 500000; i++);
}

int main(void)
{

	GPIO_Handle_t GPIOLed;                                             // Variable for GPIO_Handle_t
	GPIOLed.pGPIOx                              = GPIOB;               // Choose GPIO port as GPIOA
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_6;       // Pin number 6
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_MS;       // Output speed is configured as medium speed
	GPIOLed.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_OUT;       // Output mode is arranged as output
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_OP_TYPE_PP;     // Output type is configured as push-pull
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_NO_PUPD;        // No pull-up or pull-down resistors

    GPIO_PeriClockControl(GPIOB, ENABLE);                              // Activate the GPIOA peripheral clock
    GPIO_Init(&GPIOLed);                                               // Initialize the GPIOA pin 6

    while(1)
    {
    	GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_6);
    	delay();
    }

    return 0;
}
*/
