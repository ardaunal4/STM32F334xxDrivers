/*
 * button_interrupt.c
 *
 *  Created on: Apr 2, 2024
 *      Author: ardau
 */
/*
#include "stm32f334xx.h"
#include <string.h>

void delay(void)
{
	for (int i = 0; i < 50000; i++);
}

int main(void)
{

	GPIO_Handle_t GPIOLed, GPIOBtn;

    memset(&GPIOLed, 0, sizeof(GPIOLed));                              // Clear all structures values before implement it
    memset(&GPIOBtn, 0, sizeof(GPIOBtn));                              // Clear all structures values before implement it

	GPIOBtn.pGPIOx                              = GPIOA;               // Choose GPIO port as GPIOA
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_0;       // Pin number 0
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_IT_FT;     // Pin mode is arranged as INTERRUPT FALLING EDGE DETECTION
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_MS;       // Output speed is configured as medium speed
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_NO_PUPD;        // No pull-up or pull-down resistors

	GPIOLed.pGPIOx                              = GPIOB;               // Choose GPIO port as GPIOB
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_6;       // Pin number 6
	GPIOLed.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_OUT;       // Pin mode is arranged as output
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_MS;       // Output speed is configured as medium speed
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType       = GPIO_OP_TYPE_PP;     // Output type is configured as push-pull
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_NO_PUPD;        // No pull-up or pull-down resistors

    GPIO_PeriClockControl(GPIOA, ENABLE);                              // Activate the GPIOA peripheral clock
    GPIO_PeriClockControl(GPIOB, ENABLE);                              // Activate the GPIOB peripheral clock

    GPIO_Init(&GPIOBtn);                                               // Initialize the GPIOA pin 0
    GPIO_Init(&GPIOLed);                                               // Initialize the GPIOB pin 6

    // IRQ Configurations
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRIO15);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

    while (1);

	return 0;
}

void EXTI0_IRQHandler(void)
{
    // Handle the interrupt
	delay();                                                   // Put some delay because of button bouncing
    GPIO_IRQHandling(GPIO_PIN_NO_0);
    GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_6);                // Toggle the LED

}
*/
