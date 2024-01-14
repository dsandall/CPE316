#include "gpio.h"
#include "main.h"
/*
 *
 * note: coming back to this later... gonna focus on the task at hand before optimization
 * looking into multiline macros so that I can set and clear in one macro, like these functions
 *
 *

#define PIN_MODE(port, pin, mode) \
	port->MODER &= ~(0x3 << (pin * 2)))


*/

void pinMode(char port, uint32_t pin, uint32_t mode){

    switch (port) {

        case 'A':

			GPIOA->MODER &= ~(0x3 << (pin * 2));

			GPIOA->MODER |= (mode << (pin * 2));

        case 'B':

			GPIOB->MODER &= ~(0x3 << (pin * 2));

			GPIOB->MODER |= (mode << (pin * 2));

        case 'C':

			GPIOC->MODER &= ~(0x3 << (pin * 2));

			GPIOC->MODER |= (mode << (pin * 2));

        case 'D':

			GPIOD->MODER &= ~(0x3 << (pin * 2));

			GPIOD->MODER |= (mode << (pin * 2));

        default:

        	return;

    }

}



void digitalWrite(char port, uint8_t pin, uint8_t value){	// port, pin, value

    switch (port) {

        case 'A':

        	(value == 1) ? (GPIOA->ODR |= (1 << pin)) : (GPIOA->ODR &= ~(1 << pin));

        case 'B':

        	(value == 1) ? (GPIOB->ODR |= (1 << pin)) : (GPIOB->ODR &= ~(1 << pin));

        case 'C':

        	(value == 1) ? (GPIOC->ODR |= (1 << pin)) : (GPIOC->ODR &= ~(1 << pin));

        case 'D':

        	(value == 1) ? (GPIOD->ODR |= (1 << pin)) : (GPIOD->ODR &= ~(1 << pin));

        default:

        	return;

    }

}
