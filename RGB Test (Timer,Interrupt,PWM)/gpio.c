#include "gpio.h"
#include "main.h"
/*
 *
 * note: coming back to this later... gonna focus on the task at hand before optimization
 * looking into multiline macros so that I can set and clear in one macro, like these functions
 *
 *
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

    }

}


uint8_t digitalRead(char port, uint32_t pin){	// port, pin
    switch (port) {
        case 'A':
        	return (GPIOA->IDR & (1 << pin)) ? 1 : 0;
        case 'B':
        	return (GPIOB->IDR & (1 << pin)) ? 1 : 0;
        case 'C':
        	return (GPIOC->IDR & (1 << pin)) ? 1 : 0;
        case 'D':
        	return (GPIOD->IDR & (1 << pin)) ? 1 : 0;
        default:
        	return -1; //error
    }
}

void pinPull(char port, uint32_t pin, uint32_t dir){	// port, pin, none/up/down  0/1/2
    switch (port) {
        case 'A':
        	GPIOA->PUPDR &= ~(0x3 << (pin * 2));	// clear bits
        	GPIOA->PUPDR |= (dir << (pin * 2)); 	// write pull direction
        case 'B':
        	GPIOB->PUPDR &= ~(0x3 << (pin * 2));	// clear bits
			GPIOB->PUPDR |= (dir << (pin * 2)); 	// write pull direction
        case 'C':
        	GPIOC->PUPDR &= ~(0x3 << (pin * 2));	// clear bits
			GPIOC->PUPDR |= (dir << (pin * 2)); 	// write pull direction
        case 'D':
        	GPIOD->PUPDR &= ~(0x3 << (pin * 2));	// clear bits
			GPIOD->PUPDR |= (dir << (pin * 2)); 	// write pull direction
    }
}

#pragma GCC push_options
#pragma GCC optimize ("O0")
//compile this function without optimizations

void func_delay(uint8_t time){
	for (int i = time; i>0;){
		i--;
	}
//	HAL_Delay(10);
}

#pragma GCC pop_options

