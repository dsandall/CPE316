/*
 * gpio.c
 *
 *  Created on: Feb 12, 2024
 *      Author: chris
 */

#include "gpio.h"
#include "main.h"

void pinMode(char port, uint32_t pin, uint32_t mode){  // port, pin, mode INPUT: 0 OUTPUT: 1
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

void digitalWrite(char port, uint32_t pin, uint32_t value){	// port, pin, value
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

int digitalRead(char port, uint32_t pin){	// port, pin
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
        	return -1;
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
