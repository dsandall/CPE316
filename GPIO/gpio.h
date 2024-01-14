#ifndef GPIO_H
#define GPIO_H

#define OUT 0b01
#define IN 0b00
#define HI 0b1
#define LO 0b0

#include <stdint.h>

void pinMode(char port, uint32_t pin, uint32_t mode);
void digitalWrite(char port, uint8_t pin, uint8_t value);



#define PIN_MODE(port, pin, mode) \
	do{	\
	(port)->MODER &= ~(0x3 << ((pin) * 2));\
	(port)->MODER |= ((mode) << ((pin) * 2));\
	}while(0)\



//usage: PIN_W(GPIOA, 1, HI);
#define PIN_W(port, pin, value) \
	do{	 \
	((value) == 1) ? ((port)->ODR |= (1 << (pin))) : ((port)->ODR &= ~(1 << (pin)));   \
	}while(0)  \



#endif // GPIO_H
