#ifndef GPIO_H
#define GPIO_H

#define OUT 0b01
#define IN 0b00
#define HI 0b1
#define LO 0b0

#include <stdint.h>

void pinMode(char port, uint32_t pin, uint32_t mode);
void digitalWrite(char port, uint8_t pin, uint8_t value);
uint8_t digitalRead(char port, uint32_t pin);
void pinPull(char port, uint32_t pin, uint32_t dir);
void func_delay(uint8_t time);



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



#define PIN_R(port, pin, mode) \
	((port)->IDR & (1 << pin)) \


#endif // GPIO_H
