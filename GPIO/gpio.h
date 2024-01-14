#ifndef GPIO_H
#define GPIO_H

#define OUT 0b01
#define IN 0b00
#define HI 0b1
#define LO 0b0

#include <stdint.h>

void pinMode(char port, uint32_t pin, uint32_t mode);
void digitalWrite(char port, uint8_t pin, uint8_t value);

#endif // GPIO_H
