/*
 * gpio.h
 *
 *  Created on: Feb 12, 2024
 *      Author: chris
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include <stdint.h>


void pinMode(char port, uint32_t pin, uint32_t mode);
void digitalWrite(char port, uint32_t pin, uint32_t value);
int digitalRead(char port, uint32_t pin);
void pinPull(char port, uint32_t pin, uint32_t dir);


#endif /* INC_GPIO_H_ */
