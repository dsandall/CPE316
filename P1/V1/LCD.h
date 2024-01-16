/*
 * LCD.h
 *
 *  Created on: Jan 13, 2024
 *      Author: thebu
 */



#ifndef LCD_H
#define LCD_H

#include <stdint.h>

void LCD_init(void);
void LCD_command(uint8_t command, uint8_t type);
void LCD_write_string(const char* str);

#endif
