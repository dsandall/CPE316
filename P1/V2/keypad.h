/*
 * keypad.h
 *
 *  Created on: Jan 14, 2024
 *      Author: thebu
 */

#ifndef INC_KEYPAD_H_
#define INC_KEYPAD_H_

void setup_keypad();
uint8_t keypad_pressed();
uint8_t scan_keypad();
void disp_LED(uint8_t num);
char id2char (uint8_t btn);



#endif /* INC_KEYPAD_H_ */
