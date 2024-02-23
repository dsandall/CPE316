/*
 * keypad.c
 *
 *  Created on: Jan 14, 2024
 *      Author: thebu
 */


#include "gpio.h"
#include "main.h"
#include "keypad.h"

void setup_keypad(){
	  // 4 row pins (output mode)
	  pinMode('C', 0, 1);
	  pinMode('C', 1, 1);
	  pinMode('C', 2, 1);
	  pinMode('C', 3, 1);

	  // Pins for LEDs (output mode)
	  pinMode('A', 5, 1);
	  pinMode('A', 6, 1);
	  pinMode('A', 7, 1);
	  pinMode('A', 8, 1);

	  // 4 column pins (input mode)
	  pinMode('C', 5, 0);
	  pinMode('C', 6, 0);
	  pinMode('C', 7, 0);
	  pinMode('C', 8, 0);

	  // set pins to pull down mode
//	  pinPull('C', 5, 2);
//	  pinPull('C', 6, 2);
//	  pinPull('C', 7, 2);
//	  pinPull('C', 8, 2);

	  // set pins to pull down
	  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD5);
	  GPIOC->PUPDR |= (GPIO_PUPDR_PUPD5_1); // pull low

	  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD6);
	  GPIOC->PUPDR |= (GPIO_PUPDR_PUPD6_1); // pull low

	  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD7);
	  GPIOC->PUPDR |= (GPIO_PUPDR_PUPD7_1); // pull low

	  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD8);
	  GPIOC->PUPDR |= (GPIO_PUPDR_PUPD8_1); // pull low

	  // set all rows high
	  digitalWrite('C', 0, 1);
	  digitalWrite('C', 1, 1);
	  digitalWrite('C', 2, 1);
	  digitalWrite('C', 3, 1);

//	  digitalWrite('A', 5, 1);
//	  digitalWrite('A', 6, 1);
//	  digitalWrite('A', 7, 1);
//	  digitalWrite('A', 8, 1);

	  return;
}

uint8_t keypad_pressed(){
		  return ((GPIOC->IDR & GPIO_PIN_5) || (GPIOC->IDR & GPIO_PIN_6) || (GPIOC->IDR & GPIO_PIN_7) || (GPIOC->IDR & GPIO_PIN_8));
}

uint8_t scan_keypad(){
	  //set all low, begin polling
	  digitalWrite('C', 0, 0);
	  digitalWrite('C', 1, 0);
	  digitalWrite('C', 2, 0);
	  digitalWrite('C', 3, 0);

	  uint8_t btn_ID;

	  // go through each row
	  for (int i = 0; i < 4; i++){
		  digitalWrite('C', i, 1);

		  // check if any column is high
		  for (int j = 0; j < 4; j++){	// column pin = j + 5
			  if (GPIOC->IDR & (1 << (j + 5))){
				  // button pressed = i, j
				  btn_ID = ((i * 4) + j);
				  break;
			  }
		  }

		  digitalWrite('C', 0, 0);
		  digitalWrite('C', 1, 0);
		  digitalWrite('C', 2, 0);
		  digitalWrite('C', 3, 0);
	  }

	  // set all rows high (listen to all rows)
	  digitalWrite('C', 0, 1);
	  digitalWrite('C', 1, 1);
	  digitalWrite('C', 2, 1);
	  digitalWrite('C', 3, 1);

	  return btn_ID;
}

void disp_LED(uint8_t num){
//	  digitalWrite('A', 5, (num & 1));
//	  digitalWrite('A', 6, (num & 2));
//	  digitalWrite('A', 7, (num & 4));
//	  digitalWrite('A', 8, (num & 8));

	 if (num & 1){
	  digitalWrite('A', 5, 1);
	} else {
	  digitalWrite('A', 5, 0);
	}
	if (num & 2){
	  digitalWrite('A', 6, 1);
	} else {
	  digitalWrite('A', 6, 0);
	}
	if (num & 4){
	  digitalWrite('A', 7, 1);
	} else {
	  digitalWrite('A', 7, 0);
	}
	if (num & 8){
	  digitalWrite('A', 8, 1);
	} else {
	  digitalWrite('A', 8, 0);
	}

}


char id2char (uint8_t btn){
	char characters[] = {'1', '2', '3', 'A', '4', '5', '6', 'B', '7', '8', '9', 'C', '*', '0', '#', 'D'};
	return characters[btn];
}

