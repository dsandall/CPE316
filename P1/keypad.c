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
	  PIN_MODE(GPIOC, 0, 1);
	  PIN_MODE(GPIOC, 1, 1);
	  PIN_MODE(GPIOC, 2, 1);
	  PIN_MODE(GPIOC, 3, 1);

	  // Pins for LEDs (output mode)
	  PIN_MODE(GPIOA, 5, 1);
	  PIN_MODE(GPIOA, 6, 1);
	  PIN_MODE(GPIOA, 7, 1);
	  PIN_MODE(GPIOA, 8, 1);

	  // 4 column pins (input mode)
	  PIN_MODE(GPIOC, 5, 0);
	  PIN_MODE(GPIOC, 6, 0);
	  PIN_MODE(GPIOC, 7, 0);
	  PIN_MODE(GPIOC, 8, 0);

	  // set pins to pull down mode
	  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD5);
	  GPIOC->PUPDR |= (GPIO_PUPDR_PUPD5_1); // pull low

	  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD6);
	  GPIOC->PUPDR |= (GPIO_PUPDR_PUPD6_1); // pull low

	  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD7);
	  GPIOC->PUPDR |= (GPIO_PUPDR_PUPD7_1); // pull low

	  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD8);
	  GPIOC->PUPDR |= (GPIO_PUPDR_PUPD8_1); // pull low

	  // set all rows high (listen to all rows)
	  PIN_W(GPIOC,0,1);
	  PIN_W(GPIOC,1,1);
	  PIN_W(GPIOC,2,1);
	  PIN_W(GPIOC,3,1);

	  return;
}

uint8_t keypad_pressed(){
		  return ((GPIOC->IDR & GPIO_PIN_5) || (GPIOC->IDR & GPIO_PIN_6) || (GPIOC->IDR & GPIO_PIN_7) || (GPIOC->IDR & GPIO_PIN_8));
}

uint8_t scan_keypad(){
	  //set all low, begin polling
	  PIN_W(GPIOC, 0, 0);
	  PIN_W(GPIOC, 1, 0);
	  PIN_W(GPIOC, 2, 0);
	  PIN_W(GPIOC, 3, 0);

	  uint8_t btn_ID;

	  // go through each row
	  for (int i = 0; i < 4; i++){
		  PIN_W(GPIOC, i, 1);

		  // check if any column is high
		  for (int j = 0; j < 4; j++){	// column pin = j + 5
			  if (GPIOC->IDR & (1 << (j + 5))){
				  // button pressed = i, j
				  btn_ID = ((i * 4) + j);
				  break;
			  }
		  }

		  PIN_W(GPIOC, 0, 0);
		  PIN_W(GPIOC, 1, 0);
		  PIN_W(GPIOC, 2, 0);
		  PIN_W(GPIOC, 3, 0);
	  }

	  // set all rows high (listen to all rows)
	  PIN_W(GPIOC,0,1);
	  PIN_W(GPIOC,1,1);
	  PIN_W(GPIOC,2,1);
	  PIN_W(GPIOC,3,1);

	  return btn_ID;
}

void disp_LED(uint8_t num){
	  if (num & 1){
		  PIN_W(GPIOA, 5, 1);
	  } else {
		  PIN_W(GPIOA, 5, 0);
	  }
	  if (num & 2){
		  PIN_W(GPIOA, 6, 1);
	  } else {
		  PIN_W(GPIOA, 6, 0);
	  }
	  if (num & 4){
		  PIN_W(GPIOA, 7, 1);
	  } else {
		  PIN_W(GPIOA, 7, 0);
	  }
	  if (num & 8){
		  PIN_W(GPIOA, 8, 1);
	  } else {
		  PIN_W(GPIOA, 8, 0);
	  }

	  return;
}


char id2char (uint8_t btn){
	char chairs[] = {'1', '2', '3', 'A', '4', '5', '6', 'B', '7', '8', '9', 'C', '*', '#', 'D'};
	return chairs[btn];
}

