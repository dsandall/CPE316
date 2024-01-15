/*
 * LCD.c
 *
 *  Created on: Jan 13, 2024
 *      Author: thebu
 */

#include "LCD.h"
#include "gpio.h"
#include "main.h"

// There seems to be 40 characters per row (16 on screen at once)

	// RS	   : PA0
	// RW      : PA1
	// ENABLE  : PA4
	// DB4-DB7 : PB0-PB3

void LCD_init(void){
	//makes use of LCD_command to set the screen up

	const int WAKEY = 0x28;		//function set, 4 bit mode, 2 lines, 8dot font
	const int SHOW_DISP = 0x0F; //display on, cursor on, blink on
	const int MV_CRSR = 0x06;   //entry mode set: move cursor right after character. no display shift
	const int CLEAR = 0x01;

	LCD_command(WAKEY, 0);
	HAL_Delay(1);

	LCD_command(WAKEY, 0);
	HAL_Delay(1);

	LCD_command(WAKEY, 0);
	HAL_Delay(1);

	LCD_command(SHOW_DISP, 0);
	HAL_Delay(1);

	LCD_command(MV_CRSR, 0);
	HAL_Delay(1);

	LCD_command(CLEAR, 0);
	HAL_Delay(1);

	return;
}


void LCD_command(uint8_t command, uint8_t type){
	//the smallest operation of the 3. wiggles and waggles data lines with the appropriate fervor
	// command - the 8 bit command to be sent, in 4 bit chunks
	// type - 1 for chars, 0 for configuration/commands

	const int RS = 0;
	const int RW = 1;
	const int ENABLE = 4;
	const int DB4 = 0;
	const int DB5 = 1;
	const int DB6 = 2;
	const int DB7 = 3;

	PIN_W(GPIOA, RS, (0b1 & type));
	PIN_W(GPIOA, RW, 0);

	//min 100ns delay
	HAL_Delay(1);

	//enable goes high
	PIN_W(GPIOA, ENABLE, 1);
	//first 4 bits
	PIN_W(GPIOB, DB7, (command >> 7)&0b1);
	PIN_W(GPIOB, DB6, (command >> 6)&0b1);
	PIN_W(GPIOB, DB5, (command >> 5)&0b1);
	PIN_W(GPIOB, DB4, (command >> 4)&0b1);
	HAL_Delay(1);

	PIN_W(GPIOA, ENABLE, 0);
	HAL_Delay(1);

	PIN_W(GPIOA, ENABLE, 1);
	//next 4 bits
	PIN_W(GPIOB, DB7, (command >> 3)&0b1);
	PIN_W(GPIOB, DB6, (command >> 2)&0b1);
	PIN_W(GPIOB, DB5, (command >> 1)&0b1);
	PIN_W(GPIOB, DB4, (command >> 0)&0b1);
	HAL_Delay(1);

	PIN_W(GPIOA, ENABLE, 0);
	HAL_Delay(1);


	//change to BF clear mode
	//this should probably actually read BF, but let's just assume the display receives our commands
	PIN_W(GPIOA, RS, 0);
	PIN_W(GPIOA, RW, 1);
	HAL_Delay(1);

	PIN_W(GPIOA, ENABLE, 1);
	HAL_Delay(1);

	PIN_W(GPIOA, ENABLE, 0);
	HAL_Delay(1);

	PIN_W(GPIOA, ENABLE, 1);
	HAL_Delay(1);

	PIN_W(GPIOA, ENABLE, 0);
	HAL_Delay(1);

	return;
}


void LCD_write_string(const char* str) {
   for (int i = 0; str[i] != '\0'; i++) {
		  LCD_command(str[i], 1);
	}
}



// the following information is a synthesis of the NHD-C0220AA-FSW-FTW Graphic Liquid Crystal Display Module datasheet's timing information
// this is for a single data send operation
//clear RS and RW

//wait for tAS (min 60/100ns 8bit/4bit)
//set Enable (high for at least 300 ns, with a period of at least 500ns)

//clear 8 bit Data bus
//set data bus

// data is read on low transition of enable

//delay for ~300ns (give it min 100 ns before lowering enable)
//clear enable
//hold data and RS/RW for at least 10ns after enable is cleared to ensure proper operation


//delay 500ns
