/*
 * timer.c
 *
 *  Created on: Jan 15, 2024
 *      Author: thebu
 */
#include "timer.h"
#include "main.h"
#include "gpio.h"


//PAGE 1084 of big book
const int prescale = 65535;
const int autoreload = 256;


void TimerInit(void){

//  TIMER ESSENTIALS
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; //Enable Clock input to TIM2

	TIM2->PSC = prescale; // Set prescaler value
	TIM2->ARR = autoreload;   // Set auto-reload value
	TIM2->CR1 |= TIM_CR1_CEN; // Enable TIM2 in config register 1
	TIM2->CR1 &= ~(TIM_CR1_DIR);  // Clear DIR bit to set upcount mode (not super essential, cuz it defaults here)

}


void TimerPWM(float duty){
	//	PWM / compare mode
		TIM2->CCR1 |= (uint32_t)(duty*autoreload); // capture/compare register #1 - 32/16 bits to compare with the count

		TIM2->CCMR1 &= 0x0; //clear CCMR1
		TIM2->CCMR1 |= (0x6 << 4) ; // capture/compare mode register #2 - set TIM2 to PWM1 Mode

		TIM2->CCER |= TIM_CCER_CC1E; //capture/compare #1 enable - enables output
}

void TimerInterrupt(){
	//  Interrupt trigger Mode

	// 		Enable interrupt on UEV (Update Event = CNT goes to 0 in up/downcount mode)
		TIM2->DIER |= TIM_DIER_UIE;

	//// 	Enable TIM2 interrupt in NVIC (THE interrupter for STM32)
	// 	Using HAL: NVIC_EnableIRQ(TIM2_IRQn);
		NVIC->ISER[TIM2_IRQn / 32] = (1 << (TIM2_IRQn % 32));
		//evaluates to: NVIC->ISER[0] = (1 << 28);

	////	Set Interrupt Priority (Optional): - default for TIM2: 35
	//  Using HAL: NVIC_SetPriority(TIM2_IRQn, 6);
	// 	Using Direct Register Access:
//		const int priority = 35;
//		NVIC->IP[TIM2_IRQn] = priority << 4; // Adjust shift based on your MCU's NVIC priority field width

}




volatile int toggle = 1; //volatile for global variables used by interrupts!
void TIM2_IRQHandler(void) {
	if (TIM2->SR & TIM_SR_UIF) {TIM2->SR &= ~TIM_SR_UIF;}
	// Clear update interrupt flag (receive interrupt, prevents multiple triggers)

	toggle = !(toggle);

	PIN_W(GPIOA, 5, toggle);
	PIN_W(GPIOA, 6, toggle);
	PIN_W(GPIOA, 7, toggle);
	PIN_W(GPIOA, 8, toggle);

}


//### Notes:
//
//- Replace `prescaler_value` and `auto_reload_value` with appropriate values depending on your clock settings and the desired timer frequency. The timer clock frequency is determined by the formula: \(\text{Timer Frequency} = \frac{\text{Clock Frequency}}{(\text{Prescaler} + 1) \times \text{Auto Reload Value}}\).
//- Ensure that you calculate the prescaler and auto-reload values correctly based on your system clock frequency to get the desired timer interval.
//- The priority in `NVIC_SetPriority` should be set according to your application's needs and the NVIC priority grouping.
//- It's crucial to clear the appropriate interrupt flag inside the ISR to prevent the ISR from being called continuously.
//- Direct register access requires a good understanding of the microcontroller's hardware, so it's always recommended to refer to the STM32 reference manual for detailed information about the registers.
//- This code assumes a typical STM32 setup. Depending on your specific microcontroller model and development environment, some details may vary.
//



//RCC
//RCC->APB1RSTR1 (bit 0 resets TIM2 when high)
//RCC->APB1ENR1 (bit 2 enables TIM2 clock)
//


//What is the UEV?
// Triggers when the count reaches 0.
// Updates registers, such as the prescaler and ARR - writes are buffered until UEV (TIM2->CR1 ARPE enables buffer).
// Also triggers the interrupt if enabled, by setting the TIM2->SR UIF


//TIM2_CR1
//	UDIS bit (UEV disable)
//	CEN bit (enable timer)
//  OPM bit (one pulse mode, disables timer after 1 UAV)
// 	DIR bit (direction)
//
//TIM2_CR2
//TIM2_SMCR
//TIM2_DIER
//TIM2_SR
//TIM2_EGR
//	event generation register
//	has UG bit
//TIM2_CCMR1
//TIM2_CCMR2
//TIM2_CCER
//TIM2_CNT
//	the count
//TIM2_PSC
// 	prescaler (slower downer)
//TIM2_ARR
// 	auto reload (the max)
//TIM2_CCR1
//TIM2_CCR2
//	capture/compare register (timer 2, channel 2)
//TIM2_CCR3
//TIM2_CCR4
//TIM2_DCR
//TIM2_DMAR
//TIM2_OR1
//


