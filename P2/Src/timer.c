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
const int prescale = 39; //larger numbers mean smaller frequency, means larger period
const int autoreload = 99; //larger numbers mean smaller frequency, means larger period


void TimerInit(void){

//  TIMER ESSENTIALS
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; //Enable Clock input to TIM2

	TIM2->PSC = prescale; // Set prescaler value
	TIM2->ARR = autoreload;   // Set auto-reload value
	TIM2->CR1 |= TIM_CR1_CEN; // Enable TIM2 in config register 1
	TIM2->CR1 &= ~(TIM_CR1_DIR);  // Clear DIR bit to set upcount mode (not super essential, cuz it defaults here)

}

//void TimerPWM_C1(int duty){
//	//	PWM / compare mode
//		TIM2->CCR1 = (uint32_t)((duty*autoreload)/256); // capture/compare register #1 - 32/16 bits to compare with the count
//
//		TIM2->CCMR1 &= ~(0x7 << 4); //clear CCMR1
//		TIM2->CCMR1 |= (0x6 << 4) ; // capture/compare mode register #2 - set TIM2 to PWM1 Mode (active when above CCR1)
//
//		TIM2->CCER |= TIM_CCER_CC1E; //capture/compare #1 enable - enables output channel 1
//}
//
//void TimerPWM_C2(int duty){
//	//	PWM / compare mode
//		TIM2->CCR2 = (uint32_t)((duty*autoreload)/256); // capture/compare register #2 - 32/16 bits to compare with the count
//
//		TIM2->CCMR1 &= ~(0x7 << 12); //clear CCMR2
//		TIM2->CCMR1 |= (0x6 << 12) ; // capture/compare mode register #2 - set TIM2 to PWM2 Mode (active when below CCR2)
//
//		TIM2->CCER |= TIM_CCER_CC2E; //capture/compare #2 enable - enables output channel 2
//}
//
//void TimerPWM_C3(int duty){
//	//	PWM / compare mode
//		TIM2->CCR3 = (uint32_t)((duty*autoreload)/256); // capture/compare register #1 - 32/16 bits to compare with the count
//
//		TIM2->CCMR2 &= ~(0x7 << 4); //clear CCMR2
//		TIM2->CCMR2 |= (0x6 << 4) ; // capture/compare mode register #2 - set TIM2 to PWM2 Mode (active when below CCR2)
//
//		TIM2->CCER |= TIM_CCER_CC3E;
//}

void TimerInterrupt(){
	//  Interrupt trigger Mode

	// 		Enable interrupt on UEV (Update Event = CNT goes to 0 in up/downcount mode)
		TIM2->DIER |= TIM_DIER_UIE;

	//// 	Enable TIM2 interrupt in NVIC (THE interrupter for STM32)
	// 	Using HAL: NVIC_EnableIRQ(TIM2_IRQn);
		NVIC->ISER[TIM2_IRQn / 32] = (1 << (TIM2_IRQn % 32));
		//evaluates to: NVIC->ISER[0] = (1 << 28);
		// note that the ISER has so many entries, it takes multiple 32 bit registers. Thus, it must be indexed using ISER[register offset]

	////	Set Interrupt Priority (Optional): - default for TIM2: 35
	//  Using HAL: NVIC_SetPriority(TIM2_IRQn, 6);
	// 	Using Direct Register Access:
//		const int priority = 35;
//		NVIC->IP[TIM2_IRQn] = priority << 4; // Adjust shift based on your MCU's NVIC priority field width

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


