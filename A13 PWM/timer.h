/*
 * timer.h
 *
 *  Created on: Jan 15, 2024
 *      Author: thebu
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

void TimerInit(void);
void TimerInterrupt(void);
void TimerPWM_C1(float duty);
void TimerPWM_C2(float duty);
void TIM2_IRQHandler(void);

#endif /* INC_TIMER_H_ */
