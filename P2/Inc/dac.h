/*
 * dac.h
 *
 *  Created on: Feb 12, 2024
 *      Author: chris
 */

#ifndef INC_DAC_H_
#define INC_DAC_H_

#include "gpio.h"
#include <stdint.h>

void DAC_init();
void DAC_write(uint16_t data);
uint16_t DAC_volt_conv(uint16_t voltage);

#endif /* INC_DAC_H_ */
